// NOTE: stl_model.h must be in the same sketch folder
#include "stl_model.h"

/*
 * ============================================================
 *  EGOS-1 CubeSat ADCS Ground Station Firmware
 *  Enhanced: WebGL STL Viewer | Orientation Graphs | PSO Auto-Tuned PID
 *            Magnetorquer Control (B-dot detumble + PD pointing)
 *  MCU: ESP32 | Framework: Arduino
 * ============================================================
 */

#include <Wire.h>
#include <QMC5883LCompass.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MadgwickAHRS.h>
#include <WiFi.h>
#include <WebServer.h>
#include <TinyGPSPlus.h>
#include <ESP32Servo.h>
#include <ESPmDNS.h>

// ── Sensor / Filter objects ─────────────────────────────────
QMC5883LCompass compass;
Adafruit_MPU6050 mpu;
Madgwick filter;
TinyGPSPlus gps;
WebServer server(80);
Servo reactionWheel;

// ── WiFi ────────────────────────────────────────────────────
const char* ssid     = "HAMMOUDA";     // <-- Enter your router's SSID
const char* password = "0500961615"; // <-- Enter your router's Password

// ── Telemetry ───────────────────────────────────────────────
int finalRoll = 0, finalPitch = 0, finalYaw = 0;
String cardinalDir = "N";
double gpsLat = 0.0, gpsLng = 0.0, gpsAlt = 0.0;
int gpsSats = 0;
float magX_uT = 0.0, magY_uT = 0.0, magZ_uT = 0.0;
unsigned long lastTime = 0;

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
//  MAGNETORQUER SYSTEM
//  2 torque rods: ROLL axis (X) and PITCH axis (Y)
//  Driver: L298N H-bridge, one rod per channel
//
//  Power supply:
//    L298N motor pins (VS) → 3 V external supply
//    L298N logic pin (VSS) → ESP32 5V pin
//    Common GND
//
//  Torque rod specs (from physical dimensions):
//    Core  : Nickel-iron, ∅6 mm × 60 mm  (µ_eff ≈ 50)
//    Wire  : 0.4 mm copper, N = 250 turns
//    R     ≈ 0.70 Ω   L ≈ 1.85 mH   τ ≈ 2.65 ms
//    Dipole moment ≈ 0.354 A·m² per Ampere
//    Max safe current: 1.0 A  (L298N continuous limit @ 3 V)
//    ⚠ Do NOT exceed 1.0 A – R is only 0.7 Ω so 3 V can push 4 A
//       The PWM duty is capped to enforce this limit.
//
//  L298N wiring:
//    Roll  rod → OUT1/OUT2  (IN1=G25, IN2=G26, ENA=G33)
//    Pitch rod → OUT3/OUT4  (IN3=G27, IN4=G13, ENB=G32)
//
//  Control strategy:
//    Phase 1 – B-dot detumble : active while |ω| > BDOT_THRESH (rad/s)
//    Phase 2 – PD pointing    : active when satellite is calm
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

// ── L298N pin assignments ────────────────────────────────────
// Roll channel (OUT1/OUT2)
constexpr int MTQ_ROLL_IN1 = 25;
constexpr int MTQ_ROLL_IN2 = 26;
constexpr int MTQ_ROLL_ENA = 33;   // PWM-capable pin

// Pitch channel (OUT3/OUT4)
constexpr int MTQ_PITCH_IN3 = 27;
constexpr int MTQ_PITCH_IN4 = 13;  // G13 — safe, non-boot-sensitive
constexpr int MTQ_PITCH_ENB = 32;  // PWM-capable pin

// ── LEDC PWM channels (ESP32 Arduino core v3.x) ─────────────
// In v3.x the pin IS the channel — no ledcSetup/ledcAttachPin needed.
// Just call ledcAttach(pin, freq, resolution) then ledcWrite(pin, duty).
constexpr int LEDC_FREQ  = 5000;   // 5 kHz – above audible, well below L/R corner
constexpr int LEDC_RES   = 8;      // 8-bit → 0–255 duty

// ── Torque rod physical constants (250 turns, 3 V supply) ────
constexpr float MTQ_DIPOLE_PER_AMP = 0.354f;   // A·m² / A
constexpr float MTQ_ROD_R          = 0.70f;    // Ω  – low! beware overcurrent
constexpr float MTQ_SUPPLY_V       = 3.0f;     // V
// Hard current cap — L298N continuous limit is 2 A; we use 1 A for safety
constexpr float MTQ_MAX_CURRENT    = 1.0f;     // A
// Duty at which 1 A flows: I = (duty/255) * V_supply / R  → duty = I*R/V * 255
constexpr int   MTQ_MAX_DUTY = (int)(MTQ_MAX_CURRENT * MTQ_ROD_R / MTQ_SUPPLY_V * 255); // ≈ 59

// ── B-dot controller parameters ─────────────────────────────
constexpr float BDOT_GAIN   = 5.0f;    // k in m = -k * Ḃ
constexpr float BDOT_THRESH = 0.05f;   // rad/s  – switch to PD below this

// ── PD pointing parameters (roll & pitch) ───────────────────
constexpr float MTQ_KP = 0.8f;   // proportional gain (A / degree)
constexpr float MTQ_KD = 2.5f;   // derivative gain   (A / (degree/s))
constexpr float MTQ_ANGLE_DEADBAND = 1.5f;  // degrees – no action inside this

// ── State ───────────────────────────────────────────────────
bool   mtqEnabled     = false;
int    mtqRollDuty    = 0;    // –255 to +255 (sign = direction)
int    mtqPitchDuty   = 0;
float  prevBx = 0.0f, prevBy = 0.0f;   // for B-dot differentiation
float  prevRollRate  = 0.0f;
float  prevPitchRate = 0.0f;
String mtqPhase = "IDLE";     // "BDOT", "PD", "IDLE"

// ── Helper: drive one L298N channel ─────────────────────────
// duty: –255 (full reverse) to +255 (full forward), 0 = off
void driveChannel(int in1, int in2, int enPin, int duty) {
  duty = constrain(duty, -255, 255);
  if (duty == 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(enPin, 0);
  } else if (duty > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(enPin, (uint8_t)duty);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    ledcWrite(enPin, (uint8_t)(-duty));
  }
}

// ── Main magnetorquer update – call every loop iteration ─────
void updateMagnetorquers(float roll_deg, float pitch_deg,
                         float gx, float gy,          // body rates rad/s
                         float bx, float by, float bz,// body frame B-field (µT)
                         float dt) {
  if (!mtqEnabled || dt <= 0.0f) {
    driveChannel(MTQ_ROLL_IN1,  MTQ_ROLL_IN2,  MTQ_ROLL_ENA,  0);
    driveChannel(MTQ_PITCH_IN3, MTQ_PITCH_IN4, MTQ_PITCH_ENB, 0);
    mtqRollDuty  = 0;
    mtqPitchDuty = 0;
    mtqPhase = "IDLE";
    return;
  }

  // ── Measure tumble rate magnitude ──────────────────────────
  float omega = sqrtf(gx*gx + gy*gy);

  float rollDuty  = 0.0f;
  float pitchDuty = 0.0f;

  if (omega > BDOT_THRESH) {
    // ════════════════════════════════════════════════
    //  PHASE 1 – B-dot detumble
    //  Torque command: m = -k * dB/dt
    //  L298N duty ∝ required dipole moment
    // ════════════════════════════════════════════════
    mtqPhase = "BDOT";

    float dBx = (bx - prevBx) / dt;   // µT/s
    float dBy = (by - prevBy) / dt;

    // Desired dipole (A·m²) for each axis
    float mx_cmd = -BDOT_GAIN * dBx;
    float my_cmd = -BDOT_GAIN * dBy;

    // Convert dipole → current → duty (0–255)
    float ix = mx_cmd / MTQ_DIPOLE_PER_AMP;
    float iy = my_cmd / MTQ_DIPOLE_PER_AMP;

    ix = constrain(ix, -MTQ_MAX_CURRENT, MTQ_MAX_CURRENT);
    iy = constrain(iy, -MTQ_MAX_CURRENT, MTQ_MAX_CURRENT);

    rollDuty  = (ix / MTQ_MAX_CURRENT) * MTQ_MAX_DUTY;
    pitchDuty = (iy / MTQ_MAX_CURRENT) * MTQ_MAX_DUTY;

  } else {
    // ════════════════════════════════════════════════
    //  PHASE 2 – PD pointing (roll & pitch to zero)
    // ════════════════════════════════════════════════
    mtqPhase = "PD";

    float rollRate  = (roll_deg  - prevRollRate)  / dt;  // deg/s approx
    float pitchRate = (pitch_deg - prevPitchRate) / dt;

    // Roll axis torque rod (X axis)
    float rollErr = roll_deg;   // target = 0°
    if (fabsf(rollErr) > MTQ_ANGLE_DEADBAND) {
      float ix = MTQ_KP * rollErr + MTQ_KD * rollRate;
      ix = constrain(ix, -MTQ_MAX_CURRENT, MTQ_MAX_CURRENT);
      rollDuty = (ix / MTQ_MAX_CURRENT) * MTQ_MAX_DUTY;
    }

    // Pitch axis torque rod (Y axis)
    float pitchErr = pitch_deg;  // target = 0°
    if (fabsf(pitchErr) > MTQ_ANGLE_DEADBAND) {
      float iy = MTQ_KP * pitchErr + MTQ_KD * pitchRate;
      iy = constrain(iy, -MTQ_MAX_CURRENT, MTQ_MAX_CURRENT);
      pitchDuty = (iy / MTQ_MAX_CURRENT) * MTQ_MAX_DUTY;
    }

    prevRollRate  = roll_deg;
    prevPitchRate = pitch_deg;
  }

  // ── Apply to hardware ─────────────────────────────────────
  mtqRollDuty  = constrain((int)rollDuty,  -MTQ_MAX_DUTY, MTQ_MAX_DUTY);
  mtqPitchDuty = constrain((int)pitchDuty, -MTQ_MAX_DUTY, MTQ_MAX_DUTY);

  driveChannel(MTQ_ROLL_IN1,  MTQ_ROLL_IN2,  MTQ_ROLL_ENA,  mtqRollDuty);
  driveChannel(MTQ_PITCH_IN3, MTQ_PITCH_IN4, MTQ_PITCH_ENB, mtqPitchDuty);

  // ── Save previous B for next B-dot differentiation ───────
  prevBx = bx;
  prevBy = by;
}

// ── Motor / PID ─────────────────────────────────────────────
const int motorPin  = 18;
const int IDLE_PWM    = 1500;   // ESC mid-point: motor spins at idle speed
const int STANDBY_PWM = 1000;   // guaranteed below every ESC's run threshold
const int MAX_DELTA   = 400;    // max ±µs authority around idle (1100–1900)
bool motorEnabled   = false;
int  controlMode    = 0;       // 0 = Hold, 1 = Target
int  targetYaw      = 0;
int  currentPWM     = STANDBY_PWM;

float Kp = 0.0, Ki = 0.0, Kd = 0.0;   // set by PSO at startup
float integralError = 0.0, previousError = 0.0;

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
//  PARTICLE SWARM OPTIMISATION  –  offline PID auto-tuner
//  Runs once during setup() before the main loop starts.
//
//  Plant model: reaction wheel = double integrator.
//    wheel speed  ω  integrates torque input u
//    yaw angle    θ  integrates the reaction torque (-ω * gain)
//  This correctly captures the integrating nature of the plant
//  and naturally produces low Kp, near-zero Ki, dominant Kd.
//
//  Search bounds are sized for a small CubeSat reaction wheel
//  (A2212-class BLDC, ±400 µs authority, 50 Hz control loop).
//
//  Gain hard-limits (also enforced in the live PID loop):
//    Kp : 0.1  – 8.0
//    Ki : 0.0  – 0.5
//    Kd : 0.5  – 25.0
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
namespace PSO {
  // Hard gain limits – enforced both here and in the live loop
  constexpr float KP_MIN = 0.1f,  KP_MAX =  8.0f;
  constexpr float KI_MIN = 0.0f,  KI_MAX =  0.5f;
  constexpr float KD_MIN = 0.5f,  KD_MAX = 25.0f;

  constexpr int   N_PARTICLES = 12;
  constexpr int   N_ITER      = 80;
  constexpr float W  = 0.55f;   // inertia weight
  constexpr float C1 = 1.5f;    // cognitive coeff
  constexpr float C2 = 1.5f;    // social coeff

  // Double-integrator reaction-wheel plant:
  //   dω/dt = u / J_wheel          (wheel spin-up from torque)
  //   dθ/dt = -ω * (J_wheel/J_sat) (yaw from wheel momentum)
  // Normalised so u is a fraction of MAX_DELTA, output θ in degrees.
  float cost(float kp, float ki, float kd) {
    const float dt      = 0.02f;   // 50 Hz
    const int   SIM     = 300;     // 6 s of simulation
    const float Jratio  = 0.08f;   // J_wheel / J_sat (typical small CubeSat)
    float theta = 0.0f, omega = 0.0f;
    float integ = 0.0f, prev_e = 1.0f;
    float itae  = 0.0f;
    for (int t = 0; t < SIM; t++) {
      float e    = 1.0f - theta;          // setpoint = 1 normalised degree
      integ     += e * dt;
      integ      = constrain(integ, -50.0f, 50.0f);
      float deriv = (e - prev_e) / dt;
      float u    = kp * e + ki * integ + kd * deriv;
      u          = constrain(u, -1.0f, 1.0f);   // normalised ±1 authority
      omega     += u / Jratio * dt;
      omega      = constrain(omega, -500.0f, 500.0f);
      theta     -= omega * Jratio * dt;
      // Penalty: overshoot beyond 20% is heavily punished
      float penalty = (fabsf(theta) > 1.2f) ? 10.0f * fabsf(fabsf(theta) - 1.2f) : 0.0f;
      itae      += (float)(t + 1) * dt * fabsf(e) + penalty;
      prev_e     = e;
    }
    return itae;
  }

  // Lightweight LCG random – avoids rand() seeding issues on ESP32
  uint32_t seed = 0xDEADBEEFu;
  float randf() {
    seed = seed * 1664525u + 1013904223u;
    return (float)(seed >> 8) / (float)(1u << 24);
  }

  struct Particle {
    float pos[3], vel[3], best[3];
    float bestCost;
  };

  void run(float &outKp, float &outKi, float &outKd) {
    float lo[3] = { KP_MIN, KI_MIN, KD_MIN };
    float hi[3] = { KP_MAX, KI_MAX, KD_MAX };

    Particle swarm[N_PARTICLES];
    float gBest[3];
    float gBestCost = 1e30f;

    // Initialise particles
    for (int i = 0; i < N_PARTICLES; i++) {
      for (int d = 0; d < 3; d++) {
        swarm[i].pos[d]  = lo[d] + randf() * (hi[d] - lo[d]);
        swarm[i].vel[d]  = (randf() - 0.5f) * (hi[d] - lo[d]) * 0.1f;
        swarm[i].best[d] = swarm[i].pos[d];
      }
      swarm[i].bestCost = cost(swarm[i].pos[0], swarm[i].pos[1], swarm[i].pos[2]);
      if (swarm[i].bestCost < gBestCost) {
        gBestCost = swarm[i].bestCost;
        for (int d = 0; d < 3; d++) gBest[d] = swarm[i].pos[d];
      }
      yield();   // feed watchdog
    }

    // Main PSO loop
    for (int iter = 0; iter < N_ITER; iter++) {
      for (int i = 0; i < N_PARTICLES; i++) {
        for (int d = 0; d < 3; d++) {
          float r1 = randf(), r2 = randf();
          swarm[i].vel[d] = W  * swarm[i].vel[d]
                          + C1 * r1 * (swarm[i].best[d] - swarm[i].pos[d])
                          + C2 * r2 * (gBest[d]         - swarm[i].pos[d]);
          swarm[i].pos[d] += swarm[i].vel[d];
          swarm[i].pos[d]  = constrain(swarm[i].pos[d], lo[d], hi[d]);
        }
        float c = cost(swarm[i].pos[0], swarm[i].pos[1], swarm[i].pos[2]);
        if (c < swarm[i].bestCost) {
          swarm[i].bestCost = c;
          for (int d = 0; d < 3; d++) swarm[i].best[d] = swarm[i].pos[d];
          if (c < gBestCost) {
            gBestCost = c;
            for (int d = 0; d < 3; d++) gBest[d] = swarm[i].pos[d];
          }
        }
      }
      yield();   // feed watchdog every iteration
    }

    outKp = gBest[0];
    outKi = gBest[1];
    outKd = gBest[2];
    Serial.printf("[PSO] Done — Kp=%.3f Ki=%.4f Kd=%.3f  ITAE=%.4f\n",
                  outKp, outKi, outKd, gBestCost);
  }
} // namespace PSO

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
//  HTML DASHBOARD  (stored in PROGMEM flash)
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>EGOS-1 · ADCS Ground Station</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Exo+2:wght@300;600;800&display=swap');
  :root {
    --bg:      #ffffff;
    --surface: rgba(240,245,255,0.95);
    --border:  rgba(0,150,100,0.2);
    --cyan:    #007a55;
    --pink:    #cc1166;
    --amber:   #b07800;
    --blue:    #1a5fc8;
    --dim:     #6b7280;
    --text:    #1a202c;
    --mono:    'Share Tech Mono', monospace;
    --sans:    'Exo 2', sans-serif;
  }
  *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }

  body {
    font-family: var(--sans);
    background: var(--bg);
    color: var(--text);
    height: 100vh;
    overflow: hidden;
    display: flex;
    flex-direction: column;
    background-image:
      radial-gradient(ellipse 80% 60% at 50% -10%, rgba(0,150,100,0.04) 0%, transparent 70%),
      linear-gradient(180deg, #ffffff 0%, #f0f4ff 100%);
  }

  /* ── Header ── */
  header {
    display: flex; align-items: center; justify-content: space-between;
    padding: 10px 24px; flex-shrink: 0;
    border-bottom: 1px solid var(--border);
    background: rgba(0,150,100,0.04);
  }
  .logo { display: flex; align-items: baseline; gap: 10px; }
  .logo-name { font-size: 1.6rem; font-weight: 800; color: var(--cyan); letter-spacing: 3px; text-shadow: 0 0 20px rgba(0,150,100,0.3); }
  .logo-sub  { font-family: var(--mono); font-size: 0.75rem; color: var(--dim); letter-spacing: 4px; }
  .status-bar { display: flex; gap: 20px; font-family: var(--mono); font-size: 0.7rem; }
  .status-item { display: flex; align-items: center; gap: 6px; }
  .dot { width: 7px; height: 7px; border-radius: 50%; }
  .dot-green { background: var(--cyan); box-shadow: 0 0 8px rgba(0,150,100,0.5); animation: blink 2s infinite; }
  .dot-amber { background: var(--amber); }
  @keyframes blink { 0%,100%{opacity:1} 50%{opacity:0.3} }

  /* ── Layout Grid ── */
  .grid {
    display: grid;
    grid-template-columns: 260px 1fr 380px;
    grid-template-rows: 1fr 1fr 1fr;
    gap: 10px;
    padding: 10px;
    flex: 1;
    min-height: 0;
  }
  .panel {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: 10px;
    padding: 14px;
    display: flex;
    flex-direction: column;
    overflow: hidden;
    backdrop-filter: blur(8px);
    box-shadow: 0 2px 12px rgba(0,0,0,0.06);
  }
  .panel-title {
    font-family: var(--mono);
    font-size: 0.65rem;
    color: var(--dim);
    letter-spacing: 3px;
    border-bottom: 1px solid var(--border);
    padding-bottom: 8px;
    margin-bottom: 12px;
    flex-shrink: 0;
  }

  /* ── Orientation Cards ── */
  .att-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
  .att-card {
    background: rgba(255,255,255,0.7);
    border-radius: 8px;
    padding: 12px 8px;
    text-align: center;
    border-top: 2px solid var(--cyan);
    position: relative;
    overflow: hidden;
    box-shadow: 0 1px 6px rgba(0,0,0,0.05);
  }
  .att-card::before { content:''; position:absolute; inset:0; background: linear-gradient(180deg, rgba(0,150,100,0.04) 0%, transparent 100%); pointer-events:none; }
  .att-label { font-family: var(--mono); font-size: 0.6rem; color: var(--dim); letter-spacing: 2px; margin-bottom: 4px; }
  .att-val   { font-family: var(--mono); font-size: 1.6rem; font-weight: 600; color: var(--cyan); }
  .att-card.pitch { border-top-color: var(--pink); }
  .att-card.pitch .att-val { color: var(--pink); }
  .att-card.yaw   { border-top-color: var(--amber); }
  .att-card.yaw   .att-val { color: var(--amber); }
  .att-card.dir   { border-top-color: var(--blue); }
  .att-card.dir   .att-val { color: var(--blue); font-size: 1.8rem; }

  /* ── Mag Cards ── */
  .mag-grid { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 8px; margin-top: 10px; }
  .mag-card {
    background: rgba(255,255,255,0.7);
    border-radius: 8px;
    padding: 10px 5px;
    text-align: center;
    border-top: 2px solid var(--blue);
  }
  .mag-label { font-family: var(--mono); font-size: 0.55rem; color: var(--dim); letter-spacing: 2px; margin-bottom: 3px; }
  .mag-val   { font-family: var(--mono); font-size: 0.95rem; color: var(--blue); }

  /* ── 3D Viewer (spans 2 rows, middle col) ── */
  .viewer-panel {
    grid-column: 2;
    grid-row: 1 / 4;
    display: flex;
    flex-direction: column;
  }
  #webgl-canvas {
    flex: 1;
    width: 100%;
    border-radius: 8px;
    background: radial-gradient(ellipse at center, rgba(220,235,255,0.9) 0%, rgba(235,240,255,0.95) 100%);
    display: block;
    cursor: grab;
  }
  #webgl-canvas:active { cursor: grabbing; }
  .viewer-overlay {
    position: relative;
    flex: 1;
    display: flex;
    flex-direction: column;
    min-height: 0;
  }
  .viewer-stats {
    position: absolute;
    top: 10px; left: 10px;
    font-family: var(--mono);
    font-size: 0.6rem;
    color: rgba(0,100,70,0.6);
    line-height: 1.6;
    pointer-events: none;
  }
  .model-loading {
    position: absolute;
    inset: 0;
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    color: var(--cyan);
    font-family: var(--mono);
    font-size: 0.85rem;
    letter-spacing: 3px;
    gap: 12px;
  }
  .load-bar { width: 180px; height: 2px; background: rgba(0,255,180,0.15); border-radius: 1px; overflow: hidden; }
  .load-fill { height: 100%; background: var(--cyan); animation: loadAnim 1.5s ease-in-out infinite; }
  @keyframes loadAnim { 0%{width:0%} 100%{width:100%} }

  /* ── Chart Panel ── */
  .chart-panel { grid-column: 2; display: none; } /* hidden, embedded in viewer panel */
  #orient-chart {
    width: 100%;
    flex-shrink: 0;
    height: 140px;
    border-radius: 8px;
    background: rgba(255,255,255,0.6);
    border: 1px solid var(--border);
    margin-top: 10px;
  }
  .chart-legend {
    display: flex; gap: 16px; margin-top: 6px;
    font-family: var(--mono); font-size: 0.6rem;
    flex-shrink: 0;
  }
  .legend-item { display: flex; align-items: center; gap: 5px; }
  .legend-dot  { width: 8px; height: 8px; border-radius: 50%; }

  /* ── GPS Panel ── */
  .gps-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
  .gps-card {
    background: rgba(255,255,255,0.7);
    border-radius: 8px; padding: 10px 8px; text-align: center;
    border-top: 2px solid var(--pink);
  }
  .gps-label { font-family: var(--mono); font-size: 0.55rem; color: var(--dim); letter-spacing: 2px; margin-bottom: 3px; }
  .gps-val   { font-family: var(--mono); font-size: 1.0rem; color: var(--pink); }
  .searching {
    text-align: center; padding: 20px;
    font-family: var(--mono); font-size: 0.75rem; color: var(--pink);
    letter-spacing: 3px;
    animation: blink 1.5s infinite;
  }

  /* ── Motor Control ── */
  .pwr-btn {
    width: 100%; padding: 12px;
    font-family: var(--mono); font-size: 0.9rem; letter-spacing: 3px;
    border: 1px solid; border-radius: 8px; cursor: pointer;
    transition: all 0.2s; margin-bottom: 12px; font-weight: bold;
  }
  .pwr-off { background: rgba(204,17,102,0.08); border-color: var(--pink); color: var(--pink); }
  .pwr-on  { background: rgba(0,122,85,0.12); border-color: var(--cyan); color: var(--cyan); box-shadow: 0 0 20px rgba(0,122,85,0.15); }
  .motor-stats { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; margin-bottom: 12px; }
  .mstat {
    background: rgba(255,255,255,0.7); border-radius: 8px;
    padding: 10px 5px; text-align: center;
    border-top: 2px solid var(--amber);
  }
  .mstat-label { font-family: var(--mono); font-size: 0.55rem; color: var(--dim); letter-spacing: 2px; margin-bottom: 3px; }
  .mstat-val   { font-family: var(--mono); font-size: 1.0rem; color: var(--amber); }
  .mode-group { display: flex; gap: 8px; margin-bottom: 12px; }
  .mode-btn {
    flex: 1; padding: 10px 5px;
    font-family: var(--mono); font-size: 0.65rem; letter-spacing: 1px;
    border: 1px solid var(--border); border-radius: 6px; cursor: pointer;
    transition: all 0.2s; background: rgba(255,255,255,0.5); color: var(--dim);
  }
  .mode-btn.active { background: rgba(176,120,0,0.12); border-color: var(--amber); color: var(--amber); }
  .slider-row { display: flex; align-items: center; gap: 10px; }
  .slider-row label { font-family: var(--mono); font-size: 0.65rem; color: var(--dim); white-space: nowrap; }
  input[type=range] {
    flex: 1; accent-color: var(--amber);
    cursor: pointer; height: 4px;
  }
  input[type=range]:disabled { opacity: 0.3; cursor: not-allowed; }
  #target-val { font-family: var(--mono); font-size: 1.1rem; color: var(--amber); width: 44px; text-align: right; }

  /* ── PSO PID Display (read-only) ── */
  .pid-display { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 8px; margin-top: 10px; }
  .pid-card {
    background: rgba(255,255,255,0.7);
    border-radius: 8px; padding: 10px 6px; text-align: center;
    border-top: 2px solid var(--blue);
    box-shadow: 0 1px 6px rgba(0,0,0,0.05);
  }
  .pid-card-label { font-family: var(--mono); font-size: 0.55rem; color: var(--dim); letter-spacing: 2px; margin-bottom: 4px; }
  .pid-card-val   { font-family: var(--mono); font-size: 1.05rem; font-weight: 600; color: var(--blue); }
  .pid-card-range { font-family: var(--mono); font-size: 0.48rem; color: var(--dim); margin-top: 3px; opacity: 0.7; }
  .pso-badge {
    margin-top: 8px; text-align: center;
    font-family: var(--mono); font-size: 0.58rem;
    color: var(--cyan); letter-spacing: 2px;
    opacity: 0.75;
  }
</style>
</head>
<body>

<header>
  <div class="logo">
    <span class="logo-name">EGOS-1</span>
    <span class="logo-sub">ADCS GROUND STATION</span>
  </div>
  <div style="text-align:center; font-family:var(--sans); line-height:1.4;">
    <div style="font-size:0.85rem; font-weight:800; color:var(--cyan); letter-spacing:1px;">New Mansoura University</div>
    <div style="font-size:0.7rem; font-weight:600; color:var(--dim); letter-spacing:2px;">Aerospace Engineering &nbsp;·&nbsp; Team Astro</div>
  </div>
  <div class="status-bar">
    <div class="status-item"><div class="dot dot-green"></div><span id="link-status">LINK ACTIVE</span></div>
    <div class="status-item"><div class="dot dot-amber"></div><span id="fps-counter">-- Hz</span></div>
    <div class="status-item"><div class="dot dot-amber"></div><span id="sys-time">--:--:--</span></div>
  </div>
</header>

<div class="grid">

  <!-- ═══ LEFT COL TOP: Orientation ═══ -->
  <div class="panel" style="grid-column:1; grid-row:1 / 4;">
    <div class="panel-title">◈ ATTITUDE · ORIENTATION</div>
    <div class="att-grid">
      <div class="att-card">
        <div class="att-label">ROLL</div>
        <div class="att-val" id="r">0°</div>
      </div>
      <div class="att-card pitch">
        <div class="att-label">PITCH</div>
        <div class="att-val" id="p">0°</div>
      </div>
      <div class="att-card yaw">
        <div class="att-label">YAW</div>
        <div class="att-val" id="y">0°</div>
      </div>
      <div class="att-card dir">
        <div class="att-label">HEADING</div>
        <div class="att-val" id="h">N</div>
      </div>
    </div>
    <div class="mag-grid" style="margin-top:12px;">
      <div class="mag-card"><div class="mag-label">MAG·X</div><div class="mag-val" id="mx">0.0µT</div></div>
      <div class="mag-card"><div class="mag-label">MAG·Y</div><div class="mag-val" id="my">0.0µT</div></div>
      <div class="mag-card"><div class="mag-label">MAG·Z</div><div class="mag-val" id="mz">0.0µT</div></div>
    </div>

    <!-- GPS folded into left panel -->
    <div class="panel-title" style="margin-top:16px;">◈ POSITION · GPS</div>
    <div id="gps-searching" class="searching">SEARCHING SATELLITES...</div>
    <div id="gps-data" class="gps-grid" style="display:none;">
      <div class="gps-card"><div class="gps-label">LATITUDE</div><div class="gps-val" id="lat">--</div></div>
      <div class="gps-card"><div class="gps-label">LONGITUDE</div><div class="gps-val" id="lng">--</div></div>
      <div class="gps-card"><div class="gps-label">ALTITUDE m</div><div class="gps-val" id="alt">--</div></div>
      <div class="gps-card"><div class="gps-label">SATELLITES</div><div class="gps-val" id="sats">0</div></div>
    </div>
  </div>

  <!-- ═══ CENTER: 3D Model Viewer + Chart ═══ -->
  <div class="panel viewer-panel">
    <div class="panel-title">◈ EGOS-1 · 3D ATTITUDE VISUALIZER</div>
    <div class="viewer-overlay">
      <div id="model-loading" class="model-loading">
        <span>LOADING MODEL</span>
        <div class="load-bar"><div class="load-fill"></div></div>
      </div>
      <canvas id="webgl-canvas"></canvas>
      <div class="viewer-stats" id="viewer-stats"></div>
    </div>
    <canvas id="orient-chart"></canvas>
    <div class="chart-legend">
      <div class="legend-item"><div class="legend-dot" style="background:var(--cyan)"></div><span style="font-family:var(--mono);font-size:0.6rem;color:var(--dim)">ROLL</span></div>
      <div class="legend-item"><div class="legend-dot" style="background:var(--pink)"></div><span style="font-family:var(--mono);font-size:0.6rem;color:var(--dim)">PITCH</span></div>
      <div class="legend-item"><div class="legend-dot" style="background:var(--amber)"></div><span style="font-family:var(--mono);font-size:0.6rem;color:var(--dim)">YAW/3.6</span></div>
      <div style="flex:1"></div>
      <span style="font-family:var(--mono);font-size:0.55rem;color:var(--dim)">LAST 60s</span>
    </div>
  </div>

  <!-- ═══ RIGHT COL TOP+MID: Motor Control + PID ═══ -->
  <div class="panel" style="grid-column:3; grid-row:1 / 3; overflow-y:auto;">
    <div class="panel-title">◈ REACTION WHEEL · CONTROL</div>
    <button id="pwr-btn" class="pwr-btn pwr-off" onclick="toggleMotor()">⏻ MOTOR OFFLINE</button>
    <div class="motor-stats">
      <div class="mstat"><div class="mstat-label">MODE</div><div class="mstat-val" id="mode-disp">HOLD</div></div>
      <div class="mstat"><div class="mstat-label">ESC PWM</div><div class="mstat-val" id="pwm-disp">1000µs</div></div>
    </div>
    <div class="mode-group">
      <button id="btn-hold"   class="mode-btn active" onclick="setMode(0)">ATT. HOLD</button>
      <button id="btn-target" class="mode-btn"        onclick="setMode(1)">YAW TARGET</button>
    </div>
    <div class="slider-row">
      <label>TGT</label>
      <input type="range" id="yaw-slider" min="0" max="359" value="0"
             disabled
             oninput="onSliderInput(this.value)">
      <div id="target-val">0°</div>
    </div>

    <!-- PSO PID Display (read-only) -->
    <div class="panel-title" style="margin-top:14px;">◈ PSO · AUTO-TUNED GAINS</div>
    <div class="pid-display">
      <div class="pid-card"><div class="pid-card-label">Kp</div><div class="pid-card-val" id="kp-disp">--</div><div class="pid-card-range">0.1 – 8.0</div></div>
      <div class="pid-card"><div class="pid-card-label">Ki</div><div class="pid-card-val" id="ki-disp">--</div><div class="pid-card-range">0.0 – 0.5</div></div>
      <div class="pid-card"><div class="pid-card-label">Kd</div><div class="pid-card-val" id="kd-disp">--</div><div class="pid-card-range">0.5 – 25.0</div></div>
    </div>
    <div class="pso-badge">⬡ PARTICLE SWARM OPTIMISED</div>

    <!-- Magnetorquer section folded in -->
    <div class="panel-title" style="margin-top:14px;">◈ MAGNETORQUERS · L298N</div>
    <button id="mtq-btn" class="pwr-btn pwr-off" onclick="toggleMTQ()">⏻ MTQ OFFLINE</button>
    <div style="margin-top:8px; text-align:center;">
      <span id="mtq-phase-badge" style="
        font-family:var(--mono); font-size:0.6rem; letter-spacing:2px;
        padding:3px 10px; border-radius:4px;
        background:rgba(0,0,0,0.06); color:var(--dim); border:1px solid var(--border);">
        IDLE
      </span>
    </div>
    <div style="margin-top:10px; display:flex; flex-direction:column; gap:8px;">
      <div>
        <div style="display:flex; justify-content:space-between; margin-bottom:3px;">
          <span style="font-family:var(--mono);font-size:0.55rem;color:var(--dim);letter-spacing:1px;">ROLL ROD (X)</span>
          <span id="mtq-roll-val" style="font-family:var(--mono);font-size:0.55rem;color:var(--blue);">0%</span>
        </div>
        <div style="height:6px; background:rgba(0,0,0,0.08); border-radius:3px; overflow:hidden; position:relative;">
          <div id="mtq-roll-bar-neg" style="position:absolute; right:50%; top:0; height:100%; width:0%; background:var(--pink); border-radius:3px 0 0 3px;"></div>
          <div id="mtq-roll-bar-pos" style="position:absolute; left:50%; top:0; height:100%; width:0%; background:var(--cyan); border-radius:0 3px 3px 0;"></div>
        </div>
      </div>
      <div>
        <div style="display:flex; justify-content:space-between; margin-bottom:3px;">
          <span style="font-family:var(--mono);font-size:0.55rem;color:var(--dim);letter-spacing:1px;">PITCH ROD (Y)</span>
          <span id="mtq-pitch-val" style="font-family:var(--mono);font-size:0.55rem;color:var(--blue);">0%</span>
        </div>
        <div style="height:6px; background:rgba(0,0,0,0.08); border-radius:3px; overflow:hidden; position:relative;">
          <div id="mtq-pitch-bar-neg" style="position:absolute; right:50%; top:0; height:100%; width:0%; background:var(--pink); border-radius:3px 0 0 3px;"></div>
          <div id="mtq-pitch-bar-pos" style="position:absolute; left:50%; top:0; height:100%; width:0%; background:var(--cyan); border-radius:0 3px 3px 0;"></div>
        </div>
      </div>
    </div>
    <div style="margin-top:10px; display:grid; grid-template-columns:1fr 1fr 1fr; gap:6px;">
      <div class="pid-card" style="border-top-color:var(--pink);">
        <div class="pid-card-label">N TURNS</div>
        <div class="pid-card-val" style="color:var(--pink);font-size:0.85rem;">250</div>
      </div>
      <div class="pid-card" style="border-top-color:var(--pink);">
        <div class="pid-card-label">R (Ω)</div>
        <div class="pid-card-val" style="color:var(--pink);font-size:0.85rem;">0.70</div>
      </div>
      <div class="pid-card" style="border-top-color:var(--pink);">
        <div class="pid-card-label">I MAX (A)</div>
        <div class="pid-card-val" style="color:var(--pink);font-size:0.85rem;">1.0</div>
      </div>
    </div>
    <div class="pso-badge" style="margin-top:6px;">⬡ B-DOT → PD CONTROL</div>
  </div>

  <!-- ═══ RIGHT COL BOTTOM: Error Chart ═══ -->
  <div class="panel" style="grid-column:3; grid-row:3; overflow:hidden;">
    <div class="panel-title">◈ ATTITUDE ERROR · LIVE</div>
    <canvas id="error-chart" style="flex:1; width:100%; border-radius:8px; background:rgba(255,255,255,0.6); border:1px solid var(--border);"></canvas>
    <div style="margin-top:6px; display:flex; justify-content:space-between; align-items:center;">
      <span style="font-family:var(--mono);font-size:0.55rem;color:var(--dim)">-180°</span>
      <div style="display:flex;gap:14px;">
        <span style="font-family:var(--mono);font-size:0.55rem;color:#b07800;">■ YAW</span>
        <span style="font-family:var(--mono);font-size:0.55rem;color:#007a55;">■ ROLL</span>
        <span style="font-family:var(--mono);font-size:0.55rem;color:#cc1166;">■ PITCH</span>
      </div>
      <span style="font-family:var(--mono);font-size:0.55rem;color:var(--dim)">+180°</span>
    </div>
  </div>

</div><!-- /grid -->

<!-- ══════════════════════════════════════════════════════ -->
<!--   INLINE WEBGL STL RENDERER                          -->
<!-- ══════════════════════════════════════════════════════ -->
<script>
/* ── Globals ─────────────────────────────────────────── */
let motorState = 0, currentMode = 0;
let lastFetch = Date.now(), fetchCount = 0;
let rollVal = 0, pitchVal = 0, yawVal = 0;

/* ring buffers for charts */
const HIST = 300;
const rollHist = new Float32Array(HIST), pitchHist = new Float32Array(HIST), yawHist = new Float32Array(HIST);
const errHist  = new Float32Array(HIST);
let histPtr = 0;

/* ── Clock ───────────────────────────────────────────── */
setInterval(() => {
  const d = new Date();
  document.getElementById('sys-time').textContent =
    d.getHours().toString().padStart(2,'0') + ':' +
    d.getMinutes().toString().padStart(2,'0') + ':' +
    d.getSeconds().toString().padStart(2,'0');
}, 1000);

/* ── Motor / Mode ────────────────────────────────────── */
function toggleMotor() {
  const ns = motorState === 0 ? 1 : 0;
  fetch('/command?power=' + ns);
}

/* ── Magnetorquer ────────────────────────────────────── */
let mtqState = 0;
function toggleMTQ() {
  mtqState = mtqState === 0 ? 1 : 0;
  fetch('/mtq?en=' + mtqState);
  const btn = document.getElementById('mtq-btn');
  btn.textContent = mtqState ? '⏻ MTQ ONLINE' : '⏻ MTQ OFFLINE';
  btn.className   = 'pwr-btn ' + (mtqState ? 'pwr-on' : 'pwr-off');
}

function updateMTQDisplay(d) {
  if (d.mtq === undefined) return;

  // Phase badge colours
  const badge = document.getElementById('mtq-phase-badge');
  const phaseColors = { BDOT:'#cc1166', PD:'#007a55', IDLE:'#6b7280' };
  badge.textContent = d.mtqPhase || 'IDLE';
  badge.style.color  = phaseColors[d.mtqPhase] || '#6b7280';
  badge.style.borderColor = phaseColors[d.mtqPhase] || 'rgba(0,0,0,0.1)';

  // Duty bars — duty is –255 to +255
  function setBar(valId, posId, negId, duty) {
    const pct = Math.abs(duty) / 255 * 50;  // max 50% of each half
    const pctStr = pct.toFixed(1) + '%';
    document.getElementById(valId).textContent =
      (duty >= 0 ? '+' : '') + Math.round(duty/255*100) + '%';
    if (duty >= 0) {
      document.getElementById(posId).style.width = pctStr;
      document.getElementById(negId).style.width = '0%';
    } else {
      document.getElementById(negId).style.width = pctStr;
      document.getElementById(posId).style.width = '0%';
    }
  }
  setBar('mtq-roll-val',  'mtq-roll-bar-pos',  'mtq-roll-bar-neg',  d.mtqRoll);
  setBar('mtq-pitch-val', 'mtq-pitch-bar-pos', 'mtq-pitch-bar-neg', d.mtqPitch);
}
function setMode(m) {
  currentMode = m;
  document.getElementById('btn-hold').className   = 'mode-btn' + (m===0?' active':'');
  document.getElementById('btn-target').className = 'mode-btn' + (m===1?' active':'');
  document.getElementById('yaw-slider').disabled  = (m === 0);
  fetch('/command?mode=' + m + (m===0 ? '' : '&target=' + document.getElementById('yaw-slider').value));
}

/* real-time slider – fires on EVERY pixel of drag */
let sliderDebounce = null;
function onSliderInput(val) {
  document.getElementById('target-val').textContent = val + '°';
  clearTimeout(sliderDebounce);
  sliderDebounce = setTimeout(() => {
    fetch('/command?mode=1&target=' + val);
  }, 30);          // 30 ms debounce – fast but won't flood the ESP32
}

/* ── Telemetry Poll ──────────────────────────────────── */
setInterval(function() {
  fetch('/data').then(r => r.json()).then(d => {

    /* FPS counter */
    fetchCount++;
    if (Date.now() - lastFetch >= 1000) {
      document.getElementById('fps-counter').textContent = fetchCount + ' Hz';
      fetchCount = 0; lastFetch = Date.now();
    }

    rollVal  = d.roll;
    pitchVal = d.pitch;
    yawVal   = d.yaw;

    /* Attitude */
    document.getElementById('r').textContent = d.roll + '°';
    document.getElementById('p').textContent = d.pitch + '°';
    document.getElementById('y').textContent = d.yaw + '°';
    document.getElementById('h').textContent = d.dir;

    /* Mag */
    document.getElementById('mx').textContent = d.mx.toFixed(1) + 'µT';
    document.getElementById('my').textContent = d.my.toFixed(1) + 'µT';
    document.getElementById('mz').textContent = d.mz.toFixed(1) + 'µT';

    /* GPS */
    if (d.sats === 0 && d.lat === 0.0) {
      document.getElementById('gps-searching').style.display = 'block';
      document.getElementById('gps-data').style.display = 'none';
    } else {
      document.getElementById('gps-searching').style.display = 'none';
      document.getElementById('gps-data').style.display = 'grid';
      document.getElementById('lat').textContent  = d.lat.toFixed(5);
      document.getElementById('lng').textContent  = d.lng.toFixed(5);
      document.getElementById('alt').textContent  = d.alt + 'm';
      document.getElementById('sats').textContent = d.sats;
    }

    /* Motor */
    motorState = d.power;
    const pb = document.getElementById('pwr-btn');
    if (motorState === 1) { pb.className='pwr-btn pwr-on';  pb.textContent='⏻ MOTOR ONLINE'; }
    else                  { pb.className='pwr-btn pwr-off'; pb.textContent='⏻ MOTOR OFFLINE'; }
    document.getElementById('pwm-disp').textContent  = d.pwm + 'µs';
    document.getElementById('mode-disp').textContent = d.mode===0 ? 'HOLD' : 'TARGET';

    /* Sync slider if mode==1 and target updated server-side */
    if (d.mode === 1) {
      const sl = document.getElementById('yaw-slider');
      if (!sl.matches(':active')) {
        sl.value = d.target;
        document.getElementById('target-val').textContent = d.target + '°';
      }
    }

    /* PSO PID gains (read-only display) */
    if (d.kp !== undefined) {
      document.getElementById('kp-disp').textContent = d.kp.toFixed(3);
      document.getElementById('ki-disp').textContent = d.ki.toFixed(4);
      document.getElementById('kd-disp').textContent = d.kd.toFixed(3);
    }

    /* Magnetorquer display */
    updateMTQDisplay(d);

    /* History buffers */
    rollHist[histPtr]  = d.roll;
    pitchHist[histPtr] = d.pitch;
    yawHist[histPtr]   = d.yaw;

    /* Yaw error (shortest path) */
    let err = d.target - d.yaw;
    while (err >  180) err -= 360;
    while (err < -180) err += 360;
    errHist[histPtr] = err;

    histPtr = (histPtr + 1) % HIST;

    /* 3D model rotation */
    updateModel(d.roll, d.pitch, d.yaw);

    /* Draw charts */
    drawOrientChart();
    drawErrorChart();

  }).catch(() => {
    document.getElementById('link-status').textContent = 'LINK LOST';
  });
}, 200);

/* ═══════════════════════════════════════════════════════
   ORIENTATION CHART (canvas)
   ═══════════════════════════════════════════════════════ */
const OC = document.getElementById('orient-chart');
const OCX = OC.getContext('2d');

function drawOrientChart() {
  const W = OC.offsetWidth, H = OC.offsetHeight;
  if (W === 0) return;
  OC.width = W; OC.height = H;

  // Background grid
  OCX.fillStyle = 'rgba(0,0,0,0)';
  OCX.clearRect(0,0,W,H);

  /* horizontal grid lines: -180, -90, 0, 90, 180 */
  const gridVals = [-180, -90, 0, 90, 180];
  gridVals.forEach(v => {
    const y = H/2 - (v / 180) * (H/2 - 6);
    OCX.strokeStyle = v === 0 ? 'rgba(0,0,0,0.15)' : 'rgba(0,0,0,0.06)';
    OCX.lineWidth = 1;
    OCX.beginPath(); OCX.moveTo(0,y); OCX.lineTo(W,y); OCX.stroke();
    if (v !== 0) {
      OCX.fillStyle = 'rgba(0,0,0,0.3)';
      OCX.font = '9px Share Tech Mono, monospace';
      OCX.fillText(v + '°', 2, y - 2);
    }
  });

  /* draw series */
  function drawSeries(buf, color, scale) {
    OCX.strokeStyle = color;
    OCX.lineWidth   = 1.5;
    OCX.beginPath();
    let first = true;
    for (let i = 0; i < HIST; i++) {
      const idx = (histPtr + i) % HIST;
      const x   = (i / (HIST - 1)) * W;
      const val  = buf[idx] / scale;
      const y    = H/2 - (val / 180) * (H/2 - 6);
      if (first) { OCX.moveTo(x, y); first = false; }
      else         OCX.lineTo(x, y);
    }
    OCX.stroke();
  }

  drawSeries(rollHist,  'rgba(0,255,180,0.9)',   1);
  drawSeries(pitchHist, 'rgba(255,45,135,0.9)',  1);
  drawSeries(yawHist,   'rgba(255,170,0,0.7)',   3.6); // scale yaw to ±180 range

  /* current value ticks on right edge */
  [
    {v: rollVal,  c: '#00ffb4'},
    {v: pitchVal, c: '#ff2d87'},
  ].forEach(({v, c}) => {
    const y = H/2 - (v/180)*(H/2-6);
    OCX.fillStyle = c;
    OCX.beginPath(); OCX.arc(W-3, y, 3, 0, Math.PI*2); OCX.fill();
  });
}

/* ═══════════════════════════════════════════════════════
   ERROR CHART
   ═══════════════════════════════════════════════════════ */
const EC = document.getElementById('error-chart');
const ECX = EC.getContext('2d');

function drawErrorChart() {
  const W = EC.offsetWidth, H = EC.offsetHeight;
  if (W === 0) return;
  EC.width = W; EC.height = H;
  ECX.clearRect(0,0,W,H);

  /* zero line */
  ECX.strokeStyle = 'rgba(0,0,0,0.12)';
  ECX.lineWidth = 1;
  ECX.beginPath(); ECX.moveTo(0,H/2); ECX.lineTo(W,H/2); ECX.stroke();

  /* fill area under error curve */
  const gradient = ECX.createLinearGradient(0,0,0,H);
  gradient.addColorStop(0, 'rgba(255,170,0,0.3)');
  gradient.addColorStop(0.5, 'rgba(255,170,0,0.05)');
  gradient.addColorStop(1, 'rgba(255,170,0,0.0)');

  ECX.beginPath();
  ECX.moveTo(0, H/2);
  for (let i = 0; i < HIST; i++) {
    const idx = (histPtr + i) % HIST;
    const x   = (i / (HIST - 1)) * W;
    const y   = H/2 - (errHist[idx] / 180) * (H/2 - 4);
    ECX.lineTo(x, y);
  }
  ECX.lineTo(W, H/2);
  ECX.closePath();
  ECX.fillStyle = gradient;
  ECX.fill();

  /* line on top */
  ECX.strokeStyle = '#ffaa00';
  ECX.lineWidth   = 1.5;
  ECX.beginPath();
  for (let i = 0; i < HIST; i++) {
    const idx = (histPtr + i) % HIST;
    const x   = (i / (HIST - 1)) * W;
    const y   = H/2 - (errHist[idx] / 180) * (H/2 - 4);
    i === 0 ? ECX.moveTo(x,y) : ECX.lineTo(x,y);
  }
  ECX.stroke();

  /* current error text */
  const curErr = errHist[(histPtr - 1 + HIST) % HIST];
  ECX.fillStyle = '#ffaa00';
  ECX.font = 'bold 16px Share Tech Mono, monospace';
  ECX.textAlign = 'center';
  ECX.fillText((curErr >= 0 ? '+' : '') + curErr.toFixed(1) + '°', W/2, 20);
  ECX.textAlign = 'left';
}

/* ═══════════════════════════════════════════════════════
   WEBGL STL RENDERER
   ═══════════════════════════════════════════════════════ */
(async function initWebGL() {
  const canvas = document.getElementById('webgl-canvas');
  const gl = canvas.getContext('webgl') || canvas.getContext('experimental-webgl');
  if (!gl) { document.getElementById('model-loading').innerHTML = '<span style="color:#ff2d87">WebGL NOT SUPPORTED</span>'; return; }

  /* ── Shaders ── */
  const vsrc = `
    attribute vec3 aPos;
    attribute vec3 aNorm;
    uniform mat4 uMVP;
    uniform mat4 uModel;
    varying vec3 vNorm;
    varying vec3 vPos;
    void main(){
      vec4 worldPos = uModel * vec4(aPos, 1.0);
      vPos  = worldPos.xyz;
      vNorm = mat3(uModel) * aNorm;
      gl_Position = uMVP * vec4(aPos, 1.0);
    }`;

  const fsrc = `
    precision mediump float;
    varying vec3 vNorm;
    varying vec3 vPos;
    uniform vec3 uLightPos;
    uniform vec3 uCamPos;
    void main(){
      vec3 N = normalize(vNorm);
      vec3 L = normalize(uLightPos - vPos);
      vec3 V = normalize(uCamPos - vPos);
      vec3 H = normalize(L + V);

      float ambient  = 0.18;
      float diffuse  = max(dot(N, L), 0.0) * 0.65;
      float specular = pow(max(dot(N, H), 0.0), 32.0) * 0.4;

      // Edge-glow based on fresnel
      float fresnel = pow(1.0 - abs(dot(N, V)), 2.0) * 0.5;

      vec3 baseCol = vec3(0.05, 0.25, 0.18);   // dark teal body
      vec3 edgeCol = vec3(0.0,  0.55,  0.38);   // teal rim
      vec3 col = baseCol * (ambient + diffuse) + vec3(0.8,0.9,1.0) * specular + edgeCol * fresnel;
      gl_FragColor = vec4(col, 1.0);
    }`;

  function makeShader(type, src) {
    const s = gl.createShader(type);
    gl.shaderSource(s, src); gl.compileShader(s);
    return s;
  }
  const prog = gl.createProgram();
  gl.attachShader(prog, makeShader(gl.VERTEX_SHADER, vsrc));
  gl.attachShader(prog, makeShader(gl.FRAGMENT_SHADER, fsrc));
  gl.linkProgram(prog);
  gl.useProgram(prog);

  const aPos  = gl.getAttribLocation(prog, 'aPos');
  const aNorm = gl.getAttribLocation(prog, 'aNorm');
  const uMVP     = gl.getUniformLocation(prog, 'uMVP');
  const uModel   = gl.getUniformLocation(prog, 'uModel');
  const uLightPos = gl.getUniformLocation(prog, 'uLightPos');
  const uCamPos   = gl.getUniformLocation(prog, 'uCamPos');

  /* ── Load STL from ESP32 ── */
  let vertBuf, normBuf, vertCount = 0;

  try {
    const resp = await fetch('/model');
    const ab   = await resp.arrayBuffer();
    const view  = new DataView(ab);
    const nTri  = view.getUint32(80, true);

    const positions = new Float32Array(nTri * 9);
    const normals   = new Float32Array(nTri * 9);

    /* find model center/scale for normalisation */
    let minX=Infinity,minY=Infinity,minZ=Infinity;
    let maxX=-Infinity,maxY=-Infinity,maxZ=-Infinity;
    let off = 84;
    for (let i = 0; i < nTri; i++) {
      off += 12; // skip normal
      for (let v = 0; v < 3; v++) {
        const x=view.getFloat32(off,true); off+=4;
        const y=view.getFloat32(off,true); off+=4;
        const z=view.getFloat32(off,true); off+=4;
        if(x<minX)minX=x; if(x>maxX)maxX=x;
        if(y<minY)minY=y; if(y>maxY)maxY=y;
        if(z<minZ)minZ=z; if(z>maxZ)maxZ=z;
      }
      off += 2;
    }
    const cx=(minX+maxX)/2, cy=(minY+maxY)/2, cz=(minZ+maxZ)/2;
    const scale = 1.8 / Math.max(maxX-minX, maxY-minY, maxZ-minZ);

    off = 84;
    for (let i = 0; i < nTri; i++) {
      const nx=view.getFloat32(off,true); off+=4;
      const ny=view.getFloat32(off,true); off+=4;
      const nz=view.getFloat32(off,true); off+=4;
      for (let v = 0; v < 3; v++) {
        const vi = i*9 + v*3;
        positions[vi]   = (view.getFloat32(off,true)-cx)*scale; off+=4;
        positions[vi+1] = (view.getFloat32(off,true)-cy)*scale; off+=4;
        positions[vi+2] = (view.getFloat32(off,true)-cz)*scale; off+=4;
        normals[vi] = nx; normals[vi+1] = ny; normals[vi+2] = nz;
      }
      off += 2;
    }
    vertCount = nTri * 3;

    vertBuf = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, vertBuf);
    gl.bufferData(gl.ARRAY_BUFFER, positions, gl.STATIC_DRAW);

    normBuf = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, normBuf);
    gl.bufferData(gl.ARRAY_BUFFER, normals, gl.STATIC_DRAW);

    document.getElementById('model-loading').style.display = 'none';
    document.getElementById('viewer-stats').textContent = nTri + ' TRIANGLES';

  } catch(e) {
    document.getElementById('model-loading').innerHTML = '<span style="color:#ff2d87">MODEL LOAD FAILED</span>';
    return;
  }

  /* ── Math helpers ── */
  function mat4() { return new Float32Array(16); }
  function identity(m) { for(let i=0;i<16;i++)m[i]=i%5===0?1:0; return m; }
  function mul(out,a,b) {
    // Column-major: element(row,col) lives at col*4+row
    // C[j*4+i] = sum_k  A[k*4+i] * B[j*4+k]
    for(let i=0;i<4;i++) for(let j=0;j<4;j++) {
      let s=0; for(let k=0;k<4;k++) s+=a[k*4+i]*b[j*4+k];
      out[j*4+i]=s;
    } return out;
  }
  function rotX(a) {
    // Column-major: (1,1)→m[5]=c  (2,1)→m[6]=s  (1,2)→m[9]=-s  (2,2)→m[10]=c
    const m=identity(mat4()), c=Math.cos(a), s=Math.sin(a);
    m[5]=c; m[6]=s; m[9]=-s; m[10]=c; return m;
  }
  function rotY(a) {
    // Column-major: (0,0)→m[0]=c  (2,0)→m[2]=-s  (0,2)→m[8]=s  (2,2)→m[10]=c
    const m=identity(mat4()), c=Math.cos(a), s=Math.sin(a);
    m[0]=c; m[2]=-s; m[8]=s; m[10]=c; return m;
  }
  function rotZ(a) {
    // Column-major: (0,0)→m[0]=c  (1,0)→m[1]=s  (0,1)→m[4]=-s  (1,1)→m[5]=c
    const m=identity(mat4()), c=Math.cos(a), s=Math.sin(a);
    m[0]=c; m[1]=s; m[4]=-s; m[5]=c; return m;
  }
  function perspective(fov, aspect, near, far) {
    const m=mat4(), f=1/Math.tan(fov/2), nf=1/(near-far);
    m[0]=f/aspect; m[5]=f; m[10]=(far+near)*nf; m[11]=-1; m[14]=2*far*near*nf;
    return m;
  }
  function translate(tx,ty,tz) {
    const m=identity(mat4()); m[12]=tx; m[13]=ty; m[14]=tz; return m;
  }

  let roll=0, pitch=0, yaw=0;
  function updateModel(r,p,y) {
    roll  = r * Math.PI / 180;
    pitch = p * Math.PI / 180;
    yaw   = y * Math.PI / 180;
  }
  window.updateModel = updateModel;

  /* ── Render loop ── */
  function resize() {
    canvas.width  = canvas.offsetWidth;
    canvas.height = canvas.offsetHeight;
    gl.viewport(0, 0, canvas.width, canvas.height);
  }

  let animId;
  function render() {
    animId = requestAnimationFrame(render);
    resize();

    gl.clearColor(0.92, 0.95, 1.0, 0);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    gl.enable(gl.DEPTH_TEST);
    gl.disable(gl.CULL_FACE);   // binary STL has no guaranteed winding order

    const aspect = canvas.width / canvas.height;
    const proj = perspective(0.8, aspect, 0.1, 100);
    const view = translate(0, 0, -3.5);

    /* Apply roll→pitch→yaw rotation matching Madgwick convention */
    const modelRot = mul(mat4(), mul(mat4(), rotZ(roll), rotX(pitch)), rotY(-yaw));

    const mv  = mul(mat4(), view, modelRot);
    const mvp = mul(mat4(), proj, mv);

    gl.useProgram(prog);
    gl.uniformMatrix4fv(uMVP,   false, mvp);
    gl.uniformMatrix4fv(uModel, false, modelRot);
    gl.uniform3f(uLightPos,  3, 4, 5);
    gl.uniform3f(uCamPos,    0, 0, 3.5);

    /* Positions */
    gl.bindBuffer(gl.ARRAY_BUFFER, vertBuf);
    gl.enableVertexAttribArray(aPos);
    gl.vertexAttribPointer(aPos, 3, gl.FLOAT, false, 0, 0);

    /* Normals */
    gl.bindBuffer(gl.ARRAY_BUFFER, normBuf);
    gl.enableVertexAttribArray(aNorm);
    gl.vertexAttribPointer(aNorm, 3, gl.FLOAT, false, 0, 0);

    gl.drawArrays(gl.TRIANGLES, 0, vertCount);
  }
  render();
})();
</script>
</body>
</html>
)rawliteral";

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
//  SETUP
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);

  // ── 1. ESC / Motor Init ──────────────────────────────────
  ESP32PWM::allocateTimer(0);
  reactionWheel.setPeriodHertz(50);
  reactionWheel.attach(motorPin, 1000, 2000);
  // Arm ESC: hold minimum signal so it recognises the controller,
  // then drop back to standby (motor stopped, ESC stays armed).
  reactionWheel.writeMicroseconds(1000);
  delay(2000);                              // ESC arm beep sequence
  reactionWheel.writeMicroseconds(STANDBY_PWM);  // stopped, awaiting Online btn
  delay(500);

  // ── 2. WiFi Station Mode ─────────────────────────────────
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  Serial.print("Connecting to WiFi: ");
  Serial.print(ssid);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // ── 3. mDNS ──────────────────────────────────────────────
  if (MDNS.begin("astro-cubesat"))
    Serial.println("mDNS: http://astro-cubesat.local");

  // ── 4. Routes ────────────────────────────────────────────
  // HTML dashboard
  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html", index_html);
  });

  // Binary STL model – served directly from PROGMEM
  server.on("/model", HTTP_GET, []() {
    server.sendHeader("Content-Type", "application/octet-stream");
    server.sendHeader("Content-Disposition", "inline; filename=\"cubesat.stl\"");
    server.sendHeader("Access-Control-Allow-Origin", "*");
    // Stream STL from PROGMEM in chunks
    WiFiClient client = server.client();
    // Send HTTP/1.1 header manually since send_P doesn't take raw bytes
    String header = "HTTP/1.1 200 OK\r\n";
    header += "Content-Type: application/octet-stream\r\n";
    header += "Content-Length: " + String(STL_SIZE) + "\r\n";
    header += "Connection: close\r\n\r\n";
    client.print(header);

    const size_t CHUNK = 512;
    uint8_t buf[CHUNK];
    for (uint32_t sent = 0; sent < STL_SIZE; sent += CHUNK) {
      uint32_t len = min((uint32_t)CHUNK, STL_SIZE - sent);
      memcpy_P(buf, STL_DATA + sent, len);
      client.write(buf, len);
    }
  });

  // JSON telemetry
  server.on("/data", HTTP_GET, []() {
    float safeMx = isnan(magX_uT) ? 0.0f : magX_uT;
    float safeMy = isnan(magY_uT) ? 0.0f : magY_uT;
    float safeMz = isnan(magZ_uT) ? 0.0f : magZ_uT;

    String json = "{";
    json += "\"roll\":"  + String(finalRoll)  + ",";
    json += "\"pitch\":" + String(finalPitch) + ",";
    json += "\"yaw\":"   + String(finalYaw)   + ",";
    json += "\"dir\":\""  + cardinalDir + "\",";
    json += "\"mx\":"    + String(safeMx, 1)  + ",";
    json += "\"my\":"    + String(safeMy, 1)  + ",";
    json += "\"mz\":"    + String(safeMz, 1)  + ",";
    json += "\"lat\":"   + String(isnan(gpsLat) ? 0.0 : gpsLat, 6) + ",";
    json += "\"lng\":"   + String(isnan(gpsLng) ? 0.0 : gpsLng, 6) + ",";
    json += "\"alt\":"   + String((int)gpsAlt)  + ",";
    json += "\"sats\":"  + String(gpsSats)      + ",";
    json += "\"pwm\":"   + String(currentPWM)   + ",";
    json += "\"mode\":"  + String(controlMode)  + ",";
    json += "\"target\":" + String(targetYaw)   + ",";
    json += "\"power\":" + String(motorEnabled ? 1 : 0) + ",";
    json += "\"kp\":"    + String(Kp, 3) + ",";
    json += "\"ki\":"    + String(Ki, 4) + ",";
    json += "\"kd\":"    + String(Kd, 3) + ",";
    json += "\"mtq\":"        + String(mtqEnabled   ? 1 : 0) + ",";
    json += "\"mtqPhase\":\"" + mtqPhase + "\",";
    json += "\"mtqRoll\":"    + String(mtqRollDuty)  + ",";
    json += "\"mtqPitch\":"   + String(mtqPitchDuty);
    json += "}";
    server.send(200, "application/json", json);
  });

  // Motor / mode command
  server.on("/command", HTTP_GET, []() {
    if (server.hasArg("power")) {
      motorEnabled = (server.arg("power").toInt() == 1);
      if (motorEnabled) {
        // Snap target to current yaw so PID starts with zero error
        targetYaw     = finalYaw;
        integralError = 0.0;
        previousError = 0.0;
      }
    }
    if (server.hasArg("mode")) {
      controlMode = server.arg("mode").toInt();
      if (controlMode == 0) {
        targetYaw    = finalYaw;
        integralError = 0.0;
        previousError = 0.0;
      }
    }
    if (server.hasArg("target")) {
      targetYaw = server.arg("target").toInt();
      // Reset integrator on large target change to avoid windup
      integralError = 0.0;
    }
    server.send(200, "text/plain", "OK");
  });

  // Magnetorquer enable/disable
  server.on("/mtq", HTTP_GET, []() {
    if (server.hasArg("en")) {
      mtqEnabled = (server.arg("en").toInt() == 1);
      if (!mtqEnabled) {
        // Immediately cut current to both rods
        driveChannel(MTQ_ROLL_IN1,  MTQ_ROLL_IN2,  MTQ_ROLL_ENA,  0);
        driveChannel(MTQ_PITCH_IN3, MTQ_PITCH_IN4, MTQ_PITCH_ENB, 0);
        mtqPhase = "IDLE";
      }
      Serial.printf("[MTQ] %s\n", mtqEnabled ? "ENABLED" : "DISABLED");
    }
    server.send(200, "text/plain", "OK");
  });

  server.begin();

  // ── 5. Sensor Init ───────────────────────────────────────
  if (!mpu.begin()) { Serial.println("MPU6050 FAIL"); while (1) delay(10); }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  compass.init();
  compass.setSmoothing(10, true);
  compass.setCalibrationOffsets(743.00, -420.00, 1250.00);
  compass.setCalibrationScales(1.05, 0.87, 1.11);

  // ── 6. Madgwick warm-up (500 iterations) ─────────────────
  filter.begin(50);
  for (int i = 0; i < 500; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    compass.read();
    filter.update(
      g.gyro.x * RAD_TO_DEG, g.gyro.y * RAD_TO_DEG, g.gyro.z * RAD_TO_DEG,
      a.acceleration.x, a.acceleration.y, a.acceleration.z,
      compass.getY(), -compass.getX(), compass.getZ()
    );
    delay(2);
  }

  lastTime = micros();

  // ── 7. PSO – auto-tune PID gains ─────────────────────────
  Serial.println("[PSO] Running particle swarm optimisation...");
  PSO::run(Kp, Ki, Kd);

  // ── 8. Magnetorquer / L298N Init ─────────────────────────
  pinMode(MTQ_ROLL_IN1,  OUTPUT); digitalWrite(MTQ_ROLL_IN1,  LOW);
  pinMode(MTQ_ROLL_IN2,  OUTPUT); digitalWrite(MTQ_ROLL_IN2,  LOW);
  pinMode(MTQ_PITCH_IN3, OUTPUT); digitalWrite(MTQ_PITCH_IN3, LOW);
  pinMode(MTQ_PITCH_IN4, OUTPUT); digitalWrite(MTQ_PITCH_IN4, LOW);
  ledcAttach(MTQ_ROLL_ENA,  LEDC_FREQ, LEDC_RES);
  ledcAttach(MTQ_PITCH_ENB, LEDC_FREQ, LEDC_RES);
  ledcWrite(MTQ_ROLL_ENA,  0);
  ledcWrite(MTQ_PITCH_ENB, 0);
  Serial.printf("[MTQ] Init OK — MAX_DUTY=%d (%.0f mA cap)\n",
                MTQ_MAX_DUTY, MTQ_MAX_CURRENT * 1000.0f);

  Serial.println("EGOS-1 ADCS Online");
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
//  LOOP
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
void loop() {
  server.handleClient();

  // ── GPS decode ───────────────────────────────────────────
  while (Serial2.available() > 0) {
    if (gps.encode(Serial2.read())) {
      if (gps.location.isValid()) { gpsLat = gps.location.lat(); gpsLng = gps.location.lng(); }
      if (gps.altitude.isValid())  gpsAlt  = gps.altitude.meters();
      if (gps.satellites.isValid()) gpsSats = gps.satellites.value();
    }
  }

  // ── Attitude math ────────────────────────────────────────
  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) / 1e6f;
  if (dt <= 0.0f) { delay(1); return; }
  lastTime = currentTime;

  filter.begin(1.0f / dt);

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  compass.read();

  float mx = compass.getY();
  float my = -1.0f * compass.getX();
  float mz = compass.getZ();

  magX_uT = mx / 30.0f;
  magY_uT = my / 30.0f;
  magZ_uT = mz / 30.0f;

  filter.update(
    g.gyro.x * RAD_TO_DEG, g.gyro.y * RAD_TO_DEG, g.gyro.z * RAD_TO_DEG,
    a.acceleration.x, a.acceleration.y, a.acceleration.z,
    mx, my, mz
  );

  float roll  = filter.getRoll();
  float pitch = filter.getPitch();
  float yaw   = filter.getYaw() + 4.5f;  // local magnetic declination offset
  if (yaw <   0.0f) yaw += 360.0f;
  if (yaw >= 360.0f) yaw -= 360.0f;

  finalRoll  = (int)roundf(roll);
  finalPitch = (int)roundf(pitch);
  finalYaw   = (int)roundf(yaw);

  const char* dirs[] = {"N","NE","E","SE","S","SW","W","NW"};
  cardinalDir = dirs[(int)((yaw + 22.5f) / 45.0f) % 8];

  // ── PID / Reaction Wheel ─────────────────────────────────
  if (!motorEnabled) {
    currentPWM    = STANDBY_PWM;  // motor stopped, ESC stays armed
    integralError = 0.0f;
    previousError = 0.0f;
  } else {
    float error = (float)(targetYaw - finalYaw);
    // Shortest angular path
    while (error >  180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    integralError += error * dt;
    // Anti-windup clamp
    integralError = constrain(integralError, -100.0f, 100.0f);

    float derivative = (dt > 0.0f) ? (error - previousError) / dt : 0.0f;

    // Hard-clamp gains to PSO limits before use (safety guardrail)
    float kp = constrain(Kp, PSO::KP_MIN, PSO::KP_MAX);
    float ki = constrain(Ki, PSO::KI_MIN, PSO::KI_MAX);
    float kd = constrain(Kd, PSO::KD_MIN, PSO::KD_MAX);

    float pidOut = (kp * error) + (ki * integralError) + (kd * derivative);

    // PID output is an offset from idle – clamp to ±MAX_DELTA
    int delta   = constrain((int)pidOut, -MAX_DELTA, MAX_DELTA);
    currentPWM  = IDLE_PWM + delta;
    previousError = error;
  }

  reactionWheel.writeMicroseconds(currentPWM);

  // ── Magnetorquers ─────────────────────────────────────────
  updateMagnetorquers(
    roll, pitch,
    g.gyro.x, g.gyro.y,        // body rates (rad/s from MPU6050)
    magX_uT, magY_uT, magZ_uT, // ambient B-field from QMC5883L (µT)
    dt
  );

  delay(15);  // ~50 Hz sensor loop
}