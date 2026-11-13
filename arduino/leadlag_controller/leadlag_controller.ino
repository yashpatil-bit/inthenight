// ES 245 Part 2 - Task 8: lead(-lag) compensator with reference prefilter
// ------------------------------------------------------------------------
// Control law (all in hardware units: ball distance in cm, servo deviation in deg)
//
//   r_f = F(s) r                    reference prefilter (cancels the lead zeros)
//   e   = r_f - y
//   v   = Lead(s)^N_LEAD e
//   u   = KC * Lag(s) v
//   servo command = SERVO_CENTER + DIRECTION * u        (clamped)
//
//   Lead(s) = (s + LEAD_Z)/(s + LEAD_P)    Lag(s) = (s + LAG_Z)/(s + LAG_P)
//   F(s)    = (LEAD_Z/(s + LEAD_Z))^N_LEAD
//
// LAG_P = 0 makes the lag a PI-type integrator (zero steady-state error to a
// constant offset such as a wrong SERVO_CENTER or friction). Its contribution
// Lag(s)v - v is clamped to +-LAG_LIMIT_DEG of servo angle (anti-windup) and is
// frozen while the servo command is saturated.
//
// Every first-order block is discretised with the Tustin (bilinear) rule in setup(),
// so only the continuous-time numbers printed by matlab/task7_compensator.m have to
// be copied below. Cascaded first-order sections stay numerically well-behaved in
// 32-bit float, unlike one high-order difference equation.
//
// Serial commands (Serial Monitor, newline): "r25" -> set point 25 cm,
//                                             "s"   -> start/stop automatic steps.
// Serial output, one CSV line per sample:  t_ms,volt,dist_cm,u_deg,ref_cm,reff_cm
#include <Servo.h>

// ----------------------------- user settings -----------------------------
const uint8_t SENSOR_PIN   = A1;
const uint8_t SERVO_PIN    = 3;
const float   T_S          = 0.02f;    // 20 ms, same as Chirp_Signal_IR; MUST equal P.Ts
const float   SERVO_CENTER = 90.0f;    // = centerAngle of the chirp sketch (level beam)
const float   SERVO_MIN    = 60.0f;
const float   SERVO_MAX    = 120.0f;
const float   DIRECTION    = +1.0f;    // same value as in Chirp_Signal_IR_modified
// distance model of Chirp_Signal_IR: L = A_CAL * V^B_CAL (same values in both sketches)
const float   A_CAL        = 35.7f;
const float   B_CAL        = -1.15f;
const float   DIST_MIN     = 4.0f;     // same valid range as the chirp sketch
const float   DIST_MAX     = 48.0f;
const float   FILTER_ALPHA = 0.3f;     // MUST equal FILTER_ALPHA in the chirp sketch

// ---- paste the block printed by task7_compensator.m here ----
// Pre-lab default: design from task7_compensator.m on a SIMULATED chirp test of
// this rig (structured fit K = 2.78 cm/(deg s^2), tau = 74 ms, delay 47 ms).
// REPLACE these with the numbers task7_compensator.m prints for YOUR data.
const float   KC           = 8.1776f;  // deg per cm
const int     N_LEAD       = 2;        // number of identical lead sections (0, 1 or 2)
const float   LEAD_Z       = 1.2000f;  // rad/s
const float   LEAD_P       = 4.8000f;  // rad/s
const bool    USE_LAG      = true;
const float   LAG_Z        = 0.6000f;  // rad/s
const float   LAG_P        = 0.0000f;  // rad/s (0 = integrating, PI-type lag)
const bool    USE_PREFILTER = true;
// --------------------------------------------------------------
const float   LAG_LIMIT_DEG = 15.0f;   // anti-windup: max servo angle from the lag

// reference: constant REF_CM, or automatic steps between REF_LOW and REF_HIGH
const float   REF_CM       = 22.0f;
const float   REF_LOW      = 17.0f;
const float   REF_HIGH     = 27.0f;
const float   STEP_PERIOD  = 15.0f;    // s spent at each level in step mode
bool          stepMode     = false;    // toggle with "s"
// --------------------------------------------------------------------------

struct Section {            // y = (n1 s + n0)/(s + p) x, Tustin
  float b0, b1, a1, x1, y1;
};

void sectionInit(Section &s, float n1, float n0, float p, float x0) {
  const float k = 2.0f / T_S, d = k + p;
  s.b0 = (n1 * k + n0) / d;
  s.b1 = (n0 - n1 * k) / d;
  s.a1 = (p - k) / d;
  s.x1 = x0;
  s.y1 = (p > 0.0f) ? x0 * n0 / p : 0.0f;   // steady state for a constant input x0
}

float sectionStep(Section &s, float x) {
  float y = s.b0 * x + s.b1 * s.x1 - s.a1 * s.y1;
  s.x1 = x; s.y1 = y;
  return y;
}

Servo servo;
Section lead[2], lag, pre[2];
bool  wasSaturated = false;
const unsigned long T_US = (unsigned long)(T_S * 1e6f);
unsigned long tStart, tNext, tStepStart;
float vFilt, ref = REF_CM;

int angleToMicros(float deg) {
  return (int)(544.0f + deg * (2400.0f - 544.0f) / 180.0f + 0.5f);
}
float readVolts() {                     // 10 readings, as in Chirp_Signal_IR
  long s = 0;
  for (int i = 0; i < 10; i++) s += analogRead(SENSOR_PIN);
  return s * (5.0f / 1023.0f / 10.0f);
}
float lastDist = 22.0f;
float voltsToCm(float v) {              // invalid reading -> keep the last one
  if (v <= 0.0f) return lastDist;
  float d = A_CAL * pow(v, B_CAL);
  if (d < DIST_MIN || d > DIST_MAX) return lastDist;
  lastDist = d;
  return d;
}

void handleSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == 'r') {
    float r = Serial.parseFloat();
    if (r >= 10.0f && r <= 40.0f) { ref = r; stepMode = false; }
  } else if (c == 's') {
    stepMode = !stepMode;
    tStepStart = micros();
  }
}

void setup() {
  Serial.begin(115200);
  servo.attach(SERVO_PIN);
  servo.writeMicroseconds(angleToMicros(SERVO_CENTER));
  delay(1500);
  vFilt = readVolts();
  float y0 = voltsToCm(vFilt);
  for (int i = 0; i < 2; i++) {
    sectionInit(lead[i], 1.0f, LEAD_Z, LEAD_P, 0.0f);
    sectionInit(pre[i], 0.0f, LEAD_Z, LEAD_Z, y0);   // prefilter starts at the ball
  }
  sectionInit(lag, 1.0f, LAG_Z, LAG_P, 0.0f);
  Serial.println(F("t_ms,volt,dist_cm,u_deg,ref_cm,reff_cm"));
  tStart = tNext = tStepStart = micros();
}

void loop() {
  handleSerial();
  if ((long)(micros() - tNext) < 0) return;
  tNext += T_US;
  float t = (tNext - T_US - tStart) * 1e-6f;

  if (stepMode) {
    float ts = (tNext - tStepStart) * 1e-6f;
    ref = (((long)(ts / STEP_PERIOD)) % 2 == 0) ? REF_LOW : REF_HIGH;
  }

  // 1. measure
  vFilt = FILTER_ALPHA * readVolts() + (1.0f - FILTER_ALPHA) * vFilt;
  float y = voltsToCm(vFilt);

  // 2. reference prefilter
  float rf = ref;
  if (USE_PREFILTER)
    for (int i = 0; i < N_LEAD; i++) rf = sectionStep(pre[i], rf);

  // 3. compensator
  float e = rf - y;
  float v = e;
  for (int i = 0; i < N_LEAD; i++) v = sectionStep(lead[i], v);
  float w = v;
  if (USE_LAG) {
    Section saved = lag;
    w = sectionStep(lag, v);
    if (wasSaturated) {                    // conditional integration
      lag = saved;
      lag.x1 = v;
      lag.y1 = v + (saved.y1 - saved.x1);  // keep the old lag contribution
      w = lag.y1;
    }
    const float lim = LAG_LIMIT_DEG / KC;  // anti-windup clamp
    w = v + constrain(w - v, -lim, lim);
    lag.y1 = w;
  }
  float u = KC * w;

  // 4. actuate
  float cmd = constrain(SERVO_CENTER + DIRECTION * u, SERVO_MIN, SERVO_MAX);
  wasSaturated = (cmd <= SERVO_MIN || cmd >= SERVO_MAX);
  u = (cmd - SERVO_CENTER) / DIRECTION;
  servo.writeMicroseconds(angleToMicros(cmd));

  // 5. log
  Serial.print((unsigned long)(t * 1000.0f)); Serial.print(',');
  Serial.print(vFilt, 3); Serial.print(',');
  Serial.print(y, 2);     Serial.print(',');
  Serial.print(u, 2);     Serial.print(',');
  Serial.print(ref, 1);   Serial.print(',');
  Serial.println(rf, 2);
}
