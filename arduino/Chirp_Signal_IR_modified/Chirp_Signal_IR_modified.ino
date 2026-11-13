// Chirp_Signal_IR (instructor's file) - modified for the Group 4 ball-and-beam rig
// ------------------------------------------------------------------------------
// Same structure, pins, IR distance model, linear chirp and 20 ms sampling as the
// provided Chirp_Signal_IR.ino. Every change is marked "CHANGED:" with the reason.
//
// Why the ball-holding loop (HOLD_BALL): the ball on the beam is a double
// integrator. In open loop the ball only stays on the beam if centerAngle is the
// level angle to within ~1 deg and the swing is 2-6 deg (checked in simulation);
// otherwise it rolls to the end in 2-5 s and the rest of the record is useless.
// With HOLD_BALL = true a weak PID adds a small correction to the chirp, so the
// servo input is still a chirp-rich signal and n4sid works on (servo_deg -> dist_cm).
// Set HOLD_BALL = false to run exactly the instructor's open-loop experiment.
//
// Output (115200 baud): time_ms,servo_deg,dist_cm,chirp_deg
//   servo_deg = command actually sent to the servo (chirp + correction)
//   chirp_deg = the chirp alone (used by task5_identify.m)

#include <Servo.h>
#include <math.h>

// Pin locations
const int irSensorPin = A1;
const int servoPin = 3;

// Variables
Servo servo;

// Chirp Signal Parameters
// CHANGED: 0.05-5 Hz in 20 s -> 0.1-2 Hz in 60 s. Above ~2 Hz the ball moves
// < 0.1 mm per degree (below sensor noise); a slower sweep gives more cycles
// at the low frequencies that matter for the design (crossover ~1-3 rad/s).
const float f0 = 0.1;            // Starting frequency (Hz)
const float f1 = 2.0;            // Ending frequency (Hz)
const float T_chirp = 60.0;      // Chirp duration (seconds)
const float T_settle = 5.0;      // CHANGED: chirp starts after 5 s (ball settles)

// CHANGED: amplitude was 0 (no excitation at all).
const float amplitude = 6;       // Servo swing (+/- degrees); 3-4 if HOLD_BALL = false
// CHANGED: set this to the angle at which YOUR beam is level (spirit level / phone).
const float centerAngle = 90;    // Centre position (degrees)
// CHANGED: 60-180 -> 60-120 (Part 1 limits; 180 can jam the four-bar linkage)
const float servoMin = 60.0;
const float servoMax = 120.0;

// CHANGED: weak PID that keeps the ball on the beam during the chirp
const bool  HOLD_BALL = true;
const float REF_CM = 22.0;       // hold the ball around here
const float KP = 0.6;            // deg per cm
const float KD = 0.8;            // deg per (cm/s)
const float TAU_D = 0.10;        // s, derivative filter
const float KI = 0.10;           // deg per (cm s)
const float I_LIMIT = 15.0;      // deg
const float DIRECTION = +1.0;    // +1: larger servo angle moves the ball AWAY from
                                 // the sensor. Flip if the ball is pushed away from REF_CM.
// CHANGED: light low-pass filter on the voltage. The controller sketch uses the
// SAME filter, so the identified model already contains its lag.
const float FILTER_ALPHA = 0.3;  // weight of a new sample (time constant ~56 ms)

// Timing
unsigned long startMillis;
unsigned long lastMillis;

const unsigned long samplePeriod = 20;  // 50 Hz target sampling

float vFilt = -1.0, lastDist = REF_CM, yPrev = REF_CM, dFilt = 0.0, iTerm = 0.0;

// CHANGED: writeMicroseconds gives ~0.1 deg steps; servo.write() truncates to 1 deg
void writeServo(float deg) {
  servo.writeMicroseconds((int)(544.0 + deg * (2400.0 - 544.0) / 180.0 + 0.5));
}

// --------------------------------
// IR Sensor Distance Measurement
// --------------------------------
float getDistance() {

  int sum = 0;
  const int N = 10;

  // Average 10 consecutive analog readings
  for (int i = 0; i < N; i++) {
    sum += analogRead(irSensorPin);
  }

  float avgSensorValue = sum / (float)N;

  float voltage = avgSensorValue * (5.0 / 1023.0);

  // CHANGED: low-pass filter (see FILTER_ALPHA)
  vFilt = (vFilt < 0) ? voltage : FILTER_ALPHA * voltage + (1.0 - FILTER_ALPHA) * vFilt;
  voltage = vFilt;

  // Avoid invalid voltage
  if (voltage <= 0) {
    return -1.0;
  }

  // Sharp GP2Y0A21YK0F empirical distance model
  float distance_cm = 35.7 * pow(voltage, -1.15);

  // Check valid sensor range
  if (distance_cm < 4.0 || distance_cm > 48.0) {
    return -1.0;
  }

  return distance_cm;
}

// --------------------------------
// Setup
// --------------------------------
void setup() {

  Serial.begin(115200);          // CHANGED: 9600 baud cannot carry ~22 chars every 20 ms

  servo.attach(servoPin);
  writeServo(centerAngle);

  Serial.println("time_ms,servo_deg,dist_cm,chirp_deg");

  delay(1500);                   // CHANGED: time to place the ball near REF_CM

  startMillis = millis();
  lastMillis = startMillis;
}

// --------------------------------
// Main Loop
// --------------------------------
void loop() {

  unsigned long now = millis();

  if (now - lastMillis < samplePeriod) {
    return;
  }

  lastMillis += samplePeriod;    // CHANGED: fixed rate (was lastMillis = now, which drifts)

  // Elapsed time in seconds
  float t = (now - startMillis) / 1000.0;
  float tc = t - T_settle;       // time since the chirp started

  // Chirp frequency rate
  float k = (f1 - f0) / T_chirp;

  // Stop after chirp duration (+ 3 s tail)
  if (tc >= T_chirp + 3.0) {
    writeServo(centerAngle);
    return;
  }

  // Measure IR distance (CHANGED: measured before the command is computed)
  float dist = getDistance();
  if (dist < 0) dist = lastDist; // CHANGED: hold the last valid reading
  lastDist = dist;

  // Calculate chirp phase
  float chirp = 0.0;
  if (tc >= 0 && tc < T_chirp) {
    float phi = 2.0 * PI * (f0 * tc + 0.5 * k * tc * tc);
    chirp = amplitude * sin(phi);
  }

  // CHANGED: weak ball-holding PID (derivative on measurement)
  float hold = 0.0;
  if (HOLD_BALL) {
    const float Ts = samplePeriod / 1000.0;
    dFilt = (TAU_D * dFilt + (dist - yPrev)) / (TAU_D + Ts);
    iTerm = constrain(iTerm + KI * (REF_CM - dist) * Ts, -I_LIMIT, I_LIMIT);
    hold = KP * (REF_CM - dist) - KD * dFilt + iTerm;
  }
  yPrev = dist;

  // Calculate servo command
  float servoCmd = centerAngle + DIRECTION * (hold + chirp);

  // Safety limits
  servoCmd = constrain(servoCmd, servoMin, servoMax);

  // Move servo
  writeServo(servoCmd);

  // Output CSV data
  Serial.print(now - startMillis);
  Serial.print(",");
  Serial.print(servoCmd, 2);
  Serial.print(",");
  Serial.print(dist, 2);
  Serial.print(",");
  Serial.println(DIRECTION * chirp, 2);
}
