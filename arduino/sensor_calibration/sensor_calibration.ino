// ES 245 Part 2 - Sharp GP2Y0A21 sensor calibration helper
// ------------------------------------------------------------
// Holds the servo at SERVO_CENTER and prints, once per second, the mean and
// standard deviation of the IR sensor voltage (100 readings per second).
//
// Procedure: put the ball at known distances from the sensor face
// (e.g. 10, 12, 14, 16, 18, 20, 22, 25, 28, 31, 34, 37, 40 cm; hold it with a
// ruler or a small wedge so that it does not roll), wait 3 s, and write down the
// "mean" voltage next to the distance. Enter the table in matlab/calibrate_sensor.m.
//
// Also use this sketch to find SERVO_CENTER: send a number such as "92" in the Serial
// Monitor (115200 baud, newline) to move the servo, and look for the command at which
// the beam is level (a phone spirit-level app on the beam is accurate enough).
#include <Servo.h>

const uint8_t SENSOR_PIN = A1;
const uint8_t SERVO_PIN  = 3;
float servoCmd = 90.0f;        // start at the Part 1 neutral command

Servo servo;

int angleToMicros(float deg) {            // same mapping as Servo.write(), finer steps
  return (int)(544.0f + deg * (2400.0f - 544.0f) / 180.0f + 0.5f);
}

void setup() {
  Serial.begin(115200);
  servo.attach(SERVO_PIN);
  servo.writeMicroseconds(angleToMicros(servoCmd));
  Serial.println(F("# sensor calibration: mean_V, std_V, servo_cmd_deg"));
}

void loop() {
  // read a new servo command if one was typed
  if (Serial.available()) {
    float c = Serial.parseFloat();
    if (c >= 45.0f && c <= 135.0f) {
      servoCmd = c;
      servo.writeMicroseconds(angleToMicros(servoCmd));
    }
    while (Serial.available()) Serial.read();
  }

  // 100 samples over one second
  float sum = 0.0f, sum2 = 0.0f;
  for (int i = 0; i < 100; i++) {
    float v = analogRead(SENSOR_PIN) * (5.0f / 1023.0f);
    sum += v; sum2 += v * v;
    delay(10);
  }
  float mean = sum / 100.0f;
  float var  = sum2 / 100.0f - mean * mean;
  Serial.print(F("mean_V=")); Serial.print(mean, 3);
  Serial.print(F("  std_V=")); Serial.print(sqrt(var > 0 ? var : 0), 3);
  Serial.print(F("  servo_cmd=")); Serial.println(servoCmd, 1);
}
