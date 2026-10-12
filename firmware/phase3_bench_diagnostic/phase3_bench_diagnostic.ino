// B.U.D.-E phase 3 BENCH DIAGNOSTIC ONLY. ONE UNLOADED SERVO, CH0.
// Requires Adafruit PWM Servo Driver + Adafruit INA219 (Arduino ESP32).
// OE needs an EXTERNAL 10k pull-up to 3.3V: reset/disconnect => PWM off.
// Hardware NC E-stop on servo DC rail is mandatory; software is NOT safety rated.
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_INA219.h>
#include <math.h>

Adafruit_PWMServoDriver pwm(0x40);
Adafruit_INA219 sensor(0x41);
static const int OE = 4;
static const uint32_t TIMEOUT_MS = 500;
static const float LIMIT_MA = 900.0f; // bench example; NOT a collision threshold
bool armed = false, fault = true, sensorPresent = false;
float voltage = 0, current = 0;
uint32_t lastCommand = 0, lastSample = 0, startMove = 0, duration = 0;
float startUs = 1500, targetUs = 1500, currentUs = 1500;
bool moving = false;
char line[64]; size_t used = 0;

void disableDrive() {
  digitalWrite(OE, HIGH);
  armed = false;
  moving = false;
  pwm.setPWM(0, 0, 0);
}
void trip(const char* reason) {
  bool newlyFaulted = !fault;
  disableDrive(); fault = true;
  if (newlyFaulted) { Serial.print("FAULT "); Serial.println(reason); }
}
uint16_t ticks(float us) { return (uint16_t)lroundf(us * 4096.0f / 20000.0f); }
void sample() {
  if (!sensorPresent) { trip("INA219_absent"); return; }
  voltage = sensor.getBusVoltage_V(); // VIN- side (servo rail after E-stop)
  current = sensor.getCurrent_mA();
  if (!isfinite(voltage) || !isfinite(current) || voltage < 4.8f || voltage > 6.5f ||
      current < -100.0f || current > LIMIT_MA) trip("power_or_telemetry");
}
void command(char* s) {
  lastCommand = millis(); // only newline-terminated commands count as heartbeat
  if (!strcmp(s, "STATUS")) {
    Serial.printf("STATUS armed=%d fault=%d voltage=%.2f current_ma=%.0f\n", armed, fault, voltage, current);
  } else if (!strcmp(s, "DISARM")) { disableDrive(); Serial.println("OK DISARM"); }
  else if (!strcmp(s, "CLEAR")) {
    disableDrive(); sample();
    if (voltage >= 4.8f && voltage <= 6.5f && current >= -100 && current <= LIMIT_MA) {
      fault = false; Serial.println("OK CLEAR (still disabled)");
    } else Serial.println("ERR supply or telemetry");
  } else if (!strcmp(s, "ARM")) {
    sample();
    if (fault) { Serial.println("ERR fault; check E-stop and CLEAR first"); return; }
    // Set a safe pulse before enabling. Verify the unloaded horn was removed.
    currentUs = startUs = targetUs = 1500; moving = false;
    pwm.setPWM(0, 0, ticks(currentUs));
    digitalWrite(OE, LOW); armed = true; Serial.println("OK ARM CH0 ONLY");
  } else if (!strncmp(s, "MOVE ", 5)) {
    int ch, ms; float deg; char extra;
    if (sscanf(s, "MOVE %d %f %d %c", &ch, &deg, &ms, &extra) != 3 ||
        !armed || fault || ch != 0 || !isfinite(deg) || fabsf(deg) > 15 || ms < 500 || ms > 10000) {
      Serial.println("ERR expected MOVE 0 degrees[-15..15] duration_ms[500..10000]"); return;
    }
    startUs = currentUs; targetUs = 1500.0f + deg * 10.0f; // uncalibrated example!
    startMove = millis(); duration = (uint32_t)ms; moving = true;
    Serial.println("OK MOVE");
  } else Serial.println("ERR unknown command");
}
void setup() {
  pinMode(OE, OUTPUT); digitalWrite(OE, HIGH);
  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.setClock(100000); // avoid aggressive bus timing on breadboard
  pwm.begin(); pwm.setPWMFreq(50); pwm.setPWM(0, 0, 0);
  sensorPresent = sensor.begin();
  if (!sensorPresent) { Serial.println("FAULT INA219 absent at 0x41"); fault = true; }
  else { sensor.setCalibration_32V_2A(); sample(); } // 2A setting: never use for full arm
  disableDrive(); fault = true; lastCommand = millis();
  Serial.println("BENCH CH0 ONLY; inspect power, then CLEAR; ARM; STATUS heartbeat");
}
void loop() {
  const uint32_t now = millis();
  if (now - lastSample >= 50) { // telemetry sampling is NOT a certified safety loop
    lastSample = now; sample();
  }
  if (armed && now - lastCommand > TIMEOUT_MS) trip("host_timeout");
  if (armed && !fault && moving) {
    float t = fminf(1.0f, (float)(now - startMove) / duration);
    float blend = t*t*t*(10 + t*(-15 + 6*t));
    currentUs = startUs + (targetUs - startUs) * blend;
    pwm.setPWM(0, 0, ticks(currentUs));
    if (t >= 1) moving = false;
  }
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') { line[used] = '\0'; command(line); used = 0; }
    else if (c != '\r') {
      if (used < sizeof(line)-1) line[used++] = c;
      else { used = 0; trip("serial_overflow"); }
    }
  }
  delay(2);
}
