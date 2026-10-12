/**
 * ============================================================================
 * B.U.D.-E — Phase 1 Minimal Working Firmware: Single-Joint Bringup & Safety
 * Target MCU : ESP32-WROOM-32 DevKitC (Arduino IDE / PlatformIO)
 * Peripherals:
 *   - PCA9685 16-Channel 12-Bit PWM Driver (I2C addr 0x40)
 *   - INA219 High-Side DC Current/Voltage Sensor (I2C addr 0x41 or 0x40)
 *   - Optional 4th-wire servo wiper analog feedback on GPIO 34 (ADC1_CH6)
 *
 * Libraries Required (Install via Arduino Library Manager):
 *   1. "Adafruit PWM Servo Driver Library" by Adafruit
 *   2. "Adafruit INA219" by Adafruit
 *   3. "ArduinoJson" by Benoit Blanchon (or uses built-in lightweight parser below)
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_INA219.h>
#include <math.h>

// --- Pin Definitions (ESP32-WROOM-32) ---
static const int PIN_I2C_SDA       = 21;
static const int PIN_I2C_SCL       = 22;
static const int PIN_PCA9685_OE    = 4;   // Active-LOW Output Enable (HIGH = instant PWM disable)
static const int PIN_SERVO_ADC_CH0 = 34;  // Optional 4th-wire analog pot wiper (input-only ADC1)

// --- Constants ---
static const uint8_t PCA9685_I2C_ADDR = 0x40;
static const uint8_t INA219_I2C_ADDR  = 0x41; // Solder A0 jumper on INA219 so it doesn't clash with PCA9685 (0x40)
static const float   PWM_FREQ_HZ      = 50.0f;
static const uint32_t INTERP_PERIOD_MS = 10;  // 100 Hz cosine trajectory update rate
static const uint32_t INA_PERIOD_MS    = 5;   // 200 Hz current safety watchdog rate

Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(PCA9685_I2C_ADDR);
Adafruit_INA219         ina219(INA219_I2C_ADDR);

bool     g_ina219_present     = false;
bool     g_estop_tripped      = false;
float    g_current_limit_ma   = 450.0f; // Conservative 450 mA limit for 1-joint unloaded bringup
float    g_bus_voltage_v      = 6.0f;
float    g_bus_current_ma     = 0.0f;

// --- Cosine Interpolator State (Channel 0 for Phase 1 Minimal Bringup) ---
struct JointInterpolator {
  uint8_t  channel;
  float    start_us;
  float    current_us;
  float    target_us;
  uint32_t move_start_ms;
  uint32_t move_duration_ms;
  bool     active;
};

JointInterpolator g_joint0 = {0, 1500.0f, 1500.0f, 1500.0f, 0, 500, false};

// Convert microseconds (500..2500 us) to 12-bit PCA9685 tick (0..4095 at 50 Hz = 20,000 us period)
uint16_t usToTicks(float pulse_us) {
  float clamped = constrain(pulse_us, 500.0f, 2500.0f);
  return static_cast<uint16_t>((clamped / 20000.0f) * 4096.0f);
}

void disableAllServosHardware() {
  digitalWrite(PIN_PCA9685_OE, HIGH); // Immediately tri-states all PCA9685 outputs
  for (uint8_t ch = 0; ch < 6; ++ch) {
    pca9685.setPWM(ch, 0, 0);
  }
  g_joint0.active = false;
}

void commandSmoothMove(uint8_t ch, float target_us, uint32_t duration_ms) {
  if (g_estop_tripped) return;
  digitalWrite(PIN_PCA9685_OE, LOW); // Enable PCA9685 outputs
  g_joint0.channel          = ch;
  g_joint0.start_us         = g_joint0.current_us;
  g_joint0.target_us        = constrain(target_us, 500.0f, 2500.0f);
  g_joint0.move_start_ms    = millis();
  g_joint0.move_duration_ms = max(duration_ms, (uint32_t)50);
  g_joint0.active           = true;
}

void updateCosineInterpolator() {
  if (!g_joint0.active || g_estop_tripped) return;
  uint32_t elapsed = millis() - g_joint0.move_start_ms;
  if (elapsed >= g_joint0.move_duration_ms) {
    g_joint0.current_us = g_joint0.target_us;
  } else {
    // Cosine S-curve blend: s(t) = 0.5 * (1 - cos(pi * t / T))
    float progress = static_cast<float>(elapsed) / static_cast<float>(g_joint0.move_duration_ms);
    float blend    = 0.5f * (1.0f - cosf(PI * progress));
    g_joint0.current_us = g_joint0.start_us + blend * (g_joint0.target_us - g_joint0.start_us);
  }
  pca9685.setPWM(g_joint0.channel, 0, usToTicks(g_joint0.current_us));
}

void pollCurrentSafetyWatchdog() {
  if (!g_ina219_present) return;
  g_bus_voltage_v  = ina219.getBusVoltage_V();
  g_bus_current_ma = ina219.getCurrent_mA();

  if (g_bus_current_ma > g_current_limit_ma) {
    g_estop_tripped = true;
    disableAllServosHardware();
    Serial.printf("{\"ok\":false,\"event\":\"OVERCURRENT_TRIP\",\"current_ma\":%.1f,\"limit_ma\":%.1f}\n",
                  g_bus_current_ma, g_current_limit_ma);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_PCA9685_OE, OUTPUT);
  digitalWrite(PIN_PCA9685_OE, HIGH); // Start with servos safely disabled
  analogReadResolution(12);           // 12-bit ADC (0..4095) on GPIO 34

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);              // 400 kHz Fast I2C

  pca9685.begin();
  pca9685.setOscillatorFrequency(27000000);
  pca9685.setPWMFreq(PWM_FREQ_HZ);
  disableAllServosHardware();

  g_ina219_present = ina219.begin();
}

void loop() {
  static uint32_t last_ina_ms    = 0;
  static uint32_t last_interp_ms = 0;
  uint32_t now = millis();

  // 1. 200 Hz Hardware Current Watchdog
  if (now - last_ina_ms >= INA_PERIOD_MS) {
    last_ina_ms = now;
    pollCurrentSafetyWatchdog();
  }

  // 2. 100 Hz Smooth Cosine Servo Interpolator
  if (now - last_interp_ms >= INTERP_PERIOD_MS) {
    last_interp_ms = now;
    updateCosineInterpolator();
  }

  // 3. Process Incoming USB Serial Commands from Laptop
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.indexOf("\"PING\"") >= 0) {
      Serial.printf("{\"ok\":true,\"fw\":\"BUDE-ESP32-v1.0\",\"pca9685\":\"0x40\",\"ina219\":\"%s\"}\n",
                    g_ina219_present ? "0x41" : "sim");
    } else if (line.indexOf("\"SET_CURRENT_LIMIT\"") >= 0) {
      int idx = line.indexOf("\"max_ma\":");
      if (idx >= 0) {
        g_current_limit_ma = line.substring(idx + 9).toFloat();
      }
      g_estop_tripped = false;
      Serial.printf("{\"ok\":true,\"limit_ma\":%.1f,\"bus_v\":%.2f,\"current_ma\":%.1f}\n",
                    g_current_limit_ma, g_bus_voltage_v, g_bus_current_ma);
    } else if (line.indexOf("\"MOVE\"") >= 0) {
      int ch_idx  = line.indexOf("\"ch\":");
      int pwm_idx = line.indexOf("\"pwm_us\":");
      int dur_idx = line.indexOf("\"duration_ms\":");
      uint8_t  ch      = (ch_idx >= 0)  ? line.substring(ch_idx + 5).toInt() : 0;
      float    pwm_us  = (pwm_idx >= 0) ? line.substring(pwm_idx + 9).toFloat() : 1500.0f;
      uint32_t dur_ms  = (dur_idx >= 0) ? line.substring(dur_idx + 14).toInt() : 500;
      commandSmoothMove(ch, pwm_us, dur_ms);
      uint16_t raw_adc = analogRead(PIN_SERVO_ADC_CH0);
      Serial.printf("{\"ok\":true,\"ch\":%d,\"pwm_us\":%.0f,\"bus_v\":%.2f,\"current_ma\":%.1f,\"adc\":%u}\n",
                    ch, pwm_us, g_bus_voltage_v, g_bus_current_ma, raw_adc);
    } else if (line.indexOf("\"DISABLE\"") >= 0) {
      disableAllServosHardware();
      Serial.println("{\"ok\":true,\"state\":\"DISABLED\"}");
    }
  }
}
