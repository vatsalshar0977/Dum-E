/**
 * ============================================================================
 *  B.U.D.-E — PHASE 3 FULL FIRMWARE: 6-DOF ARM + 2-WHEEL ROVER + CLIFF REFLEX
 *  Target MCU : ESP32-WROOM-32 DevKitC (Arduino ESP32 core 2.0.0 or newer)
 *  File       : firmware/phase3_full_6dof_esp32/phase3_full_6dof_esp32.ino
 * ============================================================================
 *
 * WHAT THIS SKETCH IS
 * -------------------
 *   1. All six arm channels (J1..J5 + jaw) plus BOTH rover wheels on PCA9685 channels 6 and
 *      7, with `W <left_us> <right_us>` for the wheels and `M` / `G` for the arm.
 *   2. A HARDWARE CLIFF REFLEX. GPIO 25 / GPIO 26 (TCRT5000) are watched by an interrupt and
 *      by the loop. If either reads HIGH while the rover is driving FORWARD, this chip - not
 *      the laptop - puts both wheels into reverse (1680 / 1320 us) for 350 ms and then back
 *      to neutral (1500 / 1500 us). No USB serial round trip is involved.
 *   3. FMEA #11: an ESP32 task watchdog, plus a 500 ms host-liveness timeout. Silence from
 *      the laptop while the wheels are turning returns them to neutral by itself.
 *   4. FMEA #12: `i2c_bus_recovery()` - nine SCL clocks and a STOP - at boot before
 *      `Wire.begin()`, and again on every NACK. Three consecutive failures cut the PCA9685
 *      OE pin, because a driver that holds its last pulse width is the worst case there is.
 *
 * LIBRARIES: none beyond the ESP32 Arduino core. The PCA9685 and the INA219 are driven
 * straight from `Wire`, which is what makes NACK detection (and therefore FMEA #12) real.
 *
 * ============================================================================
 *  VERIFICATION STATUS — READ THIS BEFORE YOU TRUST IT
 * ============================================================================
 *   * The SAFETY LOGIC (reflex timing, the 350 / 500 ms rules, the mirrored wheel
 *     directions, the NACK policy, the non-blocking line parser) lives in `reflex_logic.h`
 *     and IS executed on a laptop by `test_reflex_logic.cpp`: 73 checks, all passing.
 *     Run it with `python3 -m unittest tests.test_firmware` or by hand:
 *         g++ -std=c++11 -Wall -Wextra -o /tmp/t test_reflex_logic.cpp && /tmp/t
 *   * THIS SKETCH ITSELF HAS NEVER BEEN COMPILED BY arduino-cli OR FLASHED. The I2C glue, the
 *     ISR plumbing, the watchdog configuration and every pin assignment are unverified
 *     against hardware. The first flash is bring-up, not a test.
 *   * The <5 ms reflex figure is a REQUIREMENT, not a measurement. The sketch measures its
 *     own edge-to-write latency (ISR timestamp -> I2C write complete) and reports worst case
 *     in `STATUS`. That number only becomes real when you flash it and trip a sensor.
 *   * `Serial` can block for about 5 ms at 115200 baud when the TX buffer is full. That is
 *     why the hot path prints nothing: replies only follow commands, and the loop services
 *     the cliff sensors BEFORE the serial port.
 *   * Nothing here is a safety device. The LA38-11ZS e-stop in the servo rail is the safety
 *     device; this sketch is the thing that tries to deserve it.
 *
 * ============================================================================
 *  WIRING  (see docs/phase3_electronics_and_wiring.md)
 * ============================================================================
 *   GPIO 21 / 22   PCA9685 + INA219 SDA / SCL              (400 kHz)
 *   GPIO 4         PCA9685 OE, 10 kOhm pull-up to 3V3. HIGH = all outputs tri-stated.
 *   GPIO 25        LEFT  TCRT5000 digital out (HIGH = no floor under the sensor)
 *   GPIO 26        RIGHT TCRT5000 digital out (HIGH = no floor under the sensor)
 *   GPIO 18        HC-SR04 TRIG   (10 us pulse, issued from a non-blocking state machine)
 *   GPIO 19        HC-SR04 ECHO   (interrupt captured - `pulseIn` is banned in this file)
 *   GPIO 34        optional J1 4th-wire pot wiper (input only)
 *   PCA9685 CH0-4  J1 yaw, J2 shoulder, J3 elbow, J4 wrist pitch, J5 wrist roll
 *   PCA9685 CH5    J6 jaw
 *   PCA9685 CH6    LEFT  drive servo (continuous rotation)
 *   PCA9685 CH7    RIGHT drive servo (continuous rotation)
 *   Servo V+ : 6 V brick -> fuse -> latching NC e-stop -> INA219 shunt -> PCA9685 V+
 *   PCA9685 VCC from 3V3. NEVER put 6 V on VCC or on the ESP32.
 *
 * ============================================================================
 *  SERIAL PROTOCOL (115200 baud, newline terminated)
 * ============================================================================
 *   PING                       identity, addresses, wheel channels, recoil duration
 *   W <left_us> <right_us>     BOTH WHEELS. `W 1500 1500` stop, `W 1320 1680` forward,
 *                              `W 1680 1320` reverse. This is the command the 500 ms
 *                              host-liveness timer watches.
 *   M <ch>:<us>[,<ch>:<us>...]          immediate set, up to 8 channels, one line
 *   G <ms> <ch>:<us>[,<ch>:<us>...]     cosine-blended move over <ms> (arm channels only;
 *                                       wheels are always immediate - see NOTE ON BLENDING)
 *   OE <0|1>                   0 enables the outputs, 1 tri-states them (the safe state)
 *   STATUS                     JSON: bus volts, current, cliffs, distance, reflex statistics
 *   HELP
 *
 *   NOTE ON BLENDING: wheel channels ignore the `G` blend duration on purpose. A blended
 *   wheel command would fight the cliff reflex for ownership of the same channels, and two
 *   writers on a safety path is how you lose one.
 *
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>
#include <esp_err.h>

#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif

#include "reflex_logic.h"

using namespace bude;

// ===========================================================================
// PINS AND DEVICES
// ===========================================================================
static const int PIN_I2C_SDA = 21;
static const int PIN_I2C_SCL = 22;
static const int PIN_PCA9685_OE = 4;      // HIGH = outputs disabled (tri-stated)
// -----------------------------------------------------------------------------------------
// SENSOR POLARITY — READ THIS BEFORE YOUR FIRST CLIFF TEST
//
// TCRT5000 breakouts are sold with two different output conventions, and the difference is
// not written on the board. As shipped, this firmware assumes the SAFE reading is HIGH:
//
//     rover flat on the desk (floor seen)  ->  GPIO 25/26 LOW   ->  STATUS "cliff":[0,0]
//     rover lifted so the sensor sees air  ->  GPIO 25/26 HIGH  ->  STATUS "cliff":[1,1]
//
// The pins are INPUT_PULLUP, so an UNPLUGGED sensor also reads HIGH - i.e. "cliff". That is
// deliberate: a broken wire leaves the rover unable to drive forward, and the boot log tells
// you the sensor is missing rather than pretending it is fine.
//
// Verify the polarity on your own modules (docs/testing_guide.md, Gate D) before any floor
// test. If yours reads the other way round, change THIS ONE LINE to `false` and reflash; the
// reads, the input gate and the interrupt edge all follow it.
static const bool CLIFF_TRIPPED_IS_HIGH = true;

/** Is the floor missing under this sensor? One definition, used by every reader. */
static bool cliffTripped(int pin) {
  return CLIFF_TRIPPED_IS_HIGH ? (digitalRead(pin) == HIGH) : (digitalRead(pin) == LOW);
}

static const int PIN_CLIFF_LEFT = 25;
static const int PIN_CLIFF_RIGHT = 26;
static const int PIN_SONAR_TRIG = 18;
static const int PIN_SONAR_ECHO = 19;
static const int PIN_SERVO_ADC_CH0 = 34;  // optional, input only

static const uint8_t PCA9685_ADDR = 0x40;
static const uint8_t INA219_ADDR = 0x41;

static const uint8_t PCA9685_REG_MODE1 = 0x00;
static const uint8_t PCA9685_REG_MODE2 = 0x01;
static const uint8_t PCA9685_REG_LED0_ON_L = 0x06;
static const uint8_t PCA9685_REG_PRESCALE = 0xFE;

static const uint8_t INA219_REG_CONFIG = 0x00;
static const uint8_t INA219_REG_BUS_VOLTAGE = 0x02;
static const uint8_t INA219_REG_CURRENT = 0x04;
static const uint8_t INA219_REG_CALIBRATION = 0x05;

/** The brief fixes the wheels at channels 6 and 7. */
static const uint8_t WHEEL_LEFT_CH = 6;
static const uint8_t WHEEL_RIGHT_CH = 7;
static const uint8_t CHANNEL_COUNT = 8;

static const float PWM_FREQ_HZ = 50.0f;
static const float PCA9685_OSC_HZ = 25000000.0f;   // nominal; calibrate if the servos buzz
static const uint32_t PWM_PERIOD_US = 20000;       // 50 Hz
static const uint32_t BLEND_PERIOD_US = 10000;     // 100 Hz cosine blend update
static const uint32_t INA_PERIOD_MS = 5;           // 200 Hz current / voltage watchdog
static const uint32_t SONAR_PERIOD_MS = 60;        // HC-SR04 wants >= 60 ms between pings
static const uint32_t SONAR_TIMEOUT_US = 25000;    // ~4.3 m round trip, then give up
static const uint32_t WATCHDOG_TIMEOUT_MS = 5000;  // FMEA #11
static const float BROWNOUT_V = 4.90f;
static const float ESTOP_V = 0.05f;                // the LA38-11ZS has cut the servo rail

// ===========================================================================
// STATE
// ===========================================================================
static WheelSafety g_wheels;                  // the reflex + liveness state machine
static UltrasonicGuard g_front(80, 500);      // 80 mm stop, 500 ms stale timeout
static LineAccumulator g_rx;
static I2cSupervisor g_bus(3);

/**
 * THE SHADOW: the last pulse width written to each channel. Every write goes through here,
 * and every block write is composed from here, so no code path can ever transmit an
 * uninitialised buffer to the servos.
 */
static uint16_t g_pulse_us[CHANNEL_COUNT] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

static bool g_servo_enabled = false;              // mirrors the OE pin
static bool g_ina_present = false;
static float g_bus_v = 6.0f;
static float g_current_ma = 0.0f;
static bool g_estop_latched = false;
static bool g_wdt_ready = false;
static uint32_t g_loop_iterations = 0;
static uint32_t g_sonar_timeouts = 0;
static uint32_t g_last_sonar_attempt_ms = 0;
static uint32_t g_last_ina_ms = 0;
static uint32_t g_last_blend_us = 0;

struct Blend {
  bool active;
  uint16_t start_us;
  uint16_t target_us;
  uint32_t start_ms;
  uint32_t duration_ms;
};
static Blend g_blend[CHANNEL_COUNT];

// HC-SR04 capture - written only from the echo ISR
static volatile uint32_t g_echo_rise_us = 0;
static volatile uint32_t g_echo_fall_us = 0;
static volatile bool g_echo_rise_seen = false;
static volatile bool g_echo_complete = false;
static bool g_sonar_waiting = false;
static uint32_t g_sonar_trig_us = 0;

// ===========================================================================
// I2C
// ===========================================================================
static void i2c_bus_recovery();
static bool i2cRecoverAndRetry();
static void handleBusFailure(const char* what);

/**
 * FMEA #12: release a slave that is holding SDA low.
 *
 * A PCA9685 or INA219 interrupted mid-byte can hold SDA down, and `Wire.begin()` will then
 * hang or fail forever. Clocking SCL nine times lets the slave finish shifting its byte out
 * and release the line; the trailing STOP returns the bus to idle. Nine is the magic number:
 * eight data bits plus the acknowledge slot.
 *
 * Both pins are driven OPEN DRAIN, because on a real I2C bus "high" must mean "released",
 * never "actively driven to 3V3 into a slave that is holding the line low".
 */
static void i2c_bus_recovery() {
  Wire.end();
  pinMode(PIN_I2C_SDA, INPUT_PULLUP);
  pinMode(PIN_I2C_SCL, OUTPUT_OPEN_DRAIN);
  digitalWrite(PIN_I2C_SCL, HIGH);
  delayMicroseconds(5);

  for (uint8_t i = 0; i < 9; ++i) {
    digitalWrite(PIN_I2C_SCL, LOW);
    delayMicroseconds(5);
    digitalWrite(PIN_I2C_SCL, HIGH);
    delayMicroseconds(5);
  }

  // STOP condition: SDA rises while SCL is high.
  pinMode(PIN_I2C_SDA, OUTPUT_OPEN_DRAIN);
  digitalWrite(PIN_I2C_SDA, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_I2C_SCL, HIGH);
  delayMicroseconds(5);
  digitalWrite(PIN_I2C_SDA, HIGH);
  delayMicroseconds(5);

  pinMode(PIN_I2C_SDA, INPUT_PULLUP);
  pinMode(PIN_I2C_SCL, INPUT_PULLUP);
}

static void i2cBegin() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(20);   // ms. A wedged bus must not stall the loop: the budget is 5 ms.
}

static bool pcaWriteBytes(uint8_t reg, const uint8_t* data, uint8_t len) {
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(reg);
  for (uint8_t i = 0; i < len; ++i) {
    Wire.write(data[i]);
  }
  return Wire.endTransmission() == 0;
}

static bool pcaWriteRegister(uint8_t reg, uint8_t value) {
  return pcaWriteBytes(reg, &value, 1);
}

static bool pcaReadRegister(uint8_t reg, uint8_t* out) {
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) {
    return false;
  }
  if (Wire.requestFrom(static_cast<uint8_t>(PCA9685_ADDR), static_cast<uint8_t>(1)) != 1) {
    return false;
  }
  *out = static_cast<uint8_t>(Wire.read());
  return true;
}

/** 12-bit tick count for a pulse width. Integer maths: no float in the reflex path. */
static inline uint16_t pulseToTicks(uint16_t pulse_us) {
  return static_cast<uint16_t>((static_cast<uint32_t>(pulse_us) * 4096u + 10000u) /
                               PWM_PERIOD_US);
}

/**
 * One transaction for a contiguous run of channels: the PCA9685 auto-increments the register
 * pointer, so a single Write covers LEDn_ON_L .. LED(n+count-1)_OFF_H.
 *
 * `pulses_us` is ONLY ever the shadow array or a stack copy of it - never an array with
 * uninitialised entries, which would command a servo to a random pulse width.
 */
static bool pcaWriteChannels(uint8_t first_ch, const uint16_t* pulses_us, uint8_t count) {
  if (count == 0 || static_cast<uint16_t>(first_ch) + count > 16) {
    return false;
  }
  uint8_t payload[16 * 4];
  for (uint8_t i = 0; i < count; ++i) {
    const uint16_t ticks = pulseToTicks(pulses_us[i]);
    payload[4 * i + 0] = 0x00;                              // ON low byte: always 0
    payload[4 * i + 1] = 0x00;                              // ON high byte
    payload[4 * i + 2] = static_cast<uint8_t>(ticks & 0xFF);
    payload[4 * i + 3] = static_cast<uint8_t>(ticks >> 8);
  }
  return pcaWriteBytes(static_cast<uint8_t>(PCA9685_REG_LED0_ON_L + 4 * first_ch), payload,
                       static_cast<uint8_t>(4 * count));
}

/** Write the shadow for one channel. The ONLY way a pulse reaches a servo. */
static bool writeChannel(uint8_t ch, uint16_t pulse_us) {
  if (ch >= CHANNEL_COUNT) {
    return false;
  }
  g_pulse_us[ch] = pulse_us;
  return pcaWriteChannels(ch, &g_pulse_us[ch], 1);
}

/** Write both wheel channels in ONE transaction (~200 us at 400 kHz). The reflex fast path. */
static bool writeWheels(uint16_t left_us, uint16_t right_us) {
  g_pulse_us[WHEEL_LEFT_CH] = left_us;
  g_pulse_us[WHEEL_RIGHT_CH] = right_us;
  return pcaWriteChannels(WHEEL_LEFT_CH, &g_pulse_us[WHEEL_LEFT_CH], 2);
}

static bool pcaBegin() {
  if (!pcaWriteRegister(PCA9685_REG_MODE1, 0x10)) {    // SLEEP, so the prescale can be set
    return false;
  }
  const float prescale_f = PCA9685_OSC_HZ / (4096.0f * PWM_FREQ_HZ) - 1.0f;
  const uint8_t prescale = static_cast<uint8_t>(prescale_f + 0.5f);
  if (!pcaWriteRegister(PCA9685_REG_PRESCALE, prescale)) {
    return false;
  }
  if (!pcaWriteRegister(PCA9685_REG_MODE2, 0x04)) {    // OUTDRV: totem pole, for servos
    return false;
  }
  if (!pcaWriteRegister(PCA9685_REG_MODE1, 0x20)) {    // wake, AUTO_INCREMENT
    return false;
  }
  delayMicroseconds(500);                              // oscillator start-up
  uint8_t mode1 = 0;
  if (!pcaReadRegister(PCA9685_REG_MODE1, &mode1)) {
    return false;
  }
  return (mode1 & 0x10) == 0;                          // SLEEP must be clear
}

// ===========================================================================
// OUTPUT ENABLE
// ===========================================================================
static void setServoEnable(bool enabled) {
  g_servo_enabled = enabled;
  digitalWrite(PIN_PCA9685_OE, enabled ? LOW : HIGH);
}

static void printEvent(const char* event, const char* detail) {
  Serial.printf("{\"ok\":false,\"event\":\"%s\",\"detail\":\"%s\"}\n", event, detail);
}

static void handleBusFailure(const char* what) {
  const I2cSupervisor::Action action = g_bus.noteFailure();
  if (action == I2cSupervisor::Action::kDisableOutputs) {
    setServoEnable(false);
    Serial.printf("{\"ok\":false,\"event\":\"I2C_BUS_DEAD\",\"where\":\"%s\","
                  "\"consecutive\":%u}\n",
                  what, static_cast<unsigned>(g_bus.consecutiveFailures()));
    return;
  }
  Serial.printf("{\"ok\":false,\"event\":\"I2C_RECOVER\",\"where\":\"%s\","
                "\"consecutive\":%u}\n",
                what, static_cast<unsigned>(g_bus.consecutiveFailures()));
  i2cRecoverAndRetry();
}

static bool i2cRecoverAndRetry() {
  i2c_bus_recovery();
  i2cBegin();
  const bool pca_ok = pcaBegin();
  if (pca_ok) {
    // A PCA9685 that was just reset has all channels off. Restore the shadow and put the
    // wheels back to neutral FIRST, because "no pulse" on a continuous-rotation servo is
    // not "stop" - it is whichever direction the last pulse commanded.
    writeWheels(g_wheels.config().neutral_us, g_wheels.config().neutral_us);
    for (uint8_t ch = 0; ch < WHEEL_LEFT_CH; ++ch) {
      pcaWriteChannels(ch, &g_pulse_us[ch], 1);
    }
  }
  return pca_ok;
}

// ===========================================================================
// CLIFF REFLEX
// ===========================================================================
/**
 * The ISR does two things and nothing else: it stamps the edge with a microsecond clock
 * (`esp_timer_get_time()` is IRAM-safe) and raises a flag the loop reads. It deliberately
 * does NOT touch I2C, because `Wire` is interrupt-driven and calling it from an ISR blocks
 * or deadlocks. The <5 ms promise is kept by the LOOP being non-blocking instead, and the
 * loop geometry is simple enough to argue about: the longest blocking call anywhere in it is
 * a single I2C transaction (about 200 us), plus 10-25 us of HC-SR04 trigger handling.
 */
static void IRAM_ATTR cliffLeftIsr() {
  g_wheels.noteCliffEdge(static_cast<uint32_t>(esp_timer_get_time()));
}

static void IRAM_ATTR cliffRightIsr() {
  g_wheels.noteCliffEdge(static_cast<uint32_t>(esp_timer_get_time()));
}

/**
 * Service the reflex. Called FIRST in every loop iteration, before the serial port, so a
 * burst of incoming commands cannot delay it.
 */
static void serviceCliffReflex() {
  const uint32_t now_ms = millis();
  const uint32_t now_us = static_cast<uint32_t>(esp_timer_get_time());
  const bool cliff_left = cliffTripped(PIN_CLIFF_LEFT);
  const bool cliff_right = cliffTripped(PIN_CLIFF_RIGHT);

  const WheelSafety::Output out = g_wheels.update(now_ms, now_us, cliff_left, cliff_right);
  if (!out.changed) {
    return;
  }

  if (!writeWheels(out.left_us, out.right_us)) {
    handleBusFailure("wheel write");
    return;
  }
  g_bus.noteOk();

  if (out.recoil_active) {
    const WheelSafety::WriteReport report =
        g_wheels.noteReflexWritten(static_cast<uint32_t>(esp_timer_get_time()));
    if (report.measured) {
      Serial.printf("{\"ok\":true,\"event\":\"CLIFF_REFLEX\",\"left_us\":%u,\"right_us\":%u,"
                    "\"latency_us\":%lu,\"over_budget\":%s}\n",
                    static_cast<unsigned>(out.left_us), static_cast<unsigned>(out.right_us),
                    static_cast<unsigned long>(report.latency_us),
                    report.over_budget ? "true" : "false");
    }
  } else if (out.command_timed_out) {
    printEvent("WHEEL_TIMEOUT", "no host wheel command for 500 ms: wheels to neutral");
  }
}

// ===========================================================================
// ULTRASONIC (HC-SR04, entirely non-blocking - `pulseIn` is banned in this file)
// ===========================================================================
static void IRAM_ATTR sonarEchoIsr() {
  const uint32_t now_us = static_cast<uint32_t>(esp_timer_get_time());
  if (digitalRead(PIN_SONAR_ECHO) == HIGH) {
    g_echo_rise_us = now_us;
    g_echo_rise_seen = true;
  } else if (g_echo_rise_seen) {
    g_echo_fall_us = now_us;
    g_echo_rise_seen = false;
    g_echo_complete = true;
  }
}

static void serviceSonar() {
  const uint32_t now_ms = millis();
  const uint32_t now_us = static_cast<uint32_t>(esp_timer_get_time());

  if (g_echo_complete) {
    g_echo_complete = false;
    g_sonar_waiting = false;
    const uint32_t width_us = g_echo_fall_us - g_echo_rise_us;
    if (width_us > 0 && width_us < SONAR_TIMEOUT_US) {
      // Sound: 343 m/s, round trip. 1 us of echo = 0.1715 mm, so distance = width_us / 5.83.
      // Dividing by 6 instead reports about 3% SHORT, which errs toward stopping earlier.
      const uint32_t mm = width_us / 6u;
      g_front.noteReading(static_cast<uint16_t>(mm > 4000u ? 4000u : mm), now_ms);
    }
  }

  if (g_sonar_waiting && (now_us - g_sonar_trig_us) > SONAR_TIMEOUT_US) {
    g_sonar_waiting = false;
    ++g_sonar_timeouts;
  }

  if (!g_sonar_waiting && (now_ms - g_last_sonar_attempt_ms) >= SONAR_PERIOD_MS) {
    g_last_sonar_attempt_ms = now_ms;
    g_echo_rise_seen = false;
    digitalWrite(PIN_SONAR_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_SONAR_TRIG, HIGH);
    delayMicroseconds(10);        // bounded and tiny: this is the only delay in the loop
    digitalWrite(PIN_SONAR_TRIG, LOW);
    g_sonar_trig_us = static_cast<uint32_t>(esp_timer_get_time());
    g_sonar_waiting = true;
  }
}

// ===========================================================================
// CURRENT AND VOLTAGE (INA219 read straight from Wire, so a NACK is visible)
// ===========================================================================
static bool inaWriteRegister(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(INA219_ADDR);
  Wire.write(reg);
  Wire.write(static_cast<uint8_t>(value >> 8));
  Wire.write(static_cast<uint8_t>(value & 0xFF));
  return Wire.endTransmission() == 0;
}

static bool inaReadRegister(uint8_t reg, uint16_t* out) {
  Wire.beginTransmission(INA219_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) {
    return false;
  }
  if (Wire.requestFrom(static_cast<uint8_t>(INA219_ADDR), static_cast<uint8_t>(2)) != 2) {
    return false;
  }
  const uint8_t hi = static_cast<uint8_t>(Wire.read());
  const uint8_t lo = static_cast<uint8_t>(Wire.read());
  *out = (static_cast<uint16_t>(hi) << 8) | lo;
  return true;
}

static bool inaBegin() {
  // 32 V range, /8 gain (+/-320 mV), 12-bit, 532 us conversions, continuous shunt + bus.
  if (!inaWriteRegister(INA219_REG_CONFIG, 0x399F)) {
    return false;
  }
  // cal = 0.04096 / (current_lsb * r_shunt): 0.1 ohm shunt, 100 uA per bit -> 4096.
  // That gives current_ma = raw * 0.1 and 4 mV per bus-voltage bit.
  return inaWriteRegister(INA219_REG_CALIBRATION, 4096);
}

static void serviceIna219() {
  const uint32_t now_ms = millis();
  if (now_ms - g_last_ina_ms < INA_PERIOD_MS) {
    return;
  }
  g_last_ina_ms = now_ms;
  if (!g_ina_present) {
    return;
  }

  uint16_t raw = 0;
  if (!inaReadRegister(INA219_REG_BUS_VOLTAGE, &raw)) {
    handleBusFailure("ina219 bus voltage");
    return;
  }
  g_bus.noteOk();
  g_bus_v = static_cast<float>(raw >> 3) * 0.004f;

  if (inaReadRegister(INA219_REG_CURRENT, &raw)) {
    g_current_ma = static_cast<float>(static_cast<int16_t>(raw)) * 0.1f;
  }

  // The physical e-stop removes servo power, so the INA219 sees about 0 V. Latch it.
  if (g_bus_v <= ESTOP_V && !g_estop_latched) {
    g_estop_latched = true;
    setServoEnable(false);
    printEvent("ESTOP_LATCHED", "bus voltage about 0: the hardware e-stop is open");
  }
  if (!g_estop_latched && g_bus_v < BROWNOUT_V && g_servo_enabled) {
    setServoEnable(false);
    printEvent("BROWNOUT", "rail below 4.90 V: servo torque dropped, the arm may sag");
  }
}

// ===========================================================================
// COSINE BLEND (arm channels only - see the NOTE ON BLENDING in the header)
// ===========================================================================
static void noteBlend(uint8_t ch, uint16_t target_us, uint32_t duration_ms) {
  if (ch >= WHEEL_LEFT_CH) {
    return;                                     // wheels are never blended
  }
  Blend& b = g_blend[ch];
  if (!b.active) {
    b.start_us = g_pulse_us[ch];
  } else {
    const uint32_t elapsed = millis() - b.start_ms;
    if (elapsed >= b.duration_ms) {
      b.start_us = b.target_us;
    } else {
      const float progress = static_cast<float>(elapsed) / static_cast<float>(b.duration_ms);
      const float blend = 0.5f * (1.0f - cosf(PI * progress));
      b.start_us = static_cast<uint16_t>(b.start_us + blend * (b.target_us - b.start_us));
    }
  }
  b.target_us = target_us;
  b.start_ms = millis();
  b.duration_ms = duration_ms < 20 ? 20 : duration_ms;
  b.active = true;
}

static void serviceBlend() {
  const uint32_t now_us = static_cast<uint32_t>(esp_timer_get_time());
  if (now_us - g_last_blend_us < BLEND_PERIOD_US) {
    return;
  }
  g_last_blend_us = now_us;

  bool any = false;
  for (uint8_t ch = 0; ch < WHEEL_LEFT_CH; ++ch) {
    Blend& b = g_blend[ch];
    if (!b.active) {
      continue;
    }
    any = true;
    const uint32_t elapsed = millis() - b.start_ms;
    uint16_t pulse = b.target_us;
    if (elapsed < b.duration_ms) {
      const float progress = static_cast<float>(elapsed) / static_cast<float>(b.duration_ms);
      const float blend = 0.5f * (1.0f - cosf(PI * progress));
      pulse = static_cast<uint16_t>(b.start_us + blend * (b.target_us - b.start_us));
    } else {
      b.active = false;
    }
    g_pulse_us[ch] = pulse;
  }

  // One transaction for all six arm channels, composed from the shadow - never from a
  // partially filled array.
  if (any && !pcaWriteChannels(0, g_pulse_us, WHEEL_LEFT_CH)) {
    handleBusFailure("blend write");
    return;
  }
  if (any) {
    g_bus.noteOk();
  }
}

// ===========================================================================
// SERIAL COMMANDS
// ===========================================================================
static uint16_t clampArmPulse(long us) {
  if (us < 500) { return 500; }
  if (us > 2500) { return 2500; }
  return static_cast<uint16_t>(us);
}

static uint16_t clampWheelPulse(long us) {
  if (us < 1000) { return 1000; }
  if (us > 2000) { return 2000; }
  return static_cast<uint16_t>(us);
}

static bool wheelForwardAllowed(uint16_t pulse_us, uint8_t ch, uint32_t now_ms) {
  const bool forward_above = (ch == WHEEL_LEFT_CH)
                                 ? g_wheels.config().left_forward_is_above_neutral
                                 : g_wheels.config().right_forward_is_above_neutral;
  if (!g_wheels.isForward(pulse_us, forward_above)) {
    return true;                                  // reversing away is always allowed
  }
  if (cliffTripped(PIN_CLIFF_LEFT) || cliffTripped(PIN_CLIFF_RIGHT)) {
    return false;
  }
  return g_front.allowForward(now_ms);
}

/**
 * THE ONE PLACE A HOST WHEEL COMMAND REACHES THE SERVOS.
 *
 * Both `W` and the wheel entries inside `M` go through here, because keeping two copies of
 * this logic is how the first version of this file shipped a hole: `W` refreshed the liveness
 * timer and wrote the pulses but never asked the cliff sensors, so `W 1320 1680` would drive
 * straight off an edge that `M 6:1320,7:1680` politely refused. The host test caught it.
 *
 * The gate is only on FORWARD. Reversing away from an edge is always allowed, and the reflex
 * in `serviceCliffReflex()` deliberately does NOT come through here - a safety response must
 * never be filtered by its own interlock.
 */
static void applyWheelPair(uint16_t left_us, uint16_t right_us, const char* source) {
  const uint32_t now_ms = millis();
  bool refused = false;

  if (!wheelForwardAllowed(left_us, WHEEL_LEFT_CH, now_ms)) {
    left_us = g_wheels.config().neutral_us;
    refused = true;
  }
  if (!wheelForwardAllowed(right_us, WHEEL_RIGHT_CH, now_ms)) {
    right_us = g_wheels.config().neutral_us;
    refused = true;
  }

  if (!writeWheels(left_us, right_us)) {
    handleBusFailure(source);
    return;
  }
  g_bus.noteOk();
  g_wheels.noteHostWheelCommand(now_ms, left_us, right_us);

  if (refused) {
    printEvent("FORWARD_REFUSED", "cliff or obstacle in the way");
  }
  Serial.printf("{\"ok\":true,\"wheel_cmd\":[%u,%u],\"source\":\"%s\"}\n",
                static_cast<unsigned>(left_us), static_cast<unsigned>(right_us), source);
}

/**
 * `M <ch>:<us>,...` and `G <ms> <ch>:<us>,...`
 *
 * Wheel channels are special: they refresh the liveness timer, they are clamped to the
 * continuous-rotation range, they are never blended, and a FORWARD pulse is refused while a
 * cliff pin is HIGH or the front sensor reports something closer than 80 mm.
 */
static void handleChannelList(const char* p, uint32_t duration_ms, bool blended) {
  // Seeded from the shadow, NOT left uninitialised: `M 6:1320` mentions only one wheel, and
  // `applyWheelPair()` writes BOTH wheel channels as one transaction. Without this seed the
  // right wheel would receive whatever was on the stack.
  uint16_t wanted[CHANNEL_COUNT];
  for (uint8_t ch = 0; ch < CHANNEL_COUNT; ++ch) {
    wanted[ch] = g_pulse_us[ch];
  }
  bool pending[CHANNEL_COUNT] = {false};
  int pairs = 0;
  bool wheel_pair_ready = false;

  for (uint8_t ch = 0; ch < CHANNEL_COUNT; ++ch) {
    wanted[ch] = g_pulse_us[ch];                  // safe default: no change
  }

  while (*p != '\0' && pairs < CHANNEL_COUNT) {
    while (*p == ' ' || *p == ',') {
      ++p;
    }
    if (*p < '0' || *p > '9') {
      break;
    }
    long ch = 0;
    while (*p >= '0' && *p <= '9') {
      ch = ch * 10 + (*p - '0');
      ++p;
    }
    if (*p != ':') {
      break;
    }
    ++p;
    long us = 0;
    while (*p >= '0' && *p <= '9') {
      us = us * 10 + (*p - '0');
      ++p;
    }
    if (ch >= CHANNEL_COUNT) {
      printEvent("BAD_CHANNEL", "channel outside 0..7");
      return;
    }
    const bool is_wheel = (ch == WHEEL_LEFT_CH || ch == WHEEL_RIGHT_CH);
    uint16_t pulse = is_wheel ? clampWheelPulse(us) : clampArmPulse(us);
    if (is_wheel) {
      if (blended) {
        printEvent("WHEEL_NOT_BLENDED", "wheel channels are immediate; duration ignored");
        blended = false;
      }
    }
    wanted[ch] = pulse;
    pending[ch] = true;
    ++pairs;
  }

  if (pairs == 0) {
    printEvent("BAD_COMMAND", "no ch:us pairs");
    return;
  }

  // Wheels first, as one transaction, through the same guarded path `W` uses.
  const bool touched_left = pending[WHEEL_LEFT_CH];
  const bool touched_right = pending[WHEEL_RIGHT_CH];
  if (touched_left || touched_right) {
    applyWheelPair(wanted[WHEEL_LEFT_CH], wanted[WHEEL_RIGHT_CH], "M/G");
    wheel_pair_ready = true;
  }

  // Then the arm channels: blended over time, or immediate.
  for (uint8_t ch = 0; ch < WHEEL_LEFT_CH; ++ch) {
    if (!pending[ch]) {
      continue;
    }
    if (blended) {
      noteBlend(ch, wanted[ch], duration_ms);
    } else if (!writeChannel(ch, wanted[ch])) {
      handleBusFailure("channel command");
    } else {
      g_bus.noteOk();
    }
  }

  Serial.printf("{\"ok\":true,\"set\":%d,\"blended\":%s,\"wheels\":[%u,%u]%s}\n",
                pairs, blended ? "true" : "false",
                static_cast<unsigned>(g_pulse_us[WHEEL_LEFT_CH]),
                static_cast<unsigned>(g_pulse_us[WHEEL_RIGHT_CH]),
                wheel_pair_ready ? "" : ",\"note\":\"wheels untouched\"");
}

static void printStatus() {
  Serial.printf("{\"ok\":true,\"fw\":\"BUDE-ESP32-full-1.0\",\"bus_v\":%.2f,\"current_ma\":%.1f,"
                "\"cliff\":[%d,%d],\"distance_mm\":%u,\"sonar_fresh\":%s,\"sonar_timeouts\":%lu,"
                "\"estop_latched\":%s,\"outputs\":\"%s\",\"loop\":%lu,"
                "\"wheels\":[%u,%u],\"recoil_active\":%s,\"recoils\":%lu,"
                "\"reflex_max_us\":%lu,\"reflex_last_us\":%lu,\"reflex_over_budget\":%lu,"
                "\"i2c_recoveries\":%lu,\"i2c_consecutive_failures\":%u,"
                "\"wdt_ms\":%lu,\"adc34\":%u}\n",
                g_bus_v, g_current_ma,
                cliffTripped(PIN_CLIFF_LEFT) ? 1 : 0,
                cliffTripped(PIN_CLIFF_RIGHT) ? 1 : 0,
                static_cast<unsigned>(g_front.distanceMm()),
                g_front.readingFresh(millis()) ? "true" : "false",
                static_cast<unsigned long>(g_sonar_timeouts),
                g_estop_latched ? "true" : "false",
                g_servo_enabled ? "on" : "off",
                static_cast<unsigned long>(g_loop_iterations),
                static_cast<unsigned>(g_wheels.leftUs()),
                static_cast<unsigned>(g_wheels.rightUs()),
                g_wheels.recoilActive() ? "true" : "false",
                static_cast<unsigned long>(g_wheels.recoilCount()),
                static_cast<unsigned long>(g_wheels.maxLatencyUs()),
                static_cast<unsigned long>(g_wheels.lastLatencyUs()),
                static_cast<unsigned long>(g_wheels.overBudgetCount()),
                static_cast<unsigned long>(g_bus.recoveryCount()),
                static_cast<unsigned>(g_bus.consecutiveFailures()),
                static_cast<unsigned long>(WATCHDOG_TIMEOUT_MS),
                analogRead(PIN_SERVO_ADC_CH0));
}

static void printHelp() {
  Serial.println("{\"ok\":true,\"commands\":[\"PING\",\"W <left_us> <right_us>\","
                 "\"M <ch>:<us>[,<ch>:<us>]\",\"G <ms> <ch>:<us>[,<ch>:<us>]\","
                 "\"OE <0|1>\",\"STATUS\",\"HELP\"]}");
}

static void dispatchLine(const char* line) {
  if (line == NULL || line[0] == '\0') {
    return;
  }

  const WheelCommand wc = parseWheelCommand(line);
  if (wc.valid) {
    applyWheelPair(wc.left_us, wc.right_us, "W");
    return;
  }

  const char head = static_cast<char>(tolower(line[0]));
  if (head == 'm') {
    handleChannelList(line + 1, 0, false);
    return;
  }
  if (head == 'g') {
    const char* p = line + 1;
    while (*p == ' ') {
      ++p;
    }
    long ms = 0;
    while (*p >= '0' && *p <= '9') {
      ms = ms * 10 + (*p - '0');
      ++p;
    }
    if (ms <= 0) {
      printEvent("BAD_COMMAND", "G needs a duration in milliseconds");
      return;
    }
    handleChannelList(p, static_cast<uint32_t>(ms), true);
    return;
  }
  if (head == 'o') {
    const char* p = line + 1;
    while (*p == ' ') {
      ++p;
    }
    if (*p == '1') {
      setServoEnable(false);
      Serial.println("{\"ok\":true,\"outputs\":\"off\"}");
    } else if (*p == '0') {
      if (g_estop_latched) {
        printEvent("ESTOP_LATCHED", "release the physical e-stop before enabling outputs");
        return;
      }
      setServoEnable(true);
      Serial.println("{\"ok\":true,\"outputs\":\"on\"}");
    } else {
      printEvent("BAD_COMMAND", "OE takes 0 or 1");
    }
    return;
  }
  if (head == 'p') {
    Serial.printf("{\"ok\":true,\"fw\":\"BUDE-ESP32-full-1.0\",\"pca9685\":\"0x%02X\","
                  "\"ina219\":\"%s\",\"wheel_channels\":[%u,%u],\"recoil_ms\":%lu,"
                  "\"host_timeout_ms\":%lu}\n",
                  PCA9685_ADDR, g_ina_present ? "0x41" : "absent",
                  static_cast<unsigned>(WHEEL_LEFT_CH), static_cast<unsigned>(WHEEL_RIGHT_CH),
                  static_cast<unsigned long>(g_wheels.config().recoil_ms),
                  static_cast<unsigned long>(g_wheels.config().command_timeout_ms));
    return;
  }
  if (head == 's') {
    printStatus();
    return;
  }
  if (head == 'h') {
    printHelp();
    return;
  }
  printEvent("UNKNOWN_COMMAND", "send HELP");
}

// ===========================================================================
// WATCHDOG (FMEA #11)
// ===========================================================================
/**
 * Two independent liveness guards:
 *
 *   a) The ESP32 TASK WATCHDOG. If this loop ever stops reaching `esp_task_wdt_reset()` for
 *      5 s - a wedged I2C call, an infinite loop, a dead task - the chip panics and reboots,
 *      and `setup()` comes up with OE HIGH (outputs tri-stated) before it does anything else.
 *   b) The 500 ms WHEEL-COMMAND TIMEOUT inside `WheelSafety`. A host can be alive while its
 *      control loop is not, and that case never reaches a task watchdog.
 *
 * Neither is a safety device. The servo rail's own e-stop remains the only thing that
 * protects a person.
 */
static void taskWatchdogBegin() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t cfg = {};
  cfg.timeout_ms = WATCHDOG_TIMEOUT_MS;
  cfg.idle_core_mask = 0;
  cfg.trigger_panic = true;
  const esp_err_t err = esp_task_wdt_init(&cfg);
  if (err == ESP_ERR_INVALID_STATE) {
    esp_task_wdt_reconfigure(&cfg);              // the core already initialised it
  }
#else
  const esp_err_t err = esp_task_wdt_init(WATCHDOG_TIMEOUT_MS / 1000, true);
  (void)err;
#endif
  g_wdt_ready = (esp_task_wdt_add(NULL) == ESP_OK);
}

// ===========================================================================
// SETUP AND LOOP
// ===========================================================================
void setup() {
  // 1. THE SAFE STATE FIRST, before Serial, before I2C, before anything can move.
  pinMode(PIN_PCA9685_OE, OUTPUT);
  digitalWrite(PIN_PCA9685_OE, HIGH);
  g_servo_enabled = false;

  pinMode(PIN_CLIFF_LEFT, INPUT_PULLUP);
  pinMode(PIN_CLIFF_RIGHT, INPUT_PULLUP);
  pinMode(PIN_SONAR_TRIG, OUTPUT);
  digitalWrite(PIN_SONAR_TRIG, LOW);
  pinMode(PIN_SONAR_ECHO, INPUT);

  Serial.begin(115200);

  // 2. FMEA #12: clear a bus that a previous reset left held low, THEN start Wire.
  i2c_bus_recovery();
  i2cBegin();

  const bool pca_ok = pcaBegin();
  g_ina_present = inaBegin();

  // 3. Everything neutral, in the shadow and on the wire, before outputs are ever enabled.
  for (uint8_t ch = 0; ch < CHANNEL_COUNT; ++ch) {
    g_pulse_us[ch] = 1500;
    g_blend[ch] = Blend();
  }
  pcaWriteChannels(0, g_pulse_us, CHANNEL_COUNT);
  g_wheels.begin(millis());
  g_wheels.update(millis(), static_cast<uint32_t>(esp_timer_get_time()), false, false);

  // 4. Interrupts and the watchdog.
  // Watch the transition INTO the tripped state, whichever level that is on your modules.
  const int trip_edge = CLIFF_TRIPPED_IS_HIGH ? RISING : FALLING;
  attachInterrupt(digitalPinToInterrupt(PIN_CLIFF_LEFT), cliffLeftIsr, trip_edge);
  attachInterrupt(digitalPinToInterrupt(PIN_CLIFF_RIGHT), cliffRightIsr, trip_edge);
  attachInterrupt(digitalPinToInterrupt(PIN_SONAR_ECHO), sonarEchoIsr, CHANGE);
  taskWatchdogBegin();

  // 5. Rover builds that always carry the HC-SR04 should require it: set true so a dead
  //    sensor fails closed instead of driving blind. Left false here so the stationary arm
  //    can be bench-tested with no sonar fitted at all.
  g_front.requireSensor(false);

  Serial.printf("{\"ok\":true,\"event\":\"BOOT\",\"fw\":\"BUDE-ESP32-full-1.0\","
                "\"pca9685\":%s,\"ina219\":%s,\"outputs\":\"off\","
                "\"cliff_pins\":[%d,%d],\"cliff_tripped_is_high\":%s,"
                "\"wheel_channels\":[%u,%u],\"recoil_ms\":%lu,"
                "\"host_timeout_ms\":%lu,\"wdt_ms\":%lu}\n",
                pca_ok ? "true" : "false", g_ina_present ? "true" : "false",
                PIN_CLIFF_LEFT, PIN_CLIFF_RIGHT,
                CLIFF_TRIPPED_IS_HIGH ? "true" : "false",
                static_cast<unsigned>(WHEEL_LEFT_CH),
                static_cast<unsigned>(WHEEL_RIGHT_CH),
                static_cast<unsigned long>(g_wheels.config().recoil_ms),
                static_cast<unsigned long>(g_wheels.config().command_timeout_ms),
                static_cast<unsigned long>(WATCHDOG_TIMEOUT_MS));
  printHelp();
}

void loop() {
  ++g_loop_iterations;

  // Order matters: the reflex is served before anything that can block on the outside world.
  serviceCliffReflex();
  serviceSonar();
  serviceIna219();
  serviceBlend();

  while (Serial.available() > 0) {
    if (g_rx.feed(static_cast<char>(Serial.read()))) {
      dispatchLine(g_rx.line());
      g_rx.clear();
    }
  }

  if (g_wdt_ready) {
    esp_task_wdt_reset();
  }
}
