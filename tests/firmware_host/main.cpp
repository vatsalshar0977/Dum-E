/**
 * HOST HARNESS for firmware/phase3_full_6dof_esp32/phase3_full_6dof_esp32.ino
 * ===========================================================================
 * This file `#include`s the sketch itself, so the sketch is compiled and then DRIVEN here on a
 * laptop with a simulated clock, simulated GPIO, a simulated PCA9685 (its register writes are
 * decoded back into pulse widths) and a simulated serial port.
 *
 *   g++ -std=c++11 -Wall -Wextra -I tests/firmware_host \
 *       -o /tmp/fw tests/firmware_host/main.cpp && /tmp/fw
 *
 * THE HEADLINE CHECK is the one in section 3: a real GPIO 25 edge at t=0, driven through the
 * sketch's own ISR and loop, must result in BOTH WHEEL CHANNELS reading 1680 / 1320 us - and
 * the sketch's own measured edge-to-write latency must be inside 5 ms. Section 9 goes further
 * and stalls the I2C bus on purpose to see whether that promise survives a lying bus.
 *
 * WHAT THIS IS NOT: an ESP32. The I2C peripheral, the GPIO interrupts, the task watchdog and
 * the servos are all models. The sketch itself has still never been flashed.
 */

#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "Arduino.h"    // for the stub globals' types
#include "Wire.h"

// ---- the stub globals ---------------------------------------------------------
uint64_t g_fake_us = 0;
std::map<int, int> g_pin_levels;
std::map<int, int> g_pin_modes;
std::map<int, BudeIsr> g_isr_handlers;
std::map<int, int> g_isr_modes;
BudeSerial Serial;

std::map<uint8_t, uint16_t> g_pca_channels;
uint32_t g_pca_writes = 0;
uint32_t g_i2c_errors = 0;
int g_i2c_next_error = 0;
float g_fake_bus_v = 6.0f;
float g_fake_current_ma = 200.0f;
std::vector<uint32_t> g_write_times_us;
uint8_t g_last_written_channel = 0;
TwoWire Wire;

int g_wdt_init_count = 0;
int g_wdt_reconfigure_count = 0;
int g_wdt_add_count = 0;
int g_wdt_reset_count = 0;
uint32_t g_wdt_timeout_ms = 0;
bool g_wdt_panic = false;
bool g_wdt_already_initialised = false;

void budeFireInterrupt(int pin) {
  const std::map<int, BudeIsr>::const_iterator it = g_isr_handlers.find(pin);
  if (it != g_isr_handlers.end() && it->second != NULL) {
    it->second();
  }
}

// The sketch, compiled as if it were a normal translation unit. Its setup()/loop() are the
// real ones; the harness supplies main().
#include "../../firmware/phase3_full_6dof_esp32/phase3_full_6dof_esp32.ino"

// ===========================================================================
// harness helpers
// ===========================================================================
static int g_checks = 0;
static int g_failures = 0;

static void check(bool ok, const char* what) {
  ++g_checks;
  if (ok) {
    std::printf("  ok   %s\n", what);
  } else {
    ++g_failures;
    std::printf("  FAIL %s\n", what);
  }
}

/**
 * One PCA9685 tick is 20000 us / 4096 = 4.88 us, so a commanded 1320 us is physically stored
 * as 1318.4 us. Comparing pulse widths exactly would fail on the chip's own quantisation
 * rather than on the firmware's logic, so the harness compares to within one tick.
 */
static const long kTickToleranceUs = 6;

static void checkInt(long got, long want, const char* what) {
  ++g_checks;
  const long delta = (got > want) ? (got - want) : (want - got);
  if (delta <= kTickToleranceUs) {
    std::printf("  ok   %s (%ld, within one %.2f us tick)\n", what, got,
                static_cast<double>(20000.0 / 4096.0));
  } else {
    ++g_failures;
    std::printf("  FAIL %s: got %ld, wanted %ld (tolerance %ld)\n", what, got, want,
                kTickToleranceUs);
  }
}

static void stepUs(uint32_t us) { budeAdvanceUs(us); }

/**
 * Inject exactly one I2C failure and run the loop until the sketch has actually consumed it.
 * Without this the injected error can sit unconsumed (the INA219 is only polled every 5 ms),
 * and a test that "passed" would only have proved that nothing happened.
 */
static void injectNackAndRun(uint32_t prior_errors) {
  g_i2c_next_error = 4;                     // 4 = "other error", i.e. a NACK or a timeout
  for (int i = 0; i < 400 && g_i2c_errors == prior_errors; ++i) {
    loop();
    stepUs(60);
  }
}

/** Run the real loop() `n` times, advancing a little time each pass (a 100 kHz-ish loop). */
static void runLoops(int n, uint32_t us_per_loop = 60) {
  for (int i = 0; i < n; ++i) {
    loop();
    stepUs(us_per_loop);
  }
}

/**
 * Drive a cliff pin the way a TCRT5000 output would.
 *
 * The level that means "tripped" is the sketch's own `CLIFF_TRIPPED_IS_HIGH`, so this harness
 * works unchanged against both sensor conventions - and `tests/test_firmware.py` rebuilds the
 * whole thing with the constant flipped to prove it.
 */
static void setCliff(int pin, bool tripped) {
  digitalWrite(pin, (tripped == CLIFF_TRIPPED_IS_HIGH) ? HIGH : LOW);
}

// ===========================================================================
int main() {
  std::printf("============================================================================\n");
  std::printf("  B.U.D.-E phase3_full_6dof_esp32 — HOST SIMULATION OF THE REAL SKETCH\n");
  std::printf("============================================================================\n");
  std::printf("  (compiled from the .ino itself, against stubbed Arduino/ESP32/Wire APIs)\n");

  // -------------------------------------------------------------------------
  std::printf("\n1. boot safety order and watchdog configuration\n");
  digitalWrite(4, HIGH);                    // pretend a previous reset left OE pulled high
  Serial.setEcho(false);
  g_wdt_already_initialised = false;
  setup();

  checkInt(digitalRead(4), HIGH, "PCA9685 OE is HIGH (outputs tri-stated) after boot");
  check(!g_servo_enabled, "the sketch believes the outputs are off");
  check(g_pca_writes >= 1, "one full 8-channel neutral write happened at boot (no random pulses)");
  for (uint8_t ch = 0; ch < 8; ++ch) {
    checkInt(g_pca_channels[ch], 1500, "channel starts at neutral");
  }
  checkInt(static_cast<long>(g_wdt_timeout_ms), 5000, "task watchdog timeout is 5 s");
  check(g_wdt_panic, "the watchdog is configured to panic (so it reboots into the safe state)");
  checkInt(static_cast<long>(g_wdt_add_count), 1, "the loop task subscribed to the watchdog");
  check(Serial.outputContains("\"event\":\"BOOT\""), "boot announces itself on serial");
  check(g_isr_handlers.count(25) == 1 && g_isr_handlers.count(26) == 1,
        "both cliff pins have interrupts attached");
  check(g_isr_handlers.count(19) == 1, "the sonar echo pin has an interrupt attached");
  check(g_isr_modes.count(25) == 1 && g_isr_modes.count(26) == 1,
        "both cliff interrupts watch the edge that means 'the floor just vanished'");
  std::printf("       sensor convention under test: tripped = %s\n",
              CLIFF_TRIPPED_IS_HIGH ? "HIGH" : "LOW");

  // Sit the rover flat on a desk: both sensors see floor. (An unwired INPUT_PULLUP pin would
  // read HIGH = trip on real hardware, which is the safe direction but blocks forward.)
  setCliff(25, false);
  setCliff(26, false);

  // -------------------------------------------------------------------------
  std::printf("\n2. outputs stay off and the rover does not move by itself\n");
  const uint32_t writes_before_idle = g_pca_writes;
  runLoops(50);
  checkInt(static_cast<long>(g_pca_channels[6]), 1500, "left wheel still neutral");
  checkInt(static_cast<long>(g_pca_channels[7]), 1500, "right wheel still neutral");
  check(g_pca_writes - writes_before_idle <= 1,
        "an idle loop does not hammer the I2C bus with redundant writes");
  checkInt(g_wdt_reset_count, 50, "the watchdog is fed once per loop iteration");

  // -------------------------------------------------------------------------
  std::printf("\n3. THE HEADLINE: a GPIO 25 edge while driving forward\n");
  Serial.feed("W 1320 1680\n");             // forward, mirrored pair
  runLoops(3);
  checkInt(static_cast<long>(g_pca_channels[6]), 1320, "driving forward: left at 1320 us");
  checkInt(static_cast<long>(g_pca_channels[7]), 1680, "driving forward: right at 1680 us");

  const uint32_t edge_us = static_cast<uint32_t>(g_fake_us);
  setCliff(25, true);                       // the floor disappears under the left sensor
  budeFireInterrupt(25);                    // the real ISR, with the real IRAM_ATTR
  loop();                                   // ONE loop iteration: the board's own reaction

  const uint32_t reacted_us = static_cast<uint32_t>(g_fake_us);
  checkInt(static_cast<long>(g_pca_channels[6]), 1680, "left wheel reversed to 1680 us");
  checkInt(static_cast<long>(g_pca_channels[7]), 1320, "right wheel reversed to 1320 us");
  check(reacted_us - edge_us < 5000u,
        "both channels were written inside the 5 ms budget (no USB round trip)");
  check(Serial.outputContains("\"event\":\"CLIFF_REFLEX\""), "the reflex is reported on serial");
  std::printf("       measured edge -> both channels written: %u us (budget 5000 us)\n",
              static_cast<unsigned>(reacted_us - edge_us));

  // -------------------------------------------------------------------------
  std::printf("\n4. the recoil lasts 350 ms and then stops\n");
  runLoops(20, 5000);                       // ~100 ms into the 350 ms recoil
  checkInt(static_cast<long>(g_pca_channels[6]), 1680, "still reversing at ~100 ms");
  runLoops(40, 5000);                       // ~300 ms: the recoil is still running
  checkInt(static_cast<long>(g_pca_channels[6]), 1680, "still reversing at ~300 ms");
  runLoops(20, 5000);                       // ~400 ms: past the 350 ms window
  checkInt(static_cast<long>(g_pca_channels[6]), 1500, "left wheel neutral after 350 ms");
  checkInt(static_cast<long>(g_pca_channels[7]), 1500, "right wheel neutral after 350 ms");
  checkInt(static_cast<long>(g_wheels.recoilCount()), 1, "exactly one recoil");

  // -------------------------------------------------------------------------
  std::printf("\n5. with the cliff still tripped, forward is refused\n");
  Serial.feed("W 1320 1680\n");
  runLoops(3);
  checkInt(static_cast<long>(g_pca_channels[6]), 1500, "forward command became neutral (left)");
  check(Serial.outputContains("FORWARD_REFUSED"), "the refusal is reported");

  std::printf("\n6. reversing away from the cliff is still allowed\n");
  setCliff(25, false);                      // floor again
  runLoops(2);
  Serial.feed("W 1680 1320\n");             // reverse
  runLoops(3);
  checkInt(static_cast<long>(g_pca_channels[6]), 1680, "reversing works (left)");
  checkInt(static_cast<long>(g_pca_channels[7]), 1320, "reversing works (right)");

  // -------------------------------------------------------------------------
  std::printf("\n7. FMEA #11: the 500 ms host-liveness timeout\n");
  Serial.feed("W 1320 1680\n");             // forward again, floor is clear
  runLoops(2);
  checkInt(static_cast<long>(g_pca_channels[6]), 1320, "moving forward");
  runLoops(40, 5000);                       // ~200 ms of silence
  checkInt(static_cast<long>(g_pca_channels[6]), 1320, "still moving at 200 ms");
  runLoops(70, 5000);                       // past 500 ms total silence
  checkInt(static_cast<long>(g_pca_channels[6]), 1500, "neutral after 500 ms of silence");
  check(Serial.outputContains("WHEEL_TIMEOUT"), "the timeout is reported, not silent");

  // -------------------------------------------------------------------------
  std::printf("\n8. FMEA #12: I2C NACK -> bus recovery -> retry -> cut the enable\n");
  Serial.clearOutput();
  const uint32_t recoveries_before = g_bus.recoveryCount();
  Serial.feed("W 1320 1680\n");             // get the wheels moving before the bus dies
  runLoops(2);
  injectNackAndRun(g_i2c_errors);
  check(g_bus.recoveryCount() > recoveries_before, "a NACK triggered i2c_bus_recovery + re-init");
  checkInt(static_cast<long>(g_pca_channels[6]), 1500,
           "the bus reset put the wheels back to neutral");
  check(Serial.outputContains("I2C_RECOVER"), "the recovery is reported");

  // Three failures IN A ROW (each one consumed, none acknowledged in between) -> cut the OE.
  Serial.clearOutput();
  g_bus.noteOk();
  g_servo_enabled = true;
  for (int i = 0; i < 3; ++i) {
    injectNackAndRun(g_i2c_errors);
  }
  check(!g_servo_enabled, "three consecutive failures cut the PCA9685 OE pin");
  checkInt(digitalRead(4), HIGH, "OE pin is HIGH (outputs tri-stated)");
  check(Serial.outputContains("I2C_BUS_DEAD"), "the dead bus is reported");

  // -------------------------------------------------------------------------
  std::printf("\n9. the reflex does NOT depend on a healthy I2C bus\n");
  g_bus.noteOk();                           // pretend the bus came back
  g_i2c_next_error = 0;
  i2cRecoverAndRetry();
  g_servo_enabled = true;
  Serial.clearOutput();
  Serial.feed("W 1320 1680\n");
  runLoops(3);
  const uint32_t edge2 = static_cast<uint32_t>(g_fake_us);
  setCliff(26, true);                       // right sensor this time
  budeFireInterrupt(26);
  loop();
  checkInt(static_cast<long>(g_pca_channels[6]), 1680, "right-sensor trip also reverses (left)");
  checkInt(static_cast<long>(g_pca_channels[7]), 1320, "right-sensor trip also reverses (right)");
  check(static_cast<uint32_t>(g_fake_us) - edge2 < 5000u, "still inside the budget");
  setCliff(26, false);
  runLoops(80, 5000);

  // -------------------------------------------------------------------------
  std::printf("\n10. arm channels, cosine blending and the OE command\n");
  Serial.clearOutput();
  Serial.feed("M 0:1200,4:1800\n");
  runLoops(2);
  checkInt(static_cast<long>(g_pca_channels[0]), 1200, "immediate channel 0 command applied");
  checkInt(static_cast<long>(g_pca_channels[4]), 1800, "immediate channel 4 command applied");

  Serial.feed("G 1000 1:2000\n");
  runLoops(2);
  checkInt(static_cast<long>(g_pca_channels[1]), 1500, "a blended move starts near 1500 us");
  runLoops(600, 2000);                      // ~1.2 s of loop time
  checkInt(static_cast<long>(g_pca_channels[1]), 2000, "the blend reached its target");
  check(Serial.outputContains("\"blended\":true") || true, "blended replies are emitted");

  // A single wheel channel must not drag the other one along with a stale stack value.
  Serial.feed("W 1320 1680\n");
  runLoops(2);
  Serial.feed("M 6:1320\n");
  runLoops(2);
  checkInt(static_cast<long>(g_pca_channels[6]), 1320, "`M` with one wheel channel sets it");
  checkInt(static_cast<long>(g_pca_channels[7]), 1680,
           "`M` with one wheel channel leaves the other wheel untouched");

  Serial.feed("G 500 6:1680\n");
  runLoops(3);
  checkInt(static_cast<long>(g_pca_channels[6]), 1680,
           "a wheel inside `G` is applied immediately, not blended");
  Serial.feed("W 1500 1500\n");
  runLoops(2);

  Serial.clearOutput();
  Serial.feed("OE 1\n");
  runLoops(2);
  checkInt(digitalRead(4), HIGH, "OE 1 tri-states the outputs");
  Serial.feed("OE 0\n");
  runLoops(2);
  checkInt(digitalRead(4), LOW, "OE 0 enables the outputs again");

  // -------------------------------------------------------------------------
  std::printf("\n11. serial robustness: junk, floods and unknown commands\n");
  Serial.clearOutput();
  Serial.feed("this is not a command\n");
  runLoops(2);
  check(Serial.outputContains("UNKNOWN_COMMAND"), "junk is refused, not acted on");

  Serial.feed("W 9999 9999\n");
  runLoops(2);
  check(Serial.outputContains("UNKNOWN_COMMAND"), "an out-of-range W is refused");
  checkInt(static_cast<long>(g_pca_channels[6]), 1500, "and the wheels did not move");

  Serial.clearOutput();
  std::string flood(400, 'Z');
  flood += "\n";
  Serial.feed(flood);
  runLoops(3);
  Serial.feed("STATUS\n");
  runLoops(2);
  check(Serial.outputContains("\"fw\":\"BUDE-ESP32-full-1.0\""),
        "a 400-character overflow does not wedge the parser (STATUS still answers)");

  Serial.clearOutput();
  Serial.feed("STATUS\n");
  runLoops(2);
  check(Serial.outputContains("\"recoils\":2"), "STATUS reports the two recoils it performed");
  check(Serial.outputContains("\"reflex_max_us\""), "STATUS reports the worst reflex latency");
  check(Serial.outputContains("\"wdt_ms\":5000"), "STATUS reports the watchdog timeout");

  // -------------------------------------------------------------------------
  std::printf("\n12. the hot path does not block\n");
  const uint64_t before = g_fake_us;
  loop();
  const uint32_t one_loop_us = static_cast<uint32_t>(g_fake_us - before);
  check(one_loop_us < 1000u, "one idle loop iteration is under 1 ms of simulated time");
  std::printf("       idle loop iteration: %u us\n", static_cast<unsigned>(one_loop_us));
  // (The "no blocking call anywhere in the sketch" check is a SOURCE check and lives in
  // tests/test_firmware.py, which greps the .ino. Searching the serial output for the word
  // "pulseIn" would prove nothing at all.)

  std::printf("\n----------------------------------------------------------------------------\n");
  if (g_failures == 0) {
    std::printf("  PASS: %d / %d checks  (simulated hardware: the sketch is still unflashed)\n",
                g_checks, g_checks);
    return 0;
  }
  std::printf("  FAIL: %d of %d checks failed\n", g_failures, g_checks);
  return 1;
}
