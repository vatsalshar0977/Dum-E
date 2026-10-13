/**
 * Host test for reflex_logic.h — the safety logic of phase3_full_6dof_esp32.ino, executed on
 * a laptop with g++ before the sketch ever reaches a board.
 *
 *   g++ -std=c++11 -Wall -Wextra -o /tmp/reflex_test test_reflex_logic.cpp && /tmp/reflex_test
 *
 * Everything asserted here is a time or an ordering rule that a human would otherwise have to
 * verify with a stopwatch and a live rover. The sketch itself (I2C, ISRs, the ESP32 watchdog)
 * is NOT covered by this file and has never been flashed.
 */

#include "reflex_logic.h"

#include <cstdio>
#include <cstring>

using namespace bude;

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

// ---------------------------------------------------------------------------
static void test_neutral_and_change_flags() {
  std::printf("\n1. idle wheels stay neutral and do not spam the I2C bus\n");
  WheelSafety w;
  w.begin(0);
  WheelSafety::Output o = w.update(0, 0, false, false);
  check(o.left_us == 1500 && o.right_us == 1500, "idle output is 1500/1500 us");
  check(!o.changed, "no write is requested while nothing changes");
  o = w.update(1, 1000, false, false);
  check(!o.changed, "still no write one millisecond later");

  w.noteHostWheelCommand(2, 1320, 1680);          // forward
  o = w.update(2, 2000, false, false);
  check(o.left_us == 1320 && o.right_us == 1680, "a host command is obeyed");
  check(o.changed, "a real change asks for exactly one bus write");
  o = w.update(3, 3000, false, false);
  check(!o.changed, "repeating the same command does not write again");
}

// ---------------------------------------------------------------------------
static void test_cliff_reflex_while_driving() {
  std::printf("\n2. cliff while driving forward: reverse 350 ms, then neutral\n");
  WheelSafety w;
  w.begin(0);
  w.noteHostWheelCommand(0, 1320, 1680);          // driving forward
  w.update(0, 0, false, false);

  // The ISR records the edge at t = 2.000 s, the loop sees the HIGH level in the same pass.
  w.noteCliffEdge(2000000);
  WheelSafety::Output o = w.update(2000, 2000100, true, false);
  check(o.recoil_active, "a forward-moving rover starts the recoil on the first pass");
  check(o.left_us == 1680 && o.right_us == 1320, "the recoil pair is 1680/1320 us");
  WheelSafety::WriteReport report = w.noteReflexWritten(2000150);
  check(report.measured && report.latency_us == 150,
        "edge-to-write latency is measured from the ISR timestamp (150 us here)");
  check(!report.over_budget, "150 us is inside the 5 ms budget");

  o = w.update(2300, 2300000, true, false);
  check(o.recoil_active && o.left_us == 1680, "still reversing at 300 ms");
  o = w.update(2349, 2349000, true, false);
  check(o.recoil_active, "still reversing at 349 ms");
  o = w.update(2350, 2350000, true, false);
  check(!o.recoil_active && o.left_us == 1500 && o.right_us == 1500,
        "neutral at exactly 350 ms");
  check(w.recoilCount() == 1, "exactly one recoil was issued");
}

// ---------------------------------------------------------------------------
static void test_cliff_when_not_driving_forward() {
  std::printf("\n3. a cliff while stopped or reversing does not trigger a recoil\n");
  WheelSafety w;
  w.begin(0);
  WheelSafety::Output o = w.update(0, 0, true, true);     // parked on the edge, wheels idle
  check(!o.recoil_active, "no recoil: there was no forward motion to stop");
  // The wheels are mirrored, so "reverse" is left 1680 / right 1320 and "forward" is the
  // opposite pair. Both spellings are checked here because getting this wrong is the
  // difference between backing away from an edge and pirouetting on it.
  w.noteHostWheelCommand(1, 1680, 1320);                   // already reversing
  o = w.update(2, 2000, true, true);
  check(!o.recoil_active, "no recoil while already backing away");
  check(o.left_us == 1680 && o.right_us == 1320, "the reverse command is left alone");
  WheelSafety fwd;
  fwd.begin(0);
  fwd.noteHostWheelCommand(0, 1320, 1680);                 // forward = the mirrored pair
  o = fwd.update(0, 0, false, false);
  check(o.left_us == 1320 && o.right_us == 1680, "W 1320 1680 is forward");
  check(!o.forward_blocked, "clearing away from a cliff is not blocked");
}

// ---------------------------------------------------------------------------
static void test_forward_is_blocked_over_a_cliff() {
  std::printf("\n4. forward is refused while a cliff is HIGH\n");
  WheelSafety w;
  w.begin(0);
  w.noteHostWheelCommand(0, 1320, 1680);          // forward, over a cliff
  WheelSafety::Output o = w.update(0, 0, true, false);
  // The first pass sees a forward command over a cliff: the recoil fires, then the block.
  for (int i = 0; i < 40; ++i) {
    o = w.update(10 * i + 10, 10000 * i, true, false);
  }
  check(o.forward_blocked, "the blocked flag is set");
  check(o.left_us == 1500 && o.right_us == 1500, "a forward command over a cliff becomes stop");
  w.noteHostWheelCommand(600, 1680, 1320);                 // reversing is still allowed
  o = w.update(600, 600000, true, false);
  check(o.left_us == 1680 && o.right_us == 1320, "reversing away from the cliff still works");
}

// ---------------------------------------------------------------------------
static void test_host_liveness_timeout() {
  std::printf("\n5. 500 ms of host silence stops the wheels  (FMEA #11)\n");
  WheelSafety w;
  w.begin(0);
  w.noteHostWheelCommand(0, 1320, 1680);
  WheelSafety::Output o = w.update(0, 0, false, false);
  check(o.left_us == 1320, "moving after the command");
  o = w.update(499, 499000, false, false);
  check(o.left_us == 1320, "still moving at 499 ms: the timeout has not fired");
  o = w.update(500, 500000, false, false);
  check(o.left_us == 1320, "500 ms is the boundary: a command exactly at the limit counts");
  o = w.update(501, 501000, false, false);
  check(o.left_us == 1500 && o.right_us == 1500, "neutral at 501 ms");
  check(o.command_timed_out, "the timeout is reported, not silent");
  o = w.update(502, 502000, false, false);
  check(!o.command_timed_out, "the flag is a one-shot edge, not a permanent state");

  w.noteHostWheelCommand(600, 1320, 1680);
  o = w.update(600, 600000, false, false);
  check(o.left_us == 1320, "a fresh command re-arms the wheels");
}

// ---------------------------------------------------------------------------
static void test_timeout_does_not_cut_a_recoil_short() {
  std::printf("\n6. the liveness timeout must not interrupt a recoil\n");
  WheelSafety w;
  w.begin(0);
  w.noteHostWheelCommand(0, 1320, 1680);
  w.update(0, 0, false, false);
  w.noteCliffEdge(1000000);
  w.update(1000, 1000100, true, false);                    // recoil starts at t = 1000 ms
  WheelSafety::Output o = w.update(1200, 1200000, true, false);   // 1.2 s of host silence
  check(o.left_us == 1680 && o.right_us == 1320 && o.recoil_active,
        "the recoil still owns the wheels while the host is silent");
  check(!o.command_timed_out, "the liveness timeout did not short-circuit the recoil");
  o = w.update(1349, 1349000, true, false);
  check(o.recoil_active, "still reversing at 349 ms");
  o = w.update(1350, 1350000, true, false);
  check(o.left_us == 1500 && o.right_us == 1500, "neutral when the 350 ms recoil ends");
}

// ---------------------------------------------------------------------------
static void test_latency_budget_accounting() {
  std::printf("\n7. the 5 ms budget is enforced, not assumed\n");
  WheelSafetyConfig cfg;
  WheelSafety w(cfg);
  w.begin(0);
  w.noteHostWheelCommand(0, 1320, 1680);
  w.update(0, 0, false, false);
  w.noteCliffEdge(5000000);
  w.update(5000, 5000000, true, false);
  WheelSafety::WriteReport r1 = w.noteReflexWritten(5004500);
  check(r1.measured && !r1.over_budget, "4.5 ms passes");
  check(w.maxLatencyUs() == 4500, "the worst case is remembered");

  WheelSafety w2(cfg);
  w2.begin(0);
  w2.noteHostWheelCommand(0, 1320, 1680);
  w2.update(0, 0, false, false);
  w2.noteCliffEdge(7000000);
  WheelSafety::Output o = w2.update(7000, 7000000, true, false);
  WheelSafety::WriteReport r2 = w2.noteReflexWritten(7006100);
  check(o.recoil_active && r2.measured, "the recoil write happened first");
  check(r2.over_budget, "6.1 ms is reported over budget (an I2C stall, in practice)");
  check(w2.overBudgetCount() == 1, "the over-budget count increments");
}

// ---------------------------------------------------------------------------
static void test_wheel_command_parsing() {
  std::printf("\n8. `W <left_us> <right_us>` parsing\n");
  WheelCommand c = parseWheelCommand("W 1680 1320");
  check(c.valid && c.left_us == 1680 && c.right_us == 1320, "the brief's reverse pair parses");
  c = parseWheelCommand("w 1500 1500");
  check(c.valid && c.left_us == 1500 && c.right_us == 1500, "lowercase and neutral parse");
  c = parseWheelCommand("W 1400");
  check(c.valid && c.left_us == 1400 && c.right_us == 1400, "one number sets both wheels");
  c = parseWheelCommand("  W   1320   1680  ");
  check(c.valid && c.left_us == 1320, "leading and repeated spaces are tolerated");
  check(!parseWheelCommand("W 900 1500").valid, "900 us is out of range and refused");
  check(!parseWheelCommand("W 2100 1500").valid, "2100 us is out of range and refused");
  check(!parseWheelCommand("W 1500 1500 7").valid, "trailing junk is refused");
  check(!parseWheelCommand("W abc").valid, "non-numeric is refused");
  check(!parseWheelCommand("M 1500 1500").valid, "a different command letter is refused");
  check(!parseWheelCommand("").valid, "an empty line is refused");
  check(!parseWheelCommand("W").valid, "a bare W is refused");
  check(!parseWheelCommand(NULL).valid, "NULL is refused");
}

// ---------------------------------------------------------------------------
static void test_line_accumulator_never_blocks() {
  std::printf("\n9. the serial accumulator never blocks and never overflows\n");
  LineAccumulator acc;
  const char* text = "W 1500 1500\nW 1320 1680\n";
  int lines = 0;
  for (const char* p = text; *p != '\0'; ++p) {
    if (acc.feed(*p)) {
      ++lines;
      if (lines == 1) {
        check(std::strcmp(acc.line(), "W 1500 1500") == 0, "line 1 is assembled correctly");
      }
      check(parseWheelCommand(acc.line()).valid, "each assembled line parses");
      acc.clear();
    }
  }
  check(lines == 2, "two lines from two newlines");

  LineAccumulator junk;
  for (int i = 0; i < 400; ++i) {
    junk.feed('X');                                    // a corrupt host blasting bytes
  }
  check(!junk.ready(), "400 characters without a newline produce no line");
  check(junk.feed('\n') == false, "the newline ends the discarded line without a result");
  check(!junk.overflowed(), "the accumulator resets after discarding");
  int parsed = 0;
  const char* good = "W 1000 2000\n";
  for (const char* p = good; *p != '\0'; ++p) {
    if (junk.feed(*p)) {
      parsed = 1;
    }
  }
  check(parsed == 1, "a good line right after the discard still parses");
}

// ---------------------------------------------------------------------------
static void test_i2c_failure_policy() {
  std::printf("\n10. I2C NACK -> recovery -> retry -> cut the servo enable  (FMEA #12)\n");
  I2cSupervisor bus(3);
  check(bus.noteFailure() == I2cSupervisor::Action::kRecover, "first NACK: recover and retry");
  check(bus.consecutiveFailures() == 1, "one failure counted");
  check(bus.noteFailure() == I2cSupervisor::Action::kRecover, "second NACK: recover and retry");
  check(bus.noteFailure() == I2cSupervisor::Action::kDisableOutputs,
        "third NACK in a row: the bus is dead, cut the outputs");
  check(bus.recoveryCount() == 2, "exactly two bus recoveries were asked for");
  bus.noteOk();
  check(bus.consecutiveFailures() == 0, "a good transaction clears the counter");
  check(bus.noteFailure() == I2cSupervisor::Action::kRecover, "and the policy re-arms");
}

// ---------------------------------------------------------------------------
static void test_ultrasonic_guard() {
  std::printf("\n11. front obstacle guard fails closed only after the sensor has worked\n");
  UltrasonicGuard g(80, 500);
  check(g.allowForward(0), "before any reading, a bench build without an HC-SR04 may move");
  g.noteReading(60, 1000);
  check(!g.allowForward(1000), "60 mm blocks forward motion");
  check(g.allowForward(1000) == false && g.readingFresh(1000), "the reading is fresh");
  g.noteReading(300, 1100);
  check(g.allowForward(1100), "300 mm clears the way");
  check(!g.allowForward(1601), "after 500 ms of silence it fails closed");
  check(g.everHadReading(), "the sensor is known to have worked");
  g.requireSensor(true);
  UltrasonicGuard strict(80, 500);
  strict.requireSensor(true);
  check(!strict.allowForward(0), "a rover that requires the sensor will not move without it");
  check(strict.sensorRequired(), "and it says so");
}

// ---------------------------------------------------------------------------
int main() {
  std::printf("============================================================================\n");
  std::printf("  B.U.D.-E firmware safety logic - host test (no hardware, no ESP32)\n");
  std::printf("============================================================================\n");
  test_neutral_and_change_flags();
  test_cliff_reflex_while_driving();
  test_cliff_when_not_driving_forward();
  test_forward_is_blocked_over_a_cliff();
  test_host_liveness_timeout();
  test_timeout_does_not_cut_a_recoil_short();
  test_latency_budget_accounting();
  test_wheel_command_parsing();
  test_line_accumulator_never_blocks();
  test_i2c_failure_policy();
  test_ultrasonic_guard();
  std::printf("\n----------------------------------------------------------------------------\n");
  if (g_failures == 0) {
    std::printf("  PASS: %d / %d checks  (logic only: the sketch has never been flashed)\n",
                g_checks, g_checks);
    return 0;
  }
  std::printf("  FAIL: %d of %d checks failed\n", g_failures, g_checks);
  return 1;
}
