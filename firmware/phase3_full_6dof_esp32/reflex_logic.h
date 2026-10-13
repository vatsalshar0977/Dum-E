/**
 * B.U.D.-E — hardware-independent safety logic for phase3_full_6dof_esp32.ino
 * ===========================================================================
 * WHY THIS FILE EXISTS
 * --------------------
 * The three things in that sketch which can hurt someone are pure logic, not Arduino calls:
 *
 *   1. the cliff reflex  (drive forward -> edge appears -> reverse for a fixed time -> stop)
 *   2. the host-liveness timeout  (no wheel command for 500 ms -> wheels to neutral)
 *   3. the I2C failure policy  (NACK -> bus recovery -> retry -> cut the servo enable)
 *
 * None of them need a microcontroller to be *checked*, so they live here as plain C++ with no
 * Arduino, ESP32 or Wire dependency. `firmware/phase3_full_6dof_esp32/test_reflex_logic.cpp`
 * compiles this header with g++ on a laptop and drives the real state machines through the
 * real sequences, including the 5 ms budget accounting.
 *
 * What that buys you: the timing rules below are executed and asserted before the code ever
 * reaches the board. What it does NOT buy you: the sketch itself has never been compiled by
 * arduino-cli or flashed, the I2C and ISR glue is unverified, and no amount of host testing
 * proves a physical wheel stops in 5 ms.
 *
 * Everything here is C++11 with no allocations, no exceptions and no floating point, so it is
 * safe to call from a 100 kHz loop and (for the ISR entry points) from an interrupt.
 */

#ifndef BUDE_REFLEX_LOGIC_H
#define BUDE_REFLEX_LOGIC_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

namespace bude {

// ===========================================================================
// 1. I2C supervision  (FMEA #12)
// ===========================================================================
class I2cSupervisor {
 public:
  enum class Action { kNone, kRecover, kDisableOutputs };

  explicit I2cSupervisor(uint8_t max_consecutive_failures = 3)
      : max_failures_(max_consecutive_failures), consecutive_(0), recoveries_(0) {}

  /** Any successful transaction proves the bus is alive again. */
  void noteOk() { consecutive_ = 0; }

  /**
   * A NACK, a bus timeout or a short write. One failure = recover the bus and retry; after
   * `max_consecutive_failures` in a row the bus is presumed dead and the caller must cut the
   * servo enable, because a PCA9685 that cannot be written holds its LAST pulse width - which
   * for a wheel means "keep driving", the worst possible failure mode.
   */
  Action noteFailure() {
    if (consecutive_ < 0xFF) {
      ++consecutive_;
    }
    if (consecutive_ >= max_failures_) {
      return Action::kDisableOutputs;
    }
    ++recoveries_;
    return Action::kRecover;
  }

  uint8_t consecutiveFailures() const { return consecutive_; }
  uint32_t recoveryCount() const { return recoveries_; }

 private:
  uint8_t max_failures_;
  uint8_t consecutive_;
  uint32_t recoveries_;
};

// ===========================================================================
// 2. Cliff reflex and wheel liveness  (FMEA #11 and the <5 ms deadline)
// ===========================================================================
/**
 * MIRRORED WHEELS — the one calibration fact this file cannot guess.
 *
 * The brief's recoil pair is 1680/1320 us, which means the two drive servos are mounted
 * facing each other: on the left channel a pulse ABOVE neutral drives that wheel BACKWARD,
 * and on the right channel a pulse ABOVE neutral drives it FORWARD. Getting this wrong makes
 * the "reflex" spin the rover in place on a table edge instead of backing it away, so it is
 * an explicit flag rather than a hard-coded assumption.
 *
 * Bench check: send `W 1320 1680` with the rover on the floor. Both wheels must drive the
 * rover AWAY from you. If it pirouettes instead, flip the flags.
 */
struct WheelSafetyConfig {
  uint16_t neutral_us = 1500;          // both servos are continuous rotation
  uint16_t reverse_left_us = 1680;     // the brief's recoil pair
  uint16_t reverse_right_us = 1320;
  bool left_forward_is_above_neutral = false;    // left: 1320 = forward, 1680 = reverse
  bool right_forward_is_above_neutral = true;    // right: 1680 = forward, 1320 = reverse
  uint16_t deadband_us = 40;           // |pulse - neutral| beyond this counts as "moving"
  uint32_t recoil_ms = 350;            // how long the recoil pulse is held
  uint32_t command_timeout_ms = 500;   // host silence -> neutral (task watchdog territory)
  uint32_t reflex_budget_us = 5000;    // the declared deadline: edge -> reverse written
};

class WheelSafety {
 public:
  struct Output {
    uint16_t left_us = 1500;
    uint16_t right_us = 1500;
    bool changed = false;              // only write the PCA9685 when this is true
    bool recoil_active = false;
    bool forward_blocked = false;      // a forward command was refused because of a cliff
    bool command_timed_out = false;    // the 500 ms host-liveness timeout fired
    bool over_budget = false;          // measured latency exceeded reflex_budget_us
    uint32_t latency_us = 0;           // edge -> reverse written, for the LAST reflex
    bool latency_valid = false;
  };

  WheelSafety() {}
  explicit WheelSafety(const WheelSafetyConfig& cfg) : cfg_(cfg) {}

  void begin(uint32_t now_ms) {
    output_ = Output();
    output_.left_us = cfg_.neutral_us;
    output_.right_us = cfg_.neutral_us;
    reported_left_ = cfg_.neutral_us;
    reported_right_ = cfg_.neutral_us;
    host_left_ = cfg_.neutral_us;
    host_right_ = cfg_.neutral_us;
    last_host_cmd_ms_ = now_ms;
    recoil_until_ms_ = 0;
    edge_pending_ = false;
    latency_pending_ = false;
    begun_ = true;
  }

  /** Called from the `W` serial command. This is the ONLY thing that refreshes liveness. */
  void noteHostWheelCommand(uint32_t now_ms, uint16_t left_us, uint16_t right_us) {
    host_left_ = clampPulse(left_us);
    host_right_ = clampPulse(right_us);
    last_host_cmd_ms_ = now_ms;
    output_.command_timed_out = false;
  }

  /** Called from the GPIO 25/26 ISR. No locks, no I/O: it records the edge and nothing else. */
  void noteCliffEdge(uint32_t now_us) {
    edge_us_ = now_us;
    edge_pending_ = true;
  }

  /**
   * The whole decision, once per loop iteration. `now_us` is a free-running microsecond clock
   * used only for the latency measurement; `now_ms` drives the 350 ms and 500 ms timers.
   */
  Output update(uint32_t now_ms, uint32_t now_us, bool cliff_left, bool cliff_right) {
    if (!begun_) {
      begin(now_ms);
    }
    const bool cliff = cliff_left || cliff_right;
    output_.forward_blocked = false;
    output_.over_budget = false;
    output_.command_timed_out = false;

    // --- 1. A recoil already running owns the wheels until its timer expires. -------------
    if (recoil_active_) {
      if (static_cast<int32_t>(now_ms - recoil_until_ms_) >= 0) {
        recoil_active_ = false;
        setOutput(cfg_.neutral_us, cfg_.neutral_us);
      } else {
        setOutput(cfg_.reverse_left_us, cfg_.reverse_right_us);
        output_.recoil_active = true;
        return publish();
      }
    }

    // --- 2. A fresh edge while driving forward starts a recoil in THIS iteration. --------
    const bool driving_forward =
        isForward(output_.left_us, cfg_.left_forward_is_above_neutral) ||
        isForward(output_.right_us, cfg_.right_forward_is_above_neutral);
    if (cliff && driving_forward) {
      recoil_active_ = true;
      recoil_until_ms_ = now_ms + cfg_.recoil_ms;
      ++recoil_count_;
      setOutput(cfg_.reverse_left_us, cfg_.reverse_right_us);
      output_.recoil_active = true;
      if (edge_pending_) {
        latency_start_us_ = edge_us_;
        edge_pending_ = false;
        latency_pending_ = true;
      } else {
        latency_start_us_ = now_us;      // no ISR edge: take the loop's own observation
        latency_pending_ = true;
      }
      return publish();
    }

    // --- 3. Host liveness. Silence while the wheels are turning means neutral. -----------
    if (static_cast<int32_t>(now_ms - last_host_cmd_ms_) > static_cast<int32_t>(
            cfg_.command_timeout_ms)) {
      if (output_.left_us != cfg_.neutral_us || output_.right_us != cfg_.neutral_us) {
        output_.command_timed_out = true;
      }
      setOutput(cfg_.neutral_us, cfg_.neutral_us);
      return publish();
    }

    // --- 4. Otherwise obey the host, refusing to drive forward over a cliff. -------------
    uint16_t want_left = host_left_;
    uint16_t want_right = host_right_;
    if (cliff) {
      if (isForward(want_left, cfg_.left_forward_is_above_neutral)) {
        want_left = cfg_.neutral_us;
        output_.forward_blocked = true;
      }
      if (isForward(want_right, cfg_.right_forward_is_above_neutral)) {
        want_right = cfg_.neutral_us;
        output_.forward_blocked = true;
      }
    }
    setOutput(want_left, want_right);
    return publish();
  }

  struct WriteReport {
    uint32_t latency_us = 0;          // ISR edge -> this write completing
    bool over_budget = false;         // exceeded reflex_budget_us
    bool measured = false;            // false if there was no reflex write to time
  };

  /**
   * Call immediately after the two channel writes that followed a recoil. `edge_us` came from
   * the ISR, so this is the real edge-to-command latency - the number the 5 ms claim rests on.
   * It RETURNS the report rather than mutating a struct the caller already holds a copy of.
   */
  WriteReport noteReflexWritten(uint32_t now_us) {
    WriteReport report;
    if (!latency_pending_) {
      return report;
    }
    latency_pending_ = false;
    report.measured = true;
    report.latency_us = static_cast<uint32_t>(now_us - latency_start_us_);
    output_.latency_us = report.latency_us;
    output_.latency_valid = true;
    if (report.latency_us > max_latency_us_) {
      max_latency_us_ = report.latency_us;
    }
    if (report.latency_us > cfg_.reflex_budget_us) {
      report.over_budget = true;
      output_.over_budget = true;
      ++over_budget_count_;
    }
    return report;
  }

  // --- introspection for the STATUS command and the tests --------------------------------
  const WheelSafetyConfig& config() const { return cfg_; }
  uint16_t leftUs() const { return output_.left_us; }
  uint16_t rightUs() const { return output_.right_us; }
  bool recoilActive() const { return recoil_active_; }
  uint32_t recoilCount() const { return recoil_count_; }
  uint32_t overBudgetCount() const { return over_budget_count_; }
  uint32_t maxLatencyUs() const { return max_latency_us_; }
  uint32_t lastLatencyUs() const { return output_.latency_us; }

  /**
   * Is this pulse width asking the wheel to drive FORWARD? Which way that is depends on how
   * the servo is mounted, hence the flag: the two drive servos face each other.
   */
  bool isForward(uint16_t us, bool forward_above_neutral) const {
    if (forward_above_neutral) {
      return us > cfg_.neutral_us + cfg_.deadband_us;
    }
    return us < cfg_.neutral_us - cfg_.deadband_us;
  }

 private:
  uint16_t clampPulse(uint16_t us) const {
    if (us < 1000) {
      return 1000;
    }
    if (us > 2000) {
      return 2000;
    }
    return us;
  }

  void setOutput(uint16_t left, uint16_t right) {
    output_.left_us = clampPulse(left);
    output_.right_us = clampPulse(right);
  }

  Output publish() {
    // `output_` is a member so the caller can read it after `noteReflexWritten`, which means
    // every flag it carries has to be resynced here or it goes stale the moment the recoil
    // state changes. (That was a real bug: `recoil_active` stayed true after the recoil had
    // finished, so the caller skipped the neutral write that ends the reverse pulse.)
    output_.recoil_active = recoil_active_;
    output_.changed = (output_.left_us != reported_left_) || (output_.right_us != reported_right_);
    reported_left_ = output_.left_us;
    reported_right_ = output_.right_us;
    return output_;
  }

  WheelSafetyConfig cfg_;
  Output output_;
  uint16_t reported_left_ = 1500;
  uint16_t reported_right_ = 1500;
  uint16_t host_left_ = 1500;
  uint16_t host_right_ = 1500;
  uint32_t last_host_cmd_ms_ = 0;
  uint32_t recoil_until_ms_ = 0;
  volatile uint32_t edge_us_ = 0;
  volatile bool edge_pending_ = false;
  uint32_t latency_start_us_ = 0;
  bool latency_pending_ = false;
  uint32_t max_latency_us_ = 0;
  uint32_t recoil_count_ = 0;
  uint32_t over_budget_count_ = 0;
  bool recoil_active_ = false;
  bool begun_ = false;
};

// ===========================================================================
// 3. Front obstacle guard  (HC-SR04, non-blocking)
// ===========================================================================
class UltrasonicGuard {
 public:
  UltrasonicGuard(uint16_t stop_mm = 80, uint32_t stale_ms = 500)
      : stop_mm_(stop_mm), stale_ms_(stale_ms) {}

  void noteReading(uint16_t mm, uint32_t now_ms) {
    distance_mm_ = mm;
    last_reading_ms_ = now_ms;
    have_reading_ = true;
  }

  bool everHadReading() const { return have_reading_; }
  bool readingFresh(uint32_t now_ms) const {
    return have_reading_ &&
           static_cast<uint32_t>(now_ms - last_reading_ms_) <= stale_ms_;
  }
  uint16_t distanceMm() const { return distance_mm_; }

  /**
   * Fail-closed once the sensor has proved it works and then goes quiet: a sensor that stops
   * answering is a fault, not a clear road. Before the first reading ever arrives the answer
   * is `allow_without_sensor`, which defaults to true so the stationary arm bench-tests
   * without an HC-SR04 fitted - the rover build should set it to false.
   */
  bool allowForward(uint32_t now_ms) const {
    if (!have_reading_) {
      return allow_without_sensor;
    }
    if (!readingFresh(now_ms)) {
      return false;
    }
    return distance_mm_ > stop_mm_;
  }

  void requireSensor(bool b) { allow_without_sensor = !b; }
  bool sensorRequired() const { return !allow_without_sensor; }
  uint16_t stopMm() const { return stop_mm_; }

 private:
  uint16_t stop_mm_;
  uint32_t stale_ms_;
  uint16_t distance_mm_ = 0;
  uint32_t last_reading_ms_ = 0;
  bool have_reading_ = false;
  bool allow_without_sensor = true;
};

// ===========================================================================
// 4. Non-blocking serial line assembly
// ===========================================================================
/**
 * The blocking killer in a 5 ms budget is `Serial.readStringUntil('\n')`: it stalls the loop
 * for the whole serial timeout (1000 ms by default) whenever the host sends no newline. This
 * class never blocks. Over-long lines are discarded up to the next newline, so a corrupt host
 * cannot wedge the firmware.
 */
class LineAccumulator {
 public:
  static const size_t kCapacity = 96;

  /** Returns true when a complete line is ready in `line()`. */
  bool feed(char c) {
    if (c == '\n' || c == '\r') {
      if (len_ == 0 || discarding_) {
        len_ = 0;
        discarding_ = false;
        return false;
      }
      buf_[len_] = '\0';
      ready_ = true;
      return true;
    }
    if (discarding_) {
      return false;
    }
    if (len_ >= kCapacity - 1) {
      discarding_ = true;                 // too long: throw it away, do NOT block
      len_ = 0;
      return false;
    }
    buf_[len_++] = c;
    return false;
  }

  const char* line() const { return buf_; }
  void clear() { len_ = 0; ready_ = false; discarding_ = false; buf_[0] = '\0'; }
  bool ready() const { return ready_; }
  bool overflowed() const { return discarding_; }

 private:
  char buf_[kCapacity] = {0};
  size_t len_ = 0;
  bool ready_ = false;
  bool discarding_ = false;
};

// ===========================================================================
// 5. Command parsing
// ===========================================================================
struct WheelCommand {
  bool valid = false;
  uint16_t left_us = 1500;
  uint16_t right_us = 1500;
};

inline bool isSpace(char c) { return c == ' ' || c == '\t'; }

/**
 * `W <left_us> <right_us>` and the bench shorthand `W <us>` (both wheels).
 * Exactly the brief's syntax; anything else is refused rather than guessed at, because a
 * mis-parsed wheel command drives a rover.
 */
inline WheelCommand parseWheelCommand(const char* line) {
  WheelCommand out;
  if (line == NULL) {
    return out;
  }
  const char* p = line;
  while (isSpace(*p)) {
    ++p;
  }
  if (*p != 'W' && *p != 'w') {
    return out;
  }
  ++p;
  long values[2] = {0, 0};
  int count = 0;
  while (count < 2) {
    while (isSpace(*p)) {
      ++p;
    }
    if (*p < '0' || *p > '9') {
      break;
    }
    long value = 0;
    while (*p >= '0' && *p <= '9') {
      value = value * 10 + (*p - '0');
      if (value > 99999) {
        return out;                       // absurd input: refuse
      }
      ++p;
    }
    values[count++] = value;
  }
  while (isSpace(*p)) {
    ++p;
  }
  if (*p != '\0' || count == 0) {
    return out;                           // trailing junk: refuse
  }
  if (values[0] < 1000 || values[0] > 2000) {
    return out;
  }
  if (count == 1) {
    values[1] = values[0];
  }
  if (values[1] < 1000 || values[1] > 2000) {
    return out;
  }
  out.left_us = static_cast<uint16_t>(values[0]);
  out.right_us = static_cast<uint16_t>(values[1]);
  out.valid = true;
  return out;
}

}  // namespace bude

#endif  // BUDE_REFLEX_LOGIC_H
