/**
 * HOST STUB of the Arduino / ESP32 API, used to COMPILE AND RUN the real .ino on a laptop.
 * ===========================================================================
 * This file exists so that `firmware/phase3_full_6dof_esp32/phase3_full_6dof_esp32.ino` is
 * checked by an actual C++ compiler instead of by eye. With `-I tests/firmware_host` the
 * sketch's `#include <Arduino.h>` resolves here, and the third-party-free `main.cpp` next to
 * this file drives setup() and loop() with a simulated clock, simulated pins, a simulated
 * PCA9685 and a simulated serial port.
 *
 * WHAT IT PROVES: the sketch compiles, its control flow is right, no banned blocking call is
 * in the hot path, the reflex write really does follow a cliff edge inside one loop
 * iteration, and the watchdog is reset every iteration.
 *
 * WHAT IT DOES NOT PROVE: that the ESP32's I2C peripheral, GPIO interrupts, task watchdog,
 * timers or servos behave as modelled here. Every timing in this harness is simulated.
 */

#ifndef BUDE_TEST_ARDUINO_STUB_H
#define BUDE_TEST_ARDUINO_STUB_H

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdarg>
#include <deque>
#include <map>
#include <string>
#include <vector>

// ---- fixed-width names the ESP32 core provides --------------------------------
// (stdint.h already gives us uint8_t etc., so nothing to do here.)

// ---- pin modes and levels -----------------------------------------------------
#define HIGH 0x1
#define LOW 0x0
#define INPUT 0x01
#define OUTPUT 0x03
#define INPUT_PULLUP 0x05
#define OUTPUT_OPEN_DRAIN 0x0F
#define RISING 0x01
#define FALLING 0x02
#define CHANGE 0x03
#define IRAM_ATTR

#ifndef PI
#define PI 3.1415926535897932384626433832795f
#endif

// ---- the simulated clock ------------------------------------------------------
extern uint64_t g_fake_us;
/** Advance the simulated clock. The harness owns time; the sketch only reads it. */
inline void budeAdvanceUs(uint32_t us) { g_fake_us += us; }

inline uint32_t millis() { return static_cast<uint32_t>(g_fake_us / 1000ull); }
inline uint32_t micros() { return static_cast<uint32_t>(g_fake_us); }
inline void delayMicroseconds(uint32_t us) { budeAdvanceUs(us); }
inline void delay(uint32_t ms) { budeAdvanceUs(ms * 1000u); }

// ---- digital IO ---------------------------------------------------------------
extern std::map<int, int> g_pin_levels;
extern std::map<int, int> g_pin_modes;

inline void pinMode(int pin, int mode) { g_pin_modes[pin] = mode; }
inline void digitalWrite(int pin, int level) { g_pin_levels[pin] = (level == HIGH) ? 1 : 0; }
inline int digitalRead(int pin) {
  const std::map<int, int>::const_iterator it = g_pin_levels.find(pin);
  return (it != g_pin_levels.end() && it->second) ? HIGH : LOW;
}
inline int analogRead(int) { return 2048; }
inline void analogReadResolution(int) {}
inline int digitalPinToInterrupt(int pin) { return pin; }

// ---- interrupts ---------------------------------------------------------------
typedef void (*BudeIsr)(void);
extern std::map<int, BudeIsr> g_isr_handlers;
extern std::map<int, int> g_isr_modes;
inline void attachInterrupt(int pin, BudeIsr handler, int mode) {
  g_isr_handlers[pin] = handler;
  g_isr_modes[pin] = mode;
}
/** The harness's way of saying "the GPIO edge just happened". */
extern void budeFireInterrupt(int pin);

// ---- serial -------------------------------------------------------------------
class BudeSerial {
 public:
  void begin(unsigned long) {}
  int available() { return static_cast<int>(in_.size()); }
  int read() {
    if (in_.empty()) {
      return -1;
    }
    const int c = in_.front();
    in_.pop_front();
    return c;
  }
  void println(const char* text) {
    out_ += text;
    out_ += "\n";
    if (echo_) {
      std::printf("%s\n", text);
    }
  }
  void printf(const char* fmt, ...) {
    char buffer[1024];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    out_ += buffer;
    if (echo_) {
      std::fputs(buffer, stdout);
    }
  }
  void write(uint8_t) {}
  void flush() {}

  // harness-side API
  void feed(const std::string& text) {
    for (size_t i = 0; i < text.size(); ++i) {
      in_.push_back(static_cast<unsigned char>(text[i]));
    }
  }
  void setEcho(bool on) { echo_ = on; }
  const std::string& output() const { return out_; }
  void clearOutput() { out_.clear(); }
  bool outputContains(const char* needle) const { return out_.find(needle) != std::string::npos; }

 private:
  std::deque<int> in_;
  std::string out_;
  bool echo_ = false;
};

extern BudeSerial Serial;

// ---- a couple of Arduino utility templates the sketch may use ------------------
template <typename T>
inline T budeMin(T a, T b) { return a < b ? a : b; }

#endif  // BUDE_TEST_ARDUINO_STUB_H
