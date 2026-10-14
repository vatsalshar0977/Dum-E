/**
 * HOST STUB of the ESP32 `Wire` (I2C) library.
 *
 * It decodes real PCA9685 register writes into per-channel pulse widths and answers INA219
 * reads with configurable values, so the harness can assert exactly which pulse widths the
 * firmware commanded and when. `endTransmission()` returns an injectable error code, which is
 * how the I2C recovery path (FMEA #12) is exercised.
 *
 * Timing: every transaction advances the simulated clock by roughly what a 400 kHz bus would
 * cost, so the sketch's own latency instrumentation produces a plausible number. This is a
 * MODEL of the bus, not a measurement of one.
 */

#ifndef BUDE_TEST_WIRE_STUB_H
#define BUDE_TEST_WIRE_STUB_H

#include <cstdint>
#include <map>
#include <vector>

#include "Arduino.h"

extern std::map<uint8_t, uint16_t> g_pca_channels;   // channel -> commanded pulse (us)
extern uint32_t g_pca_writes;                        // transactions committed to 0x40
extern uint32_t g_i2c_errors;                        // injected failures consumed
extern int g_i2c_next_error;                         // 0 = ok, non-zero = NACK / timeout
extern float g_fake_bus_v;
extern float g_fake_current_ma;
extern std::vector<uint32_t> g_write_times_us;       // clock at each committed transaction
extern uint8_t g_last_written_channel;               // harness bookkeeping

class TwoWire {
 public:
  bool begin(int, int) { return true; }
  void setClock(uint32_t) {}
  void setTimeOut(uint32_t) {}
  void end() {}

  void beginTransmission(uint8_t address) {
    address_ = address;
    payload_.clear();
    first_byte_ = true;
  }
  size_t write(uint8_t value) {
    if (first_byte_) {
      reg_ = value;
      first_byte_ = false;
    } else {
      payload_.push_back(value);
    }
    return 1;
  }
  size_t write(const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
      write(data[i]);
    }
    return len;
  }

  /**
   * Commit. A real 400 kHz bus spends about 23 us per byte (9 bit times) plus start/stop, so
   * the simulated cost is modelled the same way: this is what makes the sketch's measured
   * reflex latency a believable number rather than a constant zero.
   */
  uint8_t endTransmission() {
    const uint32_t bytes = static_cast<uint32_t>(payload_.size()) + 1u;
    budeAdvanceUs(30u + 23u * bytes);
    if (g_i2c_next_error != 0) {
      const int error = g_i2c_next_error;
      g_i2c_next_error = 0;
      ++g_i2c_errors;
      last_read_valid_ = false;
      return static_cast<uint8_t>(error);
    }
    if (address_ == 0x40) {
      commitPca();
    }
    last_reg_ = reg_;
    last_addr_ = address_;
    last_read_valid_ = true;
    return 0;
  }

  uint8_t requestFrom(uint8_t address, uint8_t count) {
    budeAdvanceUs(30u + 23u * count);
    if (!last_read_valid_ || address != last_addr_) {
      return 0;
    }
    read_buffer_.clear();
    if (address == 0x41) {
      // INA219: 4 mV per bus-voltage bit (value << 3) and 0.1 mA per current bit.
      const uint16_t bus_raw = static_cast<uint16_t>((g_fake_bus_v / 0.004f) * 8.0f);
      const int16_t current_raw = static_cast<int16_t>(g_fake_current_ma / 0.1f);
      if (last_reg_ == 0x02) {
        read_buffer_.push_back(static_cast<uint8_t>(bus_raw >> 8));
        read_buffer_.push_back(static_cast<uint8_t>(bus_raw & 0xFF));
      } else if (last_reg_ == 0x04) {
        read_buffer_.push_back(static_cast<uint8_t>(current_raw >> 8));
        read_buffer_.push_back(static_cast<uint8_t>(current_raw & 0xFF));
      } else {
        read_buffer_.push_back(0x39);
        read_buffer_.push_back(0x9F);
      }
    } else if (address == 0x40) {
      // PCA9685 MODE1 read-back: everything clear except AUTO_INCREMENT.
      read_buffer_.push_back((last_reg_ == 0x00) ? 0x20 : 0x00);
    }
    return static_cast<uint8_t>(read_buffer_.size() < count ? read_buffer_.size() : count);
  }

  int read() {
    if (read_buffer_.empty()) {
      return 0;
    }
    const int value = read_buffer_.front();
    read_buffer_.erase(read_buffer_.begin());
    return value;
  }

 private:
  void commitPca() {
    // LEDn_ON_L = 0x06 + 4n. Four bytes per channel: ON low, ON high, OFF low, OFF high.
    if (reg_ < 0x06 || reg_ > 0x45) {
      return;                                    // MODE1 / MODE2 / PRESCALE: not a pulse
    }
    const uint8_t first = static_cast<uint8_t>((reg_ - 0x06) / 4);
    for (size_t i = 0; i + 3 < payload_.size(); i += 4) {
      const uint16_t ticks = static_cast<uint16_t>(payload_[i + 2]) |
                             (static_cast<uint16_t>(payload_[i + 3]) << 8);
      // 20 ms period, 4096 ticks: pulse_us = ticks * 20000 / 4096
      const uint16_t pulse_us = static_cast<uint16_t>((static_cast<uint32_t>(ticks) * 20000u +
                                                       2048u) / 4096u);
      const uint8_t channel = static_cast<uint8_t>(first + (i / 4));
      g_pca_channels[channel] = pulse_us;
      g_last_written_channel = channel;
    }
    ++g_pca_writes;
    g_write_times_us.push_back(static_cast<uint32_t>(g_fake_us));
  }

  uint8_t address_ = 0;
  uint8_t reg_ = 0;
  uint8_t last_reg_ = 0;
  uint8_t last_addr_ = 0;
  bool first_byte_ = true;
  bool last_read_valid_ = false;
  std::vector<uint8_t> payload_;
  std::vector<uint8_t> read_buffer_;
};

extern TwoWire Wire;

#endif  // BUDE_TEST_WIRE_STUB_H
