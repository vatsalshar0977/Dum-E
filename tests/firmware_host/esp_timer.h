/** HOST STUB: microsecond clock, backed by the harness's simulated time. */
#ifndef BUDE_TEST_ESP_TIMER_STUB_H
#define BUDE_TEST_ESP_TIMER_STUB_H

#include <cstdint>
#include "Arduino.h"

inline int64_t esp_timer_get_time() { return static_cast<int64_t>(g_fake_us); }

#endif
