/**
 * HOST STUB: the ESP32 task watchdog (FMEA #11).
 *
 * It records init / add / reset / reconfigure calls so the harness can assert that the
 * firmware configures a 5 s watchdog and feeds it on every loop iteration. It cannot fire:
 * on real silicon a missed reset panics and reboots the chip.
 */
#ifndef BUDE_TEST_ESP_TASK_WDT_STUB_H
#define BUDE_TEST_ESP_TASK_WDT_STUB_H

#include <cstdint>
#include "esp_err.h"

struct esp_task_wdt_config_t {
  uint32_t timeout_ms;
  uint32_t idle_core_mask;
  bool trigger_panic;
};

extern int g_wdt_init_count;
extern int g_wdt_reconfigure_count;
extern int g_wdt_add_count;
extern int g_wdt_reset_count;
extern uint32_t g_wdt_timeout_ms;
extern bool g_wdt_panic;
extern bool g_wdt_already_initialised;

inline esp_err_t esp_task_wdt_init(uint32_t timeout_s, bool panic) {
  ++g_wdt_init_count;
  g_wdt_timeout_ms = timeout_s * 1000u;
  g_wdt_panic = panic;
  return ESP_OK;
}

inline esp_err_t esp_task_wdt_init(const esp_task_wdt_config_t* cfg) {
  if (g_wdt_already_initialised) {
    return ESP_ERR_INVALID_STATE;
  }
  ++g_wdt_init_count;
  g_wdt_timeout_ms = cfg->timeout_ms;
  g_wdt_panic = cfg->trigger_panic;
  return ESP_OK;
}

inline esp_err_t esp_task_wdt_reconfigure(const esp_task_wdt_config_t* cfg) {
  ++g_wdt_reconfigure_count;
  g_wdt_timeout_ms = cfg->timeout_ms;
  g_wdt_panic = cfg->trigger_panic;
  return ESP_OK;
}

inline esp_err_t esp_task_wdt_add(void*) {
  ++g_wdt_add_count;
  return ESP_OK;
}

inline esp_err_t esp_task_wdt_reset() {
  ++g_wdt_reset_count;
  return ESP_OK;
}

#endif
