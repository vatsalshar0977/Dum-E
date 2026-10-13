/**
 * HOST STUB: pretends to be arduino-esp32 v3.x so the harness compiles the SAME branch the
 * user's current core would take (`esp_task_wdt_config_t`). The 2.x branch is a two-line
 * difference; both are in the sketch under one #if, and only this one is compiled here.
 */
#ifndef BUDE_TEST_ESP_ARDUINO_VERSION_STUB_H
#define BUDE_TEST_ESP_ARDUINO_VERSION_STUB_H

#define ESP_ARDUINO_VERSION_MAJOR 3
#define ESP_ARDUINO_VERSION_MINOR 0
#define ESP_ARDUINO_VERSION_PATCH 0

#endif
