#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BUTTON_EVENT_NONE = 0,
    BUTTON_EVENT_SHORT,
    BUTTON_EVENT_LONG,
} button_event_t;

typedef struct {
    gpio_num_t gpio;
    bool active_low;
    uint32_t long_press_ms;

    // internal
    bool last_pressed;
    int64_t press_start_us;
} button_t;

esp_err_t button_init(button_t *btn, gpio_num_t gpio, bool active_low, uint32_t long_press_ms);

// For deep-sleep wake: if currently pressed, measure until release (or max_ms). Returns press duration.
uint32_t button_measure_press_ms(button_t *btn, uint32_t max_ms);

// Polling-based event detection (call periodically, e.g. every 10-50ms).
button_event_t button_poll(button_t *btn);

#ifdef __cplusplus
}
#endif
