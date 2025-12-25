#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool inited;
    gpio_num_t adc_gpio;
    gpio_num_t en_gpio;
    bool en_active_high;

    // ADC
    void *unit; // adc_oneshot_unit_handle_t (opaque here to keep header light)
    int channel; // adc_channel_t

    // calibration
    void *cali; // adc_cali_handle_t (opaque)
    bool cali_enabled;
} battery_t;

// Initializes ADC and calibration (best-effort). en_gpio will be driven low when idle.
esp_err_t battery_init(battery_t *bat, gpio_num_t adc_gpio, gpio_num_t en_gpio);

// Reads battery voltage (mV) at the battery terminals.
// Returns ESP_OK and sets out_mv.
esp_err_t battery_read_mv(battery_t *bat, uint32_t *out_mv);

// Reads battery percent (0..100). Uses a simple voltage->percent mapping.
esp_err_t battery_read_percent(battery_t *bat, uint8_t *out_percent);

// Converts a measured battery voltage (mV) to percent (0..100) using current mapping.
uint8_t battery_mv_to_percent(uint32_t mv);

// Deinitializes ADC resources.
void battery_deinit(battery_t *bat);

#ifdef __cplusplus
}
#endif
