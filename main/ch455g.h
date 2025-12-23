#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    gpio_num_t sda;
    gpio_num_t scl;
    uint8_t sys_param; // cached 0x48 byte2
} ch455g_t;

// intensity: 0..7 where 0 means 8/8 (max), 7 means 7/8 per datasheet mapping.
esp_err_t ch455g_init(ch455g_t *dev, gpio_num_t sda, gpio_num_t scl, uint8_t intensity);

esp_err_t ch455g_set_enabled(ch455g_t *dev, bool enabled);
esp_err_t ch455g_set_sleep(ch455g_t *dev, bool sleep);

esp_err_t ch455g_set_digit_raw(ch455g_t *dev, uint8_t dig_index_0_3, uint8_t seg_byte);
esp_err_t ch455g_set_4digits_raw(ch455g_t *dev, uint8_t dig0, uint8_t dig1, uint8_t dig2, uint8_t dig3);

esp_err_t ch455g_show_hhmm(ch455g_t *dev, int hour, int minute);
esp_err_t ch455g_clear(ch455g_t *dev);

#ifdef __cplusplus
}
#endif
