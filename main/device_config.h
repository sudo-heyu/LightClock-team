#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t alarm_hour;   // 0-23
    uint8_t alarm_minute; // 0-59
    uint8_t color_temp;   // 0-100 (0=cool, 100=warm)
    uint8_t wake_bright;  // 0-100 (max brightness target for alarm gradient)
    uint8_t sunrise_duration; // 5-60 minutes (sunrise simulation duration)
} device_config_t;

#define DEVICE_CONFIG_DEFAULT_HOUR   (7)
#define DEVICE_CONFIG_DEFAULT_MINUTE (0)
#define DEVICE_CONFIG_DEFAULT_COLOR_TEMP  (50)
#define DEVICE_CONFIG_DEFAULT_WAKE_BRIGHT (100)
#define DEVICE_CONFIG_DEFAULT_SUNRISE_DURATION_MINUTES (30)

esp_err_t device_config_load(device_config_t *out_cfg);
esp_err_t device_config_save(const device_config_t *cfg);

bool device_config_parse_hhmm_ascii(const uint8_t *data, size_t len, device_config_t *out_cfg);
void device_config_format_hhmm_ascii(const device_config_t *cfg, uint8_t out4[4]);

#ifdef __cplusplus
}
#endif
