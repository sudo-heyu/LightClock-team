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
} device_config_t;

#define DEVICE_CONFIG_DEFAULT_HOUR   (7)
#define DEVICE_CONFIG_DEFAULT_MINUTE (0)

esp_err_t device_config_load(device_config_t *out_cfg);
esp_err_t device_config_save(const device_config_t *cfg);

bool device_config_parse_hhmm_ascii(const uint8_t *data, size_t len, device_config_t *out_cfg);
void device_config_format_hhmm_ascii(const device_config_t *cfg, uint8_t out4[4]);

#ifdef __cplusplus
}
#endif
