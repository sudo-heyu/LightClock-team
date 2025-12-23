#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*ble_alarm_on_connect_t)(void *ctx);
typedef void (*ble_alarm_on_disconnect_t)(void *ctx);

typedef bool (*ble_alarm_on_write_hhmm_t)(const uint8_t hhmm4[4], void *ctx);
typedef void (*ble_alarm_on_read_hhmm_t)(uint8_t out_hhmm4[4], void *ctx);

esp_err_t ble_alarm_init(ble_alarm_on_write_hhmm_t on_write,
                         ble_alarm_on_read_hhmm_t on_read,
                         ble_alarm_on_connect_t on_connect,
                         ble_alarm_on_disconnect_t on_disconnect,
                         void *ctx);

esp_err_t ble_alarm_start_advertising(void);
esp_err_t ble_alarm_stop_advertising(void);

bool ble_alarm_is_connected(void);
esp_err_t ble_alarm_disconnect(void);

esp_err_t ble_alarm_deinit(void);

#ifdef __cplusplus
}
#endif
