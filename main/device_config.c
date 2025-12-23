#include "device_config.h"

#include <string.h>

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "CFG";

static const char *NVS_NS = "cfg";
static const char *KEY_ALARM_H = "alarm_h";
static const char *KEY_ALARM_M = "alarm_m";

static bool cfg_valid(const device_config_t *cfg)
{
    if (!cfg) {
        return false;
    }
    return (cfg->alarm_hour < 24) && (cfg->alarm_minute < 60);
}

static device_config_t cfg_default(void)
{
    device_config_t cfg = {
        .alarm_hour = DEVICE_CONFIG_DEFAULT_HOUR,
        .alarm_minute = DEVICE_CONFIG_DEFAULT_MINUTE,
    };
    return cfg;
}

esp_err_t device_config_save(const device_config_t *cfg)
{
    if (!cfg_valid(cfg)) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_u8(handle, KEY_ALARM_H, cfg->alarm_hour);
    if (err == ESP_OK) {
        err = nvs_set_u8(handle, KEY_ALARM_M, cfg->alarm_minute);
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

esp_err_t device_config_load(device_config_t *out_cfg)
{
    if (!out_cfg) {
        return ESP_ERR_INVALID_ARG;
    }

    device_config_t cfg = cfg_default();

    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NS, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        *out_cfg = cfg;
        ESP_LOGW(TAG, "NVS namespace missing; saving defaults %02u%02u", cfg.alarm_hour, cfg.alarm_minute);
        return device_config_save(&cfg);
    }
    if (err != ESP_OK) {
        return err;
    }

    uint8_t h = 0, m = 0;
    esp_err_t eh = nvs_get_u8(handle, KEY_ALARM_H, &h);
    esp_err_t em = nvs_get_u8(handle, KEY_ALARM_M, &m);
    nvs_close(handle);

    if (eh == ESP_OK && em == ESP_OK) {
        cfg.alarm_hour = h;
        cfg.alarm_minute = m;
        if (cfg_valid(&cfg)) {
            *out_cfg = cfg;
            return ESP_OK;
        }
        ESP_LOGW(TAG, "Invalid cfg in NVS (%u:%u); reset to defaults", h, m);
    } else {
        ESP_LOGW(TAG, "Cfg missing in NVS; reset to defaults");
    }

    cfg = cfg_default();
    *out_cfg = cfg;
    return device_config_save(&cfg);
}

bool device_config_parse_hhmm_ascii(const uint8_t *data, size_t len, device_config_t *out_cfg)
{
    if (!data || !out_cfg || len != 4) {
        return false;
    }

    for (size_t i = 0; i < 4; i++) {
        if (data[i] < '0' || data[i] > '9') {
            return false;
        }
    }

    uint8_t hh = (uint8_t)((data[0] - '0') * 10 + (data[1] - '0'));
    uint8_t mm = (uint8_t)((data[2] - '0') * 10 + (data[3] - '0'));

    device_config_t tmp = {.alarm_hour = hh, .alarm_minute = mm};
    if (!cfg_valid(&tmp)) {
        return false;
    }

    *out_cfg = tmp;
    return true;
}

void device_config_format_hhmm_ascii(const device_config_t *cfg, uint8_t out4[4])
{
    if (!cfg || !out4) {
        return;
    }
    out4[0] = (uint8_t)('0' + (cfg->alarm_hour / 10));
    out4[1] = (uint8_t)('0' + (cfg->alarm_hour % 10));
    out4[2] = (uint8_t)('0' + (cfg->alarm_minute / 10));
    out4[3] = (uint8_t)('0' + (cfg->alarm_minute % 10));
}
