#include "battery.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

static const char *TAG = "BATT";

// Divider ratio per requirement.md: 15.1k / 5.1k
// Vbat = Vadc * (Rtop + Rbot) / Rbot
#define BATT_DIV_NUMERATOR_MOHM   (15100 + 5100)
#define BATT_DIV_DENOMINATOR_MOHM (5100)

// Simple Li-ion mapping (can be tuned later)
#define BATT_EMPTY_MV (3300)
#define BATT_FULL_MV  (4200)

static uint32_t scale_divider_to_battery_mv(uint32_t vadc_mv)
{
    // Use integer math with rounding.
    uint64_t v = (uint64_t)vadc_mv * (uint64_t)BATT_DIV_NUMERATOR_MOHM;
    v = (v + (BATT_DIV_DENOMINATOR_MOHM / 2)) / (uint64_t)BATT_DIV_DENOMINATOR_MOHM;
    return (uint32_t)v;
}

static uint8_t mv_to_percent(uint32_t mv)
{
    if (mv <= BATT_EMPTY_MV) {
        return 0;
    }
    if (mv >= BATT_FULL_MV) {
        return 100;
    }
    return (uint8_t)(((mv - BATT_EMPTY_MV) * 100U) / (BATT_FULL_MV - BATT_EMPTY_MV));
}

esp_err_t battery_init(battery_t *bat, gpio_num_t adc_gpio, gpio_num_t en_gpio)
{
    if (!bat) {
        return ESP_ERR_INVALID_ARG;
    }

    *bat = (battery_t){0};
    bat->adc_gpio = adc_gpio;
    bat->en_gpio = en_gpio;

    gpio_config_t en = {
        .pin_bit_mask = (1ULL << en_gpio),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&en);
    if (err != ESP_OK) {
        return err;
    }
    gpio_set_level(en_gpio, 0);

    adc_unit_t unit_id;
    adc_channel_t channel;
    err = adc_oneshot_io_to_channel(adc_gpio, &unit_id, &channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_io_to_channel failed for gpio=%d: %s", (int)adc_gpio, esp_err_to_name(err));
        return err;
    }

    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = unit_id,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    adc_oneshot_unit_handle_t unit = NULL;
    err = adc_oneshot_new_unit(&unit_cfg, &unit);
    if (err != ESP_OK) {
        return err;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_oneshot_config_channel(unit, channel, &chan_cfg);
    if (err != ESP_OK) {
        adc_oneshot_del_unit(unit);
        return err;
    }

    bat->unit = unit;
    bat->channel = (int)channel;

    // Calibration is best-effort.
    adc_cali_handle_t cali = NULL;
    bool cali_ok = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    {
        adc_cali_curve_fitting_config_t cali_cfg = {
            .unit_id = unit_id,
            .chan = channel,
            .atten = ADC_ATTEN_DB_11,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali) == ESP_OK) {
            cali_ok = true;
        }
    }
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    {
        adc_cali_line_fitting_config_t cali_cfg = {
            .unit_id = unit_id,
            .atten = ADC_ATTEN_DB_11,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        if (adc_cali_create_scheme_line_fitting(&cali_cfg, &cali) == ESP_OK) {
            cali_ok = true;
        }
    }
#endif

    bat->cali = cali;
    bat->cali_enabled = cali_ok;
    bat->inited = true;

    ESP_LOGI(TAG, "battery init: gpio_adc=%d unit=%d chan=%d cali=%d", (int)adc_gpio, (int)unit_id, (int)channel, (int)cali_ok);
    return ESP_OK;
}

esp_err_t battery_read_mv(battery_t *bat, uint32_t *out_mv)
{
    if (!bat || !bat->inited || !out_mv) {
        return ESP_ERR_INVALID_ARG;
    }

    gpio_set_level(bat->en_gpio, 1);
    vTaskDelay(pdMS_TO_TICKS(5));

    adc_oneshot_unit_handle_t unit = (adc_oneshot_unit_handle_t)bat->unit;
    adc_channel_t channel = (adc_channel_t)bat->channel;

    // Average a few samples to reduce noise.
    int raw_sum = 0;
    const int samples = 8;
    for (int i = 0; i < samples; i++) {
        int raw = 0;
        esp_err_t err = adc_oneshot_read(unit, channel, &raw);
        if (err != ESP_OK) {
            gpio_set_level(bat->en_gpio, 0);
            return err;
        }
        raw_sum += raw;
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    gpio_set_level(bat->en_gpio, 0);

    int raw_avg = raw_sum / samples;

    uint32_t vadc_mv = 0;
    if (bat->cali_enabled && bat->cali) {
        int mv = 0;
        esp_err_t err = adc_cali_raw_to_voltage((adc_cali_handle_t)bat->cali, raw_avg, &mv);
        if (err == ESP_OK) {
            vadc_mv = (uint32_t)mv;
        }
    }

    if (vadc_mv == 0) {
        // Fallback rough conversion when calibration is not available.
        // For 12-bit default, raw range is 0..4095.
        vadc_mv = (uint32_t)((uint64_t)raw_avg * 3300ULL / 4095ULL);
    }

    uint32_t vbat_mv = scale_divider_to_battery_mv(vadc_mv);
    *out_mv = vbat_mv;
    return ESP_OK;
}

esp_err_t battery_read_percent(battery_t *bat, uint8_t *out_percent)
{
    if (!out_percent) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t mv = 0;
    esp_err_t err = battery_read_mv(bat, &mv);
    if (err != ESP_OK) {
        return err;
    }

    *out_percent = mv_to_percent(mv);
    return ESP_OK;
}

void battery_deinit(battery_t *bat)
{
    if (!bat || !bat->inited) {
        return;
    }

    if (bat->cali_enabled && bat->cali) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        (void)adc_cali_delete_scheme_curve_fitting((adc_cali_handle_t)bat->cali);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
        (void)adc_cali_delete_scheme_line_fitting((adc_cali_handle_t)bat->cali);
#endif
    }

    if (bat->unit) {
        (void)adc_oneshot_del_unit((adc_oneshot_unit_handle_t)bat->unit);
    }

    gpio_set_level(bat->en_gpio, 0);

    *bat = (battery_t){0};
}
