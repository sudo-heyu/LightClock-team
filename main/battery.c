#include "battery.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "sdkconfig.h"

static const char *TAG = "BATT";

// Actual hardware divider (schematic): Rtop=10k (R18) in series, Rbot=5.1k (R19)
// Vbat = Vadc * (Rtop + Rbot) / Rbot = Vadc * 15100 / 5100 ≈ 2.96x
#define BATT_DIV_NUMERATOR_MOHM   (10000 + 5100)
#define BATT_DIV_DENOMINATOR_MOHM (5100)

// New requirement (2S pack):
//  - ~7.0V => 0%
//  - ~8.4V => 100%
// Clamp outside the range.
#define BATT_EMPTY_MV (7000)
#define BATT_FULL_MV  (8400)

// Fallback conversion constants when ADC calibration is unavailable.
// SAR ADC (12-bit): Vdata = Vref * data / 4095
// ESP32-C3 default Vref is ~1100mV (factory). With 11dB attenuation, measurable range ~3.55V.
// Use a simple rational approximation for 11dB scaling: 355/110 ≈ 3.227, but we keep headroom and use 3.2.
#define ADC_FALLBACK_VREF_MV (1100)
#define ADC_FALLBACK_ATTEN_DB11_NUM (32)
#define ADC_FALLBACK_ATTEN_DB11_DEN (10)

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

uint8_t battery_mv_to_percent(uint32_t mv)
{
    return mv_to_percent(mv);
}

static int battery_read_raw_avg(adc_oneshot_unit_handle_t unit, adc_channel_t channel)
{
    int raw_sum = 0;
    const int samples = 8;
    for (int i = 0; i < samples; i++) {
        int raw = 0;
        if (adc_oneshot_read(unit, channel, &raw) != ESP_OK) {
            return -1;
        }
        raw_sum += raw;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return raw_sum / samples;
}

esp_err_t battery_init(battery_t *bat, gpio_num_t adc_gpio, gpio_num_t en_gpio)
{
    if (!bat) {
        return ESP_ERR_INVALID_ARG;
    }

    *bat = (battery_t){0};
    bat->adc_gpio = adc_gpio;
    bat->en_gpio = en_gpio;
    bat->en_active_high = true;

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
        // Use 11dB attenuation to avoid saturation near 2.0V ADC input (≈8.4V battery after divider).
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

#if CONFIG_LIGHT_ALARM_DEBUG_BAT_ADC_EN_ALWAYS_HIGH
    // Debug mode: keep BAT_ADC_EN high all the time as requested.
    // In this mode we assume active-high gating and do not auto-detect polarity.
    bat->en_active_high = true;
    gpio_set_level(bat->en_gpio, 1);
    ESP_LOGW(TAG, "DEBUG: BAT_ADC_EN forced HIGH permanently");
    ESP_LOGI(TAG, "battery init: gpio_adc=%d unit=%d chan=%d cali=%d en_active_high=%d",
             (int)adc_gpio,
             (int)unit_id,
             (int)channel,
             (int)cali_ok,
             (int)bat->en_active_high);
    return ESP_OK;
#endif

    // Auto-detect BAT_ADC_EN polarity (some boards wire the enable transistor inverted).
    // We assume the enabled state produces a significantly higher ADC reading than the disabled state.
    {
        adc_oneshot_unit_handle_t u = (adc_oneshot_unit_handle_t)bat->unit;
        adc_channel_t ch = (adc_channel_t)bat->channel;

        gpio_set_level(bat->en_gpio, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
        int raw_high = battery_read_raw_avg(u, ch);

        gpio_set_level(bat->en_gpio, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        int raw_low = battery_read_raw_avg(u, ch);

        // Return to "disabled" by default (active level decided below).
        const int margin = 50;
        if (raw_high >= 0 && raw_low >= 0) {
            if (raw_high > raw_low + margin) {
                bat->en_active_high = true;
            } else if (raw_low > raw_high + margin) {
                bat->en_active_high = false;
            } else {
                // Ambiguous; keep default and warn.
                bat->en_active_high = true;
                ESP_LOGW(TAG, "BAT_ADC_EN polarity ambiguous (raw_high=%d raw_low=%d); default active-high", raw_high, raw_low);
            }
        }

        ESP_LOGI(TAG, "battery init: gpio_adc=%d unit=%d chan=%d cali=%d en_active_high=%d raw_high=%d raw_low=%d",
                 (int)adc_gpio,
                 (int)unit_id,
                 (int)channel,
                 (int)cali_ok,
                 (int)bat->en_active_high,
                 raw_high,
                 raw_low);

        // Ensure sampling path is disabled when idle.
        gpio_set_level(bat->en_gpio, bat->en_active_high ? 0 : 1);
    }
    return ESP_OK;
}

esp_err_t battery_read_mv(battery_t *bat, uint32_t *out_mv)
{
    if (!bat || !bat->inited || !out_mv) {
        return ESP_ERR_INVALID_ARG;
    }

#if CONFIG_LIGHT_ALARM_DEBUG_BAT_ADC_EN_ALWAYS_HIGH
    int enable_level = 1;
    int disable_level = 1; // keep high after read
#else
    int enable_level = bat->en_active_high ? 1 : 0;
    int disable_level = bat->en_active_high ? 0 : 1;
#endif

    gpio_set_level(bat->en_gpio, enable_level);
    // Requirement.md suggests >=10ms settle time after enabling BAT_ADC_EN.
    vTaskDelay(pdMS_TO_TICKS(10));

    adc_oneshot_unit_handle_t unit = (adc_oneshot_unit_handle_t)bat->unit;
    adc_channel_t channel = (adc_channel_t)bat->channel;

    int raw_avg = battery_read_raw_avg(unit, channel);
    gpio_set_level(bat->en_gpio, disable_level);
    if (raw_avg < 0) {
        return ESP_FAIL;
    }

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
        uint64_t mv = (uint64_t)raw_avg * (uint64_t)ADC_FALLBACK_VREF_MV;
        mv = (mv + 2047ULL) / 4095ULL;
        // Apply attenuation scaling (11dB ~3.2x in this approximation).
        mv = (mv * (uint64_t)ADC_FALLBACK_ATTEN_DB11_NUM + (ADC_FALLBACK_ATTEN_DB11_DEN / 2)) / (uint64_t)ADC_FALLBACK_ATTEN_DB11_DEN;
        vadc_mv = (uint32_t)mv;
    }

    uint32_t vbat_mv = scale_divider_to_battery_mv(vadc_mv);
    *out_mv = vbat_mv;

    // Helpful for diagnosing saturation / wiring / divider issues.
    // Bump to INFO so it is visible without raising global log level.
    ESP_LOGI(TAG, "adc raw_avg=%d vadc_mv=%u vbat_mv=%u", raw_avg, (unsigned)vadc_mv, (unsigned)vbat_mv);
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

    uint8_t pct = mv_to_percent(mv);
    *out_percent = pct;

    // Rate-limited info log: prints measured battery voltage and mapped percent.
    static int64_t s_last_info_us;
    int64_t now_us = esp_timer_get_time();
    if (now_us - s_last_info_us >= 5LL * 1000000LL) {
        s_last_info_us = now_us;
        ESP_LOGI(TAG, "battery: %u mV (%.2f V) -> %u%% (map %u..%u mV)",
                 (unsigned)mv,
                 (double)mv / 1000.0,
                 (unsigned)pct,
                 (unsigned)BATT_EMPTY_MV,
                 (unsigned)BATT_FULL_MV);
    }

    if (mv < 1000) {
        ESP_LOGW(TAG, "battery voltage too low (%u mV); check BAT_ADC/BAT_ADC_EN wiring and divider", (unsigned)mv);
    }
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
