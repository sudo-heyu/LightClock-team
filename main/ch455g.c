#include "ch455g.h"

#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "CH455";

// Commands per WCH CH455 datasheet
#define CH455_CMD_SYS_PARAM 0x48
#define CH455_CMD_DIG0      0x68
#define CH455_CMD_DIG1      0x6A
#define CH455_CMD_DIG2      0x6C
#define CH455_CMD_DIG3      0x6E

// sys param byte2: [KOFF][INTENS(3)][7SEG][SLEEP]0[ENA]
#define SYS_ENA_BIT   (1u << 0)
#define SYS_SLEEP_BIT (1u << 2)
#define SYS_7SEG_BIT  (1u << 3)
#define SYS_INTENS_SHIFT 4
#define SYS_KOFF_BIT  (1u << 7)

static inline void delay_half_period(void)
{
    // ~100kHz-ish bitbang; keep conservative for signal integrity
    esp_rom_delay_us(5);
}

static inline void scl_high(const ch455g_t *d) { gpio_set_level(d->scl, 1); }
static inline void scl_low(const ch455g_t *d)  { gpio_set_level(d->scl, 0); }
static inline void sda_high(const ch455g_t *d) { gpio_set_level(d->sda, 1); }
static inline void sda_low(const ch455g_t *d)  { gpio_set_level(d->sda, 0); }

static void start_cond(const ch455g_t *d)
{
    sda_high(d);
    scl_high(d);
    delay_half_period();
    sda_low(d);
    delay_half_period();
    scl_low(d);
}

static void stop_cond(const ch455g_t *d)
{
    sda_low(d);
    delay_half_period();
    scl_high(d);
    delay_half_period();
    sda_high(d);
    delay_half_period();
}

static void write_bit(const ch455g_t *d, int bit)
{
    if (bit) {
        sda_high(d);
    } else {
        sda_low(d);
    }
    delay_half_period();
    scl_high(d);
    delay_half_period();
    scl_low(d);
}

static void write_byte_msb_first(const ch455g_t *d, uint8_t byte)
{
    for (int i = 7; i >= 0; i--) {
        write_bit(d, (byte >> i) & 1);
    }
}

static void write_fixed_ack_1(const ch455g_t *d)
{
    // Datasheet: ACK is fixed to 1 (no slave-driven ACK). Keep SDA high for this clock.
    write_bit(d, 1);
}

static esp_err_t ch455_write2(const ch455g_t *d, uint8_t b1, uint8_t b2)
{
    if (!d) {
        return ESP_ERR_INVALID_ARG;
    }

    start_cond(d);
    write_byte_msb_first(d, b1);
    write_fixed_ack_1(d);
    write_byte_msb_first(d, b2);
    write_fixed_ack_1(d);
    stop_cond(d);
    return ESP_OK;
}

static uint8_t sys_param_build(uint8_t intensity_0_7, bool enabled, bool sleep)
{
    // INTENS encoding: 000 => 8/8 (max). 001..111 => 1/8..7/8.
    uint8_t intens = (intensity_0_7 == 0) ? 0 : (uint8_t)(intensity_0_7 & 0x7);

    uint8_t b = 0;
    b |= SYS_KOFF_BIT; // disable keyscan for lower overhead
    b |= (uint8_t)(intens << SYS_INTENS_SHIFT);
    // 8-seg mode (7SEG=0) to keep DP available
    if (sleep) {
        b |= SYS_SLEEP_BIT;
    }
    if (enabled) {
        b |= SYS_ENA_BIT;
    }
    return b;
}

esp_err_t ch455g_init(ch455g_t *dev, gpio_num_t sda, gpio_num_t scl, uint8_t intensity)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }

    dev->sda = sda;
    dev->scl = scl;
    dev->sys_param = sys_param_build(intensity, true, false);

    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << sda) | (1ULL << scl),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));

    // idle high
    gpio_set_level(sda, 1);
    gpio_set_level(scl, 1);

    esp_err_t err = ch455_write2(dev, CH455_CMD_SYS_PARAM, dev->sys_param);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "init sys param failed");
        return err;
    }

    return ch455g_clear(dev);
}

esp_err_t ch455g_set_enabled(ch455g_t *dev, bool enabled)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }
    dev->sys_param = (dev->sys_param & (uint8_t)~SYS_ENA_BIT) | (enabled ? SYS_ENA_BIT : 0);
    return ch455_write2(dev, CH455_CMD_SYS_PARAM, dev->sys_param);
}

esp_err_t ch455g_set_sleep(ch455g_t *dev, bool sleep)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }
    dev->sys_param = (dev->sys_param & (uint8_t)~SYS_SLEEP_BIT) | (sleep ? SYS_SLEEP_BIT : 0);
    return ch455_write2(dev, CH455_CMD_SYS_PARAM, dev->sys_param);
}

esp_err_t ch455g_set_digit_raw(ch455g_t *dev, uint8_t dig_index_0_3, uint8_t seg_byte)
{
    if (!dev || dig_index_0_3 > 3) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t cmd = CH455_CMD_DIG0;
    switch (dig_index_0_3) {
    case 0: cmd = CH455_CMD_DIG0; break;
    case 1: cmd = CH455_CMD_DIG1; break;
    case 2: cmd = CH455_CMD_DIG2; break;
    case 3: cmd = CH455_CMD_DIG3; break;
    default: break;
    }

    return ch455_write2(dev, cmd, seg_byte);
}

esp_err_t ch455g_set_4digits_raw(ch455g_t *dev, uint8_t dig0, uint8_t dig1, uint8_t dig2, uint8_t dig3)
{
    esp_err_t err = ch455g_set_digit_raw(dev, 0, dig0);
    if (err != ESP_OK) return err;
    err = ch455g_set_digit_raw(dev, 1, dig1);
    if (err != ESP_OK) return err;
    err = ch455g_set_digit_raw(dev, 2, dig2);
    if (err != ESP_OK) return err;
    return ch455g_set_digit_raw(dev, 3, dig3);
}

static uint8_t seg_for_digit(int d)
{
    // bit0..6 = A..G, bit7 = DP
    static const uint8_t map[10] = {
        0b00111111, // 0
        0b00000110, // 1
        0b01011011, // 2
        0b01001111, // 3
        0b01100110, // 4
        0b01101101, // 5
        0b01111101, // 6
        0b00000111, // 7
        0b01111111, // 8
        0b01101111, // 9
    };
    if (d < 0 || d > 9) {
        return 0;
    }
    return map[d];
}

esp_err_t ch455g_show_hhmm(ch455g_t *dev, int hour, int minute)
{
    if (!dev || hour < 0 || hour > 23 || minute < 0 || minute > 59) {
        return ESP_ERR_INVALID_ARG;
    }

    int h1 = hour / 10;
    int h2 = hour % 10;
    int m1 = minute / 10;
    int m2 = minute % 10;

    // Hardware wiring confirmed in-field: DIG0..DIG3 is left->right.
    // Example: 15:40 should display as 1 5 4 0.
    uint8_t dig0 = seg_for_digit(h1);
    uint8_t dig1 = seg_for_digit(h2);
    uint8_t dig2 = seg_for_digit(m1);
    uint8_t dig3 = seg_for_digit(m2);

    // Middle separator wiring varies by module.
    // Per latest hardware confirmation: there is NO dedicated colon; only a single DP dot is available.
    // Use a single dot between hour and minute by enabling DP on the center-left digit (hours units).
    dig1 |= 0x80;

    return ch455g_set_4digits_raw(dev, dig0, dig1, dig2, dig3);
}

esp_err_t ch455g_clear(ch455g_t *dev)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }
    return ch455g_set_4digits_raw(dev, 0, 0, 0, 0);
}
