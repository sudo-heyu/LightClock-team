#include "ble_alarm.h"

#include <string.h>

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "nvs_flash.h"

static const char *TAG = "BLE";

#define DEVICE_NAME_DEFAULT "LightClock_001"

#define SVC_UUID_16  0xFF10
#define ALARM_CHAR_UUID_16     0xFF11
#define TIME_SYNC_CHAR_UUID_16 0xFF12
#define BATT_CHAR_UUID_16      0xFF13
#define COLOR_TEMP_CHAR_UUID_16  0xFF14
#define WAKE_BRIGHT_CHAR_UUID_16 0xFF15
#define SUNRISE_DUR_CHAR_UUID_16  0xFF16
#define UUID16_CCCD            0x2902

// Primary service + (char decl/value) + descriptors.
// Keep some headroom as we extend characteristics.
#define NUM_HANDLES  20

static ble_alarm_on_write_hhmme_t s_on_write;
static ble_alarm_on_read_hhmme_t s_on_read;
static ble_alarm_on_write_hhmmss_t s_on_time_sync;
static ble_alarm_on_read_batt_percent_t s_on_batt_read;
static ble_alarm_on_write_u8_t s_on_color_temp_write;
static ble_alarm_on_write_u8_t s_on_wake_bright_write;
static ble_alarm_on_write_u8_t s_on_sunrise_dur_write;
static ble_alarm_on_connect_t s_on_connect;
static ble_alarm_on_disconnect_t s_on_disconnect;
static void *s_ctx;

static bool s_inited;
static bool s_connected;
static bool s_adv_started;
static bool s_want_adv;
static bool s_adv_data_ready;
static bool s_adv_config_attempted;

static esp_timer_handle_t s_adv_retry_timer;

static void ble_alarm_try_config_adv_payloads(void);
static void ble_alarm_adv_retry_cb(void *arg);

static void ble_alarm_schedule_adv_retry_ms(uint32_t delay_ms)
{
    if (!s_adv_retry_timer) {
        return;
    }

    // Best-effort: stop any existing schedule and start a new one.
    (void)esp_timer_stop(s_adv_retry_timer);
    (void)esp_timer_start_once(s_adv_retry_timer, (uint64_t)delay_ms * 1000ULL);
}

static uint16_t s_gatts_if;
static uint16_t s_conn_id;
static esp_bd_addr_t s_remote_bda;
static uint16_t s_service_handle;
static uint16_t s_alarm_char_handle;
static uint16_t s_time_sync_char_handle;
static uint16_t s_batt_char_handle;
static uint16_t s_batt_cccd_handle;
static uint16_t s_color_temp_char_handle;
static uint16_t s_wake_bright_char_handle;
static uint16_t s_sunrise_dur_char_handle;
static bool s_batt_notify_enabled;

static esp_attr_value_t s_char_val;
static uint8_t s_char_val_buf[5] = {'0','7','0','0','1'};

static uint16_t s_cccd_val = 0x0000;

static esp_attr_value_t s_cccd_attr = {
    .attr_max_len = sizeof(s_cccd_val),
    .attr_len = sizeof(s_cccd_val),
    .attr_value = (uint8_t *)&s_cccd_val,
};

static uint8_t s_adv_config_done;
#define ADV_CONFIG_FLAG      (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)

static bool is_ascii_digit(uint8_t c)
{
    return (c >= '0' && c <= '9');
}

static bool normalize_hhmmss6(const uint8_t *data, uint16_t len, uint8_t out_hhmmss6[6])
{
    if (!data || !out_hhmmss6 || len == 0) {
        return false;
    }

    // Preferred: exactly 6 ASCII digits.
    if (len == 6) {
        for (int i = 0; i < 6; i++) {
            if (!is_ascii_digit(data[i])) {
                return false;
            }
        }
        memcpy(out_hhmmss6, data, 6);
        return true;
    }

    // Robust: collect first 6 digits from payload (e.g. "12:30:05" or with \r\n).
    uint8_t tmp[6];
    int n = 0;
    for (uint16_t i = 0; i < len && n < 6; i++) {
        if (is_ascii_digit(data[i])) {
            tmp[n++] = data[i];
        }
    }
    if (n != 6) {
        return false;
    }
    memcpy(out_hhmmss6, tmp, 6);
    return true;
}

static bool try_normalize_write_to_hhmm4(const uint8_t *data, uint16_t len, uint8_t out_hhmm4[4])
{
    if (!data || !out_hhmm4 || len == 0) {
        return false;
    }

    // 1) Preferred format: 4 ASCII digits "HHMM".
    if (len == 4 && is_ascii_digit(data[0]) && is_ascii_digit(data[1]) && is_ascii_digit(data[2]) && is_ascii_digit(data[3])) {
        memcpy(out_hhmm4, data, 4);
        return true;
    }

    // 2) Common tooling format: ASCII with terminator / CRLF / spaces.
    uint16_t start = 0;
    uint16_t end = len;
    while (start < end && (data[start] == 0 || data[start] == ' ' || data[start] == '\r' || data[start] == '\n' || data[start] == '\t')) {
        start++;
    }
    while (end > start && (data[end - 1] == 0 || data[end - 1] == ' ' || data[end - 1] == '\r' || data[end - 1] == '\n' || data[end - 1] == '\t')) {
        end--;
    }
    if (end > start) {
        uint16_t trimmed_len = (uint16_t)(end - start);
        if (trimmed_len == 4 && is_ascii_digit(data[start]) && is_ascii_digit(data[start + 1]) && is_ascii_digit(data[start + 2]) &&
            is_ascii_digit(data[start + 3])) {
            memcpy(out_hhmm4, &data[start], 4);
            return true;
        }

        // 3) Tooling sometimes sends decimal string (e.g. "1325") but as a number with fewer digits.
        // Accept 1..4 digits and zero-pad to 4.
        if (trimmed_len >= 1 && trimmed_len <= 4) {
            bool all_digits = true;
            for (uint16_t i = 0; i < trimmed_len; i++) {
                if (!is_ascii_digit(data[start + i])) {
                    all_digits = false;
                    break;
                }
            }
            if (all_digits) {
                // Left pad with '0'
                uint16_t pad = (uint16_t)(4 - trimmed_len);
                memset(out_hhmm4, '0', pad);
                memcpy(&out_hhmm4[pad], &data[start], trimmed_len);
                return true;
            }
        }
    }

    // 4) Packed BCD (2 bytes): 0xHH 0xMM (each nibble 0..9)
    if (len == 2) {
        uint8_t hh = data[0];
        uint8_t mm = data[1];
        uint8_t hh_hi = (uint8_t)((hh >> 4) & 0x0F);
        uint8_t hh_lo = (uint8_t)(hh & 0x0F);
        uint8_t mm_hi = (uint8_t)((mm >> 4) & 0x0F);
        uint8_t mm_lo = (uint8_t)(mm & 0x0F);
        if (hh_hi <= 9 && hh_lo <= 9 && mm_hi <= 9 && mm_lo <= 9) {
            out_hhmm4[0] = (uint8_t)('0' + hh_hi);
            out_hhmm4[1] = (uint8_t)('0' + hh_lo);
            out_hhmm4[2] = (uint8_t)('0' + mm_hi);
            out_hhmm4[3] = (uint8_t)('0' + mm_lo);
            return true;
        }
    }

    // 5) Little-endian integer (2 or 4 bytes) representing HHMM as a decimal number (e.g. 1325).
    if (len == 2 || len == 4) {
        uint32_t v_le = 0;
        for (uint16_t i = 0; i < len; i++) {
            v_le |= ((uint32_t)data[i]) << (8 * i);
        }

        if (v_le <= 2359) {
            uint32_t hh = v_le / 100;
            uint32_t mm = v_le % 100;
            if (hh < 24 && mm < 60) {
                out_hhmm4[0] = (uint8_t)('0' + (hh / 10));
                out_hhmm4[1] = (uint8_t)('0' + (hh % 10));
                out_hhmm4[2] = (uint8_t)('0' + (mm / 10));
                out_hhmm4[3] = (uint8_t)('0' + (mm % 10));
                return true;
            }
        }
    }

    return false;
}

static bool try_normalize_write_to_hhmme5(const uint8_t *data, uint16_t len, uint8_t out_hhmme5[5])
{
    if (!data || !out_hhmme5 || len == 0) {
        return false;
    }

    // 1) Preferred: exactly 5 ASCII digits "HHMME" where E is 0/1.
    if (len == 5) {
        if (is_ascii_digit(data[0]) && is_ascii_digit(data[1]) && is_ascii_digit(data[2]) && is_ascii_digit(data[3]) &&
            (data[4] == '0' || data[4] == '1')) {
            memcpy(out_hhmme5, data, 5);
            return true;
        }
    }

    // 2) Backward-compatible: accept HHMM (enable defaults to 1)
    {
        uint8_t hhmm4[4] = {0};
        if (try_normalize_write_to_hhmm4(data, len, hhmm4)) {
            memcpy(out_hhmme5, hhmm4, 4);
            out_hhmme5[4] = '1';
            return true;
        }
    }

    // 3) Robust: trim and collect digits; use first 4 digits as HHMM and optional 5th as E.
    uint16_t start = 0;
    uint16_t end = len;
    while (start < end && (data[start] == 0 || data[start] == ' ' || data[start] == '\r' || data[start] == '\n' || data[start] == '\t')) {
        start++;
    }
    while (end > start && (data[end - 1] == 0 || data[end - 1] == ' ' || data[end - 1] == '\r' || data[end - 1] == '\n' || data[end - 1] == '\t')) {
        end--;
    }
    if (end <= start) {
        return false;
    }

    uint8_t digits[5] = {0};
    int n = 0;
    for (uint16_t i = start; i < end && n < 5; i++) {
        if (is_ascii_digit(data[i])) {
            digits[n++] = data[i];
        }
    }
    if (n < 4) {
        return false;
    }
    memcpy(out_hhmme5, digits, 4);
    out_hhmme5[4] = (n >= 5) ? digits[4] : (uint8_t)'1';
    if (out_hhmme5[4] != '0' && out_hhmme5[4] != '1') {
        out_hhmme5[4] = '1';
    }
    return true;
}

// Raw advertising payload (legacy, <= 31 bytes):
//  - Flags: 0x06 (LE General Discoverable + BR/EDR not supported)
//  - Complete list of 16-bit Service UUIDs: 0xFF10
static const uint8_t s_adv_raw[] = {0x02, 0x01, 0x06, 0x03, 0x03, 0x10, 0xFF};

static void ble_alarm_try_config_adv_payloads(void)
{
    if (!s_inited) {
        return;
    }
    if (s_adv_data_ready || s_adv_config_done != 0) {
        return;
    }
    if (s_adv_config_attempted) {
        return;
    }
    s_adv_config_attempted = true;

    esp_err_t err = esp_ble_gap_set_device_name(DEVICE_NAME_DEFAULT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set dev name failed: %s", esp_err_to_name(err));
    }

    // Configure raw advertising and scan response payloads.
    s_adv_data_ready = false;
    s_adv_config_done = (uint8_t)(ADV_CONFIG_FLAG | SCAN_RSP_CONFIG_FLAG);

    // Build scan response: Complete Local Name (AD type 0x09)
    // Legacy scan response payload limit is 31 bytes total.
    size_t name_len = strlen(DEVICE_NAME_DEFAULT);
    if (name_len > 29) {
        ESP_LOGW(TAG, "device name too long for scan rsp (%u), truncating", (unsigned)name_len);
        name_len = 29;
    }
    uint8_t scan_rsp_raw[31];
    scan_rsp_raw[0] = (uint8_t)(1 + name_len); // length of (type + data)
    scan_rsp_raw[1] = 0x09;                    // Complete Local Name
    memcpy(&scan_rsp_raw[2], DEVICE_NAME_DEFAULT, name_len);
    const uint8_t scan_rsp_len = (uint8_t)(2 + name_len);

    ESP_LOGI(TAG, "adv raw len=%u scan_rsp len=%u name_len=%u", (unsigned)sizeof(s_adv_raw), (unsigned)scan_rsp_len,
             (unsigned)name_len);

    err = esp_ble_gap_config_adv_data_raw((uint8_t *)s_adv_raw, sizeof(s_adv_raw));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "config adv raw failed: %s", esp_err_to_name(err));
        // Clear flag to avoid indefinite defer loops.
        s_adv_config_done &= (uint8_t)(~ADV_CONFIG_FLAG);
    }

    err = esp_ble_gap_config_scan_rsp_data_raw(scan_rsp_raw, scan_rsp_len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "config scan rsp raw failed: %s", esp_err_to_name(err));
        s_adv_config_done &= (uint8_t)(~SCAN_RSP_CONFIG_FLAG);
    }
}

static esp_ble_adv_params_t s_adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static void gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        s_adv_config_done &= (uint8_t)(~ADV_CONFIG_FLAG);
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        s_adv_config_done &= (uint8_t)(~SCAN_RSP_CONFIG_FLAG);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "adv start failed: %d", param->adv_start_cmpl.status);
            s_adv_started = false;
            // Retry soon; some stacks sporadically fail start after connections.
            if (s_want_adv && !s_connected) {
                ble_alarm_schedule_adv_retry_ms(500);
            }
        } else {
            ESP_LOGI(TAG, "adv started");
            s_adv_started = true;
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        ESP_LOGI(TAG, "adv stopped");
        s_adv_started = false;
        // Self-heal: if we still want advertising and we're not connected, restart.
        if (s_want_adv && !s_connected) {
            ble_alarm_schedule_adv_retry_ms(200);
        }
        break;
    default:
        break;
    }

    // Start advertising once BOTH raw payloads are configured.
    if (!s_adv_data_ready && s_adv_config_done == 0) {
        s_adv_data_ready = true;
        ESP_LOGI(TAG, "adv payloads ready");
    }

    if (s_adv_config_done == 0 && s_adv_data_ready && s_want_adv && !s_adv_started && !s_connected) {
        esp_err_t err = esp_ble_gap_start_advertising(&s_adv_params);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ble_gap_start_advertising failed: %s", esp_err_to_name(err));
            ble_alarm_schedule_adv_retry_ms(800);
        }
    }
}

static void gatts_cb(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT: {
        s_gatts_if = gatts_if;

        // Configure advertising payloads lazily (on first adv request) to avoid any
        // timing issues during early init.
        // If ALWAYS_ON calls start immediately, ble_alarm_start_advertising() will trigger it.
        // Otherwise, first call to ble_alarm_start_advertising() will do it.

        esp_gatt_srvc_id_t svc_id = {
            .is_primary = true,
            .id = {
                .inst_id = 0,
                .uuid = {.len = ESP_UUID_LEN_16, .uuid = {.uuid16 = SVC_UUID_16}},
            },
        };

        esp_ble_gatts_create_service(gatts_if, &svc_id, NUM_HANDLES);
        break;
    }
    case ESP_GATTS_CREATE_EVT: {
        s_service_handle = param->create.service_handle;
        esp_ble_gatts_start_service(s_service_handle);

        esp_bt_uuid_t char_uuid = {.len = ESP_UUID_LEN_16, .uuid = {.uuid16 = ALARM_CHAR_UUID_16}};

        s_char_val.attr_max_len = sizeof(s_char_val_buf);
        s_char_val.attr_len = sizeof(s_char_val_buf);
        s_char_val.attr_value = s_char_val_buf;

        esp_gatt_char_prop_t prop = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE;
        esp_err_t err = esp_ble_gatts_add_char(s_service_handle,
                                              &char_uuid,
                                              ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                              prop,
                                              &s_char_val,
                                              NULL);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "add char failed: %s", esp_err_to_name(err));
        }
        break;
    }
    case ESP_GATTS_ADD_CHAR_EVT:
        if (param->add_char.char_uuid.len == ESP_UUID_LEN_16) {
            uint16_t uuid16 = param->add_char.char_uuid.uuid.uuid16;
            if (uuid16 == ALARM_CHAR_UUID_16) {
                s_alarm_char_handle = param->add_char.attr_handle;
                ESP_LOGI(TAG, "alarm char handle=%u", (unsigned)s_alarm_char_handle);

                // Add time sync characteristic (write-only)
                esp_bt_uuid_t time_uuid = {.len = ESP_UUID_LEN_16, .uuid = {.uuid16 = TIME_SYNC_CHAR_UUID_16}};
                esp_gatt_char_prop_t prop = ESP_GATT_CHAR_PROP_BIT_WRITE;
                esp_err_t err = esp_ble_gatts_add_char(s_service_handle,
                                                      &time_uuid,
                                                      ESP_GATT_PERM_WRITE,
                                                      prop,
                                                      NULL,
                                                      NULL);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "add time sync char failed: %s", esp_err_to_name(err));
                }
            } else if (uuid16 == TIME_SYNC_CHAR_UUID_16) {
                s_time_sync_char_handle = param->add_char.attr_handle;
                ESP_LOGI(TAG, "time sync char handle=%u", (unsigned)s_time_sync_char_handle);

                // Add battery characteristic (read + notify)
                esp_bt_uuid_t batt_uuid = {.len = ESP_UUID_LEN_16, .uuid = {.uuid16 = BATT_CHAR_UUID_16}};
                esp_gatt_char_prop_t prop = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
                esp_err_t err = esp_ble_gatts_add_char(s_service_handle,
                                                      &batt_uuid,
                                                      ESP_GATT_PERM_READ,
                                                      prop,
                                                      NULL,
                                                      NULL);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "add batt char failed: %s", esp_err_to_name(err));
                }
            } else if (uuid16 == BATT_CHAR_UUID_16) {
                s_batt_char_handle = param->add_char.attr_handle;
                ESP_LOGI(TAG, "batt char handle=%u", (unsigned)s_batt_char_handle);

                // Add CCCD for notifications.
                esp_bt_uuid_t cccd_uuid = {.len = ESP_UUID_LEN_16, .uuid = {.uuid16 = UUID16_CCCD}};
                esp_err_t err = esp_ble_gatts_add_char_descr(s_service_handle,
                                                           &cccd_uuid,
                                                           ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                           &s_cccd_attr,
                                                           NULL);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "add cccd failed: %s", esp_err_to_name(err));
                }
            } else if (uuid16 == COLOR_TEMP_CHAR_UUID_16) {
                s_color_temp_char_handle = param->add_char.attr_handle;
                ESP_LOGI(TAG, "color temp char handle=%u", (unsigned)s_color_temp_char_handle);

                // Add wake brightness characteristic (write-only)
                esp_bt_uuid_t wb_uuid = {.len = ESP_UUID_LEN_16, .uuid = {.uuid16 = WAKE_BRIGHT_CHAR_UUID_16}};
                esp_gatt_char_prop_t prop = ESP_GATT_CHAR_PROP_BIT_WRITE;
                esp_err_t err = esp_ble_gatts_add_char(s_service_handle,
                                                      &wb_uuid,
                                                      ESP_GATT_PERM_WRITE,
                                                      prop,
                                                      NULL,
                                                      NULL);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "add wake bright char failed: %s", esp_err_to_name(err));
                }
            } else if (uuid16 == WAKE_BRIGHT_CHAR_UUID_16) {
                s_wake_bright_char_handle = param->add_char.attr_handle;
                ESP_LOGI(TAG, "wake bright char handle=%u", (unsigned)s_wake_bright_char_handle);

                // Add sunrise duration characteristic (write-only, minutes)
                esp_bt_uuid_t sd_uuid = {.len = ESP_UUID_LEN_16, .uuid = {.uuid16 = SUNRISE_DUR_CHAR_UUID_16}};
                esp_gatt_char_prop_t prop = ESP_GATT_CHAR_PROP_BIT_WRITE;
                esp_err_t err = esp_ble_gatts_add_char(s_service_handle,
                                                      &sd_uuid,
                                                      ESP_GATT_PERM_WRITE,
                                                      prop,
                                                      NULL,
                                                      NULL);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "add sunrise dur char failed: %s", esp_err_to_name(err));
                }
            } else if (uuid16 == SUNRISE_DUR_CHAR_UUID_16) {
                s_sunrise_dur_char_handle = param->add_char.attr_handle;
                ESP_LOGI(TAG, "sunrise dur char handle=%u", (unsigned)s_sunrise_dur_char_handle);
            }
        }
        break;

    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        s_batt_cccd_handle = param->add_char_descr.attr_handle;
        ESP_LOGI(TAG, "batt cccd handle=%u", (unsigned)s_batt_cccd_handle);

        // Add color temperature characteristic (write-only)
        {
            esp_bt_uuid_t ct_uuid = {.len = ESP_UUID_LEN_16, .uuid = {.uuid16 = COLOR_TEMP_CHAR_UUID_16}};
            esp_gatt_char_prop_t prop = ESP_GATT_CHAR_PROP_BIT_WRITE;
            esp_err_t err = esp_ble_gatts_add_char(s_service_handle,
                                                  &ct_uuid,
                                                  ESP_GATT_PERM_WRITE,
                                                  prop,
                                                  NULL,
                                                  NULL);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "add color temp char failed: %s", esp_err_to_name(err));
            }
        }
        break;

    case ESP_GATTS_CONNECT_EVT:
        s_connected = true;
        // Most stacks automatically stop advertising upon a successful connection,
        // but do not always emit ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT. Keep our state consistent.
        s_adv_started = false;
        s_conn_id = param->connect.conn_id;
        memcpy(s_remote_bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(TAG, "connected");
        if (s_on_connect) {
            s_on_connect(s_ctx);
        }
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        s_connected = false;
        // Ensure we allow immediate restart; advertising likely stopped while connected.
        s_adv_started = false;
        s_conn_id = 0;
        memset(s_remote_bda, 0, sizeof(s_remote_bda));
        ESP_LOGI(TAG, "disconnected reason=0x%02x", param->disconnect.reason);
        if (s_on_disconnect) {
            s_on_disconnect(s_ctx);
        }
        // Self-heal: if app wants advertising, restart after disconnect.
        if (s_want_adv) {
            ble_alarm_schedule_adv_retry_ms(50);
        }
        break;

    case ESP_GATTS_READ_EVT: {
        if (!param->read.need_rsp) {
            break;
        }
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(rsp));
        rsp.attr_value.handle = param->read.handle;

        if (param->read.handle == s_alarm_char_handle) {
            uint8_t hhmme[5] = {0};
            if (s_on_read) {
                s_on_read(hhmme, s_ctx);
            } else {
                memcpy(hhmme, s_char_val_buf, sizeof(hhmme));
            }
            memcpy(rsp.attr_value.value, hhmme, sizeof(hhmme));
            rsp.attr_value.len = sizeof(hhmme);
            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
        } else if (param->read.handle == s_batt_char_handle) {
            uint8_t pct = 0;
            if (s_on_batt_read) {
                pct = s_on_batt_read(s_ctx);
            }
            rsp.attr_value.value[0] = pct;
            rsp.attr_value.len = 1;
            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
        } else {
            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_READ_NOT_PERMIT, &rsp);
        }
        break;
    }

    case ESP_GATTS_WRITE_EVT: {
        ESP_LOGI(TAG, "write: handle=%u len=%u need_rsp=%u", (unsigned)param->write.handle, (unsigned)param->write.len,
                 (unsigned)param->write.need_rsp);
        if (param->write.len > 0 && param->write.value) {
            ESP_LOG_BUFFER_HEX_LEVEL(TAG, param->write.value, param->write.len, ESP_LOG_INFO);
        }

        if (param->write.handle == s_batt_cccd_handle) {
            // Client configuration descriptor for battery notifications.
            esp_gatt_status_t st = ESP_GATT_OK;
            if (param->write.len == 2) {
                uint16_t v = (uint16_t)(param->write.value[0] | (param->write.value[1] << 8));
                s_cccd_val = v;
                s_batt_notify_enabled = ((v & 0x0001) != 0);
                ESP_LOGI(TAG, "batt notify %s", s_batt_notify_enabled ? "EN" : "DIS");

                // Requirement: after a successful connection, provide battery to the client.
                // Typical BLE flow enables CCCD right after connect; send once immediately.
                if (s_batt_notify_enabled && s_on_batt_read) {
                    uint8_t pct = s_on_batt_read(s_ctx);
                    (void)ble_alarm_notify_battery(pct);
                }
            } else {
                st = ESP_GATT_INVALID_ATTR_LEN;
            }
            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, st, NULL);
            }
            break;
        }

        if (param->write.handle == s_time_sync_char_handle) {
            esp_gatt_status_t st = ESP_GATT_OK;
            bool accepted = false;
            uint8_t hhmmss6[6] = {0};
            bool normalized = normalize_hhmmss6(param->write.value, param->write.len, hhmmss6);
            if (normalized && s_on_time_sync) {
                ESP_LOGI(TAG, "time sync normalized to HHMMSS='%c%c%c%c%c%c'", hhmmss6[0], hhmmss6[1], hhmmss6[2], hhmmss6[3], hhmmss6[4],
                         hhmmss6[5]);
                accepted = s_on_time_sync(hhmmss6, s_ctx);
            } else {
                ESP_LOGW(TAG, "time sync rejected: normalized=%d cb=%d", (int)normalized, (int)(s_on_time_sync != NULL));
            }
            if (!accepted) {
                st = ESP_GATT_INVALID_ATTR_LEN;
            }
            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, st, NULL);
            }
            break;
        }

        if (param->write.handle == s_color_temp_char_handle || param->write.handle == s_wake_bright_char_handle ||
            param->write.handle == s_sunrise_dur_char_handle) {
            esp_gatt_status_t st = ESP_GATT_OK;
            bool accepted = false;

            if (param->write.len == 1 && param->write.value) {
                uint8_t v = param->write.value[0];
                if (param->write.handle == s_color_temp_char_handle) {
                    if (v <= 100) {
                        ESP_LOGI(TAG, "color temp write=%u", (unsigned)v);
                        if (s_on_color_temp_write) {
                            accepted = s_on_color_temp_write(v, s_ctx);
                        }
                    }
                } else if (param->write.handle == s_wake_bright_char_handle) {
                    if (v <= 100) {
                        ESP_LOGI(TAG, "wake bright write=%u", (unsigned)v);
                        if (s_on_wake_bright_write) {
                            accepted = s_on_wake_bright_write(v, s_ctx);
                        }
                    }
                } else if (param->write.handle == s_sunrise_dur_char_handle) {
                    if (v >= 1 && v <= 60) {
                        ESP_LOGI(TAG, "sunrise dur write=%u min", (unsigned)v);
                        if (s_on_sunrise_dur_write) {
                            accepted = s_on_sunrise_dur_write(v, s_ctx);
                        }
                    }
                }
            }

            if (!accepted) {
                st = ESP_GATT_INVALID_ATTR_LEN;
            }
            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, st, NULL);
            }
            break;
        }

        if (param->write.handle != s_alarm_char_handle) {
            ESP_LOGW(TAG, "write ignored (unknown handle): got_handle=%u alarm=%u time=%u batt_cccd=%u", (unsigned)param->write.handle,
                     (unsigned)s_alarm_char_handle, (unsigned)s_time_sync_char_handle, (unsigned)s_batt_cccd_handle);
            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_WRITE_NOT_PERMIT, NULL);
            }
            break;
        }

        esp_gatt_status_t st = ESP_GATT_OK;
        bool accepted = false;

        uint8_t hhmme5[5] = {0};
        bool normalized = try_normalize_write_to_hhmme5(param->write.value, param->write.len, hhmme5);
        if (normalized && s_on_write) {
            ESP_LOGI(TAG, "write normalized to HHMME='%c%c%c%c%c'", hhmme5[0], hhmme5[1], hhmme5[2], hhmme5[3], hhmme5[4]);
            accepted = s_on_write(hhmme5, s_ctx);
        } else {
            ESP_LOGW(TAG, "write rejected: normalized=%d on_write=%d", (int)normalized, (int)(s_on_write != NULL));
        }
        if (!accepted) {
            st = ESP_GATT_INVALID_ATTR_LEN;
        } else {
            memcpy(s_char_val_buf, hhmme5, 5);
        }

        if (param->write.need_rsp) {
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, st, NULL);
        }
        break;
    }

    default:
        break;
    }
}

esp_err_t ble_alarm_init(ble_alarm_on_write_hhmme_t on_write,
                         ble_alarm_on_read_hhmme_t on_read,
                         ble_alarm_on_write_hhmmss_t on_time_sync,
                         ble_alarm_on_read_batt_percent_t on_batt_read,
                         ble_alarm_on_write_u8_t on_write_color_temp,
                         ble_alarm_on_write_u8_t on_write_wake_bright,
                         ble_alarm_on_write_u8_t on_write_sunrise_duration,
                         ble_alarm_on_connect_t on_connect,
                         ble_alarm_on_disconnect_t on_disconnect,
                         void *ctx)
{
    if (s_inited) {
        return ESP_OK;
    }

    s_on_write = on_write;
    s_on_read = on_read;
    s_on_time_sync = on_time_sync;
    s_on_batt_read = on_batt_read;
    s_on_color_temp_write = on_write_color_temp;
    s_on_wake_bright_write = on_write_wake_bright;
    s_on_sunrise_dur_write = on_write_sunrise_duration;
    s_on_connect = on_connect;
    s_on_disconnect = on_disconnect;
    s_ctx = ctx;

    // NVS must be initialized by app.

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t err = esp_bt_controller_init(&bt_cfg);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "bt ctrl init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "bt ctrl enable failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_bluedroid_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "bluedroid init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_bluedroid_enable();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "bluedroid enable failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_ble_gap_register_callback(gap_cb);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_ble_gatts_register_callback(gatts_cb);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_ble_gatts_app_register(0);
    if (err != ESP_OK) {
        return err;
    }

    if (!s_adv_retry_timer) {
        const esp_timer_create_args_t args = {
            .callback = &ble_alarm_adv_retry_cb,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "ble_adv_retry",
            .skip_unhandled_events = true,
        };
        (void)esp_timer_create(&args, &s_adv_retry_timer);
    }

    s_adv_config_done = 0;
    s_adv_started = false;
    s_want_adv = false;
    s_adv_data_ready = false;
    s_adv_config_attempted = false;
    s_inited = true;
    return ESP_OK;
}

esp_err_t ble_alarm_start_advertising(void)
{
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_connected) {
        return ESP_OK;
    }

    s_want_adv = true;

    // Ensure adv payloads are configured (or at least attempted) before starting.
    ble_alarm_try_config_adv_payloads();

    // Even if something stops advertising later, keep a retry scheduled.
    ble_alarm_schedule_adv_retry_ms(2000);

    // If adv payload isn't configured yet, defer starting until GAP callback.
    if (!s_adv_data_ready || s_adv_config_done != 0) {
        ESP_LOGI(TAG, "adv deferred (data_ready=%d cfg_done=0x%02x)", (int)s_adv_data_ready, (unsigned)s_adv_config_done);
        return ESP_OK;
    }

    // Adv data is configured; attempt to start immediately.
    // Note: some controller/stack versions may return INVALID_STATE if advertising is already active;
    // treat that as benign and keep the self-heal timer alive.
    esp_err_t err = esp_ble_gap_start_advertising(&s_adv_params);
    if (err != ESP_OK) {
        if (err == ESP_ERR_INVALID_STATE) {
            return ESP_OK;
        }
        ESP_LOGE(TAG, "esp_ble_gap_start_advertising failed: %s", esp_err_to_name(err));
        ble_alarm_schedule_adv_retry_ms(800);
        return err;
    }
    return ESP_OK;
}

esp_err_t ble_alarm_stop_advertising(void)
{
    if (!s_inited) {
        return ESP_OK;
    }
    s_want_adv = false;
    s_adv_started = false;
    if (s_adv_retry_timer) {
        (void)esp_timer_stop(s_adv_retry_timer);
    }
    return esp_ble_gap_stop_advertising();
}

bool ble_alarm_is_connected(void)
{
    return s_connected;
}

bool ble_alarm_is_advertising(void)
{
    return s_adv_started;
}

esp_err_t ble_alarm_disconnect(void)
{
    if (!s_inited || !s_connected) {
        return ESP_OK;
    }
    return esp_ble_gap_disconnect(s_remote_bda);
}

esp_err_t ble_alarm_deinit(void)
{
    if (!s_inited) {
        return ESP_OK;
    }

    (void)ble_alarm_stop_advertising();

    if (s_connected) {
        (void)ble_alarm_disconnect();
    }

    // Best-effort teardown for deep sleep.
    (void)esp_bluedroid_disable();
    (void)esp_bluedroid_deinit();
    (void)esp_bt_controller_disable();
    (void)esp_bt_controller_deinit();

    if (s_adv_retry_timer) {
        (void)esp_timer_stop(s_adv_retry_timer);
        (void)esp_timer_delete(s_adv_retry_timer);
        s_adv_retry_timer = NULL;
    }

    s_inited = false;
    s_connected = false;
    s_adv_started = false;
    s_adv_data_ready = false;
    s_gatts_if = 0;
    s_conn_id = 0;
    s_service_handle = 0;
    s_alarm_char_handle = 0;
    s_time_sync_char_handle = 0;
    s_batt_char_handle = 0;
    s_batt_cccd_handle = 0;
    s_color_temp_char_handle = 0;
    s_wake_bright_char_handle = 0;
    s_sunrise_dur_char_handle = 0;
    s_batt_notify_enabled = false;
    s_cccd_val = 0;

    return ESP_OK;
}

esp_err_t ble_alarm_notify_battery(uint8_t percent)
{
    if (!s_inited || !s_connected || !s_batt_notify_enabled || s_batt_char_handle == 0) {
        return ESP_OK;
    }
    uint8_t v = percent;
    return esp_ble_gatts_send_indicate(s_gatts_if, s_conn_id, s_batt_char_handle, sizeof(v), &v, false);
}

static void ble_alarm_adv_retry_cb(void *arg)
{
    (void)arg;

    if (!s_inited || !s_want_adv || s_connected) {
        return;
    }

    // Ensure payload is configured (first time) and retry advertising start.
    ble_alarm_try_config_adv_payloads();
    if (s_adv_data_ready && s_adv_config_done == 0) {
        esp_err_t err = esp_ble_gap_start_advertising(&s_adv_params);
        if (err != ESP_OK) {
            if (err != ESP_ERR_INVALID_STATE) {
                ESP_LOGW(TAG, "adv retry start failed: %s", esp_err_to_name(err));
                ble_alarm_schedule_adv_retry_ms(1200);
                return;
            }
        }
    }
    // Keep the self-heal alive.
    ble_alarm_schedule_adv_retry_ms(2000);
}
