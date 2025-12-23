#include "ble_alarm.h"

#include <string.h>

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"

#include "nvs_flash.h"

static const char *TAG = "BLE";

#define DEVICE_NAME_DEFAULT "LightClock_001"

#define SVC_UUID_16  0xFF10
#define CHAR_UUID_16 0xFF11

#define NUM_HANDLES  4

static ble_alarm_on_write_hhmm_t s_on_write;
static ble_alarm_on_read_hhmm_t s_on_read;
static ble_alarm_on_connect_t s_on_connect;
static ble_alarm_on_disconnect_t s_on_disconnect;
static void *s_ctx;

static bool s_inited;
static bool s_connected;
static bool s_adv_started;
static bool s_want_adv;
static bool s_adv_data_ready;
static bool s_adv_config_attempted;

static uint16_t s_gatts_if;
static uint16_t s_conn_id;
static esp_bd_addr_t s_remote_bda;
static uint16_t s_service_handle;
static uint16_t s_char_handle;

static esp_attr_value_t s_char_val;
static uint8_t s_char_val_buf[4] = {'0','7','0','0'};

static uint8_t s_adv_config_done;
#define ADV_CONFIG_FLAG      (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)

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
        } else {
            ESP_LOGI(TAG, "adv started");
            s_adv_started = true;
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        ESP_LOGI(TAG, "adv stopped");
        s_adv_started = false;
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

        esp_bt_uuid_t char_uuid = {.len = ESP_UUID_LEN_16, .uuid = {.uuid16 = CHAR_UUID_16}};

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
        s_char_handle = param->add_char.attr_handle;
        ESP_LOGI(TAG, "char handle=%u", (unsigned)s_char_handle);
        break;

    case ESP_GATTS_CONNECT_EVT:
        s_connected = true;
        s_conn_id = param->connect.conn_id;
        memcpy(s_remote_bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(TAG, "connected");
        if (s_on_connect) {
            s_on_connect(s_ctx);
        }
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        s_connected = false;
        s_conn_id = 0;
        memset(s_remote_bda, 0, sizeof(s_remote_bda));
        ESP_LOGI(TAG, "disconnected reason=0x%02x", param->disconnect.reason);
        if (s_on_disconnect) {
            s_on_disconnect(s_ctx);
        }
        // stay stopped; app decides whether to restart advertising
        break;

    case ESP_GATTS_READ_EVT: {
        if (!param->read.need_rsp) {
            break;
        }
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(rsp));
        rsp.attr_value.handle = param->read.handle;

        if (param->read.handle == s_char_handle) {
            uint8_t hhmm[4] = {0};
            if (s_on_read) {
                s_on_read(hhmm, s_ctx);
            } else {
                memcpy(hhmm, s_char_val_buf, sizeof(hhmm));
            }
            memcpy(rsp.attr_value.value, hhmm, sizeof(hhmm));
            rsp.attr_value.len = sizeof(hhmm);
            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
        } else {
            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_READ_NOT_PERMIT, &rsp);
        }
        break;
    }

    case ESP_GATTS_WRITE_EVT: {
        if (param->write.handle != s_char_handle) {
            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_WRITE_NOT_PERMIT, NULL);
            }
            break;
        }

        esp_gatt_status_t st = ESP_GATT_OK;
        bool accepted = false;

        if (param->write.len == 4 && s_on_write) {
            accepted = s_on_write(param->write.value, s_ctx);
        }
        if (!accepted) {
            st = ESP_GATT_INVALID_ATTR_LEN;
        } else {
            memcpy(s_char_val_buf, param->write.value, 4);
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

esp_err_t ble_alarm_init(ble_alarm_on_write_hhmm_t on_write,
                         ble_alarm_on_read_hhmm_t on_read,
                         ble_alarm_on_connect_t on_connect,
                         ble_alarm_on_disconnect_t on_disconnect,
                         void *ctx)
{
    if (s_inited) {
        return ESP_OK;
    }

    s_on_write = on_write;
    s_on_read = on_read;
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

    // If adv payload isn't configured yet, defer starting until GAP callback.
    if (!s_adv_data_ready || s_adv_config_done != 0) {
        ESP_LOGI(TAG, "adv deferred (data_ready=%d cfg_done=0x%02x)", (int)s_adv_data_ready, (unsigned)s_adv_config_done);
        return ESP_OK;
    }

    if (!s_adv_started) {
        // Adv data is configured; start immediately.
        esp_err_t err = esp_ble_gap_start_advertising(&s_adv_params);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ble_gap_start_advertising failed: %s", esp_err_to_name(err));
            return err;
        }
        return ESP_OK;
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

    s_inited = false;
    s_connected = false;
    s_adv_started = false;
    s_adv_data_ready = false;
    s_gatts_if = 0;
    s_conn_id = 0;
    s_service_handle = 0;
    s_char_handle = 0;

    return ESP_OK;
}
