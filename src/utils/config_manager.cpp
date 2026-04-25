/**
 * @file config_manager.cpp
 * @brief CUBE32 NVS Configuration Manager — Implementation
 */

#include "utils/config_manager.h"
#include "cube32_config.h"

#include <cstring>
#include <nvs_flash.h>
#include <nvs.h>
#include <esp_log.h>

static const char* TAG = "cube32_cfg";

/* ============================================================================
 * Singleton
 * ============================================================================ */

static cube32_cfg_t s_cfg = {};

const cube32_cfg_t* cube32_cfg(void)
{
    return &s_cfg;
}

cube32_cfg_t* cube32_cfg_mut(void)
{
    return &s_cfg;
}

/* ============================================================================
 * Display-model helpers
 * ============================================================================ */

uint8_t cube32_cfg_compiled_display_model_id(void)
{
#if defined(CONFIG_CUBE32_DISPLAY_CUBE_TFT_TOUCH_154)
    return 1;
#elif defined(CONFIG_CUBE32_DISPLAY_CUBE_TFT_TOUCH_200)
    return 2;
#else
    return 0;   /* unknown / display disabled */
#endif
}

uint16_t cube32_cfg_default_rotation(uint8_t display_model_id)
{
    (void)display_model_id;
    return CUBE32_LCD_DEFAULT_ROTATION;
}

/* ============================================================================
 * Compiled defaults — used when NVS has no stored value
 * ============================================================================ */

static void cfg_apply_defaults(cube32_cfg_t* cfg)
{
    memset(cfg, 0, sizeof(*cfg));

    /* Display */
    cfg->display_model_id = cube32_cfg_compiled_display_model_id();
    cfg->display_rotation = cube32_cfg_default_rotation(cfg->display_model_id);
    cfg->display_prism    = false;

    /* Audio */
    cfg->audio_volume      = 70;
    cfg->audio_max_db      = 18;
    cfg->audio_input_rate  = 24000;
    cfg->audio_output_rate = 24000;

    /* RTC */
    strncpy(cfg->rtc_timezone, "SGT-8", sizeof(cfg->rtc_timezone) - 1);
    cfg->rtc_ntp_sync = false;

    /* Camera */
    cfg->camera_h_mirror = false;
    cfg->camera_v_flip   = false;

    /* WiFi — empty by default, user must provision */
    cfg->wifi_ssid[0] = '\0';
    cfg->wifi_pass[0] = '\0';

    /* Modem */
    strncpy(cfg->modem_apn, "internet", sizeof(cfg->modem_apn) - 1);

    /* BLE */
    strncpy(cfg->ble_device_name, "CUBE32-OTA", sizeof(cfg->ble_device_name) - 1);

    /* Component active flags — default to active (if hardware is present) */
    cfg->active_camera     = true;
    cfg->active_audio      = true;
    cfg->active_modem      = true;
    cfg->active_sdcard     = true;
    cfg->active_usb_input  = true;
    cfg->active_ble_ota    = true;
    cfg->active_adc_button = true;
    cfg->active_servo      = true;
    cfg->active_imu        = true;
    cfg->active_mag        = true;

    cfg->loaded = false;
    cfg->nvs_found = 0;
}

/* ============================================================================
 * NVS helpers
 * ============================================================================ */

static esp_err_t nvs_read_u8(nvs_handle_t h, const char* key, uint8_t* out)
{
    return nvs_get_u8(h, key, out);
}

static esp_err_t nvs_read_u16(nvs_handle_t h, const char* key, uint16_t* out)
{
    return nvs_get_u16(h, key, out);
}

static esp_err_t nvs_read_u32(nvs_handle_t h, const char* key, uint32_t* out)
{
    return nvs_get_u32(h, key, out);
}

static esp_err_t nvs_read_str(nvs_handle_t h, const char* key, char* buf, size_t buf_len)
{
    size_t len = buf_len;
    return nvs_get_str(h, key, buf, &len);
}

static esp_err_t nvs_read_bool(nvs_handle_t h, const char* key, bool* out)
{
    uint8_t v = 0;
    esp_err_t err = nvs_get_u8(h, key, &v);
    if (err == ESP_OK) {
        *out = (v != 0);
    }
    return err;
}

/* ============================================================================
 * Load
 * ============================================================================ */

cube32_result_t cube32_cfg_load(void)
{
    /* Start from compiled defaults */
    cfg_apply_defaults(&s_cfg);

    nvs_handle_t h;
    esp_err_t err = nvs_open(CUBE32_CFG_NVS_NAMESPACE, NVS_READONLY, &h);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        /* First boot — no namespace yet. Defaults are fine. */
        ESP_LOGI(TAG, "No saved config — using compiled defaults");
        s_cfg.loaded = true;
        return CUBE32_OK;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        s_cfg.loaded = true;   /* still usable with defaults */
        return CUBE32_ERROR;
    }

    /* --- Display-model change detection ---------------------------------- */
    uint8_t stored_model = 0;
    bool model_changed = false;
    if (nvs_read_u8(h, CUBE32_CFG_KEY_DISP_MODEL, &stored_model) == ESP_OK) {
        uint8_t compiled_model = cube32_cfg_compiled_display_model_id();
        if (stored_model != compiled_model) {
            ESP_LOGW(TAG, "Display model changed (%u → %u) — resetting model-dependent defaults",
                     stored_model, compiled_model);
            model_changed = true;
            /* rotation keeps the new default already set above */
        }
    }

    /* --- Read stored values, overwriting defaults where present ----------- */

    /* Display — skip rotation/prism if model changed (keep new defaults) */
    if (!model_changed) {
        if (nvs_read_u16(h, CUBE32_CFG_KEY_DISP_ROTATION, &s_cfg.display_rotation) == ESP_OK)
            s_cfg.nvs_found |= CUBE32_CFG_NVS_DISP_ROTATION;
        if (nvs_read_bool(h, CUBE32_CFG_KEY_DISP_PRISM, &s_cfg.display_prism) == ESP_OK)
            s_cfg.nvs_found |= CUBE32_CFG_NVS_DISP_PRISM;
    }

    /* Audio */
    if (nvs_read_u8(h, CUBE32_CFG_KEY_AUDIO_VOL, &s_cfg.audio_volume) == ESP_OK)
        s_cfg.nvs_found |= CUBE32_CFG_NVS_AUDIO_VOL;
    if (nvs_read_u8(h, CUBE32_CFG_KEY_AUDIO_MAX_DB, &s_cfg.audio_max_db) == ESP_OK)
        s_cfg.nvs_found |= CUBE32_CFG_NVS_AUDIO_MAX_DB;
    if (nvs_read_u32(h, CUBE32_CFG_KEY_AUDIO_IN_RATE, &s_cfg.audio_input_rate) == ESP_OK)
        s_cfg.nvs_found |= CUBE32_CFG_NVS_AUDIO_IN_RATE;
    if (nvs_read_u32(h, CUBE32_CFG_KEY_AUDIO_OUT_RATE, &s_cfg.audio_output_rate) == ESP_OK)
        s_cfg.nvs_found |= CUBE32_CFG_NVS_AUDIO_OUT_RATE;

    /* RTC */
    if (nvs_read_str(h, CUBE32_CFG_KEY_RTC_TIMEZONE, s_cfg.rtc_timezone, sizeof(s_cfg.rtc_timezone)) == ESP_OK)
        s_cfg.nvs_found |= CUBE32_CFG_NVS_RTC_TZ;
    if (nvs_read_bool(h, CUBE32_CFG_KEY_RTC_NTP_SYNC, &s_cfg.rtc_ntp_sync) == ESP_OK)
        s_cfg.nvs_found |= CUBE32_CFG_NVS_RTC_NTP;

    /* Camera */
    if (nvs_read_bool(h, CUBE32_CFG_KEY_CAM_HMIRROR, &s_cfg.camera_h_mirror) == ESP_OK)
        s_cfg.nvs_found |= CUBE32_CFG_NVS_CAM_HMIRROR;
    if (nvs_read_bool(h, CUBE32_CFG_KEY_CAM_VFLIP, &s_cfg.camera_v_flip) == ESP_OK)
        s_cfg.nvs_found |= CUBE32_CFG_NVS_CAM_VFLIP;

    /* WiFi */
    if (nvs_read_str(h, CUBE32_CFG_KEY_WIFI_SSID, s_cfg.wifi_ssid, sizeof(s_cfg.wifi_ssid)) == ESP_OK)
        s_cfg.nvs_found |= CUBE32_CFG_NVS_WIFI_SSID;
    if (nvs_read_str(h, CUBE32_CFG_KEY_WIFI_PASS, s_cfg.wifi_pass, sizeof(s_cfg.wifi_pass)) == ESP_OK)
        s_cfg.nvs_found |= CUBE32_CFG_NVS_WIFI_PASS;

    /* Modem */
    if (nvs_read_str(h, CUBE32_CFG_KEY_MODEM_APN, s_cfg.modem_apn, sizeof(s_cfg.modem_apn)) == ESP_OK)
        s_cfg.nvs_found |= CUBE32_CFG_NVS_MODEM_APN;

    /* BLE */
    if (nvs_read_str(h, CUBE32_CFG_KEY_BLE_DEV_NAME, s_cfg.ble_device_name, sizeof(s_cfg.ble_device_name)) == ESP_OK)
        s_cfg.nvs_found |= CUBE32_CFG_NVS_BLE_NAME;

    /* Component active flags */
    nvs_read_bool(h, CUBE32_CFG_KEY_ACTIVE_CAMERA, &s_cfg.active_camera);
    nvs_read_bool(h, CUBE32_CFG_KEY_ACTIVE_AUDIO, &s_cfg.active_audio);
    nvs_read_bool(h, CUBE32_CFG_KEY_ACTIVE_MODEM, &s_cfg.active_modem);
    nvs_read_bool(h, CUBE32_CFG_KEY_ACTIVE_SDCARD, &s_cfg.active_sdcard);
    nvs_read_bool(h, CUBE32_CFG_KEY_ACTIVE_USB_IN, &s_cfg.active_usb_input);
    nvs_read_bool(h, CUBE32_CFG_KEY_ACTIVE_BLE_OTA, &s_cfg.active_ble_ota);
    nvs_read_bool(h, CUBE32_CFG_KEY_ACTIVE_ADC_BTN, &s_cfg.active_adc_button);
    nvs_read_bool(h, CUBE32_CFG_KEY_ACTIVE_SERVO, &s_cfg.active_servo);
    nvs_read_bool(h, CUBE32_CFG_KEY_ACTIVE_IMU, &s_cfg.active_imu);
    nvs_read_bool(h, CUBE32_CFG_KEY_ACTIVE_MAG, &s_cfg.active_mag);

    nvs_close(h);

    s_cfg.loaded = true;
    ESP_LOGI(TAG, "Configuration loaded from NVS%s", model_changed ? " (model defaults reset)" : "");

    /* If model changed, persist the new model id + reset values immediately */
    if (model_changed) {
        s_cfg.display_model_id = cube32_cfg_compiled_display_model_id();
        cube32_cfg_save();
    }

    return CUBE32_OK;
}

/* ============================================================================
 * Save
 * ============================================================================ */

cube32_result_t cube32_cfg_save(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(CUBE32_CFG_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open(RW) failed: %s", esp_err_to_name(err));
        return CUBE32_ERROR;
    }

    /* Display */
    nvs_set_u8(h, CUBE32_CFG_KEY_DISP_MODEL, s_cfg.display_model_id);
    nvs_set_u16(h, CUBE32_CFG_KEY_DISP_ROTATION, s_cfg.display_rotation);
    nvs_set_u8(h, CUBE32_CFG_KEY_DISP_PRISM, s_cfg.display_prism ? 1 : 0);

    /* Audio */
    nvs_set_u8(h, CUBE32_CFG_KEY_AUDIO_VOL, s_cfg.audio_volume);
    nvs_set_u8(h, CUBE32_CFG_KEY_AUDIO_MAX_DB, s_cfg.audio_max_db);
    nvs_set_u32(h, CUBE32_CFG_KEY_AUDIO_IN_RATE, s_cfg.audio_input_rate);
    nvs_set_u32(h, CUBE32_CFG_KEY_AUDIO_OUT_RATE, s_cfg.audio_output_rate);

    /* RTC */
    nvs_set_str(h, CUBE32_CFG_KEY_RTC_TIMEZONE, s_cfg.rtc_timezone);
    nvs_set_u8(h, CUBE32_CFG_KEY_RTC_NTP_SYNC, s_cfg.rtc_ntp_sync ? 1 : 0);

    /* Camera */
    nvs_set_u8(h, CUBE32_CFG_KEY_CAM_HMIRROR, s_cfg.camera_h_mirror ? 1 : 0);
    nvs_set_u8(h, CUBE32_CFG_KEY_CAM_VFLIP, s_cfg.camera_v_flip ? 1 : 0);

    /* WiFi */
    nvs_set_str(h, CUBE32_CFG_KEY_WIFI_SSID, s_cfg.wifi_ssid);
    nvs_set_str(h, CUBE32_CFG_KEY_WIFI_PASS, s_cfg.wifi_pass);

    /* Modem */
    nvs_set_str(h, CUBE32_CFG_KEY_MODEM_APN, s_cfg.modem_apn);

    /* BLE */
    nvs_set_str(h, CUBE32_CFG_KEY_BLE_DEV_NAME, s_cfg.ble_device_name);

    /* Component active flags */
    nvs_set_u8(h, CUBE32_CFG_KEY_ACTIVE_CAMERA, s_cfg.active_camera ? 1 : 0);
    nvs_set_u8(h, CUBE32_CFG_KEY_ACTIVE_AUDIO, s_cfg.active_audio ? 1 : 0);
    nvs_set_u8(h, CUBE32_CFG_KEY_ACTIVE_MODEM, s_cfg.active_modem ? 1 : 0);
    nvs_set_u8(h, CUBE32_CFG_KEY_ACTIVE_SDCARD, s_cfg.active_sdcard ? 1 : 0);
    nvs_set_u8(h, CUBE32_CFG_KEY_ACTIVE_USB_IN, s_cfg.active_usb_input ? 1 : 0);
    nvs_set_u8(h, CUBE32_CFG_KEY_ACTIVE_BLE_OTA, s_cfg.active_ble_ota ? 1 : 0);
    nvs_set_u8(h, CUBE32_CFG_KEY_ACTIVE_ADC_BTN, s_cfg.active_adc_button ? 1 : 0);
    nvs_set_u8(h, CUBE32_CFG_KEY_ACTIVE_SERVO, s_cfg.active_servo ? 1 : 0);
    nvs_set_u8(h, CUBE32_CFG_KEY_ACTIVE_IMU, s_cfg.active_imu ? 1 : 0);
    nvs_set_u8(h, CUBE32_CFG_KEY_ACTIVE_MAG, s_cfg.active_mag ? 1 : 0);

    err = nvs_commit(h);
    nvs_close(h);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_commit failed: %s", esp_err_to_name(err));
        return CUBE32_ERROR;
    }

    ESP_LOGI(TAG, "Configuration saved to NVS");
    return CUBE32_OK;
}

/* ============================================================================
 * Reset
 * ============================================================================ */

cube32_result_t cube32_cfg_reset(void)
{
    ESP_LOGW(TAG, "Resetting all configuration to defaults");
    cfg_apply_defaults(&s_cfg);
    s_cfg.loaded = true;

    /* Erase the namespace and re-save defaults */
    nvs_handle_t h;
    esp_err_t err = nvs_open(CUBE32_CFG_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        nvs_erase_all(h);
        nvs_commit(h);
        nvs_close(h);
    }

    return cube32_cfg_save();
}

/* ============================================================================
 * Individual setters — modify RAM, persist, (future: post event)
 * ============================================================================ */

/* Helper: save a single u8 key without full save */
static cube32_result_t save_single_u8(const char* key, uint8_t val)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(CUBE32_CFG_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) return CUBE32_ERROR;
    nvs_set_u8(h, key, val);
    nvs_commit(h);
    nvs_close(h);
    return CUBE32_OK;
}

static cube32_result_t save_single_u16(const char* key, uint16_t val)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(CUBE32_CFG_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) return CUBE32_ERROR;
    nvs_set_u16(h, key, val);
    nvs_commit(h);
    nvs_close(h);
    return CUBE32_OK;
}

static cube32_result_t save_single_u32(const char* key, uint32_t val)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(CUBE32_CFG_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) return CUBE32_ERROR;
    nvs_set_u32(h, key, val);
    nvs_commit(h);
    nvs_close(h);
    return CUBE32_OK;
}

static cube32_result_t save_single_str(const char* key, const char* val)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(CUBE32_CFG_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) return CUBE32_ERROR;
    nvs_set_str(h, key, val);
    nvs_commit(h);
    nvs_close(h);
    return CUBE32_OK;
}

/* --- Display --------------------------------------------------------------- */

cube32_result_t cube32_cfg_set_display_rotation(uint16_t rotation)
{
    if (rotation != 0 && rotation != 90 && rotation != 180 && rotation != 270) {
        return CUBE32_INVALID_ARG;
    }
    s_cfg.display_rotation = rotation;
    return save_single_u16(CUBE32_CFG_KEY_DISP_ROTATION, rotation);
}

cube32_result_t cube32_cfg_set_display_prism(bool enable)
{
    s_cfg.display_prism = enable;
    return save_single_u8(CUBE32_CFG_KEY_DISP_PRISM, enable ? 1 : 0);
}

/* --- Audio ----------------------------------------------------------------- */

cube32_result_t cube32_cfg_set_audio_volume(uint8_t volume)
{
    if (volume > 100) return CUBE32_INVALID_ARG;
    s_cfg.audio_volume = volume;
    return save_single_u8(CUBE32_CFG_KEY_AUDIO_VOL, volume);
}

cube32_result_t cube32_cfg_set_audio_max_db(uint8_t db)
{
    if (db > 32) return CUBE32_INVALID_ARG;
    s_cfg.audio_max_db = db;
    return save_single_u8(CUBE32_CFG_KEY_AUDIO_MAX_DB, db);
}

cube32_result_t cube32_cfg_set_audio_input_rate(uint32_t rate)
{
    s_cfg.audio_input_rate = rate;
    return save_single_u32(CUBE32_CFG_KEY_AUDIO_IN_RATE, rate);
}

cube32_result_t cube32_cfg_set_audio_output_rate(uint32_t rate)
{
    s_cfg.audio_output_rate = rate;
    return save_single_u32(CUBE32_CFG_KEY_AUDIO_OUT_RATE, rate);
}

/* --- RTC / Time ------------------------------------------------------------ */

cube32_result_t cube32_cfg_set_rtc_timezone(const char* tz)
{
    if (!tz) return CUBE32_INVALID_ARG;
    strncpy(s_cfg.rtc_timezone, tz, sizeof(s_cfg.rtc_timezone) - 1);
    s_cfg.rtc_timezone[sizeof(s_cfg.rtc_timezone) - 1] = '\0';
    return save_single_str(CUBE32_CFG_KEY_RTC_TIMEZONE, s_cfg.rtc_timezone);
}

cube32_result_t cube32_cfg_set_rtc_ntp_sync(bool enable)
{
    s_cfg.rtc_ntp_sync = enable;
    return save_single_u8(CUBE32_CFG_KEY_RTC_NTP_SYNC, enable ? 1 : 0);
}

/* --- Camera ---------------------------------------------------------------- */

cube32_result_t cube32_cfg_set_camera_h_mirror(bool enable)
{
    s_cfg.camera_h_mirror = enable;
    return save_single_u8(CUBE32_CFG_KEY_CAM_HMIRROR, enable ? 1 : 0);
}

cube32_result_t cube32_cfg_set_camera_v_flip(bool enable)
{
    s_cfg.camera_v_flip = enable;
    return save_single_u8(CUBE32_CFG_KEY_CAM_VFLIP, enable ? 1 : 0);
}

/* --- WiFi ------------------------------------------------------------------ */

cube32_result_t cube32_cfg_set_wifi(const char* ssid, const char* pass)
{
    if (!ssid || !pass) return CUBE32_INVALID_ARG;

    strncpy(s_cfg.wifi_ssid, ssid, sizeof(s_cfg.wifi_ssid) - 1);
    s_cfg.wifi_ssid[sizeof(s_cfg.wifi_ssid) - 1] = '\0';

    strncpy(s_cfg.wifi_pass, pass, sizeof(s_cfg.wifi_pass) - 1);
    s_cfg.wifi_pass[sizeof(s_cfg.wifi_pass) - 1] = '\0';

    nvs_handle_t h;
    esp_err_t err = nvs_open(CUBE32_CFG_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) return CUBE32_ERROR;
    nvs_set_str(h, CUBE32_CFG_KEY_WIFI_SSID, s_cfg.wifi_ssid);
    nvs_set_str(h, CUBE32_CFG_KEY_WIFI_PASS, s_cfg.wifi_pass);
    nvs_commit(h);
    nvs_close(h);
    return CUBE32_OK;
}

/* --- Modem ----------------------------------------------------------------- */

cube32_result_t cube32_cfg_set_modem_apn(const char* apn)
{
    if (!apn) return CUBE32_INVALID_ARG;
    strncpy(s_cfg.modem_apn, apn, sizeof(s_cfg.modem_apn) - 1);
    s_cfg.modem_apn[sizeof(s_cfg.modem_apn) - 1] = '\0';
    return save_single_str(CUBE32_CFG_KEY_MODEM_APN, s_cfg.modem_apn);
}

/* --- BLE ------------------------------------------------------------------- */

cube32_result_t cube32_cfg_set_ble_device_name(const char* name)
{
    if (!name) return CUBE32_INVALID_ARG;
    strncpy(s_cfg.ble_device_name, name, sizeof(s_cfg.ble_device_name) - 1);
    s_cfg.ble_device_name[sizeof(s_cfg.ble_device_name) - 1] = '\0';
    return save_single_str(CUBE32_CFG_KEY_BLE_DEV_NAME, s_cfg.ble_device_name);
}

/* --- Active flags ---------------------------------------------------------- */

cube32_result_t cube32_cfg_set_active_camera(bool active)
{
    s_cfg.active_camera = active;
    return save_single_u8(CUBE32_CFG_KEY_ACTIVE_CAMERA, active ? 1 : 0);
}

cube32_result_t cube32_cfg_set_active_audio(bool active)
{
    s_cfg.active_audio = active;
    return save_single_u8(CUBE32_CFG_KEY_ACTIVE_AUDIO, active ? 1 : 0);
}

cube32_result_t cube32_cfg_set_active_modem(bool active)
{
    s_cfg.active_modem = active;
    return save_single_u8(CUBE32_CFG_KEY_ACTIVE_MODEM, active ? 1 : 0);
}

cube32_result_t cube32_cfg_set_active_sdcard(bool active)
{
    s_cfg.active_sdcard = active;
    return save_single_u8(CUBE32_CFG_KEY_ACTIVE_SDCARD, active ? 1 : 0);
}

cube32_result_t cube32_cfg_set_active_usb_input(bool active)
{
    s_cfg.active_usb_input = active;
    return save_single_u8(CUBE32_CFG_KEY_ACTIVE_USB_IN, active ? 1 : 0);
}

cube32_result_t cube32_cfg_set_active_ble_ota(bool active)
{
    s_cfg.active_ble_ota = active;
    return save_single_u8(CUBE32_CFG_KEY_ACTIVE_BLE_OTA, active ? 1 : 0);
}

cube32_result_t cube32_cfg_set_active_adc_button(bool active)
{
    s_cfg.active_adc_button = active;
    return save_single_u8(CUBE32_CFG_KEY_ACTIVE_ADC_BTN, active ? 1 : 0);
}

cube32_result_t cube32_cfg_set_active_servo(bool active)
{
    s_cfg.active_servo = active;
    return save_single_u8(CUBE32_CFG_KEY_ACTIVE_SERVO, active ? 1 : 0);
}

cube32_result_t cube32_cfg_set_active_imu(bool active)
{
    s_cfg.active_imu = active;
    return save_single_u8(CUBE32_CFG_KEY_ACTIVE_IMU, active ? 1 : 0);
}

cube32_result_t cube32_cfg_set_active_mag(bool active)
{
    s_cfg.active_mag = active;
    return save_single_u8(CUBE32_CFG_KEY_ACTIVE_MAG, active ? 1 : 0);
}

/* ============================================================================
 * Debug print
 * ============================================================================ */

void cube32_cfg_print(const cube32_cfg_t* cfg)
{
    if (!cfg) cfg = &s_cfg;

    /* Helper: returns "" if bit is set in nvs_found (user-configured),
     * otherwise " (default)" to indicate compiled default in use */
    #define NVS_TAG(bit) ((cfg->nvs_found & (bit)) ? "" : " (default)")

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "CUBE32 Runtime Configuration");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Display: model=%u  rotation=%u%s  prism=%s%s",
             cfg->display_model_id, cfg->display_rotation,
             NVS_TAG(CUBE32_CFG_NVS_DISP_ROTATION),
             cfg->display_prism ? "ON" : "OFF",
             NVS_TAG(CUBE32_CFG_NVS_DISP_PRISM));
    ESP_LOGI(TAG, "Audio:   vol=%u%s  max_db=%u%s  in=%luHz%s  out=%luHz%s",
             cfg->audio_volume, NVS_TAG(CUBE32_CFG_NVS_AUDIO_VOL),
             cfg->audio_max_db, NVS_TAG(CUBE32_CFG_NVS_AUDIO_MAX_DB),
             (unsigned long)cfg->audio_input_rate, NVS_TAG(CUBE32_CFG_NVS_AUDIO_IN_RATE),
             (unsigned long)cfg->audio_output_rate, NVS_TAG(CUBE32_CFG_NVS_AUDIO_OUT_RATE));
    ESP_LOGI(TAG, "RTC:     tz=%s%s  ntp_sync=%s%s",
             cfg->rtc_timezone, NVS_TAG(CUBE32_CFG_NVS_RTC_TZ),
             cfg->rtc_ntp_sync ? "ON" : "OFF", NVS_TAG(CUBE32_CFG_NVS_RTC_NTP));
    ESP_LOGI(TAG, "Camera:  h_mirror=%s%s  v_flip=%s%s",
             cfg->camera_h_mirror ? "ON" : "OFF", NVS_TAG(CUBE32_CFG_NVS_CAM_HMIRROR),
             cfg->camera_v_flip ? "ON" : "OFF", NVS_TAG(CUBE32_CFG_NVS_CAM_VFLIP));
    ESP_LOGI(TAG, "WiFi:    ssid=%s%s",
             cfg->wifi_ssid[0] ? cfg->wifi_ssid : "(not set)",
             (cfg->nvs_found & CUBE32_CFG_NVS_WIFI_SSID) ? "" : "");
    ESP_LOGI(TAG, "Modem:   apn=%s%s",
             cfg->modem_apn, NVS_TAG(CUBE32_CFG_NVS_MODEM_APN));
    ESP_LOGI(TAG, "BLE:     name=%s%s",
             cfg->ble_device_name, NVS_TAG(CUBE32_CFG_NVS_BLE_NAME));
    ESP_LOGI(TAG, "Active:  cam=%d aud=%d modem=%d sd=%d usb=%d ble=%d btn=%d servo=%d imu=%d mag=%d",
             cfg->active_camera, cfg->active_audio, cfg->active_modem,
             cfg->active_sdcard, cfg->active_usb_input, cfg->active_ble_ota,
             cfg->active_adc_button, cfg->active_servo,
             cfg->active_imu, cfg->active_mag);
    ESP_LOGI(TAG, "========================================");

    #undef NVS_TAG
}
