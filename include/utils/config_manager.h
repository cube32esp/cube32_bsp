/**
 * @file config_manager.h
 * @brief CUBE32 NVS Configuration Manager
 *
 * Centralised runtime configuration stored in NVS flash.  Values are loaded
 * once at boot with compiled defaults as fallback.  When a configuration
 * value is changed (e.g. from the mobile app via BLE), it is immediately
 * persisted and an optional event is posted so drivers can hot-reconfigure.
 *
 * Key design rules:
 *   - Every setting has a compiled default that is used when NVS is empty.
 *   - When the display-model Kconfig changes, model-dependent defaults
 *     (e.g. rotation) are silently reset so the user doesn't get a stale
 *     rotation from a previous display model.
 *   - Component active/inactive flags are stored here and also mirrored
 *     into the hardware manifest for convenience.
 *
 * NVS namespace: "cube32_cfg"
 */

#ifndef CUBE32_UTILS_CONFIG_MANAGER_H
#define CUBE32_UTILS_CONFIG_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * NVS namespace & key definitions
 * ============================================================================ */

#define CUBE32_CFG_NVS_NAMESPACE    "cube32_cfg"

/* --- Display --------------------------------------------------------------- */
#define CUBE32_CFG_KEY_DISP_MODEL       "d_model"       ///< uint8  — last compiled display model id
#define CUBE32_CFG_KEY_DISP_ROTATION    "d_rotation"    ///< uint16 — display rotation (0/90/180/270)
#define CUBE32_CFG_KEY_DISP_PRISM       "d_prism"       ///< uint8  — prism/mirror mode (0/1)

/* --- Audio ----------------------------------------------------------------- */
#define CUBE32_CFG_KEY_AUDIO_VOL        "a_vol"         ///< uint8  — output volume (0-100)
#define CUBE32_CFG_KEY_AUDIO_MAX_DB     "a_max_db"      ///< uint8  — max volume gain dB
#define CUBE32_CFG_KEY_AUDIO_IN_RATE    "a_in_rate"     ///< uint32 — input sample rate Hz
#define CUBE32_CFG_KEY_AUDIO_OUT_RATE   "a_out_rate"    ///< uint32 — output sample rate Hz

/* --- RTC / Time ------------------------------------------------------------ */
#define CUBE32_CFG_KEY_RTC_TIMEZONE     "r_tz"          ///< string — POSIX TZ string
#define CUBE32_CFG_KEY_RTC_NTP_SYNC     "r_ntp_sync"    ///< uint8  — sync on boot (0/1)

/* --- Camera ---------------------------------------------------------------- */
#define CUBE32_CFG_KEY_CAM_HMIRROR      "c_hmirror"     ///< uint8  — horizontal mirror (0/1)
#define CUBE32_CFG_KEY_CAM_VFLIP        "c_vflip"       ///< uint8  — vertical flip (0/1)

/* --- WiFi ------------------------------------------------------------------ */
#define CUBE32_CFG_KEY_WIFI_SSID        "w_ssid"        ///< string — WiFi SSID
#define CUBE32_CFG_KEY_WIFI_PASS        "w_pass"        ///< string — WiFi password

/* --- Modem ----------------------------------------------------------------- */
#define CUBE32_CFG_KEY_MODEM_APN        "m_apn"         ///< string — cellular APN

/* --- BLE OTA --------------------------------------------------------------- */
#define CUBE32_CFG_KEY_BLE_DEV_NAME     "b_dev_name"    ///< string — BLE advertised device name

/* --- Component active flags ------------------------------------------------ */
#define CUBE32_CFG_KEY_ACTIVE_CAMERA    "act_camera"    ///< uint8
#define CUBE32_CFG_KEY_ACTIVE_AUDIO     "act_audio"     ///< uint8
#define CUBE32_CFG_KEY_ACTIVE_MODEM     "act_modem"     ///< uint8
#define CUBE32_CFG_KEY_ACTIVE_SDCARD    "act_sdcard"    ///< uint8
#define CUBE32_CFG_KEY_ACTIVE_USB_IN    "act_usb_in"    ///< uint8
#define CUBE32_CFG_KEY_ACTIVE_BLE_OTA   "act_ble_ota"   ///< uint8
#define CUBE32_CFG_KEY_ACTIVE_ADC_BTN   "act_adc_btn"   ///< uint8
#define CUBE32_CFG_KEY_ACTIVE_SERVO     "act_servo"     ///< uint8
#define CUBE32_CFG_KEY_ACTIVE_IMU       "act_imu"       ///< uint8
#define CUBE32_CFG_KEY_ACTIVE_MAG       "act_mag"       ///< uint8

/* ============================================================================
 * Configuration data structure — mirrors NVS contents in RAM
 * ============================================================================ */

#define CUBE32_CFG_STR_MAX  64      ///< Max length for string config values

typedef struct {
    /* Display */
    uint8_t     display_model_id;       ///< Tracks Kconfig model for change detection
    uint16_t    display_rotation;       ///< 0, 90, 180, 270
    bool        display_prism;          ///< Horizontal mirror / prism mode

    /* Audio */
    uint8_t     audio_volume;           ///< 0–100
    uint8_t     audio_max_db;           ///< Max gain dB (0–32)
    uint32_t    audio_input_rate;       ///< Input sample rate Hz
    uint32_t    audio_output_rate;      ///< Output sample rate Hz

    /* RTC / Time */
    char        rtc_timezone[CUBE32_CFG_STR_MAX];   ///< POSIX TZ string
    bool        rtc_ntp_sync;                        ///< Sync NTP on boot

    /* Camera */
    bool        camera_h_mirror;
    bool        camera_v_flip;

    /* WiFi */
    char        wifi_ssid[CUBE32_CFG_STR_MAX];
    char        wifi_pass[CUBE32_CFG_STR_MAX];

    /* Modem */
    char        modem_apn[CUBE32_CFG_STR_MAX];

    /* BLE OTA */
    char        ble_device_name[32];

    /* Component active flags */
    bool        active_camera;
    bool        active_audio;
    bool        active_modem;
    bool        active_sdcard;
    bool        active_usb_input;
    bool        active_ble_ota;
    bool        active_adc_button;
    bool        active_servo;
    bool        active_imu;
    bool        active_mag;

    /* Metadata */
    bool        loaded;                 ///< true after load() succeeds
    uint32_t    nvs_found;              ///< Bitmask: bit set = value was read from NVS (not default)
} cube32_cfg_t;

/* Bit definitions for cube32_cfg_t::nvs_found */
#define CUBE32_CFG_NVS_DISP_ROTATION   (1u <<  0)
#define CUBE32_CFG_NVS_DISP_PRISM      (1u <<  1)
#define CUBE32_CFG_NVS_AUDIO_VOL       (1u <<  2)
#define CUBE32_CFG_NVS_AUDIO_MAX_DB    (1u <<  3)
#define CUBE32_CFG_NVS_AUDIO_IN_RATE   (1u <<  4)
#define CUBE32_CFG_NVS_AUDIO_OUT_RATE  (1u <<  5)
#define CUBE32_CFG_NVS_RTC_TZ          (1u <<  6)
#define CUBE32_CFG_NVS_RTC_NTP         (1u <<  7)
#define CUBE32_CFG_NVS_CAM_HMIRROR     (1u <<  8)
#define CUBE32_CFG_NVS_CAM_VFLIP       (1u <<  9)
#define CUBE32_CFG_NVS_WIFI_SSID       (1u << 10)
#define CUBE32_CFG_NVS_WIFI_PASS       (1u << 11)
#define CUBE32_CFG_NVS_MODEM_APN       (1u << 12)
#define CUBE32_CFG_NVS_BLE_NAME        (1u << 13)

/* ============================================================================
 * Compiled defaults — derived from Kconfig where available
 *
 * These provide the initial value when NVS has no stored data.
 * ============================================================================ */

/**
 * @brief Get the default display rotation for a given display model.
 *
 * When the compiled display model differs from the model stored in NVS,
 * the rotation is reset to this default.
 */
uint16_t cube32_cfg_default_rotation(uint8_t display_model_id);

/**
 * @brief Get a unique model ID for the currently compiled display model.
 *
 * Uses Kconfig choices to return a stable integer.
 */
uint8_t cube32_cfg_compiled_display_model_id(void);

/* ============================================================================
 * Public API
 * ============================================================================ */

/**
 * @brief Get the global config singleton (read-only snapshot after load).
 */
const cube32_cfg_t* cube32_cfg(void);

/**
 * @brief Get a mutable pointer (internal use / config manager writes).
 */
cube32_cfg_t* cube32_cfg_mut(void);

/**
 * @brief Load configuration from NVS, applying compiled defaults where
 *        no stored value exists.
 *
 * Handles display-model change detection: if the compiled model differs
 * from the NVS-stored model ID, model-dependent settings are reset.
 *
 * Call once, after NVS flash has been initialised.
 *
 * @return CUBE32_OK on success.
 */
cube32_result_t cube32_cfg_load(void);

/**
 * @brief Persist the current in-RAM config back to NVS.
 *
 * Typically called after a setter modifies one or more values.
 *
 * @return CUBE32_OK on success.
 */
cube32_result_t cube32_cfg_save(void);

/**
 * @brief Reset all configuration to compiled defaults and persist.
 */
cube32_result_t cube32_cfg_reset(void);

/* ---- Convenience setters (modify RAM + persist + post event) -------------- */

cube32_result_t cube32_cfg_set_display_rotation(uint16_t rotation);
cube32_result_t cube32_cfg_set_display_prism(bool enable);
cube32_result_t cube32_cfg_set_audio_volume(uint8_t volume);
cube32_result_t cube32_cfg_set_audio_max_db(uint8_t db);
cube32_result_t cube32_cfg_set_audio_input_rate(uint32_t rate);
cube32_result_t cube32_cfg_set_audio_output_rate(uint32_t rate);
cube32_result_t cube32_cfg_set_rtc_timezone(const char* tz);
cube32_result_t cube32_cfg_set_rtc_ntp_sync(bool enable);
cube32_result_t cube32_cfg_set_camera_h_mirror(bool enable);
cube32_result_t cube32_cfg_set_camera_v_flip(bool enable);
cube32_result_t cube32_cfg_set_wifi(const char* ssid, const char* pass);
cube32_result_t cube32_cfg_set_modem_apn(const char* apn);
cube32_result_t cube32_cfg_set_ble_device_name(const char* name);

/* Active-flag setters */
cube32_result_t cube32_cfg_set_active_camera(bool active);
cube32_result_t cube32_cfg_set_active_audio(bool active);
cube32_result_t cube32_cfg_set_active_modem(bool active);
cube32_result_t cube32_cfg_set_active_sdcard(bool active);
cube32_result_t cube32_cfg_set_active_usb_input(bool active);
cube32_result_t cube32_cfg_set_active_ble_ota(bool active);
cube32_result_t cube32_cfg_set_active_adc_button(bool active);
cube32_result_t cube32_cfg_set_active_servo(bool active);
cube32_result_t cube32_cfg_set_active_imu(bool active);
cube32_result_t cube32_cfg_set_active_mag(bool active);

/**
 * @brief Log the full configuration at INFO level.
 */
void cube32_cfg_print(const cube32_cfg_t* cfg);

#ifdef __cplusplus
}
#endif

#endif /* CUBE32_UTILS_CONFIG_MANAGER_H */
