/**
 * @file hw_manifest.h
 * @brief CUBE32 Hardware Manifest — Boot-time hardware detection & board profiling
 *
 * The hardware manifest is populated at boot by scanning the I2C bus and
 * probing other peripherals.  It provides three tiers of information:
 *
 *   1. Board Profile  — identifies the core-module variant by I2C fingerprint
 *   2. Hardware Present flags — per-component booleans + detected I2C addresses
 *   3. Active/Inactive flags — runtime state loaded from NVS, controllable
 *      via the mobile app through BLE configuration service
 *
 * Usage:
 *   cube32_hw_manifest_t* m = cube32_hw_manifest();   // singleton
 *   cube32_hw_manifest_scan(m);                        // run once at boot
 *   if (m->pmu_present) { ... }
 */

#ifndef CUBE32_UTILS_HW_MANIFEST_H
#define CUBE32_UTILS_HW_MANIFEST_H

#include <stdint.h>
#include <stdbool.h>
#include "common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Known I2C Addresses (current Core V1 module)
 * ============================================================================ */

#define CUBE32_I2C_ADDR_AXP2101         0x34    ///< PMU
#define CUBE32_I2C_ADDR_BM8563          0x51    ///< RTC
#define CUBE32_I2C_ADDR_CST816S         0x15    ///< Touch (capacitive)
#define CUBE32_I2C_ADDR_FT6336          0x38    ///< Touch (capacitive, alt IC)
#define CUBE32_I2C_ADDR_ES8311          0x18    ///< Audio DAC
#define CUBE32_I2C_ADDR_ES7210          0x40    ///< Audio ADC (microphone)
#define CUBE32_I2C_ADDR_TCA9554_AUDIO   0x20    ///< Audio module IO expander
#define CUBE32_I2C_ADDR_TCA9554_MODEM   0x22    ///< Modem module IO expander
#define CUBE32_I2C_ADDR_LSM6DSO         0x6A    ///< 6-axis IMU (accel + gyro)
#define CUBE32_I2C_ADDR_LIS3MDL         0x1C    ///< 3-axis magnetometer

/* ============================================================================
 * Board Profiles
 *
 * Each profile maps to a unique combination of I2C addresses for the
 * system-core chips (PMU + RTC).  Future core modules will use different
 * address pairs so that the profile can be determined automatically.
 * ============================================================================ */

typedef enum {
    CUBE32_PROFILE_UNKNOWN = 0,

    /**
     * Core V1 — ESP32-S3 full-featured core module.
     * PMU: AXP2101 @ 0x34   RTC: BM8563 @ 0x51
     * Supports: Display, Touch, Camera, SD Card, USB Input
     */
    CUBE32_PROFILE_CORE_V1,

    /**
     * Core Audio V1 — ESP32-S3 + Audio module, no SD card slot.
     * PMU & RTC at different addresses (TBD — placeholder for future HW).
     */
    CUBE32_PROFILE_CORE_AUDIO_V1,

    /* Add future profiles here */

    CUBE32_PROFILE_COUNT,           ///< Number of defined profiles (sentinel)
} cube32_board_profile_t;

/* ============================================================================
 * Module presence flags (detected at boot via probing)
 * ============================================================================ */

/*
 * cube32_touch_ic_t is defined in drivers/touch/touch.h
 * (CUBE32_TOUCH_IC_NONE, CUBE32_TOUCH_IC_CST816, CUBE32_TOUCH_IC_FT6336)
 * — included via cube32.h.  No duplicate definition here.
 */

/* ============================================================================
 * Component categories
 *
 * SYSTEM_CORE  — always compiled, never user-disablable (PMU, RTC)
 * CORE_PERIPH  — build-gated, auto-detected, not user-disablable
 * OPTIONAL     — build-gated, auto-detected, user can activate/deactivate
 * SOFTWARE     — build-gated, no hardware, user can activate/deactivate
 * ============================================================================ */

typedef enum {
    CUBE32_COMP_CAT_SYSTEM_CORE = 0,
    CUBE32_COMP_CAT_CORE_PERIPH,
    CUBE32_COMP_CAT_OPTIONAL,
    CUBE32_COMP_CAT_SOFTWARE,
} cube32_component_category_t;

/* ============================================================================
 * Hardware Manifest
 * ============================================================================ */

typedef struct {
    /* ---- Board Identity ------------------------------------------------- */
    cube32_board_profile_t  profile;        ///< Detected board profile

    /* ---- System Core (always compiled) ---------------------------------- */
    bool        pmu_present;                ///< PMU chip found on I2C
    uint8_t     pmu_i2c_addr;               ///< Detected PMU I2C address (0 = absent)

    bool        rtc_present;                ///< RTC chip found on I2C
    uint8_t     rtc_i2c_addr;               ///< Detected RTC I2C address (0 = absent)

    /* ---- Core Peripherals (build-gated, auto-detected) ------------------- */
    bool        display_present;            ///< Display initialised successfully
    bool        touch_present;              ///< Touch IC found on I2C
    uint8_t     touch_ic;                   ///< Detected touch IC type (cube32_touch_ic_t from touch.h)
    uint8_t     touch_i2c_addr;             ///< Detected touch I2C address (0 = absent)

    /* ---- Module Detection (build-gated, I2C fingerprint) ----------------- */
    bool        audio_module_present;       ///< Audio codec pair detected (ES8311+ES7210)
    bool        audio_dac_present;          ///< ES8311 found at 0x18
    bool        audio_adc_present;          ///< ES7210 found at 0x40

    bool        modem_module_present;       ///< TCA9554 found at 0x22 → modem module installed
    bool        camera_present;             ///< Camera probed successfully
    char        camera_sensor_name[16];     ///< Detected sensor model (e.g. "OV2640"), set after Camera::begin()
    bool        sdcard_present;             ///< SD card mounted successfully

    bool        imu_present;               ///< LSM6DSO found on I2C
    uint8_t     imu_i2c_addr;              ///< Detected IMU I2C address (0 = absent)
    bool        mag_present;               ///< LIS3MDL found on I2C
    uint8_t     mag_i2c_addr;              ///< Detected magnetometer I2C address (0 = absent)

    /* ---- Kconfig Build Flags (compiled-in, immutable at runtime) --------- */
    bool        pmu_built;                  ///< CONFIG_CUBE32_PMU_ENABLED
    bool        rtc_built;                  ///< CONFIG_CUBE32_RTC_ENABLED
    bool        display_built;              ///< CONFIG_CUBE32_DISPLAY_ENABLED
    bool        touch_built;                ///< CONFIG_CUBE32_TOUCH_ENABLED
    bool        audio_built;                ///< CONFIG_CUBE32_AUDIO_ENABLED
    bool        camera_built;               ///< CONFIG_CUBE32_CAMERA_ENABLED
    bool        sdcard_built;               ///< CONFIG_CUBE32_SDCARD_ENABLED
    bool        modem_built;                ///< CONFIG_CUBE32_MODEM_ENABLED
    bool        ble_ota_built;              ///< CONFIG_CUBE32_BLE_OTA_ENABLED
    bool        usb_input_built;            ///< CONFIG_CUBE32_USB_INPUT_ENABLED
    bool        adc_button_built;           ///< CONFIG_CUBE32_ADC_BUTTON_ENABLED
    bool        servo_built;                ///< CONFIG_CUBE32_SERVO_ENABLED
    bool        imu_built;                  ///< CONFIG_CUBE32_IMU_ENABLED
    bool        mag_built;                  ///< CONFIG_CUBE32_MAG_ENABLED
    bool        lvgl_built;                 ///< CONFIG_CUBE32_LVGL_ENABLED

    /* ---- Raw I2C scan results ------------------------------------------- */
    uint8_t     i2c_devices[16];            ///< Addresses found during scan (0-terminated)
    uint8_t     i2c_device_count;           ///< Number of I2C devices found

    /* ---- Runtime Active State (loaded from NVS) ------------------------- */
    bool        camera_active;
    bool        audio_active;
    bool        modem_active;
    bool        sdcard_active;
    bool        usb_input_active;
    bool        ble_ota_active;
    bool        adc_button_active;
    bool        servo_active;
    bool        imu_active;
    bool        mag_active;

    /* ---- Scan metadata -------------------------------------------------- */
    bool        scanned;                    ///< true after scan has been performed

    /* ---- Driver Init Status (populated during cube32_begin) -------------- */
    cube32_init_record_t init_records[CUBE32_DRV_COUNT];  ///< Per-driver init results
    bool        init_done;                  ///< true after all drivers have been initialized
} cube32_hw_manifest_t;

/* ============================================================================
 * Public API (C linkage)
 * ============================================================================ */

/**
 * @brief Get the global hardware manifest (singleton).
 *
 * The manifest is zero-initialised on first call.
 */
cube32_hw_manifest_t* cube32_hw_manifest(void);

/**
 * @brief Run hardware detection and populate the manifest.
 *
 * Call once, after I2C bus is initialised but before individual drivers.
 * - Scans I2C bus for known addresses
 * - Determines board profile from PMU+RTC address combo
 * - Sets all `*_present` flags
 * - Loads `*_active` states from NVS (falls back to true for defaults)
 *
 * @param[out] manifest  Pointer returned by cube32_hw_manifest().
 * @return CUBE32_OK on success
 */
cube32_result_t cube32_hw_manifest_scan(cube32_hw_manifest_t* manifest);

/**
 * @brief Log the full manifest contents at INFO level.
 */
void cube32_hw_manifest_print(const cube32_hw_manifest_t* manifest);

/**
 * @brief Log the post-initialization driver status summary at INFO level.
 *
 * Similar to cube32_hw_manifest_print() but includes the Init column
 * and resolves "pend" entries to their actual status.  Call after all
 * drivers have been initialized (at the end of cube32_begin).
 */
void cube32_hw_manifest_print_init_status(const cube32_hw_manifest_t* manifest);

/**
 * @brief Get a human-readable short label for an init code.
 *
 * Returns a 4-character label suitable for the status table:
 *   OK, FAIL, INIT, SKIP, OFF, N/B, --
 */
const char* cube32_init_code_name(cube32_init_code_t code);

/**
 * @brief Get the human-readable name for a driver index.
 */
const char* cube32_driver_name(cube32_driver_index_t index);

/**
 * @brief Get a human-readable name for a board profile.
 */
const char* cube32_profile_name(cube32_board_profile_t profile);

#ifdef __cplusplus
}
#endif

#endif /* CUBE32_UTILS_HW_MANIFEST_H */
