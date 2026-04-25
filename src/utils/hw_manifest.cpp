/**
 * @file hw_manifest.cpp
 * @brief CUBE32 Hardware Manifest — Implementation
 *
 * Scans the I2C bus after initialisation and populates the hardware
 * manifest with detected devices, board profile, and active states
 * loaded from the NVS config manager.
 */

#include "utils/hw_manifest.h"
#include "utils/i2c_bus.h"
#include "utils/config_manager.h"
#include "drivers/touch/touch.h"

#include <cstring>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* TAG = "cube32_hw";

/* ============================================================================
 * Singleton
 * ============================================================================ */

static cube32_hw_manifest_t s_manifest = {};

cube32_hw_manifest_t* cube32_hw_manifest(void)
{
    return &s_manifest;
}

/* ============================================================================
 * Profile name helper
 * ============================================================================ */

const char* cube32_profile_name(cube32_board_profile_t profile)
{
    switch (profile) {
        case CUBE32_PROFILE_CORE_V1:        return "Core V1 (Full)";
        case CUBE32_PROFILE_CORE_AUDIO_V1:  return "Core Audio V1";
        default:                            return "Unknown";
    }
}

/* ============================================================================
 * Board profile detection
 *
 * Matches the combination of PMU + RTC I2C addresses to a known profile.
 * ============================================================================ */

static cube32_board_profile_t detect_profile(uint8_t pmu_addr, uint8_t rtc_addr)
{
    /* Core V1: AXP2101 @ 0x34 + BM8563 @ 0x51 */
    if (pmu_addr == CUBE32_I2C_ADDR_AXP2101 && rtc_addr == CUBE32_I2C_ADDR_BM8563) {
        return CUBE32_PROFILE_CORE_V1;
    }

    /*
     * Future profiles:
     *   CORE_AUDIO_V1 — different PMU/RTC address pair (TBD)
     *   if (pmu_addr == 0xNN && rtc_addr == 0xMM)
     *       return CUBE32_PROFILE_CORE_AUDIO_V1;
     */

    return CUBE32_PROFILE_UNKNOWN;
}

/* ============================================================================
 * I2C probe helper (wraps the I2C bus singleton)
 * ============================================================================ */

static bool i2c_probe(uint8_t addr)
{
    return cube32_i2c_probe(addr) == CUBE32_OK;
}

/* ============================================================================
 * Scan & Populate
 * ============================================================================ */

cube32_result_t cube32_hw_manifest_scan(cube32_hw_manifest_t* m)
{
    if (!m) return CUBE32_INVALID_ARG;

    memset(m, 0, sizeof(*m));

    ESP_LOGI(TAG, "Scanning hardware...");

    /* ------------------------------------------------------------------
     * 1. Power-on settling delay
     * LSM6DSOX and LIS3MDL require ~20ms after Vdd to complete internal
     * POR before responding safely to I2C. Probing them too early can
     * leave SDA stuck low and corrupt subsequent I2C transactions
     * (including ES7210/ES8311 codec init).
     * ------------------------------------------------------------------ */
    vTaskDelay(pdMS_TO_TICKS(50));

    /* ------------------------------------------------------------------
     * 2. I2C bus scan — collect all present addresses
     * ------------------------------------------------------------------ */
    uint8_t count = 0;
    for (uint8_t addr = 0x08; addr < 0x78 && count < sizeof(m->i2c_devices); addr++) {
        if (i2c_probe(addr)) {
            m->i2c_devices[count++] = addr;
            ESP_LOGI(TAG, "  I2C device at 0x%02X", addr);
        }
    }
    m->i2c_device_count = count;
    ESP_LOGI(TAG, "I2C scan: %u device(s) found", count);

    /* Helper lambda: check if addr was found */
    auto found = [&](uint8_t addr) -> bool {
        for (uint8_t i = 0; i < m->i2c_device_count; i++) {
            if (m->i2c_devices[i] == addr) return true;
        }
        return false;
    };

    /* ------------------------------------------------------------------
     * 3. System Core — PMU & RTC
     * ------------------------------------------------------------------ */

    /* Try known PMU addresses */
    if (found(CUBE32_I2C_ADDR_AXP2101)) {
        m->pmu_present  = true;
        m->pmu_i2c_addr = CUBE32_I2C_ADDR_AXP2101;
    }
    /* Future: else if (found(0xNN)) { ... CORE_AUDIO_V1 PMU ... } */

    /* Try known RTC addresses */
    if (found(CUBE32_I2C_ADDR_BM8563)) {
        m->rtc_present  = true;
        m->rtc_i2c_addr = CUBE32_I2C_ADDR_BM8563;
    }
    /* Future: else if (found(0xMM)) { ... other RTC ... } */

    /* ------------------------------------------------------------------
     * 3. Board Profile
     * ------------------------------------------------------------------ */
    m->profile = detect_profile(m->pmu_i2c_addr, m->rtc_i2c_addr);
    ESP_LOGI(TAG, "Board profile: %s", cube32_profile_name(m->profile));

    /* ------------------------------------------------------------------
     * 4. Core Peripherals — Touch
     * ------------------------------------------------------------------ */
    if (found(CUBE32_I2C_ADDR_CST816S)) {
        m->touch_present  = true;
        m->touch_ic       = CUBE32_TOUCH_IC_CST816;
        m->touch_i2c_addr = CUBE32_I2C_ADDR_CST816S;
    } else if (found(CUBE32_I2C_ADDR_FT6336)) {
        m->touch_present  = true;
        m->touch_ic       = CUBE32_TOUCH_IC_FT6336;
        m->touch_i2c_addr = CUBE32_I2C_ADDR_FT6336;
    }

    /* Display — we can't probe SPI here. Mark as unknown;
     * the display driver's begin() will set this later. */
    m->display_present = false;

    /* ------------------------------------------------------------------
     * 5. Module Detection
     * ------------------------------------------------------------------ */

    /* Audio module: both DAC + ADC must be present */
    m->audio_dac_present = found(CUBE32_I2C_ADDR_ES8311);
    m->audio_adc_present = found(CUBE32_I2C_ADDR_ES7210);
    m->audio_module_present = m->audio_dac_present && m->audio_adc_present;

    /* Modem module: TCA9554 at 0x22 */
    m->modem_module_present = found(CUBE32_I2C_ADDR_TCA9554_MODEM);

    /* Camera & SD card cannot be probed via I2C; left false.
     * The respective drivers will update these after begin(). */
    m->camera_present = false;
    m->sdcard_present = false;

    /* IMU: LSM6DSO */
    if (found(CUBE32_I2C_ADDR_LSM6DSO)) {
        m->imu_present  = true;
        m->imu_i2c_addr = CUBE32_I2C_ADDR_LSM6DSO;
    }

    /* Magnetometer: LIS3MDL */
    if (found(CUBE32_I2C_ADDR_LIS3MDL)) {
        m->mag_present  = true;
        m->mag_i2c_addr = CUBE32_I2C_ADDR_LIS3MDL;
    }

    /* ------------------------------------------------------------------
     * 6. Load active states from NVS config manager
     * ------------------------------------------------------------------ */
    const cube32_cfg_t* cfg = cube32_cfg();
    if (cfg && cfg->loaded) {
        m->camera_active     = cfg->active_camera;
        m->audio_active      = cfg->active_audio;
        m->modem_active      = cfg->active_modem;
        m->sdcard_active     = cfg->active_sdcard;
        m->usb_input_active  = cfg->active_usb_input;
        m->ble_ota_active    = cfg->active_ble_ota;
        m->adc_button_active = cfg->active_adc_button;
        m->servo_active      = cfg->active_servo;
        m->imu_active        = cfg->active_imu;
        m->mag_active        = cfg->active_mag;
    } else {
        /* Config not loaded yet — default everything to active */
        m->camera_active     = true;
        m->audio_active      = true;
        m->modem_active      = true;
        m->sdcard_active     = true;
        m->usb_input_active  = true;
        m->ble_ota_active    = true;
        m->adc_button_active = true;
        m->servo_active      = true;
        m->imu_active        = true;
        m->mag_active        = true;
    }

    /* ------------------------------------------------------------------
     * 7. Kconfig build flags (compile-time constants)
     * ------------------------------------------------------------------ */
#ifdef CONFIG_CUBE32_PMU_ENABLED
    m->pmu_built = true;
#endif
#ifdef CONFIG_CUBE32_RTC_ENABLED
    m->rtc_built = true;
#endif
#ifdef CONFIG_CUBE32_DISPLAY_ENABLED
    m->display_built = true;
#endif
#ifdef CONFIG_CUBE32_TOUCH_ENABLED
    m->touch_built = true;
#endif
#ifdef CONFIG_CUBE32_AUDIO_ENABLED
    m->audio_built = true;
#endif
#ifdef CONFIG_CUBE32_CAMERA_ENABLED
    m->camera_built = true;
#endif
#ifdef CONFIG_CUBE32_SDCARD_ENABLED
    m->sdcard_built = true;
#endif
#ifdef CONFIG_CUBE32_MODEM_ENABLED
    m->modem_built = true;
#endif
#ifdef CONFIG_CUBE32_BLE_OTA_ENABLED
    m->ble_ota_built = true;
#endif
#ifdef CONFIG_CUBE32_USB_INPUT_ENABLED
    m->usb_input_built = true;
#endif
#ifdef CONFIG_CUBE32_ADC_BUTTON_ENABLED
    m->adc_button_built = true;
#endif
#ifdef CONFIG_CUBE32_SERVO_ENABLED
    m->servo_built = true;
#endif
#ifdef CONFIG_CUBE32_IMU_ENABLED
    m->imu_built = true;
#endif
#ifdef CONFIG_CUBE32_MAG_ENABLED
    m->mag_built = true;
#endif
#ifdef CONFIG_CUBE32_LVGL_ENABLED
    m->lvgl_built = true;
#endif

    m->scanned = true;
    ESP_LOGI(TAG, "Hardware scan complete");
    return CUBE32_OK;
}

/* ============================================================================
 * Print helpers
 * ============================================================================ */

/** Returns the display board model string from the Kconfig selection. */
static const char* display_model_str(void)
{
#if defined(CONFIG_CUBE32_DISPLAY_CUBE_TFT_TOUCH_154)
    return "CUBE_TFT_TOUCH_154 (240x240)";
#elif defined(CONFIG_CUBE32_DISPLAY_CUBE_TFT_TOUCH_200)
    return "CUBE_TFT_TOUCH_200 (240x320)";
#else
    return "Unknown";
#endif
}

/** Returns the touch IC chip name from the detected IC type. */
static const char* touch_ic_name(uint8_t ic)
{
    switch ((cube32_touch_ic_t)ic) {
        case CUBE32_TOUCH_IC_CST816:  return "CST816S";
        case CUBE32_TOUCH_IC_FT6336:  return "FT6336";
        default:                       return "Unknown";
    }
}

/* ============================================================================
 * Print
 * ============================================================================ */

void cube32_hw_manifest_print(const cube32_hw_manifest_t* m)
{
    if (!m) m = &s_manifest;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "CUBE32 Hardware Manifest");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Board   : %s", CONFIG_CUBE32_BOARD_NAME);
    ESP_LOGI(TAG, "Profile : %s", cube32_profile_name(m->profile));
    ESP_LOGI(TAG, "I2C devs: %u", m->i2c_device_count);
    ESP_LOGI(TAG, "%-10s | %-5s | %-7s | %-6s | %s", "Component", "Built", "Present", "Active", "Info");
    ESP_LOGI(TAG, "---------- | ----- | ------- | ------ | ----");
    ESP_LOGI(TAG, "%-10s | %-5s | %-7s | %-6s | AXP2101 @ 0x%02X", "PMU",
             m->pmu_built ? "YES" : "no", m->pmu_present ? "YES" : "no", "  --  ", m->pmu_i2c_addr);
    ESP_LOGI(TAG, "%-10s | %-5s | %-7s | %-6s | BM8563 @ 0x%02X", "RTC",
             m->rtc_built ? "YES" : "no", m->rtc_present ? "YES" : "no", "  --  ", m->rtc_i2c_addr);
    ESP_LOGI(TAG, "%-10s | %-5s | %-7s | %-6s | %s", "Display",
             m->display_built ? "YES" : "no", m->display_present ? "YES" : "pend", "  --  ", display_model_str());
    ESP_LOGI(TAG, "%-10s | %-5s | %-7s | %-6s | %s @ 0x%02X", "Touch",
             m->touch_built ? "YES" : "no", m->touch_present ? "YES" : "no", "  --  ",
             touch_ic_name(m->touch_ic), m->touch_i2c_addr);
    ESP_LOGI(TAG, "%-10s | %-5s | %-7s | %-6s | ES8311@0x%02X  ES7210@0x%02X  IOX@0x%02X", "Audio",
             m->audio_built ? "YES" : "no", m->audio_module_present ? "YES" : "no",
             m->audio_active ? "YES" : "no",
             CUBE32_I2C_ADDR_ES8311, CUBE32_I2C_ADDR_ES7210, CUBE32_I2C_ADDR_TCA9554_AUDIO);
    ESP_LOGI(TAG, "%-10s | %-5s | %-7s | %-6s | IOX@0x%02X", "Modem",
             m->modem_built ? "YES" : "no", m->modem_module_present ? "YES" : "no",
             m->modem_active ? "YES" : "no", CUBE32_I2C_ADDR_TCA9554_MODEM);
    ESP_LOGI(TAG, "%-10s | %-5s | %-7s | %-6s | %s", "Camera",
             m->camera_built ? "YES" : "no", m->camera_present ? "YES" : "pend",
             m->camera_active ? "YES" : "no",
             m->camera_sensor_name[0] ? m->camera_sensor_name : "pend");
    ESP_LOGI(TAG, "%-10s | %-5s | %-7s | %-6s |", "SD Card",
             m->sdcard_built ? "YES" : "no", m->sdcard_present ? "YES" : "pend",
             m->sdcard_active ? "YES" : "no");
    ESP_LOGI(TAG, "%-10s | %-5s | %-7s | %-6s |", "BLE OTA",
             m->ble_ota_built ? "YES" : "no", "  --  ",
             m->ble_ota_active ? "YES" : "no");
    ESP_LOGI(TAG, "%-10s | %-5s | %-7s | %-6s |", "USB Input",
             m->usb_input_built ? "YES" : "no", "  --  ",
             m->usb_input_active ? "YES" : "no");
    ESP_LOGI(TAG, "%-10s | %-5s | %-7s | %-6s |", "ADC Button",
             m->adc_button_built ? "YES" : "no", "  --  ",
             m->adc_button_active ? "YES" : "no");
    ESP_LOGI(TAG, "%-10s | %-5s | %-7s | %-6s |", "Servo",
             m->servo_built ? "YES" : "no", "  --  ",
             m->servo_active ? "YES" : "no");
    ESP_LOGI(TAG, "%-10s | %-5s | %-7s | %-6s |", "LVGL",
             m->lvgl_built ? "YES" : "no", "  --  ", "  --  ");
    ESP_LOGI(TAG, "%-10s | %-5s | %-7s | %-6s | LSM6DSOX @ 0x%02X", "IMU",
             m->imu_built ? "YES" : "no", m->imu_present ? "YES" : "no",
             m->imu_active ? "YES" : "no", m->imu_i2c_addr);
    ESP_LOGI(TAG, "%-10s | %-5s | %-7s | %-6s | LIS3MDL @ 0x%02X", "Mag",
             m->mag_built ? "YES" : "no", m->mag_present ? "YES" : "no",
             m->mag_active ? "YES" : "no", m->mag_i2c_addr);
    ESP_LOGI(TAG, "========================================");
}

/* ============================================================================
 * Init code / driver name helpers
 * ============================================================================ */

const char* cube32_init_code_name(cube32_init_code_t code)
{
    switch (code) {
        case CUBE32_INIT_OK:            return " OK ";
        case CUBE32_INIT_FAIL:          return "FAIL";
        case CUBE32_INIT_INITIALIZING:  return "INIT";
        case CUBE32_INIT_SKIPPED:       return "SKIP";
        case CUBE32_INIT_NOT_REQUESTED: return " OFF";
        case CUBE32_INIT_NOT_PRESENT:   return " N/B";
        default:                        return " -- ";
    }
}

const char* cube32_driver_name(cube32_driver_index_t index)
{
    switch (index) {
        case CUBE32_DRV_PMU:        return "PMU";
        case CUBE32_DRV_RTC:        return "RTC";
        case CUBE32_DRV_DISPLAY:    return "Display";
        case CUBE32_DRV_LVGL:       return "LVGL";
        case CUBE32_DRV_TOUCH:      return "Touch";
        case CUBE32_DRV_AUDIO:      return "Audio";
        case CUBE32_DRV_MODEM:      return "Modem";
        case CUBE32_DRV_CAMERA:     return "Camera";
        case CUBE32_DRV_SDCARD:     return "SD Card";
        case CUBE32_DRV_BLE_OTA:    return "BLE OTA";
        case CUBE32_DRV_USB_INPUT:  return "USB Input";
        case CUBE32_DRV_ADC_BUTTON: return "ADC Button";
        case CUBE32_DRV_SERVO:      return "Servo";
        case CUBE32_DRV_ROBOT_HEAD: return "Robot Head";
        case CUBE32_DRV_IMU:        return "IMU";
        case CUBE32_DRV_MAG:        return "Mag";
        default:                    return "?";
    }
}

/* ============================================================================
 * Post-Init Status Print
 * ============================================================================ */

void cube32_hw_manifest_print_init_status(const cube32_hw_manifest_t* m)
{
    if (!m) m = &s_manifest;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "CUBE32 Driver Init Status");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Board   : %s", CONFIG_CUBE32_BOARD_NAME);
    ESP_LOGI(TAG, "Profile : %s", cube32_profile_name(m->profile));
    ESP_LOGI(TAG, "%-10s | %-5s | %-7s | %-6s | %-4s | %s",
             "Component", "Built", "Present", "Active", "Init", "Info");
    ESP_LOGI(TAG, "---------- | ----- | ------- | ------ | ---- | ----");

    /* Helper macro to reduce repetition for the Init column */
    #define INIT_STR(idx) cube32_init_code_name(m->init_records[idx].code)

    ESP_LOGI(TAG, "%-10s | %-5s | %-7s | %-6s | %-4s | AXP2101 @ 0x%02X", "PMU",
             m->pmu_built ? "YES" : "no", m->pmu_present ? "YES" : "no",
             "  --  ", INIT_STR(CUBE32_DRV_PMU), m->pmu_i2c_addr);
    ESP_LOGI(TAG, "%-10s | %-5s | %-7s | %-6s | %-4s | BM8563 @ 0x%02X", "RTC",
             m->rtc_built ? "YES" : "no", m->rtc_present ? "YES" : "no",
             "  --  ", INIT_STR(CUBE32_DRV_RTC), m->rtc_i2c_addr);
    ESP_LOGI(TAG, "%-10s | %-5s | %-7s | %-6s | %-4s | %s", "Display",
             m->display_built ? "YES" : "no", m->display_present ? "YES" : "no",
             "  --  ", INIT_STR(CUBE32_DRV_DISPLAY), display_model_str());
    ESP_LOGI(TAG, "%-10s | %-5s | %-7s | %-6s | %-4s |", "LVGL",
             m->lvgl_built ? "YES" : "no", "  --  ",
             "  --  ", INIT_STR(CUBE32_DRV_LVGL));
    ESP_LOGI(TAG, "%-10s | %-5s | %-7s | %-6s | %-4s | %s @ 0x%02X", "Touch",
             m->touch_built ? "YES" : "no", m->touch_present ? "YES" : "no",
             "  --  ", INIT_STR(CUBE32_DRV_TOUCH),
             touch_ic_name(m->touch_ic), m->touch_i2c_addr);
    ESP_LOGI(TAG, "%-10s | %-5s | %-7s | %-6s | %-4s | ES8311@0x%02X  ES7210@0x%02X  IOX@0x%02X", "Audio",
             m->audio_built ? "YES" : "no", m->audio_module_present ? "YES" : "no",
             m->audio_active ? "YES" : "no", INIT_STR(CUBE32_DRV_AUDIO),
             CUBE32_I2C_ADDR_ES8311, CUBE32_I2C_ADDR_ES7210, CUBE32_I2C_ADDR_TCA9554_AUDIO);
    ESP_LOGI(TAG, "%-10s | %-5s | %-7s | %-6s | %-4s | IOX@0x%02X", "Modem",
             m->modem_built ? "YES" : "no", m->modem_module_present ? "YES" : "no",
             m->modem_active ? "YES" : "no", INIT_STR(CUBE32_DRV_MODEM),
             CUBE32_I2C_ADDR_TCA9554_MODEM);
    ESP_LOGI(TAG, "%-10s | %-5s | %-7s | %-6s | %-4s | %s", "Camera",
             m->camera_built ? "YES" : "no", m->camera_present ? "YES" : "no",
             m->camera_active ? "YES" : "no", INIT_STR(CUBE32_DRV_CAMERA),
             m->camera_sensor_name[0] ? m->camera_sensor_name : "N/A");
    ESP_LOGI(TAG, "%-10s | %-5s | %-7s | %-6s | %-4s |", "SD Card",
             m->sdcard_built ? "YES" : "no", m->sdcard_present ? "YES" : "no",
             m->sdcard_active ? "YES" : "no", INIT_STR(CUBE32_DRV_SDCARD));
    ESP_LOGI(TAG, "%-10s | %-5s | %-7s | %-6s | %-4s |", "BLE OTA",
             m->ble_ota_built ? "YES" : "no", "  --  ",
             m->ble_ota_active ? "YES" : "no", INIT_STR(CUBE32_DRV_BLE_OTA));
    ESP_LOGI(TAG, "%-10s | %-5s | %-7s | %-6s | %-4s |", "USB Input",
             m->usb_input_built ? "YES" : "no", "  --  ",
             m->usb_input_active ? "YES" : "no", INIT_STR(CUBE32_DRV_USB_INPUT));
    ESP_LOGI(TAG, "%-10s | %-5s | %-7s | %-6s | %-4s |", "ADC Button",
             m->adc_button_built ? "YES" : "no", "  --  ",
             m->adc_button_active ? "YES" : "no", INIT_STR(CUBE32_DRV_ADC_BUTTON));
    ESP_LOGI(TAG, "%-10s | %-5s | %-7s | %-6s | %-4s |", "Servo",
             m->servo_built ? "YES" : "no", "  --  ",
             m->servo_active ? "YES" : "no", INIT_STR(CUBE32_DRV_SERVO));
    ESP_LOGI(TAG, "%-10s | %-5s | %-7s | %-6s | %-4s |", "Robot Head",
             m->servo_built ? "YES" : "no", "  --  ",
             "  --  ", INIT_STR(CUBE32_DRV_ROBOT_HEAD));
    ESP_LOGI(TAG, "%-10s | %-5s | %-7s | %-6s | %-4s | LSM6DSOX @ 0x%02X", "IMU",
             m->imu_built ? "YES" : "no", m->imu_present ? "YES" : "no",
             m->imu_active ? "YES" : "no", INIT_STR(CUBE32_DRV_IMU), m->imu_i2c_addr);
    ESP_LOGI(TAG, "%-10s | %-5s | %-7s | %-6s | %-4s | LIS3MDL @ 0x%02X", "Mag",
             m->mag_built ? "YES" : "no", m->mag_present ? "YES" : "no",
             m->mag_active ? "YES" : "no", INIT_STR(CUBE32_DRV_MAG), m->mag_i2c_addr);

    #undef INIT_STR
    ESP_LOGI(TAG, "========================================");
}
