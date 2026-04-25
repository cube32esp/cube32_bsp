/**
 * @file cube32.cpp
 * @brief CUBE32 Board Support Package - Core Implementation
 */

#include "cube32.h"
#include "utils/hw_manifest.h"
#include "utils/config_manager.h"

#include <cstdio>
#include <cinttypes>
#include <cstring>

#include <esp_err.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_chip_info.h>
#include <esp_flash.h>
#include <nvs_flash.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <esp_ota_ops.h>
#include <esp_app_format.h>

static const char *TAG = "cube32";

static bool s_core_initialized = false;

/**
 * @brief Get CUBE32 BSP version string
 */
const char* cube32_get_version(void)
{
    static char version[16];
    snprintf(version, sizeof(version), "%d.%d.%d",
             CUBE32_BSP_VERSION_MAJOR,
             CUBE32_BSP_VERSION_MINOR,
             CUBE32_BSP_VERSION_PATCH);
    return version;
}

/**
 * @brief Print board information to log
 */
void cube32_print_board_info(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "CUBE32 BSP Version: %s", cube32_get_version());
    ESP_LOGI(TAG, "========================================");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    
    ESP_LOGI(TAG, "Chip: %s with %d CPU core(s)", CONFIG_IDF_TARGET, chip_info.cores);
    
    ESP_LOGI(TAG, "Features: %s%s%s%s",
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? "/802.15.4" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    ESP_LOGI(TAG, "Silicon revision: v%d.%d", major_rev, minor_rev);

    uint32_t flash_size;
    esp_flash_get_size(NULL, &flash_size);
    ESP_LOGI(TAG, "Flash: %" PRIu32 " MB %s", 
             flash_size / (1024 * 1024),
             (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    ESP_LOGI(TAG, "Free heap: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "Min free heap: %" PRIu32 " bytes", esp_get_minimum_free_heap_size());
    ESP_LOGI(TAG, "========================================");
}

/**
 * @brief Initialize only the core system components
 */
esp_err_t cube32_init_core(void)
{
    if (s_core_initialized) {
        ESP_LOGW(TAG, "Core already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing CUBE32 core...");

    /* Initialize NVS flash */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "Erasing NVS flash to fix corruption");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized");

    /* Initialize network interface */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_LOGI(TAG, "Network interface initialized");

    /* Initialize event loop */
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_LOGI(TAG, "Event loop initialized");

    s_core_initialized = true;
    ESP_LOGI(TAG, "CUBE32 core initialization complete");

    return ESP_OK;
}

/**
 * @brief Initialize CUBE32 board with all enabled peripherals
 */
esp_err_t cube32_init(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Initializing CUBE32 Board...");
    ESP_LOGI(TAG, "BSP Version: %s", cube32_get_version());
    ESP_LOGI(TAG, "========================================");

    /* ---- OTA boot partition info & app rollback validation ---- */
    {
        const esp_partition_t *running = esp_ota_get_running_partition();
        const esp_partition_t *boot    = esp_ota_get_boot_partition();
        esp_app_desc_t app_desc;
        esp_ota_get_partition_description(running, &app_desc);

        ESP_LOGI(TAG, "Boot partition : %s (offset=0x%08" PRIx32 ")",
                 boot ? boot->label : "unknown",
                 boot ? (uint32_t)boot->address : 0);
        ESP_LOGI(TAG, "Running partition: %s (offset=0x%08" PRIx32 ")",
                 running ? running->label : "unknown",
                 running ? (uint32_t)running->address : 0);
        ESP_LOGI(TAG, "Firmware: %s v%s (%s %s)",
                 app_desc.project_name, app_desc.version,
                 app_desc.date, app_desc.time);

        // When app rollback is enabled, we MUST confirm the running app is valid.
        // Otherwise the bootloader will roll back to the previous firmware on next boot.
        esp_ota_img_states_t ota_state;
        if (running && esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
            ESP_LOGI(TAG, "OTA state: %d (%s)", (int)ota_state,
                     ota_state == ESP_OTA_IMG_NEW            ? "NEW" :
                     ota_state == ESP_OTA_IMG_PENDING_VERIFY ? "PENDING_VERIFY" :
                     ota_state == ESP_OTA_IMG_VALID          ? "VALID" :
                     ota_state == ESP_OTA_IMG_INVALID        ? "INVALID" :
                     ota_state == ESP_OTA_IMG_ABORTED        ? "ABORTED" :
                     ota_state == ESP_OTA_IMG_UNDEFINED      ? "UNDEFINED" : "UNKNOWN");

            if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
                ESP_LOGI(TAG, "OTA: First boot after update — marking app as VALID");
                esp_err_t ota_err = esp_ota_mark_app_valid_cancel_rollback();
                if (ota_err != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to mark app valid: %s", esp_err_to_name(ota_err));
                } else {
                    ESP_LOGI(TAG, "OTA: App confirmed valid, rollback cancelled");
                }
            }
        } else {
            ESP_LOGI(TAG, "OTA state: not available (factory partition or no otadata)");
        }
    }

    /* Initialize core components */
    ret = cube32_init_core();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize core: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Print board information */
    cube32_print_board_info();

    /* Initialize shared I2C bus (required for PMU, Touch, etc.) */
    ESP_LOGI(TAG, "I2C Bus: Initializing shared bus...");
    cube32_result_t i2c_ret = cube32::I2CBus::instance().init();
    if (i2c_ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C bus: %d", i2c_ret);
        // Continue anyway - but I2C devices will fail
    } else {
        ESP_LOGI(TAG, "I2C Bus: Initialized (SDA:%d, SCL:%d)", 
                 CUBE32_I2C_SDA_PIN, CUBE32_I2C_SCL_PIN);
    }

    /* Initialize shared SPI bus (required for Display, SD Card, etc.) */
    ESP_LOGI(TAG, "SPI Bus: Initializing shared bus...");
    cube32_result_t spi_ret = cube32::SPIBus::instance().init();
    if (spi_ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %d", spi_ret);
        // Continue anyway - but SPI devices will fail
    } else {
        ESP_LOGI(TAG, "SPI Bus: Initialized (MOSI:%d, MISO:%d, SCLK:%d)", 
                 CUBE32_SPI_MOSI_PIN, CUBE32_SPI_MISO_PIN, CUBE32_SPI_SCLK_PIN);
    }

    /* Load runtime configuration from NVS (with compiled defaults as fallback) */
    ESP_LOGI(TAG, "Config: Loading runtime configuration...");
    cube32_result_t cfg_ret = cube32_cfg_load();
    if (cfg_ret != CUBE32_OK) {
        ESP_LOGW(TAG, "Config: Load returned %d — using compiled defaults", cfg_ret);
    }
    cube32_cfg_print(nullptr);

    /* Hardware manifest: scan I2C bus and detect board profile / modules */
    ESP_LOGI(TAG, "Hardware: Scanning for installed modules...");
    cube32_hw_manifest_t* hw = cube32_hw_manifest();
    cube32_result_t hw_ret = cube32_hw_manifest_scan(hw);
    if (hw_ret != CUBE32_OK) {
        ESP_LOGW(TAG, "Hardware: Scan returned %d", hw_ret);
    }
    cube32_hw_manifest_print(hw);

    /* Convenience macro for recording driver init results */
    #define CUBE32_INIT_SET(drv, c) do { hw->init_records[drv].code = (c); } while(0)

    /* ====================================================================
     * Pointer aliases for manifest & config used throughout init
     * ==================================================================== */
    const cube32_cfg_t* cfg = cube32_cfg();

    /* Initialize PMU — System Core (always compiled, skip if not detected) */
#ifdef CONFIG_CUBE32_PMU_ENABLED
    if (hw->pmu_present) {
        ESP_LOGI(TAG, "PMU: Initializing AXP2101 @ 0x%02X...", hw->pmu_i2c_addr);
        cube32_result_t pmu_ret = cube32::PMU::instance().begin();
        if (pmu_ret != CUBE32_OK) {
            ESP_LOGE(TAG, "Failed to initialize PMU: %d", pmu_ret);
            CUBE32_INIT_SET(CUBE32_DRV_PMU, CUBE32_INIT_FAIL);
        } else {
            ESP_LOGI(TAG, "PMU: Initialized successfully");
            CUBE32_INIT_SET(CUBE32_DRV_PMU, CUBE32_INIT_OK);
            cube32::PMU::instance().printStatus();
        }
    } else {
        ESP_LOGW(TAG, "PMU: Not detected on I2C bus — skipping");
        CUBE32_INIT_SET(CUBE32_DRV_PMU, CUBE32_INIT_SKIPPED);
    }
#else
    ESP_LOGI(TAG, "PMU: Not compiled");
    CUBE32_INIT_SET(CUBE32_DRV_PMU, CUBE32_INIT_NOT_PRESENT);
#endif

    /* Initialize RTC — System Core (always compiled, skip if not detected) */
#ifdef CONFIG_CUBE32_RTC_ENABLED
    if (hw->rtc_present) {
        ESP_LOGI(TAG, "RTC: Initializing BM8563 @ 0x%02X...", hw->rtc_i2c_addr);
        cube32_result_t rtc_ret = cube32::RTC::instance().begin();
        if (rtc_ret != CUBE32_OK) {
            ESP_LOGE(TAG, "Failed to initialize RTC: %d", rtc_ret);
            CUBE32_INIT_SET(CUBE32_DRV_RTC, CUBE32_INIT_FAIL);
        } else {
            ESP_LOGI(TAG, "RTC: Initialized successfully");
            CUBE32_INIT_SET(CUBE32_DRV_RTC, CUBE32_INIT_OK);
        }
    } else {
        ESP_LOGW(TAG, "RTC: Not detected on I2C bus — skipping");
        CUBE32_INIT_SET(CUBE32_DRV_RTC, CUBE32_INIT_SKIPPED);
    }
#else
    ESP_LOGI(TAG, "RTC: Not compiled");
    CUBE32_INIT_SET(CUBE32_DRV_RTC, CUBE32_INIT_NOT_PRESENT);
#endif

    /* Initialize BLE OTA if enabled (before Display/LVGL to control memory usage) */
    bool ble_ota_active = false;  // Track if BLE OTA service is running (NimBLE stack loaded)
    bool ble_ota_bypass = false;  // Track if non-essential drivers should be bypassed (double-boot OTA mode)
#ifdef CONFIG_CUBE32_BLE_OTA_ENABLED
    if (hw->ble_ota_active) {
        ESP_LOGI(TAG, "BLE OTA: Initializing...");
        cube32::BleOtaConfig ble_config = CUBE32_BLE_OTA_CONFIG_DEFAULT();
        cube32_result_t ble_ret = cube32::BleOta::instance().begin(ble_config);
        if (ble_ret != CUBE32_OK) {
            ESP_LOGE(TAG, "Failed to initialize BLE OTA: %d", ble_ret);
            CUBE32_INIT_SET(CUBE32_DRV_BLE_OTA, CUBE32_INIT_FAIL);
        } else {
            cube32::BleOta& ble = cube32::BleOta::instance();
            if (ble.isBypassed()) {
                // DOUBLE_BOOT_ENABLE is on but double-boot was NOT detected
                // System starts normally without BLE OTA service
                ble_ota_active = false;
                ble_ota_bypass = false;
                ESP_LOGI(TAG, "BLE OTA: STANDBY (double-boot not detected)");
                ESP_LOGI(TAG, "  Device name: %s", ble.getDeviceName());
                CUBE32_INIT_SET(CUBE32_DRV_BLE_OTA, CUBE32_INIT_OK);
            } else {
                // BLE OTA service is active (NimBLE stack running)
                ble_ota_active = true;
#if CONFIG_CUBE32_BLE_OTA_DOUBLE_BOOT_ENABLE
                // With DOUBLE_BOOT enabled, active means double-boot was detected
                // → bypass non-essential drivers and show built-in OTA UI
                ble_ota_bypass = true;
                ESP_LOGI(TAG, "BLE OTA: BYPASS MODE (double-boot detected)");
                ESP_LOGI(TAG, "  Device name: %s", ble.getDeviceName());
                ESP_LOGI(TAG, "  Note: Using single LVGL buffer to save memory for NimBLE");
                CUBE32_INIT_SET(CUBE32_DRV_BLE_OTA, CUBE32_INIT_OK);
#else
                // Without DOUBLE_BOOT, BLE OTA always runs in background alongside app
                ble_ota_bypass = false;
                ESP_LOGI(TAG, "BLE OTA: BACKGROUND MODE (always active)");
                ESP_LOGI(TAG, "  Device name: %s", ble.getDeviceName());
                CUBE32_INIT_SET(CUBE32_DRV_BLE_OTA, CUBE32_INIT_OK);
#endif
            }
        }
    } else {
        ESP_LOGI(TAG, "BLE OTA: Deactivated by user");
        CUBE32_INIT_SET(CUBE32_DRV_BLE_OTA, CUBE32_INIT_NOT_REQUESTED);
    }
#endif  // CONFIG_CUBE32_BLE_OTA_ENABLED

    /* Initialize Display if enabled — Core Peripheral */
#ifdef CONFIG_CUBE32_DISPLAY_ENABLED
    ESP_LOGI(TAG, "Display: Initializing ST7789...");
    cube32_result_t disp_ret = cube32::ST7789Display::instance().begin();
    if (disp_ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to initialize display: %d", disp_ret);
        CUBE32_INIT_SET(CUBE32_DRV_DISPLAY, CUBE32_INIT_FAIL);
    } else {
        hw->display_present = true;
        ESP_LOGI(TAG, "Display: Initialized successfully (%dx%d)",
                 cube32::ST7789Display::instance().getWidth(),
                 cube32::ST7789Display::instance().getHeight());
        CUBE32_INIT_SET(CUBE32_DRV_DISPLAY, CUBE32_INIT_OK);

        /* Initialize LVGL if enabled */
#ifdef CONFIG_CUBE32_LVGL_ENABLED
        ESP_LOGI(TAG, "LVGL: Initializing...");
        cube32_lvgl_config_t lvgl_config = CUBE32_LVGL_CONFIG_DEFAULT();
#ifdef CONFIG_CUBE32_LVGL_DOUBLE_BUFFER
        // When BLE OTA is in bypass mode (double-boot), use single buffer to save memory
        if (ble_ota_bypass) {
            lvgl_config.double_buffer = false;
            ESP_LOGI(TAG, "LVGL: Using single buffer (BLE OTA bypass mode, saving memory)");
        } else {
            lvgl_config.double_buffer = true;
        }
#else
        lvgl_config.double_buffer = false;
#endif
#ifdef CONFIG_CUBE32_LVGL_USE_SPIRAM
        lvgl_config.use_spiram = true;
#else
        lvgl_config.use_spiram = false;
#endif
#ifdef CONFIG_CUBE32_LVGL_TASK_PRIORITY
        lvgl_config.task_priority = CONFIG_CUBE32_LVGL_TASK_PRIORITY;
#endif
#ifdef CONFIG_CUBE32_LVGL_TASK_STACK_SIZE
        lvgl_config.task_stack_size = CONFIG_CUBE32_LVGL_TASK_STACK_SIZE;
#endif
        cube32_result_t lvgl_ret = cube32::LvglDisplay::instance().begin(lvgl_config);
        if (lvgl_ret != CUBE32_OK) {
            ESP_LOGE(TAG, "Failed to initialize LVGL: %d", lvgl_ret);
            CUBE32_INIT_SET(CUBE32_DRV_LVGL, CUBE32_INIT_FAIL);
        } else {
            ESP_LOGI(TAG, "LVGL: Initialized successfully");
            CUBE32_INIT_SET(CUBE32_DRV_LVGL, CUBE32_INIT_OK);

            // Apply display rotation from NVS config
            if (cfg->display_rotation != 0) {
                cube32::LvglDisplay::instance().setRotation(cfg->display_rotation);
                ESP_LOGI(TAG, "Display: Rotation set to %u° (from config)", cfg->display_rotation);
            }
        }

        // Apply prism mode from NVS config (must be done after LVGL sets rotation)
        if (cfg->display_prism) {
            uint16_t current_rotation = cube32::LvglDisplay::instance().getRotation();
            cube32::ST7789Display::instance().setPrismMode(true, current_rotation);
            ESP_LOGI(TAG, "Display: Prism mode enabled (from config)");
        }

        // Show built-in BLE OTA UI only in bypass mode (double-boot detected)
#ifdef CONFIG_CUBE32_BLE_OTA_ENABLED
        if (ble_ota_bypass) {
            cube32::BleOta& ble = cube32::BleOta::instance();
            if (ble.isInitialized() && !ble.isBypassed()) {
                ESP_LOGI(TAG, "Built-in BLE OTA UI: Creating...");
                cube32::ble_ota_ui_create();
                ESP_LOGI(TAG, "Built-in BLE OTA UI: Created");
            }
        }
        // Start OTA monitor whenever BLE OTA is active (bypass or background)
        // The monitor auto-shows a pop-up overlay when OTA update begins
        if (ble_ota_active) {
            ESP_LOGI(TAG, "BLE OTA Monitor: Starting...");
            cube32::ble_ota_monitor_start();
        }
#endif
#else
        ESP_LOGI(TAG, "LVGL: Disabled");
        CUBE32_INIT_SET(CUBE32_DRV_LVGL, CUBE32_INIT_NOT_PRESENT);

        // Apply display rotation from NVS config (non-LVGL path)
        if (cfg->display_rotation != 0) {
            cube32::ST7789Display::instance().setRotation(cfg->display_rotation);
            ESP_LOGI(TAG, "Display: Rotation set to %u° (from config)", cfg->display_rotation);
        }

        // Apply prism mode from NVS config for non-LVGL use
        if (cfg->display_prism) {
            uint16_t current_rotation = cube32::ST7789Display::instance().getRotation();
            cube32::ST7789Display::instance().setPrismMode(true, current_rotation);
            ESP_LOGI(TAG, "Display: Prism mode enabled (from config)");
        }
#endif
    }
#else
    ESP_LOGI(TAG, "Display: Disabled");
    CUBE32_INIT_SET(CUBE32_DRV_DISPLAY, CUBE32_INIT_NOT_PRESENT);
    CUBE32_INIT_SET(CUBE32_DRV_LVGL, CUBE32_INIT_SKIPPED);
#endif

    /* Initialize Touch — Core Peripheral (skip if not detected on I2C) */
#ifdef CONFIG_CUBE32_TOUCH_ENABLED
    if (hw->touch_present) {
    ESP_LOGI(TAG, "Touch: Initializing (IC=0x%02X)...", hw->touch_i2c_addr);
    cube32_result_t touch_ret = cube32::Touch::instance().begin();
    if (touch_ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to initialize touch: %d", touch_ret);
        CUBE32_INIT_SET(CUBE32_DRV_TOUCH, CUBE32_INIT_FAIL);
    } else {
        const char* ic_name = "Unknown";
        switch (cube32::Touch::instance().getICType()) {
            case CUBE32_TOUCH_IC_CST816: ic_name = "CST816S"; break;
            case CUBE32_TOUCH_IC_FT6336: ic_name = "FT6336"; break;
            default: break;
        }
        ESP_LOGI(TAG, "Touch: Initialized successfully (%s, %dx%d)",
                 ic_name,
                 cube32::Touch::instance().getWidth(),
                 cube32::Touch::instance().getHeight());
        CUBE32_INIT_SET(CUBE32_DRV_TOUCH, CUBE32_INIT_OK);

        /* Add touch to LVGL if LVGL is enabled and initialized */
#ifdef CONFIG_CUBE32_LVGL_ENABLED
        if (cube32::LvglDisplay::instance().isInitialized()) {
            cube32_result_t lvgl_touch_ret = cube32::LvglDisplay::instance().addTouch();
            if (lvgl_touch_ret != CUBE32_OK) {
                ESP_LOGW(TAG, "Failed to add touch to LVGL: %d", lvgl_touch_ret);
            } else {
                ESP_LOGI(TAG, "Touch: Added to LVGL successfully");
            }
            // Note: Rotation is already applied by LVGL begin(). LVGL automatically
            // transforms touch coordinates based on the display rotation, so no need
            // to call setRotation() again here.
        }
#endif
    }
    } else {
        ESP_LOGW(TAG, "Touch: Not detected on I2C bus — skipping");
        CUBE32_INIT_SET(CUBE32_DRV_TOUCH, CUBE32_INIT_SKIPPED);
    }
#else
    ESP_LOGI(TAG, "Touch: Not compiled");
    CUBE32_INIT_SET(CUBE32_DRV_TOUCH, CUBE32_INIT_NOT_PRESENT);
#endif

    /* ====================================================================
     * Remaining driver initialization
     * When BLE OTA is in bypass mode (double-boot detected), skip
     * non-essential drivers to minimize memory usage and focus on OTA.
     * Core drivers (I2C, SPI, PMU, RTC, Display, LVGL, Touch) are
     * always initialized regardless of BLE OTA mode.
     * ==================================================================== */
    if (ble_ota_bypass) {
        ESP_LOGI(TAG, "BLE OTA bypass mode: Skipping non-essential drivers");
        ESP_LOGI(TAG, "  Bypassed: USB Input, Audio, ADC Button, SD Card, Modem, Camera");
        CUBE32_INIT_SET(CUBE32_DRV_USB_INPUT, CUBE32_INIT_SKIPPED);
        CUBE32_INIT_SET(CUBE32_DRV_AUDIO, CUBE32_INIT_SKIPPED);
        CUBE32_INIT_SET(CUBE32_DRV_ADC_BUTTON, CUBE32_INIT_SKIPPED);
        CUBE32_INIT_SET(CUBE32_DRV_SDCARD, CUBE32_INIT_SKIPPED);
        CUBE32_INIT_SET(CUBE32_DRV_MODEM, CUBE32_INIT_SKIPPED);
        CUBE32_INIT_SET(CUBE32_DRV_CAMERA, CUBE32_INIT_SKIPPED);
        CUBE32_INIT_SET(CUBE32_DRV_IMU, CUBE32_INIT_SKIPPED);
        CUBE32_INIT_SET(CUBE32_DRV_MAG, CUBE32_INIT_SKIPPED);
        CUBE32_INIT_SET(CUBE32_DRV_SERVO, CUBE32_INIT_SKIPPED);
        CUBE32_INIT_SET(CUBE32_DRV_ROBOT_HEAD, CUBE32_INIT_SKIPPED);
    } else {

    /* Initialize USB Input — Optional (build + active check) */
#ifdef CONFIG_CUBE32_USB_INPUT_ENABLED
    if (hw->usb_input_active) {
    ESP_LOGI(TAG, "USB Input: Initializing...");
    
    // Enable USB OTG 5V power via PMU (BLDO2)
#ifdef CONFIG_CUBE32_PMU_ENABLED
    if (cube32::PMU::instance().isInitialized()) {
        ESP_LOGI(TAG, "USB Input: Enabling USB OTG 5V power (BLDO2)...");
        cube32::PMU::instance().setUsbOtgVoltage(3300);
        cube32::PMU::instance().setUsbOtgPower(true);
    }
#endif

    cube32_result_t usb_ret = cube32::USBInput::instance().begin();
    if (usb_ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to initialize USB Input: %d", usb_ret);
        CUBE32_INIT_SET(CUBE32_DRV_USB_INPUT, CUBE32_INIT_FAIL);
        // Continue anyway - USB Input failure is not fatal
    } else {
        ESP_LOGI(TAG, "USB Input: Initialized successfully");
        CUBE32_INIT_SET(CUBE32_DRV_USB_INPUT, CUBE32_INIT_OK);

        /* Add USB mouse to LVGL if enabled */
#if defined(CONFIG_CUBE32_LVGL_ENABLED) && defined(CONFIG_CUBE32_USB_INPUT_LVGL_MOUSE)
        if (cube32::LvglDisplay::instance().isInitialized()) {
            cube32_result_t lvgl_mouse_ret = cube32::LvglDisplay::instance().addUsbMouse();
            if (lvgl_mouse_ret != CUBE32_OK) {
                ESP_LOGW(TAG, "Failed to add USB mouse to LVGL: %d", lvgl_mouse_ret);
            } else {
                ESP_LOGI(TAG, "USB Input: USB Mouse added to LVGL successfully");
            }
        }
#endif
    }
    } else {
        ESP_LOGI(TAG, "USB Input: Deactivated by user");
        CUBE32_INIT_SET(CUBE32_DRV_USB_INPUT, CUBE32_INIT_NOT_REQUESTED);
    }
#else
    ESP_LOGI(TAG, "USB Input: Not compiled");
    CUBE32_INIT_SET(CUBE32_DRV_USB_INPUT, CUBE32_INIT_NOT_PRESENT);
#endif

    /* Initialize Audio — Optional (build + module present + active) */
#ifdef CONFIG_CUBE32_AUDIO_ENABLED
    if (hw->audio_module_present && hw->audio_active) {
        ESP_LOGI(TAG, "Audio: Initializing ES8311/ES7210 codec...");
        cube32_result_t audio_ret = cube32::AudioCodec::instance().begin();
        if (audio_ret != CUBE32_OK) {
            ESP_LOGE(TAG, "Failed to initialize audio codec: %d", audio_ret);
            CUBE32_INIT_SET(CUBE32_DRV_AUDIO, CUBE32_INIT_FAIL);
        } else {
            ESP_LOGI(TAG, "Audio: Initialized successfully");
            CUBE32_INIT_SET(CUBE32_DRV_AUDIO, CUBE32_INIT_OK);
            ESP_LOGI(TAG, "  Input sample rate: %d Hz", cube32::AudioCodec::instance().getInputSampleRate());
            ESP_LOGI(TAG, "  Output sample rate: %d Hz", cube32::AudioCodec::instance().getOutputSampleRate());
        }
    } else if (!hw->audio_module_present) {
        ESP_LOGW(TAG, "Audio: Module not detected (ES8311/ES7210 absent) — skipping");
        CUBE32_INIT_SET(CUBE32_DRV_AUDIO, CUBE32_INIT_SKIPPED);
    } else {
        ESP_LOGI(TAG, "Audio: Deactivated by user");
        CUBE32_INIT_SET(CUBE32_DRV_AUDIO, CUBE32_INIT_NOT_REQUESTED);
    }
#else
    ESP_LOGI(TAG, "Audio: Not compiled");
    CUBE32_INIT_SET(CUBE32_DRV_AUDIO, CUBE32_INIT_NOT_PRESENT);
#endif

    /* Initialize ADC Button Array — Optional (build + audio module + active) */
#ifdef CONFIG_CUBE32_ADC_BUTTON_ENABLED
    if (hw->audio_module_present && hw->adc_button_active) {
        ESP_LOGI(TAG, "ADC Button: Initializing button array...");
        cube32_result_t btn_ret = cube32::ADCButton::instance().begin();
        if (btn_ret != CUBE32_OK) {
            ESP_LOGE(TAG, "Failed to initialize ADC button: %d", btn_ret);
            CUBE32_INIT_SET(CUBE32_DRV_ADC_BUTTON, CUBE32_INIT_FAIL);
        } else {
            ESP_LOGI(TAG, "ADC Button: Initialized successfully (%zu buttons)", 
                     cube32::ADCButton::instance().getButtonCount());
            CUBE32_INIT_SET(CUBE32_DRV_ADC_BUTTON, CUBE32_INIT_OK);
        }
    } else if (!hw->audio_module_present) {
        ESP_LOGW(TAG, "ADC Button: Audio module not detected — skipping");
        CUBE32_INIT_SET(CUBE32_DRV_ADC_BUTTON, CUBE32_INIT_SKIPPED);
    } else {
        ESP_LOGI(TAG, "ADC Button: Deactivated by user");
        CUBE32_INIT_SET(CUBE32_DRV_ADC_BUTTON, CUBE32_INIT_NOT_REQUESTED);
    }
#else
    ESP_LOGI(TAG, "ADC Button: Not compiled");
    CUBE32_INIT_SET(CUBE32_DRV_ADC_BUTTON, CUBE32_INIT_NOT_PRESENT);
#endif

    /* Initialize SD Card — Optional (build + active) */
#ifdef CONFIG_CUBE32_SDCARD_ENABLED
    if (hw->sdcard_active) {
        ESP_LOGI(TAG, "SD Card: Initializing...");
        cube32_result_t sd_ret = cube32::SDCard::instance().begin();
        if (sd_ret != CUBE32_OK) {
            ESP_LOGE(TAG, "Failed to initialize SD card: %d", sd_ret);
            CUBE32_INIT_SET(CUBE32_DRV_SDCARD, CUBE32_INIT_FAIL);
        } else {
            hw->sdcard_present = true;
            ESP_LOGI(TAG, "SD Card: Initialized successfully");
            CUBE32_INIT_SET(CUBE32_DRV_SDCARD, CUBE32_INIT_OK);
        }
    } else {
        ESP_LOGI(TAG, "SD Card: Deactivated by user");
        CUBE32_INIT_SET(CUBE32_DRV_SDCARD, CUBE32_INIT_NOT_REQUESTED);
    }
#else
    ESP_LOGI(TAG, "SD Card: Not compiled");
    CUBE32_INIT_SET(CUBE32_DRV_SDCARD, CUBE32_INIT_NOT_PRESENT);
#endif

    /* Initialize Modem if enabled
     * NOTE: USB Modem MUST be initialized BEFORE SD Card to avoid interrupt allocation conflict.
     * The USB Host Controller requires specific interrupt resources (Level 2/3) that become
     * unavailable after SDMMC initialization consumes them. Error if order is wrong:
     * "No free interrupt inputs for USB interrupt (flags 0x802)"
     */
#ifdef CONFIG_CUBE32_MODEM_ENABLED
    if (hw->modem_module_present && hw->modem_active) {
    ESP_LOGI(TAG, "Modem: Initializing A7670 (module detected via IOX@0x22)...");
    cube32::ModemConfig modem_config = CUBE32_MODEM_CONFIG_DEFAULT();
    
    // Apply configuration from Kconfig
#ifdef CONFIG_CUBE32_MODEM_CONNECTION_USB
    modem_config.connectionType     = cube32::ModemConnectionType::USB;
    // USB descriptor values are fixed by SimCom A7670 firmware (cube32_config.h)
    modem_config.usb_vid            = CUBE32_MODEM_USB_VID;
    modem_config.usb_pid            = CUBE32_MODEM_USB_PID;
    modem_config.usb_at_interface   = CUBE32_MODEM_USB_AT_INTERFACE;
    modem_config.usb_data_interface = CUBE32_MODEM_USB_DATA_INTERFACE;
#ifdef CONFIG_CUBE32_MODEM_USB_TIMEOUT_MS
    modem_config.usb_timeout_ms     = CONFIG_CUBE32_MODEM_USB_TIMEOUT_MS;
#endif
#else
    modem_config.connectionType = cube32::ModemConnectionType::UART;
    // UART port, TX pin, RX pin defaults are from cube32_config.h via ModemConfig
#ifdef CONFIG_CUBE32_MODEM_UART_BAUD_RATE
    modem_config.uart_baud_rate = CONFIG_CUBE32_MODEM_UART_BAUD_RATE;
#endif
#endif
    
    // APN from NVS config (falls back to compiled default)
    if (cfg->modem_apn[0] != '\0') {
        modem_config.apn = cfg->modem_apn;
    }

    // IO Expander configuration (fixed hardware, defined in cube32_config.h)
    modem_config.iox_i2c_addr    = CUBE32_MODEM_IOX_ADDR;
    modem_config.iox_pwrkey_pin  = CUBE32_MODEM_IOX_PWRKEY_PIN;
    modem_config.iox_dtr_pin     = CUBE32_MODEM_IOX_DTR_PIN;
    modem_config.iox_pwr_rail_pin = CUBE32_MODEM_IOX_PWR_RAIL_PIN;
    modem_config.iox_comm_sel_pin = CUBE32_MODEM_IOX_COMM_SEL_PIN;
    
    // Use async initialization so modem doesn't block other drivers
    // The modem init (power on, USB/UART setup, network registration) 
    // runs in a background task while other drivers continue to initialize
#ifdef CONFIG_CUBE32_MODEM_AUTO_CONNECT
    bool auto_register = true;
#else
    bool auto_register = false;
#endif
    
    cube32_result_t modem_ret = cube32::A7670Modem::instance().beginAsync(
        modem_config, 
        auto_register,
        nullptr  // No callback - applications should register their own if needed
    );
    if (modem_ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to start async modem initialization: %d", modem_ret);
        CUBE32_INIT_SET(CUBE32_DRV_MODEM, CUBE32_INIT_FAIL);
        // Continue anyway - modem failure is not fatal
    } else {
        ESP_LOGI(TAG, "Modem: Async initialization started (non-blocking)");
        CUBE32_INIT_SET(CUBE32_DRV_MODEM, CUBE32_INIT_INITIALIZING);
    }
    } else if (!hw->modem_module_present) {
        ESP_LOGW(TAG, "Modem: Module not detected (IOX@0x22 absent) — skipping");
        CUBE32_INIT_SET(CUBE32_DRV_MODEM, CUBE32_INIT_SKIPPED);
    } else {
        ESP_LOGI(TAG, "Modem: Deactivated by user");
        CUBE32_INIT_SET(CUBE32_DRV_MODEM, CUBE32_INIT_NOT_REQUESTED);
    }
#else
    ESP_LOGI(TAG, "Modem: Not compiled");
    CUBE32_INIT_SET(CUBE32_DRV_MODEM, CUBE32_INIT_NOT_PRESENT);
#endif

    /* Initialize Camera — Optional (build + active) */
#ifdef CONFIG_CUBE32_CAMERA_ENABLED
    if (hw->camera_active) {
    ESP_LOGI(TAG, "Camera: Initializing...");
    
    // Configure camera with settings from Kconfig (frame size, pixel format)
    // Mirror/flip from NVS config
    cube32_camera_config_t cam_config = CUBE32_CAMERA_CONFIG_DEFAULT();
    
    // Apply frame size from Kconfig
#if defined(CONFIG_CUBE32_CAMERA_FRAME_SIZE_QQVGA)
    cam_config.frame_size = FRAMESIZE_QQVGA;
#elif defined(CONFIG_CUBE32_CAMERA_FRAME_SIZE_QVGA)
    cam_config.frame_size = FRAMESIZE_QVGA;
#elif defined(CONFIG_CUBE32_CAMERA_FRAME_SIZE_VGA)
    cam_config.frame_size = FRAMESIZE_VGA;
#elif defined(CONFIG_CUBE32_CAMERA_FRAME_SIZE_SVGA)
    cam_config.frame_size = FRAMESIZE_SVGA;
#elif defined(CONFIG_CUBE32_CAMERA_FRAME_SIZE_XGA)
    cam_config.frame_size = FRAMESIZE_XGA;
#elif defined(CONFIG_CUBE32_CAMERA_FRAME_SIZE_SXGA)
    cam_config.frame_size = FRAMESIZE_SXGA;
#endif

    // Apply pixel format from Kconfig
#if defined(CONFIG_CUBE32_CAMERA_PIXEL_FORMAT_RGB565)
    cam_config.pixel_format = PIXFORMAT_RGB565;
#elif defined(CONFIG_CUBE32_CAMERA_PIXEL_FORMAT_JPEG)
    cam_config.pixel_format = PIXFORMAT_JPEG;
    // JPEG format requires 2 frame buffers for continuous streaming mode
    cam_config.fb_count = 2;
    // Use GRAB_WHEN_EMPTY for stable high-resolution streaming
    cam_config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
#elif defined(CONFIG_CUBE32_CAMERA_PIXEL_FORMAT_GRAYSCALE)
    cam_config.pixel_format = PIXFORMAT_GRAYSCALE;
#endif

    // Apply h_mirror and v_flip from NVS config
    cam_config.h_mirror = cfg->camera_h_mirror ? 1 : 0;
    cam_config.v_flip   = cfg->camera_v_flip   ? 1 : 0;

    // Camera init with retry — the shared I2C bus may be transiently
    // unstable after modem IO expander initialization, causing SCCB
    // writes to fail with ESP_ERR_INVALID_STATE.  A brief delay and
    // retry reliably recovers.
    static const int CAMERA_INIT_MAX_RETRIES = 3;
    cube32_result_t cam_ret = CUBE32_ERROR;
    for (int attempt = 0; attempt < CAMERA_INIT_MAX_RETRIES; attempt++) {
        cam_ret = cube32::Camera::instance().begin(cam_config);
        if (cam_ret == CUBE32_OK) break;
        ESP_LOGW(TAG, "Camera init attempt %d/%d failed (err %d), retrying...",
                 attempt + 1, CAMERA_INIT_MAX_RETRIES, cam_ret);
        esp_camera_deinit();  // Clean up partial init before retry
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    if (cam_ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to initialize camera after %d attempts: %d",
                 CAMERA_INIT_MAX_RETRIES, cam_ret);
        CUBE32_INIT_SET(CUBE32_DRV_CAMERA, CUBE32_INIT_FAIL);
        // Continue anyway - camera failure is not fatal
    } else {
        hw->camera_present = true;
        // Record the detected sensor model in the hardware manifest
        {
            sensor_t* cam_s = cube32::Camera::instance().getSensor();
            const char* cam_name = "Unknown";
            if (cam_s) {
                switch ((camera_pid_t)cam_s->id.PID) {
                    case OV9650_PID:   cam_name = "OV9650";   break;
                    case OV7725_PID:   cam_name = "OV7725";   break;
                    case OV2640_PID:   cam_name = "OV2640";   break;
                    case OV3660_PID:   cam_name = "OV3660";   break;
                    case OV5640_PID:   cam_name = "OV5640";   break;
                    case OV7670_PID:   cam_name = "OV7670";   break;
                    case NT99141_PID:  cam_name = "NT99141";  break;
                    case GC2145_PID:   cam_name = "GC2145";   break;
                    case GC032A_PID:   cam_name = "GC032A";   break;
                    case GC0308_PID:   cam_name = "GC0308";   break;
                    case BF3005_PID:   cam_name = "BF3005";   break;
                    case BF20A6_PID:   cam_name = "BF20A6";   break;
                    case SC101IOT_PID: cam_name = "SC101IOT"; break;
                    case SC030IOT_PID: cam_name = "SC030IOT"; break;
                    case SC031GS_PID:  cam_name = "SC031GS";  break;
                    case MEGA_CCM_PID: cam_name = "MEGA_CCM"; break;
                    case HM1055_PID:   cam_name = "HM1055";   break;
                    case HM0360_PID:   cam_name = "HM0360";   break;
                    default:           cam_name = "Unknown";  break;
                }
            }
            strncpy(hw->camera_sensor_name, cam_name, sizeof(hw->camera_sensor_name) - 1);
            hw->camera_sensor_name[sizeof(hw->camera_sensor_name) - 1] = '\0';
        }
        ESP_LOGI(TAG, "Camera: Initialized successfully (h_mirror: %d, v_flip: %d)",
                 cam_config.h_mirror, cam_config.v_flip);
        CUBE32_INIT_SET(CUBE32_DRV_CAMERA, CUBE32_INIT_OK);
    }
    } else {
        ESP_LOGI(TAG, "Camera: Deactivated by user");
        CUBE32_INIT_SET(CUBE32_DRV_CAMERA, CUBE32_INIT_NOT_REQUESTED);
    }
#else
    ESP_LOGI(TAG, "Camera: Not compiled");
    CUBE32_INIT_SET(CUBE32_DRV_CAMERA, CUBE32_INIT_NOT_PRESENT);
#endif

    /* Initialize IMU — Optional (build + present + active) */
#ifdef CONFIG_CUBE32_IMU_ENABLED
    if (hw->imu_present && hw->imu_active) {
        ESP_LOGI(TAG, "IMU: Initializing LSM6DSO @ 0x%02X...", hw->imu_i2c_addr);
        cube32_result_t imu_ret = cube32::IMU::instance().begin();
        if (imu_ret != CUBE32_OK) {
            ESP_LOGE(TAG, "Failed to initialize IMU: %d", imu_ret);
            CUBE32_INIT_SET(CUBE32_DRV_IMU, CUBE32_INIT_FAIL);
        } else {
            ESP_LOGI(TAG, "IMU: LSM6DSO initialized successfully");
            CUBE32_INIT_SET(CUBE32_DRV_IMU, CUBE32_INIT_OK);
        }
    } else if (!hw->imu_present) {
        ESP_LOGW(TAG, "IMU: LSM6DSO not detected on I2C bus — skipping");
        CUBE32_INIT_SET(CUBE32_DRV_IMU, CUBE32_INIT_SKIPPED);
    } else {
        ESP_LOGI(TAG, "IMU: Deactivated by user");
        CUBE32_INIT_SET(CUBE32_DRV_IMU, CUBE32_INIT_NOT_REQUESTED);
    }
#else
    ESP_LOGI(TAG, "IMU: Not compiled");
    CUBE32_INIT_SET(CUBE32_DRV_IMU, CUBE32_INIT_NOT_PRESENT);
#endif

    /* Initialize Magnetometer — Optional (build + present + active) */
#ifdef CONFIG_CUBE32_MAG_ENABLED
    if (hw->mag_present && hw->mag_active) {
        ESP_LOGI(TAG, "Mag: Initializing LIS3MDL @ 0x%02X...", hw->mag_i2c_addr);
        cube32_result_t mag_ret = cube32::Magnetometer::instance().begin();
        if (mag_ret != CUBE32_OK) {
            ESP_LOGE(TAG, "Failed to initialize Magnetometer: %d", mag_ret);
            CUBE32_INIT_SET(CUBE32_DRV_MAG, CUBE32_INIT_FAIL);
        } else {
            ESP_LOGI(TAG, "Mag: LIS3MDL initialized successfully");
            CUBE32_INIT_SET(CUBE32_DRV_MAG, CUBE32_INIT_OK);
        }
    } else if (!hw->mag_present) {
        ESP_LOGW(TAG, "Mag: LIS3MDL not detected on I2C bus — skipping");
        CUBE32_INIT_SET(CUBE32_DRV_MAG, CUBE32_INIT_SKIPPED);
    } else {
        ESP_LOGI(TAG, "Mag: Deactivated by user");
        CUBE32_INIT_SET(CUBE32_DRV_MAG, CUBE32_INIT_NOT_REQUESTED);
    }
#else
    ESP_LOGI(TAG, "Mag: Not compiled");
    CUBE32_INIT_SET(CUBE32_DRV_MAG, CUBE32_INIT_NOT_PRESENT);
#endif

    /* Initialize PWM Servo — Optional (build + active) */
#ifdef CONFIG_CUBE32_SERVO_ENABLED
    if (hw->servo_active) {
        ESP_LOGI(TAG, "PWM Servo: Initializing...");
        cube32_result_t servo_ret = cube32::PwmServo::instance().begin();
        if (servo_ret != CUBE32_OK) {
            ESP_LOGE(TAG, "Failed to initialize PWM Servo: %d", servo_ret);
            CUBE32_INIT_SET(CUBE32_DRV_SERVO, CUBE32_INIT_FAIL);
        } else {
            ESP_LOGI(TAG, "PWM Servo: Initialized (%d channels)",
                     cube32::PwmServo::instance().getNumChannels());
            CUBE32_INIT_SET(CUBE32_DRV_SERVO, CUBE32_INIT_OK);
        }

        /* Initialize Robot Head action engine if enabled */
#ifdef CONFIG_CUBE32_ROBOT_HEAD_ENABLED
        ESP_LOGI(TAG, "Robot Head: Initializing...");
        static cube32::PwmServoAdapter s_pwm_adapter;
        cube32_result_t head_ret = cube32::RobotHead::instance().begin(&s_pwm_adapter);
        if (head_ret != CUBE32_OK) {
            ESP_LOGE(TAG, "Failed to initialize Robot Head: %d", head_ret);
            CUBE32_INIT_SET(CUBE32_DRV_ROBOT_HEAD, CUBE32_INIT_FAIL);
        } else {
            ESP_LOGI(TAG, "Robot Head: Initialized");
            CUBE32_INIT_SET(CUBE32_DRV_ROBOT_HEAD, CUBE32_INIT_OK);
        }
#endif
    } else {
        ESP_LOGI(TAG, "PWM Servo: Deactivated by user");
        CUBE32_INIT_SET(CUBE32_DRV_SERVO, CUBE32_INIT_NOT_REQUESTED);
        CUBE32_INIT_SET(CUBE32_DRV_ROBOT_HEAD, CUBE32_INIT_SKIPPED);
    }
#else
    ESP_LOGI(TAG, "PWM Servo: Not compiled");
    CUBE32_INIT_SET(CUBE32_DRV_SERVO, CUBE32_INIT_NOT_PRESENT);
    CUBE32_INIT_SET(CUBE32_DRV_ROBOT_HEAD, CUBE32_INIT_NOT_PRESENT);
#endif

    } // end of non-BLE-OTA driver initialization

    /* Note: Modem initialization now runs asynchronously via beginAsync().
     * The entire modem initialization (power on, USB/UART setup) and 
     * network registration runs in a background task.
     * Applications can use:
     *   - A7670Modem::isInitialized() to check if modem is ready
     *   - A7670Modem::isInitializing() to check if init is in progress
     *   - A7670Modem::isNetworkReady() to check network registration
     *   - Wait for MODEM_INIT_COMPLETE_BIT or MODEM_NETWORK_REGISTERED_BIT events
     */

    /* Note: BLE OTA initialization is done early (before Display/LVGL) to allow
     * NimBLE stack to be unloaded if BLE OTA is not triggered, freeing memory
     * for LVGL double buffering. See BLE OTA section after RTC initialization.
     */

    /* Mark init complete and print the driver init status summary */
    hw->init_done = true;
    cube32_hw_manifest_print_init_status(hw);

    #undef CUBE32_INIT_SET

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "CUBE32 Board initialization complete!");
    ESP_LOGI(TAG, "========================================");

    return ESP_OK;
}
