/**
 * @file cube32.h
 * @brief CUBE32 Board Support Package - Main Header
 * 
 * This is the main include file for CUBE32 BSP. Include this file to access
 * all CUBE32 board functionality including display, touch, audio, camera, etc.
 */

#ifndef CUBE32_H
#define CUBE32_H

#include <esp_err.h>
#include <esp_log.h>

#include "cube32_config.h"
#include "utils/common.h"
#include "utils/i2c_bus.h"
#include "utils/spi_bus.h"

// C++ only headers - must be included outside extern "C"
#ifdef __cplusplus
#include "drivers/pmu/axp2101.h"
#include "drivers/display/st7789.h"
#include "drivers/touch/touch.h"
#ifdef CONFIG_CUBE32_CAMERA_ENABLED
#include "drivers/camera/camera.h"
#endif
#ifdef CONFIG_CUBE32_SDCARD_ENABLED
#include "drivers/sdcard/sdcard.h"
#endif
#ifdef CONFIG_CUBE32_AUDIO_ENABLED
#include "drivers/audio/audio_codec.h"
#endif
#ifdef CONFIG_CUBE32_ADC_BUTTON_ENABLED
#include "drivers/button/adc_button.h"
#endif
#ifdef CONFIG_CUBE32_LVGL_ENABLED
#include "drivers/lvgl/lvgl_driver.h"
#endif
#ifdef CONFIG_CUBE32_RTC_ENABLED
#include "drivers/rtc/bm8563.h"
#endif
#ifdef CONFIG_CUBE32_MODEM_ENABLED
#include "drivers/modem/a7670_modem.h"
#endif
#ifdef CONFIG_CUBE32_IMU_ENABLED
#include "drivers/imu/lsm6dso.h"
#endif
#ifdef CONFIG_CUBE32_MAG_ENABLED
#include "drivers/mag/lis3mdl.h"
#endif
#ifdef CONFIG_CUBE32_USB_INPUT_ENABLED
#include "drivers/usb/usb_input.h"
#endif
#ifdef CONFIG_CUBE32_BLE_OTA_ENABLED
#include "drivers/ble/ble_ota.h"
#include "drivers/ble/ble_ota_ui.h"
#endif
#ifdef CONFIG_CUBE32_SERVO_ENABLED
#include "drivers/servo/pwm_servo.h"
#include "drivers/servo/servo_interface.h"
#include "drivers/servo/pwm_servo_adapter.h"
#endif
#ifdef CONFIG_CUBE32_ROBOT_HEAD_ENABLED
#include "drivers/robot/robot_head.h"
#endif
#ifdef CONFIG_CUBE32_FACE_EXPRESSION_ENABLED
#include "drivers/robot/face_expression.h"
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief CUBE32 BSP version
 */
#define CUBE32_BSP_VERSION_MAJOR 1
#define CUBE32_BSP_VERSION_MINOR 0
#define CUBE32_BSP_VERSION_PATCH 0

/**
 * @brief Initialize CUBE32 board
 * 
 * This function initializes all enabled hardware components based on
 * the configuration in Kconfig. Call this at the beginning of app_main().
 * 
 * Initialization includes:
 * - NVS Flash
 * - Event Loop
 * - Network Interface
 * - Display (if enabled)
 * - Touch (if enabled)
 * - Audio (if enabled)
 * - Camera (if enabled)
 * - SD Card (if enabled)
 * - PMU (if enabled)
 * 
 * @return ESP_OK on success, or an error code on failure
 */
esp_err_t cube32_init(void);

/**
 * @brief Initialize only the core system components
 * 
 * Initializes NVS, Event Loop, and Network Interface.
 * Use this if you want manual control over peripheral initialization.
 * 
 * @return ESP_OK on success, or an error code on failure
 */
esp_err_t cube32_init_core(void);

/**
 * @brief Get CUBE32 BSP version string
 * 
 * @return Version string in format "X.Y.Z"
 */
const char* cube32_get_version(void);

/**
 * @brief Print board information to log
 * 
 * Prints chip info, flash size, heap size, etc.
 */
void cube32_print_board_info(void);

#ifdef __cplusplus
}
#endif

#endif /* CUBE32_H */
