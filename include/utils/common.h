/**
 * @file common.h
 * @brief CUBE32 Common Types and Definitions
 * 
 * This file contains common types, enums, and result codes used across
 * all CUBE32 drivers.
 */

#ifndef CUBE32_UTILS_COMMON_H
#define CUBE32_UTILS_COMMON_H

#include <esp_err.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Result codes for CUBE32 driver operations
 */
typedef enum {
    CUBE32_OK = 0,              ///< Operation successful
    CUBE32_ERROR,               ///< Generic error
    CUBE32_NOT_SUPPORTED,       ///< Feature not supported
    CUBE32_NOT_INITIALIZED,     ///< Driver not initialized
    CUBE32_ALREADY_INITIALIZED, ///< Driver already initialized
    CUBE32_INVALID_ARG,         ///< Invalid argument
    CUBE32_NO_MEM,              ///< Memory allocation failed
    CUBE32_TIMEOUT,             ///< Operation timed out
    CUBE32_NOT_FOUND,           ///< Device not found
    CUBE32_IO_ERROR,            ///< I/O communication error
    CUBE32_BUSY,                ///< Resource is busy
} cube32_result_t;

/**
 * @brief Driver initialization codes for detailed status reporting
 */
typedef enum {
    CUBE32_INIT_NOT_REQUESTED = 0,  ///< Runtime mask said "no"
    CUBE32_INIT_NOT_PRESENT,        ///< Hardware feature disabled at compile time
    CUBE32_INIT_SKIPPED,            ///< Dependency not ready
    CUBE32_INIT_OK,                 ///< Initialization successful
    CUBE32_INIT_FAIL,               ///< Initialization failed
    CUBE32_INIT_INITIALIZING,       ///< Async initialization in progress
} cube32_init_code_t;

/**
 * @brief Driver index for init status tracking
 *
 * Each value corresponds to an index in the init_records[] array
 * stored in the hardware manifest.
 */
typedef enum {
    CUBE32_DRV_PMU = 0,
    CUBE32_DRV_RTC,
    CUBE32_DRV_DISPLAY,
    CUBE32_DRV_LVGL,
    CUBE32_DRV_TOUCH,
    CUBE32_DRV_AUDIO,
    CUBE32_DRV_MODEM,
    CUBE32_DRV_CAMERA,
    CUBE32_DRV_SDCARD,
    CUBE32_DRV_BLE_OTA,
    CUBE32_DRV_USB_INPUT,
    CUBE32_DRV_ADC_BUTTON,
    CUBE32_DRV_SERVO,
    CUBE32_DRV_ROBOT_HEAD,
    CUBE32_DRV_IMU,
    CUBE32_DRV_MAG,
    CUBE32_DRV_COUNT,           ///< Number of tracked drivers (sentinel)
} cube32_driver_index_t;

/**
 * @brief Driver activation bitmask
 */
typedef enum {
    CUBE32_DRIVER_PMU       = (1u << 0),
    CUBE32_DRIVER_DISPLAY   = (1u << 1),
    CUBE32_DRIVER_TOUCH     = (1u << 2),
    CUBE32_DRIVER_LVGL      = (1u << 3),
    CUBE32_DRIVER_SDCARD    = (1u << 4),
    CUBE32_DRIVER_CAMERA    = (1u << 5),
    CUBE32_DRIVER_AUDIO     = (1u << 6),
    CUBE32_DRIVER_MODEM     = (1u << 7),
    CUBE32_DRIVER_IMU       = (1u << 8),
    CUBE32_DRIVER_IO_EXP    = (1u << 9),
    CUBE32_DRIVER_ALL       = 0xFFFFFFFFu,
} cube32_driver_t;

/**
 * @brief Driver initialization record for status tracking
 */
typedef struct {
    bool present;                ///< Hardware feature present (compile time)
    bool requested;              ///< Requested in activation mask
    cube32_init_code_t code;     ///< Initialization result code
    const char* error_msg;       ///< Optional error message
} cube32_init_record_t;

/**
 * @brief Convert CUBE32 result to ESP-IDF error code
 */
static inline esp_err_t cube32_to_esp_err(cube32_result_t result) {
    switch (result) {
        case CUBE32_OK:                  return ESP_OK;
        case CUBE32_INVALID_ARG:         return ESP_ERR_INVALID_ARG;
        case CUBE32_NO_MEM:              return ESP_ERR_NO_MEM;
        case CUBE32_TIMEOUT:             return ESP_ERR_TIMEOUT;
        case CUBE32_NOT_FOUND:           return ESP_ERR_NOT_FOUND;
        case CUBE32_NOT_SUPPORTED:       return ESP_ERR_NOT_SUPPORTED;
        case CUBE32_ALREADY_INITIALIZED: return ESP_OK;
        default:                         return ESP_FAIL;
    }
}

/**
 * @brief Convert ESP-IDF error code to CUBE32 result
 */
static inline cube32_result_t esp_err_to_cube32(esp_err_t err) {
    switch (err) {
        case ESP_OK:              return CUBE32_OK;
        case ESP_ERR_INVALID_ARG: return CUBE32_INVALID_ARG;
        case ESP_ERR_NO_MEM:      return CUBE32_NO_MEM;
        case ESP_ERR_TIMEOUT:     return CUBE32_TIMEOUT;
        case ESP_ERR_NOT_FOUND:   return CUBE32_NOT_FOUND;
        case ESP_ERR_NOT_SUPPORTED: return CUBE32_NOT_SUPPORTED;
        default:                  return CUBE32_ERROR;
    }
}

#ifdef __cplusplus
}
#endif

#endif // CUBE32_UTILS_COMMON_H
