/**
 * @file touch.h
 * @brief CUBE32 Touch Panel Driver
 * 
 * This driver provides support for touch panels on the CUBE32 board:
 * - CST816S for CUBE_TFT_TOUCH_154 (240x240) display
 * - FT6336 (FT5x06 compatible) for CUBE_TFT_TOUCH_200 (240x320) display
 * 
 * The driver uses the shared I2C bus from utils/i2c_bus.h and
 * ESP-IDF's esp_lcd_touch component for efficient touch operations.
 */

#ifndef CUBE32_DRIVERS_TOUCH_TOUCH_H
#define CUBE32_DRIVERS_TOUCH_TOUCH_H

#include "utils/common.h"
#include "utils/i2c_bus.h"
#include "cube32_config.h"

#include <esp_lcd_touch.h>
#include <esp_lcd_panel_io.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Constants
// ============================================================================

/**
 * @brief Touch IC types supported
 */
typedef enum {
    CUBE32_TOUCH_IC_NONE = 0,    ///< No touch IC
    CUBE32_TOUCH_IC_CST816,      ///< CST816S (1.54" display)
    CUBE32_TOUCH_IC_FT6336,      ///< FT6336/FT5x06 (2.0" display)
} cube32_touch_ic_t;

/**
 * @brief Maximum touch points supported
 */
#define CUBE32_TOUCH_MAX_POINTS  1

/**
 * @brief Touch point data structure
 */
typedef struct {
    uint16_t x;         ///< X coordinate
    uint16_t y;         ///< Y coordinate
    uint16_t strength;  ///< Touch strength/pressure (if supported)
    bool pressed;       ///< Touch is pressed
} cube32_touch_point_t;

// ============================================================================
// Configuration Structures
// ============================================================================

/**
 * @brief Touch panel configuration structure
 */
typedef struct {
    // I2C configuration
    int rst_pin;                    ///< Reset GPIO pin (-1 if not used)
    int int_pin;                    ///< Interrupt GPIO pin (-1 if not used)
    
    // Display configuration (for coordinate mapping)
    uint16_t h_res;                 ///< Horizontal resolution
    uint16_t v_res;                 ///< Vertical resolution
    uint16_t rotation;              ///< Rotation angle (0, 90, 180, 270) - should match display
    
    // Touch configuration (auto-calculated from rotation if not explicitly set)
    bool swap_xy;                   ///< Swap X and Y coordinates
    bool mirror_x;                  ///< Mirror X axis
    bool mirror_y;                  ///< Mirror Y axis
    
    // Touch IC type (auto-detected if set to NONE)
    cube32_touch_ic_t ic_type;      ///< Touch IC type
} cube32_touch_config_t;

/**
 * @brief Default touch configuration using pins from cube32_config.h
 *        Rotation starts at 0 (base state) - LVGL will handle rotation
 */
#define CUBE32_TOUCH_ROTATION_DEFAULT 0

#if defined(CONFIG_CUBE32_DISPLAY_CUBE_TFT_TOUCH_154)
#define CUBE32_TOUCH_CONFIG_DEFAULT() { \
    .rst_pin = CUBE32_TOUCH_RST_PIN, \
    .int_pin = CUBE32_TOUCH_INT_PIN, \
    .h_res = 240, \
    .v_res = 240, \
    .rotation = CUBE32_TOUCH_ROTATION_DEFAULT, \
    .swap_xy = false, \
    .mirror_x = false, \
    .mirror_y = false, \
    .ic_type = CUBE32_TOUCH_IC_CST816, \
}
#elif defined(CONFIG_CUBE32_DISPLAY_CUBE_TFT_TOUCH_200)
#define CUBE32_TOUCH_CONFIG_DEFAULT() { \
    .rst_pin = CUBE32_TOUCH_RST_PIN, \
    .int_pin = CUBE32_TOUCH_INT_PIN, \
    .h_res = 240, \
    .v_res = 320, \
    .rotation = CUBE32_TOUCH_ROTATION_DEFAULT, \
    .swap_xy = false, \
    .mirror_x = false, \
    .mirror_y = false, \
    .ic_type = CUBE32_TOUCH_IC_FT6336, \
}
#else
#define CUBE32_TOUCH_CONFIG_DEFAULT() { \
    .rst_pin = CUBE32_TOUCH_RST_PIN, \
    .int_pin = CUBE32_TOUCH_INT_PIN, \
    .h_res = 240, \
    .v_res = 240, \
    .rotation = CUBE32_TOUCH_ROTATION_DEFAULT, \
    .swap_xy = false, \
    .mirror_x = false, \
    .mirror_y = false, \
    .ic_type = CUBE32_TOUCH_IC_CST816, \
}
#endif

// ============================================================================
// C Interface
// ============================================================================

/**
 * @brief Initialize touch with default configuration
 * 
 * @return CUBE32_OK on success, or error code
 */
cube32_result_t cube32_touch_init(void);

/**
 * @brief Initialize touch with custom configuration
 * 
 * @param config Pointer to touch configuration structure
 * @return CUBE32_OK on success, or error code
 */
cube32_result_t cube32_touch_init_config(const cube32_touch_config_t* config);

/**
 * @brief Deinitialize touch
 * 
 * @return CUBE32_OK on success, or error code
 */
cube32_result_t cube32_touch_deinit(void);

/**
 * @brief Check if touch is initialized
 * 
 * @return true if initialized, false otherwise
 */
bool cube32_touch_is_initialized(void);

/**
 * @brief Read touch data
 * 
 * @param point Pointer to touch point data structure to fill
 * @return CUBE32_OK on success, or error code
 */
cube32_result_t cube32_touch_read(cube32_touch_point_t* point);

/**
 * @brief Check if screen is currently being touched
 * 
 * @return true if touched, false otherwise
 */
bool cube32_touch_is_pressed(void);

/**
 * @brief Get the touch IC type
 * 
 * @return Touch IC type
 */
cube32_touch_ic_t cube32_touch_get_ic_type(void);

#ifdef __cplusplus
}

// ============================================================================
// C++ Interface
// ============================================================================

namespace cube32 {

/**
 * @brief Touch Panel Driver Class (Singleton)
 * 
 * Object-oriented interface for the touch panel.
 * Uses the shared I2C bus managed by I2CBus singleton.
 * 
 * Usage:
 * @code
 *   cube32::I2CBus::instance().init();
 *   cube32::Touch& touch = cube32::Touch::instance();
 *   touch.begin();
 *   
 *   cube32_touch_point_t point;
 *   if (touch.read(point) == CUBE32_OK && point.pressed) {
 *       printf("Touch at: %d, %d\n", point.x, point.y);
 *   }
 * @endcode
 */
class Touch {
public:
    /**
     * @brief Get the singleton instance
     */
    static Touch& instance();

    /**
     * @brief Initialize with default configuration
     */
    cube32_result_t begin();

    /**
     * @brief Initialize with custom configuration
     */
    cube32_result_t begin(const cube32_touch_config_t& config);

    /**
     * @brief Deinitialize touch
     */
    cube32_result_t end();

    /**
     * @brief Check if initialized
     */
    bool isInitialized() const { return m_initialized; }

    /**
     * @brief Get the touch handle for LVGL integration
     */
    esp_lcd_touch_handle_t getHandle() const { return m_touch_handle; }

    // ---- Touch Reading ----
    
    /**
     * @brief Read touch data
     * 
     * @param point Reference to touch point to fill
     * @return CUBE32_OK on success
     */
    cube32_result_t read(cube32_touch_point_t& point);

    /**
     * @brief Check if touched
     */
    bool isPressed();

    /**
     * @brief Get X coordinate of last touch
     */
    uint16_t getX() const { return m_last_point.x; }

    /**
     * @brief Get Y coordinate of last touch
     */
    uint16_t getY() const { return m_last_point.y; }

    /**
     * @brief Get touch strength of last touch
     */
    uint16_t getStrength() const { return m_last_point.strength; }

    // ---- Configuration ----

    /**
     * @brief Get touch IC type
     */
    cube32_touch_ic_t getICType() const { return m_config.ic_type; }

    /**
     * @brief Get horizontal resolution
     */
    uint16_t getWidth() const { return m_config.h_res; }

    /**
     * @brief Get vertical resolution
     */
    uint16_t getHeight() const { return m_config.v_res; }

    /**
     * @brief Set coordinate transformation
     * 
     * @param swap_xy Swap X and Y coordinates
     * @param mirror_x Mirror X axis
     * @param mirror_y Mirror Y axis
     */
    cube32_result_t setTransform(bool swap_xy, bool mirror_x, bool mirror_y);

    /**
     * @brief Set rotation angle (updates swap_xy, mirror_x, mirror_y)
     * 
     * @param rotation Rotation angle (0, 90, 180, 270)
     */
    cube32_result_t setRotation(uint16_t rotation);

    // Singleton - no copy/move
    Touch(const Touch&) = delete;
    Touch& operator=(const Touch&) = delete;

private:
    Touch() = default;
    ~Touch();

    cube32_result_t initCST816();
    cube32_result_t initFT6336();

    esp_lcd_touch_handle_t m_touch_handle = nullptr;
    esp_lcd_panel_io_handle_t m_io_handle = nullptr;
    cube32_touch_config_t m_config = {};
    cube32_touch_point_t m_last_point = {};
    bool m_initialized = false;
};

} // namespace cube32

#endif // __cplusplus

#endif // CUBE32_DRIVERS_TOUCH_TOUCH_H
