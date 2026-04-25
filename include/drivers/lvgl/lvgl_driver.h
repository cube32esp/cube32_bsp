/**
 * @file lvgl_driver.h
 * @brief CUBE32 LVGL Display Driver
 * 
 * This driver integrates LVGL with the ST7789 display driver
 * using esp_lvgl_port component for efficient display operations.
 * 
 * The driver uses the ST7789 panel handle and creates an LVGL display
 * with appropriate buffer configuration.
 */

#ifndef CUBE32_DRIVERS_LVGL_LVGL_DRIVER_H
#define CUBE32_DRIVERS_LVGL_LVGL_DRIVER_H

#include "utils/common.h"
#include "drivers/display/st7789.h"
#ifdef CONFIG_CUBE32_TOUCH_ENABLED
#include "drivers/touch/touch.h"
#endif
#ifdef CONFIG_CUBE32_USB_INPUT_ENABLED
#include "drivers/usb/usb_input.h"
#endif

#include <esp_lvgl_port.h>
#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Constants
// ============================================================================

/**
 * @brief LVGL default task parameters
 */
#define CUBE32_LVGL_TASK_PRIORITY       4
#define CUBE32_LVGL_TASK_STACK_SIZE     (8 * 1024)
#define CUBE32_LVGL_TASK_MAX_SLEEP_MS   500
#define CUBE32_LVGL_TIMER_PERIOD_MS     5
#define CUBE32_LVGL_BUFFER_SIZE_PERCENT 20  // Buffer size as percentage of screen

// ============================================================================
// Configuration Structures
// ============================================================================

/**
 * @brief LVGL driver configuration structure
 */
typedef struct {
    // Task configuration
    int task_priority;              ///< LVGL task priority
    int task_stack_size;            ///< LVGL task stack size
    int task_affinity;              ///< LVGL task pinned to core (-1 is no affinity)
    int task_max_sleep_ms;          ///< Maximum sleep in LVGL task
    int timer_period_ms;            ///< LVGL timer tick period in ms
    
    // Display buffer configuration
    uint32_t buffer_size;           ///< Custom buffer size (0 = auto calculate)
    bool double_buffer;             ///< Use double buffering
    bool use_spiram;                ///< Allocate buffers in SPIRAM
    bool swap_bytes;                ///< Swap bytes in RGB565 color format
    bool full_refresh;              ///< Always refresh the whole screen
    bool direct_mode;               ///< Use direct mode (screen-sized buffers)
} cube32_lvgl_config_t;

/**
 * @brief Default LVGL configuration
 */
#define CUBE32_LVGL_CONFIG_DEFAULT() { \
    .task_priority = CUBE32_LVGL_TASK_PRIORITY, \
    .task_stack_size = CUBE32_LVGL_TASK_STACK_SIZE, \
    .task_affinity = -1, \
    .task_max_sleep_ms = CUBE32_LVGL_TASK_MAX_SLEEP_MS, \
    .timer_period_ms = CUBE32_LVGL_TIMER_PERIOD_MS, \
    .buffer_size = 0, \
    .double_buffer = true, \
    .use_spiram = false, \
    .swap_bytes = false, \
    .full_refresh = false, \
    .direct_mode = false, \
}

#ifdef __cplusplus
} // extern "C"

// ============================================================================
// C++ Interface
// ============================================================================

namespace cube32 {

/**
 * @brief LVGL Display Driver Class (Singleton)
 * 
 * Object-oriented interface for LVGL display integration.
 * Uses the ST7789Display singleton and esp_lvgl_port for efficient
 * display updates.
 * 
 * Usage:
 * @code
 *   // Initialize ST7789 display first
 *   cube32::ST7789Display::instance().begin();
 *   
 *   // Initialize LVGL
 *   cube32::LvglDisplay& lvgl = cube32::LvglDisplay::instance();
 *   lvgl.begin();
 *   
 *   // Use LVGL as usual, with mutex protection
 *   if (lvgl.lock(1000)) {
 *       lv_obj_t *label = lv_label_create(lv_screen_active());
 *       lv_label_set_text(label, "Hello CUBE32!");
 *       lvgl.unlock();
 *   }
 * @endcode
 */
class LvglDisplay {
public:
    /**
     * @brief Get the singleton instance
     */
    static LvglDisplay& instance();

    /**
     * @brief Initialize with default configuration
     * 
     * @note ST7789Display must be initialized before calling this
     */
    cube32_result_t begin();

    /**
     * @brief Initialize with custom configuration
     * 
     * @note ST7789Display must be initialized before calling this
     */
    cube32_result_t begin(const cube32_lvgl_config_t& config);

    /**
     * @brief Deinitialize LVGL
     */
    cube32_result_t end();

    /**
     * @brief Check if initialized
     */
    bool isInitialized() const { return m_initialized; }

    /**
     * @brief Get the LVGL display handle
     */
    lv_display_t* getDisplay() const { return m_display; }

    // ---- Thread Safety ----

    /**
     * @brief Lock LVGL mutex for thread-safe operations
     * 
     * @param timeout_ms Timeout in milliseconds (0 = block indefinitely)
     * @return true if mutex was acquired, false if timeout
     */
    bool lock(uint32_t timeout_ms = 0);

    /**
     * @brief Unlock LVGL mutex
     */
    void unlock();

    // ---- Display Control ----

    /**
     * @brief Get display width
     */
    uint16_t getWidth() const { return m_width; }

    /**
     * @brief Get display height
     */
    uint16_t getHeight() const { return m_height; }

    /**
     * @brief Set display rotation
     * 
     * @param rotation Rotation in degrees (0, 90, 180, 270)
     */
    cube32_result_t setRotation(uint16_t rotation);

    /**
     * @brief Get current display rotation
     * 
     * @return Rotation in degrees (0, 90, 180, 270)
     */
    uint16_t getRotation() const;

    // ---- Touch Input ----

    /**
     * @brief Add touch input to LVGL (call after display is initialized)
     * 
     * @note Touch driver must be initialized before calling this
     * @return CUBE32_OK on success, error code on failure
     */
    cube32_result_t addTouch();

    /**
     * @brief Remove touch input from LVGL
     */
    cube32_result_t removeTouch();

    /**
     * @brief Check if touch is added
     */
    bool hasTouchInput() const { return m_touch_indev != nullptr; }

    /**
     * @brief Get the LVGL touch input device handle
     */
    lv_indev_t* getTouchIndev() const { return m_touch_indev; }

    // ---- USB Mouse Input ----

    /**
     * @brief Add USB mouse input to LVGL (call after display and USB Input are initialized)
     * 
     * @note USB Input driver must be initialized before calling this
     * @return CUBE32_OK on success, error code on failure
     */
    cube32_result_t addUsbMouse();

    /**
     * @brief Remove USB mouse input from LVGL
     */
    cube32_result_t removeUsbMouse();

    /**
     * @brief Check if USB mouse is added
     */
    bool hasUsbMouseInput() const { return m_usb_mouse_indev != nullptr; }

    /**
     * @brief Get the LVGL USB mouse input device handle
     */
    lv_indev_t* getUsbMouseIndev() const { return m_usb_mouse_indev; }

    /**
     * @brief Get current USB mouse X position
     */
    int16_t getUsbMouseX() const { return m_usb_mouse_x; }

    /**
     * @brief Get current USB mouse Y position
     */
    int16_t getUsbMouseY() const { return m_usb_mouse_y; }

    /**
     * @brief Process USB mouse events (called by internal callback)
     * @note This is called internally by the USB mouse read callback
     */
    void processUsbMouseEvents(lv_indev_data_t *data);

    // Singleton - no copy/move
    LvglDisplay(const LvglDisplay&) = delete;
    LvglDisplay& operator=(const LvglDisplay&) = delete;

private:
    LvglDisplay() = default;
    ~LvglDisplay();

    cube32_result_t initLvglPort(const cube32_lvgl_config_t& config);
    cube32_result_t addDisplay(const cube32_lvgl_config_t& config);

    lv_display_t* m_display = nullptr;
    lv_indev_t* m_touch_indev = nullptr;
    lv_indev_t* m_usb_mouse_indev = nullptr;
    lv_obj_t* m_usb_mouse_cursor = nullptr;
    int16_t m_usb_mouse_x = 0;
    int16_t m_usb_mouse_y = 0;
    uint8_t m_usb_mouse_buttons = 0;
    cube32_lvgl_config_t m_config = {};
    uint16_t m_width = 0;
    uint16_t m_height = 0;
    bool m_lvgl_port_initialized = false;
    bool m_initialized = false;
};

} // namespace cube32

#endif // __cplusplus

#endif // CUBE32_DRIVERS_LVGL_LVGL_DRIVER_H
