/**
 * @file st7789.h
 * @brief CUBE32 ST7789 TFT Display Driver
 * 
 * This driver provides support for the ST7789 TFT display controller
 * commonly used with 240x240 and 240x320 resolution displays.
 * 
 * The driver uses the shared SPI bus from utils/spi_bus.h and
 * ESP-IDF's esp_lcd component for efficient display operations.
 */

#ifndef CUBE32_DRIVERS_DISPLAY_ST7789_H
#define CUBE32_DRIVERS_DISPLAY_ST7789_H

#include "utils/common.h"
#include "utils/spi_bus.h"
#include "cube32_config.h"

#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Constants
// ============================================================================

/**
 * @brief ST7789 display default parameters
 */
#define CUBE32_ST7789_LCD_CMD_BITS    8
#define CUBE32_ST7789_LCD_PARAM_BITS  8
#define CUBE32_ST7789_LCD_BIT_DEPTH   16

/**
 * @brief ST7789 MADCTL register definitions for mirror/rotation control
 */
#define ST7789_MADCTL       0x36  ///< Memory Data Access Control register
#define ST7789_MADCTL_MY    0x80  ///< Row Address Order (Y-Mirror)
#define ST7789_MADCTL_MX    0x40  ///< Column Address Order (X-Mirror)
#define ST7789_MADCTL_MV    0x20  ///< Row/Column Exchange
#define ST7789_MADCTL_ML    0x10  ///< Vertical Refresh Order
#define ST7789_MADCTL_BGR   0x08  ///< BGR color order (vs RGB)

// ============================================================================
// Configuration Structures
// ============================================================================

/**
 * @brief ST7789 display configuration structure
 */
typedef struct {
    // SPI configuration
    int cs_pin;                     ///< Chip select GPIO pin
    int dc_pin;                     ///< Data/Command GPIO pin
    int rst_pin;                    ///< Reset GPIO pin (-1 if not used)
    int bl_pin;                     ///< Backlight GPIO pin (-1 if not used)
    uint32_t pixel_clock_hz;        ///< SPI clock frequency for pixel transfer
    
    // Display configuration
    uint16_t h_res;                 ///< Horizontal resolution
    uint16_t v_res;                 ///< Vertical resolution
    uint16_t x_gap;                 ///< X offset/gap (for displays smaller than frame buffer)
    uint16_t y_gap;                 ///< Y offset/gap (for displays smaller than frame buffer)
    uint16_t rotation;              ///< Display rotation (0, 90, 180, 270)
    bool mirror_x;                  ///< Mirror X axis
    bool mirror_y;                  ///< Mirror Y axis
    bool swap_xy;                   ///< Swap X and Y axis
    bool invert_color;              ///< Invert colors
    bool bgr_order;                 ///< Use BGR color order instead of RGB
    
    // Backlight configuration
    uint8_t bl_on_level;            ///< Backlight on level (1 = active high, 0 = active low)
} cube32_st7789_config_t;

/**
 * @brief Default ST7789 configuration using pins from cube32_config.h
 *
 * Rotation defaults to 0; actual rotation is applied from NVS config at runtime.
 */
#define CUBE32_ST7789_CONFIG_DEFAULT() { \
    .cs_pin = CUBE32_LCD_CS_PIN, \
    .dc_pin = CUBE32_LCD_DC_PIN, \
    .rst_pin = CUBE32_LCD_RST_PIN, \
    .bl_pin = CUBE32_LCD_BL_PIN, \
    .pixel_clock_hz = CUBE32_LCD_PIXEL_CLK_HZ, \
    .h_res = CUBE32_LCD_H_RES, \
    .v_res = CUBE32_LCD_V_RES, \
    .x_gap = 0, \
    .y_gap = 0, \
    .rotation = 0, \
    .mirror_x = false, \
    .mirror_y = false, \
    .swap_xy = false, \
    .invert_color = true, \
    .bgr_order = false, \
    .bl_on_level = 1, \
}

/**
 * @brief Convert RGB888 to RGB565
 * 
 * @param r Red value (0-255)
 * @param g Green value (0-255)
 * @param b Blue value (0-255)
 * @return RGB565 color value
 */
static inline uint16_t cube32_rgb565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

// Common colors in RGB565 format
#define CUBE32_COLOR_BLACK       0x0000
#define CUBE32_COLOR_WHITE       0xFFFF
#define CUBE32_COLOR_RED         0xF800
#define CUBE32_COLOR_GREEN       0x07E0
#define CUBE32_COLOR_BLUE        0x001F
#define CUBE32_COLOR_YELLOW      0xFFE0
#define CUBE32_COLOR_CYAN        0x07FF
#define CUBE32_COLOR_MAGENTA     0xF81F
#define CUBE32_COLOR_ORANGE      0xFD20
#define CUBE32_COLOR_PURPLE      0x8010
#define CUBE32_COLOR_GRAY        0x8410
#define CUBE32_COLOR_DARK_GRAY   0x4208
#define CUBE32_COLOR_LIGHT_GRAY  0xC618

#ifdef __cplusplus
} // extern "C"

// ============================================================================
// C++ Interface
// ============================================================================

namespace cube32 {

/**
 * @brief ST7789 Display Driver Class (Singleton)
 * 
 * Object-oriented interface for the ST7789 TFT display.
 * Uses the shared SPI bus managed by SPIBus singleton.
 * 
 * Usage:
 * @code
 *   cube32::SPIBus::instance().init();
 *   cube32::ST7789Display& display = cube32::ST7789Display::instance();
 *   display.begin();
 *   
 *   display.clear(CUBE32_COLOR_BLACK);
 *   display.fillRect(10, 10, 50, 50, CUBE32_COLOR_RED);
 * @endcode
 */
class ST7789Display {
public:
    /**
     * @brief Get the singleton instance
     */
    static ST7789Display& instance();

    /**
     * @brief Initialize with default configuration
     */
    cube32_result_t begin();

    /**
     * @brief Initialize with custom configuration
     */
    cube32_result_t begin(const cube32_st7789_config_t& config);

    /**
     * @brief Deinitialize the display
     */
    cube32_result_t end();

    /**
     * @brief Check if initialized
     */
    bool isInitialized() const { return m_initialized; }

    /**
     * @brief Get the panel handle
     */
    esp_lcd_panel_handle_t getPanelHandle() const { return m_panel_handle; }

    /**
     * @brief Get the panel IO handle
     */
    esp_lcd_panel_io_handle_t getIOHandle() const { return m_io_handle; }

    // ---- Display Control ----
    
    /**
     * @brief Turn on display
     */
    cube32_result_t displayOn();

    /**
     * @brief Turn off display
     */
    cube32_result_t displayOff();

    /**
     * @brief Set backlight brightness (0-100)
     */
    cube32_result_t setBacklight(uint8_t brightness_percent);

    /**
     * @brief Set display rotation (0, 90, 180, 270)
     */
    cube32_result_t setRotation(uint16_t rotation);

    /**
     * @brief Enable or disable prism/mirror mode
     * 
     * Prism mode horizontally mirrors the display output, which is useful
     * for HUD projections where the image is reflected on a prism or glass.
     * This directly writes to the ST7789 MADCTL register to toggle the MX bit.
     * 
     * @param enable true to enable horizontal mirroring, false to disable
     * @param effective_rotation The actual hardware rotation (0, 90, 180, 270).
     *        When using LVGL, pass the LVGL rotation value. Use 0xFFFF to use
     *        the stored configuration rotation (for non-LVGL use cases).
     * @return CUBE32_OK on success
     */
    cube32_result_t setPrismMode(bool enable, uint16_t effective_rotation = 0xFFFF);

    /**
     * @brief Check if prism mode is currently enabled
     */
    bool isPrismMode() const { return m_prism_mode; }

    // ---- Drawing Functions ----

    /**
     * @brief Clear display with a color
     */
    cube32_result_t clear(uint16_t color = CUBE32_COLOR_BLACK);

    /**
     * @brief Draw a single pixel
     */
    cube32_result_t drawPixel(uint16_t x, uint16_t y, uint16_t color);

    /**
     * @brief Fill a rectangle with color
     */
    cube32_result_t fillRect(uint16_t x_start, uint16_t y_start,
                             uint16_t x_end, uint16_t y_end, uint16_t color);

    /**
     * @brief Draw bitmap data
     */
    cube32_result_t drawBitmap(uint16_t x_start, uint16_t y_start,
                               uint16_t x_end, uint16_t y_end, const void* data);

    /**
     * @brief Get display width (accounts for rotation)
     */
    uint16_t getWidth() const { return m_effective_width; }

    /**
     * @brief Get display height (accounts for rotation)
     */
    uint16_t getHeight() const { return m_effective_height; }

    /**
     * @brief Get base display width (ignores rotation)
     */
    uint16_t getBaseWidth() const { return m_config.h_res; }

    /**
     * @brief Get base display height (ignores rotation)
     */
    uint16_t getBaseHeight() const { return m_config.v_res; }

    /**
     * @brief Get current rotation angle
     */
    uint16_t getRotation() const { return m_config.rotation; }

    /**
     * @brief Check if X/Y are currently swapped (rotation 90 or 270)
     */
    bool isSwapXY() const { return (m_config.rotation == 90 || m_config.rotation == 270); }

    /**
     * @brief Check if X is currently mirrored
     */
    bool isMirrorX() const { return (m_config.rotation == 180 || m_config.rotation == 270); }

    /**
     * @brief Check if Y is currently mirrored
     */
    bool isMirrorY() const { return (m_config.rotation == 90 || m_config.rotation == 180); }

    // Singleton - no copy/move
    ST7789Display(const ST7789Display&) = delete;
    ST7789Display& operator=(const ST7789Display&) = delete;

private:
    ST7789Display() = default;
    ~ST7789Display();

    cube32_result_t initBacklight();
    cube32_result_t initReset();
    cube32_result_t initPanelIO();
    cube32_result_t initPanel();

    esp_lcd_panel_handle_t m_panel_handle = nullptr;
    esp_lcd_panel_io_handle_t m_io_handle = nullptr;
    cube32_st7789_config_t m_config = {};
    uint16_t m_effective_width = 0;   ///< Effective width after rotation
    uint16_t m_effective_height = 0;  ///< Effective height after rotation
    bool m_initialized = false;
    bool m_prism_mode = false;        ///< Prism/mirror mode state
};

} // namespace cube32

#endif // __cplusplus

#endif // CUBE32_DRIVERS_DISPLAY_ST7789_H
