/**
 * @file st7796.h
 * @brief CUBE32 ST7796S TFT Display Driver
 *
 * This driver provides support for the ST7796S TFT display controller
 * used on the 320x320 GT911-touch display board.
 *
 * The driver mirrors drivers/display/st7789.h's public API (duck-typed,
 * no shared base class) so callers such as cube32.cpp and
 * drivers/lvgl/lvgl_driver.cpp can dispatch between the two display
 * drivers at runtime based on the auto-detected display model
 * (see cube32_display_ic_t in st7789.h and CUBE32_DISPLAY_MODEL_TABLE
 * in cube32_config.h).
 *
 * NOTE: displayOn()/displayOff()/setBacklight() issue the generic
 * esp_lcd_panel_disp_on_off()/GPIO backlight commands, but do NOT call
 * into PMU::setDisplayBacklight() — the backlight/power-rail wiring for
 * this board has not been finalized yet. setPrismMode() likewise only
 * writes the MADCTL register (no board-specific control involved).
 */

#ifndef CUBE32_DRIVERS_DISPLAY_ST7796_H
#define CUBE32_DRIVERS_DISPLAY_ST7796_H

#include "utils/common.h"
#include "utils/spi_bus.h"
#include "cube32_config.h"
#include "drivers/display/st7789.h"  // reuses cube32_display_ic_t, MADCTL bit macros, cube32_rgb565()/color macros

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
 * @brief ST7796S display default parameters
 */
#define CUBE32_ST7796_LCD_CMD_BITS    8
#define CUBE32_ST7796_LCD_PARAM_BITS  8
#define CUBE32_ST7796_LCD_BIT_DEPTH   16

/**
 * @brief ST7796S default SPI pixel clock.
 *
 * Deliberately lower than CUBE32_LCD_PIXEL_CLK_HZ (40MHz, tuned/validated
 * on the ST7789 boards' wiring). On first hardware bring-up this panel
 * showed color corruption + vertical banding at 40MHz — a classic SPI
 * signal-integrity symptom (bit errors from clock speed vs. wiring/cable
 * length) — so this board defaults to a more conservative 10MHz. Raise
 * this back toward 40MHz once the wiring/signal integrity is validated.
 */
#define CUBE32_ST7796_PIXEL_CLK_HZ    (40 * 1000 * 1000)

// ============================================================================
// Configuration Structures
// ============================================================================

/**
 * @brief ST7796S display configuration structure
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
    uint16_t x_gap;                 ///< X offset/gap
    uint16_t y_gap;                 ///< Y offset/gap
    uint16_t rotation;              ///< Display rotation (0, 90, 180, 270)
    bool mirror_x;                  ///< Mirror X axis
    bool mirror_y;                  ///< Mirror Y axis
    bool swap_xy;                   ///< Swap X and Y axis
    bool invert_color;               ///< Invert colors
    bool bgr_order;                  ///< Use BGR color order instead of RGB

    // Backlight configuration
    uint8_t bl_on_level;            ///< Backlight on level (1 = active high, 0 = active low)
} cube32_st7796_config_t;

/**
 * @brief Default ST7796S configuration using the same SPI/CS pins as the
 *        existing TFT display (see cube32_config.h). Resolution is
 *        auto-detected/overridden by the caller (see cube32.cpp) before
 *        begin(), exactly like CUBE32_ST7789_CONFIG_DEFAULT().
 *
 * All of the settings below are CONFIRMED against the panel vendor's own
 * init sequence (BOE3.92IPS(GV039Z2Q-N80)-ST7796U-2.2Gamma-20211203.INI),
 * validated via the standalone apps/LCD_ST7796_Test raw bring-up test:
 *
 * - invert_color = true — matches the vendor's Display Inversion ON (0x21)
 *   command baked into s_vendor_init_cmds (st7796.cpp).
 * - bgr_order = true — matches the vendor's MADCTL (0x36) = 0x48, which has
 *   the BGR bit set. A clean color-bar/corner-square test with the vendor
 *   init sequence confirmed correct hues (RED shows red, GREEN shows
 *   green, etc.) with this setting.
 * - y_gap = 0 — the vendor init's Display Function Control (0xB6) command
 *   sets the gate line count (NL) to exactly 320. The ESP-IDF
 *   esp_lcd_st7796 driver's built-in default init sequence never sends
 *   0xB6 at all, leaving the panel at its power-on-default gate count
 *   (very likely 480, the common ST7796 native GRAM) — THAT was the real
 *   root cause of the earlier "top half shows static" bug, not a
 *   fundamental 480-row hardware limit. With 0xB6 now baked into
 *   s_vendor_init_cmds (st7796.cpp), no y_gap workaround is needed.
 * - mirror_x/mirror_y/swap_xy = false — this is the LOGICAL "no rotation"
 *   request. The vendor's MADCTL=0x48 bakes in MX=1 as this panel's native
 *   "book" orientation, so ST7796Display wraps the panel handle in an
 *   orientation-correction shim (see st7796.cpp) that transparently
 *   inverts the MX bit on every mirror() call — including esp_lvgl_port's
 *   — so these logical false/false/false values still mean "normal
 *   reading orientation" on screen.
 */
#define CUBE32_ST7796_CONFIG_DEFAULT() { \
    .cs_pin = CUBE32_LCD_CS_PIN, \
    .dc_pin = CUBE32_LCD_DC_PIN, \
    .rst_pin = CUBE32_LCD_RST_PIN, \
    .bl_pin = CUBE32_LCD_BL_PIN, \
    .pixel_clock_hz = CUBE32_ST7796_PIXEL_CLK_HZ, \
    .h_res = 320, \
    .v_res = 320, \
    .x_gap = 0, \
    .y_gap = 0, \
    .rotation = 0, \
    .mirror_x = false, \
    .mirror_y = false, \
    .swap_xy = false, \
    .invert_color = true, \
    .bgr_order = true, \
    .bl_on_level = 1, \
}

#ifdef __cplusplus
} // extern "C"

// ============================================================================
// C++ Interface
// ============================================================================

namespace cube32 {

/**
 * @brief ST7796S Display Driver Class (Singleton)
 *
 * Public API intentionally mirrors ST7789Display method-for-method so
 * callers can dispatch between the two at runtime without a shared base
 * class (see file header note).
 *
 * Usage:
 * @code
 *   cube32::SPIBus::instance().init();
 *   cube32::ST7796Display& display = cube32::ST7796Display::instance();
 *   display.begin();
 *
 *   display.clear(CUBE32_COLOR_BLACK);
 *   display.fillRect(10, 10, 50, 50, CUBE32_COLOR_RED);
 * @endcode
 */
class ST7796Display {
public:
    /**
     * @brief Get the singleton instance
     */
    static ST7796Display& instance();

    /**
     * @brief Initialize with default configuration
     */
    cube32_result_t begin();

    /**
     * @brief Initialize with custom configuration
     */
    cube32_result_t begin(const cube32_st7796_config_t& config);

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
     * @brief Turn on display.
     *
     * @note Issues the generic esp_lcd_panel_disp_on_off() command only.
     * PMU/backlight-rail control for this board is not implemented yet
     * (board control logic TBD) — see file header note.
     */
    cube32_result_t displayOn();

    /**
     * @brief Turn off display (see displayOn() note).
     */
    cube32_result_t displayOff();

    /**
     * @brief Set backlight brightness (0-100).
     *
     * @note Drives CUBE32_LCD_BL_PIN directly if configured (mirrors
     * ST7789Display behavior); no PMU power-rail control (board control
     * logic TBD for this board) — see file header note.
     */
    cube32_result_t setBacklight(uint8_t brightness_percent);

    /**
     * @brief Set display rotation (0, 90, 180, 270)
     */
    cube32_result_t setRotation(uint16_t rotation);

    /**
     * @brief Enable or disable prism/mirror mode (MADCTL MX bit toggle).
     *
     * @param enable true to enable horizontal mirroring, false to disable
     * @param effective_rotation The actual hardware rotation (0, 90, 180, 270).
     *        When using LVGL, pass the LVGL rotation value. Use 0xFFFF to use
     *        the stored configuration rotation (for non-LVGL use cases).
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
    ST7796Display(const ST7796Display&) = delete;
    ST7796Display& operator=(const ST7796Display&) = delete;

private:
    ST7796Display() = default;
    ~ST7796Display();

    cube32_result_t initBacklight();
    cube32_result_t initReset();
    cube32_result_t initPanelIO();
    cube32_result_t initPanel();
    void autoDisplayOn();  ///< Turn on display after first successful draw (once only)

    /** Signal completion of a queued SPI color transfer. */
    static bool onColorTransDone(esp_lcd_panel_io_handle_t io,
                                 esp_lcd_panel_io_event_data_t* edata,
                                 void* user_ctx);

    /** Queue a color transfer and wait before the caller may reuse its buffer. */
    esp_err_t drawBitmapAndWait(uint16_t x_start, uint16_t y_start,
                                uint16_t x_end, uint16_t y_end,
                                const void* data);

    esp_lcd_panel_handle_t m_panel_handle = nullptr;
    esp_lcd_panel_io_handle_t m_io_handle = nullptr;
    void* m_trans_sem = nullptr;  ///< SemaphoreHandle_t; opaque to keep FreeRTOS out of this header
    cube32_st7796_config_t m_config = {};
    uint16_t m_effective_width = 0;   ///< Effective width after rotation
    uint16_t m_effective_height = 0;  ///< Effective height after rotation
    bool m_initialized = false;
    bool m_prism_mode = false;        ///< Prism/mirror mode state
    bool m_display_on = false;        ///< Tracks whether displayOn() has been called
};

} // namespace cube32

#endif // __cplusplus

#endif // CUBE32_DRIVERS_DISPLAY_ST7796_H
