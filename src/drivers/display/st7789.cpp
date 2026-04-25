/**
 * @file st7789.cpp
 * @brief CUBE32 ST7789 TFT Display Driver Implementation
 */

#include "drivers/display/st7789.h"
#include "drivers/pmu/axp2101.h"
#include <esp_log.h>
#include <esp_heap_caps.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <string.h>

static const char* TAG = "cube32_st7789";

namespace cube32 {

// ============================================================================
// Singleton Implementation
// ============================================================================

ST7789Display& ST7789Display::instance() {
    static ST7789Display s_instance;
    return s_instance;
}

ST7789Display::~ST7789Display() {
    if (m_initialized) {
        end();
    }
}

cube32_result_t ST7789Display::begin() {
    cube32_st7789_config_t config = CUBE32_ST7789_CONFIG_DEFAULT();
    return begin(config);
}

cube32_result_t ST7789Display::begin(const cube32_st7789_config_t& config) {
    if (m_initialized) {
        ESP_LOGW(TAG, "ST7789 display already initialized");
        return CUBE32_ALREADY_INITIALIZED;
    }

    // Store configuration
    m_config = config;

    ESP_LOGI(TAG, "Initializing ST7789 display...");
    ESP_LOGI(TAG, "  Resolution: %dx%d", config.h_res, config.v_res);
    ESP_LOGI(TAG, "  CS: GPIO%d, DC: GPIO%d, RST: GPIO%d, BL: GPIO%d",
             config.cs_pin, config.dc_pin, config.rst_pin, config.bl_pin);

    // Ensure SPI bus is initialized
    if (!SPIBus::instance().isInitialized()) {
        ESP_LOGI(TAG, "Initializing SPI bus...");
        cube32_result_t ret = SPIBus::instance().init();
        if (ret != CUBE32_OK) {
            ESP_LOGE(TAG, "Failed to initialize SPI bus: %d", ret);
            return ret;
        }
    }

    cube32_result_t ret;

    // Initialize backlight GPIO
    ret = initBacklight();
    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to initialize backlight");
        return ret;
    }

    // Initialize reset GPIO and perform hardware reset
    ret = initReset();
    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to initialize reset");
        return ret;
    }

    // Initialize panel IO
    ret = initPanelIO();
    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to initialize panel IO");
        return ret;
    }

    // Initialize panel
    ret = initPanel();
    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to initialize panel");
        return ret;
    }

    m_initialized = true;
    ESP_LOGI(TAG, "ST7789 display initialized successfully");

    // Note: Prism mode is NOT applied here because:
    // - When using LVGL, LVGL resets the display rotation and handles it via esp_lvgl_port
    // - LVGL's LvglDisplay::setRotation() will apply prism mode after setting rotation
    // - For non-LVGL use, call setPrismMode() manually after initialization

    // Note: Display panel and backlight are kept OFF here to prevent white flash
    // If using LVGL: LVGL driver will turn them on after first frame is drawn
    // If not using LVGL: Call displayOn() to turn on display and backlight

    return CUBE32_OK;
}

cube32_result_t ST7789Display::initBacklight() {
    if (m_config.bl_pin < 0) {
        ESP_LOGI(TAG, "Backlight pin not configured, skipping");
        return CUBE32_OK;
    }

    gpio_config_t bl_gpio_config = {
        .pin_bit_mask = (1ULL << m_config.bl_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t ret = gpio_config(&bl_gpio_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure backlight GPIO: %s", esp_err_to_name(ret));
        return esp_err_to_cube32(ret);
    }

    // Start with backlight off
    gpio_set_level((gpio_num_t)m_config.bl_pin, !m_config.bl_on_level);

    return CUBE32_OK;
}

cube32_result_t ST7789Display::initReset() {
    if (m_config.rst_pin < 0) {
        ESP_LOGI(TAG, "Reset pin not configured, skipping hardware reset");
        return CUBE32_OK;
    }

    gpio_config_t rst_gpio_config = {
        .pin_bit_mask = (1ULL << m_config.rst_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t ret = gpio_config(&rst_gpio_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure reset GPIO: %s", esp_err_to_name(ret));
        return esp_err_to_cube32(ret);
    }

    // Perform hardware reset
    ESP_LOGI(TAG, "Performing hardware reset...");
    gpio_set_level((gpio_num_t)m_config.rst_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level((gpio_num_t)m_config.rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(120));

    return CUBE32_OK;
}

cube32_result_t ST7789Display::initPanelIO() {
    spi_host_device_t spi_host = SPIBus::instance().getHost();

    esp_lcd_panel_io_spi_config_t io_config = {
        .cs_gpio_num = m_config.cs_pin,
        .dc_gpio_num = m_config.dc_pin,
        .spi_mode = 0,
        .pclk_hz = m_config.pixel_clock_hz,
        .trans_queue_depth = 10,
        .on_color_trans_done = nullptr,
        .user_ctx = nullptr,
        .lcd_cmd_bits = CUBE32_ST7789_LCD_CMD_BITS,
        .lcd_param_bits = CUBE32_ST7789_LCD_PARAM_BITS,
        .flags = {
            .dc_low_on_data = 0,
            .octal_mode = 0,
            .quad_mode = 0,
            .sio_mode = 0,
            .lsb_first = 0,
            .cs_high_active = 0,
        },
    };

    esp_err_t ret = esp_lcd_new_panel_io_spi(spi_host, &io_config, &m_io_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create panel IO: %s", esp_err_to_name(ret));
        return esp_err_to_cube32(ret);
    }

    return CUBE32_OK;
}

cube32_result_t ST7789Display::initPanel() {
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = m_config.rst_pin,
        .rgb_ele_order = m_config.bgr_order ? LCD_RGB_ELEMENT_ORDER_BGR : LCD_RGB_ELEMENT_ORDER_RGB,
        .data_endian = LCD_RGB_DATA_ENDIAN_LITTLE,
        .bits_per_pixel = CUBE32_ST7789_LCD_BIT_DEPTH,
        .flags = {
            .reset_active_high = 0,
        },
        .vendor_config = nullptr,
    };

    esp_err_t ret = esp_lcd_new_panel_st7789(m_io_handle, &panel_config, &m_panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create ST7789 panel: %s", esp_err_to_name(ret));
        return esp_err_to_cube32(ret);
    }

    // Reset panel
    ret = esp_lcd_panel_reset(m_panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset panel: %s", esp_err_to_name(ret));
        return esp_err_to_cube32(ret);
    }

    // Initialize panel
    ret = esp_lcd_panel_init(m_panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize panel: %s", esp_err_to_name(ret));
        return esp_err_to_cube32(ret);
    }

    // Calculate mirror, swap, and gap settings based on rotation
    // ST7789 controller typically has 320x240 internal frame buffer
    // For 240x240 displays, there's an 80-pixel offset that changes with rotation
    bool mirror_x = m_config.mirror_x;
    bool mirror_y = m_config.mirror_y;
    bool swap_xy = m_config.swap_xy;
    uint16_t x_gap = m_config.x_gap;
    uint16_t y_gap = m_config.y_gap;

    // Calculate the offset for displays smaller than the internal buffer
    // ST7789 internal buffer: 320x240
    // For 240x240 display: offset = 320 - 240 = 80
    // For 240x320 display: offset = 0 (uses full buffer)
    uint16_t offset = 0;
    if (m_config.h_res == 240 && m_config.v_res == 240) {
        offset = 80;  // ST7789 internal is 320x240, display is 240x240
    }

    switch (m_config.rotation) {
        case 0:
            mirror_x = false;
            mirror_y = false;
            swap_xy = false;
            x_gap = m_config.x_gap;
            y_gap = m_config.y_gap;
            break;
        case 90:
            mirror_x = false;
            mirror_y = true;
            swap_xy = true;
            x_gap = m_config.y_gap + offset;
            y_gap = m_config.x_gap;
            break;
        case 180:
            mirror_x = true;
            mirror_y = true;
            swap_xy = false;
            x_gap = m_config.x_gap;
            y_gap = m_config.y_gap + offset;  // Apply offset to y
            break;
        case 270:
            mirror_x = true;
            mirror_y = false;
            swap_xy = true;
            x_gap = m_config.y_gap;
            y_gap = m_config.x_gap;
            break;
        default:
            ESP_LOGW(TAG, "Invalid rotation %d, using 0", m_config.rotation);
            break;
    }

    // Update effective dimensions based on swap_xy
    if (swap_xy) {
        m_effective_width = m_config.v_res;
        m_effective_height = m_config.h_res;
    } else {
        m_effective_width = m_config.h_res;
        m_effective_height = m_config.v_res;
    }

    ESP_LOGI(TAG, "  Rotation: %d (mirror_x=%d, mirror_y=%d, swap_xy=%d, x_gap=%d, y_gap=%d, offset=%d)",
             m_config.rotation, mirror_x, mirror_y, swap_xy, x_gap, y_gap, offset);
    ESP_LOGI(TAG, "  Effective resolution: %dx%d", m_effective_width, m_effective_height);

    // Configure panel orientation
    ret = esp_lcd_panel_mirror(m_panel_handle, mirror_x, mirror_y);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set mirror: %s", esp_err_to_name(ret));
    }

    ret = esp_lcd_panel_swap_xy(m_panel_handle, swap_xy);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set swap_xy: %s", esp_err_to_name(ret));
    }

    // Set gap/offset for displays with frame buffer larger than visible area
    ret = esp_lcd_panel_set_gap(m_panel_handle, x_gap, y_gap);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set gap: %s", esp_err_to_name(ret));
    }

    // Invert colors if needed (some displays need this)
    ret = esp_lcd_panel_invert_color(m_panel_handle, m_config.invert_color);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set invert color: %s", esp_err_to_name(ret));
    }

    // Note: Display panel is kept OFF here to prevent white flash during boot
    // The LVGL driver will turn it on after drawing the first frame
    // esp_lcd_panel_disp_on_off(m_panel_handle, true);

    return CUBE32_OK;
}

cube32_result_t ST7789Display::end() {
    if (!m_initialized) {
        return CUBE32_NOT_INITIALIZED;
    }

    // Turn off backlight
    setBacklight(0);

    // Delete panel
    if (m_panel_handle) {
        esp_lcd_panel_del(m_panel_handle);
        m_panel_handle = nullptr;
    }

    // Delete panel IO
    if (m_io_handle) {
        esp_lcd_panel_io_del(m_io_handle);
        m_io_handle = nullptr;
    }

    m_initialized = false;
    ESP_LOGI(TAG, "ST7789 display deinitialized");
    return CUBE32_OK;
}

cube32_result_t ST7789Display::displayOn() {
    if (!m_initialized || !m_panel_handle) {
        return CUBE32_NOT_INITIALIZED;
    }

    esp_err_t ret = esp_lcd_panel_disp_on_off(m_panel_handle, true);
    if (ret != ESP_OK) {
        return esp_err_to_cube32(ret);
    }
    
    // CUBE32 uses AXP2101 PMU ALDO3 for LCD backlight control
    PMU& pmu = PMU::instance();
    if (pmu.isInitialized()) {
        pmu.setDisplayBacklight(true);
    } else {
        ESP_LOGW(TAG, "PMU not initialized, backlight may not work");
    }
    
    return CUBE32_OK;
}

cube32_result_t ST7789Display::displayOff() {
    if (!m_initialized || !m_panel_handle) {
        return CUBE32_NOT_INITIALIZED;
    }

    // CUBE32 uses AXP2101 PMU ALDO3 for LCD backlight control
    PMU& pmu = PMU::instance();
    if (pmu.isInitialized()) {
        pmu.setDisplayBacklight(false);
    }
    
    esp_err_t ret = esp_lcd_panel_disp_on_off(m_panel_handle, false);
    return esp_err_to_cube32(ret);
}

cube32_result_t ST7789Display::setBacklight(uint8_t brightness_percent) {
    if (m_config.bl_pin < 0) {
        return CUBE32_OK; // No backlight pin configured
    }

    // Simple on/off control - for PWM control, use LEDC driver
    int level = (brightness_percent > 0) ? m_config.bl_on_level : !m_config.bl_on_level;
    gpio_set_level((gpio_num_t)m_config.bl_pin, level);
    
    return CUBE32_OK;
}

cube32_result_t ST7789Display::setRotation(uint16_t rotation) {
    if (!m_initialized || !m_panel_handle) {
        return CUBE32_NOT_INITIALIZED;
    }

    bool mirror_x = false;
    bool mirror_y = false;
    bool swap_xy = false;
    uint16_t x_gap = m_config.x_gap;
    uint16_t y_gap = m_config.y_gap;

    // Calculate the offset for displays smaller than the internal buffer
    // ST7789 internal buffer: 320x240
    // For 240x240 display: offset = 320 - 240 = 80
    // For 240x320 display: offset = 0 (uses full buffer)
    uint16_t offset = 0;
    if (m_config.h_res == 240 && m_config.v_res == 240) {
        offset = 80;  // ST7789 internal is 320x240, display is 240x240
    }

    switch (rotation) {
        case 0:
            mirror_x = false;
            mirror_y = false;
            swap_xy = false;
            x_gap = m_config.x_gap;
            y_gap = m_config.y_gap;
            ESP_LOGI(TAG, "==> Rotate 0");
            break;
        case 90:
            mirror_x = false;
            mirror_y = true;
            swap_xy = true;
            x_gap = m_config.y_gap + offset;
            y_gap = m_config.x_gap;
            ESP_LOGI(TAG, "==> Rotate 90");
            break;
        case 180:
            mirror_x = true;
            mirror_y = true;
            swap_xy = false;
            x_gap = m_config.x_gap;
            y_gap = m_config.y_gap + offset;
            ESP_LOGI(TAG, "==> Rotate 180");
            break;
        case 270:
            mirror_x = true;
            mirror_y = false;
            swap_xy = true;
            x_gap = m_config.y_gap;
            y_gap = m_config.x_gap;
            ESP_LOGI(TAG, "==> Rotate 270");
            break;
        default:
            ESP_LOGW(TAG, "Invalid rotation: %d", rotation);
            return CUBE32_INVALID_ARG;
    }

    // Update effective dimensions based on swap_xy
    if (swap_xy) {
        m_effective_width = m_config.v_res;
        m_effective_height = m_config.h_res;
    } else {
        m_effective_width = m_config.h_res;
        m_effective_height = m_config.v_res;
    }

    ESP_LOGI(TAG, "setRotation: %d (mirror_x=%d, mirror_y=%d, swap_xy=%d, x_gap=%d, y_gap=%d, offset=%d)",
             rotation, mirror_x, mirror_y, swap_xy, x_gap, y_gap, offset);

    esp_err_t ret = esp_lcd_panel_mirror(m_panel_handle, mirror_x, mirror_y);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set mirror: %s", esp_err_to_name(ret));
        return esp_err_to_cube32(ret);
    }

    ret = esp_lcd_panel_swap_xy(m_panel_handle, swap_xy);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set swap_xy: %s", esp_err_to_name(ret));
        return esp_err_to_cube32(ret);
    }

    // Set gap/offset for displays with frame buffer larger than visible area
    ret = esp_lcd_panel_set_gap(m_panel_handle, x_gap, y_gap);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set gap: %s", esp_err_to_name(ret));
        return esp_err_to_cube32(ret);
    }

    m_config.rotation = rotation;
    return CUBE32_OK;
}

cube32_result_t ST7789Display::setPrismMode(bool enable, uint16_t effective_rotation) {
    if (!m_initialized || !m_io_handle) {
        return CUBE32_NOT_INITIALIZED;
    }

    // Prism mode toggles the horizontal mirror (MX bit) by directly writing to MADCTL register.
    // We must include all settings (mirror, swap_xy, BGR) in the MADCTL value.
    //
    // MADCTL bit layout:
    // Bit 7 (MY):  Row Address Order
    // Bit 6 (MX):  Column Address Order  
    // Bit 5 (MV):  Row/Column Exchange (swap_xy)
    // Bit 3 (BGR): RGB-BGR Order
    //
    // When using LVGL, the effective_rotation parameter represents the actual hardware
    // rotation that LVGL/esp_lvgl_port has set on the display.
    //
    // Hardware states for each rotation (as set by esp_lvgl_port):
    // Rotation 0:   mirror_x=0, mirror_y=0, swap_xy=0 -> MADCTL = BGR = 0x08
    // Rotation 90:  mirror_x=0, mirror_y=1, swap_xy=1 -> MADCTL = MY | MV | BGR = 0xA8
    // Rotation 180: mirror_x=1, mirror_y=1, swap_xy=0 -> MADCTL = MX | MY | BGR = 0xC8
    // Rotation 270: mirror_x=1, mirror_y=0, swap_xy=1 -> MADCTL = MX | MV | BGR = 0x68
    
    // Use effective_rotation if provided, otherwise use stored config rotation
    uint16_t rotation = (effective_rotation != 0xFFFF) ? effective_rotation : m_config.rotation;
    
    uint8_t madctl = ST7789_MADCTL_BGR;  // Base with BGR color order

    switch (rotation) {
        case 0:
            // mirror_x=0, mirror_y=0, swap_xy=0
            madctl = ST7789_MADCTL_BGR;  // 0x08
            if (enable) {
                madctl |= ST7789_MADCTL_MX;  // Add MX for prism -> 0x48
            }
            break;
        case 90:
            // mirror_x=0, mirror_y=1, swap_xy=1
            madctl = ST7789_MADCTL_MY | ST7789_MADCTL_MV | ST7789_MADCTL_BGR;  // 0xA8
            if (enable) {
                madctl |= ST7789_MADCTL_MX;  // Add MX for prism -> 0xE8
            }
            break;
        case 180:
            // mirror_x=1, mirror_y=1, swap_xy=0
            madctl = ST7789_MADCTL_MX | ST7789_MADCTL_MY | ST7789_MADCTL_BGR;  // 0xC8
            if (enable) {
                madctl &= ~ST7789_MADCTL_MX;  // Remove MX for prism -> 0x88
            }
            break;
        case 270:
            // mirror_x=1, mirror_y=0, swap_xy=1
            madctl = ST7789_MADCTL_MX | ST7789_MADCTL_MV | ST7789_MADCTL_BGR;  // 0x68
            if (enable) {
                madctl &= ~ST7789_MADCTL_MX;  // Remove MX for prism -> 0x28
            }
            break;
        default:
            ESP_LOGW(TAG, "Unknown rotation %d for prism mode", rotation);
            break;
    }
    
    // Write MADCTL register directly via panel IO
    esp_err_t ret = esp_lcd_panel_io_tx_param(m_io_handle, ST7789_MADCTL, &madctl, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write MADCTL register: %s", esp_err_to_name(ret));
        return esp_err_to_cube32(ret);
    }
    
    m_prism_mode = enable;
    ESP_LOGI(TAG, "Prism mode %s (MADCTL=0x%02X, rotation=%d)", 
             enable ? "ENABLED" : "DISABLED", madctl, rotation);
    
    return CUBE32_OK;
}

cube32_result_t ST7789Display::clear(uint16_t color) {
    return fillRect(0, 0, m_effective_width, m_effective_height, color);
}

cube32_result_t ST7789Display::drawPixel(uint16_t x, uint16_t y, uint16_t color) {
    if (!m_initialized || !m_panel_handle) {
        return CUBE32_NOT_INITIALIZED;
    }

    if (x >= m_effective_width || y >= m_effective_height) {
        return CUBE32_INVALID_ARG;
    }

    esp_err_t ret = esp_lcd_panel_draw_bitmap(m_panel_handle, x, y, x + 1, y + 1, &color);
    return esp_err_to_cube32(ret);
}

cube32_result_t ST7789Display::fillRect(uint16_t x_start, uint16_t y_start,
                                         uint16_t x_end, uint16_t y_end,
                                         uint16_t color) {
    if (!m_initialized || !m_panel_handle) {
        return CUBE32_NOT_INITIALIZED;
    }

    // Clamp coordinates
    if (x_end > m_effective_width) x_end = m_effective_width;
    if (y_end > m_effective_height) y_end = m_effective_height;
    if (x_start >= x_end || y_start >= y_end) {
        return CUBE32_INVALID_ARG;
    }

    uint16_t width = x_end - x_start;
    uint16_t height = y_end - y_start;
    size_t buffer_size = width * height * sizeof(uint16_t);

    // Allocate buffer in DMA-capable memory
    uint16_t* buffer = (uint16_t*)heap_caps_malloc(buffer_size, MALLOC_CAP_DMA);
    if (!buffer) {
        ESP_LOGE(TAG, "Failed to allocate fill buffer (%zu bytes)", buffer_size);
        return CUBE32_NO_MEM;
    }

    // Fill buffer with color
    for (size_t i = 0; i < width * height; i++) {
        buffer[i] = color;
    }

    esp_err_t ret = esp_lcd_panel_draw_bitmap(m_panel_handle, x_start, y_start, x_end, y_end, buffer);
    
    heap_caps_free(buffer);
    return esp_err_to_cube32(ret);
}

cube32_result_t ST7789Display::drawBitmap(uint16_t x_start, uint16_t y_start,
                                           uint16_t x_end, uint16_t y_end,
                                           const void* data) {
    if (!m_initialized || !m_panel_handle) {
        return CUBE32_NOT_INITIALIZED;
    }

    if (!data) {
        return CUBE32_INVALID_ARG;
    }

    esp_err_t ret = esp_lcd_panel_draw_bitmap(m_panel_handle, x_start, y_start, x_end, y_end, data);
    return esp_err_to_cube32(ret);
}

} // namespace cube32
