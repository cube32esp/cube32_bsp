/**
 * @file touch.cpp
 * @brief CUBE32 Touch Panel Driver Implementation
 */

#include "drivers/touch/touch.h"
#include <esp_log.h>
#include <driver/gpio.h>
#include <string.h>

// Include touch IC specific headers
#include <esp_lcd_touch_cst816s.h>
#include <esp_lcd_touch_ft5x06.h>

static const char* TAG = "cube32_touch";

namespace cube32 {

// ============================================================================
// Singleton Implementation
// ============================================================================

Touch& Touch::instance() {
    static Touch s_instance;
    return s_instance;
}

Touch::~Touch() {
    if (m_initialized) {
        end();
    }
}

cube32_result_t Touch::begin() {
    cube32_touch_config_t config = CUBE32_TOUCH_CONFIG_DEFAULT();
    return begin(config);
}

cube32_result_t Touch::begin(const cube32_touch_config_t& config) {
    if (m_initialized) {
        ESP_LOGW(TAG, "Touch already initialized");
        return CUBE32_ALREADY_INITIALIZED;
    }

    // Store configuration
    m_config = config;

    // Apply rotation transformation to swap_xy, mirror_x, mirror_y
    // This must match the display driver's rotation logic
    switch (m_config.rotation) {
        case 0:
            m_config.swap_xy = false;
            m_config.mirror_x = false;
            m_config.mirror_y = false;
            break;
        case 90:
            m_config.swap_xy = true;
            m_config.mirror_x = false;
            m_config.mirror_y = true;
            break;
        case 180:
            m_config.swap_xy = false;
            m_config.mirror_x = true;
            m_config.mirror_y = true;
            break;
        case 270:
            m_config.swap_xy = true;
            m_config.mirror_x = true;
            m_config.mirror_y = false;
            break;
        default:
            ESP_LOGW(TAG, "Invalid rotation %d, using 0", m_config.rotation);
            m_config.rotation = 0;
            m_config.swap_xy = false;
            m_config.mirror_x = false;
            m_config.mirror_y = false;
            break;
    }

    ESP_LOGI(TAG, "Initializing touch panel...");
    ESP_LOGI(TAG, "  Resolution: %dx%d", config.h_res, config.v_res);
    ESP_LOGI(TAG, "  RST: GPIO%d, INT: GPIO%d", config.rst_pin, config.int_pin);
    ESP_LOGI(TAG, "  Rotation: %d (swap_xy=%d, mirror_x=%d, mirror_y=%d)",
             m_config.rotation, m_config.swap_xy, m_config.mirror_x, m_config.mirror_y);

    // Ensure I2C bus is initialized
    if (!I2CBus::instance().isInitialized()) {
        ESP_LOGI(TAG, "Initializing I2C bus...");
        cube32_result_t ret = I2CBus::instance().init();
        if (ret != CUBE32_OK) {
            ESP_LOGE(TAG, "Failed to initialize I2C bus: %d", ret);
            return ret;
        }
    }

    cube32_result_t ret;

    // Initialize based on IC type
    switch (m_config.ic_type) {
        case CUBE32_TOUCH_IC_CST816:
            ESP_LOGI(TAG, "  IC Type: CST816S");
            ret = initCST816();
            break;
        case CUBE32_TOUCH_IC_FT6336:
            ESP_LOGI(TAG, "  IC Type: FT6336 (FT5x06)");
            ret = initFT6336();
            break;
        default:
            ESP_LOGE(TAG, "Unknown touch IC type: %d", m_config.ic_type);
            return CUBE32_NOT_SUPPORTED;
    }

    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to initialize touch IC");
        return ret;
    }

    m_initialized = true;
    ESP_LOGI(TAG, "Touch panel initialized successfully");

    return CUBE32_OK;
}

cube32_result_t Touch::initCST816() {
    // Create I2C panel IO for CST816S
    // Note: ESP_LCD_TOUCH_IO_I2C_CST816S_CONFIG() has out-of-order designated
    // initializers (scl_speed_hz before dev_addr) which is a C++20 error.
    // Manually initialize in struct declaration order instead.
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr           = ESP_LCD_TOUCH_IO_I2C_CST816S_ADDRESS,
        .on_color_trans_done = nullptr,
        .user_ctx           = nullptr,
        .control_phase_bytes = 1,
        .dc_bit_offset      = 0,
        .lcd_cmd_bits       = 8,
        .lcd_param_bits     = 0,
        .flags = {
            .dc_low_on_data         = 0,
            .disable_control_phase  = 1,
        },
        .scl_speed_hz = 400000,
    };
    
    // Note: Using shared I2C bus - increase timeout for concurrent access
    // This helps when audio codec is using I2C simultaneously
    
    esp_err_t ret = esp_lcd_new_panel_io_i2c(
        I2CBus::instance().getHandle(),
        &io_config,
        &m_io_handle
    );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C panel IO: %s", esp_err_to_name(ret));
        return esp_err_to_cube32(ret);
    }

    // Configure touch panel
    // Note: x_max/y_max should be the max coordinate value (resolution - 1)
    // for correct mirror calculations in esp_lcd_touch
    esp_lcd_touch_config_t tp_cfg = {
        .x_max = (uint16_t)(m_config.h_res - 1),
        .y_max = (uint16_t)(m_config.v_res - 1),
        .rst_gpio_num = (gpio_num_t)m_config.rst_pin,
        .int_gpio_num = (gpio_num_t)m_config.int_pin,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = m_config.swap_xy ? 1u : 0u,
            .mirror_x = m_config.mirror_x ? 1u : 0u,
            .mirror_y = m_config.mirror_y ? 1u : 0u,
        },
        .process_coordinates = nullptr,
        .interrupt_callback = nullptr,
        .user_data = nullptr,
        .driver_data = nullptr,
    };

    ret = esp_lcd_touch_new_i2c_cst816s(m_io_handle, &tp_cfg, &m_touch_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create CST816S touch: %s", esp_err_to_name(ret));
        esp_lcd_panel_io_del(m_io_handle);
        m_io_handle = nullptr;
        return esp_err_to_cube32(ret);
    }

    return CUBE32_OK;
}

cube32_result_t Touch::initFT6336() {
    // Create I2C panel IO for FT6336/FT5x06
    esp_lcd_panel_io_i2c_config_t io_config = ESP_LCD_TOUCH_IO_I2C_FT5x06_CONFIG();
    io_config.scl_speed_hz = 400000;  // Match I2C bus speed
    
    // Note: Using shared I2C bus - increase timeout for concurrent access
    // This helps when audio codec is using I2C simultaneously
    
    esp_err_t ret = esp_lcd_new_panel_io_i2c(
        I2CBus::instance().getHandle(),
        &io_config,
        &m_io_handle
    );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C panel IO: %s", esp_err_to_name(ret));
        return esp_err_to_cube32(ret);
    }

    // Configure touch panel
    // Note: x_max/y_max should be the max coordinate value (resolution - 1)
    // for correct mirror calculations in esp_lcd_touch
    esp_lcd_touch_config_t tp_config = {
        .x_max = (uint16_t)(m_config.h_res - 1),
        .y_max = (uint16_t)(m_config.v_res - 1),
        .rst_gpio_num = (gpio_num_t)m_config.rst_pin,
        .int_gpio_num = (gpio_num_t)m_config.int_pin,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = m_config.swap_xy ? 1u : 0u,
            .mirror_x = m_config.mirror_x ? 1u : 0u,
            .mirror_y = m_config.mirror_y ? 1u : 0u,
        },
        .process_coordinates = nullptr,
        .interrupt_callback = nullptr,
        .user_data = nullptr,
        .driver_data = nullptr,
    };

    ret = esp_lcd_touch_new_i2c_ft5x06(m_io_handle, &tp_config, &m_touch_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create FT6336 touch: %s", esp_err_to_name(ret));
        esp_lcd_panel_io_del(m_io_handle);
        m_io_handle = nullptr;
        return esp_err_to_cube32(ret);
    }

    return CUBE32_OK;
}

cube32_result_t Touch::end() {
    if (!m_initialized) {
        return CUBE32_NOT_INITIALIZED;
    }

    if (m_touch_handle) {
        esp_lcd_touch_del(m_touch_handle);
        m_touch_handle = nullptr;
    }

    if (m_io_handle) {
        esp_lcd_panel_io_del(m_io_handle);
        m_io_handle = nullptr;
    }

    m_initialized = false;
    ESP_LOGI(TAG, "Touch panel deinitialized");
    return CUBE32_OK;
}

cube32_result_t Touch::read(cube32_touch_point_t& point) {
    if (!m_initialized || !m_touch_handle) {
        return CUBE32_NOT_INITIALIZED;
    }

    // Read touch data from controller
    // Note: I2C errors can occur when sharing bus with audio codec and other devices
    // We handle errors gracefully by returning last known state
    esp_err_t ret = esp_lcd_touch_read_data(m_touch_handle);

    // Get touch coordinates using the new API
    esp_lcd_touch_point_data_t touch_data[CUBE32_TOUCH_MAX_POINTS];
    uint8_t point_num = 0;

    if (ret == ESP_OK) {
        ret = esp_lcd_touch_get_data(
            m_touch_handle,
            touch_data,
            &point_num,
            CUBE32_TOUCH_MAX_POINTS
        );
    }

    // If we got valid touch data, use it
    if (ret == ESP_OK && point_num > 0) {
        point.x = touch_data[0].x;
        point.y = touch_data[0].y;
        point.strength = touch_data[0].strength;
        point.pressed = true;
        
        // Store last valid point
        m_last_point = point;
    } else {
        // On I2C error or no touch, return last known state (not pressed)
        // This prevents LVGL from getting stuck due to temporary I2C bus conflicts
        point.x = m_last_point.x;
        point.y = m_last_point.y;
        point.strength = 0;
        point.pressed = false;
    }

    // Always return OK to LVGL - don't propagate I2C errors
    // LVGL polls continuously and temporary I2C conflicts are expected
    return CUBE32_OK;
}

bool Touch::isPressed() {
    cube32_touch_point_t point;
    if (read(point) == CUBE32_OK) {
        return point.pressed;
    }
    return false;
}

cube32_result_t Touch::setTransform(bool swap_xy, bool mirror_x, bool mirror_y) {
    if (!m_initialized || !m_touch_handle) {
        return CUBE32_NOT_INITIALIZED;
    }

    m_config.swap_xy = swap_xy;
    m_config.mirror_x = mirror_x;
    m_config.mirror_y = mirror_y;

    // Update touch controller configuration
    esp_lcd_touch_set_swap_xy(m_touch_handle, swap_xy);
    esp_lcd_touch_set_mirror_x(m_touch_handle, mirror_x);
    esp_lcd_touch_set_mirror_y(m_touch_handle, mirror_y);

    // Verify the flags were actually set
    bool verify_swap = false, verify_mx = false, verify_my = false;
    esp_lcd_touch_get_swap_xy(m_touch_handle, &verify_swap);
    esp_lcd_touch_get_mirror_x(m_touch_handle, &verify_mx);
    esp_lcd_touch_get_mirror_y(m_touch_handle, &verify_my);
    ESP_LOGI(TAG, "Touch transform verified: swap_xy=%d, mirror_x=%d, mirror_y=%d", 
             verify_swap, verify_mx, verify_my);

    return CUBE32_OK;
}

cube32_result_t Touch::setRotation(uint16_t rotation) {
    if (!m_initialized || !m_touch_handle) {
        return CUBE32_NOT_INITIALIZED;
    }

    // Calculate transform based on rotation
    // Transform order in esp_lcd_touch: mirror_x, mirror_y, then swap_xy
    // We need to map physical touch coordinates to LVGL coordinates
    // 
    // At 0°: LVGL matches physical orientation
    // At 90° CW: LVGL X = physical Y, LVGL Y = (physical_max_x - physical X)
    // At 180°: LVGL X = (physical_max_x - physical X), LVGL Y = (physical_max_y - physical Y)
    // At 270° CW: LVGL X = (physical_max_y - physical Y), LVGL Y = physical X
    bool swap_xy = false;
    bool mirror_x = false;
    bool mirror_y = false;

    switch (rotation) {
        case 0:
            swap_xy = false;
            mirror_x = false;
            mirror_y = false;
            break;
        case 90:
            // Physical (0,319) → LVGL (0,0), Physical (239,0) → LVGL (319,239)
            // Formula: LVGL_x = y_max - py, LVGL_y = px
            // Transform order: mx, my, swap → need my=true to invert Y before swap
            swap_xy = true;
            mirror_x = false;
            mirror_y = true;
            break;
        case 180:
            // Both X and Y inverted, no swap
            swap_xy = false;
            mirror_x = true;
            mirror_y = true;
            break;
        case 270:
            // Physical (239,0) → LVGL (0,0), Physical (0,319) → LVGL (319,239)
            // Formula: LVGL_x = py, LVGL_y = x_max - px
            // Transform order: mx, my, swap → need mx=true to invert X before swap
            swap_xy = true;
            mirror_x = true;
            mirror_y = false;
            break;
        default:
            ESP_LOGW(TAG, "Invalid touch rotation %d, using 0", rotation);
            rotation = 0;
            break;
    }

    ESP_LOGI(TAG, "Setting touch rotation to %d (swap_xy=%d, mirror_x=%d, mirror_y=%d)",
             rotation, swap_xy, mirror_x, mirror_y);

    m_config.rotation = rotation;
    cube32_result_t ret = setTransform(swap_xy, mirror_x, mirror_y);
    
    // Verify the transform was applied
    if (m_touch_handle) {
        bool cur_swap, cur_mx, cur_my;
        esp_lcd_touch_get_swap_xy(m_touch_handle, &cur_swap);
        esp_lcd_touch_get_mirror_x(m_touch_handle, &cur_mx);
        esp_lcd_touch_get_mirror_y(m_touch_handle, &cur_my);
        ESP_LOGI(TAG, "Touch transform verified: swap_xy=%d, mirror_x=%d, mirror_y=%d",
                 cur_swap, cur_mx, cur_my);
    }
    
    return ret;
}

} // namespace cube32

// ============================================================================
// C Interface Implementation
// ============================================================================

extern "C" {

cube32_result_t cube32_touch_init(void) {
    return cube32::Touch::instance().begin();
}

cube32_result_t cube32_touch_init_config(const cube32_touch_config_t* config) {
    if (!config) {
        return CUBE32_INVALID_ARG;
    }
    return cube32::Touch::instance().begin(*config);
}

cube32_result_t cube32_touch_deinit(void) {
    return cube32::Touch::instance().end();
}

bool cube32_touch_is_initialized(void) {
    return cube32::Touch::instance().isInitialized();
}

cube32_result_t cube32_touch_read(cube32_touch_point_t* point) {
    if (!point) {
        return CUBE32_INVALID_ARG;
    }
    return cube32::Touch::instance().read(*point);
}

bool cube32_touch_is_pressed(void) {
    return cube32::Touch::instance().isPressed();
}

cube32_touch_ic_t cube32_touch_get_ic_type(void) {
    return cube32::Touch::instance().getICType();
}

} // extern "C"
