/**
 * @file i2c_bus.cpp
 * @brief CUBE32 Shared I2C Bus Manager Implementation
 */

#include "utils/i2c_bus.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

static const char* TAG = "cube32_i2c";

namespace cube32 {

// ============================================================================
// Singleton Implementation
// ============================================================================

I2CBus& I2CBus::instance() {
    static I2CBus s_instance;
    return s_instance;
}

I2CBus::~I2CBus() {
    if (m_initialized) {
        deinit();
    }
}

cube32_result_t I2CBus::init() {
    cube32_i2c_config_t config = CUBE32_I2C_CONFIG_DEFAULT();
    return init(config);
}

cube32_result_t I2CBus::init(const cube32_i2c_config_t& config) {
    if (m_initialized) {
        ESP_LOGW(TAG, "I2C bus already initialized");
        return CUBE32_ALREADY_INITIALIZED;
    }

    ESP_LOGI(TAG, "Initializing shared I2C bus...");
    ESP_LOGI(TAG, "  SDA: GPIO%d, SCL: GPIO%d, Freq: %lu Hz",
             config.sda_pin, config.scl_pin, config.freq_hz);

    // Configure I2C master bus
    // Note: trans_queue_depth = 0 for synchronous operation required by esp_lcd_panel_io_i2c
    // The touch driver uses esp_lcd_panel_io which is incompatible with async I2C transactions
    // Error handling in individual drivers (touch, audio) prevents conflicts on shared bus
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = config.port,
        .sda_io_num = (gpio_num_t)config.sda_pin,
        .scl_io_num = (gpio_num_t)config.scl_pin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,  // Must be 0 for esp_lcd_panel_io compatibility
        .flags = {
            .enable_internal_pullup = config.enable_pullup ? 1u : 0u,
        },
    };

    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &m_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
        return esp_err_to_cube32(ret);
    }

    m_port = config.port;
    m_initialized = true;

    ESP_LOGI(TAG, "I2C bus initialized successfully");
    return CUBE32_OK;
}

cube32_result_t I2CBus::deinit() {
    if (!m_initialized) {
        return CUBE32_NOT_INITIALIZED;
    }

    if (m_bus_handle) {
        esp_err_t ret = i2c_del_master_bus(m_bus_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to delete I2C bus: %s", esp_err_to_name(ret));
            return esp_err_to_cube32(ret);
        }
        m_bus_handle = nullptr;
    }

    m_initialized = false;
    ESP_LOGI(TAG, "I2C bus deinitialized");
    return CUBE32_OK;
}

cube32_result_t I2CBus::probe(uint8_t addr) {
    if (!m_initialized || !m_bus_handle) {
        return CUBE32_NOT_INITIALIZED;
    }

    esp_err_t ret = i2c_master_probe(m_bus_handle, addr, pdMS_TO_TICKS(100));
    if (ret == ESP_OK) {
        return CUBE32_OK;
    } else if (ret == ESP_ERR_NOT_FOUND) {
        return CUBE32_NOT_FOUND;
    }
    return esp_err_to_cube32(ret);
}

void I2CBus::scan() {
    if (!m_initialized) {
        ESP_LOGW(TAG, "I2C bus not initialized");
        return;
    }

    ESP_LOGI(TAG, "Scanning I2C bus...");
    int found = 0;
    
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (probe(addr) == CUBE32_OK) {
            ESP_LOGI(TAG, "  Found device at 0x%02X", addr);
            found++;
        }
    }

    ESP_LOGI(TAG, "I2C scan complete: %d device(s) found", found);
}

} // namespace cube32

// ============================================================================
// C Interface Implementation
// ============================================================================

extern "C" {

cube32_result_t cube32_i2c_init(void) {
    return cube32::I2CBus::instance().init();
}

cube32_result_t cube32_i2c_init_config(const cube32_i2c_config_t* config) {
    if (!config) {
        return CUBE32_INVALID_ARG;
    }
    return cube32::I2CBus::instance().init(*config);
}

cube32_result_t cube32_i2c_deinit(void) {
    return cube32::I2CBus::instance().deinit();
}

bool cube32_i2c_is_initialized(void) {
    return cube32::I2CBus::instance().isInitialized();
}

i2c_master_bus_handle_t cube32_i2c_get_bus_handle(void) {
    return cube32::I2CBus::instance().getHandle();
}

i2c_port_t cube32_i2c_get_port(void) {
    return cube32::I2CBus::instance().getPort();
}

cube32_result_t cube32_i2c_probe(uint8_t device_addr) {
    return cube32::I2CBus::instance().probe(device_addr);
}

void cube32_i2c_scan(void) {
    cube32::I2CBus::instance().scan();
}

} // extern "C"
