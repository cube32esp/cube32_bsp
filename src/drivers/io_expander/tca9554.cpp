/**
 * @file tca9554.cpp
 * @brief TCA9554 I/O Expander Driver Implementation
 */

#include "drivers/io_expander/tca9554.h"

#include <esp_log.h>
#include <esp_io_expander_tca9554.h>

static const char* TAG = "cube32_tca9554";

namespace cube32 {

TCA9554::~TCA9554() {
    if (m_initialized) {
        end();
    }
}

cube32_result_t TCA9554::begin(uint8_t i2c_addr) {
    if (m_initialized) {
        ESP_LOGW(TAG, "TCA9554 at 0x%02X already initialized", i2c_addr);
        return CUBE32_ALREADY_INITIALIZED;
    }
    
    // Get the shared I2C bus handle
    i2c_master_bus_handle_t i2c_bus = I2CBus::instance().getHandle();
    if (i2c_bus == nullptr) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return CUBE32_NOT_INITIALIZED;
    }
    
    m_address = i2c_addr;
    
    ESP_LOGI(TAG, "Initializing TCA9554 at I2C address 0x%02X", i2c_addr);
    
    // Try to create the TCA9554 expander
    esp_err_t ret = esp_io_expander_new_i2c_tca9554(i2c_bus, i2c_addr, &m_handle);
    
    if (ret != ESP_OK) {
        // Try TCA9554A variant (different address range)
        ESP_LOGW(TAG, "TCA9554 not found at 0x%02X, trying as TCA9554A", i2c_addr);
        ret = esp_io_expander_new_i2c_tca9554(i2c_bus, i2c_addr, &m_handle);
        
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize TCA9554 at 0x%02X: %s", 
                     i2c_addr, esp_err_to_name(ret));
            return esp_err_to_cube32(ret);
        }
    }
    
    m_initialized = true;
    ESP_LOGI(TAG, "TCA9554 initialized successfully at 0x%02X", i2c_addr);
    
    return CUBE32_OK;
}

cube32_result_t TCA9554::end() {
    if (!m_initialized) {
        return CUBE32_NOT_INITIALIZED;
    }
    
    if (m_handle != nullptr) {
        esp_err_t ret = esp_io_expander_del(m_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to delete TCA9554: %s", esp_err_to_name(ret));
            return esp_err_to_cube32(ret);
        }
        m_handle = nullptr;
    }
    
    m_initialized = false;
    ESP_LOGI(TAG, "TCA9554 at 0x%02X deinitialized", m_address);
    
    return CUBE32_OK;
}

cube32_result_t TCA9554::setDirection(TCA9554Pin pin, TCA9554Direction direction) {
    return setDirectionMask(static_cast<uint32_t>(pin), direction);
}

cube32_result_t TCA9554::setDirectionMask(uint32_t pin_mask, TCA9554Direction direction) {
    if (!m_initialized) {
        return CUBE32_NOT_INITIALIZED;
    }
    
    esp_err_t ret = esp_io_expander_set_dir(m_handle, pin_mask, 
                                             static_cast<esp_io_expander_dir_t>(direction));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set direction: %s", esp_err_to_name(ret));
        return esp_err_to_cube32(ret);
    }
    
    return CUBE32_OK;
}

cube32_result_t TCA9554::setLevel(TCA9554Pin pin, bool level) {
    return setLevelMask(static_cast<uint32_t>(pin), level);
}

cube32_result_t TCA9554::setLevelMask(uint32_t pin_mask, bool level) {
    if (!m_initialized) {
        return CUBE32_NOT_INITIALIZED;
    }
    
    esp_err_t ret = esp_io_expander_set_level(m_handle, pin_mask, level ? 1 : 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set level: %s", esp_err_to_name(ret));
        return esp_err_to_cube32(ret);
    }
    
    return CUBE32_OK;
}

cube32_result_t TCA9554::getLevel(TCA9554Pin pin, bool& level) {
    if (!m_initialized) {
        return CUBE32_NOT_INITIALIZED;
    }
    
    uint32_t levels = 0;
    esp_err_t ret = esp_io_expander_get_level(m_handle, static_cast<uint32_t>(pin), &levels);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get level: %s", esp_err_to_name(ret));
        return esp_err_to_cube32(ret);
    }
    
    level = (levels & static_cast<uint32_t>(pin)) != 0;
    return CUBE32_OK;
}

cube32_result_t TCA9554::getAllLevels(uint32_t& levels) {
    if (!m_initialized) {
        return CUBE32_NOT_INITIALIZED;
    }
    
    // Read all pins by using the full mask
    uint32_t all_pins = 0xFF;
    esp_err_t ret = esp_io_expander_get_level(m_handle, all_pins, &levels);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get all levels: %s", esp_err_to_name(ret));
        return esp_err_to_cube32(ret);
    }
    
    return CUBE32_OK;
}

} // namespace cube32
