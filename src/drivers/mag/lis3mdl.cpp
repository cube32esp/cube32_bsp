/**
 * @file lis3mdl.cpp
 * @brief CUBE32 Magnetometer Driver - LIS3MDL Implementation
 */

#include "drivers/mag/lis3mdl.h"
#include "cube32_config.h"
#include <esp_log.h>
#include <cstring>

static const char *TAG = "lis3mdl";

namespace cube32 {

// ============================================================================
// Singleton Instance
// ============================================================================

Magnetometer& Magnetometer::instance() {
    static Magnetometer instance;
    return instance;
}

Magnetometer::~Magnetometer() {
    if (m_i2c_dev) {
        i2c_master_bus_rm_device(m_i2c_dev);
        m_i2c_dev = nullptr;
    }
    if (m_mutex) {
        vSemaphoreDelete(m_mutex);
        m_mutex = nullptr;
    }
}

// ============================================================================
// Initialization
// ============================================================================

cube32_result_t Magnetometer::begin() {
    if (m_initialized) {
        ESP_LOGW(TAG, "Magnetometer already initialized");
        return CUBE32_OK;
    }

    ESP_LOGI(TAG, "Initializing LIS3MDL Magnetometer...");

    // Create mutex for thread safety
    m_mutex = xSemaphoreCreateMutex();
    if (m_mutex == nullptr) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return CUBE32_NO_MEM;
    }

    // Check I2C bus is initialized
    if (!I2CBus::instance().isInitialized()) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        vSemaphoreDelete(m_mutex);
        m_mutex = nullptr;
        return CUBE32_ERROR;
    }

    // Create I2C device handle
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = CUBE32_MAG_LIS3MDL_ADDR,
        .scl_speed_hz = 400000,
    };

    esp_err_t err = i2c_master_bus_add_device(I2CBus::instance().getHandle(), &dev_cfg, &m_i2c_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(err));
        vSemaphoreDelete(m_mutex);
        m_mutex = nullptr;
        return CUBE32_ERROR;
    }

    // Verify WHO_AM_I
    uint8_t who_am_i;
    cube32_result_t ret = readRegister(REG_WHO_AM_I, who_am_i);
    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register");
        i2c_master_bus_rm_device(m_i2c_dev);
        m_i2c_dev = nullptr;
        vSemaphoreDelete(m_mutex);
        m_mutex = nullptr;
        return ret;
    }

    if (who_am_i != WHO_AM_I_VALUE) {
        ESP_LOGE(TAG, "Unexpected WHO_AM_I: 0x%02X (expected 0x%02X)", who_am_i, WHO_AM_I_VALUE);
        i2c_master_bus_rm_device(m_i2c_dev);
        m_i2c_dev = nullptr;
        vSemaphoreDelete(m_mutex);
        m_mutex = nullptr;
        return CUBE32_NOT_FOUND;
    }

    ESP_LOGI(TAG, "WHO_AM_I: 0x%02X — LIS3MDL confirmed", who_am_i);

    // Software reset
    ret = reset();
    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Software reset failed");
        i2c_master_bus_rm_device(m_i2c_dev);
        m_i2c_dev = nullptr;
        vSemaphoreDelete(m_mutex);
        m_mutex = nullptr;
        return ret;
    }

    // CTRL_REG1: Enable temp sensor, 10 Hz ODR, Ultra-high XY performance
    // Bits: TEMP_EN(1) | OM1:OM0(11) = Ultra-high | DO2:DO0(100) = 10Hz | 0 | ST(0)
    // = 0x80 | 0x60 | 0x10 = 0xF0
    ret = writeRegister(REG_CTRL_REG1, CTRL1_TEMP_EN |
                        (static_cast<uint8_t>(MagPerformance::PERF_ULTRA_HIGH) << 5) |
                        (static_cast<uint8_t>(MagODR::ODR_10HZ) << 2));
    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to configure CTRL_REG1");
        i2c_master_bus_rm_device(m_i2c_dev);
        m_i2c_dev = nullptr;
        vSemaphoreDelete(m_mutex);
        m_mutex = nullptr;
        return ret;
    }

    // CTRL_REG2: ±4 gauss full-scale
    m_range = MagRange::RANGE_4GAUSS;
    ret = writeRegister(REG_CTRL_REG2, static_cast<uint8_t>(m_range) << 5);
    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to configure CTRL_REG2");
        i2c_master_bus_rm_device(m_i2c_dev);
        m_i2c_dev = nullptr;
        vSemaphoreDelete(m_mutex);
        m_mutex = nullptr;
        return ret;
    }

    // CTRL_REG3: Continuous conversion mode
    ret = writeRegister(REG_CTRL_REG3, static_cast<uint8_t>(MagMode::CONTINUOUS));
    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to configure CTRL_REG3");
        i2c_master_bus_rm_device(m_i2c_dev);
        m_i2c_dev = nullptr;
        vSemaphoreDelete(m_mutex);
        m_mutex = nullptr;
        return ret;
    }

    // CTRL_REG4: Ultra-high Z-axis performance
    ret = writeRegister(REG_CTRL_REG4, static_cast<uint8_t>(MagPerformance::PERF_ULTRA_HIGH) << 2);
    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to configure CTRL_REG4");
        i2c_master_bus_rm_device(m_i2c_dev);
        m_i2c_dev = nullptr;
        vSemaphoreDelete(m_mutex);
        m_mutex = nullptr;
        return ret;
    }

    // CTRL_REG5: Enable BDU (Block Data Update)
    ret = writeRegister(REG_CTRL_REG5, CTRL5_BDU);
    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to configure CTRL_REG5");
        i2c_master_bus_rm_device(m_i2c_dev);
        m_i2c_dev = nullptr;
        vSemaphoreDelete(m_mutex);
        m_mutex = nullptr;
        return ret;
    }

    m_initialized = true;
    ESP_LOGI(TAG, "LIS3MDL Magnetometer initialized (±4 gauss, 10Hz, UHP, continuous)");
    return CUBE32_OK;
}

// ============================================================================
// I2C Communication
// ============================================================================

cube32_result_t Magnetometer::readRegister(uint8_t reg, uint8_t& value) {
    if (!m_i2c_dev) return CUBE32_ERROR;

    esp_err_t ret = i2c_master_transmit_receive(m_i2c_dev, &reg, 1, &value, 1, -1);
    return (ret == ESP_OK) ? CUBE32_OK : CUBE32_IO_ERROR;
}

cube32_result_t Magnetometer::writeRegister(uint8_t reg, uint8_t value) {
    if (!m_i2c_dev) return CUBE32_ERROR;

    uint8_t data[2] = {reg, value};
    esp_err_t ret = i2c_master_transmit(m_i2c_dev, data, 2, -1);
    return (ret == ESP_OK) ? CUBE32_OK : CUBE32_IO_ERROR;
}

cube32_result_t Magnetometer::readRegisters(uint8_t reg, uint8_t* data, size_t len) {
    if (!m_i2c_dev) return CUBE32_ERROR;

    // LIS3MDL requires MSB of sub-address set for multi-byte read (auto-increment)
    uint8_t addr = reg | 0x80;
    esp_err_t ret = i2c_master_transmit_receive(m_i2c_dev, &addr, 1, data, len, -1);
    return (ret == ESP_OK) ? CUBE32_OK : CUBE32_IO_ERROR;
}

// ============================================================================
// Sensitivity Helper
// ============================================================================

float Magnetometer::magSensitivity() const {
    switch (m_range) {
        case MagRange::RANGE_4GAUSS:  return 6842.0f;   // LSB/gauss
        case MagRange::RANGE_8GAUSS:  return 3421.0f;
        case MagRange::RANGE_12GAUSS: return 2281.0f;
        case MagRange::RANGE_16GAUSS: return 1711.0f;
        default:                      return 6842.0f;
    }
}

// ============================================================================
// Data Read Functions
// ============================================================================

cube32_result_t Magnetometer::getData(MagData& data) {
    if (!m_initialized) return CUBE32_NOT_INITIALIZED;

    xSemaphoreTake(m_mutex, portMAX_DELAY);

    // Burst read 6 bytes: OUT_X_L through OUT_Z_H (0x28-0x2D)
    uint8_t raw[6];
    cube32_result_t ret = readRegisters(REG_OUT_X_L, raw, 6);

    if (ret == CUBE32_OK) {
        float sens = magSensitivity();
        int16_t raw_x = static_cast<int16_t>((raw[1] << 8) | raw[0]);
        int16_t raw_y = static_cast<int16_t>((raw[3] << 8) | raw[2]);
        int16_t raw_z = static_cast<int16_t>((raw[5] << 8) | raw[4]);
        data.x = CUBE32_MAG_AXIS_X_SIGN * raw_x / sens;
        data.y = CUBE32_MAG_AXIS_Y_SIGN * raw_y / sens;
        data.z = CUBE32_MAG_AXIS_Z_SIGN * raw_z / sens;
    }

    xSemaphoreGive(m_mutex);
    return ret;
}

cube32_result_t Magnetometer::getTemperature(float& temp_c) {
    if (!m_initialized) return CUBE32_NOT_INITIALIZED;

    xSemaphoreTake(m_mutex, portMAX_DELAY);

    uint8_t raw[2];
    cube32_result_t ret = readRegisters(REG_TEMP_OUT_L, raw, 2);

    if (ret == CUBE32_OK) {
        int16_t raw_temp = static_cast<int16_t>((raw[1] << 8) | raw[0]);
        // LIS3MDL: 8 LSB/°C, offset 25 °C
        temp_c = (raw_temp / 8.0f) + 25.0f;
    }

    xSemaphoreGive(m_mutex);
    return ret;
}

// ============================================================================
// Configuration Functions
// ============================================================================

cube32_result_t Magnetometer::setRange(MagRange range) {
    if (!m_initialized) return CUBE32_NOT_INITIALIZED;

    xSemaphoreTake(m_mutex, portMAX_DELAY);

    cube32_result_t ret = writeRegister(REG_CTRL_REG2, static_cast<uint8_t>(range) << 5);
    if (ret == CUBE32_OK) {
        m_range = range;
    }

    xSemaphoreGive(m_mutex);
    return ret;
}

cube32_result_t Magnetometer::setODR(MagODR odr) {
    if (!m_initialized) return CUBE32_NOT_INITIALIZED;

    xSemaphoreTake(m_mutex, portMAX_DELAY);

    // Read current CTRL_REG1 to preserve TEMP_EN and performance bits
    uint8_t ctrl1;
    cube32_result_t ret = readRegister(REG_CTRL_REG1, ctrl1);
    if (ret == CUBE32_OK) {
        ctrl1 = (ctrl1 & 0xE3) | (static_cast<uint8_t>(odr) << 2);
        ret = writeRegister(REG_CTRL_REG1, ctrl1);
    }

    xSemaphoreGive(m_mutex);
    return ret;
}

cube32_result_t Magnetometer::setMode(MagMode mode) {
    if (!m_initialized) return CUBE32_NOT_INITIALIZED;

    xSemaphoreTake(m_mutex, portMAX_DELAY);

    cube32_result_t ret = writeRegister(REG_CTRL_REG3, static_cast<uint8_t>(mode));

    xSemaphoreGive(m_mutex);
    return ret;
}

cube32_result_t Magnetometer::setPerformance(MagPerformance perf) {
    if (!m_initialized) return CUBE32_NOT_INITIALIZED;

    xSemaphoreTake(m_mutex, portMAX_DELAY);

    // Update XY performance in CTRL_REG1
    uint8_t ctrl1;
    cube32_result_t ret = readRegister(REG_CTRL_REG1, ctrl1);
    if (ret == CUBE32_OK) {
        ctrl1 = (ctrl1 & 0x9F) | (static_cast<uint8_t>(perf) << 5);
        ret = writeRegister(REG_CTRL_REG1, ctrl1);
    }

    // Update Z performance in CTRL_REG4
    if (ret == CUBE32_OK) {
        uint8_t ctrl4;
        ret = readRegister(REG_CTRL_REG4, ctrl4);
        if (ret == CUBE32_OK) {
            ctrl4 = (ctrl4 & 0xF3) | (static_cast<uint8_t>(perf) << 2);
            ret = writeRegister(REG_CTRL_REG4, ctrl4);
        }
    }

    xSemaphoreGive(m_mutex);
    return ret;
}

// ============================================================================
// Status Functions
// ============================================================================

uint8_t Magnetometer::getWhoAmI() {
    uint8_t value = 0;
    readRegister(REG_WHO_AM_I, value);
    return value;
}

cube32_result_t Magnetometer::reset() {
    // Set SOFT_RST bit in CTRL_REG2
    cube32_result_t ret = writeRegister(REG_CTRL_REG2, CTRL2_SOFT_RST);
    if (ret != CUBE32_OK) return ret;

    // Wait for reset to complete
    vTaskDelay(pdMS_TO_TICKS(20));

    // Verify reset completed (SOFT_RST self-clears)
    uint8_t ctrl2;
    ret = readRegister(REG_CTRL_REG2, ctrl2);
    if (ret != CUBE32_OK) return ret;

    if (ctrl2 & CTRL2_SOFT_RST) {
        ESP_LOGE(TAG, "Software reset did not complete");
        return CUBE32_TIMEOUT;
    }

    return CUBE32_OK;
}

bool Magnetometer::isDataReady() {
    if (!m_initialized) return false;

    uint8_t status;
    if (readRegister(REG_STATUS, status) == CUBE32_OK) {
        return (status & STATUS_ZYXDA) != 0;
    }
    return false;
}

} // namespace cube32
