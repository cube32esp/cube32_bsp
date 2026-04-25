/**
 * @file lsm6dso.cpp
 * @brief CUBE32 IMU Driver - LSM6DSO Implementation
 */

#include "drivers/imu/lsm6dso.h"
#include "cube32_config.h"
#include <esp_log.h>
#include <esp_rom_sys.h>
#include <cstring>

static const char *TAG = "lsm6dso";

namespace cube32 {

// ============================================================================
// Singleton Instance
// ============================================================================

IMU& IMU::instance() {
    static IMU instance;
    return instance;
}

IMU::~IMU() {
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

cube32_result_t IMU::begin() {
    if (m_initialized) {
        ESP_LOGW(TAG, "IMU already initialized");
        return CUBE32_OK;
    }

    ESP_LOGI(TAG, "Initializing LSM6DSO IMU...");

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
        .device_address = CUBE32_IMU_LSM6DSO_ADDR,
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

    ESP_LOGI(TAG, "WHO_AM_I: 0x%02X — LSM6DSO confirmed", who_am_i);

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

    // Enable BDU (Block Data Update) and IF_INC (auto-increment)
    ret = writeRegister(REG_CTRL3_C, CTRL3_BDU | CTRL3_IF_INC);
    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to configure CTRL3_C");
        i2c_master_bus_rm_device(m_i2c_dev);
        m_i2c_dev = nullptr;
        vSemaphoreDelete(m_mutex);
        m_mutex = nullptr;
        return ret;
    }

    // Set default accelerometer config: ±4g @ 104 Hz
    m_accel_range = AccelRange::RANGE_4G;
    ret = writeRegister(REG_CTRL1_XL,
                        (static_cast<uint8_t>(AccelODR::ODR_104HZ) << 4) |
                        (static_cast<uint8_t>(m_accel_range) << 2));
    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to configure accelerometer");
        i2c_master_bus_rm_device(m_i2c_dev);
        m_i2c_dev = nullptr;
        vSemaphoreDelete(m_mutex);
        m_mutex = nullptr;
        return ret;
    }

    // Set default gyroscope config: ±500 dps @ 104 Hz
    m_gyro_range = GyroRange::RANGE_500DPS;
    ret = writeRegister(REG_CTRL2_G,
                        (static_cast<uint8_t>(GyroODR::ODR_104HZ) << 4) |
                        (static_cast<uint8_t>(m_gyro_range) << 1));
    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to configure gyroscope");
        i2c_master_bus_rm_device(m_i2c_dev);
        m_i2c_dev = nullptr;
        vSemaphoreDelete(m_mutex);
        m_mutex = nullptr;
        return ret;
    }

    m_initialized = true;
    ESP_LOGI(TAG, "LSM6DSO IMU initialized (±4g, ±500dps, 104Hz)");
    return CUBE32_OK;
}

// ============================================================================
// I2C Communication
// ============================================================================

cube32_result_t IMU::readRegister(uint8_t reg, uint8_t& value) {
    if (!m_i2c_dev) return CUBE32_ERROR;

    esp_err_t ret = i2c_master_transmit_receive(m_i2c_dev, &reg, 1, &value, 1, -1);
    return (ret == ESP_OK) ? CUBE32_OK : CUBE32_IO_ERROR;
}

cube32_result_t IMU::writeRegister(uint8_t reg, uint8_t value) {
    if (!m_i2c_dev) return CUBE32_ERROR;

    uint8_t data[2] = {reg, value};
    esp_err_t ret = i2c_master_transmit(m_i2c_dev, data, 2, -1);
    return (ret == ESP_OK) ? CUBE32_OK : CUBE32_IO_ERROR;
}

cube32_result_t IMU::readRegisters(uint8_t reg, uint8_t* data, size_t len) {
    if (!m_i2c_dev) return CUBE32_ERROR;

    esp_err_t ret = i2c_master_transmit_receive(m_i2c_dev, &reg, 1, data, len, -1);
    return (ret == ESP_OK) ? CUBE32_OK : CUBE32_IO_ERROR;
}

cube32_result_t IMU::writeRegisters(uint8_t reg, const uint8_t* data, size_t len) {
    if (!m_i2c_dev) return CUBE32_ERROR;

    // Build buffer: [reg_addr, data[0], data[1], ...]
    uint8_t buf[len + 1];
    buf[0] = reg;
    memcpy(&buf[1], data, len);
    esp_err_t ret = i2c_master_transmit(m_i2c_dev, buf, len + 1, -1);
    return (ret == ESP_OK) ? CUBE32_OK : CUBE32_IO_ERROR;
}

// ============================================================================
// Sensitivity Helpers
// ============================================================================

float IMU::accelSensitivity() const {
    switch (m_accel_range) {
        case AccelRange::RANGE_2G:  return 0.061f;   // mg/LSB
        case AccelRange::RANGE_4G:  return 0.122f;
        case AccelRange::RANGE_8G:  return 0.244f;
        case AccelRange::RANGE_16G: return 0.488f;
        default:                    return 0.122f;
    }
}

float IMU::gyroSensitivity() const {
    switch (m_gyro_range) {
        case GyroRange::RANGE_125DPS:  return 4.375f;   // mdps/LSB
        case GyroRange::RANGE_250DPS:  return 8.750f;
        case GyroRange::RANGE_500DPS:  return 17.500f;
        case GyroRange::RANGE_1000DPS: return 35.000f;
        case GyroRange::RANGE_2000DPS: return 70.000f;
        default:                       return 17.500f;
    }
}

// ============================================================================
// Data Read Functions
// ============================================================================

cube32_result_t IMU::getAccel(AccelData& data) {
    if (!m_initialized) return CUBE32_NOT_INITIALIZED;

    xSemaphoreTake(m_mutex, portMAX_DELAY);

    uint8_t raw[6];
    cube32_result_t ret = readRegisters(REG_OUTX_L_A, raw, 6);

    if (ret == CUBE32_OK) {
        float sens = accelSensitivity() / 1000.0f;  // Convert mg/LSB to g/LSB
        int16_t raw_x = static_cast<int16_t>((raw[1] << 8) | raw[0]);
        int16_t raw_y = static_cast<int16_t>((raw[3] << 8) | raw[2]);
        int16_t raw_z = static_cast<int16_t>((raw[5] << 8) | raw[4]);
        data.x = CUBE32_IMU_AXIS_X_SIGN * raw_x * sens;
        data.y = CUBE32_IMU_AXIS_Y_SIGN * raw_y * sens;
        data.z = CUBE32_IMU_AXIS_Z_SIGN * raw_z * sens;
    }

    xSemaphoreGive(m_mutex);
    return ret;
}

cube32_result_t IMU::getGyro(GyroData& data) {
    if (!m_initialized) return CUBE32_NOT_INITIALIZED;

    xSemaphoreTake(m_mutex, portMAX_DELAY);

    uint8_t raw[6];
    cube32_result_t ret = readRegisters(REG_OUTX_L_G, raw, 6);

    if (ret == CUBE32_OK) {
        float sens = gyroSensitivity() / 1000.0f;  // Convert mdps/LSB to dps/LSB
        int16_t raw_x = static_cast<int16_t>((raw[1] << 8) | raw[0]);
        int16_t raw_y = static_cast<int16_t>((raw[3] << 8) | raw[2]);
        int16_t raw_z = static_cast<int16_t>((raw[5] << 8) | raw[4]);
        data.x = CUBE32_IMU_AXIS_X_SIGN * raw_x * sens;
        data.y = CUBE32_IMU_AXIS_Y_SIGN * raw_y * sens;
        data.z = CUBE32_IMU_AXIS_Z_SIGN * raw_z * sens;
    }

    xSemaphoreGive(m_mutex);
    return ret;
}

cube32_result_t IMU::getData(IMUData& data) {
    if (!m_initialized) return CUBE32_NOT_INITIALIZED;

    xSemaphoreTake(m_mutex, portMAX_DELAY);

    // Burst read: TEMP(2) + GYRO(6) + ACCEL(6) = 14 bytes from 0x20
    uint8_t raw[14];
    cube32_result_t ret = readRegisters(REG_OUT_TEMP_L, raw, 14);

    if (ret == CUBE32_OK) {
        // Temperature: offset 0-1 (registers 0x20-0x21)
        int16_t raw_temp = static_cast<int16_t>((raw[1] << 8) | raw[0]);
        data.temperature = (raw_temp / 256.0f) + 25.0f;

        // Gyroscope: offset 2-7 (registers 0x22-0x27)
        float gyro_sens = gyroSensitivity() / 1000.0f;
        int16_t raw_gx = static_cast<int16_t>((raw[3] << 8) | raw[2]);
        int16_t raw_gy = static_cast<int16_t>((raw[5] << 8) | raw[4]);
        int16_t raw_gz = static_cast<int16_t>((raw[7] << 8) | raw[6]);
        data.gyro.x = CUBE32_IMU_AXIS_X_SIGN * raw_gx * gyro_sens;
        data.gyro.y = CUBE32_IMU_AXIS_Y_SIGN * raw_gy * gyro_sens;
        data.gyro.z = CUBE32_IMU_AXIS_Z_SIGN * raw_gz * gyro_sens;

        // Accelerometer: offset 8-13 (registers 0x28-0x2D)
        float accel_sens = accelSensitivity() / 1000.0f;
        int16_t raw_ax = static_cast<int16_t>((raw[9] << 8) | raw[8]);
        int16_t raw_ay = static_cast<int16_t>((raw[11] << 8) | raw[10]);
        int16_t raw_az = static_cast<int16_t>((raw[13] << 8) | raw[12]);
        data.accel.x = CUBE32_IMU_AXIS_X_SIGN * raw_ax * accel_sens;
        data.accel.y = CUBE32_IMU_AXIS_Y_SIGN * raw_ay * accel_sens;
        data.accel.z = CUBE32_IMU_AXIS_Z_SIGN * raw_az * accel_sens;
    }

    xSemaphoreGive(m_mutex);
    return ret;
}

cube32_result_t IMU::getTemperature(float& temp_c) {
    if (!m_initialized) return CUBE32_NOT_INITIALIZED;

    xSemaphoreTake(m_mutex, portMAX_DELAY);

    uint8_t raw[2];
    cube32_result_t ret = readRegisters(REG_OUT_TEMP_L, raw, 2);

    if (ret == CUBE32_OK) {
        int16_t raw_temp = static_cast<int16_t>((raw[1] << 8) | raw[0]);
        temp_c = (raw_temp / 256.0f) + 25.0f;
    }

    xSemaphoreGive(m_mutex);
    return ret;
}

// ============================================================================
// Configuration Functions
// ============================================================================

cube32_result_t IMU::setAccelRange(AccelRange range) {
    if (!m_initialized) return CUBE32_NOT_INITIALIZED;

    xSemaphoreTake(m_mutex, portMAX_DELAY);

    // Read current CTRL1_XL to preserve ODR bits
    uint8_t ctrl1;
    cube32_result_t ret = readRegister(REG_CTRL1_XL, ctrl1);
    if (ret == CUBE32_OK) {
        ctrl1 = (ctrl1 & 0xF0) | (static_cast<uint8_t>(range) << 2);
        ret = writeRegister(REG_CTRL1_XL, ctrl1);
        if (ret == CUBE32_OK) {
            m_accel_range = range;
        }
    }

    xSemaphoreGive(m_mutex);
    return ret;
}

cube32_result_t IMU::setGyroRange(GyroRange range) {
    if (!m_initialized) return CUBE32_NOT_INITIALIZED;

    xSemaphoreTake(m_mutex, portMAX_DELAY);

    // Read current CTRL2_G to preserve ODR bits
    uint8_t ctrl2;
    cube32_result_t ret = readRegister(REG_CTRL2_G, ctrl2);
    if (ret == CUBE32_OK) {
        ctrl2 = (ctrl2 & 0xF0) | (static_cast<uint8_t>(range) << 1);
        ret = writeRegister(REG_CTRL2_G, ctrl2);
        if (ret == CUBE32_OK) {
            m_gyro_range = range;
        }
    }

    xSemaphoreGive(m_mutex);
    return ret;
}

cube32_result_t IMU::setAccelODR(AccelODR odr) {
    if (!m_initialized) return CUBE32_NOT_INITIALIZED;

    xSemaphoreTake(m_mutex, portMAX_DELAY);

    // Read current CTRL1_XL to preserve FS bits
    uint8_t ctrl1;
    cube32_result_t ret = readRegister(REG_CTRL1_XL, ctrl1);
    if (ret == CUBE32_OK) {
        ctrl1 = (ctrl1 & 0x0F) | (static_cast<uint8_t>(odr) << 4);
        ret = writeRegister(REG_CTRL1_XL, ctrl1);
    }

    xSemaphoreGive(m_mutex);
    return ret;
}

cube32_result_t IMU::setGyroODR(GyroODR odr) {
    if (!m_initialized) return CUBE32_NOT_INITIALIZED;

    xSemaphoreTake(m_mutex, portMAX_DELAY);

    // Read current CTRL2_G to preserve FS bits
    uint8_t ctrl2;
    cube32_result_t ret = readRegister(REG_CTRL2_G, ctrl2);
    if (ret == CUBE32_OK) {
        ctrl2 = (ctrl2 & 0x0F) | (static_cast<uint8_t>(odr) << 4);
        ret = writeRegister(REG_CTRL2_G, ctrl2);
    }

    xSemaphoreGive(m_mutex);
    return ret;
}

// ============================================================================
// Status Functions
// ============================================================================

uint8_t IMU::getWhoAmI() {
    uint8_t value = 0;
    readRegister(REG_WHO_AM_I, value);
    return value;
}

cube32_result_t IMU::reset() {
    // Set SW_RESET bit in CTRL3_C
    cube32_result_t ret = writeRegister(REG_CTRL3_C, CTRL3_SW_RESET);
    if (ret != CUBE32_OK) return ret;

    // Wait for reset to complete (SW_RESET bit self-clears)
    vTaskDelay(pdMS_TO_TICKS(20));

    // Verify reset completed
    uint8_t ctrl3;
    ret = readRegister(REG_CTRL3_C, ctrl3);
    if (ret != CUBE32_OK) return ret;

    if (ctrl3 & CTRL3_SW_RESET) {
        ESP_LOGE(TAG, "Software reset did not complete");
        return CUBE32_TIMEOUT;
    }

    return CUBE32_OK;
}

bool IMU::isAccelDataReady() {
    if (!m_initialized) return false;

    uint8_t status;
    if (readRegister(REG_STATUS, status) == CUBE32_OK) {
        return (status & STATUS_XLDA) != 0;
    }
    return false;
}

bool IMU::isGyroDataReady() {
    if (!m_initialized) return false;

    uint8_t status;
    if (readRegister(REG_STATUS, status) == CUBE32_OK) {
        return (status & STATUS_GDA) != 0;
    }
    return false;
}

// ============================================================================
// Machine Learning Core (MLC) Functions
// ============================================================================

cube32_result_t IMU::loadMLC(const uint8_t* ucf_data, size_t ucf_len) {
    if (!m_initialized) return CUBE32_NOT_INITIALIZED;
    if (!ucf_data || ucf_len < 2 || (ucf_len % 2) != 0) return CUBE32_ERROR;

    xSemaphoreTake(m_mutex, portMAX_DELAY);

    ESP_LOGI(TAG, "Loading MLC configuration (%u register writes)...", (unsigned)(ucf_len / 2));

    m_mlc_active = false;
    cube32_result_t ret = CUBE32_OK;

    // Write UCF register pairs: {address, value}, {address, value}, ...
    for (size_t i = 0; i < ucf_len; i += 2) {
        uint8_t reg = ucf_data[i];
        uint8_t val = ucf_data[i + 1];
        ret = writeRegister(reg, val);
        if (ret != CUBE32_OK) {
            ESP_LOGE(TAG, "MLC UCF write failed at offset %u (reg=0x%02X val=0x%02X)",
                     (unsigned)i, reg, val);
            break;
        }
    }

    if (ret == CUBE32_OK) {
        m_mlc_active = true;
        ESP_LOGI(TAG, "MLC configuration loaded successfully");
    }

    xSemaphoreGive(m_mutex);
    return ret;
}

cube32_result_t IMU::loadMLC(const ucf_line_t* program, size_t count) {
    if (!m_initialized) return CUBE32_NOT_INITIALIZED;
    if (!program || count == 0) return CUBE32_ERROR;

    xSemaphoreTake(m_mutex, portMAX_DELAY);

    ESP_LOGI(TAG, "Loading MLC configuration (%u register writes)...", (unsigned)count);

    m_mlc_active = false;
    cube32_result_t ret = CUBE32_OK;

    for (size_t i = 0; i < count; i++) {
        ret = writeRegister(program[i].address, program[i].data);
        if (ret != CUBE32_OK) {
            ESP_LOGE(TAG, "MLC UCF write failed at entry %u (reg=0x%02X val=0x%02X)",
                     (unsigned)i, program[i].address, program[i].data);
            break;
        }

        // PAGE_VALUE (0x09) writes go to internal program memory which needs
        // time to commit.  FUNC_CFG_ACCESS (0x01) page switches also need a
        // brief settle.  At 400 kHz I2C the bus is faster than the internal
        // write cycle on some LSM6DSOX silicon revisions.
        if (program[i].address == 0x09) {
            esp_rom_delay_us(100);  // 100 µs per program-memory byte
        } else if (program[i].address == 0x01) {
            esp_rom_delay_us(300);  // 300 µs for page switch settle
        }
    }

    if (ret == CUBE32_OK) {
        m_mlc_active = true;
        ESP_LOGI(TAG, "MLC configuration loaded successfully");
    }

    xSemaphoreGive(m_mutex);
    return ret;
}

cube32_result_t IMU::readMLCOutput(uint8_t tree_index, uint8_t& result) {
    if (!m_initialized) return CUBE32_NOT_INITIALIZED;
    if (tree_index > 7) return CUBE32_ERROR;

    xSemaphoreTake(m_mutex, portMAX_DELAY);

    // Official ST driver reads MLC0_SRC from the embedded functions page.
    // See lsm6dsox_mlc_out_get() in lsm6dsox_reg.c.
    cube32_result_t ret = writeRegister(REG_FUNC_CFG_ACCESS, FUNC_CFG_ACCESS_EMB);
    if (ret == CUBE32_OK) {
        ret = readRegister(REG_MLC0_SRC + tree_index, result);
        writeRegister(REG_FUNC_CFG_ACCESS, 0x00);
    }

    xSemaphoreGive(m_mutex);
    return ret;
}

cube32_result_t IMU::readMLCOutputAll(uint8_t results[8]) {
    if (!m_initialized) return CUBE32_NOT_INITIALIZED;

    xSemaphoreTake(m_mutex, portMAX_DELAY);

    cube32_result_t ret = writeRegister(REG_FUNC_CFG_ACCESS, FUNC_CFG_ACCESS_EMB);
    if (ret == CUBE32_OK) {
        ret = readRegisters(REG_MLC0_SRC, results, 8);
        writeRegister(REG_FUNC_CFG_ACCESS, 0x00);
    }

    xSemaphoreGive(m_mutex);
    return ret;
}

uint8_t IMU::getMLCStatus() {
    if (!m_initialized) return 0;

    uint8_t status = 0;
    // MLC_STATUS_MAINPAGE is readable from the main register page
    readRegister(REG_MLC_STATUS_MAINPAGE, status);
    return status;
}

void IMU::dumpMLCDiagnostics() {
    if (!m_initialized) {
        ESP_LOGW(TAG, "[MLC-DIAG] IMU not initialized");
        return;
    }
    if (!m_mlc_debug) {
        return;
    }

    xSemaphoreTake(m_mutex, portMAX_DELAY);

    uint8_t ctrl1_xl = 0, ctrl2_g = 0, ctrl3_c = 0;
    uint8_t mlc_status = 0, mlc0_src = 0;
    uint8_t func_cfg = 0;
    uint8_t emb_func_en_b = 0;
    uint8_t int1_ctrl = 0;

    // --- Main page registers ---
    readRegister(REG_CTRL1_XL, ctrl1_xl);
    readRegister(REG_CTRL2_G, ctrl2_g);
    readRegister(REG_CTRL3_C, ctrl3_c);
    readRegister(REG_MLC_STATUS_MAINPAGE, mlc_status);
    readRegister(REG_MLC0_SRC, mlc0_src);
    readRegister(REG_FUNC_CFG_ACCESS, func_cfg);
    readRegister(REG_INT1_CTRL, int1_ctrl);

    ESP_LOGI(TAG, "[MLC-DIAG] === Main Page Registers ===");
    ESP_LOGI(TAG, "[MLC-DIAG] FUNC_CFG_ACCESS (0x01) = 0x%02X", func_cfg);
    ESP_LOGI(TAG, "[MLC-DIAG] CTRL1_XL       (0x10) = 0x%02X", ctrl1_xl);
    ESP_LOGI(TAG, "[MLC-DIAG] CTRL2_G        (0x11) = 0x%02X", ctrl2_g);
    ESP_LOGI(TAG, "[MLC-DIAG] CTRL3_C        (0x12) = 0x%02X", ctrl3_c);
    ESP_LOGI(TAG, "[MLC-DIAG] INT1_CTRL      (0x0D) = 0x%02X", int1_ctrl);
    ESP_LOGI(TAG, "[MLC-DIAG] MLC_STATUS     (0x38) = 0x%02X", mlc_status);
    ESP_LOGI(TAG, "[MLC-DIAG] MLC0_SRC       (0x70) = 0x%02X", mlc0_src);

    // --- Switch to embedded functions page ---
    writeRegister(REG_FUNC_CFG_ACCESS, FUNC_CFG_ACCESS_EMB);
    vTaskDelay(pdMS_TO_TICKS(1));  // brief settle

    readRegister(REG_EMB_FUNC_EN_B, emb_func_en_b);

    // Read MLC0_SRC from embedded page (official ST read method)
    uint8_t mlc0_emb = 0;
    readRegister(REG_MLC0_SRC, mlc0_emb);

    // Read additional MLC-related registers on embedded page
    uint8_t mlc_int1 = 0, emb_func_en_a = 0;
    readRegister(0x0D, mlc_int1);     // MLC_INT1
    readRegister(0x04, emb_func_en_a); // EMB_FUNC_EN_A

    // --- Read back MLC program memory (informational) ---
    uint8_t prog_readback[8] = {0};
    writeRegister(0x17, 0x20);    // PAGE_RW = 0x20 (enable page READ)
    writeRegister(0x02, 0x11);    // PAGE_SEL = 0x11
    writeRegister(0x08, 0xEA);    // PAGE_ADDRESS = 0xEA
    esp_rom_delay_us(100);        // settle
    readRegisters(0x09, prog_readback, 8);  // burst read PAGE_VALUE with auto-increment
    writeRegister(0x17, 0x00);    // Disable page read

    // Switch back to main page
    writeRegister(REG_FUNC_CFG_ACCESS, 0x00);

    ESP_LOGI(TAG, "[MLC-DIAG] === Embedded Page Registers ===");
    ESP_LOGI(TAG, "[MLC-DIAG] EMB_FUNC_EN_A  (0x04) = 0x%02X", emb_func_en_a);
    ESP_LOGI(TAG, "[MLC-DIAG] EMB_FUNC_EN_B  (0x05) = 0x%02X  (bit4=MLC_EN)", emb_func_en_b);
    ESP_LOGI(TAG, "[MLC-DIAG] MLC_INT1       (0x0D) = 0x%02X", mlc_int1);
    ESP_LOGI(TAG, "[MLC-DIAG] MLC0_SRC       (0x70) = 0x%02X  (embedded page)", mlc0_emb);

    ESP_LOGI(TAG, "[MLC-DIAG] === Program Memory (page 0x11, addr 0xEA+) ===");
    ESP_LOGI(TAG, "[MLC-DIAG] %02X %02X %02X %02X %02X %02X %02X %02X",
             prog_readback[0], prog_readback[1], prog_readback[2], prog_readback[3],
             prog_readback[4], prog_readback[5], prog_readback[6], prog_readback[7]);

    // --- Interpretation ---
    if ((ctrl1_xl >> 4) == 0x00) {
        ESP_LOGW(TAG, "[MLC-DIAG] CTRL1_XL ODR=0 => accelerometer is OFF");
    }
    if (!(emb_func_en_b & EMB_FUNC_EN_B_MLC_EN)) {
        ESP_LOGW(TAG, "[MLC-DIAG] EMB_FUNC_EN_B.MLC_EN is NOT set => MLC disabled");
    }
    if ((emb_func_en_b & EMB_FUNC_EN_B_MLC_EN) && ((ctrl1_xl >> 4) != 0x00)) {
        ESP_LOGI(TAG, "[MLC-DIAG] MLC configured OK");
    }

    xSemaphoreGive(m_mutex);
}

} // namespace cube32
