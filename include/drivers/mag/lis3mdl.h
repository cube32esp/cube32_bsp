/**
 * @file lis3mdl.h
 * @brief CUBE32 Magnetometer Driver - LIS3MDL (3-axis Magnetometer)
 *
 * This driver provides a C++ class interface for the LIS3MDL magnetometer
 * with support for:
 * - 3-axis magnetic field measurement (±4 / ±8 / ±12 / ±16 gauss)
 * - Temperature sensor
 * - Configurable output data rates and operating modes
 * - Continuous / single / power-down modes
 *
 * The driver uses the shared I2C bus from utils/i2c_bus.h.
 * No interrupt pin is connected on the CUBE32 board.
 */

#ifndef CUBE32_DRIVERS_MAG_LIS3MDL_H
#define CUBE32_DRIVERS_MAG_LIS3MDL_H

#include "utils/common.h"
#include "utils/i2c_bus.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <cstdint>

#define CUBE32_MAG_LIS3MDL_ADDR     0x1C    ///< LIS3MDL I2C address (SDO/SA1 = GND)

namespace cube32 {

// ============================================================================
// Types
// ============================================================================

/**
 * @brief 3-axis magnetometer data (in gauss)
 */
struct MagData {
    float x = 0.0f;    ///< X-axis magnetic field (gauss)
    float y = 0.0f;    ///< Y-axis magnetic field (gauss)
    float z = 0.0f;    ///< Z-axis magnetic field (gauss)
};

/**
 * @brief Magnetometer full-scale range
 */
enum class MagRange : uint8_t {
    RANGE_4GAUSS  = 0x00,   ///< ±4 gauss  (default)
    RANGE_8GAUSS  = 0x01,   ///< ±8 gauss
    RANGE_12GAUSS = 0x02,   ///< ±12 gauss
    RANGE_16GAUSS = 0x03,   ///< ±16 gauss
};

/**
 * @brief Magnetometer output data rate
 */
enum class MagODR : uint8_t {
    ODR_0_625HZ = 0x00,   ///< 0.625 Hz
    ODR_1_25HZ  = 0x01,   ///< 1.25 Hz
    ODR_2_5HZ   = 0x02,   ///< 2.5 Hz
    ODR_5HZ     = 0x03,   ///< 5 Hz
    ODR_10HZ    = 0x04,   ///< 10 Hz  (default)
    ODR_20HZ    = 0x05,   ///< 20 Hz
    ODR_40HZ    = 0x06,   ///< 40 Hz
    ODR_80HZ    = 0x07,   ///< 80 Hz
};

/**
 * @brief Magnetometer operating mode
 */
enum class MagMode : uint8_t {
    CONTINUOUS  = 0x00,   ///< Continuous conversion (default)
    SINGLE      = 0x01,   ///< Single conversion
    POWER_DOWN  = 0x03,   ///< Power-down
};

/**
 * @brief XY-axis performance mode (also affects ODR capability)
 */
enum class MagPerformance : uint8_t {
    PERF_LOW_POWER   = 0x00,   ///< Low-power
    PERF_MEDIUM      = 0x01,   ///< Medium performance
    PERF_HIGH        = 0x02,   ///< High performance
    PERF_ULTRA_HIGH  = 0x03,   ///< Ultra-high performance  (default)
};

// ============================================================================
// LIS3MDL Magnetometer Class
// ============================================================================

/**
 * @brief LIS3MDL Magnetometer driver class (Singleton)
 *
 * Provides access to the LIS3MDL 3-axis magnetometer including:
 * - Reading magnetic field data and temperature
 * - Configuring full-scale range, ODR, and performance mode
 * - Software reset
 *
 * No interrupt functionality (INT pin not wired on CUBE32).
 */
class Magnetometer {
public:
    /**
     * @brief Get the singleton instance
     * @return Reference to the Magnetometer instance
     */
    static Magnetometer& instance();

    // Delete copy constructor and assignment operator
    Magnetometer(const Magnetometer&) = delete;
    Magnetometer& operator=(const Magnetometer&) = delete;

    /**
     * @brief Initialize the magnetometer driver
     *
     * Verifies WHO_AM_I, performs software reset, configures BDU,
     * and sets defaults (±4 gauss, 10 Hz, continuous, ultra-high perf).
     *
     * @return CUBE32_OK on success, error code otherwise
     */
    cube32_result_t begin();

    /**
     * @brief Check if magnetometer is initialized
     * @return true if initialized
     */
    bool isInitialized() const { return m_initialized; }

    // ========================================================================
    // Data Read Functions
    // ========================================================================

    /**
     * @brief Read magnetometer data
     * @param data Output magnetic field data in gauss
     * @return CUBE32_OK on success
     */
    cube32_result_t getData(MagData& data);

    /**
     * @brief Read chip temperature
     * @param temp_c Output temperature in °C
     * @return CUBE32_OK on success
     */
    cube32_result_t getTemperature(float& temp_c);

    // ========================================================================
    // Configuration Functions
    // ========================================================================

    /**
     * @brief Set magnetometer full-scale range
     * @param range Full-scale range
     * @return CUBE32_OK on success
     */
    cube32_result_t setRange(MagRange range);

    /**
     * @brief Set magnetometer output data rate
     * @param odr Output data rate
     * @return CUBE32_OK on success
     */
    cube32_result_t setODR(MagODR odr);

    /**
     * @brief Set operating mode
     * @param mode Operating mode (continuous / single / power-down)
     * @return CUBE32_OK on success
     */
    cube32_result_t setMode(MagMode mode);

    /**
     * @brief Set XY-axis performance mode
     * @param perf Performance mode
     * @return CUBE32_OK on success
     */
    cube32_result_t setPerformance(MagPerformance perf);

    // ========================================================================
    // Status Functions
    // ========================================================================

    /**
     * @brief Read the WHO_AM_I register
     * @return WHO_AM_I value (expect 0x3D for LIS3MDL)
     */
    uint8_t getWhoAmI();

    /**
     * @brief Perform software reset
     * @return CUBE32_OK on success
     */
    cube32_result_t reset();

    /**
     * @brief Check if new data is available
     * @return true if new XYZ data ready
     */
    bool isDataReady();

private:
    Magnetometer() = default;
    ~Magnetometer();

    // I2C Communication
    cube32_result_t readRegister(uint8_t reg, uint8_t& value);
    cube32_result_t writeRegister(uint8_t reg, uint8_t value);
    cube32_result_t readRegisters(uint8_t reg, uint8_t* data, size_t len);

    // Sensitivity lookup
    float magSensitivity() const;

    // Member variables
    bool m_initialized = false;
    SemaphoreHandle_t m_mutex = nullptr;
    i2c_master_dev_handle_t m_i2c_dev = nullptr;
    MagRange m_range = MagRange::RANGE_4GAUSS;

    // ========================================================================
    // LIS3MDL Register Map
    // ========================================================================
    static constexpr uint8_t REG_WHO_AM_I    = 0x0F;
    static constexpr uint8_t REG_CTRL_REG1   = 0x20;   ///< Temp EN, ODR, perf XY, self-test
    static constexpr uint8_t REG_CTRL_REG2   = 0x21;   ///< Full-scale, reboot, soft reset
    static constexpr uint8_t REG_CTRL_REG3   = 0x22;   ///< Operating mode
    static constexpr uint8_t REG_CTRL_REG4   = 0x23;   ///< Z-axis perf, endianness
    static constexpr uint8_t REG_CTRL_REG5   = 0x24;   ///< BDU
    static constexpr uint8_t REG_STATUS      = 0x27;
    static constexpr uint8_t REG_OUT_X_L     = 0x28;
    static constexpr uint8_t REG_OUT_X_H     = 0x29;
    static constexpr uint8_t REG_OUT_Y_L     = 0x2A;
    static constexpr uint8_t REG_OUT_Y_H     = 0x2B;
    static constexpr uint8_t REG_OUT_Z_L     = 0x2C;
    static constexpr uint8_t REG_OUT_Z_H     = 0x2D;
    static constexpr uint8_t REG_TEMP_OUT_L  = 0x2E;
    static constexpr uint8_t REG_TEMP_OUT_H  = 0x2F;

    // WHO_AM_I expected value
    static constexpr uint8_t WHO_AM_I_VALUE  = 0x3D;

    // CTRL_REG1 bits
    static constexpr uint8_t CTRL1_TEMP_EN   = 0x80;   ///< Temperature sensor enable

    // CTRL_REG2 bits
    static constexpr uint8_t CTRL2_REBOOT    = 0x08;
    static constexpr uint8_t CTRL2_SOFT_RST  = 0x04;

    // CTRL_REG5 bits
    static constexpr uint8_t CTRL5_BDU       = 0x40;   ///< Block Data Update

    // STATUS register bits
    static constexpr uint8_t STATUS_ZYXDA    = 0x08;   ///< XYZ new data available
};

} // namespace cube32

#endif // CUBE32_DRIVERS_MAG_LIS3MDL_H
