/**
 * @file lsm6dso.h
 * @brief CUBE32 IMU Driver - LSM6DSOX (6-axis Accelerometer + Gyroscope + MLC)
 *
 * This driver provides a C++ class interface for the LSM6DSOX IMU with
 * support for:
 * - 3-axis accelerometer (±2g / ±4g / ±8g / ±16g)
 * - 3-axis gyroscope (±125 / ±250 / ±500 / ±1000 / ±2000 dps)
 * - Temperature sensor
 * - Configurable output data rates
 * - Block Data Update (BDU) for consistent reads
 * - Machine Learning Core (MLC) — on-sensor decision tree inference
 *
 * The driver uses the shared I2C bus from utils/i2c_bus.h.
 * No interrupt pin is connected on the CUBE32 board.
 * MLC output is polled via MLC_STATUS_MAINPAGE register.
 */

#ifndef CUBE32_DRIVERS_IMU_LSM6DSO_H
#define CUBE32_DRIVERS_IMU_LSM6DSO_H

#include "utils/common.h"
#include "utils/i2c_bus.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <cstdint>

#define CUBE32_IMU_LSM6DSO_ADDR     0x6A    ///< LSM6DSO I2C address (SDO/SA0 = GND)

// UCF line type (ST convention) — shared with UCF headers
#ifndef MEMS_UCF_SHARED_TYPES
#define MEMS_UCF_SHARED_TYPES
typedef struct { uint8_t address; uint8_t data; } ucf_line_t;
#endif

namespace cube32 {

// ============================================================================
// Types
// ============================================================================

/**
 * @brief 3-axis accelerometer data (in g)
 */
struct AccelData {
    float x = 0.0f;    ///< X-axis acceleration (g)
    float y = 0.0f;    ///< Y-axis acceleration (g)
    float z = 0.0f;    ///< Z-axis acceleration (g)
};

/**
 * @brief 3-axis gyroscope data (in degrees per second)
 */
struct GyroData {
    float x = 0.0f;    ///< X-axis angular rate (dps)
    float y = 0.0f;    ///< Y-axis angular rate (dps)
    float z = 0.0f;    ///< Z-axis angular rate (dps)
};

/**
 * @brief Combined IMU data (accel + gyro)
 */
struct IMUData {
    AccelData accel;
    GyroData  gyro;
    float     temperature = 0.0f;   ///< Chip temperature (°C)
};

/**
 * @brief Accelerometer full-scale range
 */
enum class AccelRange : uint8_t {
    RANGE_2G  = 0x00,   ///< ±2 g
    RANGE_4G  = 0x02,   ///< ±4 g  (default)
    RANGE_8G  = 0x03,   ///< ±8 g
    RANGE_16G = 0x01,   ///< ±16 g
};

/**
 * @brief Gyroscope full-scale range
 */
enum class GyroRange : uint8_t {
    RANGE_125DPS  = 0x01,   ///< ±125 dps
    RANGE_250DPS  = 0x00,   ///< ±250 dps
    RANGE_500DPS  = 0x02,   ///< ±500 dps  (default)
    RANGE_1000DPS = 0x04,   ///< ±1000 dps
    RANGE_2000DPS = 0x06,   ///< ±2000 dps
};

/**
 * @brief Accelerometer output data rate
 */
enum class AccelODR : uint8_t {
    ODR_OFF     = 0x00,   ///< Power-down
    ODR_12_5HZ  = 0x01,   ///< 12.5 Hz
    ODR_26HZ    = 0x02,   ///< 26 Hz
    ODR_52HZ    = 0x03,   ///< 52 Hz
    ODR_104HZ   = 0x04,   ///< 104 Hz  (default)
    ODR_208HZ   = 0x05,   ///< 208 Hz
    ODR_416HZ   = 0x06,   ///< 416 Hz
    ODR_833HZ   = 0x07,   ///< 833 Hz
    ODR_1660HZ  = 0x08,   ///< 1.66 kHz
    ODR_3330HZ  = 0x09,   ///< 3.33 kHz
    ODR_6660HZ  = 0x0A,   ///< 6.66 kHz
};

/**
 * @brief Gyroscope output data rate
 */
enum class GyroODR : uint8_t {
    ODR_OFF     = 0x00,   ///< Power-down
    ODR_12_5HZ  = 0x01,   ///< 12.5 Hz
    ODR_26HZ    = 0x02,   ///< 26 Hz
    ODR_52HZ    = 0x03,   ///< 52 Hz
    ODR_104HZ   = 0x04,   ///< 104 Hz  (default)
    ODR_208HZ   = 0x05,   ///< 208 Hz
    ODR_416HZ   = 0x06,   ///< 416 Hz
    ODR_833HZ   = 0x07,   ///< 833 Hz
    ODR_1660HZ  = 0x08,   ///< 1.66 kHz
    ODR_3330HZ  = 0x09,   ///< 3.33 kHz
    ODR_6660HZ  = 0x0A,   ///< 6.66 kHz
};

// ============================================================================
// LSM6DSO IMU Class
// ============================================================================

/**
 * @brief LSM6DSOX IMU driver class (Singleton)
 *
 * Provides access to the LSM6DSOX 6-axis IMU (accel + gyro) including:
 * - Reading accelerometer, gyroscope, and temperature data
 * - Configuring full-scale ranges and output data rates
 * - Machine Learning Core (MLC) — load UCF configs, read classification
 * - Software reset
 *
 * No interrupt functionality (INT pins not wired on CUBE32).
 * MLC results are polled via MLC_STATUS_MAINPAGE.
 */
class IMU {
public:
    /**
     * @brief Get the singleton instance
     * @return Reference to the IMU instance
     */
    static IMU& instance();

    // Delete copy constructor and assignment operator
    IMU(const IMU&) = delete;
    IMU& operator=(const IMU&) = delete;

    /**
     * @brief Initialize the IMU driver
     *
     * Verifies WHO_AM_I, performs software reset, configures BDU,
     * and sets default ranges (±4g, ±500dps) and ODR (104 Hz).
     *
     * @return CUBE32_OK on success, error code otherwise
     */
    cube32_result_t begin();

    /**
     * @brief Check if IMU is initialized
     * @return true if initialized
     */
    bool isInitialized() const { return m_initialized; }

    // ========================================================================
    // Data Read Functions
    // ========================================================================

    /**
     * @brief Read accelerometer data
     * @param data Output accelerometer data in g
     * @return CUBE32_OK on success
     */
    cube32_result_t getAccel(AccelData& data);

    /**
     * @brief Read gyroscope data
     * @param data Output gyroscope data in dps
     * @return CUBE32_OK on success
     */
    cube32_result_t getGyro(GyroData& data);

    /**
     * @brief Read combined accelerometer + gyroscope + temperature
     * @param data Output IMU data
     * @return CUBE32_OK on success
     */
    cube32_result_t getData(IMUData& data);

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
     * @brief Set accelerometer full-scale range
     * @param range Full-scale range
     * @return CUBE32_OK on success
     */
    cube32_result_t setAccelRange(AccelRange range);

    /**
     * @brief Set gyroscope full-scale range
     * @param range Full-scale range
     * @return CUBE32_OK on success
     */
    cube32_result_t setGyroRange(GyroRange range);

    /**
     * @brief Set accelerometer output data rate
     * @param odr Output data rate
     * @return CUBE32_OK on success
     */
    cube32_result_t setAccelODR(AccelODR odr);

    /**
     * @brief Set gyroscope output data rate
     * @param odr Output data rate
     * @return CUBE32_OK on success
     */
    cube32_result_t setGyroODR(GyroODR odr);

    // ========================================================================
    // Status Functions
    // ========================================================================

    /**
     * @brief Read the WHO_AM_I register
     * @return WHO_AM_I value (expect 0x6C for LSM6DSO)
     */
    uint8_t getWhoAmI();

    /**
     * @brief Perform software reset
     * @return CUBE32_OK on success
     */
    cube32_result_t reset();

    /**
     * @brief Check if new accelerometer data is available
     * @return true if new data ready
     */
    bool isAccelDataReady();

    /**
     * @brief Check if new gyroscope data is available
     * @return true if new data ready
     */
    bool isGyroDataReady();

    // ========================================================================
    // Machine Learning Core (MLC) Functions
    // ========================================================================

    /**
     * @brief Load a UCF (Unico Configuration File) into the MLC
     *
     * The UCF is an array of {register, value} pairs that configures the
     * MLC decision trees, filters, and sensor ODR/FS settings.
     * Performs a software reset before loading to ensure clean state.
     *
     * @param ucf_data Pointer to UCF data array (pairs of uint8_t: reg, val)
     * @param ucf_len  Number of bytes in the UCF array (must be even)
     * @return CUBE32_OK on success
     */
    cube32_result_t loadMLC(const uint8_t* ucf_data, size_t ucf_len);

    /**
     * @brief Load a UCF into the MLC (ucf_line_t struct array)
     *
     * Overload accepting ST-format ucf_line_t array from Unico Tool.
     *
     * @param program  Pointer to ucf_line_t array
     * @param count    Number of entries in the array
     * @return CUBE32_OK on success
     */
    cube32_result_t loadMLC(const ucf_line_t* program, size_t count);

    /**
     * @brief Read the output of a specific MLC decision tree
     *
     * @param tree_index Decision tree index (0–7)
     * @param result     Output classification value
     * @return CUBE32_OK on success
     */
    cube32_result_t readMLCOutput(uint8_t tree_index, uint8_t& result);

    /**
     * @brief Read all 8 MLC decision tree outputs at once
     *
     * @param results Array of 8 bytes to receive MLC0_SRC through MLC7_SRC
     * @return CUBE32_OK on success
     */
    cube32_result_t readMLCOutputAll(uint8_t results[8]);

    /**
     * @brief Check if any MLC decision tree has new output
     *
     * Reads MLC_STATUS_MAINPAGE (0x38) from the main register page.
     * Each bit corresponds to one decision tree (bit 0 = tree 0, etc.)
     *
     * @return Bitmask of trees with new data (0 = no new data)
     */
    uint8_t getMLCStatus();

    /**
     * @brief Check if MLC is currently loaded and active
     * @return true if MLC configuration has been loaded
     */
    bool isMLCActive() const { return m_mlc_active; }

    /**
     * @brief Enable or disable verbose MLC diagnostic logging
     * @param enable true to enable diagnostic output
     */
    void setMLCDebug(bool enable) { m_mlc_debug = enable; }

    /**
     * @brief Dump MLC diagnostic registers to serial log
     *
     * Reads and logs CTRL1_XL, CTRL3_C, MLC_STATUS, MLC0_SRC,
     * and EMB_FUNC_EN_B (requires page switch) to verify MLC setup.
     * Output is only produced when MLC debug is enabled via setMLCDebug().
     */
    void dumpMLCDiagnostics();

private:
    IMU() = default;
    ~IMU();

    // I2C Communication
    cube32_result_t readRegister(uint8_t reg, uint8_t& value);
    cube32_result_t writeRegister(uint8_t reg, uint8_t value);
    cube32_result_t readRegisters(uint8_t reg, uint8_t* data, size_t len);
    cube32_result_t writeRegisters(uint8_t reg, const uint8_t* data, size_t len);

    // Sensitivity lookup
    float accelSensitivity() const;
    float gyroSensitivity() const;

    // Member variables
    bool m_initialized = false;
    bool m_mlc_active = false;
    bool m_mlc_debug = false;
    SemaphoreHandle_t m_mutex = nullptr;
    i2c_master_dev_handle_t m_i2c_dev = nullptr;
    AccelRange m_accel_range = AccelRange::RANGE_4G;
    GyroRange m_gyro_range = GyroRange::RANGE_500DPS;

    // ========================================================================
    // LSM6DSOX Register Map
    // ========================================================================
    static constexpr uint8_t REG_FUNC_CFG_ACCESS = 0x01;
    static constexpr uint8_t REG_INT1_CTRL       = 0x0D;
    static constexpr uint8_t REG_INT2_CTRL       = 0x0E;
    static constexpr uint8_t REG_WHO_AM_I        = 0x0F;
    static constexpr uint8_t REG_CTRL1_XL        = 0x10;   ///< Accel ODR + FS
    static constexpr uint8_t REG_CTRL2_G         = 0x11;   ///< Gyro ODR + FS
    static constexpr uint8_t REG_CTRL3_C         = 0x12;   ///< BDU, IF_INC, SW_RESET
    static constexpr uint8_t REG_CTRL4_C         = 0x13;
    static constexpr uint8_t REG_CTRL5_C         = 0x14;
    static constexpr uint8_t REG_CTRL6_C         = 0x15;
    static constexpr uint8_t REG_CTRL7_G         = 0x16;
    static constexpr uint8_t REG_CTRL8_XL        = 0x17;
    static constexpr uint8_t REG_STATUS          = 0x1E;
    static constexpr uint8_t REG_OUT_TEMP_L      = 0x20;
    static constexpr uint8_t REG_OUT_TEMP_H      = 0x21;
    static constexpr uint8_t REG_OUTX_L_G        = 0x22;   ///< Gyro X low byte
    static constexpr uint8_t REG_OUTX_L_A        = 0x28;   ///< Accel X low byte

    // MLC registers (main page)
    static constexpr uint8_t REG_MLC_STATUS_MAINPAGE = 0x38; ///< MLC status on main page
    static constexpr uint8_t REG_MLC0_SRC        = 0x70;   ///< MLC tree 0 output (embedded page, per ST driver)
    static constexpr uint8_t REG_MLC1_SRC        = 0x71;
    static constexpr uint8_t REG_MLC2_SRC        = 0x72;
    static constexpr uint8_t REG_MLC3_SRC        = 0x73;
    static constexpr uint8_t REG_MLC4_SRC        = 0x74;
    static constexpr uint8_t REG_MLC5_SRC        = 0x75;
    static constexpr uint8_t REG_MLC6_SRC        = 0x76;
    static constexpr uint8_t REG_MLC7_SRC        = 0x77;

    // Embedded functions page registers
    static constexpr uint8_t REG_EMB_FUNC_EN_B   = 0x05;   ///< Embedded function enable B (embedded page)

    // FUNC_CFG_ACCESS bits
    static constexpr uint8_t FUNC_CFG_ACCESS_EMB    = 0x80; ///< Access embedded function registers
    static constexpr uint8_t FUNC_CFG_ACCESS_SHUB   = 0x40; ///< Access sensor hub registers

    // EMB_FUNC_EN_B bits
    static constexpr uint8_t EMB_FUNC_EN_B_MLC_EN   = 0x10; ///< Enable Machine Learning Core

    // WHO_AM_I expected value
    static constexpr uint8_t WHO_AM_I_VALUE      = 0x6C;

    // CTRL3_C bits
    static constexpr uint8_t CTRL3_SW_RESET      = 0x01;
    static constexpr uint8_t CTRL3_IF_INC        = 0x04;   ///< Auto-increment address
    static constexpr uint8_t CTRL3_BDU           = 0x40;   ///< Block Data Update

    // STATUS register bits
    static constexpr uint8_t STATUS_XLDA         = 0x01;   ///< Accel new data available
    static constexpr uint8_t STATUS_GDA          = 0x02;   ///< Gyro new data available
    static constexpr uint8_t STATUS_TDA          = 0x04;   ///< Temp new data available
};

} // namespace cube32

#endif // CUBE32_DRIVERS_IMU_LSM6DSO_H
