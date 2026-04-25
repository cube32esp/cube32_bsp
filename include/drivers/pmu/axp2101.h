/**
 * @file axp2101.h
 * @brief CUBE32 PMU (Power Management Unit) Driver - AXP2101 (Pure C++)
 * 
 * This driver provides a C++ class interface for the AXP2101 PMU chip
 * using the XPowersLib library.
 * 
 * The driver uses the shared I2C bus from utils/i2c_bus.h
 */

#ifndef CUBE32_DRIVERS_PMU_AXP2101_H
#define CUBE32_DRIVERS_PMU_AXP2101_H

#include "utils/common.h"
#include "utils/i2c_bus.h"

#define XPOWERS_CHIP_AXP2101
#include "XPowersLib.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <cstdint>

// ============================================================================
// Constants
// ============================================================================

#define CUBE32_PMU_AXP2101_ADDR     0x34    ///< AXP2101 I2C address
#define CUBE32_PMU_AXP2101_CHIP_ID  0x4A    ///< AXP2101 chip ID

namespace cube32 {

// ============================================================================
// Types
// ============================================================================

/**
 * @brief PMU status structure
 */
struct PMUStatus {
    bool isCharging = false;         ///< Battery is charging
    bool isDischarging = false;      ///< Battery is discharging
    bool isStandby = false;          ///< PMU is in standby mode
    bool vbusPresent = false;        ///< VBUS (USB) power is present
    bool vbusGood = false;           ///< VBUS voltage is good
    bool batteryPresent = false;     ///< Battery is connected
    uint16_t batteryVoltage = 0;     ///< Battery voltage in mV
    uint16_t vbusVoltage = 0;        ///< VBUS voltage in mV
    uint16_t systemVoltage = 0;      ///< System voltage in mV
    int8_t batteryPercent = -1;      ///< Battery percentage (0-100, -1 if unknown)
    float temperature = 0.0f;        ///< PMU temperature in Celsius
    uint8_t chargeStatus = 0;        ///< Charge status code
};

/**
 * @brief Charge status codes
 */
enum class ChargeStatus : uint8_t {
    TriState = 0,   ///< Tri-state charging
    PreCharge,      ///< Pre-charge
    ConstantCurrent,///< Constant current
    ConstantVoltage,///< Constant voltage
    Done,           ///< Charge done
    Stopped,        ///< Not charging
};

/**
 * @brief Power channel identifiers
 */
enum class PowerChannel : uint8_t {
    DC1 = 0,
    DC2,
    DC3,
    DC4,
    DC5,
    ALDO1,
    ALDO2,
    ALDO3,
    ALDO4,
    BLDO1,
    BLDO2,
    DLDO1,
    DLDO2,
    CPUSLDO,
    Count,
};

/**
 * @brief Charging current options (mA)
 */
enum class ChargeCurrent : uint16_t {
    MA_100 = 100,
    MA_125 = 125,
    MA_150 = 150,
    MA_175 = 175,
    MA_200 = 200,
    MA_300 = 300,
    MA_400 = 400,
    MA_500 = 500,
    MA_600 = 600,
    MA_700 = 700,
    MA_800 = 800,
    MA_900 = 900,
    MA_1000 = 1000,
};

// ============================================================================
// PMU Class
// ============================================================================

/**
 * @brief PMU Driver Class (AXP2101)
 * 
 * Object-oriented interface for the AXP2101 PMU, similar to the Arduino driver.
 * Uses the shared I2C bus managed by I2CBus singleton.
 * 
 * Usage:
 * @code
 *   cube32::I2CBus::instance().init();
 *   cube32::PMU& pmu = cube32::PMU::instance();
 *   pmu.begin();
 *   
 *   if (pmu.isVbusPresent()) {
 *       int percent = pmu.getBatteryPercent();
 *       ESP_LOGI("PMU", "Battery: %d%%", percent);
 *   }
 * @endcode
 */
class PMU {
public:
    /**
     * @brief Get the singleton PMU instance
     */
    static PMU& instance();

    /**
     * @brief Initialize the PMU driver
     * 
     * @return CUBE32_OK on success
     */
    cube32_result_t begin();

    /**
     * @brief Deinitialize the PMU driver
     */
    cube32_result_t end();

    /**
     * @brief Check if initialized
     */
    bool isInitialized() const { return m_initialized; }

    /**
     * @brief Get the chip ID (0x4A for AXP2101)
     */
    uint8_t getChipId() const;

    // ---- Status ----
    cube32_result_t getStatus(PMUStatus& status);
    void printStatus();

    // ---- Battery ----
    bool isBatteryPresent();
    bool isCharging();
    bool isDischarging();
    int getBatteryPercent();
    uint16_t getBatteryVoltage();

    // ---- Power Source ----
    bool isVbusPresent();
    bool isVbusGood();
    uint16_t getVbusVoltage();
    uint16_t getSystemVoltage();

    // ---- Charging ----
    cube32_result_t setChargeCurrent(ChargeCurrent current);
    cube32_result_t enableCharging(bool enable);
    ChargeStatus getChargeStatus();

    // ---- Power Channels ----
    cube32_result_t setChannelVoltage(PowerChannel channel, uint16_t voltage_mv);
    uint16_t getChannelVoltage(PowerChannel channel);
    cube32_result_t enableChannel(PowerChannel channel, bool enable);
    bool isChannelEnabled(PowerChannel channel);

    // ---- Power Control ----
    cube32_result_t shutdown();
    cube32_result_t reset();

    // ---- Temperature ----
    float getTemperature();

    // ---- Display Power Control ----
    /**
     * @brief Set display backlight power (ALDO3)
     * @param enable true to turn on, false to turn off
     * @return CUBE32_OK on success
     */
    cube32_result_t setDisplayBacklight(bool enable);

    /**
     * @brief Set display backlight voltage (ALDO3)
     * @param voltage_mv Voltage in millivolts (500-3500mV)
     * @return CUBE32_OK on success
     */
    cube32_result_t setDisplayBacklightVoltage(uint16_t voltage_mv);

    /**
     * @brief Check if display backlight is enabled
     * @return true if backlight power is on
     */
    bool isDisplayBacklightEnabled();

    // ---- USB OTG Power Control ----
    /**
     * @brief Enable/disable USB OTG 5V power output (via BLDO2)
     * 
     * BLDO2 is connected to a power switch that controls the USB OTG 5V output.
     * When enabled, the ESP32-S3 can power USB devices connected to its USB port.
     * 
     * @param enable true to turn on USB OTG power, false to turn off
     * @return CUBE32_OK on success
     */
    cube32_result_t setUsbOtgPower(bool enable);

    /**
     * @brief Set USB OTG power switch control voltage (BLDO2)
     * 
     * Typically set to 3.3V to drive the power switch.
     * 
     * @param voltage_mv Voltage in millivolts (500-3500mV, typically 3300mV)
     * @return CUBE32_OK on success
     */
    cube32_result_t setUsbOtgVoltage(uint16_t voltage_mv);

    /**
     * @brief Check if USB OTG power is enabled
     * @return true if USB OTG power is on
     */
    bool isUsbOtgPowerEnabled();

    /**
     * @brief Get USB OTG control voltage
     * @return Voltage in millivolts
     */
    uint16_t getUsbOtgVoltage();

    // ---- Direct Access to XPowersLib (advanced usage) ----
    XPowersAXP2101* getDriver() { return &m_pmu; }

    // ---- Utility ----
    static const char* chargeStatusToString(ChargeStatus status);

    // Singleton - no copy/move
    PMU(const PMU&) = delete;
    PMU& operator=(const PMU&) = delete;

private:
    PMU() = default;
    ~PMU();

    /**
     * @brief Apply CUBE32-S3 board-specific configuration
     */
    cube32_result_t configureCube32S3();

    void lock();
    void unlock();

    XPowersAXP2101 m_pmu;
    bool m_initialized = false;
    SemaphoreHandle_t m_mutex = nullptr;
};

} // namespace cube32

#endif // CUBE32_DRIVERS_PMU_AXP2101_H
