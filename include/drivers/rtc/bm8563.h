/**
 * @file bm8563.h
 * @brief CUBE32 RTC (Real-Time Clock) Driver - BM8563
 * 
 * This driver provides a C++ class interface for the BM8563 RTC chip
 * with support for:
 * - Date/Time get and set
 * - Timer (countdown timer with interrupt)
 * - Alarm (daily/weekly alarm with interrupt)
 * - Clock output control
 * 
 * The driver uses the shared I2C bus from utils/i2c_bus.h
 */

#ifndef CUBE32_DRIVERS_RTC_BM8563_H
#define CUBE32_DRIVERS_RTC_BM8563_H

#include "utils/common.h"
#include "utils/i2c_bus.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <cstdint>
#include <ctime>
#include <functional>

// ============================================================================
// Constants
// ============================================================================

#define CUBE32_RTC_BM8563_ADDR      0x51    ///< BM8563 I2C address

namespace cube32 {

// ============================================================================
// Types
// ============================================================================

/**
 * @brief RTC Date structure
 */
struct RTCDate {
    uint16_t year = 2024;     ///< Year (2000-2099)
    uint8_t month = 1;        ///< Month (1-12)
    uint8_t day = 1;          ///< Day of month (1-31)
    uint8_t weekday = 0;      ///< Day of week (0=Sunday, 6=Saturday)

    bool operator==(const RTCDate& other) const {
        return year == other.year && month == other.month && 
               day == other.day && weekday == other.weekday;
    }
    bool operator!=(const RTCDate& other) const { return !(*this == other); }
};

/**
 * @brief RTC Time structure
 */
struct RTCTime {
    uint8_t hour = 0;         ///< Hour (0-23)
    uint8_t minute = 0;       ///< Minute (0-59)
    uint8_t second = 0;       ///< Second (0-59)

    bool operator==(const RTCTime& other) const {
        return hour == other.hour && minute == other.minute && second == other.second;
    }
    bool operator!=(const RTCTime& other) const { return !(*this == other); }
};

/**
 * @brief RTC DateTime structure (combined date and time)
 */
struct RTCDateTime {
    RTCDate date;
    RTCTime time;

    bool operator==(const RTCDateTime& other) const {
        return date == other.date && time == other.time;
    }
    bool operator!=(const RTCDateTime& other) const { return !(*this == other); }
};

/**
 * @brief Alarm configuration structure
 */
struct RTCAlarm {
    uint8_t minute = 0;       ///< Alarm minute (0-59), 0xFF = disabled
    uint8_t hour = 0;         ///< Alarm hour (0-23), 0xFF = disabled
    uint8_t day = 0;          ///< Alarm day (1-31), 0xFF = disabled
    uint8_t weekday = 0;      ///< Alarm weekday (0-6), 0xFF = disabled
    bool enabled = false;     ///< Alarm enabled
};

/**
 * @brief Timer clock source options
 */
enum class TimerClockSource : uint8_t {
    FREQ_4096_HZ = 0,         ///< 4096 Hz (244μs period)
    FREQ_64_HZ = 1,           ///< 64 Hz (15.625ms period)
    FREQ_1_HZ = 2,            ///< 1 Hz (1 second period)
    FREQ_1_60_HZ = 3,         ///< 1/60 Hz (1 minute period)
};

/**
 * @brief Clock output frequency options
 */
enum class ClockOutFreq : uint8_t {
    FREQ_32768_HZ = 0,        ///< 32.768 kHz
    FREQ_1024_HZ = 1,         ///< 1.024 kHz
    FREQ_32_HZ = 2,           ///< 32 Hz
    FREQ_1_HZ = 3,            ///< 1 Hz
    DISABLED = 0x80,          ///< Clock output disabled
};

/**
 * @brief Interrupt callback type
 */
using RTCInterruptCallback = std::function<void()>;

// ============================================================================
// BM8563 RTC Class
// ============================================================================

/**
 * @brief BM8563 Real-Time Clock driver class (Singleton)
 * 
 * This class provides access to the BM8563 RTC chip functionality including:
 * - Setting and reading date/time
 * - Configuring countdown timer with interrupt
 * - Configuring alarm with interrupt
 * - Controlling clock output pin
 */
class RTC {
public:
    /**
     * @brief Get the singleton instance
     * @return Reference to the RTC instance
     */
    static RTC& instance();

    // Delete copy constructor and assignment operator
    RTC(const RTC&) = delete;
    RTC& operator=(const RTC&) = delete;

    /**
     * @brief Initialize the RTC driver
     * @return CUBE32_OK on success, error code otherwise
     */
    cube32_result_t begin();

    /**
     * @brief Check if RTC is initialized
     * @return true if initialized
     */
    bool isInitialized() const { return m_initialized; }

    // ========================================================================
    // Date/Time Functions
    // ========================================================================

    /**
     * @brief Get current date and time
     * @param dt Output datetime structure
     * @return CUBE32_OK on success
     */
    cube32_result_t getDateTime(RTCDateTime& dt);

    /**
     * @brief Set date and time
     * @param dt Datetime structure to set
     * @return CUBE32_OK on success
     */
    cube32_result_t setDateTime(const RTCDateTime& dt);

    /**
     * @brief Get current date and time in local timezone
     * @param dt Output datetime structure in local timezone
     * @return CUBE32_OK on success
     * @note This function gets RTC time and converts to local timezone
     *       using the configured CUBE32_RTC_TIMEZONE setting
     */
    cube32_result_t getLocalDateTime(RTCDateTime& dt);

    /**
     * @brief Set date and time from local timezone
     * @param dt Datetime structure in local timezone
     * @return CUBE32_OK on success
     * @note This function converts local time to RTC time format
     */
    cube32_result_t setLocalDateTime(const RTCDateTime& dt);

    /**
     * @brief Get current time as std::tm structure
     * @param tm Output tm structure
     * @return CUBE32_OK on success
     */
    cube32_result_t getTime(std::tm& tm);

    /**
     * @brief Set time from std::tm structure
     * @param tm Time to set
     * @return CUBE32_OK on success
     */
    cube32_result_t setTime(const std::tm& tm);

    /**
     * @brief Get Unix timestamp
     * @return Unix timestamp (seconds since 1970-01-01 00:00:00 UTC)
     */
    time_t getUnixTime();

    /**
     * @brief Set time from Unix timestamp
     * @param timestamp Unix timestamp
     * @return CUBE32_OK on success
     */
    cube32_result_t setUnixTime(time_t timestamp);

    /**
     * @brief Get configured timezone string
     * @return Timezone string (e.g., "SGT-8")
     */
    const char* getTimezone() const;

    /**
     * @brief Set system time from RTC
     * @return CUBE32_OK on success
     * @note This sets the ESP32 system time to match RTC time
     */
    cube32_result_t syncSystemTime();

    // ========================================================================
    // Timer Functions (Countdown Timer)
    // ========================================================================

    /**
     * @brief Configure and start countdown timer
     * @param value Timer countdown value (0-255)
     * @param source Timer clock source
     * @param interruptEnable Enable timer interrupt
     * @return CUBE32_OK on success
     */
    cube32_result_t setTimer(uint8_t value, TimerClockSource source, bool interruptEnable = true);

    /**
     * @brief Stop the timer
     * @return CUBE32_OK on success
     */
    cube32_result_t stopTimer();

    /**
     * @brief Check if timer has expired
     * @return true if timer flag is set
     */
    bool isTimerExpired();

    /**
     * @brief Clear timer flag
     * @return CUBE32_OK on success
     */
    cube32_result_t clearTimerFlag();

    /**
     * @brief Get timer counter value
     * @return Current timer counter value
     */
    uint8_t getTimerValue();

    /**
     * @brief Set timer interrupt callback
     * @param callback Function to call when timer expires
     */
    void setTimerCallback(RTCInterruptCallback callback) { m_timerCallback = callback; }

    // ========================================================================
    // Alarm Functions
    // ========================================================================

    /**
     * @brief Configure alarm
     * @param alarm Alarm configuration
     * @return CUBE32_OK on success
     */
    cube32_result_t setAlarm(const RTCAlarm& alarm);

    /**
     * @brief Get current alarm configuration
     * @param alarm Output alarm configuration
     * @return CUBE32_OK on success
     */
    cube32_result_t getAlarm(RTCAlarm& alarm);

    /**
     * @brief Enable or disable alarm
     * @param enable Enable flag
     * @return CUBE32_OK on success
     */
    cube32_result_t enableAlarm(bool enable);

    /**
     * @brief Check if alarm has triggered
     * @return true if alarm flag is set
     */
    bool isAlarmTriggered();

    /**
     * @brief Clear alarm flag
     * @return CUBE32_OK on success
     */
    cube32_result_t clearAlarmFlag();

    /**
     * @brief Set alarm interrupt callback
     * @param callback Function to call when alarm triggers
     */
    void setAlarmCallback(RTCInterruptCallback callback) { m_alarmCallback = callback; }

    // ========================================================================
    // Clock Output Functions
    // ========================================================================

    /**
     * @brief Set clock output frequency
     * @param freq Output frequency
     * @return CUBE32_OK on success
     */
    cube32_result_t setClockOutput(ClockOutFreq freq);

    /**
     * @brief Disable clock output
     * @return CUBE32_OK on success
     */
    cube32_result_t disableClockOutput();

    // ========================================================================
    // Status Functions
    // ========================================================================

    /**
     * @brief Check if oscillator has stopped (voltage loss detected)
     * @return true if oscillator stopped (time may be invalid)
     */
    bool isOscillatorStopped();

    /**
     * @brief Clear oscillator stopped flag
     * @return CUBE32_OK on success
     */
    cube32_result_t clearOscillatorStoppedFlag();

    /**
     * @brief Print current time to log
     */
    void printDateTime();

private:
    RTC() = default;
    ~RTC();

    // I2C Communication
    cube32_result_t readRegister(uint8_t reg, uint8_t& value);
    cube32_result_t writeRegister(uint8_t reg, uint8_t value);
    cube32_result_t readRegisters(uint8_t reg, uint8_t* data, size_t len);
    cube32_result_t writeRegisters(uint8_t reg, const uint8_t* data, size_t len);

    // BCD conversion utilities
    static uint8_t decToBcd(uint8_t val) { return ((val / 10) << 4) | (val % 10); }
    static uint8_t bcdToDec(uint8_t val) { return ((val >> 4) * 10) + (val & 0x0F); }

    // Member variables
    bool m_initialized = false;
    SemaphoreHandle_t m_mutex = nullptr;
    i2c_master_dev_handle_t m_i2c_dev = nullptr;

    // Callbacks
    RTCInterruptCallback m_timerCallback = nullptr;
    RTCInterruptCallback m_alarmCallback = nullptr;

    // BM8563 Register addresses
    static constexpr uint8_t REG_CONTROL1 = 0x00;
    static constexpr uint8_t REG_CONTROL2 = 0x01;
    static constexpr uint8_t REG_SECONDS = 0x02;
    static constexpr uint8_t REG_MINUTES = 0x03;
    static constexpr uint8_t REG_HOURS = 0x04;
    static constexpr uint8_t REG_DAYS = 0x05;
    static constexpr uint8_t REG_WEEKDAYS = 0x06;
    static constexpr uint8_t REG_MONTHS = 0x07;
    static constexpr uint8_t REG_YEARS = 0x08;
    static constexpr uint8_t REG_MINUTE_ALARM = 0x09;
    static constexpr uint8_t REG_HOUR_ALARM = 0x0A;
    static constexpr uint8_t REG_DAY_ALARM = 0x0B;
    static constexpr uint8_t REG_WEEKDAY_ALARM = 0x0C;
    static constexpr uint8_t REG_CLKOUT_CONTROL = 0x0D;
    static constexpr uint8_t REG_TIMER_CONTROL = 0x0E;
    static constexpr uint8_t REG_TIMER = 0x0F;

    // Control register bits
    static constexpr uint8_t CTRL1_TEST1 = 0x80;
    static constexpr uint8_t CTRL1_STOP = 0x20;
    static constexpr uint8_t CTRL1_TESTC = 0x08;

    static constexpr uint8_t CTRL2_TI_TP = 0x10;
    static constexpr uint8_t CTRL2_AF = 0x08;
    static constexpr uint8_t CTRL2_TF = 0x04;
    static constexpr uint8_t CTRL2_AIE = 0x02;
    static constexpr uint8_t CTRL2_TIE = 0x01;

    // Timer control bits
    static constexpr uint8_t TIMER_CTRL_TE = 0x80;
    static constexpr uint8_t TIMER_CTRL_TD_MASK = 0x03;

    // Alarm disable bit
    static constexpr uint8_t ALARM_DISABLE = 0x80;

    // Seconds register VL bit (voltage low/oscillator stopped)
    static constexpr uint8_t SECONDS_VL = 0x80;

    // Century bit in months register
    static constexpr uint8_t MONTHS_CENTURY = 0x80;

    // Clock output control
    static constexpr uint8_t CLKOUT_FE = 0x80;
    static constexpr uint8_t CLKOUT_FD_MASK = 0x03;
};

} // namespace cube32

#endif // CUBE32_DRIVERS_RTC_BM8563_H
