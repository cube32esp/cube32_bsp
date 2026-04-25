/**
 * @file bm8563.cpp
 * @brief CUBE32 RTC (Real-Time Clock) Driver - BM8563 Implementation
 */

#include "drivers/rtc/bm8563.h"
#include "utils/config_manager.h"
#include <esp_log.h>
#include <cstring>
#include <sys/time.h>

static const char *TAG = "bm8563";

namespace cube32 {

// ============================================================================
// Singleton Instance
// ============================================================================

RTC& RTC::instance() {
    static RTC instance;
    return instance;
}

RTC::~RTC() {
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

cube32_result_t RTC::begin() {
    if (m_initialized) {
        ESP_LOGW(TAG, "RTC already initialized");
        return CUBE32_OK;
    }

    ESP_LOGI(TAG, "Initializing BM8563 RTC...");

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
        .device_address = CUBE32_RTC_BM8563_ADDR,
        .scl_speed_hz = 400000,  // 400kHz
    };

    esp_err_t err = i2c_master_bus_add_device(I2CBus::instance().getHandle(), &dev_cfg, &m_i2c_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(err));
        vSemaphoreDelete(m_mutex);
        m_mutex = nullptr;
        return CUBE32_ERROR;
    }

    // Try to read control register to verify communication
    uint8_t ctrl1;
    cube32_result_t ret = readRegister(REG_CONTROL1, ctrl1);
    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to communicate with BM8563");
        vSemaphoreDelete(m_mutex);
        m_mutex = nullptr;
        return ret;
    }

    ESP_LOGI(TAG, "BM8563 Control1: 0x%02X", ctrl1);

    // Initialize control registers
    // Clear STOP bit to start oscillator, clear test bits
    ret = writeRegister(REG_CONTROL1, 0x00);
    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to initialize control1");
        vSemaphoreDelete(m_mutex);
        m_mutex = nullptr;
        return ret;
    }

    // Clear all interrupt flags and disable interrupts initially
    ret = writeRegister(REG_CONTROL2, 0x00);
    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to initialize control2");
        vSemaphoreDelete(m_mutex);
        m_mutex = nullptr;
        return ret;
    }

    // Check if oscillator stopped (voltage loss)
    if (isOscillatorStopped()) {
        ESP_LOGW(TAG, "Oscillator stopped flag set - time may be invalid");
    }

    m_initialized = true;
    
    // Set system timezone from NVS config
    const cube32_cfg_t* cfg = cube32_cfg();
    if (cfg->loaded && cfg->rtc_timezone[0] != '\0') {
        setenv("TZ", cfg->rtc_timezone, 1);
        tzset();
        ESP_LOGI(TAG, "Timezone set to: %s", cfg->rtc_timezone);
    }

    // Sync system time from RTC
    syncSystemTime();
    
    ESP_LOGI(TAG, "BM8563 RTC initialized successfully");

    // Print current time
    printDateTime();

    return CUBE32_OK;
}

// ============================================================================
// I2C Communication
// ============================================================================

cube32_result_t RTC::readRegister(uint8_t reg, uint8_t& value) {
    if (!m_i2c_dev) return CUBE32_ERROR;
    
    esp_err_t ret = i2c_master_transmit_receive(m_i2c_dev, &reg, 1, &value, 1, -1);
    return (ret == ESP_OK) ? CUBE32_OK : CUBE32_ERROR;
}

cube32_result_t RTC::writeRegister(uint8_t reg, uint8_t value) {
    if (!m_i2c_dev) return CUBE32_ERROR;
    
    uint8_t data[2] = {reg, value};
    esp_err_t ret = i2c_master_transmit(m_i2c_dev, data, 2, -1);
    return (ret == ESP_OK) ? CUBE32_OK : CUBE32_ERROR;
}

cube32_result_t RTC::readRegisters(uint8_t reg, uint8_t* data, size_t len) {
    if (!m_i2c_dev) return CUBE32_ERROR;
    
    esp_err_t ret = i2c_master_transmit_receive(m_i2c_dev, &reg, 1, data, len, -1);
    return (ret == ESP_OK) ? CUBE32_OK : CUBE32_ERROR;
}

cube32_result_t RTC::writeRegisters(uint8_t reg, const uint8_t* data, size_t len) {
    if (!m_i2c_dev) return CUBE32_ERROR;
    
    uint8_t* buf = new uint8_t[len + 1];
    if (!buf) return CUBE32_NO_MEM;
    
    buf[0] = reg;
    memcpy(buf + 1, data, len);
    
    esp_err_t ret = i2c_master_transmit(m_i2c_dev, buf, len + 1, -1);
    delete[] buf;
    
    return (ret == ESP_OK) ? CUBE32_OK : CUBE32_ERROR;
}

// ============================================================================
// Date/Time Functions
// ============================================================================

cube32_result_t RTC::getDateTime(RTCDateTime& dt) {
    if (!m_initialized) return CUBE32_ERROR;

    xSemaphoreTake(m_mutex, portMAX_DELAY);

    uint8_t data[7];
    cube32_result_t ret = readRegisters(REG_SECONDS, data, 7);
    
    if (ret == CUBE32_OK) {
        dt.time.second = bcdToDec(data[0] & 0x7F);
        dt.time.minute = bcdToDec(data[1] & 0x7F);
        dt.time.hour = bcdToDec(data[2] & 0x3F);
        dt.date.day = bcdToDec(data[3] & 0x3F);
        dt.date.weekday = bcdToDec(data[4] & 0x07);
        dt.date.month = bcdToDec(data[5] & 0x1F);
        
        // Handle century bit
        int baseYear = (data[5] & MONTHS_CENTURY) ? 1900 : 2000;
        dt.date.year = baseYear + bcdToDec(data[6]);
    }

    xSemaphoreGive(m_mutex);
    return ret;
}

cube32_result_t RTC::setDateTime(const RTCDateTime& dt) {
    if (!m_initialized) return CUBE32_ERROR;

    xSemaphoreTake(m_mutex, portMAX_DELAY);

    uint8_t data[7];
    data[0] = decToBcd(dt.time.second) & 0x7F;  // Clear VL bit
    data[1] = decToBcd(dt.time.minute);
    data[2] = decToBcd(dt.time.hour);
    data[3] = decToBcd(dt.date.day);
    data[4] = decToBcd(dt.date.weekday);
    
    // Set century bit for dates before 2000
    uint8_t centuryBit = (dt.date.year < 2000) ? MONTHS_CENTURY : 0;
    data[5] = decToBcd(dt.date.month) | centuryBit;
    data[6] = decToBcd(dt.date.year % 100);

    cube32_result_t ret = writeRegisters(REG_SECONDS, data, 7);

    xSemaphoreGive(m_mutex);

    if (ret == CUBE32_OK) {
        ESP_LOGI(TAG, "DateTime set to %04d-%02d-%02d %02d:%02d:%02d",
                 dt.date.year, dt.date.month, dt.date.day,
                 dt.time.hour, dt.time.minute, dt.time.second);
    }

    return ret;
}

cube32_result_t RTC::getTime(std::tm& tm) {
    RTCDateTime dt;
    cube32_result_t ret = getDateTime(dt);
    
    if (ret == CUBE32_OK) {
        tm.tm_sec = dt.time.second;
        tm.tm_min = dt.time.minute;
        tm.tm_hour = dt.time.hour;
        tm.tm_mday = dt.date.day;
        tm.tm_mon = dt.date.month - 1;  // tm_mon is 0-11
        tm.tm_year = dt.date.year - 1900;  // tm_year is years since 1900
        tm.tm_wday = dt.date.weekday;
        tm.tm_yday = -1;  // Let mktime calculate
        tm.tm_isdst = -1;  // Let system determine DST

        // Normalize the time structure
        mktime(&tm);
    }

    return ret;
}

cube32_result_t RTC::setTime(const std::tm& tm) {
    RTCDateTime dt;
    
    dt.time.second = tm.tm_sec;
    dt.time.minute = tm.tm_min;
    dt.time.hour = tm.tm_hour;
    dt.date.day = tm.tm_mday;
    dt.date.month = tm.tm_mon + 1;  // tm_mon is 0-11
    dt.date.year = tm.tm_year + 1900;  // tm_year is years since 1900
    dt.date.weekday = tm.tm_wday;

    return setDateTime(dt);
}

cube32_result_t RTC::getLocalDateTime(RTCDateTime& dt) {
    // Get RTC time
    cube32_result_t ret = getDateTime(dt);
    if (ret != CUBE32_OK) {
        return ret;
    }
    
    // RTC already stores local time, just return it directly
    return CUBE32_OK;
}

cube32_result_t RTC::setLocalDateTime(const RTCDateTime& dt) {
    // Set RTC time (RTC stores local time)
    return setDateTime(dt);
}

time_t RTC::getUnixTime() {
    std::tm tm;
    if (getTime(tm) == CUBE32_OK) {
        return mktime(&tm);
    }
    return 0;
}

cube32_result_t RTC::setUnixTime(time_t timestamp) {
    std::tm* tm = localtime(&timestamp);
    if (tm) {
        return setTime(*tm);
    }
    return CUBE32_ERROR;
}

// ============================================================================
// Timer Functions
// ============================================================================

cube32_result_t RTC::setTimer(uint8_t value, TimerClockSource source, bool interruptEnable) {
    if (!m_initialized) return CUBE32_ERROR;

    xSemaphoreTake(m_mutex, portMAX_DELAY);

    // Set timer value
    cube32_result_t ret = writeRegister(REG_TIMER, value);
    if (ret != CUBE32_OK) {
        xSemaphoreGive(m_mutex);
        return ret;
    }

    // Configure timer control: enable timer + clock source
    uint8_t timerCtrl = TIMER_CTRL_TE | (static_cast<uint8_t>(source) & TIMER_CTRL_TD_MASK);
    ret = writeRegister(REG_TIMER_CONTROL, timerCtrl);
    if (ret != CUBE32_OK) {
        xSemaphoreGive(m_mutex);
        return ret;
    }

    // Enable/disable timer interrupt
    uint8_t ctrl2;
    ret = readRegister(REG_CONTROL2, ctrl2);
    if (ret == CUBE32_OK) {
        if (interruptEnable) {
            ctrl2 |= CTRL2_TIE;
        } else {
            ctrl2 &= ~CTRL2_TIE;
        }
        // Clear timer flag
        ctrl2 &= ~CTRL2_TF;
        ret = writeRegister(REG_CONTROL2, ctrl2);
    }

    xSemaphoreGive(m_mutex);

    ESP_LOGI(TAG, "Timer set: value=%d, source=%d, interrupt=%d", value, (int)source, interruptEnable);
    return ret;
}

cube32_result_t RTC::stopTimer() {
    if (!m_initialized) return CUBE32_ERROR;

    xSemaphoreTake(m_mutex, portMAX_DELAY);

    // Disable timer
    cube32_result_t ret = writeRegister(REG_TIMER_CONTROL, 0x00);

    // Disable timer interrupt
    if (ret == CUBE32_OK) {
        uint8_t ctrl2;
        ret = readRegister(REG_CONTROL2, ctrl2);
        if (ret == CUBE32_OK) {
            ctrl2 &= ~CTRL2_TIE;
            ret = writeRegister(REG_CONTROL2, ctrl2);
        }
    }

    xSemaphoreGive(m_mutex);
    return ret;
}

bool RTC::isTimerExpired() {
    if (!m_initialized) return false;

    uint8_t ctrl2;
    if (readRegister(REG_CONTROL2, ctrl2) == CUBE32_OK) {
        return (ctrl2 & CTRL2_TF) != 0;
    }
    return false;
}

cube32_result_t RTC::clearTimerFlag() {
    if (!m_initialized) return CUBE32_ERROR;

    xSemaphoreTake(m_mutex, portMAX_DELAY);

    uint8_t ctrl2;
    cube32_result_t ret = readRegister(REG_CONTROL2, ctrl2);
    if (ret == CUBE32_OK) {
        ctrl2 &= ~CTRL2_TF;
        ret = writeRegister(REG_CONTROL2, ctrl2);
    }

    xSemaphoreGive(m_mutex);
    return ret;
}

uint8_t RTC::getTimerValue() {
    if (!m_initialized) return 0;

    uint8_t value;
    if (readRegister(REG_TIMER, value) == CUBE32_OK) {
        return value;
    }
    return 0;
}

// ============================================================================
// Alarm Functions
// ============================================================================

cube32_result_t RTC::setAlarm(const RTCAlarm& alarm) {
    if (!m_initialized) return CUBE32_ERROR;

    xSemaphoreTake(m_mutex, portMAX_DELAY);

    uint8_t data[4];
    
    // Minute alarm (0x80 bit disables matching)
    data[0] = (alarm.minute == 0xFF) ? ALARM_DISABLE : decToBcd(alarm.minute);
    
    // Hour alarm
    data[1] = (alarm.hour == 0xFF) ? ALARM_DISABLE : decToBcd(alarm.hour);
    
    // Day alarm
    data[2] = (alarm.day == 0xFF) ? ALARM_DISABLE : decToBcd(alarm.day);
    
    // Weekday alarm
    data[3] = (alarm.weekday == 0xFF) ? ALARM_DISABLE : decToBcd(alarm.weekday);

    cube32_result_t ret = writeRegisters(REG_MINUTE_ALARM, data, 4);

    // Enable/disable alarm interrupt
    if (ret == CUBE32_OK) {
        ret = enableAlarm(alarm.enabled);
    }

    xSemaphoreGive(m_mutex);

    ESP_LOGI(TAG, "Alarm set: min=%d, hour=%d, day=%d, wday=%d, enabled=%d",
             alarm.minute, alarm.hour, alarm.day, alarm.weekday, alarm.enabled);
    return ret;
}

cube32_result_t RTC::getAlarm(RTCAlarm& alarm) {
    if (!m_initialized) return CUBE32_ERROR;

    xSemaphoreTake(m_mutex, portMAX_DELAY);

    uint8_t data[4];
    cube32_result_t ret = readRegisters(REG_MINUTE_ALARM, data, 4);

    if (ret == CUBE32_OK) {
        alarm.minute = (data[0] & ALARM_DISABLE) ? 0xFF : bcdToDec(data[0] & 0x7F);
        alarm.hour = (data[1] & ALARM_DISABLE) ? 0xFF : bcdToDec(data[1] & 0x3F);
        alarm.day = (data[2] & ALARM_DISABLE) ? 0xFF : bcdToDec(data[2] & 0x3F);
        alarm.weekday = (data[3] & ALARM_DISABLE) ? 0xFF : bcdToDec(data[3] & 0x07);

        // Check if alarm interrupt is enabled
        uint8_t ctrl2;
        if (readRegister(REG_CONTROL2, ctrl2) == CUBE32_OK) {
            alarm.enabled = (ctrl2 & CTRL2_AIE) != 0;
        }
    }

    xSemaphoreGive(m_mutex);
    return ret;
}

cube32_result_t RTC::enableAlarm(bool enable) {
    if (!m_initialized) return CUBE32_ERROR;

    uint8_t ctrl2;
    cube32_result_t ret = readRegister(REG_CONTROL2, ctrl2);
    if (ret == CUBE32_OK) {
        if (enable) {
            ctrl2 |= CTRL2_AIE;
        } else {
            ctrl2 &= ~CTRL2_AIE;
        }
        // Clear alarm flag
        ctrl2 &= ~CTRL2_AF;
        ret = writeRegister(REG_CONTROL2, ctrl2);
    }
    return ret;
}

bool RTC::isAlarmTriggered() {
    if (!m_initialized) return false;

    uint8_t ctrl2;
    if (readRegister(REG_CONTROL2, ctrl2) == CUBE32_OK) {
        return (ctrl2 & CTRL2_AF) != 0;
    }
    return false;
}

cube32_result_t RTC::clearAlarmFlag() {
    if (!m_initialized) return CUBE32_ERROR;

    xSemaphoreTake(m_mutex, portMAX_DELAY);

    uint8_t ctrl2;
    cube32_result_t ret = readRegister(REG_CONTROL2, ctrl2);
    if (ret == CUBE32_OK) {
        ctrl2 &= ~CTRL2_AF;
        ret = writeRegister(REG_CONTROL2, ctrl2);
    }

    xSemaphoreGive(m_mutex);
    return ret;
}

// ============================================================================
// Clock Output Functions
// ============================================================================

cube32_result_t RTC::setClockOutput(ClockOutFreq freq) {
    if (!m_initialized) return CUBE32_ERROR;

    xSemaphoreTake(m_mutex, portMAX_DELAY);

    uint8_t value;
    if (freq == ClockOutFreq::DISABLED) {
        value = 0x00;  // FE=0 disables output
    } else {
        value = CLKOUT_FE | (static_cast<uint8_t>(freq) & CLKOUT_FD_MASK);
    }

    cube32_result_t ret = writeRegister(REG_CLKOUT_CONTROL, value);

    xSemaphoreGive(m_mutex);
    return ret;
}

cube32_result_t RTC::disableClockOutput() {
    return setClockOutput(ClockOutFreq::DISABLED);
}

// ============================================================================
// Status Functions
// ============================================================================

bool RTC::isOscillatorStopped() {
    if (!m_initialized) return true;

    uint8_t seconds;
    if (readRegister(REG_SECONDS, seconds) == CUBE32_OK) {
        return (seconds & SECONDS_VL) != 0;
    }
    return true;
}

cube32_result_t RTC::clearOscillatorStoppedFlag() {
    if (!m_initialized) return CUBE32_ERROR;

    xSemaphoreTake(m_mutex, portMAX_DELAY);

    uint8_t seconds;
    cube32_result_t ret = readRegister(REG_SECONDS, seconds);
    if (ret == CUBE32_OK) {
        seconds &= ~SECONDS_VL;
        ret = writeRegister(REG_SECONDS, seconds);
    }

    xSemaphoreGive(m_mutex);
    return ret;
}

const char* RTC::getTimezone() const {
    const cube32_cfg_t* cfg = cube32_cfg();
    if (cfg->loaded && cfg->rtc_timezone[0] != '\0') {
        return cfg->rtc_timezone;
    }
    return "UTC0";
}

cube32_result_t RTC::syncSystemTime() {
    if (!m_initialized) return CUBE32_ERROR;
    
    // Get time from RTC
    time_t rtc_time = getUnixTime();
    if (rtc_time == 0) {
        ESP_LOGW(TAG, "Failed to get RTC time for system sync");
        return CUBE32_ERROR;
    }
    
    // Set system time
    struct timeval tv = {
        .tv_sec = rtc_time,
        .tv_usec = 0
    };
    
    if (settimeofday(&tv, NULL) != 0) {
        ESP_LOGE(TAG, "Failed to set system time");
        return CUBE32_ERROR;
    }
    
    ESP_LOGI(TAG, "System time synchronized with RTC");
    return CUBE32_OK;
}

void RTC::printDateTime() {
    RTCDateTime dt;
    if (getDateTime(dt) == CUBE32_OK) {
        ESP_LOGI(TAG, "Current time: %04d-%02d-%02d %02d:%02d:%02d (weekday=%d)",
                 dt.date.year, dt.date.month, dt.date.day,
                 dt.time.hour, dt.time.minute, dt.time.second,
                 dt.date.weekday);
    }
}

} // namespace cube32
