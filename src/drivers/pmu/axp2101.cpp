/**
 * @file axp2101.cpp
 * @brief CUBE32 PMU (Power Management Unit) Driver Implementation - AXP2101 (Pure C++)
 * 
 * This implementation provides a C++ class interface for the AXP2101 PMU,
 * using the shared I2C bus from utils/i2c_bus.
 */

#include "drivers/pmu/axp2101.h"
#include "cube32_config.h"
#include <esp_log.h>
#include <cstring>

static const char* TAG = "cube32_pmu";

namespace cube32 {

// ============================================================================
// PMU Class Implementation
// ============================================================================

PMU& PMU::instance() {
    static PMU s_instance;
    return s_instance;
}

PMU::~PMU() {
    if (m_initialized) {
        end();
    }
}

void PMU::lock() {
    if (m_mutex) {
        xSemaphoreTake(m_mutex, portMAX_DELAY);
    }
}

void PMU::unlock() {
    if (m_mutex) {
        xSemaphoreGive(m_mutex);
    }
}

cube32_result_t PMU::begin() {
    if (m_initialized) {
        ESP_LOGW(TAG, "PMU already initialized");
        return CUBE32_ALREADY_INITIALIZED;
    }

    // Check if shared I2C bus is initialized
    if (!I2CBus::instance().isInitialized()) {
        ESP_LOGE(TAG, "Shared I2C bus not initialized. Call I2CBus::instance().init() first.");
        return CUBE32_NOT_INITIALIZED;
    }

    ESP_LOGI(TAG, "Initializing AXP2101 PMU...");

    // Create mutex for thread safety
    m_mutex = xSemaphoreCreateMutex();
    if (m_mutex == nullptr) {
        ESP_LOGE(TAG, "Failed to create PMU mutex");
        return CUBE32_NO_MEM;
    }

    // Get the shared I2C bus handle
    i2c_master_bus_handle_t bus_handle = I2CBus::instance().getHandle();

    // Probe for AXP2101
    if (I2CBus::instance().probe(CUBE32_PMU_AXP2101_ADDR) != CUBE32_OK) {
        ESP_LOGE(TAG, "AXP2101 not found at address 0x%02X", CUBE32_PMU_AXP2101_ADDR);
        vSemaphoreDelete(m_mutex);
        m_mutex = nullptr;
        return CUBE32_NOT_FOUND;
    }

    // Initialize AXP2101 using the shared I2C bus
    if (!m_pmu.begin(bus_handle, AXP2101_SLAVE_ADDRESS)) {
        ESP_LOGE(TAG, "Failed to initialize AXP2101");
        vSemaphoreDelete(m_mutex);
        m_mutex = nullptr;
        return CUBE32_ERROR;
    }

    // Verify chip ID
    uint8_t chip_id = m_pmu.getChipID();
    ESP_LOGI(TAG, "AXP2101 Chip ID: 0x%02X (expected 0x%02X)", chip_id, CUBE32_PMU_AXP2101_CHIP_ID);

    if (chip_id != CUBE32_PMU_AXP2101_CHIP_ID) {
        ESP_LOGW(TAG, "Unexpected chip ID, but continuing...");
    }

    // Apply CUBE32-S3 board configuration
    cube32_result_t ret = configureCube32S3();
    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to configure PMU for CUBE32-S3");
        vSemaphoreDelete(m_mutex);
        m_mutex = nullptr;
        return ret;
    }

    m_initialized = true;
    ESP_LOGI(TAG, "PMU initialized successfully");
    return CUBE32_OK;
}

cube32_result_t PMU::end() {
    if (!m_initialized) {
        return CUBE32_NOT_INITIALIZED;
    }

    lock();
    
    // No explicit cleanup needed for XPowersAXP2101
    // The I2C bus is managed by I2CBus singleton
    
    m_initialized = false;
    
    unlock();

    if (m_mutex) {
        vSemaphoreDelete(m_mutex);
        m_mutex = nullptr;
    }

    ESP_LOGI(TAG, "PMU deinitialized");
    return CUBE32_OK;
}

cube32_result_t PMU::configureCube32S3() {
    ESP_LOGI(TAG, "Configuring AXP2101 for CUBE32-S3");

    // Disable All DCs except DC1 (main 3.3V)
    m_pmu.disableDC2();
    m_pmu.disableDC3();
    m_pmu.disableDC4();
    m_pmu.disableDC5();

    // Disable All LDOs initially
    m_pmu.disableALDO1();
    m_pmu.disableALDO2();
    m_pmu.disableALDO3();
    m_pmu.disableALDO4();
    m_pmu.disableBLDO1();
    m_pmu.disableBLDO2();
    m_pmu.disableDLDO1();
    m_pmu.disableDLDO2();

    // Set input current limit to 2000mA
    m_pmu.setVbusCurrentLimit(XPOWERS_AXP2101_VBUS_CUR_LIM_2000MA);

    // DLDO1 - SysOn - 3.3V
    m_pmu.setDLDO1Voltage(3300);
    m_pmu.enableDLDO1();
    ESP_LOGI(TAG, "DLDO1 (SysOn): %s  %u mV", 
             m_pmu.isEnableDLDO1() ? "ON" : "OFF", m_pmu.getDLDO1Voltage());

    // BLDO2 - OTG_EN - 3.3V (disabled by default)
    m_pmu.setBLDO2Voltage(3300);
    ESP_LOGI(TAG, "BLDO2 (OTG_EN): %s  %u mV", 
             m_pmu.isEnableBLDO2() ? "ON" : "OFF", m_pmu.getBLDO2Voltage());

    // ALDO3 - LCD_BL - 3.1V (keep disabled, display driver will enable it)
    m_pmu.setALDO3Voltage(3100);
    // Note: ALDO3 is NOT enabled here to prevent white flash during boot
    // The LVGL driver will enable it after the first frame is drawn
    ESP_LOGI(TAG, "ALDO3 (LCD_BL): %s  %u mV (will be enabled by display driver)", 
             m_pmu.isEnableALDO3() ? "ON" : "OFF", m_pmu.getALDO3Voltage());

    // ALDO2 - CAM_DVDD - 1.2V
    m_pmu.setALDO2Voltage(1200);
    m_pmu.enableALDO2();
    ESP_LOGI(TAG, "ALDO2 (CAM_DVDD): %s  %u mV", 
             m_pmu.isEnableALDO2() ? "ON" : "OFF", m_pmu.getALDO2Voltage());

    // ALDO1 - CAM_DOVDD - 2.8V
    m_pmu.setALDO1Voltage(2800);
    m_pmu.enableALDO1();
    ESP_LOGI(TAG, "ALDO1 (CAM_DOVDD): %s  %u mV", 
             m_pmu.isEnableALDO1() ? "ON" : "OFF", m_pmu.getALDO1Voltage());

    // ALDO4 - CAM_AVDD - 2.8V
    m_pmu.setALDO4Voltage(2800);
    m_pmu.enableALDO4();
    ESP_LOGI(TAG, "ALDO4 (CAM_AVDD): %s  %u mV", 
             m_pmu.isEnableALDO4() ? "ON" : "OFF", m_pmu.getALDO4Voltage());

    // Power key configuration
    m_pmu.setPowerKeyPressOffTime(XPOWERS_POWEROFF_4S);
    m_pmu.setPowerKeyPressOnTime(XPOWERS_POWERON_1S);

    // Disable TS pin measurement (no battery temp sensor)
    m_pmu.disableTSPinMeasure();

    // Charge configuration
    m_pmu.setPrechargeCurr(XPOWERS_AXP2101_PRECHARGE_75MA);
    m_pmu.setChargerTerminationCurr(XPOWERS_AXP2101_CHG_ITERM_25MA);
    m_pmu.setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_300MA);
    m_pmu.setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V2);
    m_pmu.setLinearChargerVsysDpm(XPOWERS_AXP2101_VSYS_VOL_4V1);
    m_pmu.setVbusVoltageLimit(XPOWERS_AXP2101_VBUS_VOL_LIM_3V88);
    m_pmu.setSysPowerDownVoltage(3000);

    // Enable voltage measurements
    m_pmu.enableBattVoltageMeasure();
    m_pmu.enableSystemVoltageMeasure();
    m_pmu.enableVbusVoltageMeasure();

    // Wait for voltages to stabilize
    vTaskDelay(pdMS_TO_TICKS(200));

    return CUBE32_OK;
}

uint8_t PMU::getChipId() const {
    if (!m_initialized) return 0;
    return const_cast<XPowersAXP2101&>(m_pmu).getChipID();
}

cube32_result_t PMU::getStatus(PMUStatus& status) {
    if (!m_initialized) return CUBE32_NOT_INITIALIZED;

    lock();
    
    status.isCharging = m_pmu.isCharging();
    status.isDischarging = m_pmu.isDischarge();
    status.isStandby = m_pmu.isStandby();
    status.vbusPresent = m_pmu.isVbusIn();
    status.vbusGood = m_pmu.isVbusGood();
    status.batteryPresent = m_pmu.isBatteryConnect();
    status.batteryVoltage = m_pmu.getBattVoltage();
    status.vbusVoltage = m_pmu.getVbusVoltage();
    status.systemVoltage = m_pmu.getSystemVoltage();
    status.batteryPercent = m_pmu.getBatteryPercent();
    status.temperature = m_pmu.getTemperature();
    status.chargeStatus = m_pmu.getChargerStatus();

    unlock();
    return CUBE32_OK;
}

void PMU::printStatus() {
    if (!m_initialized) {
        ESP_LOGW(TAG, "PMU not initialized");
        return;
    }

    PMUStatus status;
    getStatus(status);

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "AXP2101 PMU Status");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Chip ID: 0x%02X", getChipId());
    ESP_LOGI(TAG, "VBUS: %s (%s), %u mV", 
             status.vbusPresent ? "Present" : "Not present",
             status.vbusGood ? "Good" : "Low",
             status.vbusVoltage);
    ESP_LOGI(TAG, "Battery: %s, %d%%, %u mV",
             status.batteryPresent ? "Connected" : "Not connected",
             status.batteryPercent,
             status.batteryVoltage);
    ESP_LOGI(TAG, "Charging: %s, Status: %s",
             status.isCharging ? "Yes" : "No",
             chargeStatusToString(static_cast<ChargeStatus>(status.chargeStatus)));
    ESP_LOGI(TAG, "System: %u mV, Temp: %.1f°C",
             status.systemVoltage, status.temperature/10);
    ESP_LOGI(TAG, "========================================");
}

bool PMU::isBatteryPresent() {
    if (!m_initialized) return false;
    lock();
    bool result = m_pmu.isBatteryConnect();
    unlock();
    return result;
}

bool PMU::isCharging() {
    if (!m_initialized) return false;
    lock();
    bool result = m_pmu.isCharging();
    unlock();
    return result;
}

bool PMU::isDischarging() {
    if (!m_initialized) return false;
    lock();
    bool result = m_pmu.isDischarge();
    unlock();
    return result;
}

int PMU::getBatteryPercent() {
    if (!m_initialized) return -1;
    lock();
    int result = m_pmu.getBatteryPercent();
    unlock();
    return result;
}

uint16_t PMU::getBatteryVoltage() {
    if (!m_initialized) return 0;
    lock();
    uint16_t result = m_pmu.getBattVoltage();
    unlock();
    return result;
}

bool PMU::isVbusPresent() {
    if (!m_initialized) return false;
    lock();
    bool result = m_pmu.isVbusIn();
    unlock();
    return result;
}

bool PMU::isVbusGood() {
    if (!m_initialized) return false;
    lock();
    bool result = m_pmu.isVbusGood();
    unlock();
    return result;
}

uint16_t PMU::getVbusVoltage() {
    if (!m_initialized) return 0;
    lock();
    uint16_t result = m_pmu.getVbusVoltage();
    unlock();
    return result;
}

uint16_t PMU::getSystemVoltage() {
    if (!m_initialized) return 0;
    lock();
    uint16_t result = m_pmu.getSystemVoltage();
    unlock();
    return result;
}

cube32_result_t PMU::setChargeCurrent(ChargeCurrent current) {
    if (!m_initialized) return CUBE32_NOT_INITIALIZED;
    
    lock();
    
    // Map mA value to XPowers constant
    xpowers_axp2101_chg_curr_t xp_curr;
    switch (current) {
        case ChargeCurrent::MA_100:  xp_curr = XPOWERS_AXP2101_CHG_CUR_100MA; break;
        case ChargeCurrent::MA_125:  xp_curr = XPOWERS_AXP2101_CHG_CUR_125MA; break;
        case ChargeCurrent::MA_150:  xp_curr = XPOWERS_AXP2101_CHG_CUR_150MA; break;
        case ChargeCurrent::MA_175:  xp_curr = XPOWERS_AXP2101_CHG_CUR_175MA; break;
        case ChargeCurrent::MA_200:  xp_curr = XPOWERS_AXP2101_CHG_CUR_200MA; break;
        case ChargeCurrent::MA_300:  xp_curr = XPOWERS_AXP2101_CHG_CUR_300MA; break;
        case ChargeCurrent::MA_400:  xp_curr = XPOWERS_AXP2101_CHG_CUR_400MA; break;
        case ChargeCurrent::MA_500:  xp_curr = XPOWERS_AXP2101_CHG_CUR_500MA; break;
        case ChargeCurrent::MA_600:  xp_curr = XPOWERS_AXP2101_CHG_CUR_600MA; break;
        case ChargeCurrent::MA_700:  xp_curr = XPOWERS_AXP2101_CHG_CUR_700MA; break;
        case ChargeCurrent::MA_800:  xp_curr = XPOWERS_AXP2101_CHG_CUR_800MA; break;
        case ChargeCurrent::MA_900:  xp_curr = XPOWERS_AXP2101_CHG_CUR_900MA; break;
        case ChargeCurrent::MA_1000: xp_curr = XPOWERS_AXP2101_CHG_CUR_1000MA; break;
        default:
            unlock();
            return CUBE32_INVALID_ARG;
    }

    bool ok = m_pmu.setChargerConstantCurr(xp_curr);
    unlock();
    return ok ? CUBE32_OK : CUBE32_ERROR;
}

cube32_result_t PMU::enableCharging(bool enable) {
    if (!m_initialized) return CUBE32_NOT_INITIALIZED;
    lock();
    if (enable) {
        m_pmu.enableCellbatteryCharge();
    } else {
        m_pmu.disableCellbatteryCharge();
    }
    unlock();
    return CUBE32_OK;
}

ChargeStatus PMU::getChargeStatus() {
    if (!m_initialized) return ChargeStatus::Stopped;
    lock();
    uint8_t status = m_pmu.getChargerStatus();
    unlock();
    return static_cast<ChargeStatus>(status);
}

cube32_result_t PMU::setChannelVoltage(PowerChannel channel, uint16_t voltage_mv) {
    if (!m_initialized) return CUBE32_NOT_INITIALIZED;
    
    lock();
    bool ok = true;
    
    switch (channel) {
        case PowerChannel::DC1:   ok = m_pmu.setDC1Voltage(voltage_mv); break;
        case PowerChannel::DC2:   ok = m_pmu.setDC2Voltage(voltage_mv); break;
        case PowerChannel::DC3:   ok = m_pmu.setDC3Voltage(voltage_mv); break;
        case PowerChannel::DC4:   ok = m_pmu.setDC4Voltage(voltage_mv); break;
        case PowerChannel::DC5:   ok = m_pmu.setDC5Voltage(voltage_mv); break;
        case PowerChannel::ALDO1: ok = m_pmu.setALDO1Voltage(voltage_mv); break;
        case PowerChannel::ALDO2: ok = m_pmu.setALDO2Voltage(voltage_mv); break;
        case PowerChannel::ALDO3: ok = m_pmu.setALDO3Voltage(voltage_mv); break;
        case PowerChannel::ALDO4: ok = m_pmu.setALDO4Voltage(voltage_mv); break;
        case PowerChannel::BLDO1: ok = m_pmu.setBLDO1Voltage(voltage_mv); break;
        case PowerChannel::BLDO2: ok = m_pmu.setBLDO2Voltage(voltage_mv); break;
        case PowerChannel::DLDO1: ok = m_pmu.setDLDO1Voltage(voltage_mv); break;
        case PowerChannel::DLDO2: ok = m_pmu.setDLDO2Voltage(voltage_mv); break;
        default:
            unlock();
            return CUBE32_INVALID_ARG;
    }
    
    unlock();
    return ok ? CUBE32_OK : CUBE32_ERROR;
}

uint16_t PMU::getChannelVoltage(PowerChannel channel) {
    if (!m_initialized) return 0;
    
    lock();
    uint16_t voltage = 0;
    
    switch (channel) {
        case PowerChannel::DC1:   voltage = m_pmu.getDC1Voltage(); break;
        case PowerChannel::DC2:   voltage = m_pmu.getDC2Voltage(); break;
        case PowerChannel::DC3:   voltage = m_pmu.getDC3Voltage(); break;
        case PowerChannel::DC4:   voltage = m_pmu.getDC4Voltage(); break;
        case PowerChannel::DC5:   voltage = m_pmu.getDC5Voltage(); break;
        case PowerChannel::ALDO1: voltage = m_pmu.getALDO1Voltage(); break;
        case PowerChannel::ALDO2: voltage = m_pmu.getALDO2Voltage(); break;
        case PowerChannel::ALDO3: voltage = m_pmu.getALDO3Voltage(); break;
        case PowerChannel::ALDO4: voltage = m_pmu.getALDO4Voltage(); break;
        case PowerChannel::BLDO1: voltage = m_pmu.getBLDO1Voltage(); break;
        case PowerChannel::BLDO2: voltage = m_pmu.getBLDO2Voltage(); break;
        case PowerChannel::DLDO1: voltage = m_pmu.getDLDO1Voltage(); break;
        case PowerChannel::DLDO2: voltage = m_pmu.getDLDO2Voltage(); break;
        default: break;
    }
    
    unlock();
    return voltage;
}

cube32_result_t PMU::enableChannel(PowerChannel channel, bool enable) {
    if (!m_initialized) return CUBE32_NOT_INITIALIZED;
    
    lock();
    
    switch (channel) {
        case PowerChannel::DC1:   enable ? m_pmu.enableDC1() : m_pmu.disableDC1(); break;
        case PowerChannel::DC2:   enable ? m_pmu.enableDC2() : m_pmu.disableDC2(); break;
        case PowerChannel::DC3:   enable ? m_pmu.enableDC3() : m_pmu.disableDC3(); break;
        case PowerChannel::DC4:   enable ? m_pmu.enableDC4() : m_pmu.disableDC4(); break;
        case PowerChannel::DC5:   enable ? m_pmu.enableDC5() : m_pmu.disableDC5(); break;
        case PowerChannel::ALDO1: enable ? m_pmu.enableALDO1() : m_pmu.disableALDO1(); break;
        case PowerChannel::ALDO2: enable ? m_pmu.enableALDO2() : m_pmu.disableALDO2(); break;
        case PowerChannel::ALDO3: enable ? m_pmu.enableALDO3() : m_pmu.disableALDO3(); break;
        case PowerChannel::ALDO4: enable ? m_pmu.enableALDO4() : m_pmu.disableALDO4(); break;
        case PowerChannel::BLDO1: enable ? m_pmu.enableBLDO1() : m_pmu.disableBLDO1(); break;
        case PowerChannel::BLDO2: enable ? m_pmu.enableBLDO2() : m_pmu.disableBLDO2(); break;
        case PowerChannel::DLDO1: enable ? m_pmu.enableDLDO1() : m_pmu.disableDLDO1(); break;
        case PowerChannel::DLDO2: enable ? m_pmu.enableDLDO2() : m_pmu.disableDLDO2(); break;
        default:
            unlock();
            return CUBE32_INVALID_ARG;
    }
    
    unlock();
    return CUBE32_OK;
}

bool PMU::isChannelEnabled(PowerChannel channel) {
    if (!m_initialized) return false;
    
    lock();
    bool enabled = false;
    
    switch (channel) {
        case PowerChannel::DC1:   enabled = m_pmu.isEnableDC1(); break;
        case PowerChannel::DC2:   enabled = m_pmu.isEnableDC2(); break;
        case PowerChannel::DC3:   enabled = m_pmu.isEnableDC3(); break;
        case PowerChannel::DC4:   enabled = m_pmu.isEnableDC4(); break;
        case PowerChannel::DC5:   enabled = m_pmu.isEnableDC5(); break;
        case PowerChannel::ALDO1: enabled = m_pmu.isEnableALDO1(); break;
        case PowerChannel::ALDO2: enabled = m_pmu.isEnableALDO2(); break;
        case PowerChannel::ALDO3: enabled = m_pmu.isEnableALDO3(); break;
        case PowerChannel::ALDO4: enabled = m_pmu.isEnableALDO4(); break;
        case PowerChannel::BLDO1: enabled = m_pmu.isEnableBLDO1(); break;
        case PowerChannel::BLDO2: enabled = m_pmu.isEnableBLDO2(); break;
        case PowerChannel::DLDO1: enabled = m_pmu.isEnableDLDO1(); break;
        case PowerChannel::DLDO2: enabled = m_pmu.isEnableDLDO2(); break;
        default: break;
    }
    
    unlock();
    return enabled;
}

cube32_result_t PMU::shutdown() {
    if (!m_initialized) return CUBE32_NOT_INITIALIZED;
    lock();
    m_pmu.shutdown();
    unlock();
    return CUBE32_OK;
}

cube32_result_t PMU::reset() {
    if (!m_initialized) return CUBE32_NOT_INITIALIZED;
    lock();
    m_pmu.reset();
    unlock();
    return CUBE32_OK;
}

float PMU::getTemperature() {
    if (!m_initialized) return 0.0f;
    lock();
    float temp = m_pmu.getTemperature();
    unlock();
    return temp;
}

const char* PMU::chargeStatusToString(ChargeStatus status) {
    switch (status) {
        case ChargeStatus::TriState:        return "Tri-state";
        case ChargeStatus::PreCharge:       return "Pre-charge";
        case ChargeStatus::ConstantCurrent: return "Constant Current";
        case ChargeStatus::ConstantVoltage: return "Constant Voltage";
        case ChargeStatus::Done:            return "Done";
        case ChargeStatus::Stopped:         return "Stopped";
        default:                            return "Unknown";
    }
}

// ============================================================================
// Display Power Control
// ============================================================================

cube32_result_t PMU::setDisplayBacklight(bool enable) {
    if (!m_initialized) return CUBE32_NOT_INITIALIZED;
    
    lock();
    bool ok;
    if (enable) {
        ok = m_pmu.enableALDO3();
        ESP_LOGI(TAG, "Display backlight: ON (%u mV)", m_pmu.getALDO3Voltage());
    } else {
        ok = m_pmu.disableALDO3();
        ESP_LOGI(TAG, "Display backlight: OFF");
    }
    unlock();
    
    return ok ? CUBE32_OK : CUBE32_ERROR;
}

cube32_result_t PMU::setDisplayBacklightVoltage(uint16_t voltage_mv) {
    if (!m_initialized) return CUBE32_NOT_INITIALIZED;
    
    // ALDO3 voltage range: 500-3500mV
    if (voltage_mv < 500 || voltage_mv > 3500) {
        ESP_LOGW(TAG, "Display backlight voltage out of range: %u mV (valid: 500-3500)", voltage_mv);
        return CUBE32_INVALID_ARG;
    }
    
    lock();
    bool ok = m_pmu.setALDO3Voltage(voltage_mv);
    unlock();
    
    if (ok) {
        ESP_LOGI(TAG, "Display backlight voltage set to %u mV", voltage_mv);
    }
    
    return ok ? CUBE32_OK : CUBE32_ERROR;
}

bool PMU::isDisplayBacklightEnabled() {
    if (!m_initialized) return false;
    
    lock();
    bool enabled = m_pmu.isEnableALDO3();
    unlock();
    
    return enabled;
}

// ============================================================================
// USB OTG Power Control
// ============================================================================

cube32_result_t PMU::setUsbOtgPower(bool enable) {
    if (!m_initialized) return CUBE32_NOT_INITIALIZED;
    
    lock();
    bool ok;
    if (enable) {
        ok = m_pmu.enableBLDO2();
        ESP_LOGI(TAG, "USB OTG power: ON (%u mV)", m_pmu.getBLDO2Voltage());
    } else {
        ok = m_pmu.disableBLDO2();
        ESP_LOGI(TAG, "USB OTG power: OFF");
    }
    unlock();
    
    return ok ? CUBE32_OK : CUBE32_ERROR;
}

cube32_result_t PMU::setUsbOtgVoltage(uint16_t voltage_mv) {
    if (!m_initialized) return CUBE32_NOT_INITIALIZED;
    
    // BLDO2 voltage range: 500-3500mV
    if (voltage_mv < 500 || voltage_mv > 3500) {
        ESP_LOGW(TAG, "USB OTG voltage out of range: %u mV (valid: 500-3500)", voltage_mv);
        return CUBE32_INVALID_ARG;
    }
    
    lock();
    bool ok = m_pmu.setBLDO2Voltage(voltage_mv);
    unlock();
    
    if (ok) {
        ESP_LOGI(TAG, "USB OTG voltage set to %u mV", voltage_mv);
    }
    
    return ok ? CUBE32_OK : CUBE32_ERROR;
}

bool PMU::isUsbOtgPowerEnabled() {
    if (!m_initialized) return false;
    
    lock();
    bool enabled = m_pmu.isEnableBLDO2();
    unlock();
    
    return enabled;
}

uint16_t PMU::getUsbOtgVoltage() {
    if (!m_initialized) return 0;
    
    lock();
    uint16_t voltage = m_pmu.getBLDO2Voltage();
    unlock();
    
    return voltage;
}

} // namespace cube32
