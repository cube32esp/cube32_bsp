/**
 * @file a7670_modem.cpp
 * @brief CUBE32 A7670 Modem Driver Implementation
 * 
 * This implementation provides a C++ class interface for the SimCom A7670
 * LTE modem using the ESP-Modem component.
 */

#include "drivers/modem/a7670_modem.h"
#include "cube32_config.h"

#include <esp_log.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <esp_netif_ppp.h>
#include <cstring>

// ESP-Modem includes
#include "cxx_include/esp_modem_api.hpp"
#include "cxx_include/esp_modem_dce.hpp"
#include "esp_modem_config.h"

#ifdef CONFIG_CUBE32_MODEM_CONNECTION_USB
#include "esp_modem_usb_config.h"
#include "cxx_include/esp_modem_usb_api.hpp"
#endif

#ifdef CONFIG_CUBE32_MODEM_CONNECTION_UART
#include "driver/gpio.h"
#endif

static const char* TAG = "cube32_modem";

namespace cube32 {

// ============================================================================
// A7670Modem Class Implementation
// ============================================================================

A7670Modem& A7670Modem::instance() {
    static A7670Modem s_instance;
    return s_instance;
}

A7670Modem::~A7670Modem() {
    if (m_state != ModemState::NOT_INITIALIZED) {
        end();
    }
}

void A7670Modem::lock() {
    if (m_mutex) {
        xSemaphoreTake(m_mutex, portMAX_DELAY);
    }
}

void A7670Modem::unlock() {
    if (m_mutex) {
        xSemaphoreGive(m_mutex);
    }
}

static const char* stateToString(ModemState s) {
    switch (s) {
        case ModemState::NOT_INITIALIZED: return "NOT_INITIALIZED";
        case ModemState::INITIALIZING:    return "INITIALIZING";
        case ModemState::INITIALIZED:     return "INITIALIZED";
        case ModemState::CONNECTED:       return "CONNECTED";
        case ModemState::COMMAND_MODE:    return "COMMAND_MODE";
        case ModemState::DATA_MODE:       return "DATA_MODE";
        case ModemState::ERROR:           return "ERROR";
        default:                          return "UNKNOWN";
    }
}

void A7670Modem::setState(ModemState state) {
    ModemState old_state = m_state.exchange(state);
    
    if (old_state != state) {
        ESP_LOGI(TAG, "Modem state: %s -> %s", 
                 stateToString(old_state), stateToString(state));
        
        if (m_state_callback) {
            m_state_callback(state);
        }
    }
}

const char* A7670Modem::getStateString() const {
    return stateToString(m_state.load());
}

void A7670Modem::pppEventHandler(void* arg, esp_event_base_t event_base,
                                  int32_t event_id, void* event_data) {
    A7670Modem* modem = static_cast<A7670Modem*>(arg);
    
    if (event_id == NETIF_PPP_ERRORUSER) {
        ESP_LOGI(TAG, "PPP user interrupt");
        modem->m_ppp_connected = false;
        if (modem->m_event_group) {
            xEventGroupSetBits(modem->m_event_group, MODEM_PPP_DISCONNECTED_BIT);
            xEventGroupClearBits(modem->m_event_group, MODEM_PPP_CONNECTED_BIT);
        }
    } else if (event_id == NETIF_PPP_PHASE_DEAD) {
        ESP_LOGI(TAG, "PPP phase dead");
        modem->m_ppp_connected = false;
    } else if (event_id == NETIF_PPP_ERRORCONNECT) {
        ESP_LOGW(TAG, "PPP connection lost");
        modem->m_ppp_connected = false;
        if (modem->m_event_group) {
            xEventGroupSetBits(modem->m_event_group, MODEM_PPP_DISCONNECTED_BIT);
            xEventGroupClearBits(modem->m_event_group, MODEM_PPP_CONNECTED_BIT);
        }
    }
}

cube32_result_t A7670Modem::begin(const ModemConfig& config) {
    if (m_state != ModemState::NOT_INITIALIZED) {
        ESP_LOGW(TAG, "Modem already initialized");
        return CUBE32_ALREADY_INITIALIZED;
    }

    ESP_LOGI(TAG, "Initializing A7670 Modem...");
    m_config = config;

    // Create mutex for thread safety
    m_mutex = xSemaphoreCreateMutex();
    if (m_mutex == nullptr) {
        ESP_LOGE(TAG, "Failed to create modem mutex");
        return CUBE32_NO_MEM;
    }

    // Create event group
    m_event_group = xEventGroupCreate();
    if (m_event_group == nullptr) {
        ESP_LOGE(TAG, "Failed to create event group");
        vSemaphoreDelete(m_mutex);
        m_mutex = nullptr;
        return CUBE32_NO_MEM;
    }

    // Initialize IO Expander for modem control
    cube32_result_t iox_ret = initIOExpander();
    if (iox_ret != CUBE32_OK) {
        ESP_LOGW(TAG, "IO Expander initialization failed, modem power control unavailable");
        // Continue anyway - modem may already be powered on
    }

    // Power on the modem if IO Expander is available
    if (m_iox_initialized) {
        ESP_LOGI(TAG, "Powering on modem...");
        cube32_result_t pwr_ret = powerOn();
        if (pwr_ret != CUBE32_OK) {
            ESP_LOGW(TAG, "Modem power-on sequence failed, continuing anyway");
        }
    }

    // Initialize based on connection type
    cube32_result_t ret;
    if (config.connectionType == ModemConnectionType::USB) {
        ret = initUSB();
    } else {
        ret = initUART();
    }

    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to initialize modem connection");
        vEventGroupDelete(m_event_group);
        m_event_group = nullptr;
        vSemaphoreDelete(m_mutex);
        m_mutex = nullptr;
        return ret;
    }

    setState(ModemState::INITIALIZED);
    ESP_LOGI(TAG, "A7670 Modem initialized successfully");
    return CUBE32_OK;
}

// ============================================================================
// Async Modem Initialization
// ============================================================================

void A7670Modem::modemInitTask(void* arg) {
    A7670Modem* modem = static_cast<A7670Modem*>(arg);
    
    ESP_LOGI(TAG, "Modem initialization task started...");
    
    bool success = false;
    
    // Cold boot stabilization delay
    // During cold boot, the system (I2C bus, IO expander, modem module) needs time to stabilize
    // This delay ensures reliable power-on especially when power is first applied
    ESP_LOGI(TAG, "[Async] Waiting for cold boot stabilization (%d ms)...", CUBE32_MODEM_COLD_BOOT_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(CUBE32_MODEM_COLD_BOOT_DELAY_MS));
    
    // Power on the modem if IO Expander is available
    if (modem->m_iox_initialized) {
        ESP_LOGI(TAG, "[Async] Powering on modem...");
        cube32_result_t pwr_ret = modem->powerOn();
        if (pwr_ret != CUBE32_OK) {
            ESP_LOGW(TAG, "[Async] Modem power-on sequence failed, continuing anyway");
        }
    }
    
    // Initialize based on connection type
    cube32_result_t ret;
    if (modem->m_config.connectionType == ModemConnectionType::USB) {
        ret = modem->initUSB();
    } else {
        ret = modem->initUART();
    }
    
    if (ret == CUBE32_OK) {
        modem->setState(ModemState::INITIALIZED);
        ESP_LOGI(TAG, "[Async] A7670 Modem initialized successfully");
        success = true;
        xEventGroupSetBits(modem->m_event_group, MODEM_INIT_COMPLETE_BIT);
        
        // Start network registration if auto_register is enabled
        if (modem->m_auto_register) {
            ESP_LOGI(TAG, "[Async] Starting network registration...");
            modem->startNetworkRegistration(
                modem->m_network_timeout_ms,
                modem->m_network_callback
            );
        }
    } else {
        ESP_LOGE(TAG, "[Async] Failed to initialize modem connection");
        modem->setState(ModemState::ERROR);
        xEventGroupSetBits(modem->m_event_group, MODEM_INIT_FAILED_BIT);
    }
    
    // Call the callback if set
    if (modem->m_init_callback) {
        modem->m_init_callback(success, modem->m_state);
    }
    
    // Clear task handle before exiting
    modem->m_init_task = nullptr;
    vTaskDelete(nullptr);
}

cube32_result_t A7670Modem::beginAsync(const ModemConfig& config, bool auto_register, 
                                        ModemInitCallback callback) {
    if (m_state != ModemState::NOT_INITIALIZED) {
        ESP_LOGW(TAG, "Modem already initialized or initializing");
        return CUBE32_ALREADY_INITIALIZED;
    }
    
    ESP_LOGI(TAG, "Starting async A7670 Modem initialization...");
    m_config = config;
    m_auto_register = auto_register;
    m_init_callback = callback;
    
    // Create mutex for thread safety
    m_mutex = xSemaphoreCreateMutex();
    if (m_mutex == nullptr) {
        ESP_LOGE(TAG, "Failed to create modem mutex");
        return CUBE32_NO_MEM;
    }
    
    // Create event group
    m_event_group = xEventGroupCreate();
    if (m_event_group == nullptr) {
        ESP_LOGE(TAG, "Failed to create event group");
        vSemaphoreDelete(m_mutex);
        m_mutex = nullptr;
        return CUBE32_NO_MEM;
    }
    
    // Initialize IO Expander first (this is quick and needed for power control)
    cube32_result_t iox_ret = initIOExpander();
    if (iox_ret != CUBE32_OK) {
        ESP_LOGW(TAG, "IO Expander initialization failed, modem power control unavailable");
    }
    
    // Set state to INITIALIZING
    setState(ModemState::INITIALIZING);
    
    // Get network timeout from Kconfig
#ifdef CONFIG_CUBE32_MODEM_NETWORK_TIMEOUT_MS
    m_network_timeout_ms = CONFIG_CUBE32_MODEM_NETWORK_TIMEOUT_MS;
#else
    m_network_timeout_ms = CUBE32_MODEM_CONNECT_TIMEOUT_MS;
#endif
    
    // Create the background task for initialization
    BaseType_t ret = xTaskCreate(
        modemInitTask,
        "modem_init",
        6144,  // Larger stack for USB initialization
        this,
        5,  // Priority
        &m_init_task
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create modem init task");
        setState(ModemState::NOT_INITIALIZED);
        vEventGroupDelete(m_event_group);
        m_event_group = nullptr;
        vSemaphoreDelete(m_mutex);
        m_mutex = nullptr;
        return CUBE32_NO_MEM;
    }
    
    ESP_LOGI(TAG, "Modem async initialization started");
    return CUBE32_OK;
}

// ============================================================================
// IO Expander Control Implementation
// ============================================================================

cube32_result_t A7670Modem::initIOExpander() {
    ESP_LOGI(TAG, "Initializing modem IO Expander (addr: 0x%02X)...", m_config.iox_i2c_addr);
    
    cube32_result_t ret = m_io_expander.begin(m_config.iox_i2c_addr);
    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to initialize IO Expander");
        return ret;
    }

    // Configure all pins as outputs
    // Pins: 0=PWRKEY, 1=DTR, 2=PWR_RAIL, 3=COMM_SEL
    uint32_t output_pins = (1 << m_config.iox_pwrkey_pin) |
                           (1 << m_config.iox_dtr_pin) |
                           (1 << m_config.iox_pwr_rail_pin) |
                           (1 << m_config.iox_comm_sel_pin);
    
    ret = m_io_expander.setDirectionMask(output_pins, TCA9554Direction::DIR_OUTPUT);
    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to configure IO Expander pins as outputs");
        return ret;
    }

    // Set initial state (considering inverting circuit on PWRKEY):
    // - PWRKEY: LOW on IOX = HIGH on A7670 (idle state)
    // - DTR: HIGH (idle state)
    // - PWR_RAIL: LOW (off initially)
    // - COMM_SEL: LOW (USB mode) or HIGH (UART mode)
    m_iox_output_state = (1 << m_config.iox_dtr_pin);  // Only DTR high initially
    // PWRKEY stays LOW (idle due to inverting circuit)
    
    // Set comm port based on connection type
    if (m_config.connectionType == ModemConnectionType::UART) {
        m_iox_output_state |= (1 << m_config.iox_comm_sel_pin);  // UART mode
    }
    
    // Apply initial output states
    ret = m_io_expander.setLevelMask(output_pins, false);  // Clear all first
    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to clear IO Expander outputs");
        return ret;
    }
    
    // Set the pins that should be high
    for (int i = 0; i < 8; i++) {
        if (m_iox_output_state & (1 << i)) {
            ret = m_io_expander.setLevel(static_cast<TCA9554Pin>(1 << i), true);
            if (ret != CUBE32_OK) {
                ESP_LOGW(TAG, "Failed to set pin %d high", i);
            }
        }
    }
    
    m_iox_initialized = true;
    ESP_LOGI(TAG, "IO Expander initialized (state: 0x%02X)", m_iox_output_state);
    return CUBE32_OK;
}

cube32_result_t A7670Modem::powerOn() {
    if (!m_iox_initialized) {
        ESP_LOGW(TAG, "IO Expander not initialized");
        return CUBE32_NOT_INITIALIZED;
    }

    ESP_LOGI(TAG, "Modem power-on sequence starting...");

    // 1. Enable power rail
    cube32_result_t ret = setPowerRail(true);
    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to enable power rail");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(CUBE32_MODEM_PWR_RAIL_SETTLE_MS));  // Let power stabilize

    // 2. Set DTR high (idle)
    ret = setDTR(true);
    if (ret != CUBE32_OK) {
        ESP_LOGW(TAG, "Failed to set DTR high");
    }

    // 3. Ensure PWRKEY IOX is LOW initially (A7670 sees HIGH = idle)
    ret = setPwrKey(false);  // IOX LOW = A7670 PWRKEY HIGH (idle)
    if (ret != CUBE32_OK) {
        ESP_LOGW(TAG, "Failed to set PWRKEY to idle");
    }
    vTaskDelay(pdMS_TO_TICKS(CUBE32_MODEM_PWRKEY_SETTLE_MS));

    // 4. Pulse PWRKEY to power on modem
    ret = pwrKeyOn();
    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to pulse PWRKEY for power on");
        return ret;
    }

    // 5. Wait for modem to be ready
    ESP_LOGI(TAG, "Waiting for modem to power up...");
    vTaskDelay(pdMS_TO_TICKS(CUBE32_MODEM_POWERON_WAIT_MS));  // Wait for modem to be ready

    ESP_LOGI(TAG, "Modem power-on sequence complete");
    return CUBE32_OK;
}

cube32_result_t A7670Modem::powerOff() {
    if (!m_iox_initialized) {
        ESP_LOGW(TAG, "IO Expander not initialized");
        return CUBE32_NOT_INITIALIZED;
    }

    ESP_LOGI(TAG, "Modem power-off sequence starting...");

    // 1. Pulse PWRKEY to turn off modem
    cube32_result_t ret = pwrKeyOff();
    if (ret != CUBE32_OK) {
        ESP_LOGW(TAG, "Failed to pulse PWRKEY for power off");
    }
    vTaskDelay(pdMS_TO_TICKS(CUBE32_MODEM_POWERON_WAIT_MS / 3));  // Wait for modem to shut down

    // 2. Disable power rail
    ret = setPowerRail(false);
    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to disable power rail");
        return ret;
    }

    ESP_LOGI(TAG, "Modem power-off sequence complete");
    return CUBE32_OK;
}

cube32_result_t A7670Modem::setPwrKey(bool high) {
    if (!m_iox_initialized) {
        return CUBE32_NOT_INITIALIZED;
    }

    // Direct control of IOX PWRKEY pin
    // Note: Hardware has inverting circuit, so:
    //   IOX HIGH = A7670 PWRKEY LOW (active)
    //   IOX LOW = A7670 PWRKEY HIGH (idle)
    TCA9554Pin pin = static_cast<TCA9554Pin>(1 << m_config.iox_pwrkey_pin);
    cube32_result_t ret = m_io_expander.setLevel(pin, high);
    
    if (ret == CUBE32_OK) {
        if (high) {
            m_iox_output_state |= (1 << m_config.iox_pwrkey_pin);
        } else {
            m_iox_output_state &= ~(1 << m_config.iox_pwrkey_pin);
        }
        ESP_LOGD(TAG, "PWRKEY IOX pin set to %s", high ? "HIGH" : "LOW");
    }
    
    return ret;
}

cube32_result_t A7670Modem::pwrKeyOn() {
    ESP_LOGI(TAG, "Pulsing PWRKEY to turn on modem (IOX: LOW-HIGH-LOW)...");
    
    // Power on sequence with inverting circuit:
    // IOX: LOW (50ms) -> HIGH (50ms pulse) -> LOW
    // A7670 sees: HIGH -> LOW (50ms pulse) -> HIGH
    
    // 1. Start with IOX LOW (A7670 PWRKEY idle HIGH)
    cube32_result_t ret = setPwrKey(false);  // IOX LOW
    if (ret != CUBE32_OK) {
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(CUBE32_MODEM_PWRKEY_SETTLE_MS));  // Settle time
    
    // 2. Set IOX HIGH (A7670 PWRKEY active LOW) - this is the pulse
    ret = setPwrKey(true);  // IOX HIGH
    if (ret != CUBE32_OK) {
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(CUBE32_MODEM_PWRKEY_ON_TIME_MS));  // PWRKEY_ON_TIME
    
    // 3. Return IOX LOW (A7670 PWRKEY idle HIGH)
    ret = setPwrKey(false);  // IOX LOW
    
    return ret;
}

cube32_result_t A7670Modem::pwrKeyOff() {
    ESP_LOGI(TAG, "Pulsing PWRKEY to turn off modem (IOX: LOW-HIGH-LOW)...");
    
    // Power off sequence with inverting circuit:
    // IOX: LOW (settle) -> HIGH (PWRKEY_OFF_TIME pulse) -> LOW
    // A7670 sees: HIGH -> LOW (PWRKEY_OFF_TIME pulse) -> HIGH
    
    // 1. Start with IOX LOW (A7670 PWRKEY idle HIGH)
    cube32_result_t ret = setPwrKey(false);  // IOX LOW
    if (ret != CUBE32_OK) {
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(CUBE32_MODEM_PWRKEY_SETTLE_MS));  // Settle time
    
    // 2. Set IOX HIGH (A7670 PWRKEY active LOW) - this is the pulse
    ret = setPwrKey(true);  // IOX HIGH
    if (ret != CUBE32_OK) {
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(CUBE32_MODEM_PWRKEY_OFF_TIME_MS));  // PWRKEY_OFF_TIME
    
    // 3. Return IOX LOW (A7670 PWRKEY idle HIGH)
    ret = setPwrKey(false);  // IOX LOW
    
    return ret;
}

cube32_result_t A7670Modem::setDTR(bool high) {
    if (!m_iox_initialized) {
        return CUBE32_NOT_INITIALIZED;
    }

    TCA9554Pin pin = static_cast<TCA9554Pin>(1 << m_config.iox_dtr_pin);
    cube32_result_t ret = m_io_expander.setLevel(pin, high);
    
    if (ret == CUBE32_OK) {
        if (high) {
            m_iox_output_state |= (1 << m_config.iox_dtr_pin);
        } else {
            m_iox_output_state &= ~(1 << m_config.iox_dtr_pin);
        }
        ESP_LOGD(TAG, "DTR set to %s", high ? "HIGH" : "LOW");
    }
    
    return ret;
}

cube32_result_t A7670Modem::setPowerRail(bool on) {
    if (!m_iox_initialized) {
        return CUBE32_NOT_INITIALIZED;
    }

    TCA9554Pin pin = static_cast<TCA9554Pin>(1 << m_config.iox_pwr_rail_pin);
    cube32_result_t ret = m_io_expander.setLevel(pin, on);
    
    if (ret == CUBE32_OK) {
        if (on) {
            m_iox_output_state |= (1 << m_config.iox_pwr_rail_pin);
        } else {
            m_iox_output_state &= ~(1 << m_config.iox_pwr_rail_pin);
        }
        ESP_LOGI(TAG, "Power rail %s", on ? "ENABLED" : "DISABLED");
    }
    
    return ret;
}

cube32_result_t A7670Modem::setCommPort(bool uart) {
    if (!m_iox_initialized) {
        return CUBE32_NOT_INITIALIZED;
    }

    TCA9554Pin pin = static_cast<TCA9554Pin>(1 << m_config.iox_comm_sel_pin);
    cube32_result_t ret = m_io_expander.setLevel(pin, uart);
    
    if (ret == CUBE32_OK) {
        if (uart) {
            m_iox_output_state |= (1 << m_config.iox_comm_sel_pin);
        } else {
            m_iox_output_state &= ~(1 << m_config.iox_comm_sel_pin);
        }
        ESP_LOGI(TAG, "Comm port set to %s", uart ? "UART" : "USB");
    }
    
    return ret;
}

cube32_result_t A7670Modem::getIOStates(uint8_t& states) {
    if (!m_iox_initialized) {
        return CUBE32_NOT_INITIALIZED;
    }

    uint32_t levels;
    cube32_result_t ret = m_io_expander.getAllLevels(levels);
    if (ret == CUBE32_OK) {
        states = static_cast<uint8_t>(levels & 0xFF);
    }
    
    return ret;
}

bool A7670Modem::isPowerRailOn() const {
    return (m_iox_output_state & (1 << m_config.iox_pwr_rail_pin)) != 0;
}

bool A7670Modem::isCommPortUART() const {
    return (m_iox_output_state & (1 << m_config.iox_comm_sel_pin)) != 0;
}

#ifdef CONFIG_CUBE32_MODEM_CONNECTION_USB
cube32_result_t A7670Modem::initUSB() {
    ESP_LOGI(TAG, "Initializing USB connection (VID: 0x%04X, PID: 0x%04X)",
             m_config.usb_vid, m_config.usb_pid);

    // Configure USB terminal using the same pattern as modem_console example
    struct esp_modem_usb_term_config usb_config = ESP_MODEM_A7670_USB_CONFIG();
    
    // Override with custom settings if provided
    if (m_config.usb_vid != 0) {
        usb_config.vid = m_config.usb_vid;
    }
    if (m_config.usb_pid != 0) {
        usb_config.pid = m_config.usb_pid;
    }
    if (m_config.usb_at_interface >= 0) {
        usb_config.interface_idx = m_config.usb_at_interface;
    }
    if (m_config.usb_data_interface >= 0) {
        usb_config.secondary_interface_idx = m_config.usb_data_interface;
    }
    if (m_config.usb_timeout_ms > 0) {
        usb_config.timeout_ms = m_config.usb_timeout_ms;
    }

    // Configure DTE — use larger buffer to handle PPP frames (MTU 1500 + framing)
    esp_modem_dte_config_t dte_config = ESP_MODEM_DTE_DEFAULT_USB_CONFIG(usb_config);
    dte_config.dte_buffer_size = CUBE32_MODEM_DTE_BUFFER_SIZE;

    // Configure DCE
    esp_modem_dce_config_t dce_config = ESP_MODEM_DCE_DEFAULT_CONFIG(m_config.apn.c_str());

    ESP_LOGI(TAG, "Waiting for USB modem to connect...");

    // Create USB DTE
    m_dte = esp_modem::create_usb_dte(&dte_config);
    if (!m_dte) {
        ESP_LOGE(TAG, "Failed to create USB DTE");
        return CUBE32_ERROR;
    }

    // Set error callback for USB terminal errors
    m_dte->set_error_cb([this](esp_modem::terminal_error err) {
        // Guard against callbacks during shutdown
        if (m_shutting_down) {
            ESP_LOGD(TAG, "DTE error %d ignored (shutting down)", (int)err);
            return;
        }

        if (err == esp_modem::terminal_error::DEVICE_GONE) {
            ESP_LOGE(TAG, "USB device disconnected");
            setState(ModemState::ERROR);
            m_ppp_connected = false;
            if (m_event_group) {
                xEventGroupSetBits(m_event_group, MODEM_PPP_DISCONNECTED_BIT);
                xEventGroupClearBits(m_event_group, MODEM_PPP_CONNECTED_BIT);
            }
            return;
        }

        // Handle transient USB errors (UNEXPECTED_CONTROL_FLOW, BUFFER_OVERFLOW, CHECKSUM_ERROR)
        const char* err_name = "UNKNOWN";
        switch (err) {
            case esp_modem::terminal_error::UNEXPECTED_CONTROL_FLOW: err_name = "UNEXPECTED_CONTROL_FLOW"; break;
            case esp_modem::terminal_error::BUFFER_OVERFLOW:         err_name = "BUFFER_OVERFLOW"; break;
            case esp_modem::terminal_error::CHECKSUM_ERROR:          err_name = "CHECKSUM_ERROR"; break;
            default: break;
        }
        ESP_LOGW(TAG, "DTE transient error: %s (%d)", err_name, (int)err);

        // Track errors within a time window
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (m_usb_error_count == 0 || (now - m_first_usb_error_time) > CUBE32_MODEM_USB_ERROR_WINDOW_MS) {
            // Start a new error window
            m_first_usb_error_time = now;
            m_usb_error_count = 1;
        } else {
            m_usb_error_count++;
        }

        ESP_LOGW(TAG, "USB error count: %lu in window", (unsigned long)m_usb_error_count.load());

        // Trigger recovery if threshold exceeded
        if (m_usb_error_count >= CUBE32_MODEM_USB_ERROR_THRESHOLD) {
            ESP_LOGE(TAG, "USB error threshold reached — requesting recovery");
            m_usb_error_count = 0;
            requestRecovery();
        }
    });

    // Create PPP netif AFTER DTE is created (following modem_console pattern)
    esp_netif_config_t netif_ppp_config = ESP_NETIF_DEFAULT_PPP();
    m_netif = esp_netif_new(&netif_ppp_config);
    if (m_netif == nullptr) {
        ESP_LOGE(TAG, "Failed to create PPP netif");
        m_dte.reset();
        return CUBE32_NO_MEM;
    }

    // Register PPP event handler
    ESP_ERROR_CHECK(esp_event_handler_register(NETIF_PPP_STATUS, ESP_EVENT_ANY_ID, 
                                                &pppEventHandler, this));

    // Create DCE for SIM7600 compatible modem (A7670 uses similar AT commands)
    m_dce = esp_modem::create_SIM7600_dce(&dce_config, m_dte, m_netif);
    if (!m_dce) {
        ESP_LOGE(TAG, "Failed to create DCE");
        esp_event_handler_unregister(NETIF_PPP_STATUS, ESP_EVENT_ANY_ID, &pppEventHandler);
        esp_netif_destroy(m_netif);
        m_netif = nullptr;
        m_dte.reset();
        return CUBE32_ERROR;
    }

    ESP_LOGI(TAG, "USB modem connected successfully");
    setState(ModemState::CONNECTED);
    
    // Give modem time to initialize after USB enumeration
    // Following modem_console pattern - don't sync immediately, just set to command mode
    // The modem needs time after USB connection before AT commands work reliably
    ESP_LOGI(TAG, "Waiting for modem to be ready...");
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Try to sync with retries
    bool synced = false;
    for (int retry = 0; retry < 5; retry++) {
        ESP_LOGI(TAG, "Sync attempt %d/5...", retry + 1);
        if (sync() == CUBE32_OK) {
            synced = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    if (synced) {
        setState(ModemState::COMMAND_MODE);
    } else {
        // Even if sync fails, set to command mode to allow manual AT commands
        ESP_LOGW(TAG, "Sync failed after retries, but modem may still work");
        setState(ModemState::COMMAND_MODE);
    }

    return CUBE32_OK;
}
#else
cube32_result_t A7670Modem::initUSB() {
    ESP_LOGE(TAG, "USB modem support not enabled");
    return CUBE32_NOT_SUPPORTED;
}
#endif

#ifdef CONFIG_CUBE32_MODEM_CONNECTION_UART
cube32_result_t A7670Modem::initUART() {
    ESP_LOGI(TAG, "Initializing UART connection (Port: %d, TX: %d, RX: %d, Baud: %d)",
             m_config.uart_port, m_config.uart_tx_pin, m_config.uart_rx_pin, m_config.uart_baud_rate);

    // GPIO 19/20 are shared with ESP32-S3 USB D-/D+ pins.
    // Reset them to detach from internal USB PHY before UART driver claims them.
    ESP_LOGI(TAG, "Reconfiguring GPIO %d/%d from USB to GPIO for UART",
             m_config.uart_tx_pin, m_config.uart_rx_pin);
    gpio_reset_pin(static_cast<gpio_num_t>(m_config.uart_tx_pin));
    gpio_reset_pin(static_cast<gpio_num_t>(m_config.uart_rx_pin));

    // Configure DTE with UART defaults, then override with our settings
    esp_modem_dte_config_t dte_config = ESP_MODEM_DTE_DEFAULT_CONFIG();
    dte_config.uart_config.port_num = static_cast<uart_port_t>(m_config.uart_port);
    dte_config.uart_config.tx_io_num = m_config.uart_tx_pin;
    dte_config.uart_config.rx_io_num = m_config.uart_rx_pin;
    dte_config.uart_config.baud_rate = m_config.uart_baud_rate;
    dte_config.uart_config.flow_control = ESP_MODEM_FLOW_CONTROL_NONE;
    dte_config.uart_config.rts_io_num = -1;
    dte_config.uart_config.cts_io_num = -1;
    dte_config.dte_buffer_size = CUBE32_MODEM_DTE_BUFFER_SIZE;

    // Configure DCE
    esp_modem_dce_config_t dce_config = ESP_MODEM_DCE_DEFAULT_CONFIG(m_config.apn.c_str());

    // Create UART DTE
    m_dte = esp_modem::create_uart_dte(&dte_config);
    if (!m_dte) {
        ESP_LOGE(TAG, "Failed to create UART DTE");
        return CUBE32_ERROR;
    }

    // Create PPP netif
    esp_netif_config_t netif_ppp_config = ESP_NETIF_DEFAULT_PPP();
    m_netif = esp_netif_new(&netif_ppp_config);
    if (m_netif == nullptr) {
        ESP_LOGE(TAG, "Failed to create PPP netif");
        m_dte.reset();
        return CUBE32_NO_MEM;
    }

    // Register PPP event handler
    ESP_ERROR_CHECK(esp_event_handler_register(NETIF_PPP_STATUS, ESP_EVENT_ANY_ID,
                                                &pppEventHandler, this));

    // Create DCE for SIM7600 compatible modem (A7670 uses similar AT commands)
    m_dce = esp_modem::create_SIM7600_dce(&dce_config, m_dte, m_netif);
    if (!m_dce) {
        ESP_LOGE(TAG, "Failed to create DCE");
        esp_event_handler_unregister(NETIF_PPP_STATUS, ESP_EVENT_ANY_ID, &pppEventHandler);
        esp_netif_destroy(m_netif);
        m_netif = nullptr;
        m_dte.reset();
        return CUBE32_ERROR;
    }

    ESP_LOGI(TAG, "UART modem connected successfully");
    setState(ModemState::CONNECTED);

    // Try to sync with retries
    ESP_LOGI(TAG, "Syncing with modem...");
    bool synced = false;
    for (int retry = 0; retry < 5; retry++) {
        ESP_LOGI(TAG, "Sync attempt %d/5...", retry + 1);
        if (sync() == CUBE32_OK) {
            synced = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    if (synced) {
        setState(ModemState::COMMAND_MODE);
    } else {
        ESP_LOGW(TAG, "Sync failed after retries, but modem may still work");
        setState(ModemState::COMMAND_MODE);
    }

    return CUBE32_OK;
}
#else
cube32_result_t A7670Modem::initUART() {
    ESP_LOGE(TAG, "UART modem support not enabled");
    return CUBE32_NOT_SUPPORTED;
}
#endif

// ============================================================================
// USB Error Recovery
// ============================================================================

void A7670Modem::requestRecovery() {
    // Prevent re-entrant recovery requests
    bool expected = false;
    if (!m_recovery_pending.compare_exchange_strong(expected, true)) {
        ESP_LOGD(TAG, "Recovery already pending, skipping");
        return;
    }

    // Spawn a one-shot recovery task (error callback runs in USB host context and must not block)
    if (m_recovery_task == nullptr) {
        BaseType_t ret = xTaskCreate(recoveryTask, "modem_recover", 4096, this, 5, &m_recovery_task);
        if (ret != pdPASS) {
            ESP_LOGE(TAG, "Failed to create recovery task");
            m_recovery_pending = false;
        }
    } else {
        // Task already exists — notify it
        xTaskNotifyGive(m_recovery_task);
    }
}

void A7670Modem::recoveryTask(void* arg) {
    A7670Modem* modem = static_cast<A7670Modem*>(arg);

    ESP_LOGW(TAG, "Recovery task started");

    // Brief delay to let any in-flight USB transfers settle
    vTaskDelay(pdMS_TO_TICKS(500));

    if (modem->m_shutting_down) {
        ESP_LOGI(TAG, "Recovery aborted — modem shutting down");
        goto done;
    }

    // Stage 1: Try to toggle mode (data → command → data)
    if (modem->m_state.load() == ModemState::DATA_MODE) {
        ESP_LOGI(TAG, "Recovery stage 1: toggling mode");
        if (modem->setCommandMode() == CUBE32_OK) {
            vTaskDelay(pdMS_TO_TICKS(500));
            if (!modem->m_shutting_down && modem->setDataMode() == CUBE32_OK) {
                ESP_LOGI(TAG, "Recovery stage 1 succeeded");
                goto done;
            }
        }
    }

    if (modem->m_shutting_down) goto done;

    // Stage 2: Reset modem to command mode as a fallback
    ESP_LOGW(TAG, "Recovery stage 2: resetting to command mode");
    if (modem->m_dce) {
        modem->lock();
        modem->m_dce->set_mode(esp_modem::modem_mode::COMMAND_MODE);
        modem->unlock();
    }
    modem->setState(ModemState::COMMAND_MODE);
    modem->m_ppp_connected = false;
    if (modem->m_event_group) {
        xEventGroupSetBits(modem->m_event_group, MODEM_PPP_DISCONNECTED_BIT);
        xEventGroupClearBits(modem->m_event_group, MODEM_PPP_CONNECTED_BIT);
    }
    ESP_LOGW(TAG, "Recovery complete — modem in command mode. Re-enter 'data' to reconnect.");

done:
    modem->m_recovery_pending = false;
    modem->m_recovery_task = nullptr;
    vTaskDelete(nullptr);
}

// ============================================================================
// Lifecycle
// ============================================================================

cube32_result_t A7670Modem::end() {
    if (m_state == ModemState::NOT_INITIALIZED) {
        return CUBE32_NOT_INITIALIZED;
    }

    // Signal shutdown to prevent error callbacks from triggering recovery
    m_shutting_down = true;

    // Wait for any pending recovery to finish
    if (m_recovery_task != nullptr) {
        // Give recovery task time to notice m_shutting_down and exit
        for (int i = 0; i < 20 && m_recovery_task != nullptr; i++) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        if (m_recovery_task != nullptr) {
            vTaskDelete(m_recovery_task);
            m_recovery_task = nullptr;
        }
    }

    // Stop network registration task if running
    if (m_network_task != nullptr) {
        vTaskDelete(m_network_task);
        m_network_task = nullptr;
    }

    lock();

    // Switch to command mode if in data mode
    if (m_state == ModemState::DATA_MODE) {
        setCommandMode();
    }

    // Clean up DCE and DTE
    m_dce.reset();
    m_dte.reset();

    // Clean up netif
    if (m_netif) {
        esp_event_handler_unregister(NETIF_PPP_STATUS, ESP_EVENT_ANY_ID, &pppEventHandler);
        esp_netif_destroy(m_netif);
        m_netif = nullptr;
    }

    // Power off modem if IO Expander is available
    if (m_iox_initialized) {
        powerOff();
        m_io_expander.end();
        m_iox_initialized = false;
    }

    setState(ModemState::NOT_INITIALIZED);
    m_network_ready = false;
    m_usb_error_count = 0;
    m_recovery_pending = false;
    
    unlock();

    // Clean up synchronization objects
    if (m_event_group) {
        vEventGroupDelete(m_event_group);
        m_event_group = nullptr;
    }
    if (m_mutex) {
        vSemaphoreDelete(m_mutex);
        m_mutex = nullptr;
    }

    m_shutting_down = false;  // Reset for potential re-initialization

    ESP_LOGI(TAG, "Modem deinitialized");
    return CUBE32_OK;
}

cube32_result_t A7670Modem::sync() {
    if (!m_dce) {
        return CUBE32_NOT_INITIALIZED;
    }

    lock();
    esp_modem::command_result result = m_dce->sync();
    unlock();

    if (result == esp_modem::command_result::OK) {
        ESP_LOGI(TAG, "Modem sync successful");
        return CUBE32_OK;
    }
    
    ESP_LOGW(TAG, "Modem sync failed");
    return CUBE32_ERROR;
}

cube32_result_t A7670Modem::reset() {
    if (!m_dce) {
        return CUBE32_NOT_INITIALIZED;
    }

    lock();
    esp_modem::command_result result = m_dce->reset();
    unlock();

    if (result == esp_modem::command_result::OK) {
        ESP_LOGI(TAG, "Modem reset successful");
        return CUBE32_OK;
    }
    
    ESP_LOGE(TAG, "Modem reset failed");
    return CUBE32_ERROR;
}

cube32_result_t A7670Modem::waitForNetwork(uint32_t timeout_ms) {
    if (!m_dce) {
        return CUBE32_NOT_INITIALIZED;
    }

    ESP_LOGI(TAG, "Waiting for network registration (timeout: %lu ms)...", timeout_ms);

    uint32_t start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    NetworkStatus status;

    while ((xTaskGetTickCount() * portTICK_PERIOD_MS - start_time) < timeout_ms) {
        status = getNetworkStatus();
        
        if (status == NetworkStatus::REGISTERED_HOME || 
            status == NetworkStatus::REGISTERED_ROAMING) {
            ESP_LOGI(TAG, "Network registered: %s", 
                     status == NetworkStatus::REGISTERED_HOME ? "Home" : "Roaming");
            m_network_ready = true;
            xEventGroupSetBits(m_event_group, MODEM_CONNECTED_BIT | MODEM_NETWORK_REGISTERED_BIT);
            return CUBE32_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGW(TAG, "Network registration timeout");
    xEventGroupSetBits(m_event_group, MODEM_NETWORK_REGISTRATION_FAILED_BIT);
    return CUBE32_TIMEOUT;
}

void A7670Modem::networkRegistrationTask(void* arg) {
    A7670Modem* modem = static_cast<A7670Modem*>(arg);
    
    ESP_LOGI(TAG, "Network registration task started (timeout: %lu ms)...", modem->m_network_timeout_ms);
    
    uint32_t start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    NetworkStatus status = NetworkStatus::UNKNOWN;
    bool success = false;

    while ((xTaskGetTickCount() * portTICK_PERIOD_MS - start_time) < modem->m_network_timeout_ms) {
        status = modem->getNetworkStatus();
        
        if (status == NetworkStatus::REGISTERED_HOME || 
            status == NetworkStatus::REGISTERED_ROAMING) {
            ESP_LOGI(TAG, "Network registered: %s", 
                     status == NetworkStatus::REGISTERED_HOME ? "Home" : "Roaming");
            modem->m_network_ready = true;
            success = true;
            xEventGroupSetBits(modem->m_event_group, MODEM_CONNECTED_BIT | MODEM_NETWORK_REGISTERED_BIT);
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    if (!success) {
        ESP_LOGW(TAG, "Network registration timeout");
        xEventGroupSetBits(modem->m_event_group, MODEM_NETWORK_REGISTRATION_FAILED_BIT);
    }

    // Call the callback if set
    if (modem->m_network_callback) {
        modem->m_network_callback(success, status);
    }

    // Print status if successful
    if (success) {
        modem->printStatus();
    }

    // Clear task handle before exiting
    modem->m_network_task = nullptr;
    vTaskDelete(nullptr);
}

cube32_result_t A7670Modem::startNetworkRegistration(uint32_t timeout_ms, NetworkReadyCallback callback) {
    if (!m_dce) {
        return CUBE32_NOT_INITIALIZED;
    }

    // Check if task is already running
    if (m_network_task != nullptr) {
        ESP_LOGW(TAG, "Network registration task already running");
        return CUBE32_BUSY;
    }

    // Clear any previous registration status
    m_network_ready = false;
    xEventGroupClearBits(m_event_group, MODEM_NETWORK_REGISTERED_BIT | MODEM_NETWORK_REGISTRATION_FAILED_BIT);

    // Store configuration
    m_network_timeout_ms = timeout_ms;
    if (callback) {
        m_network_callback = callback;
    }

    // Create the background task
    BaseType_t ret = xTaskCreate(
        networkRegistrationTask,
        "modem_net_reg",
        4096,
        this,
        5,  // Priority
        &m_network_task
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create network registration task");
        return CUBE32_NO_MEM;
    }

    ESP_LOGI(TAG, "Network registration started asynchronously (timeout: %lu ms)", timeout_ms);
    return CUBE32_OK;
}

NetworkStatus A7670Modem::getNetworkStatus() {
    if (!m_dce) {
        return NetworkStatus::UNKNOWN;
    }

    lock();
    int stat = 0;
    esp_modem::command_result result = m_dce->get_network_registration_state(stat);
    unlock();

    if (result != esp_modem::command_result::OK) {
        return NetworkStatus::UNKNOWN;
    }

    switch (stat) {
        case 0: return NetworkStatus::NOT_REGISTERED;
        case 1: return NetworkStatus::REGISTERED_HOME;
        case 2: return NetworkStatus::SEARCHING;
        case 3: return NetworkStatus::DENIED;
        case 5: return NetworkStatus::REGISTERED_ROAMING;
        default: return NetworkStatus::UNKNOWN;
    }
}

int A7670Modem::getSignalQuality() {
    if (!m_dce) {
        return 99;
    }

    lock();
    int rssi = 99, ber = 99;
    esp_modem::command_result result = m_dce->get_signal_quality(rssi, ber);
    unlock();

    if (result == esp_modem::command_result::OK) {
        return rssi;
    }
    return 99;
}

cube32_result_t A7670Modem::setDataMode() {
    if (!m_dce) {
        return CUBE32_NOT_INITIALIZED;
    }

    if (m_state == ModemState::DATA_MODE) {
        ESP_LOGW(TAG, "Already in data mode");
        return CUBE32_OK;
    }

    ESP_LOGI(TAG, "Switching to data mode...");

    lock();
    bool success = m_dce->set_mode(esp_modem::modem_mode::DATA_MODE);
    unlock();

    if (success) {
        // Allow USB terminal swap and PPP to stabilize before declaring success.
        // In DUAL_MODE the DTE swaps primary/secondary terminals and re-sets callbacks;
        // without this delay, incoming PPP data can race with callback setup.
        vTaskDelay(pdMS_TO_TICKS(CUBE32_MODEM_DATA_MODE_SETTLE_MS));

        setState(ModemState::DATA_MODE);
        m_ppp_connected = true;
        m_usb_error_count = 0;  // Reset error counter on successful mode switch
        xEventGroupSetBits(m_event_group, MODEM_PPP_CONNECTED_BIT);
        xEventGroupClearBits(m_event_group, MODEM_PPP_DISCONNECTED_BIT);
        ESP_LOGI(TAG, "Data mode active");
        return CUBE32_OK;
    }

    ESP_LOGE(TAG, "Failed to switch to data mode");
    return CUBE32_ERROR;
}

cube32_result_t A7670Modem::setCommandMode() {
    if (!m_dce) {
        return CUBE32_NOT_INITIALIZED;
    }

    if (m_state == ModemState::COMMAND_MODE) {
        ESP_LOGW(TAG, "Already in command mode");
        return CUBE32_OK;
    }

    ESP_LOGI(TAG, "Switching to command mode...");

    lock();
    bool success = m_dce->set_mode(esp_modem::modem_mode::COMMAND_MODE);
    unlock();

    if (success) {
        setState(ModemState::COMMAND_MODE);
        m_ppp_connected = false;
        xEventGroupSetBits(m_event_group, MODEM_PPP_DISCONNECTED_BIT);
        xEventGroupClearBits(m_event_group, MODEM_PPP_CONNECTED_BIT);
        ESP_LOGI(TAG, "Command mode active");
        return CUBE32_OK;
    }

    ESP_LOGE(TAG, "Failed to switch to command mode");
    return CUBE32_ERROR;
}

cube32_result_t A7670Modem::getModemInfo(ModemInfo& info) {
    if (!m_dce) {
        ESP_LOGE(TAG, "getModemInfo: DCE not initialized");
        return CUBE32_NOT_INITIALIZED;
    }

    lock();
    
    std::string str;
    esp_modem::command_result result;
    
    // Get manufacturer using AT+CGMI
    ESP_LOGI(TAG, "Getting manufacturer (AT+CGMI)...");
    result = m_dce->at("AT+CGMI", str, 3000);
    ESP_LOGI(TAG, "AT+CGMI result: %d, response: %s", (int)result, str.c_str());
    if (result == esp_modem::command_result::OK) {
        // Clean up response - remove "OK" and extra whitespace
        size_t ok_pos = str.find("OK");
        if (ok_pos != std::string::npos) {
            str = str.substr(0, ok_pos);
        }
        // Trim whitespace
        while (!str.empty() && (str.back() == '\r' || str.back() == '\n' || str.back() == ' ')) {
            str.pop_back();
        }
        info.manufacturer = str;
    }
    
    // Get model using get_module_name
    ESP_LOGI(TAG, "Getting module name...");
    result = m_dce->get_module_name(str);
    ESP_LOGI(TAG, "get_module_name result: %d, response: %s", (int)result, str.c_str());
    if (result == esp_modem::command_result::OK) {
        info.model = str;
    }
    
    // Get revision using AT+CGMR
    ESP_LOGI(TAG, "Getting revision (AT+CGMR)...");
    result = m_dce->at("AT+CGMR", str, 3000);
    ESP_LOGI(TAG, "AT+CGMR result: %d, response: %s", (int)result, str.c_str());
    if (result == esp_modem::command_result::OK) {
        // Clean up response
        size_t ok_pos = str.find("OK");
        if (ok_pos != std::string::npos) {
            str = str.substr(0, ok_pos);
        }
        while (!str.empty() && (str.back() == '\r' || str.back() == '\n' || str.back() == ' ')) {
            str.pop_back();
        }
        info.revision = str;
    }
    
    // Get IMEI
    ESP_LOGI(TAG, "Getting IMEI...");
    result = m_dce->get_imei(str);
    ESP_LOGI(TAG, "get_imei result: %d, response: %s", (int)result, str.c_str());
    if (result == esp_modem::command_result::OK) {
        info.imei = str;
    }
    
    // Get IMSI
    ESP_LOGI(TAG, "Getting IMSI...");
    result = m_dce->get_imsi(str);
    ESP_LOGI(TAG, "get_imsi result: %d, response: %s", (int)result, str.c_str());
    if (result == esp_modem::command_result::OK) {
        info.imsi = str;
    }

    // Get signal quality
    ESP_LOGI(TAG, "Getting signal quality...");
    int rssi = 99, ber = 99;
    result = m_dce->get_signal_quality(rssi, ber);
    ESP_LOGI(TAG, "get_signal_quality result: %d, rssi: %d, ber: %d", (int)result, rssi, ber);
    info.signalQuality = rssi;
    
    // Get network status (don't block, use quick check)
    ESP_LOGI(TAG, "Getting network status...");
    info.networkStatus = NetworkStatus::UNKNOWN;
    int stat = 0;
    result = m_dce->get_network_registration_state(stat);
    ESP_LOGI(TAG, "get_network_registration_state result: %d, stat: %d", (int)result, stat);
    if (result == esp_modem::command_result::OK) {
        switch (stat) {
            case 0: info.networkStatus = NetworkStatus::NOT_REGISTERED; break;
            case 1: info.networkStatus = NetworkStatus::REGISTERED_HOME; break;
            case 2: info.networkStatus = NetworkStatus::SEARCHING; break;
            case 3: info.networkStatus = NetworkStatus::DENIED; break;
            case 5: info.networkStatus = NetworkStatus::REGISTERED_ROAMING; break;
            default: info.networkStatus = NetworkStatus::UNKNOWN; break;
        }
    }
    
    // Get operator name - only if registered to network
    if (info.networkStatus == NetworkStatus::REGISTERED_HOME || 
        info.networkStatus == NetworkStatus::REGISTERED_ROAMING) {
        ESP_LOGI(TAG, "Getting operator name...");
        int act = 0;
        result = m_dce->get_operator_name(str, act);
        ESP_LOGI(TAG, "get_operator_name result: %d, operator: %s, act: %d", (int)result, str.c_str(), act);
        if (result == esp_modem::command_result::OK) {
            info.operatorName = str;
        }
    } else {
        ESP_LOGI(TAG, "Skipping operator name (not registered to network)");
        info.operatorName = "Not registered";
    }
    
    ESP_LOGI(TAG, "getModemInfo completed successfully");
    unlock();
    return CUBE32_OK;
}

cube32_result_t A7670Modem::getIMEI(std::string& imei) {
    if (!m_dce) {
        return CUBE32_NOT_INITIALIZED;
    }

    lock();
    esp_modem::command_result result = m_dce->get_imei(imei);
    unlock();

    return (result == esp_modem::command_result::OK) ? CUBE32_OK : CUBE32_ERROR;
}

cube32_result_t A7670Modem::getIMSI(std::string& imsi) {
    if (!m_dce) {
        return CUBE32_NOT_INITIALIZED;
    }

    lock();
    esp_modem::command_result result = m_dce->get_imsi(imsi);
    unlock();

    return (result == esp_modem::command_result::OK) ? CUBE32_OK : CUBE32_ERROR;
}

cube32_result_t A7670Modem::getOperatorName(std::string& name) {
    if (!m_dce) {
        return CUBE32_NOT_INITIALIZED;
    }

    lock();
    int act = 0;
    esp_modem::command_result result = m_dce->get_operator_name(name, act);
    unlock();

    return (result == esp_modem::command_result::OK) ? CUBE32_OK : CUBE32_ERROR;
}

cube32_result_t A7670Modem::sendATCommand(const std::string& command,
                                           uint32_t timeout_ms) {
    if (!m_dce) {
        return CUBE32_NOT_INITIALIZED;
    }

    lock();
    esp_modem::command_result result = m_dce->command(
        command,
        [](uint8_t *data, size_t len) -> esp_modem::command_result {
            std::string response((char*)data, len);
            ESP_LOGD(TAG, "AT Response: %s", response.c_str());
            // Return OK on finding "OK" in response
            if (response.find("OK") != std::string::npos) {
                return esp_modem::command_result::OK;
            }
            if (response.find("ERROR") != std::string::npos) {
                return esp_modem::command_result::FAIL;
            }
            return esp_modem::command_result::TIMEOUT;
        },
        timeout_ms
    );
    unlock();

    return (result == esp_modem::command_result::OK) ? CUBE32_OK : CUBE32_ERROR;
}

cube32_result_t A7670Modem::sendCommand(const std::string& command,
                                         std::string& response,
                                         uint32_t timeout_ms) {
    if (!m_dce) {
        return CUBE32_NOT_INITIALIZED;
    }

    std::string captured_response;
    
    lock();
    esp_modem::command_result result = m_dce->command(
        "AT" + command + "\r",
        [&captured_response](uint8_t *data, size_t len) -> esp_modem::command_result {
            captured_response.append((char*)data, len);
            if (captured_response.find("OK") != std::string::npos) {
                return esp_modem::command_result::OK;
            }
            if (captured_response.find("ERROR") != std::string::npos) {
                return esp_modem::command_result::FAIL;
            }
            return esp_modem::command_result::TIMEOUT;
        },
        timeout_ms
    );
    unlock();

    response = captured_response;
    return (result == esp_modem::command_result::OK) ? CUBE32_OK : CUBE32_ERROR;
}

cube32_result_t A7670Modem::getIPAddress(std::string& ip) {
    if (!m_netif) {
        return CUBE32_NOT_INITIALIZED;
    }

    esp_netif_ip_info_t ip_info;
    if (esp_netif_get_ip_info(m_netif, &ip_info) == ESP_OK) {
        char ip_str[16];
        snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ip_info.ip));
        ip = ip_str;
        return CUBE32_OK;
    }

    return CUBE32_ERROR;
}

void A7670Modem::printStatus() {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "A7670 Modem Status");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "State: %s", getStateString());
    ESP_LOGI(TAG, "Connection: %s", 
             m_config.connectionType == ModemConnectionType::USB ? "USB" : "UART");
    
    // IO Expander status
    if (m_iox_initialized) {
        ESP_LOGI(TAG, "IO Expander: Initialized (addr: 0x%02X)", m_config.iox_i2c_addr);
        ESP_LOGI(TAG, "  Power Rail: %s", isPowerRailOn() ? "ON" : "OFF");
        ESP_LOGI(TAG, "  Comm Port: %s", isCommPortUART() ? "UART" : "USB");
        ESP_LOGI(TAG, "  Output State: 0x%02X", m_iox_output_state);
    } else {
        ESP_LOGI(TAG, "IO Expander: Not available");
    }
    
    if (m_state >= ModemState::CONNECTED) {
        ModemInfo info;
        if (getModemInfo(info) == CUBE32_OK) {
            ESP_LOGI(TAG, "Manufacturer: %s", info.manufacturer.c_str());
            ESP_LOGI(TAG, "Model: %s", info.model.c_str());
            ESP_LOGI(TAG, "IMEI: %s", info.imei.c_str());
            ESP_LOGI(TAG, "Signal Quality: %d", info.signalQuality);
            ESP_LOGI(TAG, "Operator: %s", info.operatorName.c_str());
        }
        
        std::string ip;
        if (getIPAddress(ip) == CUBE32_OK) {
            ESP_LOGI(TAG, "IP Address: %s", ip.c_str());
        }
    }
    
    ESP_LOGI(TAG, "PPP Connected: %s", m_ppp_connected ? "Yes" : "No");
    ESP_LOGI(TAG, "========================================");
}

} // namespace cube32
