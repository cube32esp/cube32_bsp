/**
 * @file a7670_modem.h
 * @brief CUBE32 A7670 Modem Driver (Pure C++)
 * 
 * This driver provides a C++ class interface for the SimCom A7670 LTE modem
 * using the ESP-Modem component via USB or UART connection.
 * 
 * The driver supports:
 * - USB connection (primary, using esp_modem_usb_dte)
 * - UART connection (optional, for future implementation)
 * - PPP data mode for internet connectivity
 * - AT command interface
 * - Network status monitoring
 * 
 * USB Mode:
 * - Uses USB CDC-ACM class
 * - VID: 0x1E0E, PID: 0x9011
 * - Interfaces: 4 (AT) and 5 (Data)
 */

#ifndef CUBE32_DRIVERS_MODEM_A7670_H
#define CUBE32_DRIVERS_MODEM_A7670_H

#include "utils/common.h"
#include "drivers/io_expander/tca9554.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/event_groups.h>
#include <esp_netif.h>
#include <cstdint>
#include <atomic>
#include <string>
#include <functional>
#include <memory>

// Forward declarations for ESP-Modem types
namespace esp_modem {
    class DCE;
    class DTE;
}

namespace cube32 {

// ============================================================================
// Constants
// ============================================================================

// A7670 USB configuration
#define CUBE32_MODEM_A7670_USB_VID           0x1E0E
#define CUBE32_MODEM_A7670_USB_PID           0x9011
#define CUBE32_MODEM_A7670_USB_AT_INTF       4
#define CUBE32_MODEM_A7670_USB_DATA_INTF     5

// Default timeout values (ms)
#define CUBE32_MODEM_DEFAULT_TIMEOUT_MS      10000
#define CUBE32_MODEM_CONNECT_TIMEOUT_MS      60000
#define CUBE32_MODEM_USB_WAIT_TIMEOUT_MS     30000

// USB DTE buffer size (must accommodate PPP MTU 1500 + framing)
#define CUBE32_MODEM_DTE_BUFFER_SIZE         1600

// USB error recovery thresholds
#define CUBE32_MODEM_USB_ERROR_THRESHOLD     3       ///< Errors within window to trigger recovery
#define CUBE32_MODEM_USB_ERROR_WINDOW_MS     5000    ///< Time window for error counting
#define CUBE32_MODEM_DATA_MODE_SETTLE_MS     200     ///< Stabilization delay after entering data mode

// Event group bits
#define MODEM_CONNECTED_BIT                  BIT0
#define MODEM_DISCONNECTED_BIT               BIT1
#define MODEM_DATA_READY_BIT                 BIT2
#define MODEM_PPP_CONNECTED_BIT              BIT3
#define MODEM_PPP_DISCONNECTED_BIT           BIT4
#define MODEM_NETWORK_REGISTERED_BIT         BIT5   ///< Network registration completed
#define MODEM_NETWORK_REGISTRATION_FAILED_BIT BIT6  ///< Network registration failed/timeout
#define MODEM_INIT_COMPLETE_BIT              BIT7   ///< Modem initialization completed
#define MODEM_INIT_FAILED_BIT                BIT8   ///< Modem initialization failed

// ============================================================================
// Types
// ============================================================================

/**
 * @brief Modem connection type
 */
enum class ModemConnectionType : uint8_t {
    USB = 0,        ///< USB CDC-ACM connection
    UART,           ///< UART connection (future)
};

/**
 * @brief Modem state
 */
enum class ModemState : uint8_t {
    NOT_INITIALIZED = 0, ///< Driver not initialized
    INITIALIZING,        ///< Initialization in progress (async)
    INITIALIZED,         ///< Driver initialized, modem not connected
    CONNECTED,           ///< Modem connected (USB/UART)
    COMMAND_MODE,        ///< In AT command mode
    DATA_MODE,           ///< In PPP data mode
    ERROR,               ///< Error state
};

/**
 * @brief Network registration status
 */
enum class NetworkStatus : uint8_t {
    NOT_REGISTERED = 0,  ///< Not registered
    REGISTERED_HOME,     ///< Registered, home network
    SEARCHING,           ///< Searching for network
    DENIED,              ///< Registration denied
    UNKNOWN,             ///< Unknown status
    REGISTERED_ROAMING,  ///< Registered, roaming
};

/**
 * @brief Modem information structure
 */
struct ModemInfo {
    std::string manufacturer;    ///< Manufacturer name
    std::string model;           ///< Model name
    std::string revision;        ///< Firmware revision
    std::string imei;            ///< IMEI number
    std::string imsi;            ///< IMSI number
    std::string iccid;           ///< SIM ICCID
    int signalQuality = 0;       ///< Signal quality (0-31, 99=unknown)
    NetworkStatus networkStatus = NetworkStatus::NOT_REGISTERED;
    std::string operatorName;    ///< Operator name
};

/**
 * @brief Modem configuration structure
 */
struct ModemConfig {
    ModemConnectionType connectionType = ModemConnectionType::USB;
    
    // USB configuration
    uint16_t usb_vid = CUBE32_MODEM_A7670_USB_VID;
    uint16_t usb_pid = CUBE32_MODEM_A7670_USB_PID;
    int usb_at_interface = CUBE32_MODEM_A7670_USB_AT_INTF;
    int usb_data_interface = CUBE32_MODEM_A7670_USB_DATA_INTF;
    uint32_t usb_timeout_ms = CUBE32_MODEM_USB_WAIT_TIMEOUT_MS;
    
    // UART configuration
    int uart_port = CUBE32_MODEM_UART_PORT_NUM;
    int uart_tx_pin = CUBE32_MODEM_UART_TX_PIN;
    int uart_rx_pin = CUBE32_MODEM_UART_RX_PIN;
    int uart_baud_rate = CUBE32_MODEM_UART_BAUD_RATE_DEF;
    
    // IO Expander configuration
    uint8_t iox_i2c_addr = 0x22;     // TCA9554 I2C address for modem control
    uint8_t iox_pwrkey_pin = 0;      // A7670 PWRKEY PIN
    uint8_t iox_dtr_pin = 1;         // A7670 DTR PIN (idle high)
    uint8_t iox_pwr_rail_pin = 2;    // A7670 Power Rail: L-Off, H-On
    uint8_t iox_comm_sel_pin = 3;    // MCU-A7670 Comm Port: L-USB, H-UART
    
    // General configuration
    std::string apn = "internet";
    uint32_t command_timeout_ms = CUBE32_MODEM_DEFAULT_TIMEOUT_MS;
    bool auto_reconnect = true;
};

/**
 * @brief Callback type for AT command responses
 */
using ATResponseCallback = std::function<void(const std::string& response)>;

/**
 * @brief Callback type for modem state changes
 */
using ModemStateCallback = std::function<void(ModemState state)>;

/**
 * @brief Callback type for network registration events
 * @param success true if registered successfully, false if failed/timeout
 * @param status The network status at the time of callback
 */
using NetworkReadyCallback = std::function<void(bool success, NetworkStatus status)>;

/**
 * @brief Callback type for modem initialization completion
 * @param success true if initialization completed successfully
 * @param state The modem state after initialization
 */
using ModemInitCallback = std::function<void(bool success, ModemState state)>;

/**
 * @brief Default modem configuration
 */
#define CUBE32_MODEM_CONFIG_DEFAULT() cube32::ModemConfig{}

// ============================================================================
// A7670Modem Class
// ============================================================================

/**
 * @brief A7670 Modem Driver Class
 * 
 * Object-oriented interface for the SimCom A7670 LTE modem using ESP-Modem.
 * 
 * Usage (USB mode):
 * @code
 *   cube32::A7670Modem& modem = cube32::A7670Modem::instance();
 *   cube32::ModemConfig config = CUBE32_MODEM_CONFIG_DEFAULT();
 *   config.apn = "my_apn";
 *   
 *   if (modem.begin(config) == CUBE32_OK) {
 *       // Wait for network
 *       if (modem.waitForNetwork(30000) == CUBE32_OK) {
 *           // Enter data mode
 *           modem.setDataMode();
 *           // Now PPP connection is active
 *       }
 *   }
 * @endcode
 */
class A7670Modem {
public:
    /**
     * @brief Get the singleton A7670Modem instance
     */
    static A7670Modem& instance();

    /**
     * @brief Initialize the modem driver
     * @param config Modem configuration
     * @return CUBE32_OK on success
     */
    cube32_result_t begin(const ModemConfig& config = CUBE32_MODEM_CONFIG_DEFAULT());

    /**
     * @brief Initialize the modem driver asynchronously
     * 
     * This method starts modem initialization in a background task, allowing
     * other drivers to initialize without blocking. The entire modem init
     * sequence (power on, USB/UART init, and optionally network registration)
     * runs in the background.
     * 
     * Use isInitialized() or wait for MODEM_INIT_COMPLETE_BIT to know when ready.
     * 
     * @param config Modem configuration
     * @param auto_register If true, automatically starts network registration after init
     * @param callback Optional callback when initialization completes
     * @return CUBE32_OK if task started successfully
     */
    cube32_result_t beginAsync(const ModemConfig& config = CUBE32_MODEM_CONFIG_DEFAULT(),
                                bool auto_register = true,
                                ModemInitCallback callback = nullptr);

    /**
     * @brief Check if async initialization is in progress
     */
    bool isInitializing() const { return m_state == ModemState::INITIALIZING; }

    /**
     * @brief Deinitialize the modem driver
     * @return CUBE32_OK on success
     */
    cube32_result_t end();

    /**
     * @brief Check if initialized (complete, not initializing)
     */
    bool isInitialized() const { 
        return m_state != ModemState::NOT_INITIALIZED && 
               m_state != ModemState::INITIALIZING; 
    }

    /**
     * @brief Get current modem state
     */
    ModemState getState() const { return m_state; }

    /**
     * @brief Get state as string
     */
    const char* getStateString() const;

    /**
     * @brief Get the configured connection type (USB or UART)
     */
    ModemConnectionType getConnectionType() const { return m_config.connectionType; }

    // ---- Network Operations ----

    /**
     * @brief Wait for network registration (blocking)
     * @param timeout_ms Timeout in milliseconds
     * @return CUBE32_OK if registered, CUBE32_TIMEOUT if timeout
     */
    cube32_result_t waitForNetwork(uint32_t timeout_ms = CUBE32_MODEM_CONNECT_TIMEOUT_MS);

    /**
     * @brief Start asynchronous network registration
     * 
     * This method starts a background task that waits for network registration.
     * The callback is called when registration succeeds or times out.
     * Use isNetworkReady() to check registration status at any time.
     * Use getEventGroup() and MODEM_NETWORK_REGISTERED_BIT to wait for completion.
     * 
     * @param timeout_ms Timeout in milliseconds
     * @param callback Optional callback when registration completes
     * @return CUBE32_OK if task started successfully
     */
    cube32_result_t startNetworkRegistration(uint32_t timeout_ms = CUBE32_MODEM_CONNECT_TIMEOUT_MS,
                                              NetworkReadyCallback callback = nullptr);

    /**
     * @brief Check if network is registered and ready
     * @return true if network is registered (home or roaming)
     */
    bool isNetworkReady() const { return m_network_ready; }

    /**
     * @brief Get network registration status
     */
    NetworkStatus getNetworkStatus();

    /**
     * @brief Get signal quality (RSSI)
     * @return Signal quality (0-31, 99=unknown)
     */
    int getSignalQuality();

    // ---- Mode Control ----

    /**
     * @brief Switch to data mode (PPP)
     * @return CUBE32_OK on success
     */
    cube32_result_t setDataMode();

    /**
     * @brief Switch to command mode
     * @return CUBE32_OK on success
     */
    cube32_result_t setCommandMode();

    /**
     * @brief Check if in data mode
     */
    bool isDataMode() const { return m_state == ModemState::DATA_MODE; }

    /**
     * @brief Check if in command mode
     */
    bool isCommandMode() const { return m_state == ModemState::COMMAND_MODE; }

    // ---- Modem Information ----

    /**
     * @brief Get modem information
     * @param info Output parameter for modem info
     * @return CUBE32_OK on success
     */
    cube32_result_t getModemInfo(ModemInfo& info);

    /**
     * @brief Get IMEI
     * @param imei Output string
     * @return CUBE32_OK on success
     */
    cube32_result_t getIMEI(std::string& imei);

    /**
     * @brief Get IMSI
     * @param imsi Output string
     * @return CUBE32_OK on success
     */
    cube32_result_t getIMSI(std::string& imsi);

    /**
     * @brief Get operator name
     * @param name Output string
     * @return CUBE32_OK on success
     */
    cube32_result_t getOperatorName(std::string& name);

    // ---- AT Commands ----

    /**
     * @brief Send AT command and wait for response
     * @param command AT command (without AT prefix)
     * @param response Output response string
     * @param timeout_ms Timeout in milliseconds
     * @return CUBE32_OK on success
     */
    cube32_result_t sendCommand(const std::string& command, 
                                std::string& response,
                                uint32_t timeout_ms = CUBE32_MODEM_DEFAULT_TIMEOUT_MS);

    /**
     * @brief Send raw AT command
     * @param command Full AT command string
     * @param timeout_ms Timeout in milliseconds
     * @return CUBE32_OK if OK response received
    */
    cube32_result_t sendATCommand(const std::string& command,
                                  uint32_t timeout_ms = CUBE32_MODEM_DEFAULT_TIMEOUT_MS);

    // ---- PPP/Network Interface ----

    /**
     * @brief Get the network interface handle
     * @return esp_netif_t pointer or nullptr if not available
     */
    esp_netif_t* getNetif() const { return m_netif; }

    /**
     * @brief Get IP address string
     * @param ip Output IP address string
     * @return CUBE32_OK if IP is available
     */
    cube32_result_t getIPAddress(std::string& ip);

    /**
     * @brief Check if PPP is connected
     */
    bool isPPPConnected() const { return m_ppp_connected; }

    // ---- Callbacks ----

    /**
     * @brief Set state change callback
     * @param callback Callback function
     */
    void setStateCallback(ModemStateCallback callback) { m_state_callback = callback; }

    /**
     * @brief Set network ready callback
     * @param callback Callback function called when network registration completes or fails
     */
    void setNetworkReadyCallback(NetworkReadyCallback callback) { m_network_callback = callback; }

    // ---- Event Group ----

    /**
     * @brief Get event group handle for waiting on events
     */
    EventGroupHandle_t getEventGroup() const { return m_event_group; }

    // ---- IO Expander Control ----

    /**
     * @brief Check if IO Expander is initialized
     * @return true if IO Expander is available
     */
    bool isIOExpanderReady() const { return m_iox_initialized; }

    /**
     * @brief Power on the modem (full power-on sequence)
     * 
     * This performs the complete power-on sequence:
     * 1. Enable power rail
     * 2. Set DTR high (idle)
     * 3. Pulse PWRKEY to turn on modem
     * 4. Wait for modem to be ready
     * 
     * @return CUBE32_OK on success
     */
    cube32_result_t powerOn();

    /**
     * @brief Power off the modem
     * 
     * This performs power-off sequence:
     * 1. Pulse PWRKEY to turn off modem
     * 2. Disable power rail
     * 
     * @return CUBE32_OK on success
     */
    cube32_result_t powerOff();

    /**
     * @brief Set PWRKEY pin state (pulse to toggle modem power)
     * @param on true to assert PWRKEY (low), false to release (high)
     * @return CUBE32_OK on success
     */
    cube32_result_t setPwrKey(bool on);

    /**
     * @brief Pulse PWRKEY to turn on modem
     * @return CUBE32_OK on success
     */
    cube32_result_t pwrKeyOn();

    /**
     * @brief Pulse PWRKEY to turn off modem
     * @return CUBE32_OK on success
     */
    cube32_result_t pwrKeyOff();

    /**
     * @brief Set DTR pin state
     * @param high true for high (idle), false for low
     * @return CUBE32_OK on success
     */
    cube32_result_t setDTR(bool high);

    /**
     * @brief Set power rail state
     * @param on true to enable power rail, false to disable
     * @return CUBE32_OK on success
     */
    cube32_result_t setPowerRail(bool on);

    /**
     * @brief Set communication port selection
     * @param uart true for UART, false for USB
     * @return CUBE32_OK on success
     */
    cube32_result_t setCommPort(bool uart);

    /**
     * @brief Get IO Expander pin states
     * @param states Output: bitmask of current pin states
     * @return CUBE32_OK on success
     */
    cube32_result_t getIOStates(uint8_t& states);

    /**
     * @brief Get power rail state
     * @return true if power rail is on
     */
    bool isPowerRailOn() const;

    /**
     * @brief Get communication port selection
     * @return true if UART, false if USB
     */
    bool isCommPortUART() const;

    // ---- Diagnostics ----

    /**
     * @brief Print modem status to log
     */
    void printStatus();

    /**
     * @brief Reset the modem
     * @return CUBE32_OK on success
     */
    cube32_result_t reset();

    /**
     * @brief Sync with modem (send AT and check response)
     * @return CUBE32_OK if modem responds
     */
    cube32_result_t sync();

private:
    A7670Modem() = default;
    ~A7670Modem();
    
    // Prevent copying
    A7670Modem(const A7670Modem&) = delete;
    A7670Modem& operator=(const A7670Modem&) = delete;

    // Thread safety
    void lock();
    void unlock();

    // Internal helpers
    cube32_result_t initUSB();
    cube32_result_t initUART();
    cube32_result_t initIOExpander();
    void setState(ModemState state);
    static void pppEventHandler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data);
    static void networkRegistrationTask(void* arg);
    static void modemInitTask(void* arg);
    static void recoveryTask(void* arg);
    void requestRecovery();

    // State
    std::atomic<ModemState> m_state{ModemState::NOT_INITIALIZED};
    bool m_shutting_down = false;
    ModemConfig m_config;
    bool m_ppp_connected = false;
    bool m_network_ready = false;  ///< Network registration completed successfully

    // IO Expander for modem control
    TCA9554 m_io_expander;
    bool m_iox_initialized = false;
    uint8_t m_iox_output_state = 0;  // Cache of output pin states

    // ESP-Modem objects
    std::shared_ptr<esp_modem::DTE> m_dte;
    std::unique_ptr<esp_modem::DCE> m_dce;
    esp_netif_t* m_netif = nullptr;

    // Synchronization
    SemaphoreHandle_t m_mutex = nullptr;
    EventGroupHandle_t m_event_group = nullptr;

    // Async network registration
    TaskHandle_t m_network_task = nullptr;
    uint32_t m_network_timeout_ms = CUBE32_MODEM_CONNECT_TIMEOUT_MS;

    // Async initialization
    TaskHandle_t m_init_task = nullptr;
    bool m_auto_register = true;
    ModemInitCallback m_init_callback = nullptr;

    // USB error tracking and recovery
    TaskHandle_t m_recovery_task = nullptr;
    std::atomic<uint32_t> m_usb_error_count{0};
    uint32_t m_first_usb_error_time = 0;  ///< Tick count of first error in current window
    std::atomic<bool> m_recovery_pending{false};

    // Callbacks
    ModemStateCallback m_state_callback = nullptr;
    NetworkReadyCallback m_network_callback = nullptr;
};

} // namespace cube32

#endif // CUBE32_DRIVERS_MODEM_A7670_H
