/**
 * @file ble_ota.h
 * @brief CUBE32 BLE OTA Driver
 * 
 * This driver provides BLE-based OTA firmware update and command/response
 * communication for the CUBE32 board using the ESP BLE OTA component
 * with NimBLE stack.
 * 
 * Features:
 * - BLE GATT server with OTA service
 * - Command/Response communication via custom characteristic
 * - OTA firmware updates from BLE client
 * - Double-reset detection for BLE OTA start trigger
 * - Status and configuration query commands
 * - System restart and power off commands
 * 
 * BLE Services:
 * - Device Information Service (0x180A)
 * - OTA Service with characteristics:
 *   - RECV_FW_CHAR (0x8020): Receive firmware data
 *   - PROGRESS_BAR_CHAR (0x8021): OTA progress
 *   - COMMAND_CHAR (0x8022): Command/ACK
 *   - CUSTOMER_CHAR (0x8023): Custom data (commands/responses)
 */

#ifndef CUBE32_DRIVERS_BLE_OTA_H
#define CUBE32_DRIVERS_BLE_OTA_H

#include "utils/common.h"
#include <cstdint>
#include <cstring>
#include <functional>
#include <string>

// ============================================================================
// Constants
// ============================================================================

#define CUBE32_BLE_OTA_DEVICE_NAME_MAX      32
#define CUBE32_BLE_OTA_CMD_MAX_LEN          512
#define CUBE32_BLE_OTA_RESP_MAX_LEN         256

// Command IDs for CUBE32 BLE protocol
#define CUBE32_BLE_CMD_GET_STATUS           0x01
#define CUBE32_BLE_CMD_GET_CONFIG           0x02
#define CUBE32_BLE_CMD_SET_CONFIG           0x03
#define CUBE32_BLE_CMD_TEXT_MESSAGE         0x04
#define CUBE32_BLE_CMD_SET_DEVICE_ID        0x05
#define CUBE32_BLE_CMD_RESET_CONFIG         0x06
#define CUBE32_BLE_CMD_RESTART              0x10
#define CUBE32_BLE_CMD_POWER_OFF            0x11
#define CUBE32_BLE_CMD_START_OTA            0x20
#define CUBE32_BLE_CMD_OTA_DATA             0x21
#define CUBE32_BLE_CMD_OTA_END              0x22
#define CUBE32_BLE_CMD_OTA_ABORT            0x23

// Response status codes
#define CUBE32_BLE_RESP_OK                  0x00
#define CUBE32_BLE_RESP_ERROR               0x01
#define CUBE32_BLE_RESP_INVALID_CMD         0x02
#define CUBE32_BLE_RESP_BUSY                0x03
#define CUBE32_BLE_RESP_OTA_READY           0x10
#define CUBE32_BLE_RESP_OTA_PROGRESS        0x11
#define CUBE32_BLE_RESP_OTA_COMPLETE        0x12
#define CUBE32_BLE_RESP_OTA_ERROR           0x13

// Config key IDs for SET_CONFIG command
#define CUBE32_BLE_CFG_KEY_ROTATION         0x01
#define CUBE32_BLE_CFG_KEY_PRISM            0x02
#define CUBE32_BLE_CFG_KEY_VOLUME           0x03
#define CUBE32_BLE_CFG_KEY_MAX_DB           0x04
#define CUBE32_BLE_CFG_KEY_IN_RATE          0x05
#define CUBE32_BLE_CFG_KEY_OUT_RATE         0x06
#define CUBE32_BLE_CFG_KEY_TIMEZONE         0x07
#define CUBE32_BLE_CFG_KEY_NTP_SYNC         0x08
#define CUBE32_BLE_CFG_KEY_CAM_HMIRROR      0x09
#define CUBE32_BLE_CFG_KEY_CAM_VFLIP        0x0A
#define CUBE32_BLE_CFG_KEY_WIFI_SSID        0x0B
#define CUBE32_BLE_CFG_KEY_WIFI_PASS        0x0C
#define CUBE32_BLE_CFG_KEY_MODEM_APN        0x0D
#define CUBE32_BLE_CFG_KEY_BLE_NAME         0x0E
#define CUBE32_BLE_CFG_KEY_ACT_CAMERA       0x10
#define CUBE32_BLE_CFG_KEY_ACT_AUDIO        0x11
#define CUBE32_BLE_CFG_KEY_ACT_MODEM        0x12
#define CUBE32_BLE_CFG_KEY_ACT_SDCARD       0x13
#define CUBE32_BLE_CFG_KEY_ACT_USB_IN       0x14
#define CUBE32_BLE_CFG_KEY_ACT_BLE_OTA      0x15
#define CUBE32_BLE_CFG_KEY_ACT_ADC_BTN      0x16
#define CUBE32_BLE_CFG_KEY_ACT_SERVO        0x17
#define CUBE32_BLE_CFG_KEY_ACT_IMU          0x18
#define CUBE32_BLE_CFG_KEY_ACT_MAG          0x19

namespace cube32 {

// ============================================================================
// Types
// ============================================================================

/**
 * @brief BLE connection state
 */
enum class BleConnectionState {
    DISCONNECTED = 0,
    CONNECTING,
    CONNECTED,
    DISCONNECTING
};

/**
 * @brief BLE OTA state
 */
enum class BleOtaState {
    IDLE = 0,
    ADVERTISING,
    CONNECTED,
    OTA_IN_PROGRESS,
    OTA_COMPLETE,
    ERROR,
    BYPASSED           ///< BLE bypassed (double-boot not detected)
};

/**
 * @brief OTA firmware information received from client
 */
struct OtaFirmwareInfo {
    uint32_t total_size = 0;        // Total firmware size in bytes
    uint32_t received_size = 0;     // Received size so far
    char version[32] = {0};         // Firmware version string
    char project_name[32] = {0};    // Project name
    char build_date[32] = {0};      // Build date
    char build_time[16] = {0};      // Build time  
    char app_full_name[64] = {0};   // Human-readable full app name (from BLE client)
    uint32_t checksum = 0;          // Optional checksum
    bool is_valid = false;          // Whether info is valid
};

/**
 * @brief BLE OTA driver state for UI display
 */
struct BleOtaStatus {
    BleOtaState state = BleOtaState::IDLE;
    BleConnectionState connection_state = BleConnectionState::DISCONNECTED;
    bool is_initialized = false;
    bool is_advertising = false;
    bool is_connected = false;
    uint16_t conn_handle = 0xFFFF;
    uint8_t connected_addr[6] = {0};
    int ota_progress = 0;           // 0-100%
    uint32_t ota_total_size = 0;
    uint32_t ota_received_size = 0;
    OtaFirmwareInfo firmware_info;  // Firmware info from client
    bool ota_ready_to_restart = false;  // OTA complete, waiting for restart
    char last_command[64] = {0};
    char last_response[64] = {0};
    uint32_t rx_count = 0;          // Received command count
    uint32_t tx_count = 0;          // Sent response count
    char ota_error_msg[64] = {0};   // Last OTA error message
};

/**
 * @brief BLE OTA configuration
 */
struct BleOtaConfig {
    char device_name[CUBE32_BLE_OTA_DEVICE_NAME_MAX] = "CUBE32-OTA";
    bool enable_on_double_boot = false;
    uint32_t double_boot_timeout_ms = 5000;
    uint16_t adv_interval_min = 0x20;   // 20ms
    uint16_t adv_interval_max = 0x40;   // 40ms
    uint16_t mtu_size = 512;            // MTU size
};

/**
 * @brief Callback for received commands
 */
using BleCommandCallback = std::function<void(uint8_t cmd_id, const uint8_t* data, size_t len)>;

/**
 * @brief Callback for connection state changes
 */
using BleConnectionCallback = std::function<void(BleConnectionState state)>;

/**
 * @brief Callback for OTA progress
 */
using BleOtaProgressCallback = std::function<void(int progress, uint32_t received, uint32_t total)>;

/**
 * @brief Callback for text message subscribers
 * @param text Null-terminated text message from BLE client
 * @param len  Length of text (excluding null terminator)
 */
using BleTextMessageCallback = std::function<void(const char* text, size_t len)>;

/** Maximum number of text message subscribers */
#define CUBE32_BLE_TEXT_MSG_MAX_SUBSCRIBERS  4

/**
 * @brief Default BLE OTA configuration from Kconfig
 */
#ifdef CONFIG_CUBE32_BLE_OTA_ENABLED

// Provide defaults for optional Kconfig options
#ifndef CONFIG_CUBE32_BLE_OTA_DOUBLE_BOOT_ENABLE
#define CONFIG_CUBE32_BLE_OTA_DOUBLE_BOOT_ENABLE 0
#endif

#ifndef CONFIG_CUBE32_BLE_OTA_DOUBLE_BOOT_TIME_MS
#define CONFIG_CUBE32_BLE_OTA_DOUBLE_BOOT_TIME_MS 5000
#endif

#define CUBE32_BLE_OTA_CONFIG_DEFAULT() { \
    .device_name = "CUBE32-OTA", \
    .enable_on_double_boot = CONFIG_CUBE32_BLE_OTA_DOUBLE_BOOT_ENABLE, \
    .double_boot_timeout_ms = CONFIG_CUBE32_BLE_OTA_DOUBLE_BOOT_TIME_MS, \
    .adv_interval_min = 0x20, \
    .adv_interval_max = 0x40, \
    .mtu_size = 512, \
}
#else
#define CUBE32_BLE_OTA_CONFIG_DEFAULT() { \
    .device_name = "CUBE32-OTA", \
    .enable_on_double_boot = false, \
    .double_boot_timeout_ms = 5000, \
    .adv_interval_min = 0x20, \
    .adv_interval_max = 0x40, \
    .mtu_size = 512, \
}
#endif

// ============================================================================
// BLE OTA Driver Class
// ============================================================================

/**
 * @brief BLE OTA Driver (Singleton)
 * 
 * Provides BLE-based OTA firmware updates and command/response communication
 * using ESP BLE OTA component with NimBLE stack.
 */
class BleOta {
public:
    /**
     * @brief Get the singleton instance
     */
    static BleOta& instance();

    /**
     * @brief Initialize the BLE OTA driver with default configuration
     * @return CUBE32_OK on success, error code otherwise
     */
    cube32_result_t begin();

    /**
     * @brief Initialize the BLE OTA driver with custom configuration
     * @param config BLE OTA configuration
     * @return CUBE32_OK on success, error code otherwise
     */
    cube32_result_t begin(const BleOtaConfig& config);

    /**
     * @brief Deinitialize the BLE OTA driver
     * @return CUBE32_OK on success, error code otherwise
     */
    cube32_result_t end();

    /**
     * @brief Check if driver is initialized
     */
    bool isInitialized() const { return m_initialized; }

    /**
     * @brief Check if BLE is advertising
     */
    bool isAdvertising() const { return m_status.is_advertising; }

    /**
     * @brief Check if a client is connected
     */
    bool isConnected() const { return m_status.is_connected; }

    /**
     * @brief Check if BLE was bypassed (double-reset not detected)
     */
    bool isBypassed() const { return m_status.state == BleOtaState::BYPASSED; }

    /**
     * @brief Get the full device name (including device ID)
     */
    const char* getDeviceName() const;

    /**
     * @brief Get current status
     */
    const BleOtaStatus& getStatus() const { return m_status; }

    /**
     * @brief Start BLE advertising
     * @return CUBE32_OK on success, error code otherwise
     */
    cube32_result_t startAdvertising();

    /**
     * @brief Stop BLE advertising
     * @return CUBE32_OK on success, error code otherwise
     */
    cube32_result_t stopAdvertising();

    /**
     * @brief Disconnect current client
     * @return CUBE32_OK on success, error code otherwise
     */
    cube32_result_t disconnect();

    /**
     * @brief Send response to BLE client
     * @param cmd_id Command ID being responded to
     * @param status Response status code
     * @param data Response data (can be nullptr)
     * @param len Response data length
     * @return CUBE32_OK on success, error code otherwise
     */
    cube32_result_t sendResponse(uint8_t cmd_id, uint8_t status, 
                                  const uint8_t* data = nullptr, size_t len = 0);

    /**
     * @brief Send text response to BLE client
     * @param text Response text string
     * @return CUBE32_OK on success, error code otherwise
     */
    cube32_result_t sendTextResponse(const char* text);

    /**
     * @brief Register command callback
     * @param callback Function to call when command is received
     */
    void setCommandCallback(BleCommandCallback callback) { m_cmd_callback = callback; }

    /**
     * @brief Register connection state callback
     * @param callback Function to call when connection state changes
     */
    void setConnectionCallback(BleConnectionCallback callback) { m_conn_callback = callback; }

    /**
     * @brief Register OTA progress callback
     * @param callback Function to call when OTA progress updates
     */
    void setOtaProgressCallback(BleOtaProgressCallback callback) { m_ota_callback = callback; }

    /**
     * @brief Subscribe to text messages received from BLE client
     * @param callback Function to call when text message arrives
     * @return Subscription ID (>= 0) on success, -1 if subscriber list full
     */
    int subscribeTextMessage(BleTextMessageCallback callback);

    /**
     * @brief Unsubscribe from text messages
     * @param id Subscription ID returned by subscribeTextMessage()
     */
    void unsubscribeTextMessage(int id);

    /**
     * @brief Check if double reset was detected
     * @return true if double reset was detected
     */
    static bool isDoubleResetDetected();

    /**
     * @brief Record boot time for double reset detection
     */
    static void recordBootTime();

    /**
     * @brief Clear double reset state
     */
    static void clearDoubleResetState();

    // Internal callbacks (called from C code)
    void onConnected(uint16_t conn_handle, const uint8_t* addr);
    void onDisconnected(uint16_t conn_handle, int reason);
    void onMtuChanged(uint16_t conn_handle, uint16_t mtu);
    void onConnectionParamsUpdated(uint16_t interval, uint16_t latency, uint16_t timeout);
    void onDataReceived(uint8_t char_type, const uint8_t* data, size_t len);
    void onOtaProgress(int progress, uint32_t received, uint32_t total);
    void onAdvertisingStarted();  // Called when advertising starts

private:
    BleOta();
    ~BleOta();
    BleOta(const BleOta&) = delete;
    BleOta& operator=(const BleOta&) = delete;

    cube32_result_t initBleStack();
    cube32_result_t initGattServer();
    void processCommand(const uint8_t* data, size_t len);
    void handleGetStatus();
    void handleGetConfig();
    void handleSetConfig(const uint8_t* data, size_t len);
    void handleResetConfig();
    void handleTextMessage(const uint8_t* data, size_t len);
    void handleRestart();
    void handlePowerOff();
    void handleSetDeviceId(const uint8_t* data, size_t len);
    void handleOtaStart(const uint8_t* data, size_t len);
    void handleOtaData(const uint8_t* data, size_t len);
    void handleOtaEnd(const uint8_t* data, size_t len);
    void handleOtaAbort();
    void generateOrLoadDeviceId();
    void buildFullDeviceName();
    void cleanupOta();

    bool m_initialized = false;
    BleOtaConfig m_config;
    BleOtaStatus m_status;
    BleCommandCallback m_cmd_callback = nullptr;
    BleConnectionCallback m_conn_callback = nullptr;
    BleOtaProgressCallback m_ota_callback = nullptr;
    BleTextMessageCallback m_text_subscribers[CUBE32_BLE_TEXT_MSG_MAX_SUBSCRIBERS] = {};
    uint16_t m_mtu = 23;
    
    // OTA state
    void* m_ota_handle = nullptr;       // esp_ota_handle_t
    void* m_update_partition = nullptr; // esp_partition_t*
};

} // namespace cube32

#endif // CUBE32_DRIVERS_BLE_OTA_H
