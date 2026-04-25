/**
 * @file ble_ota.cpp
 * @brief CUBE32 BLE OTA Driver Implementation
 * 
 * Implements BLE-based OTA firmware update and command/response
 * communication using ESP BLE OTA component with NimBLE stack.
 */

#include "drivers/ble/ble_ota.h"

#include <cstdio>
#include <cstring>
#include <cinttypes>

#include <esp_log.h>
#include <esp_system.h>
#include <esp_random.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_app_format.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <time.h>
#include <sys/time.h>
#include "utils/config_manager.h"
#include "utils/hw_manifest.h"

#ifdef CONFIG_CUBE32_PMU_ENABLED
#include "drivers/pmu/axp2101.h"
#endif

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#ifdef CONFIG_CUBE32_BLE_OTA_ENABLED

// NimBLE headers
#include <nimble/nimble_port.h>
#include <nimble/nimble_port_freertos.h>
#include <host/ble_hs.h>
#include <host/util/util.h>
#include <services/gap/ble_svc_gap.h>
#include <services/gatt/ble_svc_gatt.h>

static const char *TAG = "ble_ota";

// ============================================================================
// NVS Keys for Double Reset Detection
// ============================================================================

#define NVS_NAMESPACE           "cube32_ble"
#define NVS_KEY_LAST_BOOT       "last_boot"
#define NVS_KEY_DEVICE_ID       "device_id"

// ============================================================================
// BLE UUIDs
// ============================================================================

// OTA Service UUID (16-bit short form)
#define BLE_OTA_SERVICE_UUID            0x8018

// OTA Characteristics UUIDs
#define BLE_OTA_RECV_FW_UUID            0x8020
#define BLE_OTA_PROGRESS_UUID           0x8021
#define BLE_OTA_COMMAND_UUID            0x8022
#define BLE_OTA_CUSTOMER_UUID           0x8023

// Device Information Service
#define BLE_DIS_SERVICE_UUID            0x180A
#define BLE_DIS_MODEL_UUID              0x2A24
#define BLE_DIS_SERIAL_UUID             0x2A25
#define BLE_DIS_FW_REV_UUID             0x2A26
#define BLE_DIS_HW_REV_UUID             0x2A27
#define BLE_DIS_SW_REV_UUID             0x2A28
#define BLE_DIS_MFG_NAME_UUID           0x2A29

// ============================================================================
// Static Variables
// ============================================================================

static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_customer_char_handle = 0;
static uint16_t s_command_char_handle = 0;
static uint16_t s_progress_char_handle = 0;
static bool s_ota_in_progress = false;
static uint32_t s_ota_total_len = 0;
static uint32_t s_ota_received_len = 0;

// Device info strings
static char s_device_name[CUBE32_BLE_OTA_DEVICE_NAME_MAX] = "CUBE32-OTA";
static char s_full_device_name[CUBE32_BLE_OTA_DEVICE_NAME_MAX + 8] = "CUBE32-OTA";  // Base name + "-" + 6-digit ID
static uint32_t s_device_id = 0;
static const char* s_model_number = "CUBE32";
static const char* s_serial_number = "000001";
static const char* s_firmware_rev = "1.0.0";
static const char* s_hardware_rev = "1.0";
static const char* s_software_rev = "1.0.0";
static const char* s_manufacturer = "CUBE32";

// ============================================================================
// Forward Declarations
// ============================================================================

static int ble_gap_event_handler(struct ble_gap_event *event, void *arg);
static int ble_gatt_handler(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg);
static void ble_on_reset(int reason);
static void ble_on_sync(void);
static void ble_host_task(void *param);
static void start_advertising(void);

// ============================================================================
// UUID Definitions (C++ compatible - pre-defined static structures)
// ============================================================================

// Device Information Service UUIDs
static ble_uuid16_t uuid_dis_svc = BLE_UUID16_INIT(BLE_DIS_SERVICE_UUID);
static ble_uuid16_t uuid_dis_model = BLE_UUID16_INIT(BLE_DIS_MODEL_UUID);
static ble_uuid16_t uuid_dis_serial = BLE_UUID16_INIT(BLE_DIS_SERIAL_UUID);
static ble_uuid16_t uuid_dis_fw_rev = BLE_UUID16_INIT(BLE_DIS_FW_REV_UUID);
static ble_uuid16_t uuid_dis_hw_rev = BLE_UUID16_INIT(BLE_DIS_HW_REV_UUID);
static ble_uuid16_t uuid_dis_sw_rev = BLE_UUID16_INIT(BLE_DIS_SW_REV_UUID);
static ble_uuid16_t uuid_dis_mfg_name = BLE_UUID16_INIT(BLE_DIS_MFG_NAME_UUID);

// OTA Service UUIDs
static ble_uuid16_t uuid_ota_svc = BLE_UUID16_INIT(BLE_OTA_SERVICE_UUID);
static ble_uuid16_t uuid_ota_recv_fw = BLE_UUID16_INIT(BLE_OTA_RECV_FW_UUID);
static ble_uuid16_t uuid_ota_progress = BLE_UUID16_INIT(BLE_OTA_PROGRESS_UUID);
static ble_uuid16_t uuid_ota_command = BLE_UUID16_INIT(BLE_OTA_COMMAND_UUID);
static ble_uuid16_t uuid_ota_customer = BLE_UUID16_INIT(BLE_OTA_CUSTOMER_UUID);

// ============================================================================
// GATT Service Definitions
// ============================================================================

// Device Information Service characteristics
static const struct ble_gatt_chr_def dis_chars[] = {
    {
        .uuid = &uuid_dis_model.u,
        .access_cb = ble_gatt_handler,
        .arg = NULL,
        .descriptors = NULL,
        .flags = BLE_GATT_CHR_F_READ,
        .min_key_size = 0,
        .val_handle = NULL,
        .cpfd = NULL,
    },
    {
        .uuid = &uuid_dis_serial.u,
        .access_cb = ble_gatt_handler,
        .arg = NULL,
        .descriptors = NULL,
        .flags = BLE_GATT_CHR_F_READ,
        .min_key_size = 0,
        .val_handle = NULL,
        .cpfd = NULL,
    },
    {
        .uuid = &uuid_dis_fw_rev.u,
        .access_cb = ble_gatt_handler,
        .arg = NULL,
        .descriptors = NULL,
        .flags = BLE_GATT_CHR_F_READ,
        .min_key_size = 0,
        .val_handle = NULL,
        .cpfd = NULL,
    },
    {
        .uuid = &uuid_dis_hw_rev.u,
        .access_cb = ble_gatt_handler,
        .arg = NULL,
        .descriptors = NULL,
        .flags = BLE_GATT_CHR_F_READ,
        .min_key_size = 0,
        .val_handle = NULL,
        .cpfd = NULL,
    },
    {
        .uuid = &uuid_dis_sw_rev.u,
        .access_cb = ble_gatt_handler,
        .arg = NULL,
        .descriptors = NULL,
        .flags = BLE_GATT_CHR_F_READ,
        .min_key_size = 0,
        .val_handle = NULL,
        .cpfd = NULL,
    },
    {
        .uuid = &uuid_dis_mfg_name.u,
        .access_cb = ble_gatt_handler,
        .arg = NULL,
        .descriptors = NULL,
        .flags = BLE_GATT_CHR_F_READ,
        .min_key_size = 0,
        .val_handle = NULL,
        .cpfd = NULL,
    },
    { .uuid = NULL } // End
};

// OTA Service characteristics
static const struct ble_gatt_chr_def ota_chars[] = {
    {
        // Receive Firmware Characteristic
        .uuid = &uuid_ota_recv_fw.u,
        .access_cb = ble_gatt_handler,
        .arg = NULL,
        .descriptors = NULL,
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_INDICATE,
        .min_key_size = 0,
        .val_handle = NULL,
        .cpfd = NULL,
    },
    {
        // Progress Bar Characteristic
        .uuid = &uuid_ota_progress.u,
        .access_cb = ble_gatt_handler,
        .arg = NULL,
        .descriptors = NULL,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_INDICATE,
        .min_key_size = 0,
        .val_handle = &s_progress_char_handle,
        .cpfd = NULL,
    },
    {
        // Command Characteristic
        .uuid = &uuid_ota_command.u,
        .access_cb = ble_gatt_handler,
        .arg = NULL,
        .descriptors = NULL,
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_INDICATE,
        .min_key_size = 0,
        .val_handle = &s_command_char_handle,
        .cpfd = NULL,
    },
    {
        // Customer Characteristic (for custom commands/responses)
        .uuid = &uuid_ota_customer.u,
        .access_cb = ble_gatt_handler,
        .arg = NULL,
        .descriptors = NULL,
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP | 
                 BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
        .min_key_size = 0,
        .val_handle = &s_customer_char_handle,
        .cpfd = NULL,
    },
    { .uuid = NULL } // End
};

// Service definitions
static const struct ble_gatt_svc_def gatt_services[] = {
    {
        // Device Information Service
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &uuid_dis_svc.u,
        .includes = NULL,
        .characteristics = dis_chars,
    },
    {
        // OTA Service
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &uuid_ota_svc.u,
        .includes = NULL,
        .characteristics = ota_chars,
    },
    { .type = 0 } // End
};

// ============================================================================
// BLE Callbacks
// ============================================================================

static int ble_gatt_handler(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    const ble_uuid_t *uuid = ctxt->chr->uuid;
    uint16_t uuid16 = ble_uuid_u16(uuid);
    
    ESP_LOGD(TAG, "GATT access: op=%d, uuid=0x%04X, attr_handle=%d", 
             ctxt->op, uuid16, attr_handle);
    
    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        // Handle read operations
        switch (uuid16) {
        case BLE_DIS_MODEL_UUID:
            return os_mbuf_append(ctxt->om, s_model_number, strlen(s_model_number));
        case BLE_DIS_SERIAL_UUID:
            return os_mbuf_append(ctxt->om, s_serial_number, strlen(s_serial_number));
        case BLE_DIS_FW_REV_UUID:
            return os_mbuf_append(ctxt->om, s_firmware_rev, strlen(s_firmware_rev));
        case BLE_DIS_HW_REV_UUID:
            return os_mbuf_append(ctxt->om, s_hardware_rev, strlen(s_hardware_rev));
        case BLE_DIS_SW_REV_UUID:
            return os_mbuf_append(ctxt->om, s_software_rev, strlen(s_software_rev));
        case BLE_DIS_MFG_NAME_UUID:
            return os_mbuf_append(ctxt->om, s_manufacturer, strlen(s_manufacturer));
        case BLE_OTA_PROGRESS_UUID:
            {
                // Return OTA progress percentage
                uint8_t progress = (s_ota_total_len > 0) ? 
                    (uint8_t)((s_ota_received_len * 100) / s_ota_total_len) : 0;
                return os_mbuf_append(ctxt->om, &progress, 1);
            }
        default:
            return BLE_ATT_ERR_UNLIKELY;
        }
        break;
        
    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        // Handle write operations
        {
            uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
            uint8_t buf[CUBE32_BLE_OTA_CMD_MAX_LEN];
            
            if (len > sizeof(buf)) {
                ESP_LOGW(TAG, "Write data too long: %d > %zu", len, sizeof(buf));
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }
            
            int rc = ble_hs_mbuf_to_flat(ctxt->om, buf, len, NULL);
            if (rc != 0) {
                return BLE_ATT_ERR_UNLIKELY;
            }
            
            ESP_LOGD(TAG, "Write received on UUID 0x%04X, len=%d", uuid16, len);
            
            switch (uuid16) {
            case BLE_OTA_CUSTOMER_UUID:
                // Custom command - pass to driver
                cube32::BleOta::instance().onDataReceived(0, buf, len);
                break;
            case BLE_OTA_COMMAND_UUID:
                // OTA command
                cube32::BleOta::instance().onDataReceived(1, buf, len);
                break;
            case BLE_OTA_RECV_FW_UUID:
                // OTA firmware data
                cube32::BleOta::instance().onDataReceived(2, buf, len);
                break;
            default:
                return BLE_ATT_ERR_UNLIKELY;
            }
        }
        return 0;
        
    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
    
    return 0;
}

// Request faster connection parameters for high-speed OTA transfer
static void request_fast_connection_params(uint16_t conn_handle)
{
    // Connection interval: 7.5ms - 15ms (min allowed is 7.5ms = 6 * 1.25ms)
    // These fast parameters significantly improve throughput
    struct ble_gap_upd_params params;
    memset(&params, 0, sizeof(params));
    params.itvl_min = 6;    // 7.5ms (6 * 1.25ms) - minimum BLE spec allows
    params.itvl_max = 12;   // 15ms  (12 * 1.25ms)
    params.latency = 0;     // No slave latency - respond to every connection event
    params.supervision_timeout = 200;  // 2 seconds (200 * 10ms)
    params.min_ce_len = 0;
    params.max_ce_len = 0;
    
    int rc = ble_gap_update_params(conn_handle, &params);
    if (rc != 0) {
        ESP_LOGW(TAG, "Failed to request connection param update: rc=%d", rc);
    } else {
        ESP_LOGI(TAG, "Requested fast connection params: interval=7.5-15ms, latency=0");
    }
}

static int ble_gap_event_handler(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "BLE GAP Event: Connect, status=%d", event->connect.status);
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            
            // Get peer address
            struct ble_gap_conn_desc desc;
            ble_gap_conn_find(event->connect.conn_handle, &desc);
            
            // Log current connection parameters
            ESP_LOGI(TAG, "Initial conn params: interval=%d (%.2fms), latency=%d, timeout=%d",
                     desc.conn_itvl, desc.conn_itvl * 1.25f, 
                     desc.conn_latency, desc.supervision_timeout);
            
            // Request faster connection parameters for OTA transfer
            // This is crucial for high-speed data transfer!
            request_fast_connection_params(event->connect.conn_handle);
            
            cube32::BleOta::instance().onConnected(
                event->connect.conn_handle,
                desc.peer_id_addr.val
            );
        } else {
            // Connection failed, restart advertising
            start_advertising();
        }
        break;
        
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "BLE GAP Event: Disconnect, reason=0x%02X", 
                 event->disconnect.reason);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        
        cube32::BleOta::instance().onDisconnected(
            event->disconnect.conn.conn_handle,
            event->disconnect.reason
        );
        
        // Restart advertising after disconnect
        start_advertising();
        break;
        
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "BLE GAP Event: Advertising complete");
        // Restart advertising if needed
        if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
            start_advertising();
        }
        break;
        
    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "BLE GAP Event: MTU update, mtu=%d", event->mtu.value);
        cube32::BleOta::instance().onMtuChanged(
            event->mtu.conn_handle,
            event->mtu.value
        );
        break;
        
    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(TAG, "BLE GAP Event: Subscribe, attr_handle=%d, cur_notify=%d", 
                 event->subscribe.attr_handle, event->subscribe.cur_notify);
        break;
        
    case BLE_GAP_EVENT_CONN_UPDATE:
        // Connection parameters have been updated
        {
            struct ble_gap_conn_desc desc;
            int rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
            if (rc == 0) {
                ESP_LOGI(TAG, "BLE GAP Event: Conn Update - interval=%d (%.2fms), latency=%d, timeout=%d",
                         desc.conn_itvl, desc.conn_itvl * 1.25f,
                         desc.conn_latency, desc.supervision_timeout);
                
                // Notify about connection parameter change
                cube32::BleOta::instance().onConnectionParamsUpdated(
                    desc.conn_itvl, desc.conn_latency, desc.supervision_timeout);
            }
        }
        break;
        
    case BLE_GAP_EVENT_CONN_UPDATE_REQ:
        // Central is requesting connection parameter update - accept fast parameters
        ESP_LOGI(TAG, "BLE GAP Event: Conn Update Request");
        // Return 0 to accept, non-zero to reject
        // We accept all requests from central as they usually know best
        break;
        
    default:
        ESP_LOGD(TAG, "BLE GAP Event: type=%d", event->type);
        break;
    }
    
    return 0;
}

static void ble_on_reset(int reason)
{
    ESP_LOGW(TAG, "BLE Host reset: reason=%d", reason);
}

static void ble_on_sync(void)
{
    ESP_LOGI(TAG, "BLE Host synchronized");
    
    // Make sure we have proper identity address set
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to ensure address: %d", rc);
        return;
    }
    
    // Start advertising
    start_advertising();
}

static void start_advertising(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    struct ble_hs_adv_fields rsp_fields;
    int rc;
    
    memset(&fields, 0, sizeof(fields));
    memset(&rsp_fields, 0, sizeof(rsp_fields));
    
    // Advertising data - include full name (with device ID) for passive scanners
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)s_full_device_name;
    fields.name_len = strlen(s_full_device_name);
    fields.name_is_complete = 1;
    
    ESP_LOGI(TAG, "Setting adv fields: name='%s', len=%d", s_full_device_name, fields.name_len);
    
    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set advertising fields: rc=%d", rc);
        return;
    }
    ESP_LOGI(TAG, "Advertising fields set successfully");
    
    // Scan response data - include service UUID for apps doing active scans
    static ble_uuid16_t adv_uuid = BLE_UUID16_INIT(BLE_OTA_SERVICE_UUID);
    rsp_fields.uuids16 = &adv_uuid;
    rsp_fields.num_uuids16 = 1;
    rsp_fields.uuids16_is_complete = 1;
    rsp_fields.tx_pwr_lvl_is_present = 1;
    rsp_fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    
    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set scan response: rc=%d", rc);
        return;
    }
    ESP_LOGI(TAG, "Scan response fields set successfully");
    
    // Advertising parameters - use longer intervals for better compatibility
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;  // Undirected connectable
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;  // General discoverable
    adv_params.itvl_min = 0x30;  // 30ms (was 20ms)
    adv_params.itvl_max = 0x60;  // 60ms (was 40ms)
    adv_params.channel_map = 0x07;  // Use all 3 advertising channels (37, 38, 39)
    
    // Use public address (0) - the device has a valid MAC
    uint8_t own_addr_type = BLE_OWN_ADDR_PUBLIC;
    
    // Print our BLE address for debugging
    uint8_t addr[6];
    rc = ble_hs_id_copy_addr(own_addr_type, addr, NULL);
    if (rc == 0) {
        ESP_LOGI(TAG, "BLE Address: %02x:%02x:%02x:%02x:%02x:%02x",
                 addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
    }
    
    ESP_LOGI(TAG, "Starting advertising: conn_mode=%d, disc_mode=%d, itvl=%d-%d, chan_map=0x%02x",
             adv_params.conn_mode, adv_params.disc_mode,
             adv_params.itvl_min, adv_params.itvl_max, adv_params.channel_map);
    
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                          &adv_params, ble_gap_event_handler, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to start advertising: rc=%d", rc);
        return;
    }
    
    ESP_LOGI(TAG, "Advertising started successfully as '%s'", s_full_device_name);
    
    // Update status via public method
    cube32::BleOta::instance().onAdvertisingStarted();
}

static void ble_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE Host Task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// ============================================================================
// BleOta Class Implementation
// ============================================================================

namespace cube32 {

BleOta& BleOta::instance()
{
    static BleOta instance;
    return instance;
}

BleOta::BleOta()
{
    memset(&m_status, 0, sizeof(m_status));
}

BleOta::~BleOta()
{
    if (m_initialized) {
        end();
    }
}

cube32_result_t BleOta::begin()
{
    BleOtaConfig config = CUBE32_BLE_OTA_CONFIG_DEFAULT();
    return begin(config);
}

cube32_result_t BleOta::begin(const BleOtaConfig& config)
{
    if (m_initialized) {
        ESP_LOGW(TAG, "BLE OTA already initialized");
        return CUBE32_OK;
    }
    
    ESP_LOGI(TAG, "Initializing BLE OTA driver...");
    
    // Store configuration
    m_config = config;
    strncpy(s_device_name, config.device_name, sizeof(s_device_name) - 1);
    s_device_name[sizeof(s_device_name) - 1] = '\0';
    
    // Generate or load device ID from NVS
    generateOrLoadDeviceId();
    
    // Build full device name with device ID
    buildFullDeviceName();
    
    ESP_LOGI(TAG, "Device ID: %06" PRIu32, s_device_id);
    ESP_LOGI(TAG, "Full device name: %s", s_full_device_name);
    
    // Check double-boot logic if enabled
    // All double-boot detection logic is handled internally here
    if (config.enable_on_double_boot) {
        // Record boot time and check for double boot
        // This is the ONLY place recordBootTime() should be called
        recordBootTime();
        
        // Check if double-boot was detected
        if (!isDoubleResetDetected()) {
            ESP_LOGI(TAG, "Double-boot not detected, BLE OTA bypassed");
            m_initialized = true;
            m_status.is_initialized = true;
            m_status.state = BleOtaState::BYPASSED;
            return CUBE32_OK;
        }
        
        // Double boot detected - clear state and continue to start BLE
        ESP_LOGI(TAG, "Double-boot detected, starting BLE OTA");
        clearDoubleResetState();
    }
    
    // Initialize BLE stack
    cube32_result_t ret = initBleStack();
    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to initialize BLE stack");
        return ret;
    }
    
    // Initialize GATT server
    ret = initGattServer();
    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to initialize GATT server");
        return ret;
    }
    
    m_initialized = true;
    m_status.is_initialized = true;
    m_status.state = BleOtaState::IDLE;
    
    ESP_LOGI(TAG, "BLE OTA driver initialized successfully");
    return CUBE32_OK;
}

cube32_result_t BleOta::initBleStack()
{
    esp_err_t ret;
    
    // Initialize NimBLE port
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NimBLE port: %s", esp_err_to_name(ret));
        return CUBE32_NOT_INITIALIZED;
    }
    
    // Configure NimBLE host
    ble_hs_cfg.reset_cb = ble_on_reset;
    ble_hs_cfg.sync_cb = ble_on_sync;
    ble_hs_cfg.gatts_register_cb = NULL;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    
    return CUBE32_OK;
}

cube32_result_t BleOta::initGattServer()
{
    int rc;
    
    // Initialize GAP and GATT services
    ble_svc_gap_init();
    ble_svc_gatt_init();
    
    // Set device name (must be after ble_svc_gap_init) - use full name with device ID
    int name_rc = ble_svc_gap_device_name_set(s_full_device_name);
    if (name_rc != 0) {
        ESP_LOGW(TAG, "Failed to set GAP device name: %d", name_rc);
    }
    
    // Register custom GATT services
    rc = ble_gatts_count_cfg(gatt_services);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to count GATT config: %d", rc);
        return CUBE32_NOT_INITIALIZED;
    }
    
    rc = ble_gatts_add_svcs(gatt_services);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to add GATT services: %d", rc);
        return CUBE32_NOT_INITIALIZED;
    }
    
    // Start NimBLE host task
    nimble_port_freertos_init(ble_host_task);
    
    ESP_LOGI(TAG, "GATT server initialized");
    return CUBE32_OK;
}

cube32_result_t BleOta::end()
{
    if (!m_initialized) {
        return CUBE32_OK;
    }
    
    ESP_LOGI(TAG, "Deinitializing BLE OTA driver...");
    
    // Stop advertising and disconnect
    if (m_status.is_connected) {
        disconnect();
    }
    
    stopAdvertising();
    
    // Deinitialize NimBLE
    int rc = nimble_port_stop();
    if (rc != 0) {
        ESP_LOGW(TAG, "Failed to stop NimBLE port: %d", rc);
    }
    
    nimble_port_deinit();
    
    m_initialized = false;
    m_status.is_initialized = false;
    m_status.state = BleOtaState::IDLE;
    
    ESP_LOGI(TAG, "BLE OTA driver deinitialized");
    return CUBE32_OK;
}

cube32_result_t BleOta::startAdvertising()
{
    if (!m_initialized) {
        return CUBE32_NOT_INITIALIZED;
    }
    
    if (m_status.is_advertising) {
        return CUBE32_OK;
    }
    
    start_advertising();
    return CUBE32_OK;
}

cube32_result_t BleOta::stopAdvertising()
{
    if (!m_initialized) {
        return CUBE32_NOT_INITIALIZED;
    }
    
    if (!m_status.is_advertising) {
        return CUBE32_OK;
    }
    
    int rc = ble_gap_adv_stop();
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGW(TAG, "Failed to stop advertising: %d", rc);
    }
    
    m_status.is_advertising = false;
    return CUBE32_OK;
}

cube32_result_t BleOta::disconnect()
{
    if (!m_initialized || !m_status.is_connected) {
        return CUBE32_OK;
    }
    
    int rc = ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    if (rc != 0) {
        ESP_LOGW(TAG, "Failed to terminate connection: %d", rc);
        return CUBE32_ERROR;
    }
    
    return CUBE32_OK;
}

cube32_result_t BleOta::sendResponse(uint8_t cmd_id, uint8_t status, 
                                      const uint8_t* data, size_t len)
{
    if (!m_initialized || !m_status.is_connected) {
        return CUBE32_ERROR;
    }
    
    // Build response packet: [cmd_id, status, data...]
    uint8_t resp[CUBE32_BLE_OTA_RESP_MAX_LEN];
    resp[0] = cmd_id;
    resp[1] = status;
    
    size_t resp_len = 2;
    if (data && len > 0 && (len + 2) <= sizeof(resp)) {
        memcpy(&resp[2], data, len);
        resp_len += len;
    }
    
    // Send via notification on customer characteristic
    struct os_mbuf *om = ble_hs_mbuf_from_flat(resp, resp_len);
    if (!om) {
        ESP_LOGE(TAG, "Failed to allocate mbuf");
        return CUBE32_NO_MEM;
    }
    
    int rc = ble_gatts_notify_custom(s_conn_handle, s_customer_char_handle, om);
    if (rc != 0) {
        ESP_LOGW(TAG, "Failed to send notification: %d", rc);
        return CUBE32_IO_ERROR;
    }
    
    m_status.tx_count++;
    snprintf(m_status.last_response, sizeof(m_status.last_response),
             "CMD:0x%02X STS:0x%02X", cmd_id, status);
    
    ESP_LOGD(TAG, "Sent response: cmd=0x%02X, status=0x%02X, len=%zu", 
             cmd_id, status, resp_len);
    
    return CUBE32_OK;
}

cube32_result_t BleOta::sendTextResponse(const char* text)
{
    if (!text) {
        return CUBE32_INVALID_ARG;
    }
    
    return sendResponse(CUBE32_BLE_CMD_TEXT_MESSAGE, CUBE32_BLE_RESP_OK,
                       (const uint8_t*)text, strlen(text));
}

void BleOta::onConnected(uint16_t conn_handle, const uint8_t* addr)
{
    m_status.is_connected = true;
    m_status.is_advertising = false;
    m_status.conn_handle = conn_handle;
    m_status.connection_state = BleConnectionState::CONNECTED;
    m_status.state = BleOtaState::CONNECTED;
    
    if (addr) {
        memcpy(m_status.connected_addr, addr, 6);
    }
    
    ESP_LOGI(TAG, "Client connected: %02X:%02X:%02X:%02X:%02X:%02X",
             addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
    
    if (m_conn_callback) {
        m_conn_callback(BleConnectionState::CONNECTED);
    }
}

void BleOta::onDisconnected(uint16_t conn_handle, int reason)
{
    m_status.is_connected = false;
    m_status.conn_handle = 0xFFFF;
    m_status.connection_state = BleConnectionState::DISCONNECTED;
    m_status.state = BleOtaState::IDLE;
    memset(m_status.connected_addr, 0, 6);
    
    // Reset OTA state
    s_ota_in_progress = false;
    s_ota_total_len = 0;
    s_ota_received_len = 0;
    m_status.ota_progress = 0;
    
    ESP_LOGI(TAG, "Client disconnected: reason=0x%02X", reason);
    
    if (m_conn_callback) {
        m_conn_callback(BleConnectionState::DISCONNECTED);
    }
}

void BleOta::onAdvertisingStarted()
{
    m_status.is_advertising = true;
    m_status.state = BleOtaState::ADVERTISING;
}

void BleOta::onMtuChanged(uint16_t conn_handle, uint16_t mtu)
{
    m_mtu = mtu;
    ESP_LOGI(TAG, "MTU updated: %d", mtu);
}

void BleOta::onConnectionParamsUpdated(uint16_t interval, uint16_t latency, uint16_t timeout)
{
    // Log the actual connection parameters for reference
    // Lower interval = faster transfers (7.5ms is minimum BLE spec allows)
    float interval_ms = interval * 1.25f;
    ESP_LOGI(TAG, "Connection params updated: interval=%.2fms, latency=%d, timeout=%dms",
             interval_ms, latency, timeout * 10);
    
    // Estimate theoretical max throughput based on connection interval
    // Each connection event can transfer multiple packets (up to negotiated MTU)
    // With 7.5ms interval and 512 MTU: ~65 KB/s theoretical max
    if (interval_ms <= 15.0f) {
        ESP_LOGI(TAG, "Fast connection params active - OTA transfer should be optimized");
    }
}

void BleOta::onDataReceived(uint8_t char_type, const uint8_t* data, size_t len)
{
    if (!data || len == 0) {
        return;
    }
    
    m_status.rx_count++;
    
    ESP_LOGD(TAG, "Data received: char_type=%d, len=%zu", char_type, len);
    
    switch (char_type) {
    case 0:  // Customer characteristic - custom commands
        processCommand(data, len);
        break;
    case 1:  // Command characteristic - OTA commands
        // Handle OTA-specific commands
        if (len >= 2 && data[0] == 0x01 && data[1] == 0x00) {
            // Start OTA command
            if (len >= 6) {
                s_ota_total_len = data[2] | (data[3] << 8) | 
                                  (data[4] << 16) | (data[5] << 24);
                s_ota_received_len = 0;
                s_ota_in_progress = true;
                m_status.ota_total_size = s_ota_total_len;
                m_status.state = BleOtaState::OTA_IN_PROGRESS;
                ESP_LOGI(TAG, "OTA started: total_size=%" PRIu32, s_ota_total_len);
            }
        }
        break;
    case 2:  // Firmware data
        if (s_ota_in_progress) {
            s_ota_received_len += len;
            m_status.ota_received_size = s_ota_received_len;
            
            if (s_ota_total_len > 0) {
                m_status.ota_progress = (s_ota_received_len * 100) / s_ota_total_len;
            }
            
            if (m_ota_callback) {
                m_ota_callback(m_status.ota_progress, s_ota_received_len, s_ota_total_len);
            }
            
            ESP_LOGD(TAG, "OTA progress: %d%% (%" PRIu32 "/%" PRIu32 ")",
                     m_status.ota_progress, s_ota_received_len, s_ota_total_len);
        }
        break;
    }
    
    // Call user callback if registered
    if (m_cmd_callback && char_type == 0 && len >= 1) {
        m_cmd_callback(data[0], data + 1, len - 1);
    }
}

void BleOta::onOtaProgress(int progress, uint32_t received, uint32_t total)
{
    m_status.ota_progress = progress;
    m_status.ota_received_size = received;
    m_status.ota_total_size = total;
    
    if (m_ota_callback) {
        m_ota_callback(progress, received, total);
    }
}

void BleOta::processCommand(const uint8_t* data, size_t len)
{
    if (len < 1) {
        return;
    }
    
    uint8_t cmd_id = data[0];
    
    // Update status
    snprintf(m_status.last_command, sizeof(m_status.last_command),
             "CMD:0x%02X LEN:%zu", cmd_id, len);
    
    ESP_LOGI(TAG, "Processing command: 0x%02X, data_len=%zu", cmd_id, len - 1);
    
    switch (cmd_id) {
    case CUBE32_BLE_CMD_GET_STATUS:
        handleGetStatus();
        break;
        
    case CUBE32_BLE_CMD_GET_CONFIG:
        handleGetConfig();
        break;
        
    case CUBE32_BLE_CMD_SET_CONFIG:
        handleSetConfig(data + 1, len - 1);
        break;
        
    case CUBE32_BLE_CMD_RESET_CONFIG:
        handleResetConfig();
        break;
        
    case CUBE32_BLE_CMD_SET_DEVICE_ID:
        handleSetDeviceId(data + 1, len - 1);
        break;
        
    case CUBE32_BLE_CMD_TEXT_MESSAGE:
        handleTextMessage(data + 1, len - 1);
        break;
        
    case CUBE32_BLE_CMD_RESTART:
        handleRestart();
        break;
        
    case CUBE32_BLE_CMD_POWER_OFF:
        handlePowerOff();
        break;
        
    case CUBE32_BLE_CMD_START_OTA:
        handleOtaStart(data + 1, len - 1);
        break;
        
    case CUBE32_BLE_CMD_OTA_DATA:
        handleOtaData(data + 1, len - 1);
        break;
        
    case CUBE32_BLE_CMD_OTA_END:
        handleOtaEnd(data + 1, len - 1);
        break;
        
    case CUBE32_BLE_CMD_OTA_ABORT:
        handleOtaAbort();
        break;
        
    default:
        ESP_LOGW(TAG, "Unknown command: 0x%02X", cmd_id);
        sendResponse(cmd_id, CUBE32_BLE_RESP_INVALID_CMD, nullptr, 0);
        break;
    }
}

void BleOta::handleGetStatus()
{
    // Build status response
    char status_buf[128];
    snprintf(status_buf, sizeof(status_buf),
             "CUBE32 Status: OK\n"
             "BLE: Connected\n"
             "Free Heap: %" PRIu32 "\n"
             "Uptime: %" PRId64 "s",
             esp_get_free_heap_size(),
             esp_timer_get_time() / 1000000);
    
    sendResponse(CUBE32_BLE_CMD_GET_STATUS, CUBE32_BLE_RESP_OK,
                (const uint8_t*)status_buf, strlen(status_buf));
    
    ESP_LOGI(TAG, "Status sent");
}

void BleOta::handleGetConfig()
{
    const cube32_cfg_t* cfg = cube32_cfg();
    const cube32_hw_manifest_t* hw = cube32_hw_manifest();

    // Send config as text key=value lines in pages (each fits 254 bytes)
    // Page 0: Display + Audio + RTC + Camera + nvs_found
    // Page 1: WiFi + Modem + BLE + Active flags
    // Page 2: HW manifest
    // Each page is a separate notification with page number as first payload byte

    auto send_page = [&](uint8_t page_num, const char* text) {
        uint8_t buf[CUBE32_BLE_OTA_RESP_MAX_LEN - 2];
        buf[0] = page_num;
        size_t tlen = strlen(text);
        if (tlen > sizeof(buf) - 1) tlen = sizeof(buf) - 1;
        memcpy(buf + 1, text, tlen);
        sendResponse(CUBE32_BLE_CMD_GET_CONFIG, CUBE32_BLE_RESP_OK, buf, tlen + 1);
    };

    char buf[240];

    // Page 0
    snprintf(buf, sizeof(buf),
        "rot=%u\nprism=%u\nvol=%u\nmaxdb=%u\nin_rate=%" PRIu32 "\nout_rate=%" PRIu32
        "\ntz=%s\nntp=%u\nhmirror=%u\nvflip=%u\nnvs=0x%04" PRIx32,
        cfg->display_rotation, cfg->display_prism ? 1 : 0,
        cfg->audio_volume, cfg->audio_max_db,
        cfg->audio_input_rate, cfg->audio_output_rate,
        cfg->rtc_timezone, cfg->rtc_ntp_sync ? 1 : 0,
        cfg->camera_h_mirror ? 1 : 0, cfg->camera_v_flip ? 1 : 0,
        cfg->nvs_found);
    send_page(0, buf);

    // Page 1
    snprintf(buf, sizeof(buf),
        "ssid=%.32s\napn=%.32s\nble=%.31s\n"
        "act_cam=%u\nact_aud=%u\nact_mod=%u\nact_sd=%u\n"
        "act_usb=%u\nact_ble=%u\nact_adc=%u\nact_srv=%u\n"
        "act_imu=%u\nact_mag=%u",
        cfg->wifi_ssid, cfg->modem_apn, cfg->ble_device_name,
        cfg->active_camera ? 1 : 0, cfg->active_audio ? 1 : 0,
        cfg->active_modem ? 1 : 0, cfg->active_sdcard ? 1 : 0,
        cfg->active_usb_input ? 1 : 0, cfg->active_ble_ota ? 1 : 0,
        cfg->active_adc_button ? 1 : 0, cfg->active_servo ? 1 : 0,
        cfg->active_imu ? 1 : 0, cfg->active_mag ? 1 : 0);
    send_page(1, buf);

    // Page 2: HW manifest
    // Build I2C address list string
    char i2c_str[64] = {0};
    int pos = 0;
    for (int i = 0; i < hw->i2c_device_count && i < 16; i++) {
        if (i > 0) i2c_str[pos++] = ',';
        pos += snprintf(i2c_str + pos, sizeof(i2c_str) - pos, "0x%02x", hw->i2c_devices[i]);
    }

    // Format: component=<present>,<built>[@addr]
    snprintf(buf, sizeof(buf),
        "prof=%s\npmu=%u,%u@0x%02x\nrtc=%u,%u@0x%02x\ndisp=%u,%u\n"
        "touch=%u,%u@0x%02x\naud=%u,%u\nmodem=%u,%u\n"
        "cam=%u,%u\nsd=%u,%u\nimu=%u,%u@0x%02x\nmag=%u,%u@0x%02x\ni2c=%s",
        cube32_profile_name(hw->profile),
        hw->pmu_present ? 1 : 0, hw->pmu_built ? 1 : 0, hw->pmu_i2c_addr,
        hw->rtc_present ? 1 : 0, hw->rtc_built ? 1 : 0, hw->rtc_i2c_addr,
        hw->display_present ? 1 : 0, hw->display_built ? 1 : 0,
        hw->touch_present ? 1 : 0, hw->touch_built ? 1 : 0, hw->touch_i2c_addr,
        hw->audio_module_present ? 1 : 0, hw->audio_built ? 1 : 0,
        hw->modem_module_present ? 1 : 0, hw->modem_built ? 1 : 0,
        hw->camera_present ? 1 : 0, hw->camera_built ? 1 : 0,
        hw->sdcard_present ? 1 : 0, hw->sdcard_built ? 1 : 0,
        hw->imu_present ? 1 : 0, hw->imu_built ? 1 : 0, hw->imu_i2c_addr,
        hw->mag_present ? 1 : 0, hw->mag_built ? 1 : 0, hw->mag_i2c_addr,
        i2c_str);
    send_page(2, buf);

    // Page 3: Build flags for software/optional components
    snprintf(buf, sizeof(buf),
        "ble=%u,%u\nusb=%u,%u\nadc=%u,%u\nservo=%u,%u\nlvgl=%u,%u\n"
        "imu=%u,%u\nmag=%u,%u",
        hw->ble_ota_active ? 1 : 0, hw->ble_ota_built ? 1 : 0,
        hw->usb_input_active ? 1 : 0, hw->usb_input_built ? 1 : 0,
        hw->adc_button_active ? 1 : 0, hw->adc_button_built ? 1 : 0,
        hw->servo_active ? 1 : 0, hw->servo_built ? 1 : 0,
        0, hw->lvgl_built ? 1 : 0,
        hw->imu_active ? 1 : 0, hw->imu_built ? 1 : 0,
        hw->mag_active ? 1 : 0, hw->mag_built ? 1 : 0);
    send_page(3, buf);

    ESP_LOGI(TAG, "Config sent (4 pages)");
}

void BleOta::handleSetConfig(const uint8_t* data, size_t len)
{
    if (len < 2) {
        sendResponse(CUBE32_BLE_CMD_SET_CONFIG, CUBE32_BLE_RESP_ERROR, nullptr, 0);
        return;
    }

    uint8_t key_id = data[0];
    const uint8_t* val = data + 1;
    size_t val_len = len - 1;
    cube32_result_t rc = CUBE32_INVALID_ARG;

    switch (key_id) {
    case CUBE32_BLE_CFG_KEY_ROTATION:
        if (val_len >= 2) {
            uint16_t rot;
            memcpy(&rot, val, 2);  // LE
            rc = cube32_cfg_set_display_rotation(rot);
        }
        break;
    case CUBE32_BLE_CFG_KEY_PRISM:
        if (val_len >= 1) rc = cube32_cfg_set_display_prism(val[0] != 0);
        break;
    case CUBE32_BLE_CFG_KEY_VOLUME:
        if (val_len >= 1) rc = cube32_cfg_set_audio_volume(val[0]);
        break;
    case CUBE32_BLE_CFG_KEY_MAX_DB:
        if (val_len >= 1) rc = cube32_cfg_set_audio_max_db(val[0]);
        break;
    case CUBE32_BLE_CFG_KEY_IN_RATE:
        if (val_len >= 4) {
            uint32_t rate;
            memcpy(&rate, val, 4);
            rc = cube32_cfg_set_audio_input_rate(rate);
        }
        break;
    case CUBE32_BLE_CFG_KEY_OUT_RATE:
        if (val_len >= 4) {
            uint32_t rate;
            memcpy(&rate, val, 4);
            rc = cube32_cfg_set_audio_output_rate(rate);
        }
        break;
    case CUBE32_BLE_CFG_KEY_TIMEZONE:
        if (val_len > 0) {
            char tz[CUBE32_CFG_STR_MAX];
            size_t n = (val_len < sizeof(tz) - 1) ? val_len : sizeof(tz) - 1;
            memcpy(tz, val, n);
            tz[n] = '\0';
            rc = cube32_cfg_set_rtc_timezone(tz);
        }
        break;
    case CUBE32_BLE_CFG_KEY_NTP_SYNC:
        if (val_len >= 1) rc = cube32_cfg_set_rtc_ntp_sync(val[0] != 0);
        break;
    case CUBE32_BLE_CFG_KEY_CAM_HMIRROR:
        if (val_len >= 1) rc = cube32_cfg_set_camera_h_mirror(val[0] != 0);
        break;
    case CUBE32_BLE_CFG_KEY_CAM_VFLIP:
        if (val_len >= 1) rc = cube32_cfg_set_camera_v_flip(val[0] != 0);
        break;
    case CUBE32_BLE_CFG_KEY_WIFI_SSID: {
        // SSID only — password not changed unless followed by WIFI_PASS
        char ssid[CUBE32_CFG_STR_MAX];
        size_t n = (val_len < sizeof(ssid) - 1) ? val_len : sizeof(ssid) - 1;
        memcpy(ssid, val, n);
        ssid[n] = '\0';
        const cube32_cfg_t* cfg = cube32_cfg();
        rc = cube32_cfg_set_wifi(ssid, cfg->wifi_pass);
        break;
    }
    case CUBE32_BLE_CFG_KEY_WIFI_PASS: {
        char pass[CUBE32_CFG_STR_MAX];
        size_t n = (val_len < sizeof(pass) - 1) ? val_len : sizeof(pass) - 1;
        memcpy(pass, val, n);
        pass[n] = '\0';
        const cube32_cfg_t* cfg = cube32_cfg();
        rc = cube32_cfg_set_wifi(cfg->wifi_ssid, pass);
        break;
    }
    case CUBE32_BLE_CFG_KEY_MODEM_APN:
        if (val_len > 0) {
            char apn[CUBE32_CFG_STR_MAX];
            size_t n = (val_len < sizeof(apn) - 1) ? val_len : sizeof(apn) - 1;
            memcpy(apn, val, n);
            apn[n] = '\0';
            rc = cube32_cfg_set_modem_apn(apn);
        }
        break;
    case CUBE32_BLE_CFG_KEY_BLE_NAME:
        if (val_len > 0) {
            char name[32];
            size_t n = (val_len < sizeof(name) - 1) ? val_len : sizeof(name) - 1;
            memcpy(name, val, n);
            name[n] = '\0';
            rc = cube32_cfg_set_ble_device_name(name);
        }
        break;
    case CUBE32_BLE_CFG_KEY_ACT_CAMERA:
        if (val_len >= 1) rc = cube32_cfg_set_active_camera(val[0] != 0);
        break;
    case CUBE32_BLE_CFG_KEY_ACT_AUDIO:
        if (val_len >= 1) rc = cube32_cfg_set_active_audio(val[0] != 0);
        break;
    case CUBE32_BLE_CFG_KEY_ACT_MODEM:
        if (val_len >= 1) rc = cube32_cfg_set_active_modem(val[0] != 0);
        break;
    case CUBE32_BLE_CFG_KEY_ACT_SDCARD:
        if (val_len >= 1) rc = cube32_cfg_set_active_sdcard(val[0] != 0);
        break;
    case CUBE32_BLE_CFG_KEY_ACT_USB_IN:
        if (val_len >= 1) rc = cube32_cfg_set_active_usb_input(val[0] != 0);
        break;
    case CUBE32_BLE_CFG_KEY_ACT_BLE_OTA:
        if (val_len >= 1) rc = cube32_cfg_set_active_ble_ota(val[0] != 0);
        break;
    case CUBE32_BLE_CFG_KEY_ACT_ADC_BTN:
        if (val_len >= 1) rc = cube32_cfg_set_active_adc_button(val[0] != 0);
        break;
    case CUBE32_BLE_CFG_KEY_ACT_SERVO:
        if (val_len >= 1) rc = cube32_cfg_set_active_servo(val[0] != 0);
        break;
    case CUBE32_BLE_CFG_KEY_ACT_IMU:
        if (val_len >= 1) rc = cube32_cfg_set_active_imu(val[0] != 0);
        break;
    case CUBE32_BLE_CFG_KEY_ACT_MAG:
        if (val_len >= 1) rc = cube32_cfg_set_active_mag(val[0] != 0);
        break;
    default:
        ESP_LOGW(TAG, "Unknown config key: 0x%02X", key_id);
        break;
    }

    uint8_t resp_status = (rc == CUBE32_OK) ? CUBE32_BLE_RESP_OK : CUBE32_BLE_RESP_ERROR;
    sendResponse(CUBE32_BLE_CMD_SET_CONFIG, resp_status, &key_id, 1);
    ESP_LOGI(TAG, "SetConfig key=0x%02X result=%d", key_id, rc);
}

void BleOta::handleResetConfig()
{
    cube32_result_t rc = cube32_cfg_reset();
    uint8_t resp_status = (rc == CUBE32_OK) ? CUBE32_BLE_RESP_OK : CUBE32_BLE_RESP_ERROR;
    sendResponse(CUBE32_BLE_CMD_RESET_CONFIG, resp_status, nullptr, 0);
    ESP_LOGI(TAG, "ResetConfig result=%d", rc);
}

void BleOta::handleTextMessage(const uint8_t* data, size_t len)
{
    if (len > 0) {
        // Use a buffer that will definitely fit in response with prefix
        constexpr size_t prefix_len = 10;  // "Received: " 
        constexpr size_t max_msg_len = CUBE32_BLE_OTA_RESP_MAX_LEN - prefix_len - 1;
        char msg[max_msg_len + 1];
        size_t copy_len = (len < max_msg_len) ? len : max_msg_len;
        memcpy(msg, data, copy_len);
        msg[copy_len] = '\0';
        
        ESP_LOGI(TAG, "Text message received: %s", msg);
        
        // Echo back
        char response[CUBE32_BLE_OTA_RESP_MAX_LEN];
        snprintf(response, sizeof(response), "Received: %s", msg);
        sendResponse(CUBE32_BLE_CMD_TEXT_MESSAGE, CUBE32_BLE_RESP_OK,
                    (const uint8_t*)response, strlen(response));

        // Dispatch to text message subscribers
        for (int i = 0; i < CUBE32_BLE_TEXT_MSG_MAX_SUBSCRIBERS; i++) {
            if (m_text_subscribers[i]) {
                m_text_subscribers[i](msg, copy_len);
            }
        }
    }
}

int BleOta::subscribeTextMessage(BleTextMessageCallback callback)
{
    for (int i = 0; i < CUBE32_BLE_TEXT_MSG_MAX_SUBSCRIBERS; i++) {
        if (!m_text_subscribers[i]) {
            m_text_subscribers[i] = callback;
            ESP_LOGI(TAG, "Text message subscriber %d registered", i);
            return i;
        }
    }
    ESP_LOGW(TAG, "Text message subscriber list full");
    return -1;
}

void BleOta::unsubscribeTextMessage(int id)
{
    if (id >= 0 && id < CUBE32_BLE_TEXT_MSG_MAX_SUBSCRIBERS) {
        m_text_subscribers[id] = nullptr;
        ESP_LOGI(TAG, "Text message subscriber %d removed", id);
    }
}

void BleOta::handleRestart()
{
    ESP_LOGI(TAG, "System restart requested");
    
    // Send acknowledgment
    sendResponse(CUBE32_BLE_CMD_RESTART, CUBE32_BLE_RESP_OK, nullptr, 0);
    
    // Wait for response to be sent
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Restart system
    esp_restart();
}

void BleOta::handlePowerOff()
{
    ESP_LOGI(TAG, "System power off requested");
    
    // Send acknowledgment
    sendResponse(CUBE32_BLE_CMD_POWER_OFF, CUBE32_BLE_RESP_OK, nullptr, 0);
    
    // Wait for response to be sent
    vTaskDelay(pdMS_TO_TICKS(500));
    
#ifdef CONFIG_CUBE32_PMU_ENABLED
    // Power off via PMU singleton
    PMU::instance().shutdown();
#else
    // If PMU not available, enter deep sleep instead
    ESP_LOGW(TAG, "PMU not enabled, entering deep sleep");
    esp_deep_sleep_start();
#endif
}

void BleOta::handleSetDeviceId(const uint8_t* data, size_t len)
{
    ESP_LOGI(TAG, "Set Device ID requested");
    
    if (len < 4) {
        ESP_LOGW(TAG, "Invalid device ID length: %zu (need 4 bytes)", len);
        sendResponse(CUBE32_BLE_CMD_SET_DEVICE_ID, CUBE32_BLE_RESP_ERROR, nullptr, 0);
        return;
    }
    
    // Parse device ID (4 bytes, little endian)
    uint32_t new_id = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
    
    // Limit to 6 digits (0-999999)
    new_id = new_id % 1000000;
    
    // Store in NVS
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
        sendResponse(CUBE32_BLE_CMD_SET_DEVICE_ID, CUBE32_BLE_RESP_ERROR, nullptr, 0);
        return;
    }
    
    err = nvs_set_u32(handle, NVS_KEY_DEVICE_ID, new_id);
    if (err == ESP_OK) {
        nvs_commit(handle);
        s_device_id = new_id;
        buildFullDeviceName();
        ESP_LOGI(TAG, "Device ID set to: %06" PRIu32, s_device_id);
        ESP_LOGI(TAG, "New device name: %s", s_full_device_name);
        
        // Return the new device ID
        uint8_t resp[4];
        resp[0] = s_device_id & 0xFF;
        resp[1] = (s_device_id >> 8) & 0xFF;
        resp[2] = (s_device_id >> 16) & 0xFF;
        resp[3] = (s_device_id >> 24) & 0xFF;
        sendResponse(CUBE32_BLE_CMD_SET_DEVICE_ID, CUBE32_BLE_RESP_OK, resp, 4);
    } else {
        ESP_LOGE(TAG, "Failed to set device ID: %s", esp_err_to_name(err));
        sendResponse(CUBE32_BLE_CMD_SET_DEVICE_ID, CUBE32_BLE_RESP_ERROR, nullptr, 0);
    }
    
    nvs_close(handle);
}

void BleOta::generateOrLoadDeviceId()
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to open NVS for device ID: %s", esp_err_to_name(err));
        // Generate random ID anyway
        s_device_id = esp_random() % 1000000;
        return;
    }
    
    // Try to load existing device ID
    err = nvs_get_u32(handle, NVS_KEY_DEVICE_ID, &s_device_id);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        // Generate new random ID (6 digits: 0-999999)
        s_device_id = esp_random() % 1000000;
        
        // Store it
        err = nvs_set_u32(handle, NVS_KEY_DEVICE_ID, s_device_id);
        if (err == ESP_OK) {
            nvs_commit(handle);
            ESP_LOGI(TAG, "Generated new device ID: %06" PRIu32, s_device_id);
        } else {
            ESP_LOGW(TAG, "Failed to store device ID: %s", esp_err_to_name(err));
        }
    } else if (err == ESP_OK) {
        ESP_LOGI(TAG, "Loaded device ID: %06" PRIu32, s_device_id);
    } else {
        ESP_LOGW(TAG, "Failed to read device ID: %s", esp_err_to_name(err));
        s_device_id = esp_random() % 1000000;
    }
    
    nvs_close(handle);
}

void BleOta::buildFullDeviceName()
{
    // Build full device name: base_name + "-" + ID (no padding)
    snprintf(s_full_device_name, sizeof(s_full_device_name), 
             "%s-%" PRIu32, s_device_name, s_device_id);
}

const char* BleOta::getDeviceName() const
{
    return s_full_device_name;
}

// ============================================================================
// Double Reset Detection - Simple Sequential Logic
// ============================================================================

// Static variable to store double reset detection result
static bool s_double_reset_detected = false;

// Helper function to format time as string
static void format_time_str(time_t t, char* buf, size_t buf_size)
{
    struct tm timeinfo;
    localtime_r(&t, &timeinfo);
    strftime(buf, buf_size, "%Y-%m-%d %H:%M:%S", &timeinfo);
}

bool BleOta::isDoubleResetDetected()
{
    return s_double_reset_detected;
}

void BleOta::recordBootTime()
{
    // Simple sequential logic:
    // 1. Get current system time
    // 2. Load last boot time from NVS
    // 3. Compare: if gap <= timeout AND last_boot is valid -> double reset detected
    // 4. Save current time to NVS (always)
    
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
        s_double_reset_detected = false;
        return;
    }
    
    // Step 1: Get current system time (from RTC)
    time_t current_time = time(NULL);
    
    // Step 2: Get last boot time from NVS
    int64_t last_boot_i64 = 0;
    err = nvs_get_i64(handle, NVS_KEY_LAST_BOOT, &last_boot_i64);
    time_t last_boot = (time_t)last_boot_i64;
    
    // Format times for logging
    char current_time_str[32];
    char last_boot_str[32];
    format_time_str(current_time, current_time_str, sizeof(current_time_str));
    format_time_str(last_boot, last_boot_str, sizeof(last_boot_str));
    
    ESP_LOGI(TAG, "Current time: %s (epoch: %lld)", current_time_str, (long long)current_time);
    ESP_LOGI(TAG, "Last boot:    %s (epoch: %lld)", last_boot_str, (long long)last_boot);
    
    // Get timeout value in seconds
#ifdef CONFIG_CUBE32_BLE_OTA_DOUBLE_BOOT_TIME_MS
    int64_t timeout_sec = CONFIG_CUBE32_BLE_OTA_DOUBLE_BOOT_TIME_MS / 1000;
#else
    int64_t timeout_sec = 5;
#endif
    
    // Step 3: Check if this is a double reset
    // Conditions:
    // - last_boot must be valid (after year 2020: epoch > 1577836800)
    // - current_time must be valid (after year 2020)
    // - time_gap must be positive and within timeout
    bool last_boot_valid = (last_boot > 1577836800);  // After 2020-01-01
    bool current_time_valid = (current_time > 1577836800);  // After 2020-01-01
    int64_t time_gap_sec = (int64_t)current_time - (int64_t)last_boot;
    
    ESP_LOGI(TAG, "Time gap: %lld sec, Timeout: %lld sec", (long long)time_gap_sec, (long long)timeout_sec);
    ESP_LOGI(TAG, "Last boot valid: %s, Current time valid: %s", 
             last_boot_valid ? "yes" : "no", current_time_valid ? "yes" : "no");
    
    if (last_boot_valid && current_time_valid && time_gap_sec >= 0 && time_gap_sec <= timeout_sec) {
        // Double boot detected!
        s_double_reset_detected = true;
        ESP_LOGI(TAG, "*** DOUBLE BOOT DETECTED! ***");
    } else {
        // Normal boot
        s_double_reset_detected = false;
        if (!last_boot_valid) {
            ESP_LOGI(TAG, "First boot or RTC was not set previously");
        } else if (!current_time_valid) {
            ESP_LOGI(TAG, "RTC not synchronized yet (time before 2020)");
        } else {
            ESP_LOGI(TAG, "Normal boot (gap %lld sec > timeout %lld sec)", 
                     (long long)time_gap_sec, (long long)timeout_sec);
        }
    }
    
    // Step 4: Always save current time as the new boot time for next comparison
    err = nvs_set_i64(handle, NVS_KEY_LAST_BOOT, (int64_t)current_time);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save boot time: %s", esp_err_to_name(err));
    }
    
    err = nvs_commit(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(err));
    }
    
    nvs_close(handle);
    ESP_LOGI(TAG, "Boot time saved to NVS");
}

void BleOta::clearDoubleResetState()
{
    // Simply reset the static flag
    s_double_reset_detected = false;
    ESP_LOGI(TAG, "Double reset state cleared");
}

// ============================================================================
// OTA Firmware Update Handlers
// ============================================================================

void BleOta::handleOtaStart(const uint8_t* data, size_t len)
{
    ESP_LOGI(TAG, "OTA Start command received, data_len=%zu", len);
    
    // Check if OTA is already in progress
    if (m_status.state == BleOtaState::OTA_IN_PROGRESS) {
        ESP_LOGW(TAG, "OTA already in progress");
        sendResponse(CUBE32_BLE_CMD_START_OTA, CUBE32_BLE_RESP_BUSY, nullptr, 0);
        return;
    }
    
    // Parse OTA start packet:
    // Bytes 0-3: Total firmware size (uint32_t, little-endian)
    // Bytes 4-7: Checksum (optional, uint32_t, little-endian)
    // Bytes 8+: Version string (null-terminated, optional)
    // Following null: Project name (null-terminated, optional)
    // Following null: Build date (null-terminated, optional)
    // Following null: Build time (null-terminated, optional)
    
    if (len < 4) {
        ESP_LOGE(TAG, "OTA start packet too short");
        snprintf(m_status.ota_error_msg, sizeof(m_status.ota_error_msg), "Invalid start packet");
        sendResponse(CUBE32_BLE_CMD_START_OTA, CUBE32_BLE_RESP_ERROR, nullptr, 0);
        return;
    }
    
    // Parse firmware size
    uint32_t fw_size = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
    
    // Reset firmware info
    memset(&m_status.firmware_info, 0, sizeof(m_status.firmware_info));
    m_status.firmware_info.total_size = fw_size;
    
    // Parse optional checksum
    if (len >= 8) {
        m_status.firmware_info.checksum = data[4] | (data[5] << 8) | (data[6] << 16) | (data[7] << 24);
    }
    
    // Parse optional strings (version, project, date, time, app_full_name)
    size_t offset = 8;
    const char* strings[5] = {nullptr, nullptr, nullptr, nullptr, nullptr};
    char* dest[5] = {
        m_status.firmware_info.version,
        m_status.firmware_info.project_name,
        m_status.firmware_info.build_date,
        m_status.firmware_info.build_time,
        m_status.firmware_info.app_full_name
    };
    size_t dest_size[5] = {
        sizeof(m_status.firmware_info.version),
        sizeof(m_status.firmware_info.project_name),
        sizeof(m_status.firmware_info.build_date),
        sizeof(m_status.firmware_info.build_time),
        sizeof(m_status.firmware_info.app_full_name)
    };
    
    for (int i = 0; i < 5 && offset < len; i++) {
        strings[i] = (const char*)(data + offset);
        size_t str_len = strnlen(strings[i], len - offset);
        if (str_len < dest_size[i]) {
            strncpy(dest[i], strings[i], dest_size[i] - 1);
            dest[i][dest_size[i] - 1] = '\0';
        }
        offset += str_len + 1;  // Skip past null terminator
    }
    
    // Backward compatibility: if app_full_name not sent, use project_name
    if (strlen(m_status.firmware_info.app_full_name) == 0 &&
        strlen(m_status.firmware_info.project_name) > 0) {
        strncpy(m_status.firmware_info.app_full_name,
                m_status.firmware_info.project_name,
                sizeof(m_status.firmware_info.app_full_name) - 1);
    }
    
    ESP_LOGI(TAG, "OTA Firmware Info:");
    ESP_LOGI(TAG, "  Size: %" PRIu32 " bytes", fw_size);
    ESP_LOGI(TAG, "  Version: %s", m_status.firmware_info.version);
    ESP_LOGI(TAG, "  Project: %s", m_status.firmware_info.project_name);
    ESP_LOGI(TAG, "  App Name: %s", m_status.firmware_info.app_full_name);
    ESP_LOGI(TAG, "  Build Date: %s", m_status.firmware_info.build_date);
    ESP_LOGI(TAG, "  Build Time: %s", m_status.firmware_info.build_time);
    
    // Validate firmware size
    if (fw_size == 0 || fw_size > 4 * 1024 * 1024) {  // Max 4MB
        ESP_LOGE(TAG, "Invalid firmware size: %" PRIu32, fw_size);
        snprintf(m_status.ota_error_msg, sizeof(m_status.ota_error_msg), "Invalid size: %" PRIu32, fw_size);
        sendResponse(CUBE32_BLE_CMD_START_OTA, CUBE32_BLE_RESP_ERROR, nullptr, 0);
        return;
    }
    
    // Find OTA partition
    const esp_partition_t* update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition == NULL) {
        ESP_LOGE(TAG, "No OTA partition found");
        snprintf(m_status.ota_error_msg, sizeof(m_status.ota_error_msg), "No OTA partition");
        sendResponse(CUBE32_BLE_CMD_START_OTA, CUBE32_BLE_RESP_ERROR, nullptr, 0);
        return;
    }
    
    ESP_LOGI(TAG, "OTA partition: %s, offset=0x%08" PRIx32 ", size=%" PRIu32,
             update_partition->label, update_partition->address, update_partition->size);
    
    // Check if firmware fits
    if (fw_size > update_partition->size) {
        ESP_LOGE(TAG, "Firmware too large for partition");
        snprintf(m_status.ota_error_msg, sizeof(m_status.ota_error_msg), "FW too large");
        sendResponse(CUBE32_BLE_CMD_START_OTA, CUBE32_BLE_RESP_ERROR, nullptr, 0);
        return;
    }
    
    // Begin OTA
    esp_ota_handle_t ota_handle;
    esp_err_t err = esp_ota_begin(update_partition, fw_size, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        snprintf(m_status.ota_error_msg, sizeof(m_status.ota_error_msg), "OTA begin failed");
        sendResponse(CUBE32_BLE_CMD_START_OTA, CUBE32_BLE_RESP_ERROR, nullptr, 0);
        return;
    }
    
    // Store OTA handles
    m_ota_handle = (void*)(uintptr_t)ota_handle;
    m_update_partition = (void*)update_partition;
    
    // Update status
    m_status.state = BleOtaState::OTA_IN_PROGRESS;
    m_status.ota_total_size = fw_size;
    m_status.ota_received_size = 0;
    m_status.ota_progress = 0;
    m_status.ota_ready_to_restart = false;
    m_status.firmware_info.is_valid = true;
    memset(m_status.ota_error_msg, 0, sizeof(m_status.ota_error_msg));
    
    ESP_LOGI(TAG, "OTA started successfully");
    
    // Send response with partition info
    uint8_t resp[8];
    resp[0] = (update_partition->size >> 0) & 0xFF;
    resp[1] = (update_partition->size >> 8) & 0xFF;
    resp[2] = (update_partition->size >> 16) & 0xFF;
    resp[3] = (update_partition->size >> 24) & 0xFF;
    resp[4] = (m_mtu >> 0) & 0xFF;
    resp[5] = (m_mtu >> 8) & 0xFF;
    resp[6] = 0;  // Reserved
    resp[7] = 0;  // Reserved
    
    sendResponse(CUBE32_BLE_CMD_START_OTA, CUBE32_BLE_RESP_OTA_READY, resp, sizeof(resp));
    
    // Notify callback
    if (m_ota_callback) {
        m_ota_callback(0, 0, fw_size);
    }
}

void BleOta::handleOtaData(const uint8_t* data, size_t len)
{
    if (m_status.state != BleOtaState::OTA_IN_PROGRESS) {
        ESP_LOGW(TAG, "OTA data received but OTA not in progress");
        sendResponse(CUBE32_BLE_CMD_OTA_DATA, CUBE32_BLE_RESP_ERROR, nullptr, 0);
        return;
    }
    
    if (len == 0 || data == nullptr) {
        ESP_LOGW(TAG, "Empty OTA data packet");
        return;
    }
    
    // Write data to OTA partition
    esp_ota_handle_t ota_handle = (esp_ota_handle_t)(uintptr_t)m_ota_handle;
    esp_err_t err = esp_ota_write(ota_handle, data, len);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
        snprintf(m_status.ota_error_msg, sizeof(m_status.ota_error_msg), "Write failed");
        cleanupOta();
        m_status.state = BleOtaState::ERROR;
        sendResponse(CUBE32_BLE_CMD_OTA_DATA, CUBE32_BLE_RESP_OTA_ERROR, nullptr, 0);
        return;
    }
    
    // Update progress
    m_status.ota_received_size += len;
    m_status.firmware_info.received_size = m_status.ota_received_size;
    
    if (m_status.ota_total_size > 0) {
        m_status.ota_progress = (m_status.ota_received_size * 100) / m_status.ota_total_size;
    }
    
    // Log progress periodically (every 10%)
    static int last_logged_progress = -1;
    int current_progress_10 = m_status.ota_progress / 10;
    if (current_progress_10 != last_logged_progress) {
        last_logged_progress = current_progress_10;
        ESP_LOGI(TAG, "OTA progress: %d%% (%" PRIu32 "/%" PRIu32 " bytes)",
                 m_status.ota_progress, m_status.ota_received_size, m_status.ota_total_size);
    }
    
    // Notify callback
    if (m_ota_callback) {
        m_ota_callback(m_status.ota_progress, m_status.ota_received_size, m_status.ota_total_size);
    }
    
    // Send progress response every 5%
    static int last_sent_progress = -1;
    int current_progress_5 = m_status.ota_progress / 5;
    if (current_progress_5 != last_sent_progress) {
        last_sent_progress = current_progress_5;
        uint8_t resp[8];
        resp[0] = m_status.ota_progress;
        resp[1] = (m_status.ota_received_size >> 0) & 0xFF;
        resp[2] = (m_status.ota_received_size >> 8) & 0xFF;
        resp[3] = (m_status.ota_received_size >> 16) & 0xFF;
        resp[4] = (m_status.ota_received_size >> 24) & 0xFF;
        resp[5] = 0;
        resp[6] = 0;
        resp[7] = 0;
        sendResponse(CUBE32_BLE_CMD_OTA_DATA, CUBE32_BLE_RESP_OTA_PROGRESS, resp, sizeof(resp));
    }
}

void BleOta::handleOtaEnd(const uint8_t* data, size_t len)
{
    ESP_LOGI(TAG, "OTA End command received");
    
    if (m_status.state != BleOtaState::OTA_IN_PROGRESS) {
        ESP_LOGW(TAG, "OTA end received but OTA not in progress");
        sendResponse(CUBE32_BLE_CMD_OTA_END, CUBE32_BLE_RESP_ERROR, nullptr, 0);
        return;
    }
    
    // Verify we received all data
    if (m_status.ota_received_size != m_status.ota_total_size) {
        ESP_LOGE(TAG, "OTA size mismatch: received %" PRIu32 ", expected %" PRIu32,
                 m_status.ota_received_size, m_status.ota_total_size);
        snprintf(m_status.ota_error_msg, sizeof(m_status.ota_error_msg), 
                 "Size mismatch: %" PRIu32 "/%" PRIu32, m_status.ota_received_size, m_status.ota_total_size);
        cleanupOta();
        m_status.state = BleOtaState::ERROR;
        sendResponse(CUBE32_BLE_CMD_OTA_END, CUBE32_BLE_RESP_OTA_ERROR, nullptr, 0);
        return;
    }
    
    // End OTA and verify
    esp_ota_handle_t ota_handle = (esp_ota_handle_t)(uintptr_t)m_ota_handle;
    esp_err_t err = esp_ota_end(ota_handle);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
            snprintf(m_status.ota_error_msg, sizeof(m_status.ota_error_msg), "Validation failed");
        } else {
            snprintf(m_status.ota_error_msg, sizeof(m_status.ota_error_msg), "OTA end failed");
        }
        m_ota_handle = nullptr;
        m_status.state = BleOtaState::ERROR;
        sendResponse(CUBE32_BLE_CMD_OTA_END, CUBE32_BLE_RESP_OTA_ERROR, nullptr, 0);
        return;
    }
    
    m_ota_handle = nullptr;
    
    // Set boot partition
    const esp_partition_t* update_partition = (const esp_partition_t*)m_update_partition;
    err = esp_ota_set_boot_partition(update_partition);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        snprintf(m_status.ota_error_msg, sizeof(m_status.ota_error_msg), "Set boot failed");
        m_status.state = BleOtaState::ERROR;
        sendResponse(CUBE32_BLE_CMD_OTA_END, CUBE32_BLE_RESP_OTA_ERROR, nullptr, 0);
        return;
    }
    
    // Log partition info for debugging
    {
        const esp_partition_t *boot_part = esp_ota_get_boot_partition();
        const esp_partition_t *running_part = esp_ota_get_running_partition();
        ESP_LOGI(TAG, "OTA boot partition set: %s (offset=0x%08" PRIx32 ")",
                 update_partition->label, (uint32_t)update_partition->address);
        ESP_LOGI(TAG, "  Current running: %s, Next boot: %s",
                 running_part ? running_part->label : "unknown",
                 boot_part ? boot_part->label : "unknown");
    }
    
    m_update_partition = nullptr;
    
    // Success!
    m_status.state = BleOtaState::OTA_COMPLETE;
    m_status.ota_progress = 100;
    m_status.ota_ready_to_restart = true;
    
    ESP_LOGI(TAG, "OTA completed successfully! Ready to restart.");
    
    // Send success response
    sendResponse(CUBE32_BLE_CMD_OTA_END, CUBE32_BLE_RESP_OTA_COMPLETE, nullptr, 0);
    
    // Notify callback
    if (m_ota_callback) {
        m_ota_callback(100, m_status.ota_received_size, m_status.ota_total_size);
    }
}

void BleOta::handleOtaAbort()
{
    ESP_LOGI(TAG, "OTA Abort command received");
    
    if (m_status.state != BleOtaState::OTA_IN_PROGRESS) {
        ESP_LOGW(TAG, "OTA abort received but OTA not in progress");
        sendResponse(CUBE32_BLE_CMD_OTA_ABORT, CUBE32_BLE_RESP_OK, nullptr, 0);
        return;
    }
    
    cleanupOta();
    
    m_status.state = BleOtaState::CONNECTED;
    m_status.ota_progress = 0;
    m_status.ota_received_size = 0;
    m_status.ota_total_size = 0;
    m_status.ota_ready_to_restart = false;
    memset(&m_status.firmware_info, 0, sizeof(m_status.firmware_info));
    snprintf(m_status.ota_error_msg, sizeof(m_status.ota_error_msg), "Aborted by user");
    
    ESP_LOGI(TAG, "OTA aborted");
    sendResponse(CUBE32_BLE_CMD_OTA_ABORT, CUBE32_BLE_RESP_OK, nullptr, 0);
}

void BleOta::cleanupOta()
{
    if (m_ota_handle != nullptr) {
        esp_ota_handle_t ota_handle = (esp_ota_handle_t)(uintptr_t)m_ota_handle;
        esp_ota_abort(ota_handle);
        m_ota_handle = nullptr;
    }
    m_update_partition = nullptr;
}

} // namespace cube32

#else // CONFIG_CUBE32_BLE_OTA_ENABLED not defined

// Stub implementation when BLE OTA is disabled
namespace cube32 {

BleOta& BleOta::instance()
{
    static BleOta instance;
    return instance;
}

BleOta::BleOta() {}
BleOta::~BleOta() {}

cube32_result_t BleOta::begin() { return CUBE32_NOT_SUPPORTED; }
cube32_result_t BleOta::begin(const BleOtaConfig& config) { return CUBE32_NOT_SUPPORTED; }
cube32_result_t BleOta::end() { return CUBE32_OK; }
cube32_result_t BleOta::startAdvertising() { return CUBE32_NOT_SUPPORTED; }
cube32_result_t BleOta::stopAdvertising() { return CUBE32_NOT_SUPPORTED; }
cube32_result_t BleOta::disconnect() { return CUBE32_NOT_SUPPORTED; }
cube32_result_t BleOta::sendResponse(uint8_t, uint8_t, const uint8_t*, size_t) { return CUBE32_NOT_SUPPORTED; }
cube32_result_t BleOta::sendTextResponse(const char*) { return CUBE32_NOT_SUPPORTED; }
void BleOta::onConnected(uint16_t, const uint8_t*) {}
void BleOta::onDisconnected(uint16_t, int) {}
void BleOta::onAdvertisingStarted() {}
void BleOta::onMtuChanged(uint16_t, uint16_t) {}
void BleOta::onConnectionParamsUpdated(uint16_t, uint16_t, uint16_t) {}
void BleOta::onDataReceived(uint8_t, const uint8_t*, size_t) {}
void BleOta::onOtaProgress(int, uint32_t, uint32_t) {}
bool BleOta::isDoubleResetDetected() { return false; }
void BleOta::recordBootTime() {}
void BleOta::clearDoubleResetState() {}
void BleOta::processCommand(const uint8_t*, size_t) {}
void BleOta::handleGetStatus() {}
void BleOta::handleGetConfig() {}
void BleOta::handleSetConfig(const uint8_t*, size_t) {}
void BleOta::handleResetConfig() {}
void BleOta::handleTextMessage(const uint8_t*, size_t) {}
int BleOta::subscribeTextMessage(BleTextMessageCallback) { return -1; }
void BleOta::unsubscribeTextMessage(int) {}
void BleOta::handleRestart() {}
void BleOta::handlePowerOff() {}
void BleOta::handleSetDeviceId(const uint8_t*, size_t) {}
void BleOta::handleOtaStart(const uint8_t*, size_t) {}
void BleOta::handleOtaData(const uint8_t*, size_t) {}
void BleOta::handleOtaEnd(const uint8_t*, size_t) {}
void BleOta::handleOtaAbort() {}
void BleOta::cleanupOta() {}
cube32_result_t BleOta::initBleStack() { return CUBE32_NOT_SUPPORTED; }
cube32_result_t BleOta::initGattServer() { return CUBE32_NOT_SUPPORTED; }
void BleOta::generateOrLoadDeviceId() {}
void BleOta::buildFullDeviceName() {}
const char* BleOta::getDeviceName() const { return ""; }

} // namespace cube32

#endif // CONFIG_CUBE32_BLE_OTA_ENABLED
