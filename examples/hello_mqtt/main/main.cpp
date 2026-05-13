/**
 * @file main.cpp
 * @brief CUBE32 Hello MQTT Example
 * 
 * This example demonstrates:
 * - MQTT client with AWS IoT support
 * - Automatic internet connection via WiFi or LTE Modem
 * - SSL/TLS support with CA certificates from SD Card
 * - LVGL 9.x dark theme UI displaying:
 *   - Connection mode (WiFi/Modem) and status
 *   - Reconnect time countdown
 *   - Signal/WiFi strength
 *   - Subscription topic and received messages
 *   - Publishing topic and status
 *   - Button to publish system status JSON
 * 
 * Internet Connection:
 * - If CONFIG_CUBE32_MODEM_ENABLED: Uses LTE modem (A7670) via PPP
 * - Otherwise: Uses WiFi connection from CUBE32 WiFi Configuration
 * 
 * MQTT Configuration (defined in program):
 * - Host URL and Port
 * - Username and Password (optional)
 * - With or without SSL/TLS
 * - CA files on SD Card: CA File, Client Cert File, Client Key File
 * - Connect Timeout, Keep Alive, Auto Reconnect, Reconnect Period
 * - Subscription and Publishing topic names
 * 
 * Prerequisites:
 * - Enable LVGL in menuconfig: CUBE32 Board Configuration → Display Configuration → Enable LVGL
 * - Configure WiFi in menuconfig: CUBE32 WiFi Configuration (if not using modem)
 * - Enable Modem in menuconfig: CUBE32 Board Configuration → Modem Configuration (optional)
 * - Enable SD Card for SSL certificates (optional)
 * - Enable these fonts in menuconfig:
 *   - LV_FONT_MONTSERRAT_14 (for labels and status)
 *   - LV_FONT_MONTSERRAT_24 (for larger text)
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <esp_heap_caps.h>
#include <nvs_flash.h>
#include <mqtt_client.h>
#include <esp_tls.h>
#include <esp_crt_bundle.h>
#include <cJSON.h>

#include "cube32.h"
#include "utils/hw_manifest.h"

static const char *TAG = "hello_mqtt";

// ============================================================================
// MQTT Configuration - Modify these settings for your broker
// ============================================================================

// MQTT Broker Configuration
#define MQTT_BROKER_HOST        "broker.hivemq.com"     // MQTT broker hostname
#define MQTT_BROKER_PORT        1883                     // MQTT broker port (1883=plain, 8883=TLS)
#define MQTT_USE_SSL            false                    // Enable SSL/TLS
#define MQTT_BROKER_USERNAME    ""                       // Username (empty if not required)
#define MQTT_BROKER_PASSWORD    ""                       // Password (empty if not required)

// Connection Settings
#define MQTT_CONNECT_TIMEOUT_MS 30000                    // Connection timeout in ms
#define MQTT_KEEPALIVE_SEC      60                       // Keep-alive interval in seconds
#define MQTT_AUTO_RECONNECT     true                     // Auto reconnect on disconnect
#define MQTT_RECONNECT_PERIOD_MS 5000                    // Reconnect period in ms

// Topics
#define MQTT_SUBSCRIBE_TOPIC    "cube32/demo/command"    // Topic to subscribe to
#define MQTT_PUBLISH_TOPIC      "cube32/demo/status"     // Topic to publish to

// SSL Certificate Files on SD Card (only used if MQTT_USE_SSL is true)
#define MQTT_CA_CERT_FILE       "/sdcard/certs/ca.pem"           // CA certificate
#define MQTT_CLIENT_CERT_FILE   "/sdcard/certs/client.crt"       // Client certificate
#define MQTT_CLIENT_KEY_FILE    "/sdcard/certs/client.key"       // Client private key

// AWS IoT Configuration (alternative to HiveMQ)
// Uncomment these for AWS IoT Core:
// #define MQTT_BROKER_HOST     "your-endpoint.iot.region.amazonaws.com"
// #define MQTT_BROKER_PORT     8883
// #define MQTT_USE_SSL         true
// #define MQTT_SUBSCRIBE_TOPIC "cube32/sub"
// #define MQTT_PUBLISH_TOPIC   "cube32/pub"

// ============================================================================
// UI Update Configuration
// ============================================================================

#define UI_UPDATE_INTERVAL_MS   1000    // Update UI every second
#define MSG_BUFFER_SIZE         256     // Buffer size for received messages

// WiFi configuration
#define WIFI_CONNECT_TIMEOUT_MS 30000   // 30 seconds timeout
#define WIFI_CONNECTED_BIT      BIT0
#define WIFI_FAIL_BIT           BIT1

// ============================================================================
// State Variables
// ============================================================================

// Network state
static EventGroupHandle_t s_wifi_event_group = nullptr;
static int s_retry_num = 0;
static bool s_network_connected = false;
static char s_ip_address[32] = "N/A";
static int s_wifi_rssi = -100;

// MQTT state
static esp_mqtt_client_handle_t s_mqtt_client = nullptr;
static bool s_mqtt_connected = false;
static int s_reconnect_countdown = 0;
static int s_msg_count_rx = 0;
static int s_msg_count_tx = 0;
static char s_last_message[MSG_BUFFER_SIZE] = "";
static char s_publish_status[64] = "Not connected";
static SemaphoreHandle_t s_mqtt_mutex = nullptr;

// Tracks which network interface was actually used at runtime
static bool s_using_modem = false;

// SSL Certificate buffers
static char *s_ca_cert = nullptr;
static char *s_client_cert = nullptr;
static char *s_client_key = nullptr;

// LVGL widgets
static lv_obj_t *s_conn_mode_label = nullptr;
static lv_obj_t *s_conn_status_label = nullptr;
static lv_obj_t *s_mqtt_status_label = nullptr;
static lv_obj_t *s_reconnect_label = nullptr;
static lv_obj_t *s_signal_label = nullptr;
static lv_obj_t *s_signal_bars = nullptr;
static lv_obj_t *s_sub_topic_label = nullptr;
static lv_obj_t *s_sub_message_label = nullptr;
static lv_obj_t *s_pub_topic_label = nullptr;
static lv_obj_t *s_pub_status_label = nullptr;
static lv_obj_t *s_pub_button = nullptr;
static lv_obj_t *s_msg_count_label = nullptr;

// Timers
static lv_timer_t *s_ui_update_timer = nullptr;

// ============================================================================
// SSL Certificate Loading from SD Card
// ============================================================================

#if MQTT_USE_SSL
static char* load_file_from_sdcard(const char* path) {
    FILE* f = fopen(path, "r");
    if (f == nullptr) {
        ESP_LOGW(TAG, "Failed to open file: %s", path);
        return nullptr;
    }
    
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    
    if (size <= 0) {
        fclose(f);
        ESP_LOGW(TAG, "File is empty: %s", path);
        return nullptr;
    }
    
    char* buffer = (char*)heap_caps_malloc(size + 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (buffer == nullptr) {
        buffer = (char*)malloc(size + 1);
    }
    
    if (buffer == nullptr) {
        fclose(f);
        ESP_LOGE(TAG, "Failed to allocate memory for file: %s", path);
        return nullptr;
    }
    
    size_t read = fread(buffer, 1, size, f);
    buffer[read] = '\0';
    fclose(f);
    
    ESP_LOGI(TAG, "Loaded certificate file: %s (%ld bytes)", path, size);
    return buffer;
}

static void load_ssl_certificates(void) {
    ESP_LOGI(TAG, "Loading SSL certificates from SD card...");
    
    s_ca_cert = load_file_from_sdcard(MQTT_CA_CERT_FILE);
    s_client_cert = load_file_from_sdcard(MQTT_CLIENT_CERT_FILE);
    s_client_key = load_file_from_sdcard(MQTT_CLIENT_KEY_FILE);
    
    if (s_ca_cert == nullptr) {
        ESP_LOGW(TAG, "CA certificate not loaded - will use ESP certificate bundle");
    }
}
#endif

// ============================================================================
// Signal Strength Helpers
// ============================================================================

static void update_signal_bars(int strength) {
    if (!s_signal_bars) return;
    
    // Strength: 0-4 bars
    int bars = 0;

    if (s_using_modem) {
        // For modem: RSSI ranges 0-31 (99=unknown)
        if (strength != 99) {
            if (strength >= 20) bars = 4;
            else if (strength >= 15) bars = 3;
            else if (strength >= 10) bars = 2;
            else if (strength >= 1) bars = 1;
        }
    } else {
        // For WiFi: RSSI in dBm (-30 to -90)
        if (strength > -50) bars = 4;
        else if (strength > -60) bars = 3;
        else if (strength > -70) bars = 2;
        else if (strength > -80) bars = 1;
    }
    
    // Update bar colors
    uint32_t child_cnt = lv_obj_get_child_count(s_signal_bars);
    for (uint32_t i = 0; i < child_cnt && i < 4; i++) {
        lv_obj_t *bar = lv_obj_get_child(s_signal_bars, i);
        if (bar) {
            if ((int)i < bars) {
                lv_obj_set_style_bg_color(bar, lv_color_hex(0x00FF00), 0);  // Green
            } else {
                lv_obj_set_style_bg_color(bar, lv_color_hex(0x404040), 0);  // Dark gray
            }
        }
    }
}

static void create_signal_bars(lv_obj_t* parent) {
    s_signal_bars = lv_obj_create(parent);
    lv_obj_remove_style_all(s_signal_bars);
    lv_obj_set_size(s_signal_bars, 50, 20);
    lv_obj_set_flex_flow(s_signal_bars, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(s_signal_bars, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_END);
    lv_obj_set_style_pad_column(s_signal_bars, 3, 0);
    
    // Create 4 bars with increasing height
    int heights[] = {5, 10, 15, 20};
    for (int i = 0; i < 4; i++) {
        lv_obj_t *bar = lv_obj_create(s_signal_bars);
        lv_obj_remove_style_all(bar);
        lv_obj_set_size(bar, 8, heights[i]);
        lv_obj_set_style_bg_color(bar, lv_color_hex(0x404040), 0);
        lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(bar, 2, 0);
    }
}

// ============================================================================
// MQTT Event Handler
// ============================================================================

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, 
                                int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    
    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT Connected");
            s_mqtt_connected = true;
            s_reconnect_countdown = 0;
            
            // Subscribe to topic
            esp_mqtt_client_subscribe(s_mqtt_client, MQTT_SUBSCRIBE_TOPIC, 1);
            ESP_LOGI(TAG, "Subscribed to: %s", MQTT_SUBSCRIBE_TOPIC);
            
            if (xSemaphoreTake(s_mqtt_mutex, pdMS_TO_TICKS(100))) {
                strncpy(s_publish_status, "Connected", sizeof(s_publish_status));
                xSemaphoreGive(s_mqtt_mutex);
            }
            break;
            
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT Disconnected");
            s_mqtt_connected = false;
            s_reconnect_countdown = MQTT_RECONNECT_PERIOD_MS / 1000;
            
            if (xSemaphoreTake(s_mqtt_mutex, pdMS_TO_TICKS(100))) {
                strncpy(s_publish_status, "Disconnected", sizeof(s_publish_status));
                xSemaphoreGive(s_mqtt_mutex);
            }
            break;
            
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT Subscribed, msg_id=%d", event->msg_id);
            break;
            
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT Unsubscribed, msg_id=%d", event->msg_id);
            break;
            
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT Published, msg_id=%d", event->msg_id);
            s_msg_count_tx++;
            
            if (xSemaphoreTake(s_mqtt_mutex, pdMS_TO_TICKS(100))) {
                strncpy(s_publish_status, "Published OK", sizeof(s_publish_status));
                xSemaphoreGive(s_mqtt_mutex);
            }
            break;
            
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT Data received on topic: %.*s", event->topic_len, event->topic);
            s_msg_count_rx++;
            
            if (xSemaphoreTake(s_mqtt_mutex, pdMS_TO_TICKS(100))) {
                // Copy received message (truncate if too long)
                int copy_len = event->data_len < MSG_BUFFER_SIZE - 1 ? event->data_len : MSG_BUFFER_SIZE - 1;
                memcpy(s_last_message, event->data, copy_len);
                s_last_message[copy_len] = '\0';
                xSemaphoreGive(s_mqtt_mutex);
            }
            ESP_LOGI(TAG, "Data: %.*s", event->data_len, event->data);
            break;
            
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT Error");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                ESP_LOGE(TAG, "Transport error: %s", strerror(event->error_handle->esp_transport_sock_errno));
            }
            
            if (xSemaphoreTake(s_mqtt_mutex, pdMS_TO_TICKS(100))) {
                strncpy(s_publish_status, "Error", sizeof(s_publish_status));
                xSemaphoreGive(s_mqtt_mutex);
            }
            break;
            
        default:
            break;
    }
}

// ============================================================================
// MQTT Client Initialization
// ============================================================================

static esp_err_t mqtt_init(void) {
    ESP_LOGI(TAG, "Initializing MQTT client...");
    
    // Build broker URI
    char uri[256];
    if (MQTT_USE_SSL) {
        snprintf(uri, sizeof(uri), "mqtts://%s:%d", MQTT_BROKER_HOST, MQTT_BROKER_PORT);
    } else {
        snprintf(uri, sizeof(uri), "mqtt://%s:%d", MQTT_BROKER_HOST, MQTT_BROKER_PORT);
    }
    
    ESP_LOGI(TAG, "MQTT Broker: %s", uri);
    
    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri = uri;
    mqtt_cfg.session.keepalive = MQTT_KEEPALIVE_SEC;
    mqtt_cfg.network.timeout_ms = MQTT_CONNECT_TIMEOUT_MS;
    mqtt_cfg.network.reconnect_timeout_ms = MQTT_RECONNECT_PERIOD_MS;
    mqtt_cfg.network.disable_auto_reconnect = !MQTT_AUTO_RECONNECT;
    
    // Set credentials if provided
    if (strlen(MQTT_BROKER_USERNAME) > 0) {
        mqtt_cfg.credentials.username = MQTT_BROKER_USERNAME;
    }
    if (strlen(MQTT_BROKER_PASSWORD) > 0) {
        mqtt_cfg.credentials.authentication.password = MQTT_BROKER_PASSWORD;
    }
    
    // Configure SSL/TLS
    if (MQTT_USE_SSL) {
#if MQTT_USE_SSL
        load_ssl_certificates();
        
        if (s_ca_cert) {
            mqtt_cfg.broker.verification.certificate = s_ca_cert;
        } else {
            // Use ESP certificate bundle if no CA cert provided
            mqtt_cfg.broker.verification.crt_bundle_attach = esp_crt_bundle_attach;
        }
        
        if (s_client_cert && s_client_key) {
            mqtt_cfg.credentials.authentication.certificate = s_client_cert;
            mqtt_cfg.credentials.authentication.key = s_client_key;
        }
#endif
    }
    
    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (s_mqtt_client == nullptr) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return ESP_FAIL;
    }
    
    esp_mqtt_client_register_event(s_mqtt_client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, 
                                   mqtt_event_handler, nullptr);
    
    esp_err_t ret = esp_mqtt_client_start(s_mqtt_client);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT client: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "MQTT client started");
    return ESP_OK;
}

// ============================================================================
// System Status JSON Publishing
// ============================================================================

static void publish_system_status(void) {
    if (!s_mqtt_connected) {
        ESP_LOGW(TAG, "Cannot publish - not connected");
        if (xSemaphoreTake(s_mqtt_mutex, pdMS_TO_TICKS(100))) {
            strncpy(s_publish_status, "Not connected", sizeof(s_publish_status));
            xSemaphoreGive(s_mqtt_mutex);
        }
        return;
    }
    
    // Create JSON status
    cJSON *root = cJSON_CreateObject();
    
    // Device info
    cJSON_AddStringToObject(root, "device", "CUBE32");
    cJSON_AddNumberToObject(root, "uptime_sec", (int)(esp_timer_get_time() / 1000000));
    
    // Memory info
    cJSON_AddNumberToObject(root, "free_heap", (int)esp_get_free_heap_size());
    cJSON_AddNumberToObject(root, "min_free_heap", (int)esp_get_minimum_free_heap_size());
    
    // Network info
    cJSON_AddStringToObject(root, "ip", s_ip_address);
    if (s_using_modem) {
        cJSON_AddStringToObject(root, "connection", "modem");
#ifdef CONFIG_CUBE32_MODEM_ENABLED
        cube32::A7670Modem& modem = cube32::A7670Modem::instance();
        cJSON_AddNumberToObject(root, "signal_rssi", modem.getSignalQuality());
#endif
    } else {
        cJSON_AddStringToObject(root, "connection", "wifi");
        cJSON_AddNumberToObject(root, "wifi_rssi", s_wifi_rssi);
    }
    
    // MQTT stats
    cJSON_AddNumberToObject(root, "mqtt_rx", s_msg_count_rx);
    cJSON_AddNumberToObject(root, "mqtt_tx", s_msg_count_tx);
    
    // Timestamp
    time_t now;
    time(&now);
    cJSON_AddNumberToObject(root, "timestamp", (int)now);
    
    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    
    if (json_str == nullptr) {
        ESP_LOGE(TAG, "Failed to create JSON");
        return;
    }
    
    ESP_LOGI(TAG, "Publishing: %s", json_str);
    
    if (xSemaphoreTake(s_mqtt_mutex, pdMS_TO_TICKS(100))) {
        strncpy(s_publish_status, "Publishing...", sizeof(s_publish_status));
        xSemaphoreGive(s_mqtt_mutex);
    }
    
    int msg_id = esp_mqtt_client_publish(s_mqtt_client, MQTT_PUBLISH_TOPIC, 
                                          json_str, strlen(json_str), 1, 0);
    
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Publish failed");
        if (xSemaphoreTake(s_mqtt_mutex, pdMS_TO_TICKS(100))) {
            strncpy(s_publish_status, "Publish failed", sizeof(s_publish_status));
            xSemaphoreGive(s_mqtt_mutex);
        }
    }
    
    free(json_str);
}

// ============================================================================
// Button Event Handler
// ============================================================================

static void pub_button_event_cb(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED) {
        ESP_LOGI(TAG, "Publish button clicked");
        publish_system_status();
    }
}

// ============================================================================
// Network Connection (WiFi or Modem)
// ============================================================================

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_network_connected = false;
        if (s_retry_num < CONFIG_CUBE32_WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retrying WiFi connection (%d/%d)...", s_retry_num, CONFIG_CUBE32_WIFI_MAX_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "WiFi connection failed after %d retries", CONFIG_CUBE32_WIFI_MAX_RETRY);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        snprintf(s_ip_address, sizeof(s_ip_address), IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Connected! IP: %s", s_ip_address);
        s_retry_num = 0;
        s_network_connected = true;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static esp_err_t wifi_init_sta(void) {
    s_wifi_event_group = xEventGroupCreate();

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        nullptr,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        nullptr,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {};
    strncpy((char*)wifi_config.sta.ssid, CONFIG_CUBE32_WIFI_SSID, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char*)wifi_config.sta.password, CONFIG_CUBE32_WIFI_PASSWORD, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi STA initialized, connecting to %s...", CONFIG_CUBE32_WIFI_SSID);

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE, pdFALSE,
            pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Successfully connected to SSID: %s", CONFIG_CUBE32_WIFI_SSID);
        return ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to SSID: %s", CONFIG_CUBE32_WIFI_SSID);
        return ESP_FAIL;
    } else {
        ESP_LOGE(TAG, "WiFi connection timeout");
        return ESP_ERR_TIMEOUT;
    }
}

static esp_err_t init_network(void) {
#ifdef CONFIG_CUBE32_MODEM_ENABLED
    // Use modem only when the module is physically detected AND the active flag is set
    const cube32_hw_manifest_t* hw = cube32_hw_manifest();
    if (hw->modem_module_present && hw->modem_active) {
        ESP_LOGI(TAG, "Modem module detected and active — using LTE for internet connection...");

        cube32::A7670Modem& modem = cube32::A7670Modem::instance();

        // Wait for modem to initialize
        int timeout = 60;  // 60 seconds timeout
        while (!modem.isInitialized() && timeout > 0) {
            ESP_LOGI(TAG, "Waiting for modem initialization... (%d)", timeout);
            vTaskDelay(pdMS_TO_TICKS(1000));
            timeout--;
        }

        if (!modem.isInitialized()) {
            ESP_LOGE(TAG, "Modem failed to initialize");
            return ESP_FAIL;
        }

        // Wait for network registration
        timeout = 60;
        while (!modem.isNetworkReady() && timeout > 0) {
            ESP_LOGI(TAG, "Waiting for network registration... (%d)", timeout);
            vTaskDelay(pdMS_TO_TICKS(1000));
            timeout--;
        }

        if (!modem.isNetworkReady()) {
            ESP_LOGE(TAG, "Network registration failed");
            return ESP_FAIL;
        }

        // Switch to data mode (PPP)
        ESP_LOGI(TAG, "Switching to data mode (PPP)...");
        if (modem.setDataMode() != CUBE32_OK) {
            ESP_LOGE(TAG, "Failed to enable data mode");
            return ESP_FAIL;
        }

        // Wait for PPP connection
        timeout = 30;
        while (!modem.isPPPConnected() && timeout > 0) {
            ESP_LOGI(TAG, "Waiting for PPP connection... (%d)", timeout);
            vTaskDelay(pdMS_TO_TICKS(1000));
            timeout--;
        }

        if (!modem.isPPPConnected()) {
            ESP_LOGE(TAG, "PPP connection failed");
            return ESP_FAIL;
        }

        // Wait for valid IP address (not 0.0.0.0)
        std::string ip;
        timeout = 10;
        while (timeout > 0) {
            if (modem.getIPAddress(ip) == CUBE32_OK && !ip.empty() && ip != "0.0.0.0") {
                strncpy(s_ip_address, ip.c_str(), sizeof(s_ip_address) - 1);
                break;
            }
            ESP_LOGI(TAG, "Waiting for IP address... (%d)", timeout);
            vTaskDelay(pdMS_TO_TICKS(500));
            timeout--;
        }

        if (strcmp(s_ip_address, "N/A") == 0 || strcmp(s_ip_address, "0.0.0.0") == 0) {
            // Try one more time from netif
            esp_netif_ip_info_t ip_info;
            esp_netif_t *netif = esp_netif_get_handle_from_ifkey("PPP_DEF");
            if (netif && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
                snprintf(s_ip_address, sizeof(s_ip_address), IPSTR, IP2STR(&ip_info.ip));
            }
        }

        s_network_connected = true;
        s_using_modem = true;
        ESP_LOGI(TAG, "Modem connected! IP: %s", s_ip_address);
        return ESP_OK;
    } else if (hw->modem_module_present && !hw->modem_active) {
        ESP_LOGW(TAG, "Modem detected but Active flag is off — falling back to WiFi");
        s_using_modem = false;
    } else {
        ESP_LOGW(TAG, "Modem enabled in Kconfig but module not detected — falling back to WiFi");
    }
#endif
    // Use WiFi for network connection
    ESP_LOGI(TAG, "Using WiFi for internet connection...");
    return wifi_init_sta();
}

// ============================================================================
// UI Update Callback
// ============================================================================

static void update_ui_timer_cb(lv_timer_t *timer) {
    (void)timer;
    
    cube32::LvglDisplay& lvgl = cube32::LvglDisplay::instance();
    if (!lvgl.lock(10)) {
        return;
    }
    
    // Update connection mode
    if (s_conn_mode_label) {
        if (s_using_modem) {
            lv_label_set_text(s_conn_mode_label, LV_SYMBOL_CALL " 4G/LTE");
        } else {
            lv_label_set_text(s_conn_mode_label, LV_SYMBOL_WIFI " WiFi");
        }
    }
    
    // Update connection status
    if (s_conn_status_label) {
        if (s_network_connected) {
            lv_label_set_text_fmt(s_conn_status_label, "IP: %s", s_ip_address);
            lv_obj_set_style_text_color(s_conn_status_label, lv_color_hex(0x44FF44), 0);
        } else {
            lv_label_set_text(s_conn_status_label, "Connecting...");
            lv_obj_set_style_text_color(s_conn_status_label, lv_color_hex(0xFFAA00), 0);
        }
    }
    
    // Update MQTT status
    if (s_mqtt_status_label) {
        if (s_mqtt_connected) {
            lv_label_set_text(s_mqtt_status_label, "MQTT: Connected");
            lv_obj_set_style_text_color(s_mqtt_status_label, lv_color_hex(0x44FF44), 0);
        } else if (s_network_connected) {
            lv_label_set_text(s_mqtt_status_label, "MQTT: Connecting...");
            lv_obj_set_style_text_color(s_mqtt_status_label, lv_color_hex(0xFFAA00), 0);
        } else {
            lv_label_set_text(s_mqtt_status_label, "MQTT: Waiting for network");
            lv_obj_set_style_text_color(s_mqtt_status_label, lv_color_hex(0xFF4444), 0);
        }
    }
    
    // Update reconnect countdown
    if (s_reconnect_label) {
        if (!s_mqtt_connected && s_reconnect_countdown > 0) {
            lv_label_set_text_fmt(s_reconnect_label, "Reconnect in: %ds", s_reconnect_countdown);
            lv_obj_set_style_text_color(s_reconnect_label, lv_color_hex(0xFFAA00), 0);
            s_reconnect_countdown--;
        } else if (s_mqtt_connected) {
            lv_label_set_text(s_reconnect_label, "");
        } else {
            lv_label_set_text(s_reconnect_label, "");
        }
    }
    
    // Update signal strength
    int signal = 0;
    if (s_using_modem) {
#ifdef CONFIG_CUBE32_MODEM_ENABLED
        cube32::A7670Modem& modem = cube32::A7670Modem::instance();
        signal = modem.getSignalQuality();
        if (s_signal_label) {
            if (signal != 99) {
                int dbm = -113 + (2 * signal);
                lv_label_set_text_fmt(s_signal_label, "%d dBm", dbm);
            } else {
                lv_label_set_text(s_signal_label, "N/A");
            }
        }
#endif
    } else {
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            s_wifi_rssi = ap_info.rssi;
            signal = ap_info.rssi;
            if (s_signal_label) {
                lv_label_set_text_fmt(s_signal_label, "%d dBm", ap_info.rssi);
            }
        } else {
            if (s_signal_label) {
                lv_label_set_text(s_signal_label, "N/A");
            }
        }
    }
    update_signal_bars(signal);
    
    // Update received message
    if (s_sub_message_label) {
        if (xSemaphoreTake(s_mqtt_mutex, pdMS_TO_TICKS(10))) {
            if (strlen(s_last_message) > 0) {
                lv_label_set_text(s_sub_message_label, s_last_message);
            } else {
                lv_label_set_text(s_sub_message_label, "(no message)");
            }
            xSemaphoreGive(s_mqtt_mutex);
        }
    }
    
    // Update publish status
    if (s_pub_status_label) {
        if (xSemaphoreTake(s_mqtt_mutex, pdMS_TO_TICKS(10))) {
            lv_label_set_text(s_pub_status_label, s_publish_status);
            xSemaphoreGive(s_mqtt_mutex);
        }
    }
    
    // Update message counts
    if (s_msg_count_label) {
        lv_label_set_text_fmt(s_msg_count_label, "RX: %d  TX: %d", s_msg_count_rx, s_msg_count_tx);
    }
    
    // Update button state
    if (s_pub_button) {
        if (s_mqtt_connected) {
            lv_obj_remove_state(s_pub_button, LV_STATE_DISABLED);
            lv_obj_set_style_bg_color(s_pub_button, lv_color_hex(0x0066CC), 0);
        } else {
            lv_obj_add_state(s_pub_button, LV_STATE_DISABLED);
            lv_obj_set_style_bg_color(s_pub_button, lv_color_hex(0x404040), 0);
        }
    }
    
    lvgl.unlock();
}

// ============================================================================
// UI Creation
// ============================================================================

static void create_ui(void) {
    cube32::LvglDisplay& lvgl = cube32::LvglDisplay::instance();
    
    if (!lvgl.lock(1000)) {
        ESP_LOGE(TAG, "Failed to lock LVGL mutex");
        return;
    }

    // Get active screen
    lv_obj_t *scr = lv_screen_active();
    uint16_t scr_width = lvgl.getWidth();
    uint16_t scr_height = lvgl.getHeight();
    
    // Set dark theme background (black for prism display compatibility)
    lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);

    // Create main container with flex layout
    lv_obj_t *main_cont = lv_obj_create(scr);
    lv_obj_remove_style_all(main_cont);
    lv_obj_set_size(main_cont, scr_width, scr_height);
    lv_obj_set_style_bg_color(main_cont, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(main_cont, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(main_cont, 8, 0);
    lv_obj_set_flex_flow(main_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(main_cont, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_row(main_cont, 4, 0);

    // ========================================================================
    // Header row: Title + Signal bars
    // ========================================================================
    lv_obj_t *header_row = lv_obj_create(main_cont);
    lv_obj_remove_style_all(header_row);
    lv_obj_set_size(header_row, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(header_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(header_row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    
    // Title
    lv_obj_t *title = lv_label_create(header_row);
    lv_label_set_text(title, "MQTT Client");
    lv_obj_set_style_text_color(title, lv_color_hex(0x00BFFF), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    
    // Signal strength container
    lv_obj_t *signal_cont = lv_obj_create(header_row);
    lv_obj_remove_style_all(signal_cont);
    lv_obj_set_size(signal_cont, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(signal_cont, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(signal_cont, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(signal_cont, 5, 0);
    
    s_signal_label = lv_label_create(signal_cont);
    lv_label_set_text(s_signal_label, "N/A");
    lv_obj_set_style_text_color(s_signal_label, lv_color_hex(0xCCCCCC), 0);
    lv_obj_set_style_text_font(s_signal_label, &lv_font_montserrat_14, 0);
    
    create_signal_bars(signal_cont);

    // ========================================================================
    // Connection section
    // ========================================================================
    lv_obj_t *conn_section = lv_obj_create(main_cont);
    lv_obj_remove_style_all(conn_section);
    lv_obj_set_size(conn_section, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(conn_section, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(conn_section, 2, 0);
    
    // Connection mode
    s_conn_mode_label = lv_label_create(conn_section);
    lv_label_set_text(s_conn_mode_label,
        s_using_modem ? LV_SYMBOL_CALL " 4G/LTE" : LV_SYMBOL_WIFI " WiFi");
    lv_obj_set_style_text_color(s_conn_mode_label, lv_color_hex(0xCCCCCC), 0);
    lv_obj_set_style_text_font(s_conn_mode_label, &lv_font_montserrat_14, 0);
    
    // Connection status (IP)
    s_conn_status_label = lv_label_create(conn_section);
    lv_label_set_text(s_conn_status_label, "Connecting...");
    lv_obj_set_style_text_color(s_conn_status_label, lv_color_hex(0xFFAA00), 0);
    lv_obj_set_style_text_font(s_conn_status_label, &lv_font_montserrat_14, 0);
    
    // MQTT status
    s_mqtt_status_label = lv_label_create(conn_section);
    lv_label_set_text(s_mqtt_status_label, "MQTT: Waiting for network");
    lv_obj_set_style_text_color(s_mqtt_status_label, lv_color_hex(0xFF4444), 0);
    lv_obj_set_style_text_font(s_mqtt_status_label, &lv_font_montserrat_14, 0);
    
    // Reconnect countdown
    s_reconnect_label = lv_label_create(conn_section);
    lv_label_set_text(s_reconnect_label, "");
    lv_obj_set_style_text_color(s_reconnect_label, lv_color_hex(0xFFAA00), 0);
    lv_obj_set_style_text_font(s_reconnect_label, &lv_font_montserrat_14, 0);

    // Separator
    lv_obj_t *sep1 = lv_obj_create(main_cont);
    lv_obj_remove_style_all(sep1);
    lv_obj_set_size(sep1, LV_PCT(100), 2);
    lv_obj_set_style_bg_color(sep1, lv_color_hex(0x404060), 0);
    lv_obj_set_style_bg_opa(sep1, LV_OPA_COVER, 0);

    // ========================================================================
    // Subscribe section
    // ========================================================================
    lv_obj_t *sub_section = lv_obj_create(main_cont);
    lv_obj_remove_style_all(sub_section);
    lv_obj_set_size(sub_section, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(sub_section, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(sub_section, 2, 0);
    
    // Subscribe topic label
    s_sub_topic_label = lv_label_create(sub_section);
    lv_label_set_text_fmt(s_sub_topic_label, LV_SYMBOL_DOWNLOAD " %s", MQTT_SUBSCRIBE_TOPIC);
    lv_obj_set_style_text_color(s_sub_topic_label, lv_color_hex(0x00BFFF), 0);
    lv_obj_set_style_text_font(s_sub_topic_label, &lv_font_montserrat_14, 0);
    lv_obj_set_width(s_sub_topic_label, LV_PCT(100));
    lv_label_set_long_mode(s_sub_topic_label, LV_LABEL_LONG_SCROLL_CIRCULAR);
    
    // Received message
    s_sub_message_label = lv_label_create(sub_section);
    lv_label_set_text(s_sub_message_label, "(no message)");
    lv_obj_set_style_text_color(s_sub_message_label, lv_color_hex(0x88FF88), 0);
    lv_obj_set_style_text_font(s_sub_message_label, &lv_font_montserrat_14, 0);
    lv_obj_set_width(s_sub_message_label, LV_PCT(100));
    lv_label_set_long_mode(s_sub_message_label, LV_LABEL_LONG_SCROLL_CIRCULAR);

    // ========================================================================
    // Publish section
    // ========================================================================
    lv_obj_t *pub_section = lv_obj_create(main_cont);
    lv_obj_remove_style_all(pub_section);
    lv_obj_set_size(pub_section, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(pub_section, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(pub_section, 2, 0);
    
    // Publish topic label
    s_pub_topic_label = lv_label_create(pub_section);
    lv_label_set_text_fmt(s_pub_topic_label, LV_SYMBOL_UPLOAD " %s", MQTT_PUBLISH_TOPIC);
    lv_obj_set_style_text_color(s_pub_topic_label, lv_color_hex(0xFF8800), 0);
    lv_obj_set_style_text_font(s_pub_topic_label, &lv_font_montserrat_14, 0);
    lv_obj_set_width(s_pub_topic_label, LV_PCT(100));
    lv_label_set_long_mode(s_pub_topic_label, LV_LABEL_LONG_SCROLL_CIRCULAR);
    
    // Publish status
    s_pub_status_label = lv_label_create(pub_section);
    lv_label_set_text(s_pub_status_label, "Not connected");
    lv_obj_set_style_text_color(s_pub_status_label, lv_color_hex(0xCCCCCC), 0);
    lv_obj_set_style_text_font(s_pub_status_label, &lv_font_montserrat_14, 0);

    // ========================================================================
    // Bottom section: Message counts + Publish button
    // ========================================================================
    lv_obj_t *bottom_section = lv_obj_create(main_cont);
    lv_obj_remove_style_all(bottom_section);
    lv_obj_set_size(bottom_section, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(bottom_section, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(bottom_section, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_flex_grow(bottom_section, 1);
    lv_obj_set_style_pad_top(bottom_section, 5, 0);
    
    // Message counts
    s_msg_count_label = lv_label_create(bottom_section);
    lv_label_set_text(s_msg_count_label, "RX: 0  TX: 0");
    lv_obj_set_style_text_color(s_msg_count_label, lv_color_hex(0x888888), 0);
    lv_obj_set_style_text_font(s_msg_count_label, &lv_font_montserrat_14, 0);
    
    // Publish button
    s_pub_button = lv_btn_create(bottom_section);
    lv_obj_set_size(s_pub_button, 100, 35);
    lv_obj_set_style_bg_color(s_pub_button, lv_color_hex(0x404040), 0);
    lv_obj_set_style_radius(s_pub_button, 8, 0);
    lv_obj_add_state(s_pub_button, LV_STATE_DISABLED);
    lv_obj_add_event_cb(s_pub_button, pub_button_event_cb, LV_EVENT_CLICKED, nullptr);
    
    lv_obj_t *btn_label = lv_label_create(s_pub_button);
    lv_label_set_text(btn_label, "Publish");
    lv_obj_set_style_text_color(btn_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(btn_label, &lv_font_montserrat_14, 0);
    lv_obj_center(btn_label);

    // Create UI update timer
    s_ui_update_timer = lv_timer_create(update_ui_timer_cb, UI_UPDATE_INTERVAL_MS, nullptr);
    
    lvgl.unlock();
    
    ESP_LOGI(TAG, "UI created successfully (%dx%d)", scr_width, scr_height);
}

// ============================================================================
// Main Application Entry Point
// ============================================================================

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "CUBE32 Hello MQTT Example");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Broker: %s:%d", MQTT_BROKER_HOST, MQTT_BROKER_PORT);
    ESP_LOGI(TAG, "SSL/TLS: %s", MQTT_USE_SSL ? "Enabled" : "Disabled");
    ESP_LOGI(TAG, "Subscribe: %s", MQTT_SUBSCRIBE_TOPIC);
    ESP_LOGI(TAG, "Publish: %s", MQTT_PUBLISH_TOPIC);

    // Create MQTT mutex
    s_mqtt_mutex = xSemaphoreCreateMutex();
    if (s_mqtt_mutex == nullptr) {
        ESP_LOGE(TAG, "Failed to create MQTT mutex");
        return;
    }

    // Initialize CUBE32 board
    esp_err_t ret = cube32_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize CUBE32: %s", esp_err_to_name(ret));
        return;
    }

#ifndef CONFIG_CUBE32_LVGL_ENABLED
    ESP_LOGE(TAG, "LVGL is not enabled!");
    ESP_LOGE(TAG, "Please enable it in menuconfig:");
    ESP_LOGE(TAG, "  CUBE32 Board Configuration -> Display Configuration -> Enable LVGL");
    return;
#else
    cube32::LvglDisplay& lvgl = cube32::LvglDisplay::instance();
    if (!lvgl.isInitialized()) {
        ESP_LOGE(TAG, "LVGL failed to initialize");
        return;
    }

    // Pre-determine connection mode so create_ui() renders the correct badge
#ifdef CONFIG_CUBE32_MODEM_ENABLED
    {
        const cube32_hw_manifest_t* hw = cube32_hw_manifest();
        s_using_modem = (hw->modem_module_present && hw->modem_active);
        if (hw->modem_module_present && !hw->modem_active) {
            ESP_LOGI(TAG, "Modem detected but Active flag is off — will use WiFi");
        }
    }
#endif

    // Create the UI first (shows "Connecting..." status)
    create_ui();

    // Initialize network connection (WiFi or Modem)
    ESP_LOGI(TAG, "Initializing network connection...");
    ret = init_network();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Network connection failed");
        // Continue anyway - MQTT will retry when network becomes available
    } else {
        // Initialize MQTT client
        ret = mqtt_init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "MQTT initialization failed");
        }
    }

    ESP_LOGI(TAG, "MQTT Demo started!");
    if (s_using_modem) {
        ESP_LOGI(TAG, "Network: LTE Modem");
    } else {
        ESP_LOGI(TAG, "Network: WiFi (%s)", CONFIG_CUBE32_WIFI_SSID);
    }

    // Main loop - LVGL handles rendering via its own task
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
#endif
}
