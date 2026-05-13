/**
 * @file main.cpp
 * @brief CUBE32 Hello Modem USB Console Example
 * 
 * This example demonstrates:
 * - A7670 LTE modem driver via USB connection
 * - Console interface for AT commands
 * - LVGL 9.x dark theme UI displaying:
 *   - Modem connection status
 *   - Network registration status
 *   - Signal quality
 *   - Operator name
 *   - IP address (when connected)
 *   - Current date/time (from network or RTC)
 * 
 * Console Commands:
 * - AT<cmd>  : Send AT command to modem (e.g., AT+CSQ for signal)
 * - status   : Print modem status
 * - info     : Print modem information
 * - data     : Switch to data mode (PPP)
 * - cmd      : Switch to command mode
 * - reset    : Reset modem
 * - help     : Show available commands
 * 
 * Prerequisites:
 * - Enable Modem in menuconfig: CUBE32 Board Configuration → Modem Configuration → Enable Modem
 * - Select USB connection type
 * - Enable LVGL in menuconfig: CUBE32 Board Configuration → Display Configuration → Enable LVGL
 * - Enable Touch (optional): CUBE32 Board Configuration → Touch Configuration → Enable Touch
 * - Enable these fonts in menuconfig:
 *   - LV_FONT_MONTSERRAT_14 (for labels and status)
 *   - LV_FONT_MONTSERRAT_24 (for larger text)
 *   - LV_FONT_MONTSERRAT_32 (for time display)
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_console.h>
#include <esp_vfs_dev.h>
#include <driver/uart.h>
#include <linenoise/linenoise.h>
#include "esp_netif.h"
#include "esp_http_client.h"
#include "esp_tls.h"
#include "esp_crt_bundle.h"
#include "ping/ping_sock.h"
#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"

#include "cube32.h"

static const char *TAG = "hello_modem";

// ============================================================================
// Configuration
// ============================================================================

#define UI_UPDATE_INTERVAL_MS   1000    // Update UI every second
#define CONSOLE_MAX_CMD_LEN     256
#define TIMEZONE                "CST-8"  // Default timezone (GMT+8)

// ============================================================================
// State Variables
// ============================================================================

// LVGL widgets
static lv_obj_t *s_status_label = nullptr;
static lv_obj_t *s_network_label = nullptr;
static lv_obj_t *s_signal_label = nullptr;
static lv_obj_t *s_operator_label = nullptr;
static lv_obj_t *s_ip_label = nullptr;
static lv_obj_t *s_time_label = nullptr;
static lv_obj_t *s_mode_label = nullptr;
static lv_obj_t *s_imei_label = nullptr;
static lv_obj_t *s_title_label = nullptr;

// Signal quality icon container
static lv_obj_t *s_signal_bars = nullptr;

// Update timer
static lv_timer_t *s_update_timer = nullptr;

// Console task handle
static TaskHandle_t s_console_task_handle = nullptr;

// Network ready state tracking
static bool s_network_ready_notified = false;

// ============================================================================
// Signal Strength Icon Helper
// ============================================================================

static void update_signal_bars(int rssi) {
    if (!s_signal_bars) return;
    
    // RSSI ranges: 0-9 (poor), 10-14 (fair), 15-19 (good), 20-31 (excellent), 99 (unknown)
    int bars = 0;
    if (rssi != 99) {
        if (rssi >= 20) bars = 4;
        else if (rssi >= 15) bars = 3;
        else if (rssi >= 10) bars = 2;
        else if (rssi >= 1) bars = 1;
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

// ============================================================================
// UI Update Callback
// ============================================================================

static void update_ui_timer_cb(lv_timer_t *timer) {
    (void)timer;
    
    cube32::LvglDisplay& lvgl = cube32::LvglDisplay::instance();
    if (!lvgl.lock(10)) {
        return;
    }
    
    cube32::A7670Modem& modem = cube32::A7670Modem::instance();
    
    // Update modem status with color coding
    if (s_status_label) {
        const char* status_str = modem.getStateString();
        lv_color_t status_color = lv_color_hex(0xCCCCCC);
        
        if (modem.isInitializing()) {
            status_color = lv_color_hex(0xFFAA00);  // Orange while initializing
        } else if (modem.getState() == cube32::ModemState::ERROR) {
            status_color = lv_color_hex(0xFF4444);  // Red on error
        } else if (modem.isInitialized()) {
            status_color = lv_color_hex(0x44FF44);  // Green when ready
        }
        
        lv_label_set_text_fmt(s_status_label, "Status: %s", status_str);
        lv_obj_set_style_text_color(s_status_label, status_color, 0);
    }
    
    // Update mode - show actual state
    if (s_mode_label) {
        const char* mode;
        lv_color_t mode_color = lv_color_hex(0xCCCCCC);
        
        if (modem.isInitializing()) {
            mode = "INITIALIZING...";
            mode_color = lv_color_hex(0xFFAA00);  // Orange
        } else if (modem.isDataMode()) {
            mode = "DATA (PPP)";
            mode_color = lv_color_hex(0x44FF44);  // Green when in data mode
        } else if (modem.isCommandMode()) {
            mode = "COMMAND (AT)";
        } else if (modem.getState() == cube32::ModemState::ERROR) {
            mode = "ERROR";
            mode_color = lv_color_hex(0xFF4444);  // Red
        } else {
            // Could be INITIALIZED, CONNECTED
            mode = modem.getStateString();
            mode_color = lv_color_hex(0xFFFF44);  // Yellow for transitional states
        }
        
        if (!modem.isNetworkReady() && modem.isInitialized()) {
            lv_label_set_text_fmt(s_mode_label, "Mode: %s (Registering...)", mode);
            mode_color = lv_color_hex(0xFFFF44);  // Yellow while registering
        } else {
            lv_label_set_text_fmt(s_mode_label, "Mode: %s", mode);
        }
        lv_obj_set_style_text_color(s_mode_label, mode_color, 0);
    }
    
    // Notify console when network becomes ready (one-time notification)
    if (modem.isNetworkReady() && !s_network_ready_notified) {
        s_network_ready_notified = true;
        ESP_LOGI(TAG, "Network registered! Ready for data mode.");
        printf("\n*** Network registered! Type 'data' to enable PPP connection ***\n");
        printf("modem> ");  // Re-print prompt
        fflush(stdout);
    }
    
    // Update network status
    if (s_network_label) {
        const char* status_str = "Unknown";
        lv_color_t status_color = lv_color_hex(0xFFFFFF);
        
        if (modem.isInitializing()) {
            status_str = "Waiting for modem...";
            status_color = lv_color_hex(0xFFAA00);  // Orange
        } else if (modem.isInitialized()) {
            cube32::NetworkStatus net_status = modem.getNetworkStatus();
            
            switch (net_status) {
                case cube32::NetworkStatus::NOT_REGISTERED:
                    status_str = "Not Registered";
                    status_color = lv_color_hex(0xFF4444);
                    break;
                case cube32::NetworkStatus::REGISTERED_HOME:
                    status_str = "Registered (Home)";
                    status_color = lv_color_hex(0x44FF44);
                    break;
                case cube32::NetworkStatus::SEARCHING:
                    status_str = "Searching...";
                    status_color = lv_color_hex(0xFFFF44);
                    break;
                case cube32::NetworkStatus::DENIED:
                    status_str = "Denied";
                    status_color = lv_color_hex(0xFF4444);
                    break;
                case cube32::NetworkStatus::REGISTERED_ROAMING:
                    status_str = "Registered (Roaming)";
                    status_color = lv_color_hex(0x44FFFF);
                    break;
                default:
                    break;
            }
        } else {
            status_str = "Modem not ready";
            status_color = lv_color_hex(0x888888);  // Dim
        }
        
        lv_label_set_text_fmt(s_network_label, "Network: %s", status_str);
        lv_obj_set_style_text_color(s_network_label, status_color, 0);
    }
    
    // Update signal quality
    int rssi = modem.getSignalQuality();
    if (s_signal_label) {
        if (rssi != 99) {
            // Convert RSSI to dBm: dBm = -113 + (2 * rssi)
            int dbm = -113 + (2 * rssi);
            lv_label_set_text_fmt(s_signal_label, "Signal: %d dBm", dbm);
        } else {
            lv_label_set_text(s_signal_label, "Signal: N/A");
        }
    }
    update_signal_bars(rssi);
    
    // Update operator name - query when modem is initialized and not in data mode
    if (s_operator_label) {
        if (modem.isDataMode()) {
            // In data mode, can't query AT commands - keep last known value
            // Don't update the label
        } else if (modem.isInitialized() && !modem.isDataMode()) {
            // Modem initialized and in command mode - try to query operator
            std::string op_name;
            if (modem.getOperatorName(op_name) == CUBE32_OK && !op_name.empty()) {
                lv_label_set_text_fmt(s_operator_label, "Operator: %s", op_name.c_str());
            } else if (!modem.isNetworkReady()) {
                // Query failed and not registered yet
                lv_label_set_text(s_operator_label, "Operator: Registering...");
            } else {
                // Network ready but can't get operator name - might need AT+COPS?
                lv_label_set_text(s_operator_label, "Operator: (Query failed)");
            }
        } else if (modem.isInitializing()) {
            lv_label_set_text(s_operator_label, "Operator: Initializing...");
        } else {
            // Not initialized
            lv_label_set_text(s_operator_label, "Operator: ---");
        }
    }
    
    // Update IP address
    if (s_ip_label) {
        if (modem.isPPPConnected()) {
            std::string ip;
            if (modem.getIPAddress(ip) == CUBE32_OK && !ip.empty()) {
                lv_label_set_text_fmt(s_ip_label, "IP: %s", ip.c_str());
                lv_obj_set_style_text_color(s_ip_label, lv_color_hex(0x44FF44), 0);  // Green when connected
            } else {
                lv_label_set_text(s_ip_label, "IP: Connecting...");
                lv_obj_set_style_text_color(s_ip_label, lv_color_hex(0xFFFF44), 0);  // Yellow
            }
        } else if (modem.isDataMode()) {
            lv_label_set_text(s_ip_label, "IP: PPP Connecting...");
            lv_obj_set_style_text_color(s_ip_label, lv_color_hex(0xFFFF44), 0);  // Yellow
        } else if (modem.isNetworkReady()) {
            lv_label_set_text(s_ip_label, "IP: Ready (use 'data')");
            lv_obj_set_style_text_color(s_ip_label, lv_color_hex(0xCCCCCC), 0);  // Normal
        } else if (modem.isInitialized()) {
            lv_label_set_text(s_ip_label, "IP: Registering...");
            lv_obj_set_style_text_color(s_ip_label, lv_color_hex(0xFFFF44), 0);  // Yellow
        } else if (modem.isInitializing()) {
            lv_label_set_text(s_ip_label, "IP: Modem initializing...");
            lv_obj_set_style_text_color(s_ip_label, lv_color_hex(0xFFAA00), 0);  // Orange
        } else {
            lv_label_set_text(s_ip_label, "IP: Not initialized");
            lv_obj_set_style_text_color(s_ip_label, lv_color_hex(0x888888), 0);  // Dim
        }
    }
    
    // Update time
    if (s_time_label) {
        time_t now;
        struct tm timeinfo;
        time(&now);
        localtime_r(&now, &timeinfo);
        
        char time_str[32];
        strftime(time_str, sizeof(time_str), "%H:%M:%S", &timeinfo);
        lv_label_set_text(s_time_label, time_str);
    }
    
    lvgl.unlock();
}

// ============================================================================
// UI Creation
// ============================================================================

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

static void create_ui(void) {
    cube32::LvglDisplay& lvgl = cube32::LvglDisplay::instance();
    if (!lvgl.lock(100)) {
        ESP_LOGE(TAG, "Failed to lock LVGL");
        return;
    }
    
    // Get active screen
    lv_obj_t *scr = lv_screen_active();
    
    // Apply dark theme background
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x1a1a2e), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    
    // Get screen dimensions
    int32_t scr_width = lv_obj_get_width(scr);
    int32_t scr_height = lv_obj_get_height(scr);
    
    // Create main container with flex layout
    lv_obj_t *main_cont = lv_obj_create(scr);
    lv_obj_remove_style_all(main_cont);
    lv_obj_set_size(main_cont, scr_width, scr_height);
    lv_obj_set_style_bg_color(main_cont, lv_color_hex(0x1a1a2e), 0);
    lv_obj_set_style_bg_opa(main_cont, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(main_cont, 10, 0);
    lv_obj_set_flex_flow(main_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(main_cont, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_row(main_cont, 5, 0);
    
    // Title with modem icon
    lv_obj_t *title_cont = lv_obj_create(main_cont);
    lv_obj_remove_style_all(title_cont);
    lv_obj_set_size(title_cont, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(title_cont, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(title_cont, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    
    s_title_label = lv_label_create(title_cont);
    // Show connection type (USB/UART) in title
    cube32::A7670Modem& modem_ref = cube32::A7670Modem::instance();
    const char* conn_str = (modem_ref.getConnectionType() == cube32::ModemConnectionType::USB) ? "USB" : "UART";
    lv_label_set_text_fmt(s_title_label, LV_SYMBOL_CALL " A7670 [%s]", conn_str);
    lv_obj_set_style_text_color(s_title_label, lv_color_hex(0x00BFFF), 0);
    lv_obj_set_style_text_font(s_title_label, &lv_font_montserrat_24, 0);
    
    // Signal bars in title row
    create_signal_bars(title_cont);
    
    // Time label (large)
    s_time_label = lv_label_create(main_cont);
    lv_label_set_text(s_time_label, "--:--:--");
    lv_obj_set_style_text_color(s_time_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(s_time_label, &lv_font_montserrat_32, 0);
    lv_obj_set_width(s_time_label, LV_PCT(100));
    lv_obj_set_style_text_align(s_time_label, LV_TEXT_ALIGN_CENTER, 0);
    
    // Separator line
    lv_obj_t *line1 = lv_obj_create(main_cont);
    lv_obj_remove_style_all(line1);
    lv_obj_set_size(line1, LV_PCT(100), 2);
    lv_obj_set_style_bg_color(line1, lv_color_hex(0x404060), 0);
    lv_obj_set_style_bg_opa(line1, LV_OPA_COVER, 0);
    
    // Status section
    lv_obj_t *status_cont = lv_obj_create(main_cont);
    lv_obj_remove_style_all(status_cont);
    lv_obj_set_size(status_cont, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(status_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(status_cont, 4, 0);
    
    // Status label
    s_status_label = lv_label_create(status_cont);
    lv_label_set_text(s_status_label, "Status: INITIALIZING");
    lv_obj_set_style_text_color(s_status_label, lv_color_hex(0xFFAA00), 0);  // Orange for initializing
    lv_obj_set_style_text_font(s_status_label, &lv_font_montserrat_14, 0);
    
    // Mode label
    s_mode_label = lv_label_create(status_cont);
    lv_label_set_text(s_mode_label, "Mode: INITIALIZING...");
    lv_obj_set_style_text_color(s_mode_label, lv_color_hex(0xFFAA00), 0);  // Orange for initializing
    lv_obj_set_style_text_font(s_mode_label, &lv_font_montserrat_14, 0);
    
    // Network status label
    s_network_label = lv_label_create(status_cont);
    lv_label_set_text(s_network_label, "Network: Waiting...");
    lv_obj_set_style_text_color(s_network_label, lv_color_hex(0xCCCCCC), 0);
    lv_obj_set_style_text_font(s_network_label, &lv_font_montserrat_14, 0);
    
    // Signal quality label
    s_signal_label = lv_label_create(status_cont);
    lv_label_set_text(s_signal_label, "Signal: N/A");
    lv_obj_set_style_text_color(s_signal_label, lv_color_hex(0xCCCCCC), 0);
    lv_obj_set_style_text_font(s_signal_label, &lv_font_montserrat_14, 0);
    
    // Operator label
    s_operator_label = lv_label_create(status_cont);
    lv_label_set_text(s_operator_label, "Operator: ---");
    lv_obj_set_style_text_color(s_operator_label, lv_color_hex(0xCCCCCC), 0);
    lv_obj_set_style_text_font(s_operator_label, &lv_font_montserrat_14, 0);
    
    // IP address label
    s_ip_label = lv_label_create(status_cont);
    lv_label_set_text(s_ip_label, "IP: Not Connected");
    lv_obj_set_style_text_color(s_ip_label, lv_color_hex(0xCCCCCC), 0);
    lv_obj_set_style_text_font(s_ip_label, &lv_font_montserrat_14, 0);
    
    // IMEI label
    s_imei_label = lv_label_create(status_cont);
    lv_label_set_text(s_imei_label, "IMEI: ---");
    lv_obj_set_style_text_color(s_imei_label, lv_color_hex(0x888888), 0);
    lv_obj_set_style_text_font(s_imei_label, &lv_font_montserrat_14, 0);
    
    // Console hint at bottom
    lv_obj_t *hint = lv_label_create(main_cont);
    lv_label_set_text(hint, "Use serial console for AT commands");
    lv_obj_set_style_text_color(hint, lv_color_hex(0x666666), 0);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_14, 0);
    lv_obj_set_width(hint, LV_PCT(100));
    lv_obj_set_style_text_align(hint, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_flex_grow(hint, 1);
    lv_obj_set_style_pad_top(hint, 10, 0);
    
    // Create update timer
    s_update_timer = lv_timer_create(update_ui_timer_cb, UI_UPDATE_INTERVAL_MS, nullptr);
    
    lvgl.unlock();
    
    ESP_LOGI(TAG, "UI created successfully");
}

static void update_imei_label(void) {
    cube32::LvglDisplay& lvgl = cube32::LvglDisplay::instance();
    if (!lvgl.lock(100)) return;
    
    cube32::A7670Modem& modem = cube32::A7670Modem::instance();
    std::string imei;
    if (modem.getIMEI(imei) == CUBE32_OK) {
        lv_label_set_text_fmt(s_imei_label, "IMEI: %s", imei.c_str());
    }
    
    lvgl.unlock();
}

// ============================================================================
// Console Commands
// ============================================================================

// HTTP event handler
static esp_err_t http_event_handler(esp_http_client_event_t *evt) {
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            if (!esp_http_client_is_chunked_response(evt->client)) {
                // Write out data (print to console)
                printf("%.*s", evt->data_len, (char*)evt->data);
            }
            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            printf("\n");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
        case HTTP_EVENT_REDIRECT:
            ESP_LOGD(TAG, "HTTP_EVENT_REDIRECT");
            break;
    }
    return ESP_OK;
}

// HTTP GET command handler
static void cmd_httpget(const char* url) {
    cube32::A7670Modem& modem = cube32::A7670Modem::instance();
    
    // Check if network is registered
    if (!modem.isNetworkReady()) {
        printf("Error: Network not registered yet\n");
        printf("Wait for network registration to complete\n");
        return;
    }
    
    // Check if modem is in data mode (PPP)
    if (!modem.isDataMode()) {
        printf("Error: Modem is not in data mode (PPP)\n");
        printf("Switch to data mode first using 'data' command\n");
        return;
    }
    
    printf("HTTP GET: %s\n", url);
    
    esp_http_client_config_t config = {};
    config.url = url;
    config.event_handler = http_event_handler;
    config.timeout_ms = 10000;
    config.buffer_size = 1024;
    config.buffer_size_tx = 1024;
    
    // Enable HTTPS support
    if (strncmp(url, "https://", 8) == 0) {
        config.transport_type = HTTP_TRANSPORT_OVER_SSL;
        config.crt_bundle_attach = esp_crt_bundle_attach;
    }
    
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        printf("Failed to initialize HTTP client\n");
        return;
    }
    
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        int status = esp_http_client_get_status_code(client);
        int64_t content_len = esp_http_client_get_content_length(client);
        printf("\nHTTP Status: %d, Content-Length: %lld\n", status, content_len);
    } else {
        printf("\nHTTP request failed: %s\n", esp_err_to_name(err));
    }
    
    esp_http_client_cleanup(client);
}

// Ping callback
static void ping_on_success(esp_ping_handle_t hdl, void *args) {
    uint8_t ttl;
    uint16_t seqno;
    uint32_t elapsed_time, recv_len;
    ip_addr_t target_addr;
    esp_ping_get_profile(hdl, ESP_PING_PROF_SEQNO, &seqno, sizeof(seqno));
    esp_ping_get_profile(hdl, ESP_PING_PROF_TTL, &ttl, sizeof(ttl));
    esp_ping_get_profile(hdl, ESP_PING_PROF_IPADDR, &target_addr, sizeof(target_addr));
    esp_ping_get_profile(hdl, ESP_PING_PROF_SIZE, &recv_len, sizeof(recv_len));
    esp_ping_get_profile(hdl, ESP_PING_PROF_TIMEGAP, &elapsed_time, sizeof(elapsed_time));
    ESP_LOGI(TAG, "%ld bytes from %s icmp_seq=%d ttl=%d time=%ld ms",
             recv_len, inet_ntoa(target_addr.u_addr.ip4), seqno, ttl, elapsed_time);
}

static void ping_on_timeout(esp_ping_handle_t hdl, void *args) {
    uint16_t seqno;
    ip_addr_t target_addr;
    esp_ping_get_profile(hdl, ESP_PING_PROF_SEQNO, &seqno, sizeof(seqno));
    esp_ping_get_profile(hdl, ESP_PING_PROF_IPADDR, &target_addr, sizeof(target_addr));
    ESP_LOGE(TAG, "From %s icmp_seq=%d timeout", inet_ntoa(target_addr.u_addr.ip4), seqno);
}

static void ping_on_end(esp_ping_handle_t hdl, void *args) {
    ip_addr_t target_addr;
    uint32_t transmitted;
    uint32_t received;
    uint32_t total_time_ms;
    esp_ping_get_profile(hdl, ESP_PING_PROF_REQUEST, &transmitted, sizeof(transmitted));
    esp_ping_get_profile(hdl, ESP_PING_PROF_REPLY, &received, sizeof(received));
    esp_ping_get_profile(hdl, ESP_PING_PROF_IPADDR, &target_addr, sizeof(target_addr));
    esp_ping_get_profile(hdl, ESP_PING_PROF_DURATION, &total_time_ms, sizeof(total_time_ms));
    
    ESP_LOGI(TAG, "--- %s ping statistics ---", inet_ntoa(target_addr.u_addr.ip4));
    if (transmitted > 0) {
        uint32_t loss = (uint32_t)((1.0f - ((float)received) / transmitted) * 100);
        ESP_LOGI(TAG, "%ld packets transmitted, %ld received, %ld%% packet loss, time %ldms",
                 transmitted, received, loss, total_time_ms);
    } else {
        ESP_LOGI(TAG, "0 packets transmitted, 0 received");
    }
    
    // Delete the ping session in the callback (same as modem_console example)
    esp_ping_delete_session(hdl);
}

// Ping command handler (matches modem_console example)
static void cmd_ping(const char* host, int count) {
    cube32::A7670Modem& modem = cube32::A7670Modem::instance();
    
    // Check if network is registered
    if (!modem.isNetworkReady()) {
        printf("Error: Network not registered yet\n");
        printf("Wait for network registration to complete\n");
        return;
    }
    
    // Check if modem is in data mode (PPP)
    if (!modem.isDataMode()) {
        printf("Error: Modem is not in data mode (PPP)\n");
        printf("Switch to data mode first using 'data' command\n");
        return;
    }
    
    printf("PING %s (%d times)\n", host, count);
    
    // Parse IP address (same as modem_console example)
    ip_addr_t target_addr;
    struct addrinfo hint;
    struct addrinfo *res = NULL;
    memset(&hint, 0, sizeof(hint));
    memset(&target_addr, 0, sizeof(target_addr));
    
    // Convert domain name to IP address
    if (getaddrinfo(host, NULL, &hint, &res) != 0) {
        printf("ping: unknown host %s\n", host);
        return;
    }
    
    if (res->ai_family == AF_INET) {
        struct in_addr addr4 = ((struct sockaddr_in *)(res->ai_addr))->sin_addr;
        inet_addr_to_ip4addr(ip_2_ip4(&target_addr), &addr4);
    } else {
        struct in6_addr addr6 = ((struct sockaddr_in6 *)(res->ai_addr))->sin6_addr;
        inet6_addr_to_ip6addr(ip_2_ip6(&target_addr), &addr6);
    }
    freeaddrinfo(res);
    
    // Configure ping (same as modem_console example - no interface binding)
    esp_ping_config_t config = ESP_PING_DEFAULT_CONFIG();
    config.target_addr = target_addr;
    config.count = count;
    
    // Set callback functions
    esp_ping_callbacks_t cbs = {
        .cb_args = NULL,
        .on_ping_success = ping_on_success,
        .on_ping_timeout = ping_on_timeout,
        .on_ping_end = ping_on_end
    };
    
    esp_ping_handle_t ping;
    esp_ping_new_session(&config, &cbs, &ping);
    esp_ping_start(ping);
    
    // Don't wait here - ping runs asynchronously and session is deleted in on_ping_end callback
}

static void print_help(void) {
    printf("\n========================================\n");
    printf("A7670 Modem Console Commands\n");
    printf("========================================\n");
    printf("AT<cmd>    - Send AT command (e.g., AT+CSQ)\n");
    printf("status     - Print modem status\n");
    printf("info       - Print modem information\n");
    printf("netstat    - Show network registration status\n");
    printf("data       - Switch to data mode (PPP)\n");
    printf("cmd        - Switch to command mode\n");
    printf("reset      - Reset modem\n");
    printf("sync       - Sync with modem\n");
    printf("httpget <url> - HTTP GET request (http/https)\n");
    printf("ping <host> [count] - Ping host (default count=4)\n");
    printf("\n--- IO Expander Control ---\n");
    printf("iox        - Show IO Expander status\n");
    printf("pwron      - Power on modem (full sequence)\n");
    printf("pwroff     - Power off modem\n");
    printf("pwrkey on  - Assert PWRKEY (low)\n");
    printf("pwrkey off - Release PWRKEY (high)\n");
    printf("dtr <0|1>  - Set DTR pin (0=low, 1=high)\n");
    printf("pwr <0|1>  - Set power rail (0=off, 1=on)\n");
    printf("comm <usb|uart> - Set comm port\n");
    printf("\n--- General ---\n");
    printf("help       - Show this help\n");
    printf("========================================\n");
    printf("NOTE: Wait for network registration before\n");
    printf("      using 'data', 'httpget', or 'ping'\n");
    printf("========================================\n\n");
}

static void process_command(const char* cmd) {
    cube32::A7670Modem& modem = cube32::A7670Modem::instance();
    
    if (strncasecmp(cmd, "AT", 2) == 0) {
        // AT command
        std::string response;
        printf("Sending: %s\n", cmd);
        cube32_result_t ret = modem.sendCommand(cmd + 2, response, 5000);
        if (ret == CUBE32_OK) {
            printf("Response:\n%s\n", response.c_str());
        } else {
            printf("Command failed: %d\n", ret);
        }
    }
    else if (strcasecmp(cmd, "status") == 0) {
        modem.printStatus();
    }
    else if (strcasecmp(cmd, "netstat") == 0) {
        printf("\n========================================\n");
        printf("Network Status\n");
        printf("========================================\n");
        cube32::NetworkStatus net_status = modem.getNetworkStatus();
        const char* status_str = "Unknown";
        switch (net_status) {
            case cube32::NetworkStatus::NOT_REGISTERED: status_str = "Not Registered"; break;
            case cube32::NetworkStatus::REGISTERED_HOME: status_str = "Registered (Home)"; break;
            case cube32::NetworkStatus::SEARCHING: status_str = "Searching..."; break;
            case cube32::NetworkStatus::DENIED: status_str = "Registration Denied"; break;
            case cube32::NetworkStatus::REGISTERED_ROAMING: status_str = "Registered (Roaming)"; break;
            default: break;
        }
        printf("Registration: %s\n", status_str);
        printf("Network Ready: %s\n", modem.isNetworkReady() ? "Yes" : "No");
        printf("PPP Connected: %s\n", modem.isPPPConnected() ? "Yes" : "No");
        printf("Mode: %s\n", modem.isDataMode() ? "DATA" : "COMMAND");
        
        int rssi = modem.getSignalQuality();
        if (rssi != 99) {
            int dbm = -113 + (2 * rssi);
            printf("Signal: %d (%d dBm)\n", rssi, dbm);
        } else {
            printf("Signal: N/A\n");
        }
        
        std::string ip;
        if (modem.isPPPConnected() && modem.getIPAddress(ip) == CUBE32_OK) {
            printf("IP Address: %s\n", ip.c_str());
        }
        printf("========================================\n\n");
    }
    else if (strcasecmp(cmd, "info") == 0) {
        cube32::ModemInfo info;
        if (modem.getModemInfo(info) == CUBE32_OK) {
            printf("\n========================================\n");
            printf("Modem Information\n");
            printf("========================================\n");
            printf("Manufacturer: %s\n", info.manufacturer.c_str());
            printf("Model: %s\n", info.model.c_str());
            printf("Revision: %s\n", info.revision.c_str());
            printf("IMEI: %s\n", info.imei.c_str());
            printf("IMSI: %s\n", info.imsi.c_str());
            printf("Operator: %s\n", info.operatorName.c_str());
            printf("Signal: %d\n", info.signalQuality);
            printf("========================================\n\n");
        } else {
            printf("Failed to get modem info\n");
        }
    }
    else if (strcasecmp(cmd, "data") == 0) {
        if (!modem.isNetworkReady()) {
            printf("Error: Network not registered yet\n");
            printf("Wait for network registration to complete before switching to data mode\n");
            return;
        }
        printf("Switching to data mode...\n");
        if (modem.setDataMode() == CUBE32_OK) {
            printf("Data mode active\n");
        } else {
            printf("Failed to switch to data mode\n");
        }
    }
    else if (strcasecmp(cmd, "cmd") == 0) {
        printf("Switching to command mode...\n");
        if (modem.setCommandMode() == CUBE32_OK) {
            printf("Command mode active\n");
        } else {
            printf("Failed to switch to command mode\n");
        }
    }
    else if (strcasecmp(cmd, "reset") == 0) {
        printf("Resetting modem...\n");
        if (modem.reset() == CUBE32_OK) {
            printf("Modem reset successful\n");
        } else {
            printf("Modem reset failed\n");
        }
    }
    else if (strcasecmp(cmd, "sync") == 0) {
        printf("Syncing with modem...\n");
        if (modem.sync() == CUBE32_OK) {
            printf("Sync successful\n");
        } else {
            printf("Sync failed\n");
        }
    }
    else if (strncasecmp(cmd, "httpget ", 8) == 0) {
        const char* url = cmd + 8;
        // Skip leading spaces
        while (*url == ' ') url++;
        if (strlen(url) > 0) {
            cmd_httpget(url);
        } else {
            printf("Usage: httpget <url>\n");
            printf("Example: httpget http://example.com\n");
            printf("Example: httpget https://www.google.com\n");
        }
    }
    else if (strncasecmp(cmd, "ping ", 5) == 0) {
        char host[128];
        int count = 4;  // Default count
        int parsed = sscanf(cmd + 5, "%127s %d", host, &count);
        if (parsed >= 1) {
            cmd_ping(host, count);
        } else {
            printf("Usage: ping <host> [count]\n");
            printf("Example: ping 8.8.8.8\n");
            printf("Example: ping www.google.com 10\n");
        }
    }
    // --- IO Expander Commands ---
    else if (strcasecmp(cmd, "iox") == 0) {
        printf("\n========================================\n");
        printf("IO Expander Status\n");
        printf("========================================\n");
        if (modem.isIOExpanderReady()) {
            printf("Status: Initialized\n");
            printf("Power Rail: %s\n", modem.isPowerRailOn() ? "ON" : "OFF");
            printf("Comm Port: %s\n", modem.isCommPortUART() ? "UART" : "USB");
            uint8_t states;
            if (modem.getIOStates(states) == CUBE32_OK) {
                printf("Pin States: 0x%02X\n", states);
                printf("  P0 (PWRKEY): %s\n", (states & 0x01) ? "HIGH" : "LOW");
                printf("  P1 (DTR):    %s\n", (states & 0x02) ? "HIGH" : "LOW");
                printf("  P2 (PWR):    %s\n", (states & 0x04) ? "HIGH (ON)" : "LOW (OFF)");
                printf("  P3 (COMM):   %s\n", (states & 0x08) ? "HIGH (UART)" : "LOW (USB)");
            }
        } else {
            printf("Status: Not initialized\n");
        }
        printf("========================================\n\n");
    }
    else if (strcasecmp(cmd, "pwron") == 0) {
        if (!modem.isIOExpanderReady()) {
            printf("Error: IO Expander not available\n");
            return;
        }
        printf("Running modem power-on sequence...\n");
        if (modem.powerOn() == CUBE32_OK) {
            printf("Modem power-on complete\n");
        } else {
            printf("Failed to power on modem\n");
        }
    }
    else if (strcasecmp(cmd, "pwroff") == 0) {
        if (!modem.isIOExpanderReady()) {
            printf("Error: IO Expander not available\n");
            return;
        }
        printf("Running modem power-off sequence...\n");
        if (modem.powerOff() == CUBE32_OK) {
            printf("Modem power-off complete\n");
        } else {
            printf("Failed to power off modem\n");
        }
    }
    else if (strncasecmp(cmd, "pwrkey ", 7) == 0) {
        if (!modem.isIOExpanderReady()) {
            printf("Error: IO Expander not available\n");
            return;
        }
        const char* arg = cmd + 7;
        while (*arg == ' ') arg++;
        if (strcasecmp(arg, "on") == 0) {
            printf("Turn On Modem via PWRKEY ...\n");
            if (modem.powerOn() == CUBE32_OK) {
            } else {
                printf("Failed to set PWRKEY\n");
            }
        } else if (strcasecmp(arg, "off") == 0) {
            printf("Turn Off Modem via PWRKEY ...\n");
            if (modem.powerOff() == CUBE32_OK) {
                printf("PWRKEY released\n");
            } else {
                printf("Failed to set PWRKEY\n");
            }
        } else {
            printf("Usage: pwrkey <on|off>\n");
        }
    }
    else if (strncasecmp(cmd, "dtr ", 4) == 0) {
        if (!modem.isIOExpanderReady()) {
            printf("Error: IO Expander not available\n");
            return;
        }
        int val;
        if (sscanf(cmd + 4, "%d", &val) == 1) {
            bool high = (val != 0);
            printf("Setting DTR to %s...\n", high ? "HIGH" : "LOW");
            if (modem.setDTR(high) == CUBE32_OK) {
                printf("DTR set to %s\n", high ? "HIGH" : "LOW");
            } else {
                printf("Failed to set DTR\n");
            }
        } else {
            printf("Usage: dtr <0|1>\n");
        }
    }
    else if (strncasecmp(cmd, "pwr ", 4) == 0) {
        if (!modem.isIOExpanderReady()) {
            printf("Error: IO Expander not available\n");
            return;
        }
        int val;
        if (sscanf(cmd + 4, "%d", &val) == 1) {
            bool on = (val != 0);
            printf("Setting power rail to %s...\n", on ? "ON" : "OFF");
            if (modem.setPowerRail(on) == CUBE32_OK) {
                printf("Power rail %s\n", on ? "enabled" : "disabled");
            } else {
                printf("Failed to set power rail\n");
            }
        } else {
            printf("Usage: pwr <0|1>\n");
        }
    }
    else if (strncasecmp(cmd, "comm ", 5) == 0) {
        if (!modem.isIOExpanderReady()) {
            printf("Error: IO Expander not available\n");
            return;
        }
        const char* arg = cmd + 5;
        while (*arg == ' ') arg++;
        if (strcasecmp(arg, "usb") == 0) {
            printf("Setting comm port to USB...\n");
            if (modem.setCommPort(false) == CUBE32_OK) {
                printf("Comm port set to USB\n");
            } else {
                printf("Failed to set comm port\n");
            }
        } else if (strcasecmp(arg, "uart") == 0) {
            printf("Setting comm port to UART...\n");
            if (modem.setCommPort(true) == CUBE32_OK) {
                printf("Comm port set to UART\n");
            } else {
                printf("Failed to set comm port\n");
            }
        } else {
            printf("Usage: comm <usb|uart>\n");
        }
    }
    else if (strcasecmp(cmd, "help") == 0 || strcasecmp(cmd, "?") == 0) {
        print_help();
    }
    else if (strlen(cmd) > 0) {
        printf("Unknown command: %s\n", cmd);
        printf("Type 'help' for available commands\n");
    }
}

static void console_task(void *arg) {
    ESP_LOGI(TAG, "Console task started");
    
    // Configure UART for console
    const uart_port_t uart_num = (uart_port_t)CONFIG_ESP_CONSOLE_UART_NUM;
    
    // Initialize VFS & UART so we can use stdin/stdout
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    
    // Install UART driver for interrupt-driven reads and writes
    ESP_ERROR_CHECK(uart_driver_install(uart_num, 256, 0, 0, NULL, 0));
    
    // Tell VFS to use UART driver (suppress deprecation warning)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    esp_vfs_dev_uart_use_driver(uart_num);
#pragma GCC diagnostic pop
    
    // Configure linenoise
    linenoiseSetMultiLine(0);
    linenoiseSetDumbMode(1);  // Disable special sequences for simple terminal
    linenoiseHistorySetMaxLen(20);
    
    print_help();
    
    char* line;
    while (true) {
        line = linenoise("modem> ");
        if (line == nullptr) {
            // EOF or error
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        // Skip empty lines
        if (strlen(line) > 0) {
            linenoiseHistoryAdd(line);
            process_command(line);
        }
        
        linenoiseFree(line);
    }
}

// ============================================================================
// Main Application
// ============================================================================

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "CUBE32 Hello Modem USB Console Example");
    ESP_LOGI(TAG, "========================================");
    
    // Set timezone
    setenv("TZ", TIMEZONE, 1);
    tzset();
    
    // Initialize CUBE32 with all enabled peripherals
    // Modem now initializes asynchronously, so cube32_init() returns quickly
    // and the screen can display status immediately
    esp_err_t err = cube32_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize CUBE32: %s", esp_err_to_name(err));
        return;
    }

#ifndef CONFIG_CUBE32_MODEM_ENABLED
    ESP_LOGE(TAG, "Modem is not enabled in menuconfig!");
    ESP_LOGE(TAG, "Enable it at: CUBE32 Board Configuration → Modem Configuration → Enable Modem");
    return;
#endif

#ifndef CONFIG_CUBE32_LVGL_ENABLED
    ESP_LOGW(TAG, "LVGL is not enabled. Running console-only mode.");
#else
    // Create UI immediately - modem status will update as it initializes
    // This allows the screen to show status rather than being blank during startup
    create_ui();
    ESP_LOGI(TAG, "UI created - modem status will update as it initializes");
#endif
    
    // Start console task - this can run while modem is still initializing
    xTaskCreate(console_task, "console", 4096, nullptr, 5, &s_console_task_handle);
    
    ESP_LOGI(TAG, "Application started. Type 'help' in console for commands.");
    ESP_LOGI(TAG, "Modem initialization is running in background...");
    
#ifdef CONFIG_CUBE32_LVGL_ENABLED
    // Wait for modem to be at least initialized before trying to get IMEI
    // Use event group to wait efficiently instead of fixed delay
    cube32::A7670Modem& modem = cube32::A7670Modem::instance();
    EventGroupHandle_t evt_group = modem.getEventGroup();
    
    if (evt_group) {
        // Wait up to 30 seconds for modem init to complete
        EventBits_t bits = xEventGroupWaitBits(
            evt_group,
            MODEM_INIT_COMPLETE_BIT | MODEM_INIT_FAILED_BIT,
            pdFALSE,  // Don't clear bits
            pdFALSE,  // Wait for any bit
            pdMS_TO_TICKS(30000)
        );
        
        if (bits & MODEM_INIT_COMPLETE_BIT) {
            ESP_LOGI(TAG, "Modem initialized - updating IMEI label");
            update_imei_label();
        } else if (bits & MODEM_INIT_FAILED_BIT) {
            ESP_LOGW(TAG, "Modem initialization failed");
        } else {
            ESP_LOGW(TAG, "Modem initialization timeout");
        }
    }
#endif
    
    // Main loop - just keep the task alive
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
