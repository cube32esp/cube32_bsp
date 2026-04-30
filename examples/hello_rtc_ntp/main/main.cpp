/**
 * @file main.cpp
 * @brief CUBE32 Hello RTC NTP Example
 *
 * This example demonstrates:
 * - BM8563 RTC initialization and usage
 * - Network connection via WiFi or LTE Modem (A7670) for NTP sync
 *   - Uses modem when CONFIG_CUBE32_MODEM_ENABLED is set AND the A7670
 *     module is physically detected at boot (TCA9554 @ I2C 0x22)
 *   - Automatically falls back to WiFi if modem is enabled but not present
 * - NTP time synchronization with time.cloudflare.com
 * - Local time display with timezone support
 * - LVGL 9.x dark theme UI displaying:
 *   - Current date and time (large format with short month names)
 *   - Network connection status (WiFi or LTE)
 *   - NTP synchronization status
 *   - Last sync timestamp
 *
 * Timezone Configuration:
 * - Configure timezone in menuconfig:
 *   CUBE32 Board Configuration → RTC Configuration → Local Timezone
 * - Default is SGT-8 (Singapore Time, GMT+8)
 * - NTP servers always return UTC time - timezone conversion happens locally
 *
 * Prerequisites:
 * - Enable RTC in menuconfig: CUBE32 Board Configuration → RTC Configuration → Enable RTC
 * - Enable LVGL in menuconfig: CUBE32 Board Configuration → Display Configuration → Enable LVGL
 * - Configure WiFi in menuconfig: CUBE32 WiFi Configuration (always required as fallback)
 * - Optional: Enable Modem in CUBE32 Board Configuration → Modem Configuration
 * - Enable these fonts in menuconfig:
 *   - LV_FONT_MONTSERRAT_14 (for labels and status)
 *   - LV_FONT_MONTSERRAT_32 (for date display)
 *   - LV_FONT_MONTSERRAT_64 (for time display)
 */

#include <cstdio>
#include <cinttypes>
#include <cstdlib>
#include <cstring>
#include <ctime>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_sntp.h>
#include <nvs_flash.h>

#include "cube32.h"
#include "utils/hw_manifest.h"

#include <string>

static const char *TAG = "hello_rtc_ntp";

// ============================================================================
// Configuration
// ============================================================================

#define NTP_SERVER              "time.cloudflare.com"
#define NTP_SYNC_INTERVAL_SEC   3600    // Re-sync NTP every hour
#define UI_UPDATE_INTERVAL_MS   500     // Update UI every 500ms
#define WIFI_CONNECT_TIMEOUT_MS 30000   // 30 seconds timeout

// WiFi event group bits
#define WIFI_CONNECTED_BIT      BIT0
#define WIFI_FAIL_BIT           BIT1

// ============================================================================
// State Variables
// ============================================================================

static EventGroupHandle_t s_wifi_event_group = nullptr;
static int s_retry_num = 0;

// WiFi/NTP status
static bool s_wifi_connected = false;
static bool s_ntp_synced = false;
static char s_wifi_ip[32] = "N/A";
static char s_last_sync_time[32] = "Never";
static bool s_using_modem = false;

// LVGL widgets
static lv_obj_t *s_time_label = nullptr;
static lv_obj_t *s_date_label = nullptr;
static lv_obj_t *s_weekday_label = nullptr;
static lv_obj_t *s_wifi_status_label = nullptr;
static lv_obj_t *s_ntp_status_label = nullptr;
static lv_obj_t *s_sync_info_label = nullptr;

// Update timer
static lv_timer_t *s_update_timer = nullptr;

// ============================================================================
// Helper Functions
// ============================================================================

static const char* get_weekday_name(uint8_t weekday) {
    static const char* weekdays[] = {
        "Sunday", "Monday", "Tuesday", "Wednesday",
        "Thursday", "Friday", "Saturday"
    };
    return (weekday <= 6) ? weekdays[weekday] : "Unknown";
}

static const char* get_month_name(uint8_t month) {
    static const char* months[] = {
        "", "Jan", "Feb", "Mar", "Apr", "May", "Jun",
        "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
    };
    return (month >= 1 && month <= 12) ? months[month] : "Unknown";
}

// ============================================================================
// WiFi Event Handler
// ============================================================================

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_wifi_connected = false;
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
        snprintf(s_wifi_ip, sizeof(s_wifi_ip), IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Connected! IP: %s", s_wifi_ip);
        s_retry_num = 0;
        s_wifi_connected = true;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// ============================================================================
// WiFi Initialization
// ============================================================================

static esp_err_t wifi_init_sta(void) {
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
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

    // Wait for connection or failure
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
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

// ============================================================================
// Network Initialization (WiFi or Modem)
// ============================================================================

static esp_err_t init_network(void) {
#ifdef CONFIG_CUBE32_MODEM_ENABLED
    // Use modem only when the module is physically detected (TCA9554 @ 0x22)
    const cube32_hw_manifest_t* hw = cube32_hw_manifest();
    if (hw->modem_module_present) {
        ESP_LOGI(TAG, "Modem module detected — using LTE for NTP synchronization...");

        cube32::A7670Modem& modem = cube32::A7670Modem::instance();

        int timeout = 60;
        while (!modem.isInitialized() && timeout > 0) {
            ESP_LOGI(TAG, "Waiting for modem initialization... (%d)", timeout);
            vTaskDelay(pdMS_TO_TICKS(1000));
            timeout--;
        }
        if (!modem.isInitialized()) {
            ESP_LOGE(TAG, "Modem failed to initialize");
            return ESP_FAIL;
        }

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

        ESP_LOGI(TAG, "Switching to data mode (PPP)...");
        if (modem.setDataMode() != CUBE32_OK) {
            ESP_LOGE(TAG, "Failed to enable data mode");
            return ESP_FAIL;
        }

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

        // Get IP address from modem
        std::string ip;
        timeout = 10;
        while (timeout > 0) {
            if (modem.getIPAddress(ip) == CUBE32_OK && !ip.empty() && ip != "0.0.0.0") {
                strncpy(s_wifi_ip, ip.c_str(), sizeof(s_wifi_ip) - 1);
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(500));
            timeout--;
        }
        if (strcmp(s_wifi_ip, "N/A") == 0 || strcmp(s_wifi_ip, "0.0.0.0") == 0) {
            esp_netif_ip_info_t ip_info;
            esp_netif_t *netif = esp_netif_get_handle_from_ifkey("PPP_DEF");
            if (netif && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
                snprintf(s_wifi_ip, sizeof(s_wifi_ip), IPSTR, IP2STR(&ip_info.ip));
            }
        }

        s_wifi_connected = true;
        s_using_modem = true;
        ESP_LOGI(TAG, "Modem connected! IP: %s", s_wifi_ip);
        return ESP_OK;
    }

    ESP_LOGW(TAG, "Modem enabled in Kconfig but module not detected — falling back to WiFi");
#endif
    ESP_LOGI(TAG, "Using WiFi for internet connection...");
    return wifi_init_sta();
}

// ============================================================================
// NTP Synchronization
// ============================================================================

static void ntp_sync_notification_cb(struct timeval *tv) {
    ESP_LOGI(TAG, "NTP time synchronized!");
    s_ntp_synced = true;
    
    // Get current time and format for display
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    strftime(s_last_sync_time, sizeof(s_last_sync_time), "%H:%M:%S", &timeinfo);
    
    // Update BM8563 RTC with synchronized time
#if CONFIG_CUBE32_RTC_ENABLED
    cube32::RTC& rtc = cube32::RTC::instance();
    cube32::RTCDateTime dt;
    dt.date.year = timeinfo.tm_year + 1900;
    dt.date.month = timeinfo.tm_mon + 1;
    dt.date.day = timeinfo.tm_mday;
    dt.date.weekday = timeinfo.tm_wday;
    dt.time.hour = timeinfo.tm_hour;
    dt.time.minute = timeinfo.tm_min;
    dt.time.second = timeinfo.tm_sec;
    
    if (rtc.setDateTime(dt) == CUBE32_OK) {
        ESP_LOGI(TAG, "RTC updated with NTP time: %04d-%02d-%02d %02d:%02d:%02d",
                 dt.date.year, dt.date.month, dt.date.day,
                 dt.time.hour, dt.time.minute, dt.time.second);
    } else {
        ESP_LOGE(TAG, "Failed to update RTC with NTP time");
    }
#endif
}

static void ntp_init(void) {
    ESP_LOGI(TAG, "Initializing NTP with server: %s", NTP_SERVER);
#if CONFIG_CUBE32_RTC_ENABLED
    cube32::RTC& rtc = cube32::RTC::instance();
    ESP_LOGI(TAG, "Timezone: %s", rtc.getTimezone());
#endif
    
    // NTP always returns UTC time - timezone conversion happens locally
    // Timezone is already set by RTC driver during initialization
    
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, NTP_SERVER);
    esp_sntp_set_time_sync_notification_cb(ntp_sync_notification_cb);
    esp_sntp_init();
}

// ============================================================================
// LVGL UI Creation
// ============================================================================

static void update_ui_timer_cb(lv_timer_t *timer) {
    (void)timer;
    
    cube32::LvglDisplay& lvgl = cube32::LvglDisplay::instance();
    if (!lvgl.lock(10)) {
        return;
    }
    
    // Get current time from RTC or system
#if CONFIG_CUBE32_RTC_ENABLED
    cube32::RTC& rtc = cube32::RTC::instance();
    cube32::RTCDateTime dt;
    if (rtc.getDateTime(dt) == CUBE32_OK) {
        // Update time label
        if (s_time_label) {
            char time_str[16];
            snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d",
                     dt.time.hour, dt.time.minute, dt.time.second);
            lv_label_set_text(s_time_label, time_str);
        }
        
        // Update date label
        if (s_date_label) {
            char date_str[32];
            snprintf(date_str, sizeof(date_str), "%s %d, %04d",
                     get_month_name(dt.date.month), dt.date.day, dt.date.year);
            lv_label_set_text(s_date_label, date_str);
        }
        
        // Update weekday label
        if (s_weekday_label) {
            lv_label_set_text(s_weekday_label, get_weekday_name(dt.date.weekday));
        }
    }
#else
    // Fallback to system time
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    
    if (s_time_label) {
        char time_str[16];
        snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d",
                 timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
        lv_label_set_text(s_time_label, time_str);
    }
    
    if (s_date_label) {
        char date_str[32];
        snprintf(date_str, sizeof(date_str), "%s %d, %04d",
                 get_month_name(timeinfo.tm_mon + 1), timeinfo.tm_mday, timeinfo.tm_year + 1900);
        lv_label_set_text(s_date_label, date_str);
    }
    
    if (s_weekday_label) {
        lv_label_set_text(s_weekday_label, get_weekday_name(timeinfo.tm_wday));
    }
#endif
    
    // Update network status
    if (s_wifi_status_label) {
        if (s_wifi_connected) {
            char net_str[64];
            if (s_using_modem) {
                snprintf(net_str, sizeof(net_str), LV_SYMBOL_CALL "  LTE: %s", s_wifi_ip);
            } else {
                snprintf(net_str, sizeof(net_str), LV_SYMBOL_WIFI "  Connected: %s", s_wifi_ip);
            }
            lv_label_set_text(s_wifi_status_label, net_str);
            lv_obj_set_style_text_color(s_wifi_status_label, lv_color_hex(0x00E676), LV_PART_MAIN);
        } else {
            lv_label_set_text(s_wifi_status_label,
                s_using_modem ? LV_SYMBOL_CALL "  Connecting LTE..." : LV_SYMBOL_WIFI "  Disconnected");
            lv_obj_set_style_text_color(s_wifi_status_label, lv_color_hex(0xFF5252), LV_PART_MAIN);
        }
    }
    
    // Update NTP status
    if (s_ntp_status_label) {
        if (s_ntp_synced) {
            lv_label_set_text(s_ntp_status_label, LV_SYMBOL_OK "  NTP Synced");
            lv_obj_set_style_text_color(s_ntp_status_label, lv_color_hex(0x00E676), LV_PART_MAIN);
        } else {
            lv_label_set_text(s_ntp_status_label, LV_SYMBOL_REFRESH "  Waiting for NTP...");
            lv_obj_set_style_text_color(s_ntp_status_label, lv_color_hex(0xFFD740), LV_PART_MAIN);
        }
    }
    
    // Update sync info
    if (s_sync_info_label) {
        char sync_str[64];
        snprintf(sync_str, sizeof(sync_str), "Last sync: %s", s_last_sync_time);
        lv_label_set_text(s_sync_info_label, sync_str);
    }
    
    lvgl.unlock();
}

static void create_ui(void) {
    cube32::LvglDisplay& lvgl = cube32::LvglDisplay::instance();
    
    if (!lvgl.lock(1000)) {
        ESP_LOGE(TAG, "Failed to lock LVGL mutex");
        return;
    }

    // Get active screen
    lv_obj_t *scr = lv_screen_active();
    
    // Set dark theme background
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x121212), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);

    // ========================================================================
    // Header - Title
    // ========================================================================
    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "CUBE32 RTC + NTP");
    lv_obj_set_style_text_color(title, lv_color_hex(0x64B5F6), LV_PART_MAIN);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 5);

    // ========================================================================
    // Main Time Display (Large)
    // ========================================================================
    s_time_label = lv_label_create(scr);
    lv_label_set_text(s_time_label, "00:00:00");
    lv_obj_set_style_text_color(s_time_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
#if LV_FONT_MONTSERRAT_64
    lv_obj_set_style_text_font(s_time_label, &lv_font_montserrat_64, LV_PART_MAIN);
    lv_obj_align(s_time_label, LV_ALIGN_TOP_MID, 0, 22);
#elif LV_FONT_MONTSERRAT_48
    lv_obj_set_style_text_font(s_time_label, &lv_font_montserrat_48, LV_PART_MAIN);
    lv_obj_align(s_time_label, LV_ALIGN_TOP_MID, 0, 22);
#elif LV_FONT_MONTSERRAT_24
    lv_obj_set_style_text_font(s_time_label, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_align(s_time_label, LV_ALIGN_TOP_MID, 0, 22);
#else
    lv_obj_set_style_text_font(s_time_label, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(s_time_label, LV_ALIGN_TOP_MID, 0, 22);
#endif

    // ========================================================================
    // Date Display
    // ========================================================================
    s_date_label = lv_label_create(scr);
    lv_label_set_text(s_date_label, "Jan 1, 2024");
    lv_obj_set_style_text_color(s_date_label, lv_color_hex(0xBBBBBB), LV_PART_MAIN);
#if LV_FONT_MONTSERRAT_32
    lv_obj_set_style_text_font(s_date_label, &lv_font_montserrat_32, LV_PART_MAIN);
    lv_obj_align(s_date_label, LV_ALIGN_TOP_MID, 0, 88);
#elif LV_FONT_MONTSERRAT_24
    lv_obj_set_style_text_font(s_date_label, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_align(s_date_label, LV_ALIGN_TOP_MID, 0, 80);
#else
    lv_obj_set_style_text_font(s_date_label, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(s_date_label, LV_ALIGN_TOP_MID, 0, 72);
#endif

    // ========================================================================
    // Weekday Display
    // ========================================================================
    s_weekday_label = lv_label_create(scr);
    lv_label_set_text(s_weekday_label, "Monday");
    lv_obj_set_style_text_color(s_weekday_label, lv_color_hex(0x888888), LV_PART_MAIN);
    lv_obj_set_style_text_font(s_weekday_label, &lv_font_montserrat_14, LV_PART_MAIN);
#if LV_FONT_MONTSERRAT_32
    lv_obj_align(s_weekday_label, LV_ALIGN_TOP_MID, 0, 125);
#elif LV_FONT_MONTSERRAT_24
    lv_obj_align(s_weekday_label, LV_ALIGN_TOP_MID, 0, 110);
#else
    lv_obj_align(s_weekday_label, LV_ALIGN_TOP_MID, 0, 100);
#endif

    // ========================================================================
    // Divider Line
    // ========================================================================
    lv_obj_t *divider = lv_obj_create(scr);
    lv_obj_set_size(divider, 200, 2);
    lv_obj_set_style_bg_color(divider, lv_color_hex(0x333333), LV_PART_MAIN);
    lv_obj_set_style_border_width(divider, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(divider, 0, LV_PART_MAIN);
#if LV_FONT_MONTSERRAT_32
    lv_obj_align(divider, LV_ALIGN_TOP_MID, 0, 145);
#elif LV_FONT_MONTSERRAT_24
    lv_obj_align(divider, LV_ALIGN_TOP_MID, 0, 130);
#else
    lv_obj_align(divider, LV_ALIGN_TOP_MID, 0, 118);
#endif

    // ========================================================================
    // Status Panel
    // ========================================================================
    
    // WiFi Status
    s_wifi_status_label = lv_label_create(scr);
    lv_label_set_text(s_wifi_status_label, LV_SYMBOL_WIFI "  Connecting...");
    lv_obj_set_style_text_color(s_wifi_status_label, lv_color_hex(0xFFD740), LV_PART_MAIN);
    lv_obj_set_style_text_font(s_wifi_status_label, &lv_font_montserrat_14, LV_PART_MAIN);
#if LV_FONT_MONTSERRAT_32
    lv_obj_align(s_wifi_status_label, LV_ALIGN_TOP_MID, 0, 155);
#elif LV_FONT_MONTSERRAT_24
    lv_obj_align(s_wifi_status_label, LV_ALIGN_TOP_MID, 0, 142);
#else
    lv_obj_align(s_wifi_status_label, LV_ALIGN_TOP_MID, 0, 128);
#endif

    // NTP Status
    s_ntp_status_label = lv_label_create(scr);
    lv_label_set_text(s_ntp_status_label, LV_SYMBOL_REFRESH "  Waiting for network...");
    lv_obj_set_style_text_color(s_ntp_status_label, lv_color_hex(0x888888), LV_PART_MAIN);
    lv_obj_set_style_text_font(s_ntp_status_label, &lv_font_montserrat_14, LV_PART_MAIN);
#if LV_FONT_MONTSERRAT_32
    lv_obj_align(s_ntp_status_label, LV_ALIGN_TOP_MID, 0, 175);
#elif LV_FONT_MONTSERRAT_24
    lv_obj_align(s_ntp_status_label, LV_ALIGN_TOP_MID, 0, 162);
#else
    lv_obj_align(s_ntp_status_label, LV_ALIGN_TOP_MID, 0, 148);
#endif

    // Last Sync Info
    s_sync_info_label = lv_label_create(scr);
    lv_label_set_text(s_sync_info_label, "Last sync: Never");
    lv_obj_set_style_text_color(s_sync_info_label, lv_color_hex(0x666666), LV_PART_MAIN);
    lv_obj_set_style_text_font(s_sync_info_label, &lv_font_montserrat_14, LV_PART_MAIN);
#if LV_FONT_MONTSERRAT_32
    lv_obj_align(s_sync_info_label, LV_ALIGN_TOP_MID, 0, 195);
#elif LV_FONT_MONTSERRAT_24
    lv_obj_align(s_sync_info_label, LV_ALIGN_TOP_MID, 0, 182);
#else
    lv_obj_align(s_sync_info_label, LV_ALIGN_TOP_MID, 0, 168);
#endif

    // ========================================================================
    // Footer
    // ========================================================================
    lv_obj_t *footer = lv_label_create(scr);
    char footer_text[64];
    snprintf(footer_text, sizeof(footer_text), "NTP: %s", NTP_SERVER);
    lv_label_set_text(footer, footer_text);
    lv_obj_set_style_text_color(footer, lv_color_hex(0x444444), LV_PART_MAIN);
    lv_obj_set_style_text_font(footer, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(footer, LV_ALIGN_BOTTOM_MID, 0, -10);

    // ========================================================================
    // Create update timer
    // ========================================================================
    s_update_timer = lv_timer_create(update_ui_timer_cb, UI_UPDATE_INTERVAL_MS, nullptr);

    lvgl.unlock();

    ESP_LOGI(TAG, "UI created successfully");
}

// ============================================================================
// Main Entry Point
// ============================================================================

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " CUBE32 Hello RTC NTP Example");
    ESP_LOGI(TAG, "========================================");

    // Initialize CUBE32 board (includes RTC, Display, LVGL if enabled)
    esp_err_t ret = cube32_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize CUBE32: %d", ret);
        return;
    }

    // Create the UI (LVGL is already initialized by cube32_init)
#if CONFIG_CUBE32_LVGL_ENABLED
    cube32::LvglDisplay& lvgl = cube32::LvglDisplay::instance();
    if (lvgl.isInitialized()) {
        ESP_LOGI(TAG, "LVGL display ready (%dx%d)", lvgl.getWidth(), lvgl.getHeight());
        create_ui();
    } else {
        ESP_LOGW(TAG, "LVGL failed to initialize during cube32_init");
    }
#else
    ESP_LOGW(TAG, "LVGL is not enabled. Enable it in menuconfig for UI display.");
#endif

    // Verify RTC is working
#if CONFIG_CUBE32_RTC_ENABLED
    cube32::RTC& rtc = cube32::RTC::instance();
    cube32::RTCDateTime dt;
    if (rtc.getDateTime(dt) == CUBE32_OK) {
        ESP_LOGI(TAG, "RTC current time: %04d-%02d-%02d %02d:%02d:%02d",
                 dt.date.year, dt.date.month, dt.date.day,
                 dt.time.hour, dt.time.minute, dt.time.second);
    } else {
        ESP_LOGW(TAG, "Failed to read RTC time");
    }
#else
    ESP_LOGW(TAG, "RTC is not enabled. Enable it in menuconfig.");
#endif

    // Initialize network (modem if present and enabled, else WiFi)
    ESP_LOGI(TAG, "Initializing network connection...");
    esp_err_t net_ret = init_network();

    // Initialize NTP if network is connected
    if (net_ret == ESP_OK) {
        ntp_init();
        ESP_LOGI(TAG, "NTP initialized, waiting for synchronization...");
    } else {
        ESP_LOGW(TAG, "Network not connected, NTP sync will not be available");
    }

    // Main loop - just keep the task alive
    ESP_LOGI(TAG, "Application running. Ctrl+C to exit.");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
