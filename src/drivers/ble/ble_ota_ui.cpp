/**
 * @file ble_ota_ui.cpp
 * @brief CUBE32 Built-In BLE OTA UI and OTA Pop-up UI
 * 
 * This file provides:
 * 1. Built-in BLE OTA UI — Full-screen UI for bypass/double-boot mode.
 *    Shows device ID, BLE state, connection status, and restart button.
 *    OTA progress is NOT shown here; the pop-up handles it.
 * 
 * 2. OTA Pop-up UI — Modal overlay on top of any screen (works in both
 *    bypass and background modes). Shows firmware name, progress bar,
 *    bytes transferred, and restart button when OTA completes.
 * 
 * 3. OTA Monitor — Lightweight LVGL timer that watches BLE OTA state
 *    and auto-creates/destroys the OTA pop-up as needed.
 */

#include "sdkconfig.h"

#if defined(CONFIG_CUBE32_BLE_OTA_ENABLED) && defined(CONFIG_CUBE32_LVGL_ENABLED)

#include <cstdio>
#include <cinttypes>
#include <cstring>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_system.h>
#include <lvgl.h>

#include "drivers/ble/ble_ota.h"
#include "drivers/lvgl/lvgl_driver.h"

static const char *TAG = "ble_ota_ui";

// ============================================================================
// UI Configuration
// ============================================================================

#define UI_UPDATE_INTERVAL_MS       200     // Built-in UI update interval
#define MONITOR_INTERVAL_MS         300     // OTA monitor check interval
#define POPUP_UPDATE_INTERVAL_MS    150     // Pop-up update interval

// ============================================================================
// UI Colors (Dark Theme)
// ============================================================================

#define COLOR_BG            0x1a1a2e    // Dark background
#define COLOR_PANEL         0x16213e    // Panel background
#define COLOR_POPUP_BG      0x0f1524    // Pop-up background (darker)
#define COLOR_POPUP_OVERLAY 0x000000    // Overlay color (semi-transparent black)
#define COLOR_TEXT          0xeaeaea    // Primary text
#define COLOR_TEXT_DIM      0x888888    // Dimmed text
#define COLOR_ACCENT        0x00d4ff    // Accent color (cyan)
#define COLOR_SUCCESS       0x00ff88    // Success (green)
#define COLOR_WARNING       0xffaa00    // Warning (orange)
#define COLOR_ERROR         0xff4444    // Error (red)
#define COLOR_BLE_IDLE      0x666666    // BLE Idle (gray)
#define COLOR_BLE_ADV       0x00aaff    // BLE Advertising (blue)
#define COLOR_BLE_CONN      0x00ff88    // BLE Connected (green)

// ============================================================================
// Built-in UI Widgets
// ============================================================================

static lv_obj_t *s_scr = nullptr;
static lv_obj_t *s_title_label = nullptr;
static lv_obj_t *s_device_id_label = nullptr;
static lv_obj_t *s_state_indicator = nullptr;
static lv_obj_t *s_state_label = nullptr;
static lv_obj_t *s_conn_status_label = nullptr;
static lv_obj_t *s_restart_btn = nullptr;
static lv_timer_t *s_update_timer = nullptr;

static bool s_ui_initialized = false;

// ============================================================================
// OTA Pop-up Widgets
// ============================================================================

static lv_obj_t *s_popup_overlay = nullptr;         // Semi-transparent overlay
static lv_obj_t *s_popup_panel = nullptr;            // Main pop-up panel
static lv_obj_t *s_popup_title = nullptr;            // "Firmware Update" title
static lv_obj_t *s_popup_fw_name = nullptr;          // Firmware full name label
static lv_obj_t *s_popup_fw_version = nullptr;       // Firmware version label
static lv_obj_t *s_popup_progress_bar = nullptr;     // Progress bar
static lv_obj_t *s_popup_percent_label = nullptr;    // Percentage text
static lv_obj_t *s_popup_bytes_label = nullptr;      // Bytes transferred
static lv_obj_t *s_popup_complete_label = nullptr;   // "OTA Complete!" label
static lv_obj_t *s_popup_restart_btn = nullptr;      // Restart button
static lv_obj_t *s_popup_error_label = nullptr;      // Error message label
static lv_timer_t *s_popup_timer = nullptr;          // Pop-up update timer

static bool s_popup_active = false;
static bool s_popup_complete_shown = false;          // Track if complete state is shown

// ============================================================================
// OTA Monitor
// ============================================================================

static lv_timer_t *s_monitor_timer = nullptr;
static bool s_monitor_active = false;
static cube32::BleOtaState s_last_monitored_state = cube32::BleOtaState::IDLE;

// ============================================================================
// Helper Functions
// ============================================================================

static const char* get_state_text(cube32::BleOtaState state)
{
    switch (state) {
        case cube32::BleOtaState::IDLE:            return "IDLE";
        case cube32::BleOtaState::ADVERTISING:     return "ADVERTISING";
        case cube32::BleOtaState::CONNECTED:       return "CONNECTED";
        case cube32::BleOtaState::OTA_IN_PROGRESS: return "OTA IN PROGRESS";
        case cube32::BleOtaState::OTA_COMPLETE:    return "OTA COMPLETE";
        case cube32::BleOtaState::ERROR:           return "ERROR";
        case cube32::BleOtaState::BYPASSED:        return "BYPASSED";
        default:                                    return "UNKNOWN";
    }
}

static uint32_t get_state_color(cube32::BleOtaState state)
{
    switch (state) {
        case cube32::BleOtaState::IDLE:            return COLOR_BLE_IDLE;
        case cube32::BleOtaState::ADVERTISING:     return COLOR_BLE_ADV;
        case cube32::BleOtaState::CONNECTED:       return COLOR_BLE_CONN;
        case cube32::BleOtaState::OTA_IN_PROGRESS: return COLOR_WARNING;
        case cube32::BleOtaState::OTA_COMPLETE:    return COLOR_SUCCESS;
        case cube32::BleOtaState::ERROR:           return COLOR_ERROR;
        case cube32::BleOtaState::BYPASSED:        return COLOR_WARNING;
        default:                                    return COLOR_TEXT_DIM;
    }
}

// ============================================================================
// Built-in UI Update (simplified — no OTA progress widgets)
// ============================================================================

static void update_builtin_ui(lv_timer_t *timer)
{
    if (!s_ui_initialized) {
        return;
    }

    cube32::BleOta& ble = cube32::BleOta::instance();
    const cube32::BleOtaStatus& status = ble.getStatus();

    // Update state indicator color
    lv_obj_set_style_bg_color(s_state_indicator,
        lv_color_hex(get_state_color(status.state)), LV_PART_MAIN);

    // Update state label
    lv_label_set_text(s_state_label, get_state_text(status.state));
    lv_obj_set_style_text_color(s_state_label,
        lv_color_hex(get_state_color(status.state)), LV_PART_MAIN);

    // Update connection status
    if (status.is_connected) {
        lv_label_set_text(s_conn_status_label, LV_SYMBOL_OK " Connected");
        lv_obj_set_style_text_color(s_conn_status_label,
            lv_color_hex(COLOR_SUCCESS), LV_PART_MAIN);
    } else if (status.is_advertising) {
        lv_label_set_text(s_conn_status_label, LV_SYMBOL_LOOP " Waiting for connection...");
        lv_obj_set_style_text_color(s_conn_status_label,
            lv_color_hex(COLOR_BLE_ADV), LV_PART_MAIN);
    } else {
        lv_label_set_text(s_conn_status_label, LV_SYMBOL_CLOSE " Disconnected");
        lv_obj_set_style_text_color(s_conn_status_label,
            lv_color_hex(COLOR_TEXT_DIM), LV_PART_MAIN);
    }
}

// ============================================================================
// OTA Pop-up Update
// ============================================================================

static void update_ota_popup(lv_timer_t *timer)
{
    if (!s_popup_active) {
        return;
    }

    cube32::BleOta& ble = cube32::BleOta::instance();
    const cube32::BleOtaStatus& status = ble.getStatus();

    char buf[128];

    if (status.state == cube32::BleOtaState::OTA_IN_PROGRESS) {
        // Show progress elements, hide complete/error
        lv_obj_clear_flag(s_popup_progress_bar, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_popup_percent_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_popup_bytes_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_popup_complete_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_popup_restart_btn, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_popup_error_label, LV_OBJ_FLAG_HIDDEN);

        // Update progress bar
        lv_bar_set_value(s_popup_progress_bar, status.ota_progress, LV_ANIM_ON);

        // Update percentage
        snprintf(buf, sizeof(buf), "%d%%", status.ota_progress);
        lv_label_set_text(s_popup_percent_label, buf);

        // Update firmware name and version (separate lines)
        if (status.firmware_info.is_valid) {
            // Display app full name (falls back to project_name if empty)
            const char* display_name = strlen(status.firmware_info.app_full_name) > 0
                                       ? status.firmware_info.app_full_name
                                       : status.firmware_info.project_name;
            if (strlen(display_name) > 0) {
                lv_label_set_text(s_popup_fw_name, display_name);
            }
            // Display version on separate line
            if (strlen(status.firmware_info.version) > 0) {
                snprintf(buf, sizeof(buf), "v%s", status.firmware_info.version);
                lv_label_set_text(s_popup_fw_version, buf);
                lv_obj_clear_flag(s_popup_fw_version, LV_OBJ_FLAG_HIDDEN);
            }
        }

        // Update bytes transferred
        uint32_t received = status.firmware_info.received_size > 0
                            ? status.firmware_info.received_size
                            : status.ota_received_size;
        uint32_t total = status.ota_total_size;
        if (total >= 1024 * 1024) {
            snprintf(buf, sizeof(buf), "%.1f / %.1f MB",
                     received / (1024.0f * 1024.0f),
                     total / (1024.0f * 1024.0f));
        } else if (total > 0) {
            snprintf(buf, sizeof(buf), "%" PRIu32 " / %" PRIu32 " KB",
                     received / 1024, total / 1024);
        } else {
            snprintf(buf, sizeof(buf), "%" PRIu32 " bytes", received);
        }
        lv_label_set_text(s_popup_bytes_label, buf);

        s_popup_complete_shown = false;

    } else if ((status.state == cube32::BleOtaState::OTA_COMPLETE ||
                status.ota_ready_to_restart) && !s_popup_complete_shown) {
        // OTA complete — show success message and restart button
        lv_obj_add_flag(s_popup_progress_bar, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_popup_percent_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_popup_bytes_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_popup_error_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_popup_complete_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_popup_restart_btn, LV_OBJ_FLAG_HIDDEN);

        // Show final firmware info
        if (status.firmware_info.is_valid) {
            const char* display_name = strlen(status.firmware_info.app_full_name) > 0
                                       ? status.firmware_info.app_full_name
                                       : status.firmware_info.project_name;
            if (strlen(display_name) > 0) {
                lv_label_set_text(s_popup_fw_name, display_name);
                lv_obj_set_style_text_color(s_popup_fw_name,
                    lv_color_hex(COLOR_SUCCESS), LV_PART_MAIN);
            }
            if (strlen(status.firmware_info.version) > 0) {
                snprintf(buf, sizeof(buf), "New: v%s", status.firmware_info.version);
                lv_label_set_text(s_popup_fw_version, buf);
                lv_obj_set_style_text_color(s_popup_fw_version,
                    lv_color_hex(COLOR_SUCCESS), LV_PART_MAIN);
                lv_obj_clear_flag(s_popup_fw_version, LV_OBJ_FLAG_HIDDEN);
            }
        }

        s_popup_complete_shown = true;

    } else if (status.state == cube32::BleOtaState::ERROR) {
        // OTA error — show error message and restart button
        lv_obj_add_flag(s_popup_progress_bar, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_popup_percent_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_popup_bytes_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_popup_complete_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_popup_error_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_popup_restart_btn, LV_OBJ_FLAG_HIDDEN);

        if (strlen(status.ota_error_msg) > 0) {
            snprintf(buf, sizeof(buf), LV_SYMBOL_WARNING " %s", status.ota_error_msg);
        } else {
            snprintf(buf, sizeof(buf), LV_SYMBOL_WARNING " OTA Error");
        }
        lv_label_set_text(s_popup_error_label, buf);
    }
}

// ============================================================================
// Forward declarations for functions defined inside namespace cube32
// (needed by static C callbacks that are defined before the namespace block)
// ============================================================================

namespace cube32 {
void ble_ota_popup_show(void);
} // namespace cube32

// ============================================================================
// OTA Monitor Timer Callback
// ============================================================================

static void ota_monitor_callback(lv_timer_t *timer)
{
    if (!s_monitor_active) {
        return;
    }

    cube32::BleOta& ble = cube32::BleOta::instance();
    const cube32::BleOtaStatus& status = ble.getStatus();
    cube32::BleOtaState current = status.state;

    // Detect transition into OTA_IN_PROGRESS → show pop-up
    if (current == cube32::BleOtaState::OTA_IN_PROGRESS &&
        s_last_monitored_state != cube32::BleOtaState::OTA_IN_PROGRESS) {
        if (!s_popup_active) {
            ESP_LOGI(TAG, "OTA started, showing pop-up UI");
            cube32::ble_ota_popup_show();
        }
    }

    s_last_monitored_state = current;
}

// ============================================================================
// Built-in BLE OTA UI — Public API
// ============================================================================

namespace cube32 {

void ble_ota_ui_create(void)
{
    cube32::LvglDisplay& lvgl = cube32::LvglDisplay::instance();

    if (!lvgl.isInitialized()) {
        ESP_LOGW(TAG, "LVGL not initialized, skipping built-in UI");
        return;
    }

    if (!lvgl.lock(1000)) {
        ESP_LOGE(TAG, "Failed to lock LVGL mutex");
        return;
    }

    // Get screen
    s_scr = lv_screen_active();

    // Set dark background
    lv_obj_set_style_bg_color(s_scr, lv_color_hex(COLOR_BG), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_scr, LV_OPA_COVER, LV_PART_MAIN);

    // Get screen dimensions
    uint16_t scr_width = lvgl.getWidth();
    ESP_LOGI(TAG, "Creating built-in BLE OTA UI for %dx%d screen", scr_width, lvgl.getHeight());

    // ---- Title ----
    s_title_label = lv_label_create(s_scr);
    lv_label_set_text(s_title_label, LV_SYMBOL_BLUETOOTH " BLE OTA");
    lv_obj_set_style_text_color(s_title_label, lv_color_hex(COLOR_ACCENT), LV_PART_MAIN);
    lv_obj_set_style_text_font(s_title_label, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(s_title_label, LV_ALIGN_TOP_MID, 0, 6);

    // ---- Device ID (large, prominent for easy identification) ----
    s_device_id_label = lv_label_create(s_scr);
    lv_label_set_text(s_device_id_label, "");
    lv_obj_set_style_text_color(s_device_id_label, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
    lv_obj_set_style_text_font(s_device_id_label, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_align(s_device_id_label, LV_ALIGN_TOP_MID, 0, 26);

    // Set device name immediately
    cube32::BleOta& ble = cube32::BleOta::instance();
    lv_label_set_text(s_device_id_label, ble.getDeviceName());

    // ---- Separator ----
    lv_obj_t *sep1 = lv_obj_create(s_scr);
    lv_obj_set_size(sep1, scr_width - 20, 2);
    lv_obj_set_style_bg_color(sep1, lv_color_hex(COLOR_PANEL), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(sep1, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(sep1, 0, LV_PART_MAIN);
    lv_obj_align(sep1, LV_ALIGN_TOP_MID, 0, 58);

    // ---- State indicator (colored circle) + State text ----
    s_state_indicator = lv_obj_create(s_scr);
    lv_obj_set_size(s_state_indicator, 12, 12);
    lv_obj_set_style_radius(s_state_indicator, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_state_indicator, lv_color_hex(COLOR_BLE_IDLE), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_state_indicator, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_state_indicator, 0, LV_PART_MAIN);
    lv_obj_align(s_state_indicator, LV_ALIGN_TOP_LEFT, 10, 68);

    s_state_label = lv_label_create(s_scr);
    lv_label_set_text(s_state_label, "IDLE");
    lv_obj_set_style_text_color(s_state_label, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
    lv_obj_set_style_text_font(s_state_label, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align_to(s_state_label, s_state_indicator, LV_ALIGN_OUT_RIGHT_MID, 8, 0);

    // ---- Connection status ----
    s_conn_status_label = lv_label_create(s_scr);
    lv_label_set_text(s_conn_status_label, LV_SYMBOL_CLOSE " Disconnected");
    lv_obj_set_style_text_color(s_conn_status_label, lv_color_hex(COLOR_TEXT_DIM), LV_PART_MAIN);
    lv_obj_set_style_text_font(s_conn_status_label, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(s_conn_status_label, LV_ALIGN_TOP_LEFT, 10, 88);

    // ---- Restart button ----
    s_restart_btn = lv_button_create(s_scr);
    lv_obj_set_size(s_restart_btn, 100, 36);
    lv_obj_set_style_bg_color(s_restart_btn, lv_color_hex(COLOR_ERROR), LV_PART_MAIN);
    lv_obj_set_style_radius(s_restart_btn, 8, LV_PART_MAIN);
    lv_obj_align(s_restart_btn, LV_ALIGN_BOTTOM_MID, 0, -8);

    lv_obj_t *btn_label = lv_label_create(s_restart_btn);
    lv_label_set_text(btn_label, LV_SYMBOL_REFRESH " Restart");
    lv_obj_set_style_text_color(btn_label, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
    lv_obj_center(btn_label);

    lv_obj_add_event_cb(s_restart_btn, [](lv_event_t *e) {
        ESP_LOGI(TAG, "Restart button pressed");
        esp_restart();
    }, LV_EVENT_CLICKED, nullptr);

    // ---- Create update timer ----
    s_update_timer = lv_timer_create(update_builtin_ui, UI_UPDATE_INTERVAL_MS, nullptr);

    s_ui_initialized = true;
    lvgl.unlock();

    ESP_LOGI(TAG, "Built-in BLE OTA UI created successfully");
}

void ble_ota_ui_destroy(void)
{
    if (!s_ui_initialized) {
        return;
    }

    cube32::LvglDisplay& lvgl = cube32::LvglDisplay::instance();
    if (lvgl.lock(1000)) {
        if (s_update_timer) {
            lv_timer_delete(s_update_timer);
            s_update_timer = nullptr;
        }
        // Widgets are cleaned up when screen is cleared
        s_ui_initialized = false;
        lvgl.unlock();
    }

    ESP_LOGI(TAG, "Built-in BLE OTA UI destroyed");
}

bool ble_ota_ui_is_initialized(void)
{
    return s_ui_initialized;
}

// ============================================================================
// OTA Pop-up UI — Public API
// ============================================================================

void ble_ota_popup_show(void)
{
    cube32::LvglDisplay& lvgl = cube32::LvglDisplay::instance();

    if (!lvgl.isInitialized()) {
        ESP_LOGW(TAG, "LVGL not initialized, skipping OTA pop-up");
        return;
    }

    if (s_popup_active) {
        ESP_LOGW(TAG, "OTA pop-up already active");
        return;
    }

    if (!lvgl.lock(1000)) {
        ESP_LOGE(TAG, "Failed to lock LVGL mutex for pop-up");
        return;
    }

    lv_obj_t *scr = lv_screen_active();
    uint16_t scr_w = lvgl.getWidth();
    uint16_t scr_h = lvgl.getHeight();

    ESP_LOGI(TAG, "Creating OTA pop-up UI (%dx%d)", scr_w, scr_h);

    // ---- Semi-transparent overlay ----
    s_popup_overlay = lv_obj_create(scr);
    lv_obj_remove_style_all(s_popup_overlay);
    lv_obj_set_size(s_popup_overlay, scr_w, scr_h);
    lv_obj_set_style_bg_color(s_popup_overlay, lv_color_hex(COLOR_POPUP_OVERLAY), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_popup_overlay, LV_OPA_70, LV_PART_MAIN);
    lv_obj_align(s_popup_overlay, LV_ALIGN_CENTER, 0, 0);
    // Make overlay clickable to absorb touches (prevent interaction with background)
    lv_obj_add_flag(s_popup_overlay, LV_OBJ_FLAG_CLICKABLE);

    // ---- Pop-up panel ----
    uint16_t panel_w = scr_w - 20;
    uint16_t panel_h = scr_h - 40;
    s_popup_panel = lv_obj_create(s_popup_overlay);
    lv_obj_set_size(s_popup_panel, panel_w, panel_h);
    lv_obj_set_style_bg_color(s_popup_panel, lv_color_hex(COLOR_POPUP_BG), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_popup_panel, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(s_popup_panel, lv_color_hex(COLOR_ACCENT), LV_PART_MAIN);
    lv_obj_set_style_border_width(s_popup_panel, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(s_popup_panel, 12, LV_PART_MAIN);
    lv_obj_set_style_pad_all(s_popup_panel, 8, LV_PART_MAIN);
    lv_obj_align(s_popup_panel, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_scrollbar_mode(s_popup_panel, LV_SCROLLBAR_MODE_OFF);

    // ---- Title ----
    s_popup_title = lv_label_create(s_popup_panel);
    lv_label_set_text(s_popup_title, LV_SYMBOL_DOWNLOAD " Firmware Update");
    lv_obj_set_style_text_color(s_popup_title, lv_color_hex(COLOR_ACCENT), LV_PART_MAIN);
    lv_obj_set_style_text_font(s_popup_title, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(s_popup_title, LV_ALIGN_TOP_MID, 0, 4);

    // ---- Firmware name (app full name) ----
    s_popup_fw_name = lv_label_create(s_popup_panel);
    lv_label_set_text(s_popup_fw_name, "Receiving firmware...");
    lv_obj_set_style_text_color(s_popup_fw_name, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
    lv_obj_set_style_text_font(s_popup_fw_name, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_width(s_popup_fw_name, panel_w - 24);
    lv_label_set_long_mode(s_popup_fw_name, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_align(s_popup_fw_name, LV_ALIGN_TOP_MID, 0, 26);

    // ---- Firmware version (separate line, initially hidden) ----
    s_popup_fw_version = lv_label_create(s_popup_panel);
    lv_label_set_text(s_popup_fw_version, "");
    lv_obj_set_style_text_color(s_popup_fw_version, lv_color_hex(COLOR_TEXT_DIM), LV_PART_MAIN);
    lv_obj_set_style_text_font(s_popup_fw_version, &lv_font_montserrat_10, LV_PART_MAIN);
    lv_obj_align(s_popup_fw_version, LV_ALIGN_TOP_MID, 0, 44);
    lv_obj_add_flag(s_popup_fw_version, LV_OBJ_FLAG_HIDDEN);

    // ---- Percentage label (large) ----
    s_popup_percent_label = lv_label_create(s_popup_panel);
    lv_label_set_text(s_popup_percent_label, "0%");
    lv_obj_set_style_text_color(s_popup_percent_label, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
    lv_obj_set_style_text_font(s_popup_percent_label, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_align(s_popup_percent_label, LV_ALIGN_CENTER, 0, -24);

    // ---- Progress bar ----
    s_popup_progress_bar = lv_bar_create(s_popup_panel);
    lv_obj_set_size(s_popup_progress_bar, panel_w - 30, 18);
    lv_bar_set_range(s_popup_progress_bar, 0, 100);
    lv_bar_set_value(s_popup_progress_bar, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(s_popup_progress_bar, lv_color_hex(COLOR_PANEL), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_popup_progress_bar, lv_color_hex(COLOR_ACCENT), LV_PART_INDICATOR);
    lv_obj_set_style_radius(s_popup_progress_bar, 5, LV_PART_MAIN);
    lv_obj_set_style_radius(s_popup_progress_bar, 5, LV_PART_INDICATOR);
    lv_obj_align(s_popup_progress_bar, LV_ALIGN_CENTER, 0, 0);

    // ---- Bytes transferred ----
    s_popup_bytes_label = lv_label_create(s_popup_panel);
    lv_label_set_text(s_popup_bytes_label, "0 / 0 KB");
    lv_obj_set_style_text_color(s_popup_bytes_label, lv_color_hex(COLOR_TEXT_DIM), LV_PART_MAIN);
    lv_obj_set_style_text_font(s_popup_bytes_label, &lv_font_montserrat_10, LV_PART_MAIN);
    lv_obj_align(s_popup_bytes_label, LV_ALIGN_CENTER, 0, 18);

    // ---- OTA Complete label (initially hidden) ----
    s_popup_complete_label = lv_label_create(s_popup_panel);
    lv_label_set_text(s_popup_complete_label, LV_SYMBOL_OK " OTA Complete!");
    lv_obj_set_style_text_color(s_popup_complete_label, lv_color_hex(COLOR_SUCCESS), LV_PART_MAIN);
    lv_obj_set_style_text_font(s_popup_complete_label, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(s_popup_complete_label, LV_ALIGN_CENTER, 0, -20);
    lv_obj_add_flag(s_popup_complete_label, LV_OBJ_FLAG_HIDDEN);

    // ---- Error label (initially hidden) ----
    s_popup_error_label = lv_label_create(s_popup_panel);
    lv_label_set_text(s_popup_error_label, "");
    lv_obj_set_style_text_color(s_popup_error_label, lv_color_hex(COLOR_ERROR), LV_PART_MAIN);
    lv_obj_set_style_text_font(s_popup_error_label, &lv_font_montserrat_10, LV_PART_MAIN);
    lv_obj_set_width(s_popup_error_label, panel_w - 24);
    lv_label_set_long_mode(s_popup_error_label, LV_LABEL_LONG_WRAP);
    lv_obj_align(s_popup_error_label, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_flag(s_popup_error_label, LV_OBJ_FLAG_HIDDEN);

    // ---- Restart button (initially hidden, shown on OTA complete) ----
    s_popup_restart_btn = lv_button_create(s_popup_panel);
    lv_obj_set_size(s_popup_restart_btn, 120, 36);
    lv_obj_set_style_bg_color(s_popup_restart_btn, lv_color_hex(COLOR_SUCCESS), LV_PART_MAIN);
    lv_obj_set_style_radius(s_popup_restart_btn, 8, LV_PART_MAIN);
    lv_obj_align(s_popup_restart_btn, LV_ALIGN_BOTTOM_MID, 0, -4);
    lv_obj_add_flag(s_popup_restart_btn, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *restart_label = lv_label_create(s_popup_restart_btn);
    lv_label_set_text(restart_label, LV_SYMBOL_REFRESH " Restart");
    lv_obj_set_style_text_color(restart_label, lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_center(restart_label);

    lv_obj_add_event_cb(s_popup_restart_btn, [](lv_event_t *e) {
        ESP_LOGI(TAG, "OTA pop-up restart button pressed");
        esp_restart();
    }, LV_EVENT_CLICKED, nullptr);

    // ---- Create pop-up update timer ----
    s_popup_timer = lv_timer_create(update_ota_popup, POPUP_UPDATE_INTERVAL_MS, nullptr);

    s_popup_active = true;
    s_popup_complete_shown = false;
    lvgl.unlock();

    ESP_LOGI(TAG, "OTA pop-up UI created successfully");
}

void ble_ota_popup_destroy(void)
{
    if (!s_popup_active) {
        return;
    }

    cube32::LvglDisplay& lvgl = cube32::LvglDisplay::instance();
    if (lvgl.lock(1000)) {
        if (s_popup_timer) {
            lv_timer_delete(s_popup_timer);
            s_popup_timer = nullptr;
        }
        if (s_popup_overlay) {
            lv_obj_delete(s_popup_overlay);
            s_popup_overlay = nullptr;
        }
        // All children (panel, labels, etc.) are deleted with overlay
        s_popup_panel = nullptr;
        s_popup_title = nullptr;
        s_popup_fw_name = nullptr;
        s_popup_fw_version = nullptr;
        s_popup_progress_bar = nullptr;
        s_popup_percent_label = nullptr;
        s_popup_bytes_label = nullptr;
        s_popup_complete_label = nullptr;
        s_popup_restart_btn = nullptr;
        s_popup_error_label = nullptr;
        s_popup_active = false;
        s_popup_complete_shown = false;
        lvgl.unlock();
    }

    ESP_LOGI(TAG, "OTA pop-up UI destroyed");
}

bool ble_ota_popup_is_active(void)
{
    return s_popup_active;
}

// ============================================================================
// OTA Monitor — Public API
// ============================================================================

void ble_ota_monitor_start(void)
{
    if (s_monitor_active) {
        ESP_LOGW(TAG, "OTA monitor already active");
        return;
    }

    cube32::LvglDisplay& lvgl = cube32::LvglDisplay::instance();
    if (!lvgl.isInitialized()) {
        ESP_LOGW(TAG, "LVGL not initialized, skipping OTA monitor");
        return;
    }

    if (!lvgl.lock(1000)) {
        ESP_LOGE(TAG, "Failed to lock LVGL mutex for OTA monitor");
        return;
    }

    s_last_monitored_state = cube32::BleOtaState::IDLE;
    s_monitor_timer = lv_timer_create(ota_monitor_callback, MONITOR_INTERVAL_MS, nullptr);
    s_monitor_active = true;
    lvgl.unlock();

    ESP_LOGI(TAG, "OTA monitor started");
}

void ble_ota_monitor_stop(void)
{
    if (!s_monitor_active) {
        return;
    }

    cube32::LvglDisplay& lvgl = cube32::LvglDisplay::instance();
    if (lvgl.lock(1000)) {
        if (s_monitor_timer) {
            lv_timer_delete(s_monitor_timer);
            s_monitor_timer = nullptr;
        }
        s_monitor_active = false;
        lvgl.unlock();
    }

    // Also destroy any active pop-up
    ble_ota_popup_destroy();

    ESP_LOGI(TAG, "OTA monitor stopped");
}

} // namespace cube32

#else // BLE OTA or LVGL not enabled

namespace cube32 {

void ble_ota_ui_create(void) {}
void ble_ota_ui_destroy(void) {}
bool ble_ota_ui_is_initialized(void) { return false; }

void ble_ota_popup_show(void) {}
void ble_ota_popup_destroy(void) {}
bool ble_ota_popup_is_active(void) { return false; }

void ble_ota_monitor_start(void) {}
void ble_ota_monitor_stop(void) {}

} // namespace cube32

#endif // CONFIG_CUBE32_BLE_OTA_ENABLED && CONFIG_CUBE32_LVGL_ENABLED