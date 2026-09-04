/**
 * @file main.cpp
 * @brief CUBE32 Hello LVGL Example
 * 
 * This example demonstrates LVGL initialization and basic widget usage
 * on the CUBE32 board with a supported TFT display (ST7789 or ST7796S,
 * auto-detected) and touch input.
 * 
 * Features demonstrated:
 * - LVGL initialization using cube32::LvglDisplay
 * - Touch input integration (if enabled)
 * - Creating basic LVGL widgets (labels, buttons, styles)
 * - Thread-safe LVGL access with lock/unlock
 * - Custom styling and animations
 * - Swipe gesture to rotate screen (swipe left/right)
 * 
 * Prerequisites:
 * - Enable LVGL in menuconfig: CUBE32 Board Configuration → Display Configuration → Enable LVGL
 * - Enable Touch in menuconfig: CUBE32 Board Configuration → Touch Configuration → Enable Touch
 */

#include <cstdio>
#include <cinttypes>
#include <cstdlib>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_timer.h>

#include "cube32.h"

static const char *TAG = "hello_lvgl";

// Widget handles for updates
static lv_obj_t *s_counter_label = nullptr;
static lv_obj_t *s_touch_label = nullptr;
static lv_obj_t *s_rotation_label = nullptr;
static int s_counter = 0;

// Rotation state
static uint16_t s_current_rotation = 0;
static const uint16_t s_rotations[] = {0, 90, 180, 270};
static int s_rotation_index = 0;

// Swipe gesture detection
static lv_point_t s_touch_start = {0, 0};
static bool s_touch_active = false;
static const int SWIPE_THRESHOLD = 50;  // Minimum pixels to trigger swipe

// Forward declaration
static void update_rotation_label(void);
static void recreate_ui(void);

/**
 * @brief Rotate screen to next rotation angle
 */
static void rotate_screen_next(void) {
    s_rotation_index = (s_rotation_index + 1) % 4;
    s_current_rotation = s_rotations[s_rotation_index];
    
    ESP_LOGI(TAG, "Rotating screen to %d degrees", s_current_rotation);
    
    cube32::LvglDisplay& lvgl = cube32::LvglDisplay::instance();
    cube32_result_t ret = lvgl.setRotation(s_current_rotation);
    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to set rotation: %d", ret);
        return;
    }
    
    // Recreate UI for new dimensions
    recreate_ui();
}

/**
 * @brief Rotate screen to previous rotation angle
 */
static void rotate_screen_prev(void) {
    s_rotation_index = (s_rotation_index + 3) % 4;  // +3 is same as -1 mod 4
    s_current_rotation = s_rotations[s_rotation_index];
    
    ESP_LOGI(TAG, "Rotating screen to %d degrees", s_current_rotation);
    
    cube32::LvglDisplay& lvgl = cube32::LvglDisplay::instance();
    cube32_result_t ret = lvgl.setRotation(s_current_rotation);
    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to set rotation: %d", ret);
        return;
    }
    
    // Recreate UI for new dimensions
    recreate_ui();
}

/**
 * @brief Screen gesture event callback for swipe detection
 */
static void screen_gesture_cb(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *scr = (lv_obj_t *)lv_event_get_target(e);
    
    if (code == LV_EVENT_PRESSED) {
        // Record touch start position
        lv_indev_t *indev = lv_indev_active();
        if (indev) {
            lv_indev_get_point(indev, &s_touch_start);
            s_touch_active = true;
        }
    }
    else if (code == LV_EVENT_RELEASED && s_touch_active) {
        // Check for swipe gesture
        lv_indev_t *indev = lv_indev_active();
        if (indev) {
            lv_point_t touch_end;
            lv_indev_get_point(indev, &touch_end);
            
            int dx = touch_end.x - s_touch_start.x;
            int dy = touch_end.y - s_touch_start.y;
            
            // Check for horizontal swipe (ignore if mostly vertical)
            if (abs(dx) > SWIPE_THRESHOLD && abs(dx) > abs(dy)) {
                if (dx > 0) {
                    ESP_LOGI(TAG, "Swipe right detected - rotating to next");
                    rotate_screen_next();
                } else {
                    ESP_LOGI(TAG, "Swipe left detected - rotating to previous");
                    rotate_screen_prev();
                }
            }
        }
        s_touch_active = false;
    }
}

/**
 * @brief Button click event callback
 */
static void btn_event_cb(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    (void)lv_event_get_target(e);  // Unused for now
    
    if (code == LV_EVENT_CLICKED) {
        s_counter++;
        ESP_LOGI(TAG, "Button clicked! Count: %d", s_counter);
        
        // Update counter label
        if (s_counter_label) {
            char buf[32];
            snprintf(buf, sizeof(buf), "Count: %d", s_counter);
            lv_label_set_text(s_counter_label, buf);
        }
    }
}

/**
 * @brief Create the main UI
 */
void create_ui(void) {
    cube32::LvglDisplay& lvgl = cube32::LvglDisplay::instance();
    
    if (!lvgl.lock(1000)) {
        ESP_LOGE(TAG, "Failed to lock LVGL mutex");
        return;
    }

    // Get screen dimensions
    uint16_t scr_width = lvgl.getWidth();
    uint16_t scr_height = lvgl.getHeight();
    ESP_LOGI(TAG, "Creating UI for %dx%d screen", scr_width, scr_height);

    // Get active screen
    lv_obj_t *scr = lv_screen_active();
    
    // Set screen background color
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x1a1a2e), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);

    // Create title label
    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "CUBE32");
    lv_obj_set_style_text_color(title, lv_color_hex(0x00d4ff), LV_PART_MAIN);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

    // Create subtitle label
    lv_obj_t *subtitle = lv_label_create(scr);
    lv_label_set_text(subtitle, "LVGL + Touch Demo");
    lv_obj_set_style_text_color(subtitle, lv_color_hex(0xaaaaaa), LV_PART_MAIN);
    lv_obj_set_style_text_font(subtitle, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(subtitle, LV_ALIGN_TOP_MID, 0, 40);

    // Create a styled button
    lv_obj_t *btn = lv_button_create(scr);
    lv_obj_set_size(btn, 120, 50);
    lv_obj_align(btn, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_CLICKED, nullptr);

    // Style the button
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x4a00e0), LV_PART_MAIN);
    lv_obj_set_style_bg_grad_color(btn, lv_color_hex(0x8e2de2), LV_PART_MAIN);
    lv_obj_set_style_bg_grad_dir(btn, LV_GRAD_DIR_HOR, LV_PART_MAIN);
    lv_obj_set_style_radius(btn, 10, LV_PART_MAIN);
    lv_obj_set_style_shadow_width(btn, 10, LV_PART_MAIN);
    lv_obj_set_style_shadow_color(btn, lv_color_hex(0x4a00e0), LV_PART_MAIN);
    lv_obj_set_style_shadow_opa(btn, LV_OPA_50, LV_PART_MAIN);

    // Pressed state style
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x3a00c0), LV_STATE_PRESSED);
    lv_obj_set_style_shadow_ofs_y(btn, 3, LV_STATE_PRESSED);

    // Button label
    lv_obj_t *btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Click Me!");
    lv_obj_set_style_text_color(btn_label, lv_color_white(), LV_PART_MAIN);
    lv_obj_center(btn_label);

    // Create counter label
    s_counter_label = lv_label_create(scr);
    lv_label_set_text(s_counter_label, "Count: 0");
    lv_obj_set_style_text_color(s_counter_label, lv_color_hex(0x00ff88), LV_PART_MAIN);
    lv_obj_set_style_text_font(s_counter_label, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(s_counter_label, LV_ALIGN_CENTER, 0, 50);

    // Create touch status label
    s_touch_label = lv_label_create(scr);
    if (lvgl.hasTouchInput()) {
        lv_label_set_text(s_touch_label, "Swipe to rotate");
        lv_obj_set_style_text_color(s_touch_label, lv_color_hex(0x00ff00), LV_PART_MAIN);
    } else {
        lv_label_set_text(s_touch_label, "Touch: N/A");
        lv_obj_set_style_text_color(s_touch_label, lv_color_hex(0xff6600), LV_PART_MAIN);
    }
    lv_obj_set_style_text_font(s_touch_label, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(s_touch_label, LV_ALIGN_CENTER, 0, 75);

    // Create rotation indicator label
    s_rotation_label = lv_label_create(scr);
    char rot_text[32];
    snprintf(rot_text, sizeof(rot_text), "Rotation: %d", s_current_rotation);
    lv_label_set_text(s_rotation_label, rot_text);
    lv_obj_set_style_text_color(s_rotation_label, lv_color_hex(0xffaa00), LV_PART_MAIN);
    lv_obj_set_style_text_font(s_rotation_label, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(s_rotation_label, LV_ALIGN_CENTER, 0, 95);

    // Add gesture event handlers to screen for swipe detection
    if (lvgl.hasTouchInput()) {
        lv_obj_add_event_cb(scr, screen_gesture_cb, LV_EVENT_PRESSED, nullptr);
        lv_obj_add_event_cb(scr, screen_gesture_cb, LV_EVENT_RELEASED, nullptr);
    }

    // Create footer with version info
    lv_obj_t *footer = lv_label_create(scr);
    char footer_text[64];
    snprintf(footer_text, sizeof(footer_text), "BSP v%s | LVGL v%d.%d.%d",
             cube32_get_version(),
             LVGL_VERSION_MAJOR, LVGL_VERSION_MINOR, LVGL_VERSION_PATCH);
    lv_label_set_text(footer, footer_text);
    lv_obj_set_style_text_color(footer, lv_color_hex(0x666666), LV_PART_MAIN);
    lv_obj_set_style_text_font(footer, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(footer, LV_ALIGN_BOTTOM_MID, 0, -10);

    lvgl.unlock();

    ESP_LOGI(TAG, "UI created successfully");
}

/**
 * @brief Update the rotation label text
 */
static void update_rotation_label(void) {
    if (s_rotation_label) {
        char rot_text[32];
        snprintf(rot_text, sizeof(rot_text), "Rotation: %d", s_current_rotation);
        lv_label_set_text(s_rotation_label, rot_text);
    }
}

/**
 * @brief Recreate UI after rotation change
 */
static void recreate_ui(void) {
    cube32::LvglDisplay& lvgl = cube32::LvglDisplay::instance();
    
    if (!lvgl.lock(1000)) {
        ESP_LOGE(TAG, "Failed to lock LVGL mutex for UI recreation");
        return;
    }

    // Clear current screen content
    lv_obj_t *scr = lv_screen_active();
    lv_obj_clean(scr);
    
    // Reset widget handles
    s_counter_label = nullptr;
    s_touch_label = nullptr;
    s_rotation_label = nullptr;
    
    lvgl.unlock();
    
    // Recreate the UI with new dimensions
    create_ui();
    
    // Restore the counter value display
    if (lvgl.lock(1000)) {
        if (s_counter_label) {
            char buf[32];
            snprintf(buf, sizeof(buf), "Count: %d", s_counter);
            lv_label_set_text(s_counter_label, buf);
        }
        lvgl.unlock();
    }
}

/**
 * @brief Animate the counter label (demonstration of LVGL animation)
 */
void animate_counter(void) {
    cube32::LvglDisplay& lvgl = cube32::LvglDisplay::instance();
    
    if (!lvgl.lock(100)) {
        return;
    }

    if (s_counter_label) {
        // Create a simple pulsing animation
        static bool growing = true;
        static int scale = 100;
        
        if (growing) {
            scale += 5;
            if (scale >= 120) growing = false;
        } else {
            scale -= 5;
            if (scale <= 100) growing = true;
        }
        
        // Note: Scale transform requires LVGL9 with transform support enabled
        // For basic demo, we just update the text color periodically
        uint8_t hue = (esp_timer_get_time() / 10000) % 360;
        lv_obj_set_style_text_color(s_counter_label, 
            lv_color_hsv_to_rgb(hue, 80, 100), LV_PART_MAIN);
    }

    lvgl.unlock();
}

/**
 * @brief Main application entry point
 */
extern "C" void app_main(void) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "CUBE32 Hello LVGL Example");
    ESP_LOGI(TAG, "========================================");

    // Initialize CUBE32 board (this initializes display and LVGL if enabled)
    esp_err_t ret = cube32_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize CUBE32: %s", esp_err_to_name(ret));
        return;
    }

    // Check if LVGL is available
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

    // Log touch status
    if (lvgl.hasTouchInput()) {
        ESP_LOGI(TAG, "Touch input: Enabled");
        ESP_LOGI(TAG, "Swipe left/right to rotate screen!");
    } else {
        ESP_LOGI(TAG, "Touch input: Not available (enable in menuconfig)");
    }

    // Get initial rotation from LVGL display (not the underlying display
    // driver directly, since LVGL handles rotation)
    s_current_rotation = lvgl.getRotation();
    for (int i = 0; i < 4; i++) {
        if (s_rotations[i] == s_current_rotation) {
            s_rotation_index = i;
            break;
        }
    }

    // Create the UI
    create_ui();

    // Main loop - animate and update UI
    ESP_LOGI(TAG, "Starting main loop...");
    ESP_LOGI(TAG, "Touch the button to increment counter!");
    ESP_LOGI(TAG, "Swipe left/right to rotate screen (0->90->180->270->0)");
    while (1) {
        // Animate the counter label with color changes
        animate_counter();
        
        // Small delay - LVGL has its own task for rendering
        vTaskDelay(pdMS_TO_TICKS(50));
    }
#endif
}
