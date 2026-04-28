/**
 * @file main.cpp
 * @brief CUBE32 Hello Display Example
 * 
 * This example demonstrates basic ST7789 TFT display initialization and
 * drawing operations on the CUBE32 board.
 * 
 * Features demonstrated:
 * - SPI bus initialization
 * - ST7789 display initialization
 * - Basic drawing operations (fill, rectangles, pixels)
 * - Color patterns and animations
 */

#include <cstdio>
#include <cinttypes>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_timer.h>

#include "cube32.h"

static const char *TAG = "hello_display";

/**
 * @brief Draw a simple test pattern
 */
void draw_test_pattern(cube32::ST7789Display& display) {
    uint16_t width = display.getWidth();
    uint16_t height = display.getHeight();
    
    // Draw color bars
    uint16_t bar_height = height / 8;
    uint16_t colors[] = {
        CUBE32_COLOR_RED,
        CUBE32_COLOR_GREEN,
        CUBE32_COLOR_BLUE,
        CUBE32_COLOR_YELLOW,
        CUBE32_COLOR_CYAN,
        CUBE32_COLOR_MAGENTA,
        CUBE32_COLOR_WHITE,
        CUBE32_COLOR_GRAY,
    };

    // Clear previous frame
    display.clear(CUBE32_COLOR_BLACK);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    for (int i = 0; i < 8; i++) {
        display.fillRect(0, i * bar_height, width, (i + 1) * bar_height, colors[i]);
        vTaskDelay(pdMS_TO_TICKS(10));  // Small delay between bars to avoid SPI buffer issues
    }
    
    ESP_LOGI(TAG, "Test pattern drawn");
}

/**
 * @brief Draw a bouncing rectangle animation
 */
void draw_bouncing_rect(cube32::ST7789Display& display, int frame) {
    uint16_t width = display.getWidth();
    uint16_t height = display.getHeight();
    
    // Calculate bounce position
    int rect_size = 40;
    int max_x = width - rect_size;
    int max_y = height - rect_size;
    
    // Use simple sine-like bounce pattern
    int x = (frame * 3) % (max_x * 2);
    int y = (frame * 2) % (max_y * 2);
    
    if (x > max_x) x = max_x * 2 - x;
    if (y > max_y) y = max_y * 2 - y;
    
    // Clear previous frame
    display.clear(CUBE32_COLOR_BLACK);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Draw rectangle
    uint16_t color = cube32_rgb565((frame * 3) % 256, (frame * 5) % 256, (frame * 7) % 256);
    display.fillRect(x, y, x + rect_size, y + rect_size, color);
    vTaskDelay(pdMS_TO_TICKS(10));
}

/**
 * @brief Draw CUBE32 text/logo pattern
 */
void draw_cube32_logo(cube32::ST7789Display& display) {
    uint16_t width = display.getWidth();
    uint16_t height = display.getHeight();
    uint16_t cx = width / 2;
    uint16_t cy = height / 2;
    
    // Draw background gradient
    display.clear(CUBE32_COLOR_BLACK);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Draw "cube" shape - a series of nested squares
    for (int i = 0; i < 5; i++) {
        uint16_t size = 80 - i * 15;
        uint16_t x = cx - size / 2;
        uint16_t y = cy - size / 2;
        
        uint16_t color = cube32_rgb565(0, 100 + i * 30, 255 - i * 30);
        display.fillRect(x, y, x + size, y + size, color);
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
    
    // Draw corner accents
    display.fillRect(0, 0, 20, 20, CUBE32_COLOR_RED);
    vTaskDelay(pdMS_TO_TICKS(10));
    display.fillRect(width - 20, 0, width, 20, CUBE32_COLOR_GREEN);
    vTaskDelay(pdMS_TO_TICKS(10));
    display.fillRect(0, height - 20, 20, height, CUBE32_COLOR_BLUE);
    vTaskDelay(pdMS_TO_TICKS(10));
    display.fillRect(width - 20, height - 20, width, height, CUBE32_COLOR_YELLOW);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    ESP_LOGI(TAG, "CUBE32 logo drawn");
}

/**
 * @brief Draw a gradient pattern
 */
void draw_gradient(cube32::ST7789Display& display) {
    uint16_t width = display.getWidth();
    uint16_t height = display.getHeight();
    
    ESP_LOGI(TAG, "Drawing gradient (this may take a moment)...");
    
    // Draw vertical gradient - row by row for efficiency
    for (uint16_t y = 0; y < height; y++) {
        uint8_t r = (y * 255) / height;
        uint8_t b = 255 - r;
        uint16_t color = cube32_rgb565(r, 0, b);
        display.fillRect(0, y, width, y + 1, color);
    }
    
    ESP_LOGI(TAG, "Gradient drawn");
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "CUBE32 Hello Display Example");
    ESP_LOGI(TAG, "========================================");

    /* Initialize CUBE32 board (includes SPI bus and display) */
    esp_err_t ret = cube32_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize CUBE32 board!");
        return;
    }

    /* Get display instance */
    auto& display = cube32::ST7789Display::instance();
    if (!display.isInitialized()) {
        ESP_LOGE(TAG, "Display not initialized - check CONFIG_CUBE32_DISPLAY_ENABLED");
        return;
    }
    
    ESP_LOGI(TAG, "Display ready: %dx%d", display.getWidth(), display.getHeight());

    /* Demo sequence */
    int demo_phase = 0;
    int frame = 0;
    int64_t last_phase_time = esp_timer_get_time();
    const int64_t PHASE_DURATION_US = 5000000; // 5 seconds per phase

    while (true) {
        int64_t current_time = esp_timer_get_time();
        
        // Switch demo phase every 5 seconds
        if (current_time - last_phase_time > PHASE_DURATION_US) {
            demo_phase = (demo_phase + 1) % 5;
            last_phase_time = current_time;
            frame = 0;
            
            ESP_LOGI(TAG, "Switching to demo phase %d", demo_phase);
        }
        
        switch (demo_phase) {
            case 0:
                // Clear screen and show info
                if (frame == 0) {
                    display.clear(CUBE32_COLOR_BLACK);
                    vTaskDelay(pdMS_TO_TICKS(10));
                    ESP_LOGI(TAG, "Phase 0: Black screen");
                    
                    // Draw some info text area
                    display.fillRect(20, 100, 220, 140, CUBE32_COLOR_DARK_GRAY);
                }
                break;
                
            case 1:
                // Test pattern
                if (frame == 0) {
                    draw_test_pattern(display);
                }
                break;
                
            case 2:
                // CUBE32 logo
                if (frame == 0) {
                    draw_cube32_logo(display);
                }
                break;
                
            case 3:
                // Gradient
                if (frame == 0) {
                    draw_gradient(display);
                }
                break;
                
            case 4:
                // Bouncing rectangle animation
                draw_bouncing_rect(display, frame);
                break;
        }
        
        frame++;
        
        // Animation frame delay
        if (demo_phase == 4) {
            vTaskDelay(pdMS_TO_TICKS(30)); // Fast animation
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}
