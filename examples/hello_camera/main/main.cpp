/**
 * @file main.cpp
 * @brief CUBE32 Hello Camera Example
 * 
 * This example demonstrates camera initialization and displaying
 * camera images on the TFT display using the CUBE32 BSP.
 * 
 * Features demonstrated:
 * - Camera initialization with the shared I2C bus
 * - Image capture to frame buffer
 * - Displaying camera images on ST7789 TFT display
 * - Handling RGB565 pixel format conversion
 * 
 * Required Kconfig settings:
 * - CONFIG_CUBE32_DISPLAY_ENABLED=y
 * - CONFIG_CUBE32_CAMERA_ENABLED=y
 * - CONFIG_CUBE32_CAMERA_PIXEL_FORMAT_RGB565=y
 */

#include <cstdio>
#include <cstring>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_heap_caps.h>

#include "cube32.h"

static const char *TAG = "hello_camera";

/**
 * @brief Display camera frame on the LCD
 * 
 * This function takes the camera frame buffer and displays it on the LCD.
 * The camera outputs RGB565 in big-endian format, while the LCD expects
 * little-endian, so byte swapping may be needed.
 * 
 * @param display Reference to the display driver
 * @param fb Pointer to the camera frame buffer
 */
void display_camera_frame(cube32::ST7789Display& display, camera_fb_t* fb) {
    if (fb == nullptr) {
        ESP_LOGE(TAG, "Frame buffer is null");
        return;
    }

    uint16_t disp_width = display.getWidth();
    uint16_t disp_height = display.getHeight();
    uint16_t cam_width = fb->width;
    uint16_t cam_height = fb->height;

    // Calculate scaling and centering
    uint16_t draw_width = (cam_width < disp_width) ? cam_width : disp_width;
    uint16_t draw_height = (cam_height < disp_height) ? cam_height : disp_height;
    uint16_t x_offset = (disp_width - draw_width) / 2;
    uint16_t y_offset = (disp_height - draw_height) / 2;

    // Crop offsets when the camera image is larger than the display
    uint16_t src_x_offset = (cam_width  > disp_width)  ? (cam_width  - disp_width)  / 2 : 0;
    uint16_t src_y_offset = (cam_height > disp_height) ? (cam_height - disp_height) / 2 : 0;

    // Allocate one full-frame buffer in DMA-capable memory.
    // Drawing the whole frame in a single esp_lcd_panel_draw_bitmap() call is
    // ~120x fewer SPI transactions than line-by-line drawing, which prevents
    // SPI/PSRAM contention from starving the camera DMA (avoids
    // "cam_hal: FB-SIZE: X != Y" warnings).
    static uint16_t* frame_buf = nullptr;
    static size_t frame_buf_pixels = 0;
    size_t needed_pixels = (size_t)draw_width * draw_height;
    if (frame_buf == nullptr || frame_buf_pixels < needed_pixels) {
        if (frame_buf) heap_caps_free(frame_buf);
        frame_buf = (uint16_t*)heap_caps_malloc(needed_pixels * sizeof(uint16_t),
                                                MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
        if (frame_buf == nullptr) {
            ESP_LOGE(TAG, "Failed to allocate frame buffer (%u bytes)",
                     (unsigned)(needed_pixels * sizeof(uint16_t)));
            frame_buf_pixels = 0;
            return;
        }
        frame_buf_pixels = needed_pixels;
    }

    // NOTE: do NOT clear the display here. The caller clears once before the
    // capture loop. Clearing every frame causes visible flicker and saturates
    // the SPI/PSRAM bus, which can starve the camera DMA and produce
    // intermittent "FB-SIZE: X != Y" warnings from cam_hal.

    // Copy + byte-swap the camera frame into the DMA buffer
    // (camera outputs big-endian RGB565, LCD expects little-endian).
    const uint16_t* src = (const uint16_t*)fb->buf;
    for (uint16_t y = 0; y < draw_height; y++) {
        const uint16_t* src_line = src + (size_t)(y + src_y_offset) * cam_width + src_x_offset;
        uint16_t* dst_line = frame_buf + (size_t)y * draw_width;
        for (uint16_t x = 0; x < draw_width; x++) {
            dst_line[x] = __builtin_bswap16(src_line[x]);
        }
    }

    // One single draw call for the whole frame
    display.drawBitmap(x_offset, y_offset,
                       x_offset + draw_width, y_offset + draw_height,
                       frame_buf);
}

/**
 * @brief Display camera info overlay
 */
void display_camera_info(cube32::ST7789Display& display, camera_fb_t* fb, int fps) {
    // In a more complete implementation, you would draw text overlay here
    // For now, just log the info
    ESP_LOGI(TAG, "Frame: %dx%d, FPS: %d", fb->width, fb->height, fps);
}

/**
 * @brief Main camera demo task
 */
void camera_demo_task(void* arg) {
    ESP_LOGI(TAG, "Starting camera demo...");

    // Get display and camera instances
    cube32::ST7789Display& display = cube32::ST7789Display::instance();
    cube32::Camera& camera = cube32::Camera::instance();

    // Verify both are initialized
    if (!display.isInitialized()) {
        ESP_LOGE(TAG, "Display not initialized!");
        vTaskDelete(nullptr);
        return;
    }

    if (!camera.isInitialized()) {
        ESP_LOGE(TAG, "Camera not initialized!");
        vTaskDelete(nullptr);
        return;
    }

    // Turn on display
    display.displayOn();
    display.setBacklight(100);  // Turn on backlight at full brightness
    
    // Clear display
    display.clear(CUBE32_COLOR_BLACK);
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "Display: %dx%d", display.getWidth(), display.getHeight());

    // Frame timing variables
    int64_t last_time = esp_timer_get_time();
    int frame_count = 0;
    int fps = 0;

    // Main capture loop
    while (true) {
        // Capture frame
        if (!camera.capture()) {
            ESP_LOGE(TAG, "Failed to capture frame");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        camera_fb_t* fb = camera.getFrameBuffer();
        if (fb == nullptr) {
            ESP_LOGE(TAG, "Frame buffer is null");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Display the frame
        display_camera_frame(display, fb);

        // Calculate FPS
        frame_count++;
        int64_t current_time = esp_timer_get_time();
        if (current_time - last_time >= 1000000) {  // 1 second
            fps = frame_count;
            frame_count = 0;
            last_time = current_time;
            display_camera_info(display, fb, fps);
        }

        // Return frame buffer
        camera.returnFrameBuffer();

        // Small delay to prevent watchdog timeout
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    vTaskDelete(nullptr);
}

#ifdef CONFIG_CUBE32_LVGL_ENABLED
/**
 * @brief Camera demo using LVGL for display
 * 
 * This version uses LVGL to display the camera image, which provides
 * better integration with LVGL-based UIs.
 */
void camera_demo_lvgl_task(void* arg) {
    ESP_LOGI(TAG, "Starting LVGL camera demo...");

    cube32::Camera& camera = cube32::Camera::instance();
    cube32::LvglDisplay& lvgl = cube32::LvglDisplay::instance();

    if (!camera.isInitialized() || !lvgl.isInitialized()) {
        ESP_LOGE(TAG, "Camera or LVGL not initialized!");
        vTaskDelete(nullptr);
        return;
    }

    // Create LVGL image object
    lvgl.lock();
    
    lv_obj_t* img = lv_img_create(lv_scr_act());
    lv_obj_center(img);
    
    // Create image descriptor
    static lv_img_dsc_t img_dsc;
    img_dsc.header.cf = LV_COLOR_FORMAT_RGB565;
    img_dsc.header.stride = 0;  // Will be set based on frame size
    img_dsc.header.w = 0;
    img_dsc.header.h = 0;
    img_dsc.data = nullptr;
    img_dsc.data_size = 0;

    lvgl.unlock();

    // Allocate buffer for converted image
    uint16_t* conv_buf = nullptr;
    size_t conv_buf_size = 0;

    // Frame timing
    int64_t last_time = esp_timer_get_time();
    int frame_count = 0;

    while (true) {
        // Capture frame
        if (!camera.capture()) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        camera_fb_t* fb = camera.getFrameBuffer();
        if (fb == nullptr || fb->format != PIXFORMAT_RGB565) {
            camera.returnFrameBuffer();
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Reallocate conversion buffer if needed
        size_t needed_size = fb->width * fb->height * sizeof(uint16_t);
        if (conv_buf_size < needed_size) {
            if (conv_buf != nullptr) {
                heap_caps_free(conv_buf);
            }
            conv_buf = (uint16_t*)heap_caps_malloc(needed_size, MALLOC_CAP_SPIRAM);
            if (conv_buf == nullptr) {
                ESP_LOGE(TAG, "Failed to allocate conversion buffer");
                camera.returnFrameBuffer();
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            conv_buf_size = needed_size;
        }

        // Convert endianness
        uint16_t* src = (uint16_t*)fb->buf;
        for (size_t i = 0; i < fb->width * fb->height; i++) {
            conv_buf[i] = __builtin_bswap16(src[i]);
        }

        // Update LVGL image
        lvgl.lock();
        
        img_dsc.header.w = fb->width;
        img_dsc.header.h = fb->height;
        img_dsc.header.stride = fb->width * 2;
        img_dsc.data = (const uint8_t*)conv_buf;
        img_dsc.data_size = needed_size;
        
        lv_img_set_src(img, &img_dsc);
        lv_obj_center(img);
        
        lvgl.unlock();

        camera.returnFrameBuffer();

        // Calculate FPS
        frame_count++;
        int64_t current_time = esp_timer_get_time();
        if (current_time - last_time >= 1000000) {
            ESP_LOGI(TAG, "FPS: %d, Frame: %dx%d", frame_count, fb->width, fb->height);
            frame_count = 0;
            last_time = current_time;
        }

        vTaskDelay(pdMS_TO_TICKS(30));  // ~30 FPS max
    }

    if (conv_buf != nullptr) {
        heap_caps_free(conv_buf);
    }

    vTaskDelete(nullptr);
}
#endif

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "CUBE32 Hello Camera Example");
    ESP_LOGI(TAG, "========================================");

    // Initialize CUBE32 board (this will init display and camera if enabled)
    esp_err_t ret = cube32_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize CUBE32: %s", esp_err_to_name(ret));
        return;
    }

    // Verify camera is enabled and initialized
#ifndef CONFIG_CUBE32_CAMERA_ENABLED
    ESP_LOGE(TAG, "Camera is not enabled in Kconfig!");
    ESP_LOGE(TAG, "Please enable CONFIG_CUBE32_CAMERA_ENABLED in menuconfig.");
    return;
#endif

#ifndef CONFIG_CUBE32_DISPLAY_ENABLED
    ESP_LOGE(TAG, "Display is not enabled in Kconfig!");
    ESP_LOGE(TAG, "Please enable CONFIG_CUBE32_DISPLAY_ENABLED in menuconfig.");
    return;
#endif

    // Print camera info
    sensor_t* sensor = cube32::Camera::instance().getSensor();
    if (sensor != nullptr) {
        ESP_LOGI(TAG, "Camera sensor detected: PID=0x%02x", sensor->id.PID);
    }

    // Start camera demo task
#ifdef CONFIG_CUBE32_LVGL_ENABLED
    ESP_LOGI(TAG, "Starting LVGL camera demo...");
    xTaskCreate(camera_demo_lvgl_task, "camera_lvgl", 8192, nullptr, 5, nullptr);
#else
    ESP_LOGI(TAG, "Starting direct display camera demo...");
    xTaskCreate(camera_demo_task, "camera_demo", 8192, nullptr, 5, nullptr);
#endif

    ESP_LOGI(TAG, "Camera demo started!");
}
