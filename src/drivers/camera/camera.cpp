/**
 * @file camera.cpp
 * @brief CUBE32 Camera Driver Implementation
 * 
 * This implementation uses the esp32-camera component for camera operations.
 * The SCCB interface shares the I2C bus with other peripherals, so pin_sccb_sda
 * and pin_sccb_scl should be set to -1 to use the existing I2C bus.
 */

#include "drivers/camera/camera.h"

#include <esp_log.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* TAG = "cube32_camera";

namespace cube32 {

// ============================================================================
// Singleton Implementation
// ============================================================================

Camera& Camera::instance() {
    static Camera s_instance;
    return s_instance;
}

Camera::~Camera() {
    if (m_initialized) {
        end();
    }
}

// ============================================================================
// Initialization
// ============================================================================

cube32_result_t Camera::begin() {
    cube32_camera_config_t config = CUBE32_CAMERA_CONFIG_DEFAULT();
    return begin(config);
}

cube32_result_t Camera::begin(const cube32_camera_config_t& config) {
    if (m_initialized) {
        ESP_LOGW(TAG, "Camera already initialized");
        return CUBE32_ALREADY_INITIALIZED;
    }

    // Store configuration
    m_config = config;

    ESP_LOGI(TAG, "Initializing camera...");
    ESP_LOGI(TAG, "  XCLK: GPIO%d, PCLK: GPIO%d", config.pin_xclk, config.pin_pclk);
    ESP_LOGI(TAG, "  VSYNC: GPIO%d, HREF: GPIO%d", config.pin_vsync, config.pin_href);
    ESP_LOGI(TAG, "  SCCB SDA: %d, SCL: %d (using shared I2C: %s)", 
             config.pin_sccb_sda, config.pin_sccb_scl,
             (config.pin_sccb_sda == -1 && config.pin_sccb_scl == -1) ? "yes" : "no");

    // Build camera_config_t from our configuration
    camera_config_t cam_config = {};
    
    // Pin configuration
    cam_config.pin_pwdn = config.pin_pwdn;
    cam_config.pin_reset = config.pin_reset;
    cam_config.pin_xclk = config.pin_xclk;
    cam_config.pin_sccb_sda = config.pin_sccb_sda;  // -1 to use existing I2C
    cam_config.pin_sccb_scl = config.pin_sccb_scl;  // -1 to use existing I2C
    cam_config.sccb_i2c_port = config.sccb_i2c_port;
    cam_config.pin_d7 = config.pin_d7;
    cam_config.pin_d6 = config.pin_d6;
    cam_config.pin_d5 = config.pin_d5;
    cam_config.pin_d4 = config.pin_d4;
    cam_config.pin_d3 = config.pin_d3;
    cam_config.pin_d2 = config.pin_d2;
    cam_config.pin_d1 = config.pin_d1;
    cam_config.pin_d0 = config.pin_d0;
    cam_config.pin_vsync = config.pin_vsync;
    cam_config.pin_href = config.pin_href;
    cam_config.pin_pclk = config.pin_pclk;
    
    // Camera settings
    cam_config.xclk_freq_hz = config.xclk_freq_hz;
    // Note: ledc_timer and ledc_channel are NOT used on ESP32-S3
    // ESP32-S3 uses a different clock source for XCLK
    cam_config.pixel_format = config.pixel_format;
    cam_config.frame_size = config.frame_size;
    cam_config.jpeg_quality = config.jpeg_quality;
    cam_config.fb_count = config.fb_count;
    cam_config.fb_location = config.fb_location;
    cam_config.grab_mode = config.grab_mode;

    // Initialize camera
    esp_err_t err = esp_camera_init(&cam_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return esp_err_to_cube32(err);
    }

    m_initialized = true;
    ESP_LOGI(TAG, "Camera initialized successfully");

    // Get sensor info
    sensor_t* s = esp_camera_sensor_get();
    if (s != nullptr) {
        ESP_LOGI(TAG, "Camera sensor PID: 0x%02x", s->id.PID);
        
        // Apply sensor-specific settings
        if (s->id.PID == GC0308_PID) {
            ESP_LOGI(TAG, "Detected GC0308 sensor, applying specific settings");
            s->set_hmirror(s, 0);
        }
    }

    // Allow camera to stabilize
    vTaskDelay(pdMS_TO_TICKS(500));

    // Apply mirror/flip settings from configuration
    setHMirror(config.h_mirror);
    setVFlip(config.v_flip);

    return CUBE32_OK;
}

cube32_result_t Camera::end() {
    if (!m_initialized) {
        return CUBE32_NOT_INITIALIZED;
    }

    // Return any held frame buffer
    if (m_fb != nullptr) {
        esp_camera_fb_return(m_fb);
        m_fb = nullptr;
    }

    // Deinitialize camera
    esp_err_t err = esp_camera_deinit();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera deinit failed: %s", esp_err_to_name(err));
        return esp_err_to_cube32(err);
    }

    m_initialized = false;
    ESP_LOGI(TAG, "Camera deinitialized");

    return CUBE32_OK;
}

// ============================================================================
// Capture Operations
// ============================================================================

bool Camera::capture() {
    if (!m_initialized) {
        ESP_LOGE(TAG, "Camera not initialized");
        return false;
    }

    // Return previous frame buffer if held
    if (m_fb != nullptr) {
        esp_camera_fb_return(m_fb);
        m_fb = nullptr;
    }

    // Try to get a stable frame (discard first frame)
    int frames_to_get = 2;
    for (int i = 0; i < frames_to_get; i++) {
        if (m_fb != nullptr) {
            esp_camera_fb_return(m_fb);
        }
        m_fb = esp_camera_fb_get();
        if (m_fb == nullptr) {
            ESP_LOGE(TAG, "Camera capture failed");
            return false;
        }
    }

    ESP_LOGD(TAG, "Captured frame: %dx%d, format: %d, len: %u",
             m_fb->width, m_fb->height, m_fb->format, (unsigned int)m_fb->len);

    return true;
}

void Camera::returnFrameBuffer() {
    if (m_fb != nullptr) {
        esp_camera_fb_return(m_fb);
        m_fb = nullptr;
    }
}

// ============================================================================
// Camera Control
// ============================================================================

bool Camera::setHMirror(bool enabled) {
    sensor_t* s = esp_camera_sensor_get();
    if (s == nullptr) {
        ESP_LOGE(TAG, "Failed to get camera sensor");
        return false;
    }

    esp_err_t err = s->set_hmirror(s, enabled ? 1 : 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set horizontal mirror: %d", err);
        return false;
    }

    ESP_LOGI(TAG, "Camera horizontal mirror: %s", enabled ? "enabled" : "disabled");
    return true;
}

bool Camera::setVFlip(bool enabled) {
    sensor_t* s = esp_camera_sensor_get();
    if (s == nullptr) {
        ESP_LOGE(TAG, "Failed to get camera sensor");
        return false;
    }

    esp_err_t err = s->set_vflip(s, enabled ? 1 : 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set vertical flip: %d", err);
        return false;
    }

    ESP_LOGI(TAG, "Camera vertical flip: %s", enabled ? "enabled" : "disabled");
    return true;
}

sensor_t* Camera::getSensor() const {
    return esp_camera_sensor_get();
}

bool Camera::setFrameSize(framesize_t size) {
    sensor_t* s = esp_camera_sensor_get();
    if (s == nullptr) {
        ESP_LOGE(TAG, "Failed to get camera sensor");
        return false;
    }

    esp_err_t err = s->set_framesize(s, size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set frame size: %d", err);
        return false;
    }

    ESP_LOGI(TAG, "Frame size set to: %d", size);
    return true;
}

bool Camera::setPixelFormat(pixformat_t format) {
    sensor_t* s = esp_camera_sensor_get();
    if (s == nullptr) {
        ESP_LOGE(TAG, "Failed to get camera sensor");
        return false;
    }

    esp_err_t err = s->set_pixformat(s, format);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set pixel format: %d", err);
        return false;
    }

    ESP_LOGI(TAG, "Pixel format set to: %d", format);
    return true;
}

} // namespace cube32
