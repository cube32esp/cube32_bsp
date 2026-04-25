/**
 * @file camera.h
 * @brief CUBE32 Camera Driver
 * 
 * This driver provides support for ESP32 camera modules (OV2640, etc.)
 * using the esp32-camera component. The camera SCCB interface shares
 * the same I2C bus with other peripherals.
 * 
 * Features:
 * - Camera initialization with configurable resolution and format
 * - Image capture to frame buffer
 * - Horizontal mirror and vertical flip control
 * - Support for rotation configuration via Kconfig
 */

#ifndef CUBE32_DRIVERS_CAMERA_CAMERA_H
#define CUBE32_DRIVERS_CAMERA_CAMERA_H

#include "utils/common.h"
#include "cube32_config.h"

#include <esp_camera.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Constants
// ============================================================================

/**
 * @brief Default camera XCLK frequency
 */
#define CUBE32_CAMERA_XCLK_FREQ_DEFAULT    CUBE32_CAM_XCLK_FREQ_HZ

/**
 * @brief Default frame buffer count
 */
#define CUBE32_CAMERA_FB_COUNT_DEFAULT     1

/**
 * @brief Default JPEG quality (1-63, lower is better)
 */
#define CUBE32_CAMERA_JPEG_QUALITY_DEFAULT 12

// ============================================================================
// Configuration Structures
// ============================================================================

/**
 * @brief Camera configuration structure
 */
typedef struct {
    // Pin configuration (set to -1 to use shared I2C bus for SCCB)
    int pin_pwdn;                   ///< Power down GPIO pin (-1 if not used)
    int pin_reset;                  ///< Reset GPIO pin (-1 if not used)
    int pin_xclk;                   ///< XCLK GPIO pin
    int pin_sccb_sda;               ///< SCCB SDA GPIO pin (-1 to use existing I2C)
    int pin_sccb_scl;               ///< SCCB SCL GPIO pin (-1 to use existing I2C)
    int sccb_i2c_port;              ///< I2C port for SCCB (when using existing I2C)
    int pin_d7;                     ///< Data pin D7
    int pin_d6;                     ///< Data pin D6
    int pin_d5;                     ///< Data pin D5
    int pin_d4;                     ///< Data pin D4
    int pin_d3;                     ///< Data pin D3
    int pin_d2;                     ///< Data pin D2
    int pin_d1;                     ///< Data pin D1
    int pin_d0;                     ///< Data pin D0
    int pin_vsync;                  ///< VSYNC GPIO pin
    int pin_href;                   ///< HREF GPIO pin
    int pin_pclk;                   ///< Pixel clock GPIO pin
    
    // Camera settings
    uint32_t xclk_freq_hz;          ///< XCLK frequency in Hz
    pixformat_t pixel_format;       ///< Pixel format (PIXFORMAT_RGB565, etc.)
    framesize_t frame_size;         ///< Frame size (FRAMESIZE_QVGA, etc.)
    uint8_t jpeg_quality;           ///< JPEG quality (1-63, lower is better)
    uint8_t fb_count;               ///< Frame buffer count
    camera_fb_location_t fb_location; ///< Frame buffer location
    camera_grab_mode_t grab_mode;   ///< Frame grab mode
    
    // Display orientation
    bool h_mirror;                  ///< Enable horizontal mirror
    bool v_flip;                    ///< Enable vertical flip
} cube32_camera_config_t;

/**
 * @brief Default camera configuration using pins from cube32_config.h
 *
 * h_mirror and v_flip default to false; actual values are applied from NVS config at runtime.
 */
#define CUBE32_CAMERA_CONFIG_DEFAULT() { \
    .pin_pwdn = CUBE32_CAM_PWDN_PIN, \
    .pin_reset = CUBE32_CAM_RESET_PIN, \
    .pin_xclk = CUBE32_CAM_XCLK_PIN, \
    .pin_sccb_sda = -1, \
    .pin_sccb_scl = -1, \
    .sccb_i2c_port = CUBE32_I2C_NUM, \
    .pin_d7 = CUBE32_CAM_D7_PIN, \
    .pin_d6 = CUBE32_CAM_D6_PIN, \
    .pin_d5 = CUBE32_CAM_D5_PIN, \
    .pin_d4 = CUBE32_CAM_D4_PIN, \
    .pin_d3 = CUBE32_CAM_D3_PIN, \
    .pin_d2 = CUBE32_CAM_D2_PIN, \
    .pin_d1 = CUBE32_CAM_D1_PIN, \
    .pin_d0 = CUBE32_CAM_D0_PIN, \
    .pin_vsync = CUBE32_CAM_VSYNC_PIN, \
    .pin_href = CUBE32_CAM_HREF_PIN, \
    .pin_pclk = CUBE32_CAM_PCLK_PIN, \
    .xclk_freq_hz = CUBE32_CAMERA_XCLK_FREQ_DEFAULT, \
    .pixel_format = PIXFORMAT_RGB565, \
    .frame_size = FRAMESIZE_QVGA, \
    .jpeg_quality = CUBE32_CAMERA_JPEG_QUALITY_DEFAULT, \
    .fb_count = CUBE32_CAMERA_FB_COUNT_DEFAULT, \
    .fb_location = CAMERA_FB_IN_PSRAM, \
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY, \
    .h_mirror = false, \
    .v_flip = false, \
}

#ifdef __cplusplus
}
#endif

// ============================================================================
// C++ Class Definition
// ============================================================================

#ifdef __cplusplus

namespace cube32 {

/**
 * @brief CUBE32 Camera Driver Class
 * 
 * Singleton class for managing the ESP32 camera module.
 * Uses esp32-camera component for camera operations.
 * 
 * Example usage:
 * @code
 * // Initialize with default configuration
 * cube32_result_t ret = cube32::Camera::instance().begin();
 * 
 * // Or with custom configuration
 * cube32_camera_config_t config = CUBE32_CAMERA_CONFIG_DEFAULT();
 * config.frame_size = FRAMESIZE_VGA;
 * ret = cube32::Camera::instance().begin(config);
 * 
 * // Capture an image
 * if (cube32::Camera::instance().capture()) {
 *     camera_fb_t* fb = cube32::Camera::instance().getFrameBuffer();
 *     // Process frame buffer...
 *     cube32::Camera::instance().returnFrameBuffer();
 * }
 * @endcode
 */
class Camera {
public:
    /**
     * @brief Get singleton instance
     */
    static Camera& instance();
    
    /**
     * @brief Destructor
     */
    ~Camera();
    
    // Prevent copying
    Camera(const Camera&) = delete;
    Camera& operator=(const Camera&) = delete;
    
    /**
     * @brief Initialize camera with default configuration
     * @return CUBE32_OK on success
     */
    cube32_result_t begin();
    
    /**
     * @brief Initialize camera with custom configuration
     * @param config Camera configuration
     * @return CUBE32_OK on success
     */
    cube32_result_t begin(const cube32_camera_config_t& config);
    
    /**
     * @brief Deinitialize camera
     * @return CUBE32_OK on success
     */
    cube32_result_t end();
    
    /**
     * @brief Check if camera is initialized
     */
    bool isInitialized() const { return m_initialized; }
    
    /**
     * @brief Capture an image
     * @return true on success
     */
    bool capture();
    
    /**
     * @brief Get the current frame buffer
     * @return Pointer to frame buffer, or nullptr if not available
     */
    camera_fb_t* getFrameBuffer() const { return m_fb; }
    
    /**
     * @brief Return the frame buffer to the camera driver
     */
    void returnFrameBuffer();
    
    /**
     * @brief Set horizontal mirror
     * @param enabled true to enable mirror
     * @return true on success
     */
    bool setHMirror(bool enabled);
    
    /**
     * @brief Set vertical flip
     * @param enabled true to enable flip
     * @return true on success
     */
    bool setVFlip(bool enabled);
    
    /**
     * @brief Get current frame width
     */
    uint16_t getFrameWidth() const { return m_fb ? m_fb->width : 0; }
    
    /**
     * @brief Get current frame height
     */
    uint16_t getFrameHeight() const { return m_fb ? m_fb->height : 0; }
    
    /**
     * @brief Get camera sensor
     * @return Pointer to sensor structure
     */
    sensor_t* getSensor() const;
    
    /**
     * @brief Set frame size
     * @param size Frame size
     * @return true on success
     */
    bool setFrameSize(framesize_t size);
    
    /**
     * @brief Set pixel format
     * @param format Pixel format
     * @return true on success
     */
    bool setPixelFormat(pixformat_t format);

private:
    /**
     * @brief Private constructor for singleton
     */
    Camera() = default;
    
    bool m_initialized = false;
    cube32_camera_config_t m_config;
    camera_fb_t* m_fb = nullptr;
};

} // namespace cube32

#endif // __cplusplus

#endif // CUBE32_DRIVERS_CAMERA_CAMERA_H
