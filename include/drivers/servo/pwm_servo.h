/**
 * @file pwm_servo.h
 * @brief CUBE32 PWM Servo Driver
 * 
 * This driver provides PWM servo control using the ESP32-S3 LEDC hardware.
 * Supports up to 2 servo channels with configurable pulse width, angle,
 * and smooth rotation with speed control.
 * 
 * Features:
 * - Hardware LEDC PWM (zero CPU overhead)
 * - Up to 2 independent servo channels
 * - Support for 90°, 180°, and 360° (continuous rotation) servos
 * - Angle control with configurable pulse width range
 * - Speed-controlled smooth rotation (non-blocking via FreeRTOS task)
 * - Attach/detach for power saving
 * 
 * For 360° continuous rotation servos:
 * - setAngle(90) = stop
 * - setAngle(0)  = full speed clockwise
 * - setAngle(180) = full speed counter-clockwise
 * 
 * Example usage:
 * @code
 * // Initialize with defaults from Kconfig
 * cube32::PwmServo::instance().begin();
 * 
 * // Set servo 0 to 45 degrees
 * cube32::PwmServo::instance().setAngle(0, 45.0f);
 * 
 * // Smooth sweep servo 1 from 0 to 180 at 50% speed
 * cube32::PwmServo::instance().smoothMove(1, 180.0f, 50.0f);
 * @endcode
 */

#ifndef CUBE32_DRIVERS_SERVO_PWM_SERVO_H
#define CUBE32_DRIVERS_SERVO_PWM_SERVO_H

#include "utils/common.h"
#include "cube32_config.h"

#include <driver/ledc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Constants
// ============================================================================

#define CUBE32_SERVO_MAX_CHANNELS   2       ///< Maximum servo channels supported

#define CUBE32_SERVO_FREQ_HZ        50      ///< Standard servo PWM frequency (50 Hz)
#define CUBE32_SERVO_RESOLUTION     LEDC_TIMER_14_BIT  ///< 14-bit resolution (16384 ticks)
#define CUBE32_SERVO_RESOLUTION_MAX 16384   ///< Max duty value at 14-bit

#define CUBE32_SERVO_MIN_PULSE_US   500     ///< Default minimum pulse width (µs)
#define CUBE32_SERVO_MAX_PULSE_US   2500    ///< Default maximum pulse width (µs)

// ============================================================================
// Configuration Structures
// ============================================================================

/**
 * @brief Servo max rotation type
 */
typedef enum {
    CUBE32_SERVO_90_DEG = 90,       ///< 90-degree servo
    CUBE32_SERVO_180_DEG = 180,     ///< 180-degree servo
    CUBE32_SERVO_360_DEG = 360,     ///< 360-degree continuous rotation servo
} cube32_servo_rotation_t;

/**
 * @brief Per-channel servo configuration
 */
typedef struct {
    gpio_num_t signal_pin;          ///< GPIO pin for servo signal
    uint16_t min_pulse_us;          ///< Minimum pulse width in µs (0° position)
    uint16_t max_pulse_us;          ///< Maximum pulse width in µs (max° position)
    cube32_servo_rotation_t max_rotation;  ///< Maximum rotation (90, 180, or 360)
    ledc_channel_t ledc_channel;    ///< LEDC channel assignment
} cube32_servo_channel_config_t;

/**
 * @brief PWM Servo driver configuration
 */
typedef struct {
    ledc_timer_t timer;             ///< LEDC timer to use
    uint32_t freq_hz;               ///< PWM frequency (default 50 Hz)
    ledc_timer_bit_t resolution;    ///< Timer resolution (default 14-bit)
    uint8_t num_channels;           ///< Number of servo channels (1 or 2)
    cube32_servo_channel_config_t channels[CUBE32_SERVO_MAX_CHANNELS];
} cube32_servo_config_t;

/**
 * @brief Default servo configuration from Kconfig
 */
#ifdef CONFIG_CUBE32_SERVO_ENABLED
#define CUBE32_SERVO_CONFIG_DEFAULT() { \
    .timer = LEDC_TIMER_0, \
    .freq_hz = CUBE32_SERVO_FREQ_HZ, \
    .resolution = CUBE32_SERVO_RESOLUTION, \
    .num_channels = CONFIG_CUBE32_SERVO_NUM_CHANNELS, \
    .channels = { \
        { \
            .signal_pin = (gpio_num_t)CONFIG_CUBE32_SERVO1_PIN, \
            .min_pulse_us = CONFIG_CUBE32_SERVO1_MIN_PULSE_US, \
            .max_pulse_us = CONFIG_CUBE32_SERVO1_MAX_PULSE_US, \
            .max_rotation = (cube32_servo_rotation_t)CONFIG_CUBE32_SERVO1_MAX_ROTATION, \
            .ledc_channel = LEDC_CHANNEL_0, \
        }, \
        { \
            .signal_pin = (gpio_num_t)CONFIG_CUBE32_SERVO2_PIN, \
            .min_pulse_us = CONFIG_CUBE32_SERVO2_MIN_PULSE_US, \
            .max_pulse_us = CONFIG_CUBE32_SERVO2_MAX_PULSE_US, \
            .max_rotation = (cube32_servo_rotation_t)CONFIG_CUBE32_SERVO2_MAX_ROTATION, \
            .ledc_channel = LEDC_CHANNEL_1, \
        }, \
    }, \
}
#endif

#ifdef __cplusplus
}
#endif

// ============================================================================
// C++ Class Definition
// ============================================================================

#ifdef __cplusplus

namespace cube32 {

/**
 * @brief Per-channel servo state
 */
struct ServoChannelState {
    bool attached;          ///< Whether channel is currently attached (PWM active)
    float current_angle;    ///< Current angle in degrees
    bool moving;            ///< Whether smooth move is in progress
    float target_angle;     ///< Target angle for smooth move
    float move_speed;       ///< Move speed (degrees per second)
};

/**
 * @brief CUBE32 PWM Servo Driver Class
 * 
 * Singleton class for controlling PWM servos via ESP32-S3 LEDC hardware.
 */
class PwmServo {
public:
    /**
     * @brief Get singleton instance
     */
    static PwmServo& instance();

    /**
     * @brief Destructor
     */
    ~PwmServo();

    // Prevent copying
    PwmServo(const PwmServo&) = delete;
    PwmServo& operator=(const PwmServo&) = delete;

    /**
     * @brief Initialize with default configuration from Kconfig
     * @return CUBE32_OK on success
     */
    cube32_result_t begin();

    /**
     * @brief Initialize with custom configuration
     * @param config Servo configuration
     * @return CUBE32_OK on success
     */
    cube32_result_t begin(const cube32_servo_config_t& config);

    /**
     * @brief Deinitialize and release resources
     * @return CUBE32_OK on success
     */
    cube32_result_t end();

    /**
     * @brief Check if driver is initialized
     */
    bool isInitialized() const { return m_initialized; }

    // ====================================================================
    // Angle Control
    // ====================================================================

    /**
     * @brief Set servo angle immediately
     * 
     * For 90°/180° servos: angle is position in degrees (0 to max_rotation)
     * For 360° servos: angle controls speed/direction
     *   - 0° = full speed CW, 90° = stop, 180° = full speed CCW
     * 
     * @param channel Servo channel (0 or 1)
     * @param angle Target angle in degrees
     * @return CUBE32_OK on success
     */
    cube32_result_t setAngle(uint8_t channel, float angle);

    /**
     * @brief Get current servo angle
     * @param channel Servo channel (0 or 1)
     * @return Current angle in degrees, or -1 on error
     */
    float getAngle(uint8_t channel) const;

    /**
     * @brief Smooth move to target angle with speed control
     * 
     * Moves the servo smoothly from current position to target angle.
     * Speed is specified as a percentage (1-100) of maximum speed.
     * Maximum speed is approximately 360°/sec at 100%.
     * 
     * This function is non-blocking. Movement runs in a background task.
     * Call isMoving() to check if movement is still in progress.
     * Call stopMove() to cancel a smooth move.
     * 
     * For 360° continuous rotation servos, this function is not applicable.
     * Use setAngle() directly to control speed/direction.
     * 
     * @param channel Servo channel (0 or 1)
     * @param target_angle Target angle in degrees
     * @param speed_pct Speed percentage (1.0 - 100.0), default 50%
     * @return CUBE32_OK on success
     */
    cube32_result_t smoothMove(uint8_t channel, float target_angle, float speed_pct = 50.0f);

    /**
     * @brief Check if smooth move is in progress
     * @param channel Servo channel (0 or 1)
     * @return true if moving
     */
    bool isMoving(uint8_t channel) const;

    /**
     * @brief Stop smooth move immediately (holds current position)
     * @param channel Servo channel (0 or 1)
     * @return CUBE32_OK on success
     */
    cube32_result_t stopMove(uint8_t channel);

    // ====================================================================
    // Pulse Width Control
    // ====================================================================

    /**
     * @brief Set servo pulse width directly
     * @param channel Servo channel (0 or 1)
     * @param pulse_us Pulse width in microseconds
     * @return CUBE32_OK on success
     */
    cube32_result_t setPulseWidth(uint8_t channel, uint16_t pulse_us);

    // ====================================================================
    // Attach / Detach
    // ====================================================================

    /**
     * @brief Attach servo (enable PWM output)
     * @param channel Servo channel (0 or 1)
     * @return CUBE32_OK on success
     */
    cube32_result_t attach(uint8_t channel);

    /**
     * @brief Detach servo (stop PWM output, power saving)
     * @param channel Servo channel (0 or 1)
     * @return CUBE32_OK on success
     */
    cube32_result_t detach(uint8_t channel);

    /**
     * @brief Check if servo channel is attached
     * @param channel Servo channel (0 or 1)
     * @return true if attached
     */
    bool isAttached(uint8_t channel) const;

    // ====================================================================
    // Configuration Getters
    // ====================================================================

    /**
     * @brief Get number of configured channels
     */
    uint8_t getNumChannels() const { return m_config.num_channels; }

    /**
     * @brief Get max rotation for a channel
     * @param channel Servo channel (0 or 1)
     * @return Max rotation in degrees
     */
    uint16_t getMaxRotation(uint8_t channel) const;

    /**
     * @brief Get signal pin for a channel
     * @param channel Servo channel (0 or 1)
     * @return GPIO pin number
     */
    gpio_num_t getSignalPin(uint8_t channel) const;

    /**
     * @brief Get channel state
     * @param channel Servo channel (0 or 1)
     * @return Pointer to channel state, or nullptr
     */
    const ServoChannelState* getChannelState(uint8_t channel) const;

private:
    PwmServo() = default;

    /**
     * @brief Convert angle to LEDC duty value
     */
    uint32_t angleToDuty(uint8_t channel, float angle) const;

    /**
     * @brief Convert pulse width (µs) to LEDC duty value
     */
    uint32_t pulseUsToDuty(uint16_t pulse_us) const;

    /**
     * @brief Background task for smooth movement
     */
    static void smoothMoveTask(void* param);

    bool m_initialized = false;
    cube32_servo_config_t m_config = {};
    ServoChannelState m_state[CUBE32_SERVO_MAX_CHANNELS] = {};
    TaskHandle_t m_move_task = nullptr;
    SemaphoreHandle_t m_mutex = nullptr;
};

} // namespace cube32

#endif // __cplusplus

#endif // CUBE32_DRIVERS_SERVO_PWM_SERVO_H
