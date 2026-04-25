/**
 * @file pwm_servo.cpp
 * @brief CUBE32 PWM Servo Driver Implementation
 * 
 * Uses the ESP32-S3 LEDC peripheral for hardware PWM generation.
 * At 50 Hz with 14-bit resolution (16384 ticks), the minimum step
 * is ~1.22 µs which provides roughly 0.1° positioning accuracy.
 */

#include "drivers/servo/pwm_servo.h"

#include <esp_log.h>
#include <cmath>
#include <algorithm>

static const char* TAG = "cube32_servo";

// Smooth move task configuration
#define SMOOTH_MOVE_TASK_STACK  2048
#define SMOOTH_MOVE_TASK_PRIO   3
#define SMOOTH_MOVE_STEP_MS     20      // 20ms per step (50 steps/sec)
#define MAX_SPEED_DEG_PER_SEC   360.0f  // Max speed at 100%

namespace cube32 {

// ============================================================================
// Singleton Implementation
// ============================================================================

PwmServo& PwmServo::instance() {
    static PwmServo s_instance;
    return s_instance;
}

PwmServo::~PwmServo() {
    if (m_initialized) {
        end();
    }
}

// ============================================================================
// Initialization
// ============================================================================

cube32_result_t PwmServo::begin() {
#ifdef CONFIG_CUBE32_SERVO_ENABLED
    cube32_servo_config_t config = CUBE32_SERVO_CONFIG_DEFAULT();
    return begin(config);
#else
    ESP_LOGE(TAG, "PWM Servo not enabled in Kconfig");
    return CUBE32_NOT_SUPPORTED;
#endif
}

cube32_result_t PwmServo::begin(const cube32_servo_config_t& config) {
    if (m_initialized) {
        ESP_LOGW(TAG, "PWM Servo already initialized");
        return CUBE32_ALREADY_INITIALIZED;
    }

    if (config.num_channels == 0 || config.num_channels > CUBE32_SERVO_MAX_CHANNELS) {
        ESP_LOGE(TAG, "Invalid number of channels: %d", config.num_channels);
        return CUBE32_INVALID_ARG;
    }

    m_config = config;

    ESP_LOGI(TAG, "Initializing PWM Servo driver...");
    ESP_LOGI(TAG, "  Channels: %d, Frequency: %" PRIu32 " Hz, Resolution: %d-bit",
             config.num_channels, config.freq_hz, config.resolution);

    // Create mutex
    m_mutex = xSemaphoreCreateMutex();
    if (!m_mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return CUBE32_NO_MEM;
    }

    // Configure LEDC timer
    ledc_timer_config_t timer_conf = {};
    timer_conf.speed_mode = LEDC_LOW_SPEED_MODE;
    timer_conf.duty_resolution = config.resolution;
    timer_conf.timer_num = config.timer;
    timer_conf.freq_hz = config.freq_hz;
    timer_conf.clk_cfg = LEDC_AUTO_CLK;

    esp_err_t err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC timer: %s", esp_err_to_name(err));
        vSemaphoreDelete(m_mutex);
        m_mutex = nullptr;
        return esp_err_to_cube32(err);
    }

    // Configure each channel
    for (uint8_t i = 0; i < config.num_channels; i++) {
        const cube32_servo_channel_config_t& ch = config.channels[i];

        ESP_LOGI(TAG, "  Ch%d: GPIO%d, %d°, pulse %d-%d µs",
                 i, ch.signal_pin, (int)ch.max_rotation,
                 ch.min_pulse_us, ch.max_pulse_us);

        // Set initial position to center
        float center = (ch.max_rotation == CUBE32_SERVO_360_DEG) ? 90.0f : (ch.max_rotation / 2.0f);
        uint32_t duty = angleToDuty(i, center);

        ledc_channel_config_t ch_conf = {};
        ch_conf.gpio_num = ch.signal_pin;
        ch_conf.speed_mode = LEDC_LOW_SPEED_MODE;
        ch_conf.channel = ch.ledc_channel;
        ch_conf.timer_sel = config.timer;
        ch_conf.duty = duty;
        ch_conf.hpoint = 0;

        err = ledc_channel_config(&ch_conf);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure LEDC channel %d: %s", i, esp_err_to_name(err));
            // Clean up previously configured channels
            for (uint8_t j = 0; j < i; j++) {
                ledc_stop(LEDC_LOW_SPEED_MODE, config.channels[j].ledc_channel, 0);
            }
            vSemaphoreDelete(m_mutex);
            m_mutex = nullptr;
            return esp_err_to_cube32(err);
        }

        // Initialize state
        m_state[i].attached = true;
        m_state[i].current_angle = center;
        m_state[i].moving = false;
        m_state[i].target_angle = center;
        m_state[i].move_speed = 0;
    }

    m_initialized = true;
    ESP_LOGI(TAG, "PWM Servo driver initialized successfully");
    return CUBE32_OK;
}

cube32_result_t PwmServo::end() {
    if (!m_initialized) {
        return CUBE32_NOT_INITIALIZED;
    }

    // Stop any smooth move
    for (uint8_t i = 0; i < m_config.num_channels; i++) {
        m_state[i].moving = false;
    }

    // Wait for move task to finish
    if (m_move_task) {
        vTaskDelay(pdMS_TO_TICKS(SMOOTH_MOVE_STEP_MS * 2));
        m_move_task = nullptr;
    }

    // Stop all LEDC channels
    for (uint8_t i = 0; i < m_config.num_channels; i++) {
        ledc_stop(LEDC_LOW_SPEED_MODE, m_config.channels[i].ledc_channel, 0);
        m_state[i].attached = false;
    }

    if (m_mutex) {
        vSemaphoreDelete(m_mutex);
        m_mutex = nullptr;
    }

    m_initialized = false;
    ESP_LOGI(TAG, "PWM Servo driver deinitialized");
    return CUBE32_OK;
}

// ============================================================================
// Angle Control
// ============================================================================

cube32_result_t PwmServo::setAngle(uint8_t channel, float angle) {
    if (!m_initialized) return CUBE32_NOT_INITIALIZED;
    if (channel >= m_config.num_channels) return CUBE32_INVALID_ARG;
    if (!m_state[channel].attached) return CUBE32_ERROR;

    const cube32_servo_channel_config_t& ch = m_config.channels[channel];

    // Clamp angle to valid range
    float max_angle = (float)ch.max_rotation;
    if (ch.max_rotation == CUBE32_SERVO_360_DEG) {
        // For 360° servos, valid range is 0-180 (speed/direction control)
        max_angle = 180.0f;
    }
    angle = std::max(0.0f, std::min(angle, max_angle));

    uint32_t duty = angleToDuty(channel, angle);

    esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, ch.ledc_channel, duty);
    if (err != ESP_OK) return esp_err_to_cube32(err);

    err = ledc_update_duty(LEDC_LOW_SPEED_MODE, ch.ledc_channel);
    if (err != ESP_OK) return esp_err_to_cube32(err);

    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(100))) {
        m_state[channel].current_angle = angle;
        xSemaphoreGive(m_mutex);
    }

    return CUBE32_OK;
}

float PwmServo::getAngle(uint8_t channel) const {
    if (!m_initialized || channel >= m_config.num_channels) return -1.0f;
    return m_state[channel].current_angle;
}

cube32_result_t PwmServo::smoothMove(uint8_t channel, float target_angle, float speed_pct) {
    if (!m_initialized) return CUBE32_NOT_INITIALIZED;
    if (channel >= m_config.num_channels) return CUBE32_INVALID_ARG;
    if (!m_state[channel].attached) return CUBE32_ERROR;

    const cube32_servo_channel_config_t& ch = m_config.channels[channel];

    // 360° continuous servos don't support smooth move
    if (ch.max_rotation == CUBE32_SERVO_360_DEG) {
        ESP_LOGW(TAG, "Smooth move not supported for 360° continuous rotation servo");
        return CUBE32_NOT_SUPPORTED;
    }

    // Clamp parameters
    float max_angle = (float)ch.max_rotation;
    target_angle = std::max(0.0f, std::min(target_angle, max_angle));
    speed_pct = std::max(1.0f, std::min(speed_pct, 100.0f));

    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(100))) {
        m_state[channel].target_angle = target_angle;
        m_state[channel].move_speed = (speed_pct / 100.0f) * MAX_SPEED_DEG_PER_SEC;
        m_state[channel].moving = true;
        xSemaphoreGive(m_mutex);
    }

    // Create the move task if not already running
    if (!m_move_task) {
        BaseType_t ret = xTaskCreate(
            smoothMoveTask,
            "servo_move",
            SMOOTH_MOVE_TASK_STACK,
            this,
            SMOOTH_MOVE_TASK_PRIO,
            &m_move_task
        );
        if (ret != pdPASS) {
            ESP_LOGE(TAG, "Failed to create smooth move task");
            m_state[channel].moving = false;
            return CUBE32_NO_MEM;
        }
    }

    return CUBE32_OK;
}

bool PwmServo::isMoving(uint8_t channel) const {
    if (!m_initialized || channel >= m_config.num_channels) return false;
    return m_state[channel].moving;
}

cube32_result_t PwmServo::stopMove(uint8_t channel) {
    if (!m_initialized) return CUBE32_NOT_INITIALIZED;
    if (channel >= m_config.num_channels) return CUBE32_INVALID_ARG;

    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(100))) {
        m_state[channel].moving = false;
        xSemaphoreGive(m_mutex);
    }

    return CUBE32_OK;
}

// ============================================================================
// Pulse Width Control
// ============================================================================

cube32_result_t PwmServo::setPulseWidth(uint8_t channel, uint16_t pulse_us) {
    if (!m_initialized) return CUBE32_NOT_INITIALIZED;
    if (channel >= m_config.num_channels) return CUBE32_INVALID_ARG;
    if (!m_state[channel].attached) return CUBE32_ERROR;

    const cube32_servo_channel_config_t& ch = m_config.channels[channel];

    // Clamp to configured range
    pulse_us = std::max(ch.min_pulse_us, std::min(pulse_us, ch.max_pulse_us));

    uint32_t duty = pulseUsToDuty(pulse_us);

    esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, ch.ledc_channel, duty);
    if (err != ESP_OK) return esp_err_to_cube32(err);

    err = ledc_update_duty(LEDC_LOW_SPEED_MODE, ch.ledc_channel);
    if (err != ESP_OK) return esp_err_to_cube32(err);

    // Update current angle based on pulse width
    float range_us = (float)(ch.max_pulse_us - ch.min_pulse_us);
    float max_angle = (ch.max_rotation == CUBE32_SERVO_360_DEG) ? 180.0f : (float)ch.max_rotation;
    float angle = ((float)(pulse_us - ch.min_pulse_us) / range_us) * max_angle;

    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(100))) {
        m_state[channel].current_angle = angle;
        xSemaphoreGive(m_mutex);
    }

    return CUBE32_OK;
}

// ============================================================================
// Attach / Detach
// ============================================================================

cube32_result_t PwmServo::attach(uint8_t channel) {
    if (!m_initialized) return CUBE32_NOT_INITIALIZED;
    if (channel >= m_config.num_channels) return CUBE32_INVALID_ARG;

    if (m_state[channel].attached) {
        return CUBE32_OK;  // Already attached
    }

    const cube32_servo_channel_config_t& ch = m_config.channels[channel];
    uint32_t duty = angleToDuty(channel, m_state[channel].current_angle);

    ledc_channel_config_t ch_conf = {};
    ch_conf.gpio_num = ch.signal_pin;
    ch_conf.speed_mode = LEDC_LOW_SPEED_MODE;
    ch_conf.channel = ch.ledc_channel;
    ch_conf.timer_sel = m_config.timer;
    ch_conf.duty = duty;
    ch_conf.hpoint = 0;

    esp_err_t err = ledc_channel_config(&ch_conf);
    if (err != ESP_OK) return esp_err_to_cube32(err);

    m_state[channel].attached = true;
    ESP_LOGI(TAG, "Servo channel %d attached (GPIO%d)", channel, ch.signal_pin);
    return CUBE32_OK;
}

cube32_result_t PwmServo::detach(uint8_t channel) {
    if (!m_initialized) return CUBE32_NOT_INITIALIZED;
    if (channel >= m_config.num_channels) return CUBE32_INVALID_ARG;

    if (!m_state[channel].attached) {
        return CUBE32_OK;  // Already detached
    }

    // Stop any ongoing movement first
    m_state[channel].moving = false;

    const cube32_servo_channel_config_t& ch = m_config.channels[channel];
    ledc_stop(LEDC_LOW_SPEED_MODE, ch.ledc_channel, 0);

    m_state[channel].attached = false;
    ESP_LOGI(TAG, "Servo channel %d detached (GPIO%d)", channel, ch.signal_pin);
    return CUBE32_OK;
}

bool PwmServo::isAttached(uint8_t channel) const {
    if (!m_initialized || channel >= m_config.num_channels) return false;
    return m_state[channel].attached;
}

// ============================================================================
// Configuration Getters
// ============================================================================

uint16_t PwmServo::getMaxRotation(uint8_t channel) const {
    if (!m_initialized || channel >= m_config.num_channels) return 0;
    return (uint16_t)m_config.channels[channel].max_rotation;
}

gpio_num_t PwmServo::getSignalPin(uint8_t channel) const {
    if (!m_initialized || channel >= m_config.num_channels) return GPIO_NUM_NC;
    return m_config.channels[channel].signal_pin;
}

const ServoChannelState* PwmServo::getChannelState(uint8_t channel) const {
    if (!m_initialized || channel >= m_config.num_channels) return nullptr;
    return &m_state[channel];
}

// ============================================================================
// Private Helpers
// ============================================================================

uint32_t PwmServo::angleToDuty(uint8_t channel, float angle) const {
    const cube32_servo_channel_config_t& ch = m_config.channels[channel];

    // Map angle to pulse width
    float max_angle = (ch.max_rotation == CUBE32_SERVO_360_DEG) ? 180.0f : (float)ch.max_rotation;
    float fraction = angle / max_angle;
    float pulse_us = (float)ch.min_pulse_us + fraction * (float)(ch.max_pulse_us - ch.min_pulse_us);

    return pulseUsToDuty((uint16_t)pulse_us);
}

uint32_t PwmServo::pulseUsToDuty(uint16_t pulse_us) const {
    // period_us = 1,000,000 / freq_hz = 20,000 µs at 50 Hz
    float period_us = 1000000.0f / (float)m_config.freq_hz;
    float duty_fraction = (float)pulse_us / period_us;
    return (uint32_t)(duty_fraction * (float)CUBE32_SERVO_RESOLUTION_MAX);
}

void PwmServo::smoothMoveTask(void* param) {
    PwmServo* self = static_cast<PwmServo*>(param);
    
    ESP_LOGI(TAG, "Smooth move task started");

    while (true) {
        bool any_moving = false;

        for (uint8_t ch = 0; ch < self->m_config.num_channels; ch++) {
            if (!self->m_state[ch].moving || !self->m_state[ch].attached) continue;

            any_moving = true;

            float current, target, speed;
            if (xSemaphoreTake(self->m_mutex, pdMS_TO_TICKS(50))) {
                current = self->m_state[ch].current_angle;
                target = self->m_state[ch].target_angle;
                speed = self->m_state[ch].move_speed;
                xSemaphoreGive(self->m_mutex);
            } else {
                continue;
            }

            // Calculate step size based on speed and time interval
            float step = speed * ((float)SMOOTH_MOVE_STEP_MS / 1000.0f);
            float diff = target - current;

            if (fabsf(diff) <= step) {
                // Reached target
                self->setAngle(ch, target);
                if (xSemaphoreTake(self->m_mutex, pdMS_TO_TICKS(50))) {
                    self->m_state[ch].moving = false;
                    xSemaphoreGive(self->m_mutex);
                }
            } else {
                // Move one step toward target
                float next = current + (diff > 0 ? step : -step);
                self->setAngle(ch, next);
            }
        }

        if (!any_moving) {
            // No channels moving, exit task
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(SMOOTH_MOVE_STEP_MS));
    }

    ESP_LOGI(TAG, "Smooth move task finished");
    self->m_move_task = nullptr;
    vTaskDelete(nullptr);
}

} // namespace cube32
