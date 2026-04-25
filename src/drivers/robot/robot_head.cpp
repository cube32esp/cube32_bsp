/**
 * @file robot_head.cpp
 * @brief CUBE32 Robot Head Action Engine implementation
 */

#include "drivers/robot/robot_head.h"

#include <esp_log.h>
#include <cstring>
#include <cmath>
#include <algorithm>

#ifdef CONFIG_CUBE32_ROBOT_HEAD_ENABLED

static const char* TAG = "cube32_robot_head";

namespace cube32 {

// ============================================================================
// Built-in Action Keyframe Definitions
// ============================================================================

// Offsets are relative to center position.
// The actual angles are computed at runtime: center + offset.
// HEAD_ANGLE_HOLD means "don't move this axis".

// --- NOD (Agreement) ---
// 3 nods: tilt down, up, down, up, down, center. Pan holds.
static const HeadKeyframe s_nod_keyframes[] = {
    // pan,            tilt,           pan_spd, tilt_spd, delay_ms
    { HEAD_ANGLE_HOLD, -20.0f,         50, 70, 100 },  // tilt down
    { HEAD_ANGLE_HOLD,  10.0f,         50, 70, 100 },  // tilt up
    { HEAD_ANGLE_HOLD, -20.0f,         50, 70, 100 },  // tilt down
    { HEAD_ANGLE_HOLD,  10.0f,         50, 70, 100 },  // tilt up
    { HEAD_ANGLE_HOLD, -20.0f,         50, 70, 100 },  // tilt down
    { HEAD_ANGLE_HOLD,   0.0f,         50, 50, 200 },  // return center
};

// --- SHAKE (Disagreement) ---
// 3 shakes: pan left, right, left, right, left, center. Tilt holds.
static const HeadKeyframe s_shake_keyframes[] = {
    {  30.0f, HEAD_ANGLE_HOLD,         70, 50, 100 },  // pan left
    { -30.0f, HEAD_ANGLE_HOLD,         70, 50, 100 },  // pan right
    {  30.0f, HEAD_ANGLE_HOLD,         70, 50, 100 },  // pan left
    { -30.0f, HEAD_ANGLE_HOLD,         70, 50, 100 },  // pan right
    {  30.0f, HEAD_ANGLE_HOLD,         70, 50, 100 },  // pan left
    {   0.0f, HEAD_ANGLE_HOLD,         50, 50, 200 },  // return center
};

// --- CURIOUS (Puzzle/Confusion) ---
// Slow pan left + slight tilt, hold, return
static const HeadKeyframe s_curious_keyframes[] = {
    {  20.0f, -15.0f,                  30, 30, 1500 }, // pan left + tilt down, hold
    {   0.0f,   0.0f,                  30, 30, 300 },  // return center
};

// --- ATTENTION (Reset to Center) ---
// Quick snap to center on both axes
static const HeadKeyframe s_attention_keyframes[] = {
    {   0.0f,   0.0f,                  80, 80, 100 },  // snap to center
};

// --- LOOK LEFT ---
// Turn head left 45°, pause 1s, return
static const HeadKeyframe s_look_left_keyframes[] = {
    {  45.0f, HEAD_ANGLE_HOLD,         40, 50, 1000 }, // pan left 45°, hold 1s
    {   0.0f, HEAD_ANGLE_HOLD,         40, 50, 200 },  // return center
};

// --- LOOK RIGHT ---
// Turn head right 45°, pause 1s, return
static const HeadKeyframe s_look_right_keyframes[] = {
    { -45.0f, HEAD_ANGLE_HOLD,         40, 50, 1000 }, // pan right 45°, hold 1s
    {   0.0f, HEAD_ANGLE_HOLD,         40, 50, 200 },  // return center
};

// --- LOOK UP ---
// Tilt up, pause 1s, return
static const HeadKeyframe s_look_up_keyframes[] = {
    { HEAD_ANGLE_HOLD,  25.0f,         50, 40, 1000 }, // tilt up, hold 1s
    { HEAD_ANGLE_HOLD,   0.0f,         50, 40, 200 },  // return center
};

// --- LOOK DOWN ---
// Tilt down, pause 1s, return
static const HeadKeyframe s_look_down_keyframes[] = {
    { HEAD_ANGLE_HOLD, -25.0f,         50, 40, 1000 }, // tilt down, hold 1s
    { HEAD_ANGLE_HOLD,   0.0f,         50, 40, 200 },  // return center
};

// --- SCAN (Alert) ---
// Slow 180° panoramic scan: left 90° → right 90° → center, hold tilt
static const HeadKeyframe s_scan_keyframes[] = {
    {  90.0f, HEAD_ANGLE_HOLD,         25, 50, 300 },  // pan left 90°
    { -90.0f, HEAD_ANGLE_HOLD,         25, 50, 300 },  // pan right 90°
    {   0.0f, HEAD_ANGLE_HOLD,         25, 50, 200 },  // return center
};

// --- BOW ---
// Respectful bow: slow tilt down to limit, hold 2s, slow return
static const HeadKeyframe s_bow_keyframes[] = {
    { HEAD_ANGLE_HOLD, -30.0f,         50, 20, 2000 }, // slow tilt down, hold 2s
    { HEAD_ANGLE_HOLD,   0.0f,         50, 20, 200 },  // slow return center
};

// --- SEARCH (360) ---
// Full 360° slow rotation with slight periodic tilt changes
static const HeadKeyframe s_search_keyframes[] = {
    {  60.0f, -10.0f,                  20, 30, 800 },  // pan left + tilt down
    {  60.0f,  10.0f,                  20, 30, 800 },  // continue left + tilt up
    {  60.0f, -10.0f,                  20, 30, 800 },  // continue left + tilt down
    {  60.0f,  10.0f,                  20, 30, 800 },  // continue left + tilt up
    {  60.0f, -10.0f,                  20, 30, 800 },  // continue left + tilt down
    {  60.0f,   0.0f,                  20, 30, 800 },  // continue left + tilt center
    {   0.0f,   0.0f,                  50, 50, 200 },  // stop rotation, center tilt
};

// --- EXCITED (Happy) ---
// Gentle nods + small left-right wiggle — playful but stable
static const HeadKeyframe s_excited_keyframes[] = {
    {   8.0f,  -8.0f,                  50, 50, 150 },  // wiggle right + bob down
    {  -8.0f,   5.0f,                  50, 50, 150 },  // wiggle left + bob up
    {   8.0f,  -8.0f,                  50, 50, 150 },  // wiggle right + bob down
    {  -8.0f,   5.0f,                  50, 50, 150 },  // wiggle left + bob up
    {   8.0f,  -8.0f,                  50, 50, 150 },  // wiggle right + bob down
    {  -8.0f,   5.0f,                  50, 50, 150 },  // wiggle left + bob up
    {   0.0f,   0.0f,                  40, 40, 200 },  // return center
};

// --- SAD ---
// Slow head drop, hold 3s, slow return — dejected look
static const HeadKeyframe s_sad_keyframes[] = {
    { HEAD_ANGLE_HOLD, -25.0f,         50, 15, 500 },  // slow tilt down
    { -10.0f,          -25.0f,         15, 15, 3000 }, // slight pan right + hold 3s
    {   0.0f,            0.0f,         20, 20, 300 },  // slow return center
};

// --- DOUBLE-TAKE ---
// Quick look right, pause briefly, snap back with slight upward tilt
static const HeadKeyframe s_double_take_keyframes[] = {
    { -30.0f, HEAD_ANGLE_HOLD,         60, 50, 300 },  // quick pan right
    { -30.0f, HEAD_ANGLE_HOLD,         60, 50, 200 },  // brief pause
    {   0.0f,  10.0f,                  90, 70, 400 },  // snap back + slight up
    {   0.0f,   0.0f,                  40, 40, 200 },  // settle to center
};

// --- DIZZY (Funny) ---
// Wobbly oscillation with decreasing amplitude — dazed look
static const HeadKeyframe s_dizzy_keyframes[] = {
    {  60.0f,  15.0f,                  70, 50, 150 },  // big swing left + tilt up
    { -60.0f, -15.0f,                  70, 50, 150 },  // big swing right + tilt down
    {  40.0f,  10.0f,                  60, 40, 150 },  // medium swing left
    { -40.0f, -10.0f,                  60, 40, 150 },  // medium swing right
    {  25.0f,   8.0f,                  50, 35, 150 },  // small swing left
    { -25.0f,  -8.0f,                  50, 35, 150 },  // small swing right
    {  12.0f,   4.0f,                  40, 30, 200 },  // tiny swing left
    { -12.0f,  -4.0f,                  40, 30, 200 },  // tiny swing right
    {   0.0f,   0.0f,                  30, 30, 300 },  // settle to center
};

// ============================================================================
// Built-in Actions Table
// ============================================================================

static const HeadAction s_builtin_actions[] = {
    { "nod",        s_nod_keyframes,       sizeof(s_nod_keyframes) / sizeof(s_nod_keyframes[0]) },
    { "shake",      s_shake_keyframes,     sizeof(s_shake_keyframes) / sizeof(s_shake_keyframes[0]) },
    { "curious",    s_curious_keyframes,   sizeof(s_curious_keyframes) / sizeof(s_curious_keyframes[0]) },
    { "attention",  s_attention_keyframes, sizeof(s_attention_keyframes) / sizeof(s_attention_keyframes[0]) },
    { "look_left",  s_look_left_keyframes, sizeof(s_look_left_keyframes) / sizeof(s_look_left_keyframes[0]) },
    { "look_right", s_look_right_keyframes,sizeof(s_look_right_keyframes) / sizeof(s_look_right_keyframes[0]) },
    { "look_up",    s_look_up_keyframes,   sizeof(s_look_up_keyframes) / sizeof(s_look_up_keyframes[0]) },
    { "look_down",  s_look_down_keyframes, sizeof(s_look_down_keyframes) / sizeof(s_look_down_keyframes[0]) },
    { "scan",       s_scan_keyframes,      sizeof(s_scan_keyframes) / sizeof(s_scan_keyframes[0]) },
    { "bow",        s_bow_keyframes,       sizeof(s_bow_keyframes) / sizeof(s_bow_keyframes[0]) },
    { "search",     s_search_keyframes,    sizeof(s_search_keyframes) / sizeof(s_search_keyframes[0]) },
    { "excited",    s_excited_keyframes,   sizeof(s_excited_keyframes) / sizeof(s_excited_keyframes[0]) },
    { "sad",        s_sad_keyframes,       sizeof(s_sad_keyframes) / sizeof(s_sad_keyframes[0]) },
    { "double_take",s_double_take_keyframes,sizeof(s_double_take_keyframes) / sizeof(s_double_take_keyframes[0]) },
    { "dizzy",      s_dizzy_keyframes,     sizeof(s_dizzy_keyframes) / sizeof(s_dizzy_keyframes[0]) },
};

static const char* s_action_names[] = {
    "nod", "shake", "curious", "attention",
    "look_left", "look_right", "look_up", "look_down",
    "scan", "bow", "search",
    "excited", "sad", "double_take", "dizzy", nullptr
};

static_assert(sizeof(s_builtin_actions) / sizeof(s_builtin_actions[0]) == (uint8_t)HeadActionId::COUNT,
              "Built-in actions table size must match HeadActionId::COUNT");

// ============================================================================
// Singleton
// ============================================================================

RobotHead& RobotHead::instance() {
    static RobotHead s_instance;
    return s_instance;
}

RobotHead::~RobotHead() {
    end();
}

// ============================================================================
// Initialization
// ============================================================================

cube32_result_t RobotHead::begin(IServoDriver* driver) {
    RobotHeadConfig config = CUBE32_ROBOT_HEAD_CONFIG_DEFAULT();
    return begin(driver, config);
}

cube32_result_t RobotHead::begin(IServoDriver* driver, const RobotHeadConfig& config) {
    if (m_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return CUBE32_OK;
    }

    if (!driver) {
        ESP_LOGE(TAG, "Servo driver is null");
        return CUBE32_INVALID_ARG;
    }

    if (!driver->isReady()) {
        ESP_LOGE(TAG, "Servo driver is not ready");
        return CUBE32_NOT_INITIALIZED;
    }

    if (config.pan_channel >= driver->getNumChannels() ||
        config.tilt_channel >= driver->getNumChannels()) {
        ESP_LOGE(TAG, "Invalid channel assignment: pan=%d, tilt=%d (driver has %d channels)",
                 config.pan_channel, config.tilt_channel, driver->getNumChannels());
        return CUBE32_INVALID_ARG;
    }

    m_driver = driver;
    m_config = config;

    m_mutex = xSemaphoreCreateMutex();
    if (!m_mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return CUBE32_NO_MEM;
    }

    m_playing = false;
    m_cancel = false;
    m_task = nullptr;

    m_initialized = true;

    ESP_LOGI(TAG, "Robot Head initialized: pan=ch%d(center=%.0f°, speed=%.0f%%), tilt=ch%d(center=%.0f°, range=%.0f°-%.0f°, speed=%.0f%%)",
             m_config.pan_channel, m_config.pan_center, m_config.pan_speed_pct,
             m_config.tilt_channel, m_config.tilt_center,
             m_config.tilt_min, m_config.tilt_max, m_config.tilt_speed_pct);

    return CUBE32_OK;
}

cube32_result_t RobotHead::end() {
    if (!m_initialized) return CUBE32_OK;

    stopAction();

    // Wait for task to finish
    for (int i = 0; i < 50 && m_task; i++) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    if (m_mutex) {
        vSemaphoreDelete(m_mutex);
        m_mutex = nullptr;
    }

    m_driver = nullptr;
    m_initialized = false;

    ESP_LOGI(TAG, "Robot Head deinitialized");
    return CUBE32_OK;
}

// ============================================================================
// Action Execution
// ============================================================================

cube32_result_t RobotHead::playAction(HeadActionId id) {
    if ((uint8_t)id >= (uint8_t)HeadActionId::COUNT) {
        return CUBE32_INVALID_ARG;
    }
    return playAction(s_builtin_actions[(uint8_t)id]);
}

cube32_result_t RobotHead::playAction(const HeadAction& action) {
    if (!m_initialized) return CUBE32_NOT_INITIALIZED;
    if (!action.keyframes || action.num_keyframes == 0) return CUBE32_INVALID_ARG;

    // Stop any running action first
    if (m_playing) {
        stopAction();
        // Wait for task cleanup
        for (int i = 0; i < 50 && m_task; i++) {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }

    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(500))) {
        m_keyframes = action.keyframes;
        m_num_keyframes = action.num_keyframes;
        m_current_action_name = action.name ? action.name : "custom";
        m_cancel = false;
        m_playing = true;
        xSemaphoreGive(m_mutex);
    } else {
        return CUBE32_ERROR;
    }

    // Create background task
    BaseType_t ret = xTaskCreate(
        actionTask,
        "robot_head",
        4096,
        this,
        5,
        &m_task
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create action task");
        m_playing = false;
        return CUBE32_NO_MEM;
    }

    ESP_LOGI(TAG, "Playing action: %s (%d keyframes)", m_current_action_name, m_num_keyframes);
    return CUBE32_OK;
}

cube32_result_t RobotHead::playActionByName(const char* name) {
    if (!name) return CUBE32_INVALID_ARG;

    for (uint8_t i = 0; i < (uint8_t)HeadActionId::COUNT; i++) {
        if (strcasecmp(name, s_builtin_actions[i].name) == 0) {
            return playAction((HeadActionId)i);
        }
    }

    ESP_LOGW(TAG, "Unknown action: %s", name);
    return CUBE32_INVALID_ARG;
}

cube32_result_t RobotHead::stopAction() {
    if (!m_initialized) return CUBE32_NOT_INITIALIZED;

    m_cancel = true;

    // Stop any ongoing servo moves
    m_driver->stopMove(m_config.pan_channel);
    m_driver->stopMove(m_config.tilt_channel);

    return CUBE32_OK;
}

// ============================================================================
// Static Accessors
// ============================================================================

const HeadAction* RobotHead::getBuiltinAction(HeadActionId id) {
    if ((uint8_t)id >= (uint8_t)HeadActionId::COUNT) return nullptr;
    return &s_builtin_actions[(uint8_t)id];
}

uint8_t RobotHead::getBuiltinActionCount() {
    return (uint8_t)HeadActionId::COUNT;
}

const char* const* RobotHead::getBuiltinActionNames() {
    return s_action_names;
}

// ============================================================================
// Internal Helpers
// ============================================================================

float RobotHead::clampTilt(float angle) const {
    return std::max(m_config.tilt_min, std::min(angle, m_config.tilt_max));
}

bool RobotHead::waitForMoveComplete(uint16_t timeout_ms) {
    const uint16_t poll_ms = 20;
    uint16_t elapsed = 0;

    while (elapsed < timeout_ms && !m_cancel) {
        bool pan_done = !m_driver->isMoving(m_config.pan_channel);
        bool tilt_done = !m_driver->isMoving(m_config.tilt_channel);
        if (pan_done && tilt_done) return true;
        vTaskDelay(pdMS_TO_TICKS(poll_ms));
        elapsed += poll_ms;
    }

    return !m_cancel;
}

void RobotHead::executeKeyframes(const HeadKeyframe* keyframes, uint8_t count) {
    for (uint8_t i = 0; i < count && !m_cancel; i++) {
        const HeadKeyframe& kf = keyframes[i];

        // Convert relative offsets to absolute angles
        // Pan: center + offset (for 360° servo, we set absolute directly)
        bool move_pan = (kf.pan_angle != HEAD_ANGLE_HOLD);
        bool move_tilt = (kf.tilt_angle != HEAD_ANGLE_HOLD);

        if (move_pan) {
            float abs_pan = m_config.pan_center + kf.pan_angle;
            // For 360° pan servo, use setAngle directly (it controls speed/direction)
            uint16_t pan_rot = m_driver->getMaxRotation(m_config.pan_channel);
            if (pan_rot == 360) {
                // 360° servo: 0=CW, 90=stop, 180=CCW
                // Map signed offset: positive=left(CCW), negative=right(CW)
                // Speed proportional to |offset|, direction from sign
                // Scale by global pan speed setting
                float speed = std::min(std::fabs(kf.pan_angle) / 90.0f, 1.0f);
                speed *= m_config.pan_speed_pct / 100.0f;
                float servo_angle;
                if (std::fabs(kf.pan_angle) < 0.5f) {
                    servo_angle = 90.0f; // stop
                } else if (kf.pan_angle > 0) {
                    servo_angle = 90.0f + speed * 90.0f; // CCW (left)
                } else {
                    servo_angle = 90.0f - speed * 90.0f; // CW (right)
                }
                m_driver->setAngle(m_config.pan_channel, servo_angle);
            } else {
                // Standard servo: smooth move to absolute position
                abs_pan = std::max(0.0f, std::min(abs_pan, (float)pan_rot));
                float eff_pan_spd = kf.pan_speed_pct * m_config.pan_speed_pct / 100.0f;
                m_driver->smoothMove(m_config.pan_channel, abs_pan, eff_pan_spd);
            }
        }

        if (move_tilt) {
            float abs_tilt = m_config.tilt_center + kf.tilt_angle;
            abs_tilt = clampTilt(abs_tilt);
            float eff_tilt_spd = kf.tilt_speed_pct * m_config.tilt_speed_pct / 100.0f;
            m_driver->smoothMove(m_config.tilt_channel, abs_tilt, eff_tilt_spd);
        }

        // Wait for moves to complete (non-360° axes)
        uint16_t pan_rot = m_driver->getMaxRotation(m_config.pan_channel);
        if (pan_rot != 360) {
            // Wait for smooth move to finish
            waitForMoveComplete(3000);
        } else {
            // For 360° pan we just need to wait for tilt if it moved
            if (move_tilt) {
                // Wait until tilt finishes
                uint16_t elapsed = 0;
                while (elapsed < 3000 && !m_cancel) {
                    if (!m_driver->isMoving(m_config.tilt_channel)) break;
                    vTaskDelay(pdMS_TO_TICKS(20));
                    elapsed += 20;
                }
            }
        }

        if (m_cancel) break;

        // Post-keyframe delay
        if (kf.delay_ms > 0) {
            uint16_t elapsed = 0;
            while (elapsed < kf.delay_ms && !m_cancel) {
                uint16_t chunk = std::min((uint16_t)50, (uint16_t)(kf.delay_ms - elapsed));
                vTaskDelay(pdMS_TO_TICKS(chunk));
                elapsed += chunk;
            }
        }
    }

    // If we used 360° pan, stop it at the end
    uint16_t pan_rot = m_driver->getMaxRotation(m_config.pan_channel);
    if (pan_rot == 360 && !m_cancel) {
        m_driver->setAngle(m_config.pan_channel, 90.0f); // stop rotation
    }
}

// ============================================================================
// Background Task
// ============================================================================

void RobotHead::actionTask(void* param) {
    RobotHead* self = static_cast<RobotHead*>(param);

    self->executeKeyframes(self->m_keyframes, self->m_num_keyframes);

    self->m_playing = false;
    self->m_task = nullptr;
    ESP_LOGI(TAG, "Action '%s' %s", self->m_current_action_name,
             self->m_cancel ? "cancelled" : "completed");
    vTaskDelete(nullptr);
}

} // namespace cube32

#endif // CONFIG_CUBE32_ROBOT_HEAD_ENABLED
