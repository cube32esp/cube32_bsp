/**
 * @file robot_head.h
 * @brief CUBE32 Robot Head Action Engine
 * 
 * High-level robot head simulation using a 2-servo chassis:
 *   - Pan servo (horizontal): 360° continuous — head turning left/right
 *   - Tilt servo (vertical): 90°/180° — head nodding up/down
 * 
 * This library is servo-driver-agnostic. It is programmed against the
 * IServoDriver interface so it can work with PWM servos, one-wire bus
 * servos, or any future servo driver implementation.
 * 
 * Built-in expressions:
 *   - Nod (agreement): head bobs up-down
 *   - Shake (disagreement): head turns left-right
 *   - Curious (puzzle): tilt + slow pan, then hold
 *   - Attention (reset): snap to center position
 *   - Look Left/Right/Up/Down: glance in direction, return
 *   - Scan (alert): slow 180° panoramic scan
 *   - Bow: respectful bow gesture
 *   - Search (360): full rotation scan with tilt
 *   - Excited (happy): quick nods + left-right wiggle
 *   - Sad: slow head drop, hold, slow return
 *   - Double-Take: quick look away then snap back
 *   - Dizzy (funny): wobbly decreasing-amplitude oscillation
 * 
 * Architecture:
 *   Each action is defined as an array of keyframes. A keyframe specifies
 *   a target angle for pan and/or tilt, the movement speed, and a delay
 *   after reaching the target. Actions execute sequentially in a
 *   background FreeRTOS task and can be cancelled at any time.
 */

#ifndef CUBE32_DRIVERS_SERVO_ROBOT_HEAD_H
#define CUBE32_DRIVERS_SERVO_ROBOT_HEAD_H

#include "utils/common.h"
#include "cube32_config.h"
#include "drivers/servo/servo_interface.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Kconfig defaults
// ============================================================================

#ifndef CONFIG_CUBE32_ROBOT_HEAD_PAN_CHANNEL
#define CONFIG_CUBE32_ROBOT_HEAD_PAN_CHANNEL   0
#endif
#ifndef CONFIG_CUBE32_ROBOT_HEAD_TILT_CHANNEL
#define CONFIG_CUBE32_ROBOT_HEAD_TILT_CHANNEL  1
#endif
#ifndef CONFIG_CUBE32_ROBOT_HEAD_PAN_CENTER
#define CONFIG_CUBE32_ROBOT_HEAD_PAN_CENTER    90
#endif
#ifndef CONFIG_CUBE32_ROBOT_HEAD_TILT_CENTER
#define CONFIG_CUBE32_ROBOT_HEAD_TILT_CENTER   90
#endif
#ifndef CONFIG_CUBE32_ROBOT_HEAD_TILT_MIN
#define CONFIG_CUBE32_ROBOT_HEAD_TILT_MIN      60
#endif
#ifndef CONFIG_CUBE32_ROBOT_HEAD_TILT_MAX
#define CONFIG_CUBE32_ROBOT_HEAD_TILT_MAX      120
#endif
#ifndef CONFIG_CUBE32_ROBOT_HEAD_PAN_SPEED
#define CONFIG_CUBE32_ROBOT_HEAD_PAN_SPEED     50
#endif
#ifndef CONFIG_CUBE32_ROBOT_HEAD_TILT_SPEED
#define CONFIG_CUBE32_ROBOT_HEAD_TILT_SPEED    50
#endif

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

namespace cube32 {

// ============================================================================
// Constants
// ============================================================================

/// Special value: keep current angle (don't move this axis)
static constexpr float HEAD_ANGLE_HOLD = -999.0f;

// ============================================================================
// Keyframe Definition
// ============================================================================

/**
 * @brief Single keyframe in a head action sequence
 */
struct HeadKeyframe {
    float pan_angle;        ///< Target pan angle (or HEAD_ANGLE_HOLD to skip)
    float tilt_angle;       ///< Target tilt angle (or HEAD_ANGLE_HOLD to skip)
    float pan_speed_pct;    ///< Pan move speed % (1-100), ignored if hold
    float tilt_speed_pct;   ///< Tilt move speed % (1-100), ignored if hold
    uint16_t delay_ms;      ///< Delay after reaching target (ms)
};

/**
 * @brief Named action: a sequence of keyframes
 */
struct HeadAction {
    const char* name;               ///< Human-readable action name
    const HeadKeyframe* keyframes;  ///< Array of keyframes
    uint8_t num_keyframes;          ///< Number of keyframes in array
};

// ============================================================================
// Robot Head Configuration
// ============================================================================

/**
 * @brief Robot Head configuration
 */
struct RobotHeadConfig {
    uint8_t pan_channel;    ///< Servo channel for horizontal pan
    uint8_t tilt_channel;   ///< Servo channel for vertical tilt
    float pan_center;       ///< Pan center position (degrees)
    float tilt_center;      ///< Tilt center position (degrees)
    float tilt_min;         ///< Minimum tilt angle (degrees) — limit for down
    float tilt_max;         ///< Maximum tilt angle (degrees) — limit for up
    float pan_speed_pct;    ///< Global pan speed scaling (1-100%)
    float tilt_speed_pct;   ///< Global tilt speed scaling (1-100%)
};

/**
 * @brief Default config from Kconfig
 */
#define CUBE32_ROBOT_HEAD_CONFIG_DEFAULT() { \
    .pan_channel    = (uint8_t)CONFIG_CUBE32_ROBOT_HEAD_PAN_CHANNEL, \
    .tilt_channel   = (uint8_t)CONFIG_CUBE32_ROBOT_HEAD_TILT_CHANNEL, \
    .pan_center     = (float)CONFIG_CUBE32_ROBOT_HEAD_PAN_CENTER, \
    .tilt_center    = (float)CONFIG_CUBE32_ROBOT_HEAD_TILT_CENTER, \
    .tilt_min       = (float)CONFIG_CUBE32_ROBOT_HEAD_TILT_MIN, \
    .tilt_max       = (float)CONFIG_CUBE32_ROBOT_HEAD_TILT_MAX, \
    .pan_speed_pct  = (float)CONFIG_CUBE32_ROBOT_HEAD_PAN_SPEED, \
    .tilt_speed_pct = (float)CONFIG_CUBE32_ROBOT_HEAD_TILT_SPEED, \
}

// ============================================================================
// Built-in Action IDs
// ============================================================================

enum class HeadActionId : uint8_t {
    NOD = 0,        ///< Agreement — nod up-down
    SHAKE,          ///< Disagreement — shake left-right
    CURIOUS,        ///< Puzzle — tilt + slow pan, hold
    ATTENTION,      ///< Reset — snap to center
    LOOK_LEFT,      ///< Turn head left, pause, return
    LOOK_RIGHT,     ///< Turn head right, pause, return
    LOOK_UP,        ///< Tilt head up, pause, return
    LOOK_DOWN,      ///< Tilt head down, pause, return
    SCAN,           ///< Alert — slow 180° panoramic scan
    BOW,            ///< Respectful bow gesture
    SEARCH,         ///< Full 360° slow rotation scan
    EXCITED,        ///< Happy — quick nods + left-right wiggle
    SAD,            ///< Sad — slow head drop, hold, return
    DOUBLE_TAKE,    ///< Double-take — quick look away, snap back
    DIZZY,          ///< Funny — wobbly decreasing oscillation
    COUNT           ///< Number of built-in actions
};

// ============================================================================
// RobotHead Class
// ============================================================================

/**
 * @brief Robot Head action engine
 * 
 * Executes keyframe-based head expressions on a 2-axis servo chassis.
 * Works with any servo driver that implements IServoDriver.
 */
class RobotHead {
public:
    /**
     * @brief Get singleton instance
     */
    static RobotHead& instance();

    ~RobotHead();

    RobotHead(const RobotHead&) = delete;
    RobotHead& operator=(const RobotHead&) = delete;

    /**
     * @brief Initialize with servo driver and default Kconfig config
     * @param driver Servo driver implementing IServoDriver (must outlive RobotHead)
     * @return CUBE32_OK on success
     */
    cube32_result_t begin(IServoDriver* driver);

    /**
     * @brief Initialize with servo driver and custom config
     * @param driver Servo driver
     * @param config Robot head configuration
     * @return CUBE32_OK on success
     */
    cube32_result_t begin(IServoDriver* driver, const RobotHeadConfig& config);

    /**
     * @brief Deinitialize
     */
    cube32_result_t end();

    /**
     * @brief Check if initialized
     */
    bool isInitialized() const { return m_initialized; }

    // ====================================================================
    // Action Execution
    // ====================================================================

    /**
     * @brief Play a built-in action by ID
     * @param id Built-in action ID
     * @return CUBE32_OK on success
     */
    cube32_result_t playAction(HeadActionId id);

    /**
     * @brief Play a custom action
     * @param action Action definition with keyframes
     * @return CUBE32_OK on success
     */
    cube32_result_t playAction(const HeadAction& action);

    /**
     * @brief Play a built-in action by name (case-insensitive)
     * @param name Action name ("nod", "shake", "curious", "attention")
     * @return CUBE32_OK if found and started, CUBE32_INVALID_ARG if unknown
     */
    cube32_result_t playActionByName(const char* name);

    /**
     * @brief Stop current action immediately, hold current position
     * @return CUBE32_OK on success
     */
    cube32_result_t stopAction();

    /**
     * @brief Check if an action is currently playing
     */
    bool isPlaying() const { return m_playing; }

    /**
     * @brief Get the name of the currently playing action (or last played)
     */
    const char* getCurrentActionName() const { return m_current_action_name; }

    // ====================================================================
    // Configuration Getters
    // ====================================================================

    const RobotHeadConfig& getConfig() const { return m_config; }

    /**
     * @brief Get built-in action definition by ID
     * @param id Action ID
     * @return Pointer to action definition, or nullptr
     */
    static const HeadAction* getBuiltinAction(HeadActionId id);

    /**
     * @brief Get number of built-in actions
     */
    static uint8_t getBuiltinActionCount();

    /**
     * @brief Get list of built-in action names (null-terminated array)
     */
    static const char* const* getBuiltinActionNames();

private:
    RobotHead() = default;

    /**
     * @brief Clamp tilt angle to configured limits
     */
    float clampTilt(float angle) const;

    /**
     * @brief Execute keyframes sequentially (runs in task)
     */
    void executeKeyframes(const HeadKeyframe* keyframes, uint8_t count);

    /**
     * @brief Wait until both axes finish moving or action is cancelled
     */
    bool waitForMoveComplete(uint16_t timeout_ms = 5000);

    /**
     * @brief Background task entry point
     */
    static void actionTask(void* param);

    bool m_initialized = false;
    volatile bool m_playing = false;
    volatile bool m_cancel = false;

    IServoDriver* m_driver = nullptr;
    RobotHeadConfig m_config = {};

    // Current action being played (copy for task safety)
    const HeadKeyframe* m_keyframes = nullptr;
    uint8_t m_num_keyframes = 0;
    const char* m_current_action_name = "";

    TaskHandle_t m_task = nullptr;
    SemaphoreHandle_t m_mutex = nullptr;
};

} // namespace cube32

#endif // __cplusplus
#endif // CUBE32_DRIVERS_SERVO_ROBOT_HEAD_H
