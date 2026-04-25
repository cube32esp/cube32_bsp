/**
 * @file face_expression.h
 * @brief CUBE32 Robot Face Expression Engine
 * 
 * Renders animated two-eye face expressions on the ST7789 display via LVGL.
 * Each eye consists of:
 *   - Eyeball (white circle background)
 *   - Pupil (dark circle, position/size animated)
 *   - Highlight sparkle (small white dot for liveliness)
 *   - Upper eyelid (rectangle mask for emotion shaping)
 *   - Lower eyelid (rectangle mask for emotion shaping)
 * 
 * Expressions are keyframe-based and designed to synchronize with the
 * RobotHead action engine. When idle, the face performs periodic blinks
 * and micro-saccades (tiny random pupil movements) to appear alive.
 * 
 * Expression IDs mirror HeadActionId for easy 1:1 mapping, plus an
 * IDLE expression for the default living-face state.
 * 
 * Architecture:
 *   - Lightweight: ~12 LVGL objects total, no bitmaps
 *   - Timer-driven: 33ms LVGL timer for smooth 30fps animation
 *   - Thread-safe: All LVGL access through LvglDisplay lock/unlock
 *   - ESP32-S3 friendly: pure LVGL primitives, minimal RAM (~3KB)
 */

#ifndef CUBE32_DRIVERS_ROBOT_FACE_EXPRESSION_H
#define CUBE32_DRIVERS_ROBOT_FACE_EXPRESSION_H

#include "utils/common.h"
#include "cube32_config.h"

#include <lvgl.h>
#include <cstdint>

#ifdef __cplusplus

namespace cube32 {

// ============================================================================
// Face Expression IDs
// ============================================================================

/**
 * @brief Expression IDs matching robot head actions, plus IDLE
 */
enum class FaceExpressionId : uint8_t {
    IDLE = 0,       ///< Default living face (blink + micro-saccades)
    NOD,            ///< Agreement — happy squint, pupils bounce
    SHAKE,          ///< Disagreement — stern, pupils sway
    CURIOUS,        ///< Puzzle — asymmetric, one eye wider
    ATTENTION,      ///< Alert — wide open, pupils center + dilate  
    LOOK_LEFT,      ///< Glancing left — pupils shift left
    LOOK_RIGHT,     ///< Glancing right — pupils shift right
    LOOK_UP,        ///< Looking up — pupils shift up, lids raised
    LOOK_DOWN,      ///< Looking down — pupils shift down, lids droop
    SCAN,           ///< Alert scanning — pupils sweep
    BOW,            ///< Respectful — eyes half-close gracefully
    SEARCH,         ///< Searching — pupils move in pattern
    EXCITED,        ///< Happy — crescents, pupils jitter
    SAD,            ///< Droopy — lids sag, pupils small
    DOUBLE_TAKE,    ///< Surprised — snap + wide eyes
    DIZZY,          ///< Dazed — pupils wobble in circles
    COUNT           ///< Number of expressions
};

// ============================================================================
// Face Keyframe Definition
// ============================================================================

/**
 * @brief Single keyframe in a face expression animation
 * 
 * All positions are relative offsets from center/default values.
 * Eyelid values are in pixels: positive = more closed.
 */
struct FaceKeyframe {
    float pupil_x;          ///< Pupil X offset from eye center (-1.0 to 1.0, normalized)
    float pupil_y;          ///< Pupil Y offset from eye center (-1.0 to 1.0, normalized)
    float pupil_scale;      ///< Pupil size scale (1.0 = normal, <1 = smaller, >1 = larger)
    float upper_lid_y;      ///< Upper eyelid drop in pixels (0 = open, positive = more closed)
    float lower_lid_y;      ///< Lower eyelid rise in pixels (0 = open, positive = more closed)
    float lid_angle;        ///< Eyelid tilt angle for emotion (-1.0 to 1.0: -1=sad, 0=neutral, 1=angry)
    float eye_y_offset;     ///< Vertical offset for both eyes (for nod tracking)
    bool  asymmetric;       ///< If true, right eye uses different lid values
    float r_upper_lid_y;    ///< Right eye upper lid (only if asymmetric)
    float r_lower_lid_y;    ///< Right eye lower lid (only if asymmetric)
    uint16_t duration_ms;   ///< Duration to hold/interpolate to this keyframe
};

/**
 * @brief Named face expression: a sequence of keyframes
 */
struct FaceExpression {
    const char* name;                   ///< Human-readable name
    const FaceKeyframe* keyframes;      ///< Array of keyframes
    uint8_t num_keyframes;              ///< Number of keyframes
    bool loop;                          ///< Whether to loop the animation
};

// ============================================================================
// Face Display Configuration
// ============================================================================

/**
 * @brief Face display geometry configuration  
 * 
 * All values in pixels, designed for 240x240 screen.
 */
struct FaceConfig {
    uint16_t screen_w;      ///< Screen width
    uint16_t screen_h;      ///< Screen height
    uint16_t eye_radius;    ///< Eyeball radius
    uint16_t pupil_radius;  ///< Default pupil radius
    uint16_t highlight_r;   ///< Highlight dot radius
    uint16_t eye_spacing;   ///< Distance between eye centers
    int16_t  eye_y;         ///< Vertical center of eyes from screen center
    uint32_t bg_color;      ///< Background color
    uint32_t eye_color;     ///< Eyeball color (white)
    uint32_t pupil_color;   ///< Pupil color (dark)
    uint32_t highlight_color; ///< Highlight sparkle color
    uint32_t lid_color;     ///< Eyelid color (same as background)
};

/**
 * @brief Default face config for 240x240 screen
 */
#define CUBE32_FACE_CONFIG_DEFAULT() { \
    .screen_w       = 240, \
    .screen_h       = 240, \
    .eye_radius     = 42, \
    .pupil_radius   = 16, \
    .highlight_r    = 4, \
    .eye_spacing    = 100, \
    .eye_y          = 0, \
    .bg_color       = 0x121212, \
    .eye_color      = 0xFFFFFF, \
    .pupil_color    = 0x1A1A2E, \
    .highlight_color= 0xFFFFFF, \
    .lid_color      = 0x121212, \
}

// ============================================================================
// FaceDisplay Class
// ============================================================================

/**
 * @brief Robot face expression display engine
 * 
 * Renders animated two-eye face on LVGL display. Supports keyframe-based
 * expressions synchronized with robot head actions, plus idle living-face
 * behavior with random blinks and micro-saccades.
 */
class FaceDisplay {
public:
    /**
     * @brief Get singleton instance
     */
    static FaceDisplay& instance();

    ~FaceDisplay();

    FaceDisplay(const FaceDisplay&) = delete;
    FaceDisplay& operator=(const FaceDisplay&) = delete;

    /**
     * @brief Initialize face display on the active LVGL screen
     * @param parent LVGL parent object (screen or container). If nullptr, uses active screen.
     * @return CUBE32_OK on success
     */
    cube32_result_t begin(lv_obj_t* parent = nullptr);

    /**
     * @brief Initialize with custom config  
     * @param config Face geometry config
     * @param parent LVGL parent object   
     * @return CUBE32_OK on success
     */
    cube32_result_t begin(const FaceConfig& config, lv_obj_t* parent = nullptr);

    /**
     * @brief Deinitialize and remove face objects from display
     */
    cube32_result_t end();

    /**
     * @brief Check if initialized
     */
    bool isInitialized() const { return m_initialized; }

    // ====================================================================
    // Expression Control
    // ====================================================================

    /**
     * @brief Play a built-in face expression by ID
     * @param id Expression ID
     * @return CUBE32_OK on success
     */
    cube32_result_t playExpression(FaceExpressionId id);

    /**
     * @brief Play a custom face expression
     * @param expr Expression definition
     * @return CUBE32_OK on success
     */
    cube32_result_t playExpression(const FaceExpression& expr);

    /**
     * @brief Stop current expression and return to idle
     */
    cube32_result_t stopExpression();

    /**
     * @brief Check if an expression (non-idle) is currently playing
     */
    bool isPlaying() const { return m_playing && m_current_expr_id != FaceExpressionId::IDLE; }

    /**
     * @brief Get current expression ID
     */
    FaceExpressionId getCurrentExpressionId() const { return m_current_expr_id; }

    /**
     * @brief Get built-in expression definition
     */
    static const FaceExpression* getBuiltinExpression(FaceExpressionId id);

    /**
     * @brief Get number of built-in expressions  
     */
    static uint8_t getBuiltinExpressionCount();

private:
    FaceDisplay() = default;

    // ====================================================================
    // Internal LVGL Object Management
    // ====================================================================

    /**
     * @brief Create all LVGL objects for the face
     */
    void createFaceObjects(lv_obj_t* parent);

    /**
     * @brief Destroy all LVGL objects
     */
    void destroyFaceObjects();

    // ====================================================================
    // Animation Engine  
    // ====================================================================

    /**
     * @brief LVGL timer callback — drives animation at ~30fps
     */
    static void animTimerCb(lv_timer_t* timer);

    /**
     * @brief Process one animation tick
     */
    void animTick();

    /**
     * @brief Apply a keyframe state to the LVGL objects (interpolated)
     */
    void applyState(const FaceKeyframe& state);

    /**
     * @brief Lerp between two keyframes
     */
    FaceKeyframe lerpKeyframes(const FaceKeyframe& a, const FaceKeyframe& b, float t) const;

    // ====================================================================
    // Idle Behavior
    // ====================================================================

    /**
     * @brief Process idle behavior (blinks + micro-saccades)
     */
    void processIdle();

    /**
     * @brief Trigger a blink animation
     */
    void triggerBlink();

    // ====================================================================
    // State
    // ====================================================================

    bool m_initialized = false;
    bool m_playing = false;
    FaceConfig m_config = {};

    // Current expression
    FaceExpressionId m_current_expr_id = FaceExpressionId::IDLE;
    const FaceKeyframe* m_keyframes = nullptr;
    uint8_t m_num_keyframes = 0;
    bool m_loop = false;

    // Animation state
    uint8_t m_current_kf_index = 0;    ///< Current keyframe being interpolated to
    uint32_t m_kf_start_time = 0;       ///< When current keyframe interpolation started
    FaceKeyframe m_current_state = {};   ///< Current interpolated state
    FaceKeyframe m_prev_state = {};      ///< State at start of current interpolation

    // Idle state
    uint32_t m_last_blink_time = 0;
    uint32_t m_next_blink_interval = 0;
    bool m_blinking = false;
    uint32_t m_blink_start_time = 0;
    float m_idle_pupil_x = 0.0f;
    float m_idle_pupil_y = 0.0f;
    uint32_t m_next_saccade_time = 0;

    // LVGL timer
    lv_timer_t* m_anim_timer = nullptr;

    // ====================================================================
    // LVGL Objects — per eye
    // ====================================================================

    struct EyeObjects {
        lv_obj_t* eyeball;     ///< White circle background
        lv_obj_t* pupil;       ///< Dark circle
        lv_obj_t* highlight;   ///< White sparkle dot
        lv_obj_t* upper_lid;   ///< Upper eyelid mask
        lv_obj_t* lower_lid;   ///< Lower eyelid mask
    };

    lv_obj_t* m_face_container = nullptr;  ///< Face container object
    EyeObjects m_left_eye = {};
    EyeObjects m_right_eye = {};
};

} // namespace cube32

#endif // __cplusplus
#endif // CUBE32_DRIVERS_ROBOT_FACE_EXPRESSION_H
