/**
 * @file face_expression.cpp
 * @brief CUBE32 Robot Face Expression Engine implementation
 * 
 * Renders a two-eye animated face using LVGL primitives (circles + rectangles).
 * Each expression is a sequence of keyframes interpolated at ~30fps.
 * Idle behavior includes periodic blinks and random micro-saccades.
 */

#include "drivers/robot/face_expression.h"
#include "drivers/lvgl/lvgl_driver.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <esp_random.h>
#include <cstring>
#include <cmath>
#include <algorithm>

#if defined(CONFIG_CUBE32_FACE_EXPRESSION_ENABLED) && defined(CONFIG_CUBE32_LVGL_ENABLED)

static const char* TAG = "cube32_face";

namespace cube32 {

// ============================================================================
// Helper: get monotonic time in ms
// ============================================================================

static inline uint32_t now_ms() {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static inline float randf() {
    return (float)(esp_random() % 10000) / 10000.0f;
}

// ============================================================================
// Built-in Expression Keyframe Definitions
// ============================================================================

// Keyframe field reference:
//   pupil_x, pupil_y, pupil_scale, upper_lid_y, lower_lid_y, lid_angle,
//   eye_y_offset, asymmetric, r_upper_lid_y, r_lower_lid_y, duration_ms

// Neutral base state (used as starting point)
static const FaceKeyframe s_neutral = {
    0.0f, 0.0f, 1.0f,  0.0f, 0.0f, 0.0f,  0.0f,  false, 0.0f, 0.0f,  300
};

// --- IDLE: handled procedurally, no keyframes needed ---

// --- NOD (Agreement) ---
// Happy squint, pupils bounce up-down in sync with head nod
static const FaceKeyframe s_nod_kf[] = {
    { 0.0f, 0.25f, 1.0f,   8.0f, 6.0f, 0.0f,  4.0f,  false, 0,0, 100 },  // down + squint
    { 0.0f,-0.15f, 1.0f,   4.0f, 3.0f, 0.0f, -3.0f,  false, 0,0, 100 },  // up + less squint
    { 0.0f, 0.25f, 1.0f,   8.0f, 6.0f, 0.0f,  4.0f,  false, 0,0, 100 },  // down
    { 0.0f,-0.15f, 1.0f,   4.0f, 3.0f, 0.0f, -3.0f,  false, 0,0, 100 },  // up
    { 0.0f, 0.25f, 1.0f,   8.0f, 6.0f, 0.0f,  4.0f,  false, 0,0, 100 },  // down
    { 0.0f, 0.0f,  1.0f,   0.0f, 0.0f, 0.0f,  0.0f,  false, 0,0, 250 },  // return neutral
};

// --- SHAKE (Disagreement) ---
// Stern look, pupils sway left-right
static const FaceKeyframe s_shake_kf[] = {
    { 0.5f, 0.0f, 1.0f,   6.0f, 0.0f, 0.3f,  0.0f,  false, 0,0, 100 },  // left + stern
    {-0.5f, 0.0f, 1.0f,   6.0f, 0.0f, 0.3f,  0.0f,  false, 0,0, 100 },  // right
    { 0.5f, 0.0f, 1.0f,   6.0f, 0.0f, 0.3f,  0.0f,  false, 0,0, 100 },  // left
    {-0.5f, 0.0f, 1.0f,   6.0f, 0.0f, 0.3f,  0.0f,  false, 0,0, 100 },  // right
    { 0.5f, 0.0f, 1.0f,   6.0f, 0.0f, 0.3f,  0.0f,  false, 0,0, 100 },  // left
    { 0.0f, 0.0f, 1.0f,   0.0f, 0.0f, 0.0f,  0.0f,  false, 0,0, 250 },  // return
};

// --- CURIOUS (Puzzle) ---
// One eye wider, slight head tilt effect, pupil shifted
static const FaceKeyframe s_curious_kf[] = {
    { 0.3f,-0.1f, 1.15f,  0.0f, 0.0f,-0.4f,  0.0f,  true, 10.0f, 0.0f, 1500 }, // curious hold
    { 0.0f, 0.0f, 1.0f,   0.0f, 0.0f, 0.0f,  0.0f,  false, 0,0,  350 },        // return
};

// --- ATTENTION (Alert) ---
// Wide open, pupils snap to center + slight dilate
static const FaceKeyframe s_attention_kf[] = {
    { 0.0f, 0.0f, 1.2f,   0.0f, 0.0f, 0.0f,  0.0f,  false, 0,0, 150 },  // wide + dilate
    { 0.0f, 0.0f, 1.0f,   0.0f, 0.0f, 0.0f,  0.0f,  false, 0,0, 200 },  // settle
};

// --- LOOK LEFT ---
// Pupils shift left, slight lid narrowing
static const FaceKeyframe s_look_left_kf[] = {
    {-0.7f, 0.0f, 1.0f,   3.0f, 2.0f, 0.0f,  0.0f,  false, 0,0, 1000 }, // hold left
    { 0.0f, 0.0f, 1.0f,   0.0f, 0.0f, 0.0f,  0.0f,  false, 0,0,  250 }, // return
};

// --- LOOK RIGHT ---
// Pupils shift right
static const FaceKeyframe s_look_right_kf[] = {
    { 0.7f, 0.0f, 1.0f,   3.0f, 2.0f, 0.0f,  0.0f,  false, 0,0, 1000 }, // hold right
    { 0.0f, 0.0f, 1.0f,   0.0f, 0.0f, 0.0f,  0.0f,  false, 0,0,  250 }, // return
};

// --- LOOK UP ---
// Pupils shift up, upper lids raised
static const FaceKeyframe s_look_up_kf[] = {
    { 0.0f,-0.6f, 1.0f,   0.0f, 5.0f, 0.0f, -3.0f,  false, 0,0, 1000 }, // hold up
    { 0.0f, 0.0f, 1.0f,   0.0f, 0.0f, 0.0f,  0.0f,  false, 0,0,  250 }, // return
};

// --- LOOK DOWN ---
// Pupils shift down, upper lids droop
static const FaceKeyframe s_look_down_kf[] = {
    { 0.0f, 0.5f, 0.9f,  12.0f, 0.0f, 0.0f,  3.0f,  false, 0,0, 1000 }, // hold down
    { 0.0f, 0.0f, 1.0f,   0.0f, 0.0f, 0.0f,  0.0f,  false, 0,0,  250 }, // return
};

// --- SCAN (Alert) ---
// Wide eyes, pupils sweep left to right slowly
static const FaceKeyframe s_scan_kf[] = {
    {-0.8f, 0.0f, 1.1f,   0.0f, 0.0f, 0.0f,  0.0f,  false, 0,0, 800 },  // look far left
    { 0.8f, 0.0f, 1.1f,   0.0f, 0.0f, 0.0f,  0.0f,  false, 0,0, 800 },  // sweep to far right
    { 0.0f, 0.0f, 1.0f,   0.0f, 0.0f, 0.0f,  0.0f,  false, 0,0, 250 },  // return center
};

// --- BOW (Respectful) ---
// Eyes slowly half-close, pause, reopen
static const FaceKeyframe s_bow_kf[] = {
    { 0.0f, 0.3f, 0.9f,  20.0f, 10.0f, 0.0f, 5.0f,  false, 0,0, 800 },  // close + down
    { 0.0f, 0.3f, 0.9f,  20.0f, 10.0f, 0.0f, 5.0f,  false, 0,0, 2000 }, // hold
    { 0.0f, 0.0f, 1.0f,   0.0f,  0.0f, 0.0f, 0.0f,  false, 0,0,  400 }, // reopen
};

// --- SEARCH (Scanning pattern) ---
// Pupils move in a searching pattern
static const FaceKeyframe s_search_kf[] = {
    {-0.5f,-0.3f, 1.05f,  0.0f, 0.0f, 0.0f,  0.0f,  false, 0,0, 500 },  // upper-left
    { 0.5f,-0.3f, 1.05f,  0.0f, 0.0f, 0.0f,  0.0f,  false, 0,0, 500 },  // upper-right
    { 0.5f, 0.3f, 1.05f,  0.0f, 0.0f, 0.0f,  0.0f,  false, 0,0, 500 },  // lower-right
    {-0.5f, 0.3f, 1.05f,  0.0f, 0.0f, 0.0f,  0.0f,  false, 0,0, 500 },  // lower-left
    {-0.5f,-0.3f, 1.05f,  0.0f, 0.0f, 0.0f,  0.0f,  false, 0,0, 500 },  // upper-left again
    { 0.0f, 0.0f, 1.0f,   0.0f, 0.0f, 0.0f,  0.0f,  false, 0,0, 300 },  // center
};

// --- EXCITED (Happy) ---
// Squinted happy eyes (crescents), pupils jitter
static const FaceKeyframe s_excited_kf[] = {
    { 0.15f,-0.1f, 1.1f, 14.0f, 12.0f, 0.0f, -2.0f, false, 0,0, 150 },  // happy bounce 1
    {-0.15f, 0.1f, 1.1f, 14.0f, 12.0f, 0.0f,  2.0f, false, 0,0, 150 },  // happy bounce 2
    { 0.15f,-0.1f, 1.1f, 14.0f, 12.0f, 0.0f, -2.0f, false, 0,0, 150 },  // bounce 3
    {-0.15f, 0.1f, 1.1f, 14.0f, 12.0f, 0.0f,  2.0f, false, 0,0, 150 },  // bounce 4
    { 0.15f,-0.1f, 1.1f, 14.0f, 12.0f, 0.0f, -2.0f, false, 0,0, 150 },  // bounce 5
    {-0.15f, 0.1f, 1.1f, 14.0f, 12.0f, 0.0f,  2.0f, false, 0,0, 150 },  // bounce 6
    { 0.0f, 0.0f,  1.0f,  0.0f,  0.0f, 0.0f,  0.0f, false, 0,0, 300 },  // return
};

// --- SAD ---
// Droopy lids, pupils shift down and slightly inward, small pupils
static const FaceKeyframe s_sad_kf[] = {
    { 0.0f, 0.2f, 0.8f,  10.0f, 0.0f,-0.5f,  3.0f,  false, 0,0, 600 },  // droop
    {-0.1f, 0.25f,0.75f, 14.0f, 0.0f,-0.6f,  4.0f,  false, 0,0, 3000 }, // hold sad
    { 0.0f, 0.0f, 1.0f,   0.0f, 0.0f, 0.0f,  0.0f,  false, 0,0,  400 }, // return
};

// --- DOUBLE TAKE ---
// Quick glance right, pause, snap back with wide eyes
static const FaceKeyframe s_double_take_kf[] = {
    { 0.6f, 0.0f, 0.9f,   4.0f, 2.0f, 0.0f,  0.0f,  false, 0,0, 250 },  // glance right
    { 0.6f, 0.0f, 0.9f,   4.0f, 2.0f, 0.0f,  0.0f,  false, 0,0, 200 },  // hold
    { 0.0f,-0.05f,1.35f,  0.0f, 0.0f, 0.0f,  0.0f,  false, 0,0, 400 },  // snap back WIDE
    { 0.0f, 0.0f, 1.0f,   0.0f, 0.0f, 0.0f,  0.0f,  false, 0,0, 250 },  // settle
};

// --- DIZZY ---
// Pupils wobble in decreasing circles, uneven lids
static const FaceKeyframe s_dizzy_kf[] = {
    { 0.6f, 0.3f, 1.1f,   4.0f, 2.0f, 0.2f,  2.0f,  true, 2.0f, 4.0f, 150 },
    {-0.6f,-0.3f, 1.1f,   2.0f, 4.0f,-0.2f, -2.0f,  true, 4.0f, 2.0f, 150 },
    { 0.4f, 0.2f, 1.05f,  3.0f, 1.0f, 0.15f, 1.0f,  true, 1.0f, 3.0f, 150 },
    {-0.4f,-0.2f, 1.05f,  1.0f, 3.0f,-0.15f,-1.0f,  true, 3.0f, 1.0f, 150 },
    { 0.25f,0.12f,1.0f,   2.0f, 0.5f, 0.1f,  0.5f,  true, 0.5f, 2.0f, 180 },
    {-0.25f,-0.12f,1.0f,  0.5f, 2.0f,-0.1f, -0.5f,  true, 2.0f, 0.5f, 180 },
    { 0.1f, 0.05f,1.0f,   1.0f, 0.0f, 0.0f,  0.0f,  false, 0,0, 200 },
    {-0.1f,-0.05f,1.0f,   0.0f, 1.0f, 0.0f,  0.0f,  false, 0,0, 200 },
    { 0.0f, 0.0f, 1.0f,   0.0f, 0.0f, 0.0f,  0.0f,  false, 0,0, 350 },  // settle
};

// ============================================================================
// Built-in Expressions Table
// ============================================================================

#define EXPR_DEF(name_str, kf_arr, do_loop) \
    { name_str, kf_arr, sizeof(kf_arr) / sizeof(kf_arr[0]), do_loop }

static const FaceExpression s_builtin_expressions[] = {
    { "idle",        nullptr,           0, true  },  // IDLE — procedural
    EXPR_DEF("nod",         s_nod_kf,         false),
    EXPR_DEF("shake",       s_shake_kf,       false),
    EXPR_DEF("curious",     s_curious_kf,     false),
    EXPR_DEF("attention",   s_attention_kf,   false),
    EXPR_DEF("look_left",   s_look_left_kf,   false),
    EXPR_DEF("look_right",  s_look_right_kf,  false),
    EXPR_DEF("look_up",     s_look_up_kf,     false),
    EXPR_DEF("look_down",   s_look_down_kf,   false),
    EXPR_DEF("scan",        s_scan_kf,        false),
    EXPR_DEF("bow",         s_bow_kf,         false),
    EXPR_DEF("search",      s_search_kf,      false),
    EXPR_DEF("excited",     s_excited_kf,     false),
    EXPR_DEF("sad",         s_sad_kf,         false),
    EXPR_DEF("double_take", s_double_take_kf, false),
    EXPR_DEF("dizzy",       s_dizzy_kf,       false),
};

static_assert(sizeof(s_builtin_expressions) / sizeof(s_builtin_expressions[0]) == (uint8_t)FaceExpressionId::COUNT,
              "Built-in expressions table must match FaceExpressionId::COUNT");

// ============================================================================
// Singleton
// ============================================================================

FaceDisplay& FaceDisplay::instance() {
    static FaceDisplay s_instance;
    return s_instance;
}

FaceDisplay::~FaceDisplay() {
    end();
}

// ============================================================================
// Initialization
// ============================================================================

cube32_result_t FaceDisplay::begin(lv_obj_t* parent) {
    FaceConfig config = CUBE32_FACE_CONFIG_DEFAULT();

    // Auto-detect screen size from LVGL display
    LvglDisplay& lvgl = LvglDisplay::instance();
    if (lvgl.isInitialized()) {
        config.screen_w = lvgl.getWidth();
        config.screen_h = lvgl.getHeight();
    }

    return begin(config, parent);
}

cube32_result_t FaceDisplay::begin(const FaceConfig& config, lv_obj_t* parent) {
    if (m_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return CUBE32_OK;
    }

    LvglDisplay& lvgl = LvglDisplay::instance();
    if (!lvgl.isInitialized()) {
        ESP_LOGE(TAG, "LVGL display not initialized");
        return CUBE32_NOT_INITIALIZED;
    }

    m_config = config;

    // Create face LVGL objects (caller must hold LVGL lock)
    createFaceObjects(parent);

    // Initialize idle state
    m_current_state = s_neutral;
    m_prev_state = s_neutral;
    m_last_blink_time = now_ms();
    m_next_blink_interval = 2500 + (esp_random() % 3000);  // 2.5-5.5s
    m_next_saccade_time = now_ms() + 1000 + (esp_random() % 2000);
    m_idle_pupil_x = 0.0f;
    m_idle_pupil_y = 0.0f;
    m_blinking = false;
    m_current_expr_id = FaceExpressionId::IDLE;
    m_playing = false;

    // Apply initial neutral state
    applyState(m_current_state);

    // Create animation timer (~30fps)
    m_anim_timer = lv_timer_create(animTimerCb, 33, this);

    m_initialized = true;
    ESP_LOGI(TAG, "Face display initialized (%dx%d, eye_r=%d, spacing=%d)",
             m_config.screen_w, m_config.screen_h,
             m_config.eye_radius, m_config.eye_spacing);

    return CUBE32_OK;
}

cube32_result_t FaceDisplay::end() {
    if (!m_initialized) return CUBE32_OK;

    if (m_anim_timer) {
        lv_timer_delete(m_anim_timer);
        m_anim_timer = nullptr;
    }

    destroyFaceObjects();

    m_initialized = false;
    ESP_LOGI(TAG, "Face display deinitialized");
    return CUBE32_OK;
}

// ============================================================================
// LVGL Object Creation
// ============================================================================

static lv_obj_t* create_circle(lv_obj_t* parent, int16_t x, int16_t y, 
                                uint16_t radius, uint32_t color) {
    lv_obj_t* obj = lv_obj_create(parent);
    int16_t diameter = radius * 2;
    lv_obj_set_size(obj, diameter, diameter);
    lv_obj_set_style_radius(obj, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_bg_color(obj, lv_color_hex(color), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(obj, 0, LV_PART_MAIN);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_pos(obj, x - radius, y - radius);
    return obj;
}

static lv_obj_t* create_rect(lv_obj_t* parent, int16_t x, int16_t y,
                              uint16_t w, uint16_t h, uint32_t color) {
    lv_obj_t* obj = lv_obj_create(parent);
    lv_obj_set_size(obj, w, h);
    lv_obj_set_style_radius(obj, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(obj, lv_color_hex(color), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(obj, 0, LV_PART_MAIN);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_pos(obj, x, y);
    return obj;
}

void FaceDisplay::createFaceObjects(lv_obj_t* parent) {
    if (!parent) {
        parent = lv_screen_active();
    }

    // Face container — full screen, dark background
    m_face_container = lv_obj_create(parent);
    lv_obj_set_size(m_face_container, m_config.screen_w, m_config.screen_h);
    lv_obj_set_style_bg_color(m_face_container, lv_color_hex(m_config.bg_color), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(m_face_container, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(m_face_container, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(m_face_container, 0, LV_PART_MAIN);
    lv_obj_clear_flag(m_face_container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align(m_face_container, LV_ALIGN_CENTER, 0, 0);

    // Calculate eye centers
    int16_t cx = m_config.screen_w / 2;
    int16_t cy = m_config.screen_h / 2 + m_config.eye_y;
    int16_t left_cx = cx - m_config.eye_spacing / 2;
    int16_t right_cx = cx + m_config.eye_spacing / 2;
    uint16_t er = m_config.eye_radius;
    uint16_t pr = m_config.pupil_radius;
    uint16_t hr = m_config.highlight_r;

    // Eyelid dimensions — slightly wider/taller than eye to fully cover
    uint16_t lid_w = er * 2 + 10;
    uint16_t lid_h = er + 6;  // half eye + margin

    // Lambda to create one eye's objects
    auto createEye = [&](EyeObjects& eye, int16_t ecx, int16_t ecy) {
        // Eyeball (white circle)
        eye.eyeball = create_circle(m_face_container, ecx, ecy, er, m_config.eye_color);

        // Pupil (dark circle, centered initially)
        eye.pupil = create_circle(m_face_container, ecx, ecy, pr, m_config.pupil_color);

        // Highlight sparkle (small white dot, upper-right of pupil)
        eye.highlight = create_circle(m_face_container, ecx - pr/3, ecy - pr/3,
                                       hr, m_config.highlight_color);

        // Upper eyelid (rectangle, positioned above eye, moves down to close)
        // Starts just above the eye top edge, hidden
        eye.upper_lid = create_rect(m_face_container,
                                      ecx - lid_w / 2, ecy - er - lid_h,
                                      lid_w, lid_h,
                                      m_config.lid_color);

        // Lower eyelid (rectangle, positioned below eye, moves up to close)
        eye.lower_lid = create_rect(m_face_container,
                                      ecx - lid_w / 2, ecy + er,
                                      lid_w, lid_h,
                                      m_config.lid_color);
    };

    createEye(m_left_eye, left_cx, cy);
    createEye(m_right_eye, right_cx, cy);
}

void FaceDisplay::destroyFaceObjects() {
    if (m_face_container) {
        lv_obj_delete(m_face_container);
        m_face_container = nullptr;
    }
    m_left_eye = {};
    m_right_eye = {};
}

// ============================================================================
// Expression Control
// ============================================================================

cube32_result_t FaceDisplay::playExpression(FaceExpressionId id) {
    if ((uint8_t)id >= (uint8_t)FaceExpressionId::COUNT) {
        return CUBE32_INVALID_ARG;
    }
    return playExpression(s_builtin_expressions[(uint8_t)id]);
}

cube32_result_t FaceDisplay::playExpression(const FaceExpression& expr) {
    if (!m_initialized) return CUBE32_NOT_INITIALIZED;

    // Save previous state as interpolation start
    m_prev_state = m_current_state;

    if (!expr.keyframes || expr.num_keyframes == 0) {
        // Switch to idle
        m_current_expr_id = FaceExpressionId::IDLE;
        m_keyframes = nullptr;
        m_num_keyframes = 0;
        m_loop = false;
        m_playing = false;
        m_last_blink_time = now_ms();
        m_next_blink_interval = 2500 + (esp_random() % 3000);
        return CUBE32_OK;
    }

    m_keyframes = expr.keyframes;
    m_num_keyframes = expr.num_keyframes;
    m_loop = expr.loop;
    m_current_kf_index = 0;
    m_kf_start_time = now_ms();
    m_playing = true;

    // Determine expression ID from name match
    for (uint8_t i = 0; i < (uint8_t)FaceExpressionId::COUNT; i++) {
        if (expr.name && s_builtin_expressions[i].name &&
            strcmp(expr.name, s_builtin_expressions[i].name) == 0) {
            m_current_expr_id = (FaceExpressionId)i;
            break;
        }
    }

    ESP_LOGD(TAG, "Playing expression: %s (%d keyframes%s)", 
             expr.name ? expr.name : "custom", m_num_keyframes,
             m_loop ? ", loop" : "");

    return CUBE32_OK;
}

cube32_result_t FaceDisplay::stopExpression() {
    if (!m_initialized) return CUBE32_NOT_INITIALIZED;

    m_playing = false;
    m_current_expr_id = FaceExpressionId::IDLE;
    m_keyframes = nullptr;
    m_num_keyframes = 0;

    // Reset to neutral smoothly by making current state the prev
    m_prev_state = m_current_state;
    m_current_state = s_neutral;

    m_last_blink_time = now_ms();
    m_next_blink_interval = 2000 + (esp_random() % 2500);

    return CUBE32_OK;
}

const FaceExpression* FaceDisplay::getBuiltinExpression(FaceExpressionId id) {
    if ((uint8_t)id >= (uint8_t)FaceExpressionId::COUNT) return nullptr;
    return &s_builtin_expressions[(uint8_t)id];
}

uint8_t FaceDisplay::getBuiltinExpressionCount() {
    return (uint8_t)FaceExpressionId::COUNT;
}

// ============================================================================
// Animation Engine
// ============================================================================

void FaceDisplay::animTimerCb(lv_timer_t* timer) {
    FaceDisplay* self = static_cast<FaceDisplay*>(lv_timer_get_user_data(timer));
    if (self && self->m_initialized) {
        self->animTick();
    }
}

void FaceDisplay::animTick() {
    if (m_current_expr_id == FaceExpressionId::IDLE || !m_playing) {
        processIdle();
        applyState(m_current_state);
        return;
    }

    // Playing a keyframe expression
    uint32_t now = now_ms();
    uint32_t elapsed = now - m_kf_start_time;

    const FaceKeyframe& target = m_keyframes[m_current_kf_index];
    uint32_t duration = target.duration_ms;
    if (duration == 0) duration = 1;

    if (elapsed >= duration) {
        // Keyframe complete — apply it exactly
        m_current_state = target;
        m_prev_state = target;

        // Advance to next keyframe
        m_current_kf_index++;
        if (m_current_kf_index >= m_num_keyframes) {
            if (m_loop) {
                m_current_kf_index = 0;
            } else {
                // Expression finished — return to idle
                m_playing = false;
                m_current_expr_id = FaceExpressionId::IDLE;
                m_keyframes = nullptr;
                m_num_keyframes = 0;
                // Smoothly return to neutral
                m_prev_state = m_current_state;
                m_last_blink_time = now_ms();
                m_next_blink_interval = 2000 + (esp_random() % 2500);
                applyState(m_current_state);
                return;
            }
        }
        m_kf_start_time = now;
    } else {
        // Interpolate between previous state and target keyframe
        float t = (float)elapsed / (float)duration;
        // Ease in-out for smoother motion
        t = t * t * (3.0f - 2.0f * t);  // smoothstep
        m_current_state = lerpKeyframes(m_prev_state, target, t);
    }

    applyState(m_current_state);
}

FaceKeyframe FaceDisplay::lerpKeyframes(const FaceKeyframe& a, const FaceKeyframe& b, float t) const {
    FaceKeyframe result;
    result.pupil_x      = a.pupil_x      + (b.pupil_x      - a.pupil_x)      * t;
    result.pupil_y      = a.pupil_y      + (b.pupil_y      - a.pupil_y)      * t;
    result.pupil_scale  = a.pupil_scale  + (b.pupil_scale  - a.pupil_scale)  * t;
    result.upper_lid_y  = a.upper_lid_y  + (b.upper_lid_y  - a.upper_lid_y)  * t;
    result.lower_lid_y  = a.lower_lid_y  + (b.lower_lid_y  - a.lower_lid_y)  * t;
    result.lid_angle    = a.lid_angle    + (b.lid_angle    - a.lid_angle)    * t;
    result.eye_y_offset = a.eye_y_offset + (b.eye_y_offset - a.eye_y_offset) * t;
    result.asymmetric   = b.asymmetric;
    result.r_upper_lid_y = a.r_upper_lid_y + (b.r_upper_lid_y - a.r_upper_lid_y) * t;
    result.r_lower_lid_y = a.r_lower_lid_y + (b.r_lower_lid_y - a.r_lower_lid_y) * t;
    result.duration_ms  = b.duration_ms;
    return result;
}

// ============================================================================
// Apply State to LVGL Objects
// ============================================================================

void FaceDisplay::applyState(const FaceKeyframe& state) {
    if (!m_face_container) return;

    int16_t cx = m_config.screen_w / 2;
    int16_t cy = m_config.screen_h / 2 + m_config.eye_y + (int16_t)state.eye_y_offset;
    int16_t left_cx  = cx - m_config.eye_spacing / 2;
    int16_t right_cx = cx + m_config.eye_spacing / 2;
    uint16_t er = m_config.eye_radius;
    uint16_t pr = (uint16_t)(m_config.pupil_radius * state.pupil_scale);
    uint16_t hr = m_config.highlight_r;

    // Maximum pupil travel (stay inside eyeball)
    float max_travel = (float)(er - pr - 2);

    // --- Helper: apply state to one eye ---
    auto applyEye = [&](EyeObjects& eye, int16_t ecx, float upper_lid, float lower_lid, float lid_ang) {
        // Eyeball position (moves with eye_y_offset)
        lv_obj_set_pos(eye.eyeball, ecx - er, cy - er);

        // Pupil position (offset from eye center)
        int16_t px = ecx + (int16_t)(state.pupil_x * max_travel);
        int16_t py = cy + (int16_t)(state.pupil_y * max_travel);
        lv_obj_set_pos(eye.pupil, px - pr, py - pr);
        lv_obj_set_size(eye.pupil, pr * 2, pr * 2);

        // Highlight sparkle (upper-left of pupil)
        int16_t hx = px - (int16_t)(pr * 0.35f);
        int16_t hy = py - (int16_t)(pr * 0.35f);
        lv_obj_set_pos(eye.highlight, hx - hr, hy - hr);

        // Eyelid dimensions
        uint16_t lid_w = er * 2 + 10;
        uint16_t lid_h = er + 6;

        // Upper eyelid — positioned so its bottom edge overlaps the eye from the top
        // upper_lid value = how many pixels the lid has come down from fully open
        // When upper_lid = 0, the lid is hidden above the eye
        // When upper_lid = er*2, the eye is fully covered
        float upper_drop = upper_lid;  // pixels from open position
        int16_t upper_y = cy - er - lid_h + (int16_t)upper_drop;

        // Lid angle effect: tilt the eyelid by shifting left/right sides differently
        // For the LEFT eye: positive lid_angle = inner side lower (angry)
        // We approximate this by just shifting the upper lid position slightly
        // (True per-corner rotation would require canvas/image — too heavy for ESP32)
        int16_t lid_x = ecx - lid_w / 2;

        lv_obj_set_pos(eye.upper_lid, lid_x, upper_y);
        lv_obj_set_size(eye.upper_lid, lid_w, lid_h);

        // Lower eyelid — rises from below
        float lower_rise = lower_lid;
        int16_t lower_y = cy + er - (int16_t)lower_rise;
        lv_obj_set_pos(eye.lower_lid, lid_x, lower_y);
        lv_obj_set_size(eye.lower_lid, lid_w, lid_h);
    };

    // Left eye uses base lid values
    applyEye(m_left_eye, left_cx, state.upper_lid_y, state.lower_lid_y, -state.lid_angle);

    // Right eye: use asymmetric values if enabled, otherwise mirror
    float r_upper = state.asymmetric ? state.r_upper_lid_y : state.upper_lid_y;
    float r_lower = state.asymmetric ? state.r_lower_lid_y : state.lower_lid_y;
    applyEye(m_right_eye, right_cx, r_upper, r_lower, state.lid_angle);
}

// ============================================================================
// Idle Behavior
// ============================================================================

void FaceDisplay::processIdle() {
    uint32_t now = now_ms();

    // --- Blink ---
    if (m_blinking) {
        // Blink animation: ~200ms total (close 80ms, open 120ms)
        uint32_t blink_elapsed = now - m_blink_start_time;
        const uint32_t BLINK_CLOSE_MS = 80;
        const uint32_t BLINK_OPEN_MS = 120;
        const uint32_t BLINK_TOTAL_MS = BLINK_CLOSE_MS + BLINK_OPEN_MS;
        float blink_lid = 0.0f;

        if (blink_elapsed < BLINK_CLOSE_MS) {
            // Closing
            float t = (float)blink_elapsed / BLINK_CLOSE_MS;
            blink_lid = t * (float)(m_config.eye_radius * 2);
        } else if (blink_elapsed < BLINK_TOTAL_MS) {
            // Opening
            float t = (float)(blink_elapsed - BLINK_CLOSE_MS) / BLINK_OPEN_MS;
            blink_lid = (1.0f - t) * (float)(m_config.eye_radius * 2);
        } else {
            // Blink done
            m_blinking = false;
            blink_lid = 0.0f;
        }

        m_current_state.upper_lid_y = blink_lid;
        m_current_state.lower_lid_y = blink_lid * 0.3f;  // lower lid barely moves
    } else {
        // Check if it's time to blink
        if (now - m_last_blink_time >= m_next_blink_interval) {
            triggerBlink();
        }

        // Smooth return lids to open
        if (m_current_state.upper_lid_y > 0.1f) {
            m_current_state.upper_lid_y *= 0.85f;
        } else {
            m_current_state.upper_lid_y = 0.0f;
        }
        if (m_current_state.lower_lid_y > 0.1f) {
            m_current_state.lower_lid_y *= 0.85f;
        } else {
            m_current_state.lower_lid_y = 0.0f;
        }
    }

    // --- Micro-saccades (tiny random pupil movements) ---
    if (now >= m_next_saccade_time && !m_blinking) {
        // Generate a small random pupil offset
        m_idle_pupil_x = (randf() - 0.5f) * 0.15f;  // ±0.075 normalized
        m_idle_pupil_y = (randf() - 0.5f) * 0.1f;    // ±0.05 normalized
        m_next_saccade_time = now + 800 + (esp_random() % 2500);  // 0.8-3.3s
    }

    // Smoothly interpolate pupils toward target saccade position
    m_current_state.pupil_x += (m_idle_pupil_x - m_current_state.pupil_x) * 0.1f;
    m_current_state.pupil_y += (m_idle_pupil_y - m_current_state.pupil_y) * 0.1f;

    // Keep other values neutral during idle
    m_current_state.pupil_scale = 1.0f;
    m_current_state.lid_angle = 0.0f;
    m_current_state.eye_y_offset = 0.0f;
    m_current_state.asymmetric = false;
}

void FaceDisplay::triggerBlink() {
    m_blinking = true;
    m_blink_start_time = now_ms();
    m_last_blink_time = m_blink_start_time;
    m_next_blink_interval = 2500 + (esp_random() % 4000);  // 2.5-6.5s next blink

    // Occasionally double-blink (~20% chance)
    if ((esp_random() % 100) < 20) {
        m_next_blink_interval = 300;  // Quick second blink
    }
}

} // namespace cube32

#endif // CONFIG_CUBE32_FACE_EXPRESSION_ENABLED && CONFIG_CUBE32_LVGL_ENABLED
