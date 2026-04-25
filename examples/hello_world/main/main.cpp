/**
 * @file main.cpp
 * @brief CUBE32 Hello World — Pre-loaded product firmware
 *
 * Splash screen: interactive 3D wireframe cube driven by IMU gyro
 * (real-time device rotation) and touch swipe (angular impulse).
 * Physics-based inertia — cube coasts naturally to rest when input stops.
 *
 * Status screen: hardware manifest driver grid + battery status.
 *
 * Navigation: tap to toggle between splash and status screens.
 */

#include <cstdio>
#include <cmath>
#include <cstring>
#include <cinttypes>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_timer.h>

#include "cube32.h"
#include "utils/hw_manifest.h"

static const char *TAG = "hello_world";

// ============================================================================
// Color palette
// ============================================================================
#define BG_COLOR        0x080C14   // Deep near-black (Explorer Tech)
#define TITLE_COLOR     0x00D4FF   // Cyan — primary accent
#define DIM_COLOR       0x445566   // Hint / secondary text
#define EDGE_FRONT      0x00FFFF   // Front-face edges  (full-brightness cyan — closest face)
#define EDGE_BACK       0x001A3A   // Back-face edges   (near-invisible dark indigo — recedes)
#define EDGE_SIDE       0x0066FF   // Connecting edges  (deep electric blue — distinct from front)
#define DOT_OK          0x00FF88   // Status: OK        (green)
#define DOT_FAIL        0xFF4444   // Status: FAIL      (red)
#define DOT_SKIP        0xFFAA00   // Status: SKIPPED / INITIALIZING (amber)
#define DOT_NA          0x2A2A3A   // Status: not present / not built (dark)

// ============================================================================
// 3D cube geometry — unit cube vertices and edges
// ============================================================================
#define CANVAS_W    240            // full-screen canvas — redrawn every 33ms, prevents stale pixels
#define CANVAS_H    240
#define CANVAS_CX   120            // perspective projection centre X
#define CANVAS_CY   120            // perspective projection centre Y
#define FOV_DIST    3.0f           // perspective distance
#define CUBE_SCALE  88.0f          // pixels per unit at depth 0

// 8 vertices: v0–v3 back face (z=−0.5), v4–v7 front face (z=+0.5)
static const float k_verts[8][3] = {
    {-0.5f, -0.5f, -0.5f},   // v0 back-bottom-left
    { 0.5f, -0.5f, -0.5f},   // v1 back-bottom-right
    { 0.5f,  0.5f, -0.5f},   // v2 back-top-right
    {-0.5f,  0.5f, -0.5f},   // v3 back-top-left
    {-0.5f, -0.5f,  0.5f},   // v4 front-bottom-left
    { 0.5f, -0.5f,  0.5f},   // v5 front-bottom-right
    { 0.5f,  0.5f,  0.5f},   // v6 front-top-right
    {-0.5f,  0.5f,  0.5f},   // v7 front-top-left
};

// 12 edges as vertex index pairs
struct EdgeDef { uint8_t a, b; };
static const EdgeDef k_edges[12] = {
    {0,1}, {1,2}, {2,3}, {3,0},   // back face (originally z=-0.5)
    {4,5}, {5,6}, {6,7}, {7,4},   // front face (originally z=+0.5)
    {0,4}, {1,5}, {2,6}, {3,7},   // side connectors
};

// ============================================================================
// Rotation physics state
// ============================================================================
#define INERTIA      0.95f   // per-frame velocity retention (~1.5 s to stop)
#define GYRO_SCALE   1.0f    // dps → deg/s conversion
#define GYRO_NOISE   3.0f    // dps dead-zone to suppress sensor bias
#define KICK_SCALE   2.5f    // swipe pixels → deg/s angular impulse
#define SWIPE_THRESH 45      // px minimum to register as a swipe (45: responsive yet avoids tap misclassification)
#define DOUBLE_TAP_MS 600    // ms window for second tap (raised 400→600, comfortable under canvas load)

static float   s_yaw         =  25.0f;  // current orientation (degrees)
static float   s_pitch       = -20.0f;
static float   s_omega_yaw   =   0.0f;  // angular velocity (deg/s)
static float   s_omega_pitch =   0.0f;
static float   s_kick_yaw    =   0.0f;  // pending touch impulse (one-shot)
static float   s_kick_pitch  =   0.0f;
static int64_t s_last_us     =   0;

// ============================================================================
// Touch tracking and double-tap state
// ============================================================================
static lv_point_t s_touch_start      = {0, 0};
static bool       s_touch_active     = false;
static int64_t    s_splash_tap_us    = 0;   // timestamp of last tap on splash
static int64_t    s_status_tap_us    = 0;   // timestamp of last tap on status

// ============================================================================
// Driver name + init-code lookup (hw_manifest helper stubs not yet built)
// ============================================================================
static const char *drv_short_name(int idx)
{
    static const char *const names[CUBE32_DRV_COUNT] = {
        "PMU",   "RTC",    "Display", "LVGL",  "Touch",
        "Audio", "Modem",  "Camera",  "SD",    "BLE-OTA",
        "USB",   "Button", "Servo",   "Robot", "IMU",    "Mag",
    };
    return (idx >= 0 && idx < (int)CUBE32_DRV_COUNT) ? names[idx] : "?";
}

// ============================================================================
// LVGL UI objects
// ============================================================================
#if CONFIG_CUBE32_LVGL_ENABLED

// Returns a short status string (with LVGL symbol icons for OK/FAIL)
static const char *init_code_label(cube32_init_code_t code)
{
    switch (code) {
        case CUBE32_INIT_OK:           return LV_SYMBOL_OK;    // ✓
        case CUBE32_INIT_FAIL:         return LV_SYMBOL_CLOSE; // ✗
        case CUBE32_INIT_SKIPPED:      return "SKIP";
        case CUBE32_INIT_INITIALIZING: return "INIT";
        case CUBE32_INIT_NOT_PRESENT:  return "N/B";
        case CUBE32_INIT_NOT_REQUESTED:return "OFF";
        default:                       return "---";
    }
}

static lv_obj_t      *s_splash_screen = nullptr;
static lv_obj_t      *s_bat_lbl       = nullptr;  // live-updated battery label
static lv_obj_t      *s_status_screen = nullptr;
static lv_obj_t      *s_canvas        = nullptr;
static lv_draw_buf_t *s_draw_buf      = nullptr;

// ============================================================================
// 3D math: rotate vertex by yaw (Y axis) then pitch (X axis)
// ============================================================================
static void rotate_vertex(float yaw_deg, float pitch_deg,
                          const float in[3], float out[3])
{
    const float yaw   = yaw_deg   * (float)M_PI / 180.0f;
    const float pitch = pitch_deg * (float)M_PI / 180.0f;
    const float cy = cosf(yaw),   sy = sinf(yaw);
    const float cp = cosf(pitch), sp = sinf(pitch);

    // Ry — rotate around Y axis (yaw)
    float x1 =  cy * in[0] + sy * in[2];
    float y1 =  in[1];
    float z1 = -sy * in[0] + cy * in[2];

    // Rx — rotate around X axis (pitch)
    out[0] = x1;
    out[1] = cp * y1 - sp * z1;
    out[2] = sp * y1 + cp * z1;
}

// Perspective-project a 3D vertex to 2D canvas coordinates
static void project_vertex(const float v[3], int32_t &sx, int32_t &sy)
{
    // Camera is at z = +FOV_DIST looking toward -Z.
    // Vertices with larger vz are CLOSER and project LARGER.
    float denom = FOV_DIST - v[2];
    if (denom < 0.1f) denom = 0.1f;   // guard: vertex at or behind camera
    const float w = FOV_DIST / denom;
    sx = (int32_t)( v[0] * w * CUBE_SCALE) + CANVAS_CX;
    sy = (int32_t)(-v[1] * w * CUBE_SCALE) + CANVAS_CY;   // invert Y for screen
}

// ============================================================================
// Draw wireframe cube onto an LVGL canvas layer
// ============================================================================
static void draw_cube_wireframe(lv_layer_t *layer, float yaw, float pitch)
{
    float   rot[8][3];
    int32_t px[8], py[8];

    for (int i = 0; i < 8; i++) {
        rotate_vertex(yaw, pitch, k_verts[i], rot[i]);
        project_vertex(rot[i], px[i], py[i]);
    }

    lv_draw_line_dsc_t dsc;
    lv_draw_line_dsc_init(&dsc);
    dsc.opa = LV_OPA_COVER;

    for (const auto &e : k_edges) {
        // Dynamic depth coloring: average Z of the two rotated endpoints.
        // Threshold 0.15 cleanly separates face edges (avg ~±0.5)
        // from connector edges (avg ~0) at any orientation.
        float avg_z = (rot[e.a][2] + rot[e.b][2]) * 0.5f;
        if (avg_z > 0.15f) {
            dsc.color = lv_color_hex(EDGE_FRONT);  // front-facing: bright cyan
            dsc.width = 2;
        } else if (avg_z < -0.15f) {
            dsc.color = lv_color_hex(EDGE_BACK);   // rear-facing: dark blue
            dsc.width = 1;
        } else {
            dsc.color = lv_color_hex(EDGE_SIDE);   // side connectors: mid blue
            dsc.width = 1;
        }
        dsc.p1.x = px[e.a];  dsc.p1.y = py[e.a];
        dsc.p2.x = px[e.b];  dsc.p2.y = py[e.b];
        lv_draw_line(layer, &dsc);
    }
}

// ============================================================================
// Cube animation timer — ~30 fps
// ============================================================================
static void cube_timer_cb(lv_timer_t * /*timer*/)
{
    cube32::LvglDisplay &lvgl = cube32::LvglDisplay::instance();
    if (!lvgl.lock(5)) return;   // skip frame if LVGL is busy

    // Only animate while the splash screen is active
    if (lv_screen_active() != s_splash_screen || !s_canvas) {
        lvgl.unlock();
        return;
    }

    // ---- Time delta ----
    int64_t now_us = esp_timer_get_time();
    float dt = (s_last_us == 0) ? 0.033f
                                 : (float)(now_us - s_last_us) * 1e-6f;
    if (dt > 0.1f) dt = 0.1f;   // clamp after long suspend / first tick
    s_last_us = now_us;

    // ---- IMU drives angular velocity ----
#if CONFIG_CUBE32_IMU_ENABLED
    cube32::IMU &imu = cube32::IMU::instance();
    if (imu.isInitialized()) {
        cube32::IMUData d;
        if (imu.getData(d) == CUBE32_OK) {
            // gyro.x → yaw  (lateral roll of device → left/right cube spin)
            //               negated: positive gyro.x (tilt left) → negative yaw (turns left)
            // gyro.y → pitch (fore-aft tilt of device → up/down cube tilt)
            float tgt_yaw   = (fabsf(d.gyro.x) > GYRO_NOISE)
                                ? -d.gyro.x * GYRO_SCALE : 0.0f;
            float tgt_pitch = (fabsf(d.gyro.y) > GYRO_NOISE)
                                ?  d.gyro.y * GYRO_SCALE : 0.0f;
            // Blend toward target: tracks active rotation,
            // retains inertia when device becomes still
            s_omega_yaw   = s_omega_yaw   * INERTIA + tgt_yaw   * (1.0f - INERTIA);
            s_omega_pitch = s_omega_pitch * INERTIA + tgt_pitch * (1.0f - INERTIA);
        }
    } else
#endif
    {
        // No IMU: friction only — touch impulse decays naturally to rest
        s_omega_yaw   *= INERTIA;
        s_omega_pitch *= INERTIA;
    }

    // ---- Apply pending touch impulse (consumed once per swipe) ----
    s_omega_yaw   += s_kick_yaw;   s_kick_yaw   = 0.0f;
    s_omega_pitch += s_kick_pitch; s_kick_pitch = 0.0f;

    // ---- Integrate angles ----
    s_yaw   += s_omega_yaw   * dt;
    s_pitch += s_omega_pitch * dt;

    // ---- Redraw canvas ----
    lv_canvas_fill_bg(s_canvas, lv_color_hex(BG_COLOR), LV_OPA_COVER);
    lv_layer_t layer;
    lv_canvas_init_layer(s_canvas, &layer);
    draw_cube_wireframe(&layer, s_yaw, s_pitch);
    lv_canvas_finish_layer(s_canvas, &layer);

    lvgl.unlock();
}

// ============================================================================
// Touch + navigation event handlers
// ============================================================================
static void splash_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_PRESSED) {
        lv_indev_t *indev = lv_indev_active();
        if (indev) lv_indev_get_point(indev, &s_touch_start);
        s_touch_active = true;

    } else if (code == LV_EVENT_RELEASED && s_touch_active) {
        s_touch_active = false;

        lv_indev_t *indev = lv_indev_active();
        if (!indev) return;

        lv_point_t end;
        lv_indev_get_point(indev, &end);

        const int32_t dx = end.x - s_touch_start.x;
        const int32_t dy = end.y - s_touch_start.y;

        bool swiped = false;
        if (abs(dx) >= SWIPE_THRESH && abs(dx) >= abs(dy)) {
            s_kick_yaw += (float)dx * KICK_SCALE;
            swiped = true;
        }
        if (abs(dy) >= SWIPE_THRESH && abs(dy) > abs(dx)) {
            s_kick_pitch += (float)dy * KICK_SCALE;
            swiped = true;
        }

        // No swipe = tap → require double-tap to navigate
        if (!swiped) {
            int64_t now = esp_timer_get_time();
            if (s_splash_tap_us > 0 &&
                (now - s_splash_tap_us) < ((int64_t)DOUBLE_TAP_MS * 1000)) {
                lv_screen_load_anim(s_status_screen,
                                    LV_SCR_LOAD_ANIM_FADE_IN, 300, 0, false);
                s_splash_tap_us = 0;
            } else {
                s_splash_tap_us = now;
            }
        }
    }
}

static void status_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) == LV_EVENT_SHORT_CLICKED) {
        int64_t now = esp_timer_get_time();
        if (s_status_tap_us > 0 &&
            (now - s_status_tap_us) < ((int64_t)DOUBLE_TAP_MS * 1000)) {
            lv_screen_load_anim(s_splash_screen,
                                LV_SCR_LOAD_ANIM_FADE_IN, 300, 0, false);
            s_status_tap_us = 0;
        } else {
            s_status_tap_us = now;
        }
    }
}

// ============================================================================
// Create splash screen (3D cube + title + hint)
// ============================================================================
static void create_splash_screen(void)
{
    s_splash_screen = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(s_splash_screen, lv_color_hex(BG_COLOR), 0);
    lv_obj_set_style_bg_opa(s_splash_screen, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_splash_screen, 0, 0);
    lv_obj_clear_flag(s_splash_screen, LV_OBJ_FLAG_SCROLLABLE);

    // 240×240 canvas — covers the ENTIRE screen, filled with BG_COLOR every frame.
    // Created FIRST so title/hint labels (created after) have higher z-order and render on top.
    s_draw_buf = lv_draw_buf_create(CANVAS_W, CANVAS_H,
                                    LV_COLOR_FORMAT_RGB565, 0);
    if (s_draw_buf) {
        s_canvas = lv_canvas_create(s_splash_screen);
        lv_canvas_set_draw_buf(s_canvas, s_draw_buf);
        lv_obj_set_pos(s_canvas, 0, 0);
        lv_canvas_fill_bg(s_canvas, lv_color_hex(BG_COLOR), LV_OPA_COVER);
    } else {
        ESP_LOGE(TAG, "Canvas alloc failed (%d bytes)", CANVAS_W * CANVAS_H * 2);
    }

    // "CUBE32" title — created AFTER canvas so it renders on top
    lv_obj_t *title = lv_label_create(s_splash_screen);
    lv_label_set_text(title, "CUBE32");
    lv_obj_set_style_text_color(title, lv_color_hex(TITLE_COLOR), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 4);

    // Hint label at bottom
    lv_obj_t *hint = lv_label_create(s_splash_screen);
    lv_label_set_text(hint, LV_SYMBOL_RIGHT "  double-tap for system info");
    lv_obj_set_style_text_color(hint, lv_color_hex(DIM_COLOR), 0);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_14, 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -4);

    // Touch event registration
    lv_obj_add_flag(s_splash_screen, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_splash_screen, splash_event_cb,
                        LV_EVENT_PRESSED,  nullptr);
    lv_obj_add_event_cb(s_splash_screen, splash_event_cb,
                        LV_EVENT_RELEASED, nullptr);
}

// ============================================================================
// Battery label helper — called on creation and from the refresh timer
// ============================================================================
#if CONFIG_CUBE32_PMU_ENABLED
static void update_bat_label(void)
{
    if (!s_bat_lbl) return;
    auto &pmu = cube32::PMU::instance();
    char buf[56];
    if (!pmu.isInitialized()) {
        snprintf(buf, sizeof(buf), LV_SYMBOL_WARNING "  PMU unavailable");
    } else if (!pmu.isBatteryPresent()) {
        // No battery attached — use BATTERY_EMPTY as a "no battery" indicator
        snprintf(buf, sizeof(buf), LV_SYMBOL_BATTERY_EMPTY "  No battery");
    } else {
        int  pct = pmu.getBatteryPercent();
        auto mV  = (unsigned)pmu.getBatteryVoltage();
        // Pick icon based on charge level
        const char *icon;
        if      (pct >= 75) icon = LV_SYMBOL_BATTERY_FULL;
        else if (pct >= 50) icon = LV_SYMBOL_BATTERY_3;
        else if (pct >= 25) icon = LV_SYMBOL_BATTERY_2;
        else if (pct >= 5)  icon = LV_SYMBOL_BATTERY_1;
        else                icon = LV_SYMBOL_BATTERY_EMPTY;
        if (pmu.isCharging()) {
            snprintf(buf, sizeof(buf), "%s  %d%%  %u mV  " LV_SYMBOL_CHARGE, icon, pct, mV);
        } else {
            snprintf(buf, sizeof(buf), "%s  %d%%  %u mV", icon, pct, mV);
        }
    }
    lv_label_set_text(s_bat_lbl, buf);
}

static void bat_timer_cb(lv_timer_t * /*timer*/)
{
    cube32::LvglDisplay &lvgl = cube32::LvglDisplay::instance();
    if (!lvgl.lock(5)) return;
    update_bat_label();
    lvgl.unlock();
}
#endif // CONFIG_CUBE32_PMU_ENABLED

// ============================================================================
// Create status screen (driver grid + battery)
// ============================================================================
static void create_status_screen(void)
{
    s_status_screen = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(s_status_screen, lv_color_hex(BG_COLOR), 0);
    lv_obj_set_style_bg_opa(s_status_screen, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_status_screen, 0, 0);
    lv_obj_clear_flag(s_status_screen, LV_OBJ_FLAG_SCROLLABLE);

    // Title
    lv_obj_t *title = lv_label_create(s_status_screen);
    lv_label_set_text(title, "SYSTEM STATUS");
    lv_obj_set_style_text_color(title, lv_color_hex(TITLE_COLOR), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 4);

    // ---- Driver grid: 2 columns, 18px row height ----
    const cube32_hw_manifest_t *m = cube32_hw_manifest();
    const int COL_X[2]    = { 6, 122 };
    const int ROW_START_Y = 30;
    const int ROW_H       = 18;

    for (int i = 0; i < (int)CUBE32_DRV_COUNT; i++) {
        const cube32_init_record_t &rec = m->init_records[i];
        const char *drv_name = drv_short_name(i);
        // Always read the init code directly — it encodes all states
        const char *code_str = init_code_label(rec.code);
        uint32_t    dot_hex;
        switch (rec.code) {
            case CUBE32_INIT_OK:           dot_hex = DOT_OK;   break;
            case CUBE32_INIT_FAIL:         dot_hex = DOT_FAIL; break;
            case CUBE32_INIT_INITIALIZING: dot_hex = DOT_SKIP; break;
            case CUBE32_INIT_SKIPPED:      dot_hex = DOT_SKIP; break;
            default:                       dot_hex = DOT_NA;   break;
        }

        const int col = i % 2;
        const int row = i / 2;
        const int x   = COL_X[col];
        const int y   = ROW_START_Y + row * ROW_H;

        // Dot indicator (8×8 circle)
        lv_obj_t *dot = lv_obj_create(s_status_screen);
        lv_obj_set_size(dot, 8, 8);
        lv_obj_set_pos(dot, x, y + (ROW_H - 8) / 2);
        lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(dot, lv_color_hex(dot_hex), 0);
        lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(dot, 0, 0);
        lv_obj_set_style_pad_all(dot, 0, 0);
        lv_obj_clear_flag(dot, LV_OBJ_FLAG_SCROLLABLE);

        // Driver name + status code label
        char buf[20];
        snprintf(buf, sizeof(buf), "%-8s %s", drv_name, code_str);
        lv_obj_t *lbl = lv_label_create(s_status_screen);
        lv_label_set_text(lbl, buf);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(lbl, lv_color_hex(0xCCDDEE), 0);
        lv_obj_set_pos(lbl, x + 11, y + 1);
    }

    // ---- Separator line ----
    const int rows_used = ((int)CUBE32_DRV_COUNT + 1) / 2;
    const int bat_y_sep = ROW_START_Y + rows_used * ROW_H + 2;

    lv_obj_t *sep = lv_obj_create(s_status_screen);
    lv_obj_set_size(sep, 228, 1);
    lv_obj_set_pos(sep, 6, bat_y_sep);
    lv_obj_set_style_bg_color(sep, lv_color_hex(0x1A2D40), 0);
    lv_obj_set_style_bg_opa(sep, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(sep, 0, 0);
    lv_obj_clear_flag(sep, LV_OBJ_FLAG_SCROLLABLE);

    // ---- Battery row ----
#if CONFIG_CUBE32_PMU_ENABLED
    {
        s_bat_lbl = lv_label_create(s_status_screen);
        lv_label_set_text(s_bat_lbl, "");   // populated by update_bat_label()
        lv_obj_set_style_text_font(s_bat_lbl, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(s_bat_lbl, lv_color_hex(0xAABBCC), 0);
        lv_obj_set_pos(s_bat_lbl, 6, bat_y_sep + 4);
        update_bat_label();   // fill immediately on first display
    }
#endif

    // ---- Hint label ----
    lv_obj_t *hint = lv_label_create(s_status_screen);
    lv_label_set_text(hint, LV_SYMBOL_LEFT "  double-tap to return");
    lv_obj_set_style_text_color(hint, lv_color_hex(DIM_COLOR), 0);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_14, 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -4);

    // Double-tap to return to splash
    lv_obj_add_flag(s_status_screen, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_status_screen, status_event_cb,
                        LV_EVENT_SHORT_CLICKED, nullptr);
}

#endif // CONFIG_CUBE32_LVGL_ENABLED

// ============================================================================
// app_main
// ============================================================================
extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "CUBE32 Hello World");
    ESP_LOGI(TAG, "========================================");

    esp_err_t ret = cube32_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Board init failed! (0x%x)", ret);
        return;
    }

#if CONFIG_CUBE32_LVGL_ENABLED
    cube32::LvglDisplay &lvgl = cube32::LvglDisplay::instance();
    if (lvgl.isInitialized()) {
        lvgl.lock(1000);
        create_splash_screen();
        create_status_screen();
        lv_screen_load(s_splash_screen);
        lv_timer_create(cube_timer_cb, 33, nullptr);
#if CONFIG_CUBE32_PMU_ENABLED
        lv_timer_create(bat_timer_cb, 5000, nullptr);  // refresh battery every 5 s
#endif
        lvgl.unlock();
        ESP_LOGI(TAG, "UI ready — swipe or tilt to roll the cube, tap for status");
    } else {
        ESP_LOGW(TAG, "LVGL not initialized — console-only mode");
    }
#else
    ESP_LOGI(TAG, "LVGL not enabled — console-only mode");
#endif

    // Periodic console heartbeat
    while (true) {
#if CONFIG_CUBE32_PMU_ENABLED
        auto &pmu = cube32::PMU::instance();
        if (pmu.isInitialized()) {
            ESP_LOGI(TAG, "Heap: %" PRIu32 " | Batt: %d%% (%u mV) | %s",
                     esp_get_free_heap_size(),
                     pmu.getBatteryPercent(),
                     (unsigned)pmu.getBatteryVoltage(),
                     pmu.isCharging() ? "Charging" : "On battery");
        } else
#endif
        {
            ESP_LOGI(TAG, "Heap: %" PRIu32, esp_get_free_heap_size());
        }
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
