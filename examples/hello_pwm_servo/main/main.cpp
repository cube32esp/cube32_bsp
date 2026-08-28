/**
 * @file main.cpp
 * @brief CUBE32 Hello PWM Servo Example
 * 
 * This example demonstrates the PWM Servo driver with console commands
 * and LVGL UI showing servo status.
 * 
 * Console Commands:
 *   angle <ch> <deg>           - Set servo angle immediately
 *   move <ch> <deg> [speed]    - Smooth move (speed: 1-100%, default 50%)
 *   pulse <ch> <us>            - Set pulse width directly (microseconds)
 *   stop <ch>                  - Stop smooth move
 *   attach <ch>                - Attach servo channel
 *   detach <ch>                - Detach servo (stop PWM, power saving)
 *   action <name>              - Play head action (use 'actions' to list all)
 *   actions                    - List available actions
 *   cancel                     - Cancel running action
 *   status                     - Show all servo status
 *   help                       - Show help
 * 
 * Commands can also be sent from a BLE client via the BLE OTA text message
 * channel (cmd 0x04). The same command syntax is used.
 * 
 * Examples:
 *   angle 0 45        → Set servo 0 to 45°
 *   move 0 90 30      → Smooth move servo 0 to 90° at 30% speed
 *   move 1 0          → Smooth move servo 1 to 0° at 50% speed
 *   pulse 0 1500      → Set servo 0 pulse to 1500 µs
 *   detach 0          → Detach servo 0 (no PWM)
 * 
 * For 360° continuous rotation servos:
 *   angle 1 0         → Full speed clockwise
 *   angle 1 90        → Stop
 *   angle 1 180       → Full speed counter-clockwise
 *   angle 1 45        → Slow clockwise
 * 
 * Prerequisites:
 * - Enable Display + LVGL in Display Configuration
 * - Enable PWM Servo in PWM Servo Configuration  
 * - Enable fonts: LV_FONT_MONTSERRAT_14, LV_FONT_MONTSERRAT_24
 */

#include <cstdio>
#include <cinttypes>
#include <cstdlib>
#include <cstring>
#include <cstdarg>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_vfs_dev.h>
#include <driver/uart.h>
#include <driver/uart_vfs.h>

#include "cube32.h"

static const char *TAG = "hello_pwm_servo";

// ============================================================================
// Configuration
// ============================================================================

#define UI_UPDATE_INTERVAL_MS   200

// ============================================================================
// UI Colors (Dark Theme)
// ============================================================================

#define COLOR_BG            0x121212
#define COLOR_PANEL         0x1E1E2E
#define COLOR_TEXT          0xEAEAEA
#define COLOR_TEXT_DIM      0x888888
#define COLOR_ACCENT        0x64B5F6
#define COLOR_GREEN         0x00E676
#define COLOR_ORANGE        0xFFAB00
#define COLOR_RED           0xFF5252
#define COLOR_CYAN          0x00D4FF

// ============================================================================
// LVGL Widgets
// ============================================================================

// Per-channel widgets
struct ServoUI {
    lv_obj_t *title;
    lv_obj_t *angle_label;
    lv_obj_t *angle_arc;
    lv_obj_t *status_label;
    lv_obj_t *pin_label;
    lv_obj_t *type_label;
    lv_obj_t *moving_label;
};

static ServoUI s_servo_ui[CUBE32_SERVO_MAX_CHANNELS] = {};
static lv_timer_t *s_update_timer = nullptr;

// Track whether we're using face mode or servo panel mode
static bool s_face_mode = false;

// ============================================================================
// Face Expression <-> Head Action Mapping
// ============================================================================

#if defined(CONFIG_CUBE32_FACE_EXPRESSION_ENABLED) && defined(CONFIG_CUBE32_ROBOT_HEAD_ENABLED)
/**
 * @brief Map HeadActionId to the corresponding FaceExpressionId
 */
static cube32::FaceExpressionId headActionToFaceExpression(cube32::HeadActionId id) {
    switch (id) {
        case cube32::HeadActionId::NOD:         return cube32::FaceExpressionId::NOD;
        case cube32::HeadActionId::SHAKE:       return cube32::FaceExpressionId::SHAKE;
        case cube32::HeadActionId::CURIOUS:     return cube32::FaceExpressionId::CURIOUS;
        case cube32::HeadActionId::ATTENTION:   return cube32::FaceExpressionId::ATTENTION;
        case cube32::HeadActionId::LOOK_LEFT:   return cube32::FaceExpressionId::LOOK_LEFT;
        case cube32::HeadActionId::LOOK_RIGHT:  return cube32::FaceExpressionId::LOOK_RIGHT;
        case cube32::HeadActionId::LOOK_UP:     return cube32::FaceExpressionId::LOOK_UP;
        case cube32::HeadActionId::LOOK_DOWN:   return cube32::FaceExpressionId::LOOK_DOWN;
        case cube32::HeadActionId::SCAN:        return cube32::FaceExpressionId::SCAN;
        case cube32::HeadActionId::BOW:         return cube32::FaceExpressionId::BOW;
        case cube32::HeadActionId::SEARCH:      return cube32::FaceExpressionId::SEARCH;
        case cube32::HeadActionId::EXCITED:     return cube32::FaceExpressionId::EXCITED;
        case cube32::HeadActionId::SAD:         return cube32::FaceExpressionId::SAD;
        case cube32::HeadActionId::DOUBLE_TAKE: return cube32::FaceExpressionId::DOUBLE_TAKE;
        case cube32::HeadActionId::DIZZY:       return cube32::FaceExpressionId::DIZZY;
        default:                                return cube32::FaceExpressionId::IDLE;
    }
}

/**
 * @brief Play a head action with synchronized face expression
 * @param name Action name (case-insensitive)
 * @return CUBE32_OK on success
 */
static cube32_result_t play_action_with_face(const char* name) {
    cube32::RobotHead& head = cube32::RobotHead::instance();
    
    // Find the action ID by name for face mapping
    cube32::HeadActionId action_id = cube32::HeadActionId::COUNT;
    const char* const* names = cube32::RobotHead::getBuiltinActionNames();
    for (uint8_t i = 0; names[i]; i++) {
        if (strcasecmp(name, names[i]) == 0) {
            action_id = (cube32::HeadActionId)i;
            break;
        }
    }
    
    // Play head action
    cube32_result_t ret = head.playActionByName(name);
    
    // If face mode is active, trigger the matching face expression
    if (ret == CUBE32_OK && s_face_mode) {
        cube32::FaceDisplay& face = cube32::FaceDisplay::instance();
        if (face.isInitialized() && action_id != cube32::HeadActionId::COUNT) {
            cube32::FaceExpressionId expr_id = headActionToFaceExpression(action_id);
            face.playExpression(expr_id);
        }
    }
    
    return ret;
}
#endif

// ============================================================================
// Forward Declarations  
// ============================================================================

static void create_ui(void);
static void update_ui_timer_cb(lv_timer_t *timer);
static void console_task(void *pvParameters);
static void process_servo_command(const char *line, char *response, size_t response_size);
static void print_help(void);
static void print_status(void);

// ============================================================================
// Console Task
// ============================================================================

static void print_help() {
    printf("\n");
    printf("=== CUBE32 PWM Servo Console ===\n");
    printf("  angle <ch> <deg>           Set servo angle\n");
    printf("  move <ch> <deg> [speed]    Smooth move (speed: 1-100%%, default 50%%)\n");
    printf("  pulse <ch> <us>            Set pulse width (µs)\n");
    printf("  stop <ch>                  Stop smooth move\n");
    printf("  attach <ch>                Attach servo channel\n");
    printf("  detach <ch>                Detach servo (stop PWM)\n");
#ifdef CONFIG_CUBE32_ROBOT_HEAD_ENABLED
    printf("  action <name>              Play head action (use 'actions' to list)\n");
    printf("  actions                    List available actions\n");
    printf("  cancel                     Cancel running action\n");
#endif
    printf("  status                     Show all servo status\n");
    printf("  help                       Show this help\n");
    printf("\nFor 360° servos: 0°=CW max, 90°=stop, 180°=CCW max\n");
    printf("================================\n\n");
}

static void print_status() {
#ifdef CONFIG_CUBE32_SERVO_ENABLED
    cube32::PwmServo& servo = cube32::PwmServo::instance();
    printf("\n=== Servo Status ===\n");
    for (uint8_t ch = 0; ch < servo.getNumChannels(); ch++) {
        const cube32::ServoChannelState* state = servo.getChannelState(ch);
        if (!state) continue;
        printf("Ch%d: GPIO%d, %d°-type, angle=%.1f°, %s%s\n",
               ch, servo.getSignalPin(ch),
               servo.getMaxRotation(ch),
               state->current_angle,
               state->attached ? "attached" : "detached",
               state->moving ? ", moving" : "");
    }
    printf("====================\n\n");
#else
    printf("PWM Servo not enabled\n");
#endif
}

// ============================================================================
// Shared Command Processor
// ============================================================================

/**
 * @brief Process a servo command string (used by both console and BLE)
 * @param line  Null-terminated command string
 * @param response  Buffer to write response text (can be nullptr for console)
 * @param response_size  Size of response buffer
 */
static void process_servo_command(const char *line, char *response, size_t response_size) {
    if (!line || line[0] == '\0') return;

    // Helper to write response to both console and response buffer
    auto respond = [&](const char *fmt, ...) {
        va_list args;
        va_start(args, fmt);
        if (response && response_size > 0) {
            vsnprintf(response, response_size, fmt, args);
        }
        va_end(args);
        va_start(args, fmt);
        vprintf(fmt, args);
        va_end(args);
    };

    char cmd[16] = {};
    int ch = 0;
    float val1 = 0, val2 = 50;

#ifdef CONFIG_CUBE32_SERVO_ENABLED
    cube32::PwmServo& servo = cube32::PwmServo::instance();

    if (sscanf(line, "%15s", cmd) != 1) return;

    if (strcmp(cmd, "help") == 0) {
        print_help();
        if (response && response_size > 0) {
            snprintf(response, response_size, "Commands: angle, move, pulse, stop, attach, detach, status");
        }
    }
    else if (strcmp(cmd, "status") == 0) {
        print_status();
        if (response && response_size > 0) {
            // Build compact status for BLE
            int off = 0;
            for (uint8_t i = 0; i < servo.getNumChannels() && off < (int)response_size - 1; i++) {
                const cube32::ServoChannelState* state = servo.getChannelState(i);
                if (!state) continue;
                off += snprintf(response + off, response_size - off,
                    "Ch%d:%.1f° %s%s", i, state->current_angle,
                    state->attached ? "on" : "off",
                    (i < servo.getNumChannels() - 1) ? ", " : "");
            }
        }
    }
    else if (strcmp(cmd, "angle") == 0) {
        if (sscanf(line, "%*s %d %f", &ch, &val1) == 2) {
            cube32_result_t ret = servo.setAngle(ch, val1);
            if (ret == CUBE32_OK) {
                respond("Servo %d: angle set to %.1f°\n", ch, val1);
            } else {
                respond("Error: %d\n", ret);
            }
        } else {
            respond("Usage: angle <ch> <degrees>\n");
        }
    }
    else if (strcmp(cmd, "move") == 0) {
        int parsed = sscanf(line, "%*s %d %f %f", &ch, &val1, &val2);
        if (parsed >= 2) {
            if (parsed < 3) val2 = 50.0f;
            cube32_result_t ret = servo.smoothMove(ch, val1, val2);
            if (ret == CUBE32_OK) {
                respond("Servo %d: moving to %.1f° at %.0f%% speed\n", ch, val1, val2);
            } else if (ret == CUBE32_NOT_SUPPORTED) {
                respond("Smooth move not supported for 360° servos\n");
            } else {
                respond("Error: %d\n", ret);
            }
        } else {
            respond("Usage: move <ch> <degrees> [speed_pct]\n");
        }
    }
    else if (strcmp(cmd, "pulse") == 0) {
        int pulse_us = 0;
        if (sscanf(line, "%*s %d %d", &ch, &pulse_us) == 2) {
            cube32_result_t ret = servo.setPulseWidth(ch, (uint16_t)pulse_us);
            if (ret == CUBE32_OK) {
                respond("Servo %d: pulse width set to %d µs\n", ch, pulse_us);
            } else {
                respond("Error: %d\n", ret);
            }
        } else {
            respond("Usage: pulse <ch> <microseconds>\n");
        }
    }
    else if (strcmp(cmd, "stop") == 0) {
        if (sscanf(line, "%*s %d", &ch) == 1) {
            servo.stopMove(ch);
            respond("Servo %d: smooth move stopped\n", ch);
        } else {
            respond("Usage: stop <ch>\n");
        }
    }
    else if (strcmp(cmd, "attach") == 0) {
        if (sscanf(line, "%*s %d", &ch) == 1) {
            cube32_result_t ret = servo.attach(ch);
            respond("Servo %d: %s\n", ch, ret == CUBE32_OK ? "attached" : "error");
        } else {
            respond("Usage: attach <ch>\n");
        }
    }
    else if (strcmp(cmd, "detach") == 0) {
        if (sscanf(line, "%*s %d", &ch) == 1) {
            cube32_result_t ret = servo.detach(ch);
            respond("Servo %d: %s\n", ch, ret == CUBE32_OK ? "detached" : "error");
        } else {
            respond("Usage: detach <ch>\n");
        }
    }
#ifdef CONFIG_CUBE32_ROBOT_HEAD_ENABLED
    else if (strcmp(cmd, "action") == 0) {
        char name[32] = {};
        if (sscanf(line, "%*s %31s", name) == 1) {
#ifdef CONFIG_CUBE32_FACE_EXPRESSION_ENABLED
            cube32_result_t ret = play_action_with_face(name);
#else
            cube32::RobotHead& head = cube32::RobotHead::instance();
            cube32_result_t ret = head.playActionByName(name);
#endif
            if (ret == CUBE32_OK) {
                respond("Playing action: %s\n", name);
            } else if (ret == CUBE32_INVALID_ARG) {
                respond("Unknown action: %s\n", name);
            } else {
                respond("Error: %d\n", ret);
            }
        } else {
            respond("Usage: action <name>  (use 'actions' to list all)\n");
        }
    }
    else if (strcmp(cmd, "actions") == 0) {
        const char* const* names = cube32::RobotHead::getBuiltinActionNames();
        respond("Available actions:");
        for (int i = 0; names[i]; i++) {
            printf(" %s", names[i]);
        }
        printf("\n");
        if (response && response_size > 0) {
            int off = strlen(response);
            for (int i = 0; names[i] && off < (int)response_size - 1; i++) {
                off += snprintf(response + off, response_size - off, " %s", names[i]);
            }
        }
    }
    else if (strcmp(cmd, "cancel") == 0) {
        cube32::RobotHead& head = cube32::RobotHead::instance();
        head.stopAction();
#ifdef CONFIG_CUBE32_FACE_EXPRESSION_ENABLED
        if (s_face_mode) {
            cube32::FaceDisplay& face = cube32::FaceDisplay::instance();
            if (face.isInitialized()) {
                face.stopExpression();
            }
        }
#endif
        respond("Action cancelled\n");
    }
#endif
    else {
        respond("Unknown command: %s\n", cmd);
    }
#else
    (void)ch; (void)val1; (void)val2; (void)cmd;
    if (response && response_size > 0) {
        snprintf(response, response_size, "PWM Servo not enabled");
    }
    printf("PWM Servo not enabled in configuration\n");
#endif
}

static void console_task(void *pvParameters) {
    // Configure console I/O
    setvbuf(stdout, NULL, _IONBF, 0);
    ESP_ERROR_CHECK(uart_driver_install((uart_port_t)CONFIG_ESP_CONSOLE_UART_NUM,
                                        256, 0, 0, NULL, 0));
    uart_vfs_dev_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
    uart_vfs_dev_port_set_rx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CR);
    uart_vfs_dev_port_set_tx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CRLF);

    // Brief delay for LVGL init
    vTaskDelay(pdMS_TO_TICKS(500));

    printf("\nCUBE32 PWM Servo Demo\n");
    print_help();

    char line_buf[128];

    while (true) {
        printf("[SERVO] > ");
        fflush(stdout);

        // Read line
        int idx = 0;
        while (idx < (int)sizeof(line_buf) - 1) {
            int c = fgetc(stdin);
            if (c == EOF) {
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            if (c == '\n' || c == '\r') {
                printf("\n");
                break;
            }
            if (c == 127 || c == 8) { // Backspace
                if (idx > 0) {
                    idx--;
                    printf("\b \b");
                    fflush(stdout);
                }
                continue;
            }
            line_buf[idx++] = (char)c;
            printf("%c", (char)c);
            fflush(stdout);
        }
        line_buf[idx] = '\0';

        if (idx == 0) continue;

        process_servo_command(line_buf, nullptr, 0);
    }
}

// ============================================================================
// LVGL UI Creation
// ============================================================================

static void create_servo_panel(lv_obj_t *parent, uint8_t ch, int x, int y, int w, int h) {
    ServoUI& ui = s_servo_ui[ch];

    // Panel container
    lv_obj_t *panel = lv_obj_create(parent);
    lv_obj_set_size(panel, w, h);
    lv_obj_set_pos(panel, x, y);
    lv_obj_set_style_bg_color(panel, lv_color_hex(COLOR_PANEL), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(panel, lv_color_hex(0x333333), LV_PART_MAIN);
    lv_obj_set_style_border_width(panel, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(panel, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_all(panel, 6, LV_PART_MAIN);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);

    // Title
    ui.title = lv_label_create(panel);
    char title_str[16];
    snprintf(title_str, sizeof(title_str), "SERVO %d", ch);
    lv_label_set_text(ui.title, title_str);
    lv_obj_set_style_text_color(ui.title, lv_color_hex(COLOR_ACCENT), LV_PART_MAIN);
    lv_obj_set_style_text_font(ui.title, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(ui.title, LV_ALIGN_TOP_MID, 0, 0);

    // Arc for angle visualization
    ui.angle_arc = lv_arc_create(panel);
    int arc_size = (h < 140) ? 70 : 90;
    lv_obj_set_size(ui.angle_arc, arc_size, arc_size);
    lv_obj_align(ui.angle_arc, LV_ALIGN_CENTER, 0, -2);
    lv_arc_set_rotation(ui.angle_arc, 180);
    lv_arc_set_bg_angles(ui.angle_arc, 0, 180);
    lv_arc_set_range(ui.angle_arc, 0, 180);
    lv_arc_set_value(ui.angle_arc, 90);
    lv_obj_remove_flag(ui.angle_arc, LV_OBJ_FLAG_CLICKABLE);

    // Arc styles
    lv_obj_set_style_arc_color(ui.angle_arc, lv_color_hex(0x333333), LV_PART_MAIN);
    lv_obj_set_style_arc_width(ui.angle_arc, 6, LV_PART_MAIN);
    lv_obj_set_style_arc_color(ui.angle_arc, lv_color_hex(COLOR_CYAN), LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(ui.angle_arc, 6, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(ui.angle_arc, lv_color_hex(COLOR_CYAN), LV_PART_KNOB);
    lv_obj_set_style_pad_all(ui.angle_arc, 2, LV_PART_KNOB);

    // Angle value label (on top of arc)
    ui.angle_label = lv_label_create(panel);
    lv_label_set_text(ui.angle_label, "0.0°");
    lv_obj_set_style_text_color(ui.angle_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
#if LV_FONT_MONTSERRAT_24
    lv_obj_set_style_text_font(ui.angle_label, &lv_font_montserrat_24, LV_PART_MAIN);
#else
    lv_obj_set_style_text_font(ui.angle_label, &lv_font_montserrat_14, LV_PART_MAIN);
#endif
    lv_obj_align_to(ui.angle_label, ui.angle_arc, LV_ALIGN_CENTER, 0, 10);

    // Type label (90°/180°/360°)
    ui.type_label = lv_label_create(panel);
    lv_label_set_text(ui.type_label, "---");
    lv_obj_set_style_text_color(ui.type_label, lv_color_hex(COLOR_TEXT_DIM), LV_PART_MAIN);
    lv_obj_set_style_text_font(ui.type_label, &lv_font_montserrat_10, LV_PART_MAIN);
    lv_obj_align(ui.type_label, LV_ALIGN_BOTTOM_LEFT, 0, 0);

    // Pin label
    ui.pin_label = lv_label_create(panel);
    lv_label_set_text(ui.pin_label, "GPIO --");
    lv_obj_set_style_text_color(ui.pin_label, lv_color_hex(COLOR_TEXT_DIM), LV_PART_MAIN);
    lv_obj_set_style_text_font(ui.pin_label, &lv_font_montserrat_10, LV_PART_MAIN);
    lv_obj_align(ui.pin_label, LV_ALIGN_BOTTOM_RIGHT, 0, 0);

    // Status label
    ui.status_label = lv_label_create(panel);
    lv_label_set_text(ui.status_label, "---");
    lv_obj_set_style_text_color(ui.status_label, lv_color_hex(COLOR_TEXT_DIM), LV_PART_MAIN);
    lv_obj_set_style_text_font(ui.status_label, &lv_font_montserrat_10, LV_PART_MAIN);
    lv_obj_align(ui.status_label, LV_ALIGN_BOTTOM_MID, 0, -14);

    // Moving indicator
    ui.moving_label = lv_label_create(panel);
    lv_label_set_text(ui.moving_label, "");
    lv_obj_set_style_text_color(ui.moving_label, lv_color_hex(COLOR_ORANGE), LV_PART_MAIN);
    lv_obj_set_style_text_font(ui.moving_label, &lv_font_montserrat_10, LV_PART_MAIN);
    lv_obj_align(ui.moving_label, LV_ALIGN_TOP_RIGHT, 0, 0);
}

static void create_ui() {
#ifdef CONFIG_CUBE32_LVGL_ENABLED
    cube32::LvglDisplay& lvgl = cube32::LvglDisplay::instance();
    if (!lvgl.lock(1000)) {
        ESP_LOGE(TAG, "Failed to lock LVGL mutex");
        return;
    }

    lv_obj_t *scr = lv_screen_active();

    // Dark theme background
    lv_obj_set_style_bg_color(scr, lv_color_hex(COLOR_BG), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);

#if defined(CONFIG_CUBE32_FACE_EXPRESSION_ENABLED) && defined(CONFIG_CUBE32_ROBOT_HEAD_ENABLED)
    // Face expression mode: show animated robot face
    cube32::FaceDisplay& face = cube32::FaceDisplay::instance();
    cube32_result_t face_ret = face.begin(scr);
    if (face_ret == CUBE32_OK) {
        s_face_mode = true;
        ESP_LOGI(TAG, "Face expression UI created");
    } else {
        ESP_LOGW(TAG, "Face expression init failed (%d), falling back to servo panels", face_ret);
        s_face_mode = false;
    }
#endif

    // Servo panel mode: show servo status gauges (fallback or standalone)
    if (!s_face_mode) {
        uint16_t scr_w = lvgl.getWidth();
        uint16_t scr_h = lvgl.getHeight();

        // Title
        lv_obj_t *title = lv_label_create(scr);
        lv_label_set_text(title, LV_SYMBOL_SETTINGS " PWM Servo");
        lv_obj_set_style_text_color(title, lv_color_hex(COLOR_ACCENT), LV_PART_MAIN);
        lv_obj_set_style_text_font(title, &lv_font_montserrat_14, LV_PART_MAIN);
        lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 4);

        // Calculate panel layout
        int top_margin = 24;
        int panel_gap = 6;
        int panel_w = (scr_w - panel_gap * 3) / 2;
        int panel_h = scr_h - top_margin - panel_gap * 2;

        // Create servo panels
#ifdef CONFIG_CUBE32_SERVO_ENABLED
        cube32::PwmServo& servo = cube32::PwmServo::instance();
        uint8_t num_ch = servo.getNumChannels();

        if (num_ch == 1) {
            // Single channel - center it
            create_servo_panel(scr, 0, (scr_w - panel_w) / 2, top_margin + panel_gap, panel_w, panel_h);
        } else {
            // Two channels side by side
            create_servo_panel(scr, 0, panel_gap, top_margin + panel_gap, panel_w, panel_h);
            create_servo_panel(scr, 1, panel_gap * 2 + panel_w, top_margin + panel_gap, panel_w, panel_h);
        }

        // Initialize static labels with config info
        for (uint8_t ch = 0; ch < num_ch; ch++) {
            char buf[32];
            snprintf(buf, sizeof(buf), "%d°", servo.getMaxRotation(ch));
            lv_label_set_text(s_servo_ui[ch].type_label, buf);

            snprintf(buf, sizeof(buf), "GPIO%d", servo.getSignalPin(ch));
            lv_label_set_text(s_servo_ui[ch].pin_label, buf);
        }
#endif

        // Create UI update timer for servo panels
        s_update_timer = lv_timer_create(update_ui_timer_cb, UI_UPDATE_INTERVAL_MS, nullptr);
    }

    lvgl.unlock();
    ESP_LOGI(TAG, "UI created (mode: %s)", s_face_mode ? "face" : "servo_panels");
#endif
}

// ============================================================================
// LVGL UI Update
// ============================================================================

static void update_ui_timer_cb(lv_timer_t *timer) {
    (void)timer;

#if defined(CONFIG_CUBE32_LVGL_ENABLED) && defined(CONFIG_CUBE32_SERVO_ENABLED)
    // Servo panel mode only — face mode has its own animation timer
    if (s_face_mode) return;

    cube32::LvglDisplay& lvgl = cube32::LvglDisplay::instance();
    if (!lvgl.lock(10)) return;

    cube32::PwmServo& servo = cube32::PwmServo::instance();

    for (uint8_t ch = 0; ch < servo.getNumChannels(); ch++) {
        const cube32::ServoChannelState* state = servo.getChannelState(ch);
        if (!state) continue;

        ServoUI& ui = s_servo_ui[ch];

        // Update angle label
        if (ui.angle_label) {
            char buf[16];
            snprintf(buf, sizeof(buf), "%.1f°", state->current_angle);
            lv_label_set_text(ui.angle_label, buf);
            lv_obj_align_to(ui.angle_label, ui.angle_arc, LV_ALIGN_CENTER, 0, 10);
        }

        // Update arc value
        if (ui.angle_arc) {
            uint16_t max_rot = servo.getMaxRotation(ch);
            float max_angle = (max_rot == 360) ? 180.0f : (float)max_rot;
            int arc_val = (int)((state->current_angle / max_angle) * 180.0f);
            if (arc_val > 180) arc_val = 180;
            if (arc_val < 0) arc_val = 0;
            lv_arc_set_value(ui.angle_arc, arc_val);
        }

        // Update status label  
        if (ui.status_label) {
            if (state->attached) {
                lv_label_set_text(ui.status_label, "ATTACHED");
                lv_obj_set_style_text_color(ui.status_label, lv_color_hex(COLOR_GREEN), LV_PART_MAIN);
            } else {
                lv_label_set_text(ui.status_label, "DETACHED");
                lv_obj_set_style_text_color(ui.status_label, lv_color_hex(COLOR_RED), LV_PART_MAIN);
            }
        }

        // Update moving indicator
        if (ui.moving_label) {
            if (state->moving) {
                lv_label_set_text(ui.moving_label, LV_SYMBOL_REFRESH);
                lv_obj_set_style_text_color(ui.moving_label, lv_color_hex(COLOR_ORANGE), LV_PART_MAIN);
            } else {
                lv_label_set_text(ui.moving_label, "");
            }
        }
    }

    lvgl.unlock();
#endif
}

// ============================================================================
// Main Entry Point
// ============================================================================

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Starting CUBE32 Hello PWM Servo Demo");

    // Initialize CUBE32 board
    ESP_ERROR_CHECK(cube32_init());

    // Create LVGL UI
#ifdef CONFIG_CUBE32_LVGL_ENABLED
    create_ui();
#endif

    // Move robot head to attention (center) position on startup
    // Delay first to let servos settle at their default position
#ifdef CONFIG_CUBE32_ROBOT_HEAD_ENABLED
    {
        cube32::RobotHead& head = cube32::RobotHead::instance();
        if (head.isInitialized()) {
            ESP_LOGI(TAG, "Waiting for servos to settle before attention...");
            vTaskDelay(pdMS_TO_TICKS(1000));
            ESP_LOGI(TAG, "Setting robot head to attention position");
#ifdef CONFIG_CUBE32_FACE_EXPRESSION_ENABLED
            play_action_with_face("attention");
#else
            head.playAction(cube32::HeadActionId::ATTENTION);
#endif
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
#endif

    // Subscribe to BLE text messages for servo commands
#ifdef CONFIG_CUBE32_BLE_OTA_ENABLED
    cube32::BleOta& ble = cube32::BleOta::instance();
    if (ble.isInitialized() && !ble.isBypassed()) {
        ble.subscribeTextMessage([](const char* text, size_t len) {
            ESP_LOGI(TAG, "BLE command received: %s", text);
            char response[128];
            process_servo_command(text, response, sizeof(response));
            // Send result back to BLE client
            cube32::BleOta::instance().sendTextResponse(response);
        });
        ESP_LOGI(TAG, "BLE text message subscriber registered for servo commands");
    }
#endif

    // Start console task
    xTaskCreate(console_task, "console", 4096, nullptr, 5, nullptr);

    // Main loop
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
