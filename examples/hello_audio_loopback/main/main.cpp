/**
 * @file main.cpp
 * @brief CUBE32 Hello Audio Loopback Example
 *
 * Real-time microphone-to-speaker loopback test for evaluating Acoustic Echo
 * Cancellation (AEC). Captures microphone input and plays it back through the
 * speaker continuously with selectable AEC mode.
 *
 * AEC modes (hardware-dependent):
 *   No AEC  – direct passthrough; acoustic feedback/howling will occur at high
 *             volume. This is intentional — it demonstrates what AEC prevents.
 *   HW AEC  – hardware reference loopback (ES7210 ch1) + esp-sr afe_aec.
 *             Only available on the Dedicated Audio Module (ES7210 present).
 *             Omitted from the dropdown automatically on integrated boards.
 *   SW AEC  – digital playback buffer as reference + esp-sr aec.
 *             Works on all hardware variants.
 *
 * AEC operates at 16 kHz only.
 *
 * Architecture:
 *   Two FreeRTOS tasks connected by two queues:
 *
 *   [MIC read] → [AEC process] → [s_loopback_queue] → [Speaker write]
 *                     ↑                                       │
 *               [s_ref_queue] ←──── (SW AEC only: copy of written frame)
 */

#include <cstdio>
#include <cinttypes>
#include <cstring>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>
#include <esp_log.h>
#include <esp_heap_caps.h>

#include "cube32.h"

#include "esp_aec.h"
#include "esp_afe_aec.h"

static const char *TAG = "hello_audio_loopback";

// ============================================================================
// Configuration
// ============================================================================

#define LOOPBACK_SAMPLE_RATE_HZ  16000  // AEC requires 16 kHz
#define AEC_FILTER_LENGTH        8      // Filter taps — higher = handles longer echo paths
#define LOOPBACK_QUEUE_DEPTH     4      // Keep small for low latency
#define REF_QUEUE_DEPTH          12     // SW AEC reference queue (must be > REF_DELAY_FRAMES)
#define REF_DELAY_FRAMES         2      // Silent pre-fill frames to phase-align the reference
                                        // with the acoustic echo path (each frame = 32 ms at 16 kHz/512)
#define INITIAL_SPEAKER_VOLUME   45     // 0–100 %
#define INITIAL_MIC_GAIN         30     // dB  (PGA range 0–60 dB)

// ============================================================================
// Types
// ============================================================================

typedef enum {
    MODE_NO_AEC = 0,
    MODE_HW_AEC = 1,
    MODE_SW_AEC = 2,
} loopback_mode_t;

typedef enum {
    STATE_IDLE    = 0,
    STATE_RUNNING = 1,
} app_state_t;

#define EVT_PLAY_DONE  (1 << 0)

// ============================================================================
// Global State
// ============================================================================

static app_state_t        s_state          = STATE_IDLE;
static SemaphoreHandle_t  s_state_mutex    = nullptr;
static volatile bool      s_stop_requested = false;
static loopback_mode_t    s_current_mode   = MODE_NO_AEC;
static EventGroupHandle_t s_evt_group      = nullptr;
static QueueHandle_t      s_loopback_queue = nullptr;  // capture → play task
static QueueHandle_t      s_ref_queue      = nullptr;  // SW AEC: play → capture
static TaskHandle_t       s_cap_task       = nullptr;
static TaskHandle_t       s_play_task      = nullptr;
static int                s_aec_chunk      = 512;      // shared with play task
static bool               s_hw_aec_avail   = false;    // resolved at UI creation

// Dropdown index → AEC mode mapping (built dynamically based on hardware)
static loopback_mode_t s_mode_map[3];
static int             s_mode_count = 0;

// LVGL widgets
static lv_obj_t *s_status_label       = nullptr;
static lv_obj_t *s_mode_dropdown      = nullptr;
static lv_obj_t *s_start_btn         = nullptr;
static lv_obj_t *s_stop_btn          = nullptr;
static lv_obj_t *s_speaker_vol_slider = nullptr;
static lv_obj_t *s_speaker_vol_label  = nullptr;
static lv_obj_t *s_mic_gain_slider   = nullptr;
static lv_obj_t *s_mic_gain_label    = nullptr;
static lv_obj_t *s_hint_label        = nullptr;

// ============================================================================
// Forward Declarations
// ============================================================================

static void create_ui(void);
static void update_ui_state(void);
static void start_btn_cb(lv_event_t *e);
static void stop_btn_cb(lv_event_t *e);
static void dropdown_cb(lv_event_t *e);
static void speaker_vol_cb(lv_event_t *e);
static void mic_gain_cb(lv_event_t *e);
static void loopback_capture_task(void *arg);
static void loopback_play_task(void *arg);

// ============================================================================
// State Helpers
// ============================================================================

static app_state_t get_state(void) {
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    app_state_t st = s_state;
    xSemaphoreGive(s_state_mutex);
    return st;
}

static void set_state(app_state_t st) {
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    s_state = st;
    xSemaphoreGive(s_state_mutex);
}

// ============================================================================
// UI Creation
// ============================================================================

static void create_ui(void) {
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x1a1a2e), 0);

    // ---- Title ----
    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, LV_SYMBOL_AUDIO " Audio Loopback");
    lv_obj_set_style_text_color(title, lv_color_hex(0xeaeaea), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 6);

    // ---- AEC Mode Dropdown ----
    // Options are built at runtime based on hardware capability.
    // HW AEC is included only when ES7210 (Dedicated Audio Module) is detected.
#ifdef CONFIG_CUBE32_AUDIO_ENABLED
    s_hw_aec_avail = (cube32::AudioCodec::instance().getConfig().adc_source
                      == cube32::AdcSource::ES7210);
#endif
    s_mode_dropdown = lv_dropdown_create(scr);
    if (s_hw_aec_avail) {
        lv_dropdown_set_options(s_mode_dropdown, "No AEC\nHW AEC\nSW AEC");
        s_mode_map[0] = MODE_NO_AEC;
        s_mode_map[1] = MODE_HW_AEC;
        s_mode_map[2] = MODE_SW_AEC;
        s_mode_count  = 3;
    } else {
        lv_dropdown_set_options(s_mode_dropdown, "No AEC\nSW AEC");
        s_mode_map[0] = MODE_NO_AEC;
        s_mode_map[1] = MODE_SW_AEC;
        s_mode_count  = 2;
    }
    // Default selection: SW AEC is always the last entry in s_mode_map
    lv_dropdown_set_selected(s_mode_dropdown, (uint32_t)(s_mode_count - 1));
    lv_obj_set_size(s_mode_dropdown, 150, 32);
    lv_obj_set_style_text_font(s_mode_dropdown, &lv_font_montserrat_14, 0);
    lv_obj_set_style_bg_color(s_mode_dropdown, lv_color_hex(0x2d2d4e), 0);
    lv_obj_set_style_text_color(s_mode_dropdown, lv_color_hex(0xeaeaea), 0);
    lv_obj_set_style_border_color(s_mode_dropdown, lv_color_hex(0x4488ff), 0);
    lv_obj_align(s_mode_dropdown, LV_ALIGN_TOP_MID, 0, 28);
    lv_obj_add_event_cb(s_mode_dropdown, dropdown_cb, LV_EVENT_VALUE_CHANGED, nullptr);

    // ---- Status Label ----
    s_status_label = lv_label_create(scr);
    lv_label_set_text(s_status_label, "Idle");
    lv_obj_set_style_text_color(s_status_label, lv_color_hex(0x00ff88), 0);
    lv_obj_set_style_text_font(s_status_label, &lv_font_montserrat_14, 0);
    lv_obj_align(s_status_label, LV_ALIGN_TOP_MID, 0, 70);

    // ---- Start / Stop Buttons ----
    lv_obj_t *ctrl_row = lv_obj_create(scr);
    lv_obj_remove_style_all(ctrl_row);
    lv_obj_set_size(ctrl_row, 210, 40);
    lv_obj_align(ctrl_row, LV_ALIGN_TOP_MID, 0, 92);
    lv_obj_set_flex_flow(ctrl_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ctrl_row, LV_FLEX_ALIGN_SPACE_EVENLY,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    s_start_btn = lv_btn_create(ctrl_row);
    lv_obj_set_size(s_start_btn, 90, 36);
    lv_obj_set_style_bg_color(s_start_btn, lv_color_hex(0x22aa55), 0);
    lv_obj_add_event_cb(s_start_btn, start_btn_cb, LV_EVENT_CLICKED, nullptr);
    {
        lv_obj_t *lbl = lv_label_create(s_start_btn);
        lv_label_set_text(lbl, LV_SYMBOL_PLAY " Start");
        lv_obj_center(lbl);
    }

    s_stop_btn = lv_btn_create(ctrl_row);
    lv_obj_set_size(s_stop_btn, 90, 36);
    lv_obj_set_style_bg_color(s_stop_btn, lv_color_hex(0x666666), 0);
    lv_obj_add_event_cb(s_stop_btn, stop_btn_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_state(s_stop_btn, LV_STATE_DISABLED);
    {
        lv_obj_t *lbl = lv_label_create(s_stop_btn);
        lv_label_set_text(lbl, LV_SYMBOL_STOP " Stop");
        lv_obj_center(lbl);
    }

    // ---- Speaker Volume ----
    s_speaker_vol_label = lv_label_create(scr);
    {
        char buf[40];
        snprintf(buf, sizeof(buf), LV_SYMBOL_VOLUME_MAX " Speaker: %d%%", INITIAL_SPEAKER_VOLUME);
        lv_label_set_text(s_speaker_vol_label, buf);
    }
    lv_obj_set_style_text_color(s_speaker_vol_label, lv_color_hex(0xaaaaaa), 0);
    lv_obj_set_style_text_font(s_speaker_vol_label, &lv_font_montserrat_14, 0);
    lv_obj_align(s_speaker_vol_label, LV_ALIGN_TOP_MID, 0, 144);

    s_speaker_vol_slider = lv_slider_create(scr);
    lv_obj_set_size(s_speaker_vol_slider, 180, 10);
    lv_slider_set_range(s_speaker_vol_slider, 0, 100);
    lv_slider_set_value(s_speaker_vol_slider, INITIAL_SPEAKER_VOLUME, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(s_speaker_vol_slider, lv_color_hex(0x333333), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_speaker_vol_slider, lv_color_hex(0x4488ff), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(s_speaker_vol_slider, lv_color_hex(0x6699ff), LV_PART_KNOB);
    lv_obj_align(s_speaker_vol_slider, LV_ALIGN_TOP_MID, 0, 164);
    lv_obj_add_event_cb(s_speaker_vol_slider, speaker_vol_cb, LV_EVENT_VALUE_CHANGED, nullptr);

    // ---- Mic Gain ----
    s_mic_gain_label = lv_label_create(scr);
    {
        char buf[40];
        snprintf(buf, sizeof(buf), LV_SYMBOL_AUDIO " Mic Gain: %d dB", INITIAL_MIC_GAIN);
        lv_label_set_text(s_mic_gain_label, buf);
    }
    lv_obj_set_style_text_color(s_mic_gain_label, lv_color_hex(0xaaaaaa), 0);
    lv_obj_set_style_text_font(s_mic_gain_label, &lv_font_montserrat_14, 0);
    lv_obj_align(s_mic_gain_label, LV_ALIGN_TOP_MID, 0, 196);

    s_mic_gain_slider = lv_slider_create(scr);
    lv_obj_set_size(s_mic_gain_slider, 180, 10);
    lv_slider_set_range(s_mic_gain_slider, 0, 60);
    lv_slider_set_value(s_mic_gain_slider, INITIAL_MIC_GAIN, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(s_mic_gain_slider, lv_color_hex(0x333333), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_mic_gain_slider, lv_color_hex(0xff6644), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(s_mic_gain_slider, lv_color_hex(0xff8866), LV_PART_KNOB);
    lv_obj_align(s_mic_gain_slider, LV_ALIGN_TOP_MID, 0, 216);
    lv_obj_add_event_cb(s_mic_gain_slider, mic_gain_cb, LV_EVENT_VALUE_CHANGED, nullptr);

    // ---- Bottom Hint ----
    s_hint_label = lv_label_create(scr);
    lv_label_set_text(s_hint_label, "");
    lv_obj_set_style_text_color(s_hint_label, lv_color_hex(0xff9944), 0);
    lv_obj_set_style_text_font(s_hint_label, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_align(s_hint_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(s_hint_label, 220);
    lv_label_set_long_mode(s_hint_label, LV_LABEL_LONG_WRAP);
    lv_obj_align(s_hint_label, LV_ALIGN_BOTTOM_MID, 0, -4);

    // Set initial hint text for the default dropdown selection (No AEC)
    dropdown_cb(nullptr);
}

// ============================================================================
// UI State Update (call with lvgl_port_lock held when called from a task)
// ============================================================================

static void update_ui_state(void) {
    app_state_t st = get_state();
    if (st == STATE_IDLE) {
        lv_label_set_text(s_status_label, "Idle");
        lv_obj_set_style_text_color(s_status_label, lv_color_hex(0x00ff88), 0);
        lv_obj_clear_state(s_start_btn, LV_STATE_DISABLED);
        lv_obj_add_state(s_stop_btn, LV_STATE_DISABLED);
        lv_obj_clear_state(s_mode_dropdown, LV_STATE_DISABLED);
    } else {
        // Running: status text is set by the capture task after codec init
        lv_obj_add_state(s_start_btn, LV_STATE_DISABLED);
        lv_obj_clear_state(s_stop_btn, LV_STATE_DISABLED);
        lv_obj_add_state(s_mode_dropdown, LV_STATE_DISABLED);
    }
}

// ============================================================================
// LVGL Callbacks (called from the LVGL task — do NOT call lvgl_port_lock here)
// ============================================================================

static void dropdown_cb(lv_event_t *e) {
    (void)e;
    if (!s_mode_dropdown || !s_hint_label) return;
    int idx = (int)lv_dropdown_get_selected(s_mode_dropdown);
    if (idx < 0 || idx >= s_mode_count) return;
    switch ((loopback_mode_t)s_mode_map[idx]) {
        case MODE_NO_AEC:
            lv_label_set_text(s_hint_label,
                LV_SYMBOL_WARNING " No AEC: feedback/howling at high volume");
            break;
        case MODE_HW_AEC:
            lv_label_set_text(s_hint_label, "HW AEC: hardware echo cancellation");
            break;
        case MODE_SW_AEC:
            lv_label_set_text(s_hint_label, "SW AEC: software echo cancellation");
            break;
    }
}

static void start_btn_cb(lv_event_t *e) {
    (void)e;
    if (get_state() != STATE_IDLE) return;
#ifdef CONFIG_CUBE32_AUDIO_ENABLED
    if (!cube32::AudioCodec::instance().isInitialized()) {
        lv_label_set_text(s_status_label, "Audio not available!");
        lv_obj_set_style_text_color(s_status_label, lv_color_hex(0xff4444), 0);
        return;
    }
#endif
    int idx = (int)lv_dropdown_get_selected(s_mode_dropdown);
    s_current_mode = (idx >= 0 && idx < s_mode_count) ? s_mode_map[idx] : MODE_NO_AEC;
    s_stop_requested = false;

    ESP_LOGI(TAG, "Starting loopback: mode=%d", (int)s_current_mode);
    xTaskCreate(loopback_capture_task, "lb_cap", 8192, nullptr, 6, &s_cap_task);
}

static void stop_btn_cb(lv_event_t *e) {
    (void)e;
    if (get_state() != STATE_IDLE) {
        ESP_LOGI(TAG, "Stop requested");
        s_stop_requested = true;
    }
}

// Speaker volume — live adjustment, no task restart required
static void speaker_vol_cb(lv_event_t *e) {
    int val = lv_slider_get_value((lv_obj_t *)lv_event_get_target(e));
    char buf[40];
    snprintf(buf, sizeof(buf), LV_SYMBOL_VOLUME_MAX " Speaker: %d%%", val);
    lv_label_set_text(s_speaker_vol_label, buf);
#ifdef CONFIG_CUBE32_AUDIO_ENABLED
    cube32::AudioCodec::instance().setOutputVolume(val);
#endif
}

// Mic gain — live adjustment, no task restart required
static void mic_gain_cb(lv_event_t *e) {
    int val = lv_slider_get_value((lv_obj_t *)lv_event_get_target(e));
    char buf[40];
    snprintf(buf, sizeof(buf), LV_SYMBOL_AUDIO " Mic Gain: %d dB", val);
    lv_label_set_text(s_mic_gain_label, buf);
#ifdef CONFIG_CUBE32_AUDIO_ENABLED
    cube32::AudioCodec::instance().setInputGain(val);
#endif
}

// ============================================================================
// Loopback Play Task
// Pops processed frames from s_loopback_queue, writes them to the speaker,
// and (SW AEC only) pushes a copy back to s_ref_queue as the echo reference.
// ============================================================================

static void loopback_play_task(void *arg) {
    const int chunk = (int)(intptr_t)arg;

    cube32::AudioCodec &codec = cube32::AudioCodec::instance();

    int16_t *play_buf  = (int16_t *)heap_caps_malloc(chunk * sizeof(int16_t), MALLOC_CAP_INTERNAL);
    // dummy_buf is used only when the SW AEC ref queue is full (drop oldest frame)
    int16_t *dummy_buf = (int16_t *)heap_caps_malloc(chunk * sizeof(int16_t), MALLOC_CAP_INTERNAL);

    if (!play_buf || !dummy_buf) {
        ESP_LOGE(TAG, "play_buf alloc failed");
        heap_caps_free(play_buf);
        heap_caps_free(dummy_buf);
        xEventGroupSetBits(s_evt_group, EVT_PLAY_DONE);
        vTaskDelete(nullptr);
        return;
    }

    codec.enableOutput(true, LOOPBACK_SAMPLE_RATE_HZ);

    while (!s_stop_requested) {
        // Block up to 200 ms for the next processed frame
        if (xQueueReceive(s_loopback_queue, play_buf, pdMS_TO_TICKS(200)) != pdTRUE) {
            continue;
        }
        codec.write(play_buf, chunk);

        // SW AEC: feed a copy of the played frame back as reference for the capture task
        if (s_current_mode == MODE_SW_AEC && s_ref_queue) {
            if (xQueueSend(s_ref_queue, play_buf, 0) != pdTRUE) {
                // Queue full — drop oldest and retry
                xQueueReceive(s_ref_queue, dummy_buf, 0);
                xQueueSend(s_ref_queue, play_buf, 0);
            }
        }
    }

    codec.enableOutput(false);
    heap_caps_free(play_buf);
    heap_caps_free(dummy_buf);

    xEventGroupSetBits(s_evt_group, EVT_PLAY_DONE);
    vTaskDelete(nullptr);
}

// ============================================================================
// Loopback Capture Task
// Reinitialises the codec, creates the AEC pipeline, spawns the play task,
// then loops: read mic → AEC process → push to loopback queue.
// ============================================================================

static void loopback_capture_task(void *arg) {
    (void)arg;

    set_state(STATE_RUNNING);
    if (lvgl_port_lock(100)) { update_ui_state(); lvgl_port_unlock(); }

#ifdef CONFIG_CUBE32_AUDIO_ENABLED
    do {
        cube32::AudioCodec &codec = cube32::AudioCodec::instance();
        loopback_mode_t mode = s_current_mode;

        // ---- Reinit codec at 16 kHz with requested AEC mode ----
        cube32::AecMode aec = (mode == MODE_HW_AEC) ? cube32::AecMode::HW :
                              (mode == MODE_SW_AEC) ? cube32::AecMode::SW :
                                                      cube32::AecMode::NONE;
        codec.end();
        {
            // Preserve hw-detected settings (ES8311 addr, adc_source, PA pin)
            // by using getConfig() as the base and overriding only what we need.
            cube32::AudioCodecConfig cfg = codec.getConfig();
            cfg.output_sample_rate = LOOPBACK_SAMPLE_RATE_HZ;
            cfg.input_sample_rate  = LOOPBACK_SAMPLE_RATE_HZ;
            cfg.aec_mode           = aec;
            cube32_result_t r = codec.begin(cfg);
            if (r != CUBE32_OK) {
                ESP_LOGE(TAG, "codec.begin() failed: %d", r);
                break;
            }
        }

        // ---- HW AEC fallback (ES8311 integrated board: driver forces to NONE) ----
        // When adc_source == ES8311, the driver silently downgrades HW AEC to NONE
        // because the ES7210 hardware reference loopback is not available.
        if (mode == MODE_HW_AEC && codec.getAecMode() != cube32::AecMode::HW) {
            ESP_LOGW(TAG, "HW AEC unavailable on this board; falling back to No AEC");
            mode           = MODE_NO_AEC;
            aec            = cube32::AecMode::NONE;
            s_current_mode = MODE_NO_AEC;  // sync global so play task reads correct mode
            if (lvgl_port_lock(50)) {
                lv_label_set_text(s_status_label, "Running (No AEC - HW N/A)");
                lv_obj_set_style_text_color(s_status_label, lv_color_hex(0xffaa00), 0);
                lvgl_port_unlock();
            }
        } else {
            const char *mode_name = (mode == MODE_HW_AEC) ? "HW AEC" :
                                    (mode == MODE_SW_AEC) ? "SW AEC" : "No AEC";
            char buf[40];
            snprintf(buf, sizeof(buf), "Running (%s)", mode_name);
            if (lvgl_port_lock(50)) {
                lv_label_set_text(s_status_label, buf);
                lv_obj_set_style_text_color(s_status_label, lv_color_hex(0xff4444), 0);
                lvgl_port_unlock();
            }
        }

        int channels = codec.getInputChannels();  // 4 for ES7210 TDM, 1 for ES8311 ADC
        ESP_LOGI(TAG, "Codec: %d Hz, %d ch, aec=%d", LOOPBACK_SAMPLE_RATE_HZ, channels, (int)aec);

        // ---- Apply initial slider volumes (read from LVGL — need lock) ----
        if (lvgl_port_lock(50)) {
            codec.setOutputVolume(lv_slider_get_value(s_speaker_vol_slider));
            codec.setInputGain(lv_slider_get_value(s_mic_gain_slider));
            lvgl_port_unlock();
        }

        // ---- Create AEC handle ----
        afe_aec_handle_t *hw_aec = nullptr;
        aec_handle_t     *sw_aec = nullptr;
        int aec_chunk = 512;  // default for NO_AEC mode

        if (mode == MODE_HW_AEC) {
            // ES7210 produces four TDM slots. Slot 0 is Mic 1 and slot 1 is
            // the analog speaker-reference loopback; ignore slots 2 and 3.
            hw_aec = afe_aec_create("MRNN", AEC_FILTER_LENGTH,
                                    AFE_TYPE_VC, AFE_MODE_LOW_COST);
            if (!hw_aec) { ESP_LOGE(TAG, "afe_aec_create failed"); break; }
            aec_chunk = afe_aec_get_chunksize(hw_aec);
            ESP_LOGI(TAG, "HW AEC created, chunk=%d samples", aec_chunk);
        } else if (mode == MODE_SW_AEC) {
            sw_aec = aec_create(LOOPBACK_SAMPLE_RATE_HZ, AEC_FILTER_LENGTH, 1, AEC_MODE_VOIP_LOW_COST);
            if (!sw_aec) { ESP_LOGE(TAG, "aec_create failed"); break; }
            aec_chunk = aec_get_chunksize(sw_aec);
            ESP_LOGI(TAG, "SW AEC created, chunk=%d samples", aec_chunk);
        }
        s_aec_chunk = aec_chunk;

        // ---- Create queues ----
        s_loopback_queue = xQueueCreate(LOOPBACK_QUEUE_DEPTH, aec_chunk * sizeof(int16_t));
        if (!s_loopback_queue) { ESP_LOGE(TAG, "loopback queue alloc failed"); break; }

        if (mode == MODE_SW_AEC) {
            s_ref_queue = xQueueCreate(REF_QUEUE_DEPTH, aec_chunk * sizeof(int16_t));
            if (!s_ref_queue) { ESP_LOGE(TAG, "ref queue alloc failed"); break; }
        }

        // ---- Allocate I/O buffers ----
        int read_samples = aec_chunk * channels;
        int16_t *read_buf = (int16_t *)heap_caps_aligned_alloc(
                16, read_samples * sizeof(int16_t), MALLOC_CAP_INTERNAL);
        int16_t *out_buf  = (int16_t *)heap_caps_aligned_alloc(
                16, aec_chunk * sizeof(int16_t), MALLOC_CAP_INTERNAL);
        int16_t *mic_buf  = (int16_t *)heap_caps_aligned_alloc(
            16, aec_chunk * sizeof(int16_t), MALLOC_CAP_INTERNAL);
        int16_t *ref_buf  = nullptr;
        if (mode == MODE_SW_AEC) {
            ref_buf = (int16_t *)heap_caps_aligned_alloc(
                    16, aec_chunk * sizeof(int16_t), MALLOC_CAP_INTERNAL);
        }

        if (!read_buf || !out_buf || !mic_buf || (mode == MODE_SW_AEC && !ref_buf)) {
            ESP_LOGE(TAG, "Buffer alloc failed");
            heap_caps_free(read_buf);
            heap_caps_free(out_buf);
            heap_caps_free(mic_buf);
            heap_caps_free(ref_buf);
            if (hw_aec) afe_aec_destroy(hw_aec);
            if (sw_aec) aec_destroy(sw_aec);
            break;
        }

        // ---- Spawn play task then enable mic ----
        s_evt_group = xEventGroupCreate();
        xTaskCreate(loopback_play_task, "lb_play", 4096,
                    (void *)(intptr_t)aec_chunk, 5, &s_play_task);

        codec.enableInput(true);

        // Pre-fill s_ref_queue with REF_DELAY_FRAMES of silence before the main
        // loop starts.  This phase-aligns the reference signal with the acoustic
        // echo: the I2S TX DMA holds several frames of audio before it reaches
        // the speaker, so the echo the microphone captures is delayed relative
        // to the moment the data was written.  Without this offset the AEC
        // receives a reference that is ahead of the echo it needs to cancel,
        // causing poor convergence.  Each pre-filled silent frame adds one
        // frame-period (~32 ms at 16 kHz / 512 samples) of reference delay.
        if (mode == MODE_SW_AEC && s_ref_queue) {
            int16_t *silence = (int16_t *)heap_caps_calloc(aec_chunk, sizeof(int16_t),
                                                           MALLOC_CAP_INTERNAL);
            if (silence) {
                for (int d = 0; d < REF_DELAY_FRAMES; d++) {
                    xQueueSend(s_ref_queue, silence, 0);
                }
                heap_caps_free(silence);
            }
        }

        // ============================================================
        // Main loopback loop
        // ============================================================
        while (!s_stop_requested) {
            codec.read(read_buf, read_samples);
            if (channels == 4) {
                for (int i = 0; i < aec_chunk; ++i) {
                    mic_buf[i] = read_buf[i * 4];
                }
            } else {
                memcpy(mic_buf, read_buf, aec_chunk * sizeof(int16_t));
            }

            const int16_t *write_ptr = nullptr;

            switch (mode) {
                case MODE_NO_AEC:
                    write_ptr = mic_buf;
                    break;

                case MODE_HW_AEC:
                    // The AEC input format is "MRNN": raw ES7210 TDM slots
                    // [Mic 1, speaker reference, unused, unused].
                    afe_aec_process(hw_aec, read_buf, out_buf);
                    write_ptr = out_buf;
                    break;

                case MODE_SW_AEC: {
                    // Block up to 100 ms for a reference frame from the play task
                    if (xQueueReceive(s_ref_queue, ref_buf, pdMS_TO_TICKS(100)) == pdTRUE) {
                        aec_process(sw_aec, mic_buf, ref_buf, out_buf);
                        write_ptr = out_buf;
                    } else {
                        // No reference available yet (play task starting up or starved)
                        // Pass mic frame directly to avoid stalling the pipeline
                        write_ptr = mic_buf;
                    }
                    break;
                }

                default:
                    write_ptr = mic_buf;
                    break;
            }

            // Push processed frame to play task (50 ms timeout avoids deadlock
            // if the play task exits unexpectedly)
            xQueueSend(s_loopback_queue, write_ptr, pdMS_TO_TICKS(50));
        }
        // ============================================================

        // ---- Cleanup ----
        codec.enableInput(false);

        // Signal play task to stop and wait for it to finish
        // (s_stop_requested is already true; play task checks it each iteration)
        xEventGroupWaitBits(s_evt_group, EVT_PLAY_DONE, pdTRUE, pdTRUE, pdMS_TO_TICKS(2000));
        vEventGroupDelete(s_evt_group);
        s_evt_group = nullptr;

        heap_caps_free(read_buf);
        heap_caps_free(out_buf);
        heap_caps_free(mic_buf);
        heap_caps_free(ref_buf);

        if (hw_aec) afe_aec_destroy(hw_aec);
        if (sw_aec) aec_destroy(sw_aec);

        if (s_loopback_queue) { vQueueDelete(s_loopback_queue); s_loopback_queue = nullptr; }
        if (s_ref_queue)      { vQueueDelete(s_ref_queue);      s_ref_queue      = nullptr; }

        // Restore codec to Kconfig defaults
        codec.end();
        codec.begin();

    } while (0);
#endif  // CONFIG_CUBE32_AUDIO_ENABLED

    set_state(STATE_IDLE);
    if (lvgl_port_lock(100)) { update_ui_state(); lvgl_port_unlock(); }

    s_cap_task = nullptr;
    vTaskDelete(nullptr);
}

// ============================================================================
// Entry Point
// ============================================================================

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "CUBE32 Audio Loopback Test");
    ESP_LOGI(TAG, "========================================");

    esp_err_t ret = cube32_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "cube32_init failed: %s", esp_err_to_name(ret));
        return;
    }

    s_state_mutex = xSemaphoreCreateMutex();

    // Apply initial volume/gain so the codec is in a known state
#ifdef CONFIG_CUBE32_AUDIO_ENABLED
    cube32::AudioCodec::instance().setOutputVolume(INITIAL_SPEAKER_VOLUME);
    cube32::AudioCodec::instance().setInputGain(INITIAL_MIC_GAIN);
#endif

    if (lvgl_port_lock(1000)) {
        create_ui();
        lvgl_port_unlock();
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
