/**
 * @file main.cpp
 * @brief CUBE32 Hello AEC Test Example
 *
 * Acoustic Echo Cancellation (AEC) comparison demo.
 * Plays a 1 kHz test tone through the speaker while recording the microphone
 * in three modes (No AEC, HW AEC, SW AEC) and saves separate WAV files to SD
 * card for A/B comparison.
 *
 * Modes:
 *   No AEC  – raw mic recording; echo fully present.
 *   HW AEC  – hardware reference loopback (ES7210 ch1) + esp-sr afe_aec.
 *   SW AEC  – digital copy of playback buffer as reference + esp-sr aec.
 *
 * Prerequisites:
 *   - Enable Audio + Input Reference in menuconfig
 *   - Enable LVGL + SD Card
 *   - Insert FAT32-formatted SD card
 *   - AEC operates at 16 kHz only
 */

#include <cstdio>
#include <cinttypes>
#include <cstring>
#include <cmath>
#include <cerrno>
#include <string>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_heap_caps.h>

#include "cube32.h"

// esp-sr AEC headers
#include "esp_aec.h"
#include "esp_afe_aec.h"

static const char *TAG = "hello_aec_test";

// ============================================================================
// Configuration
// ============================================================================

#define AEC_SAMPLE_RATE_HZ       16000   // AEC requires 16 kHz
#define TEST_DURATION_SEC        5       // Recording/playback duration
#define TONE_FREQ_HZ             1000    // Test tone frequency
#define TONE_AMPLITUDE           22000   // ~0.67 * INT16_MAX
#define AEC_FILTER_LENGTH        4       // Recommended for ESP32-S3
#define INITIAL_VOLUME           80
#define REF_QUEUE_DEPTH          8       // SW AEC reference queue depth

// File names (8.3 FAT)
static const char *FILE_NONE = "aec_none.wav";
static const char *FILE_HW   = "aec_hw.wav";
static const char *FILE_SW   = "aec_sw.wav";

// ============================================================================
// WAV header
// ============================================================================

#pragma pack(push, 1)
typedef struct {
    char     riff_id[4];
    uint32_t riff_size;
    char     wave_id[4];
    char     fmt_id[4];
    uint32_t fmt_size;
    uint16_t audio_format;
    uint16_t num_channels;
    uint32_t sample_rate;
    uint32_t byte_rate;
    uint16_t block_align;
    uint16_t bits_per_sample;
    char     data_id[4];
    uint32_t data_size;
} wav_header_t;
#pragma pack(pop)

// ============================================================================
// Types
// ============================================================================

typedef enum {
    MODE_NO_AEC = 0,
    MODE_HW_AEC = 1,
    MODE_SW_AEC = 2,
    MODE_COUNT  = 3,
} aec_test_mode_t;

typedef enum {
    STATE_IDLE,
    STATE_TESTING,
    STATE_PLAYING,
} app_state_t;

static const char *mode_names[MODE_COUNT] = { "No AEC", "HW AEC", "SW AEC" };
static const char *mode_files[MODE_COUNT] = { nullptr, nullptr, nullptr };

// Event-group bits for task synchronisation
#define EVT_PLAY_DONE  (1 << 0)
#define EVT_REC_DONE   (1 << 1)

// ============================================================================
// State
// ============================================================================

static app_state_t          s_state          = STATE_IDLE;
static SemaphoreHandle_t    s_state_mutex    = nullptr;
static volatile bool        s_stop_requested = false;
static aec_test_mode_t      s_current_mode   = MODE_NO_AEC;
static EventGroupHandle_t   s_evt_group      = nullptr;
static QueueHandle_t        s_ref_queue      = nullptr;  // SW AEC ref frames
static TaskHandle_t         s_rec_task       = nullptr;
static TaskHandle_t         s_play_task      = nullptr;
static int                  s_elapsed_sec    = 0;
static int                  s_aec_chunk      = 512;   // shared with tone task
static volatile bool        s_tone_enabled   = true;  // play tone during test?

// LVGL widgets
static lv_obj_t *s_status_label   = nullptr;
static lv_obj_t *s_time_label     = nullptr;
static lv_obj_t *s_mode_dropdown  = nullptr;
static lv_obj_t *s_record_btn     = nullptr;
static lv_obj_t *s_stop_btn       = nullptr;
static lv_obj_t *s_play_btns[MODE_COUNT]   = {};
static lv_obj_t *s_play_labels[MODE_COUNT] = {};
static lv_obj_t *s_progress_bar   = nullptr;
static lv_obj_t *s_volume_slider  = nullptr;
static lv_obj_t *s_volume_label   = nullptr;
static lv_obj_t *s_tone_checkbox  = nullptr;

// ============================================================================
// Forward Declarations
// ============================================================================

static void create_ui(void);
static void update_ui_state(void);
static void record_btn_cb(lv_event_t *e);
static void stop_btn_cb(lv_event_t *e);
static void play_btn_cb(lv_event_t *e);
static void volume_slider_cb(lv_event_t *e);
static void tone_play_task(void *arg);
static void audio_record_task(void *arg);
static void wav_playback_task(void *arg);
static bool write_wav_header(FILE *f, uint32_t sr, uint16_t ch, uint32_t data_sz);
static bool update_wav_header(FILE *f, uint32_t data_sz);

// ============================================================================
// Helpers
// ============================================================================

static app_state_t get_state(void) {
    app_state_t st;
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    st = s_state;
    xSemaphoreGive(s_state_mutex);
    return st;
}

static void set_state(app_state_t st) {
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    s_state = st;
    xSemaphoreGive(s_state_mutex);
}

static const char *file_for_mode(aec_test_mode_t m) {
    switch (m) {
        case MODE_NO_AEC: return FILE_NONE;
        case MODE_HW_AEC: return FILE_HW;
        case MODE_SW_AEC: return FILE_SW;
        default: return FILE_NONE;
    }
}

/** Generate one frame of 1 kHz sine tone. */
static void generate_tone_frame(int16_t *buf, int samples, int *phase) {
    for (int i = 0; i < samples; i++) {
        buf[i] = (int16_t)(TONE_AMPLITUDE *
                  sinf(2.0f * M_PI * TONE_FREQ_HZ * (*phase) / (float)AEC_SAMPLE_RATE_HZ));
        (*phase)++;
        if (*phase >= AEC_SAMPLE_RATE_HZ) *phase -= AEC_SAMPLE_RATE_HZ;
    }
}

// ============================================================================
// WAV File Helpers
// ============================================================================

static bool write_wav_header(FILE *f, uint32_t sr, uint16_t ch, uint32_t data_sz) {
    wav_header_t h;
    memcpy(h.riff_id, "RIFF", 4);
    h.riff_size      = 36 + data_sz;
    memcpy(h.wave_id, "WAVE", 4);
    memcpy(h.fmt_id, "fmt ", 4);
    h.fmt_size        = 16;
    h.audio_format    = 1;
    h.num_channels    = ch;
    h.sample_rate     = sr;
    h.bits_per_sample = 16;
    h.byte_rate       = sr * ch * 2;
    h.block_align     = ch * 2;
    memcpy(h.data_id, "data", 4);
    h.data_size       = data_sz;
    fseek(f, 0, SEEK_SET);
    return fwrite(&h, sizeof(h), 1, f) == 1;
}

static bool update_wav_header(FILE *f, uint32_t data_sz) {
    uint32_t riff_sz = 36 + data_sz;
    fseek(f, 4, SEEK_SET);
    if (fwrite(&riff_sz, 4, 1, f) != 1) return false;
    fseek(f, 40, SEEK_SET);
    return fwrite(&data_sz, 4, 1, f) == 1;
}

// ============================================================================
// UI Creation
// ============================================================================

static void create_ui(void) {
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x1a1a2e), 0);

    // Title
    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, LV_SYMBOL_AUDIO " AEC Test");
    lv_obj_set_style_text_color(title, lv_color_hex(0xeaeaea), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 4);

    // Mode dropdown
    s_mode_dropdown = lv_dropdown_create(scr);
    lv_dropdown_set_options(s_mode_dropdown, "No AEC\nHW AEC\nSW AEC");
    lv_obj_set_size(s_mode_dropdown, 130, 30);
    lv_obj_set_style_text_font(s_mode_dropdown, &lv_font_montserrat_14, 0);
    lv_obj_set_style_bg_color(s_mode_dropdown, lv_color_hex(0x2d2d4e), 0);
    lv_obj_set_style_text_color(s_mode_dropdown, lv_color_hex(0xeaeaea), 0);
    lv_obj_set_style_border_color(s_mode_dropdown, lv_color_hex(0x4488ff), 0);
    lv_obj_align(s_mode_dropdown, LV_ALIGN_TOP_MID, -35, 24);

    // Tone checkbox
    s_tone_checkbox = lv_checkbox_create(scr);
    lv_checkbox_set_text(s_tone_checkbox, "Tone");
    lv_obj_add_state(s_tone_checkbox, LV_STATE_CHECKED);
    lv_obj_set_style_text_color(s_tone_checkbox, lv_color_hex(0xeaeaea), 0);
    lv_obj_set_style_text_font(s_tone_checkbox, &lv_font_montserrat_12, 0);
    lv_obj_align(s_tone_checkbox, LV_ALIGN_TOP_MID, 85, 30);

    // Status label
    s_status_label = lv_label_create(scr);
    lv_label_set_text(s_status_label, "Ready");
    lv_obj_set_style_text_color(s_status_label, lv_color_hex(0x00ff88), 0);
    lv_obj_set_style_text_font(s_status_label, &lv_font_montserrat_14, 0);
    lv_obj_align(s_status_label, LV_ALIGN_TOP_MID, 0, 60);

    // Time label
    s_time_label = lv_label_create(scr);
    lv_label_set_text(s_time_label, "00:00 / 00:05");
    lv_obj_set_style_text_color(s_time_label, lv_color_hex(0xffffff), 0);
    lv_obj_set_style_text_font(s_time_label, &lv_font_montserrat_14, 0);
    lv_obj_align(s_time_label, LV_ALIGN_TOP_MID, 0, 80);

    // Progress bar
    s_progress_bar = lv_bar_create(scr);
    lv_obj_set_size(s_progress_bar, 180, 8);
    lv_bar_set_range(s_progress_bar, 0, TEST_DURATION_SEC);
    lv_bar_set_value(s_progress_bar, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(s_progress_bar, lv_color_hex(0x333333), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_progress_bar, lv_color_hex(0x00ff88), LV_PART_INDICATOR);
    lv_obj_align(s_progress_bar, LV_ALIGN_TOP_MID, 0, 100);

    // Record / Stop row
    lv_obj_t *ctrl_row = lv_obj_create(scr);
    lv_obj_remove_style_all(ctrl_row);
    lv_obj_set_size(ctrl_row, 200, 40);
    lv_obj_align(ctrl_row, LV_ALIGN_TOP_MID, 0, 112);
    lv_obj_set_flex_flow(ctrl_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ctrl_row, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    // Record button (red)
    s_record_btn = lv_btn_create(ctrl_row);
    lv_obj_set_size(s_record_btn, 80, 35);
    lv_obj_set_style_bg_color(s_record_btn, lv_color_hex(0xff4444), 0);
    lv_obj_add_event_cb(s_record_btn, record_btn_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t *rec_lbl = lv_label_create(s_record_btn);
    lv_label_set_text(rec_lbl, LV_SYMBOL_NEW_LINE " Test");
    lv_obj_center(rec_lbl);

    // Stop button (grey)
    s_stop_btn = lv_btn_create(ctrl_row);
    lv_obj_set_size(s_stop_btn, 80, 35);
    lv_obj_set_style_bg_color(s_stop_btn, lv_color_hex(0x666666), 0);
    lv_obj_add_event_cb(s_stop_btn, stop_btn_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_state(s_stop_btn, LV_STATE_DISABLED);
    lv_obj_t *stop_lbl = lv_label_create(s_stop_btn);
    lv_label_set_text(stop_lbl, LV_SYMBOL_STOP " Stop");
    lv_obj_center(stop_lbl);

    // Play-back row — one button per mode
    lv_obj_t *play_row = lv_obj_create(scr);
    lv_obj_remove_style_all(play_row);
    lv_obj_set_size(play_row, 220, 35);
    lv_obj_align(play_row, LV_ALIGN_TOP_MID, 0, 155);
    lv_obj_set_flex_flow(play_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(play_row, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    const char *play_labels[MODE_COUNT] = { "None", "HW", "SW" };
    for (int i = 0; i < MODE_COUNT; i++) {
        s_play_btns[i] = lv_btn_create(play_row);
        lv_obj_set_size(s_play_btns[i], 65, 30);
        lv_obj_set_style_bg_color(s_play_btns[i], lv_color_hex(0x44aa44), 0);
        lv_obj_add_event_cb(s_play_btns[i], play_btn_cb, LV_EVENT_CLICKED,
                            reinterpret_cast<void *>(static_cast<intptr_t>(i)));
        lv_obj_add_state(s_play_btns[i], LV_STATE_DISABLED);

        s_play_labels[i] = lv_label_create(s_play_btns[i]);
        char buf[16];
        snprintf(buf, sizeof(buf), LV_SYMBOL_PLAY " %s", play_labels[i]);
        lv_label_set_text(s_play_labels[i], buf);
        lv_obj_center(s_play_labels[i]);
    }

    // Volume
    s_volume_label = lv_label_create(scr);
    char vbuf[32];
    snprintf(vbuf, sizeof(vbuf), LV_SYMBOL_VOLUME_MAX " Vol: %d%%", INITIAL_VOLUME);
    lv_label_set_text(s_volume_label, vbuf);
    lv_obj_set_style_text_color(s_volume_label, lv_color_hex(0xaaaaaa), 0);
    lv_obj_align(s_volume_label, LV_ALIGN_BOTTOM_MID, 0, -35);

    s_volume_slider = lv_slider_create(scr);
    lv_obj_set_size(s_volume_slider, 160, 8);
    lv_slider_set_range(s_volume_slider, 0, 100);
    lv_slider_set_value(s_volume_slider, INITIAL_VOLUME, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(s_volume_slider, lv_color_hex(0x333333), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_volume_slider, lv_color_hex(0x4488ff), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(s_volume_slider, lv_color_hex(0x6699ff), LV_PART_KNOB);
    lv_obj_align(s_volume_slider, LV_ALIGN_BOTTOM_MID, 0, -18);
    lv_obj_add_event_cb(s_volume_slider, volume_slider_cb, LV_EVENT_VALUE_CHANGED, nullptr);

    // SD status
    lv_obj_t *sd_lbl = lv_label_create(scr);
#ifdef CONFIG_CUBE32_SDCARD_ENABLED
    if (cube32::SDCard::instance().isInitialized()) {
        lv_label_set_text(sd_lbl, LV_SYMBOL_SD_CARD " SD: OK");
        lv_obj_set_style_text_color(sd_lbl, lv_color_hex(0x00ff88), 0);
    } else {
        lv_label_set_text(sd_lbl, LV_SYMBOL_SD_CARD " SD: N/A");
        lv_obj_set_style_text_color(sd_lbl, lv_color_hex(0xff4444), 0);
    }
#else
    lv_label_set_text(sd_lbl, LV_SYMBOL_SD_CARD " SD: Off");
    lv_obj_set_style_text_color(sd_lbl, lv_color_hex(0xff4444), 0);
#endif
    lv_obj_set_style_text_font(sd_lbl, &lv_font_montserrat_14, 0);
    lv_obj_align(sd_lbl, LV_ALIGN_BOTTOM_MID, 0, -2);
}

// ============================================================================
// UI State
// ============================================================================

/** Refresh play-button enabled state based on file existence. */
static void refresh_play_buttons(void) {
#ifdef CONFIG_CUBE32_SDCARD_ENABLED
    cube32::SDCard& sd = cube32::SDCard::instance();
    if (!sd.isInitialized()) return;
    const char *fnames[MODE_COUNT] = { FILE_NONE, FILE_HW, FILE_SW };
    for (int i = 0; i < MODE_COUNT; i++) {
        std::string path = sd.getFullPath(fnames[i]);
        FILE *f = fopen(path.c_str(), "rb");
        if (f) {
            fclose(f);
            lv_obj_clear_state(s_play_btns[i], LV_STATE_DISABLED);
        } else {
            lv_obj_add_state(s_play_btns[i], LV_STATE_DISABLED);
        }
    }
#endif
}

static void update_ui_state(void) {
    app_state_t st = get_state();
    switch (st) {
        case STATE_IDLE:
            lv_label_set_text(s_status_label, "Ready");
            lv_obj_set_style_text_color(s_status_label, lv_color_hex(0x00ff88), 0);
            lv_obj_clear_state(s_record_btn, LV_STATE_DISABLED);
            lv_obj_add_state(s_stop_btn, LV_STATE_DISABLED);
            lv_obj_clear_state(s_mode_dropdown, LV_STATE_DISABLED);
            lv_obj_clear_state(s_tone_checkbox, LV_STATE_DISABLED);
            refresh_play_buttons();
            lv_obj_set_style_bg_color(s_progress_bar, lv_color_hex(0x00ff88), LV_PART_INDICATOR);
            break;
        case STATE_TESTING: {
            char buf[48];
            snprintf(buf, sizeof(buf), LV_SYMBOL_NEW_LINE " Testing (%s)...", mode_names[s_current_mode]);
            lv_label_set_text(s_status_label, buf);
            lv_obj_set_style_text_color(s_status_label, lv_color_hex(0xff4444), 0);
            lv_obj_add_state(s_record_btn, LV_STATE_DISABLED);
            lv_obj_clear_state(s_stop_btn, LV_STATE_DISABLED);
            lv_obj_add_state(s_mode_dropdown, LV_STATE_DISABLED);
            lv_obj_add_state(s_tone_checkbox, LV_STATE_DISABLED);
            for (int i = 0; i < MODE_COUNT; i++)
                lv_obj_add_state(s_play_btns[i], LV_STATE_DISABLED);
            lv_obj_set_style_bg_color(s_progress_bar, lv_color_hex(0xff4444), LV_PART_INDICATOR);
            break;
        }
        case STATE_PLAYING:
            lv_label_set_text(s_status_label, LV_SYMBOL_PLAY " Playing...");
            lv_obj_set_style_text_color(s_status_label, lv_color_hex(0x44aa44), 0);
            lv_obj_add_state(s_record_btn, LV_STATE_DISABLED);
            lv_obj_clear_state(s_stop_btn, LV_STATE_DISABLED);
            lv_obj_add_state(s_mode_dropdown, LV_STATE_DISABLED);
            lv_obj_add_state(s_tone_checkbox, LV_STATE_DISABLED);
            for (int i = 0; i < MODE_COUNT; i++)
                lv_obj_add_state(s_play_btns[i], LV_STATE_DISABLED);
            lv_obj_set_style_bg_color(s_progress_bar, lv_color_hex(0x44aa44), LV_PART_INDICATOR);
            break;
    }
}

// ============================================================================
// Button Callbacks
// ============================================================================

static void record_btn_cb(lv_event_t *e) {
    (void)e;
#ifndef CONFIG_CUBE32_SDCARD_ENABLED
    lv_label_set_text(s_status_label, "SD Card disabled!");
    lv_obj_set_style_text_color(s_status_label, lv_color_hex(0xff4444), 0);
    return;
#else
    if (!cube32::SDCard::instance().isInitialized()) {
        lv_label_set_text(s_status_label, "Insert SD Card!");
        lv_obj_set_style_text_color(s_status_label, lv_color_hex(0xff4444), 0);
        return;
    }
    if (get_state() != STATE_IDLE) return;

    s_current_mode = (aec_test_mode_t)lv_dropdown_get_selected(s_mode_dropdown);
    s_tone_enabled = lv_obj_has_state(s_tone_checkbox, LV_STATE_CHECKED);
    s_stop_requested = false;
    s_elapsed_sec = 0;

    ESP_LOGI(TAG, "Starting AEC test: mode=%s", mode_names[s_current_mode]);

    // Create record + tone tasks
    xTaskCreate(audio_record_task, "aec_rec", 8192, nullptr, 6, &s_rec_task);
#endif
}

static void stop_btn_cb(lv_event_t *e) {
    (void)e;
    if (get_state() != STATE_IDLE) {
        ESP_LOGI(TAG, "Stop requested");
        s_stop_requested = true;
    }
}

static void play_btn_cb(lv_event_t *e) {
#ifndef CONFIG_CUBE32_SDCARD_ENABLED
    return;
#else
    if (get_state() != STATE_IDLE) return;

    int idx = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));
    if (idx < 0 || idx >= MODE_COUNT) return;

    const char *fname = file_for_mode((aec_test_mode_t)idx);
    std::string path = cube32::SDCard::instance().getFullPath(fname);
    FILE *f = fopen(path.c_str(), "rb");
    if (!f) {
        if (lvgl_port_lock(50)) {
            lv_label_set_text(s_status_label, "File not found!");
            lv_obj_set_style_text_color(s_status_label, lv_color_hex(0xffaa00), 0);
            lvgl_port_unlock();
        }
        return;
    }
    fclose(f);

    s_stop_requested = false;
    s_elapsed_sec = 0;
    s_current_mode = (aec_test_mode_t)idx;

    ESP_LOGI(TAG, "Playing %s", fname);
    xTaskCreate(wav_playback_task, "aec_play", 8192,
                reinterpret_cast<void *>(static_cast<intptr_t>(idx)), 5, &s_play_task);
#endif
}

static void volume_slider_cb(lv_event_t *e) {
    int val = lv_slider_get_value((lv_obj_t *)lv_event_get_target(e));
    char buf[32];
    snprintf(buf, sizeof(buf), LV_SYMBOL_VOLUME_MAX " Vol: %d%%", val);
    lv_label_set_text(s_volume_label, buf);
#ifdef CONFIG_CUBE32_AUDIO_ENABLED
    cube32::AudioCodec::instance().setOutputVolume(val);
#endif
}

// ============================================================================
// Tone Playback Task (runs during test)
// ============================================================================

static void tone_play_task(void *arg) {
    aec_test_mode_t mode = (aec_test_mode_t)(intptr_t)arg;
    cube32::AudioCodec &codec = cube32::AudioCodec::instance();

    // Frame size: match the AEC chunk size set by record task
    const int frame_samples = s_aec_chunk;
    int16_t *tone_buf = (int16_t *)heap_caps_malloc(frame_samples * sizeof(int16_t), MALLOC_CAP_INTERNAL);
    if (!tone_buf) {
        ESP_LOGE(TAG, "tone_buf alloc failed");
        xEventGroupSetBits(s_evt_group, EVT_PLAY_DONE);
        vTaskDelete(nullptr);
        return;
    }

    codec.enableOutput(true, AEC_SAMPLE_RATE_HZ);

    int phase = 0;
    int total_frames = (AEC_SAMPLE_RATE_HZ * TEST_DURATION_SEC) / frame_samples;

    for (int i = 0; i < total_frames && !s_stop_requested; i++) {
        if (s_tone_enabled) {
            generate_tone_frame(tone_buf, frame_samples, &phase);
        } else {
            memset(tone_buf, 0, frame_samples * sizeof(int16_t));
        }
        codec.write(tone_buf, frame_samples);

        // For SW AEC: push a copy of the playback frame to the ref queue
        if (mode == MODE_SW_AEC && s_ref_queue) {
            // If queue full, overwrite oldest (non-blocking send)
            if (xQueueSend(s_ref_queue, tone_buf, 0) != pdTRUE) {
                // Drop oldest and retry
                int16_t dummy[frame_samples];
                xQueueReceive(s_ref_queue, dummy, 0);
                xQueueSend(s_ref_queue, tone_buf, 0);
            }
        }
    }

    codec.enableOutput(false);
    heap_caps_free(tone_buf);

    xEventGroupSetBits(s_evt_group, EVT_PLAY_DONE);
    vTaskDelete(nullptr);
}

// ============================================================================
// Audio Record + AEC Task
// ============================================================================

static void audio_record_task(void *arg) {
    (void)arg;

    set_state(STATE_TESTING);
    if (lvgl_port_lock(100)) { update_ui_state(); lvgl_port_unlock(); }

#ifdef CONFIG_CUBE32_AUDIO_ENABLED
    do {
        cube32::AudioCodec &codec = cube32::AudioCodec::instance();
        aec_test_mode_t mode = s_current_mode;
        const char *fname = file_for_mode(mode);

        // ---- Reinit codec at 16 kHz with appropriate AEC mode ----
        cube32::AecMode aec = (mode == MODE_HW_AEC) ? cube32::AecMode::HW :
                              (mode == MODE_SW_AEC) ? cube32::AecMode::SW :
                                                      cube32::AecMode::NONE;
        codec.end();
        {
            cube32::AudioCodecConfig cfg = CUBE32_AUDIO_CONFIG_DEFAULT();
            cfg.output_sample_rate = AEC_SAMPLE_RATE_HZ;
            cfg.aec_mode           = aec;
            // input_sample_rate is auto-clamped to 16 kHz by driver when AEC enabled
            cube32_result_t r = codec.begin(cfg);
            if (r != CUBE32_OK) {
                ESP_LOGE(TAG, "codec.begin() failed: %d", r);
                break;
            }
        }

        int channels = codec.getInputChannels();  // 2 for HW AEC, 1 otherwise
        ESP_LOGI(TAG, "Codec reinit: %d Hz, %d ch, aec=%d",
                 AEC_SAMPLE_RATE_HZ, channels, (int)aec);

        // ---- AEC handle creation ----
        afe_aec_handle_t *hw_aec = nullptr;
        aec_handle_t     *sw_aec = nullptr;
        int aec_chunk = 512;  // default
        s_aec_chunk = 512;    // reset shared value

        if (mode == MODE_HW_AEC) {
            hw_aec = afe_aec_create("MR", AEC_FILTER_LENGTH, AFE_TYPE_VC, AFE_MODE_LOW_COST);
            if (!hw_aec) { ESP_LOGE(TAG, "afe_aec_create failed"); break; }
            aec_chunk = afe_aec_get_chunksize(hw_aec);
            ESP_LOGI(TAG, "HW AEC created, chunk=%d samples", aec_chunk);
        } else if (mode == MODE_SW_AEC) {
            sw_aec = aec_create(AEC_SAMPLE_RATE_HZ, AEC_FILTER_LENGTH, 1, AEC_MODE_VOIP_LOW_COST);
            if (!sw_aec) { ESP_LOGE(TAG, "aec_create failed"); break; }
            aec_chunk = aec_get_chunksize(sw_aec);
            ESP_LOGI(TAG, "SW AEC created, chunk=%d samples", aec_chunk);
            s_aec_chunk = aec_chunk;  // share with tone task
            // Create reference queue — each item is aec_chunk * sizeof(int16_t)
            s_ref_queue = xQueueCreate(REF_QUEUE_DEPTH, aec_chunk * sizeof(int16_t));
            if (!s_ref_queue) { ESP_LOGE(TAG, "ref queue alloc failed"); break; }
        }

        // ---- Buffers ----
        int read_samples = aec_chunk * channels;  // interleaved for HW AEC
        int16_t *read_buf = (int16_t *)heap_caps_aligned_alloc(
                16, read_samples * sizeof(int16_t), MALLOC_CAP_INTERNAL);
        int16_t *out_buf  = (int16_t *)heap_caps_aligned_alloc(
                16, aec_chunk * sizeof(int16_t), MALLOC_CAP_INTERNAL);
        int16_t *ref_buf  = nullptr;
        if (mode == MODE_SW_AEC) {
            ref_buf = (int16_t *)heap_caps_aligned_alloc(
                    16, aec_chunk * sizeof(int16_t), MALLOC_CAP_INTERNAL);
        }

        if (!read_buf || !out_buf || (mode == MODE_SW_AEC && !ref_buf)) {
            ESP_LOGE(TAG, "Buffer alloc failed");
            heap_caps_free(read_buf); heap_caps_free(out_buf); heap_caps_free(ref_buf);
            if (hw_aec) afe_aec_destroy(hw_aec);
            if (sw_aec) aec_destroy(sw_aec);
            break;
        }

        // ---- Open WAV ----
        cube32::SDCard &sd = cube32::SDCard::instance();
        std::string path = sd.getFullPath(fname);
        FILE *wav = fopen(path.c_str(), "wb");
        if (!wav) {
            ESP_LOGE(TAG, "fopen(%s) failed: %s", path.c_str(), strerror(errno));
            heap_caps_free(read_buf); heap_caps_free(out_buf); heap_caps_free(ref_buf);
            if (hw_aec) afe_aec_destroy(hw_aec);
            if (sw_aec) aec_destroy(sw_aec);
            break;
        }
        write_wav_header(wav, AEC_SAMPLE_RATE_HZ, 1, 0);

        // ---- Start tone playback task ----
        s_evt_group = xEventGroupCreate();
        xTaskCreate(tone_play_task, "aec_tone", 4096,
                    reinterpret_cast<void *>(static_cast<intptr_t>(mode)), 5, &s_play_task);

        // ---- Enable mic ----
        codec.enableInput(true);

        uint32_t data_bytes = 0;
        int64_t t0 = esp_timer_get_time();
        int last_sec = -1;
        int max_frames = (AEC_SAMPLE_RATE_HZ * TEST_DURATION_SEC) / aec_chunk;

        for (int fr = 0; fr < max_frames && !s_stop_requested; fr++) {
            // Read mic (+ hw ref if HW AEC)
            codec.read(read_buf, read_samples);

            const int16_t *write_ptr = nullptr;
            int write_samples = aec_chunk;

            switch (mode) {
                case MODE_NO_AEC:
                    write_ptr = read_buf;
                    break;

                case MODE_HW_AEC: {
                    // read_buf is interleaved [mic0, ref0, mic1, ref1, ...]
                    // afe_aec_process accepts interleaved "MR" directly
                    afe_aec_process(hw_aec, read_buf, out_buf);
                    write_ptr = out_buf;
                    break;
                }

                case MODE_SW_AEC: {
                    // Get reference frame from queue (block up to 100 ms)
                    if (xQueueReceive(s_ref_queue, ref_buf, pdMS_TO_TICKS(100)) == pdTRUE) {
                        aec_process(sw_aec, read_buf, ref_buf, out_buf);
                        write_ptr = out_buf;
                    } else {
                        // No ref available — write raw mic
                        write_ptr = read_buf;
                    }
                    break;
                }

                default:
                    write_ptr = read_buf;
                    break;
            }

            fwrite(write_ptr, sizeof(int16_t), write_samples, wav);
            data_bytes += write_samples * sizeof(int16_t);

            // UI update every second
            int sec = (int)((esp_timer_get_time() - t0) / 1000000);
            if (sec != last_sec) {
                last_sec = sec;
                s_elapsed_sec = sec;
                if (lvgl_port_lock(10)) {
                    char buf[32];
                    snprintf(buf, sizeof(buf), "%02d:%02d / %02d:%02d",
                             sec / 60, sec % 60, TEST_DURATION_SEC / 60, TEST_DURATION_SEC % 60);
                    lv_label_set_text(s_time_label, buf);
                    lv_bar_set_value(s_progress_bar, sec, LV_ANIM_OFF);
                    lvgl_port_unlock();
                }
            }
        }

        // ---- Cleanup ----
        codec.enableInput(false);
        s_stop_requested = true;  // signal tone task to stop too

        // Wait for tone task to finish
        xEventGroupWaitBits(s_evt_group, EVT_PLAY_DONE, pdTRUE, pdTRUE, pdMS_TO_TICKS(3000));
        vEventGroupDelete(s_evt_group);
        s_evt_group = nullptr;

        codec.enableOutput(false);

        update_wav_header(wav, data_bytes);
        fclose(wav);

        ESP_LOGI(TAG, "Saved %s (%lu bytes)", fname, (unsigned long)data_bytes);

        heap_caps_free(read_buf);
        heap_caps_free(out_buf);
        heap_caps_free(ref_buf);

        if (hw_aec) afe_aec_destroy(hw_aec);
        if (sw_aec) aec_destroy(sw_aec);
        if (s_ref_queue) { vQueueDelete(s_ref_queue); s_ref_queue = nullptr; }

        // Restore codec to Kconfig defaults
        codec.end();
        codec.begin();

    } while (0);
#endif

    set_state(STATE_IDLE);
    if (lvgl_port_lock(100)) {
        update_ui_state();
        lv_bar_set_value(s_progress_bar, 0, LV_ANIM_OFF);
        lv_label_set_text(s_time_label, "00:00 / 00:05");
        lvgl_port_unlock();
    }

    s_rec_task = nullptr;
    vTaskDelete(nullptr);
}

// ============================================================================
// WAV Playback Task (play recorded result)
// ============================================================================

static void wav_playback_task(void *arg) {
    int idx = static_cast<int>(reinterpret_cast<intptr_t>(arg));
    set_state(STATE_PLAYING);
    if (lvgl_port_lock(100)) { update_ui_state(); lvgl_port_unlock(); }

#ifdef CONFIG_CUBE32_AUDIO_ENABLED
    do {
        cube32::AudioCodec &codec = cube32::AudioCodec::instance();
        cube32::SDCard &sd = cube32::SDCard::instance();

        const char *fname = file_for_mode((aec_test_mode_t)idx);
        std::string path = sd.getFullPath(fname);

        FILE *f = fopen(path.c_str(), "rb");
        if (!f) { ESP_LOGE(TAG, "Cannot open %s", path.c_str()); break; }

        wav_header_t hdr;
        if (fread(&hdr, sizeof(hdr), 1, f) != 1 ||
            memcmp(hdr.riff_id, "RIFF", 4) != 0 ||
            memcmp(hdr.wave_id, "WAVE", 4) != 0) {
            ESP_LOGE(TAG, "Invalid WAV"); fclose(f); break;
        }

        ESP_LOGI(TAG, "Playing %s: %lu Hz, %d ch, %lu bytes",
                 fname, (unsigned long)hdr.sample_rate, hdr.num_channels, (unsigned long)hdr.data_size);

        // Reinit codec at the WAV sample rate to guarantee I2S clock matches
        codec.end();
        {
            cube32::AudioCodecConfig cfg = CUBE32_AUDIO_CONFIG_DEFAULT();
            cfg.output_sample_rate = hdr.sample_rate;
            cfg.input_sample_rate  = hdr.sample_rate;
            codec.begin(cfg);
        }
        codec.enableOutput(true, hdr.sample_rate);

        const int buf_samples = 2048;
        int16_t *buf = (int16_t *)heap_caps_malloc(buf_samples * sizeof(int16_t), MALLOC_CAP_INTERNAL);
        if (!buf) { ESP_LOGE(TAG, "alloc failed"); codec.enableOutput(false); fclose(f); break; }

        uint32_t total = hdr.data_size / sizeof(int16_t);
        uint32_t played = 0;
        int dur = (total > 0 && hdr.sample_rate > 0) ? total / hdr.sample_rate : 0;
        int64_t t0 = esp_timer_get_time();
        int last_sec = -1;

        if (lvgl_port_lock(10)) {
            lv_bar_set_range(s_progress_bar, 0, dur > 0 ? dur : 1);
            lvgl_port_unlock();
        }

        while (!s_stop_requested && played < total) {
            int to_read = buf_samples;
            if (played + to_read > total) to_read = total - played;
            int got = fread(buf, sizeof(int16_t), to_read, f);
            if (got <= 0) break;
            codec.write(buf, got);
            played += got;

            int sec = (int)((esp_timer_get_time() - t0) / 1000000);
            if (sec != last_sec) {
                last_sec = sec;
                if (lvgl_port_lock(10)) {
                    char tbuf[32];
                    snprintf(tbuf, sizeof(tbuf), "%02d:%02d / %02d:%02d",
                             sec / 60, sec % 60, dur / 60, dur % 60);
                    lv_label_set_text(s_time_label, tbuf);
                    lv_bar_set_value(s_progress_bar, sec, LV_ANIM_OFF);
                    lvgl_port_unlock();
                }
            }
        }

        codec.enableOutput(false);
        heap_caps_free(buf);
        fclose(f);

        // Restore codec to Kconfig defaults
        codec.end();
        codec.begin();
    } while (0);
#endif

    set_state(STATE_IDLE);
    if (lvgl_port_lock(100)) {
        update_ui_state();
        lv_bar_set_range(s_progress_bar, 0, TEST_DURATION_SEC);
        lv_bar_set_value(s_progress_bar, 0, LV_ANIM_OFF);
        lv_label_set_text(s_time_label, "00:00 / 00:05");
        lvgl_port_unlock();
    }

    s_play_task = nullptr;
    vTaskDelete(nullptr);
}

// ============================================================================
// Entry Point
// ============================================================================

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "CUBE32 AEC Test Demo");

    esp_err_t ret = cube32_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "cube32_init failed: %s", esp_err_to_name(ret));
        return;
    }

    s_state_mutex = xSemaphoreCreateMutex();

    // Initialise file-name table
    mode_files[MODE_NO_AEC] = FILE_NONE;
    mode_files[MODE_HW_AEC] = FILE_HW;
    mode_files[MODE_SW_AEC] = FILE_SW;

    if (lvgl_port_lock(1000)) {
        create_ui();
        lvgl_port_unlock();
    }

#ifdef CONFIG_CUBE32_AUDIO_ENABLED
    cube32::AudioCodec::instance().setOutputVolume(INITIAL_VOLUME);
#endif

    // Idle loop
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
