/**
 * @file main.cpp
 * @brief CUBE32 Hello Audio Recorder Example
 * 
 * This example demonstrates audio recording and playback functionality on the CUBE32 board.
 * 
 * Features demonstrated:
 * - Audio recording from microphone to SD card (WAV format)
 * - Audio playback from SD card
 * - LVGL UI for user interaction
 * - Recording duration control
 * 
 * Prerequisites:
 * - Enable Audio in menuconfig: CUBE32 Board Configuration → Audio Configuration → Enable Audio
 * - Enable SD Card in menuconfig: CUBE32 Board Configuration → SD Card Configuration → Enable SD Card
 * - Enable LVGL in menuconfig: CUBE32 Board Configuration → Display Configuration → Enable LVGL
 * - Insert a FAT32 formatted SD card
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
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_heap_caps.h>

#include "cube32.h"

static const char *TAG = "hello_audio_recorder";

// ============================================================================
// Configuration
// ============================================================================

#define MAX_RECORDING_DURATION_SEC   10      // Maximum recording duration in seconds
#define AUDIO_BUFFER_SIZE            4096    // Audio buffer size in samples
#define RECORDING_FILE_NAME          "rec.wav"  // Use 8.3 format for FAT compatibility
#define INITIAL_VOLUME               80      // Initial volume level (0-100)
#define VOLUME_STEP                  5       // Volume step for button control

// WAV file header structure
#pragma pack(push, 1)
typedef struct {
    // RIFF chunk
    char riff_id[4];         // "RIFF"
    uint32_t riff_size;      // File size - 8
    char wave_id[4];         // "WAVE"
    
    // fmt sub-chunk
    char fmt_id[4];          // "fmt "
    uint32_t fmt_size;       // 16 for PCM
    uint16_t audio_format;   // 1 for PCM
    uint16_t num_channels;   // 1 for mono, 2 for stereo
    uint32_t sample_rate;    // Sample rate
    uint32_t byte_rate;      // sample_rate * num_channels * bits_per_sample / 8
    uint16_t block_align;    // num_channels * bits_per_sample / 8
    uint16_t bits_per_sample;// 16
    
    // data sub-chunk
    char data_id[4];         // "data"
    uint32_t data_size;      // Audio data size
} wav_header_t;
#pragma pack(pop)

// ============================================================================
// State
// ============================================================================

typedef enum {
    STATE_IDLE,
    STATE_RECORDING,
    STATE_PLAYING,
} app_state_t;

static app_state_t s_state = STATE_IDLE;
static TaskHandle_t s_audio_task = nullptr;
static SemaphoreHandle_t s_state_mutex = nullptr;
static volatile bool s_stop_requested = false;
static int s_recording_seconds = 0;
static int s_playback_seconds = 0;

// Mute state for ADC button control
static bool s_muted = false;
static int s_volume_before_mute = INITIAL_VOLUME;

// LVGL widgets
static lv_obj_t *s_status_label = nullptr;
static lv_obj_t *s_time_label = nullptr;
static lv_obj_t *s_record_btn = nullptr;
static lv_obj_t *s_stop_btn = nullptr;
static lv_obj_t *s_play_btn = nullptr;
static lv_obj_t *s_progress_bar = nullptr;
static lv_obj_t *s_volume_slider = nullptr;
static lv_obj_t *s_volume_label = nullptr;

// ============================================================================
// Forward Declarations
// ============================================================================

static void create_ui(void);
static void update_ui_state(void);
static void record_btn_cb(lv_event_t *e);
static void stop_btn_cb(lv_event_t *e);
static void play_btn_cb(lv_event_t *e);
static void volume_slider_cb(lv_event_t *e);
static void audio_record_task(void *arg);
static void audio_playback_task(void *arg);
static bool write_wav_header(FILE *file, uint32_t sample_rate, uint16_t channels, uint32_t data_size);
static bool update_wav_header(FILE *file, uint32_t data_size);

// ADC Button handlers
#ifdef CONFIG_CUBE32_ADC_BUTTON_ENABLED
static void setup_adc_buttons(void);
static void volume_up_handler(void);
static void volume_down_handler(void);
static void play_stop_handler(void);
static void record_stop_handler(void);
static void mute_handler(void);
static void set_handler(void);
static void button_long_press_handler(cube32::ADCButtonIndex button);
static void update_volume_ui(int volume);
#endif

// ============================================================================
// State Management
// ============================================================================

static app_state_t get_state(void) {
    app_state_t state;
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    state = s_state;
    xSemaphoreGive(s_state_mutex);
    return state;
}

static void set_state(app_state_t state) {
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    s_state = state;
    xSemaphoreGive(s_state_mutex);
}

// ============================================================================
// UI Creation
// ============================================================================

static void create_ui(void) {
    // Get active screen
    lv_obj_t *screen = lv_scr_act();
    
    // Set dark background
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x1a1a2e), 0);
    
    // Title
    lv_obj_t *title = lv_label_create(screen);
    lv_label_set_text(title, LV_SYMBOL_AUDIO " Audio Demo");
    lv_obj_set_style_text_color(title, lv_color_hex(0xeaeaea), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);
    
    // Status label
    s_status_label = lv_label_create(screen);
    lv_label_set_text(s_status_label, "Ready");
    lv_obj_set_style_text_color(s_status_label, lv_color_hex(0x00ff88), 0);
    lv_obj_set_style_text_font(s_status_label, &lv_font_montserrat_14, 0);
    lv_obj_align(s_status_label, LV_ALIGN_TOP_MID, 0, 40);
    
    // Time label
    s_time_label = lv_label_create(screen);
    lv_label_set_text(s_time_label, "00:00 / 00:10");
    lv_obj_set_style_text_color(s_time_label, lv_color_hex(0xffffff), 0);
    lv_obj_set_style_text_font(s_time_label, &lv_font_montserrat_14, 0);
    lv_obj_align(s_time_label, LV_ALIGN_TOP_MID, 0, 65);
    
    // Progress bar
    s_progress_bar = lv_bar_create(screen);
    lv_obj_set_size(s_progress_bar, 180, 10);
    lv_bar_set_range(s_progress_bar, 0, MAX_RECORDING_DURATION_SEC);
    lv_bar_set_value(s_progress_bar, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(s_progress_bar, lv_color_hex(0x333333), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_progress_bar, lv_color_hex(0x00ff88), LV_PART_INDICATOR);
    lv_obj_align(s_progress_bar, LV_ALIGN_TOP_MID, 0, 95);
    
    // Button container
    lv_obj_t *btn_container = lv_obj_create(screen);
    lv_obj_remove_style_all(btn_container);
    lv_obj_set_size(btn_container, 200, 50);
    lv_obj_align(btn_container, LV_ALIGN_CENTER, 0, -10);
    lv_obj_set_flex_flow(btn_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(btn_container, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    
    // Record button
    s_record_btn = lv_btn_create(btn_container);
    lv_obj_set_size(s_record_btn, 55, 45);
    lv_obj_set_style_bg_color(s_record_btn, lv_color_hex(0xff4444), 0);
    lv_obj_set_style_bg_color(s_record_btn, lv_color_hex(0xff6666), LV_STATE_PRESSED);
    lv_obj_add_event_cb(s_record_btn, record_btn_cb, LV_EVENT_CLICKED, nullptr);
    
    lv_obj_t *record_label = lv_label_create(s_record_btn);
    lv_label_set_text(record_label, LV_SYMBOL_NEW_LINE);
    lv_obj_center(record_label);
    
    // Stop button
    s_stop_btn = lv_btn_create(btn_container);
    lv_obj_set_size(s_stop_btn, 55, 45);
    lv_obj_set_style_bg_color(s_stop_btn, lv_color_hex(0x666666), 0);
    lv_obj_set_style_bg_color(s_stop_btn, lv_color_hex(0x888888), LV_STATE_PRESSED);
    lv_obj_add_event_cb(s_stop_btn, stop_btn_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_state(s_stop_btn, LV_STATE_DISABLED);
    
    lv_obj_t *stop_label = lv_label_create(s_stop_btn);
    lv_label_set_text(stop_label, LV_SYMBOL_STOP);
    lv_obj_center(stop_label);
    
    // Play button
    s_play_btn = lv_btn_create(btn_container);
    lv_obj_set_size(s_play_btn, 55, 45);
    lv_obj_set_style_bg_color(s_play_btn, lv_color_hex(0x44aa44), 0);
    lv_obj_set_style_bg_color(s_play_btn, lv_color_hex(0x66cc66), LV_STATE_PRESSED);
    lv_obj_add_event_cb(s_play_btn, play_btn_cb, LV_EVENT_CLICKED, nullptr);
    
    lv_obj_t *play_label = lv_label_create(s_play_btn);
    lv_label_set_text(play_label, LV_SYMBOL_PLAY);
    lv_obj_center(play_label);
    
    // Volume control section
    s_volume_label = lv_label_create(screen);
    char vol_buf[32];
    snprintf(vol_buf, sizeof(vol_buf), LV_SYMBOL_VOLUME_MAX " Vol: %d%%", INITIAL_VOLUME);
    lv_label_set_text(s_volume_label, vol_buf);
    lv_obj_set_style_text_color(s_volume_label, lv_color_hex(0xaaaaaa), 0);
    lv_obj_align(s_volume_label, LV_ALIGN_BOTTOM_MID, 0, -50);
    
    s_volume_slider = lv_slider_create(screen);
    lv_obj_set_size(s_volume_slider, 160, 10);
    lv_slider_set_range(s_volume_slider, 0, 100);
    lv_slider_set_value(s_volume_slider, INITIAL_VOLUME, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(s_volume_slider, lv_color_hex(0x333333), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_volume_slider, lv_color_hex(0x4488ff), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(s_volume_slider, lv_color_hex(0x6699ff), LV_PART_KNOB);
    lv_obj_align(s_volume_slider, LV_ALIGN_BOTTOM_MID, 0, -25);
    lv_obj_add_event_cb(s_volume_slider, volume_slider_cb, LV_EVENT_VALUE_CHANGED, nullptr);
    
    // SD Card status
    lv_obj_t *sd_status = lv_label_create(screen);
#ifdef CONFIG_CUBE32_SDCARD_ENABLED
    if (cube32::SDCard::instance().isInitialized()) {
        lv_label_set_text(sd_status, LV_SYMBOL_SD_CARD " SD Card: OK");
        lv_obj_set_style_text_color(sd_status, lv_color_hex(0x00ff88), 0);
    } else {
        lv_label_set_text(sd_status, LV_SYMBOL_SD_CARD " SD Card: Not Found");
        lv_obj_set_style_text_color(sd_status, lv_color_hex(0xff4444), 0);
    }
#else
    lv_label_set_text(sd_status, LV_SYMBOL_SD_CARD " SD Card: Disabled");
    lv_obj_set_style_text_color(sd_status, lv_color_hex(0xff4444), 0);
#endif
    lv_obj_set_style_text_font(sd_status, &lv_font_montserrat_14, 0);
    lv_obj_align(sd_status, LV_ALIGN_BOTTOM_MID, 0, -5);
}

static void update_ui_state(void) {
    app_state_t state = get_state();
    
    switch (state) {
        case STATE_IDLE:
            lv_label_set_text(s_status_label, "Ready");
            lv_obj_set_style_text_color(s_status_label, lv_color_hex(0x00ff88), 0);
            lv_obj_clear_state(s_record_btn, LV_STATE_DISABLED);
            lv_obj_add_state(s_stop_btn, LV_STATE_DISABLED);
            lv_obj_clear_state(s_play_btn, LV_STATE_DISABLED);
            break;
            
        case STATE_RECORDING:
            lv_label_set_text(s_status_label, LV_SYMBOL_NEW_LINE " Recording...");
            lv_obj_set_style_text_color(s_status_label, lv_color_hex(0xff4444), 0);
            lv_obj_add_state(s_record_btn, LV_STATE_DISABLED);
            lv_obj_clear_state(s_stop_btn, LV_STATE_DISABLED);
            lv_obj_add_state(s_play_btn, LV_STATE_DISABLED);
            lv_obj_set_style_bg_color(s_progress_bar, lv_color_hex(0xff4444), LV_PART_INDICATOR);
            break;
            
        case STATE_PLAYING:
            lv_label_set_text(s_status_label, LV_SYMBOL_PLAY " Playing...");
            lv_obj_set_style_text_color(s_status_label, lv_color_hex(0x44aa44), 0);
            lv_obj_add_state(s_record_btn, LV_STATE_DISABLED);
            lv_obj_clear_state(s_stop_btn, LV_STATE_DISABLED);
            lv_obj_add_state(s_play_btn, LV_STATE_DISABLED);
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
    
    if (get_state() != STATE_IDLE) {
        return;
    }
    
    ESP_LOGI(TAG, "Starting recording...");
    s_stop_requested = false;
    s_recording_seconds = 0;
    
    // Create recording task
    xTaskCreate(audio_record_task, "audio_record", 8192, nullptr, 5, &s_audio_task);
#endif
}

static void stop_btn_cb(lv_event_t *e) {
    (void)e;
    
    app_state_t state = get_state();
    if (state == STATE_RECORDING || state == STATE_PLAYING) {
        ESP_LOGI(TAG, "Stop requested");
        s_stop_requested = true;
    }
}

static void play_btn_cb(lv_event_t *e) {
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
    
    if (get_state() != STATE_IDLE) {
        return;
    }
    
    // Check if recording file exists
    std::string recording_path = cube32::SDCard::instance().getFullPath(RECORDING_FILE_NAME);
    FILE *test = fopen(recording_path.c_str(), "rb");
    if (test == nullptr) {
        lv_label_set_text(s_status_label, "No recording found!");
        lv_obj_set_style_text_color(s_status_label, lv_color_hex(0xffaa00), 0);
        return;
    }
    fclose(test);
    
    ESP_LOGI(TAG, "Starting playback...");
    s_stop_requested = false;
    s_playback_seconds = 0;
    
    // Create playback task
    xTaskCreate(audio_playback_task, "audio_playback", 8192, nullptr, 5, &s_audio_task);
#endif
}

static void volume_slider_cb(lv_event_t *e) {
    lv_obj_t *slider = (lv_obj_t*)lv_event_get_target(e);
    int value = lv_slider_get_value(slider);
    
    // Update volume label
    char buf[32];
    snprintf(buf, sizeof(buf), LV_SYMBOL_VOLUME_MAX " Vol: %d%%", value);
    lv_label_set_text(s_volume_label, buf);
    
    // Set audio codec volume
#ifdef CONFIG_CUBE32_AUDIO_ENABLED
    cube32::AudioCodec::instance().setOutputVolume(value);
#endif
}

// ============================================================================
// WAV File Helpers
// ============================================================================

static bool write_wav_header(FILE *file, uint32_t sample_rate, uint16_t channels, uint32_t data_size) {
    wav_header_t header;
    
    memcpy(header.riff_id, "RIFF", 4);
    header.riff_size = 36 + data_size;
    memcpy(header.wave_id, "WAVE", 4);
    
    memcpy(header.fmt_id, "fmt ", 4);
    header.fmt_size = 16;
    header.audio_format = 1;  // PCM
    header.num_channels = channels;
    header.sample_rate = sample_rate;
    header.bits_per_sample = 16;
    header.byte_rate = sample_rate * channels * 2;
    header.block_align = channels * 2;
    
    memcpy(header.data_id, "data", 4);
    header.data_size = data_size;
    
    fseek(file, 0, SEEK_SET);
    return fwrite(&header, sizeof(header), 1, file) == 1;
}

static bool update_wav_header(FILE *file, uint32_t data_size) {
    // Update RIFF size
    uint32_t riff_size = 36 + data_size;
    fseek(file, 4, SEEK_SET);
    if (fwrite(&riff_size, 4, 1, file) != 1) return false;
    
    // Update data size
    fseek(file, 40, SEEK_SET);
    return fwrite(&data_size, 4, 1, file) == 1;
}

// ============================================================================
// Audio Tasks
// ============================================================================

static void audio_record_task(void *arg) {
    (void)arg;
    
    set_state(STATE_RECORDING);
    
    // Update UI in LVGL context
    if (lvgl_port_lock(100)) {
        update_ui_state();
        lvgl_port_unlock();
    }
    
#ifdef CONFIG_CUBE32_AUDIO_ENABLED
    do {
        cube32::AudioCodec& codec = cube32::AudioCodec::instance();
        int sample_rate = codec.getInputSampleRate();
        int channels = codec.getInputChannels();
        
        ESP_LOGI(TAG, "Recording at %d Hz, %d channel(s)", sample_rate, channels);
        
        // Build recording file path using SD card driver
        cube32::SDCard& sd = cube32::SDCard::instance();
        if (!sd.isMounted()) {
            ESP_LOGE(TAG, "SD card not mounted");
            break;
        }
        
        std::string recording_path = sd.getFullPath(RECORDING_FILE_NAME);
        ESP_LOGI(TAG, "Recording to: %s", recording_path.c_str());
        
        // Allocate audio buffer
        int16_t *audio_buffer = (int16_t*)heap_caps_malloc(AUDIO_BUFFER_SIZE * sizeof(int16_t), MALLOC_CAP_INTERNAL);
        if (audio_buffer == nullptr) {
            ESP_LOGE(TAG, "Failed to allocate audio buffer");
            break;
        }
        
        // Open file for writing
        FILE *file = fopen(recording_path.c_str(), "wb");
        if (file == nullptr) {
            ESP_LOGE(TAG, "Failed to open file for writing: %s (errno=%d: %s)", 
                     recording_path.c_str(), errno, strerror(errno));
            heap_caps_free(audio_buffer);
            break;
        }
        
        // Write initial WAV header (will update later)
        if (!write_wav_header(file, sample_rate, 1, 0)) {  // Mono output
            ESP_LOGE(TAG, "Failed to write WAV header");
            fclose(file);
            heap_caps_free(audio_buffer);
            break;
        }
        
        // Enable audio input
        codec.enableInput(true);
        
        uint32_t total_samples = 0;
        uint32_t max_samples = sample_rate * MAX_RECORDING_DURATION_SEC * channels;
        int64_t start_time = esp_timer_get_time();
        int last_second = -1;
        
        while (!s_stop_requested && total_samples < max_samples) {
            // Read audio data
            int samples_read = codec.read(audio_buffer, AUDIO_BUFFER_SIZE);
            if (samples_read > 0) {
                // If stereo input (with reference), convert to mono by taking channel 0
                if (channels == 2) {
                    for (int i = 0; i < samples_read / 2; i++) {
                        audio_buffer[i] = audio_buffer[i * 2];
                    }
                    samples_read /= 2;
                }
                
                // Write to file
                fwrite(audio_buffer, sizeof(int16_t), samples_read, file);
                total_samples += samples_read * channels;
            }
            
            // Update progress
            int64_t elapsed_us = esp_timer_get_time() - start_time;
            int elapsed_sec = elapsed_us / 1000000;
            
            if (elapsed_sec != last_second) {
                last_second = elapsed_sec;
                s_recording_seconds = elapsed_sec;
                
                // Update UI
                if (lvgl_port_lock(10)) {
                    char buf[32];
                    snprintf(buf, sizeof(buf), "%02d:%02d / %02d:%02d",
                             elapsed_sec / 60, elapsed_sec % 60,
                             MAX_RECORDING_DURATION_SEC / 60, MAX_RECORDING_DURATION_SEC % 60);
                    lv_label_set_text(s_time_label, buf);
                    lv_bar_set_value(s_progress_bar, elapsed_sec, LV_ANIM_OFF);
                    lvgl_port_unlock();
                }
            }
            
            // Check max duration
            if (elapsed_sec >= MAX_RECORDING_DURATION_SEC) {
                ESP_LOGI(TAG, "Max recording duration reached");
                break;
            }
            
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        
        // Update WAV header with actual data size
        uint32_t data_size = (total_samples / channels) * sizeof(int16_t);
        update_wav_header(file, data_size);
        
        ESP_LOGI(TAG, "Recording complete: %lu samples, %lu bytes", 
                 (unsigned long)total_samples, (unsigned long)data_size);
        
        // Disable audio input
        codec.enableInput(false);
        
        // Cleanup
        fclose(file);
        heap_caps_free(audio_buffer);
    } while (0);
#endif

    set_state(STATE_IDLE);
    
    // Update UI
    if (lvgl_port_lock(100)) {
        update_ui_state();
        lv_bar_set_value(s_progress_bar, 0, LV_ANIM_OFF);
        lv_label_set_text(s_time_label, "00:00 / 00:10");
        lv_obj_set_style_bg_color(s_progress_bar, lv_color_hex(0x00ff88), LV_PART_INDICATOR);
        lvgl_port_unlock();
    }
    
    s_audio_task = nullptr;
    vTaskDelete(nullptr);
}

static void audio_playback_task(void *arg) {
    (void)arg;
    
    set_state(STATE_PLAYING);
    
    // Update UI in LVGL context
    if (lvgl_port_lock(100)) {
        update_ui_state();
        lvgl_port_unlock();
    }
    
#ifdef CONFIG_CUBE32_AUDIO_ENABLED
    do {
        cube32::AudioCodec& codec = cube32::AudioCodec::instance();
        
        // Build recording file path using SD card driver
        cube32::SDCard& sd = cube32::SDCard::instance();
        std::string recording_path = sd.getFullPath(RECORDING_FILE_NAME);
        
        // Open file for reading
        FILE *file = fopen(recording_path.c_str(), "rb");
        if (file == nullptr) {
            ESP_LOGE(TAG, "Failed to open file for reading: %s (errno=%d: %s)",
                     recording_path.c_str(), errno, strerror(errno));
            break;
        }
        
        // Read and validate WAV header
        wav_header_t header;
        if (fread(&header, sizeof(header), 1, file) != 1) {
            ESP_LOGE(TAG, "Failed to read WAV header");
            fclose(file);
            break;
        }
        
        if (memcmp(header.riff_id, "RIFF", 4) != 0 || memcmp(header.wave_id, "WAVE", 4) != 0) {
            ESP_LOGE(TAG, "Invalid WAV file");
            fclose(file);
            break;
        }
        
        ESP_LOGI(TAG, "Playing WAV: %lu Hz, %d ch, %lu bytes",
                 (unsigned long)header.sample_rate, header.num_channels, (unsigned long)header.data_size);

        // Reinit codec at the WAV sample rate to guarantee I2S TX clock matches.
        // Simply calling enableOutput(true) would use the Kconfig output_sample_rate
        // (e.g. 24 kHz) while the WAV is 16 kHz, causing ~1.5x playback speed.
        codec.end();
        {
            cube32::AudioCodecConfig cfg = CUBE32_AUDIO_CONFIG_DEFAULT();
            cfg.output_sample_rate = (int)header.sample_rate;
            cfg.input_sample_rate  = (int)header.sample_rate;
            codec.begin(cfg);
        }
        
        // Allocate audio buffer
        int16_t *audio_buffer = (int16_t*)heap_caps_malloc(AUDIO_BUFFER_SIZE * sizeof(int16_t), MALLOC_CAP_INTERNAL);
        if (audio_buffer == nullptr) {
            ESP_LOGE(TAG, "Failed to allocate audio buffer");
            fclose(file);
            break;
        }
        
        // Enable audio output with the WAV sample rate (I2S was reinit'd to this rate above)
        codec.enableOutput(true, (int)header.sample_rate);
        
        uint32_t total_samples = header.data_size / sizeof(int16_t);
        uint32_t samples_played = 0;
        int duration_sec = total_samples / header.sample_rate;
        int64_t start_time = esp_timer_get_time();
        int last_second = -1;
        
        while (!s_stop_requested && samples_played < total_samples) {
            // Calculate how many samples to read
            int samples_to_read = AUDIO_BUFFER_SIZE;
            if (samples_played + samples_to_read > total_samples) {
                samples_to_read = total_samples - samples_played;
            }
            
            // Read from file
            int samples_read = fread(audio_buffer, sizeof(int16_t), samples_to_read, file);
            if (samples_read <= 0) {
                break;
            }
            
            // Write to audio output
            codec.write(audio_buffer, samples_read);
            samples_played += samples_read;
            
            // Update progress
            int64_t elapsed_us = esp_timer_get_time() - start_time;
            int elapsed_sec = elapsed_us / 1000000;
            
            if (elapsed_sec != last_second) {
                last_second = elapsed_sec;
                s_playback_seconds = elapsed_sec;
                
                // Update UI
                if (lvgl_port_lock(10)) {
                    char buf[32];
                    snprintf(buf, sizeof(buf), "%02d:%02d / %02d:%02d",
                             elapsed_sec / 60, elapsed_sec % 60,
                             duration_sec / 60, duration_sec % 60);
                    lv_label_set_text(s_time_label, buf);
                    
                    int progress = (duration_sec > 0) ? (elapsed_sec * MAX_RECORDING_DURATION_SEC / duration_sec) : 0;
                    if (progress > MAX_RECORDING_DURATION_SEC) progress = MAX_RECORDING_DURATION_SEC;
                    lv_bar_set_value(s_progress_bar, progress, LV_ANIM_OFF);
                    lvgl_port_unlock();
                }
            }
        }
        
        // Disable audio output and restore codec to Kconfig defaults
        codec.enableOutput(false);
        codec.end();
        codec.begin();
        
        ESP_LOGI(TAG, "Playback complete: %lu samples", (unsigned long)samples_played);
        
        // Cleanup
        heap_caps_free(audio_buffer);
        fclose(file);
    } while (0);
#endif

    set_state(STATE_IDLE);
    
    // Update UI
    if (lvgl_port_lock(100)) {
        update_ui_state();
        lv_bar_set_value(s_progress_bar, 0, LV_ANIM_OFF);
        lv_label_set_text(s_time_label, "00:00 / 00:10");
        lv_obj_set_style_bg_color(s_progress_bar, lv_color_hex(0x00ff88), LV_PART_INDICATOR);
        lvgl_port_unlock();
    }
    
    s_audio_task = nullptr;
    vTaskDelete(nullptr);
}

// ============================================================================
// Main
// ============================================================================

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "CUBE32 Hello Audio Recorder Example");
    ESP_LOGI(TAG, "========================================");

    // Initialize CUBE32 board (includes display, touch, audio, SD card, LVGL)
    esp_err_t ret = cube32_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize CUBE32 board: %s", esp_err_to_name(ret));
        return;
    }

#ifndef CONFIG_CUBE32_AUDIO_ENABLED
    ESP_LOGE(TAG, "Audio is not enabled! Enable it in menuconfig:");
    ESP_LOGE(TAG, "  CUBE32 Board Configuration -> Audio Configuration -> Enable Audio");
#endif

#ifndef CONFIG_CUBE32_LVGL_ENABLED
    ESP_LOGE(TAG, "LVGL is not enabled! Enable it in menuconfig:");
    ESP_LOGE(TAG, "  CUBE32 Board Configuration -> Display Configuration -> Enable LVGL");
    return;
#endif

#ifndef CONFIG_CUBE32_SDCARD_ENABLED
    ESP_LOGW(TAG, "SD Card is not enabled! Recording/playback will not work.");
    ESP_LOGW(TAG, "Enable it in menuconfig:");
    ESP_LOGW(TAG, "  CUBE32 Board Configuration -> SD Card Configuration -> Enable SD Card");
#endif

    // Create state mutex
    s_state_mutex = xSemaphoreCreateMutex();
    if (s_state_mutex == nullptr) {
        ESP_LOGE(TAG, "Failed to create state mutex");
        return;
    }

    // Create UI
    ESP_LOGI(TAG, "Creating UI...");
    if (lvgl_port_lock(1000)) {
        create_ui();
        lvgl_port_unlock();
    } else {
        ESP_LOGE(TAG, "Failed to acquire LVGL lock");
        return;
    }

    // Set initial volume
#ifdef CONFIG_CUBE32_AUDIO_ENABLED
    cube32::AudioCodec::instance().setOutputVolume(INITIAL_VOLUME);
#endif

    // Setup ADC button handlers
#ifdef CONFIG_CUBE32_ADC_BUTTON_ENABLED
    setup_adc_buttons();
#endif

    ESP_LOGI(TAG, "Audio demo ready!");
    ESP_LOGI(TAG, "  - Press REC to record (max %d seconds)", MAX_RECORDING_DURATION_SEC);
    ESP_LOGI(TAG, "  - Press STOP to stop recording/playback");
    ESP_LOGI(TAG, "  - Press PLAY to playback last recording");
#ifdef CONFIG_CUBE32_ADC_BUTTON_ENABLED
    ESP_LOGI(TAG, "  - ADC Buttons: Vol+, Vol-, Play/Stop, Record/Stop, Mute, SET");
#endif

    // Main loop - keep running
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ============================================================================
// ADC Button Handlers
// ============================================================================

#ifdef CONFIG_CUBE32_ADC_BUTTON_ENABLED

/**
 * @brief Update volume UI after button press
 */
static void update_volume_ui(int volume) {
    if (lvgl_port_lock(100)) {
        lv_slider_set_value(s_volume_slider, volume, LV_ANIM_ON);
        
        char buf[32];
        if (s_muted) {
            snprintf(buf, sizeof(buf), LV_SYMBOL_MUTE " MUTE");
        } else {
            snprintf(buf, sizeof(buf), LV_SYMBOL_VOLUME_MAX " Vol: %d%%", volume);
        }
        lv_label_set_text(s_volume_label, buf);
        
        lvgl_port_unlock();
    }
}

/**
 * @brief Volume Up button handler
 */
static void volume_up_handler(void) {
    ESP_LOGI(TAG, "ADC Button: Volume Up");
    
    // If muted, unmute first
    if (s_muted) {
        s_muted = false;
    }
    
#ifdef CONFIG_CUBE32_AUDIO_ENABLED
    int current_volume = cube32::AudioCodec::instance().getOutputVolume();
    int new_volume = current_volume + VOLUME_STEP;
    if (new_volume > 100) {
        new_volume = 100;
    }
    cube32::AudioCodec::instance().setOutputVolume(new_volume);
    update_volume_ui(new_volume);
    ESP_LOGI(TAG, "Volume: %d%%", new_volume);
#endif
}

/**
 * @brief Volume Down button handler
 */
static void volume_down_handler(void) {
    ESP_LOGI(TAG, "ADC Button: Volume Down");
    
    // If muted, unmute first
    if (s_muted) {
        s_muted = false;
    }
    
#ifdef CONFIG_CUBE32_AUDIO_ENABLED
    int current_volume = cube32::AudioCodec::instance().getOutputVolume();
    int new_volume = current_volume - VOLUME_STEP;
    if (new_volume < 0) {
        new_volume = 0;
    }
    cube32::AudioCodec::instance().setOutputVolume(new_volume);
    update_volume_ui(new_volume);
    ESP_LOGI(TAG, "Volume: %d%%", new_volume);
#endif
}

/**
 * @brief Play/Stop button handler
 */
static void play_stop_handler(void) {
    ESP_LOGI(TAG, "ADC Button: Play/Stop");
    
    if (get_state() == STATE_PLAYING) {
        // Stop playback
        s_stop_requested = true;
        ESP_LOGI(TAG, "Stopping playback...");
    } else if (get_state() == STATE_IDLE) {
        // Start playback
#ifdef CONFIG_CUBE32_SDCARD_ENABLED
        if (!cube32::SDCard::instance().isInitialized()) {
            ESP_LOGW(TAG, "SD Card not ready");
            return;
        }
        
        // Check if recording file exists
        std::string recording_path = cube32::SDCard::instance().getFullPath(RECORDING_FILE_NAME);
        FILE *test = fopen(recording_path.c_str(), "rb");
        if (test == nullptr) {
            ESP_LOGW(TAG, "No recording found");
            return;
        }
        fclose(test);
        
        ESP_LOGI(TAG, "Starting playback...");
        s_stop_requested = false;
        s_playback_seconds = 0;
        
        // Create playback task
        xTaskCreate(audio_playback_task, "audio_playback", 8192, nullptr, 5, &s_audio_task);
#endif
    }
}

/**
 * @brief Record/Stop button handler
 */
static void record_stop_handler(void) {
    ESP_LOGI(TAG, "ADC Button: Record/Stop");
    
    if (get_state() == STATE_RECORDING) {
        // Stop recording
        s_stop_requested = true;
        ESP_LOGI(TAG, "Stopping recording...");
    } else if (get_state() == STATE_IDLE) {
        // Start recording
#ifdef CONFIG_CUBE32_SDCARD_ENABLED
        if (!cube32::SDCard::instance().isInitialized()) {
            ESP_LOGW(TAG, "SD Card not ready");
            return;
        }
        
        ESP_LOGI(TAG, "Starting recording...");
        s_stop_requested = false;
        s_recording_seconds = 0;
        
        // Create recording task
        xTaskCreate(audio_record_task, "audio_record", 8192, nullptr, 5, &s_audio_task);
#endif
    } else if (get_state() == STATE_PLAYING) {
        // Stop playback if we want to record
        s_stop_requested = true;
        ESP_LOGI(TAG, "Stopping playback first...");
    }
}

/**
 * @brief Mute button handler
 */
static void mute_handler(void) {
    ESP_LOGI(TAG, "ADC Button: Mute");
    
#ifdef CONFIG_CUBE32_AUDIO_ENABLED
    if (s_muted) {
        // Unmute - restore previous volume
        s_muted = false;
        cube32::AudioCodec::instance().setOutputVolume(s_volume_before_mute);
        update_volume_ui(s_volume_before_mute);
        ESP_LOGI(TAG, "Unmuted, volume: %d%%", s_volume_before_mute);
    } else {
        // Mute - save current volume and set to 0
        s_muted = true;
        s_volume_before_mute = cube32::AudioCodec::instance().getOutputVolume();
        cube32::AudioCodec::instance().setOutputVolume(0);
        update_volume_ui(0);
        ESP_LOGI(TAG, "Muted");
    }
#endif
}

/**
 * @brief SET button handler (Button 2) - dummy for testing
 */
static void set_handler(void) {
    ESP_LOGI(TAG, "ADC Button: SET");
}

/**
 * @brief Long press handler for all buttons
 */
static void button_long_press_handler(cube32::ADCButtonIndex button) {
    const char* button_names[] = {
        "Volume Up",
        "Volume Down",
        "SET",
        "Play/Stop",
        "Mute",
        "Record/Stop"
    };
    
    int btn_idx = static_cast<int>(button);
    if (btn_idx >= 0 && btn_idx < 6) {
        ESP_LOGI(TAG, "ADC Button Long Press: %s (Button %d)", button_names[btn_idx], btn_idx);
    }
}

/**
 * @brief Setup ADC button callbacks
 * 
 * Button mapping:
 * - Button 0 (0.38V, 1.3K): Volume Up (onClick, onLongPress)
 * - Button 1 (0.82V, 3.3K): Volume Down (onClick, onLongPress)
 * - Button 2 (1.11V, 5.1K): SET (onClick, onLongPress) - dummy for testing
 * - Button 3 (1.65V, 10K):  Play/Stop (onClick, onLongPress)
 * - Button 4 (1.98V, 15K):  Mute (onClick, onLongPress)
 * - Button 5 (2.41V, 27K):  Record/Stop (onClick, onLongPress)
 */
static void setup_adc_buttons(void) {
    ESP_LOGI(TAG, "Setting up ADC button callbacks...");
    
    cube32::ADCButton& adc_btn = cube32::ADCButton::instance();
    
    if (!adc_btn.isInitialized()) {
        ESP_LOGW(TAG, "ADC button driver not initialized");
        return;
    }
    
    // Button 0 (0.38V): Volume Up - onClick
    adc_btn.registerCallback(cube32::ADCButtonIndex::BUTTON_0, BUTTON_SINGLE_CLICK,
        [](cube32::ADCButtonIndex btn, button_event_t event) {
            (void)btn;
            (void)event;
            volume_up_handler();
        });
    
    // Button 1 (0.82V): Volume Down - onClick
    adc_btn.registerCallback(cube32::ADCButtonIndex::BUTTON_1, BUTTON_SINGLE_CLICK,
        [](cube32::ADCButtonIndex btn, button_event_t event) {
            (void)btn;
            (void)event;
            volume_down_handler();
        });
    
    // Button 2 (1.11V): SET - onClick (dummy for testing)
    adc_btn.registerCallback(cube32::ADCButtonIndex::BUTTON_2, BUTTON_SINGLE_CLICK,
        [](cube32::ADCButtonIndex btn, button_event_t event) {
            (void)btn;
            (void)event;
            set_handler();
        });
    
    // Button 3 (1.65V): Play/Stop - onClick
    adc_btn.registerCallback(cube32::ADCButtonIndex::BUTTON_3, BUTTON_SINGLE_CLICK,
        [](cube32::ADCButtonIndex btn, button_event_t event) {
            (void)btn;
            (void)event;
            play_stop_handler();
        });
    
    // Button 4 (1.98V): Mute - onClick
    adc_btn.registerCallback(cube32::ADCButtonIndex::BUTTON_4, BUTTON_SINGLE_CLICK,
        [](cube32::ADCButtonIndex btn, button_event_t event) {
            (void)btn;
            (void)event;
            mute_handler();
        });
    
    // Button 5 (2.41V): Record/Stop - onClick
    adc_btn.registerCallback(cube32::ADCButtonIndex::BUTTON_5, BUTTON_SINGLE_CLICK,
        [](cube32::ADCButtonIndex btn, button_event_t event) {
            (void)btn;
            (void)event;
            record_stop_handler();
        });
    
    // Register long press handlers for all buttons
    for (int i = 0; i < 6; i++) {
        cube32::ADCButtonIndex btn_idx = static_cast<cube32::ADCButtonIndex>(i);
        adc_btn.registerCallback(btn_idx, BUTTON_LONG_PRESS_START,
            [](cube32::ADCButtonIndex btn, button_event_t event) {
                (void)event;
                button_long_press_handler(btn);
            });
    }
    
    ESP_LOGI(TAG, "ADC button callbacks registered:");
    ESP_LOGI(TAG, "  Button 0 (0.38V): Volume Up (onClick, onLongPress)");
    ESP_LOGI(TAG, "  Button 1 (0.82V): Volume Down (onClick, onLongPress)");
    ESP_LOGI(TAG, "  Button 2 (1.11V): SET (onClick, onLongPress)");
    ESP_LOGI(TAG, "  Button 3 (1.65V): Play/Stop (onClick, onLongPress)");
    ESP_LOGI(TAG, "  Button 4 (1.98V): Mute (onClick, onLongPress)");
    ESP_LOGI(TAG, "  Button 5 (2.41V): Record/Stop (onClick, onLongPress)");
}

#endif // CONFIG_CUBE32_ADC_BUTTON_ENABLED
