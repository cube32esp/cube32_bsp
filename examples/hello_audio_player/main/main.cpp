/**
 * @file main.cpp
 * @brief CUBE32 Hello Audio Player Example
 * 
 * This example demonstrates an audio player with LVGL UI that plays WAV and MP3
 * files from an SD card on the CUBE32 board.
 * 
 * Features demonstrated:
 * - LVGL 9.x UI with file browser
 * - SD card directory listing for audio files
 * - WAV and MP3 file header parsing for sample rate detection
 * - MP3 decoding using libhelix-mp3
 * - Audio playback with dynamic sample rate adjustment
 * - Volume control with on-screen slider and ADC buttons
 * - Display of sample rate and audio duration
 * - ADC button array control (Volume Up/Down, Play/Stop, Mute)
 * 
 * Prerequisites:
 * - Enable Audio in menuconfig: CUBE32 Board Configuration → Audio Configuration → Enable Audio
 * - Enable ADC Button in menuconfig: CUBE32 Board Configuration → Audio Configuration → Enable ADC Button Array
 * - Enable SD Card in menuconfig: CUBE32 Board Configuration → SD Card Configuration → Enable SD Card
 * - Enable LVGL in menuconfig: CUBE32 Board Configuration → Display Configuration → Enable LVGL
 * - Insert a FAT32 formatted SD card with WAV files in the root folder
 */

#include <cstdio>
#include <cinttypes>
#include <cstring>
#include <cmath>
#include <cerrno>
#include <vector>
#include <string>
#include <algorithm>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_heap_caps.h>
#include <driver/uart.h>
#include <driver/uart_vfs.h>

#include "cube32.h"

// MP3 decoder (libhelix-mp3)
#include "mp3dec.h"

static const char *TAG = "hello_audio_player";

// ============================================================================
// Configuration
// ============================================================================

#define AUDIO_BUFFER_SIZE            4096    // Audio buffer size in samples
#define INITIAL_VOLUME               80      // Initial volume level (0-100)
#define MAX_FILES_DISPLAY            20      // Maximum files to display in list
#define FILE_NAME_MAX_LEN            64      // Maximum filename length to display
#define MP3_READ_BUFFER_SIZE         (MAINBUF_SIZE * 3)  // MP3 read buffer size
#define MP3_DECODE_BUFFER_SIZE       (MAX_NCHAN * MAX_NGRAN * MAX_NSAMP)  // MP3 decode output buffer
#define VOLUME_STEP                  5       // Volume step for button control

// ============================================================================
// WAV File Header Structure
// ============================================================================

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

// WAV file info structure (parsed from header)
typedef struct {
    bool valid;
    uint32_t sample_rate;
    uint16_t num_channels;
    uint16_t bits_per_sample;
    uint32_t data_size;
    uint32_t duration_sec;
    uint32_t data_offset;    // Offset to audio data
} wav_info_t;

// ============================================================================
// MP3 File Structures
// ============================================================================

// MP3 ID3v1 tag (at end of file)
#pragma pack(push, 1)
typedef struct {
    char tag[3];         // "TAG"
    char title[30];
    char artist[30];
    char album[30];
    char year[4];
    char comment[30];
    uint8_t genre;
} mp3_id3v1_t;

// MP3 ID3v2 header (at start of file)
typedef struct {
    char id[3];          // "ID3"
    uint8_t version[2];  // Version
    uint8_t flags;
    uint8_t size[4];     // Syncsafe integer
} mp3_id3v2_header_t;
#pragma pack(pop)

// MP3 file info structure
typedef struct {
    bool valid;
    uint32_t sample_rate;
    uint16_t num_channels;
    uint16_t bits_per_sample;
    uint32_t duration_sec;
    uint32_t file_size;
    uint32_t audio_data_start;  // Offset past ID3v2 tag
} mp3_info_t;

// Audio file type
typedef enum {
    AUDIO_TYPE_UNKNOWN,
    AUDIO_TYPE_WAV,
    AUDIO_TYPE_MP3,
} audio_file_type_t;

// ============================================================================
// State
// ============================================================================

typedef enum {
    STATE_IDLE,
    STATE_PLAYING,
} app_state_t;

static app_state_t s_state = STATE_IDLE;
static TaskHandle_t s_audio_task = nullptr;
static SemaphoreHandle_t s_state_mutex = nullptr;
static volatile bool s_stop_requested = false;
static int s_playback_seconds = 0;
static int s_playback_duration = 0;
static int s_playback_sample_rate = 0;

// Mute state for ADC button control
static bool s_muted = false;
static int s_volume_before_mute = INITIAL_VOLUME;

// File list
static std::vector<std::string> s_audio_files;
static int s_selected_file_index = -1;
static std::string s_selected_file;

// LVGL widgets
static lv_obj_t *s_title_label = nullptr;
static lv_obj_t *s_status_label = nullptr;
static lv_obj_t *s_info_label = nullptr;
static lv_obj_t *s_time_label = nullptr;
static lv_obj_t *s_file_list = nullptr;
static lv_obj_t *s_play_btn = nullptr;
static lv_obj_t *s_stop_btn = nullptr;
static lv_obj_t *s_progress_bar = nullptr;
static lv_obj_t *s_volume_slider = nullptr;
static lv_obj_t *s_volume_label = nullptr;

// ============================================================================
// Forward Declarations
// ============================================================================

static void create_ui(void);
static void update_ui_state(void);
static void update_file_list(void);
static void file_list_event_cb(lv_event_t *e);
static void play_btn_cb(lv_event_t *e);
static void stop_btn_cb(lv_event_t *e);
static void volume_slider_cb(lv_event_t *e);
static void audio_playback_task(void *arg);
static wav_info_t parse_wav_header(const char* filepath);
static mp3_info_t parse_mp3_header(const char* filepath);
static audio_file_type_t get_audio_file_type(const char* filename);
static bool scan_audio_files(void);
static std::string format_time(int seconds);
static std::string format_sample_rate(uint32_t rate);

// ADC Button handlers
#ifdef CONFIG_CUBE32_ADC_BUTTON_ENABLED
static void setup_adc_buttons(void);
static void volume_up_handler(void);
static void volume_down_handler(void);
static void play_stop_handler(void);
static void mute_handler(void);
static void set_handler(void);
static void rec_handler(void);
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
// Utility Functions
// ============================================================================

static std::string format_time(int seconds) {
    char buf[16];
    int mins = seconds / 60;
    int secs = seconds % 60;
    snprintf(buf, sizeof(buf), "%02d:%02d", mins, secs);
    return std::string(buf);
}

static std::string format_sample_rate(uint32_t rate) {
    char buf[16];
    if (rate >= 1000) {
        snprintf(buf, sizeof(buf), "%.1f kHz", rate / 1000.0f);
    } else {
        snprintf(buf, sizeof(buf), "%lu Hz", (unsigned long)rate);
    }
    return std::string(buf);
}

static bool is_wav_file(const char* filename) {
    size_t len = strlen(filename);
    if (len < 4) return false;
    const char* ext = filename + len - 4;
    return (strcasecmp(ext, ".wav") == 0);
}

static bool is_mp3_file(const char* filename) {
    size_t len = strlen(filename);
    if (len < 4) return false;
    const char* ext = filename + len - 4;
    return (strcasecmp(ext, ".mp3") == 0);
}

static audio_file_type_t get_audio_file_type(const char* filename) {
    if (is_wav_file(filename)) return AUDIO_TYPE_WAV;
    if (is_mp3_file(filename)) return AUDIO_TYPE_MP3;
    return AUDIO_TYPE_UNKNOWN;
}

// ============================================================================
// WAV Header Parsing
// ============================================================================

/**
 * @brief Parse WAV file header to extract audio information
 * 
 * This function reads and parses the WAV file header to determine:
 * - Sample rate
 * - Number of channels
 * - Bits per sample
 * - Audio data size
 * - Duration
 * 
 * It handles both standard 44-byte headers and extended headers with
 * additional chunks before the data chunk.
 */
static wav_info_t parse_wav_header(const char* filepath) {
    wav_info_t info = {};
    info.valid = false;
    
#ifdef CONFIG_CUBE32_SDCARD_ENABLED
    cube32::SDCard& sd = cube32::SDCard::instance();
    std::string full_path = sd.getFullPath(filepath);
    
    FILE *file = fopen(full_path.c_str(), "rb");
    if (file == nullptr) {
        ESP_LOGE(TAG, "Failed to open file: %s", full_path.c_str());
        return info;
    }
    
    // Read RIFF header
    char riff_header[12];
    if (fread(riff_header, 1, 12, file) != 12) {
        ESP_LOGE(TAG, "Failed to read RIFF header");
        fclose(file);
        return info;
    }
    
    // Verify RIFF/WAVE signature
    if (memcmp(riff_header, "RIFF", 4) != 0 || memcmp(riff_header + 8, "WAVE", 4) != 0) {
        ESP_LOGE(TAG, "Invalid WAV file: not RIFF/WAVE format");
        fclose(file);
        return info;
    }
    
    // Parse chunks to find fmt and data
    bool found_fmt = false;
    bool found_data = false;
    
    while (!found_fmt || !found_data) {
        char chunk_id[4];
        uint32_t chunk_size;
        
        if (fread(chunk_id, 1, 4, file) != 4) {
            break;
        }
        if (fread(&chunk_size, 4, 1, file) != 1) {
            break;
        }
        
        if (memcmp(chunk_id, "fmt ", 4) == 0) {
            // Read fmt chunk
            uint16_t audio_format;
            fread(&audio_format, 2, 1, file);
            fread(&info.num_channels, 2, 1, file);
            fread(&info.sample_rate, 4, 1, file);
            
            uint32_t byte_rate;
            fread(&byte_rate, 4, 1, file);
            
            uint16_t block_align;
            fread(&block_align, 2, 1, file);
            fread(&info.bits_per_sample, 2, 1, file);
            
            // Skip any extra fmt data
            if (chunk_size > 16) {
                fseek(file, chunk_size - 16, SEEK_CUR);
            }
            
            if (audio_format != 1) {
                ESP_LOGW(TAG, "Non-PCM audio format: %d (only PCM supported)", audio_format);
            }
            
            found_fmt = true;
            ESP_LOGI(TAG, "WAV fmt: %lu Hz, %d ch, %d bits", 
                     (unsigned long)info.sample_rate, info.num_channels, info.bits_per_sample);
        }
        else if (memcmp(chunk_id, "data", 4) == 0) {
            info.data_size = chunk_size;
            info.data_offset = ftell(file);
            found_data = true;
            ESP_LOGI(TAG, "WAV data: %lu bytes at offset %lu", 
                     (unsigned long)info.data_size, (unsigned long)info.data_offset);
        }
        else {
            // Skip unknown chunk
            fseek(file, chunk_size, SEEK_CUR);
            // Align to word boundary
            if (chunk_size & 1) {
                fseek(file, 1, SEEK_CUR);
            }
        }
    }
    
    fclose(file);
    
    if (found_fmt && found_data) {
        info.valid = true;
        
        // Calculate duration
        if (info.sample_rate > 0 && info.num_channels > 0 && info.bits_per_sample > 0) {
            uint32_t bytes_per_sample = info.num_channels * (info.bits_per_sample / 8);
            uint32_t total_samples = info.data_size / bytes_per_sample;
            info.duration_sec = total_samples / info.sample_rate;
        }
        
        ESP_LOGI(TAG, "WAV duration: %lu seconds", (unsigned long)info.duration_sec);
    } else {
        ESP_LOGE(TAG, "Invalid WAV file: missing %s%s chunk", 
                 found_fmt ? "" : "fmt ", found_data ? "" : "data ");
    }
#endif
    
    return info;
}

// ============================================================================
// MP3 Header Parsing
// ============================================================================

// MP3 bitrate table (MPEG1 Layer 3)
static const int mp3_bitrate_table[16] = {
    0, 32, 40, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320, 0
};

// MP3 sample rate table (MPEG1)
static const int mp3_samplerate_table[4] = {
    44100, 48000, 32000, 0
};

/**
 * @brief Parse syncsafe integer from ID3v2 tag
 */
static uint32_t parse_syncsafe_int(const uint8_t* data) {
    return ((uint32_t)data[0] << 21) | 
           ((uint32_t)data[1] << 14) | 
           ((uint32_t)data[2] << 7) | 
           (uint32_t)data[3];
}

/**
 * @brief Parse MP3 file header to extract audio information
 * 
 * This function scans the MP3 file to determine:
 * - Sample rate
 * - Number of channels
 * - Approximate duration (based on file size and bitrate)
 * 
 * It handles ID3v2 tags at the start of the file.
 */
static mp3_info_t parse_mp3_header(const char* filepath) {
    mp3_info_t info = {};
    info.valid = false;
    info.bits_per_sample = 16;  // MP3 always decodes to 16-bit
    
#ifdef CONFIG_CUBE32_SDCARD_ENABLED
    cube32::SDCard& sd = cube32::SDCard::instance();
    std::string full_path = sd.getFullPath(filepath);
    
    FILE *file = fopen(full_path.c_str(), "rb");
    if (file == nullptr) {
        ESP_LOGE(TAG, "Failed to open MP3 file: %s", full_path.c_str());
        return info;
    }
    
    // Get file size
    fseek(file, 0, SEEK_END);
    info.file_size = ftell(file);
    fseek(file, 0, SEEK_SET);
    
    // Check for ID3v2 tag
    uint8_t header[10];
    if (fread(header, 1, 10, file) != 10) {
        fclose(file);
        return info;
    }
    
    info.audio_data_start = 0;
    
    // Check for ID3v2 tag
    if (memcmp(header, "ID3", 3) == 0) {
        uint32_t tag_size = parse_syncsafe_int(&header[6]);
        info.audio_data_start = 10 + tag_size;
        ESP_LOGI(TAG, "MP3 ID3v2 tag found, size: %lu bytes", (unsigned long)tag_size);
    }
    
    // Seek to audio data start
    fseek(file, info.audio_data_start, SEEK_SET);
    
    // Find first valid MP3 frame header
    uint8_t buf[4];
    bool found_frame = false;
    int search_limit = 8192;  // Limit search to first 8KB
    
    while (search_limit > 0 && fread(buf, 1, 4, file) == 4) {
        // Check for MP3 sync word (0xFF followed by 0xFB, 0xFA, 0xF3, 0xF2)
        if (buf[0] == 0xFF && (buf[1] & 0xE0) == 0xE0) {
            // Decode frame header
            int version = (buf[1] >> 3) & 0x03;
            int layer = (buf[1] >> 1) & 0x03;
            int bitrate_idx = (buf[2] >> 4) & 0x0F;
            int samplerate_idx = (buf[2] >> 2) & 0x03;
            int channel_mode = (buf[3] >> 6) & 0x03;
            
            // Validate: MPEG1 (version=3), Layer 3 (layer=1)
            if (version == 3 && layer == 1 && 
                bitrate_idx > 0 && bitrate_idx < 15 &&
                samplerate_idx < 3) {
                
                int bitrate = mp3_bitrate_table[bitrate_idx] * 1000;
                info.sample_rate = mp3_samplerate_table[samplerate_idx];
                info.num_channels = (channel_mode == 3) ? 1 : 2;  // 3 = mono
                
                // Estimate duration from file size and bitrate
                uint32_t audio_size = info.file_size - info.audio_data_start;
                info.duration_sec = (audio_size * 8) / bitrate;
                
                info.valid = true;
                found_frame = true;
                
                ESP_LOGI(TAG, "MP3 frame: %lu Hz, %d ch, %d kbps, ~%lu sec",
                         (unsigned long)info.sample_rate, info.num_channels,
                         bitrate / 1000, (unsigned long)info.duration_sec);
                break;
            }
        }
        
        // Move back 3 bytes to continue search
        fseek(file, -3, SEEK_CUR);
        search_limit--;
    }
    
    if (!found_frame) {
        ESP_LOGW(TAG, "No valid MP3 frame found in first 8KB");
        
        // Try using libhelix decoder to get info
        HMP3Decoder decoder = MP3InitDecoder();
        if (decoder) {
            // Allocate buffer for frame detection
            uint8_t *mp3_buf = (uint8_t*)heap_caps_malloc(MP3_READ_BUFFER_SIZE, MALLOC_CAP_INTERNAL);
            if (mp3_buf) {
                fseek(file, info.audio_data_start, SEEK_SET);
                int bytes_read = fread(mp3_buf, 1, MP3_READ_BUFFER_SIZE, file);
                
                if (bytes_read > 0) {
                    int offset = MP3FindSyncWord(mp3_buf, bytes_read);
                    if (offset >= 0) {
                        uint8_t *ptr = mp3_buf + offset;
                        int bytes_left = bytes_read - offset;
                        
                        int16_t *decode_buf = (int16_t*)heap_caps_malloc(
                            MP3_DECODE_BUFFER_SIZE * sizeof(int16_t), MALLOC_CAP_INTERNAL);
                        
                        if (decode_buf) {
                            int err = MP3Decode(decoder, &ptr, &bytes_left, decode_buf, 0);
                            if (err == ERR_MP3_NONE) {
                                MP3FrameInfo frame_info;
                                MP3GetLastFrameInfo(decoder, &frame_info);
                                
                                info.sample_rate = frame_info.samprate;
                                info.num_channels = frame_info.nChans;
                                
                                // Estimate duration
                                uint32_t audio_size = info.file_size - info.audio_data_start;
                                if (frame_info.bitrate > 0) {
                                    info.duration_sec = (audio_size * 8) / frame_info.bitrate;
                                }
                                
                                info.valid = true;
                                ESP_LOGI(TAG, "MP3 via decoder: %lu Hz, %d ch, %d bps, ~%lu sec",
                                         (unsigned long)info.sample_rate, info.num_channels,
                                         frame_info.bitrate, (unsigned long)info.duration_sec);
                            }
                            heap_caps_free(decode_buf);
                        }
                    }
                }
                heap_caps_free(mp3_buf);
            }
            MP3FreeDecoder(decoder);
        }
    }
    
    fclose(file);
#endif
    
    return info;
}

// ============================================================================
// File Scanning
// ============================================================================

/**
 * @brief Scan SD card root folder for WAV and MP3 audio files
 */
static bool scan_audio_files(void) {
    s_audio_files.clear();
    
#ifdef CONFIG_CUBE32_SDCARD_ENABLED
    cube32::SDCard& sd = cube32::SDCard::instance();
    
    if (!sd.isInitialized()) {
        ESP_LOGW(TAG, "SD card not initialized");
        return false;
    }
    
    // Get directory listing
    std::vector<cube32_sdcard_entry_t> entries;
    cube32_result_t ret = sd.listDirectory("/", entries, MAX_FILES_DISPLAY * 2);
    
    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to list directory");
        return false;
    }
    
    // Filter for audio files (WAV and MP3)
    for (const auto& entry : entries) {
        if (!entry.is_directory && 
            (is_wav_file(entry.name) || is_mp3_file(entry.name))) {
            s_audio_files.push_back(entry.name);
            ESP_LOGI(TAG, "Found audio file: %s", entry.name);
            
            if (s_audio_files.size() >= MAX_FILES_DISPLAY) {
                break;
            }
        }
    }
    
    // Sort alphabetically
    std::sort(s_audio_files.begin(), s_audio_files.end());
    
    ESP_LOGI(TAG, "Found %d audio files", (int)s_audio_files.size());
    return !s_audio_files.empty();
#else
    return false;
#endif
}

// ============================================================================
// UI Creation
// ============================================================================

static void create_ui(void) {
    // Get active screen
    lv_obj_t *screen = lv_scr_act();
    
    // Set dark background
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x1a1a2e), 0);
    
    // Get screen dimensions
    int16_t scr_width = lv_obj_get_width(screen);
    int16_t scr_height = lv_obj_get_height(screen);
    
    // Title
    s_title_label = lv_label_create(screen);
    lv_label_set_text(s_title_label, LV_SYMBOL_AUDIO " Audio Player");
    lv_obj_set_style_text_color(s_title_label, lv_color_hex(0x00d4ff), 0);
    lv_obj_set_style_text_font(s_title_label, &lv_font_montserrat_14, 0);
    lv_obj_align(s_title_label, LV_ALIGN_TOP_MID, 0, 5);
    
    // Status label
    s_status_label = lv_label_create(screen);
    lv_label_set_text(s_status_label, "Select a file to play");
    lv_obj_set_style_text_color(s_status_label, lv_color_hex(0xaaaaaa), 0);
    lv_obj_set_style_text_font(s_status_label, &lv_font_montserrat_14, 0);
    lv_obj_align(s_status_label, LV_ALIGN_TOP_MID, 0, 25);
    
    // Info label (sample rate and duration)
    s_info_label = lv_label_create(screen);
    lv_label_set_text(s_info_label, "");
    lv_obj_set_style_text_color(s_info_label, lv_color_hex(0x888888), 0);
    lv_obj_set_style_text_font(s_info_label, &lv_font_montserrat_14, 0);
    lv_obj_align(s_info_label, LV_ALIGN_TOP_MID, 0, 42);
    
    // File list container
    lv_obj_t *list_container = lv_obj_create(screen);
    lv_obj_set_size(list_container, scr_width - 20, scr_height - 170);
    lv_obj_align(list_container, LV_ALIGN_TOP_MID, 0, 58);
    lv_obj_set_style_bg_color(list_container, lv_color_hex(0x16213e), 0);
    lv_obj_set_style_border_color(list_container, lv_color_hex(0x0f3460), 0);
    lv_obj_set_style_border_width(list_container, 1, 0);
    lv_obj_set_style_radius(list_container, 5, 0);
    lv_obj_set_style_pad_all(list_container, 5, 0);
    lv_obj_set_scrollbar_mode(list_container, LV_SCROLLBAR_MODE_AUTO);
    
    // File list
    s_file_list = lv_list_create(list_container);
    lv_obj_set_size(s_file_list, lv_pct(100), lv_pct(100));
    lv_obj_set_style_bg_color(s_file_list, lv_color_hex(0x16213e), 0);
    lv_obj_set_style_border_width(s_file_list, 0, 0);
    lv_obj_set_style_pad_all(s_file_list, 0, 0);
    
    // Progress bar
    s_progress_bar = lv_bar_create(screen);
    lv_obj_set_size(s_progress_bar, scr_width - 30, 8);
    lv_bar_set_range(s_progress_bar, 0, 100);
    lv_bar_set_value(s_progress_bar, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(s_progress_bar, lv_color_hex(0x333333), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_progress_bar, lv_color_hex(0x00ff88), LV_PART_INDICATOR);
    lv_obj_set_style_radius(s_progress_bar, 4, 0);
    lv_obj_align(s_progress_bar, LV_ALIGN_BOTTOM_MID, 0, -100);
    
    // Time label
    s_time_label = lv_label_create(screen);
    lv_label_set_text(s_time_label, "00:00 / 00:00");
    lv_obj_set_style_text_color(s_time_label, lv_color_hex(0xffffff), 0);
    lv_obj_set_style_text_font(s_time_label, &lv_font_montserrat_14, 0);
    lv_obj_align(s_time_label, LV_ALIGN_BOTTOM_MID, 0, -82);
    
    // Button container
    lv_obj_t *btn_container = lv_obj_create(screen);
    lv_obj_remove_style_all(btn_container);
    lv_obj_set_size(btn_container, 140, 45);
    lv_obj_align(btn_container, LV_ALIGN_BOTTOM_MID, 0, -35);
    lv_obj_set_flex_flow(btn_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(btn_container, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    
    // Play button
    s_play_btn = lv_btn_create(btn_container);
    lv_obj_set_size(s_play_btn, 60, 40);
    lv_obj_set_style_bg_color(s_play_btn, lv_color_hex(0x44aa44), 0);
    lv_obj_set_style_bg_color(s_play_btn, lv_color_hex(0x66cc66), LV_STATE_PRESSED);
    lv_obj_set_style_bg_color(s_play_btn, lv_color_hex(0x336633), LV_STATE_DISABLED);
    lv_obj_add_event_cb(s_play_btn, play_btn_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_state(s_play_btn, LV_STATE_DISABLED);
    
    lv_obj_t *play_label = lv_label_create(s_play_btn);
    lv_label_set_text(play_label, LV_SYMBOL_PLAY);
    lv_obj_center(play_label);
    
    // Stop button
    s_stop_btn = lv_btn_create(btn_container);
    lv_obj_set_size(s_stop_btn, 60, 40);
    lv_obj_set_style_bg_color(s_stop_btn, lv_color_hex(0xff4444), 0);
    lv_obj_set_style_bg_color(s_stop_btn, lv_color_hex(0xff6666), LV_STATE_PRESSED);
    lv_obj_set_style_bg_color(s_stop_btn, lv_color_hex(0x663333), LV_STATE_DISABLED);
    lv_obj_add_event_cb(s_stop_btn, stop_btn_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_state(s_stop_btn, LV_STATE_DISABLED);
    
    lv_obj_t *stop_label = lv_label_create(s_stop_btn);
    lv_label_set_text(stop_label, LV_SYMBOL_STOP);
    lv_obj_center(stop_label);
    
    // Volume control section
    s_volume_label = lv_label_create(screen);
    char vol_buf[32];
    snprintf(vol_buf, sizeof(vol_buf), LV_SYMBOL_VOLUME_MAX " %d%%", INITIAL_VOLUME);
    lv_label_set_text(s_volume_label, vol_buf);
    lv_obj_set_style_text_color(s_volume_label, lv_color_hex(0xaaaaaa), 0);
    lv_obj_set_style_text_font(s_volume_label, &lv_font_montserrat_14, 0);
    lv_obj_align(s_volume_label, LV_ALIGN_BOTTOM_LEFT, 10, -8);
    
    s_volume_slider = lv_slider_create(screen);
    lv_obj_set_size(s_volume_slider, scr_width - 80, 8);
    lv_slider_set_range(s_volume_slider, 0, 100);
    lv_slider_set_value(s_volume_slider, INITIAL_VOLUME, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(s_volume_slider, lv_color_hex(0x333333), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_volume_slider, lv_color_hex(0x4488ff), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(s_volume_slider, lv_color_hex(0x6699ff), LV_PART_KNOB);
    lv_obj_set_style_pad_all(s_volume_slider, 3, LV_PART_KNOB);
    lv_obj_align(s_volume_slider, LV_ALIGN_BOTTOM_RIGHT, -10, -8);
    lv_obj_add_event_cb(s_volume_slider, volume_slider_cb, LV_EVENT_VALUE_CHANGED, nullptr);
    
    // Populate file list
    update_file_list();
}

static void update_file_list(void) {
    if (s_file_list == nullptr) return;
    
    // Clear existing items
    lv_obj_clean(s_file_list);
    
    // Scan for audio files
    scan_audio_files();
    
    if (s_audio_files.empty()) {
        lv_obj_t *item = lv_list_add_text(s_file_list, "No audio files found");
        lv_obj_set_style_text_color(item, lv_color_hex(0xff6666), 0);
        return;
    }
    
    // Add files to list
    for (size_t i = 0; i < s_audio_files.size(); i++) {
        lv_obj_t *btn = lv_list_add_button(s_file_list, LV_SYMBOL_AUDIO, s_audio_files[i].c_str());
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x16213e), 0);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x0f3460), LV_STATE_FOCUSED);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x1a4060), LV_STATE_PRESSED);
        lv_obj_set_style_text_color(btn, lv_color_hex(0xffffff), 0);
        lv_obj_add_event_cb(btn, file_list_event_cb, LV_EVENT_CLICKED, (void*)(intptr_t)i);
    }
}

static void update_ui_state(void) {
    app_state_t state = get_state();
    
    switch (state) {
        case STATE_IDLE:
            lv_obj_set_style_text_color(s_status_label, lv_color_hex(0xaaaaaa), 0);
            lv_obj_clear_state(s_stop_btn, LV_STATE_DISABLED);
            lv_obj_add_state(s_stop_btn, LV_STATE_DISABLED);
            lv_obj_set_style_bg_color(s_progress_bar, lv_color_hex(0x00ff88), LV_PART_INDICATOR);
            
            // Enable play button only if a file is selected
            if (s_selected_file_index >= 0) {
                lv_obj_clear_state(s_play_btn, LV_STATE_DISABLED);
            }
            break;
            
        case STATE_PLAYING:
            lv_label_set_text(s_status_label, LV_SYMBOL_PLAY " Playing...");
            lv_obj_set_style_text_color(s_status_label, lv_color_hex(0x44aa44), 0);
            lv_obj_add_state(s_play_btn, LV_STATE_DISABLED);
            lv_obj_clear_state(s_stop_btn, LV_STATE_DISABLED);
            lv_obj_set_style_bg_color(s_progress_bar, lv_color_hex(0x44aa44), LV_PART_INDICATOR);
            break;
    }
}

// ============================================================================
// Event Callbacks
// ============================================================================

static void file_list_event_cb(lv_event_t *e) {
    int index = (int)(intptr_t)lv_event_get_user_data(e);
    
    if (index < 0 || index >= (int)s_audio_files.size()) {
        return;
    }
    
    // Don't allow selection while playing
    if (get_state() == STATE_PLAYING) {
        return;
    }
    
    s_selected_file_index = index;
    s_selected_file = s_audio_files[index];
    
    ESP_LOGI(TAG, "Selected file: %s", s_selected_file.c_str());
    
    // Determine file type and parse header
    audio_file_type_t file_type = get_audio_file_type(s_selected_file.c_str());
    bool valid = false;
    uint32_t sample_rate = 0;
    uint32_t duration_sec = 0;
    int num_channels = 0;
    const char* type_str = "Audio";
    
    if (file_type == AUDIO_TYPE_WAV) {
        wav_info_t info = parse_wav_header(s_selected_file.c_str());
        valid = info.valid;
        sample_rate = info.sample_rate;
        duration_sec = info.duration_sec;
        num_channels = info.num_channels;
        type_str = "WAV";
    } else if (file_type == AUDIO_TYPE_MP3) {
        mp3_info_t info = parse_mp3_header(s_selected_file.c_str());
        valid = info.valid;
        sample_rate = info.sample_rate;
        duration_sec = info.duration_sec;
        num_channels = info.num_channels;
        type_str = "MP3";
    }
    
    if (valid) {
        // Update status label with filename
        char status_buf[FILE_NAME_MAX_LEN + 16];
        snprintf(status_buf, sizeof(status_buf), LV_SYMBOL_AUDIO " %s", s_selected_file.c_str());
        lv_label_set_text(s_status_label, status_buf);
        lv_obj_set_style_text_color(s_status_label, lv_color_hex(0x00ff88), 0);
        
        // Update info label with sample rate, duration, and type
        char info_buf[64];
        snprintf(info_buf, sizeof(info_buf), "%s • %s • %d ch • %s", 
                 format_sample_rate(sample_rate).c_str(),
                 format_time(duration_sec).c_str(),
                 num_channels,
                 type_str);
        lv_label_set_text(s_info_label, info_buf);
        
        // Update time label
        char time_buf[32];
        snprintf(time_buf, sizeof(time_buf), "00:00 / %s", format_time(duration_sec).c_str());
        lv_label_set_text(s_time_label, time_buf);
        
        // Enable play button
        lv_obj_clear_state(s_play_btn, LV_STATE_DISABLED);
    } else {
        char err_buf[32];
        snprintf(err_buf, sizeof(err_buf), "Invalid %s file!", type_str);
        lv_label_set_text(s_status_label, err_buf);
        lv_obj_set_style_text_color(s_status_label, lv_color_hex(0xff4444), 0);
        lv_label_set_text(s_info_label, "");
        lv_obj_add_state(s_play_btn, LV_STATE_DISABLED);
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
    
    if (get_state() != STATE_IDLE || s_selected_file.empty()) {
        return;
    }
    
    ESP_LOGI(TAG, "Starting playback of: %s", s_selected_file.c_str());
    s_stop_requested = false;
    s_playback_seconds = 0;
    
    // Create playback task
    xTaskCreate(audio_playback_task, "audio_playback", 8192, nullptr, 5, &s_audio_task);
#endif
}

static void stop_btn_cb(lv_event_t *e) {
    (void)e;
    
    if (get_state() == STATE_PLAYING) {
        ESP_LOGI(TAG, "Stop requested");
        s_stop_requested = true;
    }
}

static void volume_slider_cb(lv_event_t *e) {
    lv_obj_t *slider = (lv_obj_t*)lv_event_get_target(e);
    int value = lv_slider_get_value(slider);
    
    // Update volume label
    char buf[32];
    snprintf(buf, sizeof(buf), LV_SYMBOL_VOLUME_MAX " %d%%", value);
    lv_label_set_text(s_volume_label, buf);
    
    // Set audio codec volume
#ifdef CONFIG_CUBE32_AUDIO_ENABLED
    cube32::AudioCodec::instance().setOutputVolume(value);
#endif
}

// ============================================================================
// Audio Playback Task
// ============================================================================

/**
 * @brief Play WAV file
 */
static void play_wav_file(cube32::AudioCodec& codec, FILE *file, const wav_info_t& wav_info) {
    ESP_LOGI(TAG, "Playing WAV: %lu Hz, %d ch, %d bits, %lu bytes",
             (unsigned long)wav_info.sample_rate, wav_info.num_channels, 
             wav_info.bits_per_sample, (unsigned long)wav_info.data_size);
    
    s_playback_duration = wav_info.duration_sec;
    s_playback_sample_rate = wav_info.sample_rate;
    
    // Seek to audio data
    fseek(file, wav_info.data_offset, SEEK_SET);
    
    // Allocate audio buffer
    int16_t *audio_buffer = (int16_t*)heap_caps_malloc(AUDIO_BUFFER_SIZE * sizeof(int16_t), MALLOC_CAP_INTERNAL);
    if (audio_buffer == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate audio buffer");
        return;
    }
    
    // Enable audio output with the WAV file's sample rate
    codec.enableOutput(true, wav_info.sample_rate);
    
    uint32_t total_samples = wav_info.data_size / sizeof(int16_t);
    uint32_t samples_played = 0;
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
        
        // The codec is configured for MONO output (1 channel)
        // So we need to convert stereo to mono if needed
        if (wav_info.num_channels == 1) {
            codec.write(audio_buffer, samples_read);
        } else {
            // Stereo: downmix to mono
            int frames = samples_read / 2;
            for (int i = 0; i < frames; i++) {
                int32_t left = audio_buffer[i * 2];
                int32_t right = audio_buffer[i * 2 + 1];
                audio_buffer[i] = (int16_t)((left + right) / 2);
            }
            codec.write(audio_buffer, frames);
        }
        
        samples_played += samples_read;
        
        // Calculate playback position
        int elapsed_sec = (samples_played / wav_info.num_channels) / wav_info.sample_rate;
        
        if (elapsed_sec != last_second) {
            last_second = elapsed_sec;
            s_playback_seconds = elapsed_sec;
            
            // Update UI
            if (lvgl_port_lock(10)) {
                char buf[32];
                snprintf(buf, sizeof(buf), "%s / %s",
                         format_time(elapsed_sec).c_str(),
                         format_time(s_playback_duration).c_str());
                lv_label_set_text(s_time_label, buf);
                
                int progress = (s_playback_duration > 0) ? 
                               (elapsed_sec * 100 / s_playback_duration) : 0;
                if (progress > 100) progress = 100;
                lv_bar_set_value(s_progress_bar, progress, LV_ANIM_OFF);
                lvgl_port_unlock();
            }
        }
    }
    
    codec.enableOutput(false);
    ESP_LOGI(TAG, "WAV playback complete: %lu samples", (unsigned long)samples_played);
    heap_caps_free(audio_buffer);
}

/**
 * @brief Play MP3 file
 */
static void play_mp3_file(cube32::AudioCodec& codec, FILE *file, const mp3_info_t& mp3_info) {
    ESP_LOGI(TAG, "Playing MP3: %lu Hz, %d ch, ~%lu sec",
             (unsigned long)mp3_info.sample_rate, mp3_info.num_channels,
             (unsigned long)mp3_info.duration_sec);
    
    s_playback_duration = mp3_info.duration_sec;
    s_playback_sample_rate = mp3_info.sample_rate;
    
    // Initialize MP3 decoder
    HMP3Decoder decoder = MP3InitDecoder();
    if (decoder == nullptr) {
        ESP_LOGE(TAG, "Failed to initialize MP3 decoder");
        return;
    }
    
    // Allocate buffers
    uint8_t *read_buffer = (uint8_t*)heap_caps_malloc(MP3_READ_BUFFER_SIZE, MALLOC_CAP_INTERNAL);
    int16_t *decode_buffer = (int16_t*)heap_caps_malloc(MP3_DECODE_BUFFER_SIZE * sizeof(int16_t), MALLOC_CAP_INTERNAL);
    
    if (read_buffer == nullptr || decode_buffer == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate MP3 buffers");
        if (read_buffer) heap_caps_free(read_buffer);
        if (decode_buffer) heap_caps_free(decode_buffer);
        MP3FreeDecoder(decoder);
        return;
    }
    
    // Seek past ID3v2 tag
    fseek(file, mp3_info.audio_data_start, SEEK_SET);
    
    // Enable audio output with MP3 sample rate
    if (mp3_info.sample_rate > 0) {
        codec.enableOutput(true, mp3_info.sample_rate);
    } else {
        // Default to 44100 Hz until we decode a frame
        codec.enableOutput(true, 44100);
    }
    
    int bytes_in_buffer = 0;
    uint8_t *read_ptr = read_buffer;
    bool eof_reached = false;
    int last_second = -1;
    uint32_t total_frames_played = 0;
    uint32_t current_sample_rate = mp3_info.sample_rate > 0 ? mp3_info.sample_rate : 44100;
    
    while (!s_stop_requested) {
        // Calculate remaining bytes in buffer
        int unread_bytes = bytes_in_buffer - (read_ptr - read_buffer);
        
        // Refill buffer if needed
        if (unread_bytes < MAINBUF_SIZE && !eof_reached) {
            // Move remaining data to beginning of buffer
            if (unread_bytes > 0 && read_ptr != read_buffer) {
                memmove(read_buffer, read_ptr, unread_bytes);
            }
            read_ptr = read_buffer;
            
            // Read more data
            int free_space = MP3_READ_BUFFER_SIZE - unread_bytes;
            int bytes_read = fread(read_buffer + unread_bytes, 1, free_space, file);
            
            bytes_in_buffer = unread_bytes + bytes_read;
            
            if (bytes_read == 0 || feof(file)) {
                eof_reached = true;
            }
            
            unread_bytes = bytes_in_buffer;
        }
        
        if (unread_bytes <= 0) {
            break;
        }
        
        // Find sync word
        int offset = MP3FindSyncWord(read_ptr, unread_bytes);
        if (offset < 0) {
            // No sync word found
            if (eof_reached) {
                break;
            }
            // Skip some data and try again
            read_ptr = read_buffer;
            bytes_in_buffer = 0;
            continue;
        }
        
        read_ptr += offset;
        unread_bytes -= offset;
        
        // Decode frame
        int err = MP3Decode(decoder, &read_ptr, &unread_bytes, decode_buffer, 0);
        
        if (err == ERR_MP3_NONE) {
            MP3FrameInfo frame_info;
            MP3GetLastFrameInfo(decoder, &frame_info);
            
            // Update sample rate if it changed
            if ((uint32_t)frame_info.samprate != current_sample_rate) {
                current_sample_rate = frame_info.samprate;
                codec.enableOutput(true, current_sample_rate);
            }
            
            int frame_samples = frame_info.outputSamps;
            int num_channels = frame_info.nChans;
            
            // Convert to mono if needed
            if (num_channels == 1) {
                codec.write(decode_buffer, frame_samples);
            } else {
                // Stereo: downmix to mono
                int frames = frame_samples / num_channels;
                for (int i = 0; i < frames; i++) {
                    int32_t left = decode_buffer[i * 2];
                    int32_t right = decode_buffer[i * 2 + 1];
                    decode_buffer[i] = (int16_t)((left + right) / 2);
                }
                codec.write(decode_buffer, frames);
            }
            
            total_frames_played += frame_samples / num_channels;
            
            // Calculate elapsed time
            int elapsed_sec = total_frames_played / current_sample_rate;
            
            if (elapsed_sec != last_second) {
                last_second = elapsed_sec;
                s_playback_seconds = elapsed_sec;
                
                // Update UI
                if (lvgl_port_lock(10)) {
                    char buf[32];
                    snprintf(buf, sizeof(buf), "%s / %s",
                             format_time(elapsed_sec).c_str(),
                             format_time(s_playback_duration).c_str());
                    lv_label_set_text(s_time_label, buf);
                    
                    int progress = (s_playback_duration > 0) ? 
                                   (elapsed_sec * 100 / s_playback_duration) : 0;
                    if (progress > 100) progress = 100;
                    lv_bar_set_value(s_progress_bar, progress, LV_ANIM_OFF);
                    lvgl_port_unlock();
                }
            }
        } else if (err == ERR_MP3_MAINDATA_UNDERFLOW) {
            // Need more data, continue
            continue;
        } else {
            // Other errors - try to continue
            if (eof_reached && unread_bytes < 2) {
                break;
            }
            // Skip this frame
            ESP_LOGW(TAG, "MP3 decode error %d, skipping", err);
        }
    }
    
    codec.enableOutput(false);
    ESP_LOGI(TAG, "MP3 playback complete: %lu frames", (unsigned long)total_frames_played);
    
    heap_caps_free(read_buffer);
    heap_caps_free(decode_buffer);
    MP3FreeDecoder(decoder);
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
        cube32::SDCard& sd = cube32::SDCard::instance();
        
        // Build file path
        std::string file_path = sd.getFullPath(s_selected_file.c_str());
        
        // Open file for reading
        FILE *file = fopen(file_path.c_str(), "rb");
        if (file == nullptr) {
            ESP_LOGE(TAG, "Failed to open file: %s (errno=%d: %s)",
                     file_path.c_str(), errno, strerror(errno));
            break;
        }
        
        // Determine file type and play accordingly
        audio_file_type_t file_type = get_audio_file_type(s_selected_file.c_str());
        
        if (file_type == AUDIO_TYPE_WAV) {
            wav_info_t wav_info = parse_wav_header(s_selected_file.c_str());
            if (wav_info.valid) {
                play_wav_file(codec, file, wav_info);
            } else {
                ESP_LOGE(TAG, "Invalid WAV file");
            }
        } else if (file_type == AUDIO_TYPE_MP3) {
            mp3_info_t mp3_info = parse_mp3_header(s_selected_file.c_str());
            if (mp3_info.valid) {
                play_mp3_file(codec, file, mp3_info);
            } else {
                ESP_LOGE(TAG, "Invalid MP3 file");
            }
        } else {
            ESP_LOGE(TAG, "Unsupported file type");
        }
        
        fclose(file);
    } while (0);
#endif

    set_state(STATE_IDLE);
    
    // Update UI
    if (lvgl_port_lock(100)) {
        update_ui_state();
        
        if (s_selected_file_index >= 0 && !s_selected_file.empty()) {
            // Restore selected file info
            char status_buf[FILE_NAME_MAX_LEN + 16];
            snprintf(status_buf, sizeof(status_buf), LV_SYMBOL_AUDIO " %s", s_selected_file.c_str());
            lv_label_set_text(s_status_label, status_buf);
            lv_obj_set_style_text_color(s_status_label, lv_color_hex(0x00ff88), 0);
        } else {
            lv_label_set_text(s_status_label, "Select a file to play");
        }
        
        lv_bar_set_value(s_progress_bar, 0, LV_ANIM_OFF);
        
        // Reset time display
        char time_buf[32];
        snprintf(time_buf, sizeof(time_buf), "00:00 / %s", 
                 format_time(s_playback_duration).c_str());
        lv_label_set_text(s_time_label, time_buf);
        
        lvgl_port_unlock();
    }
    
    s_audio_task = nullptr;
    vTaskDelete(nullptr);
}

// ============================================================================
// Console Task (Audio IOX Test Commands)
// ============================================================================

static void print_console_help(void) {
    printf("\nAudio IOX Test Commands:\n");
    printf("  pa on     - Enable PA amplifier\n");
    printf("  pa off    - Disable PA amplifier\n");
    printf("  pa        - Read PA IOX diagnostic info\n");
    printf("  pj        - Show earphone jack (PJ_DET) status\n");
    printf("  help      - Show this help\n\n");
}

static void console_task(void *pvParameters) {
    // Configure console I/O
    setvbuf(stdout, NULL, _IONBF, 0);
    ESP_ERROR_CHECK(uart_driver_install((uart_port_t)CONFIG_ESP_CONSOLE_UART_NUM,
                                        256, 0, 0, NULL, 0));
    uart_vfs_dev_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
    uart_vfs_dev_port_set_rx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CR);
    uart_vfs_dev_port_set_tx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CRLF);

    vTaskDelay(pdMS_TO_TICKS(500));
    print_console_help();

    char line_buf[64];

    while (true) {
        printf("[AUDIO] > ");
        fflush(stdout);

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
            if (c == 127 || c == 8) {
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

#ifdef CONFIG_CUBE32_AUDIO_ENABLED
        cube32::AudioCodec& codec = cube32::AudioCodec::instance();

        if (strncmp(line_buf, "pa on", 5) == 0) {
            codec.setPAEnabled(true);
            printf("PA enabled\n");
        } else if (strncmp(line_buf, "pa off", 6) == 0) {
            codec.setPAEnabled(false);
            printf("PA disabled\n");
        } else if (strcmp(line_buf, "pa") == 0) {
            codec.readPADiagnostics();
        } else if (strncmp(line_buf, "pj", 2) == 0) {
            bool plugged = codec.isEarphonePlugged();
            printf("Earphone jack: %s\n", plugged ? "PLUGGED" : "NOT PLUGGED");
        } else if (strncmp(line_buf, "help", 4) == 0) {
            print_console_help();
        } else {
            printf("Unknown command: '%s' (type 'help')\n", line_buf);
        }
#else
        printf("Audio not enabled\n");
#endif
    }
}

// ============================================================================
// Main
// ============================================================================

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "CUBE32 Hello Audio Player Example");
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
    ESP_LOGW(TAG, "SD Card is not enabled! Playback will not work.");
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

    ESP_LOGI(TAG, "Audio player ready!");
    ESP_LOGI(TAG, "  - Supports WAV and MP3 audio files");
    ESP_LOGI(TAG, "  - Select a file from the list");
    ESP_LOGI(TAG, "  - Press PLAY to start playback");
    ESP_LOGI(TAG, "  - Press STOP to stop playback");
    ESP_LOGI(TAG, "  - Use the slider to adjust volume");
#ifdef CONFIG_CUBE32_ADC_BUTTON_ENABLED
    ESP_LOGI(TAG, "  - ADC Buttons: Vol+, Vol-, Play/Stop, Mute");
#endif
    ESP_LOGI(TAG, "  - Console: 'pa on', 'pa off', 'pj' (type 'help')");

    // Start console task for audio IOX test commands
    xTaskCreate(console_task, "console", 4096, nullptr, 5, nullptr);

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
            snprintf(buf, sizeof(buf), LV_SYMBOL_VOLUME_MAX " %d%%", volume);
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
    } else if (get_state() == STATE_IDLE && !s_selected_file.empty()) {
        // Start playback
#ifdef CONFIG_CUBE32_SDCARD_ENABLED
        if (!cube32::SDCard::instance().isInitialized()) {
            ESP_LOGW(TAG, "SD Card not ready");
            return;
        }
        
        ESP_LOGI(TAG, "Starting playback of: %s", s_selected_file.c_str());
        s_stop_requested = false;
        s_playback_seconds = 0;
        
        // Update UI to show playing state
        if (lvgl_port_lock(100)) {
            update_ui_state();
            lvgl_port_unlock();
        }
        
        // Create playback task
        xTaskCreate(audio_playback_task, "audio_playback", 8192, nullptr, 5, &s_audio_task);
#endif
    } else {
        ESP_LOGW(TAG, "No file selected or not ready");
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
 * @brief SET button handler (Button 2)
 */
static void set_handler(void) {
    ESP_LOGI(TAG, "ADC Button: SET");
}

/**
 * @brief REC button handler (Button 5)
 */
static void rec_handler(void) {
    ESP_LOGI(TAG, "ADC Button: REC");
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
        "REC"
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
 * - Button 2 (1.11V, 5.1K): SET (onClick, onLongPress)
 * - Button 3 (1.65V, 10K):  Play/Stop (onClick, onLongPress)
 * - Button 4 (1.98V, 15K):  Mute (onClick, onLongPress)
 * - Button 5 (2.41V, 27K):  REC (onClick, onLongPress)
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
    
    // Button 2 (1.11V): SET - onClick
    adc_btn.registerCallback(cube32::ADCButtonIndex::BUTTON_2, BUTTON_SINGLE_CLICK,
        [](cube32::ADCButtonIndex btn, button_event_t event) {
            (void)btn;
            (void)event;
            set_handler();
        });
    
    // Button 5 (2.41V): REC - onClick
    adc_btn.registerCallback(cube32::ADCButtonIndex::BUTTON_5, BUTTON_SINGLE_CLICK,
        [](cube32::ADCButtonIndex btn, button_event_t event) {
            (void)btn;
            (void)event;
            rec_handler();
        });
    
    // Register long press handlers for all buttons
    for (size_t i = 0; i < 6; i++) {
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
    ESP_LOGI(TAG, "  Button 5 (2.41V): REC (onClick, onLongPress)");
}

#endif // CONFIG_CUBE32_ADC_BUTTON_ENABLED
