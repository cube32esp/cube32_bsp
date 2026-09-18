/**
 * @file log_capture.cpp
 * @brief CUBE32 Boot/Runtime Log Capture — implementation
 */

#include "utils/log_capture.h"

#include <cstdio>
#include <cstdarg>
#include <cstring>

#include <esp_heap_caps.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

#ifdef CONFIG_CUBE32_LOG_CAPTURE_ENABLED

static const char* TAG = "cube32_logcap";

#ifndef CONFIG_CUBE32_LOG_CAPTURE_BOOT_BUFFER_SIZE
#define CONFIG_CUBE32_LOG_CAPTURE_BOOT_BUFFER_SIZE 65536
#endif
#ifndef CONFIG_CUBE32_LOG_CAPTURE_STREAM_QUEUE_LEN
#define CONFIG_CUBE32_LOG_CAPTURE_STREAM_QUEUE_LEN 32
#endif
#ifndef CONFIG_CUBE32_LOG_CAPTURE_MAX_LINE_LEN
#define CONFIG_CUBE32_LOG_CAPTURE_MAX_LINE_LEN 200
#endif

#define LOG_CAP_BOOT_BUFFER_SIZE   ((size_t)CONFIG_CUBE32_LOG_CAPTURE_BOOT_BUFFER_SIZE)
#define LOG_CAP_STREAM_QUEUE_LEN   ((size_t)CONFIG_CUBE32_LOG_CAPTURE_STREAM_QUEUE_LEN)
#define LOG_CAP_MAX_LINE_LEN       ((size_t)CONFIG_CUBE32_LOG_CAPTURE_MAX_LINE_LEN)

typedef struct {
    esp_log_level_t level;
    uint16_t len;
    char data[CONFIG_CUBE32_LOG_CAPTURE_MAX_LINE_LEN];
} log_stream_item_t;

// ---- Boot-log buffer state (mutex-protected) -------------------------------
static uint8_t* s_boot_buf = nullptr;
static size_t s_boot_capacity = 0;
static size_t s_boot_len = 0;
static bool s_boot_frozen = false;
static size_t s_boot_frozen_len = 0;
static bool s_boot_truncated = false;
static bool s_boot_complete = false;
static SemaphoreHandle_t s_boot_mutex = nullptr;

// ---- Live-stream state ------------------------------------------------------
static QueueHandle_t s_stream_queue = nullptr;
static StaticQueue_t s_stream_queue_struct;
static uint8_t* s_stream_queue_storage = nullptr;
static volatile bool s_stream_enabled = false;
static esp_log_level_t s_stream_min_level = ESP_LOG_WARN;
static volatile uint32_t s_stream_dropped = 0;
static portMUX_TYPE s_dropped_spinlock = portMUX_INITIALIZER_UNLOCKED;

// ---- vprintf chaining + re-entrancy guard -----------------------------------
// vprintf_like_t is declared by esp_log.h (int (*)(const char*, va_list)).
static vprintf_like_t s_orig_vprintf = nullptr;
static TaskHandle_t s_suppress_task = nullptr;
static int s_suppress_depth = 0;

static esp_log_level_t parse_level_from_line(const char* line, size_t len)
{
    size_t i = 0;
    // Skip an optional leading ANSI color escape sequence: "\033[...m"
    if (len >= 2 && line[0] == '\033' && line[1] == '[') {
        i = 2;
        while (i < len && line[i] != 'm') {
            i++;
        }
        if (i < len) {
            i++; // skip the 'm'
        }
    }
    if (i >= len) {
        return ESP_LOG_INFO;
    }
    switch (line[i]) {
    case 'E': return ESP_LOG_ERROR;
    case 'W': return ESP_LOG_WARN;
    case 'I': return ESP_LOG_INFO;
    case 'D': return ESP_LOG_DEBUG;
    case 'V': return ESP_LOG_VERBOSE;
    default:  return ESP_LOG_INFO;
    }
}

static int cube32_log_vprintf_hook(const char* fmt, va_list args)
{
    va_list copy_console;
    va_list copy_capture;
    va_copy(copy_console, args);
    va_copy(copy_capture, args);

    int ret = 0;
    if (s_orig_vprintf) {
        ret = s_orig_vprintf(fmt, copy_console);
    }
    va_end(copy_console);

    // Suppressed for this task (inside a BLE log-send path) — console output
    // above still happened, just skip re-capture/re-stream to avoid feedback.
    if (s_suppress_task != nullptr && xTaskGetCurrentTaskHandle() == s_suppress_task) {
        va_end(copy_capture);
        return ret;
    }

    char line[CONFIG_CUBE32_LOG_CAPTURE_MAX_LINE_LEN];
    int n = vsnprintf(line, sizeof(line), fmt, copy_capture);
    va_end(copy_capture);
    if (n <= 0) {
        return ret;
    }
    size_t line_len = ((size_t)n < sizeof(line)) ? (size_t)n : (sizeof(line) - 1);
    esp_log_level_t level = parse_level_from_line(line, line_len);

    // Append to boot buffer (extended capture — keeps running after board
    // init completes, until a client freezes it via the first chunk read).
    if (s_boot_buf && !s_boot_frozen && s_boot_mutex) {
        if (xSemaphoreTake(s_boot_mutex, 0) == pdTRUE) {
            if (!s_boot_frozen) {
                if (s_boot_len + line_len <= s_boot_capacity) {
                    memcpy(s_boot_buf + s_boot_len, line, line_len);
                    s_boot_len += line_len;
                } else {
                    s_boot_truncated = true;
                }
            }
            xSemaphoreGive(s_boot_mutex);
        }
    }

    // Push to live-stream queue (best-effort, never blocks the caller).
    if (s_stream_enabled && s_stream_queue && level <= s_stream_min_level) {
        log_stream_item_t item;
        item.level = level;
        item.len = (uint16_t)line_len;
        memcpy(item.data, line, line_len);
        if (xQueueSend(s_stream_queue, &item, 0) != pdTRUE) {
            portENTER_CRITICAL(&s_dropped_spinlock);
            s_stream_dropped++;
            portEXIT_CRITICAL(&s_dropped_spinlock);
        }
    }

    return ret;
}

void cube32_log_capture_init(void)
{
    if (s_boot_buf) {
        return; // already initialized
    }

    s_boot_buf = (uint8_t*)heap_caps_malloc(LOG_CAP_BOOT_BUFFER_SIZE,
                                             MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_boot_buf) {
        // Fall back gracefully — no capture, but console logging still works.
        ESP_LOGW(TAG, "Failed to allocate %u-byte PSRAM boot-log buffer — capture disabled",
                 (unsigned)LOG_CAP_BOOT_BUFFER_SIZE);
        return;
    }
    s_boot_capacity = LOG_CAP_BOOT_BUFFER_SIZE;
    s_boot_mutex = xSemaphoreCreateMutex();

    s_stream_queue_storage = (uint8_t*)heap_caps_malloc(
        LOG_CAP_STREAM_QUEUE_LEN * sizeof(log_stream_item_t),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_stream_queue_storage) {
        s_stream_queue = xQueueCreateStatic(LOG_CAP_STREAM_QUEUE_LEN,
                                             sizeof(log_stream_item_t),
                                             s_stream_queue_storage,
                                             &s_stream_queue_struct);
    }

    s_orig_vprintf = esp_log_set_vprintf(cube32_log_vprintf_hook);
    ESP_LOGI(TAG, "Boot-log capture active: %u-byte PSRAM buffer",
             (unsigned)LOG_CAP_BOOT_BUFFER_SIZE);
}

void cube32_log_capture_mark_boot_complete(void)
{
    s_boot_complete = true;
}

bool cube32_log_capture_is_boot_complete(void)
{
    return s_boot_complete;
}

void cube32_log_capture_freeze_boot_log(void)
{
    if (!s_boot_mutex || s_boot_frozen) {
        return;
    }
    xSemaphoreTake(s_boot_mutex, portMAX_DELAY);
    if (!s_boot_frozen) {
        s_boot_frozen = true;
        s_boot_frozen_len = s_boot_len;
    }
    xSemaphoreGive(s_boot_mutex);
}

bool cube32_log_capture_is_frozen(void)
{
    return s_boot_frozen;
}

size_t cube32_log_capture_get_boot_log_size(void)
{
    return s_boot_frozen ? s_boot_frozen_len : s_boot_len;
}

bool cube32_log_capture_is_boot_log_truncated(void)
{
    return s_boot_truncated;
}

size_t cube32_log_capture_read_boot_log(uint32_t offset, uint8_t* out, size_t max_len)
{
    if (!s_boot_buf || !out || max_len == 0 || !s_boot_mutex) {
        return 0;
    }
    size_t copied = 0;
    xSemaphoreTake(s_boot_mutex, portMAX_DELAY);
    size_t total = s_boot_frozen ? s_boot_frozen_len : s_boot_len;
    if (offset < total) {
        size_t remaining = total - offset;
        copied = (max_len < remaining) ? max_len : remaining;
        memcpy(out, s_boot_buf + offset, copied);
    }
    xSemaphoreGive(s_boot_mutex);
    return copied;
}

void cube32_log_capture_stream_enable(bool enable, esp_log_level_t min_level)
{
    s_stream_min_level = min_level;
    s_stream_enabled = enable;
}

bool cube32_log_capture_is_streaming(void)
{
    return s_stream_enabled;
}

bool cube32_log_capture_stream_pop(uint8_t* out, size_t max_len, size_t* out_len,
                                    TickType_t wait_ticks)
{
    if (!s_stream_queue || !out || !out_len) {
        return false;
    }
    log_stream_item_t item;
    if (xQueueReceive(s_stream_queue, &item, wait_ticks) != pdTRUE) {
        return false;
    }
    size_t copy_len = (max_len < item.len) ? max_len : item.len;
    memcpy(out, item.data, copy_len);
    *out_len = copy_len;
    return true;
}

uint32_t cube32_log_capture_get_dropped_stream_count(void)
{
    return s_stream_dropped;
}

void cube32_log_capture_suppress_begin(void)
{
    // Only meaningful for the calling task; nested calls just bump the depth.
    TaskHandle_t self = xTaskGetCurrentTaskHandle();
    if (s_suppress_task == self) {
        s_suppress_depth++;
    } else if (s_suppress_task == nullptr) {
        s_suppress_task = self;
        s_suppress_depth = 1;
    }
    // If another task already holds suppression, do nothing — this guard is
    // per-task by design (each task's own log lines are serialized on its
    // own call stack).
}

void cube32_log_capture_suppress_end(void)
{
    if (s_suppress_task == xTaskGetCurrentTaskHandle() && s_suppress_depth > 0) {
        s_suppress_depth--;
        if (s_suppress_depth == 0) {
            s_suppress_task = nullptr;
        }
    }
}

#else // !CONFIG_CUBE32_LOG_CAPTURE_ENABLED — stub implementation

void cube32_log_capture_init(void) {}
void cube32_log_capture_mark_boot_complete(void) {}
bool cube32_log_capture_is_boot_complete(void) { return false; }
void cube32_log_capture_freeze_boot_log(void) {}
bool cube32_log_capture_is_frozen(void) { return false; }
size_t cube32_log_capture_get_boot_log_size(void) { return 0; }
bool cube32_log_capture_is_boot_log_truncated(void) { return false; }
size_t cube32_log_capture_read_boot_log(uint32_t, uint8_t*, size_t) { return 0; }
void cube32_log_capture_stream_enable(bool, esp_log_level_t) {}
bool cube32_log_capture_is_streaming(void) { return false; }
bool cube32_log_capture_stream_pop(uint8_t*, size_t, size_t*, TickType_t) { return false; }
uint32_t cube32_log_capture_get_dropped_stream_count(void) { return 0; }
void cube32_log_capture_suppress_begin(void) {}
void cube32_log_capture_suppress_end(void) {}

#endif // CONFIG_CUBE32_LOG_CAPTURE_ENABLED
