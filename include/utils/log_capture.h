/**
 * @file log_capture.h
 * @brief CUBE32 Boot/Runtime Log Capture — PSRAM buffer + live-stream queue
 *
 * Mirrors ESP_LOG output (via esp_log_set_vprintf()) into a fixed-size PSRAM
 * buffer starting from the first line of cube32_init(), so the boot log can
 * be retrieved after the fact (e.g. by an AI agent over BLE OTA).
 *
 * Capture behavior:
 *   - Appends continuously, including the "extended" period after board init
 *     completes, until a client actually reads the boot log
 *     (cube32_log_capture_freeze_boot_log()) or the buffer fills up.
 *   - cube32_log_capture_mark_boot_complete() only sets an informational flag
 *     — it does NOT stop capture.
 *
 * A separate, independent path supports live log streaming: when enabled via
 * cube32_log_capture_stream_enable(), each subsequent log line (at or above
 * the given level) is pushed into a small non-blocking queue for a consumer
 * (e.g. a BLE notify task) to drain via cube32_log_capture_stream_pop().
 *
 * Usage:
 *   cube32_log_capture_init();               // first line of cube32_init()
 *   ...
 *   cube32_log_capture_mark_boot_complete();  // after hw->init_done = true
 */

#ifndef CUBE32_UTILS_LOG_CAPTURE_H
#define CUBE32_UTILS_LOG_CAPTURE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include <esp_log.h>
#include <freertos/FreeRTOS.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Allocate the PSRAM boot-log buffer and install the vprintf hook.
 *        Call as the very first statement of cube32_init(). Safe to call
 *        even if CUBE32_LOG_CAPTURE_ENABLED is off (no-op stub).
 */
void cube32_log_capture_init(void);

/**
 * @brief Mark board init as complete (informational only — does not stop
 *        capture; reported via the BLE LOG_GET_INFO command).
 */
void cube32_log_capture_mark_boot_complete(void);

/** @brief True once cube32_log_capture_mark_boot_complete() has been called. */
bool cube32_log_capture_is_boot_complete(void);

/**
 * @brief Freeze the boot-log buffer (idempotent). Called on the first actual
 *        boot-log read so later paginated reads see a consistent size.
 */
void cube32_log_capture_freeze_boot_log(void);

/** @brief True once the boot-log buffer has been frozen. */
bool cube32_log_capture_is_frozen(void);

/** @brief Current boot-log size (frozen length if frozen, else live length). */
size_t cube32_log_capture_get_boot_log_size(void);

/** @brief True if the boot-log buffer filled up before being frozen/read. */
bool cube32_log_capture_is_boot_log_truncated(void);

/**
 * @brief Pull-based, bounds-checked read of the captured boot log.
 * @return Number of bytes actually copied into out (0 if offset >= size).
 */
size_t cube32_log_capture_read_boot_log(uint32_t offset, uint8_t* out, size_t max_len);

/** @brief Enable/disable pushing subsequent log lines into the stream queue. */
void cube32_log_capture_stream_enable(bool enable, esp_log_level_t min_level);

/** @brief True if live streaming is currently enabled. */
bool cube32_log_capture_is_streaming(void);

/**
 * @brief Blocking pop of the next queued live-stream log line.
 * @return true if a line was popped (out_len set), false on timeout.
 */
bool cube32_log_capture_stream_pop(uint8_t* out, size_t max_len, size_t* out_len,
                                    TickType_t wait_ticks);

/** @brief Number of live-stream lines dropped because the queue was full. */
uint32_t cube32_log_capture_get_dropped_stream_count(void);

/**
 * @brief Suppress capture/streaming of log lines emitted by the calling task
 *        while inside a BLE log-send path — prevents the send path's own
 *        debug logs from being re-captured/re-streamed (feedback loop guard).
 *        Nestable; must be paired with cube32_log_capture_suppress_end().
 */
void cube32_log_capture_suppress_begin(void);
void cube32_log_capture_suppress_end(void);

#ifdef __cplusplus
}
#endif

#endif // CUBE32_UTILS_LOG_CAPTURE_H
