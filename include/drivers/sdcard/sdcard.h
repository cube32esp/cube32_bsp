/**
 * @file sdcard.h
 * @brief CUBE32 SD Card Driver (SDMMC Interface)
 * 
 * This driver provides support for SD cards using the SDMMC peripheral.
 * Uses ESP-IDF's FATFS integration for file system operations.
 * 
 * Features:
 * - SD card initialization and mounting with FATFS
 * - Card status information (size, type, speed)
 * - Directory listing
 * - File read/write operations
 * - Read/write speed testing
 */

#ifndef CUBE32_DRIVERS_SDCARD_SDCARD_H
#define CUBE32_DRIVERS_SDCARD_SDCARD_H

#include "utils/common.h"
#include "cube32_config.h"

#include <esp_vfs_fat.h>
#include <sdmmc_cmd.h>
#include <driver/sdmmc_host.h>

#include <string>
#include <vector>
#include <functional>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Constants
// ============================================================================

/**
 * @brief Default mount point for SD card
 */
#define CUBE32_SDCARD_MOUNT_POINT      "/sdcard"

/**
 * @brief Maximum file path length
 */
#define CUBE32_SDCARD_MAX_PATH_LEN     256

/**
 * @brief Default test buffer size for speed tests (4KB)
 */
#define CUBE32_SDCARD_TEST_BUFFER_SIZE 4096

/**
 * @brief Default test file size for speed tests (1MB)
 */
#define CUBE32_SDCARD_TEST_FILE_SIZE   (1024 * 1024)

// ============================================================================
// Configuration Structures
// ============================================================================

/**
 * @brief SD card configuration structure
 */
typedef struct {
    gpio_num_t pin_cmd;              ///< CMD pin
    gpio_num_t pin_clk;              ///< CLK pin
    gpio_num_t pin_d0;               ///< D0 pin (required for 1-bit and 4-bit mode)
    gpio_num_t pin_d1;               ///< D1 pin (optional, for 4-bit mode)
    gpio_num_t pin_d2;               ///< D2 pin (optional, for 4-bit mode)
    gpio_num_t pin_d3;               ///< D3 pin (optional, for 4-bit mode)
    gpio_num_t pin_cd;               ///< Card detect pin (GPIO_NUM_NC if not used)
    gpio_num_t pin_wp;               ///< Write protect pin (GPIO_NUM_NC if not used)
    uint8_t bus_width;               ///< Bus width (1 or 4)
    uint32_t max_freq_khz;           ///< Maximum clock frequency in kHz
    const char* mount_point;         ///< Mount point path
    bool format_if_mount_failed;     ///< Format card if mount fails
    int max_files;                   ///< Maximum number of open files
} cube32_sdcard_config_t;

/**
 * @brief Default SD card configuration using pins from cube32_config.h
 */
#define CUBE32_SDCARD_CONFIG_DEFAULT() { \
    .pin_cmd = CUBE32_SD_CMD_PIN, \
    .pin_clk = CUBE32_SD_CLK_PIN, \
    .pin_d0 = CUBE32_SD_D0_PIN, \
    .pin_d1 = GPIO_NUM_NC, \
    .pin_d2 = GPIO_NUM_NC, \
    .pin_d3 = GPIO_NUM_NC, \
    .pin_cd = GPIO_NUM_NC, \
    .pin_wp = GPIO_NUM_NC, \
    .bus_width = 1, \
    .max_freq_khz = SDMMC_FREQ_DEFAULT, \
    .mount_point = CUBE32_SDCARD_MOUNT_POINT, \
    .format_if_mount_failed = false, \
    .max_files = 5, \
}

/**
 * @brief SD card status structure
 */
typedef struct {
    bool mounted;                    ///< Card is mounted
    bool present;                    ///< Card is physically present
    uint64_t total_bytes;            ///< Total capacity in bytes
    uint64_t used_bytes;             ///< Used space in bytes
    uint64_t free_bytes;             ///< Free space in bytes
    uint32_t sector_size;            ///< Sector size in bytes
    uint32_t sector_count;           ///< Total sector count
    uint32_t max_freq_khz;           ///< Actual operating frequency
    const char* card_type;           ///< Card type string (SD, SDHC, SDXC, MMC)
    const char* speed_mode;          ///< Speed mode (Default Speed, High Speed)
    uint8_t bus_width;               ///< Actual bus width
} cube32_sdcard_status_t;

/**
 * @brief Directory entry structure
 */
typedef struct {
    char name[CUBE32_SDCARD_MAX_PATH_LEN]; ///< File/directory name
    bool is_directory;                      ///< True if directory
    uint32_t size;                          ///< File size in bytes
} cube32_sdcard_entry_t;

/**
 * @brief Speed test result structure
 */
typedef struct {
    bool success;                    ///< Test completed successfully
    float write_speed_kbps;          ///< Write speed in KB/s
    float read_speed_kbps;           ///< Read speed in KB/s
    uint32_t bytes_tested;           ///< Number of bytes tested
    uint32_t write_time_ms;          ///< Write time in milliseconds
    uint32_t read_time_ms;           ///< Read time in milliseconds
} cube32_sdcard_speed_result_t;

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

namespace cube32 {

// ============================================================================
// SDCard Class
// ============================================================================

/**
 * @brief SD Card Driver Class (SDMMC Interface)
 * 
 * Object-oriented interface for SD card operations using SDMMC peripheral.
 * Uses ESP-IDF's FATFS for file system support.
 * 
 * Usage:
 * @code
 *   cube32::SDCard& sd = cube32::SDCard::instance();
 *   if (sd.begin() == CUBE32_OK) {
 *       auto status = sd.getStatus();
 *       ESP_LOGI(TAG, "Card: %s, Size: %llu MB", 
 *                status.card_type, status.total_bytes / (1024 * 1024));
 *       
 *       // List root directory
 *       std::vector<cube32_sdcard_entry_t> entries;
 *       sd.listDirectory("/", entries);
 *       for (const auto& e : entries) {
 *           ESP_LOGI(TAG, "%s %s (%lu bytes)", 
 *                    e.is_directory ? "[DIR]" : "[FILE]",
 *                    e.name, e.size);
 *       }
 *   }
 * @endcode
 */
class SDCard {
public:
    /**
     * @brief Get the singleton SDCard instance
     */
    static SDCard& instance();

    /**
     * @brief Initialize the SD card with default configuration
     * @return CUBE32_OK on success
     */
    cube32_result_t begin();

    /**
     * @brief Initialize the SD card with custom configuration
     * @param config SD card configuration
     * @return CUBE32_OK on success
     */
    cube32_result_t begin(const cube32_sdcard_config_t& config);

    /**
     * @brief Unmount and deinitialize the SD card
     * @return CUBE32_OK on success
     */
    cube32_result_t end();

    // ========================================================================
    // Software Card Detection (no CD pin required)
    // ========================================================================

    /**
     * @brief Force cleanup of SD card state
     * 
     * This method forcibly cleans up all SD card state, ignoring errors.
     * Use when card removal is detected or I/O errors occur.
     * After calling this, checkCardStatus() can re-initialize the card.
     */
    void forceCleanup();

    /**
     * @brief Probe the card by attempting filesystem access
     * 
     * This method attempts to open the root directory and read entries
     * to verify the card is physically present and accessible. Useful when
     * no hardware card detect pin is available.
     * 
     * @return true if card is accessible, false if card was removed
     */
    bool probeCard();

    /**
     * @brief Check card status and handle removal/insertion
     * 
     * This method should be called periodically to detect card
     * removal or insertion. It will:
     * - If card was mounted and is now removed: unmount and deinit
     * - If card was not mounted and is now inserted: init and mount
     * 
     * @return true if card is currently present and mounted
     */
    bool checkCardStatus();

    /**
     * @brief Check if card was detected (last known state)
     * @return true if card was detected in last check
     */
    bool wasCardDetected() const { return m_cardPresent; }

    /**
     * @brief Check if SD card is initialized and mounted
     */
    bool isInitialized() const { return m_initialized && m_mounted; }

    /**
     * @brief Check if SD card is mounted
     */
    bool isMounted() const { return m_mounted; }

    /**
     * @brief Get SD card status information
     * @return Status structure with card info
     */
    cube32_sdcard_status_t getStatus() const;

    /**
     * @brief Get the mount point path
     */
    const char* getMountPoint() const { return m_config.mount_point; }

    /**
     * @brief Get full path with mount point prepended
     * @param path Relative or absolute path
     * @return Full path string with mount point
     */
    std::string getFullPath(const char* path) const { return buildPath(path); }

    /**
     * @brief Get the underlying sdmmc_card_t structure
     * @return Pointer to card structure, or nullptr if not mounted
     */
    sdmmc_card_t* getCard() { return m_card; }

    // ========================================================================
    // Directory Operations
    // ========================================================================

    /**
     * @brief List contents of a directory
     * @param path Directory path (relative to mount point or absolute)
     * @param entries Output vector of directory entries
     * @param maxEntries Maximum entries to return (0 = unlimited)
     * @return CUBE32_OK on success
     */
    cube32_result_t listDirectory(const char* path, 
                                   std::vector<cube32_sdcard_entry_t>& entries,
                                   size_t maxEntries = 0);

    /**
     * @brief Create a directory
     * @param path Directory path
     * @return CUBE32_OK on success
     */
    cube32_result_t createDirectory(const char* path);

    /**
     * @brief Remove a directory (must be empty)
     * @param path Directory path
     * @return CUBE32_OK on success
     */
    cube32_result_t removeDirectory(const char* path);

    /**
     * @brief Check if path exists
     * @param path Path to check
     * @return true if exists
     */
    bool exists(const char* path);

    /**
     * @brief Check if path is a directory
     * @param path Path to check
     * @return true if directory
     */
    bool isDirectory(const char* path);

    // ========================================================================
    // File Operations
    // ========================================================================

    /**
     * @brief Read entire file contents into string
     * @param path File path
     * @param content Output string for file content
     * @return CUBE32_OK on success
     */
    cube32_result_t readFile(const char* path, std::string& content);

    /**
     * @brief Read file contents into buffer
     * @param path File path
     * @param buffer Output buffer
     * @param size Buffer size / bytes to read
     * @param bytesRead Actual bytes read
     * @return CUBE32_OK on success
     */
    cube32_result_t readFile(const char* path, uint8_t* buffer, 
                              size_t size, size_t* bytesRead);

    /**
     * @brief Write string to file
     * @param path File path
     * @param content Content to write
     * @param append Append to existing file if true
     * @return CUBE32_OK on success
     */
    cube32_result_t writeFile(const char* path, const std::string& content,
                               bool append = false);

    /**
     * @brief Write buffer to file
     * @param path File path
     * @param buffer Data buffer
     * @param size Buffer size
     * @param append Append to existing file if true
     * @return CUBE32_OK on success
     */
    cube32_result_t writeFile(const char* path, const uint8_t* buffer,
                               size_t size, bool append = false);

    /**
     * @brief Delete a file
     * @param path File path
     * @return CUBE32_OK on success
     */
    cube32_result_t deleteFile(const char* path);

    /**
     * @brief Rename/move a file
     * @param oldPath Current path
     * @param newPath New path
     * @return CUBE32_OK on success
     */
    cube32_result_t renameFile(const char* oldPath, const char* newPath);

    /**
     * @brief Get file size
     * @param path File path
     * @return File size in bytes, or -1 on error
     */
    int64_t getFileSize(const char* path);

    /**
     * @brief Print card info to log
     */
    void printCardInfo() const;

private:
    SDCard() = default;
    ~SDCard();
    SDCard(const SDCard&) = delete;
    SDCard& operator=(const SDCard&) = delete;

    /**
     * @brief Build full path with mount point
     */
    std::string buildPath(const char* path) const;

    cube32_sdcard_config_t m_config = CUBE32_SDCARD_CONFIG_DEFAULT();
    sdmmc_card_t* m_card = nullptr;
    bool m_initialized = false;
    bool m_mounted = false;
    bool m_cardPresent = false;  ///< Last known card presence state
};

} // namespace cube32

#endif // __cplusplus

#endif // CUBE32_DRIVERS_SDCARD_SDCARD_H
