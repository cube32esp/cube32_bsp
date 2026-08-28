/**
 * @file sdcard.h
 * @brief CUBE32 SD Card Driver (SDMMC + SPI Interface)
 * 
 * This driver provides support for SD cards using either the SDMMC peripheral
 * or the SPI peripheral (shared with the TFT display on SPI2_HOST).
 * 
 * Interface selection:
 *   AUTO (default) — reads the hardware manifest at runtime:
 *     • ES8311 @ 0x19 detected  → CUBE32 S3 Audio (integrated) → SPI mode
 *                                  (shared SPI2_HOST, CS = GPIO 5)
 *     • Otherwise               → SDMMC mode (CMD/CLK/D0 = GPIO 7/15/4)
 *   SDMMC / SPI  — forced selection, ignores hardware manifest
 * 
 * Features:
 * - SD card initialization and mounting with FATFS
 * - Card status information (size, type, speed, interface)
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
#include <driver/sdspi_host.h>

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
 * @brief SD card interface selection
 */
typedef enum {
    CUBE32_SDCARD_IFACE_AUTO  = 0, ///< Auto-detect from hardware manifest at begin()
    CUBE32_SDCARD_IFACE_SDMMC = 1, ///< SDMMC peripheral (CMD/CLK/D0 pins)
    CUBE32_SDCARD_IFACE_SPI   = 2, ///< SPI peripheral shared with TFT (SPI2_HOST, CS=GPIO5)
} cube32_sdcard_iface_t;

/**
 * @brief SD card configuration structure
 */
typedef struct {
    /* SDMMC fields */
    gpio_num_t pin_cmd;              ///< CMD pin (SDMMC mode)
    gpio_num_t pin_clk;              ///< CLK pin (SDMMC mode)
    gpio_num_t pin_d0;               ///< D0 pin (SDMMC mode, 1-bit and 4-bit)
    gpio_num_t pin_d1;               ///< D1 pin (SDMMC mode, 4-bit only)
    gpio_num_t pin_d2;               ///< D2 pin (SDMMC mode, 4-bit only)
    gpio_num_t pin_d3;               ///< D3 pin (SDMMC mode, 4-bit only)
    gpio_num_t pin_cd;               ///< Card detect pin (GPIO_NUM_NC if not used)
    gpio_num_t pin_wp;               ///< Write protect pin (GPIO_NUM_NC if not used)
    uint8_t bus_width;               ///< Bus width (1 or 4) for SDMMC mode
    uint32_t max_freq_khz;           ///< Maximum SDMMC clock frequency in kHz
    /* Common fields */
    const char* mount_point;         ///< VFS mount point path
    bool format_if_mount_failed;     ///< Format card if mount fails
    int max_files;                   ///< Maximum number of simultaneously open files
    /* Interface selection */
    cube32_sdcard_iface_t iface;     ///< Interface: AUTO (default), SDMMC, or SPI
    /* SPI-mode fields (used when iface == CUBE32_SDCARD_IFACE_SPI or AUTO→SPI) */
    spi_host_device_t spi_host;      ///< SPI host device (e.g. SPI2_HOST, shared with TFT)
    gpio_num_t pin_cs;               ///< SPI chip-select GPIO
    uint32_t spi_freq_khz;           ///< SPI clock frequency in kHz
} cube32_sdcard_config_t;

/**
 * @brief Default SD card configuration using pins from cube32_config.h
 *
 * Interface defaults to AUTO: the driver detects the board at begin() by
 * querying the hardware manifest and selects SDMMC or SPI automatically.
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
    .iface = CUBE32_SDCARD_IFACE_AUTO, \
    .spi_host = CUBE32_SD_SPI_HOST, \
    .pin_cs = CUBE32_SD_SPI_CS_PIN, \
    .spi_freq_khz = CUBE32_SD_SPI_FREQ_KHZ, \
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
 * @brief SD Card Driver Class (SDMMC + SPI Interface, auto-detected)
 * 
 * Object-oriented interface for SD card operations.  The physical interface
 * is selected automatically from the hardware manifest (see sdcard.h header
 * for the detection rules), or can be forced via begin(config).
 * 
 * Usage:
 * @code
 *   cube32::SDCard& sd = cube32::SDCard::instance();
 *   if (sd.begin() == CUBE32_OK) {
 *       auto status = sd.getStatus();
 *       ESP_LOGI(TAG, "Card: %s via %s, %lu MB",
 *                status.card_type, sd.getInterfaceName(),
 *                (unsigned long)(status.total_bytes / (1024*1024)));
 *       
 *       std::vector<cube32_sdcard_entry_t> entries;
 *       sd.listDirectory("/", entries);
 *       for (const auto& e : entries) {
 *           ESP_LOGI(TAG, "%s %s (%lu B)",
 *                    e.is_directory ? "[DIR]" : "[FILE]",
 *                    e.name, (unsigned long)e.size);
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
     * @brief Get the active physical interface
     * @return CUBE32_SDCARD_IFACE_SDMMC or CUBE32_SDCARD_IFACE_SPI
     */
    cube32_sdcard_iface_t getInterface() const { return m_iface; }

    /**
     * @brief Get the active interface as a human-readable string
     * @return "SDMMC" or "SPI"
     */
    const char* getInterfaceName() const {
        return (m_iface == CUBE32_SDCARD_IFACE_SPI) ? "SPI" : "SDMMC";
    }

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
     * @brief Run a read/write speed test
     *
     * Writes @p testSize bytes of known-pattern data to a temporary file,
     * reads it back, calculates throughput, deletes the file, and returns
     * the results.
     *
     * @param testSize    Total bytes to transfer (default 256 KB)
     * @param bufferSize  I/O chunk size in bytes (default 4 KB)
     * @return Speed test result struct
     */
    cube32_sdcard_speed_result_t runSpeedTest(
        size_t testSize   = CUBE32_SDCARD_TEST_FILE_SIZE,
        size_t bufferSize = CUBE32_SDCARD_TEST_BUFFER_SIZE);

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

    /**
     * @brief Mount using SDMMC peripheral
     */
    cube32_result_t mountSdmmc(const cube32_sdcard_config_t& config);

    /**
     * @brief Mount using SPI peripheral (shared SPI2_HOST)
     */
    cube32_result_t mountSpi(const cube32_sdcard_config_t& config);

    /**
     * @brief Perform interface-specific unmount and host cleanup
     */
    void unmountInternal();

    cube32_sdcard_config_t  m_config       = CUBE32_SDCARD_CONFIG_DEFAULT();
    sdmmc_card_t*           m_card         = nullptr;
    bool                    m_initialized  = false;
    bool                    m_mounted      = false;
    bool                    m_cardPresent  = false;
    cube32_sdcard_iface_t   m_iface        = CUBE32_SDCARD_IFACE_SDMMC;
};

} // namespace cube32

#endif // __cplusplus

#endif // CUBE32_DRIVERS_SDCARD_SDCARD_H
