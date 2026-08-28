/**
 * @file sdcard.cpp
 * @brief CUBE32 SD Card Driver Implementation (SDMMC + SPI)
 *
 * Supports automatic interface detection:
 *   - ES8311 @ 0x19 (S3 Audio integrated) → SPI mode on SPI2_HOST, CS=GPIO5
 *   - All other boards                     → SDMMC mode (CMD/CLK/D0)
 */

#include "drivers/sdcard/sdcard.h"
#include "utils/hw_manifest.h"

#include <esp_log.h>
#include <esp_vfs_fat.h>
#include <sdmmc_cmd.h>
#include <driver/sdmmc_host.h>
#include <driver/sdspi_host.h>
#include <sys/stat.h>
#include <sys/unistd.h>
#include <dirent.h>
#include <cstring>
#include <cstdio>
#include <cerrno>
#include <esp_timer.h>
#include <esp_heap_caps.h>

static const char* TAG = "cube32_sdcard";

namespace cube32 {

// ============================================================================
// Singleton Implementation
// ============================================================================

SDCard& SDCard::instance() {
    static SDCard s_instance;
    return s_instance;
}

SDCard::~SDCard() {
    if (m_initialized) {
        end();
    }
}

// ============================================================================
// Initialization
// ============================================================================

cube32_result_t SDCard::begin() {
    cube32_sdcard_config_t config = CUBE32_SDCARD_CONFIG_DEFAULT();
    return begin(config);
}

cube32_result_t SDCard::begin(const cube32_sdcard_config_t& config) {
    if (m_initialized) {
        ESP_LOGW(TAG, "SD card already initialized");
        return CUBE32_ALREADY_INITIALIZED;
    }

    m_config = config;

    // ---- Resolve AUTO interface selection --------------------------------
    cube32_sdcard_iface_t iface = config.iface;
    if (iface == CUBE32_SDCARD_IFACE_AUTO) {
        const cube32_hw_manifest_t* m = cube32_hw_manifest();
        if (m->scanned &&
            m->core_module == CUBE32_CORE_MODULE_S3_AUDIO) {
            iface = CUBE32_SDCARD_IFACE_SPI;
            ESP_LOGI(TAG, "Auto-detect: Core=S3 Audio 2-in-1 → SPI SD (shared SPI2, CS=GPIO%d)",
                     (int)config.pin_cs);
        } else {
            iface = CUBE32_SDCARD_IFACE_SDMMC;
            ESP_LOGI(TAG, "Auto-detect: SDMMC SD (CMD=GPIO%d CLK=GPIO%d D0=GPIO%d)",
                     (int)config.pin_cmd, (int)config.pin_clk, (int)config.pin_d0);
        }
    }

    cube32_result_t ret = (iface == CUBE32_SDCARD_IFACE_SPI)
                          ? mountSpi(config)
                          : mountSdmmc(config);

    if (ret == CUBE32_OK) {
        m_iface = iface;
    }
    return ret;
}

// ============================================================================
// Private: SDMMC mount
// ============================================================================

cube32_result_t SDCard::mountSdmmc(const cube32_sdcard_config_t& config) {
    ESP_LOGI(TAG, "Initializing SD card (SDMMC mode)...");
    ESP_LOGI(TAG, "  CMD: GPIO%d, CLK: GPIO%d, D0: GPIO%d",
             config.pin_cmd, config.pin_clk, config.pin_d0);
    ESP_LOGI(TAG, "  Bus width: %d-bit, Max freq: %lu kHz",
             config.bus_width, (unsigned long)config.max_freq_khz);
    ESP_LOGI(TAG, "  Mount point: %s", config.mount_point);

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = config.max_freq_khz;

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.clk  = config.pin_clk;
    slot_config.cmd  = config.pin_cmd;
    slot_config.d0   = config.pin_d0;

    if (config.bus_width >= 4) {
        slot_config.d1    = config.pin_d1;
        slot_config.d2    = config.pin_d2;
        slot_config.d3    = config.pin_d3;
        slot_config.width = 4;
    } else {
        slot_config.width = 1;
    }

    if (config.pin_cd != GPIO_NUM_NC) slot_config.cd = config.pin_cd;
    if (config.pin_wp != GPIO_NUM_NC) slot_config.wp = config.pin_wp;

    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {};
    mount_config.format_if_mount_failed  = config.format_if_mount_failed;
    mount_config.max_files               = config.max_files;
    mount_config.allocation_unit_size    = 16 * 1024;
    mount_config.disk_status_check_enable = false;

    esp_err_t ret = esp_vfs_fat_sdmmc_mount(
        config.mount_point, &host, &slot_config, &mount_config, &m_card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                     "Set format_if_mount_failed=true to format.");
        } else if (ret == ESP_ERR_NO_MEM) {
            ESP_LOGE(TAG, "Failed to allocate memory for SD card");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Check pull-up resistors.", esp_err_to_name(ret));
        }
        return esp_err_to_cube32(ret);
    }

    m_initialized = m_mounted = m_cardPresent = true;
    m_iface = CUBE32_SDCARD_IFACE_SDMMC;   // set before printCardInfo()
    ESP_LOGI(TAG, "SD card mounted successfully (SDMMC)");
    printCardInfo();
    return CUBE32_OK;
}

// ============================================================================
// Private: SPI mount
// ============================================================================

cube32_result_t SDCard::mountSpi(const cube32_sdcard_config_t& config) {
    ESP_LOGI(TAG, "Initializing SD card (SPI mode)...");
    ESP_LOGI(TAG, "  SPI%d, CS=GPIO%d, Max freq: %lu kHz",
             (int)config.spi_host + 1, (int)config.pin_cs,
             (unsigned long)config.spi_freq_khz);
    ESP_LOGI(TAG, "  Mount point: %s", config.mount_point);

    // SPI2_HOST was already initialised by SPIBus::init() for the TFT display.
    // sdspi_host_init() only initialises the sdspi software layer — it does NOT
    // call spi_bus_initialize() — so it is safe to call on a running shared bus.
    // sdspi_host_init_device() then calls spi_bus_add_device() to register the
    // SD card as a second device alongside the LCD.
    sdmmc_host_t host    = SDSPI_HOST_DEFAULT();
    host.slot            = config.spi_host;
    host.max_freq_khz    = config.spi_freq_khz;

    sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot.gpio_cs = config.pin_cs;
    slot.host_id = config.spi_host;

    esp_vfs_fat_mount_config_t mnt = {};
    mnt.format_if_mount_failed  = config.format_if_mount_failed;
    mnt.max_files               = config.max_files;
    mnt.allocation_unit_size    = 16 * 1024;

    esp_err_t ret = esp_vfs_fat_sdspi_mount(
        config.mount_point, &host, &slot, &mnt, &m_card);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI SD mount failed: %s", esp_err_to_name(ret));
        return esp_err_to_cube32(ret);
    }

    m_initialized = m_mounted = m_cardPresent = true;
    m_iface = CUBE32_SDCARD_IFACE_SPI;     // set before printCardInfo()
    ESP_LOGI(TAG, "SD card mounted successfully (SPI)");
    printCardInfo();
    return CUBE32_OK;
}

// ============================================================================
// Private: interface-aware unmount helper
// ============================================================================

void SDCard::unmountInternal() {
    if (!m_mounted || m_card == nullptr) {
        return;
    }

    // esp_vfs_fat_sdcard_unmount handles both SDMMC and SPI:
    //   • calls host.deinit_p (→ sdmmc_host_deinit or sdspi_host_remove_device)
    //   • unmounts FATFS and unregisters the VFS path
    //   • frees the sdmmc_card_t struct (m_card becomes dangling — zero it after)
    esp_err_t err = esp_vfs_fat_sdcard_unmount(m_config.mount_point, m_card);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Unmount returned: %s", esp_err_to_name(err));
    }
    m_card    = nullptr;
    m_mounted = false;

    // NOTE: sdspi_host_deinit() is intentionally NOT called here.
    //
    // esp_vfs_fat_sdcard_unmount() already called sdspi_host_remove_device()
    // (via card->host.deinit_p), which removes the SD card's SPI device entry
    // from the bus.  The sdspi software layer (a single FreeRTOS mutex) is kept
    // alive so that hot-swap re-insertion works:
    //   • sdspi_host_init()        → no-op (mutex already exists)
    //   • sdspi_host_init_device() → registers a fresh device handle
    // Calling sdspi_host_deinit() here destroys the mutex, which means the next
    // sdspi_host_init() must recreate it; this re-create / re-add sequence has
    // an ordering issue in ESP-IDF v5.5 that prevents the second mount from
    // completing successfully.
    //
    // spi_bus_free() is also NOT called — the bus is shared with the TFT display.
    // sdspi_host_deinit() is called only in end() for explicit final shutdown.
}

// ============================================================================
// Deinitialization
// ============================================================================

cube32_result_t SDCard::end() {
    if (!m_initialized) {
        return CUBE32_NOT_INITIALIZED;
    }

    // Save interface before unmountInternal() might indirectly change flags.
    cube32_sdcard_iface_t iface_was = m_iface;

    unmountInternal();

    // For SPI mode, call sdspi_host_deinit() here (final shutdown only).
    // unmountInternal() deliberately skips this so that hot-swap re-init works.
    if (iface_was == CUBE32_SDCARD_IFACE_SPI) {
        sdspi_host_deinit();
    }

    m_initialized  = false;
    m_cardPresent  = false;
    m_iface        = CUBE32_SDCARD_IFACE_SDMMC;

    ESP_LOGI(TAG, "SD card unmounted");
    return CUBE32_OK;
}

// ============================================================================
// Software Card Detection
// ============================================================================

void SDCard::forceCleanup() {
    ESP_LOGW(TAG, "Force cleanup of SD card state");
    unmountInternal();  // handles both SDMMC and SPI, ignores errors internally
    m_initialized = false;
    m_cardPresent = false;
    m_iface       = CUBE32_SDCARD_IFACE_SDMMC;
    ESP_LOGI(TAG, "SD card state cleaned up");
}

bool SDCard::probeCard() {
    if (!m_initialized || !m_mounted || m_card == nullptr) {
        return false;
    }

    // Use sdmmc_get_status() to directly check card presence at hardware level
    // This is more reliable than filesystem operations which may use cached data
    esp_err_t ret = sdmmc_get_status(m_card);
    
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "probeCard: sdmmc_get_status failed (%s)", esp_err_to_name(ret));
        return false;  // Card communication failed - card likely removed
    }
    
    return true;
}

bool SDCard::checkCardStatus() {
    bool currentCardPresent = false;
    
    if (m_initialized && m_mounted) {
        // Card was initialized, check if it's still present
        currentCardPresent = probeCard();
        
        if (!currentCardPresent && m_cardPresent) {
            // Card was removed or I/O error occurred
            ESP_LOGW(TAG, "Card removed or I/O error, cleaning up...");
            forceCleanup();
            return false;
        } else if (currentCardPresent) {
            // Card still present
            m_cardPresent = true;
        }
    } else {
        // Card not initialized, try to detect and initialize
        // First ensure clean state
        if (m_card != nullptr || m_mounted) {
            ESP_LOGD(TAG, "Cleaning up stale state before re-init...");
            forceCleanup();
        }
        
        ESP_LOGD(TAG, "Attempting to detect and initialize card...");
        
        cube32_result_t result = begin(m_config);
        
        if (result == CUBE32_OK) {
            // Card newly detected
            ESP_LOGI(TAG, "Card newly detected and initialized");
            m_cardPresent = true;
            currentCardPresent = true;
        } else if (result == CUBE32_ALREADY_INITIALIZED) {
            // This shouldn't happen after forceCleanup, but handle it
            ESP_LOGW(TAG, "Unexpected ALREADY_INITIALIZED, forcing cleanup");
            forceCleanup();
            m_cardPresent = false;
            currentCardPresent = false;
        } else {
            m_cardPresent = false;
            currentCardPresent = false;
        }
    }
    
    return m_cardPresent;
}

// ============================================================================
// Status and Information
// ============================================================================

cube32_sdcard_status_t SDCard::getStatus() const {
    cube32_sdcard_status_t status = {};
    
    status.mounted = m_mounted;
    status.present = m_card != nullptr;

    if (!m_mounted || m_card == nullptr) {
        status.card_type = "None";
        status.speed_mode = "N/A";
        return status;
    }

    // Card type - check for high capacity flag (bit 30 in OCR)
    const uint32_t OCR_SDHC_FLAG = (1 << 30);  // CCS bit indicates SDHC/SDXC
    if (m_card->ocr & OCR_SDHC_FLAG) {
        if (m_card->csd.capacity > 67108864) {  // > 32GB
            status.card_type = "SDXC";
        } else {
            status.card_type = "SDHC";
        }
    } else if (m_card->is_mmc) {
        status.card_type = "MMC";
    } else {
        status.card_type = "SD";
    }

    // Speed mode
    if (m_card->max_freq_khz >= 40000) {
        status.speed_mode = "High Speed";
    } else {
        status.speed_mode = "Default Speed";
    }

    // Size information
    status.sector_size = m_card->csd.sector_size;
    status.sector_count = m_card->csd.capacity;
    status.total_bytes = (uint64_t)m_card->csd.sector_size * m_card->csd.capacity;
    status.max_freq_khz = m_card->max_freq_khz;
    // Bus width: SPI is always 1-bit serial; SDMMC uses log_bus_width
    if (m_iface == CUBE32_SDCARD_IFACE_SPI) {
        status.bus_width = 1;
    } else {
        status.bus_width = m_card->log_bus_width ? (1 << m_card->log_bus_width) : 1;
    }

    // Get filesystem statistics
    FATFS* fs;
    DWORD free_clusters;
    char drv[4];
    snprintf(drv, sizeof(drv), "%d:", 0);
    
    if (f_getfree(drv, &free_clusters, &fs) == FR_OK) {
        uint64_t total_sectors = (fs->n_fatent - 2) * fs->csize;
        uint64_t free_sectors = free_clusters * fs->csize;
        status.total_bytes = total_sectors * fs->ssize;
        status.free_bytes = free_sectors * fs->ssize;
        status.used_bytes = status.total_bytes - status.free_bytes;
    }

    return status;
}

void SDCard::printCardInfo() const {
    if (m_card == nullptr) {
        ESP_LOGI(TAG, "No card information available");
        return;
    }

    auto status = getStatus();
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "SD Card Information:");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Interface: %s", getInterfaceName());
    ESP_LOGI(TAG, "Name: %s", m_card->cid.name);
    ESP_LOGI(TAG, "Card Type: %s", status.card_type);
    ESP_LOGI(TAG, "Speed Mode: %s", status.speed_mode);
    ESP_LOGI(TAG, "Bus Width: %d-bit", status.bus_width);
    ESP_LOGI(TAG, "Max Frequency: %lu kHz", (unsigned long)status.max_freq_khz);
    ESP_LOGI(TAG, "Total Size: %lu MB", (unsigned long)(status.total_bytes / (1024 * 1024)));
    ESP_LOGI(TAG, "Free Space: %lu MB", (unsigned long)(status.free_bytes / (1024 * 1024)));
    ESP_LOGI(TAG, "Used Space: %lu MB", (unsigned long)(status.used_bytes / (1024 * 1024)));
    ESP_LOGI(TAG, "========================================");
}

// ============================================================================
// Path Utilities
// ============================================================================

std::string SDCard::buildPath(const char* path) const {
    if (path == nullptr || path[0] == '\0') {
        return std::string(m_config.mount_point);
    }

    std::string fullPath;
    
    // If path starts with mount point, use as-is
    if (strncmp(path, m_config.mount_point, strlen(m_config.mount_point)) == 0) {
        fullPath = path;
    }
    // If path starts with /, append to mount point
    else if (path[0] == '/') {
        fullPath = std::string(m_config.mount_point) + path;
    }
    // Otherwise, append with /
    else {
        fullPath = std::string(m_config.mount_point) + "/" + path;
    }

    return fullPath;
}

// ============================================================================
// Directory Operations
// ============================================================================

cube32_result_t SDCard::listDirectory(const char* path, 
                                       std::vector<cube32_sdcard_entry_t>& entries,
                                       size_t maxEntries) {
    if (!m_mounted) {
        return CUBE32_NOT_INITIALIZED;
    }

    entries.clear();

    std::string fullPath = buildPath(path);
    DIR* dir = opendir(fullPath.c_str());
    if (dir == nullptr) {
        ESP_LOGE(TAG, "Failed to open directory: %s", fullPath.c_str());
        return CUBE32_IO_ERROR;
    }

    struct dirent* entry;
    size_t count = 0;

    while ((entry = readdir(dir)) != nullptr) {
        if (maxEntries > 0 && count >= maxEntries) {
            break;
        }

        cube32_sdcard_entry_t e = {};
        strncpy(e.name, entry->d_name, sizeof(e.name) - 1);
        e.is_directory = (entry->d_type == DT_DIR);

        // Get file size for regular files
        if (!e.is_directory) {
            std::string filePath = fullPath + "/" + entry->d_name;
            struct stat st;
            if (stat(filePath.c_str(), &st) == 0) {
                e.size = st.st_size;
            }
        }

        entries.push_back(e);
        count++;
    }

    closedir(dir);
    return CUBE32_OK;
}

cube32_result_t SDCard::createDirectory(const char* path) {
    if (!m_mounted) {
        return CUBE32_NOT_INITIALIZED;
    }

    std::string fullPath = buildPath(path);
    
    if (mkdir(fullPath.c_str(), 0775) != 0) {
        if (errno == EEXIST) {
            return CUBE32_ALREADY_INITIALIZED;
        }
        ESP_LOGE(TAG, "Failed to create directory: %s (%s)", 
                 fullPath.c_str(), strerror(errno));
        return CUBE32_IO_ERROR;
    }

    return CUBE32_OK;
}

cube32_result_t SDCard::removeDirectory(const char* path) {
    if (!m_mounted) {
        return CUBE32_NOT_INITIALIZED;
    }

    std::string fullPath = buildPath(path);
    
    if (rmdir(fullPath.c_str()) != 0) {
        ESP_LOGE(TAG, "Failed to remove directory: %s (%s)", 
                 fullPath.c_str(), strerror(errno));
        return CUBE32_IO_ERROR;
    }

    return CUBE32_OK;
}

bool SDCard::exists(const char* path) {
    if (!m_mounted) {
        return false;
    }

    std::string fullPath = buildPath(path);
    struct stat st;
    return (stat(fullPath.c_str(), &st) == 0);
}

bool SDCard::isDirectory(const char* path) {
    if (!m_mounted) {
        return false;
    }

    std::string fullPath = buildPath(path);
    struct stat st;
    if (stat(fullPath.c_str(), &st) != 0) {
        return false;
    }
    return S_ISDIR(st.st_mode);
}

// ============================================================================
// File Operations
// ============================================================================

cube32_result_t SDCard::readFile(const char* path, std::string& content) {
    if (!m_mounted) {
        return CUBE32_NOT_INITIALIZED;
    }

    std::string fullPath = buildPath(path);
    
    FILE* f = fopen(fullPath.c_str(), "rb");
    if (f == nullptr) {
        ESP_LOGE(TAG, "Failed to open file for reading: %s", fullPath.c_str());
        return CUBE32_IO_ERROR;
    }

    // Get file size
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (size <= 0) {
        fclose(f);
        content.clear();
        return CUBE32_OK;
    }

    // Read content
    content.resize(size);
    size_t read = fread(&content[0], 1, size, f);
    fclose(f);

    if (read != (size_t)size) {
        ESP_LOGW(TAG, "Read %u bytes, expected %ld", (unsigned)read, size);
        content.resize(read);
    }

    return CUBE32_OK;
}

cube32_result_t SDCard::readFile(const char* path, uint8_t* buffer, 
                                  size_t size, size_t* bytesRead) {
    if (!m_mounted) {
        return CUBE32_NOT_INITIALIZED;
    }
    if (buffer == nullptr || bytesRead == nullptr) {
        return CUBE32_INVALID_ARG;
    }

    *bytesRead = 0;

    std::string fullPath = buildPath(path);
    
    FILE* f = fopen(fullPath.c_str(), "rb");
    if (f == nullptr) {
        ESP_LOGE(TAG, "Failed to open file for reading: %s", fullPath.c_str());
        return CUBE32_IO_ERROR;
    }

    *bytesRead = fread(buffer, 1, size, f);
    fclose(f);

    return CUBE32_OK;
}

cube32_result_t SDCard::writeFile(const char* path, const std::string& content,
                                   bool append) {
    return writeFile(path, (const uint8_t*)content.data(), content.size(), append);
}

cube32_result_t SDCard::writeFile(const char* path, const uint8_t* buffer,
                                   size_t size, bool append) {
    if (!m_mounted) {
        return CUBE32_NOT_INITIALIZED;
    }
    if (buffer == nullptr && size > 0) {
        return CUBE32_INVALID_ARG;
    }

    std::string fullPath = buildPath(path);
    
    FILE* f = fopen(fullPath.c_str(), append ? "ab" : "wb");
    if (f == nullptr) {
        ESP_LOGE(TAG, "Failed to open file for writing: %s", fullPath.c_str());
        return CUBE32_IO_ERROR;
    }

    if (size > 0) {
        size_t written = fwrite(buffer, 1, size, f);
        fclose(f);

        if (written != size) {
            ESP_LOGE(TAG, "Write failed: wrote %u of %u bytes", (unsigned)written, (unsigned)size);
            return CUBE32_IO_ERROR;
        }
    } else {
        fclose(f);
    }

    return CUBE32_OK;
}

cube32_result_t SDCard::deleteFile(const char* path) {
    if (!m_mounted) {
        return CUBE32_NOT_INITIALIZED;
    }

    std::string fullPath = buildPath(path);
    
    if (unlink(fullPath.c_str()) != 0) {
        ESP_LOGE(TAG, "Failed to delete file: %s (%s)", 
                 fullPath.c_str(), strerror(errno));
        return CUBE32_IO_ERROR;
    }

    return CUBE32_OK;
}

cube32_result_t SDCard::renameFile(const char* oldPath, const char* newPath) {
    if (!m_mounted) {
        return CUBE32_NOT_INITIALIZED;
    }

    std::string fullOldPath = buildPath(oldPath);
    std::string fullNewPath = buildPath(newPath);
    
    if (rename(fullOldPath.c_str(), fullNewPath.c_str()) != 0) {
        ESP_LOGE(TAG, "Failed to rename file: %s -> %s (%s)", 
                 fullOldPath.c_str(), fullNewPath.c_str(), strerror(errno));
        return CUBE32_IO_ERROR;
    }

    return CUBE32_OK;
}

int64_t SDCard::getFileSize(const char* path) {
    if (!m_mounted) {
        return -1;
    }

    std::string fullPath = buildPath(path);
    struct stat st;
    
    if (stat(fullPath.c_str(), &st) != 0) {
        return -1;
    }

    return st.st_size;
}

// ============================================================================
// Speed test
// ============================================================================

cube32_sdcard_speed_result_t SDCard::runSpeedTest(size_t testSize, size_t bufferSize) {
    cube32_sdcard_speed_result_t result = {};

    if (!m_mounted) {
        ESP_LOGE(TAG, "runSpeedTest: not mounted");
        return result;
    }

    const char* testPath = "/spdtest.tmp";  // 8.3 name for max FAT compat

    // ---- allocate write buffer with known pattern ----
    uint8_t* buf = static_cast<uint8_t*>(
        heap_caps_malloc(bufferSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    if (buf == nullptr) {
        ESP_LOGE(TAG, "runSpeedTest: malloc failed");
        return result;
    }
    for (size_t i = 0; i < bufferSize; i++) {
        buf[i] = static_cast<uint8_t>(i & 0xFFu);
    }

    // ---- WRITE ----
    ESP_LOGI(TAG, "runSpeedTest: writing %u KB...", (unsigned)(testSize / 1024));
    std::string fullPath = buildPath(testPath);
    FILE* f = fopen(fullPath.c_str(), "wb");
    if (!f) {
        heap_caps_free(buf);
        ESP_LOGE(TAG, "runSpeedTest: fopen write failed");
        return result;
    }

    int64_t t0 = esp_timer_get_time();
    size_t written = 0;
    while (written < testSize) {
        size_t to_write = ((testSize - written) > bufferSize)
                          ? bufferSize : (testSize - written);
        if (fwrite(buf, 1, to_write, f) != to_write) {
            fclose(f);
            heap_caps_free(buf);
            deleteFile(testPath);
            ESP_LOGE(TAG, "runSpeedTest: write error at %u", (unsigned)written);
            return result;
        }
        written += to_write;
    }
    fclose(f);
    int64_t t_write_us = esp_timer_get_time() - t0;

    // ---- READ ----
    ESP_LOGI(TAG, "runSpeedTest: reading back...");
    f = fopen(fullPath.c_str(), "rb");
    if (!f) {
        heap_caps_free(buf);
        deleteFile(testPath);
        ESP_LOGE(TAG, "runSpeedTest: fopen read failed");
        return result;
    }

    t0 = esp_timer_get_time();
    size_t bytes_read = 0;
    while (bytes_read < testSize) {
        size_t to_read = ((testSize - bytes_read) > bufferSize)
                         ? bufferSize : (testSize - bytes_read);
        size_t n = fread(buf, 1, to_read, f);
        if (n == 0) break;
        bytes_read += n;
    }
    fclose(f);
    int64_t t_read_us = esp_timer_get_time() - t0;

    heap_caps_free(buf);
    deleteFile(testPath);

    result.success      = true;
    result.bytes_tested = static_cast<uint32_t>(testSize);
    result.write_time_ms = static_cast<uint32_t>(t_write_us / 1000);
    result.read_time_ms  = static_cast<uint32_t>(t_read_us  / 1000);
    if (t_write_us > 0) {
        result.write_speed_kbps = static_cast<float>(testSize) / (t_write_us / 1000.0f);
    }
    if (t_read_us > 0) {
        result.read_speed_kbps = static_cast<float>(testSize) / (t_read_us / 1000.0f);
    }

    ESP_LOGI(TAG, "runSpeedTest: W=%.1f KB/s (%lu ms)  R=%.1f KB/s (%lu ms)",
             result.write_speed_kbps, (unsigned long)result.write_time_ms,
             result.read_speed_kbps,  (unsigned long)result.read_time_ms);
    return result;
}

} // namespace cube32
