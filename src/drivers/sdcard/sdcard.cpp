/**
 * @file sdcard.cpp
 * @brief CUBE32 SD Card Driver Implementation
 * 
 * This implementation uses the SDMMC peripheral and ESP-IDF's FATFS
 * for file system operations.
 */

#include "drivers/sdcard/sdcard.h"

#include <esp_log.h>
#include <esp_vfs_fat.h>
#include <sdmmc_cmd.h>
#include <driver/sdmmc_host.h>
#include <sys/stat.h>
#include <sys/unistd.h>
#include <dirent.h>
#include <cstring>
#include <cstdio>
#include <cerrno>
#include <esp_timer.h>

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

    // Store configuration
    m_config = config;

    ESP_LOGI(TAG, "Initializing SD card (SDMMC mode)...");
    ESP_LOGI(TAG, "  CMD: GPIO%d, CLK: GPIO%d, D0: GPIO%d", 
             config.pin_cmd, config.pin_clk, config.pin_d0);
    ESP_LOGI(TAG, "  Bus width: %d-bit, Max freq: %lu kHz",
             config.bus_width, (unsigned long)config.max_freq_khz);
    ESP_LOGI(TAG, "  Mount point: %s", config.mount_point);

    // Configure SDMMC host
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = config.max_freq_khz;

    // Configure slot
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    
    // Set GPIO pins
    slot_config.clk = config.pin_clk;
    slot_config.cmd = config.pin_cmd;
    slot_config.d0 = config.pin_d0;
    
    if (config.bus_width >= 4) {
        slot_config.d1 = config.pin_d1;
        slot_config.d2 = config.pin_d2;
        slot_config.d3 = config.pin_d3;
        slot_config.width = 4;
    } else {
        slot_config.width = 1;
    }

    // Card detect and write protect
    if (config.pin_cd != GPIO_NUM_NC) {
        slot_config.cd = config.pin_cd;
    }
    if (config.pin_wp != GPIO_NUM_NC) {
        slot_config.wp = config.pin_wp;
    }

    // Enable internal pull-ups (external pull-ups are recommended)
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    // Configure FATFS mount options
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = config.format_if_mount_failed,
        .max_files = config.max_files,
        .allocation_unit_size = 16 * 1024,  // 16KB clusters for better performance
        .disk_status_check_enable = false,
    };

    // Mount the filesystem
    esp_err_t ret = esp_vfs_fat_sdmmc_mount(
        config.mount_point,
        &host,
        &slot_config,
        &mount_config,
        &m_card
    );

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set format_if_mount_failed = true.");
        } else if (ret == ESP_ERR_NO_MEM) {
            ESP_LOGE(TAG, "Failed to allocate memory for SD card");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.",
                     esp_err_to_name(ret));
        }
        return esp_err_to_cube32(ret);
    }

    m_initialized = true;
    m_mounted = true;
    m_cardPresent = true;

    ESP_LOGI(TAG, "SD card mounted successfully");
    printCardInfo();

    return CUBE32_OK;
}

cube32_result_t SDCard::end() {
    if (!m_initialized) {
        return CUBE32_NOT_INITIALIZED;
    }

    // Unmount filesystem
    if (m_mounted) {
        esp_err_t ret = esp_vfs_fat_sdcard_unmount(m_config.mount_point, m_card);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to unmount SD card: %s", esp_err_to_name(ret));
            return esp_err_to_cube32(ret);
        }
        m_mounted = false;
    }

    m_card = nullptr;
    m_initialized = false;
    m_cardPresent = false;

    ESP_LOGI(TAG, "SD card unmounted");
    return CUBE32_OK;
}

// ============================================================================
// Software Card Detection
// ============================================================================

void SDCard::forceCleanup() {
    // Force cleanup of all state - used when card is removed or I/O error occurs
    ESP_LOGW(TAG, "Force cleanup of SD card state");
    
    if (m_mounted && m_card != nullptr) {
        // Try to unmount, ignore errors
        esp_vfs_fat_sdcard_unmount(m_config.mount_point, m_card);
    }
    
    m_mounted = false;
    m_card = nullptr;
    m_initialized = false;
    m_cardPresent = false;
    
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
    status.bus_width = m_card->log_bus_width ? (1 << m_card->log_bus_width) : 1;

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
        ESP_LOGW(TAG, "Read %zu bytes, expected %ld", read, size);
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
            ESP_LOGE(TAG, "Write failed: wrote %zu of %zu bytes", written, size);
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

} // namespace cube32
