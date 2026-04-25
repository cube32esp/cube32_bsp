/**
 * @file spi_bus.h
 * @brief CUBE32 Shared SPI Bus Manager
 * 
 * This module provides a shared SPI bus that can be used by multiple devices
 * (Display, SD Card, etc.) on the CUBE32 board.
 * 
 * The SPI bus is initialized once and shared across all drivers.
 */

#ifndef CUBE32_UTILS_SPI_BUS_H
#define CUBE32_UTILS_SPI_BUS_H

#include "utils/common.h"
#include "cube32_config.h"
#include <driver/spi_master.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief SPI bus configuration structure
 */
typedef struct {
    int mosi_pin;           ///< SPI MOSI GPIO pin
    int miso_pin;           ///< SPI MISO GPIO pin (-1 if not used)
    int sclk_pin;           ///< SPI SCLK GPIO pin
    spi_host_device_t host; ///< SPI host (SPI2_HOST or SPI3_HOST)
    int max_transfer_sz;    ///< Maximum transfer size in bytes
    int dma_channel;        ///< DMA channel (SPI_DMA_CH_AUTO recommended)
} cube32_spi_config_t;

/**
 * @brief Default SPI configuration using pins from cube32_config.h
 *
 * max_transfer_sz is the per-chunk DMA transfer limit. The SPI LCD panel IO
 * driver automatically splits larger color payloads into multiple chunks, so
 * this does NOT need to cover the full framebuffer. A smaller value reduces
 * the internal DMA bounce buffer that the SPI master allocates (important
 * when LVGL buffers live in PSRAM and audio I2S also needs DMA memory).
 */
#define CUBE32_SPI_CONFIG_DEFAULT() { \
    .mosi_pin = CUBE32_SPI_MOSI_PIN, \
    .miso_pin = CUBE32_SPI_MISO_PIN, \
    .sclk_pin = CUBE32_SPI_SCLK_PIN, \
    .host = CUBE32_SPI_HOST, \
    .max_transfer_sz = 240 * 48 * 2 + 8, \
    .dma_channel = SPI_DMA_CH_AUTO, \
}

/**
 * @brief Initialize the shared SPI bus with default configuration
 * 
 * Uses pins from cube32_config.h
 * 
 * @return CUBE32_OK on success, or error code
 */
cube32_result_t cube32_spi_init(void);

/**
 * @brief Initialize the shared SPI bus with custom configuration
 * 
 * @param config Pointer to SPI configuration structure
 * @return CUBE32_OK on success, or error code
 */
cube32_result_t cube32_spi_init_config(const cube32_spi_config_t* config);

/**
 * @brief Deinitialize the shared SPI bus
 * 
 * @note This will affect all devices using the shared bus
 * @return CUBE32_OK on success, or error code
 */
cube32_result_t cube32_spi_deinit(void);

/**
 * @brief Check if the shared SPI bus is initialized
 * 
 * @return true if initialized, false otherwise
 */
bool cube32_spi_is_initialized(void);

/**
 * @brief Get the SPI host device
 * 
 * @return SPI host device, or -1 if not initialized
 */
spi_host_device_t cube32_spi_get_host(void);

#ifdef __cplusplus
}

// ============================================================================
// C++ Interface
// ============================================================================

namespace cube32 {

/**
 * @brief Shared SPI Bus Manager (Singleton)
 * 
 * Provides a centralized SPI bus that can be shared across multiple drivers.
 * Uses the ESP-IDF spi_master API.
 */
class SPIBus {
public:
    /**
     * @brief Get the singleton instance
     */
    static SPIBus& instance();

    /**
     * @brief Initialize with default configuration
     */
    cube32_result_t init();

    /**
     * @brief Initialize with custom configuration
     */
    cube32_result_t init(const cube32_spi_config_t& config);

    /**
     * @brief Deinitialize the bus
     */
    cube32_result_t deinit();

    /**
     * @brief Check if initialized
     */
    bool isInitialized() const { return m_initialized; }

    /**
     * @brief Get the SPI host for device attachment
     */
    spi_host_device_t getHost() const { return m_host; }

    // Delete copy/move operations (singleton)
    SPIBus(const SPIBus&) = delete;
    SPIBus& operator=(const SPIBus&) = delete;

private:
    SPIBus() = default;
    ~SPIBus();

    spi_host_device_t m_host = SPI2_HOST;
    bool m_initialized = false;
};

} // namespace cube32

#endif // __cplusplus

#endif // CUBE32_UTILS_SPI_BUS_H
