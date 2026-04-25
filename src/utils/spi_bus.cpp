/**
 * @file spi_bus.cpp
 * @brief CUBE32 Shared SPI Bus Manager Implementation
 */

#include "utils/spi_bus.h"
#include <esp_log.h>

static const char* TAG = "cube32_spi";

namespace cube32 {

// ============================================================================
// Singleton Implementation
// ============================================================================

SPIBus& SPIBus::instance() {
    static SPIBus s_instance;
    return s_instance;
}

SPIBus::~SPIBus() {
    if (m_initialized) {
        deinit();
    }
}

cube32_result_t SPIBus::init() {
    cube32_spi_config_t config = CUBE32_SPI_CONFIG_DEFAULT();
    return init(config);
}

cube32_result_t SPIBus::init(const cube32_spi_config_t& config) {
    if (m_initialized) {
        ESP_LOGW(TAG, "SPI bus already initialized");
        return CUBE32_ALREADY_INITIALIZED;
    }

    ESP_LOGI(TAG, "Initializing shared SPI bus...");
    ESP_LOGI(TAG, "  MOSI: GPIO%d, MISO: GPIO%d, SCLK: GPIO%d",
             config.mosi_pin, config.miso_pin, config.sclk_pin);

    // Configure SPI bus
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = config.mosi_pin,
        .miso_io_num = config.miso_pin,
        .sclk_io_num = config.sclk_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .max_transfer_sz = config.max_transfer_sz,
        .flags = SPICOMMON_BUSFLAG_MASTER,
        .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
        .intr_flags = 0,
    };

    esp_err_t ret = spi_bus_initialize(config.host, &bus_cfg, config.dma_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return esp_err_to_cube32(ret);
    }

    m_host = config.host;
    m_initialized = true;

    ESP_LOGI(TAG, "SPI bus initialized successfully");
    return CUBE32_OK;
}

cube32_result_t SPIBus::deinit() {
    if (!m_initialized) {
        return CUBE32_NOT_INITIALIZED;
    }

    esp_err_t ret = spi_bus_free(m_host);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to free SPI bus: %s", esp_err_to_name(ret));
        return esp_err_to_cube32(ret);
    }

    m_initialized = false;
    ESP_LOGI(TAG, "SPI bus deinitialized");
    return CUBE32_OK;
}

} // namespace cube32

// ============================================================================
// C Interface Implementation
// ============================================================================

extern "C" {

cube32_result_t cube32_spi_init(void) {
    return cube32::SPIBus::instance().init();
}

cube32_result_t cube32_spi_init_config(const cube32_spi_config_t* config) {
    if (!config) {
        return CUBE32_INVALID_ARG;
    }
    return cube32::SPIBus::instance().init(*config);
}

cube32_result_t cube32_spi_deinit(void) {
    return cube32::SPIBus::instance().deinit();
}

bool cube32_spi_is_initialized(void) {
    return cube32::SPIBus::instance().isInitialized();
}

spi_host_device_t cube32_spi_get_host(void) {
    if (!cube32::SPIBus::instance().isInitialized()) {
        return (spi_host_device_t)-1;
    }
    return cube32::SPIBus::instance().getHost();
}

} // extern "C"
