/**
 * @file i2c_bus.h
 * @brief CUBE32 Shared I2C Bus Manager
 * 
 * This module provides a shared I2C bus that can be used by multiple devices
 * (PMU, IMU, Touch, Audio Codec, IO Expander, etc.) on the CUBE32 board.
 * 
 * The I2C bus is initialized once and shared across all drivers.
 */

#ifndef CUBE32_UTILS_I2C_BUS_H
#define CUBE32_UTILS_I2C_BUS_H

#include "utils/common.h"
#include "cube32_config.h"
#include <driver/i2c_master.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief I2C bus configuration structure
 */
typedef struct {
    int sda_pin;            ///< I2C SDA GPIO pin
    int scl_pin;            ///< I2C SCL GPIO pin
    i2c_port_t port;        ///< I2C port number
    uint32_t freq_hz;       ///< I2C clock frequency in Hz
    bool enable_pullup;     ///< Enable internal pull-up resistors
} cube32_i2c_config_t;

/**
 * @brief Default I2C configuration using pins from cube32_config.h
 */
#define CUBE32_I2C_CONFIG_DEFAULT() { \
    .sda_pin = CUBE32_I2C_SDA_PIN, \
    .scl_pin = CUBE32_I2C_SCL_PIN, \
    .port = CUBE32_I2C_NUM, \
    .freq_hz = CUBE32_I2C_FREQ_HZ, \
    .enable_pullup = true, \
}

/**
 * @brief Initialize the shared I2C bus with default configuration
 * 
 * Uses CUBE32_I2C_SDA_PIN and CUBE32_I2C_SCL_PIN from cube32_config.h
 * 
 * @return CUBE32_OK on success, or error code
 */
cube32_result_t cube32_i2c_init(void);

/**
 * @brief Initialize the shared I2C bus with custom configuration
 * 
 * @param config Pointer to I2C configuration structure
 * @return CUBE32_OK on success, or error code
 */
cube32_result_t cube32_i2c_init_config(const cube32_i2c_config_t* config);

/**
 * @brief Deinitialize the shared I2C bus
 * 
 * @note This will affect all devices using the shared bus
 * @return CUBE32_OK on success, or error code
 */
cube32_result_t cube32_i2c_deinit(void);

/**
 * @brief Check if the shared I2C bus is initialized
 * 
 * @return true if initialized, false otherwise
 */
bool cube32_i2c_is_initialized(void);

/**
 * @brief Get the shared I2C bus handle
 * 
 * @return I2C bus handle, or NULL if not initialized
 */
i2c_master_bus_handle_t cube32_i2c_get_bus_handle(void);

/**
 * @brief Get the I2C port number
 * 
 * @return I2C port number
 */
i2c_port_t cube32_i2c_get_port(void);

/**
 * @brief Probe for a device on the I2C bus
 * 
 * @param device_addr 7-bit I2C device address
 * @return CUBE32_OK if device responds, CUBE32_NOT_FOUND otherwise
 */
cube32_result_t cube32_i2c_probe(uint8_t device_addr);

/**
 * @brief Scan the I2C bus and print found devices
 * 
 * Scans addresses 0x08-0x77 and logs any responding devices
 */
void cube32_i2c_scan(void);

#ifdef __cplusplus
}

// ============================================================================
// C++ Interface
// ============================================================================

#include <functional>

namespace cube32 {

/**
 * @brief Shared I2C Bus Manager (Singleton)
 * 
 * Provides a centralized I2C bus that can be shared across multiple drivers.
 * Uses the new ESP-IDF 5.x i2c_master API.
 */
class I2CBus {
public:
    /**
     * @brief Get the singleton instance
     */
    static I2CBus& instance();

    /**
     * @brief Initialize with default configuration
     */
    cube32_result_t init();

    /**
     * @brief Initialize with custom configuration
     */
    cube32_result_t init(const cube32_i2c_config_t& config);

    /**
     * @brief Deinitialize the bus
     */
    cube32_result_t deinit();

    /**
     * @brief Check if initialized
     */
    bool isInitialized() const { return m_initialized; }

    /**
     * @brief Get the bus handle for device attachment
     */
    i2c_master_bus_handle_t getHandle() const { return m_bus_handle; }

    /**
     * @brief Get the I2C port
     */
    i2c_port_t getPort() const { return m_port; }

    /**
     * @brief Probe for a device
     */
    cube32_result_t probe(uint8_t addr);

    /**
     * @brief Scan and log all devices
     */
    void scan();

    // Delete copy/move operations (singleton)
    I2CBus(const I2CBus&) = delete;
    I2CBus& operator=(const I2CBus&) = delete;

private:
    I2CBus() = default;
    ~I2CBus();

    i2c_master_bus_handle_t m_bus_handle = nullptr;
    i2c_port_t m_port = I2C_NUM_0;
    bool m_initialized = false;
};

} // namespace cube32

#endif // __cplusplus

#endif // CUBE32_UTILS_I2C_BUS_H
