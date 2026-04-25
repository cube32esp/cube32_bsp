/**
 * @file tca9554.h
 * @brief TCA9554 I/O Expander Driver for CUBE32
 * 
 * This driver provides an interface to the TCA9554 I2C I/O expander.
 * Multiple instances can be created for different I2C addresses.
 * 
 * Features:
 * - 8 configurable I/O pins
 * - Individual pin direction control (input/output)
 * - Individual pin level control (high/low)
 * - Pin state reading
 * - Uses shared I2C bus
 */

#pragma once

#include <cstdint>
#include <esp_err.h>
#include <esp_io_expander.h>

#include "common.h"
#include "i2c_bus.h"

namespace cube32 {

/**
 * @brief TCA9554 pin definitions
 */
enum class TCA9554Pin : uint32_t {
    PIN_0 = IO_EXPANDER_PIN_NUM_0,
    PIN_1 = IO_EXPANDER_PIN_NUM_1,
    PIN_2 = IO_EXPANDER_PIN_NUM_2,
    PIN_3 = IO_EXPANDER_PIN_NUM_3,
    PIN_4 = IO_EXPANDER_PIN_NUM_4,
    PIN_5 = IO_EXPANDER_PIN_NUM_5,
    PIN_6 = IO_EXPANDER_PIN_NUM_6,
    PIN_7 = IO_EXPANDER_PIN_NUM_7,
};

/**
 * @brief TCA9554 pin direction
 */
enum class TCA9554Direction {
    DIR_INPUT = IO_EXPANDER_INPUT,
    DIR_OUTPUT = IO_EXPANDER_OUTPUT,
};

/**
 * @brief TCA9554 I/O Expander Driver
 * 
 * This class provides control over a TCA9554 I2C I/O expander.
 * Unlike singleton drivers, multiple instances can be created for 
 * different I2C addresses.
 * 
 * Example usage:
 * @code
 * cube32::TCA9554 ioExpander;
 * ioExpander.begin(0x20);  // Initialize with I2C address 0x20
 * ioExpander.setDirection(TCA9554Pin::PIN_0, TCA9554Direction::OUTPUT);
 * ioExpander.setLevel(TCA9554Pin::PIN_0, true);  // Set high
 * @endcode
 */
class TCA9554 {
public:
    /**
     * @brief Constructor
     */
    TCA9554() = default;
    
    /**
     * @brief Destructor
     */
    ~TCA9554();
    
    /**
     * @brief Initialize the TCA9554
     * @param i2c_addr I2C address of the TCA9554 (typically 0x20-0x27)
     * @return CUBE32_OK on success
     */
    cube32_result_t begin(uint8_t i2c_addr);
    
    /**
     * @brief Deinitialize the TCA9554
     * @return CUBE32_OK on success
     */
    cube32_result_t end();
    
    /**
     * @brief Check if the TCA9554 is initialized
     * @return true if initialized
     */
    bool isInitialized() const { return m_initialized; }
    
    /**
     * @brief Get the I2C address
     * @return I2C address
     */
    uint8_t getAddress() const { return m_address; }
    
    /**
     * @brief Set the direction of a pin
     * @param pin Pin to configure
     * @param direction Direction (INPUT or OUTPUT)
     * @return CUBE32_OK on success
     */
    cube32_result_t setDirection(TCA9554Pin pin, TCA9554Direction direction);
    
    /**
     * @brief Set the direction of multiple pins
     * @param pin_mask Bitmask of pins to configure (use | to combine TCA9554Pin values)
     * @param direction Direction (INPUT or OUTPUT)
     * @return CUBE32_OK on success
     */
    cube32_result_t setDirectionMask(uint32_t pin_mask, TCA9554Direction direction);
    
    /**
     * @brief Set the output level of a pin
     * @param pin Pin to set
     * @param level true for high, false for low
     * @return CUBE32_OK on success
     */
    cube32_result_t setLevel(TCA9554Pin pin, bool level);
    
    /**
     * @brief Set the output level of multiple pins
     * @param pin_mask Bitmask of pins to set (use | to combine TCA9554Pin values)
     * @param level true for high, false for low
     * @return CUBE32_OK on success
     */
    cube32_result_t setLevelMask(uint32_t pin_mask, bool level);
    
    /**
     * @brief Get the input level of a pin
     * @param pin Pin to read
     * @param level Output: true for high, false for low
     * @return CUBE32_OK on success
     */
    cube32_result_t getLevel(TCA9554Pin pin, bool& level);
    
    /**
     * @brief Get the input levels of all pins
     * @param levels Output: bitmask of pin levels
     * @return CUBE32_OK on success
     */
    cube32_result_t getAllLevels(uint32_t& levels);
    
    /**
     * @brief Get the underlying esp_io_expander handle
     * @return Handle or nullptr if not initialized
     */
    esp_io_expander_handle_t getHandle() const { return m_handle; }

private:
    // Prevent copy
    TCA9554(const TCA9554&) = delete;
    TCA9554& operator=(const TCA9554&) = delete;
    
    esp_io_expander_handle_t m_handle = nullptr;
    uint8_t m_address = 0;
    bool m_initialized = false;
};

} // namespace cube32
