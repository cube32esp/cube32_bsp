/**
 * @file adc_button.h
 * @brief CUBE32 ADC Button Driver
 * 
 * This driver provides ADC-based button input functionality for the CUBE32 board
 * using a resistor voltage divider array connected to an ADC pin.
 * 
 * Features:
 * - Support for multiple buttons on a single ADC pin
 * - Configurable voltage ranges for each button
 * - Event callbacks for various button events (click, double-click, long press)
 * - Uses espressif__button component for debouncing and event handling
 * 
 * Typical resistor divider setup (from 3.3V):
 * - 10K + 1.3K  -> 0.38V (Button 0)
 * - 10K + 3.3K  -> 0.82V (Button 1)
 * - 10K + 5.1K  -> 1.11V (Button 2)
 * - 10K + 10K   -> 1.65V (Button 3)
 * - 10K + 15K   -> 1.98V (Button 4)
 * - 10K + 27K   -> 2.41V (Button 5)
 */

#ifndef CUBE32_DRIVERS_BUTTON_ADC_BUTTON_H
#define CUBE32_DRIVERS_BUTTON_ADC_BUTTON_H

#include "utils/common.h"
#include "cube32_config.h"

#include <driver/gpio.h>
#include <esp_adc/adc_oneshot.h>
#include <iot_button.h>
#include <button_adc.h>

#include <cstdint>
#include <functional>
#include <array>

namespace cube32 {

// ============================================================================
// Constants
// ============================================================================

#define CUBE32_ADC_BUTTON_MAX_COUNT    6    ///< Maximum number of ADC buttons

// ============================================================================
// Types
// ============================================================================

/**
 * @brief ADC button index enumeration
 */
enum class ADCButtonIndex : uint8_t {
    BUTTON_0 = 0,    ///< Button 0 (1.3K resistor, ~0.38V)
    BUTTON_1 = 1,    ///< Button 1 (3.3K resistor, ~0.82V)
    BUTTON_2 = 2,    ///< Button 2 (5.1K resistor, ~1.11V)
    BUTTON_3 = 3,    ///< Button 3 (10K resistor, ~1.65V)
    BUTTON_4 = 4,    ///< Button 4 (15K resistor, ~1.98V)
    BUTTON_5 = 5,    ///< Button 5 (27K resistor, ~2.41V)
    COUNT = CUBE32_ADC_BUTTON_MAX_COUNT
};

/**
 * @brief Button voltage range configuration
 */
struct ADCButtonVoltageRange {
    uint16_t min_mv;    ///< Minimum voltage in millivolts
    uint16_t max_mv;    ///< Maximum voltage in millivolts
};

/**
 * @brief ADC button configuration structure
 */
struct ADCButtonConfig {
    gpio_num_t adc_pin = (gpio_num_t)CUBE32_ADC_BUTTON_PIN;  ///< ADC pin
    adc_unit_t adc_unit = ADC_UNIT_1;                        ///< ADC unit
    adc_channel_t adc_channel = ADC_CHANNEL_4;               ///< ADC channel (GPIO5 = ADC1_CH4)
    
    // Voltage ranges for each button (min/max in mV)
    ADCButtonVoltageRange button_ranges[CUBE32_ADC_BUTTON_MAX_COUNT] = {
        {CUBE32_ADC_BUTTON_0_MIN_MV, CUBE32_ADC_BUTTON_0_MAX_MV},  // Button 0: ~0.38V
        {CUBE32_ADC_BUTTON_1_MIN_MV, CUBE32_ADC_BUTTON_1_MAX_MV},  // Button 1: ~0.82V
        {CUBE32_ADC_BUTTON_2_MIN_MV, CUBE32_ADC_BUTTON_2_MAX_MV},  // Button 2: ~1.11V
        {CUBE32_ADC_BUTTON_3_MIN_MV, CUBE32_ADC_BUTTON_3_MAX_MV},  // Button 3: ~1.65V
        {CUBE32_ADC_BUTTON_4_MIN_MV, CUBE32_ADC_BUTTON_4_MAX_MV},  // Button 4: ~1.98V
        {CUBE32_ADC_BUTTON_5_MIN_MV, CUBE32_ADC_BUTTON_5_MAX_MV},  // Button 5: ~2.41V
    };
    
    uint16_t long_press_time_ms = 2000; ///< Long press threshold in ms
    uint16_t short_press_time_ms = 50;  //0;   //50;    ///< Short press threshold in ms (debounce time)
};

/**
 * @brief Button event callback function type
 */
using ADCButtonCallback = std::function<void(ADCButtonIndex button, button_event_t event)>;

/**
 * @brief Default ADC button configuration using config values
 */
#define CUBE32_ADC_BUTTON_CONFIG_DEFAULT() { \
    .adc_pin = (gpio_num_t)CUBE32_ADC_BUTTON_PIN, \
    .adc_unit = ADC_UNIT_1, \
    .adc_channel = ADC_CHANNEL_4, \
    .button_ranges = { \
        {CUBE32_ADC_BUTTON_0_MIN_MV, CUBE32_ADC_BUTTON_0_MAX_MV}, \
        {CUBE32_ADC_BUTTON_1_MIN_MV, CUBE32_ADC_BUTTON_1_MAX_MV}, \
        {CUBE32_ADC_BUTTON_2_MIN_MV, CUBE32_ADC_BUTTON_2_MAX_MV}, \
        {CUBE32_ADC_BUTTON_3_MIN_MV, CUBE32_ADC_BUTTON_3_MAX_MV}, \
        {CUBE32_ADC_BUTTON_4_MIN_MV, CUBE32_ADC_BUTTON_4_MAX_MV}, \
        {CUBE32_ADC_BUTTON_5_MIN_MV, CUBE32_ADC_BUTTON_5_MAX_MV}, \
    }, \
    .long_press_time_ms = 1000, \
    .short_press_time_ms = 100, \
}

/**
 * @brief ADC Button Driver (Singleton)
 * 
 * Provides ADC-based button input using resistor voltage dividers.
 * Uses the espressif__button component for event handling.
 */
class ADCButton {
public:
    /**
     * @brief Get the singleton instance
     */
    static ADCButton& instance();

    /**
     * @brief Initialize the ADC button driver with default configuration
     * @return CUBE32_OK on success, error code otherwise
     */
    cube32_result_t begin();

    /**
     * @brief Initialize the ADC button driver with custom configuration
     * @param config ADC button configuration
     * @return CUBE32_OK on success, error code otherwise
     */
    cube32_result_t begin(const ADCButtonConfig& config);

    /**
     * @brief Deinitialize the ADC button driver
     * @return CUBE32_OK on success, error code otherwise
     */
    cube32_result_t end();

    /**
     * @brief Check if the driver is initialized
     * @return true if initialized
     */
    bool isInitialized() const { return m_initialized; }

    /**
     * @brief Register a callback for a specific button and event
     * @param button Button index
     * @param event Button event type
     * @param callback Callback function
     * @return CUBE32_OK on success, error code otherwise
     */
    cube32_result_t registerCallback(ADCButtonIndex button, button_event_t event, 
                                      ADCButtonCallback callback);

    /**
     * @brief Unregister a callback for a specific button and event
     * @param button Button index
     * @param event Button event type
     * @return CUBE32_OK on success, error code otherwise
     */
    cube32_result_t unregisterCallback(ADCButtonIndex button, button_event_t event);

    /**
     * @brief Get the button handle for direct iot_button API access
     * @param button Button index
     * @return Button handle or nullptr if not initialized
     */
    button_handle_t getButtonHandle(ADCButtonIndex button) const;

    /**
     * @brief Get the number of configured buttons
     * @return Number of buttons
     */
    size_t getButtonCount() const { return CUBE32_ADC_BUTTON_MAX_COUNT; }

private:
    ADCButton() = default;
    ~ADCButton();
    
    // Non-copyable
    ADCButton(const ADCButton&) = delete;
    ADCButton& operator=(const ADCButton&) = delete;

    // Static callback wrapper for iot_button
    static void buttonEventCallback(void* button_handle, void* usr_data);

    // Member variables
    bool m_initialized = false;
    ADCButtonConfig m_config;
    
    // Button handles array
    std::array<button_handle_t, CUBE32_ADC_BUTTON_MAX_COUNT> m_button_handles = {nullptr};
    
    // Callback storage (one callback per button/event combination)
    struct CallbackData {
        ADCButton* instance;
        ADCButtonIndex button;
        button_event_t event;
        ADCButtonCallback callback;
    };
    
    // Store callbacks for each button and event type
    std::array<std::array<CallbackData*, static_cast<size_t>(BUTTON_EVENT_MAX)>, 
               CUBE32_ADC_BUTTON_MAX_COUNT> m_callbacks = {};
};

} // namespace cube32

#endif /* CUBE32_DRIVERS_BUTTON_ADC_BUTTON_H */
