/**
 * @file adc_button.cpp
 * @brief CUBE32 ADC Button Driver Implementation
 * 
 * Implements ADC-based button input using resistor voltage dividers.
 */

#include "drivers/button/adc_button.h"

#include <esp_log.h>
#include <cstring>

static const char* TAG = "cube32_adc_button";

namespace cube32 {

// ============================================================================
// Singleton Instance
// ============================================================================

ADCButton& ADCButton::instance() {
    static ADCButton instance;
    return instance;
}

// ============================================================================
// Destructor
// ============================================================================

ADCButton::~ADCButton() {
    end();
}

// ============================================================================
// Initialization
// ============================================================================

cube32_result_t ADCButton::begin() {
    ADCButtonConfig config = CUBE32_ADC_BUTTON_CONFIG_DEFAULT();
    return begin(config);
}

cube32_result_t ADCButton::begin(const ADCButtonConfig& config) {
    if (m_initialized) {
        ESP_LOGW(TAG, "ADC button already initialized");
        return CUBE32_OK;
    }

    // Store configuration
    m_config = config;

    ESP_LOGI(TAG, "Initializing ADC button driver...");
    ESP_LOGI(TAG, "  ADC Pin: GPIO%d", config.adc_pin);
    ESP_LOGI(TAG, "  ADC Unit: %d, Channel: %d", config.adc_unit, config.adc_channel);

    // Create button configuration
    button_config_t btn_cfg = {
        .long_press_time = config.long_press_time_ms,
        .short_press_time = config.short_press_time_ms,
    };

    // Initialize each ADC button
    for (size_t i = 0; i < CUBE32_ADC_BUTTON_MAX_COUNT; i++) {
        // Skip buttons with zero voltage range (not configured)
        if (config.button_ranges[i].min_mv == 0 && config.button_ranges[i].max_mv == 0) {
            ESP_LOGD(TAG, "  Button %zu: skipped (not configured)", i);
            continue;
        }

        button_adc_config_t adc_cfg = {
            .adc_handle = nullptr,  // Let the driver create the ADC handle
            .unit_id = config.adc_unit,
            .adc_channel = static_cast<uint8_t>(config.adc_channel),
            .button_index = static_cast<uint8_t>(i),
            .min = config.button_ranges[i].min_mv,
            .max = config.button_ranges[i].max_mv,
        };

        esp_err_t ret = iot_button_new_adc_device(&btn_cfg, &adc_cfg, &m_button_handles[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create ADC button %zu: %s", i, esp_err_to_name(ret));
            // Continue with other buttons
            m_button_handles[i] = nullptr;
        } else {
            ESP_LOGI(TAG, "  Button %zu: %d-%d mV", i, 
                     config.button_ranges[i].min_mv, config.button_ranges[i].max_mv);
        }
    }

    // Check if at least one button was initialized
    bool any_initialized = false;
    for (size_t i = 0; i < CUBE32_ADC_BUTTON_MAX_COUNT; i++) {
        if (m_button_handles[i] != nullptr) {
            any_initialized = true;
            break;
        }
    }

    if (!any_initialized) {
        ESP_LOGE(TAG, "No ADC buttons were initialized");
        return CUBE32_IO_ERROR;
    }

    m_initialized = true;
    ESP_LOGI(TAG, "ADC button driver initialized successfully");
    return CUBE32_OK;
}

cube32_result_t ADCButton::end() {
    if (!m_initialized) {
        return CUBE32_OK;
    }

    ESP_LOGI(TAG, "Deinitializing ADC button driver...");

    // Delete all button handles
    for (size_t i = 0; i < CUBE32_ADC_BUTTON_MAX_COUNT; i++) {
        if (m_button_handles[i] != nullptr) {
            iot_button_delete(m_button_handles[i]);
            m_button_handles[i] = nullptr;
        }

        // Free callback data
        for (size_t j = 0; j < static_cast<size_t>(BUTTON_EVENT_MAX); j++) {
            if (m_callbacks[i][j] != nullptr) {
                delete m_callbacks[i][j];
                m_callbacks[i][j] = nullptr;
            }
        }
    }

    m_initialized = false;
    ESP_LOGI(TAG, "ADC button driver deinitialized");
    return CUBE32_OK;
}

// ============================================================================
// Callback Management
// ============================================================================

void ADCButton::buttonEventCallback(void* button_handle, void* usr_data) {
    if (usr_data == nullptr) {
        return;
    }

    CallbackData* data = static_cast<CallbackData*>(usr_data);
    if (data->callback) {
        data->callback(data->button, data->event);
    }
}

cube32_result_t ADCButton::registerCallback(ADCButtonIndex button, button_event_t event, 
                                             ADCButtonCallback callback) {
    if (!m_initialized) {
        ESP_LOGE(TAG, "ADC button not initialized");
        return CUBE32_NOT_INITIALIZED;
    }

    size_t btn_idx = static_cast<size_t>(button);
    if (btn_idx >= CUBE32_ADC_BUTTON_MAX_COUNT) {
        ESP_LOGE(TAG, "Invalid button index: %zu", btn_idx);
        return CUBE32_INVALID_ARG;
    }

    if (m_button_handles[btn_idx] == nullptr) {
        ESP_LOGE(TAG, "Button %zu not configured", btn_idx);
        return CUBE32_INVALID_ARG;
    }

    size_t event_idx = static_cast<size_t>(event);
    if (event_idx >= static_cast<size_t>(BUTTON_EVENT_MAX)) {
        ESP_LOGE(TAG, "Invalid event: %zu", event_idx);
        return CUBE32_INVALID_ARG;
    }

    // Free existing callback if any
    if (m_callbacks[btn_idx][event_idx] != nullptr) {
        iot_button_unregister_cb(m_button_handles[btn_idx], event, nullptr);
        delete m_callbacks[btn_idx][event_idx];
        m_callbacks[btn_idx][event_idx] = nullptr;
    }

    // Create new callback data
    CallbackData* data = new CallbackData{this, button, event, callback};
    m_callbacks[btn_idx][event_idx] = data;

    // Register with iot_button
    esp_err_t ret = iot_button_register_cb(m_button_handles[btn_idx], event, nullptr, 
                                            buttonEventCallback, data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register callback for button %zu event %d: %s", 
                 btn_idx, event, esp_err_to_name(ret));
        delete m_callbacks[btn_idx][event_idx];
        m_callbacks[btn_idx][event_idx] = nullptr;
        return CUBE32_IO_ERROR;
    }

    ESP_LOGD(TAG, "Registered callback for button %zu event %d", btn_idx, event);
    return CUBE32_OK;
}

cube32_result_t ADCButton::unregisterCallback(ADCButtonIndex button, button_event_t event) {
    if (!m_initialized) {
        return CUBE32_NOT_INITIALIZED;
    }

    size_t btn_idx = static_cast<size_t>(button);
    if (btn_idx >= CUBE32_ADC_BUTTON_MAX_COUNT) {
        return CUBE32_INVALID_ARG;
    }

    if (m_button_handles[btn_idx] == nullptr) {
        return CUBE32_INVALID_ARG;
    }

    size_t event_idx = static_cast<size_t>(event);
    if (event_idx >= static_cast<size_t>(BUTTON_EVENT_MAX)) {
        return CUBE32_INVALID_ARG;
    }

    if (m_callbacks[btn_idx][event_idx] != nullptr) {
        iot_button_unregister_cb(m_button_handles[btn_idx], event, nullptr);
        delete m_callbacks[btn_idx][event_idx];
        m_callbacks[btn_idx][event_idx] = nullptr;
    }

    return CUBE32_OK;
}

button_handle_t ADCButton::getButtonHandle(ADCButtonIndex button) const {
    size_t btn_idx = static_cast<size_t>(button);
    if (btn_idx >= CUBE32_ADC_BUTTON_MAX_COUNT) {
        return nullptr;
    }
    return m_button_handles[btn_idx];
}

} // namespace cube32
