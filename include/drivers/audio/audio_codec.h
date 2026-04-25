/**
 * @file audio_codec.h
 * @brief CUBE32 Audio Codec Driver
 * 
 * This driver provides audio input/output functionality for the CUBE32 board
 * using ES8311 for speaker output and ES7210 for microphone input.
 * 
 * The driver uses the shared I2C bus from utils/i2c_bus.h
 * 
 * Features:
 * - Duplex audio (simultaneous record and playback)
 * - ES8311 DAC for speaker output (mono/stereo)
 * - ES7210 ADC for microphone input (up to 4 channels)
 * - Reference input for acoustic echo cancellation (AEC)
 * - Configurable sample rates for input and output
 * - Volume control for output
 * - Gain control for input
 */

#ifndef CUBE32_DRIVERS_AUDIO_CODEC_H
#define CUBE32_DRIVERS_AUDIO_CODEC_H

#include "utils/common.h"
#include "utils/i2c_bus.h"
#include "drivers/io_expander/tca9554.h"
#include "cube32_config.h"

#include <driver/i2s_std.h>
#include <driver/i2s_tdm.h>
#include <esp_codec_dev.h>
#include <esp_codec_dev_defaults.h>

#include <cstdint>
#include <mutex>

// ============================================================================
// Constants
// ============================================================================

#ifndef ES8311_CODEC_DEFAULT_ADDR
#define ES8311_CODEC_DEFAULT_ADDR    0x18    ///< ES8311 default I2C address
#endif

#ifndef ES7210_CODEC_DEFAULT_ADDR
#define ES7210_CODEC_DEFAULT_ADDR    0x40    ///< ES7210 default I2C address
#endif

#define CUBE32_AUDIO_DMA_DESC_NUM    6       ///< Number of DMA descriptors
#define CUBE32_AUDIO_DMA_FRAME_NUM   240     ///< DMA frame size in samples

namespace cube32 {

// ============================================================================
// Types
// ============================================================================

/**
 * @brief AEC operating mode
 */
enum class AecMode {
    NONE = 0,  ///< No echo cancellation
    HW   = 1,  ///< Hardware reference loopback (ES7210 ch1) + afe_aec
    SW   = 2,  ///< Software reference buffer + aec
};

/**
 * @brief Audio codec configuration structure
 */
struct AudioCodecConfig {
#ifdef CONFIG_CUBE32_AUDIO_ENABLED
    int input_sample_rate = CONFIG_CUBE32_AUDIO_INPUT_SAMPLE_RATE;   ///< Input sample rate in Hz
    int output_sample_rate = CONFIG_CUBE32_AUDIO_OUTPUT_SAMPLE_RATE; ///< Output sample rate in Hz
    int output_volume = CONFIG_CUBE32_AUDIO_OUTPUT_VOLUME;           ///< Output volume (0-100)
    int input_gain = CONFIG_CUBE32_AUDIO_INPUT_GAIN;                 ///< Input gain in dB
#if defined(CONFIG_CUBE32_AUDIO_AEC_HW)
    AecMode aec_mode = AecMode::HW;                                  ///< AEC mode
#elif defined(CONFIG_CUBE32_AUDIO_AEC_SW)
    AecMode aec_mode = AecMode::SW;                                  ///< AEC mode
#else
    AecMode aec_mode = AecMode::NONE;                                ///< AEC mode
#endif
#else
    int input_sample_rate = 16000;       ///< Input sample rate in Hz
    int output_sample_rate = 24000;      ///< Output sample rate in Hz
    int output_volume = 70;              ///< Output volume (0-100)
    int input_gain = 30;                 ///< Input gain in dB
    AecMode aec_mode = AecMode::HW;      ///< AEC mode
#endif
    
    // I2S GPIO pins
    gpio_num_t mclk_pin = (gpio_num_t)CUBE32_AUDIO_I2S_MCLK_PIN;
    gpio_num_t bclk_pin = (gpio_num_t)CUBE32_AUDIO_I2S_BCLK_PIN;
    gpio_num_t lrck_pin = (gpio_num_t)CUBE32_AUDIO_I2S_LRCK_PIN;
    gpio_num_t dout_pin = (gpio_num_t)CUBE32_AUDIO_I2S_DO_PIN;
    gpio_num_t din_pin = (gpio_num_t)CUBE32_AUDIO_I2S_DI_PIN;
    gpio_num_t pa_pin = (gpio_num_t)CUBE32_AUDIO_PA_PIN;
    
    // I2C addresses
    uint8_t es8311_addr = ES8311_CODEC_DEFAULT_ADDR;
    uint8_t es7210_addr = ES7210_CODEC_DEFAULT_ADDR;
    
    // IO expander pin assignments (Audio board TCA9554)
    uint8_t pa_iox_addr = CUBE32_AUDIO_IOX_ADDR;   ///< IO expander I2C address
    TCA9554Pin pa_iox_pin = static_cast<TCA9554Pin>(1ULL << CUBE32_AUDIO_IOX_PA_CTRL_PIN);   ///< PA control pin (Output)
    TCA9554Pin pj_det_iox_pin = static_cast<TCA9554Pin>(1ULL << CUBE32_AUDIO_IOX_PJ_DET_PIN); ///< Phone jack detect pin (Input)
};

/**
 * @brief Default audio configuration
 *
 * Sample rates and volume default to sensible values.
 * Actual values are applied from NVS config at runtime.
 */
#ifdef CONFIG_CUBE32_AUDIO_ENABLED
#define CUBE32_AUDIO_CONFIG_DEFAULT() { \
    .input_sample_rate = CONFIG_CUBE32_AUDIO_INPUT_SAMPLE_RATE, \
    .output_sample_rate = CONFIG_CUBE32_AUDIO_OUTPUT_SAMPLE_RATE, \
    .output_volume = CONFIG_CUBE32_AUDIO_OUTPUT_VOLUME, \
    .input_gain = CONFIG_CUBE32_AUDIO_INPUT_GAIN, \
    .aec_mode = cube32::AudioCodecConfig{}.aec_mode, \
    .mclk_pin = (gpio_num_t)CUBE32_AUDIO_I2S_MCLK_PIN, \
    .bclk_pin = (gpio_num_t)CUBE32_AUDIO_I2S_BCLK_PIN, \
    .lrck_pin = (gpio_num_t)CUBE32_AUDIO_I2S_LRCK_PIN, \
    .dout_pin = (gpio_num_t)CUBE32_AUDIO_I2S_DO_PIN, \
    .din_pin = (gpio_num_t)CUBE32_AUDIO_I2S_DI_PIN, \
    .pa_pin = (gpio_num_t)CUBE32_AUDIO_PA_PIN, \
    .es8311_addr = CUBE32_AUDIO_CODEC_ES8311_ADDR, \
    .es7210_addr = CUBE32_AUDIO_CODEC_ES7210_ADDR, \
    .pa_iox_addr = CUBE32_AUDIO_IOX_ADDR, \
    .pa_iox_pin = static_cast<cube32::TCA9554Pin>(1ULL << CUBE32_AUDIO_IOX_PA_CTRL_PIN), \
    .pj_det_iox_pin = static_cast<cube32::TCA9554Pin>(1ULL << CUBE32_AUDIO_IOX_PJ_DET_PIN), \
}
#else
#define CUBE32_AUDIO_CONFIG_DEFAULT() { \
    .input_sample_rate = 16000, \
    .output_sample_rate = 24000, \
    .output_volume = 70, \
    .input_gain = 30, \
    .aec_mode = cube32::AecMode::HW, \
    .mclk_pin = (gpio_num_t)CUBE32_AUDIO_I2S_MCLK_PIN, \
    .bclk_pin = (gpio_num_t)CUBE32_AUDIO_I2S_BCLK_PIN, \
    .lrck_pin = (gpio_num_t)CUBE32_AUDIO_I2S_LRCK_PIN, \
    .dout_pin = (gpio_num_t)CUBE32_AUDIO_I2S_DO_PIN, \
    .din_pin = (gpio_num_t)CUBE32_AUDIO_I2S_DIN_PIN, \
    .pa_pin = (gpio_num_t)CUBE32_AUDIO_PA_PIN, \
    .es8311_addr = CUBE32_AUDIO_CODEC_ES8311_ADDR, \
    .es7210_addr = CUBE32_AUDIO_CODEC_ES7210_ADDR, \
    .pa_iox_addr = CUBE32_AUDIO_IOX_ADDR, \
    .pa_iox_pin = static_cast<cube32::TCA9554Pin>(1ULL << CUBE32_AUDIO_IOX_PA_CTRL_PIN), \
    .pj_det_iox_pin = static_cast<cube32::TCA9554Pin>(1ULL << CUBE32_AUDIO_IOX_PJ_DET_PIN), \
}
#endif

/**
 * @brief Audio Codec Driver (Singleton)
 * 
 * Provides audio input/output using ES8311 (DAC) and ES7210 (ADC) codecs.
 * Uses shared I2C bus and creates duplex I2S channels for audio streaming.
 */
class AudioCodec {
public:
    /**
     * @brief Get the singleton instance
     */
    static AudioCodec& instance();

    /**
     * @brief Initialize the audio codec with default configuration
     * @return CUBE32_OK on success, error code otherwise
     */
    cube32_result_t begin();

    /**
     * @brief Initialize the audio codec with custom configuration
     * @param config Audio configuration
     * @return CUBE32_OK on success, error code otherwise
     */
    cube32_result_t begin(const AudioCodecConfig& config);

    /**
     * @brief Deinitialize the audio codec
     * @return CUBE32_OK on success, error code otherwise
     */
    cube32_result_t end();

    /**
     * @brief Check if the codec is initialized
     */
    bool isInitialized() const { return m_initialized; }

    // ========================================================================
    // Input Control
    // ========================================================================

    /**
     * @brief Enable or disable audio input (microphone)
     * @param enable true to enable, false to disable
     */
    void enableInput(bool enable);

    /**
     * @brief Check if input is enabled
     */
    bool isInputEnabled() const { return m_input_enabled; }

    /**
     * @brief Set microphone input gain
     * @param gain Gain in dB
     */
    void setInputGain(int gain);

    /**
     * @brief Get current input gain
     */
    int getInputGain() const { return m_input_gain; }

    /**
     * @brief Read audio samples from microphone
     * @param dest Destination buffer (16-bit samples)
     * @param samples Number of samples to read
     * @return Number of samples actually read
     */
    int read(int16_t* dest, int samples);

    // ========================================================================
    // Output Control
    // ========================================================================

    /**
     * @brief Enable or disable audio output (speaker)
     * @param enable true to enable, false to disable
     */
    void enableOutput(bool enable);

    /**
     * @brief Enable audio output with specific sample rate
     * @param enable true to enable, false to disable
     * @param sample_rate Sample rate for playback (if enable is true)
     * 
     * Use this when playing audio files with a different sample rate
     * than the configured output_sample_rate.
     */
    void enableOutput(bool enable, int sample_rate);

    /**
     * @brief Check if output is enabled
     */
    bool isOutputEnabled() const { return m_output_enabled; }

    /**
     * @brief Set speaker output volume
     * @param volume Volume (0-100)
     * @return ESP_OK on success, or an error code on failure
     */
    esp_err_t setOutputVolume(int volume);

    /**
     * @brief Get current output volume
     */
    int getOutputVolume() const { return m_output_volume; }

    /**
     * @brief Write audio samples to speaker
     * @param data Source buffer (16-bit samples)
     * @param samples Number of samples to write
     * @return Number of samples actually written
     */
    int write(const int16_t* data, int samples);

    // ========================================================================
    // Configuration Getters
    // ========================================================================

    int getInputSampleRate() const { return m_config.input_sample_rate; }
    int getOutputSampleRate() const { return m_config.output_sample_rate; }
    int getInputChannels() const { return m_aec_mode == AecMode::HW ? 2 : 1; }
    bool isDuplex() const { return true; }
    AecMode getAecMode() const { return m_aec_mode; }
    bool hasInputReference() const { return m_aec_mode == AecMode::HW; }

    // ========================================================================
    // PA & Earphone Jack Control
    // ========================================================================

    /**
     * @brief Manually enable or disable the PA amplifier
     * @param enable true to enable PA, false to disable
     */
    void setPAEnabled(bool enable);

    /**
     * @brief Check if an earphone is plugged in
     * @return true if earphone is detected (PJ_DET LOW), false otherwise
     */
    bool isEarphonePlugged();

    /**
     * @brief Read and print IO expander diagnostic info for PA port
     * Reads back pin level, direction register, and all pin levels.
     */
    void readPADiagnostics();

    // Delete copy/move operations (singleton)
    AudioCodec(const AudioCodec&) = delete;
    AudioCodec& operator=(const AudioCodec&) = delete;

private:
    AudioCodec() = default;
    ~AudioCodec();

    /**
     * @brief Create duplex I2S channels (TX for output, RX for input)
     */
    cube32_result_t createDuplexChannels();

    /**
     * @brief Initialize ES8311 output codec
     */
    cube32_result_t initOutputCodec();

    /**
     * @brief Initialize ES7210 input codec
     */
    cube32_result_t initInputCodec();

    /**
     * @brief Initialize IO expander for PA control
     */
    cube32_result_t initPAControl();

    /**
     * @brief Enable/disable PA via IO expander (internal implementation)
     */
    void setPAEnabledInternal(bool enable);

    // Configuration
    AudioCodecConfig m_config;
    bool m_initialized = false;
    bool m_input_enabled = false;
    bool m_output_enabled = false;
    AecMode m_aec_mode = AecMode::HW;
    int m_output_volume = 70;
    int m_input_gain = 30;

    // I2S handles
    i2s_chan_handle_t m_tx_handle = nullptr;
    i2s_chan_handle_t m_rx_handle = nullptr;

    // Codec device interfaces
    const audio_codec_data_if_t* m_data_if = nullptr;
    const audio_codec_ctrl_if_t* m_out_ctrl_if = nullptr;
    const audio_codec_if_t* m_out_codec_if = nullptr;
    const audio_codec_ctrl_if_t* m_in_ctrl_if = nullptr;
    const audio_codec_if_t* m_in_codec_if = nullptr;
    const audio_codec_gpio_if_t* m_gpio_if = nullptr;

    // Codec device handles
    esp_codec_dev_handle_t m_output_dev = nullptr;
    esp_codec_dev_handle_t m_input_dev = nullptr;
    
    // Volume curve for speaker output
    // Maps volume percentage (0-100) to dB value
    // This is needed because the default curve maps 100% to 0dB, 
    // but we want louder output using the ES8311's +32dB capability
    esp_codec_dev_vol_map_t m_vol_map[2] = {};

    // PA control via IO expander
    TCA9554 m_pa_io_expander;
    bool m_pa_io_initialized = false;

    // Thread safety
    std::mutex m_data_mutex;
};

} // namespace cube32

#endif // CUBE32_DRIVERS_AUDIO_CODEC_H
