/**
 * @file audio_codec.cpp
 * @brief CUBE32 Audio Codec Driver Implementation
 * 
 * Implements audio input/output using ES8311 (DAC) and ES7210 (ADC).
 */

#include "drivers/audio/audio_codec.h"

#include <esp_log.h>
#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cstring>
#include "utils/hw_manifest.h"

static const char* TAG = "cube32_audio";

namespace cube32 {

// ============================================================================
// Singleton Instance
// ============================================================================

AudioCodec& AudioCodec::instance() {
    static AudioCodec instance;
    return instance;
}

// ============================================================================
// Constructor / Destructor
// ============================================================================

AudioCodec::~AudioCodec() {
    end();
}

// ============================================================================
// Initialization
// ============================================================================

cube32_result_t AudioCodec::begin() {
    // After the first begin(config) call (e.g. from cube32_init()), m_config holds the
    // hw-detected settings (correct ES8311 I2C address, ADC source, PA pin).  Reuse it
    // so that end()/begin() cycles in examples preserve those runtime-detected values.
    if (m_config_valid) {
        return begin(m_config);
    }
    AudioCodecConfig config = CUBE32_AUDIO_CONFIG_DEFAULT();
    return begin(config);
}

cube32_result_t AudioCodec::begin(const AudioCodecConfig& config) {
    if (m_initialized) {
        ESP_LOGW(TAG, "Audio codec already initialized");
        return CUBE32_OK;
    }

    // Store configuration
    m_config = config;
    m_config_valid = true;
    m_aec_mode = config.aec_mode;
    m_output_volume = config.output_volume;
    m_input_gain = config.input_gain;

    // ES8311 ADC path: HW AEC is not supported — force to NONE
    if (m_config.adc_source == AdcSource::ES8311 && m_aec_mode == AecMode::HW) {
        ESP_LOGW(TAG, "HW AEC is not supported with ES8311 ADC; forcing AEC mode to NONE");
        m_aec_mode = AecMode::NONE;
        m_config.aec_mode = AecMode::NONE;
    }

    // ES8311 ADC: standard I2S RX shares BCLK/LRCK with TX, so input rate must match output rate
    if (m_config.adc_source == AdcSource::ES8311 &&
        m_config.input_sample_rate != m_config.output_sample_rate) {
        ESP_LOGW(TAG, "ES8311 ADC: syncing input sample rate %d -> %d Hz (shared I2S clock)",
                 m_config.input_sample_rate, m_config.output_sample_rate);
        m_config.input_sample_rate = m_config.output_sample_rate;
    }

    // Auto-clamp input sample rate to 16 kHz when AEC is enabled
    if (m_aec_mode != AecMode::NONE && m_config.input_sample_rate > 16000) {
        ESP_LOGW(TAG, "Clamping input sample rate from %d to 16000 Hz for AEC",
                 m_config.input_sample_rate);
        m_config.input_sample_rate = 16000;
    }

    // Check that I2C bus is initialized
    if (!I2CBus::instance().isInitialized()) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return CUBE32_NOT_INITIALIZED;
    }

    /* Combined-scenario guard: when the Integrated Core+Audio Module (ES8311@0x19) is
     * physically present on the same I2S bus as ES7210, ES8311@0x19's ADCDAT output
     * (GPIO10 / I2S DIN) may not be tri-stated once I2S clocks start — corrupting the
    * ES7210 TDM stream. Put 0x19 into the managed driver's complete suspend
    * state on EVERY begin() call (not just the first), so end()+begin()
    * cycles in examples are also covered.
     */
    if (m_config.adc_source == AdcSource::ES7210) {
        const cube32_hw_manifest_t* mf = cube32_hw_manifest();
        if (mf && mf->scanned && mf->core_module == CUBE32_CORE_MODULE_S3_AUDIO) {
            ESP_LOGI(TAG, "  [DBG] Combined modules: suspending ES8311@0x19 "
                          "(ADCDAT high-Z guard before I2S start)");
            i2c_master_bus_handle_t bus = I2CBus::instance().getHandle();
            i2c_device_config_t dev_cfg = {};
            dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
            dev_cfg.device_address  = CUBE32_I2C_ADDR_ES8311_INTEGRATED;  // 0x19
            dev_cfg.scl_speed_hz    = CUBE32_I2C_FREQ_HZ;
            i2c_master_dev_handle_t dev_19 = nullptr;
            if (i2c_master_bus_add_device(bus, &dev_cfg, &dev_19) == ESP_OK) {
                // This is the proven es8311_suspend() register sequence from
                // esp_codec_dev. A reset pulse alone is insufficient: release
                // from reset may let the unused ADC resume driving ADCDAT.
                static constexpr uint8_t suspend_seq[][2] = {
                    {0x32, 0x00}, {0x17, 0x00}, {0x0E, 0xFF},
                    {0x12, 0x02}, {0x14, 0x00}, {0x0D, 0xFA},
                    {0x15, 0x00}, {0x02, 0x10}, {0x00, 0x00},
                    {0x00, 0x1F}, {0x01, 0x30}, {0x01, 0x00},
                    {0x45, 0x00}, {0x0D, 0xFC}, {0x02, 0x00},
                };
                for (const auto &reg : suspend_seq) {
                    i2c_master_transmit(dev_19, reg, sizeof(reg), 20);
                }
                vTaskDelay(pdMS_TO_TICKS(5));
                i2c_master_bus_rm_device(dev_19);
            }
        }
    }

    ESP_LOGI(TAG, "Initializing audio codec...");
    ESP_LOGI(TAG, "  Input sample rate: %d Hz", m_config.input_sample_rate);
    ESP_LOGI(TAG, "  Output sample rate: %d Hz", m_config.output_sample_rate);
    ESP_LOGI(TAG, "  AEC mode: %s",
             m_aec_mode == AecMode::HW ? "HW" :
             m_aec_mode == AecMode::SW ? "SW" : "NONE");
    ESP_LOGI(TAG, "  ADC source: %s",
             m_config.adc_source == AdcSource::ES8311 ? "ES8311 (DAC+ADC)" : "ES7210 (dedicated ADC)");
    ESP_LOGI(TAG, "  ES8311 addr: 0x%02X, ES7210 addr: 0x%02X", 
             config.es8311_addr, config.es7210_addr);
    ESP_LOGI(TAG, "  [DBG] free heap before audio init: %lu bytes",
             (unsigned long)esp_get_free_heap_size());

    // Create I2S duplex channels
    cube32_result_t ret = createDuplexChannels();
    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to create I2S channels");
        return ret;
    }

    // Initialize codec data interface
    audio_codec_i2s_cfg_t i2s_cfg = {
        .port = I2S_NUM_0,
        .rx_handle = m_rx_handle,
        .tx_handle = m_tx_handle,
    };
    m_data_if = audio_codec_new_i2s_data(&i2s_cfg);
    if (m_data_if == nullptr) {
        ESP_LOGE(TAG, "Failed to create I2S data interface");
        return CUBE32_IO_ERROR;
    }

    // Initialize GPIO interface for PA control
    m_gpio_if = audio_codec_new_gpio();
    if (m_gpio_if == nullptr) {
        ESP_LOGE(TAG, "Failed to create GPIO interface");
        return CUBE32_IO_ERROR;
    }

    // Initialize ES8311 output codec
    ret = initOutputCodec();
    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to initialize ES8311 output codec");
        return ret;
    }

    // Initialize input codec (ES7210 or ES8311 ADC path)
    if (m_config.adc_source == AdcSource::ES8311) {
        ret = initInputCodecES8311();
        if (ret != CUBE32_OK) {
            ESP_LOGE(TAG, "Failed to initialize ES8311 ADC input");
            return ret;
        }
    } else {
        ret = initInputCodec();
        if (ret != CUBE32_OK) {
            ESP_LOGE(TAG, "Failed to initialize ES7210 input codec");
            return ret;
        }
    }

    // Initialize PA control.
    // If pa_pin is set, the ES8311 driver manages the PA GPIO directly — TCA9554 is not needed
    // (integrated CUBE32 Core+Audio module: NS4150B PA on GPIO7, no IOX fitted).
    // If pa_pin is NC, use TCA9554 IO expander (dedicated Audio Module: PA via IOX@0x20).
    if (m_config.pa_pin != GPIO_NUM_NC) {
        ESP_LOGI(TAG, "PA control: GPIO%d (direct, via ES8311 driver)", (int)m_config.pa_pin);
    } else {
        ret = initPAControl();
        if (ret != CUBE32_OK) {
            ESP_LOGW(TAG, "PA control initialization failed - speaker may not work");
            // Continue anyway, as audio input may still work
        }
    }

    m_initialized = true;
    ESP_LOGI(TAG, "Audio codec initialized successfully");
    ESP_LOGI(TAG, "  [DBG] free heap after audio init:  %lu bytes",
             (unsigned long)esp_get_free_heap_size());
    return CUBE32_OK;
}

cube32_result_t AudioCodec::end() {
    if (!m_initialized) {
        return CUBE32_OK;
    }

    ESP_LOGI(TAG, "Deinitializing audio codec...");

    // Disable input/output
    if (m_input_enabled) {
        enableInput(false);
    }
    if (m_output_enabled) {
        enableOutput(false);
    }

    // Close and delete output device
    if (m_output_dev != nullptr) {
        esp_codec_dev_close(m_output_dev);
        esp_codec_dev_delete(m_output_dev);
        m_output_dev = nullptr;
    }

    // Close and delete input device
    if (m_input_dev != nullptr) {
        esp_codec_dev_close(m_input_dev);
        esp_codec_dev_delete(m_input_dev);
        m_input_dev = nullptr;
    }

    // Delete codec interfaces
    if (m_in_codec_if != nullptr) {
        audio_codec_delete_codec_if(m_in_codec_if);
        m_in_codec_if = nullptr;
    }
    if (m_in_ctrl_if != nullptr) {
        audio_codec_delete_ctrl_if(m_in_ctrl_if);
        m_in_ctrl_if = nullptr;
    }
    if (m_out_codec_if != nullptr) {
        audio_codec_delete_codec_if(m_out_codec_if);
        m_out_codec_if = nullptr;
    }
    if (m_out_ctrl_if != nullptr) {
        audio_codec_delete_ctrl_if(m_out_ctrl_if);
        m_out_ctrl_if = nullptr;
    }
    if (m_gpio_if != nullptr) {
        audio_codec_delete_gpio_if(m_gpio_if);
        m_gpio_if = nullptr;
    }
    if (m_data_if != nullptr) {
        audio_codec_delete_data_if(m_data_if);
        m_data_if = nullptr;
    }

    // Delete I2S channels
    if (m_tx_handle != nullptr) {
        i2s_channel_disable(m_tx_handle);
        i2s_del_channel(m_tx_handle);
        m_tx_handle = nullptr;
    }
    if (m_rx_handle != nullptr) {
        i2s_channel_disable(m_rx_handle);
        i2s_del_channel(m_rx_handle);
        m_rx_handle = nullptr;
    }

    // Cleanup PA IO expander
    if (m_pa_io_initialized) {
        m_pa_io_expander.end();
        m_pa_io_initialized = false;
    }

    m_initialized = false;
    ESP_LOGI(TAG, "Audio codec deinitialized");
    return CUBE32_OK;
}

// ============================================================================
// I2S Channel Setup
// ============================================================================

cube32_result_t AudioCodec::createDuplexChannels() {
    ESP_LOGI(TAG, "Creating I2S duplex channels...");
    ESP_LOGI(TAG, "  MCLK: GPIO%d, BCLK: GPIO%d, LRCK: GPIO%d", 
             m_config.mclk_pin, m_config.bclk_pin, m_config.lrck_pin);
    ESP_LOGI(TAG, "  DOUT: GPIO%d, DIN: GPIO%d", m_config.dout_pin, m_config.din_pin);

    // Create I2S channel pair
    i2s_chan_config_t chan_cfg = {
        .id = I2S_NUM_0,
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = CUBE32_AUDIO_DMA_DESC_NUM,
        .dma_frame_num = CUBE32_AUDIO_DMA_FRAME_NUM,
        .auto_clear_after_cb = true,
        .auto_clear_before_cb = false,
        .intr_priority = 0,
    };

    esp_err_t err = i2s_new_channel(&chan_cfg, &m_tx_handle, &m_rx_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2S channel: %s", esp_err_to_name(err));
        return CUBE32_IO_ERROR;
    }

    // Configure TX channel (output) - Standard I2S mode
    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = (uint32_t)m_config.output_sample_rate,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .ext_clk_freq_hz = 0,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256
        },
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode = I2S_SLOT_MODE_STEREO,
            .slot_mask = I2S_STD_SLOT_BOTH,
            .ws_width = I2S_DATA_BIT_WIDTH_16BIT,
            .ws_pol = false,
            .bit_shift = true,
            .left_align = true,
            .big_endian = false,
            .bit_order_lsb = false
        },
        .gpio_cfg = {
            .mclk = m_config.mclk_pin,
            .bclk = m_config.bclk_pin,
            .ws = m_config.lrck_pin,
            .dout = m_config.dout_pin,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false
            }
        }
    };

    err = i2s_channel_init_std_mode(m_tx_handle, &std_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init TX channel: %s", esp_err_to_name(err));
        return CUBE32_IO_ERROR;
    }
    ESP_LOGI(TAG, "  [DBG] I2S TX (STD stereo) channel init OK");

    if (m_config.adc_source == AdcSource::ES8311) {
        // Configure RX channel (input) - Standard I2S mode for ES8311 ADC.
        // ES8311 has a single mic so RX is mono.  Note: esp_codec_dev_open()
        // reconfigures the I2S slot/clock at runtime based on the codec's sample
        // info, so the values set here are only the initial state.
        i2s_std_config_t rx_std_cfg = {
            .clk_cfg = {
                .sample_rate_hz = (uint32_t)m_config.input_sample_rate,
                .clk_src = I2S_CLK_SRC_DEFAULT,
                .ext_clk_freq_hz = 0,
                .mclk_multiple = I2S_MCLK_MULTIPLE_256
            },
            .slot_cfg = {
                .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
                .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
                .slot_mode = I2S_SLOT_MODE_MONO,
                .slot_mask = I2S_STD_SLOT_LEFT,
                .ws_width = I2S_DATA_BIT_WIDTH_16BIT,
                .ws_pol = false,
                .bit_shift = true,
                .left_align = true,
                .big_endian = false,
                .bit_order_lsb = false
            },
            .gpio_cfg = {
                .mclk = m_config.mclk_pin,  // Keep MCLK routed (same as TDM path)
                .bclk = m_config.bclk_pin,
                .ws = m_config.lrck_pin,
                .dout = I2S_GPIO_UNUSED,
                .din = m_config.din_pin,
                .invert_flags = {
                    .mclk_inv = false,
                    .bclk_inv = false,
                    .ws_inv = false
                }
            }
        };

        err = i2s_channel_init_std_mode(m_rx_handle, &rx_std_cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to init RX channel (ES8311 std mode): %s", esp_err_to_name(err));
            return CUBE32_IO_ERROR;
        }
        ESP_LOGI(TAG, "  [DBG] I2S RX (STD mono) channel init OK");
    } else {
    // Configure RX channel (input) - TDM mode for ES7210 (4 channels)
    i2s_tdm_config_t tdm_cfg = {
        .clk_cfg = {
            .sample_rate_hz = (uint32_t)m_config.input_sample_rate,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .ext_clk_freq_hz = 0,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
            // Keep ESP-IDF's TDM default. audio_codec_new_i2s_data() uses
            // the same divider when it reconfigures this channel at open().
            .bclk_div = 8,
        },
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode = I2S_SLOT_MODE_STEREO,
            .slot_mask = i2s_tdm_slot_mask_t(I2S_TDM_SLOT0 | I2S_TDM_SLOT1 | I2S_TDM_SLOT2 | I2S_TDM_SLOT3),
            .ws_width = I2S_TDM_AUTO_WS_WIDTH,
            .ws_pol = false,
            .bit_shift = true,
            .left_align = false,
            .big_endian = false,
            .bit_order_lsb = false,
            .skip_mask = false,
            .total_slot = I2S_TDM_AUTO_SLOT_NUM
        },
        .gpio_cfg = {
            .mclk = m_config.mclk_pin,
            .bclk = m_config.bclk_pin,
            .ws = m_config.lrck_pin,
            .dout = I2S_GPIO_UNUSED,
            .din = m_config.din_pin,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false
            }
        }
    };

    err = i2s_channel_init_tdm_mode(m_rx_handle, &tdm_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init RX channel: %s", esp_err_to_name(err));
        return CUBE32_IO_ERROR;
    }
    ESP_LOGI(TAG, "  [DBG] I2S RX (TDM 4-ch) channel init OK");
    }   // end if (adc_source == ES8311) ... else

    ESP_LOGI(TAG, "I2S duplex channels created");
    return CUBE32_OK;
}

// ============================================================================
// ES8311 Output Codec
// ============================================================================

cube32_result_t AudioCodec::initOutputCodec() {
    ESP_LOGI(TAG, "Initializing ES8311 output codec...");

    // Create I2C control interface for ES8311
    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = I2CBus::instance().getPort(),
        .addr = m_config.es8311_addr,
        .bus_handle = I2CBus::instance().getHandle(),
    };
    m_out_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    if (m_out_ctrl_if == nullptr) {
        ESP_LOGE(TAG, "Failed to create ES8311 I2C control interface");
        return CUBE32_IO_ERROR;
    }

    // Create ES8311 codec interface.
    // When ES8311 handles both DAC and ADC (adc_source == ES8311), use BOTH mode so
    // the chip is initialized ONCE with all paths enabled.  A second es8311_codec_new()
    // in ADC-only mode would trigger a chip software-reset ("Work in Slave mode") that
    // blows away the DAC register state set here — causing broken audio and, indirectly,
    // LVGL DMA stalls that blank the display.  Using BOTH mode avoids the second init.
    es8311_codec_cfg_t es8311_cfg = {};
    es8311_cfg.ctrl_if = m_out_ctrl_if;
    es8311_cfg.gpio_if = m_gpio_if;
    es8311_cfg.codec_mode = (m_config.adc_source == AdcSource::ES8311)
                            ? ESP_CODEC_DEV_WORK_MODE_BOTH   // integrated: DAC + ADC, one init
                            : ESP_CODEC_DEV_WORK_MODE_DAC;   // dedicated module: DAC only
    es8311_cfg.pa_pin = m_config.pa_pin;  // GPIO_NUM_NC since PA is controlled via TCA9554
    es8311_cfg.use_mclk = true;
    // Hardware gain settings - affects volume calculation
    // hw_gain = 20 * log10(dac_voltage / pa_voltage) + pa_gain
    // With pa_voltage=5.0 and dac_voltage=3.3: hw_gain = -3.6dB
    // In es8311_set_vol: db_value -= hw_gain, so negative hw_gain ADDS to volume
    // This matches xiaozhi-esp32 configuration for proper speaker output level
    es8311_cfg.hw_gain.pa_voltage = 5.0;  // Results in +3.6dB boost
    es8311_cfg.hw_gain.codec_dac_voltage = 3.3;
    es8311_cfg.hw_gain.pa_gain = 0;

    m_out_codec_if = es8311_codec_new(&es8311_cfg);
    if (m_out_codec_if == nullptr) {
        ESP_LOGE(TAG, "Failed to create ES8311 codec interface");
        return CUBE32_IO_ERROR;
    }

    // Create codec device for output
    esp_codec_dev_cfg_t dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
        .codec_if = m_out_codec_if,
        .data_if = m_data_if,
    };
    m_output_dev = esp_codec_dev_new(&dev_cfg);
    if (m_output_dev == nullptr) {
        ESP_LOGE(TAG, "Failed to create output device");
        return CUBE32_IO_ERROR;
    }

    // Set a custom volume curve to map volume 0-100 to a higher dB range
    // The default curve maps 100% to 0dB, but ES8311 supports up to +32dB
    // Max dB is configurable at runtime via NVS config manager
    // WARNING: High values may cause audio distortion
    m_vol_map[0].vol = 0;
    m_vol_map[0].db_value = -50.0f;
    m_vol_map[1].vol = 100;
    m_vol_map[1].db_value = 18.0f;
    
    esp_codec_dev_vol_curve_t vol_curve = {
        .vol_map = m_vol_map,
        .count = 2,
    };
    
    esp_err_t err = esp_codec_dev_set_vol_curve(m_output_dev, &vol_curve);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set volume curve: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Volume curve set: 0%% -> -50dB, 100%% -> +10dB");
    }

    ESP_LOGI(TAG, "ES8311 output codec initialized");
    return CUBE32_OK;
}

// ============================================================================
// ES7210 Input Codec
// ============================================================================

cube32_result_t AudioCodec::initInputCodec() {
    ESP_LOGI(TAG, "Initializing ES7210 input codec...");

    // Create I2C control interface for ES7210
    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = I2CBus::instance().getPort(),
        .addr = m_config.es7210_addr,
        .bus_handle = I2CBus::instance().getHandle(),
    };
    m_in_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    if (m_in_ctrl_if == nullptr) {
        ESP_LOGE(TAG, "Failed to create ES7210 I2C control interface");
        return CUBE32_IO_ERROR;
    }

    // Create ES7210 codec interface
    es7210_codec_cfg_t es7210_cfg = {};
    es7210_cfg.ctrl_if = m_in_ctrl_if;
    es7210_cfg.mic_selected = ES7210_SEL_MIC1 | ES7210_SEL_MIC2 | ES7210_SEL_MIC3 | ES7210_SEL_MIC4;

    m_in_codec_if = es7210_codec_new(&es7210_cfg);
    if (m_in_codec_if == nullptr) {
        ESP_LOGE(TAG, "Failed to create ES7210 codec interface");
        return CUBE32_IO_ERROR;
    }

    // Create codec device for input
    esp_codec_dev_cfg_t dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN,
        .codec_if = m_in_codec_if,
        .data_if = m_data_if,
    };
    m_input_dev = esp_codec_dev_new(&dev_cfg);
    if (m_input_dev == nullptr) {
        ESP_LOGE(TAG, "Failed to create input device");
        return CUBE32_IO_ERROR;
    }

    ESP_LOGI(TAG, "ES7210 input codec initialized");
    return CUBE32_OK;
}

// ============================================================================
// ES8311 ADC Input Codec (used when CUBE32_AUDIO_ADC_ES8311 is defined)
// ============================================================================

cube32_result_t AudioCodec::initInputCodecES8311() {
    // m_out_codec_if was created with WORK_MODE_BOTH in initOutputCodec(), so the
    // ES8311 chip is already configured for both DAC and ADC in a single init.
    // We simply create a second esp_codec_dev_handle pointing to the SAME codec_if
    // as the output device.  No second I2C control interface or chip init is needed —
    // that is what caused the chip software-reset and the subsequent display blank.
    // m_in_ctrl_if and m_in_codec_if are left nullptr; end() skips deleting nullptr.
    ESP_LOGI(TAG, "Initializing ES8311 ADC input (sharing DAC codec_if, WORK_MODE_BOTH)...");

    if (m_out_codec_if == nullptr) {
        ESP_LOGE(TAG, "ES8311 output codec_if not ready — call initOutputCodec() first");
        return CUBE32_NOT_INITIALIZED;
    }

    // Reuse the BOTH-mode codec_if; only the device type differs.
    esp_codec_dev_cfg_t dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN,
        .codec_if = m_out_codec_if,  // shared — chip already init'd in BOTH mode
        .data_if  = m_data_if,
    };
    m_input_dev = esp_codec_dev_new(&dev_cfg);
    if (m_input_dev == nullptr) {
        ESP_LOGE(TAG, "Failed to create ES8311 ADC input device");
        return CUBE32_IO_ERROR;
    }

    ESP_LOGI(TAG, "ES8311 ADC input initialized (shared codec_if)");
    return CUBE32_OK;
}

// ============================================================================
// Input Control
// ============================================================================

void AudioCodec::enableInput(bool enable) {
    std::lock_guard<std::mutex> lock(m_data_mutex);

    if (enable == m_input_enabled) {
        return;
    }

    if (enable) {
        ESP_LOGI(TAG, "Enabling audio input...");
        // ES8311: 1 mic channel.  ES7210: 4 TDM channels (mic[0..3]).
        int ch = (m_config.adc_source == AdcSource::ES8311) ? 1 : 4;
        ESP_LOGI(TAG, "  [DBG] Opening input: %d ch, %d Hz, DIN=GPIO%d",
                 ch, m_config.input_sample_rate, (int)m_config.din_pin);
        // The esp_codec_dev I2S data interface reconfigures TDM from this
        // sample-info structure when opening the device.  A mask containing
        // only channel 0 previously overwrote the four-slot setup above with
        // slot_mask=0x1 (as seen in the diagnostic log), so only one TDM slot
        // was clocked/captured. Preserve all ES7210 data slots explicitly.
        const uint16_t input_channel_mask =
            (m_config.adc_source == AdcSource::ES7210)
                ? (ESP_CODEC_DEV_MAKE_CHANNEL_MASK(0) |
                   ESP_CODEC_DEV_MAKE_CHANNEL_MASK(1) |
                   ESP_CODEC_DEV_MAKE_CHANNEL_MASK(2) |
                   ESP_CODEC_DEV_MAKE_CHANNEL_MASK(3))
                : ESP_CODEC_DEV_MAKE_CHANNEL_MASK(0);
        esp_codec_dev_sample_info_t fs = {
            .bits_per_sample = 16,
            .channel = (uint8_t)ch,
            .channel_mask = input_channel_mask,
            .sample_rate = (uint32_t)m_config.input_sample_rate,
            .mclk_multiple = 0,
        };

        esp_err_t err = esp_codec_dev_open(m_input_dev, &fs);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to open input device: %s", esp_err_to_name(err));
            return;
        }

        // ES8311 is a single-mic codec — only the overall gain API is supported.
        // esp_codec_dev_set_in_channel_gain() returns ESP_ERR_NOT_SUPPORTED for ES8311,
        // leaving the PGA at 0 dB and producing a near-silent recording.
        if (m_config.adc_source == AdcSource::ES8311) {
            err = esp_codec_dev_set_in_gain(m_input_dev, (float)m_input_gain);
        } else {
            err = esp_codec_dev_set_in_channel_gain(m_input_dev,
                input_channel_mask, (float)m_input_gain);
        }
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to set input gain (%d dB): %s",
                     m_input_gain, esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "Input PGA gain set to %d dB", m_input_gain);
        }

        m_input_enabled = true;
        ESP_LOGI(TAG, "Audio input enabled");
    } else {
        ESP_LOGI(TAG, "Disabling audio input...");
        esp_codec_dev_close(m_input_dev);
        m_input_enabled = false;
        ESP_LOGI(TAG, "Audio input disabled");
    }
}

void AudioCodec::setInputGain(int gain) {
    m_input_gain = gain;
    if (m_input_enabled && m_input_dev != nullptr) {
        if (m_config.adc_source == AdcSource::ES8311) {
            esp_codec_dev_set_in_gain(m_input_dev, (float)gain);
        } else {
            esp_codec_dev_set_in_channel_gain(m_input_dev,
                ESP_CODEC_DEV_MAKE_CHANNEL_MASK(0), (float)gain);
        }
    }
}

int AudioCodec::read(int16_t* dest, int samples) {
    if (!m_input_enabled || m_input_dev == nullptr) {
        return 0;
    }

    esp_err_t err = esp_codec_dev_read(m_input_dev, dest, samples * sizeof(int16_t));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Read error: %s", esp_err_to_name(err));
        return 0;
    }

    // Sample-level diagnostics: log max absolute value to detect silent capture.
    // Logs for the first 5 reads and every 50 thereafter (~1.6 s at 16kHz/512 samples).
    static uint32_t s_read_count = 0;
    ++s_read_count;
    if (s_read_count <= 5 || (s_read_count % 50) == 0) {
        int16_t max_abs = 0;
        for (int i = 0; i < samples; i++) {
            int16_t v = dest[i] >= 0 ? dest[i] : (int16_t)(-dest[i]);
            if (v > max_abs) max_abs = v;
        }
        /*if (max_abs < 32) {
            ESP_LOGW(TAG, "[DBG] read #%lu: n=%d max_abs=%d <<< SILENT — ES7210 not capturing",
                     (unsigned long)s_read_count, samples, (int)max_abs);
        } else {
            ESP_LOGI(TAG, "[DBG] read #%lu: n=%d max_abs=%d",
                     (unsigned long)s_read_count, samples, (int)max_abs);
        }*/
    }

    return samples;
}

// ============================================================================
// Output Control
// ============================================================================

void AudioCodec::enableOutput(bool enable) {
    // Use the configured output sample rate
    enableOutput(enable, m_config.output_sample_rate);
}

void AudioCodec::enableOutput(bool enable, int sample_rate) {
    std::lock_guard<std::mutex> lock(m_data_mutex);

    // If disabling, or if already in the requested state with same sample rate, skip
    if (!enable && !m_output_enabled) {
        return;
    }
    
    // If enabling with a different sample rate, close first then reopen
    if (enable && m_output_enabled) {
        // Close current output to allow reopening with new sample rate
        ESP_LOGI(TAG, "Closing output to reconfigure sample rate to %d Hz", sample_rate);
        esp_codec_dev_close(m_output_dev);
        m_output_enabled = false;
    }

    if (enable) {
        ESP_LOGI(TAG, "Enabling audio output at %d Hz...", sample_rate);
        esp_codec_dev_sample_info_t fs = {
            .bits_per_sample = 16,
            .channel = 1,  // Mono output - matches xiaozhi-esp32 BoxAudioCodec
            .channel_mask = 0,
            .sample_rate = (uint32_t)sample_rate,
            .mclk_multiple = 0,
        };

        esp_err_t err = esp_codec_dev_open(m_output_dev, &fs);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to open output device: %s", esp_err_to_name(err));
            return;
        }

        // Unmute output first
        err = esp_codec_dev_set_out_mute(m_output_dev, false);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to unmute output: %s", esp_err_to_name(err));
        }

        // Set volume (0-100 maps to codec volume range)
        ESP_LOGI(TAG, "Setting output volume to %d", m_output_volume);
        err = esp_codec_dev_set_out_vol(m_output_dev, m_output_volume);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to set output volume: %s", esp_err_to_name(err));
        }

        // Enable PA via IO expander
        setPAEnabledInternal(true);

        m_output_enabled = true;
        ESP_LOGI(TAG, "Audio output enabled at %d Hz", sample_rate);
    } else {
        ESP_LOGI(TAG, "Disabling audio output...");
        
        // Disable PA via IO expander first
        setPAEnabledInternal(false);
        
        esp_codec_dev_close(m_output_dev);
        m_output_enabled = false;
        ESP_LOGI(TAG, "Audio output disabled");
    }
}

esp_err_t AudioCodec::setOutputVolume(int volume) {
    if (volume < 0) volume = 0;
    if (volume > 100) volume = 100;
    
    m_output_volume = volume;
    // Examples re-create the codec to switch the AEC sample rate. Keep the
    // public configuration in sync so a slider value chosen while output is
    // closed is applied when that new output device is opened.
    m_config.output_volume = volume;
    esp_err_t ret = ESP_OK;
    if (m_output_enabled && m_output_dev != nullptr) {
        ret = esp_codec_dev_set_out_vol(m_output_dev, volume);
    }
    ESP_LOGI(TAG, "Output volume set to %d", volume);
    return ret;
}

int AudioCodec::write(const int16_t* data, int samples) {
    if (!m_output_enabled || m_output_dev == nullptr) {
        return 0;
    }

    esp_err_t err = esp_codec_dev_write(m_output_dev, (void*)data, samples * sizeof(int16_t));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Write error: %s", esp_err_to_name(err));
        return 0;
    }
    return samples;
}

// ============================================================================
// PA Control via IO Expander
// ============================================================================

cube32_result_t AudioCodec::initPAControl() {
    ESP_LOGI(TAG, "Initializing PA control via TCA9554 at 0x%02X", m_config.pa_iox_addr);
    
    // Initialize the IO expander for PA control
    cube32_result_t ret = m_pa_io_expander.begin(m_config.pa_iox_addr);
    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to initialize TCA9554 for PA control");
        return ret;
    }
    
    // Configure PA pin as output
    ret = m_pa_io_expander.setDirection(m_config.pa_iox_pin, TCA9554Direction::DIR_OUTPUT);
    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to set PA pin direction");
        m_pa_io_expander.end();
        return ret;
    }
    
    // Start with PA disabled (low)
    ret = m_pa_io_expander.setLevel(m_config.pa_iox_pin, false);
    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to set PA pin level");
        m_pa_io_expander.end();
        return ret;
    }
    
    // Configure PJ_DET pin as input (earphone jack detect)
    ret = m_pa_io_expander.setDirection(m_config.pj_det_iox_pin, TCA9554Direction::DIR_INPUT);
    if (ret != CUBE32_OK) {
        ESP_LOGW(TAG, "Failed to set PJ_DET pin direction (non-fatal)");
        // Continue — PA control still works
    } else {
        ESP_LOGI(TAG, "PJ_DET (earphone jack detect) configured on port %d",
                 CUBE32_AUDIO_IOX_PJ_DET_PIN);
    }
    
    m_pa_io_initialized = true;
    ESP_LOGI(TAG, "PA control initialized - PA disabled");
    
    return CUBE32_OK;
}

void AudioCodec::setPAEnabled(bool enable) {
    setPAEnabledInternal(enable);
}

void AudioCodec::setPAEnabledInternal(bool enable) {
    if (!m_pa_io_initialized) {
        ESP_LOGW(TAG, "PA IO expander not initialized");
        return;
    }
    
    cube32_result_t ret = m_pa_io_expander.setLevel(m_config.pa_iox_pin, enable);
    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to %s PA", enable ? "enable" : "disable");
    } else {
        ESP_LOGI(TAG, "PA %s", enable ? "enabled" : "disabled");
        // Add a small delay after enabling PA (like Arduino demo does)
        // This allows the NS4150B amplifier to stabilize
        if (enable) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

bool AudioCodec::isEarphonePlugged() {
    if (!m_pa_io_initialized) {
        ESP_LOGW(TAG, "PA IO expander not initialized");
        return false;
    }
    
    bool level = true;  // Default: not plugged (HIGH)
    cube32_result_t ret = m_pa_io_expander.getLevel(m_config.pj_det_iox_pin, level);
    if (ret != CUBE32_OK) {
        ESP_LOGW(TAG, "Failed to read PJ_DET pin");
        return false;
    }
    
    // PJ_DET is active LOW: LOW = earphone plugged, HIGH = not plugged
    return !level;
}

void AudioCodec::readPADiagnostics() {
    if (!m_pa_io_initialized) {
        printf("PA IO expander not initialized\n");
        return;
    }

    uint32_t pa_pin_mask = static_cast<uint32_t>(m_config.pa_iox_pin);
    printf("PA IOX diagnostics (TCA9554 @ 0x%02X):\n", m_config.pa_iox_addr);
    printf("  pa_iox_pin mask: 0x%02lX\n", (unsigned long)pa_pin_mask);

    // Read back PA pin level
    bool pa_level = false;
    cube32_result_t ret = m_pa_io_expander.getLevel(m_config.pa_iox_pin, pa_level);
    if (ret == CUBE32_OK) {
        printf("  PA pin (port 0) level: %d (%s)\n", pa_level ? 1 : 0,
               pa_level ? "HIGH - PA enabled" : "LOW - PA disabled");
    } else {
        printf("  PA pin read FAILED (err %d)\n", ret);
    }

    // Read all pin levels (input register)
    uint32_t all_levels = 0;
    ret = m_pa_io_expander.getAllLevels(all_levels);
    if (ret == CUBE32_OK) {
        printf("  All pins (input reg): 0x%02lX\n", (unsigned long)(all_levels & 0xFF));
        for (int i = 0; i < 8; i++) {
            printf("    Port %d: %lu\n", i, (all_levels >> i) & 1UL);
        }
    } else {
        printf("  All pins read FAILED (err %d)\n", ret);
    }

    // Print full IO expander state (direction, output, input registers)
    esp_io_expander_handle_t handle = m_pa_io_expander.getHandle();
    if (handle) {
        printf("  --- esp_io_expander register dump ---\n");
        esp_io_expander_print_state(handle);
    }
}

} // namespace cube32
