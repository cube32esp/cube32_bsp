/**
 * @file st7796.cpp
 * @brief CUBE32 ST7796S TFT Display Driver Implementation
 */

#include "drivers/display/st7796.h"
#include <esp_log.h>
#include <esp_heap_caps.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_lcd_st7796.h>
#include <esp_lcd_panel_interface.h>  // full esp_lcd_panel_t definition, needed for the orientation shim below
#include <freertos/semphr.h>
#include <esp_attr.h>
#include <string.h>

static const char* TAG = "cube32_st7796";

namespace cube32 {

// ============================================================================
// Vendor init sequence -- transcribed verbatim from the panel manufacturer's
// BOE3.92IPS(GV039Z2Q-N80)-ST7796U-2.2Gamma-20211203.INI, confirmed correct
// via the standalone apps/LCD_ST7796_Test raw bring-up test.
//
// Two key facts this sequence bakes in that the ESP-IDF esp_lcd_st7796
// driver's built-in default init sequence does NOT:
//   - Display Function Control (0xB6) = 0x8A,0x07,0x27 sets the gate line
//     count (NL) to exactly 320. Without this, the panel stays at its
//     power-on-default gate count (very likely 480, the common ST7796
//     native GRAM), which was the true root cause of the "top half shows
//     static" bug previously worked around with y_gap=160 -- with 0xB6 set
//     correctly, the panel is natively 320 rows and needs NO y_gap at all.
//   - MADCTL (0x36) = 0x48 (MY=0, MX=1, MV=0, BGR=1) is the panel's native
//     "book" orientation -- MX=1 (not the ST7789-style MX=0 assumption).
//   - COLMOD (0x3A) = 0x55 (vs. the ESP-IDF driver's default 0x05 for
//     16bpp -- likely equivalent, included here for exact vendor fidelity).
// ============================================================================

static const uint8_t k_f0_unlock1[] = {0xC3};
static const uint8_t k_f0_unlock2[] = {0x96};
static const uint8_t k_madctl[]     = {0x48};  // MY=0, MX=1, MV=0, BGR=1
static const uint8_t k_colmod[]     = {0x55};  // 16bpp
static const uint8_t k_b4[]         = {0x00};
static const uint8_t k_b6[]         = {0x8A, 0x07, 0x27};  // Display Function Control: NL=320 gates
static const uint8_t k_b7[]         = {0xC6};
static const uint8_t k_b9[]         = {0x02, 0xE0};
static const uint8_t k_c0[]         = {0x80, 0x06};
static const uint8_t k_c1[]         = {0x15};
static const uint8_t k_c2[]         = {0xA7};
static const uint8_t k_c5[]         = {0x04};
static const uint8_t k_e8[]         = {0x40, 0x8A, 0x00, 0x00, 0x29, 0x19, 0xAA, 0x33};
static const uint8_t k_e0[]         = {0xF0, 0x06, 0x0F, 0x05, 0x04, 0x20, 0x37, 0x33,
                                        0x4C, 0x37, 0x13, 0x14, 0x2B, 0x31};
static const uint8_t k_e1[]         = {0xF0, 0x11, 0x1B, 0x11, 0x0F, 0x0A, 0x37, 0x43,
                                        0x4C, 0x37, 0x13, 0x13, 0x2C, 0x32};
static const uint8_t k_f0_lock1[]   = {0x3C};
static const uint8_t k_f0_lock2[]   = {0x69};
static const uint8_t k_teon[]       = {0x00};

// Note: 0x11 (Sleep Out) is NOT repeated here -- panel_st7796_init() in
// esp_lcd_st7796_general.c already sends it (with its own ~100ms delay)
// before running this init_cmds array.
static const st7796_lcd_init_cmd_t s_vendor_init_cmds[] = {
    {0xF0, k_f0_unlock1, sizeof(k_f0_unlock1), 0},
    {0xF0, k_f0_unlock2, sizeof(k_f0_unlock2), 0},
    {0x36, k_madctl,     sizeof(k_madctl),     0},
    {0x3A, k_colmod,     sizeof(k_colmod),     0},
    {0xB4, k_b4,         sizeof(k_b4),         0},
    {0xB6, k_b6,         sizeof(k_b6),         0},
    {0xB7, k_b7,         sizeof(k_b7),         0},
    {0xB9, k_b9,         sizeof(k_b9),         0},
    {0xC0, k_c0,         sizeof(k_c0),         0},
    {0xC1, k_c1,         sizeof(k_c1),         0},
    {0xC2, k_c2,         sizeof(k_c2),         0},
    {0xC5, k_c5,         sizeof(k_c5),         0},
    {0xE8, k_e8,         sizeof(k_e8),         0},
    {0xE0, k_e0,         sizeof(k_e0),         0},
    {0xE1, k_e1,         sizeof(k_e1),         0},
    {0xF0, k_f0_lock1,   sizeof(k_f0_lock1),   0},
    {0xF0, k_f0_lock2,   sizeof(k_f0_lock2),   0},
    {0x35, k_teon,       sizeof(k_teon),       0},
    {0x29, nullptr,      0,                    0},  // Display ON
    {0x21, nullptr,      0,                    0},  // Display Inversion ON
};

// ============================================================================
// Hardware bring-up correction (confirmed on real 4.0" ST7796S+GT911 panel)
// ============================================================================
// The vendor's MADCTL=0x48 above bakes in MX=1 as this panel's native
// "book" orientation (MY=0) -- unlike the ST7789-style convention where
// MX=0 is "normal". Our rotation tables/esp_lvgl_port both assume the
// ST7789-style convention (request mirror_x=false for "no rotation"), so
// we still need a correction layer -- just for MX now, not MX+MY (an
// earlier attempt inverting both was based on pre-0xB6-fix testing, where
// the missing gate-count command was confounding the real orientation).
//
// IMPORTANT: esp_lvgl_port calls esp_lcd_panel_mirror()/swap_xy() directly
// on the raw esp_lcd_panel_handle_t whenever LVGL's rotation changes — it
// does NOT go through ST7796Display::setRotation(). So the correction
// cannot live in our C++ rotation table alone (a first attempt doing that
// was silently overwritten the moment esp_lvgl_port took over the
// display). Instead, wrap the real ST7796S panel returned by
// esp_lcd_new_panel_st7796() in a thin esp_lcd_panel_t shim that inverts
// the mirror_x bit on every call, transparently, no matter who calls it
// (our code or esp_lvgl_port). Everything else (reset, init, draw_bitmap,
// invert_color, swap_xy, set_gap, disp_on_off) forwards unchanged to the
// real panel.
//
// setPrismMode() writes MADCTL directly via esp_lcd_panel_io_tx_param(),
// bypassing the esp_lcd_panel_t abstraction (and therefore this shim), so
// it applies the same MX correction explicitly — see below.
typedef struct {
    esp_lcd_panel_t base;              // must be first member
    esp_lcd_panel_handle_t real_panel; // the actual ST7796S panel
} st7796_orientation_shim_t;

static esp_err_t shim_del(esp_lcd_panel_t *panel) {
    auto *shim = reinterpret_cast<st7796_orientation_shim_t*>(panel);
    esp_err_t ret = esp_lcd_panel_del(shim->real_panel);
    free(shim);
    return ret;
}
static esp_err_t shim_reset(esp_lcd_panel_t *panel) {
    auto *shim = reinterpret_cast<st7796_orientation_shim_t*>(panel);
    return esp_lcd_panel_reset(shim->real_panel);
}
static esp_err_t shim_init(esp_lcd_panel_t *panel) {
    auto *shim = reinterpret_cast<st7796_orientation_shim_t*>(panel);
    return esp_lcd_panel_init(shim->real_panel);
}
static esp_err_t shim_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start,
                                  int x_end, int y_end, const void *color_data) {
    auto *shim = reinterpret_cast<st7796_orientation_shim_t*>(panel);
    return esp_lcd_panel_draw_bitmap(shim->real_panel, x_start, y_start, x_end, y_end, color_data);
}
static esp_err_t shim_invert_color(esp_lcd_panel_t *panel, bool invert_color_data) {
    auto *shim = reinterpret_cast<st7796_orientation_shim_t*>(panel);
    return esp_lcd_panel_invert_color(shim->real_panel, invert_color_data);
}
static esp_err_t shim_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y) {
    auto *shim = reinterpret_cast<st7796_orientation_shim_t*>(panel);
    // Physical mounting correction: invert MX only (see note above) --
    // MY is passed through unchanged, unlike the earlier MX+MY attempt.
    return esp_lcd_panel_mirror(shim->real_panel, !mirror_x, mirror_y);
}
static esp_err_t shim_swap_xy(esp_lcd_panel_t *panel, bool swap_axes) {
    auto *shim = reinterpret_cast<st7796_orientation_shim_t*>(panel);
    return esp_lcd_panel_swap_xy(shim->real_panel, swap_axes);  // unaffected by the MX correction
}
static esp_err_t shim_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap) {
    auto *shim = reinterpret_cast<st7796_orientation_shim_t*>(panel);
    return esp_lcd_panel_set_gap(shim->real_panel, x_gap, y_gap);
}
static esp_err_t shim_disp_on_off(esp_lcd_panel_t *panel, bool on_off) {
    auto *shim = reinterpret_cast<st7796_orientation_shim_t*>(panel);
    return esp_lcd_panel_disp_on_off(shim->real_panel, on_off);
}

/** Wrap a freshly-created ST7796S panel handle with the orientation shim. */
static esp_err_t wrap_with_orientation_shim(esp_lcd_panel_handle_t real_panel, esp_lcd_panel_handle_t *out_shim) {
    auto *shim = (st7796_orientation_shim_t*)calloc(1, sizeof(st7796_orientation_shim_t));
    if (!shim) {
        return ESP_ERR_NO_MEM;
    }
    shim->real_panel = real_panel;
    shim->base.del = shim_del;
    shim->base.reset = shim_reset;
    shim->base.init = shim_init;
    shim->base.draw_bitmap = shim_draw_bitmap;
    shim->base.invert_color = shim_invert_color;
    shim->base.mirror = shim_mirror;
    shim->base.swap_xy = shim_swap_xy;
    shim->base.set_gap = shim_set_gap;
    shim->base.disp_on_off = shim_disp_on_off;
    *out_shim = &shim->base;
    return ESP_OK;
}

// ============================================================================
// Singleton Implementation
// ============================================================================

ST7796Display& ST7796Display::instance() {
    static ST7796Display s_instance;
    return s_instance;
}

ST7796Display::~ST7796Display() {
    if (m_initialized) {
        end();
    }
}

cube32_result_t ST7796Display::begin() {
    cube32_st7796_config_t config = CUBE32_ST7796_CONFIG_DEFAULT();
    return begin(config);
}

cube32_result_t ST7796Display::begin(const cube32_st7796_config_t& config) {
    if (m_initialized) {
        ESP_LOGW(TAG, "ST7796S display already initialized");
        return CUBE32_ALREADY_INITIALIZED;
    }

    // Store configuration
    m_config = config;

    ESP_LOGI(TAG, "Initializing ST7796S display...");
    ESP_LOGI(TAG, "  Resolution: %dx%d", config.h_res, config.v_res);
    ESP_LOGI(TAG, "  CS: GPIO%d, DC: GPIO%d, RST: GPIO%d, BL: GPIO%d",
             config.cs_pin, config.dc_pin, config.rst_pin, config.bl_pin);

    // Ensure SPI bus is initialized
    if (!SPIBus::instance().isInitialized()) {
        ESP_LOGI(TAG, "Initializing SPI bus...");
        cube32_result_t ret = SPIBus::instance().init();
        if (ret != CUBE32_OK) {
            ESP_LOGE(TAG, "Failed to initialize SPI bus: %d", ret);
            return ret;
        }
    }

    cube32_result_t ret;

    // Initialize backlight GPIO
    ret = initBacklight();
    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to initialize backlight");
        return ret;
    }

    // Initialize reset GPIO and perform hardware reset
    ret = initReset();
    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to initialize reset");
        return ret;
    }

    // Initialize panel IO
    ret = initPanelIO();
    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to initialize panel IO");
        return ret;
    }

    // Initialize panel
    ret = initPanel();
    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to initialize panel");
        return ret;
    }

    m_initialized = true;
    ESP_LOGI(TAG, "ST7796S display initialized successfully");

    // Note: Prism mode is NOT applied here — see ST7789Display::begin() for
    // the same rationale (LVGL/esp_lvgl_port handles rotation on its own).

    // Note: Display panel and backlight are kept OFF here to prevent white
    // flash during boot, mirroring ST7789Display. The first successful draw
    // call (fillRect/drawPixel/drawBitmap) auto-calls displayOn().

    return CUBE32_OK;
}

cube32_result_t ST7796Display::initBacklight() {
    if (m_config.bl_pin < 0) {
        ESP_LOGI(TAG, "Backlight pin not configured, skipping");
        return CUBE32_OK;
    }

    gpio_config_t bl_gpio_config = {
        .pin_bit_mask = (1ULL << m_config.bl_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t ret = gpio_config(&bl_gpio_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure backlight GPIO: %s", esp_err_to_name(ret));
        return esp_err_to_cube32(ret);
    }

    // Start with backlight off
    gpio_set_level((gpio_num_t)m_config.bl_pin, !m_config.bl_on_level);

    return CUBE32_OK;
}

cube32_result_t ST7796Display::initReset() {
    if (m_config.rst_pin < 0) {
        ESP_LOGI(TAG, "Reset pin not configured, skipping hardware reset");
        return CUBE32_OK;
    }

    gpio_config_t rst_gpio_config = {
        .pin_bit_mask = (1ULL << m_config.rst_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t ret = gpio_config(&rst_gpio_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure reset GPIO: %s", esp_err_to_name(ret));
        return esp_err_to_cube32(ret);
    }

    // Perform hardware reset
    ESP_LOGI(TAG, "Performing hardware reset...");
    gpio_set_level((gpio_num_t)m_config.rst_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level((gpio_num_t)m_config.rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(120));

    return CUBE32_OK;
}

cube32_result_t ST7796Display::initPanelIO() {
    spi_host_device_t spi_host = SPIBus::instance().getHost();

    SemaphoreHandle_t trans_sem = xSemaphoreCreateBinary();
    if (!trans_sem) {
        ESP_LOGE(TAG, "Failed to create color-transfer semaphore");
        return CUBE32_NO_MEM;
    }
    m_trans_sem = trans_sem;

    esp_lcd_panel_io_spi_config_t io_config = {
        .cs_gpio_num = m_config.cs_pin,
        .dc_gpio_num = m_config.dc_pin,
        .spi_mode = 0,
        .pclk_hz = m_config.pixel_clock_hz,
        // Direct drawing waits for each transfer. A queue depth of one keeps
        // CASET/RASET/RAMWR strictly serialized while the panel is under
        // bring-up and eliminates queued-transfer ordering as a variable.
        .trans_queue_depth = 1,
        .on_color_trans_done = onColorTransDone,
        .user_ctx = this,
        .lcd_cmd_bits = CUBE32_ST7796_LCD_CMD_BITS,
        .lcd_param_bits = CUBE32_ST7796_LCD_PARAM_BITS,
        .flags = {
            .dc_low_on_data = 0,
            .octal_mode = 0,
            .quad_mode = 0,
            .sio_mode = 0,
            .lsb_first = 0,
            .cs_high_active = 0,
        },
    };

    esp_err_t ret = esp_lcd_new_panel_io_spi(spi_host, &io_config, &m_io_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create panel IO: %s", esp_err_to_name(ret));
        vSemaphoreDelete(trans_sem);
        m_trans_sem = nullptr;
        return esp_err_to_cube32(ret);
    }

    return CUBE32_OK;
}

bool IRAM_ATTR ST7796Display::onColorTransDone(esp_lcd_panel_io_handle_t /*io*/,
                                                esp_lcd_panel_io_event_data_t* /*edata*/,
                                                void* user_ctx) {
    auto* self = static_cast<ST7796Display*>(user_ctx);
    if (!self || !self->m_trans_sem) {
        return false;
    }

    BaseType_t high_task_awoken = pdFALSE;
    xSemaphoreGiveFromISR(static_cast<SemaphoreHandle_t>(self->m_trans_sem),
                          &high_task_awoken);
    return high_task_awoken == pdTRUE;
}

esp_err_t ST7796Display::drawBitmapAndWait(uint16_t x_start, uint16_t y_start,
                                           uint16_t x_end, uint16_t y_end,
                                           const void* data) {
    SemaphoreHandle_t trans_sem = static_cast<SemaphoreHandle_t>(m_trans_sem);
    if (!trans_sem) {
        return ESP_ERR_INVALID_STATE;
    }

    // Clear a completion left by an earlier transaction BEFORE submitting this
    // one. Clearing it after submitting was the prior bug: a fast DMA
    // completion could be discarded, leaving the task blocked forever.
    xSemaphoreTake(trans_sem, 0);

    esp_err_t ret = esp_lcd_panel_draw_bitmap(m_panel_handle, x_start, y_start,
                                              x_end, y_end, data);
    if (ret != ESP_OK) {
        return ret;
    }

    xSemaphoreTake(trans_sem, portMAX_DELAY);
    return ESP_OK;
}

cube32_result_t ST7796Display::initPanel() {
    st7796_vendor_config_t vendor_config = {};
    vendor_config.init_cmds = s_vendor_init_cmds;
    vendor_config.init_cmds_size = sizeof(s_vendor_init_cmds) / sizeof(s_vendor_init_cmds[0]);

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = m_config.rst_pin,
        .rgb_ele_order = m_config.bgr_order ? LCD_RGB_ELEMENT_ORDER_BGR : LCD_RGB_ELEMENT_ORDER_RGB,
        .data_endian = LCD_RGB_DATA_ENDIAN_LITTLE,
        .bits_per_pixel = CUBE32_ST7796_LCD_BIT_DEPTH,
        .flags = {
            .reset_active_high = 0,
        },
        .vendor_config = &vendor_config,  // exact vendor init sequence (see s_vendor_init_cmds above)
    };

    esp_lcd_panel_handle_t real_panel = nullptr;
    esp_err_t ret = esp_lcd_new_panel_st7796(m_io_handle, &panel_config, &real_panel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create ST7796S panel: %s", esp_err_to_name(ret));
        return esp_err_to_cube32(ret);
    }

    // Wrap the real panel with the orientation-correction shim (see note
    // above) so every mirror() call — ours or esp_lvgl_port's — is
    // transparently corrected for this panel's MX mounting offset.
    ret = wrap_with_orientation_shim(real_panel, &m_panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create orientation shim: %s", esp_err_to_name(ret));
        esp_lcd_panel_del(real_panel);
        return esp_err_to_cube32(ret);
    }

    // Reset panel
    ret = esp_lcd_panel_reset(m_panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset panel: %s", esp_err_to_name(ret));
        return esp_err_to_cube32(ret);
    }

    // Initialize panel
    ret = esp_lcd_panel_init(m_panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize panel: %s", esp_err_to_name(ret));
        return esp_err_to_cube32(ret);
    }

    // Calculate mirror/swap settings based on rotation. The vendor B6 setup
    // configures 320 active gates, but the controller retains a 320x480 GRAM
    // address axis. 90 and 180 degrees therefore require a 160-pixel offset
    // on the axis that is mapped from that 480-row GRAM dimension.
    bool mirror_x = m_config.mirror_x;
    bool mirror_y = m_config.mirror_y;
    bool swap_xy = m_config.swap_xy;
    uint16_t x_gap = m_config.x_gap;
    uint16_t y_gap = m_config.y_gap;

    switch (m_config.rotation) {
        case 0:
            mirror_x = false;
            mirror_y = false;
            swap_xy = false;
            break;
        case 90:
            mirror_x = false;
            mirror_y = true;
            swap_xy = true;
            break;
        case 180:
            mirror_x = true;
            mirror_y = true;
            swap_xy = false;
            break;
        case 270:
            mirror_x = true;
            mirror_y = false;
            swap_xy = true;
            break;
        default:
            ESP_LOGW(TAG, "Invalid rotation %d, using 0", m_config.rotation);
            break;
    }

    if (m_config.rotation == 90) {
        x_gap += 160;
    } else if (m_config.rotation == 180) {
        y_gap += 160;
    }

    // Update effective dimensions based on swap_xy
    if (swap_xy) {
        m_effective_width = m_config.v_res;
        m_effective_height = m_config.h_res;
    } else {
        m_effective_width = m_config.h_res;
        m_effective_height = m_config.v_res;
    }

    ESP_LOGI(TAG, "  Rotation: %d (mirror_x=%d, mirror_y=%d, swap_xy=%d, x_gap=%d, y_gap=%d)",
             m_config.rotation, mirror_x, mirror_y, swap_xy, x_gap, y_gap);
    ESP_LOGI(TAG, "  Effective resolution: %dx%d", m_effective_width, m_effective_height);

    // Configure panel orientation
    ret = esp_lcd_panel_mirror(m_panel_handle, mirror_x, mirror_y);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set mirror: %s", esp_err_to_name(ret));
    }

    ret = esp_lcd_panel_swap_xy(m_panel_handle, swap_xy);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set swap_xy: %s", esp_err_to_name(ret));
    }

    ret = esp_lcd_panel_set_gap(m_panel_handle, x_gap, y_gap);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set gap: %s", esp_err_to_name(ret));
    }

    // Invert colors if needed (some panels need this — verify at bring-up)
    ret = esp_lcd_panel_invert_color(m_panel_handle, m_config.invert_color);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set invert color: %s", esp_err_to_name(ret));
    }

    // Note: Display panel is kept OFF here to prevent white flash during boot
    // The LVGL driver (or autoDisplayOn()) will turn it on after the first frame.

    return CUBE32_OK;
}

cube32_result_t ST7796Display::end() {
    if (!m_initialized) {
        return CUBE32_NOT_INITIALIZED;
    }

    // Turn off backlight
    setBacklight(0);

    // Delete panel
    if (m_panel_handle) {
        esp_lcd_panel_del(m_panel_handle);
        m_panel_handle = nullptr;
    }

    // Delete panel IO
    if (m_io_handle) {
        esp_lcd_panel_io_del(m_io_handle);
        m_io_handle = nullptr;
    }

    if (m_trans_sem) {
        vSemaphoreDelete(static_cast<SemaphoreHandle_t>(m_trans_sem));
        m_trans_sem = nullptr;
    }

    m_initialized = false;
    ESP_LOGI(TAG, "ST7796S display deinitialized");
    return CUBE32_OK;
}

cube32_result_t ST7796Display::displayOn() {
    if (!m_initialized || !m_panel_handle) {
        return CUBE32_NOT_INITIALIZED;
    }

    // NOTE: Board-level backlight/power-rail control (e.g. via PMU or an
    // IO expander) is not implemented for this display yet — only the
    // generic panel on/off command is issued. See file header note.
    esp_err_t ret = esp_lcd_panel_disp_on_off(m_panel_handle, true);
    if (ret != ESP_OK) {
        return esp_err_to_cube32(ret);
    }

    return CUBE32_OK;
}

cube32_result_t ST7796Display::displayOff() {
    if (!m_initialized || !m_panel_handle) {
        return CUBE32_NOT_INITIALIZED;
    }

    // See displayOn() note — no PMU/backlight-rail control for this board yet.
    esp_err_t ret = esp_lcd_panel_disp_on_off(m_panel_handle, false);
    return esp_err_to_cube32(ret);
}

cube32_result_t ST7796Display::setBacklight(uint8_t brightness_percent) {
    if (m_config.bl_pin < 0) {
        return CUBE32_OK; // No backlight pin configured
    }

    // Simple on/off control - for PWM control, use LEDC driver.
    // No PMU/power-rail control for this board yet — see file header note.
    int level = (brightness_percent > 0) ? m_config.bl_on_level : !m_config.bl_on_level;
    gpio_set_level((gpio_num_t)m_config.bl_pin, level);

    return CUBE32_OK;
}

cube32_result_t ST7796Display::setRotation(uint16_t rotation) {
    if (!m_initialized || !m_panel_handle) {
        return CUBE32_NOT_INITIALIZED;
    }

    bool mirror_x = false;
    bool mirror_y = false;
    bool swap_xy = false;

    switch (rotation) {
        case 0:
            mirror_x = false;
            mirror_y = false;
            swap_xy = false;
            ESP_LOGI(TAG, "==> Rotate 0");
            break;
        case 90:
            mirror_x = false;
            mirror_y = true;
            swap_xy = true;
            ESP_LOGI(TAG, "==> Rotate 90");
            break;
        case 180:
            mirror_x = true;
            mirror_y = true;
            swap_xy = false;
            ESP_LOGI(TAG, "==> Rotate 180");
            break;
        case 270:
            mirror_x = true;
            mirror_y = false;
            swap_xy = true;
            ESP_LOGI(TAG, "==> Rotate 270");
            break;
        default:
            ESP_LOGW(TAG, "Invalid rotation: %d", rotation);
            return CUBE32_INVALID_ARG;
    }

    // Update effective dimensions based on swap_xy
    if (swap_xy) {
        m_effective_width = m_config.v_res;
        m_effective_height = m_config.h_res;
    } else {
        m_effective_width = m_config.h_res;
        m_effective_height = m_config.v_res;
    }

    ESP_LOGI(TAG, "setRotation: %d (mirror_x=%d, mirror_y=%d, swap_xy=%d)",
             rotation, mirror_x, mirror_y, swap_xy);

    esp_err_t ret = esp_lcd_panel_mirror(m_panel_handle, mirror_x, mirror_y);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set mirror: %s", esp_err_to_name(ret));
        return esp_err_to_cube32(ret);
    }

    ret = esp_lcd_panel_swap_xy(m_panel_handle, swap_xy);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set swap_xy: %s", esp_err_to_name(ret));
        return esp_err_to_cube32(ret);
    }

    // The panel exposes a 320x320 viewport from a controller with a 480-row
    // GRAM axis. 90° maps that axis to X; 180° maps it to Y.
    uint16_t x_gap = m_config.x_gap;
    uint16_t y_gap = m_config.y_gap;
    if (rotation == 90) {
        x_gap += 160;
    } else if (rotation == 180) {
        y_gap += 160;
    }
    ret = esp_lcd_panel_set_gap(m_panel_handle, x_gap, y_gap);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set gap: %s", esp_err_to_name(ret));
        return esp_err_to_cube32(ret);
    }

    m_config.rotation = rotation;
    return CUBE32_OK;
}

cube32_result_t ST7796Display::setPrismMode(bool enable, uint16_t effective_rotation) {
    if (!m_initialized || !m_io_handle) {
        return CUBE32_NOT_INITIALIZED;
    }

    // Prism mode toggles the horizontal mirror (MX bit) by directly writing to
    // MADCTL, same register/bit layout as ST7789 (standard ILI-style command
    // set). See ST7789Display::setPrismMode() for the full bit-layout comment.
    //
    // NOTE: this writes MADCTL directly via esp_lcd_panel_io_tx_param(),
    // bypassing the esp_lcd_panel_t abstraction (and therefore the
    // orientation-correction shim wrapped around it in initPanel() — see
    // the shim note above), so the panel's MX mounting correction (bit
    // invert) is applied explicitly below instead.
    uint16_t rotation = (effective_rotation != 0xFFFF) ? effective_rotation : m_config.rotation;

    uint8_t madctl = ST7789_MADCTL_BGR;  // Base with BGR color order

    switch (rotation) {
        case 0:
            madctl = ST7789_MADCTL_BGR;
            if (enable) {
                madctl |= ST7789_MADCTL_MX;
            }
            break;
        case 90:
            madctl = ST7789_MADCTL_MY | ST7789_MADCTL_MV | ST7789_MADCTL_BGR;
            if (enable) {
                madctl |= ST7789_MADCTL_MX;
            }
            break;
        case 180:
            madctl = ST7789_MADCTL_MX | ST7789_MADCTL_MY | ST7789_MADCTL_BGR;
            if (enable) {
                madctl &= ~ST7789_MADCTL_MX;
            }
            break;
        case 270:
            madctl = ST7789_MADCTL_MX | ST7789_MADCTL_MV | ST7789_MADCTL_BGR;
            if (enable) {
                madctl &= ~ST7789_MADCTL_MX;
            }
            break;
        default:
            ESP_LOGW(TAG, "Unknown rotation %d for prism mode", rotation);
            break;
    }

    // Apply the same physical mounting correction as the shim (invert MX only, see note above)
    madctl ^= ST7789_MADCTL_MX;

    esp_err_t ret = esp_lcd_panel_io_tx_param(m_io_handle, ST7789_MADCTL, &madctl, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write MADCTL register: %s", esp_err_to_name(ret));
        return esp_err_to_cube32(ret);
    }

    m_prism_mode = enable;
    ESP_LOGI(TAG, "Prism mode %s (MADCTL=0x%02X, rotation=%d)",
             enable ? "ENABLED" : "DISABLED", madctl, rotation);

    return CUBE32_OK;
}

cube32_result_t ST7796Display::clear(uint16_t color) {
    return fillRect(0, 0, m_effective_width, m_effective_height, color);
}

cube32_result_t ST7796Display::drawPixel(uint16_t x, uint16_t y, uint16_t color) {
    if (!m_initialized || !m_panel_handle) {
        return CUBE32_NOT_INITIALIZED;
    }

    if (x >= m_effective_width || y >= m_effective_height) {
        return CUBE32_INVALID_ARG;
    }

    // ST7796 RAMWR consumes RGB565 most-significant byte first, whereas an
    // ESP32 stores a uint16_t least-significant byte first. The managed
    // esp_lcd_st7796 component forwards color data unchanged, so convert the
    // public RGB565 value to the panel's wire-byte order here.
    const uint16_t wire_color = __builtin_bswap16(color);
    esp_err_t ret = drawBitmapAndWait(x, y, x + 1, y + 1, &wire_color);
    if (ret == ESP_OK) autoDisplayOn();
    return esp_err_to_cube32(ret);
}

cube32_result_t ST7796Display::fillRect(uint16_t x_start, uint16_t y_start,
                                         uint16_t x_end, uint16_t y_end,
                                         uint16_t color) {
    if (!m_initialized || !m_panel_handle) {
        return CUBE32_NOT_INITIALIZED;
    }

    // Clamp coordinates
    if (x_end > m_effective_width) x_end = m_effective_width;
    if (y_end > m_effective_height) y_end = m_effective_height;
    if (x_start >= x_end || y_start >= y_end) {
        return CUBE32_INVALID_ARG;
    }

    uint16_t width = x_end - x_start;
    uint16_t height = y_end - y_start;

    // Use a fixed-size strip buffer to avoid large contiguous DMA allocations.
    // A full 320x320x2 = 204800 byte single allocation can fail; strips are safe.
    const size_t MAX_STRIP_BYTES = 8192;
    uint16_t strip_rows = (uint16_t)(MAX_STRIP_BYTES / (width * sizeof(uint16_t)));
    if (strip_rows == 0) strip_rows = 1;
    if (strip_rows > height) strip_rows = height;

    size_t strip_size = width * strip_rows * sizeof(uint16_t);
    uint16_t* buffer = (uint16_t*)heap_caps_malloc(strip_size, MALLOC_CAP_DMA);
    if (!buffer) {
        ESP_LOGE(TAG, "Failed to allocate fill buffer (%u bytes)", (unsigned int)strip_size);
        return CUBE32_NO_MEM;
    }

    // The panel needs RGB565 bytes in MSB-first order. See drawPixel().
    const uint16_t wire_color = __builtin_bswap16(color);

    // Pre-fill strip buffer with the panel wire representation of color.
    for (size_t i = 0; i < (size_t)(width * strip_rows); i++) {
        buffer[i] = wire_color;
    }

    esp_err_t ret = ESP_OK;
    for (uint16_t y = y_start; y < y_end; y += strip_rows) {
        uint16_t y_strip_end = y + strip_rows;
        if (y_strip_end > y_end) y_strip_end = y_end;
        ret = drawBitmapAndWait(x_start, y, x_end, y_strip_end, buffer);
        if (ret != ESP_OK) break;
    }

    heap_caps_free(buffer);
    if (ret == ESP_OK) autoDisplayOn();
    return esp_err_to_cube32(ret);
}

cube32_result_t ST7796Display::drawBitmap(uint16_t x_start, uint16_t y_start,
                                           uint16_t x_end, uint16_t y_end,
                                           const void* data) {
    if (!m_initialized || !m_panel_handle) {
        return CUBE32_NOT_INITIALIZED;
    }

    if (!data) {
        return CUBE32_INVALID_ARG;
    }

    if (x_start >= x_end || y_start >= y_end ||
        x_end > m_effective_width || y_end > m_effective_height) {
        return CUBE32_INVALID_ARG;
    }

    const uint16_t width = x_end - x_start;
    const uint16_t height = y_end - y_start;
    const size_t max_strip_bytes = 8192;
    uint16_t strip_rows = (uint16_t)(max_strip_bytes / (width * sizeof(uint16_t)));
    if (strip_rows == 0) {
        strip_rows = 1;
    }
    if (strip_rows > height) {
        strip_rows = height;
    }

    uint16_t* wire_buffer = static_cast<uint16_t*>(heap_caps_malloc(
        width * strip_rows * sizeof(uint16_t), MALLOC_CAP_DMA));
    if (!wire_buffer) {
        ESP_LOGE(TAG, "Failed to allocate bitmap conversion buffer");
        return CUBE32_NO_MEM;
    }

    const uint16_t* source = static_cast<const uint16_t*>(data);
    esp_err_t ret = ESP_OK;
    for (uint16_t row = 0; row < height; row += strip_rows) {
        const uint16_t rows = (row + strip_rows <= height) ? strip_rows : height - row;
        const size_t pixels = (size_t)width * rows;
        for (size_t i = 0; i < pixels; ++i) {
            wire_buffer[i] = __builtin_bswap16(source[(size_t)row * width + i]);
        }

        ret = drawBitmapAndWait(x_start, y_start + row, x_end,
                                y_start + row + rows, wire_buffer);
        if (ret != ESP_OK) {
            break;
        }
    }

    heap_caps_free(wire_buffer);
    if (ret == ESP_OK) autoDisplayOn();
    return esp_err_to_cube32(ret);
}

void ST7796Display::autoDisplayOn() {
    if (!m_display_on) {
        displayOn();
        m_display_on = true;
    }
}

} // namespace cube32
