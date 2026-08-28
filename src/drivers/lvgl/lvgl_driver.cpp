/**
 * @file lvgl_driver.cpp
 * @brief CUBE32 LVGL Display Driver Implementation
 */

#include "drivers/lvgl/lvgl_driver.h"
#include "drivers/pmu/axp2101.h"
#include "utils/hw_manifest.h"
#include <esp_log.h>
#include <esp_heap_caps.h>
#include <esp_lcd_panel_ops.h>

static const char* TAG = "cube32_lvgl";

namespace cube32 {

// ============================================================================
// Active display dispatch helpers
// ============================================================================
// LvglDisplay supports both ST7789Display and ST7796Display, which expose
// identical public method signatures (duck-typed, no shared base class -
// see st7796.h file header). These helpers route to whichever one is
// active, based on m_active_display_ic (resolved from the hardware
// manifest at the top of begin()).

bool LvglDisplay::activeIsInitialized() const {
    return (m_active_display_ic == CUBE32_DISPLAY_IC_ST7796)
        ? ST7796Display::instance().isInitialized()
        : ST7789Display::instance().isInitialized();
}

esp_lcd_panel_handle_t LvglDisplay::activePanelHandle() const {
    return (m_active_display_ic == CUBE32_DISPLAY_IC_ST7796)
        ? ST7796Display::instance().getPanelHandle()
        : ST7789Display::instance().getPanelHandle();
}

esp_lcd_panel_io_handle_t LvglDisplay::activeIOHandle() const {
    return (m_active_display_ic == CUBE32_DISPLAY_IC_ST7796)
        ? ST7796Display::instance().getIOHandle()
        : ST7789Display::instance().getIOHandle();
}

uint16_t LvglDisplay::activeBaseWidth() const {
    return (m_active_display_ic == CUBE32_DISPLAY_IC_ST7796)
        ? ST7796Display::instance().getBaseWidth()
        : ST7789Display::instance().getBaseWidth();
}

uint16_t LvglDisplay::activeBaseHeight() const {
    return (m_active_display_ic == CUBE32_DISPLAY_IC_ST7796)
        ? ST7796Display::instance().getBaseHeight()
        : ST7789Display::instance().getBaseHeight();
}

uint16_t LvglDisplay::activeWidth() const {
    return (m_active_display_ic == CUBE32_DISPLAY_IC_ST7796)
        ? ST7796Display::instance().getWidth()
        : ST7789Display::instance().getWidth();
}

uint16_t LvglDisplay::activeHeight() const {
    return (m_active_display_ic == CUBE32_DISPLAY_IC_ST7796)
        ? ST7796Display::instance().getHeight()
        : ST7789Display::instance().getHeight();
}

uint16_t LvglDisplay::activeRotation() const {
    return (m_active_display_ic == CUBE32_DISPLAY_IC_ST7796)
        ? ST7796Display::instance().getRotation()
        : ST7789Display::instance().getRotation();
}

cube32_result_t LvglDisplay::activeSetRotation(uint16_t rotation) {
    return (m_active_display_ic == CUBE32_DISPLAY_IC_ST7796)
        ? ST7796Display::instance().setRotation(rotation)
        : ST7789Display::instance().setRotation(rotation);
}

// ============================================================================
// Singleton Implementation
// ============================================================================

LvglDisplay& LvglDisplay::instance() {
    static LvglDisplay s_instance;
    return s_instance;
}

LvglDisplay::~LvglDisplay() {
    if (m_initialized) {
        end();
    }
}

cube32_result_t LvglDisplay::begin() {
    cube32_lvgl_config_t config = CUBE32_LVGL_CONFIG_DEFAULT();
    return begin(config);
}

cube32_result_t LvglDisplay::begin(const cube32_lvgl_config_t& config) {
    if (m_initialized) {
        ESP_LOGW(TAG, "LVGL display already initialized");
        return CUBE32_ALREADY_INITIALIZED;
    }

    // Resolve which concrete display driver is active (ST7789 or ST7796)
    // from the hardware manifest, populated during hw_manifest_scan().
    cube32_hw_manifest_t* hw = cube32_hw_manifest();
    m_active_display_ic = hw->display_ic ? hw->display_ic : CUBE32_DISPLAY_IC_ST7789;

    // Check if the active display driver is initialized
    if (!activeIsInitialized()) {
        ESP_LOGE(TAG, "Display driver must be initialized before LVGL");
        return CUBE32_NOT_INITIALIZED;
    }

    // Store configuration - use base dimensions
    m_config = config;
    m_width = activeBaseWidth();
    m_height = activeBaseHeight();

    // Get the configured rotation from the active display
    uint16_t configured_rotation = activeRotation();

    ESP_LOGI(TAG, "Initializing LVGL display...");
    ESP_LOGI(TAG, "  Base resolution: %dx%d", m_width, m_height);
    ESP_LOGI(TAG, "  Configured rotation: %d", configured_rotation);
    ESP_LOGI(TAG, "  Double buffer: %s", config.double_buffer ? "yes" : "no");
    ESP_LOGI(TAG, "  Use SPIRAM: %s", config.use_spiram ? "yes" : "no");

    // Reset the display to rotation 0 - LVGL will handle rotation
    if (configured_rotation != 0) {
        ESP_LOGI(TAG, "Resetting display to rotation 0 (LVGL will handle rotation)");
        activeSetRotation(0);
    }

    // Initialize LVGL port (creates LVGL task and timer)
    cube32_result_t ret = initLvglPort(config);
    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to initialize LVGL port");
        return ret;
    }

    // Add display to LVGL
    ret = addDisplay(config);
    if (ret != CUBE32_OK) {
        ESP_LOGE(TAG, "Failed to add display to LVGL");
        lvgl_port_deinit();
        m_lvgl_port_initialized = false;
        return ret;
    }

    m_initialized = true;

    // Apply the configured rotation via LVGL
    if (configured_rotation != 0) {
        ESP_LOGI(TAG, "Applying initial rotation %d via LVGL", configured_rotation);
        setRotation(configured_rotation);
    }

    // Clear screen to black before turning on display to prevent white flash
    if (lock(1000)) {
        // Set screen background to black
        lv_obj_t *scr = lv_screen_active();
        lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);
        
        // Force immediate redraw to fill display buffer with black
        lv_obj_invalidate(scr);
        lv_refr_now(m_display);
        unlock();
    }

    // Now turn on the display panel (it was kept OFF during display driver init)
    esp_lcd_panel_disp_on_off(activePanelHandle(), true);

    // Now turn on the backlight via PMU (display has content, no white flash)
    // CUBE32 uses AXP2101 ALDO3 for LCD backlight control on ST7789 boards.
    // NOTE: backlight/power-rail control for the ST7796S board is not
    // implemented yet (board control logic TBD) — skipped for that board.
    if (m_active_display_ic != CUBE32_DISPLAY_IC_ST7796) {
        PMU& pmu = PMU::instance();
        if (pmu.isInitialized()) {
            pmu.setDisplayBacklight(true);
        } else {
            ESP_LOGW(TAG, "PMU not initialized, backlight may not work");
        }
    }

    ESP_LOGI(TAG, "LVGL display initialized successfully");

    return CUBE32_OK;
}

cube32_result_t LvglDisplay::initLvglPort(const cube32_lvgl_config_t& config) {
    lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    
    lvgl_cfg.task_priority = config.task_priority;
    lvgl_cfg.task_stack = config.task_stack_size;
    lvgl_cfg.task_affinity = config.task_affinity;
    lvgl_cfg.task_max_sleep_ms = config.task_max_sleep_ms;
    lvgl_cfg.timer_period_ms = config.timer_period_ms;

    // Use internal memory for task stack (required for FreeRTOS)
    lvgl_cfg.task_stack_caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_DEFAULT;

    ESP_LOGI(TAG, "Initializing LVGL port...");
    ESP_LOGI(TAG, "  Task priority: %d", lvgl_cfg.task_priority);
    ESP_LOGI(TAG, "  Task stack: %d bytes", lvgl_cfg.task_stack);
    ESP_LOGI(TAG, "  Timer period: %d ms", lvgl_cfg.timer_period_ms);

    esp_err_t ret = lvgl_port_init(&lvgl_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LVGL port: %s", esp_err_to_name(ret));
        return esp_err_to_cube32(ret);
    }

    m_lvgl_port_initialized = true;
    ESP_LOGI(TAG, "LVGL port initialized");

    return CUBE32_OK;
}

cube32_result_t LvglDisplay::addDisplay(const cube32_lvgl_config_t& config) {
    // Always use base dimensions - LVGL will handle rotation
    uint16_t base_width = activeBaseWidth();
    uint16_t base_height = activeBaseHeight();
    
    // Calculate buffer size using base dimensions
    uint32_t buffer_size = config.buffer_size;
    if (buffer_size == 0) {
        // Auto calculate: width * CUBE32_LVGL_BUFFER_LINES lines.
        // NOTE: lvgl_port_display_cfg_t::buffer_size is in PIXELS, not bytes.
        // esp_lvgl_port multiplies by color_bytes (2 for RGB565) internally.
        //
        // When USE_SPIRAM=n, buffers are allocated from internal DMA RAM
        // (buff_dma=1).  Keeping this value small is critical: the SPI master,
        // USB host, audio DMA, and FreeRTOS task stacks all compete for the
        // same limited pool.  20 lines = 9,600 bytes for a 240-wide panel.
        //
        // When USE_SPIRAM=y, buffers live in PSRAM and a larger line count
        // may be used without affecting internal RAM.
        buffer_size = (uint32_t)base_width * CONFIG_CUBE32_LVGL_BUFFER_LINES;
        ESP_LOGI(TAG, "Auto-calculated buffer size: %" PRIu32 " pixels (%d lines)",
                 buffer_size, CONFIG_CUBE32_LVGL_BUFFER_LINES);
    }

    // Configure display
    lvgl_port_display_cfg_t disp_cfg = {};
    disp_cfg.io_handle = activeIOHandle();
    disp_cfg.panel_handle = activePanelHandle();
    disp_cfg.control_handle = nullptr;  // Use panel_handle for control
    disp_cfg.buffer_size = buffer_size;
    disp_cfg.double_buffer = config.double_buffer;
    disp_cfg.trans_size = 0;  // No additional transaction buffer
    disp_cfg.hres = base_width;
    disp_cfg.vres = base_height;
    disp_cfg.monochrome = false;

    // Rotation config - use base state (no rotation)
    // LVGL will handle rotation via lv_display_set_rotation() and
    // esp_lvgl_port will update hardware via lvgl_port_disp_rotation_update()
    disp_cfg.rotation.swap_xy = false;
    disp_cfg.rotation.mirror_x = false;
    disp_cfg.rotation.mirror_y = false;

    ESP_LOGI(TAG, "  Base rotation state: swap_xy=0, mirror_x=0, mirror_y=0");

#if LVGL_VERSION_MAJOR >= 9
    disp_cfg.color_format = LV_COLOR_FORMAT_RGB565;
#endif

    // Buffer allocation flags
    disp_cfg.flags.buff_dma = !config.use_spiram;  // Use DMA if not using SPIRAM
    disp_cfg.flags.buff_spiram = config.use_spiram;
    disp_cfg.flags.sw_rotate = false;  // Use hardware rotation via ST7789
#if LVGL_VERSION_MAJOR >= 9
    // esp_lcd_st7796 forwards RAMWR data unchanged. Its controller expects
    // RGB565 MSB first, so LVGL's CPU-endian RGB565 draw buffer needs byte
    // conversion before it is sent. ST7789 boards retain their established
    // configuration from the user-provided LVGL setting.
    disp_cfg.flags.swap_bytes = config.swap_bytes ||
                                m_active_display_ic == CUBE32_DISPLAY_IC_ST7796;
#endif
    disp_cfg.flags.full_refresh = config.full_refresh;
    disp_cfg.flags.direct_mode = config.direct_mode;

    ESP_LOGI(TAG, "Adding display to LVGL...");
    ESP_LOGI(TAG, "  Buffer size: %" PRIu32 " pixels", buffer_size);
    ESP_LOGI(TAG, "  Double buffer: %s", config.double_buffer ? "yes" : "no");
    ESP_LOGI(TAG, "  DMA buffer: %s", disp_cfg.flags.buff_dma ? "yes" : "no");
    ESP_LOGI(TAG, "  SPIRAM buffer: %s", disp_cfg.flags.buff_spiram ? "yes" : "no");

    m_display = lvgl_port_add_disp(&disp_cfg);
    if (m_display == nullptr) {
        ESP_LOGE(TAG, "Failed to add display to LVGL");
        return CUBE32_NO_MEM;
    }

    // LVGL 9.5: explicitly set as default display so new screens/objects
    // bind to this display when multiple displays may exist.
    lv_display_set_default(m_display);

    ESP_LOGI(TAG, "Display added to LVGL successfully");

    return CUBE32_OK;
}

cube32_result_t LvglDisplay::end() {
    if (!m_initialized) {
        ESP_LOGW(TAG, "LVGL display not initialized");
        return CUBE32_OK;
    }

    ESP_LOGI(TAG, "Deinitializing LVGL display...");

    // Remove touch input first
    removeTouch();

    // Remove display from LVGL
    if (m_display != nullptr) {
        esp_err_t ret = lvgl_port_remove_disp(m_display);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to remove display: %s", esp_err_to_name(ret));
        }
        m_display = nullptr;
    }

    // Deinitialize LVGL port
    if (m_lvgl_port_initialized) {
        esp_err_t ret = lvgl_port_deinit();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to deinitialize LVGL port: %s", esp_err_to_name(ret));
        }
        m_lvgl_port_initialized = false;
    }

    m_initialized = false;
    ESP_LOGI(TAG, "LVGL display deinitialized");

    return CUBE32_OK;
}

// ============================================================================
// Touch Input
// ============================================================================

cube32_result_t LvglDisplay::addTouch() {
#ifdef CONFIG_CUBE32_TOUCH_ENABLED
    if (!m_initialized) {
        ESP_LOGE(TAG, "LVGL display must be initialized before adding touch");
        return CUBE32_NOT_INITIALIZED;
    }

    if (m_touch_indev != nullptr) {
        ESP_LOGW(TAG, "Touch input already added");
        return CUBE32_ALREADY_INITIALIZED;
    }

    // Check if touch is initialized
    Touch& touch = Touch::instance();
    if (!touch.isInitialized()) {
        ESP_LOGE(TAG, "Touch driver must be initialized before adding to LVGL");
        return CUBE32_NOT_INITIALIZED;
    }

    ESP_LOGI(TAG, "Adding touch input to LVGL...");

    // Configure touch for LVGL
    lvgl_port_touch_cfg_t touch_cfg = {};
    touch_cfg.disp = m_display;
    touch_cfg.handle = touch.getHandle();

    m_touch_indev = lvgl_port_add_touch(&touch_cfg);
    if (m_touch_indev == nullptr) {
        ESP_LOGE(TAG, "Failed to add touch input to LVGL");
        return CUBE32_ERROR;
    }

    ESP_LOGI(TAG, "Touch input added to LVGL successfully");
    return CUBE32_OK;
#else
    ESP_LOGW(TAG, "Touch support not enabled in configuration");
    return CUBE32_NOT_SUPPORTED;
#endif
}

cube32_result_t LvglDisplay::removeTouch() {
#ifdef CONFIG_CUBE32_TOUCH_ENABLED
    if (m_touch_indev == nullptr) {
        return CUBE32_OK;
    }

    ESP_LOGI(TAG, "Removing touch input from LVGL...");
    esp_err_t ret = lvgl_port_remove_touch(m_touch_indev);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to remove touch: %s", esp_err_to_name(ret));
    }
    m_touch_indev = nullptr;

    return CUBE32_OK;
#else
    return CUBE32_NOT_SUPPORTED;
#endif
}

// ============================================================================
// USB Mouse Input
// ============================================================================

// Static read callback for USB mouse - needs access to instance
static LvglDisplay* s_lvgl_instance = nullptr;

static void usb_mouse_read_cb(lv_indev_t *indev, lv_indev_data_t *data) {
    if (s_lvgl_instance) {
        s_lvgl_instance->processUsbMouseEvents(data);
    }
}

void LvglDisplay::processUsbMouseEvents(lv_indev_data_t *data) {
#ifdef CONFIG_CUBE32_USB_INPUT_ENABLED
    // Poll all USB mouse events and update state
    cube32_usb_mouse_event_t mouse_event;
    while (cube32::USBInput::instance().pollMouseEvent(&mouse_event)) {
        // Mouse is already associated with display via lv_indev_set_display().
        // LVGL applies display rotation internally, so do not rotate deltas here.
        // HID relative mouse: dx(+)=right, dy(+)=down in typical host interpretation.
        int16_t dx = mouse_event.dx;
        int16_t dy = mouse_event.dy;

        // Apply rotation compensation for supported mouse report profiles.
        // This matches the original rotation mapping behavior and keeps
        // cursor movement consistent across all display rotations.
        if (mouse_event.profile == CUBE32_MOUSE_PROFILE_HIRES_8B_RID2 ||
            mouse_event.profile == CUBE32_MOUSE_PROFILE_MINI_6B_RID1 ||
            mouse_event.profile == CUBE32_MOUSE_PROFILE_BOOT_GENERIC) {
            uint16_t rotation = getRotation();
            int16_t original_dx = dx;
            int16_t original_dy = dy;

            switch (rotation) {
                case 0:
                    dx = original_dx;
                    dy = original_dy;
                    break;
                case 90:
                    dx = original_dy;
                    dy = -original_dx;
                    break;
                case 180:
                    dx = -original_dx;
                    dy = -original_dy;
                    break;
                case 270:
                    dx = -original_dy;
                    dy = original_dx;
                    break;
                default:
                    break;
            }
        }

        // Filter occasional large spikes from high-DPI/report-variant mice.
        // Keep this conservative to avoid affecting small boot-protocol mice.
        constexpr int16_t kMaxDeltaPerReport = 30;
        if (dx > kMaxDeltaPerReport) dx = kMaxDeltaPerReport;
        if (dx < -kMaxDeltaPerReport) dx = -kMaxDeltaPerReport;
        if (dy > kMaxDeltaPerReport) dy = kMaxDeltaPerReport;
        if (dy < -kMaxDeltaPerReport) dy = -kMaxDeltaPerReport;

        m_usb_mouse_x += dx;
        m_usb_mouse_y += dy;
        
        m_usb_mouse_buttons = mouse_event.buttons;
        
        // Clamp to display bounds using the active display's rotated dimensions
        int16_t disp_width = activeWidth();
        int16_t disp_height = activeHeight();
        
        if (m_usb_mouse_x < 0) m_usb_mouse_x = 0;
        if (m_usb_mouse_x >= disp_width) m_usb_mouse_x = disp_width - 1;
        if (m_usb_mouse_y < 0) m_usb_mouse_y = 0;
        if (m_usb_mouse_y >= disp_height) m_usb_mouse_y = disp_height - 1;
    }
    
    data->point.x = m_usb_mouse_x;
    data->point.y = m_usb_mouse_y;
    
    // Left button = pressed state for LVGL
    if (m_usb_mouse_buttons & CUBE32_MOUSE_BUTTON_LEFT) {
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
#endif
}

cube32_result_t LvglDisplay::addUsbMouse() {
#ifdef CONFIG_CUBE32_USB_INPUT_ENABLED
    if (!m_initialized) {
        ESP_LOGE(TAG, "LVGL display must be initialized before adding USB mouse");
        return CUBE32_NOT_INITIALIZED;
    }

    if (m_usb_mouse_indev != nullptr) {
        ESP_LOGW(TAG, "USB mouse input already added");
        return CUBE32_ALREADY_INITIALIZED;
    }

    // Check if USB Input is initialized
    cube32::USBInput& usb = cube32::USBInput::instance();
    if (!usb.isInitialized()) {
        ESP_LOGE(TAG, "USB Input driver must be initialized before adding to LVGL");
        return CUBE32_NOT_INITIALIZED;
    }

    ESP_LOGI(TAG, "Adding USB mouse input to LVGL...");

    // Store instance for callback
    s_lvgl_instance = this;

    // Initialize mouse position to center of screen
    m_usb_mouse_x = activeWidth() / 2;
    m_usb_mouse_y = activeHeight() / 2;
    m_usb_mouse_buttons = 0;

    // Lock LVGL for thread-safe operations
    if (!lock(1000)) {
        ESP_LOGE(TAG, "Failed to lock LVGL for USB mouse setup");
        return CUBE32_TIMEOUT;
    }

    // Create input device for USB mouse
    m_usb_mouse_indev = lv_indev_create();
    if (m_usb_mouse_indev == nullptr) {
        ESP_LOGE(TAG, "Failed to create USB mouse input device");
        unlock();
        return CUBE32_NO_MEM;
    }

    lv_indev_set_type(m_usb_mouse_indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(m_usb_mouse_indev, usb_mouse_read_cb);
    lv_indev_set_display(m_usb_mouse_indev, m_display);

    // Create mouse cursor
    m_usb_mouse_cursor = lv_obj_create(lv_screen_active());
    lv_obj_set_size(m_usb_mouse_cursor, 12, 12);
    lv_obj_set_style_radius(m_usb_mouse_cursor, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(m_usb_mouse_cursor, lv_color_hex(0xFF0000), 0);
    lv_obj_set_style_bg_opa(m_usb_mouse_cursor, LV_OPA_70, 0);
    lv_obj_set_style_border_width(m_usb_mouse_cursor, 2, 0);
    lv_obj_set_style_border_color(m_usb_mouse_cursor, lv_color_hex(0xFFFFFF), 0);
    lv_indev_set_cursor(m_usb_mouse_indev, m_usb_mouse_cursor);

    unlock();

    ESP_LOGI(TAG, "USB mouse input added to LVGL successfully");
    return CUBE32_OK;
#else
    ESP_LOGW(TAG, "USB Input support not enabled in configuration");
    return CUBE32_NOT_SUPPORTED;
#endif
}

cube32_result_t LvglDisplay::removeUsbMouse() {
#ifdef CONFIG_CUBE32_USB_INPUT_ENABLED
    if (m_usb_mouse_indev == nullptr) {
        return CUBE32_OK;
    }

    ESP_LOGI(TAG, "Removing USB mouse input from LVGL...");

    if (lock(1000)) {
        // Remove cursor first
        if (m_usb_mouse_cursor) {
            lv_obj_delete(m_usb_mouse_cursor);
            m_usb_mouse_cursor = nullptr;
        }
        
        // Delete input device
        lv_indev_delete(m_usb_mouse_indev);
        m_usb_mouse_indev = nullptr;
        
        unlock();
    }

    s_lvgl_instance = nullptr;

    return CUBE32_OK;
#else
    return CUBE32_NOT_SUPPORTED;
#endif
}

// ============================================================================
// Thread Safety
// ============================================================================

bool LvglDisplay::lock(uint32_t timeout_ms) {
    if (!m_initialized) {
        ESP_LOGW(TAG, "Cannot lock - LVGL not initialized");
        return false;
    }
    return lvgl_port_lock(timeout_ms);
}

void LvglDisplay::unlock() {
    if (!m_initialized) {
        return;
    }
    lvgl_port_unlock();
}

// ============================================================================
// Display Control
// ============================================================================

cube32_result_t LvglDisplay::setRotation(uint16_t rotation) {
    if (!m_initialized) {
        return CUBE32_NOT_INITIALIZED;
    }

    if (!m_display) {
        return CUBE32_NOT_INITIALIZED;
    }

    // Convert degrees to LVGL rotation enum
    lv_display_rotation_t lv_rotation;
    switch (rotation) {
        case 0:
            lv_rotation = LV_DISPLAY_ROTATION_0;
            break;
        case 90:
            lv_rotation = LV_DISPLAY_ROTATION_90;
            break;
        case 180:
            lv_rotation = LV_DISPLAY_ROTATION_180;
            break;
        case 270:
            lv_rotation = LV_DISPLAY_ROTATION_270;
            break;
        default:
            ESP_LOGW(TAG, "Invalid rotation %d, using 0", rotation);
            lv_rotation = LV_DISPLAY_ROTATION_0;
            rotation = 0;
            break;
    }

    ESP_LOGI(TAG, "Setting LVGL rotation to %d degrees", rotation);

    // NOTE: Do NOT update touch rotation here!
    // When using LVGL with esp_lvgl_port, the touch driver should always report
    // raw physical coordinates (rotation 0). LVGL handles the coordinate transformation
    // internally based on the display rotation setting.
    // The touch indev is associated with the display, so LVGL knows the rotation.

    // Lock LVGL and set rotation
    // esp_lvgl_port will handle the hardware rotation via lvgl_port_disp_rotation_update()
    if (!lock(1000)) {
        ESP_LOGE(TAG, "Failed to lock LVGL for rotation");
        return CUBE32_TIMEOUT;
    }

    // Set LVGL display rotation - this triggers the callback in esp_lvgl_port
    // which updates the hardware via esp_lcd_panel_swap_xy/mirror
    lv_display_set_rotation(m_display, lv_rotation);

    // Update the gap/offset for the display.
    // esp_lvgl_port handles swap_xy and mirror, but NOT the gap setting.
    // The ST7789 240x240 panel uses an 80-pixel offset in its 320x240 GRAM.
    // This ST7796 panel's vendor setup selects 320 active gates, but its
    // controller GRAM retains a 480-row address axis. Rotation 90 maps that
    // axis to X and rotation 180 maps it to Y, requiring the corresponding
    // 160-pixel viewport offset. Rotations 0 and 270 use the origin.
    uint16_t base_width = activeBaseWidth();
    uint16_t base_height = activeBaseHeight();
    
    // Calculate offset for displays smaller than internal buffer
    uint16_t offset = 0;
    if (base_width == 240 && base_height == 240) {
        offset = 80;  // ST7789 internal is 320x240, display is 240x240
    } else if (m_active_display_ic == CUBE32_DISPLAY_IC_ST7796 &&
               base_width == 320 && base_height == 320) {
        offset = 160; // ST7796 GRAM is 320x480; the visible viewport is 320x320
    }
    
    uint16_t x_gap = 0;
    uint16_t y_gap = 0;
    switch (rotation) {
        case 0:
            x_gap = 0;
            y_gap = 0;
            break;
        case 90:
            x_gap = offset;
            y_gap = 0;
            break;
        case 180:
            x_gap = 0;
            y_gap = offset;
            break;
        case 270:
            x_gap = 0;
            y_gap = 0;
            break;
    }
    
    ESP_LOGI(TAG, "Setting gap: x_gap=%d, y_gap=%d (offset=%d)", x_gap, y_gap, offset);
    esp_lcd_panel_set_gap(activePanelHandle(), x_gap, y_gap);

    // Update cached dimensions based on rotation
    // For 90/270, width and height are swapped
    if (rotation == 90 || rotation == 270) {
        m_width = base_height;
        m_height = base_width;
    } else {
        m_width = base_width;
        m_height = base_height;
    }

    // Force full screen refresh
    lv_obj_invalidate(lv_screen_active());

    unlock();

    ESP_LOGI(TAG, "Rotation set to %d, dimensions now %dx%d", rotation, m_width, m_height);
    return CUBE32_OK;
}

uint16_t LvglDisplay::getRotation() const {
    if (!m_initialized || !m_display) {
        return 0;
    }

    lv_display_rotation_t lv_rotation = lv_display_get_rotation(m_display);
    switch (lv_rotation) {
        case LV_DISPLAY_ROTATION_0:   return 0;
        case LV_DISPLAY_ROTATION_90:  return 90;
        case LV_DISPLAY_ROTATION_180: return 180;
        case LV_DISPLAY_ROTATION_270: return 270;
        default: return 0;
    }
}

} // namespace cube32
