/**
 * @file usb_input.h
 * @brief CUBE32 USB Host HID Input Driver
 * 
 * This driver provides USB Host support for HID keyboard and mouse devices
 * on ESP32-S3. Uses ESP-IDF USB Host library with boot protocol for maximum
 * compatibility.
 * 
 * Features:
 * - USB HID Keyboard support (boot protocol)
 * - USB HID Mouse support (boot protocol)
 * - Event queues for keyboard and mouse events
 * - Thread-safe operation
 * - Status callbacks for device connection/disconnection
 * 
 * Note: USB Hubs are NOT supported. Connect keyboard/mouse directly to
 * the ESP32-S3 USB port.
 * 
 * Note: USB Input and USB Modem cannot be used at the same time as they
 * both require the USB Host controller.
 */

#ifndef CUBE32_DRIVERS_USB_USB_INPUT_H
#define CUBE32_DRIVERS_USB_USB_INPUT_H

#include "utils/common.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include <cstdint>
#include <functional>

// ============================================================================
// Constants
// ============================================================================

#define CUBE32_USB_INPUT_KEY_QUEUE_SIZE     32
#define CUBE32_USB_INPUT_MOUSE_QUEUE_SIZE   64
#define CUBE32_USB_HOST_TASK_PRIORITY       5
#define CUBE32_USB_HOST_TASK_STACK_SIZE     4096
#define CUBE32_USB_HOST_TASK_CORE           0

// HID Keyboard scan codes
#define CUBE32_HID_KEY_A            0x04
#define CUBE32_HID_KEY_Z            0x1D
#define CUBE32_HID_KEY_1            0x1E
#define CUBE32_HID_KEY_0            0x27
#define CUBE32_HID_KEY_ENTER        0x28
#define CUBE32_HID_KEY_ESC          0x29
#define CUBE32_HID_KEY_BACKSPACE    0x2A
#define CUBE32_HID_KEY_TAB          0x2B
#define CUBE32_HID_KEY_SPACE        0x2C

// Keyboard modifier bits
#define CUBE32_HID_MOD_LCTRL        0x01
#define CUBE32_HID_MOD_LSHIFT       0x02
#define CUBE32_HID_MOD_LALT         0x04
#define CUBE32_HID_MOD_LGUI         0x08
#define CUBE32_HID_MOD_RCTRL        0x10
#define CUBE32_HID_MOD_RSHIFT       0x20
#define CUBE32_HID_MOD_RALT         0x40
#define CUBE32_HID_MOD_RGUI         0x80

// Mouse button bits
#define CUBE32_MOUSE_BUTTON_LEFT    0x01
#define CUBE32_MOUSE_BUTTON_RIGHT   0x02
#define CUBE32_MOUSE_BUTTON_MIDDLE  0x04

// Mouse report profiles (for runtime format-specific handling)
#define CUBE32_MOUSE_PROFILE_UNKNOWN       0x00
#define CUBE32_MOUSE_PROFILE_BOOT_GENERIC  0x01
#define CUBE32_MOUSE_PROFILE_MINI_6B_RID1  0x02
#define CUBE32_MOUSE_PROFILE_HIRES_8B_RID2 0x03

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Types
// ============================================================================

/**
 * @brief USB Keyboard Event
 */
typedef struct {
    uint8_t scancode;       ///< HID scancode
    char character;         ///< ASCII character (0 if not printable)
    bool is_special;        ///< True for special keys (Enter, Backspace, etc.)
    uint8_t modifiers;      ///< Modifier keys state
} cube32_usb_key_event_t;

/**
 * @brief USB Mouse Event
 */
typedef struct {
    int8_t dx;              ///< X movement delta
    int8_t dy;              ///< Y movement delta
    int8_t wheel;           ///< Scroll wheel delta
    uint8_t buttons;        ///< Button state (bit 0=left, bit 1=right, bit 2=middle)
    uint8_t profile;        ///< Mouse report profile (CUBE32_MOUSE_PROFILE_*)
} cube32_usb_mouse_event_t;

/**
 * @brief USB Device Status Callback
 * 
 * Called when keyboard or mouse connection state changes.
 * 
 * @param keyboard_connected True if keyboard is connected
 * @param mouse_connected True if mouse is connected
 */
typedef void (*cube32_usb_status_callback_t)(bool keyboard_connected, bool mouse_connected);

#ifdef __cplusplus
} // extern "C"

// ============================================================================
// C++ Interface
// ============================================================================

namespace cube32 {

/**
 * @brief USB Input Driver Class (Singleton)
 * 
 * Object-oriented interface for USB HID keyboard and mouse input.
 * 
 * Usage:
 * @code
 *   // Initialize USB Input
 *   cube32::USBInput& usb = cube32::USBInput::instance();
 *   usb.begin();
 *   
 *   // Poll for keyboard events
 *   cube32_usb_key_event_t key_event;
 *   if (usb.pollKeyEvent(&key_event)) {
 *       printf("Key pressed: %c\n", key_event.character);
 *   }
 *   
 *   // Poll for mouse events
 *   cube32_usb_mouse_event_t mouse_event;
 *   if (usb.pollMouseEvent(&mouse_event)) {
 *       printf("Mouse moved: dx=%d, dy=%d\n", mouse_event.dx, mouse_event.dy);
 *   }
 * @endcode
 */
class USBInput {
public:
    /**
     * @brief Get the singleton instance
     */
    static USBInput& instance();

    /**
     * @brief Initialize the USB Input driver
     * 
     * Starts the USB Host task and begins scanning for HID devices.
     * 
     * @return CUBE32_OK on success, error code on failure
     */
    cube32_result_t begin();

    /**
     * @brief Deinitialize the USB Input driver
     * 
     * Stops the USB Host task and releases resources.
     * 
     * @return CUBE32_OK on success
     */
    cube32_result_t end();

    /**
     * @brief Check if the driver is initialized
     */
    bool isInitialized() const { return m_initialized; }

    /**
     * @brief Check if USB Host is ready
     */
    bool isReady() const;

    /**
     * @brief Check if a keyboard is connected
     */
    bool isKeyboardConnected() const;

    /**
     * @brief Check if a mouse is connected
     */
    bool isMouseConnected() const;

    // ---- Keyboard Events ----

    /**
     * @brief Get the keyboard event queue handle
     */
    QueueHandle_t getKeyQueue() const;

    /**
     * @brief Poll for a keyboard event (non-blocking)
     * 
     * @param event Pointer to store the event
     * @return true if an event was available
     */
    bool pollKeyEvent(cube32_usb_key_event_t* event);

    /**
     * @brief Wait for a keyboard event
     * 
     * @param event Pointer to store the event
     * @param timeout_ms Timeout in milliseconds (UINT32_MAX for infinite)
     * @return true if an event was received before timeout
     */
    bool waitKeyEvent(cube32_usb_key_event_t* event, uint32_t timeout_ms);

    // ---- Mouse Events ----

    /**
     * @brief Get the mouse event queue handle
     */
    QueueHandle_t getMouseQueue() const;

    /**
     * @brief Poll for a mouse event (non-blocking)
     * 
     * @param event Pointer to store the event
     * @return true if an event was available
     */
    bool pollMouseEvent(cube32_usb_mouse_event_t* event);

    /**
     * @brief Wait for a mouse event
     * 
     * @param event Pointer to store the event
     * @param timeout_ms Timeout in milliseconds (UINT32_MAX for infinite)
     * @return true if an event was received before timeout
     */
    bool waitMouseEvent(cube32_usb_mouse_event_t* event, uint32_t timeout_ms);

    // ---- Callbacks ----

    /**
     * @brief Set device status callback
     * 
     * @param callback Callback function, or nullptr to disable
     */
    void setStatusCallback(cube32_usb_status_callback_t callback);

    // Singleton - no copy/move
    USBInput(const USBInput&) = delete;
    USBInput& operator=(const USBInput&) = delete;

private:
    USBInput() = default;
    ~USBInput();

    bool m_initialized = false;
};

} // namespace cube32

#endif // __cplusplus

#endif // CUBE32_DRIVERS_USB_USB_INPUT_H
