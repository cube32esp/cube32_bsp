/**
 * @file usb_input.cpp
 * @brief CUBE32 USB Host HID Input Driver Implementation
 * 
 * This implementation provides USB Host support for HID keyboard and mouse
 * devices on ESP32-S3. Uses ESP-IDF USB Host library with boot protocol.
 */

#include "drivers/usb/usb_input.h"
#include "cube32_config.h"

#ifdef CONFIG_CUBE32_USB_INPUT_ENABLED

#include <esp_log.h>
#include <esp_intr_alloc.h>
#include <usb/usb_types_stack.h>
#include <usb/usb_host.h>
#include <cstring>

static const char* TAG = "cube32_usb";

// ============================================================================
// Configuration
// ============================================================================

// HID Protocol types
#define HID_PROTOCOL_KEYBOARD       0x01
#define HID_PROTOCOL_MOUSE          0x02

// USB Hub class (for detection only - hubs are not supported)
#define USB_CLASS_HUB               0x09

// HID Keyboard scan codes
#define HID_KEY_ENTER               0x28
#define HID_KEY_ESC                 0x29
#define HID_KEY_BACKSPACE           0x2A
#define HID_KEY_TAB                 0x2B
#define HID_KEY_SPACE               0x2C

// Modifier bits
#define MOD_LSHIFT                  0x02
#define MOD_RSHIFT                  0x20

// ============================================================================
// Internal State
// ============================================================================

static volatile bool s_keyboard_connected = false;
static volatile bool s_mouse_connected = false;
static volatile bool s_usb_host_ready = false;
static uint8_t s_prev_keys[6] = {0};

// Event queues
static QueueHandle_t s_key_queue = nullptr;
static QueueHandle_t s_mouse_queue = nullptr;

// Status callback
static cube32_usb_status_callback_t s_status_callback = nullptr;

// USB Host handles
static usb_host_client_handle_t s_client_hdl = nullptr;
static usb_device_handle_t s_device_hdl = nullptr;

// Separate transfers for keyboard and mouse
static usb_transfer_t *s_kbd_transfer = nullptr;
static usb_transfer_t *s_mouse_transfer = nullptr;
static uint8_t s_kbd_ep_addr = 0;
static uint8_t s_mouse_ep_addr = 0;
static uint8_t s_kbd_intf_num = 0;
static uint8_t s_mouse_intf_num = 0;
static uint16_t s_kbd_mps = 8;
static uint16_t s_mouse_mps = 8;

// Previous mouse button state for click detection
static uint8_t s_prev_mouse_buttons = 0;
static uint32_t s_mouse_debug_report_count = 0;
static uint32_t s_mouse_debug_event_count = 0;

// ============================================================================
// HID Keyboard Scancode to ASCII Mapping
// ============================================================================

static const char s_hid_to_ascii_lower[128] = {
    0, 0, 0, 0, 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l',
    'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z',
    '1', '2', '3', '4', '5', '6', '7', '8', '9', '0',
    '\n', 0, '\b', '\t', ' ', '-', '=', '[', ']', '\\',
    0, ';', '\'', '`', ',', '.', '/', 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

static const char s_hid_to_ascii_upper[128] = {
    0, 0, 0, 0, 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L',
    'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z',
    '!', '@', '#', '$', '%', '^', '&', '*', '(', ')',
    '\n', 0, '\b', '\t', ' ', '_', '+', '{', '}', '|',
    0, ':', '"', '~', '<', '>', '?', 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

// ============================================================================
// Internal Helper Functions
// ============================================================================

static char hid_scancode_to_char(uint8_t scancode, bool shift) {
    if (scancode < 128) {
        return shift ? s_hid_to_ascii_upper[scancode] : s_hid_to_ascii_lower[scancode];
    }
    return 0;
}

static bool is_new_key(uint8_t scancode, const uint8_t* prev_report) {
    for (int i = 0; i < 6; i++) {
        if (prev_report[i] == scancode) return false;
    }
    return true;
}

static uint16_t get_ep_max_packet_size(const usb_ep_desc_t *ep) {
    if (!ep) {
        return 8;
    }
    // bits 10:0 contain max packet size for all endpoint speeds
    uint16_t mps = ep->wMaxPacketSize & 0x07FF;
    return (mps > 0) ? mps : 8;
}

static void notify_status_change() {
    if (s_status_callback) {
        s_status_callback(s_keyboard_connected, s_mouse_connected);
    }
}

// Process keyboard report (standard boot protocol: 8 bytes)
static void process_keyboard_report(const uint8_t* report, size_t length) {
    if (length < 8 || !s_key_queue) return;
    
    uint8_t modifier = report[0];
    bool shift = (modifier & (MOD_LSHIFT | MOD_RSHIFT)) != 0;
    
    for (int i = 2; i < 8; i++) {
        uint8_t scancode = report[i];
        if (scancode == 0 || scancode == 1) continue;
        
        if (!is_new_key(scancode, s_prev_keys)) continue;
        
        cube32_usb_key_event_t event;
        event.scancode = scancode;
        event.modifiers = modifier;
        event.is_special = false;
        event.character = 0;
        
        if (scancode == HID_KEY_ENTER) {
            event.is_special = true;
            event.character = '\n';
        } else if (scancode == HID_KEY_BACKSPACE) {
            event.is_special = true;
            event.character = '\b';
        } else if (scancode == HID_KEY_TAB) {
            event.is_special = true;
            event.character = '\t';
        } else if (scancode == HID_KEY_ESC) {
            event.is_special = true;
            event.character = 0x1B;
        } else if (scancode == HID_KEY_SPACE) {
            event.character = ' ';
        } else {
            event.character = hid_scancode_to_char(scancode, shift);
        }
        
        if (event.character != 0 || event.is_special) {
            xQueueSend(s_key_queue, &event, 0);
        }
    }
    
    memcpy(s_prev_keys, &report[2], 6);
}

// Process mouse report - handles various HID report formats
// Standard boot protocol: [buttons, dx, dy, wheel?]
// With report ID: [report_id, buttons, dx, dy, wheel?]
static void process_mouse_report(const uint8_t* report, size_t length) {
    if (length < 3 || !s_mouse_queue) return;

    s_mouse_debug_report_count++;
    if (length >= 3 && (s_mouse_debug_report_count <= 30 || (s_mouse_debug_report_count % 50) == 0)) {
        ESP_LOGI(TAG,
                 "Mouse RAW[%lu] len=%u: %02X %02X %02X %02X %02X %02X %02X %02X",
                 (unsigned long)s_mouse_debug_report_count,
                 (unsigned)length,
                 report[0],
                 report[1],
                 report[2],
                 length > 3 ? report[3] : 0,
                 length > 4 ? report[4] : 0,
                 length > 5 ? report[5] : 0,
                 length > 6 ? report[6] : 0,
                 length > 7 ? report[7] : 0);
    }
    
    cube32_usb_mouse_event_t event;
    event.buttons = 0;
    event.dx = 0;
    event.dy = 0;
    event.wheel = 0;
    event.profile = CUBE32_MOUSE_PROFILE_UNKNOWN;
    
    // Try to detect report format based on length/content.
    // Some mice use [buttons, dx, dy, wheel], others use [report_id, buttons, dx, dy, wheel].
    //
    // Some full-size mice report 8-byte packets with report_id=0x02 and split Y nibbles.
    // Empirically from logs:
    //   byte3 carries X as int8
    //   Y byte is composed from byte5 low nibble (high 4 bits) + byte4 high nibble (low 4 bits)
    // Example:
    //   02 00 00 FF 0F 00 00 00 -> dx=-1, dy=0
    //   02 00 00 00 E0 FF 00 00 -> dx=0,  dy=-2
    if (length >= 8 && report[0] == 0x02) {
        event.buttons = report[1];

        event.dx = (int8_t)report[3];
        uint8_t y_packed = (uint8_t)(((report[5] & 0x0F) << 4) | ((report[4] & 0xF0) >> 4));
        event.dy = (int8_t)y_packed;
        event.wheel = (int8_t)report[6];
        event.profile = CUBE32_MOUSE_PROFILE_HIRES_8B_RID2;
    } else if (length >= 6 && report[0] == 0x01) {
        // Mini keyboard+mouse combo observed format: [rid=1, buttons, dx, dy, wheel, ...]
        event.buttons = report[1];
        event.dx = (int8_t)report[2];
        event.dy = (int8_t)report[3];
        event.wheel = (int8_t)report[4];
        event.profile = CUBE32_MOUSE_PROFILE_MINI_6B_RID1;
    } else if (length == 3) {
        // Standard 3-byte boot protocol: [buttons, dx, dy]
        event.buttons = report[0];
        event.dx = (int8_t)report[1];
        event.dy = (int8_t)report[2];
        event.profile = CUBE32_MOUSE_PROFILE_BOOT_GENERIC;
    } else if (length == 4) {
        // Could be [buttons, dx, dy, wheel] OR [report_id, buttons, dx, dy]
        int8_t std_dx = (int8_t)report[1];
        int8_t std_dy = (int8_t)report[2];
        int8_t id_dx = (int8_t)report[2];
        int8_t id_dy = (int8_t)report[3];

        uint16_t std_mag = (uint16_t)(abs(std_dx) + abs(std_dy));
        uint16_t id_mag = (uint16_t)(abs(id_dx) + abs(id_dy));

        bool report_id_hint = (report[0] >= 0x01 && report[0] <= 0x0F);
        bool button_hint = ((report[1] & 0xE0) == 0);  // allow up to 5 button bits

        bool use_report_id_layout = (report_id_hint && button_hint && id_mag > std_mag);

        if (use_report_id_layout) {
            event.buttons = report[1];
            event.dx = id_dx;
            event.dy = id_dy;
        } else {
            event.buttons = report[0];
            event.dx = std_dx;
            event.dy = std_dy;
            event.wheel = (int8_t)report[3];
        }
        event.profile = CUBE32_MOUSE_PROFILE_BOOT_GENERIC;
    } else if (length >= 5) {
        // Extended format: choose between standard and report-id layout adaptively
        int8_t std_dx = (int8_t)report[1];
        int8_t std_dy = (int8_t)report[2];
        int8_t id_dx = (int8_t)report[2];
        int8_t id_dy = (int8_t)report[3];

        uint16_t std_mag = (uint16_t)(abs(std_dx) + abs(std_dy));
        uint16_t id_mag = (uint16_t)(abs(id_dx) + abs(id_dy));

        bool report_id_hint = (report[0] >= 0x01 && report[0] <= 0x0F);
        bool button_hint = ((report[1] & 0xE0) == 0);

        bool use_report_id_layout = (report_id_hint && button_hint && id_mag > std_mag);

        if (use_report_id_layout) {
            event.buttons = report[1];
            event.dx = id_dx;
            event.dy = id_dy;
            event.wheel = (int8_t)report[4];
        } else {
            event.buttons = report[0];
            event.dx = std_dx;
            event.dy = std_dy;
            event.wheel = (int8_t)report[3];
        }
        event.profile = CUBE32_MOUSE_PROFILE_BOOT_GENERIC;
    }
    
    // Filter out small movements during button state changes (prevents click jump)
    bool button_changed = (event.buttons != s_prev_mouse_buttons);
    if (button_changed) {
        // Zero out small movements when button state changes
        if (abs(event.dx) <= 2) event.dx = 0;
        if (abs(event.dy) <= 2) event.dy = 0;
    }
    s_prev_mouse_buttons = event.buttons;

    if (event.dx != 0 || event.dy != 0 || event.wheel != 0 || event.buttons != 0) {
        s_mouse_debug_event_count++;
        if (s_mouse_debug_event_count <= 50 || (s_mouse_debug_event_count % 50) == 0) {
        ESP_LOGI(TAG,
                 "Mouse EVT[%lu]: btn=0x%02X dx=%d dy=%d wheel=%d",
                 (unsigned long)s_mouse_debug_event_count,
                 event.buttons,
                 (int)event.dx,
                 (int)event.dy,
                 (int)event.wheel);
        }
    }
    
    // Only send if there's actual movement or button state
    if (event.dx != 0 || event.dy != 0 || event.buttons != 0 || event.wheel != 0) {
        xQueueSend(s_mouse_queue, &event, 0);
    }
}

// ============================================================================
// USB Host Callbacks
// ============================================================================

static void kbd_transfer_callback(usb_transfer_t *transfer) {
    if (transfer->status == USB_TRANSFER_STATUS_COMPLETED) {
        process_keyboard_report(transfer->data_buffer, transfer->actual_num_bytes);
    }
    
    if (s_keyboard_connected && s_kbd_transfer) {
        usb_host_transfer_submit(s_kbd_transfer);
    }
}

static void mouse_transfer_callback(usb_transfer_t *transfer) {
    if (transfer->status == USB_TRANSFER_STATUS_COMPLETED) {
        process_mouse_report(transfer->data_buffer, transfer->actual_num_bytes);
    }
    
    if (s_mouse_connected && s_mouse_transfer) {
        usb_host_transfer_submit(s_mouse_transfer);
    }
}

static void host_event_callback(const usb_host_client_event_msg_t *event_msg, void *arg) {
    switch (event_msg->event) {
        case USB_HOST_CLIENT_EVENT_NEW_DEV: {
            ESP_LOGI(TAG, "New device address=%d", event_msg->new_dev.address);
            
            usb_device_handle_t dev_hdl = nullptr;
            esp_err_t err = usb_host_device_open(s_client_hdl, event_msg->new_dev.address, &dev_hdl);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to open device: %s", esp_err_to_name(err));
                break;
            }
            
            usb_device_info_t dev_info;
            usb_host_device_info(dev_hdl, &dev_info);
            ESP_LOGI(TAG, "Speed: %s", dev_info.speed == USB_SPEED_LOW ? "Low" : 
                                        dev_info.speed == USB_SPEED_FULL ? "Full" : "High");
            
            const usb_config_desc_t *config_desc;
            usb_host_get_active_config_descriptor(dev_hdl, &config_desc);
            
            // Get device descriptor to check device class
            const usb_device_desc_t *dev_desc;
            usb_host_get_device_descriptor(dev_hdl, &dev_desc);
            
            // Check if this is a hub (class 0x09)
            bool is_hub = (dev_desc->bDeviceClass == USB_CLASS_HUB);
            
            // Also check interface class for hubs that report class at interface level
            if (!is_hub) {
                int offset = 0;
                while (offset < config_desc->wTotalLength) {
                    const usb_standard_desc_t *desc = (const usb_standard_desc_t*)((uint8_t*)config_desc + offset);
                    if (desc->bDescriptorType == USB_B_DESCRIPTOR_TYPE_INTERFACE) {
                        const usb_intf_desc_t *intf = (const usb_intf_desc_t*)desc;
                        if (intf->bInterfaceClass == USB_CLASS_HUB) {
                            is_hub = true;
                            break;
                        }
                    }
                    offset += desc->bLength;
                }
            }
            
            if (is_hub) {
                ESP_LOGW(TAG, "USB Hub detected - NOT SUPPORTED");
                ESP_LOGW(TAG, "ESP-IDF USB Host library does not support USB hubs.");
                ESP_LOGW(TAG, "Please connect keyboard/mouse directly to the USB port.");
                usb_host_device_close(s_client_hdl, dev_hdl);
                break;
            }
            
            // Not a hub - this is a regular HID device
            // Store as the current device handle for HID
            s_device_hdl = dev_hdl;
            
            // Parse descriptors to find HID interfaces
            int offset = 0;
            const usb_intf_desc_t *current_intf = nullptr;
            uint8_t current_protocol = 0;
            
            while (offset < config_desc->wTotalLength) {
                const usb_standard_desc_t *desc = (const usb_standard_desc_t*)((uint8_t*)config_desc + offset);
                
                if (desc->bDescriptorType == USB_B_DESCRIPTOR_TYPE_INTERFACE) {
                    current_intf = (const usb_intf_desc_t*)desc;
                    if (current_intf->bInterfaceClass == 0x03 && 
                        current_intf->bInterfaceSubClass == 0x01) {
                        // Boot interface subclass
                        current_protocol = current_intf->bInterfaceProtocol;
                        ESP_LOGI(TAG, "Found HID boot interface: protocol=%d (%s)", 
                                 current_protocol,
                                 current_protocol == HID_PROTOCOL_KEYBOARD ? "Keyboard" :
                                 current_protocol == HID_PROTOCOL_MOUSE ? "Mouse" : "Unknown");
                    } else {
                        current_protocol = 0;
                    }
                } else if (desc->bDescriptorType == USB_B_DESCRIPTOR_TYPE_ENDPOINT && current_intf) {
                    const usb_ep_desc_t *ep = (const usb_ep_desc_t*)desc;
                    // Only interested in IN interrupt endpoints
                    if ((ep->bEndpointAddress & 0x80) && 
                        (ep->bmAttributes & 0x03) == 0x03) {
                        if (current_protocol == HID_PROTOCOL_KEYBOARD && s_kbd_ep_addr == 0) {
                            s_kbd_ep_addr = ep->bEndpointAddress;
                            s_kbd_intf_num = current_intf->bInterfaceNumber;
                            s_kbd_mps = get_ep_max_packet_size(ep);
                            ESP_LOGI(TAG, "Keyboard endpoint: 0x%02X, interface: %d, MPS: %u", 
                                     s_kbd_ep_addr, s_kbd_intf_num, s_kbd_mps);
                        } else if (current_protocol == HID_PROTOCOL_MOUSE && s_mouse_ep_addr == 0) {
                            s_mouse_ep_addr = ep->bEndpointAddress;
                            s_mouse_intf_num = current_intf->bInterfaceNumber;
                            s_mouse_mps = get_ep_max_packet_size(ep);
                            ESP_LOGI(TAG, "Mouse endpoint: 0x%02X, interface: %d, MPS: %u", 
                                     s_mouse_ep_addr, s_mouse_intf_num, s_mouse_mps);
                        }
                    }
                }
                offset += desc->bLength;
            }
            
            // Setup keyboard if found
            if (s_kbd_ep_addr != 0) {
                err = usb_host_interface_claim(s_client_hdl, s_device_hdl, s_kbd_intf_num, 0);
                if (err == ESP_OK) {
                    usb_host_transfer_alloc(s_kbd_mps, 0, &s_kbd_transfer);
                    s_kbd_transfer->device_handle = s_device_hdl;
                    s_kbd_transfer->bEndpointAddress = s_kbd_ep_addr;
                    s_kbd_transfer->callback = kbd_transfer_callback;
                    s_kbd_transfer->context = nullptr;
                    s_kbd_transfer->num_bytes = s_kbd_mps;
                    
                    if (usb_host_transfer_submit(s_kbd_transfer) == ESP_OK) {
                        s_keyboard_connected = true;
                        ESP_LOGI(TAG, "Keyboard connected");
                        notify_status_change();
                    }
                } else {
                    ESP_LOGE(TAG, "Keyboard interface claim failed: %s", esp_err_to_name(err));
                }
            }
            
            // Setup mouse if found
            if (s_mouse_ep_addr != 0) {
                err = usb_host_interface_claim(s_client_hdl, s_device_hdl, s_mouse_intf_num, 0);
                if (err == ESP_OK) {
                    usb_host_transfer_alloc(s_mouse_mps, 0, &s_mouse_transfer);
                    s_mouse_transfer->device_handle = s_device_hdl;
                    s_mouse_transfer->bEndpointAddress = s_mouse_ep_addr;
                    s_mouse_transfer->callback = mouse_transfer_callback;
                    s_mouse_transfer->context = nullptr;
                    s_mouse_transfer->num_bytes = s_mouse_mps;
                    
                    esp_err_t mouse_err = usb_host_transfer_submit(s_mouse_transfer);
                    if (mouse_err == ESP_OK) {
                        s_mouse_connected = true;
                        ESP_LOGI(TAG, "Mouse connected");
                        notify_status_change();
                    } else {
                        ESP_LOGE(TAG, "Mouse transfer submit failed: %s", esp_err_to_name(mouse_err));
                    }
                } else {
                    ESP_LOGE(TAG, "Mouse interface claim failed: %s", esp_err_to_name(err));
                }
            }
            
            if (!s_keyboard_connected && !s_mouse_connected) {
                ESP_LOGW(TAG, "No HID keyboard or mouse found");
                usb_host_device_close(s_client_hdl, s_device_hdl);
                s_device_hdl = nullptr;
            }
            break;
        }
        
        case USB_HOST_CLIENT_EVENT_DEV_GONE: {
            ESP_LOGI(TAG, "Device disconnected");
            
            bool had_keyboard = s_keyboard_connected;
            bool had_mouse = s_mouse_connected;
            
            s_keyboard_connected = false;
            s_mouse_connected = false;
            
            vTaskDelay(pdMS_TO_TICKS(50));
            
            if (s_kbd_transfer) {
                usb_host_transfer_free(s_kbd_transfer);
                s_kbd_transfer = nullptr;
            }
            if (s_mouse_transfer) {
                usb_host_transfer_free(s_mouse_transfer);
                s_mouse_transfer = nullptr;
            }
            
            if (s_device_hdl) {
                if (s_kbd_intf_num != 0 || s_kbd_ep_addr != 0) {
                    usb_host_interface_release(s_client_hdl, s_device_hdl, s_kbd_intf_num);
                }
                if (s_mouse_intf_num != 0 || s_mouse_ep_addr != 0) {
                    usb_host_interface_release(s_client_hdl, s_device_hdl, s_mouse_intf_num);
                }
                usb_host_device_close(s_client_hdl, s_device_hdl);
                s_device_hdl = nullptr;
            }
            
            s_kbd_ep_addr = 0;
            s_mouse_ep_addr = 0;
            s_kbd_intf_num = 0;
            s_mouse_intf_num = 0;
            s_kbd_mps = 8;
            s_mouse_mps = 8;
            s_mouse_debug_report_count = 0;
            s_mouse_debug_event_count = 0;
            memset(s_prev_keys, 0, sizeof(s_prev_keys));
            
            if (had_keyboard || had_mouse) {
                notify_status_change();
            }
            
            ESP_LOGI(TAG, "Cleanup complete, ready for new device");
            break;
        }
        default:
            break;
    }
}

// ============================================================================
// USB Host Task
// ============================================================================

static void usb_host_task(void *arg) {
    ESP_LOGI(TAG, "Host task started");
    
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    
    esp_err_t err = usb_host_install(&host_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Host install failed: %s", esp_err_to_name(err));
        vTaskDelete(nullptr);
        return;
    }
    ESP_LOGI(TAG, "Host library installed");
    
    const usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .max_num_event_msg = 5,
        .async = {
            .client_event_callback = host_event_callback,
            .callback_arg = nullptr
        }
    };
    
    err = usb_host_client_register(&client_config, &s_client_hdl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register client: %s", esp_err_to_name(err));
        usb_host_uninstall();
        vTaskDelete(nullptr);
        return;
    }
    ESP_LOGI(TAG, "Host client registered");
    s_usb_host_ready = true;
    
    while (true) {
        uint32_t event_flags;
        esp_err_t err = usb_host_lib_handle_events(pdMS_TO_TICKS(10), &event_flags);
        
        if (err == ESP_OK) {
            if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
                ESP_LOGD(TAG, "No clients event");
            }
            if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
                ESP_LOGD(TAG, "All devices freed event");
            }
        }
        
        usb_host_client_handle_events(s_client_hdl, pdMS_TO_TICKS(10));
    }
}

// ============================================================================
// Namespace Implementation
// ============================================================================

namespace cube32 {

USBInput& USBInput::instance() {
    static USBInput s_instance;
    return s_instance;
}

USBInput::~USBInput() {
    if (m_initialized) {
        end();
    }
}

cube32_result_t USBInput::begin() {
    if (m_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return CUBE32_ALREADY_INITIALIZED;
    }
    
    ESP_LOGI(TAG, "Initializing USB Input driver...");
    
    // Create event queues
    s_key_queue = xQueueCreate(CUBE32_USB_INPUT_KEY_QUEUE_SIZE, sizeof(cube32_usb_key_event_t));
    s_mouse_queue = xQueueCreate(CUBE32_USB_INPUT_MOUSE_QUEUE_SIZE, sizeof(cube32_usb_mouse_event_t));
    
    if (!s_key_queue || !s_mouse_queue) {
        ESP_LOGE(TAG, "Failed to create event queues");
        if (s_key_queue) vQueueDelete(s_key_queue);
        if (s_mouse_queue) vQueueDelete(s_mouse_queue);
        s_key_queue = nullptr;
        s_mouse_queue = nullptr;
        return CUBE32_NO_MEM;
    }
    
    // Start USB host task
    BaseType_t ret = xTaskCreatePinnedToCore(
        usb_host_task,
        "usb_host",
        CUBE32_USB_HOST_TASK_STACK_SIZE,
        nullptr,
        CUBE32_USB_HOST_TASK_PRIORITY,
        nullptr,
        CUBE32_USB_HOST_TASK_CORE
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create USB host task");
        vQueueDelete(s_key_queue);
        vQueueDelete(s_mouse_queue);
        s_key_queue = nullptr;
        s_mouse_queue = nullptr;
        return CUBE32_ERROR;
    }
    
    // Wait for USB host to be ready
    uint32_t timeout = 50;
    while (!s_usb_host_ready && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(100));
        timeout--;
    }
    
    if (!s_usb_host_ready) {
        ESP_LOGE(TAG, "Timeout waiting for USB host");
        return CUBE32_TIMEOUT;
    }
    
    m_initialized = true;
    ESP_LOGI(TAG, "USB Input driver initialized");
    return CUBE32_OK;
}

cube32_result_t USBInput::end() {
    if (!m_initialized) {
        return CUBE32_NOT_INITIALIZED;
    }
    
    ESP_LOGI(TAG, "Deinitializing USB Input driver...");
    
    // Note: We don't have a clean way to stop the USB host task
    // In a production system, you'd want to implement task deletion
    // For now, just clear the initialized flag
    
    m_initialized = false;
    ESP_LOGI(TAG, "USB Input driver deinitialized");
    return CUBE32_OK;
}

bool USBInput::isReady() const {
    return s_usb_host_ready;
}

bool USBInput::isKeyboardConnected() const {
    return s_keyboard_connected;
}

bool USBInput::isMouseConnected() const {
    return s_mouse_connected;
}

QueueHandle_t USBInput::getKeyQueue() const {
    return s_key_queue;
}

QueueHandle_t USBInput::getMouseQueue() const {
    return s_mouse_queue;
}

bool USBInput::pollKeyEvent(cube32_usb_key_event_t* event) {
    if (!s_key_queue || !event) return false;
    return xQueueReceive(s_key_queue, event, 0) == pdTRUE;
}

bool USBInput::pollMouseEvent(cube32_usb_mouse_event_t* event) {
    if (!s_mouse_queue || !event) return false;
    return xQueueReceive(s_mouse_queue, event, 0) == pdTRUE;
}

bool USBInput::waitKeyEvent(cube32_usb_key_event_t* event, uint32_t timeout_ms) {
    if (!s_key_queue || !event) return false;
    TickType_t ticks = (timeout_ms == UINT32_MAX) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xQueueReceive(s_key_queue, event, ticks) == pdTRUE;
}

bool USBInput::waitMouseEvent(cube32_usb_mouse_event_t* event, uint32_t timeout_ms) {
    if (!s_mouse_queue || !event) return false;
    TickType_t ticks = (timeout_ms == UINT32_MAX) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xQueueReceive(s_mouse_queue, event, ticks) == pdTRUE;
}

void USBInput::setStatusCallback(cube32_usb_status_callback_t callback) {
    s_status_callback = callback;
}

} // namespace cube32

#endif // CONFIG_CUBE32_USB_INPUT_ENABLED
