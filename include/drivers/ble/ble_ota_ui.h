/**
 * @file ble_ota_ui.h
 * @brief CUBE32 Built-In BLE OTA UI and OTA Pop-up UI public API
 * 
 * This header provides the public API for:
 * 1. Built-in BLE OTA UI — Full-screen UI for bypass/double-boot mode.
 *    Shows device ID, BLE state, connection status, and restart button.
 *    Automatically created by cube32_init() in bypass mode.
 * 
 * 2. OTA Pop-up UI — Modal overlay shown on top of any active screen
 *    when a BLE client starts an OTA firmware update. Works in both
 *    bypass mode and background mode without interfering with the
 *    main application.
 * 
 * 3. OTA Monitor — Lightweight LVGL timer that monitors BLE OTA state
 *    and auto-creates the OTA pop-up when firmware update begins.
 *    Started by cube32_init() whenever BLE OTA is active.
 */

#pragma once

namespace cube32 {

// ============================================================================
// Built-in BLE OTA UI (for bypass/double-boot mode)
// ============================================================================

/**
 * @brief Create and display the built-in BLE OTA UI
 * 
 * Creates an LVGL-based UI showing:
 * - Device ID in large font for easy identification during BLE pairing
 * - BLE state indicator and connection status
 * - Restart button
 * 
 * OTA update progress is handled by the separate OTA Pop-up UI,
 * which appears automatically on top when OTA starts.
 * 
 * This function is a no-op when CONFIG_CUBE32_BLE_OTA_ENABLED is not defined.
 * Requires LVGL to be initialized before calling.
 */
void ble_ota_ui_create(void);

/**
 * @brief Destroy the built-in BLE OTA UI and free resources
 * 
 * Stops the UI update timer and cleans up. Call this before
 * transitioning to a different screen or cleaning up LVGL.
 */
void ble_ota_ui_destroy(void);

/**
 * @brief Check if the built-in BLE OTA UI is active
 * 
 * @return true if the built-in UI was created and is active
 * @return false if the built-in UI is not enabled or not yet created
 */
bool ble_ota_ui_is_initialized(void);

// ============================================================================
// OTA Pop-up UI (modal overlay for OTA progress)
// ============================================================================

/**
 * @brief Create and show the OTA pop-up overlay
 * 
 * Shows a modal overlay on top of the current screen with:
 * - Firmware name and version
 * - Progress bar with percentage
 * - Bytes transferred info
 * - Restart button (shown when OTA completes)
 * 
 * This pop-up works in both bypass mode and background mode.
 * In background mode, it overlays the running application UI
 * without disrupting the app logic.
 * 
 * Usually called automatically by the OTA monitor, but can be
 * called manually if needed.
 */
void ble_ota_popup_show(void);

/**
 * @brief Destroy the OTA pop-up overlay
 * 
 * Removes the pop-up and frees resources. The underlying
 * application screen remains intact.
 */
void ble_ota_popup_destroy(void);

/**
 * @brief Check if the OTA pop-up is currently showing
 * 
 * @return true if the OTA pop-up is active
 */
bool ble_ota_popup_is_active(void);

// ============================================================================
// OTA Monitor (auto-manages pop-up based on BLE OTA state)
// ============================================================================

/**
 * @brief Start the OTA state monitor
 * 
 * Creates a lightweight LVGL timer that monitors BLE OTA state.
 * When OTA begins (state → OTA_IN_PROGRESS), the pop-up is
 * automatically created. When OTA completes, the pop-up shows
 * the restart button.
 * 
 * Called by cube32_init() whenever BLE OTA is active (both bypass
 * and background modes).
 */
void ble_ota_monitor_start(void);

/**
 * @brief Stop the OTA state monitor
 * 
 * Stops the monitor timer and destroys any active pop-up.
 */
void ble_ota_monitor_stop(void);

} // namespace cube32
