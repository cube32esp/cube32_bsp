# hello_world

Pre-loaded product firmware for the **CUBE32-S3 Core** development board.  
Demonstrates real-time IMU interaction, touch input, 3D graphics, and hardware status reporting — all running on LVGL v9 with ESP-IDF v5.5.

---

## Overview

The demo runs two screens that you navigate between by double-tapping the display.

### Screen 1 — Cube (default)

An interactive 3D wireframe cube with physics-based inertia.

- **IMU gyro** rotates the cube in real time as you tilt the board  
- **Touch swipe** applies an angular impulse — the cube coasts naturally to rest  
- **Depth-based edge colours** update every frame: the closest face renders bright cyan, the farthest face near-invisible dark indigo, side connectors electric blue  
- **Double-tap** → switches to the Status screen

### Screen 2 — System Status

A hardware driver manifest grid showing the initialisation result for all 16 on-board drivers.

| Indicator | Meaning |
|-----------|---------|
| ✓ (green dot) | Driver initialised OK |
| ✗ (red dot) | Initialisation failed |
| Amber dot | Skipped or still initialising |
| Dark dot | Not present / not built |

Below the grid, a live battery row shows charge level with a dynamic icon (●●●● → empty), voltage in mV, and a charging symbol when USB power is connected. Battery info refreshes every 5 seconds.  
**Double-tap** → returns to the Cube screen.

---

## Hardware Requirements

| Component | Details |
|-----------|---------|
| Board | CUBE32-S3 Core |
| Display | 240 × 240 RGB565 TFT |
| IMU | LSM6DSOX (I²C 0x6A) |
| Touch | CST816S capacitive |
| PMU | AXP2101 (built-in on CUBE32-S3 Core) |

---

## Controls

| Gesture | Action |
|---------|--------|
| Tilt board left / right | Cube rotates left / right |
| Tilt board up / down | Cube tilts up / down |
| Swipe left / right on screen (≥ 45 px) | Kick cube left / right |
| Swipe up / down on screen (≥ 45 px) | Kick cube up / down |
| Double-tap (within 600 ms) | Switch screen |

---

## Physics Parameters

| Constant | Value | Description |
|----------|-------|-------------|
| `INERTIA` | 0.95 | Velocity retained per frame — cube coasts ~1.5 s to rest |
| `GYRO_SCALE` | 1.0 | Gyro dps → deg/s scale factor |
| `GYRO_NOISE` | 3.0 dps | Dead-zone — suppresses sensor bias when board is still |
| `KICK_SCALE` | 2.5 | Swipe pixels → deg/s angular impulse |
| `SWIPE_THRESH` | 45 px | Minimum drag distance to register as a swipe |
| `DOUBLE_TAP_MS` | 600 ms | Window for the second tap to count as a double-tap |

---

## Color Palette

| Role | Hex | Description |
|------|-----|-------------|
| Background | `0x080C14` | Deep near-black |
| Title / Primary accent | `0x00D4FF` | Cyan |
| Front face edges | `0x00FFFF` | Full-brightness cyan |
| Side connector edges | `0x0066FF` | Deep electric blue |
| Back face edges | `0x001A3A` | Near-invisible dark indigo |
| Status OK | `0x00FF88` | Green |
| Status FAIL | `0xFF4444` | Red |
| Status SKIP / INIT | `0xFFAA00` | Amber |

---

## Build & Flash

```bash
# From the project root
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

### Required LVGL Font Settings

Enable the following fonts in `menuconfig` under **Component config → LVGL → Font usage**:

| Font | Used for |
|------|---------|
| `Montserrat 14` | Driver grid labels, battery row, hint text |
| `Montserrat 20` | Status screen title |
| `Montserrat 24` | Cube screen "CUBE32" title |

```
Component config → LVGL → Font usage
  [*] Enable Montserrat 14
  [*] Enable Montserrat 20
  [*] Enable Montserrat 24
```

---

## Key Files

| File | Description |
|------|-------------|
| `main.cpp` | Full source — UI, 3D renderer, physics, IMU, touch |
| `../../include/cube32.h` | CUBE32 BSP public API |
| `../../include/utils/hw_manifest.h` | Hardware manifest API (`cube32_hw_manifest_t`) |
| `../../include/drivers/pmu/axp2101.h` | PMU API (`PMUStatus`, `isBatteryPresent()` etc.) |
