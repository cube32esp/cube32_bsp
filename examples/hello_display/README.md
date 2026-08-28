# hello_display

Basic ST7789 TFT display demo for the **CUBE32-S3 Core** board.  
Demonstrates direct SPI drawing operations — no LVGL required.

---

## Overview

The demo cycles through five visual phases automatically, switching every 5 seconds:

| Phase | Name | Description |
|-------|------|-------------|
| 0 | Black Screen | Clears the display; draws a dark-gray info bar in the center |
| 1 | Color Bars | Eight horizontal bars: RED, GREEN, BLUE, YELLOW, CYAN, MAGENTA, WHITE, GRAY |
| 2 | CUBE32 Logo | Five nested squares in blue shades with colored corner accents |
| 3 | Gradient | Vertical red→blue gradient rendered row by row |
| 4 | Bouncing Rect | 40×40 rectangle bouncing around the screen with a cycling RGB color |

Phase transitions and drawing progress are logged to the serial console via `ESP_LOGI`.

---

## Hardware Requirements

| Component | Details |
|-----------|---------|
| Board | CUBE32-S3 Core (ESP32-S3) |
| Display | ST7789 240×240 RGB565 TFT *(CUBE_TFT_TOUCH_154_240x240)* or 240×320 *(CUBE_TFT_TOUCH_200_240x320)* |
| SPI Bus | SPI2\_HOST — MOSI GPIO 0, MISO GPIO 4, SCLK GPIO 1, CS GPIO 46, DC GPIO 2 |

> The display board model (resolution, rotation) is auto-detected at boot from
> the touch controller's I2C address (0x15 or 0x38) — see
> `CUBE32_DISPLAY_MODEL_TABLE` in `cube32_config.h`. Because of this, the
> touch controller must be physically present on the I2C bus even though this
> example doesn't read touch input; if no known touch address is detected,
> display initialization is skipped.

---

## Required KConfig Settings

### 1 — Board model

```
CUBE32 Board Configuration
  Board Model
    (X) CUBE32-S3 Core               → CONFIG_CUBE32_BOARD_S3_CORE  (default)
```

### 2 — Display

```
CUBE32 Board Configuration → Display Configuration
  [*] Enable Display                 → CONFIG_CUBE32_DISPLAY_ENABLED=y  (default)
```

No display model selection is needed — it is auto-detected at boot from the touch
controller's I2C address. The demo is resolution-independent and adapts to
whichever size is detected.

> **LVGL is not required.** `CONFIG_CUBE32_LVGL_ENABLED` can be left at its default (`y`) or disabled — the demo uses the raw `cube32::ST7789Display` driver directly and does not call any LVGL API.

### Summary table

| Symbol | Required value | Default |
|--------|---------------|---------|
| `CONFIG_CUBE32_BOARD_S3_CORE` | `y` | `y` |
| `CONFIG_CUBE32_DISPLAY_ENABLED` | `y` | `y` |

---

## How to Configure

```bash
idf.py menuconfig
```

1. Go to **CUBE32 Board Configuration → Display Configuration** → confirm **Enable Display** is checked.
2. Exit and save.

---

## Build & Flash

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

---

## Expected Serial Output

```
I (xxx) hello_display: ========================================
I (xxx) hello_display: CUBE32 Hello Display Example
I (xxx) hello_display: ========================================
I (xxx) hello_display: Display ready: 240x240
I (xxx) hello_display: Switching to demo phase 1
I (xxx) hello_display: Test pattern drawn
I (xxx) hello_display: Switching to demo phase 2
I (xxx) hello_display: CUBE32 logo drawn
I (xxx) hello_display: Switching to demo phase 3
I (xxx) hello_display: Drawing gradient (this may take a moment)...
I (xxx) hello_display: Gradient drawn
I (xxx) hello_display: Switching to demo phase 4
```

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| `Display not initialized` error | `CONFIG_CUBE32_DISPLAY_ENABLED` is `n` | Enable display in menuconfig |
| Wrong resolution / clipped image | Wrong display model auto-detected | Confirm the touch controller's I2C address matches your display board (0x15 = 1.54", 0x38 = 2.0") |
| Corrupt colors / flickering | SPI bus speed too high for the cable | Check wiring; the driver runs at 40 MHz |
| Blank white screen | `cube32_init()` returned an error | Check earlier log lines for the failing driver |
