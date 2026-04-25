# CUBE32 BSP

[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-%3E%3D5.4.0-red?logo=espressif)](https://github.com/espressif/esp-idf)
[![Target](https://img.shields.io/badge/target-ESP32--S3-blue?logo=espressif)](https://www.espressif.com/en/products/socs/esp32-s3)
[![LVGL](https://img.shields.io/badge/LVGL-9.x-darkgreen)](https://lvgl.io)
[![Component Registry](https://components.espressif.com/components/cube32esp/cube32_bsp/badge.svg)](https://components.espressif.com/components/cube32esp/cube32_bsp)
[![License: MIT](https://img.shields.io/badge/license-MIT-yellow.svg)](LICENSE)

**CUBE32 BSP** is a turnkey ESP32-S3 board support package designed for AI-assisted and vibe-coded IoT development.

Drop it into any ESP-IDF project with one command. Every on-board peripheral — display, touch, IMU, audio, camera, LTE modem, PMU, RTC — initialises with a single `cube32_init()` call so you can skip the boilerplate and focus entirely on your application logic.

---

## Hardware — CUBE32-S3 Core

| Peripheral | Details |
|---|---|
| **MCU** | ESP32-S3 — dual-core Xtensa LX7 @ 240 MHz, 8 MB PSRAM (Octal), 16 MB Flash |
| **Display** | 1.54″ 240×240 RGB565 TFT (ST7789) |
| **Touch** | Capacitive touch panel — CST816S (auto-detect FT6336) |
| **IMU** | LSM6DSOX 6-axis IMU with on-chip Machine Learning Core (MLC) |
| **Magnetometer** | LIS3MDL 3-axis magnetometer |
| **PMU** | AXP2101 — battery charging, rail management, fuel gauge |
| **RTC** | BM8563 — battery-backed real-time clock |
| **Audio** | ES8311 DAC + ES7210 ADC — 24 kHz stereo speaker & 4-channel microphone |
| **Camera** | OV2640 (up to UXGA) |
| **SD Card** | SDMMC — FATFS mount at `/sdcard` |
| **LTE Modem** | SIMCom A7670 — USB / UART AT commands, PPP data |
| **USB HID** | USB keyboard + mouse input (host) |
| **ADC Buttons** | 6-button array on single ADC channel |
| **IO Expander** | TCA9554 (×2) — GPIO expansion, modem & audio power control |
| **LVGL** | LVGL 9.x with esp_lvgl_port — thread-safe UI with DMA rendering |

---

## Quick Start

### 1. Add the component

```bash
idf.py add-dependency "cube32esp/cube32_bsp"
```

Or add manually to your project's `main/idf_component.yml`:

```yaml
dependencies:
  cube32esp/cube32_bsp: "*"
```

### 2. Set target

```bash
idf.py set-target esp32s3
```

### 3. Write your application

```cpp
#include "cube32.h"

extern "C" void app_main(void)
{
    // Initialise all CUBE32 hardware drivers in the correct order
    esp_err_t ret = cube32_init();
    if (ret != ESP_OK) {
        ESP_LOGE("app", "Board init failed: %s", esp_err_to_name(ret));
        return;
    }

    // Display is up — draw something with LVGL
    cube32::LvglDisplay &lvgl = cube32::LvglDisplay::instance();
    lvgl.lock();
    lv_obj_t *label = lv_label_create(lv_screen_active());
    lv_label_set_text(label, "Hello CUBE32!");
    lv_obj_center(label);
    lvgl.unlock();

    // Read IMU
    cube32::IMU &imu = cube32::IMU::instance();
    while (true) {
        float ax, ay, az, gx, gy, gz;
        imu.getAccel(ax, ay, az);
        imu.getGyro(gx, gy, gz);
        ESP_LOGI("app", "Accel: %.2f, %.2f, %.2f  Gyro: %.2f, %.2f, %.2f",
                 ax, ay, az, gx, gy, gz);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
```

### 4. Build and flash

```bash
idf.py build flash monitor
```

---

## Driver Overview

Every driver follows the singleton pattern and returns `cube32_result_t` from `begin()`. Drivers are enabled/disabled via Kconfig so unused peripherals never enter your binary.

| Driver | Kconfig Key | Description |
|---|---|---|
| **PMU** | `CUBE32_PMU_ENABLED` | AXP2101 power management — battery %, voltage, charge state, power rails |
| **Display** | `CUBE32_DISPLAY_ENABLED` | ST7789 240×240 SPI display with DMA |
| **Touch** | `CUBE32_TOUCH_ENABLED` | Auto-detects CST816S or FT6336 capacitive touch |
| **LVGL** | `CUBE32_LVGL_ENABLED` | LVGL 9.x with esp_lvgl_port — `lock()`/`unlock()` thread safety |
| **RTC** | `CUBE32_RTC_ENABLED` | BM8563 battery-backed RTC — get/set time, alarms, timers |
| **IMU** | `CUBE32_IMU_ENABLED` | LSM6DSOX 6-axis (accel + gyro) with Machine Learning Core (MLC) |
| **Magnetometer** | `CUBE32_MAG_ENABLED` | LIS3MDL 3-axis — raw field data, calibration |
| **Audio** | `CUBE32_AUDIO_ENABLED` | ES8311+ES7210 — speaker playback, 4-ch microphone, 24 kHz |
| **Camera** | `CUBE32_CAMERA_ENABLED` | OV2640 — configurable frame size and pixel format |
| **SD Card** | `CUBE32_SDCARD_ENABLED` | SDMMC FATFS — `/sdcard` mount, 1-bit or 4-bit mode |
| **LTE Modem** | `CUBE32_MODEM_ENABLED` | A7670 over USB — AT commands, PPP internet, async init |
| **USB HID** | `CUBE32_USB_INPUT_ENABLED` | USB keyboard & mouse host — LVGL pointer integration |
| **ADC Buttons** | `CUBE32_ADC_BUTTON_ENABLED` | 6-button resistor ladder on one ADC pin |
| **IO Expander** | `CUBE32_IO_EXPANDER_ENABLED` | TCA9554 GPIO expander — modem power, audio PA enable |
| **BLE OTA** | `CUBE32_BLE_OTA_ENABLED` | NimBLE-based OTA firmware update over BLE |
| **Servo** | `CUBE32_SERVO_ENABLED` | PWM servo control via LEDC |

---

## Kconfig Highlights

Run `idf.py menuconfig` → **CUBE32 Board Configuration** to fine-tune:

| Option | Default | Description |
|---|---|---|
| Board model | `CUBE32-S3 Core` | Hardware variant selection |
| Display model | `CUBE_TFT_TOUCH_154 (240×240)` | 1.54″ or 2.0″ display |
| LVGL double buffer | `y` | Enable for tear-free rendering |
| IMU Machine Learning Core | `y` | On-device activity recognition |
| Audio sample rate | `24000` | Input/output sample rate (Hz) |
| Modem connection type | `USB` | USB or UART |
| SD card bus width | `1-bit` | 1-bit or 4-bit SDMMC |

---

## Partition Layout

The standard CUBE32 partition layout targets 16 MB flash with dual OTA banks and a large assets partition for fonts, audio clips, and ML models:

| Partition | Type | Size | Purpose |
|---|---|---|---|
| `nvs` | data/nvs | 16 KB | Key-value storage (WiFi credentials, config) |
| `otadata` | data/ota | 8 KB | OTA slot tracking |
| `phy_init` | data/phy | 4 KB | RF calibration data |
| `ota_0` | app/ota_0 | ~4 MB | Application image (slot A) |
| `ota_1` | app/ota_1 | ~4 MB | Application image (slot B) |
| `assets` | data/spiffs | 8 MB | Fonts, images, audio, ML models |

See `examples/hello_world/partitions/partitions.csv` for the full CSV definition.

---

## Examples

| Example | Description |
|---|---|
| [`hello_world`](examples/hello_world/) | Interactive 3D cube (IMU gyro + touch) and driver status screen |

More examples are coming in future releases (display, camera, audio, BLE OTA, and more).

---

## Speech Recognition (Optional)

`esp-sr` (Espressif's speech recognition + AEC library) is **not included** in the BSP by default — it adds ~8 MB to the binary. If your application requires wake-word detection, AEC, or voice commands, add it to your project's `idf_component.yml`:

```yaml
dependencies:
  cube32esp/cube32_bsp: "*"
  espressif/esp-sr: "~2.1.5"
```

---

## Build Requirements

| Requirement | Version |
|---|---|
| ESP-IDF | ≥ 5.4.0 |
| Target | `esp32s3` |
| Python | ≥ 3.8 (for IDF tools) |

```bash
# Install ESP-IDF if not already set up
# https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/get-started/

idf.py set-target esp32s3
idf.py menuconfig     # optional — review CUBE32 Board Configuration
idf.py build
```

---

## Why CUBE32 for AI / Vibe Coding?

CUBE32 eliminates the hardware integration tax that typically consumes 80 % of embedded AI project time:

- **Single init call** — `cube32_init()` handles driver ordering, retry logic, and conflict resolution automatically
- **AI-friendly API** — predictable singleton interface across all 16 drivers; easy for LLM-generated code to target
- **Rich sensor suite** — IMU with on-chip MLC activity recognition, 4-channel microphone, camera, and LTE modem — everything an edge AI demo needs in one board
- **Vibe-coding ready** — comprehensive Kconfig, consistent result codes (`cube32_result_t`), and guard macros (`#if CONFIG_CUBE32_*`) make it straightforward for AI assistants to generate correct, conditionally-compiled application code

---

## License

MIT © cube32esp — see [LICENSE](LICENSE).
