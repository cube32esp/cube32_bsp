# Hello Camera Example

This example demonstrates camera initialization and displaying camera images on the TFT display using the CUBE32 BSP.

## Features

- Camera initialization using CUBE32 camera driver with shared I2C bus
- Image capture to frame buffer
- Displaying camera images on ST7789 TFT display
- RGB565 pixel format handling with byte swapping for display
- Optional LVGL mode for camera preview with overlay information
- Frame rate calculation and display

## Required Kconfig Settings

### Camera Configuration

Navigate to `CUBE32 Camera Configuration` in menuconfig:

| Setting | Required Value | Notes |
|---------|----------------|-------|
| **Enable Camera Driver** | `y` (enabled) | Must be enabled |
| **Camera Frame Size** | QVGA (320x240) recommended | Should match or be smaller than display resolution for best performance |
| **Camera Pixel Format** | **RGB565** | Required for direct display. JPEG will not work with this example. |
| H Mirror | Optional | Enable if image appears horizontally flipped |
| V Flip | Optional | Enable if image appears vertically flipped |

> ⚠️ **Important**: The camera pixel format **must** be set to **RGB565** for this example. RGB565 format outputs raw pixel data that can be directly displayed on the TFT screen. JPEG format requires decoding and is not suitable for this direct-display example.

### Display Configuration

Navigate to `CUBE32 Display Configuration` in menuconfig:

| Setting | Required Value | Notes |
|---------|----------------|-------|
| **Enable Display** | `y` (enabled) | Must be enabled |
| **Display Board Model** | Select your board | Determines display resolution (e.g., CUBE32_DISPLAY_CUBE_TFT_TOUCH_154 = 240x240) |

### Optional: LVGL Configuration

For LVGL mode with overlay information:

| Setting | Value | Notes |
|---------|-------|-------|
| Enable LVGL | `y` | Optional, for LVGL canvas mode |

## How to Configure

1. Run menuconfig:
   ```bash
   idf.py menuconfig
   ```

2. Configure Camera:
   - Go to `Component config` → `CUBE32 Camera Configuration`
   - Enable `Enable Camera Driver`
   - Set `Camera Frame Size` to `QVGA (320x240)` (recommended for 240x320 display)
   - Set `Camera Pixel Format` to **`RGB565`**

3. Configure Display:
   - Go to `Component config` → `CUBE32 Display Configuration`
   - Enable `Enable Display`

4. Select this example:
   - Go to `CUBE32 Application Selection`
   - Select `Hello Camera Example`

5. Build and flash:
   ```bash
   idf.py build flash monitor
   ```

## Usage

After flashing, the device will:

1. Initialize the CUBE32 board (I2C, PMU, Display, Camera)
2. Display camera preview on the TFT screen
3. Show frame rate information (if LVGL is enabled)

### Display Modes

The example supports two display modes:

#### 1. Direct Display Mode (Default)
- Directly writes camera frame buffer to the LCD
- Fastest performance
- No overlay information

#### 2. LVGL Canvas Mode (if LVGL enabled)
- Uses LVGL canvas widget to display camera image
- Shows overlay information (FPS, resolution)
- Slightly lower performance due to LVGL overhead

## Frame Size Recommendations

| Display Size | Recommended Camera Frame Size | Notes |
|--------------|-------------------------------|-------|
| 240x320 | QVGA (320x240) | Exact fit (rotated) |
| 240x320 | QQVGA (160x120) | Faster, lower quality |
| 320x480 | VGA (640x480) | May need scaling |

## Troubleshooting

### Black screen / No camera image
- **Cause**: Wrong pixel format selected
- **Solution**: Ensure `Camera Pixel Format` is set to `RGB565` in Kconfig

### Image colors look wrong
- **Cause**: Byte order mismatch between camera and display
- **Solution**: The example handles RGB565 byte swapping automatically. If colors still look wrong, check display initialization settings.

### Low frame rate
- **Cause**: Frame size too large or display bandwidth limited
- **Solution**: 
  1. Use smaller frame size (QQVGA or QVGA)
  2. Disable LVGL overlay mode

### Camera capture failed
- **Cause**: Camera not properly initialized or I2C communication issue
- **Solution**: 
  1. Check camera hardware connections
  2. Ensure I2C bus is properly initialized
  3. Check power supply to camera module

### Image appears mirrored or flipped
- **Cause**: Camera orientation doesn't match expected orientation
- **Solution**: Enable/disable `H Mirror` and `V Flip` in Kconfig

## Memory Requirements

- PSRAM: Required for camera frame buffers (RGB565 uses more memory than JPEG)
  - QVGA (320x240): ~150KB per frame
  - VGA (640x480): ~600KB per frame
- Heap: ~20KB for display line buffer and tasks

## Example Console Output

```
I (1234) hello_camera: ========================================
I (1234) hello_camera: CUBE32 Hello Camera Example
I (1234) hello_camera: ========================================
I (1345) cube32_camera: Camera initialized successfully
I (1456) hello_camera: Camera initialized: 320x240, RGB565
I (1567) hello_camera: Display initialized: 240x320
I (1678) hello_camera: Starting camera preview...
I (2345) hello_camera: Frame 0: 320x240, 153600 bytes, 15.2 FPS
I (2456) hello_camera: Frame 1: 320x240, 153600 bytes, 14.8 FPS
...
```

## Pixel Format Comparison

| Format | Use Case | Memory | Display Compatible |
|--------|----------|--------|-------------------|
| **RGB565** | Direct display | High (~2 bytes/pixel) | ✅ Yes |
| JPEG | Network streaming | Low (compressed) | ❌ No (requires decoding) |
| Grayscale | Image processing | Medium (1 byte/pixel) | ⚠️ Needs conversion |

This example requires **RGB565** format because:
1. Direct memory copy to display buffer
2. No CPU-intensive decoding required
3. Real-time preview performance
