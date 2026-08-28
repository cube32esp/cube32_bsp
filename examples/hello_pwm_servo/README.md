# Hello PWM Servo

This example demonstrates the CUBE32 PWM Servo driver using the LEDC hardware PWM peripheral. It provides a console interface with visual feedback on the LVGL display.

When the **Robot Head Action Engine** and **Face Expression Engine** are both enabled, the display switches from the servo status dashboard to an animated robot face. The face plays synchronized expressions alongside head actions (nod, shake, excited, sad, etc.) and shows a living idle state with periodic blinks and micro-saccades when no action is playing.

## Features

- **Console Control**: Type commands via USB serial to control servos in real time
- **LVGL Dashboard**: Dark-themed UI with arc indicators showing angle, attach status, and movement
- **Animated Robot Face**: Two-eye face with pupils, eyelids, and highlight sparkles (Cozmo/Wall-E style)
- **Synchronized Expressions**: Face expressions play in sync with robot head servo actions
- **Idle Animation**: Periodic blinks and micro-saccades make the face appear alive
- **Smooth Movement**: Speed-controlled servo transitions (1–100% of max speed)
- **360° Support**: Continuous rotation servos treated as speed/direction control
- **Dual Channel**: Up to 2 independent servo channels on GPIO 47/48
- **BLE Control**: Commands can be sent from a BLE client via the BLE OTA text channel

## Display Modes

The example selects the display mode automatically based on Kconfig:

| Robot Head | Face Expression | Display |
|:----------:|:---------------:|---------|
| Disabled | Disabled | Servo status dashboard (arc gauges) |
| Enabled | Disabled | Servo status dashboard (arc gauges) |
| Disabled | Enabled | Servo status dashboard (arc gauges) |
| **Enabled** | **Enabled** | **Animated robot face** |

### Robot Face Design

The face uses ~12 LVGL objects (no bitmaps) and runs at 30 fps via an LVGL timer:

- **Two eyes**, each composed of:
  - White eyeball circle (background)
  - Dark pupil circle (position and size animated)
  - Small white highlight dot (sparkle for liveliness)
  - Upper eyelid rectangle (emotion shaping)
  - Lower eyelid rectangle (emotion shaping)
- **Idle behavior**: Random blinks every 3–6 seconds and micro-saccades (tiny pupil movements)

### Face Expressions

Each expression is mapped 1:1 from a head action:

| Head Action | Face Expression |
|-------------|-----------------|
| nod | Happy squint, pupils bounce up-down |
| shake | Stern look, pupils sway left-right |
| curious | Asymmetric eyes, slow pupil drift |
| attention | Alert, pupils snap to center |
| look_left | Both pupils shift left |
| look_right | Both pupils shift right |
| look_up | Pupils shift up, upper lids raised |
| look_down | Pupils shift down, upper lids droop |
| scan | Wide-open alert, pupils sweep across |
| bow | Eyes slowly close then reopen |
| search | Wide alert eyes, pupils circle |
| excited | Happy crescents, pupils jitter |
| sad | Droopy lids, pupils shrink and shift down |
| double_take | Quick snap away then wide surprise |
| dizzy | Asymmetric wobble, decreasing amplitude |

## Prerequisites

Enable the following in `menuconfig`:

1. **Display Configuration** → Enable Display → Enable LVGL
2. **PWM Servo Configuration** → Enable PWM Servo
   - Configure channels, GPIO pins, rotation type (90°/180°/360°)
   - Set min/max pulse widths for your servos
3. *(Optional)* **Robot Head Configuration** → Enable Robot Head Action Engine
4. *(Optional)* **Face Expression Configuration** → Enable Face Expression Engine
5. **Main Application** → Select "Hello PWM Servo"

## Console Commands

### Servo Commands

| Command | Description |
|---------|-------------|
| `angle <ch> <deg>` | Set servo angle immediately |
| `move <ch> <deg> [speed]` | Smooth move (speed: 1–100%, default 50%) |
| `pulse <ch> <us>` | Set pulse width in microseconds |
| `stop <ch>` | Stop smooth move in progress |
| `attach <ch>` | Attach servo channel (enable PWM) |
| `detach <ch>` | Detach servo (disable PWM, power saving) |
| `status` | Print all servo channel info |
| `help` | Show command help |

`<ch>` is the channel number (0 or 1).

### Robot Head Commands (requires Robot Head enabled)

| Command | Description |
|---------|-------------|
| `action <name>` | Play head action with synchronized face expression |
| `actions` | List all available action names |
| `cancel` | Cancel the running action and face expression |

## Examples

```text
angle 0 45        # Set servo 0 to 45°
move 0 90 30      # Smooth move servo 0 to 90° at 30% speed
pulse 0 1500      # Set servo 0 pulse to 1500 µs
detach 0          # Detach servo 0 (stop PWM output)
action excited    # Play excited head + face expression
action sad        # Play sad head + face expression
actions           # List all: nod shake curious attention ...
cancel            # Stop current action
```

### 360° Continuous Rotation

For continuous rotation servos (configured as 360°), angle maps to speed/direction:

| Angle | Behavior |
|-------|----------|
| 0° | Full speed clockwise |
| 45° | Slow clockwise |
| 90° | Stop |
| 135° | Slow counter-clockwise |
| 180° | Full speed counter-clockwise |

## Wiring

| Signal | Default GPIO | Notes |
|--------|-------------|-------|
| Servo 0 (Pan) | GPIO 47 | Configurable via Kconfig |
| Servo 1 (Tilt) | GPIO 48 | Configurable via Kconfig |

Servo power supply: Use external 5V/6V supply for servo VCC. Do not power servos from the ESP32 3.3V pin. Connect servo GND to ESP32 GND.

## Default Configuration

- **Frequency**: 50 Hz (20 ms period)
- **Resolution**: 14-bit (16384 steps, ~1.22 µs precision)
- **Servo 0**: 90° standard, 500–2500 µs
- **Servo 1**: 360° continuous, 500–2500 µs

## Resource Usage (Face Mode)

| Resource | Usage |
|----------|-------|
| LVGL objects | ~12 |
| RAM (keyframes) | ~3 KB |
| Animation timer | 1 × 33 ms (30 fps) |
| CPU impact | Negligible (LVGL primitive moves only) |
