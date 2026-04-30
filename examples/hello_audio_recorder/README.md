# Hello Audio Recorder Example

This example demonstrates audio recording and playback functionality on the CUBE32 board with an LVGL user interface.

## Features

- **Audio recording** from microphone to SD card in WAV format
- **Audio playback** of recorded files through speaker
- **LVGL 9.x UI** with record/stop/play controls
- **Volume control** via on-screen slider
- **ADC button array support** for hardware button control
- **Progress bar** showing recording/playback progress
- **Time display** with elapsed and total duration
- **Configurable recording duration** (default 10 seconds max)

## Button Mapping (ADC Buttons)

| Button | Voltage | Resistor | Function |
|--------|---------|----------|----------|
| 0 | 0.38V | 1.3K | Volume Up |
| 1 | 0.82V | 3.3K | Volume Down |
| 2 | 1.11V | 5.1K | SET (test) |
| 3 | 1.65V | 10K | Play/Stop |
| 4 | 1.98V | 15K | Mute |
| 5 | 2.41V | 27K | Record/Stop |

All buttons also support long press events for testing.

## Requirements

### Hardware
- CUBE32 board with ESP32-S3
- FAT32 formatted SD card
- Microphone (ES7210 on CUBE32)
- Speaker

### SDK Configuration (menuconfig)

Enable the following options in `idf.py menuconfig`:

1. **Display Configuration**
   - `CUBE32 Board Configuration` → `Display Configuration` → `Enable Display`
   - `CUBE32 Board Configuration` → `Display Configuration` → `Enable LVGL`

2. **Audio Configuration**
   - `CUBE32 Board Configuration` → `Audio Configuration` → `Enable Audio`
   - `CUBE32 Board Configuration` → `Audio Configuration` → `Enable ADC Button Array` (optional)

3. **SD Card Configuration**
   - `CUBE32 Board Configuration` → `SD Card Configuration` → `Enable SD Card`

4. **Application Selection**
   - `CUBE32 Application Selection` → `Hello Audio Recorder Example`

## Recording Format

- **File format**: WAV (RIFF/WAVE)
- **Audio format**: PCM (uncompressed)
- **Sample rate**: Configured in menuconfig (default 24000 Hz)
- **Channels**: Mono
- **Bit depth**: 16-bit
- **File name**: `rec.wav` (8.3 format for FAT compatibility)

## Usage

1. Insert a FAT32 formatted SD card into the CUBE32 board
2. Build and flash the example
3. Use the touch screen UI:
   - Tap **REC** (red button) to start recording
   - Tap **STOP** (gray button) to stop recording/playback
   - Tap **PLAY** (green button) to playback the last recording
   - Adjust volume using the slider
4. Or use ADC buttons for hardware control:
   - Press **Record/Stop** (Button 5) to toggle recording
   - Press **Play/Stop** (Button 3) to toggle playback
   - Press **Volume Up/Down** (Buttons 0/1) to adjust volume
   - Press **Mute** (Button 4) to toggle mute

## Build and Flash

```bash
# Configure the project
idf.py menuconfig

# Build
idf.py build

# Flash
idf.py flash

# Monitor
idf.py monitor
```

## Configuration Options

The following constants can be modified in `main.cpp`:

| Constant | Default | Description |
|----------|---------|-------------|
| `MAX_RECORDING_DURATION_SEC` | 10 | Maximum recording duration in seconds |
| `AUDIO_BUFFER_SIZE` | 4096 | Audio buffer size in samples |
| `RECORDING_FILE_NAME` | "rec.wav" | Output file name |
| `INITIAL_VOLUME` | 80 | Initial volume level (0-100) |
| `VOLUME_STEP` | 5 | Volume step for button control |

## Notes

- Recording automatically stops when the maximum duration is reached
- The previous recording is overwritten when a new recording starts
- The WAV header is updated after recording completes with the actual data size
- If stereo input is enabled (with reference channel), it is automatically converted to mono
- The recorded file can be played back on any standard audio player that supports WAV format
