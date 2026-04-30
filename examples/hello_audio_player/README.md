# Hello Audio Player Example

This example demonstrates an audio player with LVGL UI that plays WAV and MP3 files from an SD card on the CUBE32 board.

## Features

- **LVGL 9.x UI** with file browser for audio file selection
- **WAV and MP3 playback** from SD card
- **Automatic sample rate detection** from file headers
- **MP3 decoding** using libhelix-mp3 decoder
- **Dynamic sample rate adjustment** to match audio file
- **Volume control** via on-screen slider
- **ADC button array support** for hardware button control
- **Progress display** with elapsed time and duration
- **Audio file information** display (sample rate, channels, duration)

## Button Mapping (ADC Buttons)

| Button | Voltage | Resistor | Function |
|--------|---------|----------|----------|
| 0 | 0.38V | 1.3K | Volume Up |
| 1 | 0.82V | 3.3K | Volume Down |
| 2 | 1.11V | 5.1K | SET (test) |
| 3 | 1.65V | 10K | Play/Stop |
| 4 | 1.98V | 15K | Mute |
| 5 | 2.41V | 27K | REC (test) |

All buttons also support long press events for testing.

## Requirements

### Hardware
- CUBE32 board with ESP32-S3
- FAT32 formatted SD card
- Audio files (WAV or MP3) in the root folder of the SD card

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
   - `CUBE32 Application Selection` → `Hello Audio Player Example`

## Supported Audio Formats

### WAV Files
- PCM encoded (uncompressed)
- 8-bit or 16-bit samples
- Mono or Stereo (downmixed to mono for playback)
- Any sample rate (automatically detected)

### MP3 Files
- MPEG Audio Layer III
- Various bitrates supported
- ID3v2 tags are skipped automatically
- Sample rate automatically detected from frame headers

## Usage

1. Copy WAV or MP3 files to the root folder of a FAT32 formatted SD card
2. Insert the SD card into the CUBE32 board
3. Build and flash the example
4. Use the touch screen to:
   - Browse and select audio files from the list
   - Tap PLAY to start playback
   - Tap STOP to stop playback
   - Adjust volume using the slider
5. Or use ADC buttons for hardware control

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

## Notes

- Stereo audio files are automatically downmixed to mono for the CUBE32 speaker
- The default volume is 80%
- Audio files must be in the root folder of the SD card
- Maximum 20 files are displayed in the file list
