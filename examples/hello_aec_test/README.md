# hello_aec_test

Acoustic Echo Cancellation (AEC) comparison demo for the CUBE32 board.

## Overview

Records a 1 kHz test tone played through the speaker in three AEC modes and
saves a separate WAV file to the SD card for each.  Comparing the three files
lets you hear exactly how much echo each approach removes.

| Mode | Description |
|------|-------------|
| **No AEC** | Raw microphone — echo fully present |
| **HW AEC** | Hardware reference loopback via ES7210 ch1 + `esp-sr` `afe_aec` |
| **SW AEC** | Digital copy of the playback buffer as reference + `esp-sr` `aec` |

All modes operate at **16 kHz mono** (required by esp-sr AEC).

## Hardware

- CUBE32 board (ESP32-S3)
- ES8311 DAC / speaker amplifier
- ES7210 4-channel TDM ADC — ch0 = mic, ch1 = speaker reference loopback
- FAT32-formatted micro-SD card

## Prerequisites in `idf.py menuconfig`

```
CUBE32 Board → Audio → Enable Audio Support       [y]
                     → AEC Mode                    [HW AEC / SW AEC / No AEC]
CUBE32 Board → Display → Enable LVGL              [y]
CUBE32 Board → Storage → Enable SD Card           [y]
CUBE32 App Selection   → Hello AEC Test           [selected]
```

Optional audio defaults (also under `CUBE32 Board → Audio`):

| Config | Default | Meaning |
|--------|---------|--------|
| `CUBE32_AUDIO_INPUT_SAMPLE_RATE` | 16000 | ADC sample rate — auto-clamped to 16 kHz when AEC enabled |
| `CUBE32_AUDIO_OUTPUT_SAMPLE_RATE` | 24000 | DAC sample rate (Hz) |
| `CUBE32_AUDIO_OUTPUT_VOLUME` | 70 | Speaker volume (0–100) |
| `CUBE32_AUDIO_INPUT_GAIN` | 30 | Microphone gain (dB) |

## Output files on SD card

| File | Contents |
|------|----------|
| `aec_none.wav` | No AEC — raw mic |
| `aec_hw.wav` | Hardware AEC result |
| `aec_sw.wav` | Software AEC result |

WAV format: 16-bit PCM, mono, 16 kHz, 5 seconds.

## UI

The LVGL touchscreen UI has three sections:

1. **Mode dropdown** — select No AEC / HW AEC / SW AEC before recording
2. **Record / Stop buttons + progress bar** — run the 5-second capture
3. **Play buttons** (one per mode) + volume slider — audition each WAV on-device

## How it works

```
tone_play_task  ─────► speaker (ES8311)
                              │
                    hardware loopback (HW AEC only)
                              │
audio_record_task ◄─── mic + optional ref (ES7210)
        │
        ├─ No AEC:  write raw mic samples to WAV
        ├─ HW AEC:  afe_aec_process(mic+ref interleaved) → WAV
        └─ SW AEC:  aec_process(mic, queue_ref) → WAV
                              ▲
                    tone_play_task pushes each playback
                    frame to s_ref_queue for SW AEC
```

The record and tone tasks run concurrently, both at 16 kHz.  After recording,
the codec is re-initialised at Kconfig defaults for normal operation.

## esp-sr API used

| Function | Purpose |
|----------|---------|
| `aec_create(sr, filter_len, nch, mode)` | Create software AEC handle |
| `aec_get_chunksize(handle)` | Query frame size in samples |
| `aec_process(handle, mic, ref, out)` | Process one frame |
| `aec_destroy(handle)` | Free resources |
| `afe_aec_create(mic_cfg, filter_len, type, mode)` | Create hardware-optimised AEC |
| `afe_aec_get_chunksize(handle)` | Query frame size |
| `afe_aec_process(handle, in, out)` | Process interleaved mic+ref frame |
| `afe_aec_destroy(handle)` | Free resources |

## Build

```bash
idf.py set-target esp32s3
idf.py menuconfig          # select Hello AEC Test
idf.py build flash monitor
```

## Notes

- AEC only works at **16 kHz**.  The audio driver automatically clamps the input
  sample rate to 16 kHz when AEC mode is HW or SW.  The demo sets
  `output_sample_rate` to 16 kHz explicitly for the test tone.
- SW AEC uses a FreeRTOS queue (depth 8) to pass playback frames as the
  acoustic reference.  The queue depth provides ~256 ms of buffering.
- HW AEC uses the physical reference signal on ES7210 ch1, which captures the
  actual speaker output including PA and speaker non-linearities.
- The playback task reinitialises the I2S controller at the WAV sample rate to
  guarantee the clock matches the recorded data.
