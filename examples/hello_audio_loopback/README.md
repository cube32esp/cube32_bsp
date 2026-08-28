# hello_audio_loopback

Real-time microphone-to-speaker loopback test for the CUBE32 board. It is
intended for interactive Acoustic Echo Cancellation (AEC) evaluation: speak
into the microphone and listen to the processed audio immediately through the
speaker.

## Overview

The application runs the following real-time pipeline at **16 kHz mono**:

```text
microphone → AEC processing → loopback queue → speaker
                  ↑                              │
                  └──── SW AEC reference queue ◄─┘
```

It provides an LVGL touchscreen interface with:

- AEC-mode drop-down selection
- Start and Stop controls
- Independent live speaker-volume and microphone-gain controls
- A status label showing the selected/effective AEC mode

No SD card or file recording is required.

## AEC modes

| Mode | Description | Availability |
|---|---|---|
| **No AEC** | Direct mic-to-speaker passthrough. Feedback/howling is expected at high volume. | All boards |
| **SW AEC** | Uses a digital copy of each speaker playback frame as the `esp-sr` software-AEC reference. | All boards |
| **HW AEC** | Uses the ES7210 channel-1 hardware speaker-reference loopback and `esp-sr` hardware AEC. | Dedicated Audio Module only |

**SW AEC is selected by default.**

### Integrated CUBE32 Core+Audio board

On the Integrated CUBE32 Core+Audio board, the ES8311 is detected at I2C
address `0x19` and supplies both the DAC speaker output and microphone ADC
input. It has no ES7210 hardware-reference channel, so **HW AEC is omitted
from the drop-down**. The available options are No AEC and SW AEC.

### Dedicated Audio Module

On the Dedicated Audio Module, ES8311 is detected at I2C address `0x18` and
the ES7210 supplies the microphone input plus hardware speaker-reference
channel. The drop-down additionally offers HW AEC.

## Prerequisites

In `idf.py menuconfig`, enable:

```text
CUBE32 Board Configuration → Audio Configuration → Enable Audio Support       [y]
CUBE32 Board Configuration → Display Configuration → Enable LVGL              [y]
CUBE32 Application Selection → Hello Audio Loopback Example                   [selected]
```

Also enable `LV_FONT_MONTSERRAT_14`, which is used by the interface.

## Using the application

1. Select **SW AEC** (the default) from the drop-down.
2. Start with moderate speaker volume and microphone gain.
3. Tap **Start**.
4. Speak near the microphone. The processed microphone signal plays through
   the speaker.
5. Adjust **Speaker** and **Mic Gain** live to evaluate echo suppression.
6. Tap **Stop** before changing the AEC mode, then tap Start again.

The application reinitialises the codec at 16 kHz when a run starts and
restores the configured codec defaults after it stops. Therefore, AEC-mode
changes take effect only on the next Start operation; volume and gain changes
take effect immediately.

> **Warning:** No AEC mode intentionally permits acoustic feedback. Start at a
> low speaker volume to avoid sudden loud howling.

## SW AEC timing alignment

SW AEC requires its digital playback reference to align with the acoustic echo
captured by the microphone. The I2S TX/DMA pipeline, amplifier, speaker, and
acoustic path all delay this echo. The application pre-fills the reference
queue with silent frames to compensate for this latency:

| Setting | Default | Meaning |
|---|---:|---|
| `AEC_FILTER_LENGTH` | 8 | Adaptive-filter length; supports a longer acoustic echo path than the prior 4-frame setting. |
| `REF_DELAY_FRAMES` | 2 | Reference delay in audio frames. With the usual 512-sample frame at 16 kHz, each frame is approximately 32 ms, so the default is approximately 64 ms. |
| `REF_QUEUE_DEPTH` | 12 | Software reference queue capacity, including the delay frames. |

If echo remains prominent, tune `REF_DELAY_FRAMES` in
[main.cpp](main/main.cpp) one step at a time. Test values from 0 through 4 on
the target hardware at the expected speaker volume. The optimal value depends
on DMA buffering, amplifier/speaker latency, microphone position, enclosure,
and room acoustics.

## Implementation details

- `loopback_capture_task` captures microphone frames, applies the chosen AEC
  algorithm, then pushes processed mono frames to the loopback queue.
- `loopback_play_task` removes processed frames from that queue, writes them to
  the ES8311 output, and provides a copy to the SW AEC reference queue.
- SW AEC uses `aec_create()`, `aec_get_chunksize()`, and `aec_process()` from
  `esp-sr`.
- HW AEC uses `afe_aec_create()`, `afe_aec_get_chunksize()`, and
  `afe_aec_process()` when the ES7210 reference channel is available.
- The codec driver prevents HW AEC when its ADC source is ES8311. The example
  has an additional fallback guard that switches safely to No AEC if HW AEC
  cannot be enabled.

## Build

```bash
idf.py set-target esp32s3
idf.py menuconfig
idf.py build flash monitor
```

Select **Hello Audio Loopback Example** in the CUBE32 Application Selection
menu before building.
