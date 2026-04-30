# Hello RTC NTP Example

Demonstrates BM8563 RTC initialization, NTP time synchronization, and LVGL display. Supports both WiFi and LTE Modem (A7670) for network connectivity.

## Features

- **BM8563 RTC** - Initialize and read/write real-time clock
- **Automatic Network Selection** - Runtime detection selects the best available connection:
  - **LTE Modem (A7670)** - Used when `CONFIG_CUBE32_MODEM_ENABLED` is set and the modem module is physically detected at boot
  - **WiFi** - Used when modem is not enabled or not detected (automatic fallback)
- **NTP Synchronization** - Sync time with `time.cloudflare.com` over the active connection
- **Automatic RTC Update** - Update RTC with NTP time after synchronization
- **LVGL Display** - Dark theme UI showing:
  - Current date and time (large format with short month names)
  - Network connection status and IP address (WiFi or LTE)
  - NTP synchronization status
  - Last sync timestamp
- **Timezone Support** - Display time in configured local timezone

## Hardware Requirements

- CUBE32 board with:
  - ST7789 display
  - BM8563 RTC chip
  - WiFi connectivity (always required as fallback)
  - Optional: A7670 LTE modem module (auto-detected at boot)

## Configuration

### 1. Enable Required Components

In `idf.py menuconfig`:

**CUBE32 Board Configuration → RTC Configuration:**
- ✅ Enable RTC (BM8563)
- Set Local Time Zone (default: Asia/Singapore)

**CUBE32 Board Configuration → Display Configuration:**
- ✅ Enable Display
- ✅ Enable LVGL

**Component config → LVGL → Font usage:**
- ✅ Enable Montserrat 14
- ✅ Enable Montserrat 24 (for date display)
- ☑️ Enable Montserrat 32 or 48 (optional, for larger time display)
- ☑️ Enable Montserrat 64 (optional, for very large time display)

**CUBE32 WiFi Configuration:**
- Set WiFi SSID (your network name)
- Set WiFi Password

> **Note:** WiFi credentials are always required even when using the modem — they are used as the fallback connection if the modem module is not present at boot.

**Optional — LTE Modem:**

**CUBE32 Board Configuration → Modem Configuration:**
- ✅ Enable Modem (A7670)

When modem support is enabled and the A7670 module is physically detected at boot (TCA9554 GPIO expander at I2C address 0x22), the demo automatically uses LTE for NTP synchronization. If the module is enabled in Kconfig but not detected, it falls back to WiFi with a warning.

### 2. Select Application

**CUBE32 Application Selection:**
- Select "Hello RTC NTP Example"

### 3. Timezone Configuration

The timezone is configured in menuconfig:

**CUBE32 Board Configuration → RTC Configuration → Local Time Zone**

Common timezone values:
- `Asia/Singapore` - Singapore/Malaysia Time (GMT+8, default)
- `Asia/Hong_Kong` - Hong Kong Time (GMT+8)
- `Asia/Shanghai` - China Standard Time (GMT+8)
- `Asia/Tokyo` - Japan Standard Time (GMT+9)
- `UTC` - UTC
- `America/Los_Angeles` - US Pacific Time (with DST)
- `America/New_York` - US Eastern Time (with DST)
- `Europe/London` - UK Time (with DST)

Or use POSIX timezone format:
- `CST-8` - China/Hong Kong/Singapore (no DST)
- `PST8PDT,M3.2.0,M11.1.0` - US Pacific (with DST)
- `EST5EDT,M3.2.0,M11.1.0` - US Eastern (with DST)

## Build and Flash

```bash
idf.py build
idf.py -p PORT flash monitor
```

## Expected Behavior

1. Display shows "CUBE32 RTC + NTP" title
2. Network connects (LTE modem if detected, otherwise WiFi — shows IP address)
3. NTP syncs with time server (may take a few seconds)
4. Time display updates every 500ms
5. RTC is automatically updated with NTP time
6. Time persists after power cycle (battery-backed RTC)

## Display Layout

```
┌────────────────────────┐
│   CUBE32 RTC + NTP     │
│                        │
│      14:30:25          │  ← Large time
│                        │
│    Jan 9, 2024         │  ← Date
│                        │
│      Thursday          │  ← Weekday
│                        │
├────────────────────────┤
│ WiFi Connected: IP     │
│ ✓ NTP Synced           │
│ Last sync: 14:30:00    │
└────────────────────────┘
```

## Troubleshooting

**WiFi won't connect:**
- Check SSID and password in menuconfig
- Verify WiFi network is 2.4GHz (ESP32 doesn't support 5GHz)

**LTE modem not used despite being enabled:**
- Confirm the A7670 module is physically installed (modem detection is runtime, not just Kconfig)
- Check serial log for "Modem module detected" or "module not detected — falling back to WiFi"

**NTP won't sync:**
- Ensure network is connected (check IP address shown on display)
- Check firewall allows NTP (UDP port 123)
- Try different NTP server in `main.cpp` (e.g., `"pool.ntp.org"`)

**Time is wrong:**
- Verify timezone setting matches your location
- NTP always returns UTC time - timezone conversion happens locally

**Display not working:**
- Verify LVGL fonts are enabled in menuconfig
- Check display is properly connected
