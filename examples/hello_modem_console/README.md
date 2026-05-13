# Hello Modem Console Example

This example demonstrates the A7670 LTE modem driver with console interface for AT commands. Supports both **USB** and **UART** connection types, selected via `menuconfig`.

## Features

- **A7670 LTE Modem via USB or UART**: Connect to cellular network using USB CDC-ACM or UART interface
  - Connection type selected in `CUBE32 Board Configuration → Modem Configuration → Connection Type`
  - USB/UART routing handled automatically via the WAS7222Q analog switch (IOX TCA9554 Port 3)
- **Fully Non-blocking Modem Initialization**: 
  - Entire modem init (power on, USB/UART setup, network registration) runs in background
  - Display shows status immediately during startup
  - Console is available while modem initializes
- **AT Command Console**: Interactive console with linenoise for sending AT commands
- **LVGL 9.x UI**: Dark theme display showing:
  - Connection type in title bar (USB or UART)
  - Modem initialization status (with color coding)
  - Network registration status (updates in real-time)
  - Signal quality with visual bars
  - Operator name (displays after registration)
  - IP address (displays after PPP connection)
  - Current date/time
  - IMEI number
- **PPP Data Mode**: Switch between command mode and data mode (PPP for internet)
- **HTTP/HTTPS Support**: Make HTTP GET requests including HTTPS with certificate validation
- **Ping Support**: ICMP ping to test network connectivity

## Initialization Flow

The modem initialization is designed for fast system startup with immediate UI feedback:

1. **cube32_init()** - Initializes all drivers, modem starts in background task
2. **UI Created Immediately** - Screen displays "INITIALIZING" status
3. **Console Available** - Can type commands while modem initializes
4. **Background Modem Init**:
   - IO Expander configured (sets comm port to USB or UART via TCA9554 Pin 3)
   - Modem powered on via PWRKEY
   - USB CDC-ACM interface initialized, or UART DTE created (GPIO 19/20 reconfigured from USB PHY)
   - Network registration started
5. **Status Updates on Screen** - Color-coded status:
   - Orange: Initializing
   - Yellow: Registering with network
   - Green: Ready (registered)
   - Red: Error

This allows the system to show meaningful status information immediately rather than displaying a blank screen during the modem's power-on and network registration sequence (which can take 10-60 seconds).

## Status Colors

| Status | Color | Meaning |
|--------|-------|---------|
| INITIALIZING | Orange | Modem is powering on/USB initializing |
| INITIALIZED | Yellow | Modem ready, waiting for network |
| Registering | Yellow | Searching for cellular network |
| Registered | Green | Connected to network |
| DATA (PPP) | Green | PPP connection active |
| ERROR | Red | Initialization or connection failed |

## Prerequisites

### Hardware
- CUBE32 board with A7670 modem module
- Active SIM card inserted in modem
- Display (optional, for LVGL UI)
- For USB mode: USB connection between ESP32-S3 and A7670
- For UART mode: UART connection via PCB traces (ESP GPIO 20 → A7670 RXD, ESP GPIO 19 ← A7670 TXD), routed through WAS7222Q analog switch

### Configuration

Enable the following in `menuconfig`:

1. **Modem Configuration**:
   - `CUBE32 Board Configuration → Modem Configuration → Enable Modem`
   - `Connection Type`: USB or UART
   - **For USB**:
     - `USB VID`: 0x1E0E (default for A7670)
     - `USB PID`: 0x9011 (default for A7670)
   - **For UART**:
     - Port, TX pin, and RX pin are defined in `cube32_config.h` (UART1, GPIO 20, GPIO 19)
     - `Baud Rate`: 115200 (default)
     - Note: GPIO 19/20 are shared with ESP32-S3 USB D-/D+ pins and are automatically reconfigured as GPIO when UART mode is selected
   - `APN`: Your carrier's APN (e.g., "internet" or carrier-specific)

2. **Display Configuration** (Optional):
   - `CUBE32 Board Configuration → Display Configuration → Enable Display`
   - `Enable LVGL`
   - Display model: Select your display (CUBE_TFT_TOUCH_154 or CUBE_TFT_TOUCH_200)

3. **LVGL Fonts** (if using LVGL):
   - `Component config → LVGL configuration → Font usage`
   - Enable: `Montserrat 14`, `Montserrat 24`, `Montserrat 32`

4. **Application Selection**:
   - `CUBE32 Application Selection → Hello Modem Console`

## Console Commands

### Modem Control
- `status` - Print detailed modem status
- `info` - Print modem information (manufacturer, model, IMEI, IMSI, signal, operator)
- `sync` - Sync with modem (send AT command)
- `reset` - Reset the modem
- `data` - Switch to data mode (PPP for internet access)
- `cmd` - Switch back to command mode
- `mode` - Show current modem mode (command/data/transition)
- `netstat` - Show network status summary

### AT Commands
- `AT<command>` - Send any AT command directly to modem
  - Examples:
    - `AT+CSQ` - Get signal quality
    - `AT+COPS?` - Get current operator
    - `AT+CPIN?` - Check SIM status
    - `AT+CREG?` - Check network registration

### Network Commands (requires data mode)
- `httpget <url>` - HTTP GET request
  - Examples:
    - `httpget http://example.com`
    - `httpget https://www.google.com`
    - `httpget https://httpbin.org/get`
- `ping <host> [count]` - Ping a host (default count: 4)
  - Examples:
    - `ping 8.8.8.8`
    - `ping www.google.com 10`

### IO Expander Control
The A7670 modem module uses a TCA9554 IO Expander for power and communication control.

- `iox` - Show IO Expander status and pin states
- `pwron` - Run full power-on sequence (enable power rail, pulse PWRKEY)
- `pwroff` - Run power-off sequence (pulse PWRKEY, disable power rail)
- `pwrkey on` - Turn on modem via PWRKEY (full power-on sequence)
- `pwrkey off` - Turn off modem via PWRKEY (full power-off sequence)
- `dtr <0|1>` - Set DTR pin (0=low, 1=high, default high)
- `pwr <0|1>` - Set power rail (0=off, 1=on)
- `comm <usb|uart>` - Set communication port selection

IO Expander Pin Mapping:
- P0: PWRKEY - Pulse low to toggle modem power
- P1: DTR - Data Terminal Ready (idle high)
- P2: Power Rail - Module power control (L=Off, H=On)
- P3: Comm Select - MCU communication port (L=USB, H=UART)

### Help
- `help` or `?` - Show all available commands

## Usage Example

1. **Flash and Monitor**:
   ```bash
   idf.py flash monitor
   ```

2. **Wait for Modem Initialization** (USB example):
   ```
   I (564) hello_modem: Modem: Initializing A7670 (USB)...
   I (7504) cube32_modem: USB modem connected successfully
   I (8004) cube32_modem: A7670 Modem initialized successfully
   ```

   Or (UART example):
   ```
   I (564) cube32_modem: Initializing UART connection (Port: 1, TX: 20, RX: 19, Baud: 115200)
   I (564) cube32_modem: Reconfiguring GPIO 20/19 from USB to GPIO for UART
   I (1064) cube32_modem: UART modem connected successfully
   ```

3. **Check Modem Status**:
   ```
   modem> status
   ```

4. **Get Modem Information**:
   ```
   modem> info
   ```

5. **Switch to Data Mode** (for internet):
   ```
   modem> data
   ```
   Wait for network registration and PPP connection.

6. **Make HTTP Request**:
   ```
   modem> httpget https://httpbin.org/get
   ```

7. **Ping Test**:
   ```
   modem> ping 8.8.8.8
   ```

8. **Switch Back to Command Mode**:
   ```
   modem> cmd
   ```

## Typical Workflow

### 1. Basic AT Commands (Command Mode)
```
modem> AT+CSQ
modem> AT+COPS?
modem> status
modem> info
```

### 2. Internet Access (Data Mode)
```
modem> data                          # Switch to PPP mode
modem> ping 8.8.8.8                  # Test connectivity
modem> httpget https://www.google.com # HTTP request
modem> cmd                           # Back to command mode
```

## Troubleshooting

### Modem Not Detected
- Verify USB connection
- Check power supply (modem requires significant power)
- Verify USB VID/PID in configuration matches your modem
- Check USB interrupt availability (modem should be initialized before camera)

### No Network Registration
- Verify SIM card is inserted and unlocked
- Check signal quality: `AT+CSQ` (should be < 31, not 99)
- Verify APN settings match your carrier
- Try manual network selection: `AT+COPS=1,2,"XXXXX"` (where XXXXX is MCC+MNC)

### Ping/HTTP Fails
- Ensure modem is in data mode first (`data` command)
- Check network registration status
- Verify PPP connection has IP address (`status` command)
- Check signal quality is adequate
- Verify APN configuration is correct

### Sync Failed
- This is normal on first boot - modem needs time to initialize after USB enumeration
- AT commands will work once modem is ready
- The driver retries with delays

## Signal Quality Reference

RSSI values from `AT+CSQ`:
- 0-9: Poor signal
- 10-14: Fair signal
- 15-19: Good signal  
- 20-31: Excellent signal
- 99: No signal or unknown

## Network Registration Status

- `Not Registered`: No network coverage or SIM issue
- `Searching`: Looking for network
- `Denied`: SIM or network restriction
- `Registered (Home)`: Connected to home network
- `Registered (Roaming)`: Connected while roaming

## Notes

- **Non-blocking Initialization**: The modem uses async network registration - `begin()` returns quickly and network registration runs in a background task
- The modem is initialized before the camera and SD card in `cube32_init()` to avoid USB interrupt conflicts
- **Network Ready Notification**: The system notifies via console when network registration completes
- **IP Address**: Only available after switching to data mode (PPP) with the `data` command
- **IO Expander Control**: The TCA9554 IO Expander (I2C address 0x22) provides power and comm port control for the A7670 module
- **Power-on Sequence**: The modem driver automatically runs the power-on sequence during initialization if IO Expander is available
- HTTPS requests use ESP certificate bundle for TLS validation
- Ping and HTTP commands require the modem to be in data mode (PPP)
- The console uses UART for input/output (not USB Serial JTAG)
- AT commands can only be sent in command mode, not data mode
- **Operator Display**: The UI shows operator name once registered (in command mode only)

## See Also

- [ESP-MODEM Documentation](https://docs.espressif.com/projects/esp-protocols/esp_modem/docs/latest/index.html)
- [A7670 AT Command Manual](https://www.simcom.com/product/A7670C.html)
- CUBE32 Modem Driver: `components/cube32/include/drivers/modem/a7670_modem.h`
- CUBE32 TCA9554 Driver: `components/cube32/include/drivers/io_expander/tca9554.h`
