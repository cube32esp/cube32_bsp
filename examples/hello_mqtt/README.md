# Hello MQTT Example

This example demonstrates an MQTT client with AWS IoT support, featuring automatic internet connectivity via WiFi or LTE modem, SSL/TLS encryption, and a modern LVGL 9.x dark theme UI.

## Features

- **MQTT Client with AWS IoT Support**: Connect to any MQTT broker including AWS IoT Core
- **Automatic Network Selection**:
  - Uses LTE Modem (A7670) when modem is **both** enabled in Kconfig **and** the modem module is physically detected at boot **and** the modem Active flag (NVS `act_modem`) is `true`
  - Automatically falls back to WiFi if modem is enabled in Kconfig but the hardware module is not present, or if the Active flag is set to `false`
  - Always uses WiFi when modem is not enabled in Kconfig
- **Connection Mode Display**: The connection section shows `📞 4G/LTE` or `📶 WiFi`, determined at boot before the first UI render
- **SSL/TLS Support**: Secure connections with CA certificates loaded from SD card
- **LVGL 9.x Dark Theme UI** displaying:
  - Connection mode (WiFi/Modem) and status
  - Reconnect time countdown
  - Signal/WiFi strength with visual bars
  - Subscription topic name and received messages
  - Publishing topic name and status
  - Message counters (RX/TX)
  - Publish button for system status JSON
- **System Status Publishing**: One-button publishing of device info in JSON format

## MQTT Configuration

The MQTT settings are configured as defines at the top of `main.cpp`:

### Broker Settings

```cpp
#define MQTT_BROKER_HOST        "broker.hivemq.com"  // MQTT broker hostname
#define MQTT_BROKER_PORT        1883                  // Port (1883=plain, 8883=TLS)
#define MQTT_USE_SSL            false                 // Enable SSL/TLS
#define MQTT_BROKER_USERNAME    ""                    // Username (if required)
#define MQTT_BROKER_PASSWORD    ""                    // Password (if required)
```

### Connection Settings

```cpp
#define MQTT_CONNECT_TIMEOUT_MS 30000    // Connection timeout (30 seconds)
#define MQTT_KEEPALIVE_SEC      60       // Keep-alive interval (60 seconds)
#define MQTT_AUTO_RECONNECT     true     // Auto reconnect on disconnect
#define MQTT_RECONNECT_PERIOD_MS 5000    // Reconnect period (5 seconds)
```

### Topics

```cpp
#define MQTT_SUBSCRIBE_TOPIC    "cube32/demo/command"  // Subscribe topic
#define MQTT_PUBLISH_TOPIC      "cube32/demo/status"   // Publish topic
```

### SSL Certificate Files (on SD Card)

```cpp
#define MQTT_CA_CERT_FILE       "/sdcard/certs/ca.pem"        // CA certificate
#define MQTT_CLIENT_CERT_FILE   "/sdcard/certs/client.crt"    // Client certificate
#define MQTT_CLIENT_KEY_FILE    "/sdcard/certs/client.key"    // Client private key
```

## AWS IoT Configuration

To use with AWS IoT Core, modify the settings:

```cpp
#define MQTT_BROKER_HOST        "your-endpoint.iot.region.amazonaws.com"
#define MQTT_BROKER_PORT        8883
#define MQTT_USE_SSL            true
#define MQTT_SUBSCRIBE_TOPIC    "cube32/sub"
#define MQTT_PUBLISH_TOPIC      "cube32/pub"
```

Then place your AWS IoT certificates on the SD card:
- `ca.pem` - Amazon Root CA
- `client.crt` - Device certificate
- `client.key` - Device private key

## System Status JSON

When the Publish button is pressed, the following JSON is published:

```json
{
  "device": "CUBE32",
  "uptime_sec": 12345,
  "free_heap": 123456,
  "min_free_heap": 100000,
  "ip": "192.168.1.100",
  "connection": "wifi",
  "wifi_rssi": -65,
  "mqtt_rx": 10,
  "mqtt_tx": 5,
  "timestamp": 1705555200
}
```

For modem connections, `wifi_rssi` is replaced with `signal_rssi`.

## Prerequisites

### Hardware

- CUBE32 board with display
- Active internet connection:
  - WiFi access point, OR
  - A7670 modem with active SIM card
- SD card (optional, for SSL certificates)

### Configuration

Enable the following in `menuconfig`:

1. **Display Configuration**:
   - `CUBE32 Board Configuration → Display Configuration → Enable Display`
   - `Enable LVGL`
   - Display model: Select your display (CUBE_TFT_TOUCH_154 or CUBE_TFT_TOUCH_200)

2. **Network Configuration**:

   WiFi credentials are always required as the default (and fallback) connection method.

   **WiFi (required)**
   - `CUBE32 WiFi Configuration → WiFi SSID`
   - `CUBE32 WiFi Configuration → WiFi Password`

   **LTE Modem (optional)**
   - `CUBE32 Board Configuration → Modem Configuration → Enable Modem`
   - Configure APN for your carrier
   - When enabled, the modem is used **only if** the A7670 module is physically attached (detected via TCA9554 @ I2C 0x22) **and** the modem Active flag is `true`
   - Set the Active flag via the **CUBE32 Mobile App** using the BLE OTA subsystem (NVS key `act_modem`)
   - If the module is present but the Active flag is `false`, WiFi is used automatically

3. **SD Card** (for SSL certificates):
   - `CUBE32 Board Configuration → SD Card Configuration → Enable SD Card`

4. **LVGL Fonts**:
   - `Component config → LVGL configuration → Font usage`
   - Enable: `Montserrat 14`, `Montserrat 24`

5. **Application Selection**:
   - `CUBE32 Application Selection → Hello MQTT Example`

## UI Layout

```
┌────────────────────────────┐
│  MQTT Client      ▂▄▆█ dBm│
├────────────────────────────┤
│ 📶 WiFi  OR  📞 4G/LTE      │  ← Mode badge (set at boot)
│ IP: 192.168.1.100          │
│ MQTT: Connected            │
│ Reconnect in: 5s           │
├────────────────────────────┤
│ ⬇ cube32/demo/command      │
│ {"cmd": "hello"}           │
├────────────────────────────┤
│ ⬆ cube32/demo/status       │
│ Published OK               │
├────────────────────────────┤
│ RX: 10  TX: 5   [Publish]  │
└────────────────────────────┘
```

## Status Colors

| Element | Color | Meaning |
|---------|-------|---------|
| IP Address | Green | Connected |
| IP Address | Orange | Connecting |
| MQTT Status | Green | Connected |
| MQTT Status | Orange | Connecting |
| MQTT Status | Red | Waiting for network |
| Reconnect | Orange | Countdown active |

## Testing

### Using HiveMQ Public Broker (Default)

1. Build and flash the example
2. Open an MQTT client (e.g., MQTT Explorer, mosquitto_sub)
3. Connect to `broker.hivemq.com:1883`
4. Subscribe to `cube32/demo/status`
5. Publish to `cube32/demo/command` to see messages on the display
6. Press the Publish button to send system status

### Test Commands

```bash
# Subscribe to status topic
mosquitto_sub -h broker.hivemq.com -t "cube32/demo/status" -v

# Publish test message
mosquitto_pub -h broker.hivemq.com -t "cube32/demo/command" -m '{"cmd":"test"}'
```

## Troubleshooting

### LTE Modem Not Used Despite Being Enabled

- Confirm the A7670 module is physically installed (detected via TCA9554 @ I2C 0x22)
- Check that the modem Active flag is enabled — set it via the **CUBE32 Mobile App** (BLE OTA subsystem) and verify `act_modem = true` in NVS
- Check the serial log for one of:
  - `"Modem module detected and active — using LTE"` → modem path taken
  - `"Modem detected but Active flag is off — falling back to WiFi"` → module present but flag is off; WiFi will be used
  - `"Modem enabled in Kconfig but module not detected"` → module not physically installed

### MQTT Connection Failed

1. Check network connectivity (WiFi or modem)
2. Verify broker hostname and port
3. Check if SSL is required but not enabled
4. Verify credentials if broker requires authentication

### SSL Certificate Errors

1. Ensure SD card is mounted
2. Check certificate file paths
3. Verify certificate files are valid PEM format
4. For AWS IoT, ensure all three files (CA, cert, key) are present

### No Messages Received

1. Verify subscription topic matches publisher
2. Check QoS settings
3. Ensure broker is routing messages correctly

### UI Not Updating

1. Verify LVGL is enabled and working
2. Check that fonts are enabled in menuconfig
3. Ensure display is properly initialized

## Dependencies

This example requires the following ESP-IDF components:
- `mqtt` - ESP-MQTT client
- `esp_tls` - TLS/SSL support
- `esp_wifi` - WiFi connectivity (when not using modem)
- `cJSON` - JSON serialization
- `lvgl` - Graphics library

## License

This example is part of the CUBE32 project and follows the same license.
