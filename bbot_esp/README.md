# Battle Bot ESP32 Firmware

Embedded firmware for an [ESP32 Servo Driver Expansion Board](https://www.waveshare.com/servo-driver-with-esp32.htm) controlling a differential-drive battle bot via [Pico-ROS](https://github.com/MarqRazz/Pico-ROS-software) (Zenoh-pico).

## Hardware

- Waveshare ESP32 Servo Driver board
- 2x Feetech SCServo drive wheels (`bbot_wheel_left_joint`, `bbot_wheel_right_joint`)
- 1x weapon actuator (`bbot_weapon_joint`)

## Build & Flash

Requires [PlatformIO](https://platformio.org/install/ide?install=vscode) (VS Code extension or CLI).

```bash
cd bbot_esp
pio run                    # Build
pio run --target upload    # Build and flash
pio device monitor         # Serial monitor (115200 baud)
```

## WiFi & Zenoh Configuration

On first boot (or after a settings reset), the ESP32 starts a **WiFiManager captive portal**:

1. Connect to the `BattleBot-Setup` WiFi access point from your phone or laptop.
2. The captive portal opens automatically. Select your WiFi network and enter the password.
3. Set the **Zenoh router address** (default: `tcp/192.168.1.100:7447`) — this should point to the machine running the Docker container.
4. Click **Save**. The ESP32 reboots and connects to your network.

WiFi credentials and the Zenoh router address are persisted across reboots (WiFiManager + NVS).

### Reset Settings

Hold the **BOOT button** (GPIO 0) during power-on to clear saved WiFi and Zenoh settings. The captive portal will reopen.

If no one connects to the portal within 120 seconds, the ESP32 blinks red and restarts.

## LED Status

| Phase | Color | Meaning |
|---|---|---|
| Startup | Orange blink | Initializing |
| Portal active | Cyan blink | Waiting for WiFi configuration |
| WiFi connected | Blue solid (2s) | Connected to WiFi |
| Portal timeout | Red blink x6 | No config received, restarting |
| Zenoh waiting | Red blink | Waiting for Zenoh router connection |
| Running | Green blink | Publishing joint states |
