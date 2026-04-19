# Heltec Tracker GPS Lap Timer

GPS speedometer and lap timer for the **Heltec Wireless Tracker** (ESP32-S3 + built-in GNSS module + ST7735 display). Built for circuit / autocross / track-day use where you need lap times, a live delta against your best lap, and raw NMEA streamed out to any external device (RaceChrono).

## Features

- **GPS lap timer** with auto-detected start/finish line crossing (linear interpolation between fixes for sub-GPS-tick accuracy)
- **Live delta comparison** against the best recorded lap — shows how many seconds ahead/behind and the speed delta at each point on the track
- **Best-lap trace recording**: up to 3000 points per lap (lat/lon + centisecond timestamp + speed), auto-saved when a new best is set
- **Post-finish summary** — 5-second window after each crossing shows old best / current lap / diff with color coding (green = faster, red = slower)
- **Speed tracking** with min/max per segment (acceleration vs braking phases)
- **Battery indicator** with color-coded voltage level
- **Multi-output NMEA bridge** — everything from the internal GPS UART is simultaneously mirrored to:
  - USB Serial (`Serial`)
  - Wi-Fi access point + TCP/Telnet server on port 23 (up to 2 clients)
  - **Bluetooth LE** via Nordic UART Service (NUS) — compatible with nRF Connect, Serial Bluetooth Terminal, and any BLE-UART app
- **Bidirectional** — data written to any of those channels (USB, Telnet, BLE) is forwarded to the GPS module's UART (for configuration commands, rate changes, etc.)

## Hardware

- Heltec Wireless Tracker (ESP32-S3 with integrated UC6580 / UBX GNSS module and 0.96" ST7735 TFT)
- Single push button on GPIO 0 (built-in BOOT button)

## Network

- Wi-Fi AP: `WiFiBTGPS` / `87654321`
- Telnet: `192.168.4.1:23`
- BLE device name: `WiFiBTGPS`
- BLE service: Nordic UART (`6E400001-B5A3-F393-E0A9-E50E24DCCA9E`)

## Controls

- **Short press** on the button while GPS has a valid fix → sets the current position and heading as the start/finish line, starts the lap timer

## Dependencies

- Heltec ESP32 board package
- `MicroNMEA`
- `HT_st7735` (patched for BIG font)
- ESP32 BLE stack (built-in with the core)
