# Heltec Tracker GPS Lap Timer

GPS speedometer and lap timer for the **Heltec Wireless Tracker** (ESP32-S3 + built-in GNSS module + ST7735 display). Built for circuit / autocross / track-day use where you need lap times, a live delta against your best lap, and raw NMEA streamed out to any external device.

## Features

- **GPS lap timer** with auto-detected start/finish line crossing (linear interpolation between fixes for sub-GPS-tick accuracy)
- **Live delta comparison** against the best recorded lap — shows how many seconds ahead/behind and the speed delta at each point on the track
- **Best-lap trace recording**: up to 3000 points per lap (lat/lon + centisecond timestamp + speed), auto-saved when a new best is set
- **Post-finish summary** — 5-second window after each crossing shows old best / current lap / diff with color coding (green = faster, red = slower)
- **Speed tracking** with min/max per segment (acceleration vs braking phases)
- **Battery indicator** with color-coded voltage level
- **Multi-output NMEA bridge** — everything from the internal GPS UART is simultaneously mirrored to:
  - USB Serial (`Serial`) — see note below
  - Wi-Fi access point + TCP/Telnet server on port 23 (up to 2 clients)
  - **Bluetooth LE** via Nordic UART Service (NUS) — compatible with nRF Connect, Serial Bluetooth Terminal, and any BLE-UART app
  - **Bluetooth LE** via RaceChrono DIY BLE GPS protocol (service `0x1FF8`) — GPS-only mode
- **Bidirectional** — data written to any of those channels (USB, Telnet, BLE) is forwarded to the GPS module's UART (for configuration commands, rate changes, etc.)

## Hardware

- Heltec Wireless Tracker (ESP32-S3 with integrated UC6580 / UBX GNSS module and 0.96" ST7735 TFT)
- Single push button on GPIO 0 (built-in BOOT button)

## Network

- Wi-Fi AP: `WiFiBTGPS` / `87654321`
- Telnet: `192.168.4.1:23`
- BLE device name: `WiFiBTGPS`
- BLE services:
  - Nordic UART (`6E400001-B5A3-F393-E0A9-E50E24DCCA9E`) — raw NMEA passthrough
  - RaceChrono DIY GPS (`00001FF8-0000-1000-8000-00805F9B34FB`) — structured GPS characteristics

## Controls

- **Short press** on the button while GPS has a valid fix → sets the current position and heading as the start/finish line, starts the lap timer

## Display Layout

```
[HH:MM:SS] [B:XX%] [W] [B] [S:XX]   ← top row (1 Hz)
   [max]   [  speed  km/h  ]         ← main area (10 Hz)
   [min]
[best lap / delta / post-finish]     ← bottom row (10 Hz)
```

- **W** — green if a Wi-Fi/Telnet client is connected, gray otherwise
- **B** — cyan if a BLE client is connected, gray otherwise
- Speed is white when GPS fix is valid, red when not

## Architecture (FreeRTOS)

The firmware runs five concurrent FreeRTOS tasks:

| Task | Core | Rate | Responsibility |
|---|---|---|---|
| `GPS` | 1 | 20 Hz | Read Serial1 → NMEA, lap logic, enqueue BLE/WiFi TX |
| `WiFi` | 0 | 10 Hz | Telnet accept/read, drain WiFi TX queue to clients |
| `BLE` | 0 | 10 Hz | Drain BLE TX queue (NMEA notify), handle reconnect |
| `Display` | 0 | 10 Hz | Speed, min/max, lap bottom row |
| `Display` (slow) | 0 | 1 Hz | Time, satellites, battery, W/B indicators (inside Display task) |

BLE and WiFi TX paths are fully decoupled from the GPS task via dedicated ring buffers — a slow or disconnected client cannot stall GPS processing or the display.

An SPI mutex (`spiMutex`) serialises all ST7735 draw calls. An NMEA mutex (`nmeaMutex`) protects the MicroNMEA object and all derived state (lap timing, speed tracking, delta comparison).

## Serial (USB) note

`Serial` mirrors raw NMEA from the GPS module and accepts commands to forward back to the module (e.g. for changing fix rate with u-blox/UC6580 commands).

> **Important**: on ESP32-S3 the USB CDC interface resets the chip when the host disconnects.

## Dependencies

- Heltec ESP32 board package (ESP32-S3)
- `MicroNMEA`
- `HT_st7735` (big font patch)
- ESP32 BLE stack (built-in with the core)
