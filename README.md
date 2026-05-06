# Drone Detector V2

A two-board ESP32 system that passively detects consumer drones using **six simultaneous RF detection layers** and a companion simulator for end-to-end testing — no SDR, no cloud, no subscription required.

---

## Overview

| Board | Role | Chip |
|---|---|---|
| `drone_detector/` | Passive listener — detects nearby drones | ESP32-C5-DevKitC-1 v1.2 |
| `drone_simulator/` | Transmitter — impersonates real drone models for testing | ESP32-C3 Super Mini |

The detector never transmits. It silently watches 2.4 GHz Wi-Fi and BLE for the RF signatures of commercial drones. The simulator cycles through 11 real consumer drone models, broadcasting authentic RF signatures so you can verify every detection layer is working before field deployment.

---

## Detection Methods

The detector runs all six layers simultaneously:

| # | Layer | What it catches |
|---|---|---|
| 1 | **Wi-Fi OUI Fingerprinting** | First 3 bytes of MAC address match a known drone manufacturer (DJI, Parrot, Autel, Skydio, …) |
| 2 | **SSID Pattern Matching** | Beacon/probe SSID matches one of 59 known drone naming formats (DJI-Mini-4P-XXXX, ANAFI-XXXXXX, …) |
| 3 | **NAN Remote ID Frames** | 802.11 action frames sent to `51:6F:9A:01:00:00` — used by DJI for ASTM F3411 Remote ID over Wi-Fi |
| 4 | **Vendor Information Elements** | Beacon IE tag `0xDD` with OUI `FA:0B:BC` — DJI's ASTM F3411 payload embedded in standard Wi-Fi beacons |
| 5 | **BLE Remote ID (FAA §89.115)** | BLE advertisements with Service UUID `0xFFFA` — FAA-mandated Remote ID broadcast required for drones >250 g sold after Sept 2023 |
| 6 | **RSSI Tracking** | Signal strength trend over time — rising RSSI = drone approaching, falling = departing |

All 2.4 GHz Wi-Fi channels (1–13) are scanned in a hopping pattern. Wi-Fi and BLE coexist using the ESP32's hardware coexistence scheduler.

---

## Hardware

### Detector
- **ESP32-C5-DevKitC-1 v1.2** (dual-band, BT 5.3 LE)
- NeoPixel WS2812B LED on **GPIO 27**
- USB-C for power and serial monitor

### Simulator
- **ESP32-C3 Super Mini**
- Onboard LED on **GPIO 8** (active LOW) — blinks at 2 Hz while running
- USB-C or battery bank for portable operation

---

## NeoPixel Status (Detector)

| Color | Meaning |
|---|---|
| Slow blue blink | Scanning — no drone detected |
| Green | Wi-Fi OUI or SSID match |
| Yellow | NAN Remote ID frame detected |
| Blue (solid) | BLE Remote ID detected |
| White | Vendor IE match |
| Red | High-confidence detection (multiple layers) |

Signal is cleared after 5 seconds without a re-detection.

---

## Simulated Drone Models (Simulator)

The simulator cycles through these real-world models (12 seconds each):

| Model | OUI | Detection Layers |
|---|---|---|
| DJI Mini 4 Pro | `34:D2:62` | All 6 |
| DJI Air 3 | `60:60:1F` | All 6 |
| DJI Mavic 3 Classic | `34:D2:62` | Wi-Fi + NAN + VendorIE + BLE |
| DJI Mini 2 SE | `60:60:1F` | Wi-Fi OUI + SSID + BLE |
| DJI Phantom 4 Pro | `60:60:1F` | Wi-Fi OUI + SSID |
| DJI FPV | `48:1C:B9` | Wi-Fi + BLE |
| Parrot ANAFI | `90:3A:E6` | Wi-Fi + VendorIE + BLE |
| Parrot Bebop 2 | `90:3A:E6` | Wi-Fi OUI + SSID |
| Autel EVO II Pro | `2C:F4:32` | SSID + NAN + VendorIE + BLE |
| Autel EVO Nano+ | `2C:F4:32` | SSID + NAN + BLE |
| Skydio 2+ | `6C:29:95` | SSID + NAN + VendorIE + BLE |
| DJI / Ryze Tello | `60:60:1F` | Wi-Fi OUI + SSID |

Signal strength ramps from a weak RSSI up to a peak and back down, simulating a drone flying past.

---

## Build & Flash

### Prerequisites

- [ESP-IDF v6.0.1](https://github.com/espressif/esp-idf/tree/v6.0.1)
- Python 3.10+

```bash
git clone --recursive https://github.com/espressif/esp-idf.git ~/esp/esp-idf
cd ~/esp/esp-idf && git checkout v6.0.1
./install.sh esp32c5,esp32c3
source export.sh
```

### Flash the Detector (ESP32-C5)

```bash
cd drone_detector
idf.py set-target esp32c5
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

### Flash the Simulator (ESP32-C3)

```bash
cd drone_simulator
idf.py set-target esp32c3
idf.py build
idf.py -p /dev/ttyACM1 -b 460800 flash
```

> Adjust port names (`/dev/ttyACM0`, `/dev/ttyACM1`) to match your system. Run `ls /dev/ttyACM* /dev/ttyUSB*` to find them.

---

## Serial Monitor (Detector)

```bash
screen /dev/ttyACM0 115200
# or
cd drone_detector && idf.py -p /dev/ttyACM0 monitor
```

Example output when simulator is running nearby:

```
╔══════════════════════════════════════════════════════╗
║  *** DRONE DETECTED ***
║  Vendor  : DJI
║  Model   : DJI Mini 4 Pro
║  MAC     : 34:D2:62:A1:B2:C3
║  SSID    : DJI-Mini-4P-B2C3
║  RSSI    : -61 dBm  →  MEDIUM
║  Methods : WiFi-OUI  WiFi-SSID  NAN  VendorIE  BLE-RemoteID
║  Channel : 6
╚══════════════════════════════════════════════════════╝
```

---

## Project Structure

```
Drone-Detector-V2/
├── drone_detector/          # Detector firmware (ESP32-C5)
│   ├── main/
│   │   ├── main.c           # Detection engine — all 6 layers
│   │   └── drone_oui.h      # OUI table (8 entries) + SSID table (59 patterns)
│   ├── sdkconfig.defaults
│   └── CMakeLists.txt
├── drone_simulator/         # Simulator firmware (ESP32-C3)
│   ├── main/
│   │   ├── main.c           # Transmission engine
│   │   └── drone_models.h   # 11 drone model definitions with RF signatures
│   ├── sdkconfig.defaults
│   └── CMakeLists.txt
└── project.txt              # Plain-English project description and next steps
```

---

## Legal Notice

This project is for **educational and research purposes only**.  
The simulator transmits low-power RF frames — it is the user's responsibility to ensure use complies with local regulations (FCC Part 15, ETSI, etc.).  
Do not use this hardware to interfere with real drone operations or air traffic control systems.

---

## License

MIT License — see [LICENSE](LICENSE)
