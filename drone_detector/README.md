# ESP32-C5 Consumer Drone Detector

Inspired by [lab44.us/drone](https://lab44.us/drone) — multi-layer consumer drone detection firmware for the **ESP32-C5-DevKitC-1 v1.2**.

---

## What it does

Runs three independent detection layers simultaneously:

| Layer | Method | What it catches |
|-------|--------|----------------|
| **WiFi OUI** | Promiscuous beacon/probe frames, source MAC OUI lookup | DJI (two confirmed IEEE blocks), Parrot (3 confirmed) |
| **WiFi SSID** | Fingerprint match on broadcast SSID | DJI-*, MAVIC-*, TELLO-*, ANAFI, Bebop, EVO-, Skydio, … |
| **BLE Remote ID** | Passive BLE scan for service UUID `0xFFFA` (ASTM F3411-22A) | Any FAA-compliant Remote ID beacon (mandatory for drones >250 g since Sep 2023) |

All channels 1–13 (2.4 GHz) are scanned in an interleaved hop pattern.  
WiFi and BLE coexist using the ESP32-C5's built-in coexistence scheduler.

---

## NeoPixel (GPIO 27)

| State | Colour |
|-------|--------|
| Scanning (no drone) | Slow **blue** blink (0.5 Hz) |
| Drone far — RSSI < −75 dBm | Solid **RED** |
| Drone medium — RSSI −75 to −65 dBm | **ORANGE** |
| Drone close — RSSI −65 to −50 dBm | **YELLOW-GREEN** |
| Drone very close — RSSI > −50 dBm | **GREEN** |

Signal disappears after 5 s without a re-detection → returns to blue blink.

---

## Serial output (115200 baud)

Connect `/dev/ttyACM0` in any terminal (`idf.py monitor`, `minicom`, `screen`):

```
╔══════════════════════════════════════════════════════╗
║  *** DRONE DETECTED ***
║  Vendor  : DJI
║  SSID    : DJI-PT123
║  MAC     : 60:60:1F:AA:BB:CC
║  RSSI    : -58 dBm  →  CLOSE      (yellow-green)
║  Method  : WiFi-OUI
║  Channel : 6
╚══════════════════════════════════════════════════════╝
```

---

## Hardware

- **ESP32-C5-DevKitC-1 v1.2** (Espressif)  
- Onboard WS2812B RGB LED: **GPIO 27**  
  *(Verify against [Espressif DevKit docs](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32c5/esp32-c5-devkitc-1/user_guide.html) if your board revision differs — change `NEO_GPIO` in `main/main.c`)*

---

## Build requirements

- **ESP-IDF v5.3 or newer** (v6.1 recommended — your machine already has the v6.1 toolchain)
- Python 3.8+
- Git

### Step 1 — Install ESP-IDF v6.1 (one-time)

```bash
cd ~/PURDUE/drone_detector
./setup_idf.sh
```

This clones `https://github.com/espressif/esp-idf.git` at tag `v6.1` into `~/esp/esp-idf` and installs the `esp32c5` tools.  Expect ~10-15 minutes on first run.

### Step 2 — Activate IDF environment

```bash
source ~/esp/esp-idf/export.sh
```

Add this to `~/.bashrc` to avoid repeating it:

```bash
echo 'source ~/esp/esp-idf/export.sh' >> ~/.bashrc
```

### Step 3 — Build and flash

```bash
cd ~/PURDUE/drone_detector
chmod +x build_and_flash.sh
./build_and_flash.sh
```

Or manually:

```bash
idf.py set-target esp32c5
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

---

## Project structure

```
drone_detector/
├── CMakeLists.txt          # root CMake (IDF project)
├── sdkconfig.defaults      # Kconfig overrides (WiFi + NimBLE + coex)
├── setup_idf.sh            # one-time IDF v6.1 installer
├── build_and_flash.sh      # build → flash → monitor helper
└── main/
    ├── CMakeLists.txt      # component definition + REQUIRES
    ├── idf_component.yml   # managed dependency: espressif/led_strip ≥2.5
    ├── drone_oui.h         # OUI & SSID fingerprint database (inline lookups)
    └── main.c              # all application logic
```

---

## OUI database

OUI prefixes in `drone_oui.h` sourced from:
- **[IEEE Standards Registration Authority](https://standards-oui.ieee.org/)** (oui.txt public listing)
- Verified via [macvendors.com](https://macvendors.com) API (marked ✓ in source)

### Currently tracked vendors

| Vendor | IEEE-confirmed OUIs | Notes |
|--------|---------------------|-------|
| **DJI** | `60:60:1F`, `34:D2:62` | + 1 widely-reported entry |
| **Parrot** | `90:3A:E6`, `A0:14:3D`, `00:26:7E` | + 2 legacy entries |

To add a new OUI, append an entry to `DRONE_OUI_TABLE` in `main/drone_oui.h`:

```c
{ {0xAA, 0xBB, 0xCC}, "VendorName", "source / confidence note" },
```

To look up a MAC OUI:
```bash
curl https://api.macvendors.com/AA:BB:CC
```

---

## Tuning

All thresholds are `#define` constants at the top of `main/main.c`:

| Constant | Default | Description |
|----------|---------|-------------|
| `NEO_GPIO` | `27` | NeoPixel GPIO pin |
| `LED_BRIGHTNESS` | `60` | 0–255; keep low to avoid USB power issues |
| `RSSI_VERY_CLOSE` | `-50` | dBm → GREEN |
| `RSSI_CLOSE` | `-65` | dBm → YELLOW |
| `RSSI_MEDIUM` | `-75` | dBm → ORANGE; below → RED |
| `DRONE_TIMEOUT_MS` | `5000` | ms before "lost" state |
| `CHAN_DWELL_MS` | `200` | ms per channel during hop |
| `CHAN_LOCK_MS` | `3000` | ms to stay locked on drone's channel |

---

## Legal notice

> Passive RF listening (receive-only, no transmission) is generally lawful in the United States under FCC Part 15.  
> **Active RF jamming is a federal crime** under 47 U.S.C. § 333.  
> Only monitor airspace and wireless networks you own or have explicit written permission to monitor.  
> Consult FAA Part 107 and your local regulations before deploying.
