/**
 * drone_models.h  —  Consumer drone simulator model database
 *
 * Each entry describes a real consumer drone and the RF signatures it emits:
 *   - WiFi source MAC (using the real vendor OUI prefix + a device suffix)
 *   - WiFi SSID (exact real-world format with [XXXX] placeholder = last 4 MAC hex)
 *   - Whether this model uses Wi-Fi NAN Remote ID (FAA-mandated, >250g drones)
 *   - Whether this model embeds vendor IE OUI FA:0B:BC in its beacons
 *   - BLE Remote ID via UUID 0xFFFA (ASTM F3411-22A)
 *   - A simulated RSSI sweep range (min..max dBm) to emulate approach/fly-by
 *
 * TX techniques used by the simulator for each model:
 *   Layer 0 — WiFi beacon with model SSID  (detector Layer 2 / WiFi-SSID)
 *   Layer 1 — WiFi source MAC from real OUI (detector Layer 1 / WiFi-OUI)
 *   Layer 2 — WiFi NAN action frame to 51:6F:9A:01:00:00 (detector Layer 0)
 *   Layer 3 — WiFi beacon with vendor IE OUI FA:0B:BC (detector Layer 3)
 *   Layer 4 — BLE advertisement with UUID 0xFFFA in service data AD type
 *              (detector Layer 4 / BLE-RemoteID)
 *
 * ESP32-C3 Super Mini constraints:
 *   - Single-band 2.4 GHz (no 5 GHz)
 *   - WiFi and BLE cannot transmit simultaneously: interleaved via coexistence
 *   - No hardware NeoPixel — onboard LED controls are on GPIO 8 (active-low)
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ── Simulation emission flags ────────────────────────────────────────── */
#define SIM_WIFI_OUI      (1 << 0)   /* TX beacon with drone OUI source MAC   */
#define SIM_WIFI_SSID     (1 << 1)   /* TX beacon with model SSID             */
#define SIM_WIFI_NAN      (1 << 2)   /* TX NAN action frame to multicast MAC  */
#define SIM_WIFI_VENDOR_IE (1 << 3)  /* TX beacon with FA:0B:BC vendor IE     */
#define SIM_BLE_REMOTEID  (1 << 4)   /* TX BLE adv with UUID 0xFFFA service   */
#define SIM_BLE_OUI       (1 << 5)   /* TX BLE public addr with drone OUI     */

typedef struct {
    const char *model_name;      /* Human-readable model name                 */
    const char *vendor;          /* Vendor string e.g. "DJI"                  */
    uint8_t     oui[3];          /* Real OUI prefix for this model             */
    const char *ssid_fmt;        /* SSID format: %04X = last 4 nibbles of MAC  */
    uint8_t     wifi_channel;    /* Primary 2.4 GHz WiFi channel to use        */
    uint32_t    emit_flags;      /* Bitmask of SIM_* flags above               */
    int8_t      rssi_start;      /* Starting simulated RSSI (dBm) — far away  */
    int8_t      rssi_peak;       /* Peak RSSI (dBm) — drone overhead          */
    uint16_t    dwell_ms;        /* How long to hold each RSSI step (ms)       */
} drone_model_t;

/* ── Remote ID BLE service data payload (ASTM F3411-22A minimal frame) ── */
/* UUID: 0xFFFA (little-endian in BLE ADV = FA FF)                        */
/* We send a 25-byte Basic ID message (Message Type 0x00):                */
/*   Byte 0: protocol version (0x02 = F3411-22A)                          */
/*   Bytes 1-24: ID Type (0x01 = Serial Number) + 20-char ASCII UA ID    */
#define REMOTEID_UUID_LE_HI  0xFF
#define REMOTEID_UUID_LE_LO  0xFA

/* ── Drone model roster ──────────────────────────────────────────────── */
static const drone_model_t DRONE_MODELS[] = {

    /* ── DJI Mini 4 Pro  (OUI 34:D2:62) ───────────────────────────── */
    /* >250g, FAA Remote ID mandated. Broadcasts NAN + beacon + BLE.    */
    {
        .model_name   = "DJI Mini 4 Pro",
        .vendor       = "DJI",
        .oui          = {0x34, 0xD2, 0x62},
        .ssid_fmt     = "DJI MINI 4-%04X",
        .wifi_channel = 6,
        .emit_flags   = SIM_WIFI_OUI | SIM_WIFI_SSID | SIM_WIFI_NAN |
                        SIM_WIFI_VENDOR_IE | SIM_BLE_REMOTEID | SIM_BLE_OUI,
        .rssi_start   = -82,
        .rssi_peak    = -48,
        .dwell_ms     = 400,
    },

    /* ── DJI Air 3  (OUI 60:60:1F) ────────────────────────────────── */
    /* High-end prosumer. Full Remote ID stack.                          */
    {
        .model_name   = "DJI Air 3",
        .vendor       = "DJI",
        .oui          = {0x60, 0x60, 0x1F},
        .ssid_fmt     = "DJI AIR 3-%04X",
        .wifi_channel = 1,
        .emit_flags   = SIM_WIFI_OUI | SIM_WIFI_SSID | SIM_WIFI_NAN |
                        SIM_WIFI_VENDOR_IE | SIM_BLE_REMOTEID | SIM_BLE_OUI,
        .rssi_start   = -80,
        .rssi_peak    = -45,
        .dwell_ms     = 350,
    },

    /* ── DJI Mavic 3 Classic  (OUI 34:D2:62) ──────────────────────── */
    {
        .model_name   = "DJI Mavic 3 Classic",
        .vendor       = "DJI",
        .oui          = {0x34, 0xD2, 0x62},
        .ssid_fmt     = "MAVIC3-%04X",
        .wifi_channel = 11,
        .emit_flags   = SIM_WIFI_OUI | SIM_WIFI_SSID | SIM_WIFI_NAN |
                        SIM_WIFI_VENDOR_IE | SIM_BLE_REMOTEID,
        .rssi_start   = -78,
        .rssi_peak    = -50,
        .dwell_ms     = 450,
    },

    /* ── DJI Mini 2 SE  (OUI 60:60:1F) ────────────────────────────── */
    /* 249g — just under 250g limit, no mandatory Remote ID,            */
    /* but DJI still includes it in firmware. WiFi + BLE only.          */
    {
        .model_name   = "DJI Mini 2 SE",
        .vendor       = "DJI",
        .oui          = {0x60, 0x60, 0x1F},
        .ssid_fmt     = "DJI MINI 2-%04X",
        .wifi_channel = 6,
        .emit_flags   = SIM_WIFI_OUI | SIM_WIFI_SSID | SIM_BLE_REMOTEID,
        .rssi_start   = -85,
        .rssi_peak    = -52,
        .dwell_ms     = 500,
    },

    /* ── DJI Phantom 4 Pro  (OUI 60:60:1F) ────────────────────────── */
    /* Legacy platform, WiFi OUI + SSID only (older FW, no NAN).       */
    {
        .model_name   = "DJI Phantom 4 Pro",
        .vendor       = "DJI",
        .oui          = {0x60, 0x60, 0x1F},
        .ssid_fmt     = "PHANTOM4-%04X",
        .wifi_channel = 1,
        .emit_flags   = SIM_WIFI_OUI | SIM_WIFI_SSID,
        .rssi_start   = -75,
        .rssi_peak    = -42,
        .dwell_ms     = 300,
    },

    /* ── DJI FPV  (OUI 48:1C:B9) ───────────────────────────────────── */
    {
        .model_name   = "DJI FPV",
        .vendor       = "DJI",
        .oui          = {0x48, 0x1C, 0xB9},
        .ssid_fmt     = "DJI FPV-%04X",
        .wifi_channel = 11,
        .emit_flags   = SIM_WIFI_OUI | SIM_WIFI_SSID | SIM_BLE_REMOTEID,
        .rssi_start   = -80,
        .rssi_peak    = -48,
        .dwell_ms     = 350,
    },

    /* ── Parrot ANAFI  (OUI 90:3A:E6) ─────────────────────────────── */
    /* Parrot uses its own vendor IE OUI 90:3A:E6 in Remote ID beacons. */
    {
        .model_name   = "Parrot ANAFI",
        .vendor       = "Parrot",
        .oui          = {0x90, 0x3A, 0xE6},
        .ssid_fmt     = "ANAFI-%04X",
        .wifi_channel = 6,
        .emit_flags   = SIM_WIFI_OUI | SIM_WIFI_SSID | SIM_WIFI_VENDOR_IE |
                        SIM_BLE_REMOTEID,
        .rssi_start   = -82,
        .rssi_peak    = -51,
        .dwell_ms     = 420,
    },

    /* ── Parrot Bebop 2  (OUI 90:3A:E6) ────────────────────────────── */
    /* Legacy platform — WiFi OUI + SSID only.                           */
    {
        .model_name   = "Parrot Bebop 2",
        .vendor       = "Parrot",
        .oui          = {0x90, 0x3A, 0xE6},
        .ssid_fmt     = "Bebop2-%04X",
        .wifi_channel = 1,
        .emit_flags   = SIM_WIFI_OUI | SIM_WIFI_SSID,
        .rssi_start   = -80,
        .rssi_peak    = -53,
        .dwell_ms     = 400,
    },

    /* ── Autel EVO II Pro  (no confirmed OUI — SSID + Remote ID only) */
    /* OUI from commodity chip; detector catches via SSID + BLE UUID.  */
    {
        .model_name   = "Autel EVO II Pro",
        .vendor       = "Autel",
        .oui          = {0x2C, 0xF4, 0x32},   /* placeholder; Autel OUI TBD */
        .ssid_fmt     = "EVO II-%04X",
        .wifi_channel = 11,
        .emit_flags   = SIM_WIFI_SSID | SIM_WIFI_NAN | SIM_WIFI_VENDOR_IE |
                        SIM_BLE_REMOTEID,
        .rssi_start   = -79,
        .rssi_peak    = -49,
        .dwell_ms     = 420,
    },

    /* ── Autel EVO Nano+  (SSID + Remote ID; >250g FAA mandated) ───── */
    {
        .model_name   = "Autel EVO Nano+",
        .vendor       = "Autel",
        .oui          = {0x2C, 0xF4, 0x32},
        .ssid_fmt     = "EVO NANO-%04X",
        .wifi_channel = 6,
        .emit_flags   = SIM_WIFI_SSID | SIM_WIFI_NAN | SIM_BLE_REMOTEID,
        .rssi_start   = -83,
        .rssi_peak    = -54,
        .dwell_ms     = 500,
    },

    /* ── Skydio 2+  (SSID + NAN Remote ID + BLE UUID) ─────────────── */
    /* Skydio uses commodity Qualcomm chip; OUI not in public list.     */
    {
        .model_name   = "Skydio 2+",
        .vendor       = "Skydio",
        .oui          = {0x6C, 0x29, 0x95},   /* placeholder commodity OUI   */
        .ssid_fmt     = "Skydio 2-%04X",
        .wifi_channel = 1,
        .emit_flags   = SIM_WIFI_SSID | SIM_WIFI_NAN | SIM_WIFI_VENDOR_IE |
                        SIM_BLE_REMOTEID,
        .rssi_start   = -77,
        .rssi_peak    = -47,
        .dwell_ms     = 380,
    },

    /* ── DJI/Ryze Tello  (OUI 60:60:1F) ────────────────────────────── */
    /* Toy drone; no Remote ID. WiFi OUI + SSID only.                   */
    {
        .model_name   = "DJI/Ryze Tello",
        .vendor       = "DJI",
        .oui          = {0x60, 0x60, 0x1F},
        .ssid_fmt     = "TELLO-%04X",
        .wifi_channel = 11,
        .emit_flags   = SIM_WIFI_OUI | SIM_WIFI_SSID,
        .rssi_start   = -70,
        .rssi_peak    = -38,
        .dwell_ms     = 300,
    },
};

#define DRONE_MODEL_COUNT  ((int)(sizeof(DRONE_MODELS) / sizeof(DRONE_MODELS[0])))
