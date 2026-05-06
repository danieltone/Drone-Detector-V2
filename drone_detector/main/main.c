/**
 * main.c  —  ESP32-C5 Consumer Drone Detector
 * ============================================================
 * Inspired by: lab44.us/drone  (WiFi OUI + Remote ID hybrid detection)
 *
 * Detection layers:
 *   1. WiFi promiscuous — OUI match on beacon / probe-response source MAC
 *   2. WiFi promiscuous — SSID fingerprint in beacon / probe-response
 *   3. BLE passive scan  — OpenDroneID Remote ID (UUID 0xFFFA, ASTM F3411-22A)
 *   4. BLE passive scan  — OUI match on non-randomised BLE public address
 *
 * NeoPixel feedback (WS2812B, GPIO 27 on ESP32-C5-DevKitC-1 v1.2):
 *   - No drone   → slow blue blink (500 ms)
 *   - FAR        (RSSI < -75 dBm)  → RED
 *   - MEDIUM     (RSSI -75 to -60) → ORANGE
 *   - CLOSE      (RSSI -60 to -50) → YELLOW-GREEN
 *   - VERY CLOSE (RSSI > -50 dBm)  → GREEN
 *   Drone signal times out after 5 s without a new detection.
 *
 * Serial output: 115200 baud — all detections printed with MAC / RSSI / method.
 *
 * Hardware: ESP32-C5-DevKitC-1 v1.2
 * Build:    ESP-IDF v5.3+ (v6.1 recommended — toolchain already installed)
 *           idf.py set-target esp32c5
 *           idf.py build flash monitor -p /dev/ttyACM0
 *
 * ⚠ Legal:  Passive RF listening is generally lawful.
 *           Active jamming is a federal crime (FCC / FAA regulations).
 *           Only monitor airspace and networks you own or have permission for.
 * ============================================================
 */

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_netif.h"

/* NimBLE BLE stack */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/util/util.h"

/* LED strip (NeoPixel / WS2812B) — from espressif/led_strip managed component */
#include "led_strip.h"

/* Project OUI / SSID fingerprint database */
#include "drone_oui.h"

/* ============================================================
 * Hardware  —  adjust if your board revision differs
 * Espressif ESP32-C5-DevKitC-1 v1.2 schematic: RGB LED on GPIO 27
 * https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32c5/
 * ============================================================ */
#define NEO_GPIO          27     /* WS2812B data pin */
#define NEO_NUM_LEDS       1     /* only one onboard LED */
#define NEO_RMT_RES_HZ   10000000U   /* 10 MHz RMT resolution */

/* ============================================================
 * Tuning parameters
 * ============================================================ */
/* RSSI thresholds (dBm) for proximity colour mapping */
#define RSSI_VERY_CLOSE  (-50)
#define RSSI_CLOSE       (-65)
#define RSSI_MEDIUM      (-75)
/* Below RSSI_MEDIUM → RED (far away) */

/* How long to keep a drone "seen" before fading back to scan-blink */
#define DRONE_TIMEOUT_MS    5000

/* Rate-limit duplicate serial prints (ms) — avoids flooding the console */
#define PRINT_DEBOUNCE_MS   2000

/* Channel-hop dwell time (ms) — how long we sit on each 2.4 GHz channel */
#define CHAN_DWELL_MS        200

/* After a detection, lock onto that channel for this long (ms) */
#define CHAN_LOCK_MS        3000

/* LED brightness scale 0-255; keep low to avoid power/heat on DevKit */
#define LED_BRIGHTNESS       60

static const char *TAG = "DRONE";

/* ============================================================
 * Shared drone detection state  (mutex-protected)
 * ============================================================ */
typedef struct {
    bool     active;               /* drone currently visible?          */
    int8_t   rssi;                 /* strongest RSSI seen this window   */
    char     vendor[64];           /* e.g. "DJI", "Parrot"              */
    char     ssid[33];             /* SSID if extracted, else empty      */
    uint8_t  mac[6];               /* source MAC (network byte order)   */
    char     method[32];           /* "WiFi-OUI" / "WiFi-SSID" / "BLE-RemoteID" */
    int8_t   channel;              /* WiFi channel or -1 for BLE        */
    int64_t  last_seen_us;         /* esp_timer_get_time() at last hit   */
    int64_t  last_printed_us;      /* last time we printed this detection */
} drone_state_t;

static drone_state_t  g_drone;
static SemaphoreHandle_t g_mutex;

/* NeoPixel handle */
static led_strip_handle_t g_led = NULL;

/* ============================================================
 * NeoPixel helpers
 * ============================================================ */
static void neo_init(void)
{
    led_strip_config_t strip_cfg = {
        .strip_gpio_num             = NEO_GPIO,
        .max_leds                   = NEO_NUM_LEDS,
        .led_model                  = LED_MODEL_WS2812,
        .color_component_format     = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags.invert_out           = false,
    };
    led_strip_rmt_config_t rmt_cfg = {
        .clk_src        = RMT_CLK_SRC_DEFAULT,
        .resolution_hz  = NEO_RMT_RES_HZ,
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &g_led));
    led_strip_clear(g_led);
}

static void neo_set(uint8_t r, uint8_t g, uint8_t b)
{
    if (!g_led) return;
    /* Scale each channel by LED_BRIGHTNESS/255 */
    r = (uint8_t)((uint32_t)r * LED_BRIGHTNESS / 255);
    g = (uint8_t)((uint32_t)g * LED_BRIGHTNESS / 255);
    b = (uint8_t)((uint32_t)b * LED_BRIGHTNESS / 255);
    led_strip_set_pixel(g_led, 0, r, g, b);
    led_strip_refresh(g_led);
}

/**
 * Map RSSI (dBm) to an RGB colour.
 *
 *  ≥ -50   →  full GREEN     (0xFF, 0x00, 0x00)  — very close
 *  -65     →  YELLOW         (0xFF, 0xFF, 0x00)  — close
 *  -75     →  ORANGE         (0xFF, 0x80, 0x00)  — medium
 *  < -75   →  full RED       (0xFF, 0x00, 0x00)  — far
 *
 * Linear interpolation between anchor points.
 */
static void rssi_to_rgb(int8_t rssi, uint8_t *r, uint8_t *g, uint8_t *b)
{
    *b = 0;

    if (rssi >= RSSI_VERY_CLOSE) {
        /* GREEN */
        *r = 0; *g = 255;

    } else if (rssi >= RSSI_CLOSE) {
        /* Interpolate YELLOW → GREEN  (RSSI_CLOSE=-65 .. RSSI_VERY_CLOSE=-50) */
        float t = (float)(rssi - RSSI_CLOSE) / (float)(RSSI_VERY_CLOSE - RSSI_CLOSE);
        *r = (uint8_t)(255 * (1.0f - t));
        *g = 255;

    } else if (rssi >= RSSI_MEDIUM) {
        /* Interpolate RED/ORANGE → YELLOW  (RSSI_MEDIUM=-75 .. RSSI_CLOSE=-65) */
        float t = (float)(rssi - RSSI_MEDIUM) / (float)(RSSI_CLOSE - RSSI_MEDIUM);
        *r = 255;
        *g = (uint8_t)(128 * t);  /* 0=orange, 128=yellow */

    } else {
        /* RED — far */
        *r = 255; *g = 0;
    }
}

/* NeoPixel task: runs at 2 Hz, reads global state under mutex */
static void neo_task(void *arg)
{
    bool blink = false;
    while (1) {
        int64_t now = esp_timer_get_time();
        bool timed_out = false;
        bool drone_active = false;
        int8_t rssi = -100;

        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(40)) == pdTRUE) {
            if (g_drone.active) {
                timed_out = (now - g_drone.last_seen_us) > ((int64_t)DRONE_TIMEOUT_MS * 1000LL);
                if (timed_out) {
                    g_drone.active  = false;
                    g_drone.channel = -1;
                    printf("\n[DRONE] Signal lost — resuming scan\n\n");
                } else {
                    drone_active = true;
                    rssi = g_drone.rssi;
                }
            }
            xSemaphoreGive(g_mutex);
        }

        if (drone_active) {
            uint8_t r, gc, b;
            rssi_to_rgb(rssi, &r, &gc, &b);
            neo_set(r, gc, b);
        } else {
            /* Blink blue while scanning */
            blink = !blink;
            neo_set(0, 0, blink ? 255 : 0);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* ============================================================
 * Detection state update  (called from WiFi + BLE callbacks)
 * ============================================================ */
static void detection_update(const uint8_t *mac,
                             const char    *vendor,
                             const char    *ssid,
                             int8_t         rssi,
                             const char    *method,
                             int8_t         channel)
{
    int64_t now = esp_timer_get_time();

    if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(10)) != pdTRUE) return;

    bool was_active = g_drone.active;

    /* Always update if signal is stronger or this is a new sighting */
    if (!g_drone.active || rssi > g_drone.rssi) {
        g_drone.rssi    = rssi;
        g_drone.channel = channel;
        memcpy(g_drone.mac, mac, 6);
        strncpy(g_drone.vendor, vendor, sizeof(g_drone.vendor) - 1);
        g_drone.vendor[sizeof(g_drone.vendor) - 1] = '\0';
        if (ssid && ssid[0]) {
            strncpy(g_drone.ssid, ssid, sizeof(g_drone.ssid) - 1);
            g_drone.ssid[sizeof(g_drone.ssid) - 1] = '\0';
        }
        strncpy(g_drone.method, method, sizeof(g_drone.method) - 1);
        g_drone.method[sizeof(g_drone.method) - 1] = '\0';
    }
    g_drone.active       = true;
    g_drone.last_seen_us = now;

    /* Rate-limited serial output */
    bool should_print = !was_active
        || (now - g_drone.last_printed_us) > ((int64_t)PRINT_DEBOUNCE_MS * 1000LL);

    if (should_print) {
        g_drone.last_printed_us = now;

        const char *proximity =
            rssi >= RSSI_VERY_CLOSE ? "VERY CLOSE (green)" :
            rssi >= RSSI_CLOSE      ? "CLOSE      (yellow-green)" :
            rssi >= RSSI_MEDIUM     ? "MEDIUM     (orange)" :
                                      "FAR        (red)";

        printf("\n╔══════════════════════════════════════════════════════╗\n");
        printf("║  *** DRONE DETECTED ***\n");
        printf("║  Vendor  : %s\n", vendor);
        if (ssid && ssid[0])
            printf("║  SSID    : %s\n", ssid);
        printf("║  MAC     : %02X:%02X:%02X:%02X:%02X:%02X\n",
               mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        printf("║  RSSI    : %d dBm  →  %s\n", rssi, proximity);
        printf("║  Method  : %s\n", method);
        if (channel > 0)
            printf("║  Channel : %d\n", (int)channel);
        printf("╚══════════════════════════════════════════════════════╝\n");
    }

    xSemaphoreGive(g_mutex);
}

/* ============================================================
 * WiFi promiscuous-mode scanning
 * ============================================================ */

/* Minimal 802.11 management frame header */
typedef struct {
    uint16_t frame_ctrl;
    uint16_t duration_id;
    uint8_t  addr1[6];   /* destination */
    uint8_t  addr2[6];   /* source (AP BSSID in beacon/probe-resp) */
    uint8_t  addr3[6];   /* BSSID / SA */
    uint16_t seq_ctrl;
} __attribute__((packed)) mgmt_hdr_t;

/* Fixed-field byte lengths after the management header:
 *   Beacon       (sub-type 8) : 12 bytes (Timestamp + Interval + Capability)
 *   Probe Resp   (sub-type 5) : 12 bytes (same)
 *   Probe Req    (sub-type 4) :  0 bytes (IEs start immediately)
 */
#define BEACON_FIXED_PARAMS  12

/*
 * Wi-Fi Remote ID detection constants (ASTM F3411-22A / FAA mandate)
 *
 * 1. NAN (Neighbor Awareness Networking) action frame destination multicast MAC.
 *    Any drone broadcasting Remote ID over Wi-Fi NAN will send to this address.
 *    Ref: sxjack/uav_electronic_ids id_scanner.ino, Espressif OpenDroneID examples.
 */
static const uint8_t NAN_DEST_MAC[6] = {0x51, 0x6F, 0x9A, 0x01, 0x00, 0x00};

/*
 * 2. Vendor-specific IE OUIs inside Wi-Fi beacon frames (tag 0xDD):
 *    FA:0B:BC  — ASTM F3411 / OpenDroneID standard vendor OUI
 *    90:3A:E6  — Parrot SA vendor OUI (also used in their Remote ID beacon IE)
 *    Detecting either means the beacon IS a Remote ID beacon regardless of the
 *    source MAC OUI (the drone may use a commodity Qualcomm/Mediatek radio chip).
 */
static const uint8_t ODID_VENDOR_OUI[3]   = {0xFA, 0x0B, 0xBC};
static const uint8_t PARROT_VENDOR_OUI[3] = {0x90, 0x3A, 0xE6};

static void wifi_promisc_cb(void *buf, wifi_promiscuous_pkt_type_t type)
{
    if (type != WIFI_PKT_MGMT) return;

    const wifi_promiscuous_pkt_t *pkt  = (const wifi_promiscuous_pkt_t *)buf;
    const uint8_t                *data = pkt->payload;
    uint16_t                      dlen = pkt->rx_ctrl.sig_len;
    int8_t                        rssi = (int8_t)pkt->rx_ctrl.rssi;
    int8_t                        ch   = (int8_t)pkt->rx_ctrl.channel;

    if (dlen < (uint16_t)sizeof(mgmt_hdr_t)) return;

    const mgmt_hdr_t *hdr     = (const mgmt_hdr_t *)data;
    uint8_t           ftype    = (hdr->frame_ctrl & 0x000C) >> 2;
    uint8_t           fsubtype = (hdr->frame_ctrl & 0x00F0) >> 4;

    /* Only handle management frames (type == 0) */
    if (ftype != 0) return;

    const uint8_t *src = hdr->addr2;   /* transmitter (source) MAC */

    /* ── Layer 0: Wi-Fi NAN Remote ID frame ──────────────────────────────
     * FAA Remote ID mandates drones >250 g transmit ASTM F3411-22A over
     * Wi-Fi NAN (Neighbor Awareness Networking).  The frame is an action
     * frame (subtype 13 = 0x0D) addressed to the NAN multicast MAC.
     * This catches ANY compliant drone regardless of brand OUI:
     *   DJI Mini 3 Pro, DJI Air 3, Skydio 2/X2, Autel EVO II, ...        */
    if (fsubtype == 0x0D) {        /* action frame */
        if (memcmp(hdr->addr1, NAN_DEST_MAC, 6) == 0) {
            detection_update(src,
                             "FAA Remote ID (Wi-Fi NAN, ASTM F3411-22A)",
                             "[Wi-Fi NAN Remote ID]",
                             rssi, "WiFi-NAN-RemoteID", ch);
            return;
        }
    }

    /* ── Layer 1: OUI match on source MAC ───────────────────────────────*/
    const char *vendor = drone_oui_lookup(src);
    if (vendor) {
        char ssid[33] = {0};
        if (fsubtype == 8 || fsubtype == 5) {
            int ie_offset = (int)sizeof(mgmt_hdr_t) + BEACON_FIXED_PARAMS;
            int remaining = (int)dlen - ie_offset;
            if (remaining > 2) {
                const uint8_t *ie = data + ie_offset;
                if (ie[0] == 0x00) {
                    uint8_t slen = ie[1];
                    if (slen > 0 && slen <= 32) {
                        memcpy(ssid, ie + 2, slen);
                        ssid[slen] = '\0';
                    }
                }
            }
        }
        detection_update(src, vendor, ssid, rssi, "WiFi-OUI", ch);
        return;
    }

    /* ── Layers 2 & 3: beacon / probe-response deep parsing ─────────────
     * Walk every Information Element once; extract SSID, and look for the
     * vendor-specific Remote ID IE (tag 0xDD).  We walk all IEs in a single
     * pass to avoid scanning the frame twice.                              */
    if (fsubtype == 8 || fsubtype == 5) {
        int           ie_offset = (int)sizeof(mgmt_hdr_t) + BEACON_FIXED_PARAMS;
        char          ssid[33]  = {0};
        bool          has_ssid  = false;
        const uint8_t *ie       = data + ie_offset;
        const uint8_t *end      = data + dlen;

        while (ie < end - 1) {
            uint8_t ie_tag = ie[0];
            uint8_t ie_len = ie[1];
            if (ie + 2 + ie_len > end) break;   /* malformed — stop */

            /* ── Layer 2: SSID fingerprint ──────────────────────────── */
            if (ie_tag == 0x00 && !has_ssid && ie_len > 0 && ie_len <= 32) {
                memcpy(ssid, ie + 2, ie_len);
                ssid[ie_len] = '\0';
                has_ssid = true;
            }

            /* ── Layer 3: Remote ID vendor-specific IE ──────────────────
             * Vendor OUI FA:0B:BC = OpenDroneID standard (any brand).
             * Vendor OUI 90:3A:E6 = Parrot SA implementation.
             * Either OUI in tag 0xDD means this beacon IS a Remote ID
             * beacon — catches Skydio, Autel, and any brand using a
             * commodity WiFi chip whose source-MAC OUI is not in our table. */
            if (ie_tag == 0xDD && ie_len >= 4) {
                const uint8_t *oui = ie + 2;
                bool is_odid = (oui[0] == ODID_VENDOR_OUI[0] &&
                                oui[1] == ODID_VENDOR_OUI[1] &&
                                oui[2] == ODID_VENDOR_OUI[2]);
                bool is_parrot = (oui[0] == PARROT_VENDOR_OUI[0] &&
                                  oui[1] == PARROT_VENDOR_OUI[1] &&
                                  oui[2] == PARROT_VENDOR_OUI[2]);
                if (is_odid || is_parrot) {
                    const char *rid_vendor = is_parrot
                        ? "Parrot (Remote ID beacon IE)"
                        : "FAA Remote ID (Wi-Fi beacon IE, ASTM F3411-22A)";
                    detection_update(src, rid_vendor,
                                     has_ssid ? ssid : "[Remote ID beacon]",
                                     rssi, "WiFi-VendorIE-RemoteID", ch);
                    return;   /* strong match — done */
                }
            }

            ie += 2 + ie_len;
        }

        /* Reached end of IE walk without a Remote ID IE — try SSID match */
        if (has_ssid) {
            const char *fp = drone_ssid_fingerprint(ssid);
            if (fp) {
                detection_update(src, fp, ssid, rssi, "WiFi-SSID", ch);
            }
        }
    }
}

static void wifi_init(void)
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
    ESP_ERROR_CHECK(esp_wifi_start());

    wifi_promiscuous_filter_t flt = {
        .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT,
    };
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&flt));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(wifi_promisc_cb));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

    ESP_LOGI(TAG, "WiFi promiscuous mode active (management frames)");
}

/* Channel-hop task
 * Cycles 2.4 GHz channels in an interleaved order to reduce dwell time
 * before landing on channels the drone might be using.
 * After a detection the task locks onto the drone's channel.
 */
static void channel_task(void *arg)
{
    /* Interleaved scan order:  non-overlapping first (1,6,11) then fill in */
    static const uint8_t CH24[] = {1, 6, 11, 2, 7, 12, 3, 8, 13, 4, 9, 5, 10};
    const int ch_count = (int)(sizeof(CH24) / sizeof(CH24[0]));
    int idx = 0;

    while (1) {
        int64_t now_us = esp_timer_get_time();
        int8_t lock_ch = -1;

        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (g_drone.active
                && g_drone.channel > 0
                && (now_us - g_drone.last_seen_us) < ((int64_t)CHAN_LOCK_MS * 1000LL))
            {
                lock_ch = g_drone.channel;
            }
            xSemaphoreGive(g_mutex);
        }

        uint8_t next_ch = (lock_ch > 0) ? (uint8_t)lock_ch : CH24[idx++ % ch_count];
        esp_wifi_set_channel(next_ch, WIFI_SECOND_CHAN_NONE);

        vTaskDelay(pdMS_TO_TICKS(CHAN_DWELL_MS));
    }
}

/* ============================================================
 * BLE (NimBLE) — Remote ID + OUI
 * ============================================================ */

/**
 * Parse a raw BLE advertising payload for:
 *   (a) OpenDroneID Remote ID  — AD type 0x16, UUID 0xFFFA (ASTM F3411-22A)
 *   (b) Drone OUI on the BLE public address
 *
 * NimBLE stores the 6-byte BLE address in *little-endian* order:
 *   val[0]=LSB (last displayed byte), val[5]=MSB (first displayed byte).
 * Standard MAC OUI = first 3 octets in display notation = val[5..3].
 */
static void parse_ble_adv(const uint8_t *adv_data, uint8_t adv_len,
                          int8_t rssi, const uint8_t *addr_val,
                          uint8_t addr_type)
{
    /* Reconstruct standard-order MAC from little-endian NimBLE address */
    uint8_t mac[6] = {
        addr_val[5], addr_val[4], addr_val[3],
        addr_val[2], addr_val[1], addr_val[0]
    };

    /* Walk the AD structure */
    uint8_t pos = 0;
    while (pos < adv_len) {
        uint8_t ad_len = adv_data[pos];
        if (ad_len == 0 || (pos + ad_len) >= adv_len) break;

        uint8_t ad_type = adv_data[pos + 1];

        /* ── OpenDroneID: Service Data — 16-bit UUID (AD type 0x16) ─── */
        if (ad_type == 0x16 && ad_len >= 3) {
            uint16_t uuid = (uint16_t)adv_data[pos + 2]
                          | ((uint16_t)adv_data[pos + 3] << 8); /* little-endian */
            if (uuid == 0xFFFA) {
                /* ASTM F3411 / FAA Remote ID beacon — mandatory for drones >250g */
                detection_update(mac,
                                 "FAA Remote ID (ASTM F3411-22A)",
                                 "[Remote ID Beacon]",
                                 rssi, "BLE-RemoteID", -1);
                return; /* found what we came for */
            }
        }

        pos += (uint8_t)(ad_len + 1);
    }

    /* ── OUI match on BLE public address (skip random addresses) ────── */
    if (addr_type == BLE_ADDR_PUBLIC) {
        const char *vendor = drone_oui_lookup(mac);
        if (vendor) {
            detection_update(mac, vendor, "", rssi, "BLE-OUI", -1);
        }
    }
}

static int ble_gap_cb(struct ble_gap_event *event, void *arg)
{
    if (event->type == BLE_GAP_EVENT_DISC) {
        const struct ble_gap_disc_desc *d = &event->disc;
        if (d->data && d->length_data > 0) {
            parse_ble_adv(d->data, (uint8_t)d->length_data,
                          (int8_t)d->rssi,
                          d->addr.val, d->addr.type);
        }
    }
    return 0;
}

static void ble_on_sync(void)
{
    /* Passive BLE scan: no SCAN_REQ sent, so we stay stealthy.
     * filter_duplicates=0 ensures we see repeated beacons (important for
     * drones that broadcast Remote ID every ~1 second per FAA requirement). */
    struct ble_gap_disc_params dp = {
        .itvl              = BLE_GAP_SCAN_FAST_INTERVAL_MAX,
        .window            = BLE_GAP_SCAN_FAST_WINDOW,
        .filter_policy     = BLE_HCI_SCAN_FILT_NO_WL,
        .limited           = 0,
        .passive           = 1,
        .filter_duplicates = 0,
    };

    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &dp,
                          ble_gap_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "BLE disc start error: %d — retrying in 2s", rc);
        vTaskDelay(pdMS_TO_TICKS(2000));
        ble_on_sync(); /* retry */
    } else {
        ESP_LOGI(TAG, "BLE passive scan started (UUID 0xFFFA = Remote ID)");
    }
}

static void ble_on_reset(int reason)
{
    ESP_LOGW(TAG, "BLE host reset (reason=%d)", reason);
}

static void nimble_host_task(void *param)
{
    nimble_port_run();                /* blocks until nimble_port_stop() */
    nimble_port_freertos_deinit();
}

static void ble_init(void)
{
    ESP_ERROR_CHECK(nimble_port_init());
    ble_hs_cfg.sync_cb  = ble_on_sync;
    ble_hs_cfg.reset_cb = ble_on_reset;
    nimble_port_freertos_init(nimble_host_task);
    ESP_LOGI(TAG, "NimBLE host task started");
}

/* ============================================================
 * app_main
 * ============================================================ */
void app_main(void)
{
    /* ── Banner ────────────────────────────────────────────────────── */
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════╗\n");
    printf("║   ESP32-C5 Consumer Drone Detector  v1.0                 ║\n");
    printf("║   lab44.us/drone — WiFi OUI · SSID · BLE Remote ID      ║\n");
    printf("║   NeoPixel: BLUE blink=scan | RED→GREEN = proximity      ║\n");
    printf("╠═══════════════════════════════════════════════════════════╣\n");
    printf("║   Serial: 115200 baud  (/dev/ttyACM0)                    ║\n");
    printf("║   Legal:  Passive RF listening only — no jamming         ║\n");
    printf("╚═══════════════════════════════════════════════════════════╝\n\n");

    /* ── Initialisation ────────────────────────────────────────────── */
    g_mutex = xSemaphoreCreateMutex();
    configASSERT(g_mutex);
    memset(&g_drone, 0, sizeof(g_drone));
    g_drone.channel = -1;

    /* NVS — required by WiFi driver */
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* NeoPixel — solid blue during boot to confirm LED works */
    neo_init();
    neo_set(0, 0, 255);   /* boot indicator: solid blue */
    vTaskDelay(pdMS_TO_TICKS(500));

    /* WiFi — promiscuous on 2.4 GHz */
    wifi_init();

    /* BLE — NimBLE passive scan for Remote ID */
    ble_init();

    /* ── FreeRTOS tasks ─────────────────────────────────────────────── */
    /* NeoPixel task: high priority for responsive LED feedback */
    xTaskCreate(neo_task,     "neo",  3072, NULL, 5, NULL);
    /* Channel-hop task */
    xTaskCreate(channel_task, "chan", 2048, NULL, 3, NULL);

    /* ── Status messages ────────────────────────────────────────────── */
    printf("[DRONE] Scanning started.\n");
    printf("[DRONE] WiFi: channels 1-13 (2.4 GHz), %d ms dwell, interleaved\n",
           CHAN_DWELL_MS);
    printf("[DRONE] BLE:  passive scan, watching for Remote ID UUID 0xFFFA\n");
    printf("[DRONE] OUI table:  %d drone vendor entries\n",  DRONE_OUI_COUNT);
    printf("[DRONE] SSID table: %d fingerprint entries\n",   SSID_FP_COUNT);
    printf("\n[DRONE] Detection layers:\n");
    printf("  Layer 0: WiFi-NAN-RemoteID   — NAN action frame to 51:6F:9A:01:00:00\n");
    printf("  Layer 1: WiFi-OUI            — source MAC in DJI/Parrot OUI table\n");
    printf("  Layer 2: WiFi-SSID           — SSID prefix fingerprint\n");
    printf("  Layer 3: WiFi-VendorIE-RID   — beacon IE tag 0xDD, OUI FA:0B:BC\n");
    printf("  Layer 4: BLE-RemoteID        — BLE adv UUID 0xFFFA (ASTM F3411)\n");
    printf("  Layer 5: BLE-OUI             — BLE public addr OUI (DJI/Parrot)\n");
    printf("\n  → Layers 0+3+4 catch Skydio, Autel, ANY FAA-compliant drone\n");
    printf("\n");
    printf("[DRONE] RSSI colour map:\n");
    printf("         > -50 dBm  → GREEN      (very close)\n");
    printf("        -65 dBm     → YELLOW     (close)\n");
    printf("        -75 dBm     → ORANGE     (medium)\n");
    printf("         < -75 dBm  → RED        (far)\n\n");
    printf("[DRONE] Waiting for drones...\n\n");
}
