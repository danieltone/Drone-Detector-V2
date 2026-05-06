/**
 * main.c  —  ESP32-C3 Consumer Drone Simulator
 * ============================================================
 * Cycles through a roster of real consumer drone models and emits
 * all the RF signatures that the ESP32-C5 Drone Detector looks for:
 *
 *   Layer 0 — WiFi beacon frame (802.11)
 *               • Source MAC built from the real drone OUI + random suffix
 *               • SSID in the exact format the model uses (e.g. "DJI MINI 4-AB12")
 *
 *   Layer 1 — WiFi NAN action frame (FAA Remote ID, ASTM F3411-22A)
 *               • Sent to multicast MAC 51:6F:9A:01:00:00
 *               • Carries a minimal Drone ID Basic-ID message
 *
 *   Layer 2 — WiFi beacon with vendor-specific Remote ID IE
 *               • Information Element tag 0xDD, OUI FA:0B:BC (OpenDroneID)
 *               • (Parrot models use 90:3A:E6 instead)
 *
 *   Layer 3 — BLE advertisement with UUID 0xFFFA service data
 *               • ASTM F3411-22A Remote ID, Basic-ID message type 0x00
 *               • 25-byte UA-ID payload (simulated serial number)
 *
 * Each model is shown for SIM_DWELL_SEC seconds, then the next model begins.
 * The simulated RSSI ramps from rssi_start → rssi_peak → rssi_start to
 * emulate a drone approaching, passing overhead, and flying away.
 *
 * Serial output (115200 baud) logs every emission with method + RSSI.
 *
 * Hardware: ESP32-C3 Super Mini
 *   GPIO 8 = onboard LED (active-LOW; we blink it during scan/emission)
 *   No NeoPixel on this board.
 *
 * ⚠ Legal / RF notice:
 *   This device transmits on 2.4 GHz WiFi and BLE.  These are legitimate
 *   ISM-band transmissions.  Use only in environments where you control
 *   the airspace / RF environment, or in a shielded test enclosure.
 *   Do NOT operate near real drone Remote ID receivers if you wish to
 *   avoid false positives in FAA-mandated systems.
 * ============================================================
 */

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "driver/gpio.h"

/* NimBLE BLE stack */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"

/* Drone model roster */
#include "drone_models.h"

/* ============================================================
 * Hardware
 * ============================================================ */
#define LED_GPIO          8      /* ESP32-C3 Super Mini onboard LED, active LOW  */
#define LED_ON()          gpio_set_level(LED_GPIO, 0)
#define LED_OFF()         gpio_set_level(LED_GPIO, 1)
#define LED_TOGGLE()      do { static bool _s=false; _s=!_s; gpio_set_level(LED_GPIO,_s?0:1); } while(0)

/* ============================================================
 * Simulation timing
 * ============================================================ */
#define SIM_DWELL_SEC     12     /* seconds per drone model                      */
#define BEACON_PERIOD_MS  100   /* WiFi beacon interval (100 ms = 10 Hz)         */
#define NAN_PERIOD_MS     500   /* NAN Remote ID frame interval (FAA: ≤1 Hz)     */
#define BLE_ADV_MS        250   /* BLE advertisement interval                    */
#define RSSI_STEP_MS      300   /* How often to advance the RSSI ramp            */

static const char *TAG = "SIM";

/* ============================================================
 * WiFi raw frame injection helpers
 * The ESP32-C3 supports esp_wifi_80211_tx() to inject raw 802.11 frames.
 * ============================================================ */

/* Multicast MAC for Wi-Fi NAN (ASTM F3411-22A Remote ID) */
static const uint8_t NAN_DEST_MAC[6]   = {0x51, 0x6F, 0x9A, 0x01, 0x00, 0x00};
/* OpenDroneID vendor IE OUI */
static const uint8_t ODID_VENDOR_OUI[3] = {0xFA, 0x0B, 0xBC};
/* Parrot Remote ID vendor IE OUI */
static const uint8_t PARROT_VENDOR_OUI[3] = {0x90, 0x3A, 0xE6};

/* Current simulated MAC (set per model, per cycle) */
static uint8_t g_sim_mac[6];
/* Current simulated SSID */
static char    g_sim_ssid[33];
/* Current model index */
static int     g_model_idx = 0;
/* Current simulated RSSI */
static int8_t  g_sim_rssi  = -80;
/* BLE advertising started? */
static bool    g_ble_adv_running = false;
/* BLE sync reached? */
static volatile bool g_ble_ready = false;

/* ── Build a source MAC from an OUI + esp_random() device bytes ─────── */
static void make_sim_mac(const uint8_t oui[3], uint8_t out[6])
{
    out[0] = oui[0];
    out[1] = oui[1];
    out[2] = oui[2];
    uint32_t r = esp_random();
    out[3] = (r >> 16) & 0xFF;
    out[4] = (r >>  8) & 0xFF;
    out[5] = (r      ) & 0xFF;
    /* Clear the multicast + locally-administered bits in octet 0 so the
     * generated address looks like a real unicast global address.        */
    out[0] &= 0xFC;
}

/* ── Set ESP32-C3 source MAC to match simulated drone ──────────────── */
static void wifi_set_source_mac(const uint8_t mac[6])
{
    /* esp_wifi_set_mac changes the interface MAC (requires STA stopped).  */
    esp_wifi_set_mac(WIFI_IF_STA, mac);
}

/* ============================================================
 * WiFi beacon frame builder + injector
 *
 * Builds a minimal 802.11 beacon that the drone detector will match on:
 *   • Frame Control: Type=Management (0), Subtype=Beacon (8)
 *   • Source + BSSID = g_sim_mac
 *   • Destination = FF:FF:FF:FF:FF:FF (broadcast)
 *   • Fixed params: Timestamp(8) + Interval(2) + Capabilities(2)
 *   • IE 0x00 = SSID
 *   • IE 0x01 = Supported Rates (1 Mbps, 2 Mbps)
 *   • IF SIM_WIFI_VENDOR_IE: IE 0xDD = Remote ID vendor IE
 * ============================================================ */
static void inject_beacon(const drone_model_t *model)
{
    static uint8_t frame[160];
    int pos = 0;

    /* ── Frame Control (2 bytes) — management / beacon ────────────── */
    frame[pos++] = 0x80;   /* FC low byte: subtype=8 (beacon), type=0 (mgmt) */
    frame[pos++] = 0x00;   /* FC high byte: no flags                          */

    /* ── Duration (2 bytes) ──────────────────────────────────────────*/
    frame[pos++] = 0x00;
    frame[pos++] = 0x00;

    /* ── Addr1: Destination = broadcast ─────────────────────────────*/
    memset(&frame[pos], 0xFF, 6);  pos += 6;

    /* ── Addr2: Source (transmitter) = sim MAC ───────────────────────*/
    memcpy(&frame[pos], g_sim_mac, 6);  pos += 6;

    /* ── Addr3: BSSID = sim MAC ─────────────────────────────────────*/
    memcpy(&frame[pos], g_sim_mac, 6);  pos += 6;

    /* ── Sequence Control (2 bytes) — set to 0, NIC will increment ──*/
    frame[pos++] = 0x00;
    frame[pos++] = 0x00;

    /* ── Fixed parameters (12 bytes) ────────────────────────────────*/
    /*   Timestamp (8 bytes, zero) */
    memset(&frame[pos], 0, 8);    pos += 8;
    /*   Beacon Interval: 100 TU = 0x0064 */
    frame[pos++] = 0x64;
    frame[pos++] = 0x00;
    /*   Capability Info: ESS bit set (0x0001) */
    frame[pos++] = 0x01;
    frame[pos++] = 0x00;

    /* ── IE 0x00: SSID ─────────────────────────────────────────────*/
    uint8_t ssid_len = (uint8_t)strlen(g_sim_ssid);
    frame[pos++] = 0x00;          /* tag: SSID */
    frame[pos++] = ssid_len;
    memcpy(&frame[pos], g_sim_ssid, ssid_len);  pos += ssid_len;

    /* ── IE 0x01: Supported Rates (1 & 2 Mbps) ─────────────────────*/
    frame[pos++] = 0x01;   /* tag: Supported Rates */
    frame[pos++] = 0x02;   /* length                */
    frame[pos++] = 0x82;   /* 1 Mbps, basic         */
    frame[pos++] = 0x84;   /* 2 Mbps, basic         */

    /* ── IE 0x03: DS Parameter Set (channel) ────────────────────────*/
    frame[pos++] = 0x03;
    frame[pos++] = 0x01;
    frame[pos++] = model->wifi_channel;

    /* ── IE 0xDD: Vendor-specific Remote ID (if flag set) ───────────*/
    if (model->emit_flags & SIM_WIFI_VENDOR_IE) {
        /* Choose OUI: Parrot uses its own; everyone else uses ODID    */
        const uint8_t *oui = (model->oui[0] == PARROT_VENDOR_OUI[0] &&
                               model->oui[1] == PARROT_VENDOR_OUI[1] &&
                               model->oui[2] == PARROT_VENDOR_OUI[2])
                              ? PARROT_VENDOR_OUI : ODID_VENDOR_OUI;

        /* Minimal Remote ID payload:
         *   OUI (3) + App Sub-type (1) + Protocol version (1) +
         *   Message type (1 = Basic ID) + ID type (1) + 20-char UA ID  */
        uint8_t rid_payload[27];
        rid_payload[0] = oui[0];
        rid_payload[1] = oui[1];
        rid_payload[2] = oui[2];
        rid_payload[3] = 0x0D;   /* OUI sub-type: OpenDroneID          */
        rid_payload[4] = 0x02;   /* Protocol version: F3411-22A        */
        rid_payload[5] = 0x00;   /* Message type: Basic ID              */
        rid_payload[6] = 0x01;   /* ID type: Serial number              */
        /* 20-character UA-ID (padded with spaces) */
        snprintf((char *)&rid_payload[7], 20, "SIM%02X%02X%02X%02X%02X%02X",
                 g_sim_mac[0], g_sim_mac[1], g_sim_mac[2],
                 g_sim_mac[3], g_sim_mac[4], g_sim_mac[5]);
        memset(&rid_payload[7 + strlen((char *)&rid_payload[7])], ' ',
               20 - strlen((char *)&rid_payload[7]));

        frame[pos++] = 0xDD;           /* tag: Vendor Specific          */
        frame[pos++] = (uint8_t)(sizeof(rid_payload));
        memcpy(&frame[pos], rid_payload, sizeof(rid_payload));
        pos += sizeof(rid_payload);
    }

    /* ── Inject via esp_wifi_80211_tx ────────────────────────────────*/
    esp_err_t err = esp_wifi_80211_tx(WIFI_IF_STA, frame, pos, false);
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "beacon inject err: %s", esp_err_to_name(err));
    }
}

/* ============================================================
 * WiFi NAN action frame injector
 *
 * Builds a minimal 802.11 Action frame in the Vendor-Specific category
 * addressed to the NAN multicast MAC 51:6F:9A:01:00:00.
 * This is what Layer 0 of the detector looks for (subtype 0x0D + addr1).
 * ============================================================ */
static void inject_nan_frame(void)
{
    static uint8_t frame[80];
    int pos = 0;

    /* Frame Control: type=Management (0), subtype=Action (0x0D = 13) */
    frame[pos++] = 0xD0;   /* FC low: subtype=13 (0xD), type=0 */
    frame[pos++] = 0x00;

    /* Duration */
    frame[pos++] = 0x00;
    frame[pos++] = 0x00;

    /* Addr1: NAN multicast destination */
    memcpy(&frame[pos], NAN_DEST_MAC, 6);  pos += 6;

    /* Addr2: Source = sim MAC */
    memcpy(&frame[pos], g_sim_mac, 6);  pos += 6;

    /* Addr3: BSSID = sim MAC */
    memcpy(&frame[pos], g_sim_mac, 6);  pos += 6;

    /* Sequence Control */
    frame[pos++] = 0x00;
    frame[pos++] = 0x00;

    /* Action frame body:
     *   Category: Vendor Specific (0x7F = 127)
     *   OUI: FA:0B:BC (ASTM OpenDroneID)
     *   Sub-type: 0x0D (OID NAN)
     *   Protocol version + Basic ID message */
    frame[pos++] = 0x7F;           /* Category: Vendor Specific */
    frame[pos++] = ODID_VENDOR_OUI[0];
    frame[pos++] = ODID_VENDOR_OUI[1];
    frame[pos++] = ODID_VENDOR_OUI[2];
    frame[pos++] = 0x0D;           /* Sub-type */
    frame[pos++] = 0x02;           /* Protocol version F3411-22A */
    frame[pos++] = 0x00;           /* Message type: Basic ID     */
    frame[pos++] = 0x01;           /* ID type: Serial number     */
    /* UA-ID: 20 bytes, ASCII serial */
    char ua_id[21] = {0};
    snprintf(ua_id, sizeof(ua_id), "SIM%02X%02X%02X%02X%02X%02X",
             g_sim_mac[0], g_sim_mac[1], g_sim_mac[2],
             g_sim_mac[3], g_sim_mac[4], g_sim_mac[5]);
    memset(ua_id + strlen(ua_id), ' ', 20 - strlen(ua_id));
    memcpy(&frame[pos], ua_id, 20); pos += 20;

    esp_err_t err = esp_wifi_80211_tx(WIFI_IF_STA, frame, pos, false);
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "NAN inject err: %s", esp_err_to_name(err));
    }
}

/* ============================================================
 * BLE advertisement builder
 *
 * Builds an AD structure with:
 *   AD Type 0x16 (Service Data — 16-bit UUID):
 *     UUID 0xFFFA (little-endian: FA FF)
 *     + 25-byte Basic ID ASTM F3411-22A payload
 * ============================================================ */
static void ble_start_adv(const drone_model_t *model)
{
    if (!g_ble_ready) return;

    /* Stop any running advertisement first */
    ble_gap_adv_stop();

    /* Build raw advertising data */
    static uint8_t adv_data[31];
    int pos = 0;

    /* ── AD element: Flags (type 0x01) ─────────────────────────────*/
    adv_data[pos++] = 0x02;    /* length */
    adv_data[pos++] = 0x01;    /* type: Flags */
    adv_data[pos++] = 0x06;    /* LE General Discoverable + BR/EDR Not Supported */

    /* ── AD element: Complete Local Name (type 0x09) ────────────────*/
    /* Use a short model-name abbreviation */
    char short_name[10];
    snprintf(short_name, sizeof(short_name), "%.9s", model->vendor);
    uint8_t name_len = (uint8_t)strlen(short_name);
    adv_data[pos++] = (uint8_t)(name_len + 1);
    adv_data[pos++] = 0x09;    /* type: Complete Local Name */
    memcpy(&adv_data[pos], short_name, name_len);  pos += name_len;

    /* ── AD element: Service Data — 16-bit UUID 0xFFFA ─────────────*/
    /*   Per ASTM F3411-22A: UUID 0xFFFA = OpenDroneID               */
    /*   Payload: [proto_ver(1)] [msg_type(1)] [id_type(1)] [UA-ID(20)] */
    uint8_t svc_payload[23];
    svc_payload[0]  = 0xFA;   /* UUID 0xFFFA, little-endian low byte  */
    svc_payload[1]  = 0xFF;   /* UUID 0xFFFA, little-endian high byte */
    svc_payload[2]  = 0x02;   /* Protocol version: F3411-22A          */
    svc_payload[3]  = 0x00;   /* Message type: Basic ID (0x00)        */
    svc_payload[4]  = 0x01;   /* ID type: Serial Number               */
    /* 20-byte UA-ID: "SIM" + 6-byte hex MAC, padded with spaces      */
    char ua_id[21] = {0};
    snprintf(ua_id, sizeof(ua_id), "SIM%02X%02X%02X%02X%02X%02X",
             g_sim_mac[0], g_sim_mac[1], g_sim_mac[2],
             g_sim_mac[3], g_sim_mac[4], g_sim_mac[5]);
    size_t uid_written = strlen(ua_id);
    memset(ua_id + uid_written, ' ', 20 - uid_written);
    memcpy(&svc_payload[5], ua_id, 18);  /* 18 bytes fits in 31-byte adv_data */
    uint8_t svc_len = (uint8_t)(sizeof(svc_payload) - 2); /* payload after UUID */
    /* length field = type byte (1) + uuid (2) + payload              */
    if (pos + 1 + 1 + (int)sizeof(svc_payload) <= 31) {
        adv_data[pos++] = (uint8_t)(1 + sizeof(svc_payload));
        adv_data[pos++] = 0x16;    /* type: Service Data — 16-bit UUID */
        memcpy(&adv_data[pos], svc_payload, sizeof(svc_payload));
        pos += sizeof(svc_payload);
    }
    (void)svc_len; /* suppress unused warning */

    /* Set raw advertising data */
    struct ble_gap_adv_params adv_params = {
        .conn_mode  = BLE_GAP_CONN_MODE_NON,   /* non-connectable */
        .disc_mode  = BLE_GAP_DISC_MODE_NON,   /* non-discoverable service */
        .itvl_min   = BLE_GAP_ADV_FAST_INTERVAL1_MIN,
        .itvl_max   = BLE_GAP_ADV_FAST_INTERVAL1_MIN + 16,
    };

    int rc = ble_gap_adv_set_data(adv_data, pos);
    if (rc != 0) {
        ESP_LOGD(TAG, "ble_gap_adv_set_data rc=%d", rc);
        return;
    }

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC,
                           NULL,          /* no peer address (non-directed) */
                           BLE_HS_FOREVER,
                           &adv_params,
                           NULL,          /* no event callback needed       */
                           NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGD(TAG, "ble_gap_adv_start rc=%d", rc);
    } else {
        g_ble_adv_running = true;
    }
}

static void ble_stop_adv(void)
{
    if (g_ble_adv_running) {
        ble_gap_adv_stop();
        g_ble_adv_running = false;
    }
}

/* ============================================================
 * NimBLE host callbacks
 * ============================================================ */
static void ble_on_reset(int reason)
{
    ESP_LOGW(TAG, "BLE reset: %d", reason);
    g_ble_ready = false;
}

static void ble_on_sync(void)
{
    ESP_LOGI(TAG, "BLE synced — advertising ready");
    g_ble_ready = true;
}

static void nimble_host_task(void *param)
{
    nimble_port_run();
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
 * Simulated RSSI ramp
 * Returns current RSSI given elapsed_ms into the model's dwell period.
 * Ramps up from rssi_start to rssi_peak in the first half, then back down.
 * ============================================================ */
static int8_t compute_rssi(const drone_model_t *m, uint32_t elapsed_ms)
{
    uint32_t half = (uint32_t)SIM_DWELL_SEC * 500U;   /* ms for half period */
    float t;
    int8_t start = m->rssi_start;
    int8_t peak  = m->rssi_peak;

    if (elapsed_ms < half) {
        t = (float)elapsed_ms / (float)half;
    } else {
        t = 1.0f - ((float)(elapsed_ms - half) / (float)half);
        if (t < 0.0f) t = 0.0f;
    }
    return (int8_t)(start + (int)(t * (float)(peak - start)));
}

/* ============================================================
 * Main simulation task
 * ============================================================ */
static void sim_task(void *arg)
{
    uint32_t beacon_tmr  = 0;
    uint32_t nan_tmr     = 0;
    uint32_t ble_tmr     = 0;
    uint32_t rssi_tmr    = 0;
    uint32_t led_tmr     = 0;
    uint32_t model_tmr   = 0;
    uint32_t tick_ms     = 50;   /* task tick period */
    bool     wifi_ready  = false;

    /* Wait for WiFi to be ready */
    vTaskDelay(pdMS_TO_TICKS(2000));
    wifi_ready = true;

    /* Initialise first model */
    const drone_model_t *m = &DRONE_MODELS[g_model_idx];
    make_sim_mac(m->oui, g_sim_mac);
    snprintf(g_sim_ssid, sizeof(g_sim_ssid), m->ssid_fmt,
             ((uint32_t)g_sim_mac[4] << 8) | g_sim_mac[5]);
    g_sim_rssi = m->rssi_start;
    wifi_set_source_mac(g_sim_mac);

    printf("\n[SIM] ====================================================\n");
    printf("[SIM]  ESP32-C3 Consumer Drone Simulator starting\n");
    printf("[SIM]  Cycling %d drone models, %d sec each\n",
           DRONE_MODEL_COUNT, SIM_DWELL_SEC);
    printf("[SIM] ====================================================\n\n");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(tick_ms));
        beacon_tmr += tick_ms;
        nan_tmr    += tick_ms;
        ble_tmr    += tick_ms;
        rssi_tmr   += tick_ms;
        led_tmr    += tick_ms;
        model_tmr  += tick_ms;

        m = &DRONE_MODELS[g_model_idx];

        /* ── Advance model after dwell period ──────────────────── */
        if (model_tmr >= (uint32_t)SIM_DWELL_SEC * 1000U) {
            model_tmr = 0;
            ble_stop_adv();

            g_model_idx = (g_model_idx + 1) % DRONE_MODEL_COUNT;
            m = &DRONE_MODELS[g_model_idx];

            make_sim_mac(m->oui, g_sim_mac);
            snprintf(g_sim_ssid, sizeof(g_sim_ssid), m->ssid_fmt,
                     ((uint32_t)g_sim_mac[4] << 8) | g_sim_mac[5]);
            g_sim_rssi = m->rssi_start;

            wifi_set_source_mac(g_sim_mac);

            printf("\n[SIM] ── Next model: %s (%s) ──\n", m->model_name, m->vendor);
            printf("[SIM]    MAC     : %02X:%02X:%02X:%02X:%02X:%02X\n",
                   g_sim_mac[0], g_sim_mac[1], g_sim_mac[2],
                   g_sim_mac[3], g_sim_mac[4], g_sim_mac[5]);
            printf("[SIM]    SSID    : %s\n", g_sim_ssid);
            printf("[SIM]    Channel : %d\n", m->wifi_channel);
            printf("[SIM]    Layers  : %s%s%s%s%s\n",
                   (m->emit_flags & SIM_WIFI_OUI)       ? "WiFi-OUI " : "",
                   (m->emit_flags & SIM_WIFI_SSID)      ? "WiFi-SSID " : "",
                   (m->emit_flags & SIM_WIFI_NAN)       ? "WiFi-NAN " : "",
                   (m->emit_flags & SIM_WIFI_VENDOR_IE) ? "WiFi-VendorIE " : "",
                   (m->emit_flags & SIM_BLE_REMOTEID)   ? "BLE-RemoteID" : "");
            printf("[SIM]    RSSI    : %d → %d dBm (simulated fly-by)\n\n",
                   m->rssi_start, m->rssi_peak);
        }

        /* ── Advance simulated RSSI ─────────────────────────────── */
        if (rssi_tmr >= RSSI_STEP_MS) {
            rssi_tmr = 0;
            g_sim_rssi = compute_rssi(m, model_tmr);
        }

        /* ── LED heartbeat: 2 Hz (toggle every 250 ms) ──────────── */
        if (led_tmr >= 250) {
            led_tmr = 0;
            LED_TOGGLE();
        }

        if (!wifi_ready) continue;

        /* ── WiFi beacon ─────────────────────────────────────────── */
        if (beacon_tmr >= BEACON_PERIOD_MS) {
            beacon_tmr = 0;
            if (m->emit_flags & (SIM_WIFI_OUI | SIM_WIFI_SSID | SIM_WIFI_VENDOR_IE)) {
                inject_beacon(m);
                ESP_LOGD(TAG, "beacon SSID=%s RSSI=%d", g_sim_ssid, (int)g_sim_rssi);
            }
        }

        /* ── WiFi NAN Remote ID frame ────────────────────────────── */
        if (nan_tmr >= NAN_PERIOD_MS) {
            nan_tmr = 0;
            if (m->emit_flags & SIM_WIFI_NAN) {
                inject_nan_frame();
                ESP_LOGD(TAG, "NAN frame → 51:6F:9A:01:00:00");
            }
        }

        /* ── BLE Remote ID advertisement ─────────────────────────── */
        if (ble_tmr >= BLE_ADV_MS) {
            ble_tmr = 0;
            if (m->emit_flags & SIM_BLE_REMOTEID) {
                if (!g_ble_adv_running) {
                    ble_start_adv(m);
                    ESP_LOGD(TAG, "BLE adv started UUID=0xFFFA");
                }
            } else {
                ble_stop_adv();
            }
        }
    }
}

/* ============================================================
 * WiFi initialization  (STA mode, promiscuous TX capability)
 * ============================================================ */
static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    /* Start WiFi so we can call esp_wifi_80211_tx() */
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Enable raw frame injection (required for 80211_tx) */
    ESP_ERROR_CHECK(esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE));

    ESP_LOGI(TAG, "WiFi initialized (STA, raw TX enabled)");
}

/* ============================================================
 * LED setup
 * ============================================================ */
static void led_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    LED_OFF();
}

/* ============================================================
 * app_main
 * ============================================================ */
void app_main(void)
{
    /* Boot banner */
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════╗\n");
    printf("║   ESP32-C3 Consumer Drone Simulator  v1.0                ║\n");
    printf("║   Paired with: ESP32-C5 Drone Detector (lab44.us/drone)  ║\n");
    printf("╠═══════════════════════════════════════════════════════════╣\n");
    printf("║   Simulating %2d drone models via:                        ║\n",
           DRONE_MODEL_COUNT);
    printf("║   • WiFi beacons (OUI source MAC + model SSID)           ║\n");
    printf("║   • WiFi NAN action frames (FAA Remote ID)               ║\n");
    printf("║   • WiFi beacon vendor IE OUI FA:0B:BC (OpenDroneID)     ║\n");
    printf("║   • BLE advertisements with UUID 0xFFFA (ASTM F3411)     ║\n");
    printf("╚═══════════════════════════════════════════════════════════╝\n\n");

    /* NVS flash (required by WiFi) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Hardware */
    led_init();

    /* WiFi */
    wifi_init();

    /* BLE */
    ble_init();

    /* Simulator task  — high priority so timing is consistent */
    xTaskCreate(sim_task, "sim_task", 8192, NULL, 10, NULL);

    ESP_LOGI(TAG, "Simulator task started — model cycle begins in 2 s");
}
