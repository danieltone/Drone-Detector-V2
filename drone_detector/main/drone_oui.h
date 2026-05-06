/**
 * drone_oui.h  —  Consumer drone OUI and SSID fingerprint database
 *
 * OUI sources:
 *   - IEEE Standards Registration Authority: standards-oui.ieee.org
 *   - Verified via macvendors.com API (✓ = confirmed, * = widely reported)
 *
 * SSID patterns from published device specifications and community testing.
 *
 * ESP32-C5 Drone Detector — lab44.us/drone style project
 *
 * MEMORY NOTE (ESP32-C5 has 4 MB flash):
 *   OUI table  (~20 entries × 12 bytes)  = ~240 B
 *   SSID table (~50 entries × 16 bytes)  = ~800 B
 *   Total table overhead                  < 1.5 KB
 *   Full firmware binary (IDF WiFi+BLE)  ~700 KB–1.3 MB
 *   You could add 10,000 OUI entries and not notice.
 *
 * Wi-Fi Remote ID detection notes:
 *   Beyond source-MAC OUI matching, FAA-compliant drones broadcast Remote ID
 *   in two additional ways — handled in main.c, not here:
 *   1. Wi-Fi NAN action frames: destination multicast MAC = 51:6F:9A:01:00:00
 *   2. Wi-Fi beacon vendor-specific IE: tag 0xDD, vendor OUI = FA:0B:BC (ODID)
 *      (Parrot also uses OUI 90:3A:E6 in its Wi-Fi beacon vendor IE)
 *   These two vectors catch ANY FAA-compliant drone regardless of brand.
 */
#pragma once
#include <stdint.h>
#include <string.h>
#include <strings.h>   /* strncasecmp */

/* ───────────────────────────────────────────────────────────────────────────
 * OUI Table
 * Each entry: first 3 octets of the MAC (network byte order = OUI),
 * the vendor short-name, and a note on confidence / product line.
 * ─────────────────────────────────────────────────────────────────────────*/
typedef struct {
    uint8_t     oui[3];
    const char *vendor;
    const char *note;
} drone_oui_entry_t;

static const drone_oui_entry_t DRONE_OUI_TABLE[] = {

    /* ── DJI  (SZ DJI TECHNOLOGY CO.,LTD) ─────────────────────────────── */
    /* Confirmed via IEEE OUI registry + macvendors.com API                 */
    { {0x60, 0x60, 0x1F}, "DJI",    "✓ IEEE – all current DJI models"        },
    { {0x34, 0xD2, 0x62}, "DJI",    "✓ IEEE – Mavic/Mini/Air/FPV/Avata"      },
    { {0x48, 0x1C, 0xB9}, "DJI",    "* widely reported in drone-detect tools" },

    /* ── Parrot SA  (France) ─────────────────────────────────────────────*/
    /* Confirmed via macvendors.com API                                     */
    { {0x90, 0x3A, 0xE6}, "Parrot", "✓ macvendors – ANAFI/Bebop/Disco"       },
    { {0xA0, 0x14, 0x3D}, "Parrot", "✓ macvendors – ANAFI USA/Skycontroller" },
    { {0x00, 0x26, 0x7E}, "Parrot", "✓ macvendors – older Parrot models"     },
    { {0x90, 0x03, 0xB7}, "Parrot", "* Parrot SA legacy registration"        },
    { {0x00, 0x12, 0x1C}, "Parrot", "* Parrot SA older product line"         },

    /* ── Future entries: add new confirmed OUIs here ───────────────────── */
    /* To look up an OUI:  curl https://api.macvendors.com/XX:XX:XX          */
};

#define DRONE_OUI_COUNT  ((int)(sizeof(DRONE_OUI_TABLE) / sizeof(DRONE_OUI_TABLE[0])))

/**
 * drone_oui_lookup  —  Returns vendor string if the first 3 bytes of `mac`
 * match a known drone OUI, or NULL if not a drone.
 *
 * @param mac  6-byte MAC in standard network order (OUI = bytes [0..2]).
 *             For WiFi promiscuous frames use addr2 / addr3 directly.
 *             For NimBLE BLE addresses (little-endian), pass val[5..0].
 */
static inline const char *drone_oui_lookup(const uint8_t *mac)
{
    for (int i = 0; i < DRONE_OUI_COUNT; i++) {
        if (mac[0] == DRONE_OUI_TABLE[i].oui[0] &&
            mac[1] == DRONE_OUI_TABLE[i].oui[1] &&
            mac[2] == DRONE_OUI_TABLE[i].oui[2]) {
            return DRONE_OUI_TABLE[i].vendor;
        }
    }
    return NULL;
}

/* ───────────────────────────────────────────────────────────────────────────
 * SSID Fingerprint Table
 * Case-insensitive prefix matching on the SSID string.
 * These patterns are from published Espressif / DJI / Parrot device docs
 * and confirmed community observations on consumer hardware.
 * ─────────────────────────────────────────────────────────────────────────*/
typedef struct {
    const char *prefix;   /* case-insensitive prefix to match */
    const char *vendor;   /* human-readable vendor/model name */
} ssid_fp_t;

static const ssid_fp_t SSID_FINGERPRINTS[] = {

    /* ── DJI ──────────────────────────────────────────────────────────── */
    { "DJI-",        "DJI Drone"           },   /* generic DJI AP mode     */
    { "DJI_",        "DJI Drone"           },   /* alternate separator     */
    { "MAVIC-",      "DJI Mavic"           },
    { "PHANTOM",     "DJI Phantom"         },   /* Phantom 3/4/4 Pro       */
    { "SPARK-",      "DJI Spark"           },
    { "DJI MINI",    "DJI Mini"            },   /* Mini 1/2/3/3 Pro/SE     */
    { "MINI SE",     "DJI Mini SE"         },
    { "DJI FPV",     "DJI FPV"             },
    { "AVATA",       "DJI Avata"           },
    { "DJI AIR",     "DJI Air"             },   /* Air 2/2S/3              */
    { "AGRAS",       "DJI Agras"           },   /* agricultural drones     */
    { "MATRICE",     "DJI Matrice"         },   /* enterprise series       */

    /* ── DJI / Ryze Tello  (ESP32-powered, DJI flight controller) ──────*/
    { "TELLO-",      "DJI/Ryze Tello"      },
    { "TELLO",       "DJI/Ryze Tello"      },

    /* ── Parrot ────────────────────────────────────────────────────────*/
    { "Bebop",       "Parrot Bebop"        },   /* Bebop 1 / 2             */
    { "ANAFI",       "Parrot ANAFI"        },   /* ANAFI / ANAFI USA       */
    { "ardrone",     "Parrot AR.Drone"     },   /* AR.Drone 2.0            */
    { "AR.Drone",    "Parrot AR.Drone"     },
    { "Disco-",      "Parrot Disco"        },
    { "Mambo-",      "Parrot Mambo"        },
    { "Swing-",      "Parrot Swing"        },

    /* ── Autel Robotics ────────────────────────────────────────────────*/
    { "Autel-",      "Autel EVO"           },
    { "AUTEL-",      "Autel EVO"           },
    { "evo-",        "Autel EVO"           },
    { "EVO-",        "Autel EVO"           },
    { "AUTEL_",      "Autel EVO"           },

    /* ── Skydio ────────────────────────────────────────────────────────*/
    { "Skydio",      "Skydio Drone"        },
    { "SKYDIO",      "Skydio Drone"        },

    /* ── Hubsan ────────────────────────────────────────────────────────*/
    { "HUBSAN",      "Hubsan Drone"        },
    { "Hubsan",      "Hubsan Drone"        },

    /* ── DJI — newer model-specific patterns ──────────────────────────*/
    /* Air 3 broadcasts "DJI Air 3-XXXX"; covered by parent "DJI AIR" */
    { "DJI AIR 3",    "DJI Air 3"           },   /* explicit model match  */
    { "DJI AIR 2S",   "DJI Air 2S"          },
    { "MINI 4",       "DJI Mini 4 Pro"       },   /* Mini 4 Pro            */
    { "MINI 3",       "DJI Mini 3 Pro"       },   /* Mini 3 / Mini 3 Pro   */
    { "DJI MINI 2",   "DJI Mini 2"           },
    { "DJI MINI 3",   "DJI Mini 3"           },
    { "DJI MINI 4",   "DJI Mini 4 Pro"       },
    { "MAVIC3-",      "DJI Mavic 3"          },   /* Mavic 3 / Cine / Pro  */
    { "MAVIC2-",      "DJI Mavic 2"          },
    { "MAVIC AIR",    "DJI Mavic Air"         },
    { "PHANTOM4-",    "DJI Phantom 4"        },
    { "PHANTOM3-",    "DJI Phantom 3"        },

    /* ── Skydio ────────────────────────────────────────────────────────*/
    /*  OUI: Skydio has 1 IEEE MA-L registration (2019-07-02).           */
    /*  OUI prefix not resolvable via public API — detection via SSID    */
    /*  and BLE Remote ID (UUID 0xFFFA) instead.                         */
    { "Skydio",       "Skydio Drone"         },   /* R1, 2, X2, X10       */
    { "SKYDIO",       "Skydio Drone"         },
    { "Skydio 2",     "Skydio 2"             },
    { "Skydio X2",    "Skydio X2"            },
    { "Skydio X10",   "Skydio X10"           },

    /* ── Autel Robotics ────────────────────────────────────────────────*/
    /*  OUIs: Autel Intelligent Technology Corp.,Ltd (2021)               */
    /*        Autel Robotics USA LLC                   (2024)             */
    /*  OUI prefixes not resolvable via public API — detection via SSID. */
    { "Autel-",       "Autel EVO"            },
    { "AUTEL-",       "Autel EVO"            },
    { "AUTEL_",       "Autel EVO"            },
    { "EVO-",         "Autel EVO"            },
    { "EVO2-",        "Autel EVO II"         },
    { "EVO II",       "Autel EVO II"         },   /* EVO II Pro / Rugged  */
    { "EVO_II",       "Autel EVO II"         },
    { "EVO NANO",     "Autel EVO Nano"       },
    { "EVO LITE",     "Autel EVO Lite"       },
    { "DRAGONFISH",   "Autel Dragonfish"     },

    /* ── Generic Remote-ID hint (some drones broadcast service name) ──*/
    { "RemoteID",     "Remote ID Beacon"     },
    { "OpenDroneID",  "OpenDroneID Node"     },
};

#define SSID_FP_COUNT  ((int)(sizeof(SSID_FINGERPRINTS) / sizeof(SSID_FINGERPRINTS[0])))

/**
 * drone_ssid_fingerprint  —  Returns vendor name if `ssid` starts with a
 * known drone SSID prefix (case-insensitive), or NULL if no match.
 */
static inline const char *drone_ssid_fingerprint(const char *ssid)
{
    if (!ssid || ssid[0] == '\0') return NULL;
    for (int i = 0; i < SSID_FP_COUNT; i++) {
        size_t plen = strlen(SSID_FINGERPRINTS[i].prefix);
        if (strncasecmp(ssid, SSID_FINGERPRINTS[i].prefix, plen) == 0) {
            return SSID_FINGERPRINTS[i].vendor;
        }
    }
    return NULL;
}
