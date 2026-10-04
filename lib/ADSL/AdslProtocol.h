/*!
 * @file AdslProtocol.h
 * @brief ADS-L SRD860 M-Band protocol encoder / decoder for GXAirCom
 *
 * Implements EASA ADS-L 4 SRD-860 Issue 1 (Dec 2022) / Issue 2 (Dec 2025).
 * M-Band: 868.2 / 868.4 MHz, 2-GFSK, 50 kbps net (100 kbps raw + Manchester).
 *
 * Packet structure (decoded, 25 bytes):
 *   [0]     Length = 0x18 (24 bytes follow)
 *   [1]     Network byte: Protocol(4b)=0, SigFlag(1b)=0, KeyIdx(2b)=0, ECM(1b)=0
 *   [2..21] ADS-L Data (20 bytes), XXTEA-scrambled (5 words, 6 rounds, key=0)
 *             [2]     Payload Type = 0x02 (Traffic/iConspicuity)
 *             [3..5]  24-bit sender address, MSB first
 *             [6]     AMT (6b) in bits[7:2], Relay(1b) in bit[0]
 *             [7..21] iConspicuity payload (15 bytes = 120 bits)
 *   [22..24] CRC-24 big-endian (covers bytes[1..21])
 *
 * Protocol Version 0 is transmitted for Traffic payload (Issue 1/2 compatible).
 * Manchester encoding (G.E. Thomas: '0'→'10', '1'→'01') is applied by fmac.cpp.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// ─────────────────────────────────────────────────────────────────────────────
// Physical layer constants
// ─────────────────────────────────────────────────────────────────────────────
#define ADSL_FREQ_MBAND_1       868200000UL
#define ADSL_FREQ_MBAND_2       868400000UL
#define ADSL_BITRATE_RAW        100000UL      ///< on-air chip rate [bps]

// ─────────────────────────────────────────────────────────────────────────────
// Packet size constants
// ─────────────────────────────────────────────────────────────────────────────
#define ADSL_CRC_SIZE           3             ///< CRC-24 (3 bytes)
#define ADSL_PAYLOAD_SIZE       15            ///< iConspicuity payload bytes
#define ADSL_HEADER_SIZE        5             ///< ADS-L Header (type 1B + addr 4B)
#define ADSL_DATA_SIZE          20            ///< ADS-L Data = header + payload (XXTEA input)
#define ADSL_NET_SIZE           1             ///< Network layer byte
#define ADSL_CRC_COVER_SIZE     21            ///< Bytes covered by CRC-24 (Net + Data)
#define ADSL_PACKET_SIZE        25            ///< Total decoded packet bytes

// ─────────────────────────────────────────────────────────────────────────────
// Protocol constants
// ─────────────────────────────────────────────────────────────────────────────
#define ADSL_PROTOCOL_VERSION       0         ///< Issue 1/2 backward-compatible
#define ADSL_PAYLOAD_TYPE_TRAFFIC   0x02      ///< Traffic (iConspicuity) payload

/// Address Mapping Table (AMT) index values
#define ADSL_AMT_RANDOM     0   ///< Privacy / random address
#define ADSL_AMT_ICAO       5   ///< ICAO Mode-S address
#define ADSL_AMT_FLARM      6   ///< FLARM (and OEMs)
#define ADSL_AMT_OGN        7   ///< OGN-Tracker
#define ADSL_AMT_FANET      8   ///< FANET (and OEMs) — default for GXAirCom

/// TX interval [ms] — spec requires channel alternation on each TX
#define ADSL_TX_INTERVAL_MS     1000

extern uint32_t adsl_rx_count;
extern uint32_t adsl_tx_count;

// ─────────────────────────────────────────────────────────────────────────────
// Flight State  (2-bit field)
// ─────────────────────────────────────────────────────────────────────────────
typedef enum adsl_flight_state {
    ADSL_STATE_UNDEFINED = 0,
    ADSL_STATE_GROUND    = 1,
    ADSL_STATE_AIRBORNE  = 2,
    ADSL_STATE_RESERVED  = 3,
} adsl_flight_state_t;

// ─────────────────────────────────────────────────────────────────────────────
// Aircraft Category  (5-bit field, spec values)
// Note: values 3, 6, 7 have slightly different names in Issue 1 vs Issue 2.
// When Protocol Version = 0 is transmitted, Issue 1 semantics apply on-air.
// ─────────────────────────────────────────────────────────────────────────────
typedef enum adsl_aircraft_category {
    ADSL_CAT_NONE          = 0,   ///< No emitter category info
    ADSL_CAT_LIGHT_FIXED   = 1,   ///< Light fixed-wing (< 7031 kg)
    ADSL_CAT_HEAVY_FIXED   = 2,   ///< Small-to-heavy fixed-wing (>= 7031 kg)
    ADSL_CAT_ROTORCRAFT    = 3,   ///< Rotorcraft (Issue 1) / Light rotorcraft (Issue 2)
    ADSL_CAT_GLIDER        = 4,   ///< Glider / sailplane
    ADSL_CAT_LTA           = 5,   ///< Lighter-than-air (balloon, airship)
    ADSL_CAT_ULTRALIGHT    = 6,   ///< Ultralight (Issue 1) / Ultralight+motorized HG (Issue 2)
    ADSL_CAT_HANG_PARA     = 7,   ///< Hang-glider/paraglider (Issue 1) / Paraglider (Issue 2)
    ADSL_CAT_SKYDIVER      = 8,   ///< Parachutist / skydiver / wingsuit
    ADSL_CAT_EVTOL         = 9,   ///< eVTOL / UAM
    ADSL_CAT_GYROCOPTER    = 10,
    ADSL_CAT_UAS_OPEN      = 11,
    ADSL_CAT_UAS_SPECIFIC  = 12,
    ADSL_CAT_UAS_CERTIFIED = 13,
} adsl_aircraft_category_t;

// ─────────────────────────────────────────────────────────────────────────────
// Emergency Status  (3-bit field)
// ─────────────────────────────────────────────────────────────────────────────
typedef enum adsl_emergency {
    ADSL_EMERG_NONE         = 0,
    ADSL_EMERG_NO_EMERG     = 1,  ///< Explicitly "no emergency"
    ADSL_EMERG_GENERAL      = 2,
    ADSL_EMERG_MEDICAL      = 3,
    ADSL_EMERG_NO_COMMS     = 4,
    ADSL_EMERG_INTERFERENCE = 5,
    ADSL_EMERG_DOWNED       = 6,
    ADSL_EMERG_RESERVED     = 7,
} adsl_emergency_t;

// ─────────────────────────────────────────────────────────────────────────────
// iConspicuity / Traffic data structure
// ─────────────────────────────────────────────────────────────────────────────
typedef struct adsl_iconspicuity {
    // Identity
    uint32_t                address;          ///< 24-bit sender address
    uint8_t                 amt;              ///< AMT index (default ADSL_AMT_FANET=8)

    // Position (WGS-84)
    float                   latitude;         ///< decimal degrees [-90..+90]
    float                   longitude;        ///< decimal degrees [-180..+180]
    float                   altitude_wgs84_m; ///< WGS-84 ellipsoid height [m]

    // Kinematics
    float                   speed_ms;         ///< ground speed [m/s]
    float                   vertical_rate_ms; ///< climb rate [m/s], positive = up
    float                   track_deg;        ///< ground track [deg true, 0=N, 90=E]

    // Status
    uint8_t                 timestamp_qs;     ///< quarter-seconds since full UTC hour, mod 60 (0..59)
    adsl_flight_state_t     flight_state;
    adsl_aircraft_category_t aircraft_category;
    adsl_emergency_t        emergency;

    // Quality indicators (spec: non-certified devices use SIL=0, DAL=0; NIC/HFOM/VFOM dynamic)
    uint8_t                 sil;              ///< Source Integrity Level [0..3]
    uint8_t                 design_assurance; ///< Design Assurance Level [0..3]
    uint8_t                 nav_integrity;    ///< Navigation Integrity (NIC) [0..12]
    uint8_t                 h_accuracy;       ///< Horizontal Position Accuracy (HFOM) [0..7]
    uint8_t                 v_accuracy;       ///< Vertical Position Accuracy (VFOM) [0..3]
    uint8_t                 vel_accuracy;     ///< Velocity Accuracy (NACv) [0..3]

    // Privacy — set amt=ADSL_AMT_RANDOM instead where possible
    bool                    privacy_mode;     ///< true → encode AMT=0 (random address)
} adsl_iconspicuity_t;

// ─────────────────────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Encode a 25-byte ADS-L M-Band Traffic packet into buf[ADSL_PACKET_SIZE].
 * Applies XXTEA scrambling and appends CRC-24. Result passed to radio TX.
 */
bool adsl_encode_packet(uint8_t *buf, const adsl_iconspicuity_t *data);

/**
 * @brief Decode a 25-byte ADS-L M-Band packet (CRC-24 check + XXTEA decrypt).
 * @return true if CRC valid and payload type = Traffic (0x02).
 */
bool adsl_decode_packet(const uint8_t *buf, int len, adsl_iconspicuity_t *data);

/**
 * @brief CRC-24: same polynomial as Mode-S / ADS-B (poly 0xFFFA0480).
 * Covers buf[0..len-1]. Result is the 3-byte CRC value (bits [23:0]).
 */
uint32_t adsl_crc24(const uint8_t *data, int len);

/**
 * @brief Map a FANET aircraft_t to the nearest ADS-L aircraft category.
 */
adsl_aircraft_category_t adsl_category_from_fanet(uint8_t fanet_type);
