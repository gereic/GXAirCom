/*!
 * @file AdslProtocol.cpp
 * @brief ADS-L SRD860 M-Band protocol encoder / decoder for GXAirCom
 *
 * Reference: EASA ADS-L 4 SRD-860 Issue 2 (1 December 2025).
 * Packet structure and algorithm details are in AdslProtocol.h.
 */

#include "AdslProtocol.h"
#include <string.h>

uint32_t adsl_rx_count = 0;
uint32_t adsl_tx_count = 0;

// ─────────────────────────────────────────────────────────────────────────────
// CRC-24  (same polynomial as Mode-S / ADS-B)
// Generator: G(x) = x^24 + x^23 + ... + x^12 + x^10 + x^3 + 1
// Transmitted big-endian; 3 lower bytes of the result are used.
// ─────────────────────────────────────────────────────────────────────────────
static uint32_t _crc24_polypass(uint32_t crc, uint8_t input) {
    const uint32_t poly = 0xFFFA0480u;
    crc |= input;
    for (uint8_t bit = 0; bit < 8; bit++) {
        if (crc & 0x80000000u) crc ^= poly;
        crc <<= 1;
    }
    return crc;
}

uint32_t adsl_crc24(const uint8_t *data, int len) {
    uint32_t crc = 0;
    for (int i = 0; i < len; i++)
        crc = _crc24_polypass(crc, data[i]);
    crc = _crc24_polypass(crc, 0);
    crc = _crc24_polypass(crc, 0);
    crc = _crc24_polypass(crc, 0);
    return crc >> 8;
}

// ─────────────────────────────────────────────────────────────────────────────
// XXTEA  (5 words = 20 bytes, 6 rounds, all-zero key)
// Spec reference: E.2 of EASA ADS-L 4 SRD-860 Issue 2.
// ─────────────────────────────────────────────────────────────────────────────
#define ADSL_XXTEA_WORDS   5
#define ADSL_XXTEA_ROUNDS  6
#define XXTEA_DELTA        0x9e3779b9u
#define XXTEA_MX(z,y,sum,key,p,e) \
    (( ((z)>>5^(y)<<2) + ((y)>>3^(z)<<4) ) ^ ( ((sum)^(y)) + ((key)[((p)&3)^(e)]^(z)) ))

static void xxtea_encrypt(uint32_t *v, uint32_t n, const uint32_t *key, uint32_t rounds) {
    uint32_t z, y = v[0], sum = 0, e;
    uint32_t p, q;
    z = v[n - 1];
    q = rounds;
    while (q-- > 0) {
        sum += XXTEA_DELTA;
        e = sum >> 2 & 3;
        for (p = 0; p < n - 1; p++) {
            y = v[p + 1];
            v[p] += XXTEA_MX(z, y, sum, key, p, e);
            z = v[p];
        }
        y = v[0];
        v[n - 1] += XXTEA_MX(z, y, sum, key, p, e);
        z = v[n - 1];
    }
}

static void xxtea_decrypt(uint32_t *v, uint32_t n, const uint32_t *key, uint32_t rounds) {
    uint32_t z, y, sum, e;
    uint32_t p, q;
    q = rounds;
    sum = q * XXTEA_DELTA;
    y = v[0];
    while (q-- > 0) {
        e = sum >> 2 & 3;
        for (p = n - 1; p > 0; p--) {
            z = v[p - 1];
            v[p] -= XXTEA_MX(z, y, sum, key, p, e);
            y = v[p];
        }
        z = v[n - 1];
        v[0] -= XXTEA_MX(z, y, sum, key, p, e);
        y = v[0];
        sum -= XXTEA_DELTA;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Bit-packing helpers  (MSB-first per byte: bit offset 0 = MSB of byte[0])
// ─────────────────────────────────────────────────────────────────────────────
static void pack_bits(uint8_t *buf, int offset, int bits, uint32_t value) {
    for (int i = bits - 1; i >= 0; i--) {
        int g = offset + (bits - 1 - i);
        if (value & (1u << i))
            buf[g >> 3] |= (uint8_t)(0x80u >> (g & 7));
    }
}

static uint32_t unpack_bits(const uint8_t *buf, int offset, int bits) {
    uint32_t v = 0;
    for (int i = bits - 1; i >= 0; i--) {
        int g = offset + (bits - 1 - i);
        if (buf[g >> 3] & (uint8_t)(0x80u >> (g & 7)))
            v |= (1u << i);
    }
    return v;
}

static int32_t sign_extend(uint32_t v, int bits) {
    uint32_t sb = 1u << (bits - 1);
    return (v & sb) ? (int32_t)(v | ~((sb << 1) - 1)) : (int32_t)v;
}

// ─────────────────────────────────────────────────────────────────────────────
// Exponential encoding  (Issue 2 corrected formula)
//
// Unsigned layout: <exp:2><base:N>
//   encode: find largest e* in {0..3} s.t. value >= 2^(N+e*) - 2^N
//           exponent = e*, base = (value - 2^(N+e*) + 2^N) / 2^e*
//   decode: value = 2^e * (2^N + base) - 2^N
//
// Signed layout: <sign:1><exp:2><base:N>
//   Zero always encoded as sign=0.
// ─────────────────────────────────────────────────────────────────────────────
static uint32_t exp_encode_u(uint32_t value, int N) {
    // Find largest e* in {0,1,2,3} such that value >= 2^(N+e*) - 2^N
    int e = 0;
    for (int i = 3; i >= 1; i--) {
        if (value >= (uint32_t)((1u << (N + i)) - (1u << N))) {
            e = i;
            break;
        }
    }
    uint32_t offset = (1u << (N + e)) - (1u << N);  // 2^(N+e) - 2^N
    uint32_t base = (value - offset) >> e;
    uint32_t maxbase = (1u << N) - 1;
    if (base > maxbase) base = maxbase;
    return ((uint32_t)e << N) | base;
}

static uint32_t exp_decode_u(uint32_t encoded, int N) {
    uint32_t e    = encoded >> N;
    uint32_t base = encoded & ((1u << N) - 1);
    return ((1u << e) * ((1u << N) + base)) - (1u << N);
}

// ─────────────────────────────────────────────────────────────────────────────
// Aircraft type mapping: FANET → ADS-L category
// ─────────────────────────────────────────────────────────────────────────────
adsl_aircraft_category_t adsl_category_from_fanet(uint8_t fanet_type) {
    uint8_t t = (fanet_type >= 0x80) ? (fanet_type - 0x80) : fanet_type;
    switch (t) {
        case 0:  return ADSL_CAT_NONE;
        case 1:  return ADSL_CAT_HANG_PARA;    // paraglider
        case 2:  return ADSL_CAT_HANG_PARA;    // hang glider
        case 3:  return ADSL_CAT_LTA;          // balloon
        case 4:  return ADSL_CAT_GLIDER;
        case 5:  return ADSL_CAT_LIGHT_FIXED;  // powered aircraft
        case 6:  return ADSL_CAT_ROTORCRAFT;   // helicopter
        case 7:  return ADSL_CAT_UAS_OPEN;     // UAV
        case 8:  return ADSL_CAT_LIGHT_FIXED;
        case 9:  return ADSL_CAT_HEAVY_FIXED;
        case 10: return ADSL_CAT_NONE;
        case 11: return ADSL_CAT_LTA;
        case 12: return ADSL_CAT_LTA;
        case 13: return ADSL_CAT_UAS_OPEN;
        default: return ADSL_CAT_NONE;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// iConspicuity payload encoder/decoder  (120 bits = 15 bytes)
//
// Bit layout (MSB-first per byte, offset 0 = MSB of byte[0]):
//   [0..5]    Timestamp (6) — quarter-seconds since full UTC hour, mod 60
//   [6..7]    Flight State (2)
//   [8..12]   Aircraft Category (5)
//   [13..15]  Emergency Status (3)
//   [16..39]  Latitude (24, signed, LSB = 1°/93206)
//   [40..63]  Longitude (24, signed, LSB = 1°/46603)
//   [64..71]  Ground Speed (8, exp unsigned N=6, unit = 0.25 m/s)
//   [72..85]  Altitude above WGS-84 (14, exp unsigned N=12, +320 m offset)
//   [86..94]  Vertical Rate (9, exp signed N=6, unit = 0.125 m/s)
//   [95..103] Ground Track (9, cyclic, 360°/512 per bit)
//   [104..105] SIL (2)
//   [106..107] DAL (2)
//   [108..111] NIC (4)
//   [112..114] HFOM (3)
//   [115..116] VFOM (2)
//   [117..118] NACv (2)
//   [119]      Reserved (0)
// ─────────────────────────────────────────────────────────────────────────────
static void encode_iconspicuity(uint8_t *payload, const adsl_iconspicuity_t *d) {
    memset(payload, 0, ADSL_PAYLOAD_SIZE);

    // Timestamp: quarter-seconds since full UTC hour, mod 60 (values 60–63 unused)
    pack_bits(payload, 0, 6, d->timestamp_qs < 60 ? d->timestamp_qs : 0);

    pack_bits(payload, 6, 2, (uint32_t)d->flight_state & 0x03u);
    pack_bits(payload, 8, 5, (uint32_t)d->aircraft_category & 0x1Fu);
    pack_bits(payload, 13, 3, (uint32_t)d->emergency & 0x07u);

    // Latitude: 24-bit signed, LSB = 1°/93206
    int32_t lat_i = (int32_t)roundf(d->latitude * 93206.0f);
    if (lat_i < -(1 << 23)) lat_i = -(1 << 23);
    if (lat_i >  (1 << 23) - 1) lat_i = (1 << 23) - 1;
    pack_bits(payload, 16, 24, (uint32_t)(lat_i & 0xFFFFFFu));

    // Longitude: 24-bit signed, LSB = 1°/46603
    int32_t lon_i = (int32_t)roundf(d->longitude * 46603.0f);
    if (lon_i < -(1 << 23)) lon_i = -(1 << 23);
    if (lon_i >  (1 << 23) - 1) lon_i = (1 << 23) - 1;
    pack_bits(payload, 40, 24, (uint32_t)(lon_i & 0xFFFFFFu));

    // Ground Speed: 8-bit exp unsigned N=6, unit = 0.25 m/s; max = 0xFE (236 m/s)
    {
        uint32_t spd_u = (uint32_t)(d->speed_ms / 0.25f + 0.5f);
        uint32_t enc = exp_encode_u(spd_u, 6);
        if (enc > 0xFEu) enc = 0xFEu;  // 0xFF = invalid
        pack_bits(payload, 64, 8, enc);
    }

    // Altitude: 14-bit exp unsigned N=12, +320 m offset; 0x3FFF = invalid
    {
        float alt_offset = d->altitude_wgs84_m + 320.0f;
        if (alt_offset < 0.0f) alt_offset = 0.0f;
        uint32_t alt_u = (uint32_t)(alt_offset + 0.5f);
        uint32_t enc = exp_encode_u(alt_u, 12);
        if (enc > 0x3FFEu) enc = 0x3FFEu;  // 0x3FFF = invalid
        pack_bits(payload, 72, 14, enc);
    }

    // Vertical Rate: 9-bit exp signed N=6, unit = 0.125 m/s; sign in bit 8
    {
        float vr = d->vertical_rate_ms;
        uint32_t sign = (vr < 0.0f) ? 1u : 0u;
        uint32_t mag_u = (uint32_t)(fabsf(vr) / 0.125f + 0.5f);
        uint32_t enc = exp_encode_u(mag_u, 6);
        if (enc > 0xFEu) enc = 0xFEu;  // 0x1FF = invalid
        pack_bits(payload, 86, 9, (sign << 8) | enc);
    }

    // Ground Track: 9-bit cyclic, 360°/512 per bit
    {
        float trk = d->track_deg;
        while (trk <   0.0f) trk += 360.0f;
        while (trk >= 360.0f) trk -= 360.0f;
        uint32_t trk_enc = (uint32_t)(trk * 512.0f / 360.0f + 0.5f) & 0x1FFu;
        pack_bits(payload, 95, 9, trk_enc);
    }

    pack_bits(payload, 104, 2, d->sil & 0x03u);
    pack_bits(payload, 106, 2, d->design_assurance & 0x03u);
    pack_bits(payload, 108, 4, d->nav_integrity & 0x0Fu);
    pack_bits(payload, 112, 3, d->h_accuracy & 0x07u);
    pack_bits(payload, 115, 2, d->v_accuracy & 0x03u);
    pack_bits(payload, 117, 2, d->vel_accuracy & 0x03u);
    // bit 119 = Reserved, stays 0 from memset
}

static void decode_iconspicuity(const uint8_t *payload, adsl_iconspicuity_t *d) {
    d->timestamp_qs       = (uint8_t)unpack_bits(payload, 0, 6);
    d->flight_state       = (adsl_flight_state_t)unpack_bits(payload, 6, 2);
    d->aircraft_category  = (adsl_aircraft_category_t)unpack_bits(payload, 8, 5);
    d->emergency          = (adsl_emergency_t)unpack_bits(payload, 13, 3);

    int32_t lat_i = sign_extend(unpack_bits(payload, 16, 24), 24);
    d->latitude = (float)lat_i / 93206.0f;

    int32_t lon_i = sign_extend(unpack_bits(payload, 40, 24), 24);
    d->longitude = (float)lon_i / 46603.0f;

    uint32_t spd_enc = unpack_bits(payload, 64, 8);
    d->speed_ms = (spd_enc == 0xFFu) ? 0.0f : (float)exp_decode_u(spd_enc, 6) * 0.25f;

    uint32_t alt_enc = unpack_bits(payload, 72, 14);
    d->altitude_wgs84_m = (alt_enc == 0x3FFFu) ? 0.0f
                        : (float)exp_decode_u(alt_enc, 12) - 320.0f;

    uint32_t vr_enc = unpack_bits(payload, 86, 9);
    if (vr_enc == 0x1FFu) {
        d->vertical_rate_ms = 0.0f;
    } else {
        uint32_t sign = (vr_enc >> 8) & 1u;
        float mag = (float)exp_decode_u(vr_enc & 0xFFu, 6) * 0.125f;
        d->vertical_rate_ms = sign ? -mag : mag;
    }

    d->track_deg = (float)unpack_bits(payload, 95, 9) * 360.0f / 512.0f;

    d->sil              = (uint8_t)unpack_bits(payload, 104, 2);
    d->design_assurance = (uint8_t)unpack_bits(payload, 106, 2);
    d->nav_integrity    = (uint8_t)unpack_bits(payload, 108, 4);
    d->h_accuracy       = (uint8_t)unpack_bits(payload, 112, 3);
    d->v_accuracy       = (uint8_t)unpack_bits(payload, 115, 2);
    d->vel_accuracy     = (uint8_t)unpack_bits(payload, 117, 2);
}

// ─────────────────────────────────────────────────────────────────────────────
// Public: Full packet encoder
// ─────────────────────────────────────────────────────────────────────────────
bool adsl_encode_packet(uint8_t *buf, const adsl_iconspicuity_t *data) {
    if (!buf || !data) return false;

    memset(buf, 0, ADSL_PACKET_SIZE);

    uint32_t addr = data->address & 0x00FFFFFFu;
    uint8_t  amt  = data->privacy_mode ? (uint8_t)ADSL_AMT_RANDOM : (data->amt & 0x3Fu);

    // ── Data Link Header ────────────────────────────────────────────────────
    buf[0] = 0x18u;  // Packet Length = 24

    // ── Network Layer (not encrypted) ───────────────────────────────────────
    // Protocol=0, SigFlag=0, KeyIdx=0, ErrorCtrlMode=0 (CRC)
    buf[1] = 0x00u;

    // ── ADS-L Header (buf[2..6]) ────────────────────────────────────────────
    buf[2] = ADSL_PAYLOAD_TYPE_TRAFFIC;           // Payload Type = 0x02
    buf[3] = (uint8_t)((addr >> 16) & 0xFFu);    // Address MSB
    buf[4] = (uint8_t)((addr >>  8) & 0xFFu);
    buf[5] = (uint8_t)( addr        & 0xFFu);    // Address LSB
    buf[6] = (uint8_t)(amt << 2);                // AMT in bits[7:2]; Relay=0, Rsvd=0

    // ── iConspicuity Payload (buf[7..21]) ───────────────────────────────────
    encode_iconspicuity(&buf[7], data);

    // ── XXTEA scramble buf[2..21] as 5 × uint32_t LE words ─────────────────
    static const uint32_t zero_key[4] = {0, 0, 0, 0};
    uint32_t words[ADSL_XXTEA_WORDS];
    memcpy(words, &buf[2], ADSL_DATA_SIZE);       // LE on little-endian MCU
    xxtea_encrypt(words, ADSL_XXTEA_WORDS, zero_key, ADSL_XXTEA_ROUNDS);
    memcpy(&buf[2], words, ADSL_DATA_SIZE);

    // ── CRC-24 over buf[1..21] (Net byte + encrypted ADS-L Data) ────────────
    uint32_t crc = adsl_crc24(&buf[1], ADSL_CRC_COVER_SIZE);
    buf[22] = (uint8_t)((crc >> 16) & 0xFFu);    // MSB first
    buf[23] = (uint8_t)((crc >>  8) & 0xFFu);
    buf[24] = (uint8_t)( crc        & 0xFFu);

    adsl_tx_count++;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Public: Full packet decoder
// ─────────────────────────────────────────────────────────────────────────────
bool adsl_decode_packet(const uint8_t *buf, int len, adsl_iconspicuity_t *data) {
    if (!buf || !data) return false;
    if (len < ADSL_PACKET_SIZE) return false;

    // ── CRC-24 check ─────────────────────────────────────────────────────────
    uint32_t crc_calc = adsl_crc24(&buf[1], ADSL_CRC_COVER_SIZE);
    uint32_t crc_recv = ((uint32_t)buf[22] << 16) |
                        ((uint32_t)buf[23] <<  8) |
                         (uint32_t)buf[24];
    if (crc_calc != crc_recv) return false;

    // ── XXTEA decrypt buf[2..21] ─────────────────────────────────────────────
    static const uint32_t zero_key[4] = {0, 0, 0, 0};
    uint32_t words[ADSL_XXTEA_WORDS];
    memcpy(words, &buf[2], ADSL_DATA_SIZE);
    xxtea_decrypt(words, ADSL_XXTEA_WORDS, zero_key, ADSL_XXTEA_ROUNDS);

    const uint8_t *plain = (const uint8_t *)words;

    // ── Validate Payload Type ────────────────────────────────────────────────
    if (plain[0] != ADSL_PAYLOAD_TYPE_TRAFFIC) return false;

    // ── Extract address and AMT ──────────────────────────────────────────────
    data->address = ((uint32_t)plain[1] << 16) |
                    ((uint32_t)plain[2] <<  8) |
                     (uint32_t)plain[3];
    data->amt = (plain[4] >> 2) & 0x3Fu;
    data->privacy_mode = (data->amt == ADSL_AMT_RANDOM);

    // ── Decode iConspicuity payload ──────────────────────────────────────────
    decode_iconspicuity(&plain[5], data);

    adsl_rx_count++;
    return true;
}
