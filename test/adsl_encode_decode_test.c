/*
 * Standalone host-side test for adsl_encode_packet() / adsl_decode_packet().
 * Compile:  gcc -o adsl_test test/adsl_encode_decode_test.c lib/ADSL/AdslProtocol.cpp -I lib/ADSL -lm
 * Run:      ./adsl_test
 */

#define _USE_MATH_DEFINES
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

/* Stub ESP32 log macros so AdslProtocol.cpp compiles on host */
#define log_i(fmt, ...) printf("[I] " fmt "\n", ##__VA_ARGS__)
#define log_e(fmt, ...) printf("[E] " fmt "\n", ##__VA_ARGS__)
#define log_w(fmt, ...) printf("[W] " fmt "\n", ##__VA_ARGS__)

#include "AdslProtocol.h"

/* ── global counters required by AdslProtocol.cpp ──────────────────────── */
uint32_t adsl_rx_count = 0;
uint32_t adsl_tx_count = 0;

/* ── helpers ────────────────────────────────────────────────────────────── */
static void hexdump(const char *label, const uint8_t *buf, int len) {
    printf("  %-12s ", label);
    for (int i = 0; i < len; i++) printf("%02X ", buf[i]);
    printf("\n");
}

static float pct_err(float expected, float got) {
    if (fabsf(expected) < 1e-6f) return fabsf(got);
    return fabsf((got - expected) / expected) * 100.0f;
}

#define CHECK_F(field, tol_pct)                                              \
    do {                                                                     \
        float _e = pct_err(in.field, out.field);                            \
        if (_e > (tol_pct)) {                                               \
            printf("  FAIL  %-25s in=%.4f  out=%.4f  err=%.2f%%\n",        \
                   #field, (double)in.field, (double)out.field, (double)_e); \
            fail++;                                                          \
        } else {                                                             \
            printf("  OK    %-25s in=%.4f  out=%.4f\n",                     \
                   #field, (double)in.field, (double)out.field);            \
        }                                                                    \
    } while (0)

#define CHECK_U(field)                                                        \
    do {                                                                      \
        if (in.field != out.field) {                                          \
            printf("  FAIL  %-25s in=%u  out=%u\n",                          \
                   #field, (unsigned)in.field, (unsigned)out.field);          \
            fail++;                                                            \
        } else {                                                               \
            printf("  OK    %-25s in=%u  out=%u\n",                           \
                   #field, (unsigned)in.field, (unsigned)out.field);          \
        }                                                                      \
    } while (0)

int main(void) {
    printf("=== ADS-L encode/decode round-trip test ===\n\n");
    int fail = 0;

    /* ── Mock input data ───────────────────────────────────────────────── */
    adsl_iconspicuity_t in;
    memset(&in, 0, sizeof(in));

    in.address           = 0xABCDEF;           /* 24-bit FANET address   */
    in.amt               = ADSL_AMT_FANET;     /* 8                      */
    in.latitude          = 47.123456f;         /* approx Innsbruck area  */
    in.longitude         = 11.654321f;
    in.altitude_wgs84_m  = 1500.0f;            /* 1500 m                 */
    in.speed_ms          = 12.5f;              /* 45 km/h paraglider     */
    in.vertical_rate_ms  = 2.0f;              /* climbing 2 m/s         */
    in.track_deg         = 270.0f;            /* west                   */
    in.timestamp_qs      = 47;               /* quarter-secs mod 60    */
    in.flight_state      = ADSL_STATE_AIRBORNE;
    in.aircraft_category = ADSL_CAT_HANG_PARA; /* paraglider             */
    in.emergency         = ADSL_EMERG_NONE;
    in.sil               = 0;
    in.design_assurance  = 0;
    in.nav_integrity     = 0;
    in.h_accuracy        = 0;
    in.v_accuracy        = 0;
    in.vel_accuracy      = 0;
    in.privacy_mode      = false;

    /* ── Encode ─────────────────────────────────────────────────────────── */
    uint8_t buf[ADSL_PACKET_SIZE];
    memset(buf, 0xFF, sizeof(buf));

    bool enc_ok = adsl_encode_packet(buf, &in);
    printf("adsl_encode_packet: %s\n", enc_ok ? "OK" : "FAIL");
    if (!enc_ok) { printf("ENCODE FAILED — aborting\n"); return 1; }
    hexdump("encoded:", buf, ADSL_PACKET_SIZE);
    printf("  buf[0]=0x%02X (expect 0x18)\n", buf[0]);
    printf("  buf[1]=0x%02X (expect 0x00 net)\n", buf[1]);
    printf("  buf[22..24] CRC24: %02X %02X %02X\n\n", buf[22], buf[23], buf[24]);

    /* ── Decode ─────────────────────────────────────────────────────────── */
    adsl_iconspicuity_t out;
    memset(&out, 0, sizeof(out));

    bool dec_ok = adsl_decode_packet(buf, ADSL_PACKET_SIZE, &out);
    printf("adsl_decode_packet: %s\n\n", dec_ok ? "OK" : "FAIL");
    if (!dec_ok) { printf("DECODE FAILED — CRC or payload type mismatch\n"); return 1; }

    /* ── Field comparison ───────────────────────────────────────────────── */
    printf("--- Field comparison (in vs decoded out) ---\n");

    /* address */
    if (in.address != out.address) {
        printf("  FAIL  address                   in=0x%06X  out=0x%06X\n",
               in.address, out.address);  fail++;
    } else {
        printf("  OK    address                   0x%06X\n", in.address);
    }
    CHECK_U(amt);
    CHECK_F(latitude,          0.001f);   /* ≈10 cm at mid-lat */
    CHECK_F(longitude,         0.001f);
    CHECK_F(altitude_wgs84_m,  1.0f);     /* 1% = 15 m tolerance */
    CHECK_F(speed_ms,          5.0f);     /* 5% */
    CHECK_F(vertical_rate_ms,  5.0f);
    CHECK_F(track_deg,         1.5f);     /* ~5 degrees tolerance */
    CHECK_U(timestamp_qs);
    CHECK_U(flight_state);
    CHECK_U(aircraft_category);
    CHECK_U(emergency);

    printf("\n=== %s  (%d failure(s)) ===\n\n",
           fail == 0 ? "ALL PASS" : "FAILURES DETECTED", fail);

    /* ── CRC corruption test ─────────────────────────────────────────────── */
    printf("--- CRC corruption test ---\n");
    uint8_t corrupt[ADSL_PACKET_SIZE];
    memcpy(corrupt, buf, ADSL_PACKET_SIZE);
    corrupt[5] ^= 0xFF;   /* flip address byte */
    adsl_iconspicuity_t dummy;
    bool bad = adsl_decode_packet(corrupt, ADSL_PACKET_SIZE, &dummy);
    printf("  Corrupted packet rejected: %s\n\n", bad ? "FAIL (accepted!)" : "OK");
    if (bad) fail++;

    return fail;
}
