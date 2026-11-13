/* test_fuzz_parsers.c - PRNG-driven fuzz over the RF/NMEA wire parsers.
 *
 * Everything the parsers consume is untrusted input in the field sense:
 * packets arrive off the air (noise, foreign beacons, marginal CRC-adjacent
 * frames) and NMEA arrives from a GPS module that can brown out mid-sentence.
 * The unit tests prove intended behavior; this harness hunts the other
 * class - crashes, OOB reads, and UB on inputs nobody wrote down.
 *
 * Structure-aware: most iterations mutate a seed corpus of REAL packets so
 * the deep decode paths get exercised. Pure-random bytes die at the first
 * length/type gate and cover nothing new.
 *
 * Build with -fsanitize=address,undefined -fno-sanitize-recover=all (see
 * Makefile): any sanitizer hit aborts immediately, and the printed seed
 * reproduces the exact run.
 *
 * Env knobs:
 *   FUZZ_ITERS=N   iteration count (default 200000; soak with millions)
 *   FUZZ_SEED=N    xorshift seed (default 0x9E3779B9)
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rf_parser.h"
#include "gps_parser.h"
#include "gps.h"
#include "packet_format.h"

/* ------------------------------------------------------------- stubs */
/* The parsers stamp packets with HAL_GetTick() for staleness checks. A
 * crawling fake tick exercises those time paths too. */
static uint32_t fake_tick;
uint32_t HAL_GetTick(void) { return fake_tick; }
void Test_SetTick(uint32_t t) { fake_tick = t; }

/* ---------------------------------------------------------------- PRNG */
static uint64_t rng_state;
static uint64_t rng_next(void) {
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 7;
    rng_state ^= rng_state << 17;
    return rng_state;
}
static uint32_t rng_range(uint32_t n) { return (uint32_t)(rng_next() % n); }

/* ------------------------------------------------------- seed corpus */
/* Real wire layouts (see packet_format.h). Lengths are the accept-gates
 * the dispatch uses; mutating around them is what reaches the decoders. */

static const uint8_t seed_gps_v2[14] = {
    PACKET_TYPE_GPS,
    0x95, 0xC0, 0x5F, 0x02,              /* lat 39890197 -> 39.89 deg      */
    0x0D, 0x22, 0x68, 0xC1,             /* lon -1048851699 -> -104.88 deg  */
    0x72, 0x06,                          /* alt 1650 m                     */
    0x0A,                                /* sats                           */
    FLAG_LAUNCH_DETECTED | FLAG_FIX_QUALITY_GOOD,
    0x03                                 /* rocket_id                      */
};
static const uint8_t seed_gps_v1[13] = {   /* same, no rocket_id */
    PACKET_TYPE_GPS,
    0x95, 0xC0, 0x5F, 0x02,
    0x0D, 0x22, 0x68, 0xC1,
    0x72, 0x06, 0x0A, 0x40
};
static const uint8_t seed_fused_v3[19] = {
    PACKET_TYPE_FUSED,
    0x95, 0xC0, 0x5F, 0x02,
    0x0D, 0x22, 0x68, 0xC1,
    0x08, 0x52,                          /* alt_qm                         */
    0x10, 0x27, 0x20, 0x03, 0xF0, 0xFF,  /* v_n v_e v_d (cm/s)             */
    FUSED_FLAG_V3 | FUSED_FLAG_GPS_FRESH | FUSED_FLAG_LAUNCH_DETECTED,
    0x03
};
static const uint8_t seed_fused_v2[20] = {
    PACKET_TYPE_FUSED,
    0x95, 0xC0, 0x5F, 0x02,
    0x0D, 0x22, 0x68, 0xC1,
    0x08, 0x52,
    0x10, 0x27, 0x20, 0x03, 0xF0, 0xFF,
    0x00,                                /* age_ds                         */
    0xC0,                                /* flags                          */
    0x03                                 /* rocket_id                      */
};
static const uint8_t seed_fused_v1[19] = {  /* 19B but NO V3 marker bit */
    PACKET_TYPE_FUSED,
    0x95, 0xC0, 0x5F, 0x02,
    0x0D, 0x22, 0x68, 0xC1,
    0x08, 0x52,
    0x10, 0x27, 0x20, 0x03, 0xF0, 0xFF,
    0x00,                                /* age_ds                         */
    0xC0                                 /* flags (bit0 clear -> V1)       */
};
static const uint8_t seed_hb_v3[9] = {
    PACKET_TYPE_HEARTBEAT, 0x03, 0x09, 0x02,
    0x2C, 0x01,                          /* uptime_s                       */
    0x02,                                /* channel                        */
    HB_GPS_HEALTH(HB_GPS_ACQUIRING, 0),
    HB_RESET_POR
};
static const uint8_t seed_hb_v2[8] = {
    PACKET_TYPE_HEARTBEAT, 0x03, 0x09, 0x02, 0x2C, 0x01, 0x02, 0x13
};
static const uint8_t seed_hb_v1[7] = {
    PACKET_TYPE_HEARTBEAT, 0x03, 0x09, 0x02, 0x2C, 0x01, 0x02
};
static const uint8_t seed_imu_v2[18] = {
    PACKET_TYPE_IMU, 0x03, 0xE8, 0x03,
    0xD0, 0x07, 0xE8, 0x03, 0xF4, 0x01,  /* accel xyz cg                   */
    0x0A, 0x00, 0x14, 0x00, 0x1E, 0x00,  /* gyro xyz cds                   */
    0xC4, 0x09                           /* peak accel mg                  */
};
static const uint8_t seed_imu_v1[20] = {
    PACKET_TYPE_IMU, 0x03, 0xE8, 0x03,
    0xD0, 0x07, 0xE8, 0x03, 0xF4, 0x01,
    0x0A, 0x00, 0x14, 0x00, 0x1E, 0x00,
    0xC4, 0x09, 0xD2, 0x00               /* + temp_c10                     */
};
static const uint8_t seed_t0[6] = {
    PACKET_TYPE_LAUNCH_T0, 0x03, 0x2C, 0x01, 0x05, 0x00
};
static const uint8_t seed_evt[7] = {
    PACKET_TYPE_FLIGHT_EVENT, 0x03, 0x01, 0x2C, 0x01, 0x34, 0x12
};
static const uint8_t seed_maxima[11] = {
    PACKET_TYPE_MAXIMA, 0x03,
    0x72, 0x06,                          /* max_alt_m                      */
    0x2C, 0x01,                          /* t_maxalt_s                     */
    0x10, 0x27,                          /* max_speed_cms                  */
    0xE8, 0x03,                          /* max_accel_cg                   */
    0x14                                 /* max_gyro_dps16                 */
};
static const uint8_t seed_hello_v2[6] = {
    PACKET_TYPE_HELLO, 0x03, 0x78, 0x56, 0x34, 0x12
};
static const uint8_t seed_hello_v1[8] = {
    PACKET_TYPE_HELLO, 0x03, 0x78, 0x56, 0x34, 0x12, 0x05, 0x00
};
static const uint8_t seed_ack[6] = {
    PACKET_TYPE_ACK, 0x03, 0x01, 0x02, 0x42, 0x00
};

static const char *const seed_ascii[] = {
    "CALL-7 CH3",
    "40.0586,-105.2145,1650.5,8",
    "40.0586,-105.2145,1650.5",
    "-0.0000001,179.9999999,-50,1",
    "CALL-255 CH14",
};
static const char *const seed_nmea[] = {
    "$GPGGA,123519,4003.516,N,10512.870,W,1,08,0.9,1650.5,M,-21.3,M,,*47",
    "$GPRMC,123519,A,4003.516,N,10512.870,W,022.4,084.4,230394,003.1,W*6A",
    "$GNGGA,000000,4003.516,S,17959.999,E,6,12,1.0,0.0,M,0.0,M,,*47",
    "$GPGGA,,,,,,0,00,99.9,,,,,,*48",
    "$GPGLL,4003.516,N,10512.870,W,123519,A,A*52",
};

/* ------------------------------------------------------------- mutate */
#define FUZZ_BUF_MAX 384

static uint16_t mutate_binary(const uint8_t *seed, uint16_t seed_len,
                              uint8_t *out) {
    uint16_t len = seed_len;
    memcpy(out, seed, seed_len);

    uint32_t op = rng_range(8);
    switch (op) {
    case 0: case 1: case 2:                  /* byte flips (common) */
        for (uint32_t i = 0, n = 1 + rng_range(8); i < n; i++)
            out[rng_range(len)] = (uint8_t)rng_next();
        break;
    case 3:                                  /* truncate */
        len = (uint16_t)rng_range(len + 1);
        break;
    case 4:                                  /* extend with random tail */
        for (uint32_t i = 0, n = rng_range(24); i < n && len < FUZZ_BUF_MAX; i++)
            out[len++] = (uint8_t)rng_next();
        break;
    case 5:                                  /* insert a byte */
        if (len < FUZZ_BUF_MAX) {
            uint16_t at = (uint16_t)rng_range(len + 1);
            memmove(out + at + 1, out + at, len - at);
            out[at] = (uint8_t)rng_next();
            len++;
        }
        break;
    case 6:                                  /* whole random buffer */
        len = (uint16_t)rng_range(64);
        for (uint16_t i = 0; i < len; i++) out[i] = (uint8_t)rng_next();
        break;
    case 7:                                  /* bit flips (single bits) */
        for (uint32_t i = 0, n = 1 + rng_range(8); i < n; i++)
            out[rng_range(len)] ^= (uint8_t)(1u << rng_range(8));
        break;
    }
    return len;
}

static void mutate_string(const char *seed, char *out, size_t cap) {
    size_t len = strlen(seed);
    if (len >= cap) len = cap - 1;
    memcpy(out, seed, len + 1);

    switch (rng_range(6)) {
    case 0: case 1:                          /* byte flips */
        for (uint32_t i = 0, n = 1 + rng_range(6); i < n; i++) {
            if (len) out[rng_range((uint32_t)len)] = (char)(32 + rng_range(95));
        }
        break;
    case 2:                                  /* truncate */
        out[rng_range((uint32_t)len + 1)] = '\0';
        break;
    case 3:                                  /* extend with random chars */
        while (len < cap - 1 && rng_range(4))
            out[len++] = (char)(32 + rng_range(95));
        out[len] = '\0';
        break;
    case 4:                                  /* separators/controls */
        for (uint32_t i = 0, n = 1 + rng_range(6); i < n; i++) {
            if (len) {
                static const char nasty[] = ",$*!\r\n\t\\";
                out[rng_range((uint32_t)len)] = nasty[rng_range(9)];
            }
        }
        break;
    case 5:                                  /* all-random string */
        len = rng_range((uint32_t)cap - 1);
        for (size_t i = 0; i < len; i++) out[i] = (char)(1 + rng_range(126));
        out[len] = '\0';
        break;
    }
}

/* -------------------------------------------------------------- drive */
int main(void) {
    uint32_t iters = 200000;
    const char *e = getenv("FUZZ_ITERS");
    if (e && atoi(e) > 0) iters = (uint32_t)atoi(e);
    rng_state = 0x9E3779B9ULL;
    if ((e = getenv("FUZZ_SEED")) && strtoull(e, NULL, 0))
        rng_state = strtoull(e, NULL, 0);

    printf("fuzz: iters=%lu seed=0x%llX\n", (unsigned long)iters,
           (unsigned long long)rng_state);

    static const struct { const uint8_t *p; uint16_t n; } bin[] = {
        {seed_gps_v2, sizeof seed_gps_v2},   {seed_gps_v1, sizeof seed_gps_v1},
        {seed_fused_v3, sizeof seed_fused_v3},{seed_fused_v2, sizeof seed_fused_v2},
        {seed_fused_v1, sizeof seed_fused_v1},{seed_hb_v3, sizeof seed_hb_v3},
        {seed_hb_v2, sizeof seed_hb_v2},     {seed_hb_v1, sizeof seed_hb_v1},
        {seed_imu_v2, sizeof seed_imu_v2},   {seed_imu_v1, sizeof seed_imu_v1},
        {seed_t0, sizeof seed_t0},           {seed_evt, sizeof seed_evt},
        {seed_maxima, sizeof seed_maxima},   {seed_hello_v2, sizeof seed_hello_v2},
        {seed_hello_v1, sizeof seed_hello_v1},{seed_ack, sizeof seed_ack},
    };
    enum { N_BIN = sizeof bin / sizeof bin[0],
           N_ASCII = sizeof seed_ascii / sizeof seed_ascii[0],
           N_NMEA = sizeof seed_nmea / sizeof seed_nmea[0] };

    uint8_t buf[FUZZ_BUF_MAX];
    char sbuf[384];
    GPS_Data gps;

    for (uint32_t i = 0; i < iters; i++) {
        uint32_t target = rng_range(10);
        if (target < 5) {                    /* binary packet path (50%) */
            uint32_t s = rng_range(N_BIN);
            uint16_t len = mutate_binary(bin[s].p, bin[s].n, buf);
            RF_Parser_ParseBinaryPacket(buf, len);
        } else if (target < 7) {             /* fused entry point (20%)  */
            uint32_t s = 2 + rng_range(3);   /* the three fused seeds    */
            uint16_t len = mutate_binary(bin[s].p, bin[s].n, buf);
            RF_Parser_ParseFusedPacket(buf, len);
        } else if (target < 9) {             /* ASCII/callsign (20%)     */
            mutate_string(seed_ascii[rng_range(N_ASCII)], sbuf, sizeof sbuf);
            RF_Parser_ParseAsciiPacket(sbuf);
        } else {                             /* local-GPS NMEA (10%)     */
            mutate_string(seed_nmea[rng_range(N_NMEA)], sbuf, sizeof sbuf);
            GPS_ParseNMEA(sbuf, &gps);
        }
        /* Draining getters too: their buffers hold whatever the parse left */
        if ((i & 0x3FF) == 0) {
            char gb[260];
            RF_Parser_GetParsedData(&gps, gb, sizeof gb, NULL);
            RF_Parser_GetLastRawPacket(gb, sizeof gb);
            RF_Parser_GetLastValidPacket(gb, sizeof gb);
        }
    }
    printf("fuzz: %lu iterations clean\n", (unsigned long)iters);
    return 0;
}
