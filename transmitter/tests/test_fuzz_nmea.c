/* test_fuzz_nmea.c - PRNG-driven fuzz over the TX-side NMEA helpers.
 *
 * The helpers parse whatever the GPS module emits - including the garbage
 * a brown-out module produces mid-sentence. Unit tests prove the intended
 * behavior; this hunts crashes/UB/OOB on inputs nobody wrote down.
 *
 * Built with -fsanitize=address,undefined -fno-sanitize-recover=all: any
 * hit aborts with the printed seed for reproduction.
 *
 *   FUZZ_ITERS=N   iteration count (default 200000)
 *   FUZZ_SEED=N    xorshift seed
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int   nmea_get_field(const char *sentence, uint8_t index,
                     char *out, uint8_t out_size);
float nmea_coord_to_decimal(const char *nmea_coord, char direction);
int   nmea_checksum_ok(const char *sentence);

static uint64_t rng_state;
static uint64_t rng_next(void) {
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 7;
    rng_state ^= rng_state << 17;
    return rng_state;
}
static uint32_t rng_range(uint32_t n) { return (uint32_t)(rng_next() % n); }

static const char *const seeds[] = {
    "$GPGGA,123519,4003.516,N,10512.870,W,1,08,0.9,1650.5,M,-21.3,M,,*47",
    "$GPRMC,123519,A,4003.516,N,10512.870,W,022.4,084.4,230394,003.1,W*6A",
    "$GNGGA,000000,4003.516,S,17959.999,E,6,12,1.0,0.0,M,0.0,M,,*47",
    "$GPGGA,,,,,,0,00,99.9,,,,,,*48",
    "$PUBX,00,123519,4003.516,N,10512.870,W,1650.5,G3,2.1,2.0,0.0,0.0,0.0,8,0.0,0.0,0.0*00",
    "4003.516",
    "N",
    "",
    "$",
    ",,,,,,,,,,",
};

#define CAP 384
static void mutate(const char *seed, char *out) {
    size_t len = strlen(seed);
    if (len >= CAP) len = CAP - 1;
    memcpy(out, seed, len + 1);
    switch (rng_range(6)) {
    case 0: case 1:
        for (uint32_t i = 0, n = 1 + rng_range(8); i < n; i++)
            if (len) out[rng_range((uint32_t)len)] = (char)(32 + rng_range(95));
        break;
    case 2:
        out[rng_range((uint32_t)len + 1)] = '\0';
        break;
    case 3:
        while (len < CAP - 1 && rng_range(4))
            out[len++] = (char)(32 + rng_range(95));
        out[len] = '\0';
        break;
    case 4:
        for (uint32_t i = 0, n = 1 + rng_range(8); i < n; i++) {
            if (len) {
                static const char nasty[] = ",$*!\r\n\t\\-.NSWE0123456789";
                out[rng_range((uint32_t)len)] = nasty[rng_range(20)];
            }
        }
        break;
    case 5:
        len = rng_range(CAP - 1);
        for (size_t i = 0; i < len; i++) out[i] = (char)(1 + rng_range(126));
        out[len] = '\0';
        break;
    }
}

int main(void) {
    uint32_t iters = 200000;
    const char *e = getenv("FUZZ_ITERS");
    if (e && atoi(e) > 0) iters = (uint32_t)atoi(e);
    rng_state = 0xC0FFEEULL;
    if ((e = getenv("FUZZ_SEED")) && strtoull(e, NULL, 0))
        rng_state = strtoull(e, NULL, 0);

    printf("fuzz: iters=%lu seed=0x%llX\n", (unsigned long)iters,
           (unsigned long long)rng_state);

    char buf[CAP];
    char field[64];
    for (uint32_t i = 0; i < iters; i++) {
        mutate(seeds[rng_range(sizeof seeds / sizeof seeds[0])], buf);
        switch (rng_range(3)) {
        case 0:
            nmea_get_field(buf, (uint8_t)rng_range(24),
                           field, (uint8_t)(1 + rng_range((uint32_t)sizeof field)));
            break;
        case 1:
            nmea_checksum_ok(buf);
            break;
        case 2: {
            static const char dirs[] = "NSEWXnsew";
            nmea_coord_to_decimal(buf, dirs[rng_range(9)]);
            break;
        }
        }
    }
    printf("fuzz: %lu iterations clean\n", (unsigned long)iters);
    return 0;
}
