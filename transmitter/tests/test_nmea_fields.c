/**
 * @file test_nmea_fields.c
 * @brief Host-side unit tests for the transmitter's NMEA field helpers.
 *
 * nmea_get_field is the regression fix for the strtok_r empty-field-collapse
 * bug: a no-fix GGA sentence has empty lat/lon fields, and strtok_r silently
 * shifted every later field left. These tests pin down absolute-position
 * indexing, empty-field preservation, truncation, and terminator handling,
 * plus the shared NMEA coordinate -> decimal-degree conversion.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "include/nmea_fields.h"
#include "test_harness.h"

/* Representative full-fix GGA sentence */
static const char *GGA_FIX =
    "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";

/* No-fix GGA: lat/lon/dir/hdop/alt all EMPTY - the strtok_r killer */
static const char *GGA_NO_FIX =
    "$GPGGA,123519,,,,,0,00,,,M,,M,,*66";

/* ------------------------------------------------------------------ */
/* nmea_get_field: basic extraction                                    */
/* ------------------------------------------------------------------ */

TEST(test_field_extraction_by_position) {
    char f[16];

    CHECK(nmea_get_field(GGA_FIX, 0, f, sizeof(f)) == 6);
    CHECK(strcmp(f, "$GPGGA") == 0);

    CHECK(nmea_get_field(GGA_FIX, 2, f, sizeof(f)) > 0);
    CHECK(strcmp(f, "4807.038") == 0);              /* latitude    */

    CHECK(nmea_get_field(GGA_FIX, 5, f, sizeof(f)) == 1);
    CHECK(strcmp(f, "E") == 0);                     /* lon dir     */

    CHECK(nmea_get_field(GGA_FIX, 6, f, sizeof(f)) == 1);
    CHECK(strcmp(f, "1") == 0);                     /* fix quality */

    CHECK(nmea_get_field(GGA_FIX, 7, f, sizeof(f)) == 2);
    CHECK(strcmp(f, "08") == 0);                    /* satellites  */

    CHECK(nmea_get_field(GGA_FIX, 9, f, sizeof(f)) > 0);
    CHECK(strcmp(f, "545.4") == 0);                 /* altitude    */
}

TEST(test_empty_fields_do_not_shift_positions) {
    /* THE regression test: with empty lat/lon, field 6 must still be the
     * fix quality ("0") and field 7 the satellite count ("00"). Under the
     * old strtok_r parser these landed at indices 2 and 3. */
    char f[16];

    CHECK(nmea_get_field(GGA_NO_FIX, 2, f, sizeof(f)) == 0);  /* empty lat */
    CHECK(f[0] == '\0');
    CHECK(nmea_get_field(GGA_NO_FIX, 4, f, sizeof(f)) == 0);  /* empty lon */

    CHECK(nmea_get_field(GGA_NO_FIX, 6, f, sizeof(f)) == 1);
    CHECK(strcmp(f, "0") == 0);                     /* fix quality = 0 */

    CHECK(nmea_get_field(GGA_NO_FIX, 7, f, sizeof(f)) == 2);
    CHECK(strcmp(f, "00") == 0);                    /* sats = 00       */

    CHECK(nmea_get_field(GGA_NO_FIX, 9, f, sizeof(f)) == 0); /* empty alt */
}

TEST(test_field_terminators) {
    char f[16];

    /* Body ends at '*' - checksum must not leak into the last field */
    CHECK(nmea_get_field("$GPRMC,120000,A*7B", 2, f, sizeof(f)) == 1);
    CHECK(strcmp(f, "A") == 0);

    /* Body ends at CRLF */
    CHECK(nmea_get_field("$GPRMC,120000,A\r\n", 2, f, sizeof(f)) == 1);
    CHECK(strcmp(f, "A") == 0);

    /* Requesting a field past the '*' terminator fails cleanly */
    CHECK(nmea_get_field("$GPRMC,120000,A*7B", 3, f, sizeof(f)) == -1);
}

TEST(test_field_out_of_range_and_bad_args) {
    char f[16];

    CHECK(nmea_get_field(GGA_FIX, 50, f, sizeof(f)) == -1);   /* no field */
    CHECK(nmea_get_field(NULL, 0, f, sizeof(f)) == -1);       /* NULL in  */
    CHECK(nmea_get_field(GGA_FIX, 0, NULL, 8) == -1);         /* NULL out */
    CHECK(nmea_get_field(GGA_FIX, 0, f, 0) == -1);            /* no room  */
}

TEST(test_field_truncation) {
    char tiny[4];

    /* "4807.038" truncated into a 4-byte buffer -> "480" + NUL */
    CHECK(nmea_get_field(GGA_FIX, 2, tiny, sizeof(tiny)) == 3);
    CHECK(strcmp(tiny, "480") == 0);
    CHECK(tiny[3] == '\0');
}

/* ------------------------------------------------------------------ */
/* nmea_coord_to_decimal                                               */
/* ------------------------------------------------------------------ */

TEST(test_coord_conversion) {
    /* 48 deg 07.038' = 48.1173 */
    CHECK_NEAR(nmea_coord_to_decimal("4807.038", 'N'), 48.1173, 1e-4);
    CHECK_NEAR(nmea_coord_to_decimal("4807.038", 'S'), -48.1173, 1e-4);
    /* 105 deg 06.93605' = 105.115601 */
    CHECK_NEAR(nmea_coord_to_decimal("10506.93605", 'E'), 105.1156008, 1e-5);
    CHECK_NEAR(nmea_coord_to_decimal("10506.93605", 'W'), -105.1156008, 1e-5);
}

TEST(test_coord_conversion_edge_cases) {
    CHECK_NEAR(nmea_coord_to_decimal(NULL, 'N'), 0.0f, 1e-9);
    CHECK_NEAR(nmea_coord_to_decimal("", 'N'), 0.0f, 1e-9);
    /* Equator/prime-meridian style small values */
    CHECK_NEAR(nmea_coord_to_decimal("0000.100", 'N'), 0.1 / 60.0, 1e-6);
}

TEST(test_coord_conversion_rejects_malformed) {
    /* atof() parses these literally; the old code cast the result straight
     * to int (UB for NaN/Inf/huge) - now every one reports NaN, which all
     * callers reject via isfinite(). */
    CHECK(isnan(nmea_coord_to_decimal("nan", 'N')));
    CHECK(isnan(nmea_coord_to_decimal("inf", 'E')));
    CHECK(isnan(nmea_coord_to_decimal("1e30", 'N')));
    CHECK(isnan(nmea_coord_to_decimal("999999999999", 'E')));  /* 12 digits */
    CHECK(isnan(nmea_coord_to_decimal("18100.0", 'E')));       /* > 180 deg */
    CHECK(isnan(nmea_coord_to_decimal("-3953.40284", 'N')));   /* sign is dir */
    /* 60 minutes is not a carry into the next degree - corrupt field */
    CHECK(isnan(nmea_coord_to_decimal("3960.0", 'N')));
    /* Legal extremes still convert */
    CHECK_NEAR(nmea_coord_to_decimal("18000.0", 'W'), -180.0, 1e-4);
    CHECK_NEAR(nmea_coord_to_decimal("3959.99999", 'N'), 39.99999983, 1e-4);
}

TEST(test_coord_conversion_precision) {
    /* Full-precision NMEA coordinate: verify sub-meter accuracy survives.
     * 3953.40284 -> 39 + 53.40284/60 = 39.89004733... The internal math is
     * double, but the return type is float, so the best achievable error is
     * ULP(39.89)/2 = ~1.9e-6 deg (~20 cm). A pure-float pipeline compounds
     * rounding at every step and is only good to ~1e-5 deg (~1 m). */
    double expected = 39.0 + 53.40284 / 60.0;
    CHECK_NEAR(nmea_coord_to_decimal("3953.40284", 'N'), expected, 2e-6);
}

/* ------------------------------------------------------------------ */
/* nmea_checksum_ok                                                    */
/* ------------------------------------------------------------------ */

TEST(test_checksum_valid_accepted) {
    /* Real on-wire sentences with genuine checksums (recomputed) */
    CHECK(nmea_checksum_ok(
        "$GPGGA,123519,3953.40284,N,10453.11007,W,1,08,0.9,1655.4,M,46.9,M,,*66") == 1);
    CHECK(nmea_checksum_ok(
        "$GNRMC,123519,A,3954.00000,N,10454.00000,W,022.4,084.4,230394,,*10") == 1);
}

TEST(test_checksum_rejects_corruption) {
    /* One flipped payload digit: the classic "silent position shift" */
    CHECK(nmea_checksum_ok(
        "$GPGGA,123519,3953.40284,N,10453.11007,W,1,08,0.9,1655.4,M,46.9,M,,*67") == 0);
    /* One corrupted body byte, original checksum kept */
    CHECK(nmea_checksum_ok(
        "$GPGGA,123519,4953.40284,N,10453.11007,W,1,08,0.9,1655.4,M,46.9,M,,*66") == 0);
}

TEST(test_checksum_rejects_missing_or_malformed) {
    /* No '*' at all: truncated/corrupt - NMEA requires the checksum */
    CHECK(nmea_checksum_ok("$GPGGA,123519,3953.40284,N,10453.11007,W") == 0);
    /* '*' present but no hex digits */
    CHECK(nmea_checksum_ok("$GPGGA,123519*") == 0);
    CHECK(nmea_checksum_ok("$GPGGA,123519*G") == 0);
    CHECK(nmea_checksum_ok("$GPGGA,123519*ZZ") == 0);
    /* first hex digit fine, second missing or not hex */
    CHECK(nmea_checksum_ok("$GPGGA,123519*4") == 0);
    CHECK(nmea_checksum_ok("$GPGGA,123519*4G") == 0);
    /* '*' as the very last byte of an exact-size heap buffer: the checker
     * must stop at the NUL instead of reading the second hex digit past
     * it (`make sanitize` turns the old over-read into a hard failure). */
    {
        const char *src = "$GPGGA,123519*";
        char *exact = (char *)malloc(strlen(src) + 1);
        strcpy(exact, src);
        CHECK(nmea_checksum_ok(exact) == 0);
        free(exact);
    }
    /* Not an NMEA sentence */
    CHECK(nmea_checksum_ok(NULL) == 0);
    CHECK(nmea_checksum_ok("") == 0);
    CHECK(nmea_checksum_ok("GPGGA,123519*66") == 0);  /* missing '$' */
}

TEST(test_checksum_crlf_and_case) {
    /* Compute a real checksum inline so the fixture can't rot */
    const char *body = "GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M";
    uint8_t ck = 0;
    for (const char *p = body; *p; p++) ck ^= (uint8_t)*p;
    char s[96];
    snprintf(s, sizeof(s), "$%s*%02X\r\n", body, ck);
    CHECK(nmea_checksum_ok(s) == 1);
    snprintf(s, sizeof(s), "$%s*%02x\r\n", body, ck);  /* lowercase hex ok */
    CHECK(nmea_checksum_ok(s) == 1);
}

TEST(test_u8_field_clamps_and_rejects) {
    CHECK(nmea_u8_field("12", 99) == 12);
    CHECK(nmea_u8_field("0", 99) == 0);
    CHECK(nmea_u8_field("99", 99) == 99);
    CHECK(nmea_u8_field("100", 99) == 99);        /* saturates at cap */
    CHECK(nmea_u8_field("999", 99) == 99);        /* would wrap to 231 raw */
    CHECK(nmea_u8_field("999999999999", 99) == 99); /* strtol saturates */
    CHECK(nmea_u8_field("-5", 99) == 0);          /* negative -> 0 */
    CHECK(nmea_u8_field("abc", 99) == 0);         /* non-numeric -> 0 */
    CHECK(nmea_u8_field("", 99) == 0);            /* empty -> 0 */
    CHECK(nmea_u8_field(NULL, 99) == 0);
    CHECK(nmea_u8_field("9", 8) == 8);            /* fix-quality cap */
    CHECK(nmea_u8_field("12abc", 99) == 12);      /* parses prefix */
}

/* ------------------------------------------------------------------ */

int main(void) {
    run_test_field_extraction_by_position();
    run_test_empty_fields_do_not_shift_positions();
    run_test_field_terminators();
    run_test_field_out_of_range_and_bad_args();
    run_test_field_truncation();
    run_test_coord_conversion();
    run_test_coord_conversion_edge_cases();
    run_test_coord_conversion_rejects_malformed();
    run_test_coord_conversion_precision();
    run_test_checksum_valid_accepted();
    run_test_checksum_rejects_corruption();
    run_test_checksum_rejects_missing_or_malformed();
    run_test_checksum_crlf_and_case();
    run_test_u8_field_clamps_and_rejects();

    return TEST_SUMMARY();
}
