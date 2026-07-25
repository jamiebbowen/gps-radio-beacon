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

int main(void) {
    run_test_field_extraction_by_position();
    run_test_empty_fields_do_not_shift_positions();
    run_test_field_terminators();
    run_test_field_out_of_range_and_bad_args();
    run_test_field_truncation();
    run_test_coord_conversion();
    run_test_coord_conversion_edge_cases();
    run_test_coord_conversion_precision();

    return TEST_SUMMARY();
}
