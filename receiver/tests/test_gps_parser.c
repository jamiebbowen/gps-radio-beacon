/**
 * @file test_gps_parser.c
 * @brief Host-side unit tests for the receiver's local GPS NMEA parser.
 *
 * Covers GPS_ParseNMEA: sentence framing, XOR checksum validation, GGA/RMC
 * field extraction (any talker prefix), fix gating, coordinate validation
 * and the impossible-jump rejection filter.
 *
 * NOTE: gps_parser.c keeps static "last valid position" state for jump
 * detection with no reset API, so tests share that state. All valid-fix
 * fixtures therefore use the same area (48N, 11E) and the jump test runs
 * knowing a prior position has been established.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "gps_parser.h"
#include "test_harness.h"

/* Include the module under test directly (not linked - see tests/Makefile)
 * so tests can reach static helpers and exercise their defensive guards. */
#include "../firmware/src/gps_parser.c"

/* ------------------------------------------------------------------ */
/* Sentence builder                                                    */
/* ------------------------------------------------------------------ */

/**
 * Append the correct "*HH" NMEA checksum to a "$..." body.
 * Guarantees tests never fail because of a hand-computed checksum typo.
 */
static const char *make_sentence(char *out, size_t out_size, const char *body) {
    uint8_t cs = 0;
    for (const char *p = body + 1; *p; p++) {  /* XOR between '$' and '*' */
        cs ^= (uint8_t)*p;
    }
    snprintf(out, out_size, "%s*%02X", body, cs);
    return out;
}

/* Shared fixture area: 48 deg 07.038' N, 011 deg 31.000' E (Munich-ish) */
#define FIX_LAT (48.0 + 7.038 / 60.0)    /* 48.1173  */
#define FIX_LON (11.0 + 31.000 / 60.0)   /* 11.51667 */

/* ------------------------------------------------------------------ */
/* Framing / validation tests                                          */
/* ------------------------------------------------------------------ */

TEST(test_rejects_null_short_and_unframed) {
    GPS_Data gps;
    memset(&gps, 0, sizeof(gps));

    CHECK(GPS_ParseNMEA(NULL, &gps) == GPS_PARSER_ERROR);
    CHECK(GPS_ParseNMEA("$GPGGA", &gps) == GPS_PARSER_ERROR);          /* short   */
    CHECK(GPS_ParseNMEA("GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,,*00",
                        &gps) == GPS_PARSER_ERROR);                     /* no '$'  */
    CHECK(GPS_ParseNMEA("$GPGGA,123519,4807.038", &gps) == GPS_PARSER_ERROR); /* <3 commas */
}

TEST(test_rejects_missing_or_bad_checksum) {
    GPS_Data gps;
    memset(&gps, 0, sizeof(gps));
    uint32_t errors_before = gps_checksum_errors;

    /* No '*HH' at all */
    CHECK(GPS_ParseNMEA("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,",
                        &gps) == GPS_PARSER_ERROR);

    /* Deliberately wrong checksum */
    CHECK(GPS_ParseNMEA("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*00",
                        &gps) == GPS_PARSER_ERROR);
    CHECK(gps_checksum_errors == errors_before + 1);
}

TEST(test_rejects_unknown_sentence_type) {
    GPS_Data gps;
    char s[128];
    memset(&gps, 0, sizeof(gps));

    /* Valid checksum but a type this parser doesn't handle */
    make_sentence(s, sizeof(s), "$GPGSV,3,1,11,03,03,111,00,04,15,270,00");
    CHECK(GPS_ParseNMEA(s, &gps) == GPS_PARSER_ERROR);
}

/* ------------------------------------------------------------------ */
/* GGA parsing                                                         */
/* ------------------------------------------------------------------ */

TEST(test_gga_valid_sentence) {
    GPS_Data gps;
    char s[128];
    memset(&gps, 0, sizeof(gps));

    make_sentence(s, sizeof(s),
        "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
    Test_SetTick(5000);

    CHECK(GPS_ParseNMEA(s, &gps) == GPS_PARSER_OK);
    CHECK_NEAR(gps.latitude, FIX_LAT, 1e-4);
    CHECK_NEAR(gps.longitude, FIX_LON, 1e-4);
    CHECK_NEAR(gps.altitude, 545.4, 1e-2);
    CHECK(gps.satellites == 8);
    CHECK(gps.fix == 1);
    CHECK(gps.hour == 12 && gps.minute == 35 && gps.second == 19);
    CHECK(gps.timestamp == 5000);
    CHECK_NEAR(gps.hdop, 0.9, 1e-3);
}

TEST(test_gga_accepts_gn_talker) {
    /* u-blox multi-GNSS modules emit $GNGGA - must be accepted too */
    GPS_Data gps;
    char s[128];
    memset(&gps, 0, sizeof(gps));

    make_sentence(s, sizeof(s),
        "$GNGGA,123520,4807.040,N,01131.002,E,2,12,0.8,545.0,M,46.9,M,,");
    CHECK(GPS_ParseNMEA(s, &gps) == GPS_PARSER_OK);
    CHECK(gps.fix == 2);
    CHECK(gps.satellites == 12);
}

TEST(test_gga_numeric_fields_clamped) {
    GPS_Data gps;
    char s[128];

    /* A corrupt-but-checksum-valid sats field must saturate, not wrap
     * modulo 256 at the uint8_t store. */
    memset(&gps, 0, sizeof(gps));
    make_sentence(s, sizeof(s),
        "$GPGGA,123519,4807.038,N,01131.000,E,1,999,0.9,545.4,M,46.9,M,,");
    CHECK(GPS_ParseNMEA(s, &gps) == GPS_PARSER_OK);
    CHECK(gps.satellites == 99);
    CHECK(gps.fix == 1);

    /* Out-of-spec fix quality (0-8 only) reads as no-fix: rejected. */
    memset(&gps, 0, sizeof(gps));
    make_sentence(s, sizeof(s),
        "$GPGGA,123519,4807.038,N,01131.000,E,9,08,0.9,545.4,M,46.9,M,,");
    CHECK(GPS_ParseNMEA(s, &gps) == GPS_PARSER_ERROR);
    CHECK(gps.fix == 0);
}

TEST(test_gga_southern_western_hemisphere) {
    GPS_Data gps;
    char s[128];
    memset(&gps, 0, sizeof(gps));

    /* The fixture area mirrored S/W: drop the jump-filter anchor first
     * (48N -> 48S would be a teleport), and again afterwards so the next
     * test re-anchors at the shared fixture area. Rejected positions are
     * never committed, so the signs are checked on an accepted fix. */
    has_last_valid_position = 0;
    jump_candidate_valid = 0;
    make_sentence(s, sizeof(s),
        "$GPGGA,123521,4807.038,S,01131.000,W,1,08,0.9,545.4,M,46.9,M,,");
    CHECK(GPS_ParseNMEA(s, &gps) == GPS_PARSER_OK);
    CHECK_NEAR(gps.latitude, -FIX_LAT, 1e-4);
    CHECK_NEAR(gps.longitude, -FIX_LON, 1e-4);
    has_last_valid_position = 0;
}

TEST(test_gga_nan_fields_never_reach_gps_data) {
    /* atof() parses "nan"/"inf" literally - a checksum-valid corrupt field
     * must degrade to "unknown" (0), never NaN inside GPS_Data. "nan.5"
     * keeps a '.' at idx>=3 so it reaches the converter's atof path and
     * would hit the UB (int) cast without the isfinite guard. */
    GPS_Data gps;
    char s[128];
    memset(&gps, 0, sizeof(gps));

    make_sentence(s, sizeof(s),
        "$GPGGA,123519,4807.038,N,01131.000,E,1,08,nan,nan.5,M,46.9,M,,");
    CHECK(GPS_ParseNMEA(s, &gps) == GPS_PARSER_OK);
    CHECK(gps.hdop == 0.0f && gps.hdop == gps.hdop);      /* not NaN */
    CHECK(gps.altitude == 0.0f && gps.altitude == gps.altitude);

    /* NaN coordinate: converts to 0.0, flagged, rejected by validation. */
    memset(&gps, 0, sizeof(gps));
    uint32_t errs_before = gps_invalid_coordinate_errors;
    make_sentence(s, sizeof(s),
        "$GPGGA,123519,nan.5,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
    CHECK(GPS_ParseNMEA(s, &gps) == GPS_PARSER_ERROR);
    CHECK(gps.latitude == gps.latitude);                  /* not NaN */
    CHECK(gps_invalid_coordinate_errors > errs_before);
}

TEST(test_gga_no_fix_rejected) {
    GPS_Data gps;
    char s[128];
    memset(&gps, 0, sizeof(gps));

    /* fix quality 0 - parser must reject even with plausible coordinates */
    make_sentence(s, sizeof(s),
        "$GPGGA,123522,4807.038,N,01131.000,E,0,03,5.0,545.4,M,46.9,M,,");
    CHECK(GPS_ParseNMEA(s, &gps) == GPS_PARSER_ERROR);
}

/* ------------------------------------------------------------------ */
/* RMC parsing                                                         */
/* ------------------------------------------------------------------ */

TEST(test_rmc_valid_sentence) {
    GPS_Data gps;
    char s[128];
    memset(&gps, 0, sizeof(gps));

    make_sentence(s, sizeof(s),
        "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230324,003.1,W");
    CHECK(GPS_ParseNMEA(s, &gps) == GPS_PARSER_OK);
    CHECK_NEAR(gps.latitude, FIX_LAT, 1e-4);
    CHECK_NEAR(gps.longitude, FIX_LON, 1e-4);
    CHECK_NEAR(gps.speed, 22.4 * 1.852, 1e-2);   /* knots -> km/h */
    CHECK_NEAR(gps.course, 84.4, 1e-2);
    CHECK(gps.day == 23 && gps.month == 3 && gps.year == 2024);
    CHECK(gps.fix == 1);
}

TEST(test_rmc_void_status_rejected) {
    GPS_Data gps;
    char s[128];
    memset(&gps, 0, sizeof(gps));

    /* Status 'V' = void/no fix */
    make_sentence(s, sizeof(s),
        "$GPRMC,123519,V,4807.038,N,01131.000,E,022.4,084.4,230324,003.1,W");
    CHECK(GPS_ParseNMEA(s, &gps) == GPS_PARSER_ERROR);
}

/* ------------------------------------------------------------------ */
/* Coordinate validation / jump filter                                 */
/* ------------------------------------------------------------------ */

TEST(test_rejected_fix_is_not_committed) {
    /* The parser used to write lat/lon/fix into the caller's struct BEFORE
     * validating, so a rejected teleport still overwrote the position and
     * left fix=1 for GPS_IsFixed()/main.c to pick up. */
    GPS_Data gps;
    char s[128];
    memset(&gps, 0, sizeof(gps));
    has_last_valid_position = 0;
    jump_candidate_valid = 0;
    make_sentence(s, sizeof(s),
        "$GPGGA,123530,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
    CHECK(GPS_ParseNMEA(s, &gps) == GPS_PARSER_OK);
    make_sentence(s, sizeof(s),
        "$GPGGA,123531,5007.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
    CHECK(GPS_ParseNMEA(s, &gps) == GPS_PARSER_ERROR);
    CHECK_NEAR(gps.latitude, FIX_LAT, 1e-4);     /* position untouched */
    CHECK_NEAR(gps.longitude, FIX_LON, 1e-4);
    CHECK(gps.fix == 0);                         /* not a usable fix */
}

TEST(test_relocation_reanchors_after_two_agreeing_fixes) {
    /* Powering up 200 km from the last anchor (drove to the launch site
     * with the fix lost) used to be rejected forever: the filter only
     * moved its anchor on an ACCEPTED fix. Two consecutive fixes that
     * agree with each other at the new site now re-anchor it. */
    GPS_Data gps;
    char s[128];
    memset(&gps, 0, sizeof(gps));
    has_last_valid_position = 0;
    jump_candidate_valid = 0;
    make_sentence(s, sizeof(s),
        "$GPGGA,123540,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
    CHECK(GPS_ParseNMEA(s, &gps) == GPS_PARSER_OK);
    make_sentence(s, sizeof(s),
        "$GPGGA,123541,5007.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
    CHECK(GPS_ParseNMEA(s, &gps) == GPS_PARSER_ERROR);   /* first: suspect */
    make_sentence(s, sizeof(s),
        "$GPGGA,123542,5007.100,N,01131.050,E,1,08,0.9,546.0,M,46.9,M,,");
    CHECK(GPS_ParseNMEA(s, &gps) == GPS_PARSER_OK);      /* agrees: accept */
    CHECK_NEAR(gps.latitude, 50.0 + 7.100 / 60.0, 1e-4);
    CHECK(gps.fix == 1);
    has_last_valid_position = 0;                         /* leave no anchor */
    jump_candidate_valid = 0;
}

TEST(test_prime_meridian_longitude_accepted) {
    /* Longitudes between 0 and +-1 deg were rejected as "parsing errors" -
     * a receiver in London could never get a fix. */
    GPS_Data gps;
    char s[128];
    memset(&gps, 0, sizeof(gps));
    has_last_valid_position = 0;
    jump_candidate_valid = 0;
    make_sentence(s, sizeof(s),
        "$GPGGA,123550,5130.000,N,00007.500,W,1,08,0.9,20.0,M,46.9,M,,");
    CHECK(GPS_ParseNMEA(s, &gps) == GPS_PARSER_OK);
    CHECK_NEAR(gps.longitude, -0.125, 1e-4);
    has_last_valid_position = 0;
    jump_candidate_valid = 0;
}

TEST(test_jump_filter_rejects_teleport) {
    GPS_Data gps;
    char s[128];
    memset(&gps, 0, sizeof(gps));
    uint32_t errors_before = gps_invalid_coordinate_errors;

    /* Anchor the filter at the fixture area */
    make_sentence(s, sizeof(s),
        "$GPGGA,123523,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
    CHECK(GPS_ParseNMEA(s, &gps) == GPS_PARSER_OK);

    /* ~220 km jump north (2 deg latitude) - physically impossible in 1 s */
    make_sentence(s, sizeof(s),
        "$GPGGA,123524,5007.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
    CHECK(GPS_ParseNMEA(s, &gps) == GPS_PARSER_ERROR);
    CHECK(gps_invalid_coordinate_errors == errors_before + 1);

    /* A plausible small movement is still accepted afterwards */
    make_sentence(s, sizeof(s),
        "$GPGGA,123525,4807.100,N,01131.050,E,1,08,0.9,546.0,M,46.9,M,,");
    CHECK(GPS_ParseNMEA(s, &gps) == GPS_PARSER_OK);
}

TEST(test_malformed_coordinate_strings_rejected) {
    GPS_Data gps;
    char s[128];
    memset(&gps, 0, sizeof(gps));
    uint32_t errors_before = gps_invalid_coordinate_errors;

    /* Minutes >= 60 is invalid NMEA - conversion yields 0 -> COORD ERR */
    make_sentence(s, sizeof(s),
        "$GPGGA,123526,4899.999,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
    CHECK(GPS_ParseNMEA(s, &gps) == GPS_PARSER_ERROR);

    /* Non-numeric garbage in the latitude field */
    make_sentence(s, sizeof(s),
        "$GPGGA,123527,48AB.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
    CHECK(GPS_ParseNMEA(s, &gps) == GPS_PARSER_ERROR);

    CHECK(gps_invalid_coordinate_errors > errors_before);
}

TEST(test_tiny_sentence_rejected) {
    GPS_Data gps;
    memset(&gps, 0, sizeof(gps));

    /* A 3-char sentence can't hold a type: the parse must fail */
    CHECK(GPS_ParseNMEA("$A,", &gps) == GPS_PARSER_ERROR);
    CHECK(gps.fix == 0);
}

TEST(test_rmc_southern_western_hemisphere) {
    GPS_Data gps;
    char s[128];
    memset(&gps, 0, sizeof(gps));

    /* Same approach as the GGA S/W test: no anchor, accepted fix */
    has_last_valid_position = 0;
    jump_candidate_valid = 0;
    make_sentence(s, sizeof(s),
        "$GPRMC,123519,A,4807.038,S,01131.000,W,022.4,084.4,230324,003.1,W");
    CHECK(GPS_ParseNMEA(s, &gps) == GPS_PARSER_OK);
    CHECK_NEAR(gps.latitude, -FIX_LAT, 1e-4);
    CHECK_NEAR(gps.longitude, -FIX_LON, 1e-4);
    has_last_valid_position = 0;
}

TEST(test_coordinate_without_decimal_point_rejected) {
    GPS_Data gps;
    char s[128];
    memset(&gps, 0, sizeof(gps));
    uint32_t errors_before = gps_invalid_coordinate_errors;

    /* No '.' in the latitude field at all */
    make_sentence(s, sizeof(s),
        "$GPGGA,123528,4807038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
    CHECK(GPS_ParseNMEA(s, &gps) == GPS_PARSER_ERROR);
    CHECK(gps_invalid_coordinate_errors > errors_before);
}

TEST(test_coordinate_all_zeroes_rejected) {
    GPS_Data gps;
    char s[128];
    memset(&gps, 0, sizeof(gps));
    uint32_t errors_before = gps_invalid_coordinate_errors;

    /* "0000.000" converts to exactly 0.0 — a parse failure, not a fix */
    make_sentence(s, sizeof(s),
        "$GPGGA,123529,0000.000,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
    CHECK(GPS_ParseNMEA(s, &gps) == GPS_PARSER_ERROR);
    CHECK(gps_invalid_coordinate_errors > errors_before);
}

TEST(test_degrees_above_180_rejected) {
    GPS_Data gps;
    char s[128];
    memset(&gps, 0, sizeof(gps));
    uint32_t errors_before = gps_invalid_coordinate_errors;

    /* 181 degrees of longitude is not a real place */
    make_sentence(s, sizeof(s),
        "$GPGGA,123530,4807.038,N,18131.000,E,1,08,0.9,545.4,M,46.9,M,,");
    CHECK(GPS_ParseNMEA(s, &gps) == GPS_PARSER_ERROR);
    CHECK(gps_invalid_coordinate_errors > errors_before);
}

TEST(test_decimal_result_above_180_rejected) {
    GPS_Data gps;
    char s[128];
    memset(&gps, 0, sizeof(gps));
    uint32_t errors_before = gps_invalid_coordinate_errors;

    /* 180° + 59.999' = 180.99998° — passes the degree/minute checks but
     * exceeds the valid range once combined. */
    make_sentence(s, sizeof(s),
        "$GPGGA,123531,4807.038,N,18059.999,E,1,08,0.9,545.4,M,46.9,M,,");
    CHECK(GPS_ParseNMEA(s, &gps) == GPS_PARSER_ERROR);
    CHECK(gps_invalid_coordinate_errors > errors_before);
}

TEST(test_latitude_out_of_range_rejected) {
    GPS_Data gps;
    char s[128];
    memset(&gps, 0, sizeof(gps));
    uint32_t errors_before = gps_invalid_coordinate_errors;

    /* 95°N converts cleanly but is outside ±90° */
    make_sentence(s, sizeof(s),
        "$GPGGA,123532,9500.000,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
    CHECK(GPS_ParseNMEA(s, &gps) == GPS_PARSER_ERROR);
    CHECK(gps_invalid_coordinate_errors > errors_before);
}

TEST(test_subdegree_longitude_dropped_digit_caught) {
    GPS_Data gps;
    char s[128];
    memset(&gps, 0, sizeof(gps));

    /* 0.5 deg E right after a Munich fix is almost certainly a dropped
     * digit ("01131" -> "00030"). Sub-degree longitudes are legal (London),
     * so it is the ~820 km jump from the anchor that rejects it. */
    has_last_valid_position = 0;
    jump_candidate_valid = 0;
    make_sentence(s, sizeof(s),
        "$GPGGA,123532,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
    CHECK(GPS_ParseNMEA(s, &gps) == GPS_PARSER_OK);
    uint32_t errors_before = gps_invalid_coordinate_errors;
    make_sentence(s, sizeof(s),
        "$GPGGA,123533,4807.038,N,00030.000,E,1,08,0.9,545.4,M,46.9,M,,");
    CHECK(GPS_ParseNMEA(s, &gps) == GPS_PARSER_ERROR);
    CHECK(gps_invalid_coordinate_errors > errors_before);
    CHECK_NEAR(gps.longitude, FIX_LON, 1e-4);    /* not committed */
    jump_candidate_valid = 0;
}

TEST(test_origin_coordinates_rejected) {
    GPS_Data gps;
    char s[128];
    memset(&gps, 0, sizeof(gps));
    uint32_t errors_before = gps_invalid_coordinate_errors;

    /* (0.008°N, 0.0°E) — the "no fix yet" all-zero position off the
     * west African coast.  The all-zero longitude also trips the
     * zero-value conversion check, so the counter moves by two. */
    make_sentence(s, sizeof(s),
        "$GPGGA,123534,0000.500,N,00000.000,E,1,08,0.9,545.4,M,46.9,M,,");
    CHECK(GPS_ParseNMEA(s, &gps) == GPS_PARSER_ERROR);
    CHECK(gps_invalid_coordinate_errors >= errors_before + 2);
}

TEST(test_coordinate_helper_defensive_guards) {
    uint32_t before = gps_invalid_coordinate_errors;

    /* Too short to be ddmm ("12"): rejected with the error counter bumped.
     * Unreachable through ParseNMEA - the token-length >= 4 guards in both
     * sub-parsers filter shorter fields first - but kept as insurance. */
    CHECK(GPS_NMEAStringToDecimalDegrees("12") == 0.0f);
    CHECK(gps_invalid_coordinate_errors == before + 1);

    /* NULL / empty input: the else-branch guard */
    CHECK(GPS_NMEAStringToDecimalDegrees("") == 0.0f);
    CHECK(GPS_NMEAStringToDecimalDegrees(NULL) == 0.0f);
    CHECK(gps_invalid_coordinate_errors == before + 3);
}

TEST(test_checksum_helper_defensive_guards) {
    /* ParseNMEA verifies '$' framing before calling the validator, so
     * these can only arrive via a future direct caller. */
    CHECK(GPS_ValidateChecksum(NULL) == 0);
    CHECK(GPS_ValidateChecksum("GPGGA,123,x*44") == 0);  /* no '$' */
}

TEST(test_truncated_checksum_rejected) {
    GPS_Data gps;
    char s[128];
    memset(&gps, 0, sizeof(gps));
    uint32_t errors_before = gps_checksum_errors;

    /* '*' with only one hex digit after it — present but unusable */
    snprintf(s, sizeof(s), "%s", "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*4");
    CHECK(GPS_ParseNMEA(s, &gps) == GPS_PARSER_ERROR);
    CHECK(gps_checksum_errors == errors_before + 1);
}

/* ------------------------------------------------------------------ */

int main(void) {
    run_test_rejects_null_short_and_unframed();
    run_test_rejects_missing_or_bad_checksum();
    run_test_rejects_unknown_sentence_type();
    run_test_tiny_sentence_rejected();
    run_test_gga_valid_sentence();
    run_test_gga_accepts_gn_talker();
    run_test_gga_numeric_fields_clamped();
    run_test_gga_southern_western_hemisphere();
    run_test_gga_nan_fields_never_reach_gps_data();
    run_test_gga_no_fix_rejected();
    run_test_rmc_valid_sentence();
    run_test_rmc_void_status_rejected();
    run_test_rmc_southern_western_hemisphere();
    run_test_jump_filter_rejects_teleport();
    run_test_rejected_fix_is_not_committed();
    run_test_relocation_reanchors_after_two_agreeing_fixes();
    run_test_prime_meridian_longitude_accepted();
    run_test_malformed_coordinate_strings_rejected();
    run_test_coordinate_without_decimal_point_rejected();
    run_test_coordinate_all_zeroes_rejected();
    run_test_degrees_above_180_rejected();
    run_test_decimal_result_above_180_rejected();
    run_test_latitude_out_of_range_rejected();
    run_test_subdegree_longitude_dropped_digit_caught();
    run_test_origin_coordinates_rejected();
    run_test_truncated_checksum_rejected();
    run_test_coordinate_helper_defensive_guards();
    run_test_checksum_helper_defensive_guards();

    return TEST_SUMMARY();
}
