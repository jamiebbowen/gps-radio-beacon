/*
 * Pure NMEA field-extraction helpers. See include/nmea_fields.h for the
 * rationale (fixes strtok_r empty-field collapse) and API documentation.
 *
 * Keep this file free of Arduino / hardware includes - it is unit-tested
 * on the host (transmitter/tests/test_nmea_fields.c).
 */

#include "include/nmea_fields.h"
#include <string.h>
#include <stdlib.h>

int nmea_get_field(const char *sentence, uint8_t index,
                   char *out, uint8_t out_size) {
    if (sentence == NULL || out == NULL || out_size == 0) {
        return -1;
    }

    /* Walk to the start of the requested field */
    const char *p = sentence;
    uint8_t field = 0;
    while (field < index) {
        /* Advance to the next comma or end-of-body */
        while (*p && *p != ',' && *p != '*' && *p != '\r' && *p != '\n') {
            p++;
        }
        if (*p != ',') {
            out[0] = '\0';
            return -1;  /* sentence ended before the requested field */
        }
        p++;  /* skip the comma */
        field++;
    }

    /* Copy the field body (may be empty) */
    uint8_t n = 0;
    while (*p && *p != ',' && *p != '*' && *p != '\r' && *p != '\n'
           && n < (uint8_t)(out_size - 1)) {
        out[n++] = *p++;
    }
    out[n] = '\0';
    return (int)n;
}

float nmea_coord_to_decimal(const char *nmea_coord, char direction) {
    if (nmea_coord == NULL || nmea_coord[0] == '\0') {
        return 0.0f;
    }

    /* ddmm.mmmmm: degrees are everything above the hundreds place, minutes
     * are the remainder. Done in double so the ~9 significant digits of an
     * NMEA coordinate survive until the final rounding to float. */
    double nmea_value = atof(nmea_coord);
    int degrees = (int)(nmea_value / 100.0);
    double minutes = nmea_value - (degrees * 100.0);
    double decimal = (double)degrees + (minutes / 60.0);

    if (direction == 'S' || direction == 'W') {
        decimal = -decimal;
    }

    return (float)decimal;
}

static int hex_val(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

int nmea_checksum_ok(const char *sentence) {
    if (sentence == NULL || sentence[0] != '$') {
        return 0;
    }

    /* XOR of all bytes between '$' and '*' */
    uint8_t cksum = 0;
    const char *p = sentence + 1;
    while (*p && *p != '*' && *p != '\r' && *p != '\n') {
        cksum ^= (uint8_t)*p++;
    }
    if (*p != '*') {
        return 0;  /* no checksum field: truncated or corrupt */
    }

    /* Short-circuit: when '*' is the last character p[1] is the NUL
     * terminator, and reading p[2] would run past the end of the string. */
    int hi = hex_val(p[1]);
    if (hi < 0) {
        return 0;
    }
    int lo = hex_val(p[2]);
    if (lo < 0) {
        return 0;
    }
    return (uint8_t)((hi << 4) | lo) == cksum;
}

uint8_t nmea_u8_field(const char *s, uint8_t cap)
{
    /* strtol saturates on overflow (atoi() is UB out of int range) and the
     * [0, cap] clamp keeps corrupt fields like "999" from wrapping at a
     * uint8_t cast into nonsense values on the wire. */
    if (!s) return 0;
    char *endp;
    long v = strtol(s, &endp, 10);
    if (endp == s || v <= 0) return 0;
    return (v > (long)cap) ? cap : (uint8_t)v;
}
