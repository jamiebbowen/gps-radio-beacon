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
