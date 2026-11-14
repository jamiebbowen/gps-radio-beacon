#ifndef NMEA_FIELDS_H
#define NMEA_FIELDS_H

/*
 * Pure NMEA field-extraction helpers (no Arduino / hardware dependencies,
 * host-unit-testable - see transmitter/tests/test_nmea_fields.c).
 *
 * Motivation: the previous parser used strtok_r, which COLLAPSES consecutive
 * delimiters. A no-fix GGA sentence like
 *     $GPGGA,123519,,,,,0,00,,,M,,M,,
 * has empty lat/lon fields, so strtok_r silently shifted every later field
 * left and the field counter no longer matched the NMEA spec positions.
 * These helpers index fields by absolute position, preserving empty fields.
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Extract field `index` from an NMEA sentence by absolute position.
 *
 * Field 0 is the sentence type ("$GPGGA"), fields are comma-separated and
 * the sentence body ends at '*', '\r', '\n' or NUL. Empty fields are valid
 * and yield an empty string.
 *
 * @param sentence  NMEA sentence (need not contain a checksum)
 * @param index     0-based field index
 * @param out       destination buffer (always NUL-terminated on success)
 * @param out_size  size of out in bytes (must be >= 1)
 * @return number of characters written (field may legally be empty -> 0),
 *         or -1 if arguments are invalid or the field does not exist.
 *         Fields longer than out_size-1 are truncated (return = copied len).
 */
int nmea_get_field(const char *sentence, uint8_t index,
                   char *out, uint8_t out_size);

/**
 * Convert an NMEA ddmm.mmmmm coordinate string to decimal degrees.
 *
 * @param nmea_coord coordinate string, e.g. "3953.40284" or "10506.93605"
 * @param direction  'N'/'E' = positive, 'S'/'W' = negative
 * @return decimal degrees; 0.0f for NULL/empty input; NAN for a malformed
 *         field (non-finite, negative, >= 18100, or minutes >= 60), which
 *         callers reject with isfinite()
 */
double nmea_coord_to_decimal(const char *nmea_coord, char direction);

/**
 * Validate the trailing *XX checksum of an NMEA sentence.
 *
 * The checksum is the XOR of all bytes between '$' and '*', expressed as
 * two uppercase-or-lowercase hex digits after '*'. NMEA-0183 requires the
 * checksum on every emitted sentence, so a sentence without one is
 * truncated/corrupt and is rejected here (returns 0). The GPS UART feeds
 * us whatever bytes survived the wire - a single flipped bit in a digit
 * silently moves the reported position hundreds of meters, which is worse
 * than dropping the sentence.
 *
 * @param sentence NUL-terminated sentence beginning with '$' (may include
 *                 the trailing \r\n - it is ignored)
 * @return 1 if a checksum is present and matches, 0 otherwise.
 */
int nmea_checksum_ok(const char *sentence);

/**
 * Parse an NMEA numeric field into a uint8_t with saturation.
 *
 * Uses strtol (atoi() on an out-of-int-range field is undefined) and
 * clamps to [0, cap]: a corrupt-but-checksum-valid field like "999" must
 * not wrap modulo 256 at a uint8_t cast and read as nonsense on the wire.
 *
 * @param s   field text (NULL/empty/non-numeric -> 0)
 * @param cap maximum accepted value; larger inputs saturate to cap
 * @return parsed value clamped to [0, cap]
 */
uint8_t nmea_u8_field(const char *s, uint8_t cap);

#ifdef __cplusplus
}
#endif

#endif /* NMEA_FIELDS_H */
