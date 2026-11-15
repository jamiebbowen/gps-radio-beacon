/**
 * @file altimeter.h
 * @brief Decoders for the commercial altimeters on the carrier's serial
 *        inputs (J2/J5/J6: 5 V logic through 10k/20k dividers to D13/D12/
 *        D11), 9600 8N1 ASCII. Each input recognises its format from the
 *        data:
 *  - PerfectFlite StratoLogger CF telemetry: one altitude (ft) per line,
 *    CR LF, ~20 Hz in flight;
 *  - Missile Works RRC3 telemetry, all data items:
 *    "TTTT.T,+AAAAA,+VVVV,+TTT,DMA" CR (events: lowercase = continuity,
 *    UPPERCASE = fired, '-' = none);
 *  - Eggtimer telemetry: trigger byte + data + '>' elements every 2 s
 *    ('<' altitude ft, '%' apogee ft, '~' channel states, ...).
 */
#ifndef ALTIMETER_H
#define ALTIMETER_H

#include <stdint.h>

typedef enum {
    ALT_FMT_UNKNOWN = 0,
    ALT_FMT_STRATOLOGGER,
    ALT_FMT_RRC3,
    ALT_FMT_EGGTIMER
} alt_format_t;

#define ALT_EVT_DROGUE  0x01   /* drogue / apogee charge fired */
#define ALT_EVT_MAIN    0x02   /* main charge fired            */
#define ALT_EVT_AUX     0x04   /* any other channel fired      */

#define ALT_LINE_MAX    48     /* longest record kept (RRC3 all items: 27) */
#define ALT_LOCK_VOTES  3      /* consistent records before the format locks */

typedef struct {
    alt_format_t format;       /* locked format, UNKNOWN until ALT_LOCK_VOTES */
    alt_format_t candidate;
    uint8_t  votes;
    int32_t  alt_ft;           /* latest altitude, as reported          */
    int32_t  apogee_ft;        /* reported apogee, else the max seen    */
    uint8_t  events;           /* ALT_EVT_* fired so far (latched)      */
    uint32_t last_ms;          /* time of the newest decoded record     */
    uint32_t records;
    uint32_t errors;           /* overlong or undecodable records       */
    char     buf[ALT_LINE_MAX];
    uint8_t  len;
    uint8_t  overflow;
} alt_state_t;

void    alt_init(alt_state_t *s);

/** Feed one received byte. @retval 1 when a record was decoded */
uint8_t alt_feed(alt_state_t *s, char c, uint32_t now_ms);

#endif /* ALTIMETER_H */
