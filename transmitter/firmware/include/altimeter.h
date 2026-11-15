/**
 * @file altimeter.h
 * @brief Decoders for the commercial altimeters on the carrier's serial
 *        inputs (J2/J5/J6: 5 V logic through 10k/20k dividers to D13/D12/
 *        D11), 9600 8N1 ASCII. Each input recognises its format from the
 *        data:
 *  - PerfectFlite StratoLogger CF telemetry: one altitude (ft) per line,
 *    CR LF, ~20 Hz in flight;
 *  - Missile Works RRC3 telemetry, all data items:
 *    "TTTT.T,+AAAAA,+VVVV,+TTT,DMA" CR, ",X.X" battery volts appended on
 *    v1.60+ (events: UPPERCASE while the output fires; v1.5x shows '-'
 *    before and lowercase after, v1.6x '?' no continuity and lowercase
 *    continuity);
 *  - Eggtimer telemetry: trigger byte + data + '>' elements every 2 s
 *    ('{' altitude/100 ft - '<' feet on the Quark pre-D3 -, '%' apogee
 *    ft after nose-over, '~' channel states, ...).
 */
#ifndef ALTIMETER_H
#define ALTIMETER_H

#include <stdint.h>
#include "packet_format.h"     /* ALT_ST_*: the wire/flash status encoding */

typedef enum {
    ALT_FMT_UNKNOWN      = 0,
    ALT_FMT_STRATOLOGGER = ALT_ST_FMT_STRATOLOGGER,
    ALT_FMT_RRC3         = ALT_ST_FMT_RRC3,
    ALT_FMT_EGGTIMER     = ALT_ST_FMT_EGGTIMER
} alt_format_t;

#define ALT_EVT_DROGUE  ALT_ST_DROGUE   /* drogue / apogee charge fired */
#define ALT_EVT_MAIN    ALT_ST_MAIN     /* main charge fired            */
#define ALT_EVT_AUX     ALT_ST_AUX      /* any other channel fired      */
#define ALT_STALE_MS    5000u           /* slowest stream (Eggtimer): 2 s */

#define ALT_LINE_MAX    48     /* longest record kept (RRC3 all items: 27) */
#define ALT_LOCK_VOTES  3      /* consistent records before the format locks */

typedef struct {
    alt_format_t format;       /* locked format, UNKNOWN until ALT_LOCK_VOTES */
    alt_format_t candidate;
    uint8_t  votes;
    int32_t  alt_ft;           /* latest altitude, as reported          */
    int32_t  apogee_ft;        /* reported apogee, else the max seen    */
    uint8_t  events;           /* ALT_EVT_* fired so far (latched)      */
    uint8_t  rrc3_dash;        /* RRC3 channels that showed '-' (v1.5x) */
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

/** The beacon's launch: what was reported on the pad is not this flight's
 *  (a StratoLogger 'OnPad' stream opens with the site elevation MSL; a
 *  charge latched on the pad). Apogee restarts at the current altitude and
 *  the fired events clear; the recognised format stays locked. */
void    alt_flight_start(alt_state_t *s);

/** ALT_ST_* status: format, fired events, stale (no record for
 *  ALT_STALE_MS). 0 while no format is recognised. */
uint8_t alt_status(const alt_state_t *s, uint32_t now_ms);

/* ---- Software UART receiver (8N1), one line sample per call ----------
 * Sampled ALT_RX_OVERSAMPLE times per bit by a timer interrupt. A start
 * bit needs a falling edge, so an unconnected input (held low by the
 * carrier's 20k pull-down) never produces bytes; a frame whose stop bit is
 * low is dropped and the receiver waits for the line to idle high. */
#define ALT_BAUD            9600u
#define ALT_RX_OVERSAMPLE   4u

typedef struct {
    uint8_t active;     /* inside a frame                         */
    uint8_t wait_idle;  /* after a framing error: wait for high   */
    uint8_t prev;       /* previous sample (edge detection)       */
    uint8_t count;      /* samples to the next bit centre         */
    uint8_t bit;        /* 0 = start, 1..8 = data, 9 = stop       */
    uint8_t shift;
} alt_rx_t;

/** @retval 0..255 a received byte, -1 none (or a dropped frame) */
int16_t alt_rx_sample(alt_rx_t *rx, uint8_t level);

/* ---- Byte ring, one interrupt-side producer, one loop-side consumer -- */
#define ALT_RING_SIZE 128u   /* 133 ms at full 9600 baud                 */

typedef struct {
    volatile uint8_t head, tail;   /* free-running, masked on access     */
    volatile uint8_t dropped;      /* bytes lost to a full ring (sticky) */
    uint8_t buf[ALT_RING_SIZE];
} alt_ring_t;

void    alt_ring_put(alt_ring_t *r, uint8_t b);
int16_t alt_ring_get(alt_ring_t *r);

#endif /* ALTIMETER_H */
