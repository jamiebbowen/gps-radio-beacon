/**
 * @file altimeter.cpp
 * @brief Commercial altimeter serial decoders (see include/altimeter.h)
 */
#include "include/altimeter.h"
#include <string.h>

void alt_init(alt_state_t *s) { memset(s, 0, sizeof(*s)); }

/* Optional sign, then digits (an optional ".d" tail is skipped). */
static uint8_t parse_int(const char *p, const char *end, int32_t *out)
{
    int32_t sign = 1, v = 0;
    uint8_t digits = 0;
    while (p < end && *p == ' ') p++;
    if (p < end && (*p == '+' || *p == '-')) { if (*p == '-') sign = -1; p++; }
    while (p < end && *p == ' ') p++;
    for (; p < end && *p >= '0' && *p <= '9'; p++) {
        if (v > 9999999) return 0;
        v = v * 10 + (*p - '0');
        digits++;
    }
    if (p < end && *p == '.') { p++; while (p < end && *p >= '0' && *p <= '9') p++; }
    if (!digits || p != end) return 0;
    *out = sign * v;
    return 1;
}

static uint8_t accept(alt_state_t *s, alt_format_t f)
{
    if (s->format != ALT_FMT_UNKNOWN) return s->format == f;
    if (s->candidate == f) {
        if (++s->votes >= ALT_LOCK_VOTES) s->format = f;
    } else {
        s->candidate = f;
        s->votes = 1;
    }
    return 1;
}

static void altitude(alt_state_t *s, int32_t ft)
{
    s->alt_ft = ft;
    if (ft > s->apogee_ft) s->apogee_ft = ft;
}

/* CR/LF-terminated record: RRC3 (comma items) or StratoLogger (bare int) */
static uint8_t decode_line(alt_state_t *s)
{
    const char *b = s->buf, *e = s->buf + s->len;
    const char *f[6];
    uint8_t n = 0;
    f[n++] = b;
    for (const char *p = b; p < e && n < 6; p++) if (*p == ',') f[n++] = p + 1;
    if (n == 1) {
        int32_t ft;
        if (!parse_int(b, e, &ft) || !accept(s, ALT_FMT_STRATOLOGGER)) return 0;
        altitude(s, ft);
        return 1;
    }
    /* RRC3, all data items: timestamp, altitude, velocity, temp, events */
    if (n != 5) return 0;
    int32_t ts, ft, vel, temp;
    if (!parse_int(f[0], f[1] - 1, &ts) || !parse_int(f[1], f[2] - 1, &ft) ||
        !parse_int(f[2], f[3] - 1, &vel) || !parse_int(f[3], f[4] - 1, &temp))
        return 0;
    if (!accept(s, ALT_FMT_RRC3)) return 0;
    altitude(s, ft);
    for (const char *p = f[4]; p < e; p++) {
        if (*p == 'D') s->events |= ALT_EVT_DROGUE;
        else if (*p == 'M') s->events |= ALT_EVT_MAIN;
        else if (*p == 'A') s->events |= ALT_EVT_AUX;
    }
    return 1;
}

/* '>'-terminated Eggtimer element: trigger byte + data */
static uint8_t decode_egg(alt_state_t *s)
{
    if (s->len < 1 || !strchr("#{<(\\@~!=?%^[", s->buf[0])) return 0;
    const char *d = s->buf + 1, *e = s->buf + s->len;
    int32_t v;
    if (s->buf[0] == '<' || s->buf[0] == '%') {
        if (!parse_int(d, e, &v)) return 0;
    }
    if (!accept(s, ALT_FMT_EGGTIMER)) return 0;
    if (s->buf[0] == '<') altitude(s, v);
    else if (s->buf[0] == '%') s->apogee_ft = v;
    else if (s->buf[0] == '~') {
        for (const char *p = d; p < e; p++) {
            if (*p == '1') s->events |= ALT_EVT_DROGUE;
            else if (*p == '2') s->events |= ALT_EVT_MAIN;
            else if (*p >= '3' && *p <= '6') s->events |= ALT_EVT_AUX;
        }
    }
    return 1;
}

uint8_t alt_feed(alt_state_t *s, char c, uint32_t now_ms)
{
    uint8_t egg = (c == '>');
    if (c == '\r' || c == '\n' || egg) {
        uint8_t ok = 0;
        if (s->overflow) s->errors++;
        else if (s->len > 0) {
            ok = egg ? decode_egg(s) : decode_line(s);
            if (ok) { s->records++; s->last_ms = now_ms; }
            else s->errors++;
        }
        s->len = 0;
        s->overflow = 0;
        return ok;
    }
    if (c < 0x20 || c > 0x7E) return 0;          /* line noise */
    if (s->len < ALT_LINE_MAX - 1) s->buf[s->len++] = c;
    else s->overflow = 1;
    return 0;
}

uint8_t alt_status(const alt_state_t *s, uint32_t now_ms)
{
    if (s->format == ALT_FMT_UNKNOWN) return 0;
    uint8_t st = (uint8_t)s->format | s->events;
    if (now_ms - s->last_ms > ALT_STALE_MS) st |= ALT_ST_STALE;
    return st;
}

int16_t alt_rx_sample(alt_rx_t *rx, uint8_t level)
{
    int16_t out = -1;
    level = level ? 1 : 0;
    if (rx->wait_idle) {
        if (level) rx->wait_idle = 0;
    } else if (!rx->active) {
        if (rx->prev && !level) {             /* falling edge: start bit */
            rx->active = 1;
            rx->bit = 0;
            rx->shift = 0;
            rx->count = ALT_RX_OVERSAMPLE / 2;   /* to the start bit centre */
        }
    } else if (--rx->count == 0) {
        rx->count = ALT_RX_OVERSAMPLE;
        if (rx->bit == 0) {
            if (level) rx->active = 0;        /* glitch, not a start bit */
        } else if (rx->bit <= 8) {
            rx->shift = (uint8_t)((rx->shift >> 1) | (level ? 0x80 : 0));
        } else {
            rx->active = 0;
            if (level) out = rx->shift;       /* stop bit high: a byte  */
            else rx->wait_idle = 1;           /* framing error / break  */
        }
        rx->bit++;
    }
    rx->prev = level;
    return out;
}

void alt_ring_put(alt_ring_t *r, uint8_t b)
{
    uint8_t h = r->head;
    if ((uint8_t)(h - r->tail) >= ALT_RING_SIZE) {
        r->dropped = 1;
        return;
    }
    r->buf[h % ALT_RING_SIZE] = b;
    r->head = (uint8_t)(h + 1);
}

int16_t alt_ring_get(alt_ring_t *r)
{
    uint8_t t = r->tail;
    if (t == r->head) return -1;
    int16_t b = r->buf[t % ALT_RING_SIZE];
    r->tail = (uint8_t)(t + 1);
    return b;
}
