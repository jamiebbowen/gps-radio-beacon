/**
 * @file eink_summary.c
 * @brief Content of the e-ink summary page (see eink_summary.h)
 */
#include "eink_summary.h"
#include "eink.h"        /* EINK_BLACK / EINK_RED */
#include "lora.h"        /* LORA_CHANNEL_FREQ_MHZ */
#include <stdio.h>
#include <stdarg.h>

static void put(EinkSummary_Line_t *out, uint8_t *n, uint16_t x, uint16_t y,
                uint8_t size, uint8_t color, const char *fmt, ...)
{
  if (*n >= EINK_SUMMARY_MAX_LINES) return;
  EinkSummary_Line_t *l = &out[(*n)++];
  l->x = x;
  l->y = y;
  l->size = size;
  l->color = color;
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(l->text, sizeof(l->text), fmt, ap);
  va_end(ap);
}

uint8_t EinkSummary_Compose(const EinkSummary_In_t *in, EinkSummary_Line_t *out)
{
  uint8_t n = 0;
  /* Link age from any packet of our beacon: on the pad its heartbeat comes
   * once a minute and a position only every 10 minutes. */
  uint32_t age_s = (in->heard_tick > 0) ? (in->now_ms - in->heard_tick) / 1000u : 0;
  uint8_t link_lost = (in->heard_tick == 0) || (age_s > EINK_SUMMARY_LOST_S);
  const BeaconTrack_t *bt = in->beacon;

  put(out, &n, 4, 2, 2, EINK_BLACK, "GPS BEACON RX");
  put(out, &n, 4, 38, 1, EINK_BLACK, "CH%u %.2fMHz  rssi %ddBm",
      (unsigned)in->channel, LORA_CHANNEL_FREQ_MHZ(in->channel), (int)in->rssi);
  put(out, &n, 4, 50, 1, EINK_BLACK, "age %lus  pkts %lu",
      (unsigned long)age_s, (unsigned long)bt->packet_count);

  if (bt->has_valid || bt->has_last_good) {
    put(out, &n, 4, 62, 1, EINK_BLACK, "lat %.5f", (double)bt->last_good.latitude);
    put(out, &n, 4, 74, 1, EINK_BLACK, "lon %.5f", (double)bt->last_good.longitude);
    if (in->distance_km >= 0.0f) {
      put(out, &n, 4, 86, 1, EINK_BLACK, "alt %.0fm  dist %.2fkm",
          (double)bt->last_good.altitude, (double)in->distance_km);
    }
  } else {
    put(out, &n, 4, 66, 2, EINK_BLACK, "NO BEACON YET");
  }

  if (link_lost) {
    put(out, &n, 4, 100, 2, EINK_RED, "LINK LOST");
  } else {
    put(out, &n, 4, 104, 1, EINK_BLACK, "BAT %.2fV  up %lus",
        in->vdd_mv / 1000.0, (unsigned long)(in->now_ms / 1000u));
  }
  return n;
}
