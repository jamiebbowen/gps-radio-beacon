/**
 * @file eink_summary.h
 * @brief What the e-ink summary page says: channel and signal, link age,
 *        the beacon's last-good position and distance, and either the
 *        receiver's battery or a red LINK LOST.
 *
 * main.c schedules the slow refresh and draws the lines; the content rules
 * live here so they can be host-tested.
 */
#ifndef EINK_SUMMARY_H
#define EINK_SUMMARY_H

#include <stdint.h>
#include "beacon_track.h"

#define EINK_SUMMARY_MAX_LINES  8
#define EINK_SUMMARY_LOST_S     180u   /* link lost after 3 min of silence */

typedef struct {
  uint16_t x, y;
  uint8_t  size;
  uint8_t  color;          /* EINK_BLACK / EINK_RED */
  char     text[48];
} EinkSummary_Line_t;

typedef struct {
  uint8_t  channel;
  int16_t  rssi;
  uint32_t now_ms;
  uint32_t heard_tick;     /* newest packet from our beacon, 0 = never */
  float    distance_km;    /* < 0: unknown */
  uint32_t vdd_mv;         /* receiver pack */
  const BeaconTrack_t *beacon;
} EinkSummary_In_t;

/** Compose the page. @retval number of lines filled (<= MAX_LINES). The
 *  caller draws the red rule under the title itself. */
uint8_t EinkSummary_Compose(const EinkSummary_In_t *in, EinkSummary_Line_t *out);

#endif /* EINK_SUMMARY_H */
