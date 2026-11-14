/**
 * @file beacon_track.c
 * @brief The receiver's view of the beacon (see beacon_track.h)
 */
#include "beacon_track.h"
#include <string.h>

void BeaconTrack_Init(BeaconTrack_t *bt)
{
  memset(bt, 0, sizeof(*bt));
  /* Nothing heard yet: zeros, with the display strings meaning "none"
   * (0,0 is in the ocean and fails the coordinate check anyway). */
  strcpy(bt->current.fix_str, "0");
  strcpy(bt->current.satellites_str, "0");
}

static uint8_t coords_usable(const GPS_Data *g)
{
  return g->latitude != 0.0f && g->longitude != 0.0f &&
         g->latitude >= -90.0f && g->latitude <= 90.0f &&
         g->longitude >= -180.0f && g->longitude <= 180.0f;
}

uint8_t BeaconTrack_OnPacket(BeaconTrack_t *bt, const GPS_Data *pkt, uint32_t now_ms)
{
  uint8_t result = 0;
  if (coords_usable(pkt)) {
    /* Copied even when the position was held: the fused flags (DR,
     * landed, sensor state) are what the pages report next. */
    memcpy(&bt->current, pkt, sizeof(GPS_Data));
    memcpy(&bt->last_good, pkt, sizeof(GPS_Data));
    bt->has_valid = 1;
    bt->has_last_good = 1;
    result |= BEACON_TRACK_NAV_ROW;
    /* A save is a full littlefs close/commit (~100 ms+ during flash GC)
     * and BEACON.TXT is only read at boot: 10 s of staleness is harmless.
     * A held packet carries no new position - never persist it. */
    if (!pkt->fused_position_held &&
        now_ms - bt->last_save_ms >= BEACON_TRACK_SAVE_MS) {
      bt->last_save_ms = now_ms;
      result |= BEACON_TRACK_SAVE;
    }
  } else {
    bt->has_valid = 0;               /* a packet, but no usable position */
  }
  /* Position age: a held packet keeps "Last:" counting from the last real
   * fix. The link itself is the beacon-heard tick (rf_receiver). */
  if (!pkt->fused_position_held) {
    bt->pos_tick = now_ms;
  }
  bt->packet_count++;
  return result;
}

uint8_t BeaconTrack_Age(BeaconTrack_t *bt, uint32_t now_ms)
{
  if (bt->pos_tick > 0 && now_ms - bt->pos_tick > BEACON_TRACK_STALE_MS) {
    bt->has_valid = 0;
    return 1;
  }
  return 0;
}

void BeaconTrack_NewContext(BeaconTrack_t *bt)
{
  bt->has_valid = 0;
  bt->has_last_good = 0;
  bt->pos_tick = 0;
}

void BeaconTrack_Restore(BeaconTrack_t *bt, float lat, float lon, float alt)
{
  bt->current.latitude = lat;
  bt->current.longitude = lon;
  bt->current.altitude = alt;
  bt->current.fix = 1;
  bt->has_valid = 1;
  bt->last_good.latitude = lat;
  bt->last_good.longitude = lon;
  bt->last_good.altitude = alt;
  bt->last_good.fix = 1;
  bt->has_last_good = 1;
}

void BeaconTrack_Forget(BeaconTrack_t *bt)
{
  bt->current.fix = 0;
  bt->last_good.fix = 0;
  bt->has_valid = 0;
  bt->has_last_good = 0;
}

void BeaconTrack_FillPreflight(const BeaconTrack_t *bt, uint32_t now_ms,
                               uint32_t heard_tick, uint8_t hb_valid,
                               uint32_t hb_age_ms, uint8_t hb_fix,
                               uint8_t hb_sats, Preflight_Inputs_t *in)
{
  in->link_heard = (heard_tick > 0);
  in->link_age_s = (heard_tick > 0) ? (now_ms - heard_tick) / 1000u : 0;
  in->pos_valid  = (bt->pos_tick > 0);
  in->pos_age_s  = (bt->pos_tick > 0) ? (now_ms - bt->pos_tick) / 1000u : 0;
  in->pos_fix    = bt->current.fix;
  in->pos_sats   = (uint8_t)bt->current.satellites;
  in->hb_valid   = hb_valid;
  in->hb_age_s   = hb_age_ms / 1000u;
  in->hb_fix     = hb_fix;
  in->hb_sats    = hb_sats;
}
