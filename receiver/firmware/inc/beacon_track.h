/**
 * @file beacon_track.h
 * @brief The receiver's view of the beacon: the newest usable position,
 *        how old it is, when to persist it, and the link inputs of the
 *        go/no-go check.
 *
 * main.c feeds it every parsed packet and reads the state back for the
 * pages; the rules live here so they can be host-tested:
 *  - a packet with usable coordinates becomes the current and last-good
 *    position (and gets a NAV row);
 *  - a held fused packet (dead-reckoned, see rf_parser.c) still updates
 *    the fused flags but neither refreshes the position age nor is ever
 *    persisted to BEACON.TXT;
 *  - BEACON.TXT is written at most every BEACON_TRACK_SAVE_MS;
 *  - with no applied position for BEACON_TRACK_STALE_MS the current one
 *    stops counting as valid (the last-good copy stays for display).
 */
#ifndef BEACON_TRACK_H
#define BEACON_TRACK_H

#include <stdint.h>
#include "gps.h"
#include "preflight_eval.h"

#define BEACON_TRACK_STALE_MS  180000u   /* position invalid after 3 min */
#define BEACON_TRACK_SAVE_MS    10000u   /* BEACON.TXT write rate limit  */

/* BeaconTrack_OnPacket() result bits */
#define BEACON_TRACK_NAV_ROW   0x01u     /* usable coordinates: log a NAV row */
#define BEACON_TRACK_SAVE      0x02u     /* persist BEACON.TXT now            */

typedef struct {
  GPS_Data current;        /* newest packet with usable coordinates         */
  GPS_Data last_good;      /* same, kept when `current` goes stale          */
  uint8_t  has_valid;      /* `current` is usable and not stale             */
  uint8_t  has_last_good;  /* `last_good` holds a position                  */
  uint32_t pos_tick;       /* tick of the newest APPLIED position, 0 = none */
  uint32_t packet_count;   /* packets accepted ("Pkts:")                    */
  uint32_t last_save_ms;   /* BEACON.TXT rate limit                         */
} BeaconTrack_t;

void    BeaconTrack_Init(BeaconTrack_t *bt);

/** A parsed packet arrived (RF_Receiver_GetGPSData() == RF_OK).
 *  @retval BEACON_TRACK_* bits for the caller's SD logging */
uint8_t BeaconTrack_OnPacket(BeaconTrack_t *bt, const GPS_Data *pkt, uint32_t now_ms);

/** Every loop: drop validity after BEACON_TRACK_STALE_MS without an
 *  applied position. @retval 1 while stale, 0 otherwise */
uint8_t BeaconTrack_Age(BeaconTrack_t *bt, uint32_t now_ms);

/** Manual channel pick: everything so far belonged to the previous rocket. */
void    BeaconTrack_NewContext(BeaconTrack_t *bt);

/** Boot: the position saved in BEACON.TXT (shown as "Last: SD"; no age). */
void    BeaconTrack_Restore(BeaconTrack_t *bt, float lat, float lon, float alt);

/** The saved position is implausibly far from the receiver: drop it. */
void    BeaconTrack_Forget(BeaconTrack_t *bt);

/** Link and beacon-GPS inputs of the go/no-go check (preflight_eval.h):
 *  link age from the beacon-heard tick (any packet from our beacon), the
 *  position report from the newest applied position, the heartbeat report
 *  as given. Leaves the other Preflight_Inputs_t fields untouched. */
void    BeaconTrack_FillPreflight(const BeaconTrack_t *bt, uint32_t now_ms,
                                  uint32_t heard_tick, uint8_t hb_valid,
                                  uint32_t hb_age_ms, uint8_t hb_fix,
                                  uint8_t hb_sats, Preflight_Inputs_t *in);

#endif /* BEACON_TRACK_H */
