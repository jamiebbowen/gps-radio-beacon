/**
 ******************************************************************************
 * @file           : navigation_mode.c
 * @brief          : Navigation display mode implementation
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>  /* For abs and labs functions */
#include <math.h>
#include "stm32f4xx_hal.h"
#include "display_modes/navigation_mode.h"
#include "display.h"
#include "gps.h"
#include "math_utils.h"
#include "compass.h"
#include "rf_receiver.h"
#include "lora.h"  /* LORA_FREQUENCY_MHZ, LORA_TX_POWER_DBM - for theoretical max range */

extern Compass_Data compass_data;

/* ---------------------------------------------------------------------------
 * Maximum-LOS range predictor.
 *
 * Given the current measured RSSI at a known GPS distance, extrapolates how
 * far the beacon could go before the signal reaches the demodulator floor.
 * Self-calibrating: no TX power, antenna gain, or L0 assumptions needed -
 * the current (d_now, RSSI_now) pair anchors the link-budget line.
 *
 *   margin_dB = RSSI_now - RSSI_floor
 *   d_max_m   = d_now_m * 10 ^ (margin_dB / (10 * n))
 *
 * Only two knobs:
 *   LORA_SENSITIVITY_DBM  - the RSSI at which packets stop decoding. Depends
 *                           on SF/BW. SX1268 datasheet: ~-137 dBm at SF12/BW125,
 *                           ~-124 dBm at SF9/BW125, ~-118 dBm at SF7/BW125.
 *                           Default -130 dBm is a reasonable midpoint; tighten
 *                           it to match your actual radio config.
 *   LORA_PATH_LOSS_N      - 2.0 for pure free-space LOS (max-case ceiling),
 *                           2.5-3.0 for realistic outdoor with ground bounce
 *                           and foliage. Using 2.0 here because the user asked
 *                           for the "max predicted" range.
 * -------------------------------------------------------------------------*/
#ifndef LORA_SENSITIVITY_DBM
#define LORA_SENSITIVITY_DBM   -130.0f
#endif
#ifndef LORA_PATH_LOSS_N
#define LORA_PATH_LOSS_N          2.0f
#endif

/* Below this current distance the RSSI measurement is near-field / saturated
 * and extrapolation amplifies the error. Return 0 so the caller can render
 * a placeholder instead of a misleading number. */
#define LORA_MIN_ANCHOR_M           5.0f

/**
 * @brief Predict *remaining* LOS range before the link reaches demod floor.
 *
 * Returns d_max - d_now: how much further the beacon could travel from its
 * current position before packets stop decoding. Approaches 0 as margin
 * depletes, so it reads naturally as a countdown gauge.
 *
 * @param d_now_m  current beacon distance in meters (from GPS, must be > 0)
 * @param rssi     current RSSI in dBm (negative)
 * @return remaining range in meters. Returns UINT32_MAX as a sentinel when
 *         d_now_m < LORA_MIN_ANCHOR_M (anchor too close, extrapolation
 *         unreliable) so the caller can render a placeholder.
 */
#define LORA_RANGE_UNAVAILABLE  0xFFFFFFFFu
static uint32_t LoRa_RemainingRangeMeters(float d_now_m, int16_t rssi)
{
  if (d_now_m < LORA_MIN_ANCHOR_M) return LORA_RANGE_UNAVAILABLE;
  float margin_db = (float)rssi - (float)LORA_SENSITIVITY_DBM;
  if (margin_db <= 0.0f) return 0;  /* at or below floor - no budget left */
  float d_max = d_now_m * powf(10.0f, margin_db / (10.0f * (float)LORA_PATH_LOSS_N));
  float remaining = d_max - d_now_m;
  if (remaining < 0.0f)       remaining = 0.0f;
  if (remaining > 9999999.0f) remaining = 9999999.0f;
  return (uint32_t)remaining;
}

/**
 * @brief Theoretical maximum LOS range given only the radio configuration.
 *
 * Why: when the beacon is right next to the receiver, d_now is tiny and a
 * small RSSI measurement error produces wildly optimistic range
 * extrapolations ("999.9mi left" even though our radio can't physically
 * reach that far). This function computes the Friis free-space ceiling
 * from first principles - TX power, sensitivity, and carrier frequency -
 * so the UI can clamp the extrapolation at a value the radio could
 * actually achieve.
 *
 * Derivation:
 *   FSPL(d) = 20*log10(d_m) + 20*log10(f_MHz) - 27.55    (isotropic, d in m)
 *   At d_max:   Pt - FSPL(d_max) = sensitivity
 *   => 20*log10(d_max) = Pt - sens - 20*log10(f_MHz) + 27.55
 *   => d_max_m = 10 ^ ((Pt - sens - 20*log10(f_MHz) + 27.55) / 20)
 *
 * For current config (22 dBm TX, -130 dBm sens, 433 MHz) this evaluates to
 * roughly 2200 km. That's the Friis ceiling with 0 dBi antennas, no
 * fading; practice is always below it, but it's a physically meaningful
 * cap for "don't show more than this".
 *
 * Result is cached after first call; the inputs are compile-time
 * constants so the value never changes at runtime.
 */
static uint32_t LoRa_MaxTheoreticalRangeMeters(void)
{
  static uint32_t cached = 0;
  if (cached != 0) return cached;

  float pt_dbm   = (float)LORA_TX_POWER_DBM;
  float sens_dbm = (float)LORA_SENSITIVITY_DBM;
  float f_mhz    = (float)LORA_FREQUENCY_MHZ;

  float log_f = log10f(f_mhz);
  float exponent = (pt_dbm - sens_dbm - 20.0f * log_f + 27.55f) / 20.0f;
  float d_max_m = powf(10.0f, exponent);

  if (d_max_m < 0.0f)       d_max_m = 0.0f;
  if (d_max_m > 9999999.0f) d_max_m = 9999999.0f;
  cached = (uint32_t)d_max_m;
  return cached;
}

/**
 * @brief Map (RSSI, SNR) to a short user-friendly link-quality label.
 *
 * Rationale: raw RSSI/SNR are hard to interpret in the field. These tiers
 * are calibrated for the SX1268 in the bandwidth/spreading-factor range this
 * project uses, so "Excellent" really means "big link budget margin" and
 * "Poor" means "near the demodulator floor".
 *
 *   Tier       RSSI (dBm)     SNR (dB)       Meaning
 *   ----       ----------     --------       -----------------------------
 *   Excellent  >= -90         >=  7          plenty of margin, LOS/close
 *   Good       >= -100        >=  0          comfortable
 *   Fair       >= -110        >= -7          working, starting to degrade
 *   Weak       >= -120        >= -15         near demod floor, expect losses
 *   Poor       below          below          marginal / edge of decode
 *
 * The reported tier is the worse of the two (a strong-RSSI/low-SNR packet
 * usually means co-channel interference, which is not "Excellent").
 */
static const char *LoRa_QualityLabel(int16_t rssi, int8_t snr)
{
  uint8_t r_tier = (rssi >= -90)  ? 4 :
                   (rssi >= -100) ? 3 :
                   (rssi >= -110) ? 2 :
                   (rssi >= -120) ? 1 : 0;
  uint8_t s_tier = (snr  >=   7)  ? 4 :
                   (snr  >=   0)  ? 3 :
                   (snr  >=  -7)  ? 2 :
                   (snr  >= -15)  ? 1 : 0;
  uint8_t tier = (r_tier < s_tier) ? r_tier : s_tier;
  switch (tier) {
    case 4: return "Excellent";
    case 3: return "Good";
    case 2: return "Fair";
    case 1: return "Weak";
    default: return "Poor";
  }
}

/**
 * @brief Display navigation data screen
 * @param has_valid_local_gps Flag indicating if local GPS data is valid
 * @param has_valid_remote_gps Flag indicating if remote GPS data is valid
 * @param local_gps_data Local GPS data structure
 * @param remote_gps_data Remote GPS data structure
 * @param compass_heading Current compass heading
 * @param last_rf_packet_time Time of last RF packet received
 * @param rf_packet_count Count of RF packets received
 * @retval None
 */
void DisplayMode_Navigation(uint8_t has_valid_local_gps, uint8_t has_valid_remote_gps,
                           GPS_Data *local_gps_data, GPS_Data *remote_gps_data,
                           float compass_heading, uint32_t last_rf_packet_time,
                           uint32_t rf_packet_count)
{
  float distance_to_tx = -1.0f;
  float direction_to_tx = -1.0f;
  float relative_direction = -1.0f;
  
  /* Static variables to store last known good values.
   * Only distance and relative bearing are consumed downstream; the absolute
   * bearing-to-TX and the timestamp of last update were cached historically
   * but never read, so they've been removed. */
  static float last_good_distance = -1.0f;
  static float last_good_relative = -1.0f;
  
  /* Calculate distance and direction if we have valid local GPS and remote position (live or SD-loaded) */
  if (has_valid_local_gps && (last_rf_packet_time > 0 || has_valid_remote_gps)) {
    /* Validate coordinates are within reasonable ranges and non-zero */
    if (local_gps_data->latitude != 0.0f && local_gps_data->longitude != 0.0f && 
        remote_gps_data->latitude != 0.0f && remote_gps_data->longitude != 0.0f &&
        local_gps_data->latitude >= -90.0f && local_gps_data->latitude <= 90.0f &&
        local_gps_data->longitude >= -180.0f && local_gps_data->longitude <= 180.0f &&
        remote_gps_data->latitude >= -90.0f && remote_gps_data->latitude <= 90.0f &&
        remote_gps_data->longitude >= -180.0f && remote_gps_data->longitude <= 180.0f) {
      /* Calculate distance between local and remote GPS coordinates */
      /* Always use the last known remote GPS position, even if it's stale */
      distance_to_tx = calculate_distance(local_gps_data->latitude, local_gps_data->longitude,
                                        remote_gps_data->latitude, remote_gps_data->longitude) / 1000.0f; /* Convert to km */
      
      /* Calculate bearing from local to remote GPS coordinates */
      direction_to_tx = calculate_bearing(local_gps_data->latitude, local_gps_data->longitude,
                                        remote_gps_data->latitude, remote_gps_data->longitude);
      
      /* Calculate relative direction based on current compass heading */
      relative_direction = direction_to_tx - compass_heading;
      
      /* Normalize relative direction to 0-359.9 degrees */
      while (relative_direction < 0) relative_direction += 360.0f;
      while (relative_direction >= 360.0f) relative_direction -= 360.0f;
      
      /* Always cache: reaching here means GPS coords were valid and both
       * calculate_distance / calculate_bearing returned meaningful values.
       * Note: direction_to_tx can legitimately be 0.0 (beacon due North)
       * so a >= 0 guard would incorrectly skip that case. */
      last_good_distance = distance_to_tx;
      last_good_relative = relative_direction;
    } else {
      /* Set invalid values if coordinates are zero */
      distance_to_tx = -1.0f;
      direction_to_tx = -1.0f;
      relative_direction = -1.0f;
    }
  }
  
  /* If current values are invalid but we have last known good values, use them */
  if (distance_to_tx < 0.0f && last_good_distance >= 0.0f) {
    distance_to_tx = last_good_distance;
  }
  
  if (relative_direction < 0.0f && last_good_relative >= 0.0f) {
    relative_direction = last_good_relative;
  }
  
  /* Display layout (128x64, 6x8 font => 21 cols x 8 rows).
   *
   * The direction indicator is a circle of radius 15 centred at (96, 32), so
   * it occupies pixels x=[81..111], y=[17..47]. Rows 2..5 (y=16..47) overlap
   * the arrow horizontally, so text in those rows is capped at 13 columns
   * (x ends at 77, a 4 px gap to the arrow). Rows 0,1,6,7 are clear of the
   * arrow and may use the full 21 columns.
   *
   *   Row  Content                           Width cap
   *   ---  --------------------------------  ---------
   *    0   Dist: 1.23km                        21
   *    1   Last: 12s                           21
   *    2   L:Fix B:3D                          13
   *    3   S:13 12.3m/s    <- sats | speed     13
   *    4   Excellent            <- quality     13
   *    5   ~1.2mi left          <- countdown   13
   *    6   RSSI -108 SNR +12                   21
   *    7   Packets: 12345                      21
   */
  #define NAV_NARROW_COLS 13
  #define NAV_WIDE_COLS   21

  /* Show navigation data if we have remote position (live RF or SD-loaded beacon) */
  if (last_rf_packet_time > 0 || has_valid_remote_gps) {
    char narrow[NAV_NARROW_COLS + 2];  /* +1 null, +1 guard */
    char wide[NAV_WIDE_COLS   + 2];

    /* Row 0: Distance */
    if (distance_to_tx >= 0.0f) {
      if (distance_to_tx < 1.0f) {
        int meters = (int)(distance_to_tx * 1000.0f);
        if (meters <= 0) meters = 1;
        snprintf(wide, sizeof(wide), "Dist: %dm", meters);
      } else {
        int km_whole = (int)distance_to_tx;
        int km_frac  = (int)((distance_to_tx - km_whole) * 100.0f + 0.5f);
        snprintf(wide, sizeof(wide), "Dist: %d.%02dkm", km_whole, km_frac);
      }
    } else {
      snprintf(wide, sizeof(wide), "Dist: --");
    }
    Display_DrawTextRowCol(0, 0, wide);

    /* Row 1: Time since last packet (freshness of everything above/below).
     * If the beacon lost its fix and dropped back to heartbeats, show the
     * heartbeat age instead - it's the live proof-of-link, while the
     * position age just grows. A raised noise floor overrides both: it
     * threatens the link itself, which trumps freshness bookkeeping. */
    uint32_t hb_age_ms = 0;
    uint8_t  hb_heard = RF_Receiver_GetLastHeartbeat(NULL, &hb_age_ms);
    if (RF_Receiver_NoiseAlert()) {
      int16_t nf = 0;
      (void)RF_Receiver_GetNoiseFloor(&nf);
      snprintf(wide, sizeof(wide), "RF NOISE! %ddBm", (int)nf);
    } else if (last_rf_packet_time > 0) {
      uint32_t age_s = (HAL_GetTick() - last_rf_packet_time) / 1000u;
      uint32_t hb_s  = hb_age_ms / 1000u;
      if (hb_heard && hb_s < age_s) {
        /* Clamp so "Last: 99999s HB:999s" (20 chars) always fits */
        if (age_s > 99999u) age_s = 99999u;
        if (hb_s  > 999u)   hb_s  = 999u;
        snprintf(wide, sizeof(wide), "Last: %lus HB:%lus",
                 (unsigned long)age_s, (unsigned long)hb_s);
      } else {
        snprintf(wide, sizeof(wide), "Last: %lus", (unsigned long)age_s);
      }
    } else {
      snprintf(wide, sizeof(wide), "Last: SD");
    }
    Display_DrawTextRowCol(1, 0, wide);

    /* Row 2: Combined fix status (arrow-constrained) */
    const char *b_fix = "B:--";
    if (last_rf_packet_time > 0) {
      if      (remote_gps_data->fix == 0) b_fix = "B:NoFix";
      else if (remote_gps_data->fix == 1) b_fix = "B:2D";
      else                                b_fix = "B:3D";
    }
    snprintf(narrow, sizeof(narrow), "%s %s",
             has_valid_local_gps ? "L:Fix" : "L:--", b_fix);
    Display_DrawTextRowCol(2, 0, narrow);

    /* Row 3: show BOTH satellite count and ground speed on the same line so
     * the display doesn't flap as the TX alternates packet types. Each field
     * is naturally persistent in remote_gps_data:
     *   - satellites  is set only by raw GPS packets (parser leaves it alone
     *                 on FUSED packets, so the last-known count sticks)
     *   - v_north/v_east  are set only by FUSED packets (similarly sticky)
     * so whichever packet arrives updates only its own half of the line.
     *
     * Layout (fits in the 13-char arrow-constrained column):
     *   "S:13 12.3m/s"  <- both known
     *   "S:13  0.0m/s"  <- no FUSED packet received yet (speed defaults to 0)
     *   "S: 0 12.3m/s"  <- no GPS packet yet (unusual, but valid) */
    if (last_rf_packet_time > 0) {
      float vn = remote_gps_data->v_north;
      float ve = remote_gps_data->v_east;
      float speed_ms = sqrtf(vn * vn + ve * ve);
      /* Cap at 999.9: a 4-digit speed would push the row to 14 chars and
       * overdraw the direction arrow (13-col cap for rows 2-5). */
      if (speed_ms > 999.9f) speed_ms = 999.9f;
      snprintf(narrow, sizeof(narrow), "S:%-2d %4.1fm/s",
               (int)remote_gps_data->satellites, (double)speed_ms);
    } else {
      snprintf(narrow, sizeof(narrow), "S:-- ---.-m/s");
    }
    Display_DrawTextRowCol(3, 0, narrow);

    /* Rows 4 & 5: link quality label and remaining-range countdown.
     * Grab (RSSI, SNR) once so the two rows describe the same packet even if
     * a new one lands between draws. */
    int16_t rssi = 0;
    int8_t  snr  = 0;
    if (last_rf_packet_time > 0) {
      RF_Receiver_GetSignalQuality(&rssi, &snr);

      /* Row 4: single-word quality label. "Excellent" (9) is the longest. */
      snprintf(narrow, sizeof(narrow), "%s", LoRa_QualityLabel(rssi, snr));
      Display_DrawTextRowCol(4, 0, narrow);

      /* Row 5: remaining LOS range.
       *
       * Three cases, in priority order:
       *   1. Valid GPS anchor on both ends AND the Friis extrapolation
       *      from (d_now, RSSI) gives a remaining distance below the
       *      radio's theoretical ceiling -> show that as "... left".
       *      This is the normal in-flight case, and is a tight,
       *      meaningful "signal margin remaining" number.
       *   2. Either no GPS anchor, or the extrapolation explodes past
       *      the theoretical ceiling (which happens at very close range
       *      because small RSSI measurement noise dominates the log
       *      extrapolation) -> show the radio's theoretical max reach
       *      as "... max". This is what the radio could physically
       *      achieve under Friis free-space conditions given TX power,
       *      sensitivity, and carrier frequency alone.
       *   3. (Never hits): there is always a theoretical max to show. */
      uint32_t max_m = LoRa_MaxTheoreticalRangeMeters();
      uint32_t rem_m = LORA_RANGE_UNAVAILABLE;
      if (has_valid_local_gps && has_valid_remote_gps && distance_to_tx > 0.0f) {
        rem_m = LoRa_RemainingRangeMeters(distance_to_tx * 1000.0f, rssi);
      }
      bool showing_max;
      uint32_t display_m;
      if (rem_m != LORA_RANGE_UNAVAILABLE && rem_m < max_m) {
        display_m   = rem_m;
        showing_max = false;
      } else {
        display_m   = max_m;
        showing_max = true;
      }
      const char *tail = showing_max ? " max" : " left";

      /* Format: <1 mi -> feet; 1-999 mi -> tenths; >=1000 mi -> whole miles
       * (drop the decimal so the string still fits the 13-char narrow
       * column even at multi-thousand-mile theoretical ceilings). */
      uint32_t rem_ft = (uint32_t)(((uint64_t)display_m * 3281u) / 1000u);
      if (rem_ft < 5280u) {
        snprintf(narrow, sizeof(narrow), "~%luft%s",
                 (unsigned long)rem_ft, tail);
      } else {
        uint32_t miles_x10 = (rem_ft + 264u) / 528u;  /* round to 0.1 mi */
        if (miles_x10 < 10000u) {
          snprintf(narrow, sizeof(narrow), "~%u.%umi%s",
                   (unsigned)(miles_x10 / 10u),
                   (unsigned)(miles_x10 % 10u), tail);
        } else {
          /* Drop the fractional digit for >=1000 mi so the string fits. */
          unsigned miles = (unsigned)(miles_x10 / 10u);
          if (miles > 9999u) miles = 9999u;
          snprintf(narrow, sizeof(narrow), "~%umi%s", miles, tail);
        }
      }
      Display_DrawTextRowCol(5, 0, narrow);
    } else {
      Display_DrawTextRowCol(4, 0, "No signal");
      Display_DrawTextRowCol(5, 0, "");
    }

    /* Row 6: raw RSSI/SNR (below the arrow, full width available). */
    if (last_rf_packet_time > 0) {
      snprintf(wide, sizeof(wide), "RSSI %d SNR %+d",
               (int)rssi, (int)snr);
    } else {
      snprintf(wide, sizeof(wide), "RSSI --  SNR --");
    }
    Display_DrawTextRowCol(6, 0, wide);

    /* Row 7: packet count + source of last packet.
     *   "GPS" = raw PACKET_TYPE_GPS from the beacon
     *   "FUS" = PACKET_TYPE_FUSED, TX-side GPS fresh
     *   "DR"  = PACKET_TYPE_FUSED but dead-reckoning (no fresh TX-side GPS)
     * Helps spot when the link has dropped back to pure inertial. */
    const char *src = "GPS";
    if (remote_gps_data->is_fused) {
      src = remote_gps_data->fused_dr ? "DR" : "FUS";
    }
    snprintf(wide, sizeof(wide), "Pkts:%lu %s",
             (unsigned long)rf_packet_count, src);
    Display_DrawTextRowCol(7, 0, wide);

    /* Direction indicator, right side. Same geometry as before so the
     * width caps above stay valid.
     *   heading_valid   -> arrow
     *   !heading_valid  -> circle + "CAL" (prompt to figure-8 the device) */
    if (relative_direction >= 0.0f && relative_direction < 360.0f &&
        compass_data.heading_valid) {
      Display_DrawDirectionIndicator(96, 32, relative_direction);
    } else if (!compass_data.heading_valid) {
      Display_DrawCircle(96, 32, 15, 1);
      Display_DrawText(89, 28, "CAL");
    } else {
      Display_DrawCircle(96, 32, 15, 1);
    }

  } else {
    /* No position from the beacon yet. If it's sending no-fix heartbeats,
     * show them - the operator can watch sat acquisition from the pad
     * instead of staring at "No RF Data" wondering if the link is up. */
    HeartbeatPacket_t hb;
    uint32_t hb_age_ms = 0;
    char wide[NAV_WIDE_COLS + 2];

    Display_DrawTextRowCol(0, 0, has_valid_local_gps ? "L:Fix" : "L:No GPS Fix");

    if (RF_Receiver_GetLastHeartbeat(&hb, &hb_age_ms)) {
      /* Clamp so "Beacon HB 99999s ago" (20 chars) always fits 21 cols */
      uint32_t hb_s = hb_age_ms / 1000u;
      if (hb_s > 99999u) hb_s = 99999u;
      snprintf(wide, sizeof(wide), "Beacon HB %lus ago",
               (unsigned long)hb_s);
      Display_DrawTextRowCol(2, 0, wide);
      snprintf(wide, sizeof(wide), "R%u CH%u sats:%u",
               (unsigned)hb.rocket_id, (unsigned)hb.channel,
               (unsigned)hb.satellites);
      Display_DrawTextRowCol(3, 0, wide);
      int16_t hb_rssi = 0;
      int8_t  hb_snr  = 0;
      RF_Receiver_GetSignalQuality(&hb_rssi, &hb_snr);
      snprintf(wide, sizeof(wide), "No fix, up %lus",
               (unsigned long)hb.uptime_s);
      Display_DrawTextRowCol(4, 0, wide);
      snprintf(wide, sizeof(wide), "RSSI %d SNR %+d",
               (int)hb_rssi, (int)hb_snr);
      Display_DrawTextRowCol(5, 0, wide);
    } else {
      Display_DrawTextRowCol(4, 0, "No RF Data");
    }

    /* Row 7: channel noise floor - most useful exactly when nothing is
     * being received, to distinguish "beacon off" from "channel jammed". */
    int16_t nf = 0;
    if (RF_Receiver_GetNoiseFloor(&nf)) {
      if (RF_Receiver_NoiseAlert()) {
        snprintf(wide, sizeof(wide), "NF:%ddBm NOISY!", (int)nf);
      } else {
        snprintf(wide, sizeof(wide), "NF:%ddBm", (int)nf);
      }
      Display_DrawTextRowCol(7, 0, wide);
    }
  }
}
