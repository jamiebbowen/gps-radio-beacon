/**
 ******************************************************************************
 * @file           : preflight_mode.h
 * @brief          : Pre-flight go/no-go checklist display mode
 ******************************************************************************
 */

#ifndef PREFLIGHT_MODE_H
#define PREFLIGHT_MODE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Aggregated pad-side readiness check: TX link, TX GPS, RX GPS,
 *        compass, SD card, and a bottom-line READY / NOT READY verdict.
 *
 * @param link_ok      1 = a beacon packet (any type) arrived recently
 * @param link_age_s   seconds since the last beacon packet (0 when unknown)
 * @param rssi         last packet RSSI (dBm)
 * @param tx_fix       TX fix quality from the last position packet
 * @param tx_sats      TX satellite count from the last position packet
 * @param tx_hb_state  heartbeat GPS health state (HB_GPS_*), or 0xFF if no
 *                     recent heartbeat
 * @param rx_fix_ok    1 = receiver's own GPS has a fix
 * @param rx_sats      receiver satellite count
 * @param compass_ok   1 = compass heading valid
 * @param sd_ok        1 = SD card logging available (advisory only)
 */
void DisplayMode_Preflight(uint8_t link_ok, uint32_t link_age_s, int16_t rssi,
                           uint8_t tx_fix, uint8_t tx_sats, uint8_t tx_hb_state,
                           uint8_t rx_fix_ok, uint8_t rx_sats,
                           uint8_t compass_ok, uint8_t sd_ok);

#ifdef __cplusplus
}
#endif

#endif /* PREFLIGHT_MODE_H */
