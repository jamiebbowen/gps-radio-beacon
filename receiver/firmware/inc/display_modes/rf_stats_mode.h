/**
 ******************************************************************************
 * @file           : rf_stats_mode.h
 * @brief          : Full radio statistics display mode
 ******************************************************************************
 */

#ifndef RF_STATS_MODE_H
#define RF_STATS_MODE_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Display full radio statistics: channel, modem config, packet and IRQ
 *        counters, signal quality, parser results, and radio state.
 * @retval None
 */
void DisplayMode_RFStats(void);

#ifdef __cplusplus
}
#endif

#endif /* RF_STATS_MODE_H */
