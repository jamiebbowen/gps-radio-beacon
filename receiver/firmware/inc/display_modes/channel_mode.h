/**
 ******************************************************************************
 * @file           : channel_mode.h
 * @brief          : Rocket/channel selection display mode
 ******************************************************************************
 */

#ifndef CHANNEL_MODE_H
#define CHANNEL_MODE_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Display the rocket channel list with the active channel highlighted.
 *        A long button press on this page cycles to the next channel
 *        (handled in main.c).
 * @retval None
 */
void DisplayMode_ChannelSelect(void);

#ifdef __cplusplus
}
#endif

#endif /* CHANNEL_MODE_H */
