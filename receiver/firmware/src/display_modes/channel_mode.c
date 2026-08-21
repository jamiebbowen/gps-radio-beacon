/**
 ******************************************************************************
 * @file           : channel_mode.c
 * @brief          : Rocket/channel selection display mode implementation
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "display_modes/channel_mode.h"
#include "display.h"
#include "lora.h"
#include "rf_receiver.h"
#include <stdio.h>

/**
 * @brief Display the rocket channel list with the active channel highlighted
 * @retval None
 *
 * Layout (8 rows x 21 cols):
 *   ROCKET SELECT
 *   Ch  Freq
 *   >0  433.0 MHz
 *    1  433.5 MHz
 *    2  434.0 MHz
 *    3  434.5 MHz
 *   Pkts:123  Age:12s
 *   Hold btn: next ch
 */
void DisplayMode_ChannelSelect(void)
{
    char buffer[32];
    uint8_t current = RF_Receiver_GetChannel();

    Display_DrawTextRowCol(0, 0, RF_Receiver_IsScanning()
                                     ? "ROCKET SELECT  SCAN"
                                     : "ROCKET SELECT");
    Display_DrawTextRowCol(1, 0, "Ch  Freq");

    for (uint8_t ch = 0; ch < LORA_CHANNEL_COUNT; ch++) {
        /* Format frequency as fixed-point tenths of MHz to avoid pulling in
         * float printf formatting. */
        uint16_t f10 = (uint16_t)(LORA_CHANNEL_FREQ_MHZ(ch) * 10.0f + 0.5f);
        snprintf(buffer, sizeof(buffer), "%c%u  %u.%u MHz",
                 (ch == current) ? '>' : ' ',
                 (unsigned)ch,
                 (unsigned)(f10 / 10), (unsigned)(f10 % 10));
        Display_DrawTextRowCol((uint8_t)(2 + ch), 0, buffer);
    }

    /* Activity on the current channel */
    uint32_t last = RF_Receiver_GetLastPacketTime();
    uint32_t pkts = RF_Receiver_GetLoRaPacketCount();
    if (last > 0) {
        snprintf(buffer, sizeof(buffer), "Pkts:%lu  Age:%lus",
                 (unsigned long)pkts,
                 (unsigned long)((HAL_GetTick() - last) / 1000U));
    } else {
        snprintf(buffer, sizeof(buffer), "Pkts:%lu  Age:--",
                 (unsigned long)pkts);
    }
    Display_DrawTextRowCol(6, 0, buffer);

    Display_DrawTextRowCol(7, 0, "B2/hold B1: next ch");
}
