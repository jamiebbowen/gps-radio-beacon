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
 * Layout (8 rows x 21 cols), channels in two columns of 4:
 *   ROCKET SELECT  SCAN
 *   >0 433.00  4 434.00
 *    1 433.25  5 434.25
 *    2 433.50 >6 434.50
 *    3 433.75  7 434.75
 *
 *   Pkts:123  Age:12s
 *   B2/hold B1: next ch
 */
void DisplayMode_ChannelSelect(void)
{
    char buffer[32];
    uint8_t current = RF_Receiver_GetChannel();

    Display_DrawTextRowCol(0, 0, RF_Receiver_IsScanning()
                                     ? "ROCKET SELECT  SCAN"
                                     : "ROCKET SELECT");

    /* Two columns of LORA_CHANNEL_COUNT/2 channels each. Frequencies are
     * fixed-point hundredths of MHz to avoid float printf formatting. */
    const uint8_t rows = (LORA_CHANNEL_COUNT + 1) / 2;
    for (uint8_t ch = 0; ch < LORA_CHANNEL_COUNT; ch++) {
        uint32_t f100 = (uint32_t)(LORA_CHANNEL_FREQ_MHZ(ch) * 100.0f + 0.5f);
        snprintf(buffer, sizeof(buffer), "%c%u %lu.%02lu",
                 (ch == current) ? '>' : ' ',
                 (unsigned)ch,
                 (unsigned long)(f100 / 100), (unsigned long)(f100 % 100));
        uint8_t row = (uint8_t)(1 + (ch % rows));
        uint8_t col = (uint8_t)((ch / rows) * 11);
        Display_DrawTextRowCol(row, col, buffer);
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
