/**
 ******************************************************************************
 * @file           : rf_stats_mode.c
 * @brief          : Full radio statistics display mode implementation
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "display_modes/rf_stats_mode.h"
#include "display.h"
#include "lora.h"
#include "rf_receiver.h"
#include "rf_parser.h"
#include <stdio.h>

/**
 * @brief Display full radio statistics
 * @retval None
 *
 * Layout (8 rows x 21 cols):
 *   RF STATS   CH0 433.0
 *   SF9 BW125k CR4/7
 *   Pkts:12345 IRQ:12350
 *   RSSI:-102  SNR:-5
 *   Last pkt: 12s ago
 *   Parse OK:120 F:1
 *   Mode:RX SPI:OK Bsy:0
 *   IRQst:0x0002
 */
void DisplayMode_RFStats(void)
{
    char buffer[32];

    /* Gather diagnostics */
    uint32_t irq_checks = 0;
    uint16_t last_irq = 0;
    uint8_t device_mode = 0, spi_test = 0xFF, busy_state = 0;
    RF_Receiver_GetIRQDiagnostics(&irq_checks, &last_irq, &device_mode,
                                  &spi_test, &busy_state);

    uint32_t dio1_irqs = 0, lora_packets = 0, duplicates = 0;
    RF_Receiver_GetPacketLossDiagnostics(&dio1_irqs, &lora_packets, &duplicates);

    int16_t rssi = 0;
    int8_t snr = 0;
    RF_Receiver_GetSignalQuality(&rssi, &snr);

    uint32_t attempts = 0, successes = 0, failures = 0;
    RF_Parser_GetDiagnostics(&attempts, &successes, &failures);

    /* Row 0: title, channel, frequency */
    uint8_t ch = RF_Receiver_GetChannel();
    uint16_t f10 = (uint16_t)(LORA_CHANNEL_FREQ_MHZ(ch) * 10.0f + 0.5f);
    snprintf(buffer, sizeof(buffer), "RF STATS   CH%u %u.%u",
             (unsigned)ch, (unsigned)(f10 / 10), (unsigned)(f10 % 10));
    Display_DrawTextRowCol(0, 0, buffer);

    /* Row 1: modem configuration */
    snprintf(buffer, sizeof(buffer), "SF%u BW%uk CR4/%u",
             (unsigned)LORA_SPREADING_FACTOR,
             (unsigned)LORA_BANDWIDTH_KHZ,
             (unsigned)LORA_CODING_RATE);
    Display_DrawTextRowCol(1, 0, buffer);

    /* Row 2: packets read vs DIO1 interrupts (difference = missed reads) */
    snprintf(buffer, sizeof(buffer), "Pkts:%lu IRQ:%lu",
             (unsigned long)lora_packets, (unsigned long)dio1_irqs);
    Display_DrawTextRowCol(2, 0, buffer);

    /* Row 3: signal quality of last packet + ambient noise floor */
    int16_t nf = 0;
    if (RF_Receiver_GetNoiseFloor(&nf)) {
        snprintf(buffer, sizeof(buffer), "R:%d S:%d NF:%d%s",
                 (int)rssi, (int)snr, (int)nf,
                 RF_Receiver_NoiseAlert() ? "!" : "");
    } else {
        snprintf(buffer, sizeof(buffer), "RSSI:%d  SNR:%d",
                 (int)rssi, (int)snr);
    }
    Display_DrawTextRowCol(3, 0, buffer);

    /* Row 4: age of last valid packet */
    uint32_t last = RF_Receiver_GetLastPacketTime();
    if (last > 0) {
        snprintf(buffer, sizeof(buffer), "Last pkt: %lus ago",
                 (unsigned long)((HAL_GetTick() - last) / 1000U));
    } else {
        snprintf(buffer, sizeof(buffer), "Last pkt: none");
    }
    Display_DrawTextRowCol(4, 0, buffer);

    /* Row 5: parser results */
    snprintf(buffer, sizeof(buffer), "Parse OK:%lu F:%lu",
             (unsigned long)successes, (unsigned long)failures);
    Display_DrawTextRowCol(5, 0, buffer);

    /* Row 6: device mode, SPI test, BUSY pin */
    static const char *mode_names[8] = {
        "?", "?", "STBY", "XOSC", "FS", "RX", "TX", "?"
    };
    snprintf(buffer, sizeof(buffer), "Mode:%s SPI:%s Bsy:%u",
             mode_names[device_mode & 0x07],
             (spi_test == 1) ? "OK" : (spi_test == 0 ? "FAIL" : "?"),
             (unsigned)busy_state);
    Display_DrawTextRowCol(6, 0, buffer);

    /* Row 7: last non-zero IRQ status register */
    snprintf(buffer, sizeof(buffer), "IRQst:0x%04X", (unsigned)last_irq);
    Display_DrawTextRowCol(7, 0, buffer);
}
