/**
 * @file altimeter_uart.h
 * @brief Byte capture for the three altimeter serial inputs (9600 8N1):
 *        J5 -> D12/PA23 on SERCOM5 (the only SERCOM left for these pins;
 *        SERCOM3 runs the GPS), J2 -> D13/PA22 and J6 -> D11/PA21 on
 *        software receivers (EIC start-bit edge + TC mid-bit sampling).
 */
#ifndef ALTIMETER_UART_H
#define ALTIMETER_UART_H

#include <stdint.h>
#include "altimeter.h"

#define ALT_INPUTS 3            /* 0 = J2 (D13), 1 = J5 (D12), 2 = J6 (D11) */

void altimeter_init(void);

/** Drain received bytes into the decoders; safe from loop() and yield(). */
void altimeter_poll(uint32_t now_ms);

/** Decoder state of input i (0..ALT_INPUTS-1), NULL if out of range. */
const alt_state_t *altimeter_state(uint8_t i);

#endif /* ALTIMETER_UART_H */
