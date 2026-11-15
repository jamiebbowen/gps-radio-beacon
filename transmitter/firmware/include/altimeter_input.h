/**
 * @file altimeter_input.h
 * @brief The carrier's three commercial-altimeter serial inputs: J2 (D13),
 *        J5 (D12), J6 (D11), 5 V TTL through 10k/20k dividers, 9600 8N1,
 *        receive only. A 38.4 kHz TC3 interrupt samples all three lines
 *        (software UARTs - SERCOM3, the only other SERCOM on these pins
 *        besides SERCOM5, runs the GPS); the main loop and the radio-wait
 *        hook decode the bytes (altimeter.h).
 */
#ifndef ALTIMETER_INPUT_H
#define ALTIMETER_INPUT_H

#include <stdint.h>
#include "packet_format.h"

/** Inputs, line sampling and the TC3 interrupt. */
void    altimeter_init(void);

/** One line sample per input (TC3 interrupt): PORT group A's IN register. */
void    altimeter_sample(uint32_t port_in);

/** Decode what the inputs received (loop() and the radio-wait yield()). */
void    altimeter_poll(uint32_t now_ms);

/** Flight-log records: per recognised input at most every
 *  ALT_LOG_INTERVAL_MS while it reports, at once on a new deploy event. */
void    altimeter_log_service(uint32_t now_ms);

/** 1 while an input reported a deploy event not yet relayed. */
uint8_t altimeter_new_events(void);

/** Fill the relay packet (type, rocket id, recognised inputs only).
 *  @retval bytes to send, 0 if no input has a recognised format */
uint8_t altimeter_build_packet(AltimeterPacket_t *p, uint32_t now_ms);

#endif /* ALTIMETER_INPUT_H */
