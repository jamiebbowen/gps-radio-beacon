/**
 * @file altimeter_input.h
 * @brief The carrier's three commercial-altimeter serial inputs: J2 (D13),
 *        J5 (D12), J6 (D11), 5 V or 3.3 V logic through 10k/20k dividers, 9600 8N1,
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

/** USB diagnostics, loop() only: per jack whose counters moved, its format,
 *  records, rejected records and ring drops - at most every 10 s. An
 *  unplugged jack prints nothing. */
void    altimeter_report(uint32_t now_ms);

/** Flight-log records: per recognised input at most every
 *  ALT_LOG_INTERVAL_MS while it reports, at once on a new deploy event. */
void    altimeter_log_service(uint32_t now_ms);

/** Launch edge: every input drops its pad history (alt_flight_start), and
 *  nothing reported before liftoff is relayed or logged as this flight's;
 *  each input's first record after it is logged at once. */
void    altimeter_flight_start(uint32_t now_ms);

/** The heartbeat's alt_inputs byte (packet_format.h): the format each jack
 *  has recognised, and HB_ALT_STALE when a recognised one went quiet. */
uint8_t altimeter_heartbeat_byte(uint32_t now_ms);

/** 1 while an input reported a deploy event not yet relayed. */
uint8_t altimeter_new_events(void);

/** The maxima recap's altimeter tail (packet_format.h): alt_inputs and one
 *  entry per recognised input, ascending. @retval bytes the tail adds to
 *  the 11-byte recap, 0 if no input has a recognised format */
uint8_t altimeter_fill_maxima(MaximaAltPacket_t *p, uint32_t now_ms);

#endif /* ALTIMETER_INPUT_H */
