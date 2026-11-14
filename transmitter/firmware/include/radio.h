#ifndef RADIO_H
#define RADIO_H

#include <stdint.h>
#include "mpu_config.h"

// LoRa radio using E22-400M33S (SX1268) module

// Core function prototypes
void radio_init(void);
void radio_enable(void);
void radio_disable(void);

// LoRa transmission functions
int transmit_packet(const uint8_t* data, size_t length);
int transmit_string(const char* str);

// Radio status
bool radio_is_transmitting(void);

/* Command uplink window. The receiver sends commands with a ~1.3 s
 * preamble so the beacon's 40 ms listen sniff (about once a second) always
 * lands inside it. Once a preamble or LoRa header is detected the listen
 * stays open this long, measured from the start of the window: worst case
 * the whole 1.31 s preamble plus the ~0.46 s header+payload of a 6-byte
 * command, with margin. */
#define CMD_RX_EXTEND_MS 2200u

/* Two-way command channel (v2 radios). Bounded listen once per cadence
 * gap, stretched to CMD_RX_EXTEND_MS when a frame is detected; negative
 * return = driver fault, 0 = nothing inbound this window, positive =
 * bytes captured. */
int  radio_poll_rx(uint8_t *out, size_t max_len, uint16_t timeout_ms);
int  radio_transmit_ack(uint8_t rocket_id, uint8_t cmd_code,
                        uint8_t seq_hi, uint8_t seq_lo, uint8_t echo);

// Active rocket channel after backup-jumper resolution (see mpu_config.h)
uint8_t radio_get_channel(void);

#endif // RADIO_H
