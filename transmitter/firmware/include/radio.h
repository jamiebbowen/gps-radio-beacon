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

// Active rocket channel after backup-jumper resolution (see mpu_config.h)
uint8_t radio_get_channel(void);

#endif // RADIO_H
