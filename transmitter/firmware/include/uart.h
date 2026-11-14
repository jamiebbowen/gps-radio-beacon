#ifndef GPS_UART_H
#define GPS_UART_H

#include <stdint.h>
#include <stdbool.h>
#include "mpu_config.h"

// UART initialization for GPS communication
void uart_init(void);

// Switch the GPS UART baud rate (for the power-on 9600->115200 dance:
// a module with lost saved config boots at the factory 9600)
void uart_set_baud(uint32_t baud);

// Check if data is available to read
bool uart_data_available(void);

// Read a byte from UART
uint8_t uart_read_byte(void);

// Transmit a byte via UART
void uart_tx_byte(uint8_t data);

// Transmit a string via UART
void uart_tx_string(const char* str);

// Flush the UART buffer
void flush_uart_buffer(void);

#endif /* GPS_UART_H */
