#ifndef GPS_UART_H
#define GPS_UART_H

#include <stdint.h>
#include <stdbool.h>
#include "mpu_config.h"

// UART initialization for GPS communication
void uart_init(void);

// Check if data is available to read
bool uart_data_available(void);

// Read a byte from UART
uint8_t uart_read_byte(void);

// Transmit a byte via UART
void uart_tx_byte(uint8_t data);

// Transmit a string via UART
void uart_tx_string(const char* str);

// RX buffer size - must be a power of 2 for efficient wrapping
// ATtiny412 has limited RAM (256 bytes), so keep this small
#define UART_RX_BUFFER_SIZE 32

// Get UART diagnostic information
void uart_get_diagnostics(uint16_t* isr_count, uint8_t* last_byte, uint8_t* active);

// Get and clear error counter
uint16_t uart_get_and_clear_error_count(void);

// Get and clear framing error counter
uint16_t uart_get_and_clear_framing_error_count(void);

// Get and clear buffer overflow counter
uint16_t uart_get_and_clear_buffer_overflow_count(void);

// Flush the UART buffer
void flush_uart_buffer(void);

#endif /* UART_H_ */
