#include <Arduino.h>
#include "include/uart.h"
#include "include/mpu_config.h"

// Diagnostic counters
static volatile uint16_t uart_read_count = 0;     // Count total read operations
static volatile uint8_t uart_last_rx_byte = 0;    // Last byte received
static volatile uint8_t uart_rx_active = 0;       // Indicates activity
static volatile uint16_t uart_error_count = 0;    // General error count
static volatile uint16_t uart_framing_error_count = 0; // Framing errors (likely baud rate mismatch)
static volatile uint16_t uart_buffer_overflow_count = 0; // Buffer overflow errors

// Initialize UART for GPS communication
void uart_init(void) {
    // Initialize Hardware Serial1 for GPS (pins 0/1 on ItsyBitsy M4)
    // This GPS module is configured for 115200 baud
    GPS_SERIAL.begin(115200);
    
    // Wait for serial port to initialize
    delay(100);
    
    // Reset all diagnostic counters
    uart_read_count = 0;
    uart_last_rx_byte = 0;
    uart_rx_active = 0;
    uart_framing_error_count = 0;
    uart_buffer_overflow_count = 0;
    uart_error_count = 0;
    
    // Clear receive buffer
    while (GPS_SERIAL.available()) {
        GPS_SERIAL.read();
    }
}

void flush_uart_buffer(void) {
    // Clear any pending data in the hardware buffer
    while (GPS_SERIAL.available()) {
        GPS_SERIAL.read();
    }
}

// Check if data is available to read directly from UART hardware
bool uart_data_available(void) {
    return GPS_SERIAL.available() > 0;
}

// Read a byte directly from UART hardware
// Returns the read byte or 0xFF if no data available
uint8_t uart_read_byte(void) {
    uint8_t data = 0xFF; // Default return value when no data available
    
    // Check if there's data available in hardware
    if (GPS_SERIAL.available()) {
        // Read the data byte
        data = GPS_SERIAL.read();
        
        // Update activity indicator
        uart_rx_active = 1;
        uart_last_rx_byte = data;
        
        // Count this read operation for diagnostics
        uart_read_count++;
    }
    
    return data;
}

// Transmit a byte via UART
void uart_tx_byte(uint8_t data) {
    GPS_SERIAL.write(data);
}

// Transmit a string via UART
void uart_tx_string(const char* str) {
    if (!str) return;
    
    GPS_SERIAL.print(str);
}

// Get UART diagnostic information
void uart_get_diagnostics(uint16_t* read_count, uint8_t* last_byte, uint8_t* active) {
    if (read_count) *read_count = uart_read_count;
    if (last_byte) *last_byte = uart_last_rx_byte;
    if (active) {
        *active = uart_rx_active;
        uart_rx_active = 0;  // Clear activity flag after reading
    }
}

// Get and clear error counter
uint16_t uart_get_and_clear_error_count(void) {
    uint16_t count = uart_error_count;
    uart_error_count = 0;
    return count;
}

// Get and clear framing error counter
uint16_t uart_get_and_clear_framing_error_count(void) {
    uint16_t count = uart_framing_error_count;
    uart_framing_error_count = 0;
    return count;
}

// Get and clear buffer overflow counter
uint16_t uart_get_and_clear_buffer_overflow_count(void) {
    uint16_t count = uart_buffer_overflow_count;
    uart_buffer_overflow_count = 0;
    return count;
}
