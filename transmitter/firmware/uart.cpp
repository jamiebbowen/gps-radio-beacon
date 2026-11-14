#include <Arduino.h>
#include "include/uart.h"
#include "include/mpu_config.h"

// Initialize UART for GPS communication
void uart_init(void) {
    // Initialize Hardware Serial1 for GPS (pins 0/1 on ItsyBitsy M4)
    // This GPS module is configured for 115200 baud
    GPS_SERIAL.begin(115200);
    
    // Wait for serial port to initialize
    delay(100);
    
    // Clear receive buffer
    while (GPS_SERIAL.available()) {
        GPS_SERIAL.read();
    }
}

void uart_set_baud(uint32_t baud) {
    GPS_SERIAL.end();
    GPS_SERIAL.begin(baud);
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

