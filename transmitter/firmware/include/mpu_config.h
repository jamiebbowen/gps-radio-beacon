#ifndef _ITSYBITSY_M4_CONFIG_H_
#define _ITSYBITSY_M4_CONFIG_H_

// ItsyBitsy M4 Express Configuration
// SAMD51 ARM Cortex-M4 @ 120MHz

#include <Arduino.h>

// System clock configuration  
#define F_CPU 120000000UL  // 120MHz

// Pin assignments for ItsyBitsy M4 Express
// E22-400M33S LoRa Module (SX1268) - SPI Interface
#define LORA_CS             5   // SPI Chip Select
#define LORA_DIO1           9   // DIO1 interrupt pin (TxDone/RxDone)
#define LORA_BUSY           7   // BUSY status pin (NOTE: No pin 6 on ItsyBitsy M4!)
#define LORA_NRST           11  // Reset pin
#define LORA_TXEN           12  // TX enable (optional)
// SPI pins (hardware SPI): MOSI=MOSI, MISO=MISO, SCK=SCK on ItsyBitsy M4

// BNO085 IMU - I2C Interface for launch detection
#define BNO08X_RESET        10  // IMU reset pin (optional, can be -1 if not used)
// I2C pins: SDA and SCL are hardware I2C on ItsyBitsy M4

#define PIN_GPS_RX          0   // Hardware Serial1 RX (not needed, defined for reference)
#define PIN_GPS_TX          1   // Hardware Serial1 TX (not needed, defined for reference)

// GPS UART uses Hardware Serial1
#define GPS_SERIAL Serial1

// BNO085 IMU Configuration
#define IMU_I2C_ADDR        0x4A        // I2C address (0x4A or 0x4B depending on SA0 pin)
#define LAUNCH_ACCEL_THRESHOLD  20.0    // Launch detection threshold in m/s² (~2g)
#define LAUNCH_ACCEL_DURATION   100     // Minimum duration above threshold (ms)
#define LAUNCH_SETTLE_TIME      2000    // Wait time after power-on for IMU to settle (ms)

// Callsign - Update this with your ham radio callsign
#define BEACON_CALLSIGN "KE0MZS"

// LoRa Configuration for E22-400M33S (SX1268)
#define LORA_FREQUENCY      433.0       // 433 MHz
#define LORA_BANDWIDTH      125.0       // 125 kHz bandwidth
#define LORA_SPREADING      9           // Spreading Factor 9 (good range/speed balance)
#define LORA_CODING_RATE    7           // Coding Rate 4/7
#define LORA_SYNC_WORD      0x12        // Private sync word (0x12 = private, 0x34 = LoRaWAN)
#define LORA_TX_POWER       22          // 22 dBm (~160mW - SX1268 chip maximum)
#define LORA_PREAMBLE       8           // Preamble length
#define LORA_TCXO_VOLTAGE   1.8         // E22-400M33S uses 1.8V TCXO

#endif // _ITSYBITSY_M4_CONFIG_H_
