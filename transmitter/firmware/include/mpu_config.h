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
// Multi-rocket channel plan: 8 channels at 250 kHz spacing, 433.00-434.75 MHz.
//   CH0=433.00 CH1=433.25 CH2=433.50 CH3=433.75
//   CH4=434.00 CH5=434.25 CH6=434.50 CH7=434.75
// The plan sits in the 70cm auxiliary/link segment (433-435 MHz): full
// Technician privileges, clear of the 432-433 weak-signal/EME segment and
// the 435-438 amateur-satellite segment. 250 kHz spacing leaves one full
// signal bandwidth (125 kHz) of guard between adjacent channels, and every
// channel is inside one SX1268 image-calibration band (430-440 MHz).
// Must match the receiver's plan in receiver/firmware/inc/lora.h.
//
// Rocket ID and channel are INDEPENDENT: any airframe can fly on any
// channel. Set either or both from the build command without editing this
// file:  make flash ID=3 CHANNEL=5   (see Makefile).
#define LORA_CHANNEL_COUNT  8
#ifndef LORA_CHANNEL
#define LORA_CHANNEL        0           // 0-7, frequency slot for this flight
#endif
#define LORA_CHANNEL_FREQ(ch) (433.0f + 0.25f * (float)(ch))  // Channel -> MHz

#if LORA_CHANNEL < 0 || LORA_CHANNEL >= LORA_CHANNEL_COUNT
#error "LORA_CHANNEL must be 0..7"
#endif

// Rocket ID, transmitted with the callsign as "CALLSIGN-<id> CH<n>" and in
// heartbeats, so the receiver operator can confirm which airframe they are
// tracking. Identifies the AIRFRAME and never changes with frequency: the
// CH<n> suffix (and the heartbeat channel field) show the active channel.
#ifndef ROCKET_ID
#define ROCKET_ID           0
#endif

// Backup channel jumper: short CHANNEL_JUMPER_PIN to GND before power-on to
// transmit on LORA_CHANNEL_BACKUP instead of LORA_CHANNEL. Lets you move a
// rocket off a channel that is occupied at the pad without reflashing.
// The pin is read once in radio_init() with the internal pull-up enabled:
// open = primary channel, jumpered to GND = backup channel.
// Default backup = +4 channels (1 MHz away, wraps), far enough that a noisy
// primary won't also cover the backup.
#define CHANNEL_JUMPER_PIN  2                           // Free GPIO on ItsyBitsy M4
#define LORA_CHANNEL_BACKUP ((LORA_CHANNEL + 4) % LORA_CHANNEL_COUNT)

// Primary-channel frequency, for reference only: radio.cpp resolves the
// active channel (primary vs backup jumper) at boot.
#define LORA_FREQUENCY      LORA_CHANNEL_FREQ(LORA_CHANNEL)
#define LORA_BANDWIDTH      125.0       // 125 kHz bandwidth
#define LORA_SPREADING      9           // Spreading Factor 9 (good range/speed balance)
#define LORA_CODING_RATE    7           // Coding Rate 4/7
#define LORA_SYNC_WORD      0x12        // Private sync word (0x12 = private, 0x34 = LoRaWAN)
#define LORA_TX_POWER       22          // 22 dBm (~160mW - SX1268 chip maximum)
// Preamble length: 16 symbols (~66 ms at SF9/BW125) instead of the LoRa
// default 8. The receiver's boot scan sniffs each channel with CAD
// (~20 ms) in a ~0.3 s lap over all 8 channels; a longer preamble roughly
// doubles the chance a lap catches a transmission mid-preamble, at a cost
// of +33 ms airtime per packet. Reception is unaffected: the RX decodes
// any preamble length, and older 8-symbol beacons still work (the scan's
// dwell fallback finds them within one 52 s lap).
#define LORA_PREAMBLE       16          // Preamble length (symbols)
#define LORA_TCXO_VOLTAGE   1.8         // E22-400M33S uses 1.8V TCXO

#endif // _ITSYBITSY_M4_CONFIG_H_
