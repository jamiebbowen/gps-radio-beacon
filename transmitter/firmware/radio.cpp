#include "include/radio.h"
#include "include/mpu_config.h"
#include <Arduino.h>
#include <RadioLib.h>

// Create SX1268 radio object
// Using hardware SPI and defined pins
SX1268 radio = new Module(LORA_CS, LORA_DIO1, LORA_NRST, LORA_BUSY);

// Track radio state
static bool radio_initialized = false;
static bool radio_enabled = false;
static uint8_t init_retry_count = 0;
static const uint8_t MAX_INIT_RETRIES = 3;

/**
 * Initialize the E22-400M33S LoRa module (SX1268)
 */
void radio_init(void) {
    Serial.println("[Radio] Initializing E22-400M33S (SX1268)...");
    Serial.print("[Radio] Pins: CS=");
    Serial.print(LORA_CS);
    Serial.print(" DIO1=");
    Serial.print(LORA_DIO1);
    Serial.print(" RST=");
    Serial.print(LORA_NRST);
    Serial.print(" BUSY=");
    Serial.println(LORA_BUSY);
    
    // Initialize SPI
    SPI.begin();
    Serial.println("[Radio] SPI initialized");
    
    // Manual reset sequence - some modules need this before radio.begin()
    pinMode(LORA_NRST, OUTPUT);
    digitalWrite(LORA_NRST, LOW);
    delay(10);
    digitalWrite(LORA_NRST, HIGH);
    delay(100);  // Wait for module to boot
    Serial.println("[Radio] Manual reset complete");
    
    // Check BUSY pin state
    pinMode(LORA_BUSY, INPUT);
    Serial.print("[Radio] BUSY pin state: ");
    Serial.println(digitalRead(LORA_BUSY) ? "HIGH" : "LOW");
    
    // Initialize radio module
    Serial.println("[Radio] Calling radio.begin()...");
    Serial.print("[Radio] TCXO voltage: ");
    Serial.print(LORA_TCXO_VOLTAGE);
    Serial.println("V");
    int state = radio.begin(
        LORA_FREQUENCY,     // Frequency in MHz
        LORA_BANDWIDTH,     // Bandwidth in kHz
        LORA_SPREADING,     // Spreading factor
        LORA_CODING_RATE,   // Coding rate
        LORA_SYNC_WORD,     // Sync word
        LORA_TX_POWER,      // Output power in dBm
        LORA_PREAMBLE,      // Preamble length
        LORA_TCXO_VOLTAGE   // TCXO voltage for E22-400M33S
    );
    
    Serial.print("[Radio] radio.begin() returned: ");
    Serial.println(state);
    
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println("[Radio] ✓ Initialized successfully");
        radio_initialized = true;
        
        // Set to standby mode initially
        radio.standby();
        radio_enabled = false;
    } else {
        Serial.print("[Radio] ✗ Initialization failed, code: ");
        Serial.println(state);
        Serial.println("[Radio] Common error codes:");
        Serial.println("[Radio]   -2 = CHIP_NOT_FOUND");
        Serial.println("[Radio]   -3 = PACKET_TOO_LONG");
        Serial.println("[Radio]   -4 = TX_TIMEOUT");
        Serial.println("[Radio]   -5 = RX_TIMEOUT");
        Serial.println("[Radio]   -6 = CRC_MISMATCH");
        Serial.println("[Radio]   -7 = INVALID_BANDWIDTH");
        Serial.println("[Radio] Check wiring, power, and SPI connections!");
        radio_initialized = false;
    }
}

/**
 * Enable radio for transmission
 * On LoRa modules, this just sets a flag - actual TX happens per packet
 */
void radio_enable(void) {
    if (!radio_initialized) {
        Serial.println("[Radio] Warning: Enable called but not initialized");
        
        // Attempt to reinitialize if we haven't exceeded retry limit
        if (init_retry_count < MAX_INIT_RETRIES) {
            init_retry_count++;
            Serial.print("[Radio] Attempting reinitialize (attempt ");
            Serial.print(init_retry_count);
            Serial.print("/");
            Serial.print(MAX_INIT_RETRIES);
            Serial.println(")...");
            
            delay(100); // Brief delay before retry
            radio_init();
            
            // Check if initialization succeeded
            if (!radio_initialized) {
                Serial.println("[Radio] Reinitialize failed");
                return;
            } else {
                Serial.println("[Radio] Reinitialize succeeded!");
                // Continue to enable below
            }
        } else {
            Serial.println("[Radio] Max retry attempts reached, giving up");
            return;
        }
    }
    
    // For LoRa, we don't need to explicitly enable
    // Just set a flag that we're ready to transmit
    radio_enabled = true;
    
    // Optionally control TXEN pin if your module uses it
    #ifdef LORA_TXEN
    pinMode(LORA_TXEN, OUTPUT);
    digitalWrite(LORA_TXEN, HIGH);
    #endif
}

/**
 * Disable radio (put in standby mode)
 */
void radio_disable(void) {
    if (!radio_initialized) {
        return;
    }
    
    radio.standby();
    radio_enabled = false;
    
    #ifdef LORA_TXEN
    digitalWrite(LORA_TXEN, LOW);
    #endif
}

/**
 * Transmit a packet via LoRa
 * @param data Pointer to data buffer
 * @param length Length of data in bytes
 * @return RADIOLIB_ERR_NONE on success, error code otherwise
 */
int transmit_packet(const uint8_t* data, size_t length) {
    if (!radio_initialized) {
        Serial.println("[Radio] Error: Not initialized");
        return RADIOLIB_ERR_CHIP_NOT_FOUND;
    }
    
    if (!radio_enabled) {
        Serial.println("[Radio] Warning: Transmitting while disabled");
    }
    
    // Transmit the packet
    int state = radio.transmit(const_cast<uint8_t*>(data), length);
    
    if (state == RADIOLIB_ERR_NONE) {
        Serial.print("[Radio] ✓ Transmitted ");
        Serial.print(length);
        Serial.println(" bytes");
    } else {
        Serial.print("[Radio] ✗ Transmission failed, code: ");
        Serial.println(state);
    }
    
    return state;
}

/**
 * Transmit a null-terminated string via LoRa
 * @param str String to transmit
 * @return RADIOLIB_ERR_NONE on success, error code otherwise
 */
int transmit_string(const char* str) {
    if (!str) {
        return RADIOLIB_ERR_INVALID_ENCODING;
    }
    
    size_t length = strlen(str);
    return transmit_packet((const uint8_t*)str, length);
}

/**
 * Check if radio is currently transmitting
 * @return true if transmitting, false otherwise
 */
bool radio_is_transmitting(void) {
    if (!radio_initialized) {
        return false;
    }
    
    // Check if radio is busy transmitting
    // The BUSY pin goes high during transmission
    return digitalRead(LORA_BUSY);
}
