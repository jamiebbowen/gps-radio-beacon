#include "include/radio.h"
#include "include/mpu_config.h"
#include "include/config.h"
#include "include/packet_format.h"
#include <Arduino.h>
#include <RadioLib.h>

// Create SX1268 radio object
// Using hardware SPI and defined pins
SX1268 radio = new Module(LORA_CS, LORA_DIO1, LORA_NRST, LORA_BUSY);

// Track radio state
static bool radio_initialized = false;
static bool radio_enabled = false;
static uint8_t active_channel = LORA_CHANNEL;
static uint8_t init_retry_count = 0;
static const uint8_t MAX_INIT_RETRIES = 3;

/* Mid-flight wedge recovery: transmit failures were previously logged and
 * dropped, so a chip that wedges (BUSY stuck, EMI/SPI glitch from the
 * 33 dBm PA) would leave the beacon permanently silent while the main loop
 * runs on - the 16 s WDT never sees it. Count consecutive failures; past
 * the limit force a full radio_init() (NRST pulse + begin()), the only
 * reset the SX1268 honours once wedged. */
static uint8_t tx_fail_streak = 0;
#define TX_FAIL_STREAK_LIMIT 8

/* E22-400M33S RF switch control (PCB v2: TXEN on pin 12, RXEN on pin 13,
 * both MCU-driven; DIO2 is NOT used as switch control). Truth table:
 *   RX mode = RXEN 1 / TXEN 0,  TX mode = RXEN 0 / TXEN 1,  idle = both 0.
 * Break-before-make on every transition so the PA output is never routed
 * into the LNA path. Defaults to the RX side up in enable() - same idle
 * convention as the receiver firmware. */
static void rf_switch_rx(void) {
#ifdef LORA_TXEN
    digitalWrite(LORA_TXEN, LOW);
#endif
#ifdef LORA_RXEN
    digitalWrite(LORA_RXEN, HIGH);
#endif
}

static void rf_switch_tx(void) {
#ifdef LORA_RXEN
    digitalWrite(LORA_RXEN, LOW);
#endif
#ifdef LORA_TXEN
    digitalWrite(LORA_TXEN, HIGH);
#endif
}

static void rf_switch_idle(void) {  /* antenna isolated */
#ifdef LORA_RXEN
    digitalWrite(LORA_RXEN, LOW);
#endif
#ifdef LORA_TXEN
    digitalWrite(LORA_TXEN, LOW);
#endif
}

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

    // RF switch control pins: start isolated (both enables LOW) until a TX
    // burst or RX window claims the antenna. DIO2 stays an input (default).
    rf_switch_idle();
    Serial.println("[Radio] RF switch parked (RXEN=0 TXEN=0)");
    
    // Resolve the active channel from the backup-channel jumper:
    // open (pull-up reads HIGH) = primary, shorted to GND = backup.
    pinMode(CHANNEL_JUMPER_PIN, INPUT_PULLUP);
    delay(1);  // Let the pull-up settle before sampling
    bool backup_jumper = (digitalRead(CHANNEL_JUMPER_PIN) == LOW);
    active_channel = backup_jumper ? LORA_CHANNEL_BACKUP : LORA_CHANNEL;
    float freq_mhz = LORA_CHANNEL_FREQ(active_channel);
    Serial.print("[Radio] Channel: CH");
    Serial.print(active_channel);
    Serial.print(backup_jumper ? " (backup jumper) " : " (primary) ");
    Serial.print(freq_mhz);
    Serial.println(" MHz");
    
    // Initialize radio module
    Serial.println("[Radio] Calling radio.begin()...");
    Serial.print("[Radio] TCXO voltage: ");
    Serial.print(LORA_TCXO_VOLTAGE);
    Serial.println("V");
    int state = radio.begin(
        freq_mhz,           // Frequency in MHz (jumper-resolved channel)
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
        init_retry_count = 0;   /* re-arm the radio_enable() retry ladder */
        tx_fail_streak = 0;

        /* RadioLib begin() leaves the PA over-current limit at 60 mA; a
         * 22 dBm burst needs roughly double that. See LORA_TX_CURRENT_MA in
         * mpu_config.h. Failure is a warning, not an init failure: the radio
         * still works at the clamped power. */
        int16_t lim_state = radio.setCurrentLimit(LORA_TX_CURRENT_MA);
        if (lim_state != RADIOLIB_ERR_NONE) {
            Serial.print(F("[Radio] Warning: setCurrentLimit failed, code: "));
            Serial.println(lim_state);
        }

        /* Chip-side airtime sanity: the fused cadence must clear the
         * fused packet's time-on-air with a real gap, or the flight phases
         * become back-to-back blocking TX with the PA at ~100% duty. Format
         * or radio-parameter drift shows up here at boot now, instead of as
         * a thermally wedged radio on recovery day. */
        uint32_t toa_ms = (radio.getTimeOnAir(FUSED_PACKET_SIZE) + 999) / 1000;
        Serial.print(F("[Radio] Fused time-on-air: "));
        Serial.print(toa_ms);
        Serial.print(F(" ms, cadence "));
        Serial.print(FUSED_TX_INTERVAL_MS);
        Serial.println(F(" ms"));
        if (toa_ms + 100 > (uint32_t)FUSED_TX_INTERVAL_MS) {
            Serial.println(F("[Radio] WARNING: fused cadence gap under 100 ms - PA duty ~100%"));
        }

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
 * Get the active rocket channel (after backup-jumper resolution)
 */
uint8_t radio_get_channel(void) {
    return active_channel;
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

    // RF switch parked on the RX side; transmit_packet() and radio_poll_rx()
    // steer it per operation from here on.
    rf_switch_rx();
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

    rf_switch_idle();
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
        /* Dead chip: keep attempting a full re-init every STREAK_LIMIT
         * calls so a transient radio brownout self-heals. */
        if (++tx_fail_streak >= TX_FAIL_STREAK_LIMIT) {
            Serial.println(F("[Radio] Not-initialized streak - forcing full re-init"));
            tx_fail_streak = 0;
            radio_init();
        }
        return RADIOLIB_ERR_CHIP_NOT_FOUND;
    }
    
    if (!radio_enabled) {
        Serial.println("[Radio] Warning: Transmitting while disabled");
    }

    // Route the antenna to the PA for the burst, then back to the RX side.
    rf_switch_tx();
    int state = radio.transmit(const_cast<uint8_t*>(data), length);
    rf_switch_rx();

    if (state == RADIOLIB_ERR_NONE) {
        Serial.print("[Radio] ✓ Transmitted ");
        Serial.print(length);
        Serial.println(" bytes");
        tx_fail_streak = 0;
    } else {
        Serial.print("[Radio] ✗ Transmission failed, code: ");
        Serial.println(state);
        if (++tx_fail_streak >= TX_FAIL_STREAK_LIMIT) {
            Serial.println(F("[Radio] TX failure streak - forcing full re-init"));
            tx_fail_streak = 0;
            radio_initialized = false;
            radio_init();
        }
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

/**
 * Poll for a short inbound LoRa packet (two-way channel scaffolding).
 *
 * Called on the cadence gap between our own transmissions. Uses
 * RadioLib's receive-with-timeout, so it bounds the listen window to
 * timeout_ms of airtime and returns instantly if nothing is on the wire.
 * The opcode isn't message-scheduled: a packet sitting in the radio's
 * FIFO beyond this call is read out before returning.
 *
 * @param out      output buffer
 * @param max_len  buffer size; longer packets are truncated to fit
 * @param timeout_ms max listen window
 * @return number of bytes captured, 0 for none/timeout, <0 for error
 */
int radio_poll_rx(uint8_t *out, size_t max_len, uint16_t timeout_ms) {
    if (!radio_initialized || out == NULL || max_len == 0) {
        return 0;
    }

    /* Listening needs the radio powered with the RF switch on the LNA side.
     * Put the caller's power state back afterwards: between pad heartbeats
     * the radio is normally off, and leaving RXEN high kept the LNA powered
     * for nothing. */
    bool was_enabled = radio_enabled;
    if (!radio_enabled) radio_enable();

    rf_switch_rx();  // antenna on the LNA side for the listen window
    int result = 0;
    /* PREAMBLE_DETECTED on top of RadioLib's RX defaults: the command
     * uplink's long preamble is what tells us a frame is coming. */
    int state = radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF,
                                   RADIOLIB_IRQ_RX_DEFAULT_FLAGS |
                                       (1UL << RADIOLIB_IRQ_PREAMBLE_DETECTED),
                                   RADIOLIB_IRQ_RX_DEFAULT_MASK, 0);
    if (state != RADIOLIB_ERR_NONE) {
        result = state;          /* RadioLib error codes are already < 0 */
    } else {
        /* Poll the IRQ flags until the window closes. A detected preamble
         * or header stretches the window once to CMD_RX_EXTEND_MS: the
         * 40 ms sniff usually wakes up in the middle of the receiver's long
         * preamble, and standby() there would abort the frame. */
        uint32_t start = millis();
        uint32_t window = timeout_ms;
        bool extended = false;
        while (millis() - start < window) {
            uint32_t flags = radio.getIrqFlags();
            if (flags & RADIOLIB_SX126X_IRQ_RX_DONE) {
                if (!(flags & RADIOLIB_SX126X_IRQ_CRC_ERR)) { /* corrupt: ignore */
                    size_t n = radio.getPacketLength();
                    if (n > max_len) n = max_len;
                    state = radio.readData(out, n);
                    result = (state == RADIOLIB_ERR_NONE) ? (int)n : state;
                }
                break;
            }
            if (flags & (RADIOLIB_SX126X_IRQ_TIMEOUT | RADIOLIB_SX126X_IRQ_HEADER_ERR)) break;
            if (!extended && (flags & (RADIOLIB_SX126X_IRQ_PREAMBLE_DETECTED |
                                       RADIOLIB_SX126X_IRQ_HEADER_VALID))) {
                window = (CMD_RX_EXTEND_MS > timeout_ms) ? CMD_RX_EXTEND_MS : timeout_ms;
                extended = true;
            }
            delay(1);
        }
        radio.standby();
    }
    if (!was_enabled) radio_disable();
    return result;
}

/** Transmit a 6-byte ACK packet as an answer to a parsed command. */
int radio_transmit_ack(uint8_t rocket_id, uint8_t cmd_code,
                       uint8_t seq_hi, uint8_t seq_lo, uint8_t echo) {
    uint8_t buf[6];
    buf[0] = 0x09; /* PACKET_TYPE_ACK */
    buf[1] = rocket_id;
    buf[2] = cmd_code;
    buf[3] = seq_hi;
    buf[4] = seq_lo;
    buf[5] = echo;
    return transmit_packet(buf, sizeof(buf));
}
