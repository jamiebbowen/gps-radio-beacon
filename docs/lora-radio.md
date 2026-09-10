# E22-400M33S LoRa Module Conversion

## Overview
Successfully converted the GPS radio beacon receiver from UART-based RF (RXM-433-LR) to E22-400M33S LoRa module (SX1268 chipset). The transmitter was already using the E22-400M33S module.

## Changes Made

### 1. Created LoRa Driver (`lora.h` / `lora.c`)
- Complete SX1268 driver for STM32F4
- SPI-based communication
- Implements LoRa packet TX/RX
- Automatic receive mode
- RSSI and SNR measurement
- IRQ-based packet detection

### 2. Updated RF Receiver (`rf_receiver.c`)
- Replaced UART initialization with SPI initialization
- Changed from interrupt-driven UART to DIO1-interrupt-driven LoRa packet reception (EXTI on DIO1 sets a flag; the main loop drains the packet over SPI)
- Packet format at the time of conversion was `$$data$$CHKSUM`; the current
  firmware uses binary typed packets (see `transmitter/firmware/include/rf_packet.h`)

### 3. Removed UART Dependencies
- Removed `RF_Receiver_UART_RxCpltCallback()` from gps.c
- Deprecated UART calibration functions (kept for compatibility)

### 4. Updated Build System
- Added `lora.c` to Makefile
- All files compile successfully

## Hardware Connections

### STM32F4 Black Pill → E22-400M33S LoRa Module

| STM32F4 Pin | Function | E22-400M33S Pin | Description |
|-------------|----------|-----------------|-------------|
| PB13        | SPI2_SCK | SCK             | SPI Clock |
| PB14        | SPI2_MISO| MISO            | SPI Data Out |
| PB15        | SPI2_MOSI| MOSI            | SPI Data In |
| PB12        | GPIO     | NSS/CS          | Chip Select |
| PA8         | GPIO     | NRST            | Reset (active low) |
| PB0         | GPIO     | BUSY            | Busy status (moved from PA7: SD conflict) |
| PB1         | GPIO_INT | DIO1            | Interrupt (RX/TX done) (moved from PA6: SD conflict) |
| PA9         | GPIO     | RXEN            | RF switch RX enable |
| 3.3V        | Power    | VCC             | Power supply |
| GND         | Ground   | GND             | Ground |

**Important Notes:**
- E22-400M33S operates at 3.3V - do not use 5V
- Antenna must be connected to ANT pin
- DIO1 is configured for interrupt on rising edge

## LoRa Configuration

Both transmitter and receiver use identical settings:

| Parameter | Value | Description |
|-----------|-------|-------------|
| Frequency | 433.0 MHz | ISM band (8-channel plan, 433.00-434.75 MHz) |
| Bandwidth | 62.5 kHz | Narrow: +3 dB sensitivity, ~0.82 s/fused packet (19 B), LDRO on |
| Spreading Factor | 10 | SF10 (range over airtime) |
| Coding Rate | 4/6 | Forward error correction (-18% airtime vs 4/8, -0.7 dB) |
| Sync Word | 0x12 | Private network; RX programs it explicitly (reg 0x0740 = `14 24`) |
| TX Power | 22 dBm | SX1268 chip output, which is exactly what drives the M33S's internal PA to its rated 33 dBm / 2W (verified module variant; chip drive past ~20 dBm only grows harmonics) |
| Preamble Length | 8 symbols | Standard; CAD+62 s re-acquire dwell cover lock robustness |
| CRC | Enabled | Packet integrity check |

Receiver-specific radio tuning (lora.c): boosted RX gain (reg 0x08AC = 0x96,
~+3 dB vs default power-saving) and band-specific image calibration for
430-440 MHz (CalibrateImage 0x6B/0x6F). Both were missing until the 2026-09
range investigation and matter most at the marginal end of the link.

Transmitter-specific tuning (radio.cpp): the PA over-current protection
(LORA_TX_CURRENT_MA = 140 mA) must be raised after radio.begin() - both the
chip reset default and RadioLib's begin() leave it at 60 mA, which clamps
every 22 dBm burst mid-packet. Note the current budget consequence: at full
PA drive the module can pull on the order of 1 A per packet; the ItsyBitsy
M4's 600 mA 3.3 V rail should be checked for droop under TX before flight
(scope or brown-out counter), or the module given its own heftier supply.

## Packet Format

The packet format remains unchanged from the previous implementation:

### GPS Data Packet
```
$$lat,lon,alt,sats,fix,launch,callsign$$CHKSUM
```
Example: `$$3953.40363,-10506.93605,1671.7,12,1,45,KE0MZS$$2F`

### Callsign Packet
```
$$CALLSIGN$$
```
Example: `$$KE0MZS$$`

## Testing Checklist

Before deploying, verify:

- [ ] LoRa module properly connected (check pin connections)
- [ ] Antenna connected to E22-400M33S ANT pin
- [ ] Firmware flashes successfully
- [ ] RF display mode shows packet reception
- [ ] RSSI and SNR values displayed
- [ ] GPS data correctly parsed from LoRa packets
- [ ] Navigation mode works with LoRa-received coordinates

## Expected Performance

### Range Expectations
- **Previous (300 baud UART):** ~500m line-of-sight
- **LoRa SF10/CR4-6/BW62.5k @ 433MHz, boosted RX gain:** datasheet floor ~-133 dBm; real-world
  range depends mostly on antenna installation and the receiver-side noise
  floor (park walk tests 2026-09: ~2.8 km through trees at ~6 dB above the
  local -70 dBm ambient floor)

### Link Budget
- TX Power: +22 dBm (chip max; module PA not driven)
- RX Sensitivity (SF10, BW62.5, CR4/6): ~-133 dBm datasheet, before local noise floor
- Link Budget: ~150 dB on paper; in practice capped by the RX ambient floor

### Data Rate
- SF10/CR4-6 @ 62.5kHz: ~0.61 kbps effective (fused packet ~0.82 s airtime)
- Flight phase streams fused updates every 1.2 s (FUSED_TX_INTERVAL_MS);
  the raw GPS backup stream interleaves continuously
- Much better edge-of-range behavior than previous 300 baud implementation

## Benefits of LoRa

1. **Better Range:** 5-10x improvement over simple OOK modulation
2. **Noise Immunity:** Spread spectrum is very resistant to interference
3. **Lower Power:** More efficient than continuous transmission
4. **RSSI/SNR:** Real-time link quality metrics
5. **CRC Validation:** Built-in packet integrity checking

## Troubleshooting

### No Packets Received
1. Check SPI connections (MOSI/MISO/SCK/CS)
2. Verify BUSY pin not stuck high
3. Ensure both radios use same frequency/SF/BW
4. Check antenna connections on both ends

### Poor RSSI
1. Check antenna connections
2. Verify antenna is appropriate for 433 MHz
3. Ensure line-of-sight or minimal obstructions
4. Check for local interference (WiFi, etc.)

### Packet Errors
1. Monitor CRC error count
2. Check for frequency offset
3. Verify transmitter/receiver settings match
4. Consider reducing bandwidth or increasing SF for longer range

## Files Modified

### New Files
- `receiver/firmware/inc/lora.h` - LoRa driver header
- `receiver/firmware/src/lora.c` - LoRa driver implementation

### Modified Files
- `receiver/firmware/src/rf_receiver.c` - Switched to LoRa
- `receiver/firmware/inc/rf_receiver.h` - Updated header comments
- `receiver/firmware/src/gps.c` - Removed RF UART callback
- `receiver/firmware/Makefile` - Added lora.c to build
- `LORA_CONVERSION.md` - This documentation

## Build Status
✅ Firmware compiles successfully
✅ No undefined references
✅ Binary size: 73,760 bytes text + 568 bytes data + 7,096 bytes BSS = 81,424 bytes total

## Next Steps

1. Flash updated firmware to STM32F4
2. Connect E22-400M33S module per pin mapping above
3. Power on and verify LoRa initialization on display
4. Test packet reception from transmitter
5. Verify GPS parsing and navigation modes
6. Field test for range performance
