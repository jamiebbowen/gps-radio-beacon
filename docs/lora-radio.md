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
- Changed from interrupt-driven UART to polled LoRa packet reception
- Packet format remains the same: `$$data$$CHKSUM`
- Maintains compatibility with existing parser

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
| PA7         | GPIO     | BUSY            | Busy status |
| PA6         | GPIO_INT | DIO1            | Interrupt (RX/TX done) |
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
| Frequency | 433.0 MHz | ISM band |
| Bandwidth | 125 kHz | Standard LoRa BW |
| Spreading Factor | 9 | SF9 (good range/speed balance) |
| Coding Rate | 4/7 | Forward error correction |
| Sync Word | 0x12 | Private network |
| TX Power | 22 dBm | Maximum for E22-400M33S |
| Preamble Length | 8 symbols | Standard |
| CRC | Enabled | Packet integrity check |

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

### Range Improvements
- **Previous (300 baud UART):** ~500m line-of-sight
- **LoRa SF9 @ 433MHz:** ~2-5km line-of-sight, ~10-15km with good antennas

### Link Budget
- TX Power: +22 dBm
- RX Sensitivity (SF9): ~-123 dBm
- Link Budget: ~145 dB (excellent for rocketry applications)

### Data Rate
- SF9 @ 125kHz: ~1.76 kbps effective
- Typical packet (60 bytes): ~270ms transmission time
- Much faster than previous 300 baud implementation

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
