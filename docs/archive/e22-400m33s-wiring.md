# E22-400M33S LoRa Module Wiring Guide

## Module Overview

**E22-400M33S** - Ebyte LoRa module
- Chip: Semtech SX1268
- Frequency: 410-493 MHz (configured for 433 MHz)
- Power: Up to 30 dBm (1W) - configured for 22 dBm
- Interface: SPI
- Range: Up to 15km line-of-sight with good antennas

## Pin Connections

### ItsyBitsy M4 Express → E22-400M33S

| ItsyBitsy M4 | E22-400M33S | Function |
|--------------|-------------|----------|
| **5** | NSS | SPI Chip Select |
| **9** | DIO1 | Interrupt (TX/RX Done) |
| **6** | BUSY | Module Busy Status |
| **11** | NRST | Reset |
| **12** | TXEN | TX Enable (optional) |
| **MOSI** | MOSI | SPI Data Out |
| **MISO** | MISO | SPI Data In |
| **SCK** | SCK | SPI Clock |
| **3.3V** | VCC | Power (3.3V) |
| **GND** | GND | Ground |

### Hardware SPI Pins on ItsyBitsy M4
- **MOSI**: Pin labeled MOSI on board
- **MISO**: Pin labeled MISO on board  
- **SCK**: Pin labeled SCK on board

## Wiring Notes

1. **Power**: E22-400M33S needs **3.3V**. ItsyBitsy M4 has a 3.3V output pin.
2. **Antenna**: **CRITICAL** - Always connect a 433MHz antenna before powering on! Operating without an antenna can damage the RF amplifier.
3. **TX Enable**: Pin 12 (TXEN) is optional. Some E22 modules don't use this pin.
4. **Logic Levels**: All signals are 3.3V compatible (perfect for ItsyBitsy M4).

## LoRa Configuration

Current settings in `include/mpu_config.h`:

```cpp
#define LORA_FREQUENCY      433.0       // 433 MHz
#define LORA_BANDWIDTH      125.0       // 125 kHz bandwidth
#define LORA_SPREADING      9           // Spreading Factor 9
#define LORA_CODING_RATE    7           // Coding Rate 4/7
#define LORA_SYNC_WORD      0x12        // Private network
#define LORA_TX_POWER       22          // 22 dBm (~160mW)
#define LORA_PREAMBLE       8           // 8 symbols
```

### Range vs Speed Trade-offs

**Current Settings (SF9, BW125):**
- Range: ~5-8 km line-of-sight
- Speed: ~2 kbps
- Air time: ~1 second per packet
- Good balance for rocketry

**For Maximum Range (SF12, BW125):**
```cpp
#define LORA_SPREADING      12          // Slower but longer range
```
- Range: ~10-15 km line-of-sight
- Speed: ~250 bps
- Air time: ~4 seconds per packet

**For Faster Updates (SF7, BW250):**
```cpp
#define LORA_SPREADING      7
#define LORA_BANDWIDTH      250.0
```
- Range: ~2-3 km
- Speed: ~11 kbps
- Air time: ~200ms per packet

## Antenna Requirements

### For 433 MHz:
- **Quarter-wave**: 17.3 cm straight wire
- **Half-wave dipole**: 34.6 cm total (17.3 cm each side)
- **Commercial**: Any 433 MHz LoRa antenna (spring or whip)

**WARNING**: Never transmit without an antenna connected!

## Power Consumption

- **Standby**: ~2 mA
- **Receiving**: ~15 mA
- **Transmitting (22 dBm)**: ~120 mA

The firmware puts the radio in standby between transmissions to save power.

## Packet Format

Same as before - fully compatible with existing receiver:

```
$$lat,lon,alt,sats$$CHECKSUM
```

Example:
```
$$3953.40363,-10506.93605,1671.7,12$$2F
```

The LoRa layer adds:
- Forward Error Correction (FEC)
- CRC checking
- Preamble detection
- Much better sensitivity than simple FSK

## Testing

### Monitor Serial Output

```bash
make flash
make monitor
```

You should see:
```
[Radio] Initializing E22-400M33S (SX1268)...
[Radio] ✓ Initialized successfully
[Beacon] Transmitting callsign: KE0MZS
[Beacon] Transmitting GPS packet: $$...$$XX
[Radio] ✓ Transmitted XX bytes
```

### Verify with LoRa Receiver

On the receiver side, you'll need to update to use LoRa as well. The receiver should:
1. Use same frequency (433.0 MHz)
2. Use same spreading factor (9)
3. Use same bandwidth (125 kHz)
4. Use same sync word (0x12)

## Troubleshooting

### "Initialization failed, code: -2"
- Check SPI connections (MOSI, MISO, SCK)
- Verify NSS (CS) pin connection
- Check 3.3V power supply

### "Transmission failed, code: -7"
- Module is BUSY - normal during transmission
- Wait for transmission to complete

### No signal on receiver
- Verify antenna is connected
- Check frequency matches on both sides
- Confirm spreading factor and bandwidth match
- Try increasing TX power to 30 dBm (max):
  ```cpp
  #define LORA_TX_POWER       30
  ```

### Range is poor
- Check antenna connection and quality
- Increase spreading factor (9 → 10 → 11 → 12)
- Ensure good line-of-sight
- Check for RF interference

## Receiver Updates Needed

Your receiver (STM32F401) will also need to be updated to use LoRa. Key changes:

1. Replace simple RF receiver with E22-400M33S or compatible
2. Install RadioLib on receiver
3. Configure for receive mode with matching parameters
4. Parse LoRa packets (same `$$...$$` format inside)

The existing packet parser will work - just the physical layer changes from FSK to LoRa.

## Advantages Over Simple 433MHz

✅ **10-15x better range** - typical 5-8km vs ~500m  
✅ **Error correction** - FEC recovers corrupted bits  
✅ **Better sensitivity** - Can receive weaker signals  
✅ **CRC built-in** - Automatic packet validation  
✅ **Frequency hopping** - Can implement for reliability  
✅ **Commercial modules** - Cheap, widely available  

## Legal/Regulatory

**433 MHz ISM Band** (varies by country):
- **US**: 433.05-434.79 MHz, max 1W
- **EU**: 433.05-434.79 MHz, max 10 mW (10 dBm)
- **Check local regulations** before use

Current config uses 22 dBm (~160mW) which is legal in US but may need reduction in EU:
```cpp
#define LORA_TX_POWER       10  // For EU compliance
```

## References

- [RadioLib Documentation](https://jgromes.github.io/RadioLib/)
- [SX1268 Datasheet](https://www.semtech.com/products/wireless-rf/lora-core/sx1268)
- [E22-400M33S Manual](https://www.ebyte.com/en/product-view-news.html?id=1123)
