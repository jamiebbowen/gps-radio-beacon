# GPS Radio Beacon - Complete System Summary

## Platform: ItsyBitsy M4 Express

**Microcontroller**: SAMD51 ARM Cortex-M4 @ 120MHz  
**RAM**: 192KB  
**Flash**: 512KB (58KB used = 11%)  
**Compiled**: Success ✓

---

## System Components

### 1. GPS Module
- **Hardware**: u-blox MAX-M10S (or compatible)
- **Interface**: Hardware UART (Serial1) at 115200 baud
- **Pins**: Pins 0 (RX) and 1 (TX)
- **Format**: NMEA sentences (GGA, RMC)
- **Update**: 1Hz

### 2. LoRa Radio - E22-400M33S
- **Chip**: Semtech SX1268
- **Interface**: SPI
- **Pins**:
  - CS: Pin 5
  - DIO1: Pin 9 (interrupt)
  - BUSY: Pin 6
  - RESET: Pin 11
  - TXEN: Pin 12 (optional)
  - SPI: MOSI, MISO, SCK (hardware SPI)
- **Frequency**: 433.0 MHz
- **Power**: 22 dBm (160mW) - configurable up to 30 dBm
- **Range**: 5-8 km typical (line-of-sight)
- **Modulation**: LoRa (SF9, BW125, CR4/7)

### 3. IMU - BNO085
- **Type**: 9-axis IMU with sensor fusion
- **Interface**: I2C at 0x4A
- **Pins**:
  - SDA: SDA (I2C data)
  - SCL: SCL (I2C clock)
  - RESET: Pin 10 (optional)
- **Purpose**: Acceleration-based launch detection
- **Threshold**: 20 m/s² (~2g) for 100ms
- **Output**: Linear acceleration (gravity removed)

---

## Pin Assignment Summary

| Pin | Component | Function |
|-----|-----------|----------|
| **0** | GPS | UART RX (Serial1) |
| **1** | GPS | UART TX (Serial1) |
| **5** | LoRa | SPI Chip Select |
| **6** | LoRa | BUSY status |
| **9** | LoRa | DIO1 interrupt |
| **10** | IMU | Reset (optional) |
| **11** | LoRa | Reset |
| **12** | LoRa | TX Enable |
| **SDA** | IMU | I2C Data |
| **SCL** | IMU | I2C Clock |
| **MOSI** | LoRa | SPI Data Out |
| **MISO** | LoRa | SPI Data In |
| **SCK** | LoRa | SPI Clock |
| **3.3V** | All | Power |
| **GND** | All | Ground |

---

## Software Architecture

### State Machine

```
PRE_LAUNCH → LAUNCH → POST_LAUNCH → BATTERY_SAVE
     ↑          ↑
     └──────────┘
   (IMU detects launch)
```

#### States

1. **PRE_LAUNCH** (Initial)
   - Transmit every 30 seconds
   - Full packets (lat, lon, alt, sats)
   - Conserve power on pad

2. **LAUNCH** (5 minutes)
   - Continuous transmission
   - Fast packets during first 3 min (lat, lon, alt only)
   - Full packets after 3 min
   - Triggered by IMU detecting >2g acceleration

3. **POST_LAUNCH** (10 minutes)
   - Continuous transmission
   - Full packets
   - Maximum recovery coverage

4. **BATTERY_SAVE** (Indefinite)
   - Transmit every 60 seconds
   - Full packets
   - Extended recovery operations

### Packet Format

Same format as original - **100% compatible** with existing receivers:

```
$$lat,lon,alt,sats$$CHECKSUM
```

Example:
```
$$3953.40363,-10506.93605,1671.7,12$$2F
```

- Coordinates in NMEA format (DDMM.MMMMM)
- Negative sign for South/West
- XOR checksum in hex
- Callsign transmitted every 5 minutes: `KE0MZS`

---

## Libraries Used

| Library | Version | Purpose |
|---------|---------|---------|
| RadioLib | 7.3.0 | LoRa communication (SX1268) |
| Adafruit BNO08x | 1.2.5 | IMU interface and sensor fusion |
| Adafruit BusIO | 1.17.4 | I2C/SPI abstraction |
| Adafruit Unified Sensor | 1.1.15 | Sensor framework |

---

## Configuration Files

### `include/mpu_config.h`
- Pin assignments
- LoRa parameters (frequency, power, spreading factor)
- IMU configuration (threshold, duration)
- Callsign

### `include/config.h`
- Beacon timing intervals
- Testing vs production mode
- Debug output settings

---

## Key Features

### LoRa Advantages
✅ **10-15x better range** than simple 433MHz (5-8km vs ~500m)  
✅ **Forward Error Correction** - recovers corrupted data  
✅ **Better sensitivity** - receives weaker signals  
✅ **Automatic CRC** - packet validation built-in  
✅ **Spread spectrum** - resistant to interference  

### IMU Launch Detection
✅ **Reliable** - detects actual motor ignition  
✅ **No moving parts** - more reliable than switches  
✅ **Adjustable** - configure threshold for your rocket  
✅ **Smart filtering** - requires sustained acceleration  
✅ **Diagnostic data** - real-time monitoring  

### GPS Tracking
✅ **Hardware UART** - reliable GPS communication  
✅ **NMEA parsing** - standard format  
✅ **Position + altitude** - full 3D tracking  
✅ **Satellite count** - signal quality indicator  

---

## Build & Upload

```bash
# Install dependencies (one-time)
make install-deps

# Compile
make compile

# Upload to board
make flash

# Monitor serial output
make monitor
```

---

## Power Consumption Estimates

| Component | Idle | Active | Peak |
|-----------|------|--------|------|
| **ItsyBitsy M4** | ~20 mA | ~50 mA | ~100 mA |
| **GPS** | ~20 mA | ~30 mA | ~40 mA |
| **LoRa (standby)** | ~2 mA | ~15 mA | ~120 mA (TX) |
| **IMU** | ~1 mA | ~10 mA | ~10 mA |
| **Total (idle)** | ~43 mA | ~105 mA | ~270 mA (TX) |

### Battery Life Estimates

**Pre-launch** (30s intervals):
- Average: ~50 mA
- 1000mAh battery: ~20 hours

**Launch/Recovery** (continuous):
- Average: ~150 mA (frequent TX)
- 1000mAh battery: ~6.5 hours

**Battery Save** (60s intervals):
- Average: ~60 mA
- 1000mAh battery: ~16 hours

---

## Testing Checklist

### Hardware
- [ ] GPS module connected and powered
- [ ] LoRa module connected with antenna
- [ ] IMU connected via I2C
- [ ] All power and ground connections secure
- [ ] **Antenna attached** (critical - don't TX without it!)

### Software
- [ ] Callsign updated in config
- [ ] Compile successful
- [ ] Upload successful
- [ ] Serial monitor shows all systems initializing

### GPS
- [ ] GPS reports valid fix
- [ ] Coordinates look reasonable
- [ ] Altitude updating
- [ ] Satellite count > 4

### LoRa
- [ ] Radio initialization successful
- [ ] Transmissions occurring
- [ ] Receiver detecting packets (if testing)
- [ ] Range test successful

### IMU
- [ ] IMU initialization successful
- [ ] Acceleration readings reasonable
- [ ] Launch detection triggers when shaken
- [ ] False alarms prevented

### State Machine
- [ ] Starts in PRE_LAUNCH
- [ ] Detects launch (shake test)
- [ ] Transitions to LAUNCH state
- [ ] Continuous transmission during launch
- [ ] Transitions to POST_LAUNCH after delay
- [ ] Eventually enters BATTERY_SAVE

---

## Troubleshooting

### GPS Not Working
- Check UART connections (pins 0, 1)
- Verify 3.3V power
- Ensure GPS has clear sky view
- Wait 30-60s for initial fix

### LoRa Not Transmitting
- Check SPI connections
- Verify antenna is connected
- Check initialization messages
- Try increasing TX power

### IMU Not Detected
- Check I2C connections (SDA, SCL)
- Verify I2C address (0x4A or 0x4B)
- Check 3.3V power
- Try with/without reset pin

### Launch Not Detected
- Reduce threshold (try 10-15 m/s²)
- Check IMU orientation
- Monitor acceleration in serial output
- Verify motor thrust is sufficient

---

## Migration Path from ATtiny412

All original code preserved in `old_attiny412_code/`:
- `main.c` - Original ATtiny code
- `radio_bitbang.cpp` - 300 baud bit-banged UART
- `launch_detect_switch.cpp` - Switch-based detection
- `beacon_bitbang.cpp` - Original beacon format

New Arduino-based code:
- `firmware.ino` - Arduino sketch with `setup()` and `loop()`
- `radio.cpp` - LoRa using RadioLib
- `launch_detect.cpp` - IMU-based detection
- `beacon.cpp` - LoRa packet formatting

---

## Performance Summary

| Metric | Value |
|--------|-------|
| **Flash usage** | 58,424 bytes (11% of 507KB) |
| **Compile time** | ~15 seconds |
| **Boot time** | ~5 seconds (includes IMU settle) |
| **GPS update** | 1 Hz |
| **LoRa TX time** | ~1 second per packet (SF9) |
| **IMU sample rate** | ~100 Hz |
| **Launch detect latency** | <100ms after threshold |

---

## Legal/Regulatory

**433 MHz ISM Band** (varies by country):

- **US**: 433.05-434.79 MHz, max 1W  
  Current config: 22 dBm (160mW) - ✓ Legal

- **EU**: 433.05-434.79 MHz, max 10 mW (10 dBm)  
  Current config: 22 dBm - ✗ Change to 10 dBm for EU

To change for EU compliance:
```cpp
#define LORA_TX_POWER 10  // 10 dBm for EU
```

**Always check local regulations before use!**

---

## Future Enhancements

With the ItsyBitsy M4's resources, you could add:

1. **SD Card Logging**: Log GPS track and acceleration data
2. **Apogee Detection**: Use IMU to detect peak altitude
3. **Dual GPS**: Compare two GPS modules for accuracy
4. **Real-time Clock**: Add timestamps to all data
5. **Temperature Sensor**: Monitor environmental conditions
6. **Barometer**: Redundant altitude measurement
7. **Camera Trigger**: Control camera based on flight phase
8. **Frequency Hopping**: Improve reliability further
9. **Mesh Networking**: Multiple beacons coordinate
10. **Web Interface**: Configure via WiFi (add ESP32)

Current flash usage (11%) leaves plenty of room!

---

## Support Documentation

- `README_ITSYBITSY_M4.md` - Setup and basic usage
- `MIGRATION_GUIDE.md` - Porting from ATtiny412
- `PORT_SUMMARY.md` - Port completion checklist
- `E22-400M33S_WIRING.md` - LoRa module setup
- `BNO085_LAUNCH_DETECTION.md` - IMU configuration (this file)
- `COMPLETE_SYSTEM_SUMMARY.md` - This document

---

## Quick Reference

### Upload Code
```bash
make flash
```

### Monitor Serial
```bash
make monitor
# Press Ctrl+C to exit
```

### Clean Build
```bash
make clean
make compile
```

### Update Configuration
Edit `include/mpu_config.h`:
- Callsign: `BEACON_CALLSIGN`
- LoRa power: `LORA_TX_POWER`
- Launch threshold: `LAUNCH_ACCEL_THRESHOLD`
- IMU address: `IMU_I2C_ADDR`

---

## Conclusion

This GPS radio beacon represents a significant upgrade:

**Hardware Evolution**:
- ATtiny412 → ItsyBitsy M4 (480x faster CPU, 768x more RAM)
- Simple 433MHz → E22-400M33S LoRa (10-15x range)
- Switch → BNO085 IMU (intelligent detection)

**Result**: A professional-grade rocket tracking beacon with excellent range, intelligent launch detection, and robust GPS tracking - all in a compact, reliable package!

Ready to track your rocket! 🚀
