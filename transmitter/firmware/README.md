# GPS Radio Beacon - ItsyBitsy M4 Express Port

Successfully ported from ATtiny412 to **Adafruit ItsyBitsy M4 Express** (SAMD51 ARM Cortex-M4 @ 120MHz)

## Hardware Configuration

### Pin Assignments
- **Pin 11**: Radio TX (bit-banged UART at 300 baud)
- **Pin 12**: Radio Enable/Disable
- **Pin 10**: Launch Detection Switch (active low with pull-up)
- **Pins 0/1**: GPS UART (Hardware Serial1, 115200 baud)

### Connections
```
ItsyBitsy M4          Component
-----------           ---------
Pin 11         --->   Radio TX Data In
Pin 12         --->   Radio Enable/PTT
Pin 10         --->   Launch Switch (to GND when launched)
Pin 0 (RX1)    <---   GPS TX
Pin 1 (TX1)    --->   GPS RX
GND            --->   Common Ground
USB/5V         --->   Power Input
```

## Software Architecture

### Arduino Framework
The firmware now uses the Arduino framework instead of bare-metal AVR:
- **Hardware Serial1**: GPS communication (no more bit-banging)
- **Arduino timing**: `millis()`, `delay()`, `delayMicroseconds()`
- **GPIO**: `pinMode()`, `digitalWrite()`, `digitalRead()`
- **No watchdog**: SAMD51 doesn't enable watchdog by default

### Build System
Uses `arduino-cli` for command-line compilation and upload:
- No Arduino IDE required
- Makefile-based workflow
- Cross-platform support

## Installation

### 1. Install Arduino CLI

```bash
# Linux/macOS
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
sudo mv bin/arduino-cli /usr/local/bin/

# Or download from: https://arduino.github.io/arduino-cli/
```

### 2. Install Board Support

```bash
make install-deps
```

This installs Adafruit SAMD board support package.

### 3. Connect ItsyBitsy M4

Plug in the ItsyBitsy M4 via USB. It should appear as `/dev/ttyACM0` (Linux) or similar.

## Building and Uploading

### Compile Only
```bash
make compile
```

### Upload to Board
```bash
make flash
# Or specify port:
make PORT=/dev/ttyACM1 flash
```

### Monitor Serial Output
```bash
make monitor
```

### Clean Build
```bash
make clean
```

## Configuration

### Callsign
Edit `include/mpu_config.h` (yes, kept the filename for compatibility):
```c
#define BEACON_CALLSIGN "KD0ABC"  // Change to your callsign
```

### Timing Modes
Edit `include/config.h`:
```c
#define TESTING_MODE 0  // 0 = production, 1 = testing
```

**Testing Mode** (fast intervals for bench testing):
- Pre-launch: 10s intervals
- Launch: continuous for 30s
- Post-launch: continuous for 30s
- Battery save: 30s intervals
- Callsign: every 30s

**Production Mode** (optimized for flight):
- Pre-launch: 30s intervals
- Launch: continuous for 5 minutes
- Post-launch: continuous for 10 minutes  
- Battery save: 60s intervals
- Callsign: every 5 minutes

## Packet Format

### GPS Data Packet
```
$$lat,lon,alt,sats$$CHKSUM
```
Example: `$$3953.40363,-10506.93605,1671.7,12$$2F`

- Coordinates in NMEA format (DDMM.MMMMM)
- Negative sign for South/West
- XOR checksum in hex
- 300 baud transmission

### Fast GPS Packet (during launch)
```
$$lat,lon,alt$$CHKSUM
```
Omits satellite count for faster transmission.

### Callsign Packet
```
KD0ABC
```
Transmitted every 5 minutes for FCC compliance.

## Advantages Over ATtiny412

### Performance
- **480x faster CPU**: 120MHz vs 20MHz
- **768x more RAM**: 192KB vs 256 bytes
- **98x more Flash**: 512KB vs 4KB
- **Hardware UART**: No bit-banging for GPS

### Features
- USB Serial for debugging (`Serial.begin(115200)`)
- Hardware floating-point unit
- DMA support (not used yet)
- Real-time clock (not used yet)
- No memory constraints - can add features easily

### Development
- Easier debugging via USB Serial
- Faster compile times
- Arduino ecosystem libraries available
- No fuse programming required

## Beacon State Machine

1. **PRE_LAUNCH**: Waiting on pad, 30s intervals
2. **LAUNCH**: Continuous transmission for 5 min (180s fast, then full)
3. **POST_LAUNCH**: Continuous transmission for next 10 min
4. **BATTERY_SAVE**: 60s intervals indefinitely

Launch detection triggers from ANY state.

## Troubleshooting

### Board Not Found
```bash
# List connected boards
arduino-cli board list

# Try different port
make PORT=/dev/ttyACM1 flash
```

### Compilation Errors
```bash
# Reinstall board support
arduino-cli core update-index
arduino-cli core install adafruit:samd
```

### GPS Not Working
- Check Serial1 connections (pins 0/1)
- GPS expects 115200 baud, 3.3V logic
- ItsyBitsy M4 is 3.3V compatible

### Radio Not Transmitting
- Check pin 11 for TX data (300 baud)
- Check pin 12 for enable signal
- Verify radio module power

## Development Notes

### File Structure
- `main.cpp`: Main program with state machine
- `gps.cpp`: GPS parsing (NMEA)
- `radio.cpp`: Bit-banged 300 baud UART
- `uart.cpp`: Hardware Serial wrapper
- `beacon.cpp`: Packet formatting
- `launch_detect.cpp`: Launch switch debouncing
- `include/`: Headers
- `gps_radio_beacon.ino`: Arduino sketch wrapper

### Adding Features
With 192KB RAM and 512KB Flash, you can easily add:
- SD card logging
- Additional sensors (accelerometer, barometer)
- Data buffering
- More complex telemetry
- GPS configuration via UBX protocol

### Debugging
Add to `setup()`:
```cpp
Serial.begin(115200);
Serial.println("GPS Beacon Started");
```

Then use `make monitor` or any serial terminal.

## License
GPL v3 - Amateur Radio Use

## Author
Ported to ItsyBitsy M4 Express by AI Assistant
Original ATtiny412 version by Jamie Bowen
