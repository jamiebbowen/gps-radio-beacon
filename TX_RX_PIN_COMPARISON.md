# Transmitter vs Receiver - Pin Comparison

## Quick Reference: Side-by-Side Comparison

---

## LoRa Module Connections

### E22-400M33S (SX1268) - Both Systems

| Function | **Transmitter** (ItsyBitsy M4) | **Receiver** (STM32F4 Black Pill) |
|----------|-------------------------------|-----------------------------------|
| **SPI SCK** | SCK (hardware SPI) | PB13 (SPI2_SCK) |
| **SPI MISO** | MISO (hardware SPI) | PB14 (SPI2_MISO) |
| **SPI MOSI** | MOSI (hardware SPI) | PB15 (SPI2_MOSI) |
| **CS/NSS** | Pin 5 | PB12 |
| **RESET** | Pin 11 | PA8 |
| **BUSY** | Pin 6 | PB0 |
| **DIO1** | Pin 9 | PB1 |
| **TXEN** | Pin 12 (optional) | Not used |
| **Power** | 3.3V or 5V | 3.3V |
| **Antenna** | 433MHz (IPEX/stamp) | 433MHz (wire/IPEX) |

---

## GPS Module Connections

### NEO-6M or Similar

| Function | **Transmitter** (ItsyBitsy M4) | **Receiver** (STM32F4 Black Pill) |
|----------|-------------------------------|-----------------------------------|
| **GPS TX** | Pin 0 (Serial1 RX) | PA3 (USART2_RX) |
| **GPS RX** | Pin 1 (Serial1 TX) | PA2 (USART2_TX) |
| **Baud Rate** | 9600 | 9600 |
| **Power** | 3.3V | 3.3V |

---

## IMU / Compass Connections

| Function | **Transmitter** (ItsyBitsy M4) | **Receiver** (STM32F4 Black Pill) |
|----------|-------------------------------|-----------------------------------|
| **Sensor** | BNO085 (IMU) | BNO055 (Compass/IMU) |
| **Purpose** | Launch detection | Heading/orientation |
| **SDA** | SDA (hardware I2C) | PB7 (I2C1_SDA) |
| **SCL** | SCL (hardware I2C) | PB6 (I2C1_SCL) |
| **RESET** | Pin 10 | Not used |
| **I2C Address** | 0x4A (BNO085) | 0x28 (BNO055) |
| **Power** | 3.3V | 3.3V |

---

## Additional Peripherals

### Transmitter Only:
- **None** - Transmitter is minimal (GPS + LoRa + IMU)

### Receiver Only:

| Peripheral | Pins | Description |
|------------|------|-------------|
| **SSD1309 Display** | PB6 (SCL), PB7 (SDA) | 128x64 OLED, I2C @ 0x3C |
| **SD Card** | PA4 (CS), PA5 (SCK), PA6 (MISO), PA7 (MOSI), PB9 (Detect) | SPI1, data logging |
| **Mode Button** | PB10 | Cycle display modes |
| **Status LED** | PC13 | Onboard LED |

---

## LoRa Configuration - MUST MATCH!

| Parameter | Transmitter | Receiver | Match? |
|-----------|-------------|----------|--------|
| **Frequency** | 433.0 MHz | 433.0 MHz | ✅ |
| **Bandwidth** | 125 kHz | 125 kHz | ✅ |
| **Spreading Factor** | 9 (SF9) | 9 (SF9) | ✅ |
| **Coding Rate** | 4/7 | 4/7 | ✅ |
| **Sync Word** | 0x12 | 0x12 | ✅ |
| **TX Power** | 22 dBm | 22 dBm | ℹ️ Doesn't need to match |
| **Preamble** | 8 | 8 | ✅ |

---

## System Comparison

### Hardware Platforms

| Feature | **Transmitter** | **Receiver** |
|---------|----------------|--------------|
| **Board** | Adafruit ItsyBitsy M4 Express | STM32F4 Black Pill |
| **MCU** | SAMD51G19 (ARM Cortex-M4) | STM32F401CCU6 (ARM Cortex-M4) |
| **Clock Speed** | 120 MHz | 84 MHz |
| **Flash** | 512 KB | 256 KB |
| **RAM** | 192 KB | 64 KB |
| **Logic Level** | 3.3V | 3.3V |
| **Power Input** | 3.3V - 6V (USB/BAT) | 3.3V or 5V (USB) |

### Power Consumption

| State | **Transmitter** | **Receiver** |
|-------|----------------|--------------|
| **Idle** | ~100-135mA | ~165-190mA |
| **TX Active** | ~220-285mA @ 3.3V | ~120mA @ 3.3V (rarely transmits) |
| **TX Max Power** | ~1200mA @ 5V (33 dBm) | N/A (receiver doesn't TX often) |

### Battery Life (1000mAh LiPo)

| Mode | **Transmitter** | **Receiver** |
|------|----------------|--------------|
| **Standby** | ~9 hours (30s intervals) | ~5 hours (continuous) |
| **Active** | ~4 hours (continuous TX) | ~5 hours (continuous RX) |

---

## Firmware Differences

### Transmitter Features:
- ✅ GPS data acquisition
- ✅ LoRa transmission (continuous or periodic)
- ✅ Launch detection (BNO085 accelerometer)
- ✅ State machine (Pre-launch → Launch → Post-launch → Battery save)
- ✅ Callsign transmission (FCC compliance)
- ❌ No display
- ❌ No user input
- ❌ No data logging

### Receiver Features:
- ✅ GPS data acquisition (local position)
- ✅ LoRa reception (remote position from transmitter)
- ✅ Display modes (GPS, RF, Compass, Navigation, etc.)
- ✅ Compass/heading (BNO055)
- ✅ SD card data logging
- ✅ User input (mode button)
- ✅ Distance/bearing calculations
- ❌ No launch detection
- ❌ Minimal transmission (mostly RX)

---

## Available Pins

### Transmitter (ItsyBitsy M4):
- **Digital:** Pins 2, 3, 4, 7 (4 pins)
- **Analog:** A0-A5 (6 pins, can be digital too)
- **Total Available:** 10 pins

### Receiver (STM32F4):
- **Port A:** PA0, PA1, PA9, PA10, PA15 (5 pins)
- **Port B:** PB2, PB3, PB4, PB5, PB8, PB11 (6 pins)
- **Port C:** PC14, PC15 (2 pins)
- **Total Available:** 13 pins

---

## Packet Format - MUST MATCH!

Both use identical packet format:

```
$$lat,lon,alt,sats,fix,launch_status,callsign$$CHKSUM
```

**Example:**
```
$$3953.40363,-10506.93605,1671.7,12,1,45,KE0MZS$$2F
```

### Fields:
1. `lat` - Latitude (NMEA DDMM.MMMMM)
2. `lon` - Longitude (NMEA DDDMM.MMMMM)
3. `alt` - Altitude (meters)
4. `sats` - Satellites in view
5. `fix` - GPS fix quality (0/1/2)
6. `launch_status` - `N` or seconds since launch
7. `callsign` - Ham radio callsign
8. `CHKSUM` - XOR checksum (2 hex digits)

---

## Development Tools

### Transmitter:
- **IDE:** PlatformIO / Arduino IDE
- **Framework:** Arduino (SAMD51)
- **Library:** RadioLib (SX1268 support)
- **Upload:** USB bootloader (double-tap reset)
- **Debug:** Serial monitor via USB

### Receiver:
- **IDE:** Any (VS Code, etc.)
- **Framework:** STM32 HAL
- **Build:** Makefile + arm-none-eabi-gcc
- **Upload:** ST-Link or DFU mode
- **Debug:** SWD (SWDIO/SWCLK pins)

---

## Wiring Complexity

### Transmitter: ⭐⭐ (Simple)
```
3 modules, 13 wires total:
- GPS: 2 wires (RX/TX)
- LoRa: 7 wires (SPI + control)
- IMU: 3 wires (I2C + reset)
- Power: 1 wire (shared 3.3V/GND)
```

### Receiver: ⭐⭐⭐⭐ (Complex)
```
6 modules, 24 wires total:
- GPS: 2 wires (RX/TX)
- LoRa: 7 wires (SPI + control)
- Display: 2 wires (I2C)
- Compass: 2 wires (I2C, shared bus)
- SD Card: 5 wires (SPI + detect)
- Button: 1 wire (+ GND)
- Power: 1 wire (shared 3.3V/GND)
```

---

## Visual Pin Diagrams

### Transmitter (ItsyBitsy M4)
```
         ┌─────────────────┐
    BAT ─┤                 ├─ GND
    GND ─┤                 ├─ USB
    USB ─┤  ItsyBitsy M4   ├─ 13 (LED)
    3.3V─┤                 ├─ 12 (LORA_TXEN)
    ARef─┤                 ├─ 11 (LORA_RST)
     A0 ─┤                 ├─ 10 (IMU_RST)
     A1 ─┤                 ├─ 9  (LORA_DIO1)
     A2 ─┤                 ├─ 7
     A3 ─┤                 ├─ SCL (IMU)
     A4 ─┤                 ├─ SDA (IMU)
     A5 ─┤                 ├─ 6  (LORA_BUSY)
    SCK ─┤   (LoRa SPI)    ├─ 5  (LORA_CS)
   MOSI ─┤   (LoRa SPI)    ├─ 4
   MISO ─┤   (LoRa SPI)    ├─ 3
 TX (1) ─┤  (GPS TX)       ├─ 2
 RX (0) ─┤  (GPS RX)       ├─ RST
         └─────────────────┘
```

### Receiver (STM32F4 Black Pill)
```
         ┌─────────────────┐
   PB12 ─┤ (LORA_CS)       ├─ GND
   PB13 ─┤ (LORA_SCK)      ├─ GND
   PB14 ─┤ (LORA_MISO)     ├─ 3.3V
   PB15 ─┤ (LORA_MOSI)     ├─ RST
    PA8 ─┤ (LORA_RST)      ├─ PB11
    PA9 ─┤                 ├─ PB10 (BUTTON)
   PA10 ─┤                 ├─ PB1  (LORA_DIO1)
   PA15 ─┤                 ├─ PB0  (LORA_BUSY)
    PB3 ─┤  STM32F401      ├─ PA7  (SD_MOSI)
    PB4 ─┤  Black Pill     ├─ PA6  (SD_MISO)
    PB5 ─┤                 ├─ PA5  (SD_SCK)
    PB6 ─┤ (I2C_SCL)       ├─ PA4  (SD_CS)
    PB7 ─┤ (I2C_SDA)       ├─ PA3  (GPS_RX)
    PB8 ─┤                 ├─ PA2  (GPS_TX)
    PB9 ─┤ (SD_DETECT)     ├─ PA1
   5V   ─┤                 ├─ PA0
   GND  ─┤                 ├─ PC15
   3.3V ─┤                 ├─ PC14
         └─────────────────┘
               PC13 (LED)
```

---

## Compatibility Checklist

Before deploying, verify:

### LoRa Settings Match:
- [ ] Frequency: 433.0 MHz
- [ ] Bandwidth: 125 kHz
- [ ] Spreading Factor: 9
- [ ] Coding Rate: 4/7
- [ ] Sync Word: 0x12

### Packet Format Matches:
- [ ] Both use $$data$$CHKSUM format
- [ ] NMEA coordinate format (DDMM.MMMMM)
- [ ] XOR checksum calculation

### Antennas Connected:
- [ ] Transmitter has 433MHz antenna
- [ ] Receiver has 433MHz antenna
- [ ] Both are properly tuned/matched

### Power Supply:
- [ ] Transmitter battery charged (3.7V LiPo)
- [ ] Receiver powered (USB or battery)
- [ ] All modules getting correct voltage (3.3V)

### Testing:
- [ ] Transmitter sends packets (check Serial output)
- [ ] Receiver displays RF data (check RF mode)
- [ ] RSSI/SNR values reasonable (-40 to -120 dBm)
- [ ] GPS lock on both (wait 30-60s)

---

## Troubleshooting: TX → RX Communication

### No Packets Received:

1. **Check LoRa settings match** (most common!)
   - Frequency off by even 0.1 MHz = no reception
   - Wrong spreading factor = incompatible

2. **Check antennas**
   - Transmitter needs antenna to transmit
   - Poor antenna = very short range

3. **Check transmitter is transmitting**
   - USB Serial monitor should show "Transmitted: ..."
   - LED may blink during TX

4. **Check receiver is in RX mode**
   - Display should show RF mode
   - RSSI should update (even if no packets)

5. **Distance too far**
   - Start with TX and RX side-by-side
   - Should see RSSI around -40 to -60 dBm when close

### Garbled/Corrupted Packets:

1. **Interference**
   - Move away from WiFi routers
   - Avoid metal buildings
   - Check for other 433MHz devices

2. **Power supply issues**
   - LoRa needs clean power during TX
   - Add capacitors if voltage sags

3. **Antenna mismatch**
   - Use proper 433MHz antennas
   - Check SWR if using commercial antenna

---

**Both systems ready to deploy!** 🚀

See individual pinout documents for complete details:
- `TRANSMITTER_PINOUTS.md`
- `RECEIVER_WIRING.md`
