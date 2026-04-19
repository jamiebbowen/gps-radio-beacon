# GPS Radio Beacon Receiver - Complete Wiring Diagram

## STM32F401 Black Pill Pin Assignments

### Power
| Pin | Connection | Notes |
|-----|------------|-------|
| 3.3V | Power rail | Connect to all 3.3V devices |
| GND | Ground rail | Common ground for all components |
| 5V | USB power input | Optional - can power from USB or external |

---

## Component Connections

### 1. E22-400M33S LoRa Module (NEW!)

**SPI2 Interface + Control Pins**

STM32F4 → E22-400M33S
PB13 → SCK
PB14 → MISO
PB15 → MOSI
PB12 → CS
PA8  → RESET
PB0  → BUSY
PB1  → DIO1
3.3V → VCC
GND  → GND
     → ANT (433MHz antenna - REQUIRED!)

| STM32F4 Pin | Function | E22-400M33S Pin | Description |
|-------------|----------|-----------------|-------------|
| **PB13** | SPI2_SCK | SCK | SPI Clock |
| **PB14** | SPI2_MISO | MISO | SPI Master In Slave Out |
| **PB15** | SPI2_MOSI | MOSI | SPI Master Out Slave In |
| **PB12** | GPIO_OUT | NSS/CS | Chip Select (active low) |
| **PA8** | GPIO_OUT | NRST | Reset (active low) |
| **PB0** | GPIO_IN | BUSY | Busy status indicator |
| **PB1** | GPIO_INT | DIO1 | Interrupt (RX/TX done) |
| **3.3V** | Power | VCC | Power supply (3.3V only!) |
| **GND** | Ground | GND | Ground |
| - | - | **ANT** | **433MHz antenna (REQUIRED)** |

⚠️ **IMPORTANT:** 
- E22-400M33S operates at **3.3V only** - do NOT use 5V!
- **Antenna is mandatory** - transmitting without antenna will damage the module
- Use a 433MHz antenna (wire length ~17.3cm for 1/4 wave)

---

### 2. GPS Module (NEO-6M or similar)

**USART2 Interface**

| STM32F4 Pin | Function | GPS Pin | Description |
|-------------|----------|---------|-------------|
| **PA2** | USART2_TX | RX | GPS receives data (not used) |
| **PA3** | USART2_RX | TX | GPS transmits NMEA sentences |
| **3.3V** | Power | VCC | Power supply |
| **GND** | Ground | GND | Ground |

**Settings:**
- Baud rate: 9600 (configured in firmware)
- 8N1 (8 data bits, no parity, 1 stop bit)
- NMEA sentence output

---

### 3. BNO055 Compass/IMU Module

**I2C1 Interface (Shared with Display)**

| STM32F4 Pin | Function | BNO055 Pin | Description |
|-------------|----------|------------|-------------|
| **PB6** | I2C1_SCL | SCL | I2C Clock |
| **PB7** | I2C1_SDA | SDA | I2C Data |
| **3.3V** | Power | VIN/VCC | Power supply |
| **GND** | Ground | GND | Ground |

**I2C Address:** 0x28 (default, ADR pin low)  
**Alternative Address:** 0x29 (if ADR pin high)

---

### 4. SSD1309 OLED Display (128x64)

**I2C1 Interface (Shared with Compass)**

| STM32F4 Pin | Function | Display Pin | Description |
|-------------|----------|-------------|-------------|
| **PB6** | I2C1_SCL | SCL | I2C Clock |
| **PB7** | I2C1_SDA | SDA | I2C Data |
| **3.3V** | Power | VCC | Power supply |
| **GND** | Ground | GND | Ground |

**I2C Address:** 0x3C  
**Resolution:** 128x64 pixels, monochrome

---

### 5. MicroSD Card Module

**SPI1 Interface**

| STM32F4 Pin | Function | SD Card Pin | Description |
|-------------|----------|-------------|-------------|
| **PA5** | SPI1_SCK | SCK/CLK | SPI Clock |
| **PA6** | SPI1_MISO | MISO/DO | SPI Data Out |
| **PA7** | SPI1_MOSI | MOSI/DI | SPI Data In |
| **PA4** | GPIO_OUT | CS | Chip Select |
| **PB9** | GPIO_IN | CD/DETECT | Card detect (optional) |
| **3.3V** | Power | VCC | Power supply |
| **GND** | Ground | GND | Ground |

**Note:** Most SD card modules have onboard level shifters and regulators

---

### 6. Mode Button

**GPIO Input with Pull-up**

| STM32F4 Pin | Function | Button Connection |
|-------------|----------|-------------------|
| **PB10** | GPIO_IN_PULLUP | Button side 1 |
| **GND** | Ground | Button side 2 |

**Function:** Press to cycle through display modes

---

### 7. Status LED

**GPIO Output**

| STM32F4 Pin | Function | LED Connection |
|-------------|----------|----------------|
| **PC13** | GPIO_OUT | LED Anode (+) |
| **GND** | Ground | LED Cathode (-) via resistor |

**Note:** STM32F4 Black Pill has onboard LED on PC13

---

## Complete Pin Mapping Summary

### Port A (GPIOA)
| Pin | Function | Connected To |
|-----|----------|--------------|
| PA0 | Available | - |
| PA1 | Available | - |
| PA2 | USART2_TX | GPS RX |
| PA3 | USART2_RX | GPS TX |
| PA4 | SPI1_CS | SD Card CS |
| PA5 | SPI1_SCK | SD Card SCK |
| PA6 | SPI1_MISO | SD Card MISO |
| PA7 | SPI1_MOSI | SD Card MOSI |
| PA8 | GPIO_OUT | LoRa RESET |
| PA9 | Available | - |
| PA10 | Available | - |
| PA11 | USB D- | (USB if used) |
| PA12 | USB D+ | (USB if used) |
| PA13 | SWDIO | Debug/Programming |
| PA14 | SWCLK | Debug/Programming |
| PA15 | Available | - |

### Port B (GPIOB)
| Pin | Function | Connected To |
|-----|----------|--------------|
| PB0 | GPIO_IN | LoRa BUSY |
| PB1 | GPIO_INT | LoRa DIO1 |
| PB2 | Available | - |
| PB3 | Available | - |
| PB4 | Available | - |
| PB5 | Available | - |
| PB6 | I2C1_SCL | Display & Compass SCL |
| PB7 | I2C1_SDA | Display & Compass SDA |
| PB8 | Available | - |
| PB9 | GPIO_IN | SD Card Detect |
| PB10 | GPIO_IN | Mode Button |
| PB11 | Available | - |
| PB12 | GPIO_OUT | LoRa CS |
| PB13 | SPI2_SCK | LoRa SCK |
| PB14 | SPI2_MISO | LoRa MISO |
| PB15 | SPI2_MOSI | LoRa MOSI |

### Port C (GPIOC)
| Pin | Function | Connected To |
|-----|----------|--------------|
| PC13 | GPIO_OUT | Status LED |
| PC14 | Available | - |
| PC15 | Available | - |

---

## Pin Conflicts - RESOLVED ✅

All pin conflicts have been resolved in the latest firmware version:

### ✅ Fixed Issues:

1. **Button Pin Conflict** - RESOLVED
   - Button moved from PB12 → **PB10**
   - LoRa CS remains on **PB12**
   - No conflict!

2. **SD Card / LoRa Conflict** - RESOLVED
   - LoRa BUSY moved from PA7 → **PB0**
   - LoRa DIO1 moved from PA6 → **PB1**
   - SD Card MISO stays on **PA6**
   - SD Card MOSI stays on **PA7**
   - No conflict!

3. **SD Card Detect / LoRa Reset Conflict** - RESOLVED
   - SD Card Detect moved from PA8 → **PB9**
   - LoRa RESET stays on **PA8**
   - No conflict!

---

## Recommended Breadboard Layout

```
┌────────────────────────────────────────────┐
│  Power Rails                                │
│  ● 3.3V ════════════════════════════════   │
│  ● GND  ════════════════════════════════   │
└────────────────────────────────────────────┘

┌─────────────────┐  ┌──────────────┐
│   STM32F401     │  │ GPS Module   │
│   Black Pill    │  │   (NEO-6M)   │
│                 │  │              │
│  PA2 ─────────────► RX           │
│  PA3 ◄───────────── TX           │
│  3.3V ────────────► VCC          │
│  GND  ────────────► GND          │
└─────────────────┘  └──────────────┘

┌─────────────────┐  ┌──────────────┐
│   STM32F401     │  │ E22-400M33S  │
│   Black Pill    │  │ LoRa Module  │
│                 │  │              │
│  PB13 ────────────► SCK          │
│  PB14 ◄──────────── MISO         │
│  PB15 ────────────► MOSI         │
│  PB12 ────────────► CS/NSS       │
│  PA8  ────────────► NRST         │
│  PB0  ◄──────────── BUSY         │
│  PB1  ◄──────────── DIO1         │
│  3.3V ────────────► VCC          │
│  GND  ────────────► GND          │
│                 │  │ ANT ─── 🏁   │ (Antenna!)
└─────────────────┘  └──────────────┘

┌─────────────────┐  ┌──────────────┐  ┌──────────────┐
│   STM32F401     │  │   BNO055     │  │  SSD1309     │
│   Black Pill    │  │   Compass    │  │  Display     │
│                 │  │              │  │              │
│  PB6  ────────────►─┬─ SCL        │  │              │
│  PB7  ────────────►─┼─ SDA        │  │              │
│  3.3V ────────────►─┤              │  │              │
│  GND  ────────────►─┤              │  │              │
│                 │  └──────────────┘  │              │
│                 │                     │ SCL ◄────────┤
│                 │                     │ SDA ◄────────┤
│                 │                     │ VCC ◄────────┤
│                 │                     │ GND ◄────────┤
└─────────────────┘                     └──────────────┘

     [Button]
    ┌────┴────┐
    │         │
PB10 ●────────● GND
(Mode Button)
```

---

## Power Consumption Estimate

| Component | Current Draw | Notes |
|-----------|--------------|-------|
| STM32F401 | ~30-50mA | Active mode @ 84MHz |
| GPS Module | ~40-60mA | Acquiring/tracking |
| LoRa (RX) | ~12-15mA | Receive mode |
| LoRa (TX) | ~120mA | During transmission (transmitter only) |
| BNO055 | ~12mA | Normal operation |
| OLED Display | ~20-30mA | Depending on pixels lit |
| SD Card | ~50-100mA | During write operations |
| **Total (typical)** | **~165-190mA** | Without SD writes |
| **Total (max)** | **~265-315mA** | With SD active |

**Recommended Battery:** 
- 3.7V LiPo, 1000-2000mAh for 3-6 hours operation
- Use voltage regulator for stable 3.3V

---

## Testing Procedure

### 1. Power-On Test
- [ ] Connect 3.3V and GND first
- [ ] Verify voltage on power rails with multimeter
- [ ] Check LED lights on PC13

### 2. Display Test
- [ ] OLED should display initialization
- [ ] Verify I2C communication (PB6/PB7)

### 3. GPS Test
- [ ] Place near window for GPS signal
- [ ] Watch for NMEA sentences on PA3
- [ ] Display should show GPS data within 30-60s

### 4. Compass Test
- [ ] BNO055 should calibrate automatically
- [ ] Rotate receiver and verify heading changes
- [ ] Check compass mode on display

### 5. LoRa Test
- [ ] Power on transmitter with LoRa
- [ ] Verify packet reception on receiver
- [ ] Check RSSI values (should be negative dBm)

### 6. SD Card Test
- [ ] Insert formatted SD card
- [ ] Verify data logging starts
- [ ] Check SD card mode shows storage info

### 7. Button Test
- [ ] Press button to cycle display modes
- [ ] Should see: GPS → RF → Compass Raw → Compass Diag → Heading → IMU → Navigation → SD Card

---

## Troubleshooting

### Display Not Working
- Check I2C connections (PB6/PB7)
- Verify 3.3V power to display
- Check I2C address (should be 0x3C)

### GPS No Data
- Ensure clear view of sky
- Check UART connections (PA2/PA3 might be swapped)
- Verify baud rate is 9600

### LoRa No Packets
- **Check antenna is connected!**
- Verify SPI connections (PB13/14/15)
- Check CS pin (PB12) not shorted to button
- Ensure transmitter is powered and configured

### Compass Not Responding
- Check I2C shared bus with display
- Verify BNO055 address (0x28 or 0x29)
- May need calibration (wave figure-8 motion)

### SD Card Not Detected
- Verify card is FAT32 formatted
- Check SPI connections (PA5/6/7)
- Try different SD card

---

## Safety Notes

⚡ **Electrical Safety**
- Never exceed 3.3V on any STM32 pin
- Double-check polarity before connecting power
- Use proper decoupling capacitors (0.1µF near each IC)

📡 **RF Safety**
- Always connect antenna before powering LoRa module
- 433MHz is legal for low-power use (check local regulations)
- Keep antenna away from body during transmission

🚀 **Rocketry Safety**
- Secure all connections with hot glue or tape
- Use proper strain relief on wires
- Test system thoroughly on ground before flight
- Consider backup power source for critical flights

---

## Firmware Flash Instructions

```bash
# Navigate to firmware directory
cd receiver/firmware

# Clean previous build
make clean

# Build firmware
make -j4

# Flash via DFU (STM32 in bootloader mode)
make flash

# Or flash via ST-Link
st-flash write bin/gps_radio_beacon_receiver.bin 0x8000000
```

---

## Component Sources

### Recommended Parts
- **STM32F401 Black Pill:** ~$4-6 on AliExpress/Amazon
- **E22-400M33S:** ~$8-12 on AliExpress (ensure 433MHz version)
- **NEO-6M GPS:** ~$6-10 with antenna
- **BNO055:** ~$15-25 (Adafruit breakout recommended)
- **SSD1309 OLED:** ~$5-8 (128x64, I2C)
- **MicroSD module:** ~$2-4
- **Antenna 433MHz:** ~$2-5 (or make 17.3cm wire dipole)

### Total Cost
**Approximately $50-80** for complete receiver build

---

**Document Version:** 1.0  
**Last Updated:** 2024-11-15  
**Compatible Firmware:** v2.0+ (with LoRa support)
