# GPS Radio Beacon Transmitter - Complete Pin Assignment List

## Adafruit ItsyBitsy M4 Express - All Pin Assignments

### Board: ItsyBitsy M4 Express
- **MCU:** SAMD51G19 (ARM Cortex-M4 @ 120MHz)
- **Flash:** 512KB
- **RAM:** 192KB
- **Voltage:** 3.3V logic (5V tolerant power input)

---

## Pin Assignments by Function

### 🛰️ GPS Module (NEO-6M or similar)

**UART Interface (Hardware Serial1)**

| ItsyBitsy Pin | Arduino Pin | Function | GPS Pin | Description |
|---------------|-------------|----------|---------|-------------|
| **RX (0)** | 0 | Serial1 RX | TX | GPS transmits NMEA sentences |
| **TX (1)** | 1 | Serial1 TX | RX | GPS receives commands (optional) |
| - | - | - | VCC | Power (3.3V) |
| - | - | - | GND | Ground |

**Configuration:**
- Baud rate: 9600
- Format: 8N1
- Sentences: GNRMC, GNGGA (configured by firmware)

---

### 📡 LoRa Module (E22-400M33S / SX1268)

**SPI Interface + Control Pins**

| ItsyBitsy Pin | Arduino Pin | Function | E22-400M33S Pin | Description |
|---------------|-------------|----------|-----------------|-------------|
| **MOSI** | MOSI | SPI MOSI | MOSI (17) | SPI Master Out |
| **MISO** | MISO | SPI MISO | MISO (16) | SPI Master In |
| **SCK** | SCK | SPI SCK | SCK (18) | SPI Clock |
| **5** | 5 | GPIO OUT | NSS (19) | Chip Select (active low) |
| **11** | 11 | GPIO OUT | NRST (15) | Reset (active low) |
| **6** | 6 | GPIO IN | BUSY (14) | Busy status indicator |
| **9** | 9 | GPIO INT | DIO1 (13) | Interrupt (TxDone/RxDone) |
| **12** | 12 | GPIO OUT | TXEN (7) | TX Enable (optional) |
| - | - | - | VCC (9, 10) | Power (3.3V or 5V) |
| - | - | - | GND | Ground |
| - | - | - | ANT (21) | **433MHz Antenna (REQUIRED)** |

**LoRa Configuration:**
- Frequency: **433.0 MHz** (ISM band)
- Bandwidth: **125 kHz**
- Spreading Factor: **9** (SF9)
- Coding Rate: **4/7**
- Sync Word: **0x12** (private network)
- TX Power: **22 dBm** (~158mW at 3.3V, ~2W at 5V)
- Preamble: **8 symbols**

**Notes:**
- Hardware SPI uses default ItsyBitsy M4 pins (MOSI, MISO, SCK)
- TXEN pin is optional and may not be used on all E22-400M33S variants
- Maximum range: ~15-20km line-of-sight at 22 dBm

---

### 🧭 IMU / Accelerometer (BNO085)

**I2C Interface**

| ItsyBitsy Pin | Arduino Pin | Function | BNO085 Pin | Description |
|---------------|-------------|----------|------------|-------------|
| **SDA** | SDA | I2C Data | SDA | I2C Data |
| **SCL** | SCL | I2C Clock | SCL | I2C Clock |
| **10** | 10 | GPIO OUT | RST | Reset (optional, can be -1) |
| - | - | - | VCC | Power (3.3V) |
| - | - | - | GND | Ground |

**I2C Configuration:**
- Address: **0x4A** (or 0x4B if SA0 pin is high)
- Clock speed: Standard I2C (100kHz or 400kHz)
- Purpose: Launch detection via acceleration sensing

**Launch Detection Settings:**
- Threshold: **20 m/s²** (~2g sustained acceleration)
- Duration: **100ms** minimum above threshold
- Settle time: **2000ms** after power-on before monitoring

---

### 💡 Status LED (Onboard)

| ItsyBitsy Pin | Arduino Pin | Function | Description |
|---------------|-------------|----------|-------------|
| **13** | 13 / LED_BUILTIN | GPIO OUT | Red onboard LED |

**Usage:**
- Standard Arduino LED_BUILTIN
- Can be used for status indication
- Active HIGH

---

## ItsyBitsy M4 Complete Pinout

### Digital I/O Pins

| Physical Pin | Arduino Pin | Current Use | Available? | Notes |
|--------------|-------------|-------------|------------|-------|
| 0 (RX) | 0 | GPS RX | ❌ | Hardware Serial1 RX |
| 1 (TX) | 1 | GPS TX | ❌ | Hardware Serial1 TX |
| 2 | 2 | **Available** | ✅ | Digital I/O, PWM |
| 3 | 3 | **Available** | ✅ | Digital I/O, PWM |
| 4 | 4 | **Available** | ✅ | Digital I/O, PWM |
| 5 | 5 | LoRa CS | ❌ | SPI Chip Select |
| 6 | 6 | LoRa BUSY | ❌ | LoRa status |
| 7 | 7 | **Available** | ✅ | Digital I/O, PWM |
| 9 | 9 | LoRa DIO1 | ❌ | LoRa interrupt |
| 10 | 10 | IMU Reset | ❌ | BNO085 reset (optional) |
| 11 | 11 | LoRa RESET | ❌ | LoRa reset |
| 12 | 12 | LoRa TXEN | ⚠️ | LoRa TX enable (optional) |
| 13 (LED) | 13 | Onboard LED | ⚠️ | Can be repurposed |
| SCK | SCK | LoRa SPI CLK | ❌ | Hardware SPI |
| MOSI | MOSI | LoRa SPI MOSI | ❌ | Hardware SPI |
| MISO | MISO | LoRa SPI MISO | ❌ | Hardware SPI |
| SCL | SCL | IMU I2C CLK | ❌ | Hardware I2C |
| SDA | SDA | IMU I2C DATA | ❌ | Hardware I2C |

### Analog Pins

| Physical Pin | Arduino Pin | Current Use | Available? | Notes |
|--------------|-------------|-------------|------------|-------|
| A0 | A0 | **Available** | ✅ | Analog input, 12-bit ADC |
| A1 | A1 | **Available** | ✅ | Analog input, 12-bit ADC |
| A2 | A2 | **Available** | ✅ | Analog input, 12-bit ADC |
| A3 | A3 | **Available** | ✅ | Analog input, 12-bit ADC |
| A4 | A4 | **Available** | ✅ | Analog input, 12-bit ADC |
| A5 | A5 | **Available** | ✅ | Analog input, 12-bit ADC |

**Note:** Analog pins can also be used as digital I/O

---

## Available Pins for Expansion (9 pins)

### Digital Pins Available:
- **Pin 2** - Digital I/O, PWM
- **Pin 3** - Digital I/O, PWM
- **Pin 4** - Digital I/O, PWM
- **Pin 7** - Digital I/O, PWM

### Analog Pins Available:
- **A0** - Analog/Digital, 12-bit ADC
- **A1** - Analog/Digital, 12-bit ADC
- **A2** - Analog/Digital, 12-bit ADC
- **A3** - Analog/Digital, 12-bit ADC
- **A4** - Analog/Digital, 12-bit ADC
- **A5** - Analog/Digital, 12-bit ADC

### Potentially Repurposable:
- **Pin 12** - TXEN (if not needed by your E22-400M33S variant)
- **Pin 13** - LED (if visual status not needed)

---

## Interface Summary

| Interface | Pins Used | Connected To | Notes |
|-----------|-----------|--------------|-------|
| **Hardware SPI** | SCK, MOSI, MISO, Pin 5 (CS) | E22-400M33S LoRa | High-speed SPI |
| **Hardware I2C** | SDA, SCL | BNO085 IMU | Shared bus |
| **Hardware Serial1** | Pin 0 (RX), Pin 1 (TX) | GPS Module | 9600 baud UART |
| **GPIO** | Pins 6, 9, 10, 11, 12 | LoRa control, IMU reset | Digital I/O |

---

## Power Configuration

### Power Input Options:

1. **USB Power (5V)**
   - Connect via USB-C port
   - Onboard regulator provides 3.3V to board
   - Can power all peripherals

2. **Battery Power (3.7-6V)**
   - Connect to **BAT** pin (LiPo friendly)
   - Onboard charger for single-cell LiPo
   - Regulator provides 3.3V

3. **Regulated Power (3.3V)**
   - Connect directly to **3.3V** pin
   - Bypass onboard regulator
   - Use if you have external 3.3V supply

### Power Consumption Estimate:

| Component | Current Draw | Notes |
|-----------|--------------|-------|
| SAMD51 MCU | ~30-50mA | Active @ 120MHz |
| GPS Module | ~40-60mA | Acquiring/tracking |
| LoRa (Standby) | ~2µA | Sleep mode |
| LoRa (RX) | ~14mA | Receive mode (not used often) |
| LoRa (TX @ 22dBm) | ~120mA @ 3.3V | During transmission |
| LoRa (TX @ 33dBm) | ~1200mA @ 5V | Maximum power (if 5V supply) |
| BNO085 IMU | ~10-15mA | Active sampling |
| **Total (typical)** | **~100-135mA** | Between transmissions |
| **Total (TX burst)** | **~220-285mA** | During transmission @ 3.3V |

### Battery Life Estimates:

**Pre-launch (30s intervals):**
- Average: ~110mA
- 1000mAh LiPo: ~9 hours
- 2000mAh LiPo: ~18 hours

**Launch/Recovery (continuous TX):**
- Average: ~250mA
- 1000mAh LiPo: ~4 hours
- 2000mAh LiPo: ~8 hours

**Battery Save (60s intervals):**
- Average: ~105mA
- 1000mAh LiPo: ~9.5 hours
- 2000mAh LiPo: ~19 hours

---

## Firmware Configuration

### Callsign
Set in `mpu_config.h`:
```cpp
#define BEACON_CALLSIGN "KE0MZS"  // Update with your callsign
```

### LoRa Parameters
Match these with receiver:
```cpp
#define LORA_FREQUENCY      433.0       // MHz
#define LORA_BANDWIDTH      125.0       // kHz
#define LORA_SPREADING      9           // SF9
#define LORA_CODING_RATE    7           // 4/7
#define LORA_SYNC_WORD      0x12        // Private
#define LORA_TX_POWER       22          // dBm (158mW)
```

### Launch Detection
```cpp
#define LAUNCH_ACCEL_THRESHOLD  20.0    // m/s² (~2g)
#define LAUNCH_ACCEL_DURATION   100     // ms
#define LAUNCH_SETTLE_TIME      2000    // ms
```

---

## Testing vs Production Mode

Set in `config.h`:
```cpp
#define TESTING_MODE 0  // 0 = Production, 1 = Testing
```

### Testing Mode (TESTING_MODE = 1):
- Pre-launch interval: **10 seconds**
- Launch duration: **30 seconds**
- Post-launch duration: **30 seconds**
- Battery save interval: **30 seconds**
- Debug output: **Enabled**

### Production Mode (TESTING_MODE = 0):
- Pre-launch interval: **30 seconds**
- Launch duration: **5 minutes**
- Post-launch duration: **10 minutes**
- Battery save interval: **60 seconds**
- Debug output: **Disabled**

---

## Beacon State Machine

```
┌──────────────┐
│  PRE_LAUNCH  │ ← Initial state
│  (30s TX)    │
└──────┬───────┘
       │ Launch detected (2g for 100ms)
       ↓
┌──────────────┐
│    LAUNCH    │
│ (Continuous) │ ← 5 minutes
└──────┬───────┘
       │ After 5 minutes
       ↓
┌──────────────┐
│ POST_LAUNCH  │
│ (Continuous) │ ← 10 minutes
└──────┬───────┘
       │ After 10 minutes
       ↓
┌──────────────┐
│ BATTERY_SAVE │
│  (60s TX)    │ ← Until battery dies
└──────────────┘
```

**Note:** Launch detection can trigger from ANY state!

---

## Packet Format

Transmitted packets use compact NMEA format:

### GPS Data Packet:
```
$$lat,lon,alt,sats,fix,launch_status,callsign$$CHKSUM
```

**Example:**
```
$$3953.40363,-10506.93605,1671.7,12,1,45,KE0MZS$$2F
```

Fields:
- `lat`: Latitude in NMEA format (DDMM.MMMMM)
- `lon`: Longitude in NMEA format (DDDMM.MMMMM)
- `alt`: Altitude in meters
- `sats`: Number of satellites
- `fix`: GPS fix quality (0=invalid, 1=GPS, 2=DGPS)
- `launch_status`: 
  - `N` = Not launched
  - `###` = Seconds since launch (e.g., `45` = 45 seconds ago)
- `callsign`: Ham radio callsign
- `CHKSUM`: XOR checksum (2 hex digits)

### Callsign Packet:
```
$$CALLSIGN$$
```

**Example:**
```
$$KE0MZS$$
```

Transmitted every 5 minutes for FCC compliance.

---

## Hardware Connections Diagram

```
┌────────────────────────────────┐
│   Adafruit ItsyBitsy M4        │
│   SAMD51 @ 120MHz              │
│                                │
│  RX(0) ◄──── TX        GPS     │
│  TX(1) ────► RX      NEO-6M    │
│                                │
│  Pin 5 ────► NSS              │
│  Pin 11 ───► NRST             │
│  Pin 6 ◄──── BUSY   E22-400M33S│
│  Pin 9 ◄──── DIO1    LoRa      │
│  Pin 12 ───► TXEN   (SX1268)   │
│  SCK ──────► SCK               │
│  MOSI ─────► MOSI              │
│  MISO ◄──── MISO               │
│                      ANT ──🏁  │
│  SDA ◄────► SDA               │
│  SCL ─────► SCL     BNO085     │
│  Pin 10 ───► RST      IMU      │
│                                │
│  BAT ◄───── LiPo 3.7V          │
└────────────────────────────────┘
```

---

## Critical Notes

### ⚠️ Before Flight:

1. **Update Callsign**
   - Edit `BEACON_CALLSIGN` in `mpu_config.h`
   - Must be valid amateur radio callsign

2. **Antenna Connection**
   - **NEVER power on without antenna!**
   - Will damage LoRa module
   - Use proper 433MHz antenna

3. **Testing Mode**
   - Set `TESTING_MODE 0` for flight
   - Test on ground with `TESTING_MODE 1`

4. **GPS Lock**
   - Wait for GPS fix before flight
   - Needs clear sky view
   - Takes 30-60 seconds cold start

5. **Battery**
   - Use 1S LiPo (3.7V nominal)
   - Minimum 1000mAh recommended
   - 2000mAh for extended recovery

6. **IMU Calibration**
   - BNO085 auto-calibrates
   - Let sit still for 2 seconds after power-on
   - Avoid vibration during calibration

---

## Pin Usage Summary

- **Used Pins:** 13 (including SPI/I2C/UART hardware pins)
- **Available Pins:** 9 (4 digital + 6 analog, minus conflicts)
- **Potentially Repurposable:** 2 (TXEN, LED)

**Pin Utilization:** ~50% of available GPIO used

---

## Compatibility with Receiver

**Both transmitter and receiver must use identical LoRa settings:**

| Parameter | Value | Must Match |
|-----------|-------|------------|
| Frequency | 433.0 MHz | ✅ YES |
| Bandwidth | 125 kHz | ✅ YES |
| Spreading Factor | 9 | ✅ YES |
| Coding Rate | 4/7 | ✅ YES |
| Sync Word | 0x12 | ✅ YES |
| Packet Format | $$data$$CHKSUM | ✅ YES |

**TX Power does NOT need to match** (transmitter can use higher power)

---

## Document Information

**Version:** 1.0  
**Last Updated:** 2024-11-18  
**Compatible Firmware:** v2.0+ (LoRa E22-400M33S)  
**Board:** Adafruit ItsyBitsy M4 Express  
**Platform:** Arduino + PlatformIO
