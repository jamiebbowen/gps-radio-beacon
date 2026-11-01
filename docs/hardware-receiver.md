# GPS Radio Beacon Receiver - Complete Wiring Diagram (v2.0)

## STM32F401 Black Pill Pin Assignments

### Power (v2.2: two-rail architecture)

The receiver runs on **two power rails**. This is about VCC wiring only:
**all logic signals are 3.3V** (the STM32 is a 3.3V MCU) - every module
powered at 5V does so because it has an onboard regulator/level shifting,
so its I/O stays at 3.3V levels regardless.

| Rail | Fed by | Powers |
|------|--------|--------|
| **5V** | USB, or battery → boost converter (MT3608-style from 1S LiPo) | E22 LoRa VCC, Black Pill "5V" pin, SHARP VIN, NEO-6M breakout VCC, BNO055 breakout VIN, SD module VCC |
| **3.3V** | Black Pill onboard LDO (from its 5V pin), or an external 3.3V buck from the 5V rail | eInk FeatherWing 3V, jogpad VCC, all logic-level domains |
| **GND** | Common ground for everything | |

**Why the split:**
- The E22's PA only reaches its rated 2W/+33dBm at 5V; at 3.3V the module
  under-drives it. The RX's command/ACK path gets real headroom.
- Feeding 5V-tolerant peripherals from the 5V rail unloads the Black Pill's
  small onboard LDO (which would otherwise supply ~180mA average + bursts).
- USB powering stays trivial: any 5V USB source *is* the 5V rail.

**Off the 5V rail (keep at 3.3V):**
- **eInk FeatherWing 4778** - wing PCB has no regulator; its 3V pin expects
  a regulated 3.3V (Feather-style rail).
- **Jogpad** - technically 3-5V rated, but its I2C pull-up destination at 5V
  is undocumented; it draws ~10mA so there is no benefit to 5V. Keep it on
  3.3V where bus levels are guaranteed.

### v2.0 Changes vs v1.x
- **Replaced** SSD1309 I2C OLED with **two SPI displays**: SHARP Memory LCD
  (Adafruit 3502) + 2.9" tri-color eInk FeatherWing (Adafruit 4778, SSD1680)
- **Replaced** both pushbuttons with a **NULLLAB Mini I2C Jogpad** (0x5A)
- **Added** external battery voltage divider on PA0 (ADC1_IN0)
- Everything else (LoRa, GPS, BNO055, SD card, LED) is unchanged

---

## Component Connections

### 1. E22-400M33S LoRa Module

**SPI2 Interface + Control Pins**

STM32F4 → E22-400M33S
PB13 → SCK
PB14 → MISO
PB15 → MOSI
PB12 → CS
PA8  → RESET
PB0  → BUSY
PB1  → DIO1
PA9  → RXEN (RF switch RX enable)
PA10 → TXEN (RF switch TX enable)
PA15 ← DIO2 (input-only spare, NEVER an output)
5V   → VCC (3.3-5.5V rated - use the 5V rail)
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
| **PA9** | GPIO_OUT | RXEN | RF switch RX enable (active high) |
| **PA10** | GPIO_OUT | TXEN | RF switch TX enable (active high) |
| **PA15** | GPIO_IN | DIO2 | Input-only spare - **never configure as output** |
| **5V** | Power | VCC | Power supply **3.3-5.5V** - use the 5V rail (v2.2) |
| **GND** | Ground | GND | Ground |
| - | - | **ANT** | **433MHz antenna (REQUIRED)** |

⚠️ **RF switch note (v2.1):** Both RXEN and TXEN are now driven manually by the
MCU. The SX1268 runs with `SetDio2AsRfSwitchCtrl = 0x00` (DIO2 available as a
plain IRQ line, **not** auto-driving the switch). EByte documents TXEN as
"external MCU IO *or* DIO2" - wiring TXEN to the MCU while also letting DIO2
drive it would double-drive the switch node; this split avoids that entirely.
Switch truth table: RX mode = RXEN 1/TXEN 0, TX mode = RXEN 0/TXEN 1 - the
firmware always drops the old side before raising the new one.

⚠️ **DIO2 contention warning:** PA15 must stay `GPIO_MODE_INPUT` forever. DIO2
drives out when the SX1268 maps a signal to it; an MCU output on PA15 would
fight the pin driver.

⚠️ **IMPORTANT:**
- VCC is **3.3-5.5V** (datasheet) - v2.2 powers the module from the **5V
  rail**, where the onboard PA reaches its full +33dBm. Its SPI/control
  logic is still 3.3V-level - feed signals from the STM32 as before. Never
  exceed 5.5V on VCC.
- **Antenna is mandatory** - transmitting without antenna will damage the module
- Use a 433MHz antenna (wire length ~17.3cm for 1/4 wave)

---

### 2. GPS Module (NEO-6M or similar)

**USART2 Interface**

| STM32F4 Pin | Function | GPS Pin | Description |
|-------------|----------|---------|-------------|
| **PA2** | USART2_TX | RX | GPS receives data (not used) |
| **PA3** | USART2_RX | TX | GPS transmits NMEA sentences |
| **5V** | Power | VCC | 5V rail if using a **regulated breakout** (GY-NEO6MV2 etc.; most are 3.3-5V); a bare NEO-6M module is 1.8-3.6V and must use 3.3V |
| **GND** | Ground | GND | Ground |

**Settings:**
- Baud rate: 9600 (configured in firmware)
- 8N1 (8 data bits, no parity, 1 stop bit)
- NMEA sentence output

---

### 3. BNO055 Compass/IMU Module

**I2C1 Interface (Shared with Jogpad)**

| STM32F4 Pin | Function | BNO055 Pin | Description |
|-------------|----------|------------|-------------|
| **PB6** | I2C1_SCL | SCL | I2C Clock |
| **PB7** | I2C1_SDA | SDA | I2C Data |
| **5V** | Power | VIN/VCC | 5V rail if using the **Adafruit/regulated breakout**; the bare BNO055 chip is 2.4-3.6V and must use 3.3V |
| **GND** | Ground | GND | Ground |

**I2C Address:** 0x28 (default, ADR pin low)  
**Alternative Address:** 0x29 (if ADR pin high)

---

### 4. SHARP Memory LCD - Adafruit 3502 (1.3" 168x144 Monochrome)

**SPI1 Interface (shared with SD card and eInk FeatherWing)**

| STM32F4 Pin | Function | 3502 Pin | Description |
|-------------|----------|----------|-------------|
| **PA5** | SPI1_SCK | CLK | SPI Clock |
| **PA7** | SPI1_MOSI | DI | Data in (display is write-only - **no MISO**) |
| **PB10** | GPIO_OUT | CS | Chip select (**active HIGH** - inverted vs most SPI) |
| **PB8** | GPIO_OUT | DISP | Display on/off (default: drive high) |
| **5V** | Power | VIN | **Use the 5V rail**: board has regulator + level shifters (VIN 3-5V) |
| **GND** | Ground | GND | Ground |

**Notes:**
- 168x144 monochrome, daylight-readable (e-reader style), **no backlight**
- Ultra-low power: ~4µA at 1Hz refresh
- Requires a 3,024-byte framebuffer in MCU RAM (fine on F401's 96KB)
- **VCOM must be toggled ~1Hz in software** (EXTMODE command) or the display
  will be damaged over time - the firmware driver handles this
- SPI idle slow: SHARP memory LCDs max out around 2MHz - firmware switches the
  SPI1 prescaler per device (already done between SD init speed and run speed)
- Primary "live" display: navigation page, jogpad menus, GPS status

---

### 5. 2.9" Tri-Color eInk FeatherWing - Adafruit 4778 (SSD1680, 296x128 R/B/W)

**SPI1 Interface (shared with SD card and SHARP display)**

The FeatherWing carries: the SSD1680 eInk display, an SPI SRAM chip (used as the
off-chip framebuffer - 296x128x2bpp won't fit in RAM), a microSD socket (**not
used here**), and 3 onboard buttons (**not used - jogpad replaces them**).

| STM32F4 Pin | Function | FeatherWing Pin | Description |
|-------------|----------|-----------------|-------------|
| **PA5** | SPI1_SCK | SCK | SPI Clock (Feather SPI pads) |
| **PA6** | SPI1_MISO | MISO | Used by SRAM chip (eInk itself is write-only) |
| **PA7** | SPI1_MOSI | MOSI | Data out |
| **PB11** | GPIO_OUT | ECS | eInk chip select |
| **PB3** | GPIO_OUT | D/C | eInk Data/Command |
| **PB5** | GPIO_OUT | SRCS | SRAM chip select |
| **PB4** | GPIO_OUT | RST | eInk reset |
| **PA1** | GPIO_IN | BUSY | eInk busy (on breakout pad - solder a wire) |
| - | - | SDCS | **Not connected** (existing PA4 SD module is used) |
| - | - | EN | **Not connected** (internal pull-up keeps regulator on) |
| - | - | A / B / C buttons | **Not connected** (jogpad covers all input) |
| **3.3V** | Power | 3V | **Always 3.3V** - the wing has no onboard regulator; never feed it the 5V rail |
| **GND** | Ground | GND | Ground |

**Notes:**
- Tri-color full refresh is SLOW (~15-25 s) and multi-stage - treat it as the
  "persistent summary" screen (flight stats, last known rocket position,
  splash page), not a live view
- Image persists with zero power; great for post-landing
- Cut-no-trace needed; just leave SDCS/EN/buttons unwired
- Wire buttery SMC... populate SRAM CS - the SSD1680 driver renders into SRAM,
  then shuffles SRAM → eInk on refresh

---

### 6. MicroSD Card Module

**SPI1 Interface (shared bus)**

| STM32F4 Pin | Function | SD Card Pin | Description |
|-------------|----------|-------------|-------------|
| **PA5** | SPI1_SCK | SCK/CLK | SPI Clock |
| **PA6** | SPI1_MISO | MISO/DO | SPI Data Out |
| **PA7** | SPI1_MOSI | MOSI/DI | SPI Data In |
| **PA4** | GPIO_OUT | CS | Chip Select |
| **PB9** | GPIO_IN | CD/DETECT | Card detect (optional) |
| **5V** | Power | VCC | 5V rail if using a **regulated SD module** (most have level shifters + LDO); the card itself is always 3.3V |
| **GND** | Ground | GND | Ground |

**Note:** Most SD card modules have onboard level shifters and regulators  
**SPI1 bus now serves 4 devices** (SD, SHARP, eInk+SRAM) - one CS per device,
everything else shared. All CS idle HIGH at boot.

---

### 7. NULLLAB Mini I2C Jogpad (Amazon B0GT4SBBL1)

**I2C1 Interface (shared with BNO055) - replaces both v1.x buttons**

| STM32F4 Pin | Function | Jogpad Pin (PH2.0-4) | Description |
|-------------|----------|----------------------|-------------|
| **PB6** | I2C1_SCL | SCL | I2C Clock |
| **PB7** | I2C1_SDA | SDA | I2C Data |
| **3.3V** | Power | VCC | Keep on the **3.3V rail**: it is 3-5V rated, but I2C pull-up level behavior at 5V is undocumented and it only draws ~10mA - no benefit to 5V |
| **GND** | Ground | GND | Ground |

**I2C Address:** 0x5A  
**Interface:** X/Y joystick axes (0-255 each), 4 buttons (A/B/C/D) + joystick
click. The onboard MCU reports press / release / single-click / double-click /
long-press per button.

**Suggested control mapping (firmware):**
- Joystick up/down → page navigation (replaces v1.x Mode button)
- Joystick click → select/home (replaces v1.x Button 2)
- A / B / C / D → context actions (scroll, mark event, SD flush, backlight)

**Frees up GPIO:** PB2 and PB10 are no longer consumed by buttons (PB10 → SHARP
CS, PB2 → spare/BOOT1 pad).

---

### 8. Battery Voltage Divider

**ADC1_IN0 = PA0 (analog)**

The v1.x approach (VREFINT via the internal channel) only measures the
**regulated 3.3V rail**, which never moves until the regulator drops out -
useless for gauging the LiPo. v2.0 reads the raw pack voltage:

| STM32F4 Pin | Function | Connection |
|-------------|----------|------------|
| **PA0** | ADC1_IN0 | Divider midpoint |

```
BATT+ ──[100kΩ R1]──┬── PA0 (ADC1_IN0)
                    │
              [330kΩ R2]──[100nF C1]
                    │
                   GND
```

- Full-scale: 4.30V in → 3.30V at PA0 (4.2V LiPo reads 3.22V - headroom OK)
- Source resistance: 100k || 330k ≈ 77kΩ → use ≥480-cycle ADC sampling; the
  100nF cap holds the input steady
- Quiescent drain through divider: ~10µA at 4.2V - negligible, but place the
  divider **after the master power switch** so it doesn't drain a stored pack
- Firmware converts: V_batt = V_adc × (430/100) = V_adc × 4.3

---

### 9. Status LED

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
| PA0 | ADC1_IN0 | Battery divider midpoint |
| PA1 | GPIO_IN | eInk BUSY |
| PA2 | USART2_TX | GPS RX |
| PA3 | USART2_RX | GPS TX |
| PA4 | SPI1_CS | SD Card CS |
| PA5 | SPI1_SCK | SD + SHARP + eInk/SRAM SCK |
| PA6 | SPI1_MISO | SD + SRAM MISO |
| PA7 | SPI1_MOSI | SD + SHARP + eInk/SRAM MOSI |
| PA8 | GPIO_OUT | LoRa RESET |
| PA9 | GPIO_OUT | LoRa RXEN |
| PA10 | GPIO_OUT | LoRa TXEN |
| PA11 | USB D- | (USB if used) |
| PA12 | USB D+ | (USB if used) |
| PA13 | SWDIO | Debug/Programming |
| PA14 | SWCLK | Debug/Programming |
| PA15 | GPIO_IN | LoRa DIO2 (input-only spare) |

### Port B (GPIOB)
| Pin | Function | Connected To |
|-----|----------|--------------|
| PB0 | GPIO_IN | LoRa BUSY |
| PB1 | GPIO_INT | LoRa DIO1 |
| PB2 | Spare | **BOOT1!** Leave as an uncommitted pad (see conflicts section) |
| PB3 | GPIO_OUT | eInk D/C |
| PB4 | GPIO_OUT | eInk RST |
| PB5 | GPIO_OUT | eInk SRAM CS |
| PB6 | I2C1_SCL | BNO055 + Jogpad SCL |
| PB7 | I2C1_SDA | BNO055 + Jogpad SDA |
| PB8 | GPIO_OUT | SHARP DISP |
| PB9 | GPIO_IN | SD Card Detect |
| PB10 | GPIO_OUT | SHARP CS (was Mode Button in v1.x) |
| PB11 | GPIO_OUT | eInk ECS |
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

### I2C1 Bus Map (PB6/PB7, external 4.7kΩ pull-ups)
| Address | Device |
|---------|--------|
| 0x28 (alt 0x29) | BNO055 compass/IMU |
| 0x5A | NULLLAB jogpad |
| ~~0x3C~~ | ~~SSD1309 OLED~~ - removed in v2.0 |

### SPI1 Bus Map (PA5/PA6/PA7 + per-device CS)
| CS Pin | Device | MISO used? | Max speed note |
|--------|--------|-----------|----------------|
| PA4 | SD card | yes | ~82kHz init → up to 10.5MHz run |
| PB10 | SHARP memory LCD | no (write-only) | ≤2MHz, CS active **HIGH** |
| PB11 | eInk SSD1680 | no | refresh is command-driven, low rate |
| PB5 | eInk SRAM buffer | yes | up to 10MHz fine |

---

## Pin Conflicts

### v1.x → v2.0 (new hardware resolved old pressure)
1. **Buttons are gone** - PB10 and PB2 freed by the I2C jogpad. PB10 is now the
   SHARP CS (the old button/LoRa-CS PB12 conflict is fully retired).

2. **PB2 / BOOT1** - now a *spare*. Route it to a test pad with a 100kΩ
   pull-down but do **not** commit it to a peripheral that can drive it high:
   PB2 must read LOW at reset when entering the DFU bootloader (only matters
   when BOOT0 is held high; ignored on normal boots).

3. **SD Card / LoRa Conflict** - RESOLVED (carried from v1.x)
   - LoRa BUSY on **PB0**, LoRa DIO1 on **PB1**
   - SD Card MISO on **PA6**, MOSI on **PA7**

4. **SD Card Detect / LoRa Reset Conflict** - RESOLVED (carried from v1.x)
   - SD detect on **PB9**, LoRa RESET on **PA8**

### v2.0 layout cautions
- **SHARP CS is active HIGH** - opposite polarity from everything else on SPI1.
  Driving 4 devices on one bus, double-check your firmware CS polarity table.
- **SPI1 speed switching:** UART-fast SD (10MHz-class) and ≤2MHz SHARP share
  pins. Firmware already re-prescales SPI1 around every SD transaction; the
  SHARP driver must do the same.
- Don't run eInk refresh, SD logging, and SHARP drawing simultaneously in
  early firmware - sequence them; the eInk full-refresh takes many seconds.

---

## Recommended Breadboard / Layout Sketch

```
┌────────────────────────────────────────────┐
│  Power Rails (v2.2)                         │
│  ● 5V   ════════════════════════════════   │  (USB or LiPo->boost)
│  ● 3.3V ════════════════════════════════   │  (Black Pill LDO / ext. buck)
│  ● GND  ════════════════════════════════   │
│  ● BATT+ (divider, before boost/regulator) │
└────────────────────────────────────────────┘

┌─────────────────┐  ┌──────────────┐
│   STM32F401     │  │ GPS Module   │
│   Black Pill    │  │   (NEO-6M)   │
│  PA2 ─────────────► RX           │
│  PA3 ◄───────────── TX           │
└─────────────────┘  └──────────────┘

┌─────────────────┐  ┌──────────────┐
│   STM32F401     │  │ E22-400M33S  │
│  PB13 ────────────► SCK          │
│  PB14 ◄──────────── MISO         │
│  PB15 ────────────► MOSI         │
│  PB12 ────────────► CS/NSS       │
│  PA8  ────────────► NRST         │
│  PB0  ◄──────────── BUSY         │
│  PB1  ◄──────────── DIO1         │
│  PA9  ────────────► RXEN         │
│  PA10 ────────────► TXEN         │
│  PA15 ◄──────────── DIO2 (in)    │
│  5V   ────────────► VCC          │
│                 │  │ ANT ─── 🏁   │ (Antenna!)
└─────────────────┘  └──────────────┘

I2C1 bus (PB6/PB7):
┌─────────────────┐   ┌──────────────┐   ┌──────────────────┐
│  PB6 ══════════════► SCL BNO055   │   │ NULLLAB Jogpad   │
│  PB7 ══════════════► SDA          │   │  PH2.0-4         │
│        └══════════════════════════════► SCL  (0x5A)      │
│        └══════════════════════════════► SDA              │
└─────────────────┘   └──────────────┘   └──────────────────┘

SPI1 bus (PA5 SCK / PA6 MISO / PA7 MOSI shared):
┌─────────────────┐  ┌──────────────┐  ┌──────────────┐  ┌───────────────┐
│   STM32F401     │  │  SD module   │  │ SHARP 3502   │  │ 4778 FeatherWing│
│  PA4  ────────────► CS           │  │              │  │               │
│  PB10 ────────────────┐          │  │ CS ◄── PB10  │  │               │
│  PB8  ────────────────┴────────────► DISP          │  │               │
│  PB11 ───────────────────────────────┐             │  │ ECS ◄── PB11  │
│  PB3  ──► D/C ───────────────────────┼─────────────┼──► D/C           │
│  PB4  ──► RST ───────────────────────┼─────────────┼──► RST           │
│  PB5  ──► SRCS ──────────────────────┼─────────────┼──► SRCS          │
│  PA1  ◄── BUSY ──────────────────────┼─────────────┼─── BUSY          │
│                                     │  (no MISO)   │  │ SDCS, EN: NC  │
└─────────────────┘  └──────────────┘  └──────────────┘  └───────────────┘

Battery divider:
BATT+ ──[100kΩ]──┬──► PA0
          [330kΩ]│
                 ├────[100nF]──── GND
                 │
                GND
```

---

## Power Consumption Estimate (v2.2, per rail)

**5V rail** (USB or LiPo→boost):
| Component | Current Draw | Notes |
|-----------|--------------|-------|
| STM32F401 + onboard LDO | ~35-55mA | Includes 3.3V loads below |
| GPS Module | ~40-60mA | Acquiring/tracking |
| LoRa (RX) | ~12-15mA | Receive mode |
| LoRa (TX) | ~150-200mA bursts | Command/ACK TX at 5V-rated PA (receiver commands only) |
| BNO055 | ~12mA | Normal operation |
| SD Card | ~50-100mA | During write operations |
| SHARP Memory LCD | <0.1mA avg | 4µA @ 1Hz refresh, no backlight |
| **5V typical** | **~160-250mA** | Without SD writes |
| **5V max** | **~400mA** | SD write + eInk refresh + LoRa TX together |
| **Loss in LDO/boost** | ~10-20% | Budget it |

**3.3V rail** (from Black Pill LDO or external buck):
| Component | Current Draw | Notes |
|-----------|--------------|-------|
| eInk FeatherWing | ~0mA hold / ~40-50mA refresh | Image persists with power off |
| Jogpad | ~10-15mA | Onboard MCU + joystick |
| **3.3V typical** | **~15mA** | Between eInk refreshes |
| **3.3V max** | **~65mA** | |

**Recommended Battery:**
- 3.7V 1S LiPo, 1000-2000mAh for 4-8 hours operation
- **Boost converter to 5V** (MT3608-style module) between battery and 5V rail;
  or use a 5V power bank directly (they are themselves a LiPo + boost)
- **Battery monitoring:** external 100k/330k divider into PA0 (see section 8);
  connect the divider to the raw battery, NOT the 5V rail, and place it after
  the power switch

---

## Recommended Additions (v2.0 wishlist)

Things worth adding while the schematic is open anyway:

1. **Power input protection + switch** - P-channel MOSFET reverse-polarity
   protection (or a Schottky, cost: 0.3V drop) and a slide switch between
   battery and everything (including the divider - it draws 10µA otherwise).
2. **LiPo charge + boost management** - since v2.2 runs a 5V rail, a combined
   TP4056 charge + MT3608 boost board (or TP5100 for 2S configs) covers both:
   charge the pack from USB, boost it to 5V. A cheap 5V power bank is the
   zero-effort equivalent. JST-PH 2-pin battery connector is standard.
3. **Buzzer** - 2N7000-switched passive buzzer for packet-received beeps and
   the low-battery alarm. PA10 is now LoRa TXEN, so the only candidates are
   the PB2 spare pad (consumes the BOOT1 test point) or skipping the buzzer.
4. **SMA or u.FL antenna connector** for 433MHz, near the E22 ANT pad, with a
   copper keep-out and ground pour around it.
5. **4-pin SWD header** (3V3, GND, PA13, PA14). USB-DFU flashing gets annoying
   fast during bring-up; SWD is single-click.
6. **GPS backup battery** - CR1220 to the NEO-6M V_BCKP pin for warm starts
   (fix in seconds instead of 30-60s at the pad).
7. **Keep PB2 on a labeled test pad** (BOOT1) with a solder jumper - future
   spare GPIO without sacrificing DFU.
8. **Debug jumpers/test points** on SPI1 SCK/MISO/MOSI and each CS - you'll
   want a scope on them the first time four SPI devices share a bus.

---

## Firmware Impact of v2.0 (work items, not yet done)

The schematic above assumes the following firmware changes; the current
`receiver/firmware` build still targets the v1.x hardware:

- `button.c` → replace with jogpad I2C driver @ 0x5A (joystick axes + button
  event states); update page navigation and the Rocket Select / navigation
  contexts to use joystick up/down + click.
- `display.c` SSD1309 driver → new SHARP memory LCD driver (SPI1, active-HIGH
  CS, 3KB framebuffer, ~1Hz software VCOM toggle, ≤2MHz clock).
- New SSD1680/SRAM driver module for the eInk FeatherWing; decide which
  display modes render to SHARP (live pages) vs eInk (persistent summary).
- Battery task: read ADC1_IN0 on PA0 through the divider instead of the
  internal VREFINT channel (which only ever reads the 3.3V rail).
- `main.h`: SD pins unchanged; add the new display CS/DC/RST/BUSY defines.

---

## Testing Procedure

### 1. Power-On Test
- [ ] Connect the 5V rail, 3.3V rail, and GND first
- [ ] Verify BOTH rails with a multimeter (5V at the E22 VCC pin, 3.3V at the eInk 3V)
- [ ] Check LED lights on PC13

### 2. SHARP Display Test
- [ ] Verify CS polarity (active HIGH) and ≤2MHz clock
- [ ] Confirm VCOM toggle is running (~1Hz) or do a quick visual test
- [ ] Display initialization page

### 3. eInk Display Test
- [ ] Verify ECS/DC/RST/BUSY/SRCS wiring
- [ ] Full refresh shows test pattern (~15-25 s, flicker is NORMAL)

### 4. GPS Test
- [ ] Place near window for GPS signal
- [ ] Watch for NMEA sentences on PA3
- [ ] Display should show GPS data within 30-60s

### 5. Compass Test
- [ ] BNO055 should calibrate automatically
- [ ] Rotate receiver and verify heading changes

### 6. LoRa Test
- [ ] Power on transmitter with LoRa
- [ ] Verify packet reception on receiver
- [ ] Check RSSI values (should be negative dBm)

### 7. SD Card Test
- [ ] Insert formatted SD card
- [ ] Verify data logging starts

### 8. Jogpad Test
- [ ] Joystick up/down cycles display pages
- [ ] Click/press acts as select/home
- [ ] A/B/C/D events visible in a diagnostics page

### 9. Battery Monitor Test
- [ ] Confirm PA0 reads ~0.77 × pack voltage (e.g. 3.2V for a full 4.2V pack)
- [ ] Check displayed battery voltage within ±5% of multimeter reading

---

## Troubleshooting

### SHARP Display Not Working
- Check CS polarity - it is **active HIGH**, unlike typical SPI
- Keep SPI clock ≤2MHz for the SHARP
- Verify DISP pin is high
- Display will slowly degrade if VCOM never toggles - check firmware

### eInk Not Responding
- Verify both ECS and SRCS are wired (display AND SRAM buffer)
- BUSY is on a separate breakout pad - solder a wire to PA1
- Full refresh flickering for ~20s is normal behavior
- If image is garbled, check you're sharing SPI1 with the right CS held idle HIGH

### Jogpad Not Responding
- Verify I2C address 0x5A with a bus scan
- Power the pad from 3.3V, not 5V, when sharing the I2C bus with 3.3V devices
- Check external 4.7kΩ pull-ups on PB6/PB7

### GPS No Data
- Ensure clear view of sky
- Check UART connections (PA2/PA3 might be swapped)
- Verify baud rate is 9600

### LoRa No Packets
- **Check antenna is connected!**
- Verify SPI connections (PB13/14/15)
- Ensure transmitter is powered and configured

### Compass Not Responding
- Check I2C shared bus (jogpad and BNO055 must have different addresses: 0x5A vs 0x28)
- Verify BNO055 address (0x28 or 0x29)
- May need calibration (wave figure-8 motion)

### SD Card Not Detected
- Verify card is FAT32 formatted
- Check SPI connections (PA5/6/7) - remember this bus is now shared with 3 other devices
- Ensure all other SPI1 CS lines are idle HIGH during SD transactions

---

## Safety Notes

⚡ **Electrical Safety**
- Never exceed 3.3V on any STM32 **signal** pin (battery divider limits PA0 to
  3.22V max) - the 5V rail is for VCC lines only, never for logic traces
- Never feed the 5V rail into the eInk FeatherWing 3V pin or the jogpad VCC
- E22 VCC must stay within 3.3-5.5V
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
- **Adafruit 3502 SHARP Memory LCD:** ~$25
- **Adafruit 4778 2.9" tri-color eInk FeatherWing:** ~$25
- **NULLLAB I2C jogpad (B0GT4SBBL1):** ~$9
- **MicroSD module:** ~$2-4
- **Antenna 433MHz:** ~$2-5 (or make 17.3cm wire dipole)
- **100k + 330k resistors, 100nF cap, LiPo, charger module:** ~$5-10

### Total Cost
**Approximately $95-130** for complete receiver build

---

**Document Version:** 2.2  
**Last Updated:** 2026-11-01  
**Target Hardware:** Receiver v2.2 (v2.1 + dual 5V/3.3V power rails; E22, GPS, BNO055, SHARP, SD on 5V)  
**Firmware Status:** `receiver/firmware` matches this doc (power changes need no firmware)
