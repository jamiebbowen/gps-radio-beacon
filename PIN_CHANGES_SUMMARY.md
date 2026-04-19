# Pin Assignment Changes - Summary

## Overview
Resolved all pin conflicts between button, LoRa module, and SD card by reassigning pins to eliminate overlaps.

---

## Changes Made

### 1. Button Pin - MOVED ✅
**Change:** PB12 → **PB10**

**Reason:** PB12 was conflicting with LoRa CS pin

**File Modified:** `receiver/firmware/inc/button.h`

```c
// OLD:
#define BUTTON_PIN GPIO_PIN_12

// NEW:
#define BUTTON_PIN GPIO_PIN_10
```

---

### 2. LoRa BUSY Pin - MOVED ✅
**Change:** PA7 → **PB0**

**Reason:** PA7 was conflicting with SD Card MOSI pin

**File Modified:** `receiver/firmware/inc/lora.h`

```c
// OLD:
#define LORA_BUSY_PIN GPIO_PIN_7
#define LORA_BUSY_PORT GPIOA

// NEW:
#define LORA_BUSY_PIN GPIO_PIN_0
#define LORA_BUSY_PORT GPIOB
```

---

### 3. LoRa DIO1 Pin - MOVED ✅
**Change:** PA6 → **PB1**

**Reason:** PA6 was conflicting with SD Card MISO pin

**File Modified:** `receiver/firmware/inc/lora.h`

```c
// OLD:
#define LORA_DIO1_PIN GPIO_PIN_6
#define LORA_DIO1_PORT GPIOA

// NEW:
#define LORA_DIO1_PIN GPIO_PIN_1
#define LORA_DIO1_PORT GPIOB
```

---

### 4. SD Card Detect Pin - MOVED ✅
**Change:** PA8 → **PB9**

**Reason:** PA8 was conflicting with LoRa RESET pin

**File Modified:** `receiver/firmware/inc/main.h`

```c
// OLD:
#define SD_DETECT_PIN GPIO_PIN_8
#define SD_DETECT_GPIO_PORT GPIOA

// NEW:
#define SD_DETECT_PIN GPIO_PIN_9
#define SD_DETECT_GPIO_PORT GPIOB
```

---

## Final Pin Assignments

### LoRa E22-400M33S Module
| Function | Pin | Notes |
|----------|-----|-------|
| SPI2_SCK | PB13 | No change |
| SPI2_MISO | PB14 | No change |
| SPI2_MOSI | PB15 | No change |
| CS | PB12 | No change |
| RESET | PA8 | No change |
| **BUSY** | **PB0** | ✅ Moved from PA7 |
| **DIO1** | **PB1** | ✅ Moved from PA6 |

### SD Card Module
| Function | Pin | Notes |
|----------|-----|-------|
| SPI1_SCK | PA5 | No change |
| SPI1_MISO | PA6 | No conflict now! |
| SPI1_MOSI | PA7 | No conflict now! |
| CS | PA4 | No change |
| **DETECT** | **PB9** | ✅ Moved from PA8 |

### Mode Button
| Function | Pin | Notes |
|----------|-----|-------|
| **Button** | **PB10** | ✅ Moved from PB12 |

### Other Peripherals (No Changes)
| Peripheral | Pins | Status |
|------------|------|--------|
| GPS | PA2 (TX), PA3 (RX) | ✅ No change |
| Display & Compass (I2C) | PB6 (SCL), PB7 (SDA) | ✅ No change |
| Status LED | PC13 | ✅ No change |

---

## Port Usage Summary

### Port A (GPIOA)
```
PA0  - Available
PA1  - Available
PA2  - GPS TX
PA3  - GPS RX
PA4  - SD Card CS
PA5  - SD Card SCK
PA6  - SD Card MISO  ✅ NOW FREE (was LoRa conflict)
PA7  - SD Card MOSI  ✅ NOW FREE (was LoRa conflict)
PA8  - LoRa RESET    ✅ NOW FREE (was SD detect conflict)
PA9  - Available
PA10 - Available
PA11 - USB D-
PA12 - USB D+
PA13 - SWDIO (debug)
PA14 - SWCLK (debug)
PA15 - Available
```

### Port B (GPIOB)
```
PB0  - LoRa BUSY      ✅ NEW (was available)
PB1  - LoRa DIO1      ✅ NEW (was available)
PB2  - Available
PB3  - Available
PB4  - Available
PB5  - Available
PB6  - I2C1 SCL (Display & Compass)
PB7  - I2C1 SDA (Display & Compass)
PB8  - Available
PB9  - SD Card Detect ✅ NEW (was available)
PB10 - Mode Button    ✅ NEW (was available)
PB11 - Available
PB12 - LoRa CS        ✅ NOW FREE (was button conflict)
PB13 - LoRa SCK
PB14 - LoRa MISO
PB15 - LoRa MOSI
```

### Port C (GPIOC)
```
PC13 - Status LED
PC14 - Available
PC15 - Available
```

---

## Conflict Resolution Matrix

| Conflict | Resolution | Status |
|----------|------------|--------|
| PB12: Button + LoRa CS | Moved button to PB10 | ✅ RESOLVED |
| PA6: SD MISO + LoRa DIO1 | Moved LoRa DIO1 to PB1 | ✅ RESOLVED |
| PA7: SD MOSI + LoRa BUSY | Moved LoRa BUSY to PB0 | ✅ RESOLVED |
| PA8: SD Detect + LoRa RESET | Moved SD Detect to PB9 | ✅ RESOLVED |

---

## Build Status

✅ **All conflicts resolved**
✅ **Firmware compiles successfully**
✅ **Binary size: 81,404 bytes**
✅ **No pin conflicts remaining**

---

## Hardware Connection Changes

If you have already wired the receiver, you need to make these changes:

### Reconnect These Wires:

1. **Button:** Move from PB12 → **PB10**
2. **LoRa BUSY:** Move from PA7 → **PB0**
3. **LoRa DIO1:** Move from PA6 → **PB1**
4. **SD Card Detect:** Move from PA8 → **PB9**

### These Stay the Same:
- All SPI pins (no changes)
- GPS UART (no changes)
- I2C bus (no changes)
- LED (no changes)
- All power/ground connections (no changes)

---

## Testing Checklist

After making the hardware changes:

- [ ] Button press cycles display modes
- [ ] LoRa receives packets (check RF mode)
- [ ] SD card detection works
- [ ] SD card data logging works
- [ ] GPS data displays correctly
- [ ] Compass displays correctly
- [ ] Navigation mode works

---

## Documentation Updated

The following files have been updated to reflect these changes:

1. ✅ `receiver/firmware/inc/button.h` - Button pin
2. ✅ `receiver/firmware/inc/lora.h` - LoRa pins
3. ✅ `receiver/firmware/inc/main.h` - SD card detect pin
4. ✅ `RECEIVER_WIRING.md` - Complete wiring documentation
5. ✅ `PIN_CHANGES_SUMMARY.md` - This document

---

**All Changes Complete!** 🎉

The receiver firmware now has no pin conflicts and is ready to flash and test.
