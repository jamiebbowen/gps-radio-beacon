# SD Card Troubleshooting Guide

## Enhanced Diagnostics (Receiver Firmware)

The SD Card display mode now shows detailed diagnostic information:

```
SD Card: Present/Not Found
Status: Ready/Error
Err: <code>              ← FRESULT error code
<error_name>             ← Human-readable error
Files: <count>
Written: <size>
CMD0: 0x<hex>            ← CMD0 response from card
MISO:H/L DET:H/L        ← Pin states
```

## Error Code Reference

### FRESULT Error Codes
| Code | Name | Meaning | Common Causes |
|------|------|---------|---------------|
| **0** | FR_OK | Success | Normal operation |
| **1** | FR_DISK_ERR | Disk I/O error | Bad SPI connection, timing issues, card failure |
| **3** | FR_NOT_READY | Drive not ready | Card not initialized, CMD0 failed |
| **13** | FR_NO_FILESYSTEM | No FAT filesystem | Card not formatted, wrong format (exFAT/NTFS) |
| **99** | DRIVER_LINK | Driver link failed | Internal firmware error |

### CMD0 Response Codes
| Response | Meaning | Status |
|----------|---------|--------|
| **0x01** | In Idle State | ✅ Normal - card responding correctly |
| **0xFF** | No Response | ❌ Card not detected or bad SPI connection |
| **0x00** | Ready State | ⚠️ Card already initialized (unusual at startup) |
| Other | Invalid | ❌ Communication error |

### Pin States
| Signal | Expected State | Meaning if Wrong |
|--------|----------------|------------------|
| **MISO** | HIGH (H) when idle | LOW = card pulling line down (bad connection?) |
| **DET** | LOW (L) if card present | Hardware-dependent, currently bypassed |

## Common Issues & Solutions

### Issue 1: "Err: 3 NOT_READY" + "CMD0: 0xFF"

**Symptoms:**
```
SD Card: Present
Status: Error
Err: 3
NOT_READY
CMD0: 0xFF
MISO:H DET:H
```

**Diagnosis:** Card not responding to commands

**Causes:**
1. **Bad SPI wiring** - Most common
2. **SPI bus conflict** - Another device interfering
3. **Card not seated properly** in socket
4. **Defective SD card**

**Solutions:**
1. **Check wiring with multimeter:**
   ```
   STM32F401 → SD Card Socket
   PA5 (SPI1_SCK)  → Pin 5 (CLK)
   PA6 (SPI1_MISO) → Pin 7 (MISO/DAT0)
   PA7 (SPI1_MOSI) → Pin 2 (MOSI/CMD)
   PA4 (CS)        → Pin 1 (CS/DAT3)
   GND             → Pin 3, 6 (GND)
   3.3V            → Pin 4 (VDD)
   ```

2. **Check for shorts/opens:**
   - Continuity test each wire
   - Check for solder bridges on SD socket
   - Verify no shorts to ground or VCC

3. **Verify SPI bus:**
   - LoRa module (SPI2) should not interfere
   - Try removing LoRa module temporarily

4. **Try different SD card:**
   - Some cards are finicky
   - Use quality brand (SanDisk, Samsung)

### Issue 2: "Err: 13 NO_FILESYSTEM" + "CMD0: 0x01"

**Symptoms:**
```
SD Card: Present
Status: Error
Err: 13
NO_FILESYSTEM
CMD0: 0x01
MISO:H DET:H
```

**Diagnosis:** Card is working but filesystem is wrong

**Causes:**
1. **Card not formatted**
2. **Wrong filesystem** (exFAT, NTFS instead of FAT32)
3. **Corrupted filesystem**

**Solutions:**
1. **Format card on PC:**
   - **Windows:** Right-click drive → Format → FAT32
   - **Linux:** `sudo mkfs.vfat -F 32 /dev/sdX1`
   - **Mac:** Disk Utility → Erase → MS-DOS (FAT)

2. **Use SD Card Formatter tool:**
   - Download from SD Association website
   - More thorough than OS built-in formatter

3. **Important:** Must be FAT32, not exFAT!
   - Cards >32GB default to exFAT (not supported)
   - Use SD Card Formatter tool for large cards

### Issue 3: "Err: 1 DISK_ERR" + "CMD0: 0x01"

**Symptoms:**
```
SD Card: Present
Status: Error
Err: 1
DISK_ERR
CMD0: 0x01
MISO:H DET:H
```

**Diagnosis:** Card responds but I/O operations fail

**Causes:**
1. **Timing issues** - SPI too fast/slow
2. **Electrical noise**
3. **Power supply issues**
4. **Marginal card**

**Solutions:**
1. **Check SPI speed:**
   - Current: 164 kHz (APB2 21MHz / 128)
   - Should be <400 kHz for init ✅
   - Try slower if needed (prescaler 256)

2. **Check power supply:**
   - Measure 3.3V at card socket
   - Should be stable ±5%
   - Add 10µF capacitor near socket

3. **Check signal integrity:**
   - Keep wires short (<6 inches)
   - Avoid running near noisy signals
   - Add series resistors (47Ω-100Ω) on SPI lines

4. **Try different card:**
   - Some cards need more robust timing

### Issue 4: "SD Card: Not Found"

**Symptoms:**
```
SD Card: Not Found
Insert SD card
```

**Diagnosis:** Detection logic says no card present

**Note:** Currently **bypassed** in code - always returns "present"

**If you enable detection:**
```c
// In sd_card.c, change line 133:
return (pin_state == GPIO_PIN_RESET);  // Active low detect
```

**Solutions:**
1. **Check detect pin:**
   - PB9 should be LOW when card inserted
   - Check socket detect switch with multimeter

2. **Bypass detection permanently:**
   - Keep current code (`return true;`)
   - Rely on CMD0 response instead

## Hardware Checklist

### Wiring Verification
- [ ] PA5 → SD CLK (Pin 5)
- [ ] PA6 → SD MISO/DAT0 (Pin 7)
- [ ] PA7 → SD MOSI/CMD (Pin 2)
- [ ] PA4 → SD CS/DAT3 (Pin 1)
- [ ] GND → SD GND (Pins 3, 6)
- [ ] 3.3V → SD VDD (Pin 4)
- [ ] All connections have continuity
- [ ] No shorts between adjacent pins

### SD Card Requirements
- [ ] Card is microSD or SD (with adapter)
- [ ] Card capacity ≤32GB (for easy FAT32)
- [ ] Card formatted as FAT32 (not exFAT/NTFS)
- [ ] Card is quality brand (SanDisk, Samsung, Kingston)
- [ ] Card is not write-protected
- [ ] Card works in PC/camera

### Power Supply
- [ ] 3.3V measured at socket: _____ V
- [ ] Voltage stable during card access
- [ ] Ground connection solid
- [ ] Decoupling capacitor present (10µF)

## Debugging Steps

### Step 1: Visual Inspection
1. Remove receiver from case
2. Inspect SD socket solder joints
3. Check for cold solder joints, bridges
4. Verify card fully inserted

### Step 2: Electrical Testing
1. Power off, insert card
2. Measure resistance from microcontroller pins to socket pins
   - Should be <1Ω for each connection
3. Measure 3.3V at socket VDD pin
4. Check GND continuity

### Step 3: Signal Testing (with oscilloscope)
1. Power on with card inserted
2. Probe CLK line (PA5)
   - Should see clock pulses at ~164 kHz
3. Probe MOSI line (PA7)
   - Should see data transmission
4. Probe MISO line (PA6)
   - Should see card responses

### Step 4: Software Testing
1. Flash firmware with diagnostics
2. Navigate to SD Card mode (button press)
3. Read diagnostic display
4. Note error code and CMD0 response

### Step 5: Isolation Testing
1. **Test SPI1 bus:**
   - Disconnect LoRa module (SPI2)
   - Try SD card alone
   
2. **Test SD card:**
   - Try different card
   - Format on PC first
   - Verify card works in PC

3. **Test power:**
   - Measure voltage during init
   - Add larger decoupling cap if voltage drops

## Code Changes for Deeper Debugging

### Enable Verbose SPI Logging

Add to `sd_diskio.c` after CMD0 response:

```c
// In SD_initialize() after line 137:
char debug_msg[64];
snprintf(debug_msg, sizeof(debug_msg), "SD CMD0 Response: 0x%02X\r\n", cmd0_response);
// Send via UART if you have debug console
```

### Test Raw SPI Communication

Add test function in `sd_diskio.c`:

```c
void SD_TestSPI(void) {
    SD_CS_HIGH();
    HAL_Delay(100);
    
    // Send dummy bytes, check MISO response
    for (int i = 0; i < 10; i++) {
        uint8_t rx = SD_SPI_TxRx(0xFF);
        // Should see 0xFF on MISO when no card activity
        // Store rx for inspection
    }
}
```

### Slow Down SPI Even More

In `main.c`, change `MX_SPI1_Init()`:

```c
hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;  // ~82 kHz
```

## Known Working Configuration

```
Hardware:
- STM32F401CCU6 (Black Pill board)
- Generic microSD card socket
- SanDisk 8GB microSD (FAT32)

Software:
- SPI1: 164 kHz init speed
- FatFs R0.14b
- Detection bypassed (always true)

Wiring:
- PA5/PA6/PA7/PA4 on SPI1
- Short wires (<4 inches)
- 10µF cap at socket VDD
```

## Quick Diagnostic Decision Tree

```
1. Is card inserted?
   NO → Insert card, check socket
   YES → Continue

2. Navigate to SD Card mode. What does it show?

   "SD Card: Not Found"
   → Check detection pin or bypass detection code

   "CMD0: 0xFF"
   → Bad SPI connection or no card
   → Check wiring with multimeter

   "CMD0: 0x01" + "Err: 13 NO_FILESYSTEM"
   → Card responding but needs formatting
   → Format as FAT32 on PC

   "CMD0: 0x01" + "Err: 1 DISK_ERR"
   → Timing or signal integrity issue
   → Check power supply, slow down SPI

   "CMD0: 0x01" + "Status: Ready"
   → ✅ Card working! Check why you thought it wasn't

   Other responses
   → Try different card, check power
```

## Success Indicators

When SD card is working correctly, you'll see:

```
SD Card: Present
Status: Ready
Size: 7625MB           ← Actual card size
Free: 7620MB           ← Available space
Files: 1               ← Log files created
Written: 512KB         ← Data logged
CMD0: 0x01            ← Card initialized
MISO:H DET:L          ← Proper pin states
```

## Still Not Working?

### Last Resort Options

1. **Replace SD socket:**
   - Socket may be defective
   - Solder joints may be cracked

2. **Use SPI bit-bang:**
   - Slower but more reliable
   - Implement manual SPI in software

3. **Alternative storage:**
   - Use internal flash for logging
   - Add external SPI flash chip
   - Stream data via radio

4. **Hardware redesign:**
   - Consider SD card with different pinout
   - Use SD card breakout board
   - Switch to SDIO mode (needs more pins)

## Testing with Known-Good Hardware

To isolate firmware vs hardware issues:

1. **Test with Arduino:**
   ```cpp
   // Simple SD test sketch
   #include <SD.h>
   void setup() {
       if (SD.begin(4)) {  // CS on pin 4
           Serial.println("SD OK");
       }
   }
   ```

2. **Test with logic analyzer:**
   - Capture SPI transactions
   - Compare with SD card specification
   - Look for timing violations

3. **Test card in PC:**
   - Verify card is not defective
   - Check filesystem integrity

## Additional Resources

- **SD Card Specifications:** sdcard.org/downloads/pls/
- **FatFs Documentation:** elm-chan.org/fsw/ff/00index_e.html
- **SPI Mode SD Cards:** ece.ucsb.edu/~parhami/ece15...
- **STM32 SPI Guide:** st.com/resource/en/application_note/

## Revision History

- 2025-11-24: Initial version with enhanced diagnostics
- Added error code decoder to display
- Added pin state monitoring
- Added comprehensive troubleshooting steps
