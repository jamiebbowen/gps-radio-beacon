# LoRa RF Initialization Timing Fixes

## Problem Identified
The receiver was showing "RF Init Failed!" due to clock mismatches between the system configuration and SPI timing expectations.

## Root Cause Analysis

### Clock Configuration Mismatch
- **System Clock:** 42MHz (intentionally reduced for UART timing compatibility)
- **APB1 Bus (SPI2):** 42MHz ÷ 2 = **21MHz**
- **Old SPI2 Prescaler:** ÷8 → 21MHz ÷ 8 = **2.625MHz** ❌ TOO SLOW
- **Old Code Comment:** "84MHz/8 = 10.5MHz" (incorrect for 42MHz system)

The SX1268 LoRa chip in the E22-400M33S module was likely timing out or failing to respond at the too-slow 2.625MHz SPI clock.

## Fixes Applied

### 1. SPI Clock Speed Correction
**File:** `receiver/firmware/src/rf_receiver.c` line 141

**Changed:**
```c
hspi_lora.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; // 84MHz/8 = 10.5MHz
```

**To:**
```c
hspi_lora.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2; // APB1=21MHz, 21MHz/2 = 10.5MHz
```

**Result:** SPI2 now runs at 10.5MHz (correct speed for SX1268)

### 2. Increased Reset and Boot Delays
**File:** `receiver/firmware/src/lora.c` lines 72-82

**Added:**
- Increased post-reset delay: 50ms → 100ms
- Added BUSY pin diagnostic check
- Additional 100ms wait if BUSY is initially high
- Returns `LORA_BUSY` error code if module doesn't respond

This helps identify hardware connection issues vs timing issues.

### 3. Increased SPI Timeout Values
**File:** `receiver/firmware/src/lora.c` (all SPI functions)

**Changed:** All HAL_SPI timeouts from 100ms → 500ms
- `LoRa_SendCommand()` - all timeouts
- `LoRa_ReadCommand()` - all timeouts
- `LoRa_WriteRegister()` - all timeouts
- `LoRa_ReadRegister()` - all timeouts
- `LoRa_WriteBuffer()` - all timeouts
- `LoRa_ReadBuffer()` - all timeouts
- `LoRa_WaitOnBusy()` timeout parameter

**Rationale:** More generous timeouts prevent premature failures during initialization when the LoRa module is processing complex configuration commands.

## Expected Results

With these fixes, the LoRa module should now:
1. ✅ Receive SPI commands at the correct 10.5MHz clock speed
2. ✅ Have sufficient time to complete initialization sequences
3. ✅ Provide better diagnostic info if hardware issues exist (stuck BUSY pin)

## Troubleshooting

### If "RF Init Failed!" Still Appears:

**Check Hardware Connections:**
- SPI2: PB13(SCK), PB14(MISO), PB15(MOSI)
- Control: PB12(CS), PA8(RESET), PB0(BUSY), PB1(DIO1)
- Verify 3.3V power to E22-400M33S module
- Check all ground connections

**Measure with Oscilloscope (if available):**
- SPI CLK (PB13): Should show ~10.5MHz pulses during init
- BUSY (PB0): Should go LOW after reset (not stuck HIGH)
- RESET (PA8): Should pulse LOW then HIGH during init

**Common Issues:**
1. **Loose wiring** - Check all connections
2. **Wrong voltage** - E22-400M33S requires 3.3V (not 5V!)
3. **Defective module** - Try different E22-400M33S if available
4. **Pin conflicts** - Verify no other code is using these pins

## Build Information
- Compiled successfully: ✅
- Firmware size: 81,480 bytes (81KB)
- No increase in RAM usage
- Compatible with existing receiver hardware

## Related Files Modified
1. `receiver/firmware/src/rf_receiver.c` - SPI clock prescaler
2. `receiver/firmware/src/lora.c` - Timing and diagnostics
3. `receiver/firmware/src/display.c` - Display rotation (separate fix)
