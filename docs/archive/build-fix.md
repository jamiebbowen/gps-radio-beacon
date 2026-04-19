# Transmitter Build Fix

## Problem
The transmitter firmware was failing to compile with duplicate definition errors for all global variables and functions (`setup()`, `loop()`, etc.).

## Root Cause
The Arduino build system compiles all `.ino` and `.cpp` files in the sketch directory. You had:

1. **firmware.ino** - Main Arduino sketch (7808 bytes)
2. **main.cpp** - Identical content to firmware.ino (7808 bytes)  
3. **gps_radio_beacon.ino** - Wrapper that `#include "main.cpp"`

This meant the same code was being compiled **multiple times**, causing redefinition errors.

## Solution Applied

### Files Renamed (Backed Up):
- `main.cpp` → `main.cpp.bak` ✅
- `gps_radio_beacon.ino` → `gps_radio_beacon.ino.bak` ✅

### Files Kept:
- `firmware.ino` - The main Arduino sketch ✅
- All other `.cpp` files (beacon.cpp, gps.cpp, radio.cpp, uart.cpp, launch_detect.cpp) ✅

### Makefile Fixed:
Removed `*.ino` from the `clean` target to prevent accidentally deleting the main sketch file.

**Before:**
```makefile
clean:
	rm -rf $(BUILD_DIR)
	rm -f *.o *.elf *.hex *.ino
```

**After:**
```makefile
clean:
	rm -rf $(BUILD_DIR)
	rm -f *.o *.elf *.hex
```

## Build Results
✅ **Compilation successful!**
- **Sketch size:** 58,424 bytes (11% of flash)
- **Target:** ItsyBitsy M4 Express (SAMD51)
- **No errors**, only harmless F_CPU redefinition warnings

## How to Build & Flash

```bash
# Compile only
make compile

# Compile and upload to board
make flash

# Monitor serial output
make monitor

# Clean build artifacts
make clean
```

## Technical Details

### Arduino Build System Behavior:
The Arduino IDE/CLI automatically compiles:
1. All `.ino` files (concatenated together)
2. All `.cpp` files in the sketch directory
3. All `.cpp` files in subdirectories

Having both `firmware.ino` and `main.cpp` with identical content created duplicate symbols.

### Why Keep firmware.ino Instead of main.cpp?
Arduino expects at least one `.ino` file in the sketch directory. The `.ino` file:
- Gets preprocessed to add necessary Arduino headers
- Defines the sketch name
- Is the entry point for the Arduino build system

## Backup Files (Can Be Deleted If Desired)
- `main.cpp.bak` - Backup of main.cpp
- `gps_radio_beacon.ino.bak` - Backup of wrapper file

These are kept for reference but are not used in the build.

## Future Considerations
If you need to structure code differently:
1. Put implementation `.cpp` files in a `src/` subdirectory
2. Keep only one `.ino` file in the main directory
3. The `.ino` file should be minimal - just includes and `setup()`/`loop()`
