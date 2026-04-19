# Transmitter Upload Fix

## Problem
Upload fails with missing library error:
```
bossac: error while loading shared libraries: libstdc++.so.6: cannot open shared object file
```

This is caused by snap-installed arduino-cli being unable to access system libraries due to snap's sandboxing.

## Solutions

### Solution 1: Arduino IDE Upload (EASIEST)

1. Install Arduino IDE (if not already installed)
2. Open Arduino IDE
3. File → Open → `firmware.ino`
4. Tools → Board → Adafruit SAMD Boards → Adafruit ItsyBitsy M4 Express
5. Tools → Port → Select `/dev/ttyACM0` (or your board's port)
6. Click Upload button (→)

### Solution 2: Manual Bootloader Upload

1. **Double-tap the reset button** on ItsyBitsy M4 Express
   - The red LED will start pulsing/fading
   - Board is now in bootloader mode
2. A USB drive called **ITSYBOOT** will appear
3. Convert the .bin file to .uf2 format:
   ```bash
   # Install uf2conv tool (if needed)
   wget https://raw.githubusercontent.com/microsoft/uf2/master/utils/uf2conv.py
   
   # Convert binary to UF2
   python3 uf2conv.py -c -b 0x4000 -f SAMD51 build/firmware.ino.bin -o firmware.uf2
   ```
4. **Drag and drop** `firmware.uf2` onto the ITSYBOOT drive
5. Board will automatically restart with new firmware

### Solution 3: Fix Snap Arduino-CLI Permissions

Run these commands to give snap proper permissions:
```bash
sudo snap connect arduino-cli:hardware-observe
sudo snap connect arduino-cli:mount-observe  
sudo snap connect arduino-cli:raw-usb
sudo snap refresh arduino-cli
```

Then try uploading again:
```bash
make flash
```

### Solution 4: Install Arduino-CLI Without Snap

Remove snap version and install directly:
```bash
# Remove snap version
sudo snap remove arduino-cli

# Install via curl
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

# Add to PATH
echo 'export PATH=$PATH:$HOME/bin' >> ~/.bashrc
source ~/.bashrc

# Verify installation
arduino-cli version

# Re-run board installation
cd /home/jbbowen/Desktop/Rocketry/gps-radio-beacon/transmitter/firmware
bash install_arduino_cli.sh
```

Then try:
```bash
make flash
```

## Current Status

- ✅ **Compilation:** SUCCESS (58,424 bytes)
- ❌ **Upload:** FAILED (missing library in snap)
- 📋 **Port:** /dev/ttyACM0

## Recommended Approach

**Use Arduino IDE** for now - it's the quickest solution and doesn't require changing your arduino-cli installation.

If you want to continue using the Makefile workflow, apply Solution 3 or 4 to fix the arduino-cli snap issues permanently.
