#!/bin/bash
# Wrapper script to upload firmware with proper library path

BOSSAC="/home/jbbowen/snap/arduino-cli/62/.arduino15/packages/adafruit/tools/bossac/1.8.0-48-gb176eee/bossac"
PORT="${1:-/dev/ttyACM0}"
FIRMWARE="build/firmware.ino.bin"

# Check if firmware exists
if [ ! -f "$FIRMWARE" ]; then
    echo "Error: Firmware not found at $FIRMWARE"
    echo "Run 'make compile' first"
    exit 1
fi

# Check if port exists
if [ ! -e "$PORT" ]; then
    echo "Error: Port $PORT not found"
    echo "Available ports:"
    ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "  None found"
    exit 1
fi

echo "Uploading to ItsyBitsy M4 on $PORT..."
echo ""
echo "*** IMPORTANT: Double-tap the RESET button on your board NOW ***"
echo "    (The board should show a pulsing LED)"
echo ""
echo "Waiting 5 seconds for bootloader mode..."
sleep 5

# Look for the bootloader port
BOOT_PORT=""
for i in /dev/ttyACM*; do
    if [ -e "$i" ]; then
        BOOT_PORT="$i"
        echo "Found port: $BOOT_PORT"
        break
    fi
done

if [ -z "$BOOT_PORT" ]; then
    echo "Error: No bootloader port found!"
    echo "Make sure you double-tapped the reset button."
    exit 1
fi

# Upload using bossac with library path fix
echo "Flashing firmware..."
# SAMD51 needs offset=0x4000 and specific flags
LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu \
    $BOSSAC -d --port=${BOOT_PORT#/dev/} -U -i --offset=0x4000 -w -v -b "$FIRMWARE" -R

RESULT=$?

if [ $RESULT -eq 0 ]; then
    echo "✓ Upload successful!"
else
    echo "✗ Upload failed with exit code $RESULT"
fi

exit $RESULT
