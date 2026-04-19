#!/bin/bash
# Quick installation script for arduino-cli on Linux

set -e

echo "==================================="
echo "Arduino CLI Installation Script"
echo "==================================="
echo ""

# Check if already installed
if command -v arduino-cli &> /dev/null; then
    echo "✓ arduino-cli is already installed"
    arduino-cli version
    echo ""
    read -p "Reinstall anyway? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Skipping installation"
        exit 0
    fi
fi

echo "Installing arduino-cli..."
echo ""

# Download and install
cd /tmp
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=/tmp sh

# Move to system location
echo ""
echo "Installing to /usr/local/bin/ (requires sudo)..."
sudo mv /tmp/arduino-cli /usr/local/bin/
sudo chmod +x /usr/local/bin/arduino-cli

echo ""
echo "✓ arduino-cli installed successfully"
arduino-cli version

echo ""
echo "==================================="
echo "Installing Adafruit SAMD Board Support"
echo "==================================="
echo ""

# Install board support
arduino-cli core update-index
arduino-cli core install adafruit:samd

echo ""
echo "✓ Installation complete!"
echo ""
echo "Next steps:"
echo "  1. Connect your ItsyBitsy M4 via USB"
echo "  2. Update callsign in include/mpu_config.h"
echo "  3. Run: make flash"
echo ""
echo "For more info, see README_ITSYBITSY_M4.md"
