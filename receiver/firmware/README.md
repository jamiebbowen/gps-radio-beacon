# GPS Radio Beacon Receiver Firmware

This firmware is designed for an STM32F401 microcontroller-based receiver that tracks the location of a transmitter using GPS coordinates.

## Hardware Components
- STM32F401 Microcontroller
- QMX588L Compass Module
- NEO6M GPS Module
- RXM-433-LR RF Receiver
- SSD1309 OLED Display

## Functionality
The system receives GPS coordinates from a remote transmitter via the RF receiver, compares them with the local GPS position, and uses the compass to determine the relative direction and distance to the transmitter. The information is displayed on the OLED screen.

## Display Information
- Direction (degrees off from straight ahead)
- Distance to transmitter (meters)
- Time since last received ping

## Project Structure
- `inc/` - Header files
- `src/` - Source files
- `lib/` - Vendored libraries (littlefs is tracked; STM32CubeF4 is not - see below)
- `Makefile` - Build configuration

## Building

Requires `arm-none-eabi-gcc` and a local copy of STM32CubeF4, which is too
large (1+ GB) to track in this repository. On a fresh clone, fetch it once:

```bash
git clone --depth 1 --branch v1.28.2 \
    https://github.com/STMicroelectronics/STM32CubeF4.git lib/STM32CubeF4
cd lib/STM32CubeF4
git submodule update --init --depth 1 \
    Drivers/STM32F4xx_HAL_Driver \
    Drivers/CMSIS/Device/ST/STM32F4xx
```

Then build with:

```bash
make
```

Output artifacts land in `bin/` (`.elf`, `.bin`, `.hex`). CI performs the
same steps (see `.github/workflows/ci.yml`).
