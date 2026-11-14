# GPS Radio Beacon Receiver Firmware

This firmware is designed for an STM32F401 microcontroller-based receiver that tracks the location of a transmitter using GPS coordinates.

## Hardware Components
- STM32F401 "Black Pill" (HCLK 42 MHz; SPI1 on the 10.5 MHz APB2)
- E22-400M33S (SX1268) LoRa module: 433 MHz, SF10 / 62.5 kHz / CR 4/8
- NEO-6M GPS module (UART)
- BNO085 IMU as the compass (I2C, SHTP)
- SHARP memory LCD or SSD1309 OLED, optional SSD1680 e-ink panel
- 5-button I2C jogpad
- microSD card with a LittleFS log (not FAT)

## Build and flash

```sh
make compile WERROR=1   # build only, warnings as errors (what CI runs)
make flash              # explicit flash; nothing else ever writes the board
```

Host-side unit tests live in `../tests/` (`make -C ../tests test`).

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
