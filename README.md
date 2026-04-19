# GPS Radio Beacon

A long-range rocket-tracking system: an airborne **transmitter** continuously
reports GPS position over LoRa, and a handheld **receiver** displays bearing,
distance, and altitude to help recover the rocket after landing.

Tested on a Mach-0.99 / 2 km AGL flight with a 37 dB link margin — see
[`docs/flight-analysis.md`](docs/flight-analysis.md) for post-flight tooling.

## System overview

```
+-------------------------+           LoRa 433 MHz          +-------------------------+
|       TRANSMITTER       |  - - - - - - - - - - - - - ->  |         RECEIVER        |
|  (flies with the rocket)|        ~8-12 km LoS range       |   (handheld ground)     |
+-------------------------+                                 +-------------------------+
  - ItsyBitsy M4 (SAMD51)                                     - STM32F401 Black Pill
  - u-blox GPS (UART)                                         - NEO-6M GPS (UART)
  - BNO085 IMU (I2C) - launch detect                          - QMC5883L compass (I2C)
  - E22-400M33S LoRa (SPI)                                    - E22 LoRa (SPI)
  - Power: 3.3 V + LiPo                                       - SSD1309 OLED (I2C)
                                                              - microSD log (SPI)
```

The transmitter runs a state machine (`PRE_LAUNCH` -> `LAUNCH` -> `POST_LAUNCH`
-> `BATTERY_SAVE`) that auto-adapts TX cadence based on IMU-detected liftoff.
The receiver logs every packet to SD card for post-flight analysis.

## Repository layout

```
.
|-- transmitter/
|   |-- firmware/            Arduino/C++ code for the ItsyBitsy M4
|   |                        (radio, GPS, IMU, beacon state machine)
|   `-- pcb/                 KiCad schematic + PCB design
|
|-- receiver/
|   |-- firmware/            Bare-metal STM32 HAL code
|   |                        (radio, GPS, compass, OLED, SD card, navigation UI)
|   `-- tools/
|       `-- analyze_flight.py  Post-flight log analyzer + KML/MP4 renderer
|
|-- docs/                    Reference documentation (wiring, radio config,
|                            reliability, troubleshooting)
|   `-- archive/             Historical bug post-mortems and migration notes
|
`-- README.md                This file
```

## Quick start

### Transmitter (ItsyBitsy M4)

```bash
cd transmitter/firmware
make install-deps     # one-time: installs arduino-cli + Adafruit SAMD core
make compile
make flash            # flashes via /dev/ttyACM0 by default
make monitor          # optional: USB serial log
```

Edit `include/mpu_config.h` to set your amateur-radio callsign.

### Receiver (STM32F401)

```bash
cd receiver/firmware
make                  # builds bin/firmware.elf
# Flash with st-flash, OpenOCD, or DFU as appropriate for your toolchain
```

### Flight analysis

After a flight, pull the SD card from the receiver and run:

```bash
python3 receiver/tools/analyze_flight.py /path/to/Lxxxxxxx.TXT
```

Options:

| Flag | Purpose |
|---|---|
| `--kml flight.kml` | Google-Earth replay with time-animated rocket track |
| `--mp4 flight.mp4` | Headless 3-D animated replay (matplotlib + ffmpeg) |
| `--duration 15` | Compress flight into N-second video |
| `--sensitivity -134` | Override receiver sensitivity for range prediction |

Prints altitude, velocity, slant range, RSSI/SNR trace, predicted max range,
and packet-continuity statistics. See
[`docs/flight-analysis.md`](docs/flight-analysis.md) for details.

## Documentation

Detailed reference docs live in [`docs/`](docs/):

- **Hardware wiring**: receiver and transmitter pin assignments
- **LoRa radio**: E22-400M33S configuration and link-budget notes
- **Launch detection**: BNO085 IMU threshold tuning
- **GPS watchdog**: automatic recovery from fix loss
- **Reliability**: signal-quality monitoring, staleness, validation
- **SD card troubleshooting**: diagnostics for the receiver logger
- **Flight analysis**: using `analyze_flight.py`

Post-mortems of specific bug fixes are preserved under
[`docs/archive/`](docs/archive/).

## License / legal

The LoRa link operates on the 433 MHz band. In the US this falls under the
amateur-radio 70 cm allocation; the beacon transmits its callsign every
few minutes for FCC compliance. **Check your local regulations and hold
an appropriate license before transmitting.**
