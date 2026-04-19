# Documentation index

Reference docs for the GPS radio beacon. Each file is self-contained; links
between them are cross-referenced where relevant.

## Hardware / wiring

- [`hardware-receiver.md`](hardware-receiver.md) - STM32F401 Black Pill pin
  assignments, peripherals, power, and connector layout.
- [`hardware-transmitter.md`](hardware-transmitter.md) - ItsyBitsy M4 Express
  pin assignments for GPS, LoRa, IMU, and power.
- [`pin-comparison.md`](pin-comparison.md) - Side-by-side TX vs RX pin-map
  quick reference.

## Radio / RF

- [`lora-radio.md`](lora-radio.md) - E22-400M33S module configuration and
  the software conversion from the original simple 433 MHz modules.
- [`reliability.md`](reliability.md) - Consolidated reliability doc:
  transmitter data validation, receiver signal-quality monitoring, LoRa
  link budget, and regulatory notes.

## Sensors / subsystems

- [`launch-detection.md`](launch-detection.md) - BNO085 IMU-based launch
  detection: threshold, debounce, and tuning guidance.
- [`gps-watchdog.md`](gps-watchdog.md) - Automatic GPS recovery system for
  silent fix loss.

## Operations / troubleshooting

- [`sd-card-troubleshooting.md`](sd-card-troubleshooting.md) - Diagnostics
  for the receiver SD-card logger.
- [`flight-analysis.md`](flight-analysis.md) - Using
  `receiver/tools/analyze_flight.py` for post-flight stats, KML replay,
  and MP4 rendering.

## Archive

[`archive/`](archive/) holds historical material that is no longer part of
the "current truth" but is worth preserving for context:

- Bug post-mortems (`clock-analysis`, `magic-number-solution`, `sd-cmd0-fix`,
  `lora-timing-fix`, `pin-changes-summary`)
- Proposals not (yet) adopted (`binary-packet-proposal`)
- Documents superseded by the main README or by a consolidated doc in this
  folder (`complete-system-summary`, `migration-guide`, `port-summary`,
  `build-fix`, `upload-fix`, `e22-400m33s-wiring`)
