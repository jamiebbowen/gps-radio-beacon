# AGENTS.md

Notes for anyone (human or agent) changing this repository. The beacon is
flight hardware: a change that breaks telemetry or recovery is found in a
field, not on a bench. Keep changes small, test-first, and verified.

## Layout

- `transmitter/firmware/` – airborne beacon (Adafruit ItsyBitsy M4 / SAMD51,
  Arduino): u-blox GPS, BNO085 IMU, SX1268 (E22-400M33S) LoRa, on-chip
  flight recorder. Tests in `transmitter/tests/`.
- `receiver/firmware/` – handheld receiver (STM32F401 Black Pill, HAL):
  NEO-6M GPS, BNO085 compass, SX1268 LoRa, SHARP/OLED displays, LittleFS on
  microSD. Tests in `receiver/tests/`.
- `receiver/tools/`, `tools/` – host-side analysis, SD extraction, checks.
- `docs/` – hardware, radio, reliability and field procedures.

## Commands (run from the repo root)

| What | Command |
|------|---------|
| All host tests (tools + RX + TX) | `make test` |
| Same tests under ASan + UBSan + float-cast-overflow | `make sanitize` |
| Analysis-tool tests + packet-format drift guard | `make tools-test` |
| SD extraction tool tests | `make -C tools/lfs_extract test` |
| Packet-format drift guard only | `python3 tools/test_packet_format_sync.py` |
| Coverage, every module, lines + branches | `make -C transmitter/tests coverage`, `make -C receiver/tests coverage` |
| Receiver firmware, warnings as errors | `make -C receiver/firmware compile WERROR=1` |
| Transmitter firmware, all warnings | `arduino-cli compile --fqbn adafruit:samd:adafruit_itsybitsy_m4 --warnings all --build-path /tmp/tx-build transmitter/firmware` |
| Reachability (dead-function) scan | `python3 tools/reachability.py --tx-map /tmp/tx-build/firmware.ino.map` (after both builds) |

Top-level `make` only builds; it never flashes a board. Flashing is always an
explicit `make -C receiver/firmware flash` or `make -C transmitter/firmware upload`.

## Rules that protect the flight

- **Edit both `packet_format.h` copies together** (`transmitter/firmware/include/`
  and `receiver/firmware/inc/`). The drift guard fails on any mismatch, and
  `PF_STATIC_ASSERT` pins every struct to its `*_PACKET_SIZE`. Receive-side
  structs may be larger only to hold a legacy layout still accepted from the air.
- **IMU trace V3:** bit 15 of `ts_ms` (`IMU_TS_V3_FLAG`) marks true centi-g /
  milli-g / 0.1 deg/s units. Keep decoding older formats (RX `ImuFmt` log
  column, `analyze_flight.py`).
- **Never reset the GPS in flight.** The watchdog's UBX-CFG-RST ladder (hot →
  warm → cold) only runs on the ground; in LAUNCH/POST_LAUNCH it re-sends
  settings only (`gps_set_in_flight`).
- **Never erase a surviving flight log on reboot.** A FLYING marker means
  resume (`flight_log_resume`), not re-arm.
- **The TX sketch must end below `0x00040000`**, where the 256 KiB flight-log
  region (`FLOG_BASE`) starts.
- Encode wire/flash integers with the saturating helpers in
  `transmitter/firmware/include/fixed_point.h`, never a bare cast.

## Testing conventions

- Every bug fix starts with a regression test that fails before the fix.
- Fakes must behave like the real part. Several bugs here hid behind fakes
  written to the code's assumptions (the BNO085 SHTP read, the LoRa CRC
  path); when you write a fake, model the datasheet or a known-good driver.
- TX host tests build with `-fno-exceptions`, matching the SAMD core.
- No function may exist only for tests: the reachability scan must report
  nothing but functions inlined into callers in their own file.

## Git conventions

- Work on a topic branch; small, single-purpose commits.
- Subject prefixes: `tx:`, `rx:`, `proto:` (both sides' wire format),
  `tests:`, `tools:`, `docs:`, `ci:`, `build:`, `chore:`.
- Explain *why* in the body; mention anything that still needs bench
  verification on hardware.
