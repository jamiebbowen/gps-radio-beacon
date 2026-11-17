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
| Everything CI runs, before pushing | `make ci-local` |
| All host tests (tools + RX + TX) | `make test` |
| Same tests under ASan + UBSan + float-cast-overflow | `make sanitize` |
| Analysis-tool tests + packet-format drift guard | `make tools-test` |
| SD extraction tool tests | `make -C tools/lfs_extract test` |
| Packet-format drift guard only | `python3 tools/test_packet_format_sync.py` |
| Coverage, every module, lines + branches | `make -C transmitter/tests coverage`, `make -C receiver/tests coverage` |
| Mutation score of one module (on demand, not CI) | `python3 tools/mutate.py rx sd_card.c` (`tx flight_events.cpp`, `--list`, `--jobs N`) |
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
- **SAMD51 NVMCTRL takes byte addresses** (`NVMCTRL->ADDR = addr`; the SAMD21
  `addr >> 1` convention silently writes elsewhere). The test fake models the
  page buffer and byte ADDR; keep it that way.
- Encode wire/flash integers with the saturating helpers in
  `transmitter/firmware/include/fixed_point.h`, never a bare cast.
- **`yield()` is the TX's IMU service hook.** `firmware.ino` defines it:
  RadioLib's blocking `transmit()` (~1 s per packet) calls it on every turn
  of its wait, and while `radio_tx_blocking()` it runs `launch_detect_update()`
  + `nav_predict()`. Keep it re-entrancy-guarded and rate-limited, never touch
  the radio or SPI from it, and build with the Arduino USB stack (TinyUSB
  defines its own `yield()`; the sketch `#error`s on it).
- **Read the GPS between in-flight transmits** (`gps_poll_between_sends()` in
  `firmware.ino`). Serial1's receive ring is 350 bytes, ~2.3 s of GGA+RMC,
  and one airborne pass can block for several seconds of SF10 airtime: a new
  send in `loop()` while airborne gets a GPS read in front of it.
- **A write the SD card refuses is `LFS_ERR_IO`, never `LFS_ERR_CORRUPT`**
  (`lfs_sd_bd.c`). CORRUPT tells LittleFS the block is bad: it relocates
  through every free block (hours on a 16 GB card, so the IWDG resets the
  receiver) and sd_card.c's write-error ladder never sees the error. The
  card's FTL retires its own bad flash.
- **The SD card's LittleFS geometry is `LFS_SD_*` in `lfs_sd_bd.h`**, the one
  copy: the firmware, `sd_card.c`'s file caches and `tools/lfs_extract`
  build from it. Changing the block size makes every formatted card
  unreadable (and the firmware never reformats a card holding LittleFS).
- **TC3 is the altimeter-input sampler** (38.4 kHz, `altimeter_input.cpp`;
  D11/D12/D13 are the J6/J5/J2 serial inputs). Its 48 MHz source runs open
  loop in flight: keep `alt_rx_sample`'s edge re-sync (the window is pinned
  by test_altimeter's clock-tolerance sweep). Don't reuse TC3 or those pins;
  the core's `tone()` takes TC0 on the SAMD51, and the sketch must keep not using it.

## Testing conventions

- Every bug fix starts with a regression test that fails before the fix.
- Fakes must behave like the real part. Several bugs here hid behind fakes
  written to the code's assumptions (the BNO085 SHTP read, the LoRa CRC
  path, the SD block device's write error); when you write a fake, model
  the datasheet or a known-good driver.
- One SD card for the RX tests: `receiver/tests/fake_sd_card.h`, faked at the
  sd_diskio sector API under the real `lfs_sd_bd.c`. Add a fault knob there
  rather than giving a test a block device of its own.
- TX host tests build with `-fno-exceptions`, matching the SAMD core.
- Every firmware source has host coverage: the coverage gate fails a source
  missing from `make coverage`'s report. A new module needs a test and a
  `COV_PAIRS` entry; `EXEMPT` in `tools/check_coverage.py` (with the reason)
  is the only way out, and a stale exemption fails too.
- The receiver's `main.c` builds on the host (`receiver/tests/test_main.c`:
  boot and loop over the real SD stack, one forked process per boot). Keep
  it that way: target-only code goes under `#if defined(__arm__)`, and a
  new loop breadcrumb or boot step gets a scenario there.
- No function may exist only for tests: the reachability scan must report
  nothing but functions inlined into callers in their own file.

## Git conventions

- Work on a topic branch; small, single-purpose commits.
- Subject prefixes: `tx:`, `rx:`, `proto:` (both sides' wire format),
  `tests:`, `tools:`, `docs:`, `ci:`, `build:`, `chore:`.
- Explain *why* in the body; mention anything that still needs bench
  verification on hardware.
