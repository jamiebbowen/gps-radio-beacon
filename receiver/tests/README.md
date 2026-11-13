# Receiver Host-Side Unit Tests

Unit tests for hardware-independent receiver firmware modules, compiled and
run on the development machine (no STM32 hardware or ARM toolchain needed,
and no STM32CubeF4 checkout - the FatFs headers are stubbed too).

## Running

```bash
make            # build and run all tests
make coverage   # run with gcov and print line-coverage for modules under test
make clean
```

## What is covered

| Module | Test file | Line coverage (Nov 2026) |
|---|---|---|
| `rf_parser.c` — all 3 RF packet formats from the transmitter | `test_rf_parser.c` | 100% |
| `gps_parser.c` — local NMEA parser (checksum, GGA/RMC, jump filter) | `test_gps_parser.c` | 100% |
| `math_utils.c` — haversine distance, bearing, angle normalization | `test_math_utils.c` | 100% |
| `rf_receiver.c` — scan/wedge/reacquisiton/blackout pacing | `test_rf_receiver.c` | 100% |
| `button.c` — debounce/short/long-press policy | `test_button.c` | 100% |
| `display.c` — SSD1309 driver incl. headless I2C fault ladder | `test_display.c` | 100% |
| `gps.c` — sentence RX path, FIFO drop-oldest, watchdog, rx stats | `test_gps.c` | 100% |
| `sd_card.c` — LittleFS logging incl. dirty-card & IO-error throttle | `test_sd_card.c` | 92.9% |
| `lora.c` — SX1268 driver incl. CAD/DC-DC regime | `test_lora.c` | 100% |
| `compass.c` — BNO085 incl. declination lattice & quat-lock table | `test_compass.c` | 100% |
| `i2c_bus.c` — shared-bus wedge ladder (streak → slow → offline → probe) | `test_i2c_bus.c` | 100% |
| `jogpad.c` — 5-button I2C pad: wire-order remap + poll throttle | `test_jogpad.c` | 100% |
| `eink.c` — SSD1680 framebuffer, refresh FSM, SPI speed pinning | `test_eink.c` | 100% |
| `sd_diskio.c` — SD-over-SPI driver behind a scripted virtual card | `test_sd_diskio.c` | 100% |
| `navigation_mode.c` — L/O/D arrow, state decay | `test_navigation_mode.c` | (UI-critical only) |
| `preflight_mode.c` — go/no-go verdicts incl. battery veto | `test_preflight_mode.c` | (UI-critical only) |
| display_modes/ small pages — channel/RFstats/GPS/SD/RF/compass/I2C/SPI/IMU/pattern | `test_display_modes.c` | (layout contract) |

(The transmitter's EKF is tested separately in `transmitter/tests/`, ~100%.)

The remaining ~7% of `sd_card.c` is the LittleFS-fault recovery ladder
(mount-after-format, mid-session write errors, self-test readback
mismatch) — correct defensive code that the RAM block device can't
drive faithfully (a failed `lfs_prog` leaves littlefs' program cache
dirty and trips `LFS_ASSERT` on the next operation). Kept as
review-covered defensive code; see the NOTE in `test_sd_card.c`.

`rf_parser.c` decodes every packet format sent by the transmitter:

| Format | Parser | Size |
|---|---|---|
| ASCII CSV (`lat,lon,alt[,sats]` / callsign) | `RF_Parser_ParseAsciiPacket` | variable |
| Binary GPS fix | `RF_Parser_ParseBinaryPacket` | 14 B (V2, +rocket_id), 13 B legacy V1 |
| EKF fused position + velocity | `RF_Parser_ParseFusedPacket` | 20 B (V2, +rocket_id), 19 B legacy V1 |

`test_rf_receiver.c` additionally carries the launch-day scenario battery:
airframe binding and foreign-beacon filtering (same-modem co-channel
traffic), the sequential-rocket livelock breaker, over-the-horizon
blackout/re-acquisition, fringe CRC-error forensics, 49.7-day tick
wraparound, receiver cold-boots mid-flight / mid-walk, the in-link live
noise-floor estimator, and the completed-deaf-window wedge report.

Tests include NMEA→decimal-degree conversion accuracy, hemisphere signs,
malformed/NULL/short packet rejection, coordinate range validation, flag
decoding, and regression guards for two subtle behaviors:

- Fused packets must not overwrite the raw-GPS `fix` field (UI label flapping).
- Raw GPS packets must not zero the fused velocity fields (speed display
  flickering to 0.0 m/s).

`test_gps_parser.c` builds its NMEA fixtures with a checksum-computing
helper, so sentences can never fail due to a hand-computed checksum typo.
Note `gps_parser.c` keeps static jump-filter state with no reset API, so
all valid-fix fixtures use the same geographic area.

`test_sd_diskio.c` is the largest new suite: a scripted virtual SD card
behind fake SPI registers exercises SDv2-HC / SDv1 / MMC init, ACMD41
warm-up retries, single- and multi-block reads and writes, CMD12
termination, both CSD sector-count formulas, write-token rejection, busy
timeouts, the fastest-first speed ladder, and the per-transaction
prescaler reassertion that keeps the negotiated speed alive across
shared-SPI1 display traffic.

## How it works

`stubs/` contains minimal stand-ins for `stm32f4xx_hal.h` /
`stm32f4xx_hal_uart.h` / `stm32f4xx_hal_spi.h` plus the FatFs `ff.h` /
`diskio.h` surface, so the real firmware sources compile unmodified with
the host `gcc` - no vendor trees required. Keep stubs behavior-free;
tests must remain hardware-independent.

## Adding tests

Add a `TEST(name) { ... }` block in the relevant `test_<module>.c` and call
`run_<name>()` from `main`. Use `CHECK(cond)` / `CHECK_NEAR(a, b, tol)` from
`test_harness.h`. To cover a new pure-logic module, create
`test_<module>.c` and append `<module>` to `MODULES` in the Makefile — the
binary is automatically linked against `../firmware/src/<module>.c`.
