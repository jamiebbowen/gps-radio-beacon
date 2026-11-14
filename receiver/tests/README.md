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

| Module | Test file | Lines / branches |
|---|---|---|
| `rf_parser.c` — binary GPS and fused packets, callsign text | `test_rf_parser.c` | 99.3% / 82.9% |
| `gps_parser.c` — local NMEA parser (checksum, GGA/RMC, jump filter + re-anchor) | `test_gps_parser.c` | 99.1% / 84.4% |
| `math_utils.c` — haversine, bearing, angles, `Nav_Compute` | `test_math_utils.c` | 100% / 80.8% |
| `rf_receiver.c` — scan/wedge/re-acquisition, binding, heartbeats, PING scheduler | `test_rf_receiver.c` | 99.7% / 89.6% |
| `preflight_eval.c` — the shared go/no-go evaluation | `test_preflight_eval.c` | 100% / 100% |
| `button.c` — debounce/short/long-press policy | `test_button.c` | 100% / 90.0% |
| `display.c` — SSD1309 driver incl. headless I2C fault ladder | `test_display.c` | 100% / 97.7% |
| `gps.c` — sentence RX path, FIFO drop-oldest, watchdog, rx stats | `test_gps.c` | 100% / 96.7% |
| `sd_card.c` — LittleFS logging incl. IO-error retry, failed syncs, recovery ladder | `test_sd_card.c` | 98.2% / 71.3% |
| `lora.c` — SX1268 driver incl. CAD, CRC-frame capture, long-preamble TX | `test_lora.c` | 100% / 88.5% |
| `compass.c` — heading pipeline incl. declination lattice & quat-lock table | `test_compass.c` | 100% / 84.3% |
| `bno085.c` — SHTP transport and SH-2 record framing (hub-faithful fake) | `test_compass.c` | 100% / 88.0% |
| `i2c_bus.c` — shared-bus wedge ladder (streak → slow → offline → probe) | `test_i2c_bus.c` | 100% / 97.1% |
| `jogpad.c` — 5-button I2C pad: wire-order remap + poll throttle | `test_jogpad.c` | 100% / 100% |
| `eink.c` — SSD1680 framebuffer, refresh FSM, SPI speed pinning | `test_eink.c` | 100% / 89.8% |
| `sd_diskio.c` — SD-over-SPI driver behind a scripted virtual card | `test_sd_diskio.c` | 100% / 78.1% |
| `navigation_mode.c` — arrow, range budget, readiness, link/status rows | `test_navigation_mode.c` | 98.6% / 86.7% |
| `preflight_mode.c` — go/no-go checklist page | `test_preflight_mode.c` | 100% / 90.9% |
| display_modes/ small pages — channel/RF stats/GPS/SD/RF/compass | `test_display_modes.c` | 100% each |
| `rf_parser.c` + `gps_parser.c` — fuzz over mutated real packets | `test_fuzz_parsers.c` | (sanitizer) |

Figures from `make coverage`; every unit test also runs under ASan + UBSan
+ float-cast-overflow with `make sanitize`.

(The transmitter's EKF is tested separately in `transmitter/tests/`, ~100%;
its `nmea_fields.c` helpers get the same fuzz treatment in
`test_fuzz_nmea.c`.)

The remaining ~2% of `sd_card.c` is the self-test's readback-mismatch
branch: the test file is small enough to live inline in littlefs metadata,
so corrupting it on the RAM block device fails the metadata CRC instead of
reaching the comparison. The write-error retry, failed-sync and recovery
ladder paths are driven by prog-fault knobs on the RAM block device.

`rf_parser.c` decodes every packet format sent by the transmitter:

| Format | Parser | Size |
|---|---|---|
| Callsign text (`CALLSIGN-id CHn`) | `RF_Parser_ParseAsciiPacket` | variable |
| Binary GPS fix | `RF_Parser_ParseBinaryPacket` | 14 B (V2, +rocket_id), 13 B legacy V1 |
| EKF fused position + velocity | `RF_Parser_ParseFusedPacket` | 19 B V3 (flags bit 0 set, +rocket_id), 20 B V2, 19 B legacy V1 |

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
`gps_parser.c` keeps static jump-filter state with no reset API: valid-fix
fixtures share one geographic area, and tests that move the anchor reset
the static state directly (the test #includes the source).

`test_fuzz_parsers.c` runs a structure-aware fuzzer over the RF and NMEA
parsers: a seeded xorshift mutates a corpus of real wire packets (every
version of every packet type, plus valid NMEA/callsign strings) through
the real parser sources, built with `-fsanitize=address,undefined
-fno-sanitize-recover=all`. Any UB/OOB aborts with the printed seed for
reproduction. The default 200k iterations run in ~0.1 s inside `make
test`; soak runs use `FUZZ_ITERS`/`FUZZ_SEED` (30M+ iterations across
seeds are clean as of this writing). The binary skips `COVFLAGS` because
gcov and sanitizer instrumentation don't mix.

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
