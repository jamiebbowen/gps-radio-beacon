# Receiver Host-Side Unit Tests

Unit tests for hardware-independent receiver firmware modules, compiled and
run on the development machine (no STM32 hardware or ARM toolchain needed).

## Running

```bash
make            # build and run all tests
make coverage   # run with gcov and print line-coverage for modules under test
make clean
```

## What is covered

| Module | Test file | Line coverage |
|---|---|---|
| `rf_parser.c` — all 3 RF packet formats from the transmitter | `test_rf_parser.c` | ~91% |
| `gps_parser.c` — local NMEA parser (checksum, GGA/RMC, jump filter) | `test_gps_parser.c` | ~89% |
| `math_utils.c` — haversine distance, bearing, angle normalization | `test_math_utils.c` | 100% |

(The transmitter's EKF is tested separately in `transmitter/tests/`, ~100%.)

`rf_parser.c` decodes every packet format sent by the transmitter:

| Format | Parser | Size |
|---|---|---|
| ASCII CSV (`lat,lon,alt[,sats]` / callsign) | `RF_Parser_ParseAsciiPacket` | variable |
| Binary GPS fix | `RF_Parser_ParseBinaryPacket` | 13 bytes |
| EKF fused position + velocity | `RF_Parser_ParseFusedPacket` | 21 bytes |

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

## How it works

`stubs/` contains minimal stand-ins for `stm32f4xx_hal.h` /
`stm32f4xx_hal_uart.h` so the real firmware sources compile unmodified with
the host `gcc`. Keep stubs behavior-free; tests must remain
hardware-independent.

## Adding tests

Add a `TEST(name) { ... }` block in the relevant `test_<module>.c` and call
`run_<name>()` from `main`. Use `CHECK(cond)` / `CHECK_NEAR(a, b, tol)` from
`test_harness.h`. To cover a new pure-logic module, create
`test_<module>.c` and append `<module>` to `MODULES` in the Makefile — the
binary is automatically linked against `../firmware/src/<module>.c`.
