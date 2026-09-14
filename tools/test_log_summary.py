#!/usr/bin/env python3
"""Regression test for tools/log_summary.py.

Builds a synthetic flight-day log (boot, pad, launch silence, foreign
beacon traffic, binding change, heartbeats) and pins the key report lines.
Run directly (`python3 tools/test_log_summary.py`) or via the top-level
`make test-tools`. Exits non-zero on failure.

Why this exists: the heartbeat section silently never matched anything for
a whole release (rows are EVENT-prefixed; the parser looked for a HEARTBEAT
kind) and nobody noticed until a manual audit. A log tool that can rot
silently is worse than no tool: it breeds false confidence.
"""

import contextlib
import importlib.util
import io
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("log_summary",
                                              os.path.join(HERE, "log_summary.py"))
log_summary = importlib.util.module_from_spec(spec)
spec.loader.exec_module(log_summary)

SYNTHETIC_LOG = """Timestamp,Type,Data
0.000,EVENT,RESET src=POR vdd=3290mV
8.000,EVENT,RF scan locked CH0 433.00MHz
9.000,EVENT,RF bound rocket_id=3 CH0 433.00MHz
30.000,EVENT,HEARTBEAT id=3 CH0 sats=9 fix=0 up=120s rssi=-80 gps=acq rst=0
35.000,EVENT,HEARTBEAT id=3 CH0 sats=0 fix=0 up=125s rssi=-81 gps=SILENT rst=1
60.123,NAV,GPS,39.890,-105.115,1650.0,9,0.0,0.0,0.0,0,0x43,39.0,-105.0,1000.0,1.00,12.0,90.0,-85,-5
900.000,EVENT,RF LOST (>5min silence)
1500.000,EVENT,RF foreign pkts=1 dropped CH0 rssi=-78dBm snr=4
1560.000,EVENT,RF foreign pkts=5 dropped CH0 rssi=-77dBm snr=4
1800.000,BASE,39.890123,-105.115100,1652.0,8,1.1
1830.000,BASE,39.890200,-105.115050,1652.5,8,1.0
2100.000,EVENT,RF bound rocket_id=7 CH0 433.00MHz
""".strip() + "\n"


def run_report(lines: str) -> str:
    with tempfile.NamedTemporaryFile("w", suffix=".TXT", prefix="L", delete=False) as f:
        f.write(lines)
        path = f.name
    try:
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = log_summary.main(["log_summary.py", path])
        assert rc == 0
        return buf.getvalue()
    finally:
        os.unlink(path)


FAILURES = []
CHECKED = 0


def expect(needle, out):
    global CHECKED
    CHECKED += 1
    if needle not in out:
        FAILURES.append(f"missing report line: {needle!r}")


out = run_report(SYNTHETIC_LOG)

# Sections exist at all (the rot detector)
for section in ("== Boot / reset ==", "== RF link ==", "== TX GPS health",
                "== Telemetry ==", "== Airframe binding / foreign traffic =="):
    expect(section, out)

# Content wiring: each row type lands in its section
expect("scan/channel at 0:08", out)                       # RF scan lock
expect("LOST (5 min silence) at 15:00", out)              # blackout marker
expect("2 heartbeats: SILENT=1  acq=1", out)              # EVENT-prefixed HBs parse
expect("max rst=1  <-- TX had to cold-restart its GPS", out)
expect("RF bound rocket_id=3", out)                       # binding breadcrumbs
expect("RF bound rocket_id=7", out)
expect("FOREIGN TRAFFIC: 25:00..26:00", out)              # foreign drop window
expect("GPS: 1 rows", out)                                # NAV row accounting
expect("RSSI [-85..-85]", out)
expect("Operator track: 2 fixes, 30:00..30:30", out)      # BASE row accounting

# The empty-section "OK" lines must NOT claim OK when content exists
boot_idx = out.index("== Boot / reset ==")
rf_idx = out.index("== RF link ==")
boot_section = out[boot_idx:rf_idx]
expect("OK - no watchdog resets", boot_section)

if FAILURES:
    for f in FAILURES:
        print(f"FAIL: {f}")
    print("\n--- full report ---")
    print(out)
    sys.exit(1)

print(f"log_summary self-test: all {CHECKED} expectations passed")
sys.exit(0)
