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
import pathlib
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
1200.000,EVENT,RF REGAINED after 600s
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

# Firmware/tool contract: the HEARTBEAT row that receiver/tests/
# test_log_policy.c pins (marked LOG_CONTRACT) must parse here, field for
# field - a format change on either side fails one of the two suites.
import re
_c_test = (pathlib.Path(__file__).resolve().parent.parent
           / "receiver" / "tests" / "test_log_policy.c").read_text()
_pinned = re.search(r'LOG_CONTRACT heartbeat[^\n]*\n\s*CHECK\(strcmp\(m, "([^"]+)"\)', _c_test)
CHECKED += 1
if not _pinned:
    FAILURES.append("LOG_CONTRACT heartbeat row not found in test_log_policy.c")
else:
    _m = log_summary.HEARTBEAT_RE.search(_pinned.group(1))
    if not _m or _m.groups() != ("3", "0", "9", "1", "120", "-80", "acq", "0"):
        FAILURES.append(f"HEARTBEAT_RE does not parse the firmware row {_pinned.group(1)!r}")

# Sections exist at all (the rot detector)
for section in ("== Boot / reset ==", "== RF link ==", "== TX GPS health",
                "== Telemetry ==", "== Airframe binding / foreign traffic =="):
    expect(section, out)

# Content wiring: each row type lands in its section
expect("scan/channel at 0:08", out)                       # RF scan lock
expect("LOST (5 min silence) at 15:00", out)              # blackout marker
expect("REGAINED at 20:00 after 600 s of silence", out)    # blackout length
expect("2 heartbeats: SILENT=1  acq=1", out)              # EVENT-prefixed HBs parse
expect("max rst=1  <-- TX GPS watchdog had to recover it (reset ladder)", out)
expect("RF bound rocket_id=3", out)                       # binding breadcrumbs
expect("RF bound rocket_id=7", out)
expect("FOREIGN TRAFFIC: 25:00..26:00", out)              # foreign drop window
expect("GPS: 1 rows", out)                                # NAV row accounting
expect("RSSI [-85..-85]", out)
expect("Operator track: 2 fixes, 30:00..30:30", out)      # BASE row accounting

# Fused dead-reckoning share: FusedFlags (column 12) bit1 = DR. Reported
# from the FUS rows themselves - this log has no BASE rows.
FUS_LOG = """Timestamp,Type,Data
0.000,EVENT,RESET src=POR vdd=3290mV
10.000,NAV,FUS,39.890,-105.115,1650.0,9,1.0,0.0,0.0,0,0x05,39.0,-105.0,1000.0,1.00,12.0,90.0,-85,-5
11.500,NAV,FUS,39.890,-105.115,1650.0,9,1.0,0.0,0.0,255,0x03,39.0,-105.0,1000.0,1.00,12.0,90.0,-86,-5
""".strip() + "\n"
fus_out = run_report(FUS_LOG)
expect("FUS: 2 rows", fus_out)
expect("FUS: 1 dead-reckoning rows (50%)", fus_out)

# The empty-section "OK" lines must NOT claim OK when content exists
boot_idx = out.index("== Boot / reset ==")
rf_idx = out.index("== RF link ==")
boot_section = out[boot_idx:rf_idx]
expect("OK - no watchdog resets", boot_section)

# Watchdog resets are ERROR rows (current single-row wording and the older
# two-row build). The summary used to look for them only in EVENT rows and
# reported "OK - no watchdog resets" for every real log.
WD_LOG = """Timestamp,Type,Data
0.000,ERROR,RESET src=IWDG vdd=3300mV (firmware hang)
5.000,EVENT,RF scan locked CH0 433.00MHz
600.000,ERROR,Boot after IWDG watchdog reset (firmware hang)
""".strip() + "\n"
wd_out = run_report(WD_LOG)
wd_boot = wd_out[wd_out.index("== Boot / reset =="):wd_out.index("== RF link ==")]
expect("IWDG watchdog reset logged at 0:00", wd_boot)
expect("IWDG watchdog reset logged at 10:00", wd_boot)
CHECKED += 1
if "OK - no watchdog resets" in wd_boot:
    FAILURES.append("watchdog resets present but the boot section says OK")

# Every event prefix log_summary keys on must still be a message the
# receiver firmware can write: a renamed message silently drops out of
# the summary (the IWDG wording once did). Compared against the static
# part of each C string literal (up to its first % conversion); a
# template like "RESET src=%s" also needs the rest ("IWDG") as a literal.
import glob
_repo = pathlib.Path(__file__).resolve().parent.parent
_prefixes = sorted(set(re.findall(r'msg\.startswith\("([^"]+)"\)',
                                  (_repo / "tools" / "log_summary.py").read_text())))
_lits = []
for _f in glob.glob(str(_repo / "receiver" / "firmware" / "src" / "**" / "*.c"), recursive=True):
    _lits += re.findall(r'"((?:[^"\\]|\\.)*)"', open(_f, errors="ignore").read())
_static = [l.split("%", 1)[0] for l in _lits]
for _p in _prefixes:
    CHECKED += 1
    _need = min(len(_p), 6)
    _ok = False
    for _s in _static:
        if len(_s) < _need:
            continue
        if _s.startswith(_p):
            _ok = True
        elif _p.startswith(_s):
            _rest = _p[len(_s):].strip()
            _ok = not _rest or any(l.startswith(_rest) for l in _lits)
        if _ok:
            break
    if not _ok:
        FAILURES.append(f"log_summary keys on {_p!r}, but no receiver firmware message starts with it")

if FAILURES:
    for f in FAILURES:
        print(f"FAIL: {f}")
    print("\n--- full report ---")
    print(out)
    sys.exit(1)

print(f"log_summary self-test: all {CHECKED} expectations passed")
sys.exit(0)
