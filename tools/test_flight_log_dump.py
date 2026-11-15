#!/usr/bin/env python3
"""Self-test for tools/flight_log_dump.py.

1. Firmware contract: the DUMP rows transmitter/tests/test_flight_log.cpp
   pins (marked LOG_CONTRACT dump-*) must parse here, field for field - a
   format change on either side fails one of the suites.
2. A synthetic recovery dump: last known position first, timeline decoded.
"""
import importlib.util
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location("flight_log_dump", ROOT / "tools" / "flight_log_dump.py")
fld = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fld)

FAILURES = []
CHECKED = 0


def check(cond, what):
    global CHECKED
    CHECKED += 1
    if not cond:
        FAILURES.append(what)


# 1. firmware contract
src = (ROOT / "transmitter" / "tests" / "test_flight_log.cpp").read_text()
pinned = dict(re.findall(r'LOG_CONTRACT dump-(\w+)[^\n]*\n\s*CHECK\(strstr\(Serial\.log, "([^"]+)\\n"\)', src))
check(set(pinned) == {"header", "imu", "gps", "launch", "event", "fused", "altimeter", "state"},
      f"pinned dump rows found: {sorted(pinned)}")
recs = fld.parse([pinned.get(k, "") for k in ("header", "imu", "gps", "launch", "event", "fused",
                                               "altimeter", "state")])
by_kind = {k: (ms, v) for ms, k, v in recs}
check(by_kind.get(1) == (1234, [100, -200, 300, 4000, -5000, 6000, 5000, 3]), f"IMU row {by_kind.get(1)}")
check(by_kind.get(2) == (1235, [398900000, -1048851712, 1655, 8, 1]), f"GPS row {by_kind.get(2)}")
check(by_kind.get(3) == (1236, []), f"LAUNCH row {by_kind.get(3)}")
check(by_kind.get(5) == (1237, [9, -550]), f"event row {by_kind.get(5)}")
check(by_kind.get(9) == (1238, [398812345, -1048987654, 1655, 123, -45, -2200, 7]), f"fused row {by_kind.get(9)}")
check(by_kind.get(10) == (0, [1, 0]), f"state row {by_kind.get(10)}")
check(by_kind.get(11) == (1239, [1, 6, 2510, 2510]), f"altimeter row {by_kind.get(11)}")
out = fld.summarise(recs)
check("GPS    39.8900000, -104.8851712  alt 1655 m  8 sats" in out, "GPS last position line")
check("fused  39.8812345, -104.8987654  alt 1655 m  (GPS fresh, dead reckoning, IMU ok)" in out,
      "fused last position line")
check("max IMU peak acceleration 5.00 g" in out, "IMU peak")
check("flight-state marker: FLYING" in out, "state marker")
check("ALTIMETER J5 (RRC3): drogue fired at 2510 ft" in out, "altimeter event in the timeline")
check("J5 (RRC3): apogee 2510 ft, last 2510 ft, fired drogue (1 records)" in out,
      "altimeter summary line")

# 2. a synthetic recovery dump, with serial chatter around it
DUMP = """[FLOG] previous flight data present
# kind 1 (IMU): ax,ay,az centi-g; gx,gy,gz 0.1 deg/s; peak milli-g;
ms,kind,ax,ay,az,gx,gy,gz,peak,stat_everything
900,8,32,0
1000,3,
1500,1,10,20,980,5,5,5,8200,3
1600,1,10,20,980,5,5,5,700,0
2000,6,
12000,2,398900000,-1048851712,2400,9,1
14000,5,1,0
15000,9,398901000,-1048852000,2350,120,-40,1500,7
60000,5,4,0
61000,2,398912345,-1048861234,1652,10,1
61500,4,
62000,7,
0,10,2,0
garbage line,,,
"""
recs = fld.parse(DUMP.splitlines())
out = fld.summarise(recs)
check(out.startswith("LAST KNOWN POSITION\n  GPS    39.8912345, -104.8861234  alt 1652 m  10 sats"),
      "last GPS fix leads the summary")
check("BOOT - reset cause: watchdog" in out, "boot cause decoded")
for word in ("LAUNCH", "POST_LAUNCH", "APOGEE", "LANDED", "BATTERY_SAVE"):
    check(f"  {word}" in out, f"timeline has {word}")
check("flight-state marker: LANDED - landing latched" in out, "LANDED marker")
check("max altitude 2400 m MSL (GPS)" in out, "max altitude")
check("max IMU peak acceleration 8.20 g" in out, "V3 IMU peak only")
check("1 IMU record(s) from pre-V3 firmware" in out, "pre-V3 IMU flagged")
check("RECORDS  IMU 2  GPS 2" in out, "record counts")
check(fld.parse(["", "noise", "1,99,1"]) == [], "unknown kinds and noise ignored")
import contextlib
import io
with contextlib.redirect_stdout(io.StringIO()):
    check(fld.main(["x"]) == 2, "usage without a file")

if FAILURES:
    for f in FAILURES:
        print(f"FAIL: {f}")
    print("\n--- last summary ---\n" + out)
    sys.exit(1)
print(f"flight_log_dump self-test: all {CHECKED} expectations passed")
