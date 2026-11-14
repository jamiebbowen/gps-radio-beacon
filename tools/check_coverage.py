#!/usr/bin/env python3
"""Enforce coverage floors on `make coverage` output.

Reads a suite's coverage output on stdin, prints one line per module and
exits 1 (naming the module and the floor) when any module drops below its
line or branch floor - so a coverage regression fails CI with a message
instead of scrolling by.

Usage:
  make -C transmitter/tests coverage | python3 tools/check_coverage.py tx
  make -C receiver/tests coverage    | python3 tools/check_coverage.py rx
"""
import re
import sys

# (default line floor, {module: (line floor, branch floor or None)})
FLOORS = {
    "tx": (99.0, {
        "firmware.ino": (98.0, 90.0),
        "flight_events.cpp": (99.0, 98.0),
    }),
    "rx": (98.0, {
        "display_modes/navigation_mode.c": (90.0, 80.0),
        "display_modes/sd_card_mode.c": (95.0, None),
        "display_modes/compass_mode.c": (95.0, None),
        "display_modes/rf_mode.c": (95.0, None),
        "bno085.c": (98.0, 85.0),
    }),
}


def parse(text):
    mods, cur = {}, None
    for line in text.splitlines():
        m = re.match(r"File '(?:\.\./firmware/(?:src/)?)(.+)'", line)
        if m:
            cur = m.group(1)
            mods.setdefault(cur, {})
            continue
        if cur is None:
            continue
        m = re.match(r"Lines executed:([\d.]+)% of (\d+)", line)
        if m:
            mods[cur]["lines"] = float(m.group(1))
        m = re.match(r"Taken at least once:([\d.]+)% of (\d+)", line)
        if m:
            mods[cur]["branches"] = float(m.group(1))
    return mods


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in FLOORS:
        print(__doc__)
        return 2
    default, special = FLOORS[sys.argv[1]]
    mods = parse(sys.stdin.read())
    if not mods:
        print("::error::no coverage data found in the input")
        return 1
    failed = 0
    for mod in sorted(mods):
        cov = mods[mod]
        lf, bf = special.get(mod, (default, None))
        lines, branches = cov.get("lines"), cov.get("branches")
        bad = []
        if lines is None or lines < lf:
            bad.append(f"lines {lines}% < {lf}%")
        if bf is not None and (branches is None or branches < bf):
            bad.append(f"branches {branches}% < {bf}%")
        print(f"  {'FAIL' if bad else 'ok  '} {mod:40s} lines {lines}%  branches {branches}%")
        if bad:
            failed += 1
            print(f"::error::coverage {mod}: " + ", ".join(bad))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
