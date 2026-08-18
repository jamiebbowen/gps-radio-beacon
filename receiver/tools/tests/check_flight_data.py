#!/usr/bin/env python3
"""check_flight_data.py - Log-format drift detector.

Runs against whatever real logs are in flight_data/ - searched recursively,
since `make extract` puts each extraction in its own timestamped
subdirectory: every ",NAV," line in every log must be parseable by
analyze_flight.load_nav_rows. If the firmware's NAV CSV columns and the
host tools drift apart, rows silently fail to parse and this catches it
without hard-coding expectations about any particular flight.

Exit 0 = all NAV lines parsed and at least one NAV row exists overall.
"""

import glob
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from analyze_flight import load_nav_rows  # noqa: E402


def main():
    logs = sorted(glob.glob("flight_data/**/L*.TXT", recursive=True))
    if not logs:
        print("check_flight_data: no logs found (nothing to check)")
        return 0

    total = 0
    for path in logs:
        with open(path, newline="") as f:
            nav_lines = sum(1 for line in f if ",NAV," in line)
        rows = load_nav_rows(path)
        if len(rows) != nav_lines:
            print(f"FAIL: {path}: {nav_lines} NAV lines in file but parser "
                  f"returned {len(rows)} rows - log format drift?")
            return 1
        print(f"  {path}: {len(rows)} NAV rows parsed OK")
        total += len(rows)

    if total == 0:
        print("FAIL: logs present but zero NAV rows parsed from any of them")
        return 1

    print(f"check_flight_data: OK ({total} NAV rows across {len(logs)} logs)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
