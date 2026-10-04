#!/usr/bin/env python3
"""
heading_residuals.py — Arrow-bias diagnostic.

Reads a NAV log from a walk-around-a-fixed-target procedure and reports the
systematic offset between the computed bearing to the target and the compass
heading actually displayed while facing it.

Usage:
    python3 heading_residuals.py flight_data/YYYY-MM-DD_HHMMSS/L000N.TXT

The NAV log's Bearing_deg column is the great-circle bearing from the
receiver position to the beacon's last-known position; Heading_deg is the
BNO055-derived heading at the same instant. When you stand between the
receiver and the target, the arrow reads Heading-Bearing alignment; over a
controlled walk the median of (Heading - Bearing), wrapped to [-180,180],
is the mounting+declination error you can correct by.

See docs/arrow-bias-test.md for the field procedure.
"""

import csv
import math
import statistics
import sys


def wrap180(deg: float) -> float:
    while deg > 180.0:
        deg -= 360.0
    while deg < -180.0:
        deg += 360.0
    return deg


def main(argv):
    if len(argv) != 2:
        print(__doc__)
        return 2
    path = argv[1]
    residuals = []
    dist_m = []
    with open(path) as f:
        r = csv.DictReader(f)
        for row in r:
            if row.get('Type') != 'NAV':
                continue
            try:
                bearing = float(row['Bearing_deg'])
                heading = float(row['Heading_deg'])
                d_km = float(row['Distance_km'])
            except (KeyError, ValueError):
                continue
            if heading < 0 or bearing < 0:
                continue
            residuals.append(wrap180(heading - bearing))
            dist_m.append(d_km * 1000.0)

    if not residuals:
        print("No NAV rows with heading+bearing found")
        return 1

    med = statistics.median(residuals)
    mean = statistics.fmean(residuals)
    sd = statistics.stdev(residuals) if len(residuals) > 1 else 0.0
    print(f"Samples:         {len(residuals)}")
    print(f"Distances:       {min(dist_m):.0f}..{max(dist_m):.0f} m")
    print(f"Median residual: {med:+.1f} deg   (arrow error vs truth)")
    print(f"Mean residual:   {mean:+.1f} deg")
    print(f"Std dev:         {sd:.1f} deg")
    print()
    print("Interpretation:")
    print("  Median near 0      -> arrow is unbiased; scatter is local.")
    print("  Finite median      -> persistent bias (mounting offset /")
    print("                        miscalibration / decl misfire); the")
    print("                        number is your knob target.")
    if sd > 10:
        print("  Note: std > 10 deg — scatter is big; check pitch/roll held")
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
