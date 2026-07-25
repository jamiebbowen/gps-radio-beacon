#!/usr/bin/env python3
"""
calibrate_rf.py - Fit LoRa path-loss model from receiver SD card logs.

Reads one or more `L*.TXT` CSV logs produced by SD_Card_LogNavigation
(schema: Timestamp,Type,BeaconLat,BeaconLon,BeaconAlt_m,BeaconSats,
BaseLat,BaseLon,BaseAlt_m,Distance_km,Bearing_deg,Heading_deg,RSSI_dBm,SNR_dB)
and produces calibrated values for the two knobs in navigation_mode.c:

    #define LORA_PATH_LOSS_N        <fitted>
    #define LORA_SENSITIVITY_DBM    <observed_floor>

Usage:
    python3 calibrate_rf.py log1.TXT [log2.TXT ...]
    python3 calibrate_rf.py --plot log1.TXT            # also show RSSI-vs-distance plot
    python3 calibrate_rf.py --min-dist 20 log1.TXT     # drop samples closer than 20 m

Uses only the Python standard library. `--plot` needs matplotlib.
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path
from typing import Iterable


# Columns we actually need. The logger may grow more over time; we read
# defensively and validate only these.
REQUIRED_COLS = [
    "Type",
    "BeaconLat", "BeaconLon", "BeaconAlt_m",
    "BaseLat",   "BaseLon",   "BaseAlt_m",
    "Distance_km",
    "RSSI_dBm",  "SNR_dB",
]

# Floor below which a sample is treated as near-field garbage. RSSI saturates
# on the SX1268 at roughly -40 dBm and the log-distance model breaks down
# inside a few metres, so these samples would just distort the fit.
DEFAULT_MIN_DISTANCE_M = 10.0

# Samples whose RSSI looks like a rail/stuck value (receiver reported min int
# or obvious outliers from a corrupt header). Tune if you hit false positives.
RSSI_VALID_RANGE = (-140, -20)
SNR_VALID_RANGE  = (-30, 20)


def _safe_float(s: str) -> float:
    try:
        return float(s)
    except (TypeError, ValueError):
        return float("nan")


def load_logs(paths: list[Path]) -> list[dict]:
    """Concatenate one or more log files, keeping only NAV rows."""
    rows: list[dict] = []
    for p in paths:
        try:
            with p.open(newline="") as fh:
                reader = csv.DictReader(fh)
                missing = [c for c in REQUIRED_COLS if c not in (reader.fieldnames or [])]
                if missing:
                    print(f"warning: {p} missing columns {missing}, skipping",
                          file=sys.stderr)
                    continue
                for r in reader:
                    if r.get("Type") != "NAV":
                        continue
                    rows.append({
                        "BeaconLat":   _safe_float(r["BeaconLat"]),
                        "BeaconLon":   _safe_float(r["BeaconLon"]),
                        "BeaconAlt_m": _safe_float(r["BeaconAlt_m"]),
                        "BaseLat":     _safe_float(r["BaseLat"]),
                        "BaseLon":     _safe_float(r["BaseLon"]),
                        "BaseAlt_m":   _safe_float(r["BaseAlt_m"]),
                        "Distance_km": _safe_float(r["Distance_km"]),
                        "RSSI_dBm":    _safe_float(r["RSSI_dBm"]),
                        "SNR_dB":      _safe_float(r["SNR_dB"]),
                    })
        except OSError as e:
            print(f"warning: failed to read {p}: {e}", file=sys.stderr)
    if not rows:
        print("error: no usable NAV rows across the provided logs", file=sys.stderr)
        sys.exit(2)
    return rows


def slant_range_m(row: dict) -> float:
    """3D slant distance using the firmware's great-circle Distance_km and the
    altitude delta. Keeps the same distance definition the firmware used."""
    horiz_m = row["Distance_km"] * 1000.0
    dz = row["BeaconAlt_m"] - row["BaseAlt_m"]
    return math.sqrt(horiz_m * horiz_m + dz * dz)


def _in_range(x: float, lo: float, hi: float) -> bool:
    return (not math.isnan(x)) and (lo <= x <= hi)


def clean(rows: Iterable[dict], min_distance_m: float) -> list[dict]:
    """Drop rows that would bias the fit and attach slant_m."""
    kept: list[dict] = []
    dropped = 0
    for r in rows:
        r = dict(r)
        r["slant_m"] = slant_range_m(r)
        ok = (
            _in_range(r["slant_m"], min_distance_m, 1e7)
            and _in_range(r["RSSI_dBm"], *RSSI_VALID_RANGE)
            and _in_range(r["SNR_dB"],   *SNR_VALID_RANGE)
            and r["BeaconLat"] != 0.0 and r["BeaconLon"] != 0.0
            and r["BaseLat"]   != 0.0 and r["BaseLon"]   != 0.0
        )
        if ok:
            kept.append(r)
        else:
            dropped += 1
    if dropped:
        print(f"info: dropped {dropped} sample(s) as near-field / invalid")
    return kept


def fit_path_loss(rows: list[dict]) -> tuple[float, float, float]:
    """Least-squares fit of RSSI = intercept - 10 n * log10(slant_m).

    Returns (n, intercept_dbm, r_squared). Hand-rolled so we don't need numpy.
    """
    xs = [math.log10(r["slant_m"]) for r in rows]
    ys = [r["RSSI_dBm"] for r in rows]
    k = len(xs)
    mean_x = sum(xs) / k
    mean_y = sum(ys) / k
    sxx = sum((x - mean_x) ** 2 for x in xs)
    sxy = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
    if sxx <= 0.0:
        return float("nan"), float("nan"), float("nan")
    slope = sxy / sxx
    intercept = mean_y - slope * mean_x
    n = -slope / 10.0

    ss_tot = sum((y - mean_y) ** 2 for y in ys)
    ss_res = sum((y - (slope * x + intercept)) ** 2 for x, y in zip(xs, ys))
    r2 = 1.0 - (ss_res / ss_tot) if ss_tot > 0 else float("nan")
    return n, intercept, r2


def _percentile(values: list[float], pct: float) -> float:
    """Linear-interpolation percentile (same convention as numpy default)."""
    if not values:
        return float("nan")
    s = sorted(values)
    if len(s) == 1:
        return s[0]
    rank = (pct / 100.0) * (len(s) - 1)
    lo = int(math.floor(rank))
    hi = int(math.ceil(rank))
    if lo == hi:
        return s[lo]
    return s[lo] + (s[hi] - s[lo]) * (rank - lo)


def recommended_floor(rows: list[dict], percentile: float = 2.0) -> tuple[int, int]:
    """(observed_floor, recommended_floor_dbm) rounded down to integer dB.

    observed_floor   = lowest RSSI that still decoded.
    recommended_floor = Nth percentile across all samples; using a percentile
                        avoids anchoring on a single freak reception.
    """
    rssis = [r["RSSI_dBm"] for r in rows]
    observed = int(math.floor(min(rssis)))
    pct = int(math.floor(_percentile(rssis, percentile)))
    return observed, pct


def maybe_plot(rows: list[dict], n: float, intercept: float) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("warning: matplotlib not installed, skipping --plot", file=sys.stderr)
        return
    xs = [r["slant_m"] for r in rows]
    ys = [r["RSSI_dBm"] for r in rows]
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.scatter(xs, ys, s=6, alpha=0.5, label="samples")
    lo, hi = min(xs), max(xs)
    steps = 100
    log_lo, log_hi = math.log10(lo), math.log10(hi)
    line_x = [10 ** (log_lo + (log_hi - log_lo) * i / (steps - 1)) for i in range(steps)]
    line_y = [intercept - 10.0 * n * math.log10(x) for x in line_x]
    ax.plot(line_x, line_y, "r-",
            label=f"fit: n={n:.2f}, intercept={intercept:.1f} dBm")
    ax.set_xscale("log")
    ax.set_xlabel("Slant range (m)")
    ax.set_ylabel("RSSI (dBm)")
    ax.set_title("LoRa path-loss fit")
    ax.grid(True, which="both", alpha=0.3)
    ax.legend()
    plt.tight_layout()
    plt.show()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="+", type=Path, help="L*.TXT files from the SD card")
    ap.add_argument("--min-dist", type=float, default=DEFAULT_MIN_DISTANCE_M,
                    help=f"min slant range in metres to include (default {DEFAULT_MIN_DISTANCE_M})")
    ap.add_argument("--floor-pct", type=float, default=2.0,
                    help="percentile of RSSI used for recommended floor (default 2.0)")
    ap.add_argument("--plot", action="store_true",
                    help="plot RSSI vs log-distance and the fitted line")
    args = ap.parse_args()

    raw = load_logs(args.logs)
    print(f"loaded {len(raw)} NAV rows from {len(args.logs)} file(s)")

    d = clean(raw, args.min_dist)
    print(f"kept  {len(d)} rows after cleaning")

    if len(d) < 10:
        print("error: fewer than 10 usable samples - need more distance spread",
              file=sys.stderr)
        return 2

    n, intercept, r2 = fit_path_loss(d)
    observed_floor, recommended_floor_dbm = recommended_floor(d, args.floor_pct)

    slants = sorted(r["slant_m"] for r in d)
    rssis  = sorted(r["RSSI_dBm"] for r in d)
    median_slant = slants[len(slants) // 2]

    # Summary
    print()
    print("=== Path-loss fit ===")
    print(f"  path-loss exponent n   = {n:.2f}")
    print(f"  intercept (dBm at 1 m) = {intercept:.1f}")
    print(f"  R^2                    = {r2:.3f}")
    print()
    print("=== Sensitivity floor ===")
    print(f"  observed min RSSI      = {observed_floor} dBm")
    print(f"  recommended floor (P{args.floor_pct:g}) = {recommended_floor_dbm} dBm")
    print()
    print("=== Range distribution ===")
    print(f"  slant range: min {slants[0]:.0f} m, "
          f"median {median_slant:.0f} m, max {slants[-1]:.0f} m")
    print(f"  RSSI:        min {rssis[0]:.0f} dBm, max {rssis[-1]:.0f} dBm")

    # Suggested overrides, copy-paste-ready.
    print()
    print("=== Suggested firmware overrides ===")
    print("Add to receiver/firmware/Makefile CFLAGS, or edit the defines at")
    print("the top of receiver/firmware/src/display_modes/navigation_mode.c:")
    print()
    print(f"  -DLORA_PATH_LOSS_N={n:.2f}f")
    print(f"  -DLORA_SENSITIVITY_DBM={recommended_floor_dbm}.0f")

    if not (1.5 <= n <= 4.5):
        print()
        print(f"warning: fitted n={n:.2f} is outside the usual 1.8-4.0 range.")
        print("         check that BeaconAlt_m/BaseAlt_m are non-zero and that")
        print("         the log covers a wide spread of distances.")

    if args.plot:
        maybe_plot(d, n, intercept)

    return 0


if __name__ == "__main__":
    sys.exit(main())
