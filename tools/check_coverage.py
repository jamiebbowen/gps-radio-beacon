#!/usr/bin/env python3
"""Enforce coverage floors on `make coverage` output.

Reads a suite's coverage output on stdin, prints one line per module and
exits 1 (naming the module and the floor) when any module drops below its
line or branch floor - so a coverage regression fails CI with a message
instead of scrolling by.

Every firmware source must report: one with no coverage data (a new
module without a test, a COV_PAIRS entry dropped from a tests Makefile)
fails too, unless EXEMPT names it with the reason. An exemption for a
source that now has coverage, or no longer exists, fails as stale.

Usage:
  make -C transmitter/tests coverage | python3 tools/check_coverage.py tx
  make -C receiver/tests coverage    | python3 tools/check_coverage.py rx
"""
import os
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

# (default line floor, {module: (line floor, branch floor or None)})
FLOORS = {
    "tx": (99.0, {
        "firmware.ino": (98.0, 90.0),
        "flight_events.cpp": (99.0, 98.0),
    }),
    "rx": (98.0, {
        # The uncovered lines are defensive littlefs-failure branches
        # (mount after a successful format failing, self-test readback
        # mismatch) plus, on gcc 15, the first line of a two-line ternary
        # that gcc 13 attributes to its taken arm. 97.5% keeps ~2 lines
        # of compiler-attribution margin.
        "sd_card.c": (97.5, None),
        "display_modes/navigation_mode.c": (90.0, 80.0),
        "display_modes/sd_card_mode.c": (95.0, None),
        "display_modes/compass_mode.c": (95.0, None),
        "display_modes/rf_mode.c": (95.0, None),
        "bno085.c": (98.0, 85.0),
        # test_main runs the whole boot and loop; a dropped scenario shows
        # up here first (100% / 80% on gcc 13)
        "main.c": (99.0, 78.0),
    }),
}

# Where each suite's firmware sources live, named the way `make coverage`
# reports them (relative to that root).
SOURCES = {
    "tx": (REPO / "transmitter/firmware", ("*.cpp", "*.c", "*.ino")),
    "rx": (REPO / "receiver/firmware/src", ("**/*.c",)),
}

# The only sources allowed no host coverage, each with its reason.
EXEMPT = {
    "tx": {},
    "rx": {
        "syscalls.c": "newlib syscall stubs over the linker's heap symbols",
    },
}


def firmware_sources(suite):
    root, patterns = SOURCES[suite]
    return {p.relative_to(root).as_posix()
            for pat in patterns for p in root.glob(pat) if p.is_file()}


def parse(text):
    mods, cur, pending = {}, None, []
    for line in text.splitlines():
        m = re.match(r"Gcov '(.+)'", line)
        if m:                      # per-test annotated .gcov files live here
            for mod in pending:
                mods[mod].setdefault("gcov_dir", m.group(1))
            pending = []
            continue
        m = re.match(r"File '(?:\.\./firmware/(?:src/)?)(.+)'", line)
        if m:
            cur = m.group(1)
            mods.setdefault(cur, {})
            pending.append(cur)
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


def print_uncovered(mod, gcov_dir):
    """List the never-executed lines of a failing module."""
    if not gcov_dir:
        return
    path = os.path.join(gcov_dir, os.path.basename(mod) + ".gcov")
    if not os.path.exists(path):
        return
    for line in open(path, errors="ignore"):
        m = re.match(r"\s*#####:\s*(\d+):(.*)", line)
        if m:
            print(f"      uncovered {mod}:{m.group(1)}: {m.group(2).strip()[:90]}")


def check_complete(mods, sources, exempt):
    """Sources missing from the report, and stale exemptions: (lines, count)."""
    out, failed = [], 0
    for mod in sorted(sources - set(mods) - set(exempt)):
        failed += 1
        out.append(f"  FAIL {mod:40s} no coverage data")
        out.append(f"::error::coverage {mod}: no coverage data - give it a test "
                   "and a COV_PAIRS entry, or an EXEMPT entry with the reason")
    for mod in sorted(exempt):
        if mod in mods:
            failed += 1
            out.append(f"::error::coverage {mod}: exempt but covered - drop its EXEMPT entry")
        elif mod not in sources:
            failed += 1
            out.append(f"::error::coverage {mod}: exempt but not a firmware source "
                       "- drop its EXEMPT entry")
        else:
            out.append(f"  skip {mod:40s} exempt: {exempt[mod]}")
    return out, failed


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in FLOORS:
        print(__doc__)
        return 2
    suite = sys.argv[1]
    default, special = FLOORS[suite]
    mods = parse(sys.stdin.read())
    if not mods:
        print("::error::no coverage data found in the input")
        return 1
    lines, failed = check_complete(mods, firmware_sources(suite), EXEMPT[suite])
    for line in lines:
        print(line)
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
            print_uncovered(mod, cov.get("gcov_dir"))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
