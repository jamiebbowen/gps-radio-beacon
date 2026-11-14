#!/usr/bin/env python3
"""Reachability scan: find project functions no firmware code can call.

The linker map's "Discarded input sections" lists every function that
--gc-sections dropped (-ffunction-sections puts each function in its own
section). A dropped function is either genuinely dead, or live code the
compiler inlined into a caller in the same file. This scan tells them apart
by looking for references outside the function's own definition:

  DEAD           no reference anywhere in the firmware sources
  inlined-in-TU  referenced only from its own file (live, just inlined)

Exit status is 1 when any DEAD function is found.

Usage (repo root, after building both firmwares):
  make -C receiver/firmware compile
  arduino-cli compile --fqbn adafruit:samd:adafruit_itsybitsy_m4 \\
      --build-path /tmp/tx-build transmitter/firmware
  python3 tools/reachability.py --tx-map /tmp/tx-build/firmware.ino.map
"""
import argparse
import pathlib
import re
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent


def discarded(mapfile, projmods):
    txt = pathlib.Path(mapfile).read_text(errors="ignore")
    seg = txt.split("Discarded input sections", 1)[1]
    seg = seg.split("Memory Configuration", 1)[0].splitlines()
    out, pending = [], None
    for line in seg:
        m = re.match(r"^ \.text\.(\S+)(?:\s+0x[0-9a-f]+\s+0x[0-9a-f]+\s+(\S+))?", line)
        if m:
            name, obj = m.group(1), m.group(2)
            if obj is None:          # long name: object path on the next line
                pending = name
                continue
            pending = None
        else:
            m2 = re.match(r"^\s+0x[0-9a-f]+\s+0x[0-9a-f]+\s+(\S+)", line)
            if not (pending and m2):
                continue
            name, obj, pending = pending, m2.group(1), None
        mod = pathlib.Path(obj).name.split(".")[0]
        if mod in projmods:
            out.append((mod, name))
    return sorted(set(out))


def demangle(name):
    if not name.startswith("_Z"):
        return name
    r = subprocess.run(["c++filt", name], capture_output=True, text=True)
    return r.stdout.strip().split("(")[0]


def analyze(label, mapfile, globs, mods, show_all):
    texts = {}
    for g in globs:
        for p in ROOT.glob(g):
            texts[p] = p.read_text(errors="ignore")
    print("==", label)
    dead = 0
    for mod, raw in discarded(mapfile, mods):
        name = demangle(raw)
        callers = []
        for p, t in texts.items():
            for line in t.splitlines():
                if not re.search(r"\b" + re.escape(name) + r"\b", line):
                    continue
                st = line.strip()
                is_def = (re.match(r"^(static\s+|inline\s+|const\s+)*[A-Za-z_][\w]*[\s\*]+"
                                   + re.escape(name) + r"\s*\(", st)
                          and not st.startswith("return"))
                if not is_def:
                    callers.append(p.stem)
        if not callers:
            verdict = "DEAD"
            dead += 1
        elif all(c == mod for c in callers):
            verdict = "inlined-in-TU"
        else:
            verdict = "used"
        if verdict != "used" or show_all:
            print(f"  {verdict:14s} {mod}:{name}")
    return dead


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rx-map", default=str(ROOT / "receiver/firmware/build/gps_radio_beacon_receiver.map"))
    ap.add_argument("--tx-map", default="/tmp/tx-build/firmware.ino.map")
    ap.add_argument("--all", action="store_true", help="also list used functions")
    args = ap.parse_args()

    dead = 0
    if pathlib.Path(args.rx_map).exists():
        dead += analyze("RX", args.rx_map,
                        ["receiver/firmware/src/*.c", "receiver/firmware/src/display_modes/*.c"],
                        {p.stem for p in (ROOT / "receiver/firmware/src").rglob("*.c")},
                        args.all)
    else:
        print(f"== RX: no map at {args.rx_map} (build the receiver first)")
    if pathlib.Path(args.tx_map).exists():
        dead += analyze("TX", args.tx_map,
                        ["transmitter/firmware/*.cpp", "transmitter/firmware/*.c",
                         "transmitter/firmware/*.ino"],
                        {p.stem for p in (ROOT / "transmitter/firmware").glob("*.c*")} | {"firmware"},
                        args.all)
    else:
        print(f"== TX: no map at {args.tx_map} (build the transmitter first)")
    return 1 if dead else 0


if __name__ == "__main__":
    sys.exit(main())
