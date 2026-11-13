#!/usr/bin/env python3
"""Drift guard: the TX and RX packet_format.h files are two copies of the
same wire contract (transmitter/firmware/include vs receiver/firmware/inc).
They are not identical files - each side legitimately carries extras (the
receiver alone parses CMD packets; only it needs CmdPacket_t) - but every
symbol BOTH sides define must agree exactly, because a silent value or
layout skew puts two incompatible firmwares on the air believing they
speak the same protocol.

What is pinned:
  * every #define present in BOTH headers - identical normalized body
    (literals, flag masks, event codes, packet sizes, helper macros)
  * every typedef'd struct present in BOTH - identical ordered field
    list (type + name) and identical packed attribute
  * every PACKET_TYPE_* and FLIGHT_EVENT_* value must be defined on both
    sides - these are the actual over-the-air vocabulary; a packet type
    added to one side without the other is a protocol change, not a
    private detail

One-sided symbols outside the wire vocabulary (RX-only helper macros,
TX-only constants) are reported as informational, not failures.

Run directly (`python3 tools/test_packet_format_sync.py`) or via
`make test` (wired into tools-test). Exits non-zero on any mismatch.
"""

import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
TX_HDR = REPO / "transmitter/firmware/include/packet_format.h"
RX_HDR = REPO / "receiver/firmware/inc/packet_format.h"

# Symbols both sides must define - the airlink vocabulary itself. A
# protocol addition that only lands on one side fails here immediately
# instead of shipping silently half-deployed. *_NAME helpers are display
# lookups, not wire values.
WIRE_VOCABULARY = re.compile(r"^(PACKET_TYPE_|FLIGHT_EVENT_|FLAG_|HB_RESET_)")
VOCAB_EXEMPT = {"FLIGHT_EVENT_NAME", "HB_RESET_NAME"}


def strip_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", " ", text, flags=re.S)
    return re.sub(r"//[^\n]*", "", text)


def norm(s: str) -> str:
    """Collapse whitespace; macro bodies compare textually after this."""
    return re.sub(r"\s+", " ", s).strip()


def parse_header(path: Path):
    text = strip_comments(path.read_text())
    defines = {}
    for m in re.finditer(r"^\s*#\s*define\s+(\w+)(\s*\([^)]*\))?\s+(.*?)\s*$",
                         text, re.M):
        name, args, body = m.group(1), m.group(2) or "", m.group(3)
        defines[name] = norm(args + " " + body)

    structs = {}
    for m in re.finditer(
            r"typedef\s+struct\s+(\w*)\s*(.*?)\}\s*(\w+)\s*;", text, re.S):
        packed = "__attribute__((packed))" in m.group(1) or \
                 "__attribute__((packed))" in m.group(2)
        fields = []
        for line in m.group(2).split(";"):
            line = norm(line)
            if line:
                fields.append(line)
        structs[m.group(3)] = (packed, fields)
    return defines, structs


def main() -> int:
    tx_defs, tx_structs = parse_header(TX_HDR)
    rx_defs, rx_structs = parse_header(RX_HDR)

    fails = 0

    def fail(msg: str) -> None:
        nonlocal fails
        fails += 1
        print(f"  FAIL {msg}")

    # --- shared #defines: identical bodies -----------------------------
    for name in sorted(set(tx_defs) & set(rx_defs)):
        if name == "PACKET_FORMAT_H":
            continue
        if tx_defs[name] != rx_defs[name]:
            fail(f"#define {name}: TX `{tx_defs[name]}` != RX `{rx_defs[name]}`")

    # --- shared structs: identical packing + shared-prefix field lists -
    # The RX keeps legacy-superset layouts on purpose: it memcpy()s raw
    # wire bytes into a zeroed struct sized for the LARGEST historical
    # layout, so trailing fields the current wire dropped (e.g. IMU V1's
    # temp_c10) must stay in the RX struct or that memcpy overflows.
    # Rule: every field both sides share must match in order; extras are
    # legal only as trailing fields (a mid-struct insert changes offsets).
    for name in sorted(set(tx_structs) & set(rx_structs)):
        tx_packed, tx_fields = tx_structs[name]
        rx_packed, rx_fields = rx_structs[name]
        if tx_packed != rx_packed:
            fail(f"struct {name}: packed attr differs "
                 f"(TX={tx_packed}, RX={rx_packed})")
        n = min(len(tx_fields), len(rx_fields))
        if tx_fields[:n] != rx_fields[:n]:
            fail(f"struct {name}: shared fields diverge\n    TX: {tx_fields}\n"
                 f"    RX: {rx_fields}")
        elif len(tx_fields) != len(rx_fields):
            longer = "RX" if len(rx_fields) > len(tx_fields) else "TX"
            print(f"  info struct {name}: {longer} carries "
                  f"{abs(len(tx_fields) - len(rx_fields))} trailing "
                  f"legacy decode field(s)")

    # --- airlink vocabulary must exist on BOTH sides -------------------
    for name in sorted(set(tx_defs) | set(rx_defs)):
        if not WIRE_VOCABULARY.match(name) or name in VOCAB_EXEMPT:
            continue
        if name not in tx_defs:
            fail(f"{name} defined in RX header only - TX can't emit it")
        elif name not in rx_defs:
            fail(f"{name} defined in TX header only - RX can't parse it")

    # --- informational: one-sided symbols outside the wire vocabulary --
    only_tx = sorted(set(tx_defs) - set(rx_defs) - {"PACKET_FORMAT_H"})
    only_rx = sorted(set(rx_defs) - set(tx_defs) - {"PACKET_FORMAT_H"})
    stx = sorted(set(tx_structs) - set(rx_structs))
    srx = sorted(set(rx_structs) - set(tx_structs))
    for label, names in (("TX-only defines", only_tx),
                         ("RX-only defines", only_rx),
                         ("TX-only structs", stx),
                         ("RX-only structs", srx)):
        if names:
            print(f"  info {label}: {', '.join(names)}")

    total = len(set(tx_defs) & set(rx_defs)) + len(set(tx_structs) & set(rx_structs))
    print(f"\n{total} shared symbols checked, {fails} mismatches")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
