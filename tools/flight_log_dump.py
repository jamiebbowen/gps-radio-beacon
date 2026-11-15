#!/usr/bin/env python3
"""Summarise a transmitter flight-log DUMP.

The beacon keeps a crash-survivable flight log in its own flash; typing
DUMP + Enter in a USB serial monitor prints it as CSV. Save that output
to a file (extra serial chatter around it is ignored) and run:

    python3 tools/flight_log_dump.py capture.txt

The LAST KNOWN POSITION comes first - it is what a recovery crew needs
when the radio link and the receiver's SD log both came up empty - then
the timeline (boot causes, launch, flight events, state changes,
landing), the maxima and the record counts.

Row format (transmitter/firmware/flight_log.cpp, flog_print_record):
    ms,kind,...   ms since boot; kind 1 IMU, 2 GPS, 3 LAUNCH, 4 LANDED,
    5 FLIGHT_EVENT, 6 POST_LAUNCH, 7 BATTERY_SAVE, 8 BOOT, 9 FUSED,
    10 STATE (flight-state marker: 1 FLYING, 2 LANDED),
    11 ALTIMETER (input 0=J2 1=J5 2=J6, status ALT_ST_*, alt ft, apogee ft)
"""
import sys

KIND_NAMES = {1: "IMU", 2: "GPS", 3: "LAUNCH", 4: "LANDED", 5: "FLIGHT_EVENT",
              6: "POST_LAUNCH", 7: "BATTERY_SAVE", 8: "BOOT", 9: "FUSED",
              10: "STATE", 11: "ALTIMETER"}
EVENT_NAMES = {0x01: "APOGEE", 0x02: "DROGUE", 0x03: "MAIN", 0x04: "LANDED",
               0x10: "ANOMALY ballistic (no drogue)", 0x11: "ANOMALY tumble",
               0x12: "ANOMALY IMU lost", 0x13: "ANOMALY GPS outage",
               0x14: "ANOMALY reboot in flight"}
STATE_NAMES = {1: "FLYING", 2: "LANDED"}
# SAMD51 RSTC.RCAUSE bits (packet_format.h HB_RESET_*), most telling first
RESET_BITS = [(0x20, "watchdog"), (0x02, "brown-out (core)"),
              (0x04, "brown-out (3.3 V)"), (0x10, "reset pin"),
              (0x01, "power-on"), (0x40, "system reset"), (0x08, "NVM"),
              (0x80, "backup")]
FUS_FLAGS = [(0x01, "GPS fresh"), (0x02, "dead reckoning"), (0x04, "IMU ok")]
# Commercial altimeter inputs (packet_format.h ALT_ST_*)
ALT_JACKS = {0: "J2", 1: "J5", 2: "J6"}
ALT_FORMATS = {1: "StratoLogger CF", 2: "RRC3", 3: "Eggtimer"}
ALT_EVENTS = [(0x04, "drogue"), (0x08, "main"), (0x10, "aux")]


def alt_name(vals):
    return f"{ALT_JACKS.get(vals[0], '?')} ({ALT_FORMATS.get(vals[1] & 0x03, '?')})"


def parse(lines):
    """[(ms, kind, [int fields])] for every well-formed record line."""
    recs = []
    for line in lines:
        line = line.strip()
        if not line or line.startswith("#") or line.startswith("ms,kind"):
            continue
        f = line.split(",")
        try:
            ms, kind = int(f[0]), int(f[1])
            vals = [int(x) for x in f[2:] if x != ""]
        except (ValueError, IndexError):
            continue                       # serial chatter around the dump
        if kind in KIND_NAMES:
            recs.append((ms, kind, vals))
    return recs


def reset_cause(bits):
    names = [n for b, n in RESET_BITS if bits & b]
    return ", ".join(names) if names else f"unknown (0x{bits:02X})"


def t_s(ms):
    return f"t={ms / 1000.0:.3f} s"


def summarise(recs):
    out = []
    gps = [r for r in recs if r[1] == 2 and len(r[2]) >= 5]
    fus = [r for r in recs if r[1] == 9 and len(r[2]) >= 7]

    out.append("LAST KNOWN POSITION")
    if gps:
        ms = gps[-1][0]
        lat, lon, alt, sats, fixq = gps[-1][2][:5]
        out.append(f"  GPS    {lat / 1e7:.7f}, {lon / 1e7:.7f}  alt {alt} m  "
                   f"{sats} sats  fix {fixq}  {t_s(ms)}")
    if fus:
        ms, vals = fus[-1][0], fus[-1][2]
        flags = ", ".join(n for b, n in FUS_FLAGS if vals[6] & b) or "none"
        out.append(f"  fused  {vals[0] / 1e7:.7f}, {vals[1] / 1e7:.7f}  alt {vals[2]} m  "
                   f"({flags})  {t_s(ms)}")
    if not gps and not fus:
        out.append("  none - no GPS or fused record in this log")

    out.append("")
    out.append("TIMELINE")
    state = None
    alt_fired = {}
    for ms, kind, vals in recs:
        if kind == 8:
            cause = reset_cause(vals[0]) if vals else "?"
            out.append(f"  {t_s(ms)}  BOOT - reset cause: {cause}")
        elif kind in (3, 4, 6, 7):
            out.append(f"  {t_s(ms)}  {KIND_NAMES[kind]}")
        elif kind == 5:
            code = vals[0] if vals else 0
            name = EVENT_NAMES.get(code, f"event 0x{code:02X}")
            extra = f" (value {vals[1]})" if len(vals) > 1 and vals[1] else ""
            out.append(f"  {t_s(ms)}  {name}{extra}")
        elif kind == 10 and vals:
            state = STATE_NAMES.get(vals[0], f"0x{vals[0]:02X}")
        elif kind == 11 and len(vals) >= 4:
            new = vals[1] & ~alt_fired.get(vals[0], 0)
            alt_fired[vals[0]] = alt_fired.get(vals[0], 0) | vals[1]
            for bit, name in ALT_EVENTS:
                if new & bit:
                    out.append(f"  {t_s(ms)}  ALTIMETER {alt_name(vals)}: {name} fired"
                               f" at {vals[2]} ft")
    if state:
        note = {"FLYING": " - the log was still armed: landing never latched "
                          "(lost, stuck, or still in the air when powered off)",
                "LANDED": " - landing latched"}.get(state, "")
        out.append(f"  flight-state marker: {state}{note}")

    out.append("")
    out.append("MAXIMA")
    alts = [(r[2][2], "GPS") for r in gps] + [(r[2][2], "fused") for r in fus]
    if alts:
        alt, src = max(alts)
        out.append(f"  max altitude {alt} m MSL ({src})")
    if fus:
        v = max((vn * vn + ve * ve + vd * vd) ** 0.5 / 100.0
                for vn, ve, vd in (r[2][3:6] for r in fus))
        out.append(f"  max fused speed {v:.1f} m/s")
    imu = [r for r in recs if r[1] == 1 and len(r[2]) >= 7]
    v3 = [r for r in imu if len(r[2]) >= 8 and r[2][7] == 3]
    if v3:
        out.append(f"  max IMU peak acceleration {max(r[2][6] for r in v3) / 1000.0:.2f} g")
    if len(v3) < len(imu):
        out.append(f"  {len(imu) - len(v3)} IMU record(s) from pre-V3 firmware "
                   "(cm/s^2 units that may have wrapped) - not used above")

    alt = [r for r in recs if r[1] == 11 and len(r[2]) >= 4]
    if alt:
        out.append("")
        out.append("ALTIMETERS")
        for jack in sorted({r[2][0] for r in alt}):
            mine = [r[2] for r in alt if r[2][0] == jack]
            last = mine[-1]
            fired = ", ".join(n for b, n in ALT_EVENTS if last[1] & b) or "nothing"
            out.append(f"  {alt_name(last)}: apogee {max(v[3] for v in mine)} ft, "
                       f"last {last[2]} ft, fired {fired} ({len(mine)} records)")

    out.append("")
    counts = {}
    for _, kind, _ in recs:
        counts[kind] = counts.get(kind, 0) + 1
    out.append("RECORDS  " + "  ".join(f"{KIND_NAMES[k]} {n}" for k, n in sorted(counts.items()))
               if counts else "RECORDS  none")
    return "\n".join(out) + "\n"


def main(argv):
    if len(argv) != 2:
        print(__doc__)
        return 2
    with open(argv[1], errors="ignore") as f:
        recs = parse(f)
    if not recs:
        print("no flight-log records found (was DUMP's output captured?)")
        return 1
    sys.stdout.write(summarise(recs))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
