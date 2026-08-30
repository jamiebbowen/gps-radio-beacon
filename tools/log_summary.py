#!/usr/bin/env python3
"""Summarize a gps-radio-beacon SD log (L*.TXT) into field-health indicators.

Usage:
    tools/log_summary.py L0001.TXT [L0002.TXT ...]
    tools/log_summary.py flight_data/2026-08-30_105629/     # whole session dir

What it reports (one section each, "OK" when nothing was found):

  Boot/reset      IWDG watchdog resets, SD self-test failures
  RF link         scan locks, channel hops, wedge recoveries, LOST windows,
                  noise alerts
  TX GPS health   heartbeat gps= state histogram, watchdog reset count (rst=)
  Telemetry       NAV row rate/gaps by source (raw GPS vs FUS), RSSI/SNR
                  range, fused dead-reckoning fraction
  Errors          every ERROR row

Row formats (see receiver/firmware/src/sd_card.c):
  <t>,EVENT,<msg>            <t>,ERROR,<msg>
  <t>,HEARTBEAT,id=.. CH.. sats=.. fix=.. up=..s rssi=.. gps=.. rst=..
  <t>,NAV,<GPS|FUS>,lat,lon,alt,sats,vn,ve,vd,age_ds,flags,base_lat,...,
      dist_km,bearing,heading,rssi,snr

Timestamps are seconds since RX boot ("%.3f"). Pure stdlib.
"""

import os
import re
import statistics
import sys

HEARTBEAT_RE = re.compile(
    r"HEARTBEAT id=(\d+) CH(\d+) sats=(\d+) fix=(\d+) up=(\d+)s "
    r"rssi=(-?\d+) gps=(\w+) rst=(\d+)"
)


def median_or_none(xs):
    return statistics.median(xs) if xs else None


def fmt_s(t):
    m, s = divmod(int(t), 60)
    h, m = divmod(m, 60)
    return f"{h}:{m:02d}:{s:02d}" if h else f"{m}:{s:02d}"


class Session:
    def __init__(self):
        self.t_first = None
        self.t_last = None
        self.rows = 0
        self.errors = []            # (t, msg)
        self.events = []            # (t, msg)
        self.wedges = []            # (t, n)
        self.lost = []              # t
        self.locks = []             # (t, msg)
        self.noise = []             # (t, msg)
        self.iwdg = []              # t
        self.sd_fail = []           # (t, msg)
        self.callsigns = []         # (t, msg)
        self.hb_states = {}         # gps state -> count
        self.hb_max_rst = 0
        self.hb_times = []          # t of each heartbeat
        self.nav = {"GPS": [], "FUS": []}   # (t, rssi, snr, flags_hex)
        self.nav_gaps = {"GPS": [], "FUS": []}

    def add_nav(self, t, src, rssi, snr, flags_hex):
        rows = self.nav.setdefault(src, [])
        if rows:
            self.nav_gaps[src].append(t - rows[-1][0])
        rows.append((t, rssi, snr, flags_hex))

    def add_event(self, t, msg):
        self.events.append((t, msg))
        if msg.startswith("RF wedge recovered"):
            self.wedges.append((t, msg))
        elif msg.startswith("RF LOST"):
            self.lost.append(t)
        elif msg.startswith("RF scan locked") or msg.startswith("RF manual"):
            self.locks.append((t, msg))
        elif msg.startswith("RF NOISE"):
            self.noise.append((t, msg))
        elif "IWDG watchdog reset" in msg:
            self.iwdg.append(t)
        elif msg.startswith("CALLSIGN"):
            self.callsigns.append((t, msg))
        elif "SD" in msg and ("fail" in msg.lower() or "FAIL" in msg):
            self.sd_fail.append((t, msg))

    def add_heartbeat(self, t, m):
        state = m.group(7)
        rst = int(m.group(8))
        self.hb_states[state] = self.hb_states.get(state, 0) + 1
        self.hb_max_rst = max(self.hb_max_rst, rst)
        self.hb_times.append(t)


def parse_file(path, sess):
    with open(path, "r", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("Timestamp,"):
                continue
            parts = line.split(",", 2)
            if len(parts) < 2:
                continue
            try:
                t = float(parts[0])
            except ValueError:
                continue
            sess.rows += 1
            sess.t_first = t if sess.t_first is None else min(sess.t_first, t)
            sess.t_last = t if sess.t_last is None else max(sess.t_last, t)

            kind = parts[1]
            if kind == "EVENT":
                sess.add_event(t, parts[2] if len(parts) > 2 else "")
            elif kind == "ERROR":
                sess.errors.append((t, parts[2] if len(parts) > 2 else ""))
            elif kind == "HEARTBEAT":
                m = HEARTBEAT_RE.search(line)
                if m:
                    sess.add_heartbeat(t, m)
            elif kind == "NAV":
                # t,NAV,src,lat,lon,alt,sats,vn,ve,vd,age_ds,flags,...,rssi,snr
                cols = line.split(",")
                if len(cols) >= 20:
                    src = cols[2]
                    try:
                        rssi = int(cols[18])
                        snr = int(cols[19])
                    except ValueError:
                        rssi = snr = 0
                    sess.add_nav(t, src, rssi, snr, cols[10])


def print_section(title, lines, empty="OK"):
    print(f"\n== {title} ==")
    if not lines:
        print(f"  {empty}")
    for line in lines:
        print(f"  {line}")


def report(sess, label):
    print(f"\n{'=' * 60}\n{label}\n{'=' * 60}")
    span = (sess.t_last - sess.t_first) if sess.t_first is not None else 0
    print(f"  {sess.rows} log rows, span {fmt_s(span)} "
          f"({sess.t_first:.1f}s -> {sess.t_last:.1f}s)")

    print_section("Boot / reset", (
        [f"IWDG watchdog reset logged at {fmt_s(t)}" for t in sess.iwdg]
        + [f"SD problem at {fmt_s(t)}: {m}" for t, m in sess.sd_fail]
    ), empty="OK - no watchdog resets, no SD failures")

    print_section("RF link", (
        [f"wedge recovery at {fmt_s(t)} ({m})" for t, m in sess.wedges]
        + [f"LOST (5 min silence) at {fmt_s(t)}" for t in sess.lost]
        + [f"noise alert at {fmt_s(t)}: {m}" for t, m in sess.noise]
        + [f"scan/channel at {fmt_s(t)}: {m}" for t, m in sess.locks]
    ), empty="OK - no wedge recoveries, no LOST windows, no noise alerts")

    if sess.callsigns:
        print_section("Callsigns heard",
                      [f"{fmt_s(t)}: {m}" for t, m in sess.callsigns])

    hb_lines = []
    if sess.hb_states:
        total = sum(sess.hb_states.values())
        hist = "  ".join(f"{k}={v}" for k, v in sorted(sess.hb_states.items()))
        hb_lines.append(f"{total} heartbeats: {hist}")
        hb_lines.append(f"TX GPS watchdog resets: max rst={sess.hb_max_rst}"
                        + ("  <-- TX had to cold-restart its GPS"
                           if sess.hb_max_rst else ""))
    print_section("TX GPS health (heartbeats)", hb_lines,
                  empty="no heartbeats (beacon had a fix the whole time)")

    telem = []
    for src, rows in sorted(sess.nav.items()):
        if not rows:
            continue
        rssis = [r for _, r, _, _ in rows]
        snrs = [s for _, _, s, _ in rows]
        gaps = sess.nav_gaps[src]
        med_gap = median_or_none(gaps)
        worst = max(gaps) if gaps else 0
        if med_gap is not None:
            gap_txt = f"median gap {med_gap:.2f}s, worst gap {worst:.1f}s"
        else:
            gap_txt = "single row"
        telem.append(
            f"{src}: {len(rows)} rows, {gap_txt}, "
            f"RSSI [{min(rssis)}..{max(rssis)}] dBm, "
            f"SNR [{min(snrs)}..{max(snrs)}] dB")
        if src == "FUS":
            dr = sum(1 for _, _, _, fl in rows
                     if fl.strip().upper().endswith("D"))
            if dr:
                telem.append(f"FUS: {dr} dead-reckoning rows "
                             f"({100.0 * dr / len(rows):.0f}%)")
    print_section("Telemetry", telem, empty="no NAV rows")

    print_section("Errors",
                  [f"{fmt_s(t)}: {m}" for t, m in sess.errors])


def main(argv):
    paths = []
    for arg in argv[1:]:
        if os.path.isdir(arg):
            paths += sorted(os.path.join(arg, f) for f in os.listdir(arg)
                            if f.upper().startswith("L") and f.upper().endswith(".TXT"))
        else:
            paths.append(arg)
    if not paths:
        print(__doc__)
        return 2

    per_file = len(paths) > 1
    combined = Session()
    for p in paths:
        if not os.path.exists(p):
            print(f"missing: {p}", file=sys.stderr)
            continue
        if per_file:
            sess = Session()
            parse_file(p, sess)
            report(sess, p)
        parse_file(p, combined)
    if per_file and len(paths) > 1:
        report(combined, "COMBINED (all files)")
    elif not per_file:
        report(combined, paths[0])
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
