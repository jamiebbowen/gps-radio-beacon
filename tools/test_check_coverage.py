#!/usr/bin/env python3
"""Regression test for tools/check_coverage.py, the CI coverage gate.

Feeds it synthetic `make coverage` output and pins what fails: a module
under its floor, a firmware source with no coverage data at all, and an
exemption gone stale. Run directly (`python3 tools/test_check_coverage.py`)
or via `make tools-test`. Exits non-zero on failure.

Why this exists: the gate only judged the modules that appeared in the
report, so a source with no test - or one whose COV_PAIRS entry was
dropped - passed CI unseen. A gate that can wave a module through is
worse than none: it is the reason nobody looks.
"""

import importlib.util
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("check_coverage",
                                              os.path.join(HERE, "check_coverage.py"))
cc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cc)

failures = 0


def check(cond, what):
    global failures
    if not cond:
        failures += 1
        print(f"  FAIL {what}")


def report(*mods):
    """gcov -b output for (path, lines%, branches%) the way the Makefiles print it."""
    out = []
    for path, lines, branches in mods:
        out += [f"File '../firmware/{path}'",
                f"Lines executed:{lines}% of 100",
                f"Branches executed:{branches}% of 10",
                f"Taken at least once:{branches}% of 10",
                "Calls executed:100.00% of 5",
                "Gcov '/tmp/none'"]
    return "\n".join(out) + "\n"


def run(suite, text):
    p = subprocess.run([sys.executable, os.path.join(HERE, "check_coverage.py"), suite],
                       input=text, capture_output=True, text=True)
    return p.returncode, p.stdout


# parse(): both firmware layouts, branch figures included
mods = cc.parse(report(("src/display_modes/rf_mode.c", "99.00", "92.11"),
                       ("nav.cpp", "100.00", "92.16")))
check(mods["display_modes/rf_mode.c"]["lines"] == 99.0, "rx path under src/ parsed")
check(mods["nav.cpp"]["branches"] == 92.16, "tx branch figure parsed")

# check_complete(): what a report must account for
srcs = {"a.c", "b.c", "main.c"}
out, n = cc.check_complete({"a.c": {}, "b.c": {}}, srcs, {"main.c": "why"})
check(n == 0 and any("exempt: why" in l for l in out), "exempt source skipped with its reason")
out, n = cc.check_complete({"a.c": {}}, srcs, {"main.c": "why"})
check(n == 1 and any("b.c" in l and "no coverage data" in l for l in out),
      "a source with no coverage data fails, named")
out, n = cc.check_complete({"a.c": {}, "b.c": {}, "main.c": {}}, srcs, {"main.c": "why"})
check(n == 1 and any("exempt but covered" in l for l in out), "a covered exemption is stale")
out, n = cc.check_complete({"a.c": {}, "b.c": {}, "main.c": {}}, srcs, {"gone.c": "why"})
check(n == 1 and any("not a firmware source" in l for l in out),
      "an exemption for a deleted source is stale")

# The real tables: every exemption names a real source, and the source
# scan sees the trees the Makefiles cover
for suite in ("tx", "rx"):
    real = cc.firmware_sources(suite)
    check(set(cc.EXEMPT[suite]) <= real, f"{suite}: EXEMPT names only real sources")
tx, rx = cc.firmware_sources("tx"), cc.firmware_sources("rx")
check("firmware.ino" in tx and "nmea_fields.c" in tx, "tx scan finds the sketch and C sources")
check(not any("/" in s for s in tx), "tx scan stays out of build/ and other subdirectories")
check("display_modes/navigation_mode.c" in rx and "lfs_sd_bd.c" in rx,
      "rx scan recurses into display_modes/")

# End to end: a full report passes; drop one source and the gate fails on it
full = report(*[(s, "100.00", "100.00") for s in sorted(tx)])
rc, out = run("tx", full)
check(rc == 0, f"complete tx report passes (rc={rc})\n{out}")
missing = sorted(tx)[0]
rc, out = run("tx", report(*[(s, "100.00", "100.00") for s in sorted(tx) if s != missing]))
check(rc == 1 and f"coverage {missing}: no coverage data" in out,
      f"tx report without {missing} fails (rc={rc})")
rc, out = run("tx", report(*[(s, "90.00" if s == missing else "100.00", "100.00")
                             for s in sorted(tx)]))
check(rc == 1 and f"coverage {missing}: lines 90.0% < " in out, "a module under its floor fails")

print("check_coverage: " + ("all checks passed" if not failures else f"{failures} FAILED"))
sys.exit(1 if failures else 0)
