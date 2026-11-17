#!/usr/bin/env python3
"""Regression test for tools/mutate.py's mutant generator.

Pins what it mutates and what it must leave alone: operators in code
only (never in comments, strings, char literals or preprocessor lines),
the swaps, integer constants (0 <-> 1, n -> n + 1; not hex, floats,
identifiers or a declaration's array size), call-statement removal (not
returns or conditions), and the COV_PAIRS map both tests Makefiles give
it. Run directly (`python3 tools/test_mutate.py`) or via `make tools-test`.

Why this exists: a generator that silently stops producing mutants
reports a perfect score.
"""
import importlib.util
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("mutate", os.path.join(HERE, "mutate.py"))
mutate = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mutate)

failures = 0


def check(cond, what):
    global failures
    if not cond:
        failures += 1
        print(f"  FAIL {what}")


def descs(src):
    return [d for _, d, _ in mutate.mutants(src)]


SRC = """#include <stdio.h>
#define LIMIT 100   /* a < b */
// x && y
static const char *s = "a == b || c";
static char buf[24];
int f(int a, int b, int *p)
{
    if (a < b && b != 0x10) return 1;
    p->x = a << 2;
    a++;
    log_row(buf, 'x');
    double d = 1.5;
    while (a >= 7) a--;
    return g(a);
}
"""

d = descs(SRC)
check(d.count("< -> <=") == 1, f"one '<' (code only): {d}")
check("&& -> ||" in d and "!= -> ==" in d and ">= -> >" in d, "the swaps")
check(not any("<<" in x or "->" == x[:2] or "++" in x for x in d), "<<, ->, ++ left alone")
check("1 -> 0" in d and "7 -> 8" in d and "2 -> 3" in d, "constants: 1 -> 0, n -> n + 1")
check(not any(x.startswith(("100 ", "16 ", "24 ", "5 ")) for x in d),
      "no #define, hex, array-size or float-fraction constants")
check("remove log_row()" in d, "a call statement is removable")
check(not any(x.startswith("remove g") or x.startswith("remove return") for x in d),
      "returns are not removed")
check(len([x for x in d if x.startswith("remove")]) == 1, "exactly one removable call")

# Every mutant changes the source and keeps its line count
for line, desc, new in mutate.mutants(SRC):
    check(new != SRC and new.count("\n") == SRC.count("\n"), f"line {line}: {desc} keeps lines")

check(mutate.mutants("/* < */ \"<\" '<'\n#if A < B\n#endif\n") == [], "nothing outside code")
check(descs("x = 1UL;\n") == ["1UL -> 0UL"], "integer suffixes kept")

# The COV_PAIRS map, as each tests Makefile defines it
rx = mutate.cov_pairs(os.path.join(mutate.REPO, "receiver/tests"))
tx = mutate.cov_pairs(os.path.join(mutate.REPO, "transmitter/tests"))
check(rx.get("sd_card.c") == "sd_card" and rx.get("bno085.c") == "compass"
      and rx.get("display_modes/rf_mode.c") == "display_modes" and rx.get("main.c") == "main",
      f"rx pairs: {sorted(rx.items())[:6]}")
check(tx.get("firmware.ino") == "mainloop" and tx.get("altimeter_input.cpp") == "altimeter",
      f"tx pairs: {sorted(tx.items())[:6]}")

print("mutate: " + ("all checks passed" if not failures else f"{failures} FAILED"))
sys.exit(1 if failures else 0)
