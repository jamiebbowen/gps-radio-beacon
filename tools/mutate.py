#!/usr/bin/env python3
"""Mutation testing for the firmware host tests: how many deliberate bugs
does a module's own test catch?

Line coverage says a test ran a line, not that it would notice the line
being wrong. This applies small, realistic bugs to one firmware source -
a flipped comparison (< for <=, == for !=), && for ||, an integer constant
changed (0 <-> 1, n -> n + 1), a call statement removed - one at a time, rebuilds the
module's test binary and runs it. A mutant the test still passes is a gap:
a check that is missing, or a change that cannot matter (an equivalent
mutant - read it before writing a test for it).

Usage:
  python3 tools/mutate.py rx sd_card.c
  python3 tools/mutate.py tx flight_events.cpp --jobs 4
  python3 tools/mutate.py rx main.c --lines 690-720 --list
  python3 tools/mutate.py rx display_modes/rf_mode.c --max 40 --seed 7

The module is named as the coverage report names it (relative to
receiver/firmware/src or transmitter/firmware); its test is the one its
tests Makefile pairs it with in COV_PAIRS (--test overrides). Everything
runs in scratch copies of the tree - the working tree's files, uncommitted
edits included - so no mutant can be left behind in your sources. Not part
of CI: every mutant rebuilds a test binary.
"""
import argparse
import os
import random
import re
import shutil
import subprocess
import sys
import tempfile
import time
from concurrent.futures import ThreadPoolExecutor

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SUITES = {
    "rx": ("receiver/tests", "receiver/firmware/src"),
    "tx": ("transmitter/tests", "transmitter/firmware"),
}
# What the host tests build from (both suites, plus the SD tools test_lfs_sd_bd links)
COPY_PREFIXES = ("receiver/firmware/src/", "receiver/firmware/inc/",
                 "receiver/firmware/lib/littlefs/", "receiver/tests/",
                 "transmitter/firmware/", "transmitter/tests/", "tools/lfs_extract/")

SWAP = {"<": "<=", "<=": "<", ">": ">=", ">=": ">", "==": "!=", "!=": "==",
        "&&": "||", "||": "&&"}
TOKEN = re.compile(r"<<=?|>>=?|->|<=|>=|==|!=|&&|\|\||\+\+|--|[<>]"
                   r"|(?<![\w.])(?:0[xX][0-9a-fA-F]+|\d+)(?:[uU]?[lL]{0,2}|[lL]{1,2}[uU])(?![\w.])")
CALL_STMT = re.compile(r"^\s*([A-Za-z_]\w*(?:(?:\.|->)\w+)*)\s*\(.*\)\s*;\s*$")
# An array size in a declaration: a bigger buffer cannot fail a test
ARRAY_DECL = re.compile(r"[A-Za-z_]\w*[\s*]+[A-Za-z_]\w*\s*\[\s*$")
NOT_CALLS = ("return", "if", "for", "while", "switch", "sizeof", "case", "else", "do")


def code_mask(src):
    """Per character: True where it is code (not a comment, string, char
    literal or preprocessor line)."""
    mask = [True] * len(src)
    i, n, line_start = 0, len(src), True
    while i < n:
        c = src[i]
        if line_start and c in " \t":
            i += 1
            continue
        if line_start and c == "#":               # directive, with continuations
            j = i
            while j < n and not (src[j] == "\n" and src[j - 1] != "\\"):
                j += 1
            for k in range(i, j):
                mask[k] = False
            i, line_start = j, False
            continue
        line_start = (c == "\n")
        if src.startswith("//", i):
            j = src.find("\n", i)
            j = n if j < 0 else j
        elif src.startswith("/*", i):
            j = src.find("*/", i + 2)
            j = n if j < 0 else j + 2
        elif c in "\"'":
            j = i + 1
            while j < n and src[j] != c:
                j += 2 if src[j] == "\\" else 1
            j += 1
        else:
            i += 1
            continue
        for k in range(i, min(j, n)):
            mask[k] = False
        i = j
    return mask


def mutants(src):
    """(line, description, mutated source) for every mutation of src."""
    mask = code_mask(src)
    out = []
    for m in TOKEN.finditer(src):
        if not all(mask[m.start():m.end()]):
            continue
        tok = m.group()
        if tok in SWAP:
            new = SWAP[tok]
        elif tok[0].isdigit() and not tok.lower().startswith("0x"):
            line_head = src[src.rfind("\n", 0, m.start()) + 1:m.start()]
            if ARRAY_DECL.search(line_head) and src[m.end():m.end() + 1] == "]":
                continue
            digits = re.match(r"\d+", tok).group()
            v = int(digits)                # 0 <-> 1 (flags, returns), else n + 1
            new = str(1 if v == 0 else 0 if v == 1 else v + 1) + tok[len(digits):]
        else:
            continue
        line = src.count("\n", 0, m.start()) + 1
        out.append((line, f"{tok} -> {new}", src[:m.start()] + new + src[m.end():]))
    pos = 0
    for line, text in enumerate(src.split("\n"), 1):
        code = "".join(ch for ch, keep in zip(text, mask[pos:pos + len(text)]) if keep)
        call = CALL_STMT.match(code)
        if call and call.group(1) not in NOT_CALLS:
            out.append((line, f"remove {call.group(1)}()",
                        src[:pos] + re.sub(r"\S.*", ";", text, count=1) + src[pos + len(text):]))
        pos += len(text) + 1
    return sorted(out, key=lambda t: t[0])


def cov_pairs(tests_dir):
    """{module source: test name} from a tests Makefile's COV_PAIRS."""
    r = subprocess.run(["make", "-s", "--no-print-directory", "-C", tests_dir, "-f", "Makefile",
                        "-f", "-", "mutate-print-pairs"],
                       input="mutate-print-pairs:\n\t@echo $(COV_PAIRS)\n",
                       capture_output=True, text=True, check=True)
    pairs = {}
    for p in r.stdout.split():
        test, src = p.split(":", 1)
        pairs.setdefault(src, test)
    return pairs


def scratch_copy(dest):
    files = subprocess.run(["git", "ls-files", "-co", "--exclude-standard", "-z"], cwd=REPO,
                           capture_output=True, text=True, check=True).stdout.split("\0")
    for f in files:
        if f.startswith(COPY_PREFIXES) and os.path.isfile(os.path.join(REPO, f)):
            os.makedirs(os.path.join(dest, os.path.dirname(f)), exist_ok=True)
            shutil.copy2(os.path.join(REPO, f), os.path.join(dest, f))


def build_and_run(root, tests_dir, test, timeout):
    """'pass', 'fail', 'timeout' or 'build' (did not compile)."""
    binary = os.path.join(root, tests_dir, "build", "test_" + test)
    if os.path.exists(binary):
        os.remove(binary)
    b = subprocess.run(["make", "-s", "-C", os.path.join(root, tests_dir), "build/test_" + test],
                       capture_output=True, text=True)
    if b.returncode != 0:
        return "build"
    try:
        r = subprocess.run([binary], cwd=os.path.join(root, tests_dir),
                           capture_output=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return "timeout"
    return "pass" if r.returncode == 0 else "fail"


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("suite", choices=sorted(SUITES))
    ap.add_argument("module", help="as the coverage report names it, e.g. sd_card.c")
    ap.add_argument("--test", help="test binary to run (default: the module's COV_PAIRS test)")
    ap.add_argument("--lines", help="only mutate lines A-B")
    ap.add_argument("--max", type=int, help="sample at most N mutants")
    ap.add_argument("--seed", type=int, default=1, help="sampling seed (default 1)")
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 1)
    ap.add_argument("--list", action="store_true", help="print the mutants, run nothing")
    ap.add_argument("--keep", action="store_true", help="keep the scratch trees")
    args = ap.parse_args()

    tests_dir, src_root = SUITES[args.suite]
    src_rel = os.path.join(src_root, args.module)
    src = open(os.path.join(REPO, src_rel)).read()
    test = args.test or cov_pairs(os.path.join(REPO, tests_dir)).get(args.module)
    if not test:
        sys.exit(f"{args.module}: no COV_PAIRS test in {tests_dir}/Makefile (use --test)")

    todo = mutants(src)
    if args.lines:
        a, b = (int(x) for x in args.lines.split("-"))
        todo = [t for t in todo if a <= t[0] <= b]
    if args.max and len(todo) > args.max:
        todo = sorted(random.Random(args.seed).sample(todo, args.max), key=lambda t: t[0])
    lines = src.split("\n")
    if args.list:
        for line, desc, _ in todo:
            print(f"{args.module}:{line}: {desc:28s} | {lines[line - 1].strip()[:70]}")
        print(f"{len(todo)} mutants")
        return 0

    jobs = max(1, min(args.jobs, len(todo)))
    roots = [tempfile.mkdtemp(prefix="mutate-") for _ in range(jobs)]
    try:
        for r in roots:
            scratch_copy(r)
        t0 = time.time()
        if build_and_run(roots[0], tests_dir, test, None) != "pass":
            sys.exit(f"test_{test} does not pass on the unmutated tree - fix that first")
        timeout = max(10.0, 10 * (time.time() - t0))
        free = list(roots)

        def run(item):
            root = free.pop()
            try:
                path = os.path.join(root, src_rel)
                open(path, "w").write(item[2])
                verdict = build_and_run(root, tests_dir, test, timeout)
                open(path, "w").write(src)
                return item, verdict
            finally:
                free.append(root)

        print(f"{len(todo)} mutants of {src_rel}, test_{test}, {jobs} job(s)")
        tally = {"fail": 0, "timeout": 0, "pass": 0, "build": 0}
        with ThreadPoolExecutor(max_workers=jobs) as pool:
            for (line, desc, _), verdict in pool.map(run, todo):
                tally[verdict] += 1
                if verdict == "pass":
                    print(f"  SURVIVED {args.module}:{line}: {desc:28s} | {lines[line - 1].strip()[:70]}")
    finally:
        if not args.keep:
            for r in roots:
                shutil.rmtree(r, ignore_errors=True)
    killed = tally["fail"] + tally["timeout"]
    judged = killed + tally["pass"]
    print(f"\nkilled {killed}, survived {tally['pass']}, did not build {tally['build']}"
          + (f" - score {100.0 * killed / judged:.1f}%" if judged else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
