#!/usr/bin/env python3
"""Textually replace all segments/vias in panel.kicad_pcb with routed ones."""
import json
import uuid

PCB = "/home/jbbowen/Desktop/Rocketry/gps-radio-beacon/receiver/pcb/pcb.kicad_pcb"

src = open(PCB, encoding="utf-8").read()
lines = src.split("\n")

# remove top-level (segment ...) and (via ...) blocks
out = []
i = 0
removed_seg = removed_via = 0
while i < len(lines):
    line = lines[i]
    stripped = line.strip()
    if stripped.startswith("(segment") or stripped.startswith("(via"):
        # skip until matching close at depth 0 relative to block start
        depth = 0
        instr = False
        esc = False
        depth = 0
        j = i
        done = False
        while j < len(lines):
            l = lines[j]
            for ch in l:
                if instr:
                    if esc:
                        esc = False
                    elif ch == "\\":
                        esc = True
                    elif ch == '"':
                        instr = False
                    continue
                if ch == '"':
                    instr = True
                elif ch == "(":
                    depth += 1
                elif ch == ")":
                    depth -= 1
                    if depth == 0:
                        done = True
                        break
            if done:
                break
            j += 1
        if stripped.startswith("(segment"):
            removed_seg += 1
        else:
            removed_via += 1
        i = j + 1
        continue
    out.append(line)
    i += 1

print(f"removed {removed_seg} segments, {removed_via} vias")

# build new blocks
routes = json.load(open("/tmp/routes_rx.json"))
blocks = []
for net, items in routes.items():
    for it in items:
        u = str(uuid.uuid4())
        if it[0] == "VIA":
            _, vx, vy = it
            blocks.append(f"""\t(via
\t\t(at {vx:.4f} {vy:.4f})
\t\t(size 0.8)
\t\t(drill 0.4)
\t\t(layers "F.Cu" "B.Cu")
\t\t(capping no)
\t\t(covering
\t\t\t(front no)
\t\t\t(back no)
\t\t)
\t\t(plugging
\t\t\t(front no)
\t\t\t(back no)
\t\t)
\t\t(filling no)
\t\t(net "{net}")
\t\t(uuid "{u}")
\t)""")
        else:
            x1, y1, x2, y2, layer = it[:5]
            w = it[5] if len(it) > 5 else 0.35
            blocks.append(f"""\t(segment
\t\t(start {x1:.4f} {y1:.4f})
\t\t(end {x2:.4f} {y2:.4f})
\t\t(width {w})
\t\t(layer "{layer}")
\t\t(net "{net}")
\t\t(uuid "{u}")
\t)""")

# insert before the final closing paren of the file
# find index of last line that is exactly ")" (close of kicad_pcb)
# the last non-empty line should be ")"
last = None
for idx in range(len(out) - 1, -1, -1):
    if out[idx].strip() == ")":
        last = idx
        break
# we must place before the outermost close; but there may be nested closers;
# the file must end with the kicad_pcb close. The final ")" matches depth 1.
depth = 0
instr = esc = False
target = None
for idx in range(len(out) - 1, -1, -1):
    pass
# simpler: recompute depth backwards
text = "\n".join(out)
# find last ')' that brings depth to 0
depth = 0
instr = esc = False
last_zero_close = None
for k, ch in enumerate(text):
    if instr:
        if esc:
            esc = False
        elif ch == "\\":
            esc = True
        elif ch == '"':
            instr = False
        continue
    if ch == '"':
        instr = True
    elif ch == "(":
        depth += 1
    elif ch == ")":
        depth -= 1
        if depth == 0:
            last_zero_close = k
assert last_zero_close is not None
newtext = text[:last_zero_close] + "\n" + "\n".join(blocks) + "\n" + text[last_zero_close:]
open(PCB, "w", encoding="utf-8").write(newtext)
print("inserted", len(blocks), "blocks")
