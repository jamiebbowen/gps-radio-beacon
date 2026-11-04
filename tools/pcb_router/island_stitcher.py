#!/usr/bin/env python3
"""Pixel-BFS GND stitcher: find shortest two-layer path from stranded GND pads
to open space covered by the main GND pour island; emit segments into routes."""
import json
import math
from collections import deque

src = open("/tmp/route_rx.py").read().replace('if __name__ == "__main__":\n    raise SystemExit(main())', "")
env = {}
exec(compile(src, "rp", "exec"), env)
import pcbnew

board = pcbnew.LoadBoard(env["PCB_PATH"])
world = env["World"](board)
routes = json.load(open("/tmp/routes_rx.json"))
for (x1, y1, x2, y2, layer, w, net) in env["PREPLACED"]:
    world.add_route(net, [(x1, y1, x2, y2, layer)], [], w / 2)
for net, items in routes.items():
    if not items or net == "Net-(J12-In)":
        continue
    segs, vias, half = [], [], 0.125
    for it in items:
        if it[0] == "VIA":
            vias.append((it[1], it[2]))
        else:
            half = it[5] / 2
            segs.append((it[0], it[1], it[2], it[3], it[4]))
    world.add_route(net, segs, vias, half)

NM = 1e6

# ---- main pour islands per layer (biggest outline of the filled zones)
def load_main_polys():
    polys = {}
    for z in board.Zones():
        lay = z.GetLayer()
        lname = board.GetLayerName(lay)
        pset = z.GetFilledPolysList(lay)
        best = None
        bestarea = -1
        for oi in range(pset.OutlineCount()):
            ol = pset.Outline(oi)
            pts = [(ol.CPoint(i).x / NM, ol.CPoint(i).y / NM) for i in range(ol.PointCount())]
            area = 0
            for i in range(len(pts)):
                x1, y1 = pts[i]
                x2, y2 = pts[(i + 1) % len(pts)]
                area += x1 * y2 - x2 * y1
            if abs(area) > bestarea:
                bestarea = abs(area)
                best = pts
        polys[lname] = best
    return polys


MAIN = load_main_polys()


def pip(x, y, poly):
    inside = False
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


def in_main(x, y, layer):
    p = MAIN.get(layer)
    if p is None:
        return False
    return pip(x, y, p)


# ---- pixel grid BFS
STEP = 0.2
ROUNDS = [
    ("C11.2", 139.43, 133.0, (125.0, 120.0, 163.0, 142.0)),
    ("C10.2", 120.58, 133.0, (100.0, 120.0, 160.0, 142.0)),
]


def bfs_stitch(label, px, py, window):
    x0, y0, x1, y1 = window
    nx = int((x1 - x0) / STEP) + 1
    ny = int((y1 - y0) / STEP) + 1

    def cell_net(i, j):
        return (x0 + i * STEP, y0 + j * STEP)

    free = {}
    for layer in ("F.Cu", "B.Cu"):
        f = bytearray(nx * ny)
        for j in range(ny):
            for i in range(nx):
                x, y = x0 + i * STEP, y0 + j * STEP
                f[j * nx + i] = 1 if world.point_ok(x, y, layer, "FP_GND", 0.125) else 0
        free[layer] = f

    # seed from pad center on BOTH layers (TH pads carry both)
    def snap(v, o, n):
        i = max(0, min(n - 1, round((v - o) / STEP)))
        return i

    si, sj = snap(px, x0, nx), snap(py, y0, ny)
    start = []
    for layer in ("F.Cu", "B.Cu"):
        start.append((si, sj, layer))
    q = deque()
    seen = set()
    for s in start:
        q.append(s)
        seen.add(s)
    par = {}
    goal = None
    steps = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1))
    hops = 0
    while q:
        i, j, layer = q.popleft()
        hops += 1
        x, y = x0 + i * STEP, y0 + j * STEP
        if in_main(x, y, layer) and (i, j) != (si, sj):
            goal = (i, j, layer)
            break
        for (di, dj) in steps:
            ni, nj = i + di, j + dj
            if not (0 <= ni < nx and 0 <= nj < ny):
                continue
            key = (ni, nj, layer)
            if key in seen:
                continue
            if not free[layer][nj * nx + ni]:
                continue
            seen.add(key)
            par[key] = (i, j, layer)
            q.append(key)
        # layer transfer (via)
        oth = "B.Cu" if layer == "F.Cu" else "F.Cu"
        key = (i, j, oth)
        if key not in seen and free[oth][j * nx + i] and world.via_ok(x, y, "FP_GND"):
            seen.add(key)
            par[key] = (i, j, layer)
            q.append(key)
    print(f"  {label}: explored {hops} cells")
    if goal is None:
        return False
    # reconstruct + simplify to 8-dir runs
    path = []
    g = goal
    while True:
        path.append(g)
        if g in par:
            g = par[g]
        else:
            break
    path.reverse()
    # geometric simplification: merge collinear runs
    pts = [(x0 + i * STEP, y0 + j * STEP, l) for (i, j, l) in path]
    out = []
    vias = []
    i = 0
    while i < len(pts):
        layer = pts[i][2]
        j = i + 1
        while j < len(pts) and pts[j][2] == layer:
            j += 1
        run = pts[i:j]
        k = 0
        while k < len(run) - 1:
            m = k + 1
            dx0 = run[k + 1][0] - run[k][0]
            dy0 = run[k + 1][1] - run[k][1]
            L0 = math.hypot(dx0, dy0) or 1
            d0 = (dx0 / L0, dy0 / L0)
            while m < len(run) - 1:
                dx1 = run[m + 1][0] - run[m][0]
                dy1 = run[m + 1][1] - run[m][1]
                L1 = math.hypot(dx1, dy1) or 1
                d1 = (dx1 / L1, dy1 / L1)
                if abs(d1[0] - d0[0]) < 1e-3 and abs(d1[1] - d0[1]) < 1e-3:
                    m += 1
                else:
                    break
            out.append((run[k][0], run[k][1], run[m][0], run[m][1], layer, 0.25))
            k = m
        i = j
        if i < len(pts):
            vias.append((pts[i][0], pts[i][1]))
    world.add_route("FP_GND", [(a, b_, c, d, e) for (a, b_, c, d, e, w) in out], vias, 0.125)
    routes.setdefault("FP_GND", [])
    routes["FP_GND"].extend(out)
    routes["FP_GND"].extend([("VIA", vx, vy) for vx, vy in vias])
    print(f"  {label}: stitched, {len(out)} segs {len(vias)} vias -> goal ({goal[0]*STEP+x0:.2f},{goal[1]*STEP+y0:.2f},{goal[2]})", flush=True)
    for s in out:
        print("    ", s)
    return True


allok = True
for label, px, py, window in ROUNDS:
    print("stitch", label, flush=True)
    allok &= bfs_stitch(label, px, py, window)

if allok:
    json.dump(routes, open("/tmp/routes_rx.json", "w"))
    print("saved", flush=True)
else:
    print("SOME FAILED", flush=True)
