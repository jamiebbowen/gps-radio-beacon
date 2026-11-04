#!/usr/bin/env python3
"""Route the main RX board: generalizes the panel router.

- full-board outline from Edge.Cuts (chamfered rect), MH keepouts
- SMD rect/oval pads single-layer aware; TH everywhere
- power rails >bottom< biased, wide; signals 0.25; GND via pours only
- RF antenna trace pre-placed & locked (Net-(J12-In))
"""
import heapq
import math
import os
import json
import time
from collections import defaultdict

import pcbnew

PCB_PATH = "/home/jbbowen/Desktop/Rocketry/gps-radio-beacon/receiver/pcb/pcb.kicad_pcb"

TRACK_W = 0.25
TRACK_HALF = TRACK_W / 2
CLEAR = 0.2
SLACK = 0.02
VIA_R = 0.4
MH_R = 1.6
HOLE_CLEAR = 0.25
EDGE_SAFE = 0.45

TRK_VS_COPPER = CLEAR + TRACK_HALF + SLACK
VIA_VS_COPPER = CLEAR + VIA_R + SLACK
TRK_VS_MH = MH_R + HOLE_CLEAR + TRACK_HALF + SLACK
VIA_VS_MH = MH_R + HOLE_CLEAR + VIA_R + SLACK
VIA_VS_VIA = 2 * VIA_R + CLEAR + SLACK

NM = 1e6


def pt_seg_dist(px, py, qx, qy, rx, ry):
    wx, wy = rx - qx, ry - qy
    t = ((px - qx) * wx + (py - qy) * wy) / ((wx * wx + wy * wy) or 1e-30)
    t = max(0.0, min(1.0, t))
    cx, cy = qx + t * wx, qy + t * wy
    return math.hypot(px - cx, py - cy)


def seg_seg_dist(ax, ay, bx, by, cx, cy, dx, dy):
    def cross(ux, uy, vx, vy):
        return ux * vy - uy * vx
    o1 = cross(bx - ax, by - ay, cx - ax, cy - ay)
    o2 = cross(bx - ax, by - ay, dx - ax, dy - ay)
    o3 = cross(dx - cx, dy - cy, ax - cx, ay - cy)
    o4 = cross(dx - cx, dy - cy, bx - cx, by - cy)
    if o1 * o2 < 0 and o3 * o4 < 0:
        return 0.0
    return min(pt_seg_dist(ax, ay, cx, cy, dx, dy),
               pt_seg_dist(bx, by, cx, cy, dx, dy),
               pt_seg_dist(cx, cy, ax, ay, bx, by),
               pt_seg_dist(dx, dy, ax, ay, bx, by))


def pt_rect_dist(px, py, hx, hy, cx, cy):
    dx = max(abs(px - cx) - hx, 0.0)
    dy = max(abs(py - cy) - hy, 0.0)
    return math.hypot(dx, dy)


def seg_rect_dist(ax, ay, bx, by, hx, hy, cx, cy):
    if pt_rect_dist(ax, ay, hx, hy, cx, cy) == 0.0:
        return 0.0
    if pt_rect_dist(bx, by, hx, hy, cx, cy) == 0.0:
        return 0.0
    edges = [
        (cx - hx, cy - hy, cx + hx, cy - hy),
        (cx + hx, cy - hy, cx + hx, cy + hy),
        (cx + hx, cy + hy, cx - hx, cy + hy),
        (cx - hx, cy + hy, cx - hx, cy - hy),
    ]
    return min(seg_seg_dist(ax, ay, bx, by, *e) for e in edges)


class Prim:
    __slots__ = ("kind", "x1", "y1", "x2", "y2", "r", "hx", "hy", "rc",
                 "net", "extra", "pl", "via")

    def __init__(self, x1, y1, x2, y2, r):
        self.kind = "cap"
        self.x1, self.y1, self.x2, self.y2 = x1, y1, x2, y2
        self.r = r
        self.hx = self.hy = self.rc = None
        self.net = None
        self.extra = 0.0
        self.pl = "BOTH"   # pad layer(s)
        self.via = False

    @classmethod
    def rrect(cls, cx, cy, hx, hy, rc):
        p = cls(cx, cy, cx, cy, max(hx, hy))
        p.kind = "rrect"
        p.hx, p.hy, p.rc = hx, hy, rc
        return p

    def bbox(self):
        e = self.extra
        if self.kind == "rrect":
            return (self.x1 - self.hx - e, self.y1 - self.hy - e,
                    self.x1 + self.hx + e, self.y1 + self.hy + e)
        return (min(self.x1, self.x2) - self.r - e, min(self.y1, self.y2) - self.r - e,
                max(self.x1, self.x2) + self.r + e, max(self.y1, self.y2) + self.r + e)

    def dpt(self, px, py):
        if self.kind == "rrect":
            chx, chy = self.hx - self.rc, self.hy - self.rc
            return pt_rect_dist(px, py, chx, chy, self.x1, self.y1) - self.rc
        return pt_seg_dist(px, py, self.x1, self.y1, self.x2, self.y2) - self.r

    def dseg(self, ax, ay, bx, by):
        if self.kind == "rrect":
            chx, chy = self.hx - self.rc, self.hy - self.rc
            return seg_rect_dist(ax, ay, bx, by, chx, chy, self.x1, self.y1) - self.rc
        return seg_seg_dist(self.x1, self.y1, self.x2, self.y2, ax, ay, bx, by) - self.r


class Bucket:
    __slots__ = ("cell", "map")

    def __init__(self, cell=5.0):
        self.cell = cell
        self.map = defaultdict(list)

    def _cells(self, bb):
        x0, y0, x1, y1 = bb
        ix0, iy0 = int(x0 // self.cell), int(y0 // self.cell)
        ix1, iy1 = int(x1 // self.cell), int(y1 // self.cell)
        for ix in range(ix0, ix1 + 1):
            for iy in range(iy0, iy1 + 1):
                yield ix, iy

    def add(self, prim):
        for c in self._cells(prim.bbox()):
            self.map[c].append(prim)

    def near(self, px, py, rad):
        c = int(px // self.cell), int(py // self.cell)
        out = []
        span = int(rad // self.cell) + 2
        for ix in range(c[0] - span, c[0] + span + 1):
            for iy in range(c[1] - span, c[1] + span + 1):
                out.extend(self.map.get((ix, iy), ()))
        return out


class World:
    def __init__(self, board):
        self.pad_obst = defaultdict(list)
        self.holes = []
        self.segments = defaultdict(list)
        self.vias = defaultdict(list)
        for fp in board.Footprints():
            ref = fp.GetReference()
            rot = fp.GetOrientationDegrees()
            for pad in fp.Pads():
                net = pad.GetNetname()
                p = pad.GetPosition()
                x, y = p.x / NM, p.y / NM
                if ref.startswith("MH"):
                    self.holes.append((x, y, MH_R))
                    continue
                sz = pad.GetSize()
                w, h = sz.x / NM, sz.y / NM
                if rot in (90.0, 270.0, -90.0, -270.0):
                    w, h = h, w
                shape = pad.GetShape()
                if shape == pcbnew.PAD_SHAPE_ROUNDRECT:
                    rc = pad.GetRoundRectRadiusRatio() * min(w, h) / 2
                    pr = Prim.rrect(x, y, w / 2, h / 2, rc)
                elif shape == pcbnew.PAD_SHAPE_RECT:
                    pr = Prim.rrect(x, y, w / 2, h / 2, 0.02)
                elif abs(w - h) < 1e-9:
                    pr = Prim(x, y, x, y, w / 2)
                elif h > w:
                    s = (h - w) / 2
                    pr = Prim(x, y - s, x, y + s, w / 2)
                else:
                    s = (w - h) / 2
                    pr = Prim(x - s, y, x + s, y, h / 2)
                is_th = pad.GetAttribute() == pcbnew.PAD_ATTRIB_PTH
                pr.pl = "BOTH" if is_th else ("F.Cu" if pad.IsOnLayer(pcbnew.F_Cu) else "B.Cu")
                pr.extra = 0.0
                if net == "FP_GND":
                    pr.extra = 0.35
                self.pad_obst[net].append(pr)
        self.pad_index = Bucket(5.0)
        for net, prims in self.pad_obst.items():
            for pr in prims:
                pr.net = net
                self.pad_index.add(pr)
        self.seg_index = defaultdict(lambda: Bucket(5.0))
        self.outline = self._outline_from_board(board)
        self.outline_idx = Bucket(5.0)
        for (x1, y1, x2, y2) in self.outline:
            self.outline_idx.add(Prim(x1, y1, x2, y2, 0.0))

    def _outline_from_board(self, board):
        segs = []
        for d in board.GetDrawings():
            if d.GetLayerName() != "Edge.Cuts":
                continue
            if d.GetClass() != "PCB_SHAPE":
                continue
            s, e = d.GetStart(), d.GetEnd()
            x1, y1, x2, y2 = s.x / NM, s.y / NM, e.x / NM, e.y / NM
            if d.GetShape() == 1:  # arc
                m = d.GetMid()
                mx, my = m.x / NM, m.y / NM
                # circumcentre from 3 pts
                ax_, ay, ax2_ = x1, y1, x2
                bx, by = mx, my
                # chord midpoints + perpendiculars
                m1x, m1y = (x1 + mx) / 2, (y1 + my) / 2
                d1x, d1y = mx - x1, my - y1
                m2x, m2y = (mx + x2) / 2, (my + y2) / 2
                d2x, d2y = x2 - mx, y2 - my
                # solve perp intersection
                det = d1x * d2y - d1y * d2x
                if abs(det) > 1e-9:
                    t = ((m2x - m1x) * d2y - (m2y - m1y) * d2x) / det / -1 if False else \
                        ((m2x - m1x) * d2y - (m2y - m1y) * d2x) / det
                    cx = m1x + t * -d1y * 1.0
                    # use formula: c = m1 + u * perp(d1)
                    u = ((m2x - m1x) * d2x + (m2y - m1y) * d2y) / (d2x * (-d1y) - d2y * (d1x) + 1e-30) * -1
                    cx = m1x + u * d1y * 0 if False else m1x + u * d1y
                    cy = m1y - u * d1x
                    pt_c = (cx, cy)
                    r = math.hypot(x1 - cx, y1 - cy)
                    a1 = math.atan2(y1 - cy, x1 - cx)
                    a2 = math.atan2(y2 - cy, x2 - cx)
                    am = math.atan2(my - cy, mx - cx)
                    # ensure sweep passes through am
                    while a2 <= a1:
                        a2 += 2 * math.pi
                    if not (a1 <= am <= a2 + 1e-6):
                        # reverse
                        a2 -= 2 * math.pi
                        while a2 >= a1:
                            a2 -= 2 * math.pi
                        am2 = am
                        while am2 >= a2 - 1e-6:
                            pass
                    n = 16
                    last = (x1, y1)
                    for k in range(1, n + 1):
                        a = a1 + (a2 - a1) * k / n
                        pt = (cx + r * math.cos(a), cy + r * math.sin(a))
                        segs.append((last[0], last[1], pt[0], pt[1]))
                        last = pt
            else:
                segs.append((x1, y1, x2, y2))
        return segs

    def track_ok(self, ax, ay, bx, by, layer, net, half=TRACK_HALF):
        cx, cy = (ax + bx) / 2, (ay + by) / 2
        rad = math.hypot(bx - ax, by - ay) / 2 + 2.6
        for pr in self.pad_index.near(cx, cy, rad):
            if pr.net == net:
                continue
            if pr.pl != "BOTH" and pr.pl != layer:
                continue
            lim = half + CLEAR + SLACK + pr.extra
            if pr.dseg(ax, ay, bx, by) < lim:
                return False
        for pr in itertools_chain(self.seg_index[layer].near(cx, cy, rad),
                                  self.seg_index["BOTH"].near(cx, cy, rad)):
            if getattr(pr, "net", None) == net:
                continue
            lim = half + pr.r + CLEAR + SLACK + pr.extra
            if pr.dseg(ax, ay, bx, by) < lim:
                return False
        for (hx, hy, hr) in self.holes:
            if pt_seg_dist(hx, hy, ax, ay, bx, by) < hr + HOLE_CLEAR + half + SLACK:
                return False
        for pr in self.outline_idx.near(cx, cy, rad):
            if pr.dseg(ax, ay, bx, by) < 0.275 + half + SLACK:
                return False
        return True

    def point_ok(self, px, py, layer, net, half=TRACK_HALF):
        for pr in self.pad_index.near(px, py, 2.6):
            if pr.net == net:
                continue
            if pr.pl != "BOTH" and pr.pl != layer:
                continue
            lim = half + CLEAR + SLACK + pr.extra
            if pr.dpt(px, py) < lim:
                return False
        for lay in (layer, "BOTH"):
            for pr in self.seg_index[lay].near(px, py, 2.6):
                if getattr(pr, "net", None) == net:
                    continue
                lim = half + pr.r + CLEAR + SLACK + pr.extra
                if pr.dpt(px, py) < lim:
                    return False
        for (hx, hy, hr) in self.holes:
            if math.hypot(px - hx, py - hy) < TRK_VS_MH + (half - TRACK_HALF):
                return False
        for pr in self.outline_idx.near(px, py, 1.2):
            if pr.dpt(px, py) < 0.275 + half + SLACK:
                return False
        return True

    def via_ok(self, px, py, net):
        for pr in self.pad_index.near(px, py, 2.6):
            if pr.dpt(px, py) < VIA_R + CLEAR + SLACK + pr.extra:
                return False
        for lay in ("F.Cu", "B.Cu", "BOTH"):
            for pr in self.seg_index[lay].near(px, py, 2.6):
                if getattr(pr, "net", None) == net:
                    continue
                if pr.dpt(px, py) < VIA_R + pr.r + CLEAR + SLACK + pr.extra:
                    return False
        for other_net, vlist in self.vias.items():
            if other_net == net:
                continue
            for (vx, vy) in vlist:
                if math.hypot(px - vx, py - vy) < VIA_VS_VIA:
                    return False
        for (hx, hy, hr) in self.holes:
            if math.hypot(px - hx, py - hy) < VIA_VS_MH:
                return False
        for pr in self.outline_idx.near(px, py, 1.2):
            if pr.dpt(px, py) < VIA_R + 0.3:
                return False
        return True

    def overlap_penalty(self, ax, ay, bx, by, layer, net):
        other = "B.Cu" if layer == "F.Cu" else "F.Cu"
        cx, cy = (ax + bx) / 2, (ay + by) / 2
        rad = math.hypot(bx - ax, by - ay) / 2 + 1.0
        dax, day = bx - ax, by - ay
        L = math.hypot(dax, day) or 1.0
        daxn, dayn = dax / L, day / L
        pen = 0.0
        for pr in self.seg_index[other].near(cx, cy, rad) if other in self.seg_index else []:
            if getattr(pr, "net", None) == net:
                continue
            d = pr.dseg(ax, ay, bx, by) - pr.r
            if d > 0.45:
                continue
            d1 = math.hypot(pr.x2 - pr.x1, pr.y2 - pr.y1) or 1.0
            dpx = (pr.x2 - pr.x1) / d1
            dpy = (pr.y2 - pr.y1) / d1
            if abs(dpx * daxn + dpy * dayn) > 0.985:
                pen += 6.0
        return pen

    def add_route(self, net, segments, vias, half=TRACK_HALF):
        for (x1, y1, x2, y2, layer) in segments:
            pr = Prim(x1, y1, x2, y2, half)
            pr.net = net
            pr.pl = layer
            self.segments[net].append((pr, layer))
            self.seg_index[layer].add(pr)
        for (vx, vy) in vias:
            self.vias[net].append((vx, vy))
            pr = Prim(vx, vy, vx, vy, VIA_R)
            pr.net = net
            pr.pl = "BOTH"
            pr.via = True
            self.seg_index["BOTH"].add(pr)


def itertools_chain(*its):
    for it in its:
        for x in it:
            yield x


LAYERS = ("F.Cu", "B.Cu")
VIA_COST = 4.0
SQRT2 = math.sqrt(2.0)
DIAG_SPAN_MM = 15.0


def nearest_key(v):
    return round(v, 4)


def route_pair(world, net, sources_covered, goal_pad, xs, ys, valid, xi, yi,
               half=TRACK_HALF, bias_layer=None, bias_mult=2.5, region_penalty=None, own_pad_xy=None,
               pad_layers=None):
    if own_pad_xy is None:
        own_pad_xy = set()
    gx, gy = goal_pad
    gi = (xi[nearest_key(gx)], yi[nearest_key(gy)])
    gi_k = (nearest_key(gx), nearest_key(gy))

    xmap = {nearest_key(v): i for i, v in enumerate(xs)}
    ymap = {nearest_key(v): i for i, v in enumerate(ys)}

    def h(n):
        dx = abs(xs[n[0]] - gx)
        dy = abs(ys[n[1]] - gy)
        return max(dx, dy) + (SQRT2 - 1.0) * min(dx, dy)

    def moves_of(node):
        ix, iy, layer = node
        x0, y0 = xs[ix], ys[iy]
        out = []
        if ix > 0:
            out.append((ix - 1, iy, x0 - xs[ix - 1], 'L', layer))
        if ix < len(xs) - 1:
            out.append((ix + 1, iy, xs[ix + 1] - x0, 'R', layer))
        if iy > 0:
            out.append((ix, iy - 1, y0 - ys[iy - 1], 'U', layer))
        if iy < len(ys) - 1:
            out.append((ix, iy + 1, ys[iy + 1] - y0, 'D', layer))
        for dxs, dys, lbl in ((1, 1, 'DRU'), (1, -1, 'DRD'), (-1, 1, 'DLU'), (-1, -1, 'DLD')):
            j = ix + dxs
            while 0 <= j < len(xs):
                d = xs[j] - x0
                if abs(d) > DIAG_SPAN_MM:
                    break
                y2 = y0 + (d if dys > 0 else -d)
                k = ymap.get(nearest_key(y2))
                if k is not None:
                    out.append((j, k, abs(d) * SQRT2, lbl, layer))
                j += dxs
        other = "B.Cu" if layer == "F.Cu" else "F.Cu"
        out.append((ix, iy, VIA_COST, 'V', other))
        return out

    dist = {}
    parent = {}
    pq = []
    for node in sources_covered:
        if node in valid:
            dist[node] = 0.0
            parent[node] = None
            heapq.heappush(pq, (h(node), 0.0, node))
    best_seen = {}
    end = None
    pops = 0
    while pq:
        f, g, node = heapq.heappop(pq)
        pops += 1
        if pops % 50000 == 0:
            print(f"      a* pops={pops} qlen={len(pq)} node=({xs[node[0]]:.2f},{ys[node[1]]:.2f})", flush=True)
        if node in best_seen and best_seen[node] <= g - 1e-9:
            continue
        best_seen[node] = g
        if (node[0], node[1]) == gi and node not in sources_covered:
            allowed = pad_layers.get(gi_k, LAYERS) if pad_layers is not None else LAYERS
            if node[2] in allowed:
                end = node
                break
        ix, iy, layer = node
        cur_pad_center = (ix, iy) in own_pad_xy
        x0, y0v = xs[ix], ys[iy]
        prev_dir = None
        pn = parent.get(node)
        if pn is not None and pn[0] is not None:
            prev_dir = pn[1]
        for (nx, ny2, cost, lbl, nlayer) in moves_of(node):
            nn = (nx, ny2, nlayer)
            if nn not in valid:
                continue
            if lbl[0] == 'D' and (cur_pad_center or (nx, ny2) in own_pad_xy):
                continue
            if lbl == 'V':
                if not world.via_ok(xs[ix], ys[iy], net):
                    continue
                bend = 0.0
                mcost = cost
            else:
                if not world.track_ok(x0, y0v, xs[nx], ys[ny2], layer, net, half):
                    continue
                bend = 0.25 if (prev_dir is not None and prev_dir != lbl and prev_dir != 'V') else 0.0
                mcost = cost * (bias_mult if (bias_layer and nlayer != bias_layer) else 1.0)
                ex0, ey0 = xs[ix], ys[iy]
                ex1, ey1 = xs[nn[0]], ys[nn[1]]
                for (zx0, zy0, zx1, zy1, zm) in SOFT_ZONES:
                    if max(ex0, ex1) >= zx0 and min(ex0, ex1) <= zx1 and max(ey0, ey1) >= zy0 and min(ey0, ey1) <= zy1:
                        mcost *= zm
                        break
                if region_penalty:
                    x0r, y0r, x1r, y1r, mult = region_penalty
                    ax2, ay2 = xs[nn[0]], ys[nn[1]]
                    if max(x0, ax2) >= x0r and min(x0, ax2) <= x1r and max(y0v, ay2) >= y0r and min(y0v, ay2) <= y1r:
                        mcost *= mult
                mcost += world.overlap_penalty(x0, y0v, xs[nx], ys[ny2], layer, net)
            ng = g + mcost + bend
            if ng < dist.get(nn, float("inf")) - 1e-9:
                dist[nn] = ng
                parent[nn] = (node, lbl)
                heapq.heappush(pq, (ng + h(nn), ng, nn))
    if end is None:
        return None
    path = []
    n = end
    while n is not None:
        path.append(n)
        pp = parent.get(n)
        n = pp[0] if pp else None
    path.reverse()
    return path


def path_to_copper(path, xs, ys):
    segments = []
    vias = []
    i = 0
    while i < len(path):
        node = path[i]
        layer = node[2]
        j = i + 1
        while j < len(path) and path[j][2] == layer:
            j += 1
        run = path[i:j]
        def dirn(a, b2):
            dx = xs[b2[0]] - xs[a[0]]
            dy = ys[b2[1]] - ys[a[1]]
            L = math.hypot(dx, dy) or 1.0
            return (dx / L, dy / L)
        k = 0
        while k < len(run) - 1:
            m = k + 1
            d0 = dirn(run[k], run[k + 1])
            while m < len(run) - 1:
                d1 = dirn(run[m], run[m + 1])
                if abs(d1[0] - d0[0]) < 1e-6 and abs(d1[1] - d0[1]) < 1e-6:
                    m += 1
                else:
                    break
            segments.append((xs[run[k][0]], ys[run[k][1]],
                             xs[run[m][0]], ys[run[m][1]], layer))
            k = m
        i = j
        if i < len(path):
            vias.append((xs[path[i][0]], ys[path[i][1]]))
    return segments, vias


# soft keep-out bands: keep pour spillways open. Any segment entering pays x8.
SOFT_ZONES = [
    (143.0, 111.0, 153.0, 125.0, 8.0),   # east keyhole between J6 fanout & BP pocket
    (125.0, 128.8, 145.0, 130.6, 8.0),   # south seam below the BP pocket
]

WIDTHS = {"FP_VCC": 0.8, "3V3": 0.6, "VBAT": 1.0, "Net-(J4-Pin_1)": 1.0,
          "Net-(J4-Pin_2)": 1.0, "Net-(J12-In)": 1.0}
BIASES = {"FP_VCC": "B.Cu", "3V3": "B.Cu", "VBAT": "B.Cu",
          "Net-(J4-Pin_1)": "B.Cu", "Net-(J4-Pin_2)": "B.Cu"}

# pre-placed copper: RF antenna feed (locked), E22 ANT pad -> J12
PREPLACED = [
    (136.2, 75.32, 145.54, 75.46, "F.Cu", 1.0, "Net-(J12-In)"),
]


def main():
    board = pcbnew.LoadBoard(PCB_PATH)
    world = World(board)

    # pre-placed RF trace becomes an obstacle for everything else
    for (x1, y1, x2, y2, layer, w, net) in PREPLACED:
        world.add_route(net, [(x1, y1, x2, y2, layer)], [], w / 2)

    pads_by_net = defaultdict(list)
    pad_layers = {}                       # (x4, y4) -> allowed layers fro this exact pad
    for fp in board.Footprints():
        for pad in fp.Pads():
            net = pad.GetNetname()
            if not net or net.startswith("unconnected"):
                continue
            p = pad.GetPosition()
            pos = (round(p.x / NM, 4), round(p.y / NM, 4))
            if pos not in pads_by_net[net]:
                pads_by_net[net].append(pos)
            is_th = pad.GetAttribute() == pcbnew.PAD_ATTRIB_PTH
            pad_layers[pos] = frozenset(LAYERS) if is_th else frozenset(
                (("F.Cu",) if pad.IsOnLayer(pcbnew.F_Cu) else ("B.Cu",)))

    xs, ys = set(), set()
    for net, plist in pads_by_net.items():
        for (x, y) in plist:
            xs.add(nearest_key(x)); ys.add(nearest_key(y))
    for (hx, hy, hr) in world.holes:
        xs.add(nearest_key(hx)); ys.add(nearest_key(hy))
    sx, sy = sorted(xs), sorted(ys)
    for a, b in zip(sx, sx[1:]):
        xs.add(nearest_key((a + b) / 2))
    for a, b in zip(sy, sy[1:]):
        ys.add(nearest_key((a + b) / 2))
    v = 59.85
    while v <= 165.55:
        xs.add(nearest_key(v)); v += 0.75
    v = 26.9
    while v <= 150.6:
        ys.add(nearest_key(v)); v += 0.75
    xs = sorted(xs); ys = sorted(ys)
    xi = {v: i for i, v in enumerate(xs)}
    yi = {v: i for i, v in enumerate(ys)}
    print(f"grid {len(xs)} x {len(ys)} x 2 layers", flush=True)

    def build_valid(net, half=TRACK_HALF):
        valid = set()
        for layer in LAYERS:
            for ixx, x in enumerate(xs):
                for iyy, y in enumerate(ys):
                    if world.point_ok(x, y, layer, net, half):
                        valid.add((ixx, iyy, layer))
        return valid

    order_override = os.environ.get("RP_ORDER")
    order = [n for n in order_override.split(",")] if order_override else [
        "FP_VCC", "Net-(J4-Pin_1)", "Net-(J4-Pin_2)", "VBAT", "3V3",
        "Net-(U7-PA0)",
        "PA2", "PA3", "PA8", "PA9", "PA10", "PA11", "PA12",
        "PB12", "PB13", "PB14", "PB15", "PA15",
        "FP_PB7", "FP_PB8", "FP_PB9",
        "FP_PA1", "FP_PA4", "FP_PA5", "FP_PA6", "FP_PA7",
        "FP_PB10", "FP_PB3", "FP_PB4", "FP_PB5", "FP_PB6", "FP_PC15",
        # FP_GND, PB0, PB1, PB2, PC13, PC14 -> pours / no connects
    ]
    results = {}
    for net in order:
        if net not in pads_by_net:
            print(f"warn: no pads for {net}", flush=True)
            continue
        pads = pads_by_net[net]
        half = WIDTHS.get(net, TRACK_W) / 2
        valid = build_valid(net, half)
        covered = set()
        for (px, py) in pads:
            ni = (xi[nearest_key(px)], yi[nearest_key(py)])
            for L in LAYERS:
                valid.add((ni[0], ni[1], L))
        n0 = (xi[nearest_key(pads[0][0])], yi[nearest_key(pads[0][1])])
        for L in pad_layers.get(pads[0], LAYERS):
            covered.add((n0[0], n0[1], L))
        own_pad_xy = {(xi[nearest_key(px)], yi[nearest_key(py)]) for (px, py) in pads}
        routes = []
        failed = False
        while True:
            done_pads = set()
            for (px, py) in pads:
                ni = (xi[nearest_key(px)], yi[nearest_key(py)])
                if any((ni[0], ni[1], L) in covered for L in LAYERS):
                    done_pads.add((px, py))
            remaining = [p for p in pads if p not in done_pads]
            if not remaining:
                break
            goal = remaining[0]
            t0 = time.time()
            path = route_pair(world, net, covered, goal, xs, ys, valid, xi, yi,
                              half, BIASES.get(net), own_pad_xy=own_pad_xy, pad_layers=pad_layers)
            el = time.time() - t0
            print(f"    {net} -> {goal}: {el:.1f}s", flush=True)
            if path is None:
                print(f"FAILED {net} -> {goal}", flush=True)
                failed = True
                break
            segs, vias = path_to_copper(path, xs, ys)
            world.add_route(net, segs, vias, half)
            routes.extend([(s[0], s[1], s[2], s[3], s[4], half * 2) for s in segs])
            routes.extend([("VIA", vx, vy) for vx, vy in vias])
            for node in path:
                covered.add(node)
        results[net] = None if failed else routes
        print(f"{net}: {'FAIL' if failed else f'{len(routes)} pieces'}", flush=True)

    results["Net-(J12-In)"] = [(x1, y1, x2, y2, layer, w) for (x1, y1, x2, y2, layer, w, net) in PREPLACED]
    with open("/tmp/routes_rx.json", "w") as f:
        json.dump(results, f)
    ok = all(v is not None for v in results.values())
    print("ALL OK" if ok else "SOME FAILED", flush=True)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
