#!/usr/bin/env python3
"""
Snowplow visualizer: standard library only, so it runs on the laptop or the Pi.

  Live view on the robot: use dashboard.py (it reuses this file's drawing). This program is for replays,
  PNGs and the Tk window. --web works with every mode below (--replay, --sim too): no screen or tkinter needed.
      python3 visualizer.py --web --replay logs/        then open http://<pi>:8080

  Tk window instead (laptop and robot on the same network):
      laptop:  python3 visualizer.py --live                 (listens on UDP 5005)
      robot:   SNOWPLOW_VIZ=<laptop-ip>:5005 python3 demo_runner.py
    The viewer also saves what it receives to logs/live_*.jsonl.

  Replay a log afterwards (the robot always writes logs/phase{1,2}_*.jsonl):
      python3 visualizer.py --replay logs/phase1_20261112_101500.jsonl [more.jsonl ...]
      python3 visualizer.py --replay logs/                    (newest phase 1 + phase 2 pair)

  Watch the simulator:
      python3 visualizer.py --sim sample [--seed 2] [--speed 3] [--noise 2]

  Render a PNG without a window (needs Pillow, laptop only):
      python3 visualizer.py --replay run.jsonl --png out.png [--at 45]

What it draws: the board and grid, the s1-s2 gate (orange) and d1-d2 drop-off (purple) with the
bucket; what the robot believes (street = blue, houses = red hatching, exit = green, planned
path = dashed); the odometry pose and trail (blue); every ultrasonic hit as a dot plus the latest
beam; line snaps (green flashes); side glances; and, in the simulator, the true houses (tan),
true pose (grey) and pom-poms. The right panel has the phase clock out of 120 s, pose, and the
robot's console output.

Keys (replay): space = play/pause, left/right = -/+ 2 s, up/down = faster/slower, Home = restart.
"""
from __future__ import annotations
import argparse
import glob
import json
import math
import os
import socket
import subprocess
import sys
import time

T = 30.48
DEFAULT_TABLE = (-1, 7, -1, 3)                     # tiles x_min, x_max, y_min, y_max (one table); the
                                                   # meta / sim events carry the real size
PHASE_LIMIT = 120.0
HERE = os.path.dirname(os.path.abspath(__file__))

C = {
    "bg": "#1e1f24", "table": "#f4f1ea", "grid": "#5a5a5a", "axis": "#9aa0a6",
    "street": "#bcd7f5", "house_true": "#c9a87a", "house_seen": "#d64545", "exit": "#2e9e4f",
    "approach": "#e0a030", "gate": "#f08a00", "drop": "#8e44ad", "bucket": "#c39bd3",
    "plan": "#2f6fd6", "sweep": "#14a3a3", "odom": "#1f5fd1", "truth": "#7a7a7a", "us": "#ff7f0e",
    "hit": "#e8590c", "hit_old": "#f0b27a", "snap": "#16c060", "wall": "#ff2d2d",
    "pom": "#ff69b4", "pom_got": "#bdbdbd", "panel": "#26282e", "text": "#e8e8e8", "dim": "#9aa0a6",
    "ok": "#3cb371", "warn": "#f0a030", "bad": "#e04848",
}


# ============================================================ state
class State:
    """Everything known at the current playhead, built by applying events in order."""

    def __init__(self):
        self.reset({})

    def reset(self, meta):
        self.meta = meta
        self.phase = meta.get("phase")
        self.t = 0.0
        self.pose = None
        self.pose_t = 0.0
        self.err = None         # odometry vs truth, from samples taken close together
        self.trail = []
        self.truth = None
        self.truth_trail = []
        self.us = None          # (angle, dist, t)
        self.hits = []          # (x, y, t)
        self.pending_us = []    # readings waiting for the next pose, to be placed where the robot was at that time
        self.street, self.blocked, self.exit, self.approach = [], [], None, None
        self.plan, self.plan_sweep = [], False
        self.snaps = []         # (axis, value, t)
        self.walls = []         # (tile, t)
        self.glances = []       # (tile, side, open, t, dist, pose)
        self.scan = None        # (event, t) of the last stationary L/F/R scan
        self.sim = None         # {"boxes", "street", "pompoms", "outline"}
        self.picked = set()
        self.logs = []
        self.end = None
        self.dumps = []
        self.n_events = 0

    def apply(self, ev):
        k = ev.get("k")
        t = float(ev.get("t", self.t))
        self.n_events += 1
        if k == "meta":
            self.reset(ev)
            return
        self.t = max(self.t, t)
        if self.pending_us and self.t - self.pose_t > 1.2:     # no new pose for a while: the robot stands still
            for a, d, tu, p0, _ in self.pending_us:
                self._add_hit(a, d, tu, p0)
            self.pending_us = []
        if k == "pose":
            self._place_pending((ev["x"], ev["y"], ev["th"]), t)
            self.pose = (ev["x"], ev["y"], ev["th"])
            self.pose_t = t
            if not self.trail or _d2(self.trail[-1], self.pose) > 0.25:
                self.trail.append((ev["x"], ev["y"]))
        elif k == "truth":
            self.truth = (ev["x"], ev["y"], ev["th"])
            if self.pose is not None and abs(t - self.pose_t) < 0.08:
                self.err = (math.hypot(self.pose[0] - ev["x"], self.pose[1] - ev["y"]),
                            (self.pose[2] - ev["th"] + 180) % 360 - 180)
            if not self.truth_trail or _d2(self.truth_trail[-1], self.truth) > 0.25:
                self.truth_trail.append((ev["x"], ev["y"]))
        elif k == "us":
            self.us = (ev["a"], ev["d"], t)
            if ev["d"] < 250 and self.pose:
                # The robot code often reads the sensor while odometry isn't being updated (e.g. a side glance:
                # the turret swings out and back in ~0.5 s while the robot keeps driving). Place the hit at the
                # pose interpolated to the reading's time once the next pose arrives, not at the stale one.
                self.pending_us.append((ev["a"], ev["d"], t, self.pose, self.pose_t))
        elif k == "map":
            self.street = [tuple(p) for p in ev.get("street", [])]
            self.blocked = [tuple(p) for p in ev.get("blocked", [])]
            self.exit = tuple(ev["exit"]) if ev.get("exit") else self.exit
            self.approach = tuple(ev["approach"]) if ev.get("approach") else self.approach
        elif k == "blocked":
            if tuple(ev["tile"]) not in self.blocked:
                self.blocked.append(tuple(ev["tile"]))
        elif k == "plan":
            self.plan = [tuple(p) for p in ev.get("path", [])]
            self.plan_sweep = bool(ev.get("sweep"))
        elif k == "snap":
            self.snaps.append((ev["axis"], ev["v"], t))
            del self.snaps[:-30]
        elif k == "wall_ahead":
            self.walls.append((tuple(ev["tile"]), t))
        elif k == "glance":
            self.glances.append((tuple(ev["tile"]), ev["side"], ev.get("open"), t, ev.get("d", 255.0), self.pose))
        elif k == "scan":
            self.scan = (ev, t, self.pose)
        elif k == "sim":
            self.sim = ev
        elif k == "pick":
            self.picked.update(ev["i"])
        elif k == "dump":
            self.dumps.append(t)
        elif k == "log":
            self.logs.append((t, ev["msg"]))
            del self.logs[:-200]
        elif k == "end":
            self.end = ev.get("total")

    def _add_hit(self, a, d, t, pose):
        ox, oy, hd = self.us_origin(a, pose)
        self.hits.append((ox + d * math.sin(hd), oy + d * math.cos(hd), t))
        del self.hits[:-800]

    def _place_pending(self, p1, t1):
        for a, d, tu, p0, t0 in self.pending_us:
            f = 0.0 if t1 <= t0 else min(1.0, max(0.0, (tu - t0) / (t1 - t0)))
            dth = (p1[2] - p0[2] + 180.0) % 360.0 - 180.0
            self._add_hit(a, d, tu, (p0[0] + f * (p1[0] - p0[0]), p0[1] + f * (p1[1] - p0[1]), p0[2] + f * dth))
        self.pending_us = []

    def us_origin(self, rel_angle, pose=None):
        x, y, th = pose or self.pose
        fwd = self.meta.get("us_fwd", 8.0)
        face = self.meta.get("us_face", 2.5)
        h = math.radians(th)
        hd = math.radians(th + rel_angle)
        return (x + fwd * math.sin(h) + face * math.sin(hd), y + fwd * math.cos(h) + face * math.cos(hd), hd)


def _d2(a, b):
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2


# ============================================================ drawing backends
class TkDraw:
    def __init__(self, canvas):
        self.c = canvas

    def clear(self):
        self.c.delete("all")

    def rect(self, x0, y0, x1, y1, fill="", outline="", width=1, hatch=False):
        self.c.create_rectangle(x0, y0, x1, y1, fill=fill, outline=outline, width=width,
                                stipple="gray50" if hatch and fill else "")

    def poly(self, pts, fill="", outline="", width=1, dash=None):
        flat = [v for p in pts for v in p]
        self.c.create_polygon(*flat, fill=fill, outline=outline, width=width, dash=dash or "")

    def line(self, pts, color, width=1, dash=None, arrow=False):
        if len(pts) < 2:
            return
        flat = [v for p in pts for v in p]
        self.c.create_line(*flat, fill=color, width=width, dash=dash or "",
                           arrow="last" if arrow else "none", capstyle="round", joinstyle="round")

    def dot(self, x, y, r, fill, outline=""):
        self.c.create_oval(x - r, y - r, x + r, y + r, fill=fill, outline=outline or fill)

    def text(self, x, y, s, color, size=10, anchor="nw", bold=False):
        self.c.create_text(x, y, text=s, fill=color, anchor=anchor,
                           font=("Courier", size, "bold" if bold else "normal"))


class PilDraw:
    """Same calls, onto a Pillow image (for --png and headless tests)."""

    def __init__(self, w, h):
        from PIL import Image, ImageDraw, ImageFont
        self.img = Image.new("RGB", (w, h), C["bg"])
        self.d = ImageDraw.Draw(self.img)
        self._font = {}
        self._IF = ImageFont

    def font(self, size):
        if size not in self._font:
            try:
                self._font[size] = self._IF.truetype("DejaVuSansMono.ttf", size + 2)
            except Exception:
                self._font[size] = self._IF.load_default()
        return self._font[size]

    def clear(self):
        self.d.rectangle([0, 0, self.img.width, self.img.height], fill=C["bg"])

    def rect(self, x0, y0, x1, y1, fill="", outline="", width=1, hatch=False):
        self.d.rectangle([min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)], fill=fill or None,
                         outline=outline or None, width=width)
        if hatch and fill:
            lo, hi = min(x0, x1), max(x0, x1)
            top, bot = min(y0, y1), max(y0, y1)
            self.d.rectangle([lo, top, hi, bot], fill=C["table"])
            step = 7
            for k in range(int(lo - (bot - top)), int(hi), step):
                a = (max(k, lo), top + max(0, lo - k))
                b = (min(k + (bot - top), hi), top + min(bot - top, hi - k))
                self.d.line([a, b], fill=fill, width=2)
            self.d.rectangle([lo, top, hi, bot], outline=fill, width=2)

    def poly(self, pts, fill="", outline="", width=1, dash=None):
        self.d.polygon([tuple(p) for p in pts], fill=fill or None, outline=outline or None, width=width)

    def line(self, pts, color, width=1, dash=None, arrow=False):
        if len(pts) < 2:
            return
        pts = [tuple(p) for p in pts]
        if dash:
            for a, b in zip(pts, pts[1:]):
                L = math.hypot(b[0] - a[0], b[1] - a[1])
                n = max(1, int(L / 10))
                for i in range(0, n, 2):
                    p0 = (a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n)
                    p1 = (a[0] + (b[0] - a[0]) * (i + 1) / n, a[1] + (b[1] - a[1]) * (i + 1) / n)
                    self.d.line([p0, p1], fill=color, width=width)
        else:
            self.d.line(pts, fill=color, width=width, joint="curve")
        if arrow:
            (ax, ay), (bx, by) = pts[-2], pts[-1]
            ang = math.atan2(by - ay, bx - ax)
            s = 4 + 2 * width
            p1 = (bx - s * math.cos(ang - 0.45), by - s * math.sin(ang - 0.45))
            p2 = (bx - s * math.cos(ang + 0.45), by - s * math.sin(ang + 0.45))
            self.d.polygon([(bx, by), p1, p2], fill=color)

    def dot(self, x, y, r, fill, outline=""):
        self.d.ellipse([x - r, y - r, x + r, y + r], fill=fill, outline=outline or fill)

    def text(self, x, y, s, color, size=10, anchor="nw", bold=False):
        a = {"nw": "la", "n": "ma", "ne": "ra", "w": "lm", "center": "mm", "e": "rm",
             "sw": "ld", "s": "md", "se": "rd"}.get(anchor, "la")
        self.d.text((x, y), s, fill=color, font=self.font(size), anchor=a)


# ============================================================ scene
class Scene:
    PANEL_W = 330

    def __init__(self, draw, w, h, mode_label=""):
        self.g = draw
        self.mode_label = mode_label
        self.resize(w, h)

    def set_table(self, tb):
        self.tb = tuple(int(v) for v in tb)
        self.X0, self.X1 = self.tb[0] * T, (self.tb[1] + 1) * T
        self.Y0, self.Y1 = self.tb[2] * T, (self.tb[3] + 1) * T

    def resize(self, w, h):
        if not hasattr(self, "tb"):
            self.set_table(DEFAULT_TABLE)
        self.w, self.h = w, h
        bw = w - self.PANEL_W
        m = 46
        self.s = min((bw - 2 * m) / (self.X1 - self.X0), (h - 2 * m) / (self.Y1 - self.Y0))
        self.ox = m + ((bw - 2 * m) - self.s * (self.X1 - self.X0)) / 2
        self.oy = m + ((h - 2 * m) - self.s * (self.Y1 - self.Y0)) / 2

    def P(self, x, y):
        return self.ox + (x - self.X0) * self.s, self.oy + (self.Y1 - y) * self.s

    def tile_rect(self, t, inset=0.0):
        a = self.P(t[0] * T + inset, (t[1] + 1) * T - inset)
        b = self.P((t[0] + 1) * T - inset, t[1] * T + inset)
        return a[0], a[1], b[0], b[1]

    def tile_c(self, t):
        return self.P((t[0] + 0.5) * T, (t[1] + 0.5) * T)

    # ---------------------------------------------------------- main
    def draw(self, st):
        tb = (getattr(st, "meta", None) or {}).get("table") or (getattr(st, "sim", None) or {}).get("table")
        if tb and tuple(tb) != self.tb:
            self.set_table(tb)
            self.resize(self.w, self.h)
        g = self.g
        g.clear()
        self.draw_board(st)
        self.draw_robot_layers(st)
        self.draw_panel(st)

    def draw_board(self, st):
        g, meta = self.g, st.meta
        X0, X1, Y0, Y1 = self.X0, self.X1, self.Y0, self.Y1
        a, b = self.P(X0, Y1), self.P(X1, Y0)
        # bucket (outside the table) first so the table edge sits on top of it
        bucket = self.bucket_tile(meta)
        if bucket:
            x0, y0, x1, y1 = self.tile_rect(bucket)
            g.rect(x0, y0, x1, y1, fill=C["bucket"], outline=C["drop"], width=2)
            cx, cy = self.tile_c(bucket)
            g.text(cx, cy, "bucket", C["drop"], 9, "center", True)
        g.rect(a[0], a[1], b[0], b[1], fill=C["table"], outline=C["axis"], width=2)
        # true houses (sim) and believed street/houses
        if st.sim:
            for bx in st.sim.get("boxes", []):
                x0, y0, x1, y1 = self.tile_rect(bx)
                g.rect(x0, y0, x1, y1, fill=C["house_true"], outline="")
        for t in st.street:
            x0, y0, x1, y1 = self.tile_rect(t, 1.0)
            g.rect(x0, y0, x1, y1, fill=C["street"], outline="")
        for t in st.blocked:
            x0, y0, x1, y1 = self.tile_rect(t, 2.5)
            g.rect(x0, y0, x1, y1, fill=C["house_seen"], outline=C["house_seen"], width=2, hatch=True)
        for t, col, lab in ((st.exit, C["exit"], "EXIT"), (st.approach, C["approach"], "IN")):
            if t:
                x0, y0, x1, y1 = self.tile_rect(t, 2.0)
                g.rect(x0, y0, x1, y1, outline=col, width=3)
                g.text(x0 + 4, y0 + 3, lab, col, 8, "nw", True)
        if meta.get("dropoff"):
            x0, y0, x1, y1 = self.tile_rect(meta["dropoff"], 2.0)
            g.rect(x0, y0, x1, y1, outline=C["drop"], width=2)
            g.text(x0 + 4, y1 - 3, "DROP", C["drop"], 8, "sw", True)
        # grid
        for i in range(self.tb[0], self.tb[1] + 2):
            g.line([self.P(i * T, Y0), self.P(i * T, Y1)], C["grid"], 2 if i in (self.tb[0], self.tb[1] + 1) else 1)
            g.text(*self.P(i * T, Y0 - 3), str(i), C["axis"], 9, "n")
        for j in range(self.tb[2], self.tb[3] + 2):
            g.line([self.P(X0, j * T), self.P(X1, j * T)], C["grid"], 2 if j in (self.tb[2], self.tb[3] + 1) else 1)
            g.text(*self.P(X0 - 4, j * T), str(j), C["axis"], 9, "e")
        ox, oy = self.P(0, 0)
        g.dot(ox, oy, 5, "#000000")
        g.text(ox + 6, oy + 4, "O", "#000000", 9, "nw", True)
        # gate and drop-off edges
        for k1, k2, col in (("s1", "s2", C["gate"]), ("d1", "d2", C["drop"])):
            if meta.get(k1) and meta.get(k2):
                p1, p2 = self.P(meta[k1][0] * T, meta[k1][1] * T), self.P(meta[k2][0] * T, meta[k2][1] * T)
                g.line([p1, p2], col, 6)
        # pom-poms (sim)
        if st.sim and st.sim.get("pompoms"):
            r = max(1.5, 0.75 * self.s)
            for i, (px, py) in enumerate(st.sim["pompoms"]):
                x, y = self.P(px, py)
                g.dot(x, y, r, C["pom_got"] if i in st.picked else C["pom"])

    def bucket_tile(self, meta):
        d1, d2 = meta.get("d1"), meta.get("d2")
        if not (d1 and d2):
            return None
        if d1[0] == d2[0]:                       # vertical edge
            y = min(d1[1], d2[1])
            return (d1[0] - 1, y) if d1[0] <= self.tb[0] else (d1[0], y)
        x = min(d1[0], d2[0])
        return (x, d1[1] - 1) if d1[1] <= self.tb[2] else (x, d1[1])

    def draw_robot_layers(self, st):
        g = self.g
        if st.plan:
            pts = [self.tile_c(t) for t in st.plan]
            g.line(pts, C["sweep"] if st.plan_sweep else C["plan"], 2, dash=(6, 4), arrow=True)
        for (tile, t0) in st.walls:
            if st.t - t0 < 2.0:
                x0, y0, x1, y1 = self.tile_rect(tile, 4)
                g.rect(x0, y0, x1, y1, outline=C["wall"], width=3)
        for (tile, side, is_open, t0, _d, _p) in st.glances[-40:]:
            cx, cy = self.tile_c(tile)
            g.dot(cx, cy, 3, C["ok"] if is_open else C["dim"])
        # ultrasonic hits: old = pale, last 3 s = strong
        for (hx, hy, t0) in st.hits[-400:]:
            x, y = self.P(hx, hy)
            g.dot(x, y, 2, C["hit"] if st.t - t0 < 3.0 else C["hit_old"])
        for (axis, v, t0) in st.snaps:
            if st.t - t0 < 1.2 and st.pose:
                px, py, _ = st.pose
                if axis == "y":
                    g.line([self.P(px - 14, v), self.P(px + 14, v)], C["snap"], 5)
                else:
                    g.line([self.P(v, py - 14), self.P(v, py + 14)], C["snap"], 5)
        # stationary scan: three rays for 2.5 s; side glance: one ray for 1 s
        rays = []
        if st.scan and st.t - st.scan[1] < 2.5 and st.scan[2]:
            ev, _, pose = st.scan
            rays += [(pose, a, ev[k], k) for a, k in ((-90, "L"), (0, "F"), (90, "R")) if k in ev]   # only measured
        if st.glances and st.t - st.glances[-1][3] < 1.0 and st.glances[-1][5]:
            _, side, _, _, d, pose = st.glances[-1]
            rays.append((pose, 90 * side, d, "glance"))
        for pose, a, d, lab in rays:
            ox, oy, hd = st.us_origin(a, pose)
            L = min(d, 60.0)
            p0 = self.P(ox, oy)
            p1 = self.P(ox + L * math.sin(hd), oy + L * math.cos(hd))
            col = C["wall"] if d < 25 else C["ok"]
            g.line([p0, p1], col, 3, dash=None if d < 250 else (4, 3))
            g.text(p1[0], p1[1], f"{lab} {d:.0f}" if d < 250 else f"{lab} open", col, 8, "center", True)
        if st.truth_trail:
            g.line([self.P(*p) for p in st.truth_trail], C["truth"], 1)
        if st.trail:
            g.line([self.P(*p) for p in st.trail], C["odom"], 2)
        outline = st.meta.get("outline") or (st.sim or {}).get("outline") or [(-8, -9), (8, -9), (8, 10), (-8, 10)]
        sim_outline = (st.sim or {}).get("outline") or outline
        if st.truth:
            self.draw_robot(st.truth, sim_outline, C["truth"], fill="")
        if st.pose:
            self.draw_robot(st.pose, outline, C["odom"], fill="")
            if st.us and st.t - st.us[2] < 0.6:
                a, d, _ = st.us
                ox, oy, hd = st.us_origin(a)
                L = min(d, 120.0)
                g.line([self.P(ox, oy), self.P(ox + L * math.sin(hd), oy + L * math.cos(hd))],
                       C["us"], 2, dash=None if d < 250 else (3, 3))

    def draw_robot(self, pose, outline, color, fill=""):
        x, y, th = pose
        s, c = math.sin(math.radians(th)), math.cos(math.radians(th))

        def W(lat, fwd):
            return self.P(x + fwd * s + lat * c, y + fwd * c - lat * s)

        self.g.poly([W(*p) for p in outline], fill=fill, outline=color, width=2)
        self.g.line([W(0, 0), W(0, 14)], color, 2, arrow=True)
        cx, cy = W(0, 0)
        self.g.dot(cx, cy, 2.5, color)

    # ---------------------------------------------------------- panel
    def draw_panel(self, st):
        g = self.g
        x0 = self.w - self.PANEL_W
        g.rect(x0, 0, self.w, self.h, fill=C["panel"], outline="")
        x, y = x0 + 14, 12
        ph = st.phase if st.phase is not None else "-"
        g.text(x, y, f"PHASE {ph}", C["text"], 15, "nw", True)
        y += 24
        g.text(x, y, self.mode_label[:46], C["dim"], 8, "nw")
        y += 18
        # clock
        t = st.end if st.end is not None else st.t
        frac = min(1.0, t / PHASE_LIMIT)
        col = C["ok"] if t < 95 else (C["warn"] if t < PHASE_LIMIT else C["bad"])
        bw = self.PANEL_W - 28
        g.rect(x, y, x + bw, y + 16, fill="#3a3d45", outline="")
        g.rect(x, y, x + bw * frac, y + 16, fill=col, outline="")
        g.text(x + bw / 2, y + 8, f"{t:5.1f} / {PHASE_LIMIT:.0f} s" + ("  DONE" if st.end is not None else ""),
               "#ffffff", 10, "center", True)
        y += 26
        rows = []
        if st.pose:
            px, py, pt = st.pose
            rows.append(("odometry", f"x {px:6.1f}  y {py:6.1f}  th {pt:5.1f}", C["odom"]))
            rows.append(("tile", f"({int(math.floor(px / T))}, {int(math.floor(py / T))})", C["text"]))
        if st.truth:
            tx, ty, tt = st.truth
            rows.append(("truth (sim)", f"x {tx:6.1f}  y {ty:6.1f}  th {tt:5.1f}", C["truth"]))
            if st.err:
                e, eth = st.err
                rows.append(("pose error", f"{e:4.1f} cm  {eth:+4.1f} deg", C["warn"] if e > 3 else C["ok"]))
        if st.us:
            a, d, _ = st.us
            rows.append(("ultrasonic", f"{d:5.1f} cm @ {a:+.0f} deg", C["us"]))
        rows.append(("street", f"{len(st.street)} tiles   houses {len(st.blocked)}", C["text"]))
        if st.sim and st.sim.get("pompoms"):
            n = len(st.sim["pompoms"])
            rows.append(("pom-poms", f"{len(st.picked)}/{n} under the intake", C["pom"]))
        if st.dumps:
            rows.append(("dump", f"at {st.dumps[-1]:.1f} s", C["drop"]))
        for lab, val, col in rows:
            g.text(x, y, f"{lab:<11}", C["dim"], 9)
            g.text(x + 92, y, val, col, 9)
            y += 17
        y += 6
        g.line([(x, y), (x + bw, y)], "#444851", 1)
        y += 6
        # legend
        legend = [(C["street"], "street (robot's map)"), (C["house_seen"], "house (robot's map)"),
                  (C["house_true"], "true house (sim)"), (C["odom"], "odometry pose/trail"),
                  (C["truth"], "true pose (sim)"), (C["hit"], "ultrasonic hits"),
                  (C["snap"], "line snap"), (C["plan"], "planned path")]
        for i, (col, lab) in enumerate(legend):
            lx = x + (i % 2) * (bw / 2)
            ly = y + (i // 2) * 15
            g.rect(lx, ly + 3, lx + 10, ly + 12, fill=col, outline="")
            g.text(lx + 15, ly + 1, lab, C["dim"], 8)
        y += 4 * 15 + 8
        g.line([(x, y), (x + bw, y)], "#444851", 1)
        y += 6
        # console tail
        line_h = 14
        n = max(0, int((self.h - y - 8) / line_h))
        maxc = int((self.PANEL_W - 28) / 6.6)
        for (lt, msg) in st.logs[-n:] if n else []:
            s = msg.strip()
            col = C["text"]
            if "WATCHDOG" in s or "TERMINATED" in s or "Error" in s:
                col = C["bad"]
            elif "OBSTACLE" in s or "CORNER" in s or "street ended" in s:
                col = C["warn"]
            g.text(x, y, (f"{lt:5.1f} " + s)[:maxc], col, 8)
            y += line_h


# ============================================================ event sources
def load_events(paths):
    """Reads one or more .jsonl logs; returns [(timeline_t, event)] with phases back to back."""
    files = []
    for p in paths:
        if os.path.isdir(p):
            ph1 = sorted(glob.glob(os.path.join(p, "phase1_*.jsonl")))
            ph2 = sorted(glob.glob(os.path.join(p, "phase2_*.jsonl")))
            files += ph1[-1:] + ph2[-1:]
        else:
            files.append(p)
    out, offset = [], 0.0
    for f in files:
        last = 0.0
        with open(f) as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                try:
                    ev = json.loads(line)
                except ValueError:
                    continue
                last = max(last, float(ev.get("t", 0.0)))
                out.append((offset + float(ev.get("t", 0.0)), ev))
        offset += last + 2.0
    return out, files


class UdpSource:
    def __init__(self, port, bind="0.0.0.0", save_dir=None):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((bind, port))
        self.sock.setblocking(False)
        self.port = self.sock.getsockname()[1]
        self.save = None
        self.peer = None
        self.count = 0
        if save_dir:
            os.makedirs(save_dir, exist_ok=True)
            self.save_path = os.path.join(save_dir, time.strftime("live_%Y%m%d_%H%M%S.jsonl"))
            self.save = open(self.save_path, "w")

    def poll(self, limit=5000):
        evs = []
        for _ in range(limit):
            try:
                data, self.peer = self.sock.recvfrom(65535)
            except (BlockingIOError, InterruptedError):
                break
            except OSError:
                break
            try:
                ev = json.loads(data.decode())
            except ValueError:
                continue
            evs.append(ev)
            self.count += 1
            if self.save:
                self.save.write(data.decode() + "\n")
        if evs and self.save:
            self.save.flush()
        return evs

    def close(self):
        try:
            self.sock.close()
            if self.save:
                self.save.close()
        except Exception:
            pass


# ============================================================ player (shared by Tk and web)
class Player:
    """Event source (UDP, simulator or replay file) plus the playhead and the State it builds."""

    def __init__(self, args):
        self.args = args
        self.state = State()
        self.src = None
        self.proc = None
        self.events = []
        self.cursor = 0
        self.playhead = 0.0
        self.t_end = 0.0
        self.playing = True
        self.speed = 1.0
        self.dirty = True
        self.replay = bool(args.replay)
        if args.replay:
            self.events, files = load_events(args.replay)
            if not self.events:
                raise SystemExit(f"No events in {args.replay}")
            self.t_end = self.events[-1][0]
            self.mode_label = "REPLAY " + " + ".join(os.path.basename(f) for f in files)[:40]
            self.speed = args.speed
        else:
            save_dir = None if args.no_save else os.path.join(HERE, "logs")
            if args.sim:
                self.src = UdpSource(0, "127.0.0.1", save_dir=None)
                cmd = [sys.executable, os.path.join(HERE, "sim", "run_sim.py"), "--map", args.sim,
                       "--seeds", "1", "--seed-start", str(args.seed), "--noise", str(args.noise),
                       "--viz-udp", f"127.0.0.1:{self.src.port}", "--realtime", str(args.speed)]
                if args.explore:
                    cmd += ["--explore", args.explore]
                self.proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL if not args.verbose else None)
                self.mode_label = f"SIM {args.sim} seed {args.seed} x{args.speed:g}"
            else:
                self.src = UdpSource(args.port, save_dir=save_dir)
                self.mode_label = f"LIVE udp:{self.src.port}"

    # ---- replay controls
    def toggle(self):
        self.playing = not self.playing

    def set_speed(self, s):
        self.speed = max(0.125, min(32.0, float(s)))

    def restart(self):
        self.seek(0.0)

    def seek(self, t):
        if not self.replay:
            return
        t = max(0.0, min(self.t_end, float(t)))
        if t < self.playhead:
            self.state = State()
            self.cursor = 0
        self.playhead = t
        self.advance_to(t)

    def advance_to(self, t):
        while self.cursor < len(self.events) and self.events[self.cursor][0] <= t:
            self.state.apply(self.events[self.cursor][1])
            self.cursor += 1
        self.dirty = True

    def tick(self, dt):
        """Pulls new events / moves the playhead. Returns a status line."""
        if self.src is not None:
            evs = self.src.poll()
            for ev in evs:
                self.state.apply(ev)
            if evs:
                self.dirty = True
            if self.proc is not None and self.proc.poll() is not None:
                return f"sim finished (exit {self.proc.returncode})"
            if self.src.count == 0:
                return f"waiting for the robot (UDP port {self.src.port}) ..."
            txt = f"{self.src.count} events from {self.src.peer[0] if self.src.peer else '?'}"
            if self.src.save:
                txt += f"  saving {os.path.basename(self.src.save_path)}"
            return txt
        if self.playing and self.playhead < self.t_end:
            self.playhead = min(self.t_end, self.playhead + dt * self.speed)
            self.advance_to(self.playhead)
        return f"{self.playhead:6.1f} / {self.t_end:.1f} s   {self.speed:g}x"

    def close(self):
        if self.proc is not None and self.proc.poll() is None:
            self.proc.terminate()
        if self.src is not None:
            self.src.close()


# ============================================================ Tk app
class App:
    def __init__(self, args):
        import tkinter as tk
        self.args = args
        self.p = Player(args)
        self.root = tk.Tk()
        self.root.title("Snowplow visualizer")
        self.root.configure(bg=C["bg"])
        w, h = args.size
        self.canvas = tk.Canvas(self.root, width=w, height=h, bg=C["bg"], highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        self.scene = Scene(TkDraw(self.canvas), w, h, self.p.mode_label)
        self.canvas.bind("<Configure>", self.on_resize)
        bar = tk.Frame(self.root, bg=C["panel"])
        bar.pack(fill="x")
        self.status = tk.StringVar(value="")
        self.last_tick = time.time()
        if self.p.replay:
            mk = dict(bg=C["panel"], fg=C["text"], activebackground="#3a3d45", relief="flat", bd=0, padx=8)
            tk.Button(bar, text="|<", command=self.p.restart, **mk).pack(side="left")
            self.play_btn = tk.Button(bar, text="pause", width=6, command=self.toggle, **mk)
            self.play_btn.pack(side="left")
            for sp in (0.5, 1, 2, 4, 8):
                tk.Button(bar, text=f"{sp:g}x", command=lambda s=sp: self.p.set_speed(s), **mk).pack(side="left")
            self.slider = tk.Scale(bar, from_=0, to=self.p.t_end, orient="horizontal", resolution=0.1,
                                   showvalue=False, command=self.on_slider, bg=C["panel"], fg=C["text"],
                                   troughcolor="#3a3d45", highlightthickness=0, length=400)
            self.slider.pack(side="left", fill="x", expand=True, padx=8)
            self.root.bind("<space>", lambda e: self.toggle())
            self.root.bind("<Left>", lambda e: self.p.seek(self.p.playhead - 2))
            self.root.bind("<Right>", lambda e: self.p.seek(self.p.playhead + 2))
            self.root.bind("<Up>", lambda e: self.p.set_speed(self.p.speed * 2))
            self.root.bind("<Down>", lambda e: self.p.set_speed(self.p.speed / 2))
            self.root.bind("<Home>", lambda e: self.p.restart())
        tk.Label(bar, textvariable=self.status, bg=C["panel"], fg=C["dim"], anchor="e").pack(side="right", padx=8)
        self.root.protocol("WM_DELETE_WINDOW", self.quit)
        self.root.after(30, self.tick)

    def toggle(self):
        self.p.toggle()
        self.play_btn.config(text="pause" if self.p.playing else "play")

    def on_slider(self, v):
        v = float(v)
        if abs(v - self.p.playhead) > 0.15:
            self.p.seek(v)

    def tick(self):
        now = time.time()
        dt, self.last_tick = now - self.last_tick, now
        self.status.set(self.p.tick(dt))
        if self.p.replay:
            self.slider.set(self.p.playhead)
        if self.p.dirty:
            self.scene.draw(self.p.state)
            self.p.dirty = False
        self.root.after(max(20, int(1000 / self.args.fps)), self.tick)

    def on_resize(self, e):
        self.scene.resize(e.width, e.height)
        self.p.dirty = True

    def quit(self):
        self.p.close()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


# ============================================================ web viewer (no display needed on the Pi)
class CmdDraw:
    """Records the Scene's drawing calls as a compact list; the browser page replays them on a canvas."""

    def __init__(self):
        self.cmds = []

    @staticmethod
    def _f(pts):
        return [round(v, 1) for p in pts for v in p]

    def clear(self):
        self.cmds = []

    def rect(self, x0, y0, x1, y1, fill="", outline="", width=1, hatch=False):
        self.cmds.append(["r", round(x0, 1), round(y0, 1), round(x1, 1), round(y1, 1), fill, outline, width, int(hatch)])

    def poly(self, pts, fill="", outline="", width=1, dash=None):
        self.cmds.append(["p", self._f(pts), fill, outline, width, list(dash or [])])

    def line(self, pts, color, width=1, dash=None, arrow=False):
        if len(pts) >= 2:
            self.cmds.append(["l", self._f(pts), color, width, list(dash or []), int(arrow)])

    def dot(self, x, y, r, fill, outline=""):
        self.cmds.append(["d", round(x, 1), round(y, 1), round(r, 1), fill, outline or fill])

    def text(self, x, y, s, color, size=10, anchor="nw", bold=False):
        self.cmds.append(["t", round(x, 1), round(y, 1), str(s), color, size, anchor, int(bold)])


WEB_PAGE = r"""<!doctype html><html><head><meta charset="utf-8"><title>Snowplow visualizer</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
html,body{margin:0;height:100%;background:#1e1f24;color:#e8e8e8;font:13px system-ui,sans-serif;overflow:hidden}
#c{display:block;width:100vw;height:calc(100vh - 34px)}
#bar{height:34px;display:flex;align-items:center;gap:6px;padding:0 8px;background:#26282e}
button{background:#3a3d45;color:#e8e8e8;border:0;border-radius:4px;padding:4px 9px;cursor:pointer}
button.on{background:#2f6fd6}#seek{flex:1}#st{margin-left:auto;color:#9aa0a6;white-space:nowrap}
#rp{display:none;align-items:center;gap:6px;flex:1}
</style></head><body><canvas id="c"></canvas>
<div id="bar"><span id="rp"><button id="rs">|&lt;</button><button id="pp">pause</button>
<span id="sp"></span><input id="seek" type="range" min="0" max="1" step="0.1" value="0"></span>
<span id="st">connecting...</span></div>
<script>
const cv=document.getElementById('c'),g=cv.getContext('2d');let n=-1,last=null,dragging=false,fails=0;
const ctl=(q)=>fetch('/ctl?'+q).catch(()=>{});
function size(){const d=window.devicePixelRatio||1;cv.width=cv.clientWidth*d;cv.height=cv.clientHeight*d;
  g.setTransform(d,0,0,d,0,0);ctl('cmd=size&w='+cv.clientWidth+'&h='+cv.clientHeight);if(last)paint(last);}
window.addEventListener('resize',size);
const AN={nw:['left','top'],n:['center','top'],ne:['right','top'],w:['left','middle'],center:['center','middle'],
  e:['right','middle'],sw:['left','bottom'],s:['center','bottom'],se:['right','bottom']};
function path(f){g.beginPath();g.moveTo(f[0],f[1]);for(let i=2;i<f.length;i+=2)g.lineTo(f[i],f[i+1]);}
function paint(fr){g.clearRect(0,0,cv.width,cv.height);g.lineCap='round';g.lineJoin='round';
 for(const c of fr.cmds){const k=c[0];
  if(k==='r'){const[_,x0,y0,x1,y1,fill,ol,w,h]=c;const x=Math.min(x0,x1),y=Math.min(y0,y1),W=Math.abs(x1-x0),H=Math.abs(y1-y0);
    if(fill){g.globalAlpha=h?0.45:1;g.fillStyle=fill;g.fillRect(x,y,W,H);g.globalAlpha=1;}
    if(ol){g.setLineDash([]);g.strokeStyle=ol;g.lineWidth=w;g.strokeRect(x,y,W,H);}}
  else if(k==='p'){const[_,f,fill,ol,w,dash]=c;path(f);g.closePath();if(fill){g.fillStyle=fill;g.fill();}
    if(ol){g.setLineDash(dash);g.strokeStyle=ol;g.lineWidth=w;g.stroke();g.setLineDash([]);}}
  else if(k==='l'){const[_,f,col,w,dash,arrow]=c;path(f);g.setLineDash(dash);g.strokeStyle=col;g.lineWidth=w;g.stroke();g.setLineDash([]);
    if(arrow){const m=f.length,ax=f[m-4],ay=f[m-3],bx=f[m-2],by=f[m-1],a=Math.atan2(by-ay,bx-ax),s=4+2*w;
      g.beginPath();g.moveTo(bx,by);g.lineTo(bx-s*Math.cos(a-0.45),by-s*Math.sin(a-0.45));g.lineTo(bx-s*Math.cos(a+0.45),by-s*Math.sin(a+0.45));
      g.closePath();g.fillStyle=col;g.fill();}}
  else if(k==='d'){const[_,x,y,r,fill,ol]=c;g.beginPath();g.arc(x,y,r,0,7);g.fillStyle=fill;g.fill();if(ol&&ol!==fill){g.strokeStyle=ol;g.lineWidth=1;g.stroke();}}
  else if(k==='t'){const[_,x,y,s,col,sz,an,b]=c;const a=AN[an]||AN.nw;g.font=(b?'bold ':'')+Math.round(sz*1.3)+'px monospace';
    g.textAlign=a[0];g.textBaseline=a[1];g.fillStyle=col;g.fillText(s,x,y);}}}
const SPEEDS=[0.5,1,2,4,8];document.getElementById('sp').innerHTML=SPEEDS.map(s=>`<button data-s="${s}">${s}x</button>`).join('');
document.getElementById('sp').onclick=e=>{if(e.target.dataset.s)ctl('cmd=speed&v='+e.target.dataset.s);};
document.getElementById('rs').onclick=()=>ctl('cmd=restart');document.getElementById('pp').onclick=()=>ctl('cmd=toggle');
const sk=document.getElementById('seek');sk.oninput=()=>{dragging=true;ctl('cmd=seek&v='+sk.value);};sk.onchange=()=>{dragging=false;};
document.addEventListener('keydown',e=>{if(!last||!last.replay)return;const r=last.replay;
 if(e.code==='Space'){ctl('cmd=toggle');e.preventDefault();}else if(e.code==='ArrowLeft')ctl('cmd=seek&v='+(r.t-2));
 else if(e.code==='ArrowRight')ctl('cmd=seek&v='+(r.t+2));else if(e.code==='ArrowUp')ctl('cmd=speed&v='+r.speed*2);
 else if(e.code==='ArrowDown')ctl('cmd=speed&v='+r.speed/2);else if(e.code==='Home')ctl('cmd=restart');});
async function poll(){try{const r=await fetch('/frame?n='+n,{cache:'no-store'});fails=0;
  if(r.status===200){const fr=await r.json();n=fr.n;last=fr;paint(fr);
   document.getElementById('st').textContent=fr.status;const rp=fr.replay;document.getElementById('rp').style.display=rp?'flex':'none';
   if(rp){document.getElementById('pp').textContent=rp.playing?'pause':'play';sk.max=rp.end;if(!dragging)sk.value=rp.t;
    document.querySelectorAll('[data-s]').forEach(b=>b.classList.toggle('on',+b.dataset.s===rp.speed));}}}
 catch(e){if(++fails>3)document.getElementById('st').textContent='viewer not reachable (is visualizer.py --web running?)';}
 setTimeout(poll,120);}
size();poll();
</script></body></html>"""


def local_addresses():
    """IPv4 addresses other machines can use to reach this one (best effort)."""
    ips = set()
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))          # picks the outgoing interface; sends nothing
        ips.add(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    try:
        out = subprocess.run(["hostname", "-I"], capture_output=True, text=True, timeout=1.0).stdout.split()
        ips.update(a for a in out if a.count(".") == 3)
    except Exception:
        pass
    ips.discard("127.0.0.1")
    return sorted(ips)


def run_web(args):
    import threading
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
    from urllib.parse import parse_qs, urlparse
    try:
        os.nice(10)                         # on the robot, the control loop comes first
    except (AttributeError, OSError):
        pass
    p = Player(args)
    draw = CmdDraw()
    w, h = args.size
    scene = Scene(draw, w, h, p.mode_label)
    shared = {"frame": b"", "n": 0}
    ctl = []

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, body=b"", ctype="application/json"):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            if body:
                self.wfile.write(body)

        def do_GET(self):
            u = urlparse(self.path)
            q = {k: v[0] for k, v in parse_qs(u.query).items()}
            try:
                if u.path == "/":
                    self._send(200, WEB_PAGE.encode(), "text/html; charset=utf-8")
                elif u.path == "/frame":
                    if q.get("n") == str(shared["n"]) or not shared["frame"]:
                        self._send(204)
                    else:
                        self._send(200, shared["frame"])
                elif u.path == "/ctl":
                    ctl.append(q)
                    self._send(200, b"{}")
                else:
                    self._send(404, b"{}")
            except (BrokenPipeError, ConnectionResetError):
                pass

    server = ThreadingHTTPServer(("0.0.0.0", args.web), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    urls = [f"http://{ip}:{args.web}" for ip in local_addresses()] + [f"http://{socket.gethostname()}.local:{args.web}"]
    print(f"[VIZ] {p.mode_label}: open " + "  or  ".join(urls) + f"  (this machine: http://localhost:{args.web})",
          flush=True)

    last = time.time()
    last_draw = 0.0
    status = ""
    try:
        while True:
            while ctl:
                q = ctl.pop(0)
                cmd = q.get("cmd")
                try:
                    if cmd == "size":
                        w, h = max(300, int(float(q["w"]))), max(200, int(float(q["h"])))
                        scene.resize(w, h)
                        p.dirty = True
                    elif cmd == "toggle":
                        p.toggle()
                    elif cmd == "speed":
                        p.set_speed(float(q["v"]))
                    elif cmd == "seek":
                        p.seek(float(q["v"]))
                    elif cmd == "restart":
                        p.restart()
                    p.dirty = True
                except (KeyError, ValueError):
                    pass
            now = time.time()
            status = p.tick(now - last)
            last = now
            if p.dirty or now - last_draw > 1.0:
                scene.draw(p.state)
                p.dirty = False
                last_draw = now
                shared["n"] += 1
                fr = {"n": shared["n"], "cmds": draw.cmds, "status": status,
                      "replay": {"t": round(p.playhead, 2), "end": round(p.t_end, 2), "speed": p.speed,
                                 "playing": p.playing} if p.replay else None}
                shared["frame"] = json.dumps(fr, separators=(",", ":")).encode()
            time.sleep(1.0 / args.fps)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        p.close()


def render_png(args):
    events, files = load_events(args.replay)
    st = State()
    for t, ev in events:
        if args.at is not None and t > args.at:
            break
        st.apply(ev)
    w, h = args.size
    d = PilDraw(w, h)
    sc = Scene(d, w, h, "REPLAY " + " + ".join(os.path.basename(f) for f in files)[:40])
    sc.draw(st)
    d.img.save(args.png)
    print(f"wrote {args.png} ({len(events)} events, t={st.t:.1f}s)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--live", action="store_true", help="listen for the robot over UDP (default)")
    src.add_argument("--replay", nargs="+", metavar="LOG", help=".jsonl log file(s) or a logs/ directory")
    src.add_argument("--sim", metavar="MAP", help="run the simulator on MAP and show it")
    ap.add_argument("--port", type=int, default=5005)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--noise", type=float, default=1.0)
    ap.add_argument("--explore", choices=["forward", "side"])
    ap.add_argument("--speed", type=float, default=1.0, help="sim pacing / replay speed")
    ap.add_argument("--fps", type=float, default=None, help="redraw rate (default 12, or 6 with --web)")
    ap.add_argument("--web", type=int, nargs="?", const=8080, metavar="PORT",
                    help="serve the view as a web page (default port 8080) instead of opening a window; "
                         "run it on the Pi and open http://<pi>:8080 on any laptop")
    ap.add_argument("--size", type=int, nargs=2, default=(1180, 640), metavar=("W", "H"))
    ap.add_argument("--no-save", action="store_true", help="live mode: don't save received events")
    ap.add_argument("--png", help="with --replay: render to this PNG instead of opening a window")
    ap.add_argument("--at", type=float, help="with --png: time (s on the replay timeline) to render")
    ap.add_argument("-v", "--verbose", action="store_true", help="sim mode: show the robot console here too")
    args = ap.parse_args()
    if args.png:
        if not args.replay:
            ap.error("--png needs --replay")
        render_png(args)
        return
    if args.fps is None:
        args.fps = 6.0 if args.web else 12.0
    if args.web:
        run_web(args)
        return
    App(args).run()


if __name__ == "__main__":
    main()
