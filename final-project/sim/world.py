"""
Simulated board + robot for testing the snowplow code on a laptop.

What it models:
  - Differential drive with first-order motor lag, slightly wrong wheel radii /
    track width (vs. the code's constants) and random wheel slip.
  - Two downward colour sensors over 30.48 cm grid lines (and the table edge).
  - Ultrasonic sensor on a turret (port A position) or fixed, ray-cast against the
    houses with a small beam cone, grazing-angle rejection, noise, dropouts and short ghost echoes.
  - Every brick call costs a little time (SPI), so loop rates are realistic.
  - Robot footprint vs. houses (minimum clearance, contacts), wheels off the table,
    and which pom-poms pass under the intake while the roller is on.

What it does NOT model: pom-pom physics/catch rate, motor stalls, real sensor quirks.
Treat its times as optimistic-but-close and its clearances as a sanity check.
"""
from __future__ import annotations
import ast
import math
import os
import random
import time

_real_time, _real_sleep = time.time, time.sleep   # captured before run_sim patches the time module

T = 30.48

# Houses and lines as on the lab table (photo 2026-11-22): houses are cardboard boxes (10 in = 25.4 cm on a 30.48 cm
# tile), not always centred or square to the grid; the grid lines are ~0.55 cm wide. Set from the environment so
# stress.py / run_sim.py can switch them: SNOWPLOW_SIM_HOUSES = "SIZE_CM,JITTER_CM,ROT_DEG" (or "tile" = every
# house fills its whole tile, the old model), SNOWPLOW_SIM_LINE_CM = line width.
def _env_floats(name, default):
    v = os.environ.get(name, "").strip()
    if not v:
        return default
    if v == "tile":
        return None
    return tuple(float(a) for a in v.split(","))


HOUSES_DEFAULT = None           # None = full tiles (every house edge on its line: the worst case that still follows
                                # the rules); (size, jitter, rotation) = boxes. stress.py --houses box = the lab table.
LINE_CM_DEFAULT = 0.55          # measured on the lab table photo (was 0.9)


def _robot_wiring(path=os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "snowplow_core.py")):
    """Ports, drive polarity and nominal geometry straight from snowplow_core.py (not imported: run_sim imports it
    only after the world exists), so the sim follows the robot's wiring."""
    want = {"LEFT_DRIVE_PORT": "B", "RIGHT_DRIVE_PORT": "C", "TURRET_PORT": "A", "ROLLER_MOTOR_PORT": "D",
            "LEFT_COLOR_PORT": 4, "RIGHT_COLOR_PORT": 2, "ULTRASONIC_PORT": 3, "DRIVE_POLARITY": -1, "SENSOR_SPACING": 12.0,
            "TRACK_WIDTH": 12.0, "WHEEL_RADIUS": 2.10}
    try:
        for node in ast.parse(open(path).read()).body:
            if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name) \
                    and node.targets[0].id in want:
                try:
                    want[node.targets[0].id] = ast.literal_eval(node.value)
                except ValueError:
                    pass
    except (OSError, SyntaxError):
        pass
    return want


WIRING = _robot_wiring()
POWER_TO_DPS = 10.0     # unloaded motor: ~10 dps per % power (EV3 Large ~1000 dps at 100 %)
TABLE = (-1 * T, 8 * T, -1 * T, 4 * T)  # xmin, xmax, ymin, ymax (cm); one table. World(table=...) changes it

# Footprints in robot frame as (lateral +right, forward) points, cm.
FOOTPRINT_P1 = [(-8, -9), (8, -9), (8, 10.5), (-8, 10.5)]
# Phase 2: square full-width intake (24 cm roller + side plates = 25 cm), front 11.6 cm ahead of the axle.
FOOTPRINT_P2_SQUARE = [(-8, -9), (8, -9), (8, 3), (12.5, 3), (12.5, 11.6), (-12.5, 11.6), (-12.5, 3), (-8, 3)]
# Old assumption: 5 cm front chamfers (needs a ~14 cm roller; kept for comparison runs).
FOOTPRINT_P2_CHAMFER = [(-8, -9), (8, -9), (8, 3), (12, 3), (12, 6.6), (7, 11.6),
                        (-7, 11.6), (-12, 6.6), (-12, 3), (-8, 3)]
# Current build (2026-11-20 decision): pinch belt behind a card V-funnel, everything inside a 15 cm circle around
# the axle and no wider than 24 cm; funnel tips at (+-12, 9), throat 4.5 cm wide at the belt entry (1 cm ahead).
def _funnel_footprint(r=15.0, half_w=12.0, reach=9.0, throat=2.25, throat_fwd=1.0):
    import math as _m
    yb = -_m.sqrt(r * r - half_w * half_w)
    a0 = 180 - int(_m.degrees(_m.asin(half_w / r)))
    rear = [(r * _m.sin(_m.radians(a)), r * _m.cos(_m.radians(a))) for a in range(a0, 360 - a0 + 1, 5)]
    return [(half_w, reach), (half_w, yb)] + rear + [(-half_w, yb), (-half_w, reach),
                                                     (-throat, throat_fwd), (throat, throat_fwd)]


FOOTPRINT_P2_FUNNEL = [(round(a, 2), round(b, 2)) for a, b in _funnel_footprint()]
INTAKES = {   # name: (footprint, catch half-width, catch line ahead of the axle)
    "funnel": (FOOTPRINT_P2_FUNNEL, 12.0, 9.0),
    "square": (FOOTPRINT_P2_SQUARE, 12.0, 11.6),
    "chamfer": (FOOTPRINT_P2_CHAMFER, 12.0, 11.6),
}
FOOTPRINT_P2 = FOOTPRINT_P2_FUNNEL
INTAKE_HALF_WIDTH = 12.0     # cm, funnel mouth +-12
INTAKE_REACH = 9.0           # cm ahead of axle (funnel tips)


def use_intake(name):
    """Sets the Phase 2 footprint and catch line for every World made afterwards."""
    global FOOTPRINT_P2, INTAKE_HALF_WIDTH, INTAKE_REACH
    FOOTPRINT_P2, INTAKE_HALF_WIDTH, INTAKE_REACH = INTAKES[name]
POMPOM_RADIUS = 0.7          # cm (~1.4 cm test pom-poms)
POMPOM_WALL_GAP = 2.0        # cm, rule: pom-poms are at least this far from a house


def _densify(poly, step=0.5):
    pts = []
    for (a, b) in zip(poly, poly[1:] + poly[:1]):
        n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / step))
        for i in range(n):
            pts.append((a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n))
    return pts


class MotorState:
    def __init__(self):
        self.mode = "power"   # "power", "dps", "position"
        self.cmd = 0.0
        self.vel = 0.0
        self.pos = 0.0
        self.target = 0.0
        self.limit_dps = 0.0


class World:
    def __init__(self, boxes, seed=0, noise=1.0, call_cost=0.0008,
                 sensor_offset=7.0, sensor_spacing=None, us_forward=8.0, us_face=2.5,
                 wheel_radius=None, track_width=None, phase2_side_us=None, dark_pompoms=False, table=None):
        """table: (x_min, x_max, y_min, y_max) in tiles (sim/maps.py), default one 9 x 5 table."""
        self.rng = random.Random(seed)
        self.peek_rng = random.Random(seed + 7919)    # peek=True reads (dashboard graphs): never shift self.rng
        sensor_spacing = float(WIRING["SENSOR_SPACING"]) if sensor_spacing is None else sensor_spacing
        wheel_radius = float(WIRING["WHEEL_RADIUS"]) if wheel_radius is None else wheel_radius
        track_width = float(WIRING["TRACK_WIDTH"]) if track_width is None else track_width
        self.port_l, self.port_r = WIRING["LEFT_DRIVE_PORT"], WIRING["RIGHT_DRIVE_PORT"]
        self.port_a, self.port_belt = WIRING["TURRET_PORT"], WIRING["ROLLER_MOTOR_PORT"]
        self.polarity = int(WIRING["DRIVE_POLARITY"])
        self.table = TABLE if table is None else (table[0] * T, (table[1] + 1) * T, table[2] * T, (table[3] + 1) * T)
        self.boxes = set(boxes)
        # Each house's footprint: centre, half size, rotation (rad). Its own random stream: the robot's noise
        # (self.rng) stays the same whichever house model is used.
        hrng = random.Random(f"houses-{seed}")
        houses = _env_floats("SNOWPLOW_SIM_HOUSES", HOUSES_DEFAULT)
        self.house = {}
        for b in sorted(self.boxes):
            cx, cy = (b[0] + 0.5) * T, (b[1] + 0.5) * T
            if houses is None:
                self.house[b] = (cx, cy, T / 2.0, 0.0)
            else:
                size, jitter, rot = houses
                self.house[b] = (cx + hrng.uniform(-jitter, jitter), cy + hrng.uniform(-jitter, jitter), size / 2.0,
                                 math.radians(hrng.uniform(-rot, rot)))
        self.line_half = (_env_floats("SNOWPLOW_SIM_LINE_CM", (LINE_CM_DEFAULT,)) or (LINE_CM_DEFAULT,))[0] / 2.0
        self.noise = noise
        self.call_cost = call_cost
        self.t = 0.0
        self.x = self.y = self.th = 0.0
        self.phase = 1
        # True physical parameters (slightly different from the code's nominal values).
        self.r_left = wheel_radius * (1 + noise * self.rng.uniform(-0.008, 0.008))
        self.r_right = self.r_left * (1 + noise * self.rng.uniform(-0.004, 0.004))
        self.track = track_width * (1 + noise * self.rng.uniform(-0.02, 0.03))
        self.slip = 0.004 * noise
        self.sensor_offset = sensor_offset
        self.sensor_spacing = sensor_spacing
        self.us_forward = us_forward
        self.us_face = us_face
        self.phase2_side_us = phase2_side_us
        self.dark_pompoms = dark_pompoms
        self.motors = {p: MotorState() for p in "ABCD"}
        self.color_ports = {int(WIRING["LEFT_COLOR_PORT"]): -sensor_spacing / 2.0,    # lateral offsets
                            int(WIRING["RIGHT_COLOR_PORT"]): sensor_spacing / 2.0}
        # Bookkeeping
        self.min_clearance = {1: 99.0, 2: 99.0}
        self.contacts = {1: 0, 2: 0}
        self.contact_log = []
        self.offtable = []
        self.pompoms = []          # [x, y, collected]
        self._check_timer = 0.0
        self._fp_pts = {1: _densify(FOOTPRINT_P1), 2: _densify(FOOTPRINT_P2)}
        self.on_tick = None        # called every 10 ms of sim time (visualizer telemetry)
        self.picked = []           # indices of pom-poms picked up, in order
        self.realtime = None       # None = as fast as possible; else sim seconds per real second
        self._rt0 = None

    # ------------------------------------------------------------ time
    def charge(self):
        self.advance(self.call_cost)

    def advance(self, dt):
        while dt > 1e-9:
            h = min(0.002, dt)
            self._step(h)
            dt -= h
        if self.realtime:
            if self._rt0 is None:
                self._rt0 = (_real_time(), self.t)
            ahead = (self.t - self._rt0[1]) / self.realtime - (_real_time() - self._rt0[0])
            if ahead > 0.005:
                _real_sleep(ahead)

    def _step(self, h):
        for port, m in self.motors.items():
            if m.mode == "dps":
                target_v = m.cmd
            elif m.mode == "position":
                err = m.target - m.pos
                lim = m.limit_dps if m.limit_dps > 0 else 700.0
                target_v = max(-lim, min(lim, 12.0 * err))
            else:
                target_v = POWER_TO_DPS * m.cmd
            tau = 0.06 if m.mode != "power" else 0.03
            dv = (target_v - m.vel) * min(1.0, h / tau)
            dv = max(-3000 * h, min(3000 * h, dv))
            m.vel += dv
            m.pos += m.vel * h
        # Wheels: DRIVE_POLARITY -1 = motors mounted inverted (forward = negative motor degrees)
        wl = self.polarity * self.motors[self.port_l].vel
        wr = self.polarity * self.motors[self.port_r].vel
        dl = math.radians(wl * h) * self.r_left * (1 + self.rng.gauss(0, self.slip))
        dr = math.radians(wr * h) * self.r_right * (1 + self.rng.gauss(0, self.slip))
        dc = 0.5 * (dl + dr)
        dth = (dl - dr) / self.track
        mid = math.radians(self.th) + 0.5 * dth
        self.x += dc * math.sin(mid)
        self.y += dc * math.cos(mid)
        self.th = (self.th + math.degrees(dth)) % 360.0
        self.t += h
        self._check_timer += h
        if self._check_timer >= 0.01:
            self._check_timer = 0.0
            self._checks()

    # ------------------------------------------------------------ geometry
    def to_world(self, lat, fwd):
        s, c = math.sin(math.radians(self.th)), math.cos(math.radians(self.th))
        return self.x + fwd * s + lat * c, self.y + fwd * c - lat * s

    def to_robot(self, px, py):
        s, c = math.sin(math.radians(self.th)), math.cos(math.radians(self.th))
        dx, dy = px - self.x, py - self.y
        return dx * c - dy * s, dx * s + dy * c   # (lat, fwd)

    def _box_dist(self, px, py, b):
        """Distance from a point to house b's footprint (0 inside)."""
        cx, cy, hs, a = self.house[b]
        ca, sa = math.cos(a), math.sin(a)
        u = (px - cx) * ca + (py - cy) * sa            # point in the house's own frame
        v = -(px - cx) * sa + (py - cy) * ca
        return math.hypot(max(abs(u) - hs, 0.0), max(abs(v) - hs, 0.0))

    def _checks(self):
        tx, ty = int(math.floor(self.x / T)), int(math.floor(self.y / T))
        near = [b for b in self.boxes if abs(b[0] - tx) <= 1 and abs(b[1] - ty) <= 1]
        if near:
            best = 99.0
            for (lat, fwd) in self._fp_pts[self.phase]:
                px, py = self.to_world(lat, fwd)
                for b in near:
                    best = min(best, self._box_dist(px, py, b))
            if best < self.min_clearance[self.phase]:
                self.min_clearance[self.phase] = best
                wl, wr = self.polarity * self.motors[self.port_l].vel, self.polarity * self.motors[self.port_r].vel
                self.min_where = getattr(self, "min_where", {})
                self.min_where[self.phase] = (round(self.t, 1), round(self.x, 1), round(self.y, 1), round(self.th, 1),
                                              "spinning" if wl * wr < -100 else "driving")
            if best <= 0.0:
                self.contacts[self.phase] += 1
                if len(self.contact_log) < 20:
                    self.contact_log.append((self.phase, round(self.t, 2), round(self.x, 1), round(self.y, 1), round(self.th, 1)))
        for lat, fwd in ((-6, 0), (6, 0), (0, -8)):
            px, py = self.to_world(lat, fwd)
            if not (self.table[0] <= px <= self.table[1] and self.table[2] <= py <= self.table[3]):
                if len(self.offtable) < 2000:       # the dashboard's world runs for hours: keep it bounded
                    self.offtable.append((self.phase, round(self.t, 2), round(px, 1), round(py, 1)))
                self.offtable_n = getattr(self, "offtable_n", 0) + 1
        if self.phase == 2 and abs(self.motors[self.port_belt].vel) > 100:
            for p in self.pompoms:
                if p[2]:
                    continue
                lat, fwd = self.to_robot(p[0], p[1])
                if abs(lat) <= INTAKE_HALF_WIDTH - 0.3 and INTAKE_REACH - 2.5 <= fwd <= INTAKE_REACH + 0.2:
                    p[2] = True
                    self.picked.append(self.pompoms.index(p))
        if self.on_tick is not None:
            self.on_tick()

    # ------------------------------------------------------------ pom-poms
    def spawn_pompoms(self, street_tiles, max_per_tile=16):
        self.pompoms = []
        self.picked = []
        for (tx, ty) in street_tiles:
            near = [b for b in self.boxes if abs(b[0] - tx) <= 1 and abs(b[1] - ty) <= 1]
            for _ in range(self.rng.randint(0, max_per_tile)):
                for _try in range(50):
                    px = tx * T + self.rng.uniform(POMPOM_RADIUS, T - POMPOM_RADIUS)
                    py = ty * T + self.rng.uniform(POMPOM_RADIUS, T - POMPOM_RADIUS)
                    if all(self._box_dist(px, py, b) >= POMPOM_WALL_GAP + POMPOM_RADIUS for b in near):
                        break
                self.pompoms.append([px, py, False])

    # ------------------------------------------------------------ motors
    def motor_set_power(self, port, power):
        self.charge()
        m = self.motors[port]
        m.mode, m.cmd = "power", float(power)

    def motor_set_dps(self, port, dps):
        self.charge()
        m = self.motors[port]
        m.mode, m.cmd = "dps", float(dps)

    def motor_set_limits(self, port, power, dps):
        self.charge()
        self.motors[port].limit_dps = float(dps)

    def motor_set_position(self, port, position):
        self.charge()
        m = self.motors[port]
        m.mode, m.target = "position", float(position)

    def motor_get_position(self, port):
        self.charge()
        return int(round(self.motors[port].pos))

    def motor_reset(self, port):
        self.charge()
        self.motors[port].pos = 0.0

    # ------------------------------------------------------------ sensors
    def color_red(self, port, peek=False):
        """peek: a read from outside the robot code (dashboard): costs no time, own noise, same answer."""
        if not peek:
            self.charge()
        rng = self.peek_rng if peek else self.rng
        lat = self.color_ports[port]
        px, py = self.to_world(lat, self.sensor_offset)
        if not (self.table[0] <= px <= self.table[1] and self.table[2] <= py <= self.table[3]):
            return 1
        half = self.line_half
        on_line = (abs(px / T - round(px / T)) * T <= half) or (abs(py / T - round(py / T)) * T <= half)
        if self.dark_pompoms:
            for p in self.pompoms:
                if not p[2] and math.hypot(p[0] - px, p[1] - py) < 0.6:
                    on_line = True
                    break
        base = 9.0 if on_line else 58.0
        return max(0, base + rng.gauss(0, 2.5 * self.noise))

    def _us_origin_dir(self):
        if self.phase == 1:
            ang = self.motors[self.port_a].pos      # turret degrees (gear 1:1), + = right
            fwd, lat = self.us_forward, 0.0
        else:
            if self.phase2_side_us is None:
                return None
            ang = 90.0 * self.phase2_side_us
            fwd, lat = self.us_forward, 0.0
        heading = (self.th + ang) % 360.0
        ox, oy = self.to_world(lat, fwd)
        hx, hy = math.sin(math.radians(heading)), math.cos(math.radians(heading))
        return ox + self.us_face * hx, oy + self.us_face * hy, heading

    def _ray(self, ox, oy, heading):
        dx, dy = math.sin(math.radians(heading)), math.cos(math.radians(heading))
        best = None
        for b in self.boxes:
            # Ray in the house's own frame (rotated square, half size hs round its centre).
            cx, cy, hs, a = self.house[b]
            ca, sa = math.cos(a), math.sin(a)
            ou, ov = (ox - cx) * ca + (oy - cy) * sa, -(ox - cx) * sa + (oy - cy) * ca
            du, dv = dx * ca + dy * sa, -dx * sa + dy * ca
            tmin, tmax, nrm = -1e9, 1e9, None
            for (o, d, lo, hi, axis) in ((ou, du, -hs, hs, "x"), (ov, dv, -hs, hs, "y")):
                if abs(d) < 1e-9:
                    if o < lo or o > hi:
                        tmin = 1e9
                    continue
                t1, t2 = (lo - o) / d, (hi - o) / d
                if t1 > t2:
                    t1, t2 = t2, t1
                if t1 > tmin:
                    tmin, nrm = t1, axis
                tmax = min(tmax, t2)
            if tmin <= tmax and tmin > 0 and nrm is not None:
                incidence = abs(du) if nrm == "x" else abs(dv)   # cos of angle to the face normal
                if incidence >= math.cos(math.radians(50)):
                    if best is None or tmin < best:
                        best = tmin
        return best

    def us_cm(self, port, peek=False):
        if not peek:
            self.charge()
        rng = self.peek_rng if peek else self.rng
        od = self._us_origin_dir()
        if od is None:
            return 255.0
        if rng.random() < 0.02 * self.noise:
            return 255.0                                   # missed echo
        ox, oy, heading = od
        hits = [self._ray(ox, oy, heading + a) for a in (-7, -3.5, 0, 3.5, 7)]
        hits = [h for h in hits if h is not None and h < 255]
        if rng.random() < 0.01 * self.noise:               # ghost echo (multipath, the other sensor's ping)
            return round(rng.uniform(3.0, min(hits) if hits else 80.0), 1)
        if not hits:
            return 255.0
        return round(max(3.0, min(hits) + rng.gauss(0, 0.6 * self.noise)), 1)
