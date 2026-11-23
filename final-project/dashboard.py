#!/usr/bin/env python3
"""
Snowplow dashboard: the one program to run. Standard library only.

  On the robot:   python3 dashboard.py         open the printed address, type the printed access code
  On the laptop:  python3 dashboard.py --sim   same page, simulated robot (sim/world.py)

From the page:
  - see every motor (A-D) and sensor port (1-4), the battery, the map and the console, live;
  - drive by hand (hold an arrow or WASD; the robot stops 0.35 s after you let go or lose the page),
    nudge any motor, run the belt, reset encoders;
  - run Phase 1, Phase 2, the full demo, any bench test, the auto-tuner, or a small action (1 tile,
    turn 90, dump, turret scan), and answer their questions in the console;
  - calibrate the colour sensors (red or RGB);
  - STOP (button or Esc) at any time.

How it stays safe: only one program may drive the BrickPi.
  - Jobs run as separate processes (jobs.py). While one runs, the dashboard never touches the brick; the
    cards show what the job itself reads. STOP sends it Ctrl-C (it stops its motors), kills it after
    1.5 s if needed, then brakes every motor.
  - Robot scripts started some other way (e.g. over SSH) are noticed within 0.5 s (any Python script from
    this folder) and treated the same way. "Release hardware" covers anything it can't see.
  - Quitting the dashboard stops its own job but never resets the brick.
"""
from __future__ import annotations
import argparse
import ast
import atexit
import hmac
import json
import math
import os
import random
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import jobs as jobs_mod
import tuning

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_DIR = os.path.join(HERE, "sim")
CAL_FILE = os.path.join(HERE, "dashboard_colour_cal.json")
SKIP_SCRIPTS = {"dashboard.py", "visualizer.py"}
STALE_SEC = 0.5                    # a job's sensor value older than this is shown as "last read N s ago"
HIST_LEN = 100                     # samples kept per sensor for the sparkline (10 s at 10 Hz)
MOTOR_FLAGS = {1: "LOW BATTERY (motors off)", 2: "OVERLOADED"}
DEADMAN_SEC = 0.35                 # hand driving stops this long after the last "still holding"
MAX_DRIVE_DPS = 500
MAX_MOTOR_DPS = 1000
JOG_DPS = 200
MAP_W, MAP_H = 1150, 560           # the map is drawn at this size; each browser tab scales it to fit
PAGE_LOST_SEC = 1.5                # motors the dashboard started stop when no page has polled for this long
POINTS_FILE = os.path.join(HERE, "dashboard_points.json")   # last gate / drop-off typed on the page
SIM_SESSION = os.path.join(SIM_DIR, "session")               # --sim only: never the robot's own files


# ============================================================ constants from the robot code
RANDOM_CHOICES = {"random": "random board (new each pick; table size from TILE_X_MAX / TILE_Y_MAX)",
                  "random-edge": "random, street may run along the table edge",
                  "random-18x5": "random on two tables side by side (18 x 5)",
                  "random-9x10": "random on two tables, one above the other (9 x 10)"}


def read_constants(path=os.path.join(HERE, "snowplow_core.py")):
    """Ports etc. straight from snowplow_core.py without importing it (importing it would set up the
    sensors, which must not happen while a robot script is running)."""
    want = {"LEFT_DRIVE_PORT": "B", "RIGHT_DRIVE_PORT": "C", "TURRET_PORT": "B", "DUMP_PORT": "A",
            "ROLLER_MOTOR_PORT": "D", "LEFT_COLOR_PORT": 4, "RIGHT_COLOR_PORT": 2,
            "ULTRASONIC_PORT": 3, "LINE_THRESHOLD": 30, "DRIVE_POLARITY": -1, "ROLLER_SWEEP_DPS": 900,
            "ROLLER_SWEEP_POWER": None, "DUMP_PLUS": None, "DUMP_MINUS": "BACK",
            "TILE_X_MIN": -1, "TILE_X_MAX": 7, "TILE_Y_MIN": -1, "TILE_Y_MAX": 3}
    try:
        tree = ast.parse(open(path).read())
        for node in tree.body:
            if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
                name = node.targets[0].id
                if name in want:
                    try:
                        want[name] = ast.literal_eval(node.value)
                    except ValueError:
                        pass
    except (OSError, SyntaxError) as e:
        print(f"[DASH] could not read {path} ({e}); using default ports")
    return want


def motor_roles(c, phase):
    if phase == 1:      # turret and belt can share a port: the later entry wins
        roles = {c["DUMP_PORT"]: "Not used in Phase 1", c["ROLLER_MOTOR_PORT"]: "Not used in Phase 1",
                 c["TURRET_PORT"]: "Turret"}
    else:
        roles = {c["TURRET_PORT"]: "Not used in Phase 2", c["ROLLER_MOTOR_PORT"]: "Belt", c["DUMP_PORT"]: "Dump"}
    roles.update({c["LEFT_DRIVE_PORT"]: "Left wheel", c["RIGHT_DRIVE_PORT"]: "Right wheel"})
    return roles


def sensor_roles(c):
    return {int(c["LEFT_COLOR_PORT"]): ("colour", "Left colour"),
            int(c["RIGHT_COLOR_PORT"]): ("colour", "Right colour"),
            int(c["ULTRASONIC_PORT"]): ("us", "Ultrasonic")}


# ============================================================ other robot scripts
def _cmdline(pid):
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as f:
            return [a.decode(errors="replace") for a in f.read().split(b"\0") if a]
    except OSError:
        return []


def _script_of(pid, argv):
    """Absolute path of the .py file a Python process runs, or None."""
    if not argv or "python" not in os.path.basename(argv[0]):
        return None
    script = next((a for a in argv[1:] if a.endswith(".py")), None)
    if script is None:
        return None
    if not os.path.isabs(script):
        try:
            script = os.path.join(os.readlink(f"/proc/{pid}/cwd"), script)
        except OSError:
            return None
    return os.path.realpath(script)


def _holds_spi(pid):
    """True if the process has the BrickPi's SPI device open (any program, any folder, even a REPL)."""
    try:
        for fd in os.listdir(f"/proc/{pid}/fd"):
            try:
                if os.readlink(f"/proc/{pid}/fd/{fd}").startswith("/dev/spidev"):
                    return True
            except OSError:
                pass
    except OSError:
        pass
    return False


def find_robot_scripts(exclude=(), spi=True):
    """[(pid, name)] of programs that may be driving the BrickPi: Python scripts from this folder, and
    (spi=True, on the robot) any other process that has the SPI device open. Linux only."""
    if not os.path.isdir("/proc"):
        return []
    me, found = os.getpid(), []
    for d in os.listdir("/proc"):
        if not d.isdigit() or int(d) == me or int(d) in exclude:
            continue
        argv = _cmdline(d)
        if not argv:
            continue
        script = _script_of(d, argv)
        if script and os.path.dirname(script) == HERE and os.path.basename(script) not in SKIP_SCRIPTS:
            found.append((int(d), os.path.basename(script)))
        elif spi and (script is None or os.path.basename(script) not in SKIP_SCRIPTS) and _holds_spi(d):
            found.append((int(d), os.path.basename(script) if script else " ".join(argv)[:40]))
    return found


def find_viewers():
    """[(pid, command)] of visualizer.py processes (an old live viewer competes for the port and telemetry)."""
    out = []
    if os.path.isdir("/proc"):
        for d in os.listdir("/proc"):
            if d.isdigit() and int(d) != os.getpid():
                argv = _cmdline(d)
                script = _script_of(d, argv)
                if script and os.path.basename(script) == "visualizer.py":
                    out.append((int(d), " ".join(os.path.basename(a) if a.endswith(".py") else a
                                                 for a in argv[1:])))
    return out


# ============================================================ hardware: real BrickPi
class RealHardware:
    sim = False

    def __init__(self, consts):
        pid_file = os.path.expanduser("~/brickpi3_pid")
        try:
            old_pid = open(pid_file).read()
        except OSError:
            old_pid = None
        from utils import brick            # opens the BrickPi (reads only)
        # utils.brick resets the whole brick when this program exits and claims ~/brickpi3_pid.
        # Neither is wanted here: a robot script may be running when the dashboard stops.
        atexit.unregister(brick.reset_brick)
        try:
            if old_pid is not None:
                with open(pid_file, "w") as f:
                    f.write(old_pid)
            else:
                os.remove(pid_file)
        except OSError:
            pass
        self.brick = brick
        self.c = consts
        self.motors = {p: brick.Motor(p) for p in "ABCD"}   # creating a Motor sends nothing
        self.roles = sensor_roles(consts)
        self.sensors = {}                                     # port -> sensor object, only while we own it

    def setup_sensors(self, colour_mode):
        b = self.brick
        self.sensors = {}
        for port, (kind, _) in self.roles.items():
            if kind == "colour":
                self.sensors[port] = b.EV3ColorSensor(port, mode="red" if colour_mode == "red" else "component")
            else:
                self.sensors[port] = b.EV3UltrasonicSensor(port, mode="cm")

    def release(self):
        self.sensors = {}

    def battery(self):
        try:
            return round(float(self.brick.BP.get_voltage_battery()), 2)
        except Exception:
            return None

    def motor(self, port):
        try:
            flags, power, enc, dps = self.motors[port].get_status()
            return {"flags": flags, "power": power, "enc": enc, "dps": dps}
        except Exception:
            return {"flags": None, "power": None, "enc": None, "dps": None}

    def sensor(self, port):
        s = self.sensors.get(port)
        if s is None:
            return {"status": "NOT_SET_UP"}
        try:
            status = s.get_status()
        except Exception as e:
            return {"status": f"ERROR {type(e).__name__}"}
        if status != "VALID_DATA":
            return {"status": status}
        v = s.get_value()
        if v is None:
            return {"status": "NO_DATA"}
        if isinstance(v, (list, tuple)):
            return {"status": status, "rgb": [float(a) for a in v[:3]]}
        return {"status": status, "value": float(v)}

    # ---- commands (only while no job or script runs; the hub checks)
    def set_dps(self, port, dps):
        self.motors[port].set_dps(int(dps))

    def set_power(self, port, power):
        self.motors[port].set_power(int(power))

    def brake(self, port):
        self.motors[port].set_power(0)

    def jog(self, port, deg):
        m = self.motors[port]
        m.set_limits(power=60, dps=JOG_DPS)
        m.set_position_relative(int(deg))

    def reset_encoder(self, port):
        # Float the motor first: after a jog it is still holding a position target, and the BrickPi keeps that
        # target when the encoder is reset, so the motor would turn by the old reading to "get back" to it.
        self.motors[port].set_power(0)
        time.sleep(0.05)
        self.motors[port].reset_encoder()

    def clear_limits(self, port):
        self.motors[port].set_limits(0, 0)      # 0 = no limit (a jog sets a cap that would otherwise stay)

    def stop_motors(self):
        for m in self.motors.values():
            try:
                m.set_power(0)
            except Exception:
                pass


# ============================================================ hardware: simulator
class SimHardware:
    """sim/world.py's robot, advanced in real time by a background thread."""
    sim = True

    def __init__(self, consts, map_name):
        sys.path.insert(0, SIM_DIR)
        import world as world_mod
        import maps as maps_mod
        self.world_mod, self.maps, self.MAPS = world_mod, maps_mod, maps_mod.MAPS
        self.lock = threading.RLock()
        self.c = consts
        self.roles = sensor_roles(consts)
        self.colour_mode = "red"
        self.rng = random.Random(1)
        self.set_map(map_name)
        threading.Thread(target=self._clock, daemon=True).start()

    def set_map(self, map_name):
        with self.lock:
            self.map_name = map_name
            self.spec = self.maps.get_map(map_name)
            w = self.world_mod.World(self.spec["boxes"], seed=0, noise=1.0, sensor_offset=-5.0,
                                     table=self.maps.table_of(self.spec))
            w.x, w.y, w.th = 0.5 * self.world_mod.T, 0.5 * self.world_mod.T, 0.0   # tile (0,0), facing +Y
            plain_advance = w.advance

            def locked_advance(dt):
                with self.lock:
                    plain_advance(dt)
            w.advance = locked_advance
            self.w = w

    def set_pose(self, pose):
        with self.lock:
            self.w.x, self.w.y, self.w.th = pose

    def set_encoders(self, motors):
        """Carries the job's final encoder readings over, so the cards don't jump when it ends."""
        with self.lock:
            for p, v in motors.items():
                if p in self.w.motors and v:
                    m = self.w.motors[p]
                    m.pos = float(v[0])
                    if getattr(m, "target", None) is not None:
                        m.target = m.pos

    def _clock(self):
        last = time.time()
        while True:
            time.sleep(0.02)
            now = time.time()
            self.w.advance(min(0.1, now - last))
            last = now

    def setup_sensors(self, colour_mode):
        self.colour_mode = colour_mode

    def release(self):
        pass

    def battery(self):
        return None

    def motor(self, port):
        m = self.w.motors[port]
        return {"flags": 0, "power": None, "enc": int(round(m.pos)), "dps": int(round(m.vel))}

    def sensor(self, port):
        kind = self.roles.get(port, (None,))[0]
        with self.lock:
            if kind == "colour":
                red = self.w.color_red(port)
                if self.colour_mode == "red":
                    return {"status": "VALID_DATA", "value": round(red, 1)}
                # The simulator only models reflected red; fake a neutral RGB from it.
                rgb = [max(0.0, red * 6.0 * f + self.rng.gauss(0, 4)) for f in (1.0, 1.04, 0.92)]
                return {"status": "VALID_DATA", "rgb": [round(a, 1) for a in rgb]}
            if kind == "us":
                return {"status": "VALID_DATA", "value": round(self.w.us_cm(port), 1)}
        return {"status": "NOT_SET_UP"}

    def set_dps(self, port, dps):
        with self.lock:
            self.w.motor_set_dps(port, dps)

    def set_power(self, port, power):
        with self.lock:
            self.w.motor_set_power(port, power)

    def brake(self, port):
        with self.lock:
            self.w.motor_set_power(port, 0)

    def jog(self, port, deg):
        with self.lock:
            self.w.motor_set_limits(port, 60, JOG_DPS)
            self.w.motor_set_position(port, self.w.motor_get_position(port) + deg)

    def reset_encoder(self, port):
        with self.lock:
            self.w.motor_set_power(port, 0)      # same as the real robot: drop the position target first
            self.w.motor_reset(port)

    def clear_limits(self, port):
        with self.lock:
            self.w.motor_set_limits(port, 0, 0)

    def stop_motors(self):
        with self.lock:
            for p in "ABCD":
                self.w.motor_set_power(p, 0)

    def board_events(self, phase):
        with self.lock:
            self.w.phase = phase
        outline = self.world_mod.FOOTPRINT_P1 if phase == 1 else self.world_mod.FOOTPRINT_P2
        tb = list(self.maps.table_of(self.spec))
        return [{"k": "meta", "t": 0.0, "phase": phase, "outline": outline, "table": tb},
                {"k": "sim", "boxes": [list(b) for b in sorted(self.w.boxes)], "table": tb,
                 "street": [list(t) for t in self.spec["street"]], "pompoms": [], "outline": outline}]

    def pose_event(self):
        return {"k": "pose", "t": 0.0, "x": round(self.w.x, 2), "y": round(self.w.y, 2), "th": round(self.w.th, 1)}


# ============================================================ colour helpers
def colour_judgement(rgb, cal):
    """Where an RGB reading sits between the stored table and line colours.
    f: 0 = table, 1 = line (projection on the table->line axis). off: distance from that axis,
    as a fraction of the table-line distance (big = some other colour)."""
    t, l = cal.get("table"), cal.get("line")
    if not (rgb and t and l):
        return None
    u = [l[i] - t[i] for i in range(3)]
    uu = sum(a * a for a in u)
    if uu < 1e-6:
        return None
    d = [rgb[i] - t[i] for i in range(3)]
    f = sum(d[i] * u[i] for i in range(3)) / uu
    perp = math.sqrt(max(0.0, sum(a * a for a in d) - f * f * uu))
    off = perp / math.sqrt(uu)
    return {"f": round(f, 3), "off": round(off, 3),
            "verdict": "LINE" if f > 0.5 and off < 0.5 else ("table" if off < 0.5 else "other colour")}


# ============================================================ hub: owns the state
class Hub:
    def __init__(self, hw, consts, args):
        self.hw = hw
        self.c = consts
        self.args = args
        self.lock = threading.RLock()
        self.stop_lock = threading.Lock()
        self.phase = args.phase
        self.colour_mode = "red"
        self.released = False          # user pressed "Release hardware"
        self.scripts = []              # robot scripts not started by us
        self.owned = False             # sensors set up by us and safe to read
        self.hist = {p: deque(maxlen=HIST_LEN) for p in (1, 2, 3, 4)}
        self.snapshot = b"{}"
        self.n = 0
        self._message, self.message_t = "", 0.0
        self.cal = self._load_cal()
        self.last_rgb, self.last_red = {}, {}
        self.tele = None               # (received at, "hw" event) from a running job
        self.job = None                # {"proc", "name", "label", "t0", "stopped"}
        self.last_job = ""
        self.prompt = None
        self.prompt_id = 0             # numbers the questions, so a late or doubled click can't answer the next one
        self.show_truth = True         # simulator: draw the true houses / pose
        self.console = deque(maxlen=600)
        self.console_n = 0
        self.moving = set()            # ports the dashboard itself set turning (hand drive, belt)
        self.drive_deadline = 0.0
        self.last_seen = time.time()   # last time a page asked for the state (the page's heartbeat)
        self.stop_until = 0.0          # hand driving is refused for a moment after STOP
        self.sim_speed = 1.0
        self.session = SIM_SESSION if hw.sim else None    # per map: calibration + street map, kept across restarts
        self.show_world = True         # simulator: the map shows the dashboard's own robot
        self.world_reset = False       # simulator: redraw the board before showing it again
        self.points = self._load_points()
        self.bench = jobs_mod.bench_test_names()
        self.steps = jobs_mod.bringup_steps()
        threading.Thread(target=self._watchdog, daemon=True).start()

    def on_hw(self, ev):
        self.tele = (time.time(), ev)
        if self.job and ev.get("phase") in (1, 2):
            self.phase = int(ev["phase"])

    def on_meta(self, ev):
        """A running job announced which hardware it set up (the demo switches from 1 to 2 halfway)."""
        if self.job and ev.get("phase") in (1, 2):
            self.phase = int(ev["phase"])

    @property
    def message(self):
        return self._message

    @message.setter
    def message(self, text):
        self._message, self.message_t = text, time.time()

    def seen(self):
        self.last_seen = time.time()

    def log(self, line):
        with self.lock:
            self.console.append(line)
            self.console_n += 1

    # ---- files
    def _load_cal(self):
        try:
            with open(CAL_FILE) as f:
                return {int(k): v for k, v in json.load(f).items()}
        except (OSError, ValueError):
            return {}

    def _save_cal(self):
        try:
            with open(CAL_FILE, "w") as f:
                json.dump(self.cal, f, indent=1)
        except OSError as e:
            self.message = f"could not save {os.path.basename(CAL_FILE)}: {e}"

    def _load_points(self):
        pts = jobs_mod.phase1_defaults()
        try:
            with open(POINTS_FILE) as f:
                saved = json.load(f)
            pts.update({k: [int(v[0]), int(v[1])] for k, v in saved.items() if k in ("s1", "s2", "d1", "d2")})
            pts["saved"] = True
        except (OSError, ValueError, TypeError, IndexError, KeyError):
            pts["saved"] = False
        return pts

    def _save_points(self, args):
        nums = [int(v) for v in args]
        self.points = {k: nums[2 * i:2 * i + 2] for i, k in enumerate(("s1", "s2", "d1", "d2"))}
        self.points["saved"] = True
        try:
            with open(POINTS_FILE, "w") as f:
                json.dump({k: v for k, v in self.points.items() if k != "saved"}, f)
        except OSError:
            pass

    # ---- who may touch the brick
    def busy(self):
        """Why the dashboard may not command the hardware right now (None = it may)."""
        if self.job:
            return "a job is running"
        if self.scripts:
            return "another robot script is running"
        if self.released:
            return "the hardware is released"
        return None

    def _clear_map(self):
        """Blank map before a job starts; sent on the telemetry port so it arrives before anything the job sends."""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sk:
                tb = (list(self.hw.maps.table_of(self.hw.spec)) if self.hw.sim else
                      [read_constants()[k] for k in ("TILE_X_MIN", "TILE_X_MAX", "TILE_Y_MIN", "TILE_Y_MAX")])
                sk.sendto(json.dumps({"k": "meta", "t": 0.0, "phase": self.phase, "table": tb}).encode(),
                          ("127.0.0.1", self.args.udp))
        except OSError:
            pass

    def _brake_ours(self, why=None):
        """Brakes every motor the dashboard itself set turning (call before giving the brick away)."""
        with self.lock:
            if not self.moving:
                return
            for p in sorted(self.moving):
                try:
                    self.hw.brake(p)
                except Exception:
                    pass
            self.moving.clear()
        if why:
            self.log(f"[DASH] {why}: braked the motors the dashboard was running")

    # ---- commands from the page
    def command(self, q):
        try:
            return self._command(q)
        except ValueError as e:
            return {"ok": False, "error": str(e)}
        except (KeyError, TypeError, IndexError) as e:
            return {"ok": False, "error": f"bad request ({type(e).__name__}: {e})"}
        except OSError as e:
            return {"ok": False, "error": f"could not write the file: {e}"}

    def _command(self, q):
        cmd = q.get("cmd")
        if cmd == "stop":
            threading.Thread(target=self.stop, daemon=True).start()
            return {"ok": True}
        if cmd == "answer":
            return self.answer(str(q.get("text", "")), q.get("id"))
        if cmd == "phase" and q.get("v") in (1, 2, "1", "2"):
            if self.job:
                return {"ok": False, "error": "a job is running (it says which hardware it uses)"}
            self.phase = int(q["v"])
            if self.hw.sim:
                self.show_world, self.world_reset = True, True
            return {"ok": True}
        if cmd == "colour_mode" and q.get("v") in ("red", "rgb"):
            with self.lock:
                self.colour_mode = q["v"]
                self.owned = False                 # sampler sets the sensors up again
                for p in self.hist:
                    self.hist[p].clear()
            return {"ok": True}
        if cmd == "release":
            with self.lock:
                if self.job:
                    return {"ok": False, "error": "a job is running"}
                if q.get("v"):
                    self._brake_ours()
                self.released = bool(q.get("v"))
                self.owned = False
                if self.released:
                    self.hw.release()
            return {"ok": True}
        if cmd == "cal":
            return self.calibrate(q)
        if cmd == "sim_map" and self.hw.sim and not (q.get("v") in RANDOM_CHOICES or self.hw.maps.is_map_name(q.get("v"))):
            return {"ok": False, "error": f"unknown map {q.get('v')!r}"}
        if cmd == "sim_map" and self.hw.sim:
            name = q["v"]
            if name in RANDOM_CHOICES:          # a new random board each time it is picked
                if name == "random":            # on the table size in snowplow_core.py's TILE_X_MAX / TILE_Y_MAX
                    c = read_constants()
                    w, h = c["TILE_X_MAX"] - c["TILE_X_MIN"] + 1, c["TILE_Y_MAX"] - c["TILE_Y_MIN"] + 1
                    name = "random" if (w, h) == (9, 5) else f"random-{w}x{h}"
                name = f"{name}-{random.randrange(100000)}"
                try:
                    self.hw.maps.get_map(name)
                except (RuntimeError, KeyError) as e:
                    return {"ok": False, "error": str(e)}
            with self.lock:
                if self.job:
                    return {"ok": False, "error": "a job is running"}
                self.moving.clear()
                self.hw.set_map(name)
                self.show_world, self.world_reset = True, True
            return {"ok": True}
        if cmd == "sim_reset" and self.hw.sim:
            with self.lock:
                if self.job:
                    return {"ok": False, "error": "a job is running"}
                shutil.rmtree(os.path.join(self.session, self.hw.map_name), ignore_errors=True)
            self.log(f"[DASH] simulator files for the {self.hw.map_name} map deleted (calibration, street map)")
            return {"ok": True}
        if cmd == "constants_save":
            done = tuning.save_constants(str(q.get("file")), dict(q.get("changes") or {}))
            if done:
                self.c = read_constants()           # ports, threshold, belt speed shown on the page
                self.log("[DASH] saved in " + dict((f, n) for f, n, _ in tuning.FILES)[str(q.get("file"))] + ": " +
                         ", ".join(f"{n} {a} -> {b}" for n, a, b in done) +
                         (" (applies to the next run)" if self.job else ""))
            return {"ok": True, "saved": done}
        if cmd == "calibration_save":
            tuning.save_calibration(self.calibration_path(), dict(q.get("values") or {}))
            self.log(f"[DASH] calibration saved: {self.calibration_path()}")
            return {"ok": True}
        if cmd == "calibration_delete":
            try:
                os.remove(self.calibration_path())
                self.log(f"[DASH] {self.calibration_path()} deleted: the code's own values are used now")
            except FileNotFoundError:
                pass
            return {"ok": True}
        if cmd == "show_truth":
            self.show_truth = bool(q.get("v"))
            return {"ok": True}
        if cmd == "sim_speed" and self.hw.sim:
            self.sim_speed = max(0.0, min(16.0, float(q.get("v", 1))))
            return {"ok": True}
        if cmd == "job":
            return self.start_job(str(q.get("name")), [str(a) for a in q.get("args", [])])
        if cmd in ("drive", "jog", "motor_dps", "motor_power", "reset_enc"):
            return self.manual(cmd, q)
        return {"ok": False, "error": f"unknown command {cmd!r}"}

    def calibrate(self, q):
        if q.get("which") not in ("table", "line", "clear"):
            return {"ok": False, "error": "bad calibration request"}
        port = int(q.get("port", 0))
        rgb = self.colour_mode == "rgb"
        keys = {"table": "table", "line": "line"} if rgb else {"table": "red_table", "line": "red_line"}
        with self.lock:
            c = self.cal.setdefault(port, {})
            if q["which"] == "clear":
                for k in keys.values():
                    c.pop(k, None)
            else:
                samples = list((self.last_rgb if rgb else self.last_red).get(port, []))[-10:]
                if not samples:
                    return {"ok": False, "error": "no readings from this sensor yet"}
                if rgb:
                    c[keys[q["which"]]] = [round(sum(v[i] for v in samples) / len(samples), 1) for i in range(3)]
                else:
                    c[keys[q["which"]]] = round(sum(samples) / len(samples), 1)
            self._save_cal()
        return {"ok": True}

    # ---- hand control (dashboard drives the motors itself; only when nothing else does)
    def manual(self, cmd, q):
        with self.lock:
            why = self.busy()
            if why:
                return {"ok": False, "error": f"not now: {why}"}
            pol = int(self.c["DRIVE_POLARITY"])
            lp, rp = self.c["LEFT_DRIVE_PORT"], self.c["RIGHT_DRIVE_PORT"]
            port = str(q.get("port", ""))
            if cmd != "drive" and port not in ("A", "B", "C", "D"):
                return {"ok": False, "error": f"no motor port {port!r}"}
            if cmd == "drive":
                l = max(-MAX_DRIVE_DPS, min(MAX_DRIVE_DPS, float(q.get("l", 0))))
                r = max(-MAX_DRIVE_DPS, min(MAX_DRIVE_DPS, float(q.get("r", 0))))
                if l == 0 and r == 0:
                    self.hw.brake(lp)
                    self.hw.brake(rp)
                    self.moving -= {lp, rp}
                else:
                    if time.time() < self.stop_until:
                        return {"ok": False, "error": "STOP was just pressed: hand driving is paused for a second"}
                    for pt, v in ((lp, l), (rp, r)):
                        self.hw.clear_limits(pt)
                        self.hw.set_dps(pt, pol * v)
                    self.moving |= {lp, rp}
                    self.drive_deadline = time.time() + DEADMAN_SEC
            elif cmd == "jog":
                self.moving.discard(port)
                self.hw.jog(port, max(-360, min(360, float(q.get("deg", 0)))))
            elif cmd == "motor_dps":
                dps = max(-MAX_MOTOR_DPS, min(MAX_MOTOR_DPS, float(q.get("dps", 0))))
                if dps == 0:
                    self.hw.brake(port)
                    self.moving.discard(port)
                else:
                    if time.time() < self.stop_until:
                        return {"ok": False, "error": "STOP was just pressed"}
                    self.hw.clear_limits(port)
                    self.hw.set_dps(port, dps)
                    self.moving.add(port)
            elif cmd == "motor_power":
                power = max(-100.0, min(100.0, float(q.get("power", 0))))
                if power == 0:
                    self.hw.brake(port)
                    self.moving.discard(port)
                else:
                    if time.time() < self.stop_until:
                        return {"ok": False, "error": "STOP was just pressed"}
                    self.hw.clear_limits(port)
                    self.hw.set_power(port, power)
                    self.moving.add(port)
            elif cmd == "reset_enc":
                self.hw.reset_encoder(port)
            if self.hw.sim and not self.show_world:
                self.show_world, self.world_reset = True, True
        return {"ok": True}

    def _watchdog(self):
        """Hand driving stops DEADMAN_SEC after the last 'still holding'; anything the dashboard set
        turning (belt included) stops when no page has been seen for PAGE_LOST_SEC (wifi, closed laptop)."""
        lp, rp = self.c["LEFT_DRIVE_PORT"], self.c["RIGHT_DRIVE_PORT"]
        while True:
            time.sleep(0.05)
            now = time.time()
            with self.lock:
                if self.job:
                    continue
                if {lp, rp} & self.moving and now > self.drive_deadline:
                    for p in (lp, rp):
                        if p in self.moving:
                            self.hw.brake(p)
                    self.moving -= {lp, rp}
            if self.moving and now - self.last_seen > PAGE_LOST_SEC:
                self._brake_ours("no page connected")

    # ---- jobs (separate processes, see jobs.py)
    def start_job(self, name, args):
        if name not in jobs_mod.JOB_LABELS:
            return {"ok": False, "error": f"unknown job {name}"}
        label = jobs_mod.JOB_LABELS[name]
        phase = jobs_mod.job_phase(name, args)
        if name == "bench":
            if not (args and args[0].isdigit() and 0 < int(args[0]) <= len(self.bench)):
                return {"ok": False, "error": "pick a bench test"}
            label = f"Bench {args[0]}: {self.bench[int(args[0]) - 1]}"
        elif name == "turn":
            label = f"{label} {'right' if args and args[0] == 'R' else 'left'}"
        elif name == "dump":
            label = f"{label}: motor {'-' if args and args[0] == '-' else '+'}"
        elif name == "tile":
            label = "Drive 1 tile " + ("back" if args and args[0].startswith("-") else "forward")
        elif name in ("phase1", "demo") and not self.hw.sim:
            try:
                if len(args) != 8:
                    raise ValueError
                self._save_points(args)
            except ValueError:
                return {"ok": False, "error": "the gate and drop-off need 8 whole numbers"}
        with self.lock:
            why = self.busy()
            if why:
                return {"ok": False, "error": f"not now: {why}"}
            self.hw.stop_motors()                 # nothing of ours keeps running under the job
            for p in "ABCD":
                self.hw.clear_limits(p)
            self.moving.clear()
            self.hw.release()
            self.owned = False
            if phase:
                self.phase = phase
            cmd = [sys.executable, "-u", os.path.join(HERE, "jobs.py"), "--udp", str(self.args.udp),
                   "--phase", str(self.phase)]
            if self.hw.sim:
                w = self.hw.w
                cmd += ["--sim", self.hw.map_name, "--session", os.path.join(self.session, self.hw.map_name),
                        "--speed", str(self.sim_speed), "--pose", f"{w.x:.2f}", f"{w.y:.2f}", f"{w.th:.2f}"]
            cmd += [name] + args
            self._clear_map()
            try:
                proc = subprocess.Popen(cmd, cwd=HERE, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT, text=True, encoding="utf-8", errors="replace",
                                        bufsize=1, start_new_session=True)
            except OSError as e:
                return {"ok": False, "error": f"could not start the job: {e}"}
            self.job = {"proc": proc, "name": name, "label": label, "t0": time.time(), "stopped": False}
            self.prompt = None
            self.tele = None
            if self.hw.sim:
                self.show_world = False
        self.log(f"=== {label} ===")
        threading.Thread(target=self._read_job, args=(proc,), daemon=True).start()
        return {"ok": True}

    def _read_job(self, proc):
        code = None
        try:
            for line in proc.stdout:
                line = line.rstrip("\n")
                if line.startswith(jobs_mod.PROMPT_MARK):
                    with self.lock:
                        self.prompt = line[len(jobs_mod.PROMPT_MARK):].strip() or "Press Continue"
                        self.prompt_id += 1
                    self.log("? " + self.prompt)
                else:
                    self.log(line)
            code = proc.wait()
        except Exception as e:                   # never leave the dashboard stuck on "a job is running"
            self.log(f"[DASH] lost the job's output ({type(e).__name__}: {e})")
            try:
                proc.kill()
                code = proc.wait(timeout=2)
            except Exception:
                pass
        finally:
            with self.lock:
                job = self.job or {"label": "job", "t0": time.time(), "stopped": False}
                t = time.time() - job["t0"]
                if job["stopped"]:
                    how = "stopped by STOP"
                elif code == 0:
                    how = "finished"
                elif code == jobs_mod.EXIT_NEEDS_MAP:
                    how = "no street map yet: run Phase 1 first"
                else:
                    how = f"failed (exit {code}): see the console"
                self.last_job = f"{job['label']}: {how} ({t:.0f} s)"
                if self.hw.sim:
                    if code == jobs_mod.EXIT_NEEDS_MAP or not (self.tele and self.tele[1].get("pose")):
                        self.show_world, self.world_reset = True, True     # nothing drawn: show the board again
                    else:                            # the dashboard's robot is where the job left it
                        self.hw.set_pose(self.tele[1]["pose"])
                        self.hw.set_encoders(self.tele[1].get("motors") or {})
                self.job = None
                self.prompt = None
                self.owned = False                 # jobs reset the brick on exit: set the sensors up again

    def answer(self, text, pid=None):
        with self.lock:
            job, asked = self.job, self.prompt
            if not job or asked is None:
                return {"ok": False, "error": "nothing is waiting for an answer"}
            if pid is not None and int(pid) != self.prompt_id:
                return {"ok": True, "ignored": "that question was already answered"}
            self.prompt = None                     # cleared before writing: the next question can't be lost
        try:
            job["proc"].stdin.write(text + "\n")
            job["proc"].stdin.flush()
        except OSError as e:
            return {"ok": False, "error": f"could not answer: {e}"}
        return {"ok": True}

    def stop(self):
        """Ctrl-C our job and every other robot script (they stop their motors on the way out), kill
        what is left after 1.5 s, then brake all motors ourselves."""
        with self.lock:
            self.stop_until = time.time() + 1.0
            self.drive_deadline = 0.0
        if not self.stop_lock.acquire(blocking=False):
            with self.lock:
                if not self.busy():
                    self.hw.stop_motors()
            return                                  # a STOP is already in progress
        try:
            self._stop()
        finally:
            self.stop_lock.release()

    def _stop(self):
        msgs = []
        with self.lock:
            job = self.job
            if job:
                job["stopped"] = True
            if not self.busy():
                self.hw.stop_motors()               # hand driving / belt: brake right away
            self.moving.clear()
        targets = []
        if job and job["proc"].poll() is None:
            targets.append((job["proc"].pid, job["label"], True))
        if not self.hw.sim:
            own = {job["proc"].pid} if job else set()
            targets += [(pid, name, False) for pid, name in find_robot_scripts(exclude=own)]
        for pid, name, ours in targets:
            try:
                if ours:
                    os.killpg(pid, signal.SIGINT)
                else:
                    os.kill(pid, signal.SIGINT)
                msgs.append(f"Ctrl-C sent to {name}")
            except OSError as e:
                msgs.append(f"could not stop {name} ({e})")
        deadline = time.time() + 1.5
        while targets and time.time() < deadline:
            time.sleep(0.1)
            targets = [t for t in targets if _alive(t[0])]
        for pid, name, ours in targets:
            try:
                if ours:
                    os.killpg(pid, signal.SIGKILL)
                else:
                    os.kill(pid, signal.SIGKILL)
                msgs.append(f"killed {name}")
            except OSError:
                pass
        if job:
            try:
                job["proc"].wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                pass
        if self.hw.sim or not find_robot_scripts(exclude={job["proc"].pid} if job else ()):
            with self.lock:
                self.hw.stop_motors()
            msgs.append("all motors braked")
        self.message = "STOP: " + "; ".join(msgs) + time.strftime("  (%H:%M:%S)")
        self.log(self.message)
        print(f"[DASH] {self.message}", flush=True)

    # ---- sampling
    def run_sampler(self):
        try:
            os.nice(5)            # this thread only (Linux): the robot's control loop comes first
        except (AttributeError, OSError):
            pass
        period = 1.0 / self.args.hz
        last_scan = 0.0
        while True:
            t0 = time.time()
            if not self.hw.sim and t0 - last_scan >= 0.5:
                last_scan = t0
                with self.lock:
                    own = {self.job["proc"].pid} if self.job else set()
                scripts = find_robot_scripts(exclude=own)
                with self.lock:
                    if scripts and not self.scripts:
                        self._brake_ours()
                        self.owned = False
                        self.hw.release()
                        self.log(f"[DASH] {', '.join(n for _, n in scripts)} started outside the dashboard: "
                                 "not touching the brick")
                    elif self.scripts and not scripts:
                        self.log("[DASH] that script finished: reading the ports again")
                    self.scripts = scripts
            try:
                self._sample()
            except Exception as e:      # never die; show it
                self.message = f"sampler error: {type(e).__name__}: {e}"
            time.sleep(max(0.0, period - (time.time() - t0)))

    def _sim_warnings(self):
        """Simulator, hand driving: say so when the robot touches a house or leaves the table."""
        w = self.hw.w
        now = (sum(w.contacts.values()), getattr(w, "offtable_n", len(w.offtable)))
        last = getattr(self, "_sim_seen", now)
        self._sim_seen = now
        if now[0] > last[0]:
            self.message = "Simulator: the robot is touching a house"
        elif now[1] > last[1]:
            self.message = "Simulator: the robot is off the table"

    def _sample(self):
        with self.lock:
            why = self.busy()
            if self.hw.sim and not self.job:
                self._sim_warnings()
            if not why and not self.owned:
                self.hw.setup_sensors(self.colour_mode)
                self.owned = True
            tele = self.tele if self.tele and time.time() - self.tele[0] < 1.5 else None
            # Where the numbers come from: our own reads of the ports, a running job's telemetry, or nothing.
            if why:
                source = "telemetry" if tele else "none"
            else:
                source = "ports"
            roles_m = motor_roles(self.c, self.phase)
            roles_s = sensor_roles(self.c)
            motors, sensors, battery = {}, {}, None
            if source == "ports":
                battery = self.hw.battery()
                for p in "ABCD":
                    motors[p] = self.hw.motor(p)
            elif source == "telemetry":
                battery = tele[1].get("bat")
                for p in "ABCD":
                    pv = (tele[1].get("motors") or {}).get(p)
                    if pv:
                        motors[p] = {"enc": pv[0], "dps": pv[1], "power": pv[2] if len(pv) > 2 else None,
                                     "flags": pv[3] if len(pv) > 3 else None}
            for m in motors.values():
                fl = m.get("flags") or 0
                m["warn"] = [txt for bit, txt in MOTOR_FLAGS.items() if fl & bit]
            for port in (1, 2, 3, 4):
                kind, label = roles_s.get(port, ("none", "not used by the code"))
                s = {"kind": kind, "label": label}
                if kind != "none" and source != "none":
                    if source == "ports":
                        s.update(self.hw.sensor(port))
                    else:
                        v = (tele[1].get("sensors") or {}).get(str(port))
                        s.update({"status": "VALID_DATA", "value": float(v)} if v is not None
                                 else {"status": "NOT_READ"})
                        age = (tele[1].get("sensor_age") or {}).get(str(port))
                        if age is not None:     # the job's last read, not a live one: the code only reads
                            s["age"] = age      # the colour sensors near grid lines
                            s["stale"] = age > STALE_SEC
                    if s.get("status") == "VALID_DATA":
                        self._interpret(port, kind, s)
                # One history point per sample, so the sparkline keeps moving in time; None (a gap) when there
                # is no fresh reading (stale job value, sensor error, nothing reading the port).
                v = s.get("bright", s.get("value")) if s.get("status") == "VALID_DATA" and not s.get("stale") else None
                self.hist[port].append(v)
                s["hist"] = list(self.hist[port])
                sensors[str(port)] = s
            job = self.job
            self.n += 1
            snap = {
                "n": self.n, "sim": self.hw.sim, "source": source, "busy": why,
                "job": {"label": job["label"], "secs": round(time.time() - job["t0"], 1)} if job else None,
                "last_job": self.last_job, "prompt": self.prompt, "prompt_id": self.prompt_id, "console_n": self.console_n,
                "show_truth": self.show_truth,
                "scripts": [{"pid": p, "name": n} for p, n in self.scripts], "released": self.released,
                "battery": battery, "phase": self.phase, "colour_mode": self.colour_mode,
                "line_threshold": self.c["LINE_THRESHOLD"], "belt_dps": self.c["ROLLER_SWEEP_DPS"],
                "belt_power": self.c["ROLLER_SWEEP_POWER"],
                "dump_plus": self.c["DUMP_PLUS"], "dump_minus": self.c["DUMP_MINUS"],
                "message": self.message if time.time() - self.message_t < 15 else "",
                "motors": {p: dict(m, role=roles_m.get(p, "")) for p, m in motors.items()},
                "motor_roles": roles_m, "sensors": sensors, "steps": self.steps, "points": self.points,
            }
            if self.hw.sim:
                snap["sim_map"] = self.hw.map_name
                snap["sim_maps"] = (sorted(self.hw.MAPS)
                                    + ([self.hw.map_name] if self.hw.map_name not in self.hw.MAPS else [])
                                    + list(RANDOM_CHOICES))
                snap["sim_speed"] = self.sim_speed
                snap["sim_points"] = {k: list(self.hw.spec[k]) for k in ("s1", "s2", "d1", "d2")}
            self.snapshot = json.dumps(snap, separators=(",", ":")).encode()

    def _interpret(self, port, kind, s):
        """Adds line/table verdicts and calibration to one sensor reading."""
        cal = self.cal.get(port, {})
        if kind == "colour":
            if "rgb" in s:
                self.last_rgb.setdefault(port, deque(maxlen=20)).append(s["rgb"])
                s["bright"] = round(sum(s["rgb"]) / 3.0, 1)
                s["cal"] = {k: cal[k] for k in ("table", "line") if k in cal}
                s["judge"] = colour_judgement(s["rgb"], s["cal"])
            elif "value" in s:
                self.last_red.setdefault(port, deque(maxlen=20)).append(s["value"])
                s["line"] = s["value"] < float(self.c["LINE_THRESHOLD"])
                s["cal"] = {k: cal[k] for k in ("red_table", "red_line") if k in cal}
                if "red_table" in cal and "red_line" in cal:
                    s["suggest"] = round((cal["red_table"] + cal["red_line"]) / 2.0, 1)
                    s["margin"] = round(abs(cal["red_table"] - cal["red_line"]) / 2.0, 1)

    def calibration_path(self):
        """The calibration file the next job will load (the simulator keeps its own per map)."""
        if self.hw.sim:
            return os.path.join(self.session, self.hw.map_name, "calibration_params.json")
        return os.path.join(HERE, "calibration_params.json")

    def constants(self):
        exists, values = tuning.read_calibration(self.calibration_path())
        files = []
        for fid, _, title in tuning.FILES:
            try:
                files.append({"id": fid, "title": title, "sections": tuning.list_constants(fid)})
            except (OSError, SyntaxError) as e:
                files.append({"id": fid, "title": title, "sections": [], "error": str(e)})
        return {"files": files, "sim": self.hw.sim,
                "calibration": {"path": os.path.relpath(self.calibration_path(), HERE), "exists": exists,
                                "values": values, "keys": list(tuning.CALIBRATION_KEYS)}}

    def console_since(self, since):
        with self.lock:
            n = self.console_n
            lines = list(self.console)
        first = n - len(lines)
        if since > n:                      # the page saw an older dashboard (restarted): send everything
            return {"n": n, "lines": lines, "reset": True}
        start = max(since, first)
        return {"n": n, "lines": lines[start - first:], "reset": since < first}

    def shutdown(self):
        with self.lock:
            job = self.job
        if job and job["proc"].poll() is None:
            print("[DASH] stopping the running job", flush=True)
            self.stop()


def _alive(pid):
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    try:                                     # a zombie child counts as gone
        with open(f"/proc/{pid}/stat") as f:
            return f.read().split(")")[-1].split()[0] != "Z"
    except OSError:
        return True


# ============================================================ web page
PAGE = r"""<!doctype html><html><head><meta charset="utf-8"><title>Snowplow dashboard</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
/* Bench-console look: graphite surfaces, one accent (the robot's blue on the map), amber = the robot is waiting
   for you, red = stop / fault. Numbers use tabular figures; monospace only for the console and constant names. */
:root{--bg:#1b1d21;--surf:#23262b;--raised:#2d3137;--line:#373c43;--fg:#e6e8eb;--dim:#8f979f;
 --acc:#4c8dff;--acc-bg:#1f3350;--ok:#3fb27f;--warn:#f2b33d;--warn-bg:#3a2f17;--bad:#e5484d;
 --r:6px;--f:system-ui,-apple-system,"Segoe UI",Roboto,Ubuntu,sans-serif;--m:ui-monospace,"DejaVu Sans Mono",monospace}
*{box-sizing:border-box}html,body{margin:0;background:var(--bg);color:var(--fg);font:13px/1.45 var(--f)}
.wrap{padding:0 14px 14px;display:grid;gap:14px;max-width:1600px;margin:0 auto}
.head{position:sticky;top:0;z-index:5;display:grid;gap:6px;background:var(--bg);padding:10px 0 6px}
.bar{display:flex;align-items:center;gap:8px 18px;flex-wrap:wrap;background:var(--surf);border:1px solid var(--line);border-radius:var(--r);padding:7px 8px 7px 12px}
.grp{display:flex;align-items:center;gap:6px;flex-wrap:wrap}
.lbl{color:var(--dim);font-size:12px}
.seg{display:inline-flex;border:1px solid var(--line);border-radius:5px;overflow:hidden}
.seg button{margin:0;border:0;border-radius:0;background:transparent}.seg button+button{border-left:1px solid var(--line)}
.seg button.on{background:var(--acc-bg);color:#fff;box-shadow:inset 0 -2px 0 var(--acc)}
h2.sec{font-size:13px;font-weight:600;margin:4px 0 -6px;color:var(--dim)}
.grid4{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:10px}
.main{display:grid;grid-template-columns:330px minmax(0,1fr);gap:14px;align-items:start}
.col{display:grid;gap:10px;align-content:start}
@media (max-width:1000px){.grid4{grid-template-columns:repeat(2,minmax(0,1fr))}.main{grid-template-columns:1fr}}
.card{background:var(--surf);border:1px solid var(--line);border-radius:var(--r);padding:10px 12px}
.card h3{margin:0 0 8px;font-size:13px;font-weight:600;display:flex;justify-content:space-between;gap:6px;align-items:center;flex-wrap:wrap}
.port{display:inline-grid;place-items:center;min-width:20px;height:20px;border-radius:4px;background:var(--raised);
 color:var(--dim);font-size:11px;font-weight:700;margin-right:8px;padding:0 4px}
.dim{color:var(--dim);font-size:12px}.mono{font-family:var(--m)}code{font:12px var(--m)}
.num,.big,.kv b{font-variant-numeric:tabular-nums}
.big{font-size:24px;font-weight:600;letter-spacing:-.01em;line-height:1.2}
.big .u{font-size:13px;font-weight:400;color:var(--dim);margin-left:3px}
.big.dim{color:var(--dim);font-size:24px}
.tag{font-size:11px;font-weight:700;padding:1px 6px;border-radius:4px;margin-left:8px;vertical-align:middle;background:var(--raised);color:var(--dim)}
.tag.hot{background:var(--warn-bg);color:var(--warn)}
.kvs{display:flex;gap:4px 16px;flex-wrap:wrap;margin:4px 0}.kv{font-size:12px;color:var(--dim)}.kv b{color:var(--fg);font-weight:500;margin-left:4px}
.note{font-size:12px;color:var(--dim);margin-top:4px}
.dot{display:inline-block;width:8px;height:8px;border-radius:50%;margin-right:6px;vertical-align:1px}
.ok{background:var(--ok)}.warnb{background:var(--warn)}.badb{background:var(--bad)}.off{background:#5c6168}
.warn{color:var(--warn)}.good{color:var(--ok)}
button{font:inherit;font-size:12px;background:var(--raised);color:var(--fg);border:1px solid var(--line);border-radius:5px;
 padding:4px 10px;cursor:pointer;margin:4px 4px 0 0;line-height:1.4}
button:hover:not(:disabled){background:#383d44}button:disabled{opacity:.4;cursor:default}
button.on{background:var(--acc-bg);border-color:var(--acc)}
button.go{background:var(--acc);border-color:var(--acc);color:#fff;font-weight:600}button.go:hover:not(:disabled){background:#6aa0ff}
:focus-visible{outline:2px solid var(--acc);outline-offset:2px}
input,select{font:inherit;font-size:12px;background:var(--bg);color:var(--fg);border:1px solid var(--line);border-radius:5px;padding:3px 6px}
input.pt{width:44px;font-variant-numeric:tabular-nums}
#stop{background:var(--bad);border-color:var(--bad);color:#fff;font-weight:700;font-size:14px;letter-spacing:.04em;padding:7px 24px;margin:0 0 0 auto}
#stop:hover{background:#f05a5f}
.meter{height:6px;background:var(--raised);border-radius:3px;position:relative;margin:8px 0 4px}
.meter i{position:absolute;left:0;top:0;bottom:0;border-radius:3px;background:var(--acc)}
.meter s{position:absolute;top:-4px;bottom:-4px;width:2px;background:var(--warn)}
.meter u{position:absolute;top:-4px;bottom:-4px;width:2px;background:var(--ok)}
.sw{display:inline-block;width:16px;height:16px;border-radius:3px;vertical-align:middle;border:1px solid var(--line);margin-right:6px}
svg.spark{width:100%;height:30px;display:block;margin-top:6px;background:linear-gradient(var(--line),var(--line)) 0 100%/100% 1px no-repeat}
#mapc{width:100%;display:block}
#motors .card{min-height:120px}#sensors .card{min-height:172px}
#msg{position:fixed;left:50%;bottom:18px;transform:translateX(-50%);z-index:30;max-width:min(900px,92vw);background:var(--warn-bg);
 color:var(--warn);border:1px solid #5a4620;border-radius:var(--r);padding:8px 14px;box-shadow:0 6px 20px #0008}
#msg:empty{display:none}
.maptools{font-size:12px;color:var(--dim);display:none;margin:0 0 6px}
.mapwrap{background:var(--surf);border:1px solid var(--line);border-radius:var(--r);padding:8px 10px}
.banner{background:var(--warn-bg);color:var(--warn);border-radius:var(--r);padding:8px 12px;display:none}
.pad{display:grid;grid-template-columns:repeat(3,46px);gap:4px;justify-content:center;margin:4px 0 8px}
.pad button{margin:0;height:38px;font-size:15px;touch-action:none;user-select:none}
#con{height:240px;overflow:auto;background:var(--bg);border:1px solid var(--line);border-radius:5px;padding:8px 10px;font:12px/1.45 var(--m);white-space:pre-wrap;margin:0}
#con .q{color:var(--warn)}#con .h{color:#8ab4f8}
#ask{display:none;gap:8px;align-items:center;background:var(--surf);border:1px solid var(--line);color:var(--dim);border-radius:var(--r);padding:5px 8px 5px 12px}
#ask.shown{display:flex}#ask button{margin:0}
#ask.live{background:var(--warn-bg);border-color:#5a4620;color:var(--warn)}
#askq{font-weight:600;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;min-width:0;flex:0 1 auto}
#ask:not(.live) #askq{font-weight:400}
#mode{white-space:nowrap;overflow:hidden;text-overflow:ellipsis;max-width:380px;font-weight:600}
#asktxt{flex:1 1 120px;min-width:80px}#simmap{max-width:240px}
.sep{border-top:1px solid var(--line);margin:10px 0}
#drawer{position:fixed;top:0;right:0;height:100vh;width:min(640px,96vw);background:var(--surf);border-left:1px solid var(--line);box-shadow:-8px 0 24px #0009;z-index:20;
 transform:translateX(105%);transition:transform .15s;display:flex;flex-direction:column}
#drawer.open{transform:none}
@media (prefers-reduced-motion:reduce){#drawer{transition:none}}
.dhead{padding:10px 12px;border-bottom:1px solid var(--line);display:grid;gap:6px}
.dbody{flex:1;overflow:auto;padding:4px 12px 20px}
.dfoot{padding:8px 12px;border-top:1px solid var(--line);display:flex;gap:8px;align-items:center}
.crow{display:grid;grid-template-columns:minmax(150px,210px) 1fr;gap:2px 8px;align-items:center;margin-top:8px}
.crow code{overflow-wrap:anywhere}
.crow input,.crow textarea{width:100%;font:12px var(--m);background:var(--bg);color:var(--fg);border:1px solid var(--line);border-radius:4px;padding:3px 5px}
.crow .changed{border-color:var(--warn);background:var(--warn-bg)}
.chelp{grid-column:1/3;color:var(--dim);font-size:11px}
.cf{margin:18px 0 0;font-size:14px}.cs{color:var(--acc);font-size:12px;font-weight:600;margin-top:12px;border-bottom:1px solid var(--line);padding-bottom:2px}
#stepsets{margin-top:4px}#stepsets code{font-size:11px;color:#cfd3d8;margin-right:6px}
</style></head><body><div class="wrap">
<div class="head"><div class="bar">
 <span class="grp"><span id="conn"><span class="dot off"></span>connecting</span><b id="mode">-</b>
  <span class="kv">Battery<b id="bat">-</b></span></span>
 <span class="grp"><span class="lbl" title="Which attachments are on the robot: names the turret / dump / belt motors, and which hardware the small actions use. Jobs set it themselves.">Hardware</span><span class="seg"><button data-ph="1" title="Turret + ultrasonic (TURRET_PORT)">Phase 1</button><button data-ph="2" title="Dump (DUMP_PORT) + belt (ROLLER_MOTOR_PORT)">Phase 2</button></span></span>
 <span class="grp"><span class="lbl" title="Colour sensor mode for the cards and calibration. The robot code always uses red.">Colour</span><span class="seg"><button data-cm="red" title="Reflected red, 0-100 %: what the robot code uses">Red</button><button data-cm="rgb" title="Raw red/green/blue under the sensor's white light, to compare colours">RGB</button></span></span>
 <span id="simbar" class="grp" style="display:none"><span class="lbl">Map</span><select id="simmap"></select>
  <span class="lbl">Speed</span><select id="simspeed"><option value="1">1x</option><option value="2">2x</option><option value="4">4x</option><option value="8">8x</option></select>
  <button id="simreset" style="margin:0" title="Forget this map's simulated calibration and street map (they are kept across restarts in sim/session/)">Reset sim files</button></span>
 <span class="grp"><button id="cbtn" style="margin:0" title="See and change every tunable constant of the robot code">Constants</button>
 <button id="rel" style="margin:0" title="Stop reading the ports, so a script started over SSH can use the brick">Release hardware</button></span>
 <button id="stop" title="Stops the job or script, then brakes every motor (Esc)">STOP</button>
</div>
<div id="ask"><span style="white-space:nowrap">Robot asks</span><span id="askq"></span><input id="asktxt" placeholder="Type an answer, then Enter"><button id="asksend">Send</button><button data-ans="">Continue</button><button data-ans="y">Yes</button><button data-ans="n">No</button></div>
<div class="banner" id="banner"></div></div>
<div id="msg"></div>
<div class="main">
 <div class="col">
  <div class="card"><h3><span>Run</span><span id="jobst" class="dim"></span></h3>
   <div id="pts"></div>
   <button class="go" data-job="phase1">Phase 1</button><button class="go" data-job="phase2">Phase 2</button><button class="go" data-job="demo">Full demo</button>
   <div class="sep"></div>
   <div class="dim">Bring-up steps, in order (docs/BRINGUP.md)</div>
   <div><select id="step" style="max-width:100%;width:100%;margin-top:4px"></select></div>
   <button class="go" id="runstep">Run this step</button><button id="stepedit">Constants for this step</button>
   <div class="dim" id="stepsets"></div>
   <div class="dim" id="lastjob" style="margin-top:6px"></div>
  </div>
  <div class="card"><h3><span>Drive</span><span class="dim">Hold WASD or the arrows; W+D curves</span></h3>
   <div class="pad"><span></span><button data-drv="f" aria-label="forward">&#9650;</button><span></span><button data-drv="l" aria-label="turn left">&#9664;</button><button data-drv="s" aria-label="stop">&#9632;</button><button data-drv="r" aria-label="turn right">&#9654;</button><span></span><button data-drv="b" aria-label="back">&#9660;</button><span></span></div>
   <div class="dim" style="text-align:center">Speed <input id="spd" value="200" style="width:56px"> dps. Stops 0.35 s after you let go.</div>
   <div class="sep"></div>
   <button data-job="tile" data-args="1">1 tile forward</button><button data-job="tile" data-args="-1">1 tile back</button>
   <button data-job="turn" data-args="L">Turn left 90&deg;</button><button data-job="turn" data-args="R">Turn right 90&deg;</button>
   <div class="note">These use the robot code's own driving (odometry, ramps).</div>
  </div>
 </div>
 <div class="mapwrap"><label class="maptools" id="maptools" title="Tan houses and the grey robot are the simulator's truth: the robot can't see them. What the robot found is drawn in red/blue."><input type="checkbox" id="truth" checked> Show the simulator's truth (tan houses, grey robot). The robot only knows what it has found (red houses, blue robot).</label>
  <canvas id="mapc"></canvas></div>
</div>
<h2 class="sec">Sensors</h2>
<div class="grid4" id="sensors"></div>
<h2 class="sec">Motors</h2>
<div class="grid4" id="motors"></div>
<div class="card"><h3><span>Console</span><span class="dim">Everything the jobs print. Their questions appear under the top bar.</span></h3>
 <pre id="con"></pre>
</div>
</div>
<div id="drawer"><div class="dhead">
 <div style="display:flex;align-items:center;gap:8px"><b style="font-size:14px">Constants</b><span class="dim" id="cnote"></span><button id="cclose" style="margin-left:auto">Close</button><button id="stop2" style="background:var(--bad);border-color:var(--bad);color:#fff;font-weight:700;padding:6px 16px" title="Esc">STOP</button></div>
 <input id="cq" placeholder="Search a name or a word from its comment (e.g. ARC, turret, threshold)">
 <div class="dim">Saved straight into the .py files (old copy in logs/constants_backup/, every change in logs/constants_changes.log). A running job keeps its values; the next run uses the new ones.</div>
</div><div class="dbody" id="clist"></div>
<div class="dfoot"><span id="ccount" class="dim">no changes</span><button id="csave" class="go" style="margin-left:auto">Save</button><button id="cdiscard">Discard</button></div></div>
<script>
const $=id=>document.getElementById(id);let S=null,fails=0;const HL=100;
// every request gives up after a few seconds: with flaky wifi the page recovers as soon as it is back
function tfetch(u,o,ms){const c=new AbortController();const t=setTimeout(()=>c.abort(),ms||2500);
 return fetch(u,Object.assign({signal:c.signal},o||{})).finally(()=>clearTimeout(t));}
// one message line of fixed height: an error from a click shows for 5 s, otherwise the dashboard's own message
let note='',noteUntil=0;
function say(t){note=t;noteUntil=Date.now()+5000;showMsg();}
function showMsg(){const t=Date.now()<noteUntil?note:((S&&S.message)||'');if($('msg').textContent!==t)$('msg').textContent=t;}
async function ctl(o){try{const r=await tfetch('/api/ctl',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(o)});
 const j=await r.json();if(!j.ok&&j.error)say(j.error);return j;}catch(e){say('command failed: '+e);}}
function stop(){drvEnd();ctl({cmd:'stop'});}
$('stop').onclick=stop;$('stop2').onclick=stop;
document.querySelectorAll('[data-ph]').forEach(b=>b.onclick=()=>ctl({cmd:'phase',v:+b.dataset.ph}));
document.querySelectorAll('[data-cm]').forEach(b=>b.onclick=()=>ctl({cmd:'colour_mode',v:b.dataset.cm}));
$('rel').onclick=()=>ctl({cmd:'release',v:!(S&&S.released)});
$('simmap').onchange=e=>{ctl({cmd:'sim_map',v:e.target.value});e.target.blur();};
$('simspeed').onchange=e=>{ctl({cmd:'sim_speed',v:+e.target.value});e.target.blur();};
$('step').onchange=e=>e.target.blur();
$('simreset').onclick=()=>{if(confirm('Forget the simulated calibration and street map of the '+S.sim_map+' map?'))ctl({cmd:'sim_reset'});};
$('truth').onchange=e=>ctl({cmd:'show_truth',v:e.target.checked});
function ptArgs(){if(S&&S.sim)return[];const a=[];for(const k of['s1','s2','d1','d2'])for(const i of[0,1])a.push($('p_'+k+i).value.trim()||'0');return a;}
document.addEventListener('click',e=>{
 const c=e.target.closest('[data-cal]');if(c)ctl({cmd:'cal',which:c.dataset.cal,port:+c.dataset.port});
 const j=e.target.closest('[data-job]');if(j&&!j.disabled){const n=j.dataset.job;let args=j.dataset.args!=null?[j.dataset.args]:[];if(n==='phase1'||n==='demo')args=ptArgs();ctl({cmd:'job',name:n,args});}
 const m=e.target.closest('[data-mc]');if(m&&!m.disabled){const p=m.dataset.port,k=m.dataset.mc;
  if(k==='jog')ctl({cmd:'jog',port:p,deg:+m.dataset.v});else if(k==='dps')ctl({cmd:'motor_dps',port:p,dps:+m.dataset.v});else if(k==='pow')ctl({cmd:'motor_power',port:p,power:+m.dataset.v});else if(k==='reset')ctl({cmd:'reset_enc',port:p});}
 const a=e.target.closest('[data-ans]');if(a)answer(a.dataset.ans);
 const t=e.target.closest('[data-setthr]');if(t&&confirm('Set LINE_THRESHOLD = '+t.dataset.setthr+' in snowplow_core.py? (Both colour sensors share it: check the other one too.)'))
  ctl({cmd:'constants_save',file:'core',changes:{LINE_THRESHOLD:t.dataset.setthr}}).then(j=>{if(j&&j.ok)say('LINE_THRESHOLD = '+t.dataset.setthr+' saved');});});
$('runstep').onclick=()=>{const st=S.steps[+$('step').value];let args=st.args;if(st.job==='phase1'||st.job==='demo')args=ptArgs();ctl({cmd:'job',name:st.job,args});};
// a click within 0.4 s of a new question is the tail of a double-click on the previous one: ignore it
function answer(t){if(!S||S.prompt==null)return;if(Date.now()-qShown<400)return;ctl({cmd:'answer',text:t,id:S.prompt_id});$('asktxt').value='';$('asktxt').blur();}
$('asksend').onclick=()=>answer($('asktxt').value);$('asktxt').addEventListener('keydown',e=>{if(e.key==='Enter')answer($('asktxt').value);});
// ---- hand driving, like a game: W/S (or up/down) = forward/back, A/D = turn, both = curve. The newest key wins
// on each axis (hold W, press S: backward; let go of S: forward again). Sent every 100 ms while held; the
// server stops the wheels 0.35 s after the last one, so a lost connection stops the robot.
const KEYS={w:'f',arrowup:'f',s:'b',arrowdown:'b',a:'l',arrowleft:'l',d:'r',arrowright:'r'};
const held=[];let drvTimer=null,driving=false;
function drvVec(){let f=0,t=0;for(const d of held){if(d==='f')f=1;else if(d==='b')f=-1;else if(d==='l')t=-1;else if(d==='r')t=1;}
 if(!f&&!t)return null;const v=Math.max(0,Math.min(500,+$('spd').value||200));
 if(!f)return [0.6*v*t,-0.6*v*t];                       // spin in place
 const k=(f>0?t:-t)*0.35*v;return [f*v+k,f*v-k];}        // curve; reversing steers like a car
function drvSend(){const v=drvVec();if(v)ctl({cmd:'drive',l:Math.round(v[0]),r:Math.round(v[1])});}
function drvUpdate(){if(drvVec()){drvSend();if(!drvTimer)drvTimer=setInterval(drvSend,100);driving=true;}
 else{clearInterval(drvTimer);drvTimer=null;if(driving){driving=false;ctl({cmd:'drive',l:0,r:0});}}}
function press(d){const i=held.indexOf(d);if(i>=0)held.splice(i,1);held.push(d);drvUpdate();}
function release(d){const i=held.indexOf(d);if(i>=0){held.splice(i,1);drvUpdate();}}
function drvEnd(){held.length=0;drvUpdate();}
document.querySelectorAll('[data-drv]').forEach(b=>{const d=b.dataset.drv;
 b.addEventListener('pointerdown',e=>{e.preventDefault();if(b.disabled)return;b.setPointerCapture(e.pointerId);if(d==='s')drvEnd();else press(d);});
 ['pointerup','pointercancel','lostpointercapture'].forEach(ev=>b.addEventListener(ev,()=>{if(d!=='s')release(d);}));});
// keys go to the robot unless you are typing in a box (the speed box only takes digits, so it doesn't count)
const typing=()=>{const a=document.activeElement;return !!a&&a.id!=='spd'&&(a.tagName==='TEXTAREA'||(a.tagName==='INPUT'&&a.type!=='checkbox'));};
document.addEventListener('keydown',e=>{if(e.key==='Escape'){stop();e.preventDefault();return;}
 const d=KEYS[e.key.toLowerCase()];if(!d||typing()||e.ctrlKey||e.metaKey||e.altKey)return;
 e.preventDefault();if(!e.repeat)press(d);});
document.addEventListener('keyup',e=>{const d=KEYS[e.key.toLowerCase()];if(d)release(d);});
window.addEventListener('blur',drvEnd);document.addEventListener('visibilitychange',()=>{if(document.hidden)drvEnd();});
// ---- constants panel
let CD=null,cwant=null;
const esc=t=>String(t).replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
async function openConsts(names){$('drawer').classList.add('open');cwant=names||null;$('cq').value=names?names.join(' '):'';
 await loadConsts();renderConsts();}
async function loadConsts(){try{CD=await (await tfetch('/api/constants',{cache:'no-store'},5000)).json();}catch(e){say('could not load the constants: '+e);}}
function cmatch(name,help){const q=$('cq').value.trim();if(!q)return true;const ws=q.toLowerCase().split(/\s+/);
 if(cwant&&q===cwant.join(' '))return cwant.includes(name);
 return ws.some(w=>name.toLowerCase().includes(w))||(ws.length===1&&help.toLowerCase().includes(ws[0]));}
function crow(f,it){const multi=it.value.includes('\n');const v=esc(it.value);
 const box=multi?`<textarea rows="${Math.min(8,it.value.split('\n').length)}" data-f="${f}" data-n="${it.name}" data-o="${v}">${v}</textarea>`
  :`<input data-f="${f}" data-n="${it.name}" data-o="${v}" value="${v}">`;
 return `<div class="crow"><code title="line ${it.line||''}">${it.name}</code>${box}<div class="chelp">${esc(it.help||'')}</div></div>`;}
function renderConsts(){if(!CD)return;let h='';
 const cal=CD.calibration;const calItems=cal.keys.map(k=>({name:k,value:cal.values[k]!=null?String(cal.values[k]):'',help:''}));
 const calShow=calItems.filter(it=>cmatch(it.name,'calibration auto-tuner wheel track'))||[];
 const wantCal=cwant&&cwant.includes('calibration file');
 if(calShow.length||wantCal){h+=`<h4 class="cf">Auto-tuner results: ${esc(cal.path)}</h4><div class="chelp">${cal.exists?'These override WHEEL_RADIUS / scales / TRACK_WIDTH in snowplow_core.py.':'No file: the code\'s own values are used. Fill these in to create it.'}${CD.sim?' (the simulator\'s own file for this map)':''}</div>`;
  for(const it of (wantCal?calItems:calShow))h+=crow('cal',it);
  if(cal.exists)h+=`<button id="caldel">Delete this file (use the code's values)</button>`;}
 for(const f of CD.files){let fh='';for(const sec of f.sections){const items=sec.items.filter(it=>cmatch(it.name,it.help||''));
   if(items.length)fh+=`<div class="cs">${esc(sec.section)}</div>`+items.map(it=>crow(f.id,it)).join('');}
  if(fh)h+=`<h4 class="cf">${esc(f.title)}</h4>`+fh;if(f.error)h+=`<div class="warn">${esc(f.error)}</div>`;}
 $('clist').innerHTML=h||'<div class="dim" style="margin-top:10px">nothing matches</div>';
 $('cnote').textContent=CD.sim?'(files on this computer)':'(files on the robot)';
 const del=$('caldel');if(del)del.onclick=async()=>{if(confirm('Delete '+cal.path+'?')){await ctl({cmd:'calibration_delete'});await loadConsts();renderConsts();}};
 ccount();}
function cchanged(){return [...$('clist').querySelectorAll('[data-n]')].filter(e=>e.value!==e.dataset.o);}
function ccount(){const ch=cchanged();$('clist').querySelectorAll('[data-n]').forEach(e=>e.classList.toggle('changed',e.value!==e.dataset.o));
 $('ccount').textContent=ch.length?ch.length+' unsaved change'+(ch.length>1?'s':'')+': '+ch.map(e=>e.dataset.n).join(', '):'no changes';$('csave').disabled=!ch.length;}
$('clist').addEventListener('input',ccount);
$('cq').addEventListener('input',()=>{cwant=null;const keep=cchanged().length;if(keep&&!confirm('Searching redraws the list and drops unsaved changes. Continue?'))return;renderConsts();});
$('cbtn').onclick=()=>openConsts();$('cclose').onclick=()=>{if(cchanged().length&&!confirm('Close without saving?'))return;$('drawer').classList.remove('open');};
$('cdiscard').onclick=()=>renderConsts();
$('csave').onclick=async()=>{const ch=cchanged();const by={};for(const e of ch)(by[e.dataset.f]=by[e.dataset.f]||{})[e.dataset.n]=e.value;
 const names=[];for(const f in by){const j=f==='cal'?await ctl({cmd:'calibration_save',values:by[f]}):await ctl({cmd:'constants_save',file:f,changes:by[f]});
  if(!j||!j.ok)return;names.push(...Object.keys(by[f]));}
 await loadConsts();renderConsts();say('Saved '+names.join(', ')+(S&&S.job?': the running job keeps the old values, the next run uses these':': the next run uses these'));};
$('stepedit').onclick=()=>{const st=S&&S.steps[+$('step').value];openConsts(st&&st.sets.length?st.sets:null);};
// ---- cards
const f1=v=>v==null?'-':(+v).toFixed(1);
// History is one point per dashboard sample (10 Hz), newest on the right; null = no fresh reading (drawn as a gap).
function spark(h,lo,hi,marks){const ok=(h||[]).filter(v=>v!=null);if(!h||h.length<2)return'';
 lo=lo??(ok.length?Math.min(...ok):0);hi=hi??(ok.length?Math.max(...ok):1);if(hi-lo<1e-6)hi=lo+1;
 const y=v=>(26-24*(Math.min(hi,Math.max(lo,v))-lo)/(hi-lo)).toFixed(1);const off=HL-h.length;
 const segs=[];let cur=[];h.forEach((v,i)=>{if(v==null){if(cur.length)segs.push(cur);cur=[];}else cur.push((100*(i+off)/(HL-1)).toFixed(1)+','+y(v));});
 if(cur.length)segs.push(cur);
 const pts=segs.map(sg=>sg.length===1?sg[0]+' '+sg[0]:sg.join(' '));
 const mk=(marks||[]).filter(m=>m&&m[0]!=null).map(m=>`<line x1="0" x2="100" y1="${y(m[0])}" y2="${y(m[0])}" stroke="${m[1]}" stroke-dasharray="2 2" stroke-width=".8"/>`).join('');
 return `<svg class="spark" viewBox="0 0 100 28" preserveAspectRatio="none">${mk}${pts.map(p=>`<polyline fill="none" stroke="#4c8dff" stroke-width="1.5" stroke-linecap="round" vector-effect="non-scaling-stroke" points="${p}"/>`).join('')}</svg>`;}
function motorButtons(p,role,dis){const d=dis?'disabled':'';const B=(k,v,t,cls)=>`<button ${d} ${cls?`class="${cls}"`:''} data-mc="${k}" data-port="${p}" data-v="${v}">${t}</button>`;
 const jog=B('jog',-30,'&minus;30&deg;')+B('jog',30,'+30&deg;');
 if(role==='Dump'){const dp=S.dump_plus,dm=S.dump_minus;
  const db=(sg,w,t)=>w?`<button ${d} class="go" data-job="dump" data-args="${sg}" title="Full dump with the motor turning ${t}: the bin pours ${String(w).toLowerCase()}">Dump (${t}, ${String(w).toLowerCase()})</button>`
   :`<button disabled title="DUMP_${sg==='+'?'PLUS':'MINUS'} is None: the bin can't tip that way (use the jog buttons to explore)">Dump ${t}: off</button>`;
  return db('+',dp,'+')+db('-',dm,'&minus;')+B('jog',-15,'&minus;15&deg;')+B('jog',15,'+15&deg;')+B('reset',0,'Reset enc');}
 if(role==='Turret')return `<button ${d} class="go" data-job="scan">Scan L/F/R</button>`+B('jog',-15,'&minus;15&deg;')+B('jog',15,'+15&deg;')+B('reset',0,'Reset enc');
 if(role==='Belt'){const pw=S.belt_power;if(pw)return B('pow',pw,'On')+B('pow',-pw,'Reverse')+B('pow',0,'Off')+jog+B('reset',0,'Reset enc');
  const v=S.belt_dps;return B('dps',v,'On')+B('dps',-v,'Reverse')+B('dps',0,'Off')+jog+B('reset',0,'Reset enc');}
 return jog+B('reset',0,'Reset enc');}
const head=(port,label)=>`<span class="port">${port}</span>${label}`;
const kv=(k,v)=>`<span class="kv">${k}<b>${v}</b></span>`;
function motorCard(p,m,role,src,dis){
 const h=head(p,role||'not used'),btns=motorButtons(p,role,dis);
 if(src==='none'||!m.hasOwnProperty('enc'))return {head:h,body:'<div class="dim">Not read right now</div>',btns};
 const w=(m.warn||[]).map(t=>`<span class="warn">${t}</span>`).join(' ');
 return {head:h,btns,body:`<div class="big">${m.enc==null?'-':m.enc}<span class="u">deg</span></div>
 <div class="kvs">${kv('Speed',(m.dps==null?'-':m.dps)+' dps')}${kv('Power',m.power==null?'-':m.power+' %')}</div>${w?`<div>${w}</div>`:''}`};}
// Cards are built once; each part (title, values, buttons, footer) is only rewritten when its HTML changes, so a
// button is never replaced under the mouse (a click that lands on a replaced button is lost).
function put(el,html){if(el._h!==html){el._h=html;el.innerHTML=html;}}
function setCards(id,list){const box=$(id);
 while(box.children.length<list.length){const c=document.createElement('div');c.className='card';
  c.innerHTML='<h3></h3><div></div><div></div><div></div>';box.appendChild(c);}
 list.forEach((it,i)=>{const c=box.children[i].children;put(c[0],'<span>'+it.head+'</span>');put(c[1],it.body||'');put(c[2],it.btns||'');put(c[3],it.foot||'');});}
function statusDot(st){return st==='VALID_DATA'?'<span class="dot ok"></span>':(st==='CONFIGURING'?'<span class="dot warnb"></span>':(st==='NOT_READ'||st==='NOT_SET_UP'?'<span class="dot off"></span>':'<span class="dot badb"></span>'));}
const STXT={VALID_DATA:'Reading',NOT_CONFIGURED:'Not configured',CONFIGURING:'Starting up',NO_DATA:'No data (plugged in?)',I2C_ERROR:'I2C error',NOT_SET_UP:'Not set up yet',NOT_READ:'Not read by the job yet'};
function calButtons(port){return `<button data-cal="table" data-port="${port}">Store as table</button><button data-cal="line" data-port="${port}">Store as line</button><button data-cal="clear" data-port="${port}">Clear</button>`;}
function sensorCard(port,s,src){
 const h=head(port,s.label);
 if(s.kind==='none')return {head:h,body:'<div class="dim"><span class="dot off"></span>Nothing configured on this port</div>'};
 if(src==='none')return {head:h,body:'<div class="dim">Not read right now</div>'};
 const st=s.status||'-';let stl=statusDot(st)+(STXT[st]||st);
 if(s.stale)stl=`<span class="dot off"></span>Last read by the job ${f1(s.age)} s ago`+(s.kind==='colour'?' (it only looks near grid lines)':'');
 if(st!=='VALID_DATA')return {head:h,body:`<div class="${st==='NOT_READ'?'dim':''}">${stl}</div>`};
 if(s.kind==='us')return {head:h,body:`<div class="big${s.stale?' dim':''}">${s.value>=250?'No echo':f1(s.value)+'<span class="u">cm</span>'}</div><div class="note">${stl}</div>`,
  foot:spark(s.hist,0,100)+`<div class="note">Last 10 s, 0 to 100 cm</div>`};
 const cal=s.cal||{};
 if(s.rgb){const [r,g,b]=s.rgb;const mx=Math.max(r,g,b,1);const css=`rgb(${(255*r/mx)|0},${(255*g/mx)|0},${(255*b/mx)|0})`;const j=s.judge;
  const jv=j?`<div class="meter"><i style="width:${Math.max(0,Math.min(100,j.f*100))}%"></i><s style="left:50%"></s></div>
   <div class="kvs"><span class="kv"><b style="margin:0">${j.verdict}</b></span>${kv('Toward line',j.f.toFixed(2))}${kv('Off-axis',j.off.toFixed(2))}</div>`:`<div class="note">Store a table and a line colour to compare.</div>`;
  return {head:h,body:`<div class="kvs" style="align-items:center"><span class="sw" style="background:${css}"></span>${kv('R',f1(r))}${kv('G',f1(g))}${kv('B',f1(b))}</div>
   <div class="note">Raw RGB under the sensor's white light, 0 to about 1020 each${S.sim?' (made up in the simulator)':''}.</div>${jv}`,
   btns:src==='ports'?calButtons(port):'',
   foot:`<div class="kvs">${kv('Table',cal.table?cal.table.join(' / '):'-')}${kv('Line',cal.line?cal.line.join(' / '):'-')}</div>${spark(s.hist,0,null)}`};}
 const th=S.line_threshold;const sug=s.suggest;
 const calTxt=(cal.red_table!=null||cal.red_line!=null)?kv('Table',f1(cal.red_table))+kv('Line',f1(cal.red_line))+(sug!=null?`<span class="kv good">Threshold<b class="good">${f1(sug)} &plusmn; ${f1(s.margin)}</b></span>`:'')
  :'<span class="note" style="margin:0">Hold it over the table, then a line, and press the buttons.</span>';
 return {head:h,body:`<div class="big${s.stale?' dim':''}">${f1(s.value)}<span class="u">%</span><span class="tag${s.line&&!s.stale?' hot':''}">${s.line?'LINE':'table'}</span></div>
  <div class="meter" title="Reflected red, 0-100 %. Orange mark: LINE_THRESHOLD; green: suggested"><i style="width:${Math.max(0,Math.min(100,s.value))}%"></i><s style="left:${th}%"></s>${sug!=null?`<u style="left:${sug}%"></u>`:''}</div>
  <div class="note">${stl}. The code calls it a line below <span class="warn">${th}</span>.</div>`,
  btns:src==='ports'?calButtons(port)+(sug!=null&&Math.round(sug)!==th?`<button class="go" data-setthr="${Math.round(sug)}">Use ${Math.round(sug)} as LINE_THRESHOLD</button>`:''):'',
  foot:`<div class="kvs">${calTxt}</div>${spark(s.hist,0,100,[[th,'#f2b33d'],[sug,'#3fb27f']])}`};}
let ptsBuilt=null,stepsBuilt=false,lastQ=null,qShown=0;
function buildPts(){const sim=S.sim;const key=sim?'sim:'+S.sim_map:'real';if(ptsBuilt===key)return;ptsBuilt=key;
 const G='display:grid;grid-template-columns:62px 1fr 1fr;gap:4px 8px;align-items:center;font-size:12px';
 if(sim){const p=S.sim_points,c=v=>`<span class="num">${v.join(', ')}</span>`;
  $('pts').innerHTML=`<div style="${G}"><span class="lbl">Gate</span><span>s1 ${c(p.s1)}</span><span>s2 ${c(p.s2)}</span>
   <span class="lbl">Drop-off</span><span>d1 ${c(p.d1)}</span><span>d2 ${c(p.d2)}</span></div><div class="note">From the ${S.sim_map} map.</div>`;return;}
 const d=S.points||{};const row=k=>`<span>${k} <input class="pt" id="p_${k}0" value="${(d[k]||[0,0])[0]}"><input class="pt" id="p_${k}1" value="${(d[k]||[0,0])[1]}"></span>`;
 $('pts').innerHTML=`<div style="${G}"><span class="lbl">Gate</span>${row('s1')}${row('s2')}<span class="lbl">Drop-off</span>${row('d1')}${row('d2')}</div>`+
  (d.saved?`<div class="note">Last used values (saved when Phase 1 starts).</div>`:`<div class="note warn">Sample-map values: type the TA's gate and drop-off before Phase 1.</div>`);}
function render(){const s=S;const dis=!!s.busy;
 if(!stepsBuilt&&s.steps){$('step').innerHTML=s.steps.map((st,i)=>`<option value="${i}">${st.label}</option>`).join('');stepsBuilt=true;}
 const sst=s.steps&&s.steps[+$('step').value];put($('stepsets'),sst&&sst.sets.length?'Tunes '+sst.sets.map(n=>`<code>${n}</code>`).join(''):'');
 if(s.sim){$('simbar').style.display='';const sm=$('simmap'),mk=s.sim_maps.join(',');
  if(sm.dataset.k!==mk){sm.dataset.k=mk;const L={'random':'random: new board','random-edge':'random: new, street may run along the edge','random-18x5':'random: new, 2 tables wide (18x5)','random-9x10':'random: new, 2 tables tall (9x10)'};
   sm.innerHTML=s.sim_maps.map(m=>`<option value="${m}">${L[m]||m}</option>`).join('');}
  if(document.activeElement!==$('simmap'))$('simmap').value=s.sim_map;if(document.activeElement!==$('simspeed'))$('simspeed').value=String(s.sim_speed);}
 buildPts();
 $('mode').textContent=s.job?(s.prompt!=null?'Waiting for you: ':'Running: ')+s.job.label:(s.scripts.length?'Script running outside the dashboard':(s.released?'Hardware released':(s.sim?'Idle':'Idle, reading the ports')));
 $('bat').textContent=s.battery==null?'-':s.battery.toFixed(2)+' V';$('bat').parentNode.style.display=s.sim?'none':'';
 document.querySelectorAll('[data-ph]').forEach(b=>b.classList.toggle('on',+b.dataset.ph===s.phase));
 document.querySelectorAll('[data-cm]').forEach(b=>b.classList.toggle('on',b.dataset.cm===s.colour_mode));
 $('rel').textContent=s.released?'Take hardware back':'Release hardware';$('rel').disabled=!!s.job||s.scripts.length>0;
 const bn=$('banner');let bt='';
 if(s.scripts.length)bt='Running outside the dashboard: '+s.scripts.map(x=>x.name+' (pid '+x.pid+')').join(', ')+'. The dashboard is not touching the brick; STOP still stops it.';
 else if(s.released)bt='Hardware released: the dashboard is not touching the brick until you take it back.';
 bn.style.display=bt?'block':'none';bn.textContent=bt;
 showMsg();
 setCards('motors','ABCD'.split('').map(p=>motorCard(p,s.motors[p]||{},s.motor_roles[p]||'',s.source,dis)));
 setCards('sensors',['1','2','3','4'].map(p=>sensorCard(p,s.sensors[p],s.source)));
 $('maptools').style.display=s.sim?'block':'none';if(document.activeElement!==$('truth'))$('truth').checked=s.show_truth;
 document.querySelectorAll('[data-job],[data-drv],#runstep,[data-ph]').forEach(b=>b.disabled=b.dataset.ph?!!s.job:dis);
 $('jobst').innerHTML=s.job?`<span class="warn">${s.job.label}, ${s.job.secs.toFixed(0)} s</span>`:'';
 $('lastjob').textContent=s.last_job?'Last: '+s.last_job:'';
 const ask=$('ask'),was=ask.classList.contains('live');ask.classList.toggle('live',s.prompt!=null);
 ask.classList.toggle('shown',!!s.job||s.prompt!=null);   // only while a job runs: the row appears after you start one
 if(s.prompt_id!==lastQ){lastQ=s.prompt_id;qShown=Date.now();$('asktxt').value='';}            // an old answer never carries over
 if(s.prompt!=null){$('askq').textContent=s.prompt;$('askq').title=s.prompt;if(!was&&!typing()&&!held.length)$('asktxt').focus();}
 else{$('askq').textContent='Nothing right now';$('askq').title='';if(document.activeElement===$('asktxt'))$('asktxt').blur();}
 ask.querySelectorAll('button,input').forEach(e=>e.disabled=s.prompt==null);
 ['simmap','simspeed','simreset'].forEach(id=>$(id).disabled=!!s.job);
 $('rel').style.display=s.sim?'none':'';
 if(s.console_n!==conN)pullConsole();}
let conN=0,conBusy=false;
async function pullConsole(){if(conBusy)return;conBusy=true;try{const r=await tfetch('/api/console?since='+conN,{cache:'no-store'});const j=await r.json();const c=$('con');
 const atEnd=c.scrollTop+c.clientHeight>=c.scrollHeight-20;if(j.reset)c.textContent='';
 for(const l of j.lines){const sp=document.createElement('span');if(l.startsWith('? '))sp.className='q';else if(l.startsWith('==='))sp.className='h';sp.textContent=l+'\n';c.appendChild(sp);}
 while(c.childNodes.length>800)c.removeChild(c.firstChild);conN=j.n;if(atEnd)c.scrollTop=c.scrollHeight;}catch(e){}
 conBusy=false;}
async function poll(){try{const r=await tfetch('/api/state',{cache:'no-store'});if(r.status===401){location.reload();return;}
 S=await r.json();fails=0;put($('conn'),'<span class="dot ok"></span>'+(S.sim?'Simulator':'Connected'));render();}
 catch(e){if(++fails>3){$('conn').innerHTML='<span class="dot badb"></span>Dashboard not reachable (wifi?). The robot keeps going; hand driving stopped.';drvEnd();}}
 setTimeout(poll,100);}
// ---- map (same drawing commands as visualizer.py --web)
const cv=$('mapc'),g=cv.getContext('2d');let fn=-1,lastF=null;
function size(){const d=window.devicePixelRatio||1;cv.style.height=Math.round(cv.clientWidth*MAPH/MAPW)+'px';
 cv.width=cv.clientWidth*d;cv.height=cv.clientHeight*d;if(lastF)paint(lastF);}
const MAPW=__MAPW__,MAPH=__MAPH__;
window.addEventListener('resize',size);
const AN={nw:['left','top'],n:['center','top'],ne:['right','top'],w:['left','middle'],center:['center','middle'],e:['right','middle'],sw:['left','bottom'],s:['center','bottom'],se:['right','bottom']};
// visualizer.py's dark background and panel colours, matched to this page's surfaces
const RECOL={'#1e1f24':'#23262b','#26282e':'#2a2e34'};const rc=c=>RECOL[c]||c;
function path(f){g.beginPath();g.moveTo(f[0],f[1]);for(let i=2;i<f.length;i+=2)g.lineTo(f[i],f[i+1]);}
function paint(fr){const d=window.devicePixelRatio||1;g.setTransform(d,0,0,d,0,0);g.clearRect(0,0,cv.width,cv.height);
 if(fr.w&&fr.h){const sc=Math.min(cv.clientWidth/fr.w,cv.clientHeight/fr.h);g.setTransform(d*sc,0,0,d*sc,0,0);}
 g.lineCap='round';g.lineJoin='round';
 for(const c of fr.cmds){const k=c[0];
  if(k==='r'){const[_,x0,y0,x1,y1,fill,ol,w,h]=c;const x=Math.min(x0,x1),y=Math.min(y0,y1),W=Math.abs(x1-x0),H=Math.abs(y1-y0);
   if(fill){g.globalAlpha=h?0.45:1;g.fillStyle=rc(fill);g.fillRect(x,y,W,H);g.globalAlpha=1;}if(ol){g.setLineDash([]);g.strokeStyle=ol;g.lineWidth=w;g.strokeRect(x,y,W,H);}}
  else if(k==='p'){const[_,f,fill,ol,w,dash]=c;path(f);g.closePath();if(fill){g.fillStyle=rc(fill);g.fill();}if(ol){g.setLineDash(dash);g.strokeStyle=ol;g.lineWidth=w;g.stroke();g.setLineDash([]);}}
  else if(k==='l'){const[_,f,col,w,dash,arrow]=c;path(f);g.setLineDash(dash);g.strokeStyle=col;g.lineWidth=w;g.stroke();g.setLineDash([]);
   if(arrow){const m=f.length,ax=f[m-4],ay=f[m-3],bx=f[m-2],by=f[m-1],a=Math.atan2(by-ay,bx-ax),s=4+2*w;
    g.beginPath();g.moveTo(bx,by);g.lineTo(bx-s*Math.cos(a-0.45),by-s*Math.sin(a-0.45));g.lineTo(bx-s*Math.cos(a+0.45),by-s*Math.sin(a+0.45));g.closePath();g.fillStyle=col;g.fill();}}
  else if(k==='d'){const[_,x,y,r,fill,ol]=c;g.beginPath();g.arc(x,y,r,0,7);g.fillStyle=fill;g.fill();if(ol&&ol!==fill){g.strokeStyle=ol;g.lineWidth=1;g.stroke();}}
  else if(k==='t'){const[_,x,y,s,col,sz,an,b]=c;const a=AN[an]||AN.nw;g.font=(b?'bold ':'')+Math.round(sz*1.3)+'px monospace';g.textAlign=a[0];g.textBaseline=a[1];g.fillStyle=col;g.fillText(s,x,y);}}}
async function pollMap(){try{const r=await tfetch('/api/frame?n='+fn,{cache:'no-store'});if(r.status===200){const fr=await r.json();fn=fr.n;lastF=fr;paint(fr);}}catch(e){}
 setTimeout(pollMap,150);}
size();poll();pollMap();
</script></body></html>"""

PAGE = PAGE.replace("__MAPW__", str(MAP_W)).replace("__MAPH__", str(MAP_H))

LOGIN = r"""<!doctype html><html><head><meta charset="utf-8"><title>Snowplow dashboard</title>
<meta name="viewport" content="width=device-width,initial-scale=1"><style>
body{margin:0;background:#1b1d21;color:#e6e8eb;font:15px system-ui,sans-serif;display:flex;height:100vh;align-items:center;justify-content:center}
form{background:#23262b;border:1px solid #373c43;padding:24px;border-radius:6px;display:grid;gap:10px}
input,button{font:inherit;font-size:18px;padding:8px;border-radius:5px;border:1px solid #373c43}input{background:#1b1d21;color:#e6e8eb;letter-spacing:.2em}
button{background:#4c8dff;border-color:#4c8dff;color:#fff;font-weight:600;cursor:pointer}:focus-visible{outline:2px solid #4c8dff;outline-offset:2px}
</style></head><body><form method="post" action="/login"><div>Access code (printed by dashboard.py)</div>
<input name="code" autofocus autocomplete="off" inputmode="numeric"><button>Open</button>%s</form></body></html>"""


# ============================================================ server
def make_server(hub, args, code, frames):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, status, body=b"", ctype="application/json", extra=None):
            self.send_response(status)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            for k, v in (extra or {}).items():
                self.send_header(k, v)
            self.end_headers()
            if body:
                self.wfile.write(body)

        def _authed(self):
            if code is None:
                return True
            for part in self.headers.get("Cookie", "").split(";"):
                k, _, v = part.strip().partition("=")
                if k == "dash" and hmac.compare_digest(v, code):
                    return True
            return False

        def do_GET(self):
            u = urlparse(self.path)
            q = parse_qs(u.query)
            try:
                if u.path == "/":
                    page = PAGE if self._authed() else LOGIN % ""
                    self._send(200, page.encode(), "text/html; charset=utf-8")
                elif not self._authed():
                    self._send(401, b'{"error":"access code needed"}')
                elif u.path == "/api/state":
                    hub.seen()
                    self._send(200, hub.snapshot)
                elif u.path == "/api/constants":
                    self._send(200, json.dumps(hub.constants()).encode())
                elif u.path == "/api/console":
                    try:
                        since = int(q.get("since", ["0"])[0])
                    except ValueError:
                        since = 0
                    self._send(200, json.dumps(hub.console_since(since)).encode())
                elif u.path == "/api/frame":
                    n = q.get("n", ["-1"])[0]
                    if not frames["frame"] or n == str(frames["n"]):
                        self._send(204)
                    else:
                        self._send(200, frames["frame"])
                else:
                    self._send(404, b"{}")
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_POST(self):
            u = urlparse(self.path)
            try:
                n = int(self.headers.get("Content-Length", "0") or 0)
                body = self.rfile.read(min(n, 65536)).decode(errors="replace")
                if u.path == "/login":
                    got = parse_qs(body).get("code", [""])[0].strip()
                    if code is None or hmac.compare_digest(got, code):
                        self._send(303, b"", "text/plain", {"Location": "/", "Set-Cookie":
                                   f"dash={code or ''}; Path=/; SameSite=Strict; HttpOnly"})
                    else:
                        self._send(200, (LOGIN % "<div style='color:#e04848'>Wrong code</div>").encode(),
                                   "text/html; charset=utf-8")
                    return
                if not self._authed():
                    self._send(401, b'{"error":"access code needed"}')
                    return
                if u.path == "/api/ctl":
                    try:
                        q = json.loads(body or "{}")
                    except ValueError:
                        q = {}
                    if not isinstance(q, dict):
                        q = {}
                    res = hub.command(q)
                    self._send(200, json.dumps(res).encode())
                else:
                    self._send(404, b"{}")
            except (BrokenPipeError, ConnectionResetError):
                pass

    server = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    server.daemon_threads = True
    return server


def main():
    ap = argparse.ArgumentParser(description="Snowplow dashboard: live ports, hand control, jobs, STOP.")
    ap.add_argument("--sim", nargs="?", const="sample", metavar="MAP",
                    help="simulated robot instead of the BrickPi (laptop); MAP = sample/long/u_reversed, or "
                         "random-SEED / random-18x5-SEED / random-9x10-SEED / random-edge-SEED")
    ap.add_argument("--port", type=int, default=8080, help="web port (default 8080)")
    ap.add_argument("--udp", type=int, default=5005, help="telemetry port the robot code sends to (default 5005)")
    ap.add_argument("--phase", type=int, choices=(1, 2), default=2, help="which hardware is on at start")
    ap.add_argument("--hz", type=float, default=10.0, help="how often to read the ports")
    ap.add_argument("--code", help="access code (default: random 4 digits, printed at start)")
    ap.add_argument("--no-code", action="store_true", help="no access code (default with --sim)")
    args = ap.parse_args()

    consts = read_constants()
    hw = SimHardware(consts, args.sim) if args.sim else RealHardware(consts)
    hub = Hub(hw, consts, args)
    if args.no_code or (args.sim and not args.code):
        code = None
    else:
        code = args.code or f"{random.SystemRandom().randrange(10000):04d}"

    # Map + phase clock from the robot's telemetry, drawn by visualizer.py's Scene.
    import visualizer
    player = visualizer.Player(argparse.Namespace(replay=None, sim=None, no_save=True, port=args.udp, speed=1.0,
                                                  explore=None, seed=0, noise=1.0, verbose=False))
    draw = visualizer.CmdDraw()
    # One fixed drawing size; every browser tab scales it to its own window (tabs no longer fight over it).
    scene = visualizer.Scene(draw, MAP_W, MAP_H, "dashboard" + (" (simulator)" if hw.sim else ""))
    frames = {"frame": b"", "n": 0}
    plain_apply = player.state.apply

    def apply(ev):                        # "hw" snapshots feed the cards, everything else the map
        if ev.get("k") == "hw":
            hub.on_hw(ev)
        else:
            if ev.get("k") == "meta":
                hub.on_meta(ev)
            plain_apply(ev)
    player.state.apply = apply
    if hw.sim:
        for ev in hw.board_events(hub.phase):
            player.state.apply(ev)

    try:
        server = make_server(hub, args, code, frames)
    except OSError as e:
        viewers = find_viewers()
        who = ("an old live viewer is running: " + "; ".join(f"pid {p} visualizer.py {c}" for p, c in viewers)
               + f" (stop it: kill {' '.join(str(p) for p, _ in viewers)})") if viewers else \
            "is the dashboard already running?"
        player.close()
        raise SystemExit(f"[DASH] can't use port {args.port} ({e}): {who}. Or use --port {args.port + 1}.")
    for pid, c in find_viewers():
        print(f"[DASH] warning: an old live viewer (pid {pid}, visualizer.py {c}) may take the robot's telemetry; "
              f"stop it with: kill {pid}", flush=True)
    threading.Thread(target=server.serve_forever, daemon=True).start()   # jobs start from these threads:
    threading.Thread(target=hub.run_sampler, daemon=True).start()        # they keep normal priority
    try:
        os.nice(5)                        # this (drawing) thread only: the robot's control loop comes first
    except (AttributeError, OSError):
        pass

    def quit_on(signum, frame):
        raise KeyboardInterrupt           # SIGTERM (kill): same clean exit as Ctrl-C

    def keep_running(signum, frame):
        # The SSH session that started us is gone (wifi dropped, terminal closed). Quitting would stop a
        # phase in the middle of the demo, so keep going; printing now goes to logs/dashboard.log.
        try:
            os.makedirs(os.path.join(HERE, "logs"), exist_ok=True)
            fd = os.open(os.path.join(HERE, "logs", "dashboard.log"), os.O_WRONLY | os.O_CREAT | os.O_APPEND)
            os.dup2(fd, 1)
            os.dup2(fd, 2)
            os.close(fd)
        except OSError:
            pass
        hub.log("[DASH] the SSH session that started the dashboard closed: still running "
                "(it keeps working; Ctrl-C there is gone, use STOP or `pkill -INT -f dashboard.py`)")
    for sig, fn in ((signal.SIGTERM, quit_on), (getattr(signal, "SIGHUP", None), keep_running)):
        try:
            if sig is not None:
                signal.signal(sig, fn)
        except (ValueError, OSError):
            pass
    urls = [f"http://{ip}:{args.port}" for ip in visualizer.local_addresses()] or [f"http://localhost:{args.port}"]
    where = f"http://localhost:{args.port}" if hw.sim else "  or  ".join(urls)
    print(f"[DASH] open {where}" + ("" if hw.sim else f"   (on the Pi itself: http://localhost:{args.port})"),
          flush=True)
    print(f"[DASH] access code: {code}" if code else "[DASH] no access code", flush=True)
    print("[DASH] Ctrl-C quits (a job started from the page is stopped first)", flush=True)
    hub.log("[DASH] ready" + (f" (simulator, {hw.map_name} map)" if hw.sim else ""))

    last = time.time()
    last_draw = 0.0
    try:
        while True:
            now = time.time()
            status = player.tick(now - last)
            last = now
            if hw.sim and hub.show_world and not hub.job:
                if hub.world_reset:                 # back from a job: show the dashboard's own board again
                    hub.world_reset = False
                    for ev in hw.board_events(hub.phase):
                        player.state.apply(ev)
                player.state.apply(hw.pose_event())
                player.dirty = True
            if player.dirty or now - last_draw > 1.0:
                st = player.state
                saved = (st.logs, st.sim, st.truth, st.truth_trail)
                st.logs = []                                    # the console has its own box
                if not hub.show_truth:                          # only what the robot itself knows
                    st.sim = dict(st.sim, boxes=[]) if st.sim else None
                    st.truth, st.truth_trail = None, []
                try:
                    scene.draw(st)
                finally:
                    st.logs, st.sim, st.truth, st.truth_trail = saved
                player.dirty = False
                last_draw = now
                frames["n"] += 1
                frames["frame"] = json.dumps({"n": frames["n"], "cmds": draw.cmds, "status": status,
                                              "w": scene.w, "h": scene.h}, separators=(",", ":")).encode()
            time.sleep(1.0 / 6.0)
    except KeyboardInterrupt:
        pass
    finally:
        hub.shutdown()
        server.shutdown()
        player.close()
        if not hw.sim and not hub.released and not find_robot_scripts():
            hw.stop_motors()              # brake only; never reset the brick


if __name__ == "__main__":
    main()
