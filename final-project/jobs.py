#!/usr/bin/env python3
"""
One robot job (a phase, a bench test, the auto-tuner or a small action) in its own process, started by
dashboard.py. Running it as a separate process is what makes STOP reliable: the dashboard can always
Ctrl-C or kill it, and the robot code never has to share the BrickPi with the dashboard.

You don't need to run this by hand, but you can:
  python3 jobs.py bench 5
  python3 jobs.py --sim sample --speed 2 phase1 1 3 1 2 2 -1 3 -1

  - Questions the robot code asks with input() are printed as a line starting with PROMPT_MARK and the
    answer is read from stdin (the dashboard's console box).
  - Telemetry (map, console, motor/sensor snapshots) goes over UDP to the dashboard.
  - --sim runs the same robot code against sim/world.py, paced in real time (--speed), with its own
    calibration and street map files in --session (never the real calibration_params.json / street_map.json).

This module's top level imports nothing from the robot code, so dashboard.py can import JOB_LABELS and
bench_test_names() without touching the hardware.
"""
from __future__ import annotations
import argparse
import ast
import builtins
import json
import math
import os
import signal
import socket
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_DIR = os.path.join(HERE, "sim")
PROMPT_MARK = "\x1ePROMPT "
EXIT_NEEDS_MAP = 3                # phase 2 asked for before phase 1 made a street map
T = 30.48
_real_time, _real_sleep = time.time, time.sleep

JOB_LABELS = {
    "phase1": "Phase 1 (mapping)",
    "phase2": "Phase 2 (snow removal)",
    "demo": "Full demo (Phase 1, swap hardware, Phase 2)",
    "bench": "Bench test",
    "autotune": "Auto-tuner (wheel radius, track width)",
    "tile": "Drive 1 tile",
    "turn": "Turn 90",
    "dump": "Dump",
    "scan": "Turret scan L/F/R",
}


# Bring-up order from docs/BRINGUP.md: (job, args, hardware, label). hardware None = either.
BRINGUP = [   # (job, args, hardware, label, constants this step tunes)
    ("bench", ["1"], None, None, ["LINE_THRESHOLD"]),
    ("bench", ["2"], None, None, ["DRIVE_POLARITY", "LEFT_DRIVE_PORT", "RIGHT_DRIVE_PORT"]),
    ("autotune", [], None, "Auto-tuner: wheel radius, left/right scale, track width", ["calibration file"]),
    ("bench", ["4"], None, "Bench 4: In-place turns (after the auto-tuner)",
     ["KP_TURN", "TURN_DPS_MAX", "TURN_DPS_MIN", "TURN_TOLERANCE_DEG"]),
    ("bench", ["5"], None, None, ["SENSOR_OFFSET_BY_PHASE", "LINE_CROSS_MAX_DPS", "KP_DRIVE_HEADING",
                                  "K_LATERAL_DEG_PER_CM"]),
    ("bench", ["6"], None, None, []),
    ("bench", ["3"], 1, None, ["TURRET_GEAR_RATIO", "US_FACE_CM", "US_SPACING_SEC"]),
    ("bench", ["7"], 1, None, ["USE_US_LATERAL_FIX"]),
    ("phase1", [], 1, "Phase 1 on a box street", ["EXPLORE_DPS", "FORWARD_STOP_CM", "WALL_THRESHOLD_CM"]),
    ("bench", ["8"], 2, None, ["ROLLER_SWEEP_POWER", "ROLLER_SWEEP_DPS", "SWEEP_DPS"]),
    ("bench", ["9"], 2, None, ["DUMP_TILT_DEG", "DUMP_PLUS", "DUMP_MINUS"]),
    ("bench", ["10"], 2, None, ["ARC_RADIUS_CM", "ARC_KP_HEADING", "ARC_K_RADIAL_DEG_PER_CM", "CORNER_POKE_CM"]),
    ("phase2", [], 2, "Phase 2 (uses the map from Phase 1)",
     ["EDGE_LANE_SHIFT_CM", "DUMP_POINT_FORWARD_CM", "EST_SEC_PER_TILE", "EST_SEC_PER_RUN", "EST_SEC_PER_TURN",
      "EST_DUMP_SEC"]),
    ("demo", [], None, "Full demo: Phase 1, swap hardware, Phase 2", []),
]
BENCH_PHASE = {3: 1, 7: 1, 8: 2, 9: 2, 10: 2}     # bench tests that need one hardware set
# Jobs whose code starts moving without asking first: the page asks this before they start.
READY = {
    "autotune": "Put the robot about 3 cm behind a black grid line, facing it, with 3 free tiles ahead. It drives "
                "2 tiles, then spins in place 6 times. Press Continue to start",
    "3": "Phase 1 hardware on, turret pointing straight ahead. The turret turns to +90, -90, then back to 0. "
         "Press Continue to start",
}


def job_phase(name, args):
    """Which hardware a job sets up (None = whichever is on)."""
    if name in ("phase1", "demo", "scan"):
        return 1
    if name in ("phase2", "dump"):
        return 2
    if name == "bench" and args and args[0].isdigit():
        return BENCH_PHASE.get(int(args[0]))
    return None


def bringup_steps():
    """[{"job", "args", "label"}] in bring-up order, for the page's list."""
    names = bench_test_names()
    out = []
    for i, (job, args, hw, label, sets) in enumerate(BRINGUP, 1):
        if label is None:
            n = int(args[0])
            label = f"Bench {n}: " + (names[n - 1] if n <= len(names) else "?").split(" (")[0]
        hw_txt = {1: " · Phase 1 hw", 2: " · Phase 2 hw"}.get(hw, "")
        out.append({"job": job, "args": args, "label": f"{i}. {label}{hw_txt}", "sets": sets})
    return out


def bench_test_names():
    """Names from bench_tests.TESTS, read from the source (importing it would set up the sensors)."""
    try:
        tree = ast.parse(open(os.path.join(HERE, "bench_tests.py")).read())
        for node in tree.body:
            if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "TESTS" for t in node.targets):
                return [elt.elts[0].value for elt in node.value.elts]
    except (OSError, SyntaxError, AttributeError, IndexError):
        pass
    return []


def phase1_defaults():
    """DEFAULT_S1..D2 from phase1_mapping.py (the sample map), as {"s1": [x, y], ...}."""
    out = {}
    try:
        tree = ast.parse(open(os.path.join(HERE, "phase1_mapping.py")).read())
        for node in tree.body:
            if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
                name = node.targets[0].id
                if name in ("DEFAULT_S1", "DEFAULT_S2", "DEFAULT_D1", "DEFAULT_D2"):
                    out[name[-2:].lower()] = list(ast.literal_eval(node.value))
    except (OSError, SyntaxError, ValueError):
        pass
    return out


# ============================================================ child side
TELEMETRY_SEC = 0.1             # job -> dashboard motor/sensor snapshots: 10 per second (the dashboard samples at 10 Hz)
LIVE_READ_SEC = 0.15            # a sensor the robot code hasn't read for this long is read by the telemetry thread


class Telemetry:
    """Motor/sensor snapshots ("hw" events) to the dashboard, TELEMETRY_SEC apart, from a background thread."""

    def __init__(self, udp_port, read):
        self.addr = ("127.0.0.1", udp_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.read = read
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while True:
            try:
                ev = self.read()
                ev["k"] = "hw"
                ev["phase"] = CURRENT["phase"]
                self.sock.sendto(json.dumps(ev, separators=(",", ":")).encode(), self.addr)
            except Exception:
                pass
            _real_sleep(TELEMETRY_SEC)


CURRENT = {"phase": None}       # hardware the job uses now (sent with every "hw" snapshot; the demo switches)
BEFORE_PROMPT = []              # called before each question (simulator: draw the board and robot while waiting)


def prompt_input(prompt=""):
    """input() replacement: the dashboard shows the question and sends the answer on stdin."""
    for fn in BEFORE_PROMPT:
        try:
            fn()
        except Exception:
            pass
    print(PROMPT_MARK + str(prompt).replace("\n", " "), flush=True)
    line = sys.stdin.readline()
    if not line:
        raise EOFError("no answer (dashboard closed)")
    ans = line.rstrip("\n")
    print(f"> {ans}", flush=True)
    return ans


def setup_sim(args, job):
    """Builds the simulated board for this job and points the robot code at it."""
    sys.path.insert(0, SIM_DIR)
    import world as world_mod
    import utils.brick as fake_brick           # sim/utils/brick.py: forwards every call to the world
    from maps import get_map, table_of
    import run_sim
    spec = get_map(args.sim)
    boxes, phase, pose = spec["boxes"], 2, None
    if job == "bench":
        import test_bench                       # the scripted bench setups (boxes, phase) per test
        phase, boxes, _ = test_bench.CASES.get(int(args.args[0]), (2, [], []))
        pose = (0.5 * T, 0.5 * T, 0.0)
    elif job == "autotune":
        boxes, phase = [], 2
    elif job in ("phase1", "demo"):
        phase = 1
    elif job == "scan":
        phase = 1
    elif job in ("tile", "turn", "dump"):       # whichever hardware the page has on (dump needs phase 2)
        phase = 2 if job == "dump" else args.phase
    w = world_mod.World(boxes, seed=args.seed, noise=1.0, sensor_offset=-5.0, table=table_of(spec))
    w.phase = phase
    # SNOWPLOW_SIM_UNPLUG=4 (or 1,4 ...): those sensor ports send no data, to test the "check the cable" path
    w.unplugged = {int(p) for p in os.environ.get("SNOWPLOW_SIM_UNPLUG", "").split(",") if p.strip().isdigit()}
    if job == "autotune":
        w.x, w.y, w.th = 0.5 * T, T - 3.0 + 5.0, 0.0      # colour sensors (5 cm behind) 3 cm short of a line
    elif job in ("phase1", "phase2", "demo"):
        w.x = w.y = w.th = 0.0                              # "place the robot at the origin facing +Y"
    elif pose is not None:
        w.x, w.y, w.th = pose
    elif args.pose:
        w.x, w.y, w.th = args.pose
    if job == "phase2":
        w.spawn_pompoms(spec["street"])
    w.realtime = args.speed if args.speed > 0 else None
    run_sim.attach_truth_telemetry(w, {"street": spec["street"] if job in ("phase1", "phase2", "demo") else [],
                                       "table": table_of(spec)})
    fake_brick.WORLD = w
    time.time = lambda: 1_000_000.0 + w.t
    time.sleep = lambda dt: w.advance(max(0.0, dt))

    reads, read_t = {}, {}
    for name in ("color_red", "us_cm"):
        def wrapped(port, _orig=getattr(w, name)):
            v = _orig(port)
            reads[port], read_t[port] = v, w.t
            return v
        setattr(w, name, wrapped)

    def snapshot():
        return {"motors": {p: [int(round(m.pos)), int(round(m.vel))] for p, m in w.motors.items()},
                "sensors": {str(k): round(v, 1) for k, v in reads.items()},
                "sensor_age": {str(k): round(w.t - t, 2) for k, t in read_t.items()},
                "pose": [round(w.x, 2), round(w.y, 2), round(w.th, 1)], "bat": None}
    tele = Telemetry(args.udp, snapshot)

    def show_board():
        """While a question waits, simulated time stands still, so nothing would be drawn: send the board and
        the robot's true pose once (the job's own telemetry takes over when it starts moving)."""
        import vizlog
        if vizlog._active:
            w.on_tick()
            return
        outline = world_mod.FOOTPRINT_P1 if w.phase == 1 else world_mod.FOOTPRINT_P2
        for ev in ({"k": "meta", "t": 0.0, "phase": w.phase, "outline": outline},
                   {"k": "sim", "t": 0.0, "boxes": [list(b) for b in sorted(w.boxes)], "table": list(table_of(spec)),
                    "street": [list(t) for t in (spec["street"] if job in ("phase1", "phase2", "demo") else [])],
                    "pompoms": [[round(p[0], 1), round(p[1], 1)] for p in w.pompoms], "outline": outline},
                   {"k": "truth", "t": 0.0, "x": round(w.x, 2), "y": round(w.y, 2), "th": round(w.th, 1)}):
            tele.sock.sendto(json.dumps(ev, separators=(",", ":")).encode(), tele.addr)
    BEFORE_PROMPT.append(show_board)

    import snowplow_core as core
    run_sim.apply_table(core, spec)
    w.sensor_offset = core.SENSOR_OFFSET_BY_PHASE.get(phase, w.sensor_offset)   # same geometry as the code
    os.makedirs(args.session, exist_ok=True)
    core.CALIBRATION_FILE = os.path.join(args.session, "calibration_params.json")
    if not os.path.exists(core.CALIBRATION_FILE):
        run_sim.write_calibration(w, core.CALIBRATION_FILE)   # a calibrated robot until the tuner is run
    core.MAP_FILE = os.path.join(args.session, "street_map.json")
    return w, spec, core


def setup_real(args):
    import snowplow_core as core                 # the robot code owns the BrickPi from here on
    from utils import brick
    reads, read_t = {}, {}
    # port -> the sensor's own read function. The robot code's reads are recorded as they happen; a port the
    # code hasn't read for LIVE_READ_SEC (e.g. the colour sensors during "Drive 1 tile", which never looks for
    # lines) is read by the telemetry thread itself, so the page always shows live values. Reading only returns
    # the BrickPi's latest value (no ping, no mode change), so it doesn't disturb the code.
    own_read = {core.LEFT_COLOR_PORT: core.left_color.get_red, core.RIGHT_COLOR_PORT: core.right_color.get_red,
                core.ULTRASONIC_PORT: core.us_sensor.get_cm}

    def recorder(port, orig):
        def read():
            v = orig()
            if v is not None:
                reads[port] = v
                read_t[port] = time.time()
            return v
        return read
    core.left_color.get_red = recorder(core.LEFT_COLOR_PORT, own_read[core.LEFT_COLOR_PORT])
    core.right_color.get_red = recorder(core.RIGHT_COLOR_PORT, own_read[core.RIGHT_COLOR_PORT])
    core.us_sensor.get_cm = recorder(core.ULTRASONIC_PORT, own_read[core.ULTRASONIC_PORT])
    motors = {p: brick.Motor(p) for p in "ABCD"}   # extra handles, read-only use

    def snapshot():
        now = time.time()
        for port, read in own_read.items():
            if now - read_t.get(port, 0.0) > LIVE_READ_SEC:
                try:
                    v = read()
                except Exception:
                    v = None
                if v is not None:
                    reads[port], read_t[port] = v, now
        m = {}
        for p, mot in motors.items():
            st = mot.get_status()
            if st and st[2] is not None:
                m[p] = [st[2], st[3], st[1], st[0]]   # encoder, dps, power, flags
        try:
            bat = round(float(brick.BP.get_voltage_battery()), 2)
        except Exception:
            bat = None
        now = time.time()
        return {"motors": m, "sensors": {str(k): round(float(v), 1) for k, v in reads.items()},
                "sensor_age": {str(k): round(now - t, 2) for k, t in read_t.items()}, "bat": bat}
    Telemetry(args.udp, snapshot)
    return core


def _telemetry_env(args):
    os.environ.update(SNOWPLOW_WEBVIZ="0", SNOWPLOW_VIZ=f"127.0.0.1:{args.udp}")
    if args.sim:
        os.environ["SNOWPLOW_VIZ_LOGDIR"] = "none"
    else:
        os.environ.pop("SNOWPLOW_VIZ_LOGDIR", None)


def _start_viz(core, vizlog, phase):
    """Map telemetry for jobs whose code doesn't start it (auto-tuner, most bench tests). No log file:
    only phase runs are worth replaying. Odometry restarts at (0, 0) wherever the test begins."""
    vizlog.start(phase, log_dir="none", outline=core.ROBOT_OUTLINE[phase], us_fwd=core.US_FORWARD_OFFSET_CM,
                 us_face=core.US_FACE_CM)


def run_job(args):
    _telemetry_env(args)
    builtins.input = prompt_input
    job, a = args.job, args.args
    CURRENT["phase"] = job_phase(job, a) or args.phase
    w = spec = None
    if args.sim:
        w, spec, core = setup_sim(args, job)
        _telemetry_env(args)                     # sim/test_bench.py turns telemetry off when imported
    else:
        core = setup_real(args)
    import vizlog
    t0 = _real_time()
    print(f"[JOB] {JOB_LABELS.get(job, job)} {' '.join(a)}{' (simulator, ' + args.sim + ')' if args.sim else ''}",
          flush=True)
    ok = False
    try:
        if job in ("phase1", "phase2", "demo"):
            import phase1_mapping as p1
            import phase2_snow_removal as p2
            p1.MAP_FILE = p2.MAP_FILE = core.MAP_FILE
            if job in ("phase1", "demo"):
                if args.sim:
                    pts = [tuple(spec[k]) for k in ("s1", "s2", "d1", "d2")]
                else:
                    nums = [int(v) for v in a]
                    pts = [tuple(nums[i:i + 2]) for i in range(0, 8, 2)]
                p1.run_phase1(*pts)
            if job == "demo":
                core.stop_all_motors()               # phase 2 itself asks for the hardware swap
                CURRENT["phase"] = 2
                if args.sim:
                    w.phase = 2
                    w.sensor_offset = core.SENSOR_OFFSET_BY_PHASE.get(2, w.sensor_offset)
                    w.x = w.y = w.th = 0.0
                    w.spawn_pompoms(spec["street"])
            if job in ("phase2", "demo"):
                if not os.path.exists(core.MAP_FILE):
                    print("[JOB] No street map yet: run Phase 1 first.")
                    return EXIT_NEEDS_MAP
                p2.run_phase2()
        elif job == "bench":
            _start_viz(core, vizlog, job_phase(job, a) or args.phase)
            import bench_tests
            name, fn = bench_tests.TESTS[int(a[0]) - 1]
            name = name.replace("run auto_tuner.py first", "run the Auto-tuner first: Bring-up step 3")
            print(f"--- {name} ---", flush=True)
            if a[0] in READY:                    # tests that move before their first question
                input(READY[a[0]])
            if args.sim:
                print("[SIM] the simulated robot can't be picked up or moved by hand: steps that ask you to "
                      "hold a sensor over something read wherever it stands.", flush=True)
            fn()
        elif job == "autotune":
            _start_viz(core, vizlog, args.phase)
            input(READY["autotune"])
            import auto_tuner
            auto_tuner.CALIBRATION_FILE = core.CALIBRATION_FILE
            auto_tuner.main()
        elif job in ("tile", "turn", "dump", "scan"):
            phase = 1 if job == "scan" else args.phase
            core.init_hardware(phase=phase, strict=job == "scan")   # tile/turn/dump: a missing sensor only warns
            if w is not None:                       # simulator: start odometry where the robot really is
                core.x, core.y, core.theta = w.x, w.y, w.th
            vizlog.start(phase, outline=core.ROBOT_OUTLINE[phase], us_fwd=core.US_FORWARD_OFFSET_CM,
                         us_face=core.US_FACE_CM)
            if job == "tile":
                n = float(a[0]) if a else 1.0
                core.drive_straight_distance(abs(n) * T, forward=n > 0, hold_heading=core.theta)
            elif job == "turn":
                core.turn_to((core.theta + (90.0 if a and a[0] == "R" else -90.0)) % 360.0)
            elif job == "dump":
                # "+" / "-" from the page's buttons; nothing given: the way the bin is set to pour
                core.dump_snow_payload(a[0] if a and a[0] in ("+", "-") else (core.dump_directions() or ["+"])[0])
            elif job == "scan":
                for ang, label in ((-90.0, "left"), (0.0, "ahead"), (90.0, "right")):
                    print(f"   {label:6s} {core.check_direction_distance(ang):6.1f} cm", flush=True)
            core.update_odometer()
            print(f"   pose x {core.x:.1f} y {core.y:.1f} heading {core.theta:.1f}", flush=True)
        else:
            print(f"[JOB] unknown job {job!r}")
            return 2
        ok = True
    except KeyboardInterrupt:
        print("\n[JOB] stopped", flush=True)
    except EOFError as e:
        print(f"\n[JOB] stopped: {e}", flush=True)
    except core.SensorMissing as e:
        print(f"\n[JOB] stopped: {e}", flush=True)
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)   # a second STOP must not interrupt the clean-up
        try:
            core.stop_all_motors()
        except Exception:
            pass
        vizlog.close()
        if not args.sim:
            try:
                from utils.brick import reset_brick
                reset_brick()
            except Exception:
                pass
        print(f"[JOB] {'finished' if ok else 'ended'} after {_real_time() - t0:.1f} s", flush=True)
        _real_sleep(0.3)                     # let the last telemetry go out
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description="Run one robot job (used by dashboard.py).")
    ap.add_argument("--sim", metavar="MAP", help="run against the simulator on MAP")
    ap.add_argument("--session", default=os.path.join("/tmp", "snowplow_dash"),
                    help="simulator: folder for its calibration and street map files")
    ap.add_argument("--speed", type=float, default=1.0, help="simulator pacing (1 = real time, 0 = flat out)")
    ap.add_argument("--seed", type=int, default=0, help="simulator seed (same seed = same simulated robot)")
    ap.add_argument("--pose", type=float, nargs=3, metavar=("X", "Y", "TH"), help="simulator start pose")
    ap.add_argument("--phase", type=int, choices=(1, 2), default=2, help="hardware for small actions")
    ap.add_argument("--udp", type=int, default=5005, help="dashboard telemetry port")
    ap.add_argument("job", choices=sorted(JOB_LABELS))
    ap.add_argument("args", nargs="*")
    args = ap.parse_args()
    # STOP sends Ctrl-C. A program started in the background (ssh 'cmd &', rc.local) inherits "ignore
    # Ctrl-C", which would make STOP fall back to a hard kill without stopping the motors: undo that.
    signal.signal(signal.SIGINT, signal.default_int_handler)

    def on_term(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_term)
    sys.exit(run_job(args))


if __name__ == "__main__":
    main()
