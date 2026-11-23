#!/usr/bin/env python3
"""
Runs the real phase1_mapping.py / phase2_snow_removal.py against a simulated board.

  python3 sim/run_sim.py                    # sample map, seed 0
  python3 sim/run_sim.py --map long --seeds 5
  python3 sim/run_sim.py --map sample -v    # show the robot's own console output
  python3 visualizer.py --sim sample        # watch it (the visualizer starts this script)

Simulated time runs faster than real time unless --realtime is given. Nothing talks to real hardware.
Telemetry: --viz-udp HOST:PORT streams to a visualizer; --viz-log DIR keeps the .jsonl logs.
"""
import argparse
import builtins
import contextlib
import importlib
import io
import os
import sys
import tempfile
import time

SIM_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SIM_DIR)
sys.path.insert(0, SIM_DIR)          # fake utils.brick wins over the real one
sys.path.insert(1, PROJECT_DIR)

from maps import MAPS, get_map, is_map_name, table_of  # noqa: E402
import world as world_mod            # noqa: E402
import utils.brick as fake_brick     # noqa: E402
import vizlog                        # noqa: E402

_real_time, _real_sleep = time.time, time.sleep


def attach_truth_telemetry(w, spec):
    """Sends the simulator's ground truth (true pose, true houses, pom-poms) to the visualizer."""
    st = {"t0": None, "last": -1.0, "picked": 0, "hw": -1.0}
    reads, read_t = {}, {}           # last value the robot code read from each sensor port, and when

    for name in ("color_red", "us_cm"):
        def wrapped(port, _orig=getattr(w, name)):
            v = _orig(port)
            reads[port], read_t[port] = v, w.t
            return v
        setattr(w, name, wrapped)

    def tick():
        if not vizlog._active:
            return
        if st["t0"] != vizlog._t0:   # a new phase log started
            st["t0"], st["last"], st["picked"], st["hw"] = vizlog._t0, -1.0, 0, -1.0
            vizlog.emit("sim", boxes=[list(b) for b in sorted(w.boxes)], street=[list(t) for t in spec["street"]],
                        table=list(table_of(spec)),
                        pompoms=[[round(p[0], 1), round(p[1], 1)] for p in w.pompoms],
                        outline=world_mod.FOOTPRINT_P1 if w.phase == 1 else world_mod.FOOTPRINT_P2)
        if w.t - st["last"] >= 1.0 / vizlog.POSE_HZ:
            st["last"] = w.t
            vizlog.emit("truth", x=round(w.x, 2), y=round(w.y, 2), th=round(w.th, 1))
        if w.t - st["hw"] >= 0.2:    # motor/sensor snapshot for dashboard.py's cards
            st["hw"] = w.t
            vizlog.emit("hw", motors={p: [int(round(m.pos)), int(round(m.vel))] for p, m in w.motors.items()},
                        sensors={str(k): round(v, 1) for k, v in reads.items()},
                        sensor_age={str(k): round(w.t - t, 2) for k, t in read_t.items()})
        if len(w.picked) > st["picked"]:
            vizlog.emit("pick", i=w.picked[st["picked"]:])
            st["picked"] = len(w.picked)

    w.on_tick = tick


def apply_table(core, spec):
    """Point the robot code at this board's table size (its TILE_X_MIN.. constants)."""
    core.TILE_X_MIN, core.TILE_X_MAX, core.TILE_Y_MIN, core.TILE_Y_MAX = table_of(spec)


def write_calibration(w, path):
    """What auto_tuner.py would save for this robot: true values with the errors seen in
    sim/test_auto_tuner.py (radius ~0.1 %, track width ~0.5 %)."""
    rng = __import__("random").Random(w.rng.random())
    radius = w.r_left * (1 + rng.uniform(-0.001, 0.001))
    track = w.track * (1 + rng.uniform(-0.005, 0.005))
    import json
    with open(path, "w") as f:
        json.dump({"WHEEL_RADIUS": radius, "LEFT_WHEEL_SCALE": 1.0, "RIGHT_WHEEL_SCALE": w.r_right / w.r_left,
                   "TRACK_WIDTH": track, "TRACK_WIDTH_CW": track, "TRACK_WIDTH_CCW": track}, f)


def run_once(map_name, seed, noise, verbose, reposition=True, dark=False, p2_side_us=None, realtime=None,
             calibrated=True, intake="funnel", arcs=True, overrides=()):
    spec = get_map(map_name)
    world_mod.use_intake(intake)
    w = world_mod.World(spec["boxes"], seed=seed, noise=noise, dark_pompoms=dark, phase2_side_us=p2_side_us,
                        table=table_of(spec))
    w.realtime = realtime
    attach_truth_telemetry(w, spec)
    fake_brick.WORLD = w
    time.time = lambda: 1_000_000.0 + w.t
    time.sleep = lambda dt: w.advance(max(0.0, dt))
    builtins.input = lambda prompt="": ""

    for mod in ("snowplow_core", "phase1_mapping", "phase2_snow_removal"):
        sys.modules.pop(mod, None)
    core = importlib.import_module("snowplow_core")
    p1 = importlib.import_module("phase1_mapping")
    p2 = importlib.import_module("phase2_snow_removal")
    map_file = os.path.join(tempfile.gettempdir(), f"snowplow_sim_map_{os.getpid()}.json")
    core.MAP_FILE = p1.MAP_FILE = p2.MAP_FILE = map_file
    p2.PHASE2_SIDE_US = p2_side_us
    p2.ARC_CORNERS = arcs
    apply_table(core, spec)
    mods = {"core": core, "p1": p1, "p2": p2}
    for item in overrides:                     # --set core.KP_DRIVE_HEADING=3.0
        name, val = item.split("=", 1)
        mod, attr = name.split(".", 1)
        if not hasattr(mods[mod], attr):
            raise SystemExit(f"--set: {mod} has no {attr}")
        import ast
        setattr(mods[mod], attr, ast.literal_eval(val))
    core.ROBOT_OUTLINE[2] = [tuple(p) for p in world_mod.FOOTPRINT_P2]
    if os.environ.get("SNOWPLOW_EXPLORE_MODE"):
        p1.EXPLORE_MODE = os.environ["SNOWPLOW_EXPLORE_MODE"]
    if calibrated:
        core.CALIBRATION_FILE = os.path.join(tempfile.gettempdir(), f"snowplow_sim_calib_{os.getpid()}.json")
        write_calibration(w, core.CALIBRATION_FILE)
    else:
        core.CALIBRATION_FILE = os.path.join(tempfile.gettempdir(), "no_such_calibration.json")

    out = io.StringIO()
    res = {"map": map_name, "seed": seed}
    class Stamped(io.TextIOBase):
        def __init__(self, inner):
            self.inner, self.bol = inner, True
        def write(self, txt):
            for part in txt.splitlines(keepends=True):
                if self.bol and part.strip():
                    self.inner.write(f"[{w.t:6.1f}] ")
                self.inner.write(part)
                self.bol = part.endswith("\n")
            return len(txt)
    ctx = contextlib.redirect_stdout(Stamped(sys.stdout)) if verbose else contextlib.redirect_stdout(out)
    try:
        with ctx:
            w.phase = 1
            w.sensor_offset = core.SENSOR_OFFSET_BY_PHASE[1]
            t1 = p1.run_phase1(spec["s1"], spec["s2"], spec["d1"], spec["d2"])
            res["p1_time"] = t1
            res["p1_end_err"] = (w.x ** 2 + w.y ** 2) ** 0.5
            import json
            with open(map_file) as f:
                m = json.load(f)
            res["mapped"] = [tuple(t) for t in m["street_tiles"]]
            res["map_ok"] = res["mapped"] == spec["street"]
            w.phase = 2
            w.sensor_offset = core.SENSOR_OFFSET_BY_PHASE[2]
            if reposition:
                w.x = w.y = w.th = 0.0
            w.spawn_pompoms(spec["street"])
            t2 = p2.run_phase2()
            res["p2_time"] = t2
            res["p2_end_err"] = (w.x ** 2 + w.y ** 2) ** 0.5
    except BaseException as e:  # report, don't crash the batch
        import traceback
        res["error"] = f"{type(e).__name__}: {e}"
        res["trace"] = traceback.format_exc()
    finally:
        vizlog.close()
        time.time, time.sleep = _real_time, _real_sleep
    total = len(w.pompoms)
    got = sum(1 for p in w.pompoms if p[2])
    res.update({
        "pompoms": f"{got}/{total}",
        "p1_min_clear": w.min_clearance[1], "p2_min_clear": w.min_clearance[2],
        "p2_contacts": w.contacts[2], "offtable": len(w.offtable),
        "contact_log": w.contact_log, "log": out.getvalue(),
    })
    return res


def fmt(r):
    if "error" in r:
        return f"{r['map']:>10} seed {r['seed']:>2}: ERROR {r['error']}"
    return (f"{r['map']:>10} seed {r['seed']:>2}: P1 {r.get('p1_time', 0):5.1f}s "
            f"map {'OK ' if r.get('map_ok') else 'BAD'} | P2 {r.get('p2_time', 0):5.1f}s "
            f"pompoms {r['pompoms']:>7} | P1 clr {r.get('p1_min_clear', 0):4.1f} | P2 min clearance {r['p2_min_clear']:5.2f} cm, contacts {r['p2_contacts']} "
            f"| off-table {r['offtable']} | home err P1 {r.get('p1_end_err', 0):.1f} P2 {r.get('p2_end_err', 0):.1f} cm")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default="sample",
                    help=f"{', '.join(sorted(MAPS))}, all, or a random board: random-SEED, random-WxH-SEED "
                         f"(e.g. random-18x5-7 = two tables side by side), random-edge-SEED (street may run "
                         f"along the table edge)")
    ap.add_argument("--seeds", type=int, default=1)
    ap.add_argument("--noise", type=float, default=1.0)
    ap.add_argument("--dark-pompoms", action="store_true", help="pom-poms read as black lines")
    ap.add_argument("--p2-side-us", choices=["left", "right"], help="fixed side ultrasonic in Phase 2")
    ap.add_argument("--seed-start", type=int, default=0)
    ap.add_argument("--uncalibrated", action="store_true",
                    help="robot code uses the nominal 2.1 cm / 12 cm instead of auto_tuner results (stress test)")
    ap.add_argument("--explore", choices=["forward", "side"], help="Phase 1 EXPLORE_MODE override")
    ap.add_argument("--viz-udp", help="HOST:PORT of a visualizer (python3 visualizer.py --live)")
    ap.add_argument("--viz-log", default="none", help="directory for .jsonl telemetry logs (default: none)")
    ap.add_argument("--realtime", type=float, help="pace the sim: 1.0 = real time, 2.0 = twice as fast")
    ap.add_argument("--intake", choices=["funnel", "square", "chamfer"], default="funnel",
                    help="Phase 2 footprint: the pinch belt + V-funnel in a 15 cm circle (default), the old "
                         "square 25 cm roller intake, or the older 5 cm chamfered shape")
    ap.add_argument("--no-arcs", action="store_true", help="Phase 2 turns in place at street corners")
    ap.add_argument("--set", action="append", default=[], metavar="MOD.NAME=VALUE",
                    help="override a constant, e.g. --set core.USE_US_LATERAL_FIX=False (MOD = core, p1, p2)")
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args()
    os.environ["SNOWPLOW_VIZ_LOGDIR"] = a.viz_log
    os.environ["SNOWPLOW_WEBVIZ"] = "0"           # the robot code must not start a viewer for sim runs
    os.environ["SNOWPLOW_VIZ"] = a.viz_udp or "none"
    if a.explore:
        os.environ["SNOWPLOW_EXPLORE_MODE"] = a.explore
    if a.map != "all" and not is_map_name(a.map):
        raise SystemExit(f"unknown map {a.map!r}")
    maps = sorted(MAPS) if a.map == "all" else [a.map]
    for mp in maps:
        for s in range(a.seed_start, a.seed_start + a.seeds):
            side = {"left": -1, "right": 1}.get(a.p2_side_us)
            r = run_once(mp, s, a.noise, a.verbose, dark=a.dark_pompoms, p2_side_us=side, realtime=a.realtime,
                         calibrated=not a.uncalibrated, intake=a.intake, arcs=not a.no_arcs,
                         overrides=a.set)
            print(fmt(r))
            if "error" in r:
                print(r["log"][-3000:])
                print(r["trace"])
            elif r["map_ok"] is False:
                print("   mapped:", r["mapped"])
            if r["contact_log"] and a.verbose:
                print("   contacts:", r["contact_log"][:5])


if __name__ == "__main__":
    main()
