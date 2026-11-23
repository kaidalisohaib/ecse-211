#!/usr/bin/env python3
"""
Runs every bench_tests.py test against the simulator with scripted answers, to catch crashes
before they happen on the robot.   python3 sim/test_bench.py [-v] [test numbers...]
"""
from __future__ import annotations
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
sys.path.insert(0, SIM_DIR)
sys.path.insert(1, PROJECT_DIR)
os.environ.update(SNOWPLOW_WEBVIZ="0", SNOWPLOW_VIZ="none", SNOWPLOW_VIZ_LOGDIR="none")

import world as world_mod            # noqa: E402
import utils.brick as fake_brick     # noqa: E402
from run_sim import write_calibration  # noqa: E402

T = 30.48
_real_time, _real_sleep = time.time, time.sleep
CORNER = [(0, 2), (-1, 1), (-1, 2), (1, 0), (-1, 0), (1, 2)]   # right turn at tile (0, 1)

#   test: (phase, boxes, answers)
CASES = {
    1: (2, [], ["", "", ""]),
    2: (2, [], ["", "y", "", "y"]),
    3: (1, [(0, 1)], ["y", "10", "q", ""]),
    4: (2, [], [""]),
    5: (2, [], [""]),
    6: (2, [], [""]),
    7: (1, [(-1, 0), (1, 0)], ["0", "q"]),
    8: (2, [], ["", "y", "y"]),
    9: (2, [], ["190", "b", "-150", "l", "q", "+"]),
    10: (2, CORNER, ["r", "", "y"]),
}


def run(n, verbose=False):
    phase, boxes, answers = CASES[n]
    w = world_mod.World(boxes, seed=n, noise=1.0, sensor_offset=-5.0)
    w.phase = phase
    w.x, w.y, w.th = 0.5 * T, 0.5 * T, 0.0
    fake_brick.WORLD = w
    it = iter(answers)
    builtins.input = lambda prompt="": (print(prompt), next(it, "q"))[1]
    time.time = lambda: 1_000_000.0 + w.t
    time.sleep = lambda dt: w.advance(max(0.0, dt))
    for m in ("snowplow_core", "bench_tests", "vizlog"):
        sys.modules.pop(m, None)
    out = io.StringIO()
    err = None
    try:
        core = importlib.import_module("snowplow_core")
        core.CALIBRATION_FILE = os.path.join(tempfile.gettempdir(), f"bench_calib_{os.getpid()}.json")
        write_calibration(w, core.CALIBRATION_FILE)
        bench = importlib.import_module("bench_tests")
        sys.argv = ["bench_tests.py", str(n)]
        with contextlib.redirect_stdout(sys.stdout if verbose else out):
            bench.main()
    except BaseException as e:   # noqa: BLE001
        err = f"{type(e).__name__}: {e}"
    finally:
        time.time, time.sleep = _real_time, _real_sleep
    text = out.getvalue()
    if "Traceback" in text or "Error" in text:
        err = err or text[-600:]
    contacts = w.contacts.get(phase, 0) if isinstance(w.contacts, dict) else w.contacts[phase]
    return err, w, contacts, text


def main():
    verbose = "-v" in sys.argv
    picks = [int(a) for a in sys.argv[1:] if a.isdigit()] or sorted(CASES)
    for n in picks:
        err, w, contacts, text = run(n, verbose)
        status = "OK " if not err else "ERR"
        print(f"[{status}] test {n:2d}: {w.t:5.1f} s simulated, true pose x {w.x:5.1f} y {w.y:5.1f} "
              f"th {w.th % 360:5.1f}, contacts {contacts}, off-table {len(w.offtable)}" + (f"\n      {err}" if err else ""))
        for line in text.splitlines():
            if "->" in line or "stopped:" in line or "back home" in line or "ended:" in line or "error" in line:
                print("      " + line.strip())


if __name__ == "__main__":
    main()
