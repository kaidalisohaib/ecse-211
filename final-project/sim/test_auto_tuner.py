#!/usr/bin/env python3
"""
Runs the real auto_tuner.py against the simulated robot and compares what it measures with the
simulator's true wheel radius and track width.

  python3 sim/test_auto_tuner.py                  # sensors 5 cm behind the axle, 5 seeds
  python3 sim/test_auto_tuner.py --offset 7 --seeds 3 --noise 2
"""
from __future__ import annotations
import argparse
import builtins
import contextlib
import importlib
import io
import json
import os
import sys
import tempfile
import time

SIM_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SIM_DIR)
sys.path.insert(0, SIM_DIR)
sys.path.insert(1, PROJECT_DIR)

import world as world_mod            # noqa: E402
import utils.brick as fake_brick     # noqa: E402

T = 30.48
_real_time, _real_sleep = time.time, time.sleep


def run(seed, noise, offset, verbose=False):
    w = world_mod.World([], seed=seed, noise=noise, sensor_offset=offset)
    # Facing +Y on tile column 0, colour sensors 3 cm short of the line at y = 1 tile.
    w.x, w.y, w.th = 0.5 * T, T - 3.0 - offset, w.rng.uniform(-4, 4)
    fake_brick.WORLD = w
    time.time = lambda: 1_000_000.0 + w.t
    time.sleep = lambda dt: w.advance(max(0.0, dt))
    builtins.input = lambda prompt="": ""
    sys.modules.pop("auto_tuner", None)
    sys.modules.pop("snowplow_core", None)
    try:
        at = importlib.import_module("auto_tuner")
        at.CALIBRATION_FILE = os.path.join(tempfile.gettempdir(), f"tuner_test_{os.getpid()}.json")
        out = io.StringIO()
        with contextlib.redirect_stdout(sys.stdout if verbose else out):
            at.main()
        with open(at.CALIBRATION_FILE) as f:
            params = json.load(f)
    finally:
        time.time, time.sleep = _real_time, _real_sleep
    true_r = 0.5 * (w.r_left + w.r_right)
    return {
        "radius": params["WHEEL_RADIUS"], "true_radius": true_r,
        "track": params["TRACK_WIDTH"], "true_track": w.track,
        "sim_seconds": w.t, "offtable": len(w.offtable),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--noise", type=float, default=2.0)
    ap.add_argument("--offset", type=float, default=-5.0, help="colour sensors ahead of the axle (cm), - = behind")
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args()
    for s in range(a.seeds):
        r = run(s, a.noise, a.offset, a.verbose)
        er = 100 * (r["radius"] / r["true_radius"] - 1)
        et = 100 * (r["track"] / r["true_track"] - 1)
        print(f"seed {s}: radius {r['radius']:.4f} (true {r['true_radius']:.4f}, {er:+.2f}%)  "
              f"track {r['track']:.3f} (true {r['true_track']:.3f}, {et:+.2f}%)  "
              f"{r['sim_seconds']:.0f} s simulated")


if __name__ == "__main__":
    main()
