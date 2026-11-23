# ECSE 211 Group 13 snowplow

**Run one program:**

    python3 dashboard.py          # on the robot: open the printed address, type the printed code
    python3 dashboard.py --sim    # on a laptop: simulated robot, http://localhost:8080

The page shows every motor and sensor live. From it you can:
- drive by hand;
- calibrate the colour sensors;
- run Phase 1, Phase 2, the full demo and every bring-up test;
- answer the robot's questions;
- press STOP.

The bring-up order and every constant to tune are in [docs/BRINGUP.md](docs/BRINGUP.md). Progress and decisions
are in [docs/STATUS.md](docs/STATUS.md).

| File | What |
|---|---|
| `dashboard.py` | the web dashboard, the one entry point |
| `jobs.py` | runs one job (phase, bench test, tuner, small action) as its own process for the dashboard |
| `snowplow_core.py` | hardware, odometry, driving, line snaps, turret/ultrasonic, dump |
| `phase1_mapping.py`, `phase2_snow_removal.py` | the two phases |
| `bench_tests.py`, `auto_tuner.py`, `demo_runner.py` | bring-up tests, calibration, menu (also runnable alone over SSH) |
| `vizlog.py`, `visualizer.py` | telemetry from the robot code, and the map drawing (plus `--replay` of `logs/`) |
| `sim/` | simulator: `run_sim.py` (batch runs on several maps), `test_bench.py` (every bench test), `stress.py` (Phase 1 + 2 on hundreds of random boards: `python3 sim/stress.py --n 100 --kinds all`), `maps.py` (fixed maps + `random[-WxH][-edge][-loose]-SEED`) |
| `utils/` | the course's BrickPi wrapper |
