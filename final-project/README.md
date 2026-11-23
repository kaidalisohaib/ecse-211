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

## Where to change things

Every setting is a `NAME = value` line near the top of one of three files. You can edit the file, or use the
dashboard's **Constants** panel, which writes the same line and keeps a backup in `logs/constants_backup/`.

| What | File, section |
|---|---|
| Ports (motors A-D, sensors S1-S4), drive direction (`DRIVE_POLARITY`) | `snowplow_core.py`, "HARDWARE PORT CONFIGURATION" (top) |
| Wheel radius, track width, colour sensor position (`SENSOR_OFFSET_BY_PHASE`, `SENSOR_SPACING`), line threshold | `snowplow_core.py`, "GEOMETRY & CALIBRATION" |
| Speeds and steering gains | `snowplow_core.py`, "MOTION PROFILE & CONTROL GAINS" |
| Turret and ultrasonic (`US_FORWARD_OFFSET_CM`, `US_FACE_CM`, `TURRET_GEAR_RATIO`), robot outline | `snowplow_core.py`, "ULTRASONIC & TURRET" |
| Belt power, dump angle / direction / shakes | `snowplow_core.py`, "PHASE 2 INTAKE & DUMP SETTINGS" |
| Time estimates, table size | `snowplow_core.py`, "TIME BUDGET" |
| Phase 1 strategy (explore mode, street rules) | `phase1_mapping.py`, top |
| Phase 2: back-pour distances (`DUMP_BACK_LIP_CM`, `DUMP_BACK_SUPPORT_CM`), edge lane, corner arcs | `phase2_snow_removal.py`, top |

After the auto-tuner runs, `calibration_params.json` (on the robot) overrides the wheel radius, wheel scales and
track width; the Constants panel shows and edits those too. Constants edited on the robot are local changes to
the robot's git copy: commit them there (or say so) before pulling new code.

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
