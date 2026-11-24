# Robot bring-up: what to run, in order, and what to tune

## Start here: one program

| Where | Run | Then |
|---|---|---|
| on the robot (SSH into the Pi, `cd final-project`) | `python3 dashboard.py` | open the printed address on the laptop and type the printed 4-digit code |
| on the laptop, no robot | `python3 dashboard.py --sim` | open http://localhost:8080 (no code) |

Everything happens on that page:
- live motors, sensors, battery, map and console;
- hand driving (hold an arrow or WASD) and buttons on each motor;
- colour calibration;
- Phase 1, Phase 2 and the full demo, plus every step below from the **Bring-up** list in the Run card.

The robot's questions ("press Enter to start…") show under the top bar; answer them there. **Constants** (top bar) shows
every constant in the three robot files with its comment, and saves edits straight into the files (backup in
`logs/constants_backup/`, history in `logs/constants_changes.log`). "Constants for this step" in the Run card
opens just the ones the selected bring-up step tunes. The tables below remain the reference. **STOP** (or Esc) stops
whatever runs and brakes every motor. In `--sim`, the top bar also picks the map and the speed. The simulator keeps
its own street map per map and one calibration for every map (the same simulated robot) in `sim/session/`
("Reset sim files" forgets the map's street map and the calibration); it never touches the robot's files. A saved
calibration that no longer fits the simulated robot (the code's `WHEEL_RADIUS` / `TRACK_WIDTH` changed since) is
replaced, with a `[SIM]` line in the console.

Every phase run is saved in `logs/`; replay one with `python3 visualizer.py --web --replay logs/` (port 8080, so
stop the dashboard first or add `--web 8081`).

Before touching the robot, on the laptop: `python3 sim/test_bench.py` (all bench tests in the simulator) and
`python3 sim/run_sim.py --map all --seeds 3 --noise 2` (both phases on 3 maps). Both should be clean.

## Order (the dashboard's Bring-up list)

| # | Bring-up list entry | Hardware | Sets |
|---|---|---|---|
| 0 | Measure the build with a ruler | both | geometry constants (table below) |
| 1 | Bench 1: colour sensors (or the colour cards' table/line buttons) | any | `LINE_THRESHOLD` |
| 2 | Bench 2: drive direction | any | `DRIVE_POLARITY`, drive ports |
| 3 | Auto-tuner | any | `calibration_params.json` (wheel radius, L/R scale, track) |
| 4 | Bench 4: turns | any | `KP_TURN`, `TURN_DPS_MAX/MIN`, `TURN_TOLERANCE_DEG` |
| 5 | Bench 5: straight 3-tile run | any | `SENSOR_OFFSET_BY_PHASE`, `LINE_CROSS_MAX_DPS`, `KP_DRIVE_HEADING`, `K_LATERAL_DEG_PER_CM` |
| 6 | Bench 6: square loop | any | overall check: home error < 2 cm |
| 7 | Bench 3: turret + ultrasonic | Phase 1 | `TURRET_GEAR_RATIO`, `US_FACE_CM`, `US_SPACING_SEC` (wave a hand: measures how often the sensor really pings) |
| 8 | Bench 7: side ultrasonic | Phase 1 | `USE_US_LATERAL_FIX` (off unless error < 1 cm) |
| 9 | Phase 1 on a box street (type the gate and drop-off first) | Phase 1 | `EXPLORE_DPS`, `FORWARD_STOP_CM`, `WALL_THRESHOLD_CM` |
| 10 | Bench 8: belt + catch test | Phase 2 | `ROLLER_SWEEP_POWER` (sign, speed; lab: -50), `SWEEP_DPS` |
| 11 | Bench 9: dump arm | Phase 2 | `DUMP_TILT_DEG`, `DUMP_PLUS`, `DUMP_MINUS` (+ `DUMP_BACK_LIP_CM`, `DUMP_BACK_SUPPORT_CM` if it pours back) |
| 12 | Bench 10: street corner on an arc | Phase 2 | `ARC_RADIUS_CM`, `ARC_KP_HEADING`, `ARC_K_RADIAL_DEG_PER_CM`, `CORNER_POKE_CM` |
| 13 | Phase 2 (uses the map from 9) | Phase 2 | `EDGE_LANE_SHIFT_CM`, `DUMP_POINT_FORWARD_CM` |
| 14 | Compare planned vs real times in the logs | both | `EST_SEC_PER_TILE`, `EST_SEC_PER_RUN`, `EST_SEC_PER_TURN`, `EST_DUMP_SEC` |
| 15 | Full demo | both | — |

Redo 3 (the auto-tuner) whenever wheels, tyres or the battery change a lot, and after the Phase 2 hardware swap
if it changes the weight on the wheels.

## Constants

`core` = `snowplow_core.py`, `p1` = `phase1_mapping.py`, `p2` = `phase2_snow_removal.py`.

### Measure with a ruler (step 0)
| Constant | File | What |
|---|---|---|
| `SENSOR_OFFSET_BY_PHASE` | core | colour sensors ahead of the wheel axle, cm (behind = negative; measured -4 both phases) |
| `SENSOR_SPACING` | core | left-right distance between the colour sensors, centre to centre (now 12: MEASURE, a bit under the 13.75 track) |
| `US_FORWARD_OFFSET_CM` | core | turret axis ahead of the axle (now 8) |
| `US_FACE_CM` | core | turret axis to the sensor face (now 2.5); fine-tune in step 7 |
| `ORIGIN_ARCS` | core | leave / reach the origin on arcs instead of the diagonal step (saves ~5 s per phase); False if the arcs miss after bench 10 |
| `US_SPACING_SEC` | core | time between ultrasonic samples (now 0.015); set to the ping period step 7 prints, so a median of 5 is 5 real pings |
| `ROBOT_OUTLINE` | core | footprint corners per phase (lateral, forward) for the viewer; copy to `sim/world.py` footprints too |
| `DUMP_POINT_FORWARD_CM` | p2 | middle of the bin's pour opening ahead of the axle, − = behind (now −10, from the photos); centres the pour on the bucket |
| `EDGE_LANE_SHIFT_CM` | p2 | how far toward the table edge to drive on the drop-off row (keep the wheels on the table) |

### Sensors and ports
| Constant | File | Default | How |
|---|---|---|---|
| port constants | core | C left / D right drive, A turret/dump, B belt, colour 1 (L) / 2 (R), US 3 (wired 2026-11-22) | must match the cables |
| `DRIVE_POLARITY` | core | -1 | bench 2 |
| `LINE_THRESHOLD` | core | 30 | bench 1: halfway between table and line |
| `TURRET_GEAR_RATIO` | core | 1.0 | bench 3 (sign = direction) |
| `WALL_THRESHOLD_CM` | core | 25 | a house beside the robot reads ~13; open reads 45+ |
| `FORWARD_STOP_CM` | core | 45 | Phase 1 forward watch: stop when a house is this close ahead |
| `USE_US_LATERAL_FIX` | core | False | bench 7 |

### Driving
| Constant | File | Default | Tune when |
|---|---|---|---|
| `CRUISE_DPS` / `EXPLORE_DPS` / `SWEEP_DPS` | core | 360 / 320 / 170 | faster if snaps still hit and the map stays right; `SWEEP_DPS` from the catch test |
| `LINE_CROSS_MAX_DPS` | core | 280 | lower if lines are missed (bench 5 shows no snaps) |
| `KP_DRIVE_HEADING`, `MAX_HEADING_CORR_DPS` | core | 2.5, 45 | wobble: lower; slow to straighten: raise |
| `K_LATERAL_DEG_PER_CM`, `MAX_LATERAL_STEER_DEG` | core | 1.5, 6 | slow S across the lane: lower; drifts off-centre: raise |
| `KP_TURN`, `TURN_DPS_MAX`, `TURN_DPS_MIN`, `TURN_TOLERANCE_DEG` | core | 5, 240, 40, 0.8 | bench 4 |
| `ACCEL_RAMP_SEC`, `DECEL_DIST_CM` | core | 0.3, 7 | wheel slip at start/stop: longer |
| `LINE_WINDOW_CM`, `MAX_SNAP_GAP_CM`, `MAX_SNAP_HEADING_DEG` | core | 5.5, 4, 18 | rarely; only if snaps are rejected/false |

### Phase 2 mechanism
| Constant | File | Default | How |
|---|---|---|---|
| `ROLLER_SWEEP_POWER` | core | -50 (lab) | bench 8 (sign = direction); `None` = run at `ROLLER_SWEEP_DPS` |
| `DUMP_TILT_DEG`, `DUMP_SHAKES`, `DUMP_SHAKE_DEG`, `DUMP_HOLD_SEC` | core | 140, 2, 50, 1.2 (lab: -140 then +50/-50 shakes) | bench 9 |
| `DUMP_PLUS`, `DUMP_MINUS` | core | `None`, `"LEFT"` | bench 9: where motor + / − pours (`"RIGHT"`, `"LEFT"`, `"BACK"`); `None` = can't tip that way |
| `ARC_RADIUS_CM` | core | 14.5 | bench 10: brushes outside -> smaller, inside -> larger |
| `ARC_KP_HEADING`, `ARC_K_RADIAL_DEG_PER_CM` | core | 4, 4 | bench 10 wobble / drift |
| `CORNER_POKE_CM` | p2 | 17 | 0 turns it off; max = 30.48 - intake reach - 1.5 |
| `DROP_STYLES` | p2 | direct, along | drop-off approach styles allowed |

### Time budget (the watchdog uses these to decide when to stop sweeping)
`EST_SEC_PER_TILE` 2.9, `EST_SEC_PER_RUN` 1.6, `EST_SEC_PER_TURN` 1.8, `EST_DUMP_SEC` 6, `EST_HOME_STEP_SEC` 5,
`TIME_SAFETY_MARGIN_SEC` 8 (all core). After a few real runs, compare the planned times with the log
timestamps and adjust: too low and it overruns 120 s, too high and it stops sweeping early.

## Trying a change in the simulator first
Any constant can be overridden without editing files:

    python3 sim/run_sim.py --map all --seeds 6 --noise 2 --set core.ARC_RADIUS_CM=13.5 --set p2.CORNER_POKE_CM=0

To watch one run, use the dashboard: `python3 dashboard.py --sim`, then press Phase 1.

## Without the dashboard (fallback)
Every step also runs on its own over SSH, as before:
- `python3 bench_tests.py N`
- `python3 auto_tuner.py`
- `python3 demo_runner.py`, which has a menu for the phases and the full demo

If the dashboard is open, it notices the script and keeps its hands off the brick; its STOP still works. A script started this way
with no dashboard running also starts the old live viewer (`visualizer.py`) on port 8080, and that viewer keeps
running afterwards. If `dashboard.py` then says the port is taken, it prints the viewer's pid to `kill`.
