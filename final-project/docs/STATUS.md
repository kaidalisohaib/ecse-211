# Snowplow project status

Newest first. Group 13, ECSE 211 F26 final project.

## 2026-11-24 (newest): the robot dumps on the LEFT

User: "the robot we have always dumps on the left" (the code assumed a back pour, guessed from the photos).
- `DUMP_MINUS = "LEFT"` (motor A -140 empties it, lab 2026-11-22), `DUMP_PLUS = None`. Dashboard / bench 9 follow.
- `DUMP_POINT_FORWARD_CM` +6 -> **-10** (photo estimate: the bin's middle ~10 cm behind the axle; MEASURE). With
  +6 a left pour would have landed ~16 cm off the bucket centre (bucket half-width 15).
- A one-side pour fixes the final heading on the drop-off tile, so Phase 2 got:
  - "turn" drop-off style: drive in along the edge the other way, turn 180 on the edge lane, pour (for a drop
    tile whose "along" side is off the table or a house). Before the 180 next to a house the last run must be
    >= `TURN_LANE_RUN_CM` (25) to reach the edge lane: it reverses first if an arc left it ~5 cm.
  - 2-tile last runs along the edge (`plan_dump_approach(max_run_tiles=2)`), so a needed U-turn happens one tile
    further out, on a tile without a house beside it.
  - `core.pour_shift`: the axle stops `-DUMP_POINT_FORWARD_CM` past the tile centre (bin behind = past), cut short
    next to a house in the next edge tile (robot stays 1 cm from its line); 3 s penalty per cm off centre.
    "direct" now moves to that point from wherever its turn left it (was a fixed distance: 3 cm off).
  - U-turn at pre_tile (the way there ends facing away) costs 2 turns, +1000 next to a house.
  - Drop-off plan chosen again when the watchdog backed out of the street (was planned for the street's end:
    a U-turn next to a house, sim random-18x5-18 lost the robot).
  - `leave_dropoff`: backs out one tile to turn there when that's quicker than "straight ahead" (facing away
    from home on an edge lined with houses: sim random-9x10-35 return 29 s -> 10 s; P2 122 -> 104 s), and
    `finish_seconds` counts that way home.
- Sim: `run_sim` records the true pose when the bin tips (`pour`: angle to straight out over the edge, opening
  along the edge from the bucket centre, axle to the edge); stress flags "pour off the bucket" (>15 deg or
  >8 cm) and "no pour".
- Results (`sim/session/lab_nov22/`, 1t noise 1, 150 boards):
  | run | clean | touches | P2 median | pour off centre median / max | axle to edge |
  |---|---|---|---|---|---|
  | back pour, old code (`pour_back`) | 147 | 0 | 90.8 s | 0.3 / 3.5 cm | 9.6-10.6 |
  | left, first try (`pour_left`) | 138 | 1 | 93.8 s | 0.2 / 11.8 cm (8 off the bucket) | 11.7-15.4 |
  | left, final (`L_1t`) | 147 | 0 | 92.6 s | 0.2 / 5.0 cm | 11.8-12.9 |
  Final also: box houses 147/150 (`L_box`), noise 2 97/100 (same as back pour, `B`/`L_n2`), all kinds n 40
  (`L_all`): touches identical to the back pour (11, edge / 9x10-27), 18x5 23/40 (back 26), P2 +2 s median on
  2-table boards. Back pour with the new code (`B_1t`) unchanged 147/150. Bench sim 10/10.
- Still to measure on the robot: `DUMP_POINT_FORWARD_CM` (axle to the middle of the bin opening, - = behind),
  `EDGE_LANE_SHIFT_CM` (does the left lip reach over the table edge? the wheels must stay on), and confirm
  "left" = the robot's left looking forward (belt in front): the dashboard Dump button prints "bin pours left".

## 2026-11-24: lab table photo (box houses, thin lines)

User sent a photo of the lab table with the "houses" to check and adjust.
- From the photo: houses are ULINE 10x10x10 in boxes (25.4 cm) on 30.48 cm tiles, 4-6 cm apart, not always
  centred or square to the grid; one or two near the photo edge look close to (maybe over) a line. Grid lines
  ~0.5-0.6 cm wide (sim had 0.9). Red contrast good: table R~171, line R~45 in the photo.
- Sim: `SNOWPLOW_SIM_HOUSES` / `stress.py --houses` (`tile` = full tiles, default; `box` = 25.4 cm, up to
  2.5 cm off centre, up to 5 deg turned; or `SIZE,JITTER,ROT`), own random stream so the robot noise is
  unchanged. Line width default 0.9 -> 0.55 cm (`--line-cm`). P1 contacts are now reported too (were only
  visible as P1 clearance 0).
- Results, 1t noise 1, 150 boards (`sim/session/lab_nov22/`), robot code unchanged:
  | houses / lines | clean | maps OK | touches | min P2 clearance |
  |---|---|---|---|---|
  | full tiles, 0.9 cm (`live_graphs`) | 147 | 150 | 0 | 0.25 |
  | full tiles, 0.55 cm (`line055`) | 147 | 150 | 0 | 0.44 |
  | boxes, 0.55 cm (`boxes`) | 147 | 150 | 0 | 0.54 |
  | boxes up to 1.5 cm over a line (`over15`) | 135 | 150 | 12 runs | 0 |
  | 12 in boxes (`box12`) | 13 | 150 | 135 runs | 0 |
  Thin lines: P1 home error median 0.55 -> 0.33 cm, line detection keeps up. Gaps between boxes don't fool the
  ultrasonic (5-ray +-7 deg cone; a gap is never straight beside the tile centre).
- The 12 `over15` touches (traced with a call-stack hook): all during 90 deg arcs (10 street corners with houses
  on both sides, 2 origin arcs), all on the OUTSIDE of the turn, ~20 deg into the arc: the 15 cm round back
  swings to within ~0.5 cm of the tile line (sim tracking error ~1.5-2 cm off the ideal arc).
- Tried ARC_RADIUS_CM 14.5 -> 14.0 (more outside room): worse, 18/150 touches on full tiles (inside now tight),
  33/150 on `over15`. Discarded: 14.5 stays. No robot-code change: a 15 cm-radius plow in a 30.48 cm tile has
  ~0.5 cm to spare in arcs, so boxes must stay inside their lines.
- Other kinds, default settings, n 40 (`all_default`): loose 39/40, 18x5 26/40, 9x10 28/40, edge 8/40 (edge
  boards put the street on the table edge, outside the rules; same with 0.9 cm lines). Bench sim 10/10.

## 2026-11-24: live sensor graphs; code review fixes

User: show the sensors in real time when testing by hand (line detection, distance) without slowing things
down; how fast does the dashboard update during Phase 1 / 2; review the whole codebase for bugs.
- Live graphs: every sensor card has a 10 s graph of every reading (`/api/stream`, server-sent events, 25
  batches/s), thresholds drawn (LINE_THRESHOLD + suggested; WALL_THRESHOLD_CM + FORWARD_STOP_CM), readings past
  them orange; the big number / LINE / HOUSE tag follow the stream. No job: the dashboard reads the 3 sensors
  50 times/s (`--sensor-hz`). During a job: every reading the robot code takes (drive loop near lines: ~50/s in
  the sim, more on the robot), and the job's telemetry thread reads any port the code hasn't read for 0.05 s
  (so >= 20/s; sim: `World.color_red/us_cm(peek=True)`, no sim time, own noise). Capped at 250/s per port.
  `/api/state` dropped its sparkline history (3.4 kB per poll).
- Dashboard took the job's UDP telemetry only 6 times/s (tied to the map redraw): now every 0.03 s, map still
  redrawn at most 6/s. Sample -> page push: median 17 ms during a job (sim), 11 ms idle.
- Review fixes: dashboard started in the background ignored Ctrl-C / `pkill -INT` (inherited SIG_IGN; now
  restored); `TILE_X_MIN, TILE_X_MAX = -1, 7` one-line tuples were invisible to the dashboard and its Constants
  panel (split, one per line); auto-tuner spin / back-off loops had no time limit.
- Checked: sim bench 10/10, tuner within 0.4 %, 1t n1 147/150 (P1 69 / P2 91, clearance 0.25, unchanged,
  `sim/session/lab_nov22/live_graphs.json`); real path with the dummy BrickPi: tile / dump / turn / scan, dashboard
  real mode streams 50/s idle, 20/s in a job; page JS: `node --check` + a fake-DOM run of render / graphs.
  Not seen in a real browser yet.

## 2026-11-24: jobs froze after "Initializing Port ..." (dump, drive 1 tile); live sensor telemetry

User: the dump and "drive 1 tile" buttons (maybe others) printed the initialisation lines and then froze; how
often does the dashboard update the sensors (colour / ultrasonic looked slow while advancing and by hand)?
- Cause (most likely; robot offline, not seen on it): `init_hardware` called the course's `wait_ready_sensors()`,
  which waits forever for every sensor created, ultrasonic S4 included. A port that sends no data (ultrasonic
  unplugged with the Phase 2 hardware, loose cable) hung the job right after its "Initializing Port" line.
- Fix: `core.wait_sensors_ready()` waits at most `SENSOR_READY_TIMEOUT_SEC` (5 s) in all and prints each port
  ("S4 ultrasonic: NO DATA after 5 s ... check the cable"). Phases / bench tests stop (`SensorMissing`, clean
  "[JOB] stopped" line) if a sensor they need is missing: colours always, ultrasonic in Phase 1 (Phase 2 only if
  `PHASE2_SIDE_US`). Tile / turn / dump only warn; scan needs the ultrasonic. Auto-tuner uses it too.
- Also: `drive_straight_distance` (the "Drive 1 tile" button, origin moves) had no time limit; now 3 s + twice
  the cruise time, with a "[DRIVE] timeout ... check the drive ports" line.
- Telemetry: the job sent motor/sensor snapshots 5 times a second and only the values the robot code had read
  itself ("Drive 1 tile" never reads the colour sensors, so the page showed old values). Now 10 per second
  (`jobs.TELEMETRY_SEC`), and a sensor the code hasn't read for 0.15 s is read by the telemetry thread (the
  BrickPi's latest value: no ping, no mode change). Without a job the dashboard reads the ports at 10 Hz
  (`--hz`) and the page asks for them every ~0.1 s.
- Sim: `SNOWPLOW_SIM_UNPLUG=4 python3 jobs.py --sim sample tile` tests a missing sensor. Bench sim 10/10,
  1t n1 147/150 (P1 69 / P2 91 s, min clearance 0.25: unchanged, `sim/session/lab_nov22/sensor_wait.json`).
  Real path with the dummy BrickPi: tile / dump / turn / scan finish; telemetry 34 snapshots in 3.9 s, age 0.

## 2026-11-23: bug sweep for the real robot (user at home, robot offline)

User: "check everything to make sure there are no more bugs because it slows things down a lot", and where to
change ports / parameters (answered in README "Where to change things").
- Python 3.9: `vermin` says every file needs at most 3.9 (it catches e.g. `zip(strict=)`, `bisect(key=)`).
  ruff (pyflakes, bugbear subset): no real errors.
- Fake BrickPi (sim) vs real `utils/brick.py`: every method the code calls exists with the same signature. The
  dashboard's real-hardware class only uses real methods.
- Every dashboard job run on the real-robot code path with the course's dummy BrickPi (no `brickpi3` on the
  laptop): scan, tile, turn, dump, bench 1-10, Phase 1 (40 s), Phase 2 (50 s): no tracebacks (bench 3/7/9 wait
  for typed numbers, as designed; the auto-tuner stops with "did not see the target line", as it should with no
  lines). Dashboard real mode: jog, reset enc, belt power, motor dps, drive, job start all OK.
- Fixed: the `dump` job with no direction defaulted to "+" (off since `DUMP_PLUS = None`): now the set direction.
- Changed: colour sensors are created in red mode (the real `get_red()` otherwise switches the mode and waits on
  the first read, i.e. at the start of a run).
- Sim unchanged after the fixes: 1t n1 147/150, bench 10/10, tuner within 0.4 %.
- Not testable without the robot: `DRIVE_POLARITY` on the new chassis, turret direction, colour readings on the
  table (8 / 21 seen in the lab, under the threshold), sensor and turret offsets. Bench 1-3 cover them.

## 2026-11-23: Phase 1 turret on B, ultrasonic S4; robot repo repaired; dashboard on the Pi

- User: Phase 1 turret motor on B (the belt's port), ultrasonic on S4. `TURRET_PORT = "B"` / `DUMP_PORT = "A"`
  replace `TURRET_OR_DUMP_PORT`; dashboard motor roles per phase. Commit ff3fa80, pulled on the Pi.
- The Pi's `~/ecse-211` git repo was corrupt (8 empty object files, `main` pointing at an empty commit, likely a
  power cut). Backed up to `~/ecse-211.backup-20261123-0002`, removed the empty files, `main` -> 3065153 (last
  good, from the reflog), fetched, fast-forwarded. `lab4/lab4_navigation.py` keeps its uncommitted edit.
- First real job (turret scan) crashed: the Pi runs Python 3.9 and `float | None` annotations need 3.10.
  Every module now starts with `from __future__ import annotations` (fd258b9). Checked on the Pi itself:
  py_compile of all files, sim sample run (P1 56 s, P2 79 s) and bench sim 10/10 under 3.9.
- Dashboard "Reset enc" turned the motor by its current reading: after a jog the BrickPi still holds the
  position target, and resetting the encoder makes it chase that target. It now floats the motor first.
- SSH key from the laptop is on the Pi (`ssh pi@dpm-13.local` works without a password).
- Dashboard started on the Pi (log `/tmp/dashboard_run.log`): battery 12.0 V, all 4 motors report encoders,
  S1 8 / S2 21 red (both under `LINE_THRESHOLD` 30: check where the robot was standing, calibrate on the
  table), S4 ultrasonic 39 cm.

## 2026-11-23: code matched to the built robot; pushed to GitHub

User in the lab (photos): drive C left / D right, belt B (works at power -50), dump A (-140 deg empties the bin,
then +50 / -50 shakes), colour S1 left / S2 right; tyres 5.75-8 cm from the centre (track 13.75), colour sensors
4 cm behind the axle, a bit inside the wheels. The bin is at the back and tips backward (assumed from the photos:
confirm). Asked how to build the funnel and where to put the ultrasonic.
- Code: ports, `TRACK_WIDTH` 13.75, `SENSOR_OFFSET_BY_PHASE` -4, new `ROLLER_SWEEP_POWER = -50` (belt runs on
  power; dashboard belt buttons and bench 8 follow it), `DUMP_TILT_DEG` 140, `DUMP_PLUS = None`,
  `DUMP_MINUS = "BACK"`, `DUMP_SHAKE_DEG` 50. A dump toward a `None` side now refuses to move (the + button would
  swing the bin forward into the belt). `SENSOR_SPACING` still 12: MEASURE.
- `auto_tuner.py` seeded its first spin with a fixed 12 cm track: on 13.75 it stopped ~45° short and measured the
  track 17-24 % low. Seeds now come from core: sim error 0.0-0.4 %.
- Sim follows the wiring: `sim/world.py` reads ports, polarity, track, radius, sensor spacing from
  `snowplow_core.py`; motors in power mode now move (10 dps per %).
- Back pour: turns in place on the drop-off tile (to face inland, and to leave). An inland tile Phase 1 never saw
  can be a house (u_reversed: contacts on every seed). Fix: after a BACK pour, stop `EDGE_LANE_SHIFT_CM` toward
  the edge before turning. (A first try that treated unseen tiles as houses cancelled the shift away from real
  ones: 22 contacts in 150. Reverted.)
- Sim (`sim/session/lab_nov22/`): 1t n1 147/150 clean, 0 contacts, P1 69 / P2 91 s median (side pour: 88).
  n2 97/100, one contact (random-15). Fixed turret-less Phase 1 (US fixed forward): 20/60 clean, P1 median 121 s,
  14 wrong maps; with the turret 59/60. So Phase 1 needs the turret.
- Pushed `final-project/` to GitHub (commit 0c07605; `.gitignore` keeps out logs, media, PDFs, sim results,
  calibration). Pi pull at `~/ecse-211` still to do (no SSH key; the user runs it).

## 2026-11-22: why Phase 1 and Phase 2 leave the origin differently; compact S-curve rejected

User: "in phase 2 it did this curve which seemed better / in phase 1 it first turned 90 degrees then did an arc
... why is it different and does the S curve make the odometry less accurate or not?"
- Why different: the Phase 2 S-curve ends 11 cm past the start tile centre and the front reaches ~8 cm into the
  next tile. Phase 2 knows that tile is street. Phase 1 doesn't (random-132 has a house there).
- Odometry: no worse. Departure sim (±3% track-width error, pose error on reaching (0, 2)): 0.4-0.66 cm for the
  old diagonal step, turn + quarter arc, and S-curve alike. Heading error right after leaving: diagonal 1.3°, arcs
  0.05-0.2°. The two S arcs turn opposite ways, so a track-width error mostly cancels. Line snaps fix the rest.
  Time to (0, 2): diagonal 11.5 s, turn + quarter arc 10.3 s, S-curve 8.3 s.
- **Tried and rejected:** a compact S-curve for Phase 1 (two 79° arcs, radius 9.4, front stopping 1.5 cm short
  of the next tile). The controller can't hold arcs that tight: it ended 1.3 cm off the lane and 7° off heading.
  P1 clearance went 1.7 -> 0.0 cm (it turned in place 3 cm past the centre after finding a house ahead), and
  0.5 cm after a fix. A 3-5 cm margin made it worse (0.3-0.6 cm: tighter arcs). The gain was ~1 s of P1 median.
  Phase 1 is back to turn + quarter arc.
- Kept: `drive_run` backs onto the stop point after a forward-watch "wall" stop if it's already past it by more
  than 1 cm, so a later turn in place can't swing into the house. With the compact S-curve gone it rarely fires.
- Stress (`sim/session/origin_final/`), same as `gate_rule/`: 1t n1 147/150 clean, P1 median 68, P2 88. n2 98/100.
  loose 76/80, 18x5 50/80, 9x10 63/80 (random-9x10-27 still wrong). Min P1 clearance 1.07 / 0.98 / 1.07 cm,
  as before. Bench sim 10/10.

## 2026-11-22: gate check skipped when the way in already answers it

User: "why was this green check done? it feels useless" (the L reading at (0, 2) on the sample map).
- It decides which side of the s1-s2 edge is outside the street (it isn't always the tile nearer the origin:
  u_reversed). It costs ~0.8 s: 0.43 s reading plus 0.38 s turning the turret back.
- New rule in `not_street_reason` (Phase 1): if the robot drove through the tile beside a gate tile (across the gate
  axis), that tile isn't a house, so if the gate tile were the street, the street would turn into that tile.
  It can't be a street tile if it touches an origin / drop-off tile, or (STREET_RULES) is on the outer ring.
  Same assumption as the existing rule (origin / drop-off tiles are never houses or street).
- 150 boards: the reading is now skipped on 59 by the new rule and on 19 by the older rules; 72 still measure.
- Stress unchanged: 1t noise 1 / 2 all mapped right, 0 contacts; loose / 18x5 / 9x10 / edge the same as before
  (`sim/session/gate_rule/`). Sample P1 55.8 -> 55.0 s.

## 2026-11-22: origin arcs, arc out of the street, drop-off arc onto the bucket lane

User (screenshots): "we don't always need to put ourselves in the center of a square... only turn when we need to",
"at the exit in phase 2 why does it not do an arc instead of a 90", "at the drop off... this weird curve".
- **Origin arcs** (`core.ORIGIN_ARCS`, `leave_origin_on_arc`, `reach_origin_on_arc`, general `drive_arc_about`):
  - The old start was turn 45, crawl 21.6 cm, turn back: 5.9 s before the first run. The old home step was
    4.3 s.
  - Leaving, first run sideways: a quarter arc of radius 15.24 round a pivot in the origin tiles ends on the
    start tile centre already heading that way (turns in place on the origin first if needed).
  - Leaving, first run straight on (Phase 2 only, the next tile is a known path tile): an S-curve (two 60° arcs)
    ends on the lane 11 cm past the centre, with no turn at all. Phase 1 doesn't get it: the front then reaches
    ~8 cm into the next tile, unknown in Phase 1 (sim random-132 has a house there: contact).
  - Home: from an origin tile centre facing toward the origin's side, one quarter arc ends on the intersection.
    Otherwise the diagonal step as before.
  - A simple "merge while driving" was tried first: −4.5 s, but it ended 1-1.5 cm and 3-9° off. Rejected.
  - Phase 1 keeps its old route scoring (`choose_start(..., keep_route=True)`). With the new departure costs it
    picked a route through the unknown street on random-113 and the gate check then read the street as outside.
- **Arc out of the street:** the sweep's last tile (the exit, or the last street tile) turns 90° into the drop-off
  path; it's an arc now (`nexts` / `radii` in `run_phase2`). There's no corner poke on the exit tile (no snow).
- **Drop-off arc onto the bucket lane** (`arc_radius_for`): the arc before the last run was always radius 14.5,
  ending 0.7 cm inside. The last (often one tile) run then had to swerve 3-4 cm toward the bucket, which made
  the hook in the screenshot, and it poured still 12° off. The arc radius is now sized to end on the bucket lane
  (12..19 cm). Houses inside / outside the corner, and beside the tile before it (the run in ends shifted:
  only ±3 cm of room in a street), keep it at 14.5.
  - First version only checked the corner tile's neighbours: sample map clearance 1.5 -> 0.22 cm (shifting
    toward a house on the last street tile). Fixed by the "tile before" check.
  - Your board (random-1462): pours at x −18.0 (lane −18.24), y 82.6 (82.2), heading 179.3° (was ~192°).
- **Results** (`sim/session/origin_arcs/`):
  - Fixed maps:
    - sample: P1 58.7 -> 55.8 s, P2 79.3 -> 72.6 s.
    - u_reversed: P1 70.5 -> 67.6 s, P2 91.8 -> 87.0 s.
    - long: P1 61.6 -> 59.9 s, P2 96.7 -> 95.0 s.
    - 0 contacts, home error 0.4-1.2 cm.
  - 1t, noise 1, 150 boards: P1 median 71 -> 68 s, P2 93 -> 88 s, all mapped, 0 contacts, 147/150 clean.
  - Noise 2, 100 boards: P1 72 -> 68 s, P2 92 -> 87 s, all mapped, 0 contacts. Lowest clearance 0.08 cm
    (random-15, a street-corner arc during the sweep: noise drift, not a new arc).
  - loose 76/80 clean. 18x5 44 -> 50/80, 9x10 59 -> 63/80 (only random-9x10-27 wrong, known). Bench sim 10/10.
- On the robot: the arcs depend on `TRACK_WIDTH` (bench 10 first). `ORIGIN_ARCS = False` goes back to the old
  steps.

## 2026-11-22: ultrasonic filter, dashboard look, bug fixes

User: "by simply taking a median will it not corrupt some values", "if the readings are fast enough might as
well take more", "see if there are other bugs", "the right colour sensor graph when it's grayed out should still
advance", "search online for skills for better ui/ux and not vibe coded feel".
- **Sim now makes ghost echoes** (`world.us_cm`: 1 % x noise, a short reading between 3 cm and the true distance)
  besides the 2 % x noise missed echoes. On the OLD filter this gave 7/150 wrong maps (noise 1) and 14/100
  (noise 2), each with Phase 2 house contacts. Results in `sim/session/us_ab/`.
- **Ultrasonic fixes** (`snowplow_core.get_us_cm` and callers):
  - Real bug: "no echo" (255) readings were dropped before the median, so on an open side `[255, 255, ghost]`
    left only the ghost: one bad reading read as a house. 255 now votes ("nothing in range"); only failed reads
    are dropped. Ties take the lower value (`median_low`: "house" is the safer call).
  - Consensus: if a scan's readings disagree about the wall threshold, take more (`US_SCAN_EXTRA` 4,
    `US_GLANCE_EXTRA` 2). Costs time only when unsure.
  - Side glance 2 -> 3 samples (`US_GLANCE_SAMPLES`): with 2, the median is the mean, so it can't outvote anything.
  - Forward watch: rolling median of 5 (`US_FORWARD_WINDOW`, was 3). Samples are at least `US_SPACING_SEC` apart,
    so on the robot each is a new ping. Phase 1 exploring confirms a forward-watch "house ahead" with a
    stationary 5-sample reading before mapping it (transit already did).
  - Bench [3] measures the sensor's real update rate (wave a hand) and suggests `US_SPACING_SEC`. Reading faster
    than the sensor pings returns the same ping twice: a "median of 5" would really be of 1-2 pings.
  - Result (ghost sim, 1t boards): noise 1 150/150 mapped right, 0 contacts (was 143/150, 7 contacts); noise 2
    100/100, 0 contacts (was 86/100, 12 contacts). P1 median unchanged (71 s / 72 s).
- **Regression, ghost sim, noise 1, 80 boards per kind** (`sim/session/us_ab/all_n1.*`):
  - 1t, loose, 18x5, 9x10: every map right.
  - Contacts: only 9x10 has any, 2/80 (random-9x10-6 / -44). Both happen on the long drop-off drive, after a
    back-out. Phase 2 doesn't use the ultrasonic and the new re-plan didn't fire, so they come from before.
    Not fixed yet.
  - Over time: 36/80 on 18x5 and 19/80 on 9x10. These are distance-bound, as before.
  - Edge streets are still about 20 % right (known).
  - The fixed maps are the same: sample P2 79.3 s, 0 contacts. Bench sim 10/10.
- **Other bugs fixed:**
  - `drive_to_dump_and_pour`: a run cut short (timeout) left the later runs counting tiles from the wrong start.
    It now plans again from where it stopped (up to 3 times), like `travel` does.
  - Constants panel (`tuning.py`): an indented comment line continuing one constant's comment was shown as the
    help of the next constant (e.g. `US_FORWARD_OFFSET_CM` showed `GLANCE_WHILE_DRIVING`'s tail).
  - Static check (`uvx ruff`, F/B rules): no undefined names or real defects; the late-binding warnings are
    intentional default-arg captures.
- **Sensor graphs keep moving**: one history point per dashboard sample (10 Hz) for every port; a stale job value
  or an error is a gap (null), so old points scroll out instead of the line freezing.
- **Dashboard look** (guidance: Anthropic's frontend-design skill and its "AI slop" list):
  - Layout: map + Run/Drive first, then Sensors, Motors, Console.
  - Header grouped as status | settings (segmented buttons) | STOP. "Robot asks" only shows while a job runs.
    Messages are a toast at the bottom, so there is no empty strip under the header.
  - Tokens: graphite surfaces with 1 px borders; one accent (the robot's blue on the map); amber = waiting for
    you; red = stop. Tabular numbers in the UI font; monospace only for the console and constant names.
  - No middle-dot strings: label/value pairs ("Speed 0 dps", "Power 12 %"). Port badges on the cards.
    Visible keyboard focus. Reduced motion respected. The login page matches.
  - The map canvas's own dark colours are remapped to the page's surfaces.
  - Kept offline: no web fonts (the robot serves the page, the lab may have no internet).

## 2026-11-21: arcs at every transit turn; drop-off path drawn

- User: "why not always do arcs if it's quicker than 90 degrees". `p2.ARC_EVERY_TURN = True`: every 90° transit
  turn is an arc, not only next to a house (street corners already were).
  - Sim, 100 one-table boards, noise 1: P2 median 97 → 92 s, max 128 → 120 s, over-time 3 → 1, 0 contacts.
    Home error mean 0.57 → 0.51 cm. Median lowest clearance 1.65 → 1.41 cm.
  - Noise 2: P2 median 96 → 91 s, home error max 1.55 → 1.24 cm, the same 1 contact as before (random-353).
  - The fixed maps are unchanged or faster (sample P2 82.7 → 79.3 s). Real arcs depend on `TRACK_WIDTH`:
    bench 10 first.
- User: "from the exit to the drop I don't see the arrow". `drive_to_dump_and_pour` planned its path without
  sending it; it now emits `plan`.

## 2026-11-21: random boards, table size, stress test, fixes it found

User: "stress test our software with many different maps", "an option for a random map in the selection for the
sim", "do we still have the option to change the dimension of the map in case they decide to put two tables".
- **Random boards** (`sim/maps.py random_map`, name `random[-WxH][-edge][-loose]-SEED`, same name = same board):
  - Street 1 tile wide with houses on every table tile beside it; outside-corner houses with p = 0.7.
  - Open tiles before the gate and after the exit, with no house beside the exit.
  - No other houses. Origin and drop-off tiles are free.
  - Strict (default): a direct staircase route with no houses from the origin to the gate, from the street's end
    to the origin, and from the street's end to the drop-off.
  - `-loose`: only some route. `-edge`: the street may run along the table edge.
- **Dashboard sim map list:** "random: new board" (uses `TILE_X_MAX` / `TILE_Y_MAX`), "...street may run along
  the edge", "...2 tables wide (18x5)", "...2 tables tall (9x10)". Also `run_sim.py --map random-18x5-7` and
  `jobs.py --sim NAME`.
- **Table size:** `core.TILE_X_MAX` / `TILE_Y_MAX` (Constants panel); keep the MINs at −1.
  - Sim maps carry a `table`; `run_sim.apply_table` sets core from it and `World(table=)` checks wheels against it.
  - The visualizer / dashboard draw the size from the meta / sim events. vizlog adds core's table to meta.
  - Phase 2's tile loops use `core.all_tiles()` (they used a fixed −2..9 × −2..5 range).
- **`sim/stress.py`:** parallel runs (22 processes). A run fails on a wrong map, > 120 s, a Phase 2 contact,
  off-table, home error > 3 cm, or a crash. Results go to `sim/session/stress_*.json`; `--show NAME` prints a
  board.
- **Bugs found and fixed:**
  - **Phase 1 gate check:**
    - (a) "a driven tile beside it has no house" is dropped: the way to the gate can run through the street.
    - (b) One house beside a tile no longer means inside: also read the other side. Only houses on both sides
      mean inside; U-shaped streets put the other arm's houses beside the outside tile.
    - (c) A one-tile street means the wrong side was picked: explore again from the other gate tile
      (`RETRY_ONE_TILE_STREET`).
    - (d) When the rules already say which gate tile is outside (origin / drop-off tile, outer ring), head only
      for that one.
    - (e) In the "look properly" fallback, a street tile behind the robot no longer counts as an open side
      (it declared the exit a tile early after a misread glance).
  - **Phase 2 contacts:**
    - Home on an origin tile with no house beside it; the start tile likewise (`choose_start_tile(tight=)`).
    - The transit to the gate no longer drives through the street; a 90° turn at a tight gate tile is an arc
      (`travel(then_heading=)`).
    - After the pour, drive back to the tile centre line (`recentre_along`), or `SPIN_SHIFT_CM` (3) from it,
      away from a house beside the tile.
    - On a tight drop-off tile, leave straight forward or back one tile before turning (`leave_dropoff`).
    - Turns in place on the drop-off tile happen 3 cm away from a house beside it (`spin_shift`). The last drop
      run steers harder (`DROP_LANE_GAIN` 5, `DROP_MAX_STEER_DEG` 15): the 3 cm shift reached only 0.7 cm in a
      one-tile run before.
    - "direct" pours are allowed with no room behind (pour point 6 cm off the bucket centre, +30 s cost), so
      the turn-at-the-centre fallback is rarer.
  - **Generator:** no outside-corner house beside the gate or exit tile (it made the street end ambiguous).
- **Tried and dropped:**
  - Phase 1 transit cost for unknown interior tiles: no effect on 150 boards.
  - A "pivot" corner: the 24 cm robot has only ±1.6 cm sideways room in the street, so it touched walls.
  - The corner arc stays. It misses each corner tile's outer ~40%, which is the main reason for ~85%
    pom-poms.
- **Final results (sim, noise 1):**

  | boards | mapped right | P2 contacts | over 120 s |
  | --- | --- | --- | --- |
  | 1 table strict, 150 | 150 | 0 | 8 (P1 3, P2 5) |
  | 1 table loose, 150 | 149 | 1 | 15 |
  | 18x5, 150 | 150 | 0 | 74 |
  | 9x10, 150 | 149 | 2 | 43 |

  - 1 table strict medians: P1 71 s, P2 99 s; pom-poms 85% mean.
  - Noise 2, 100 one-table boards: 100/100 mapped, 1 contact (corner-arc drift), 1 over time.
  - Before the fixes, one table (150 boards): 12 wrong maps, 13 contacts on correctly mapped boards.
  - Over-time boards are distance-bound: the gate, street end and drop-off are on opposite sides. Only faster
    transit would help; `CRUISE_DPS` and `LINE_CROSS_MAX_DPS` are hardware questions.
- **Not handled:**
  - Edge streets: about 20% mapped right, with or without `STREET_RULES`. Off-table reads as open, which breaks
    the exit and gate logic.
  - A street that turns right at its first tile **and** whose gate can only be reached through the street (the
    loose / two-table wrong maps above).
- Bench 10/10; the three fixed maps are unchanged or better (u_reversed P1 76.9 → 70.4 s).

## 2026-11-21: dump direction set by where motor + / − pours

User: "can we have the option to either dump left, right, backward to have the option when we'll test the dump
mechanism", then: "what's the point of having 3 buttons for dump if they all do the same thing". They were right.
One motor (A) tips the tray, so it can only pour two ways, and "Dump back" sent the same +110 as "Dump right".
Untested on the robot; the tray mechanism isn't built yet.
- **Setting:**
  - `core.DUMP_PLUS` / `core.DUMP_MINUS` (Constants panel) = where the tray pours when motor A turns + / −:
    `"RIGHT"`, `"LEFT"`, `"BACK"`, or `None` if it can't tip that way. Default `"RIGHT"` / `"LEFT"`.
  - `DUMP_TILT_DEG` is used both ways.
  - Phase 2 plans for the quicker of the two; set one to `None` to force the other.
  - Removed: `DUMP_SIDES`, `DUMP_BACK_SIGN`, `DUMP_BACK_TILT_DEG`.
- **Dashboard:** the Dump card has "Dump motor +" and "Dump motor −" (`jobs.py dump +` / `dump -`); the console
  says where it pours. `dump_snow_payload("+" | "-" | "RIGHT" | "LEFT" | "BACK")`.
- **Bench 9:** for each angle jogged, asks which way the tray pours, then prints `DUMP_PLUS` / `DUMP_MINUS` and
  `DUMP_TILT_DEG` (the larger angle). If either way pours back, it reminds you to measure the back constants below.
- **BACK parking:**
  - "direct" drives in facing the bucket and turns 180 in place; "along" drives in along the edge and turns 90.
  - Both turn `EDGE_LANE_SHIFT_CM` (3 cm) toward the bucket from the tile centre.
  - `back_to_edge` then reverses until the lip overhangs by `DUMP_BACK_OVERHANG_CM`. The reverse is capped so the
    rear contact stays `DUMP_BACK_EDGE_MARGIN_CM` inside the edge.
  - MEASURE: `DUMP_BACK_LIP_CM` (9), `DUMP_BACK_SUPPORT_CM` (8). With these placeholders the cap wins: the axle
    ends 10 cm from the edge and the lip 1 cm inside, with a warning printed.
- **Found in testing:** BACK on `u_reversed` touched a house 17–21 times. The drop-off tile (4,−1) has a house
  at (4,0), and turning at the tile centre left 0.24 cm margin. The 3 cm shift fixed it.
- **Sim, all maps:**
  - 0 contacts for RIGHT/LEFT (default, unchanged), BACK/None, RIGHT/None and LEFT/BACK.
  - BACK-only P2 is 86.8–99.3 s, about 5 s slower than side dumps.
  - Bench 10/10; `jobs.py dump +/-` OK.

## 2026-11-21: corner arc radius 14 → 14.5 cm (funnel)

User: "is it normal that the robot is closer to the turning wall when doing the arc?" Yes, by design: the arc runs
inside the tile centreline because the funnel's outer tip (±12 cm at 9 cm ahead) and round back swing outward.
But 14 cm dated from the square intake. Ideal arc with the funnel outline: inside = radius − 12, outside 3.0 /
2.5 / 1.8 cm at 14 / 14.5 / 15.24 cm.
- Sim, all maps × 4 seeds, P2 clearance lowest / mean: 13.5 → 0.87 / 0.96, 14.0 → 1.33 / 1.44, 14.5 → 1.69 / 1.90,
  15.0 → 1.39 / 2.08 cm; 0 contacts in all; P2 time ±0.5 s; pom-poms 817 / 822 / 832 / 836 of 938.
- `ARC_RADIUS_CM = 14.5` (saved via `tuning.save_constants`, so it's logged and backed up). The comment and the
  bench test 10 hint are updated.
- Bug fixed: `drive_arc` and `arc_lane_offset` took `radius=ARC_RADIUS_CM` as a default argument, fixed when the
  file loads, so `--set core.ARC_RADIUS_CM` in the sim did nothing. They now read it at call time. Real runs and
  the Constants panel were fine, since they edit the file.
- After: 12/12 maps OK, 0 contacts; bench 10/10.

## 2026-11-21: sensor cards no longer show stale job readings as live

User screenshot: during a job, both colour cards said LINE (6.6 / 11.7) with the robot mid-tile, and the graphs
looked upside down. Cause: during a job the cards show the *last value the robot code read*. The code only reads
the colour sensors inside the ±5.5 cm window around each grid line, and stops reading a sensor once it has seen
the line. So the last value is nearly always a line value, kept until the next window. The sensors and line
detection were fine.
- `jobs.py` (sim and real) and `sim/run_sim.py`'s truth telemetry now send `sensor_age` with each snapshot.
- `dashboard.py`: values older than `STALE_SEC` (0.5 s) are greyed and say "last read by the job N s ago (it only
  looks near grid lines)"; stale repeats no longer go into the graph. Checked on the page in sim: the graph now
  shows the table ~58 with short dips at lines.

## 2026-11-21: Phase 2 trusts the map; Phase 1's drop-off visit is off

User: "there are only blocks around the road ... from phase 1 you know exactly where all the blocks are, and in
phase 2 you can just navigate to drop off". Agreed, plus corner boxes: on the handout's sample map, (6,3) sits
diagonally at the outside of a street corner, and Phase 1 doesn't record it.
- `phase2_snow_removal.py`: `TRUST_MAP = True`. Blocked = the houses beside the street + `corner_boxes()` (a tile
  diagonal to a street tile with a house on both tiles between them), minus tiles Phase 1 drove. Every other
  tile is open floor, so P2 plans anywhere. False = the old way (only tiles P1 drove).
- `phase1_mapping.py`: `SCOUT_DROPOFF_ON_RETURN = False` (turn it on again if `TRUST_MAP` is off).
- Sim: P1 long 64.0-65.4 → 60.9-62.2 s; sample and u_reversed unchanged (the drop-off was already on the way
  home). P2 unchanged (long 94.5-96.0 s, no 7 s loss). All maps OK, 0 contacts. Forced P2 limits 50/65/80 s:
  0 contacts, lowest spin clearance 3.24 cm. On `long` at 50 s, P2 now skips the street and goes round it:
  41 s (was 69 s through the street). Stress 12/12 maps (same P2 drift brushes as before). Bench 10/10.
- Not covered: a filler box that touches no street tile, e.g. (4,0) inside the U on `u_reversed`. It's walled
  in by houses, so no route can reach it.

## 2026-11-21: Phase 1 always maps the whole street; stops only measure what they need

**Whole street.** User: "we should still finish the mapping phase otherwise how would you even return to the
origin?" (The handout says Phase 1 must "map out the entire street"; it also allows touching walls in Phase 1.)
- `phase1_mapping.py`: `ALWAYS_FINISH_STREET = True`. Exploration ignores the time watchdog; the watchdog only
  decides whether to scout the drop-off on the way home. Forced 40 s limit: the old code finished at 38-50 s with
  all 3 maps wrong; now 62-82 s, maps correct, home within 0.8 cm.

**Fewer readings (user's idea: skip checks whose answer is known; at a corner with a house ahead, if one side is
impossible, just turn the other way).** `SKIP_KNOWN_READINGS = True` (False = old full L/F/R scans, 1.19 s each):
- `look_around()` measures only what the decision needs. The house ahead is known from the forward watch; the
  open side is known from the side glance. A direction is impossible (no reading) if its tile is off the table,
  a known house, already driven, or touching an earlier street tile (one-wide street with houses on both sides,
  the same rule as `infer_houses`). If only one way is left, the robot turns without looking.
- Safety net: if a blind turn runs straight into a house, the robot turns back to the heading it came in on,
  marks both as houses and looks properly. Tested by forcing a wrong rule on sample: map still OK, +4.4 s.
- Entrance: the user asked why verify at all. Tried trusting "the tile nearer the origin is outside": it broke
  `u_reversed` (its first street tile (2,0) is nearer the origin than the outside tile (2,-1)); the robot
  drove out of the street, map wrong. So the gate keeps a check, but it's one side reading (a house right beside
  = inside) instead of a full scan plus a second scan after turning round.
- Readings per stop now: gate 1, corners 0-2, exit 1-2 (was 3 each). P1 time, old → new: sample 64.1 → 62.5,
  long 69.2 → 66.1, u_reversed 84.0 → 81.5 s. All 12 runs map OK, 0 P2 contacts. `--uncalibrated --noise 2`:
  12/12 maps OK in both, 2-3 s faster. `sim/test_bench.py` 10/10.
- `visualizer.py` draws only the rays measured in a "scan" event. `REL` is hidden from the Constants panel.
- Follow-up (user screenshots: "in both those cases the scan isn't necessary"):
  - Gate: if the robot drove through a tile right beside the gate tile, that tile has no house there, so it's
    outside. No reading (sample (0,2), u_reversed (2,-1)).
  - `STREET_RULES = True` (user's rules): no street tile in the outer ring of tiles (no room for its house), and a
    house straight ahead means a corner, not the exit. So at sample (5,2) the top-row tile is ruled out and the
    robot turns right without looking. A run that ends on an outer-ring tile with no house ahead has reached the
    exit (sample (5,-1), u_reversed (6,-1)): no readings.
  - "Could be open" (exit test) and "could be the next street tile" (turn choice) are kept apart.
  - Fallbacks: a wrong blind turn → turn back and look properly. Everything ruled out → look properly (forced
    test: map OK, 61.0 s). The `long` sim map breaks the edge rule (street on row -1) and still maps OK.
  - Readings per P1 run now: sample 0 + 0 + 0, long 1 + 0 + 1 + 1, u_reversed 0 + 1 + 0 + 0. P1 times: sample
    59.1-60.4 s (was 63.7-64.9 with full scans), long 64.9-66.1 (68.8-70.0), u_reversed 78.1-79.7 (83.5-85.1).
    All maps OK, 0 contacts in normal runs; bench 10/10. Stress P2 brushes are unchanged (drift, see above).
- Follow-up 2 (user screenshots: the gate check on `long` isn't needed; "why stop at the drop-off after the exit?"):
  - Tiles known to have no house: the 4 origin tiles (the robot starts on them), the drop-off tile (it pours
    there) and every tile driven (`FREE`). A gate tile with one of them right beside it (across the gate axis),
    or that *is* an origin or drop-off tile, can't be the street → decided without a reading. This doesn't need
    `STREET_RULES`. Origin and drop-off tiles are never a next street tile, and a known house-free side counts
    as open at stops.
  - Readings at stops now: sample 0, long 0 + 0 + 1 + 1, u_reversed 0 + 1 + 0 + 0.
  - Scouting kept (`SCOUT_DROPOFF_ON_RETURN = True`): with it off, P1 is 1.7-3.2 s faster but P2 on `long` is
    ~7 s slower (95 → 102 s). P2 plans on driven tiles, and the way home from the drop-off (-1,2) wasn't driven.
    P2 is the tight phase. New: if the way home already passes through the drop-off tile (sample, u_reversed),
    the robot just drives home through it, no detour and no stop. It still detours on `long`.
  - P1 times now: sample 57.4-58.7 s, long 64.0-65.4, u_reversed 76.4-78.0 (were 63.7-64.9 / 68.8-70.0 /
    83.5-85.1 at the start of this round). P2 unchanged. All maps OK, 0 contacts; stress 12/12 maps; bench 10/10.

## 2026-11-21: Phase 2 never turns in place next to a house (watchdog wrap-up + transits fixed)

User asked if the turns in place at the street entrance and exit touch houses, then said "yes" to fixing what
came up. Probe: `/tmp/spinprobe/probe.py` (scratch, `P2_LIMIT=<s>` limits Phase 2's time only). It logs every P2
spin with its distance off the tile centre and its min footprint-to-house clearance.
- Geometry: the funnel footprint reaches 15.0 cm from the axle. A house directly beside a tile centre is 15.24 cm
  away (0.24 cm margin). A diagonal house corner is 21.5 cm away (~6.5 cm margin). The entrance, exit, drop-off
  and home turns are all on diagonal-only tiles: 5.5-8.3 cm clearance, robot ≤0.4 cm off centre (2.8 cm at the
  drop-off on purpose).
- Found: when the time watchdog cut the sweep short, the wrap-up turned in place *inside the street* (U-turns,
  90 deg on street tiles). Also, a transit through the street turned in place at its corners (long map, when the
  street was skipped). Old code, P2 limit 65/80 s: 85 contacts in 4 of 12 runs, the others down to 0.48 cm.
  (A first probe also limited Phase 1, so Phase 1 stopped mapping early; those numbers mixed two problems.)
- Fix, `phase2_snow_removal.py`:
  - The sweep goes on into the exit tile, so a street ending in a corner gets an arc and the robot is out of the
    street before it turns.
  - `way_out(i, k)` = the cheapest finish that never turns in place in the street: drive the rest at cruise,
    or (on the first straight) reverse out with `back_out()` (lane hold, steering sign flipped), or don't enter.
    The watchdog now uses that cost.
  - `travel()` replaces `navigate_tile_with_obstacle_avoidance` for P2 transits. With `TIGHT` (tiles with a
    house directly beside them), the planner refuses U-turns and starting turns there, and 90 deg turns there are
    driven as arcs (also on the drop-off approach). A "direct" drop-off on a tight drop-off tile is a last resort.
    Start/end tiles that are tight get a printed warning.
  - New constants: `BACK_OUT_DPS` 220, `BACK_OUT_SEC_PER_TILE` 4.6. `TIGHT` is hidden from the Constants panel.
- `snowplow_core.py`: an optional `tight=` on `find_shortest_tile_path` / `plan_path` / `plan_home`. Phase 1 doesn't
  pass it, so it's unchanged. `plan_path` order: known tiles respecting tight, known tiles, any tile respecting
  tight, any tile.
- Results: normal runs (3 maps x 4 seeds): 0 contacts, the same pom-poms, P2 +1-1.5 s (the exit tile), the
  lowest spin clearance 5.5 cm. Forced P2 limits 50/65/80 s (18 runs): 0 contacts, the lowest spin clearance
  2.96 cm (was 0). The wrap-ups chose "not entering", "backing out 2-3 tiles" and "drive the rest" as expected.
  `sim/test_bench.py` 10/10.
- Left as is: with an impossible budget on the long map (50 s), the only known way to the drop-off is through the
  street, so the robot still drives it (with arcs) and finishes at 69 s. If Phase 1 stops mapping early, Phase 2
  sweeps a partial street and warns when the route's end tile is tight. The `--uncalibrated --noise 2` brushes
  are lateral drift (~2.5 cm at heading ~85 deg in a street), not turning.

## 2026-11-21: side glance no longer pauses the drive loop

User asked whether odometry should keep running during the turret side glance. It did (the encoders keep counting, one
update covers the whole move), but for ~0.55 s (~7 cm) the loop skipped heading hold and lateral steering, line
checks, goal/decel checks and the tile hook.
- `snowplow_core.py`: new `GLANCE_WHILE_DRIVING = True` (in the Constants panel, "Ultrasonic and turret"). The glance
  now runs as a small out → read → back sequence inside `drive_run`'s loop (`set_turret_angle(..., wait=False)`,
  new `turret_at_target()`, same 0.04 s settle before the ping). Steering, odometry, line snaps and stop checks keep
  running. Forward readings are skipped while the turret is turned. If a run ends mid-glance, it finishes standing
  still. `False` = the old blocking code, unchanged. The glance's map logic moved into `glance_result()` (same rules).
- Sim, `--map all --seeds 4`, old → new: P1 times within 0.4 s, every map OK, 0 contacts, home error the same
  (≤1.3 cm). On `sample` seed 0: the same 6 glances, the same tiles, readings within 1 cm.
  `--uncalibrated --noise 2`, 3 seeds: the same as the old code. That includes P2 brushing houses on 4/9 runs in both:
  this was already there and the glance change didn't cause it. `sim/test_bench.py` 10/10.
- In the sim the gain is small (heading barely drifts in 7 cm). On the real robot the wheels keep heading hold
  through the glance. Check in the lab: the turret swings out and back while the robot keeps driving straight.

## 2026-11-21: Constants panel, corner poke off, ultrasonic dots placed right

- **Constants panel (user: "have the entire env file on the side … edit it, save it and run it directly").**
  - New `tuning.py`. It lists every top-level `NAME = literal` in `snowplow_core.py` (77), `phase1_mapping.py` (7)
    and `phase2_snow_removal.py` (13), grouped by the files' section headers, with their comments as help.
  - Saving rewrites only the value text: comments stay, and the type is checked (a bool stays a bool).
  - The whole batch is refused on any bad value. Each save keeps a backup in `logs/constants_backup/` and a line in
    `logs/constants_changes.log`.
  - The auto-tuner's `calibration_params.json` values are editable in the same panel, and the file can be deleted.
  - Bring-up steps list the constants they tune; "Constants for this step" opens the panel filtered to them.
  - Red colour cards get "Use N as LINE_THRESHOLD" once table and line are stored.
  - The panel has its own STOP. Keys typed in it don't drive.
  - The `.py` files stay the single source of truth.
- **Corner poke off** (user: the funnel can't keep what it pokes at; set through the panel):
  `p2.CORNER_POKE_CM = 0.0` (was 17). Sim, 4 seeds per map:

  | Map | Poke 17 | Poke 0 |
  |---|---|---|
  | sample | P2 84.8–86.4 s, 91.9 % | 79.0–80.5 s, 89.8 % |
  | long | 105.0–106.7 s, 89.8 % | 93.3–95.0 s, 88.0 % |
  | u_reversed | 103.1–104.8 s, 90.3 % | 91.4–93.0 s, 86.5 % |

  All runs: 0 contacts. The sim credits a pom-pom as soon as it crosses the funnel tip line, so the poke's +2–4 %
  is optimistic.
- **User asked why ultrasonic dots sit off the glance line, and why odometry lags the true pose.**
  - Cause: a side glance swings the turret out and back (~0.55 s at `TURRET_DPS` 400, plus settle) inside the drive
    loop. Odometry isn't updated meanwhile and the robot rolls on ~6–8 cm at `EXPLORE_DPS`.
  - So the blue box freezes, and the dot was placed from the pre-swing pose. The real pose error stays ~0.2 cm.
  - Fixed in the drawing (`visualizer.State`, no robot code): each reading is placed at the pose interpolated to its
    timestamp. Side glances: median error 4.5 → 0.5 cm, worst 4.8 → 0.7 cm. Forward readings unchanged (~0.5 cm).
  - Possible robot-code change (not made): call `update_odometer()` in `set_turret_angle`'s wait loop, so the blue box
    also moves during glances.
- Also fixed: a fast double-click on Continue could answer the next question once the page had shown it. Clicks within
  0.4 s of a new question are now ignored. Tested with a 150 ms double-click.

## 2026-11-21: dashboard bug round 2 (user reports + a browser-driven bug hunt)

- **User reported:**
  - bench steps "already starting before I pressed anything";
  - "how does test 9 already know the map";
  - flicker when clicking drive buttons;
  - WASD should work like a game;
  - the robot shape on the map is the old design;
  - do files survive a reboot.
- **Fixed:**
  - **Ready questions:** the auto-tuner ("Starting in 2 s…") and bench 3 (turret moves first) now wait for a ready
    question from `jobs.py`.
  - **Flicker:** the message line toggled 10×/s while driving and made the whole page jump. Now it is a fixed-height
    line, and click errors show for 5 s.
  - **Lost clicks:** cards are built once and only changed parts are rewritten.
  - **WASD:** game-like. W/S and A/D combine into curves, the newest key wins per axis, and reversing steers like
    a car. Dropdowns, checkboxes and the answer box hand the keys back after use.
  - **Robot shape:** Phase 2 is now the pinch belt + V-funnel inside a 15 cm circle, 24 cm wide, tips at (±12, 9).
    - Changed: `sim/world.py` (`FOOTPRINT_P2_FUNNEL`, `use_intake`, now the default) and `run_sim --intake` (`funnel`
      by default; `square` / `chamfer` kept).
    - `snowplow_core.ROBOT_OUTLINE[2]` changed too; it is display only.
    - Sim, 2 seeds per map: sample P2 85.2–86.4 s, 56/61 and 51/53; long 105.5–106.7 s, 70/78 and 70/77; u_reversed
      103.4–104.8 s; 0 contacts; min clearance 1.38 cm.
    - `sim/test_bench.py`: 10/10 OK. `test_auto_tuner`: radius within 0.06 %, track within 0.5 %.
  - **Simulator truth:** the tan houses and grey robot are the simulator's truth. A "show the simulator's truth"
    checkbox explains this and can hide them.
  - **Files:** sim files moved from `/tmp` (lost on every restart) to `sim/session/<map>/`, plus a
    "Reset sim files" button. Robot files (`calibration_params.json`, `street_map.json`,
    `dashboard_colour_cal.json`, `dashboard_points.json`, `logs/`) were always kept in `final-project/`.
- **Bug hunt** (reviewer agent with a Chrome DevTools driver in `/tmp/cdp/`; all fixed and re-tested):
  - map-clear race at job start (now sent on the telemetry port before the job starts);
  - two tabs fought over the map size (now drawn at one size, 1150×560, and each tab scales it);
  - page jump when a question appears (the question bar is always there, cards have a minimum height);
  - a typed answer carried over to the next job;
  - blank map and robot jumping to (0,0) after "Phase 2 needs a map";
  - demo showed Phase 1 parts at the hardware-swap question (jobs now send their hardware phase);
  - off-table list grew without limit in the dashboard's sim world (~125 MB/h);
  - map/speed dropdowns not locked during a job;
  - console not refreshed after a dashboard restart;
  - doubled Continue clicks: now silent and harmless (questions are numbered);
  - "Release hardware" is hidden in sim;
  - hand-driving into a house or off the table now shows a warning;
  - motor cards keep the job's final encoder values;
  - while a question waits, the sim board and robot are drawn.

## 2026-11-21: one entry point, `dashboard.py` (+ `jobs.py`); critic round applied

- **User:** "just run the dashboard and have a --sim to test things locally and inside the dashboard I can switch mode,
  do the tests etc". So: `python3 dashboard.py` on the Pi, `python3 dashboard.py --sim` on the laptop. Nothing else
  is needed. The old CLIs still work over SSH as a fallback (`docs/BRINGUP.md` and the new `README.md` say so).
  No robot code changed.
- **`jobs.py` (new):** runs one job as a child process: phase1, phase2, demo, bench N, autotune, tile, turn, dump, scan.
  - Questions asked with `input()` go to the page; the answer comes back on stdin.
  - Telemetry goes over UDP, including `hw` snapshots of motors, sensors and battery at 5 Hz.
  - `--sim MAP` builds the world; each sim map gets its own session folder for calibration and street map.
  - The real `calibration_params.json` / `street_map.json` are never touched in sim.
  - Exit code 3 = "Phase 2 needs a map".
  - Ctrl-C now works even when started from a background shell; a second STOP can't interrupt the clean-up.
- **Dashboard now does everything from the page.**
  - Top bar: map and speed (sim), Hardware 1/2, colour mode, Release, STOP.
  - Motor cards: dump / scan jobs, jogs, belt on/reverse/off, reset encoder.
  - Drive: hold-to-drive with a dead-man of 0.35 s, plus 1 tile / turn 90 jobs.
  - Run card: gate and drop-off, saved to `dashboard_points.json` when Phase 1 starts; Phase 1, Phase 2, Full demo;
    the Bring-up list in BRINGUP.md order.
  - The robot's questions appear under the sticky top bar; the console shows everything the jobs print.
- **Safety fixes from the critic round** (two reviewer agents: safety, and UX/organisation):
  - Hand drive and belt are braked when the hardware is released, when an outside script appears, and when no page
    has polled for 1.5 s.
  - STOP refuses hand driving for 1 s.
  - Jog limits are cleared before driving and before every job.
  - Prompt race fixed; the job reader can't leave the page stuck on "running"; bad requests get a clear error.
  - Only the drawing and sampling threads are niced, so jobs run at normal priority.
  - SIGTERM / SIGHUP quit cleanly.
  - Outside programs are also detected by an open `/dev/spidev*`, not just by script folder.
  - If port 8080 is taken by an old `visualizer.py` viewer, the error names its pid.
- **Tested on the laptop** (sim and the dummy BrickPi):
  - Every job type ran: sample P1 64.1 s, P2 85.3 s; the demo with its prompts; autotune into the session file.
  - Dead-man, page-lost brake, STOP stickiness, double STOP, Phase 2 with no map, map switching, bad inputs.
  - Real mode with the dummy brick: login, job + STOP, outside script detected and locked out.
  - Screenshots at 1500×1250 and 1280×800.
  - `sim/test_bench.py`: all 10 OK. `run_sim` sample seed 0 unchanged (P1 64.1 s, P2 85.2 s, 55/61).
  - Not yet run on the Pi.
- **Needs the user's go-ahead (robot code):**
  - `vizlog.AUTO_WEB_VIEWER = False`: scripts run over SSH start an old viewer on :8080 that stays up and blocks
    the dashboard's port.
  - The docstrings of `vizlog.py` / `visualizer.py` / `demo_runner.py` still describe the old viewer.
- Known limits:
  - In sim, bench steps that say "hold the sensor over X" read wherever the robot stands; the job prints a note.
  - Log replay is still `visualizer.py --replay` (not in the page).
- **Later (user approved):** `vizlog.AUTO_WEB_VIEWER = False`, and the old-viewer docstrings in `vizlog.py`,
  `visualizer.py` and `demo_runner.py` are updated. Wifi drops are handled:
  - Hand driving stops within 0.35 s; anything else the dashboard started (belt) stops after 1.5 s.
  - A running job (phase, demo, bench) keeps going on the Pi, since it is autonomous. STOP works again once the
    page reconnects. Requests time out after 2.5 s, so the page recovers right away.
  - SIGHUP (the SSH session that started the dashboard dies) no longer quits it: its prints go to
    `logs/dashboard.log`. Tested: Phase 1 kept running through a SIGHUP; Ctrl-C and SIGTERM still quit.
- **Fix (user report: map frozen during the auto-tuner in --sim):** the auto-tuner and most bench tests never call
  `vizlog.start`, so they sent no map telemetry. `jobs.py` now starts it for them (UDP only, no log file), and the
  dashboard clears the map whenever a job starts. Checked: the auto-tuner's true pose moves on the map; bench 2, 5,
  6, 8 and 10 run clean.
- The "step 1" entry below is superseded: the Drive and Run panels work now.

## 2026-11-21: dashboard step 1 built (`dashboard.py`)

- User approved the dashboard plan ("yes that sounds good"), incl. runs from the browser and an access code.
- **Built step 1, `final-project/dashboard.py`** (new file, no robot code changed). `python3 dashboard.py` on the Pi,
  `--sim [MAP]` on the laptop. Live motors A–D (encoder, dps, power, BrickPi flags), sensors 1–4 (status + value,
  sparkline), battery, Phase 1/2 labels, colour sensors in red mode (LINE_THRESHOLD) or RGB mode with per-sensor
  "This is table / This is line" (saved to `dashboard_colour_cal.json`; shows projection toward line 0–1 and off-axis
  distance), map + console from telemetry (reuses `visualizer.py` Scene/Player on UDP 5005), STOP / Esc, random
  4-digit access code (`--code`, `--no-code`).
- Safety: reads ports only when no other Python script from `final-project/` runs (scans /proc every 0.5 s); never
  sets up sensors while one runs; re-sets them up when it ends. Unregisters `utils.brick`'s exit-time `reset_brick`
  and restores `~/brickpi3_pid`, so quitting the dashboard never resets a running script. STOP = SIGINT to robot
  scripts, SIGKILL after 1.5 s, then `set_power(0)` on A–D. "Release hardware" for scripts it can't see.
- Tested on the laptop: sim mode (login 401/303, state, map frames, RGB calibration, layout screenshot), real mode with
  the dummy BrickPi (idle reads; `bench_tests.py` detected -> stopped reading; STOP sent Ctrl-C then killed it, since
  background jobs ignore SIGINT; reading resumed). Not yet run on the Pi.
- User asked: use RGB distance to stored table/line colours instead of red only? Answer: for a black line on a light
  table it measures the same thing (all channels drop); it helps against coloured markings that are dark in red, plus
  a little noise averaging. Proposed: keep red, measure with the dashboard's RGB mode (table, line, any coloured
  marks, and the read rate) and switch only if red misreads something. Robot code unchanged.
- Next: step 2 (manual control, dead-man) once the user has tried step 1 on the Pi.
- Later the same day (user: how to test locally; red calibration; why the UI differs from the wireframe):
  - Layout now matches the wireframe: motor cards carry their buttons, Drive and Run panels are shown greyed out
    with "step 2"/"step 3" tags (they don't work yet). Map + phase clock + console stay one panel (visualizer Scene).
  - Red mode calibration: per sensor "This is table / This is line" (mean of last 10 reads) -> suggested threshold
    (midpoint) and margin, green mark next to the code's LINE_THRESHOLD (orange). Saved in
    `dashboard_colour_cal.json`; the robot code is unchanged (one shared LINE_THRESHOLD).
  - `sim/run_sim.py` now emits `hw` events (motor pos/vel, last value the robot read per sensor port) at 5 Hz sim
    time; the dashboard shows them on the cards when a script/sim run sends them. Sample seed 0 unchanged
    (P1 64.1 s, P2 85.2 s, 55/61). Local test: `python3 dashboard.py --sim` + `python3 sim/run_sim.py --map sample
    --viz-udp 127.0.0.1:5005 --realtime 1` (checked end to end with a headless screenshot).
  - Step 3 will add the same `hw` events to `vizlog.py` on the robot (robot code: needs the go-ahead).

## 2026-11-20: decisions; web dashboard planned

- **User decision:** "assume a 15cm with an arc for now and we'll see later". So: everything inside 15 cm of the
  axle, width ≤ 24 cm, arc corners (current code), funnel tips ~9 cm ahead (~51° wings), container wholly behind the
  belt's top end, dump motor front-left tipping left (proposed, not decided by the user: left vs under the belt, side
  vs behind).
- **New request:** a web dashboard on the robot showing every motor and sensor, live data, manual control and all
  the buttons needed. User: "first plan things out". Plan given in chat (nothing built yet): one stdlib server on the
  Pi that owns the BrickPi while idle and runs missions/bench tests as a child process (it never touches the brick
  while one runs; STOP kills it and stops the motors), dead-man heartbeat for manual driving, access code, works
  against the simulator's fake brick. Existing pieces it builds on: `visualizer.py --web` (map page on :8080),
  `vizlog.py` (UDP telemetry), `bench_tests.py`, `demo_runner.py`. Needs the user's go-ahead (touches robot code:
  vizlog snapshots, prompts answerable from the browser).

## 2026-11-20: pinch belt built and works by hand; funnel, 13.5 cm circle, dump motor

- **Update (later the same day), supersedes the R 13.5 recommendation below.** User: the motor on the right is the
  belt's NXT; the dump motor will go on the left or under the belt; dump side vs behind undecided; no funnel yet; the
  15 cm radius was meant **with arc corners**, not turning in place. Sim, arcs (current code), footprint = 15 cm circle
  capped at a width limit, funnel tips on the circle at (±w, 9 cm):

  | Width limit | Dump sides | P2 time sample / long | Picked sample / long | Min clearance |
  |---|---|---|---|---|
  | ±12 cm | both | 84.8–86.4 / 105.0–106.7 s | 94% / 89% | 1.41–1.54 cm |
  | ±12 cm | left only | same / 107.2–109.1 s | 94% / 89% | 1.41–1.54 cm |
  | ±12.5 cm | both | 84.8–86.4 / 105.0–106.7 s | 96% / 91% | 0.92–1.05 cm |

  So the user's plan works: with arcs, the radius only sets the reach; the width is what touches houses, so keep it
  ≤ 24 cm. Tips 9 cm ahead give ~51° wings (slide if μ < 0.82), gentler than 62° for R 13.5 + spin. Spinning saves
  ~3 s (sample) / ~6.5 s (long) but needs the steeper funnel. Recommended: keep arcs + 15 cm, width ≤ 24 cm; dump motor
  on the left, front-to-back hinge along the container's left bottom edge, tip left (`DUMP_SIDES = ("LEFT",)`, the only
  code change, costs ~2.3 s on the long map). Rear pour: new drop-off code, hinge where the circle is narrowest.
- User asked again whether 13.5 + turning in place is better. Answer: it depends on the funnel. Build everything except
  the funnel inside 13.5 cm, so both options stay open (`ARC_CORNERS` is one flag). Then test a 60° card V with taped
  wings: if pom-poms slide, 13.5 + spin (saves 3–6.5 s; the long map's ~13 s sim margin will shrink on the real
  robot). If they stick, extend the funnel to 15 cm and keep arcs.
- User: tipping left, the container would hit the belt if it sits under it; and how to fit enough volume in 13.5?
  Answer: right. A side tip turns the container in the side-to-side plane, so it sweeps everything above it (up to
  √(w²+h²) from the hinge) within its own front-to-back span. So the container must sit wholly behind the belt's top
  end (belt drops pom-poms over the front wall, no chute on the belt reaching over it), with the dump motor in front at
  the left. Volume is set by y0 = how far behind the axle the belt top ends. Floor area inside the circle behind y0
  (width capped ±12) and wall height for 200 / 400 cm³ (≈100 / 200 pom-poms at an assumed ~2 cm³ each loose):

  | y0 | R 13.5 area | height 200 / 400 cm³ | R 15 area | height 200 / 400 cm³ |
  |---|---|---|---|---|
  | 6 cm | 130 cm² | 1.5 / 3.1 cm | 173 cm² | 1.2 / 2.3 cm |
  | 8 cm | 84 cm² | 2.4 / 4.8 cm | 125 cm² | 1.6 / 3.2 cm |
  | 10 cm | 44 cm² | 4.6 / 9.1 cm | 77 cm² | 2.6 / 5.2 cm |
  | 12 cm | 13 cm² | no | 37 cm² | 5.4 / 10.8 cm |

  Max possible: 16/tile → sample 112, long 192 pom-poms; sim averages 53–78. Pom-poms heap under the drop point
  (they don't roll), so allow ~2x. R 13.5 needs y0 ≤ ~8 cm, i.e. a shorter, steeper belt (~40–47°). Asked the user to
  measure where the current belt top ends (behind the axle, height) and the loose volume of 50 pom-poms.

- **Built (team, videos `~/Downloads/IMG_8145.MOV`, `IMG_8148.MOV`, photos in chat):** new chassis with two EV3 Large
  motors on car wheels. The two lab-4 treads run side by side as one ~4.5 cm belt, driven straight off the NXT motor,
  over a green card ramp whose lip touches the table between the wheels. The belt rises backward into a card
  container. Hand-fed pom-poms are pinched, carried up and dropped into the container (about 25 collected in the
  video, none seen falling back). **Not tested yet:** the robot driving into pom-poms on its own. The front photo shows
  pom-poms beside the belt sitting right in front of the wheels, so the funnel also has to clear the wheel paths.
- **User:** "I was going to have everything within a 15 cm radius from the center of rotation" (green sheet: axle and
  wheels, container behind with its back rounded). "we're missing the funnel and also to attach a motor behind to dump
  the container".
- **Sim (footprint = circle, Phase 2 turns in place at corners via `--no-arcs`, 3 seeds per map, wrapper
  `/tmp/simwrap/circ.py` patches `world.FOOTPRINT_P2_SQUARE` and the intake catch line; no repo code changed):**

  | Footprint | Corners | P2 time sample / long | Picked sample / long | Min clearance |
  |---|---|---|---|---|
  | circle R 15 | spin | 81.5–83.1 / 98.4–100.1 s | 94% / 92% | 0 cm, 555–2850 contact ticks per run |
  | circle R 14 | spin | same | 94% / 91% | 0.54–0.94 cm |
  | circle R 13.5 | spin | same | 94% / 90% | 1.04–1.44 cm |
  | 45° V (tips 10.75 cm ahead) | arcs (current code) | 84.8–86.4 / 105.0–106.7 s | 95% / 91% | 1.46–1.61 cm |
  | 60° V (tips 6.6 cm ahead) | arcs | same | 94% / 89% | 1.51–1.62 cm |

  The sim's catch line doesn't model pom-poms sliding along the wings, wheels rolling over them, or wings shoving
  them during spins. 15 cm can't work: the street is 30.48 cm, so it leaves 0.24 cm per side.
- **Recommended:** R 13.5. Funnel tips at ±12 cm sit only 6.2 cm ahead of the axle, so with the throat at the belt
  entry (~1 cm ahead, 4.5 cm wide) the wings are ~62° from the driving direction and pom-poms must slide at μ < 0.53.
  A curved wing can't do better than straight between the same end points. Hand test at 60° with taped wings decides.
  If they stick: keep arc corners and a 45° funnel (sim above). Width left inside R 13.5 behind the axle: ±12.1 cm at
  6 cm back, ±10.9 at 8, ±9.1 at 10, ±6.2 at 12, ±3.6 at 13. Sim totals are 53–78 pom-poms per run, about 160–200 cm³.
- **Dump:** EV3 Medium on port A (Phase 1 turret port, free in Phase 2), mounted beside or above a wheel near the
  front of the container, its axle as a front-to-back hinge, container tips sideways. This matches the code's
  `DUMP_SIDES`/`DUMP_TILT_DEG = 110`. A motor hanging off the back sticks out of the circle where it is narrowest, and
  a rear pour needs new drop-off code.
- Code changes this implies (not made, need the user's go-ahead): `ARC_CORNERS = False`, `ROBOT_OUTLINE[2]`,
  `DUMP_POINT_FORWARD_CM` negative (container behind the axle), belt on the NXT motor port, `SWEEP_DPS` vs belt speed.

## 2026-11-19: roller and pusher dropped; whole intake being redesigned

- **Latest (2026-11-20):** the user: "we can put a funnel in front, the track width is only a couple of cm". So: both
  treads side by side (~4–5 cm belt) behind a card V-funnel (25 cm mouth to ~4.5 cm throat, each wing covers
  10.25 cm sideways). The wing angle is set by the friction between a pom-pom and the wall: a pom-pom slides along a
  wall at angle φ from the driving direction only if μ < cot φ. Options (throat ~6 cm ahead of the axle, belt tucked
  between the wheels):

  | φ | wing length | front reach | works if μ < |
  |---|---|---|---|
  | 35° | 14.6 cm | ~21 cm | 1.43 |
  | 45° | 10.3 cm | ~16 cm | 1.0 |
  | 60° | 5.9 cm | ~12 cm (same as the roller) | 0.58 |

  Clear tape on the wings should lower friction. Asked the user for a hand test: a card V with a 4–5 cm gap pushed over
  scattered pom-poms at 45° and 60°; check they slide out the gap without sticking or jamming 2–3 abreast.
- Before that, the same day, the user's idea **G, a pinch belt**:
  "I feel like this idea is very risky and also you would need to stop. why not a conveyer belt against a surface
  with some space to bring the snow up into a container". F (grab) is dropped: risky and stop-and-go.
  - Sketch (chat only): the belt's lower run sits ~9–10 mm above a card ramp (45°) whose edge rests on the table.
    Pom-poms are squeezed (they grip when squeezed) and carried up, then fall off the top into a tray between the
    wheels. The NXT tips the tray sideways at the bucket (matches the code's side pour). Continuous; the belt runs at
    ~2x the drive speed.
  - Width fix for the old objection (one tread too narrow): the two lab-4 treads (kit No.59) at the two ends of a
    ~25 cm belt, folded-card angle cleats across them, one joined drive axle so both treads stay in step. The cleats
    also rake. The roller is gone, so its 2 rims are free again.
  - Rough numbers: front sprocket ~15 cm ahead of the axle (roller 12.9), so corners need the sim.
  - Open: tread width, loop length and number of sprockets; kit axle lengths; whether tape on the rubber treads is
    allowed. Suggested test: the lab-4 conveyor with a card under it at a ~9–10 mm gap, fed by hand.
- Earlier the same day: concept **F, two-jaw grab** (like a crane's grab bucket). Details and numbers at the end of
  this entry.
- **User (after building and testing):** "the roller creates too much vibrations and oscillations and detaches etc
  it's not stable at all, we need to rethink our whole model, also the slider idea is probably not good too it
  creates too much mechanical complexity, we need to rethink the whole idea altogether".
- v6 (and the v5 roller/crank-slider family) is **shelved**, not deleted: the model, renders and sources below stay as
  a record. Phase 2 code still assumes a roller; not changed.
- Proposed next concept (awaiting the user's pick): a **tilting front scoop** (front-loader bucket) on one pivot driven
  by one NXT motor. Thin paper lip on the floor while driving, tilt back to store/carry and before corner turns, tilt
  forward over the table edge to pour into the bucket. No fast-spinning parts, no slider. Fallback if the lip pushes
  pom-poms ahead: a slow swinging rake that pulls them over the lip. Suggested 10-minute hand test of the lip first.
- Sketched for the user (side views, chat only; source `/tmp/viz/concepts.html`): A tilting front scoop (pick),
  B scoop + slow rake, C clamshell grab into a hopper, D overshot loader into a rear hopper. Rejected: claw one at a
  time (too slow, can't sense pom-poms), suction (no fan), sticky tape (residue), plow alone (drag rule).
- **User:** "Realistically they're probably going to get pushed and slide, so no" (rules out lip-based pickup: A and D).
  New lead, sketched (`/tmp/viz/sieve.html`): **E, press-down sieve scoop**. Same single pivot/NXT as A, but the floor
  is a card sheet with ~9 mm slots (16 slots, 22 x 12 cm). Stop, press it flat onto a patch: the table pushes pom-poms
  up through the slots (12.7 mm > 9 mm, so they can't fall back). Fold for corners, tip forward to dump. Needs a hand
  test of slot width (9/10/11 mm). Second choice: C clamshell (pinch).
- **User:** "I don't think it's going to work since some might land in between and stay on the ground. Think better."
  (rules out E: pom-poms under the bars stay on the table.)
- Reasoning: light pom-poms on a hard table slide when pushed, so there are only two ways to lift them. Either squeeze
  them between two converging surfaces, or push them up a ramp with something behind them (roller, belt, rake,
  pusher). The second needs a looping or sliding part, which the user has ruled out. So: a squeeze, which means
  stop-and-go (conflicts with the earlier "ideally we wouldn't stop"; flagged to the user).
- **F, two-jaw grab** (proposed):
  - EV3 Medium closes two jaws through two meshing 24-tooth gears (axles 2.4 cm apart). The NXT raises and lowers the
    head on two parallel arms (pivots 2.4 cm apart), so the head stays level. 4 ports = 2 drive + these 2.
  - Jaws 25 cm wide (matches the code's 25 cm outline), lips 7 cm below the jaw axles. Open 65°: lips ~14 cm apart
    (patch 25 x 14 cm), head 4.05 cm above the table. Shut: head 7 cm up, so it rises ~3 cm while closing.
  - Cycle: drive shut and 2 cm up; stop; open in the air (the load drops into the middle of the new patch); lower
    until both lips touch; close while the NXT lets the head rise in step; lift; drive 12 cm (2 cm overlap).
    Dump = open over the bucket at the table edge (head ~7 cm past the edge).
  - Why it shouldn't leave pom-poms: the lips converge along the table (a pom-pom pushed by one is pushed into the
    other); solid edges, no gaps; the back lip lands on cleared ground, and anything the front lip lands on ends up
    inside the next patch thanks to the 2 cm overlap.
  - To check: ~2.5 grabs per tile, so each grab must take ≤ ~1.8 s to match today's ~4.4 s per tile (long map is
    already 105–111 s of 120). The shut grab reaches 13.6 cm ahead of the axle and is 25 cm wide (roller: 12.9, 22),
    so corners need the sim. ~200 g sits ~11 cm ahead of the axle, so the BrickPi and battery go behind it. If the head
    just rests on the lips, the EV3 Medium must lift it ~3 cm while closing (~14 N·cm vs ~8 rated), so the NXT should
    follow the jaws in position control instead (software only). No hopper: the load is re-grabbed every cycle and a
    big late pile may squeeze out of the jaw ends.

## 2026-11-19: v6 built: gravity chute instead of a roof, crank actually driven, roofed sled, corner caster

- **User:** "ok do it" (v6 with a roof). They were away mid-task, then said "continue where you left off".
- **Why not the roof:** a pom-pom can only reach the tray mouth (z −1.3, y 5.6) by being thrown, and that needs
  ≈ 1500 dps at the best release (24° below the axis).
  - Mouth at 5.0: 1444 dps. A lip at z −3.7, y 3.8: 1040 dps.
  - The code runs the roller at 900 dps; the EV3 Medium tops out at ~1440–1500 with no load.
  - A roof can only redirect; fluffy pom-poms lose their speed on card.
  - So v6 lowers the tray instead: the guide ends 5° below the axis and a 29° chute carries pom-poms down into the tray.
- **Found: v5's crank could not be driven.** The NXT output (53787) is a cross hole only and beams have round holes.
  The fix is kit **No.3**, assumed to be LDraw 60483 (1x2 liftarm with a cross hole + pin hole); **user to confirm**.
- **The model is now v6** (`docs/robot_3d.html`, sources in `robot3d_src/`; v5.1 kept as `assembly_v51.py` /
  `robot3d_v51.html`):
  - Assembly: 62 parts, 98 pins/axles, all pass, 1 connected group.
  - Black pins 60 of 61: 6 ear pins became blue 3L.
  - L-beams 14 of 14.
  - Card: tray floor 0.8 cm off the table; guide ends at −5° (z −4.42, y 4.02); chute at 29° to the lip at z −1.68
    (top 2.5); hood 125°→30° plus a ceiling down to (z −2.0, y 5.2); funnel walls reach ±4.3 by z −3.2 (above the rails).
  - Pusher:
    - NXT unmoved (hub z 0, y 10); No.3 adaptor drives a 5-hole crank; 13-hole rod to the top hole (y 2.8) of the
      sled L-beam.
    - Sled = plate (top 2.1) + cheeks + a **roof** reaching 5.56 cm forward, which slides under the chute lip.
    - Compaction stroke 3.02 cm (crank 140.9°→184.0°): the roof covers the landing zone the whole time, so it can run
      with the roller on.
    - Full stroke 10 cm (→302.5°) for the dump, with the roller stopped.
  - Caster: right rear, ball at x 8.8, z 8.0. It sits on a 7-hole `casterRail` pinned to the outer face of the right
    motor foot (black pin at z 5.6, a blue 3L through foot + rail + caster at z 7.2, black pin at z 8.8).
    `casterA/B/C` are gone. Left: an anti-tip foot (L-beam on the left motor foot, bottom 0.5 cm above the table once
    leaned).
  - The 2.1° lean is now drawn: the ball is 3 mm short with its pins at hub height, as it already was in v5.
- **Checks:**
  - `pusher_card_check.py`: the moving crank and rod stay ≥ **1.01 cm** from all card (rod vs ceiling, at rest), and
    the rod never reaches the roof.
  - `obb_check.py`: no LEGO overlaps at rest or over the stroke (only 3 intended axle ends in holes).
- **Loads and tipping (estimate, 875 g):**
  - Left wheel 47 %, right 21 %, caster 32 %.
  - The robot rocks back onto the left-wheel–caster line at ≈ **0.21 g** of forward acceleration; the foot catches it
    after ~4°.
  - Recommendation: ramp speed changes in the code (not changed).
- **Simulator** (`sim_outline_v6.py 4`, 12 runs; the caster fills part of the right-rear notch):
  - Snow **76.2 %** (v5: 76.3 %), 0 contact runs.
  - Minimum clearance **0.88 cm** (v5: 1.85).
- **Risks to bench-test:**
  - pom-poms stalling on the 29° chute;
  - piling against the steep funnel walls (~77°);
  - the floor sagging under the rod's downward push (use stiff card);
  - colour eyes at ≈ 0.4 cm once leaned;
  - the floor's back edge ≈ 0.2 cm above the table.
- **Files changed this step:**
  - `robot3d_src/`: `assembly.py`, `robot3d.html`, `pusher_card_check.py` (v5 version kept as `_v5.py`), and new
    `obb_check.py`, `gen_joints.py`, `sim_outline_v6.py`, `assembly_v51.py`, `robot3d_v51.html`.
  - `docs/`: `robot_3d.html`, `robot_connections.md` (rewritten for v6).
  - `renders/`: 1–9 re-rendered, plus new 13 (section at rest / compaction / full stroke) and 14 (crank adaptor).
- **Waiting on the user:**
  - Confirm No.3 = 1x2 liftarm with a cross hole and a pin hole.
  - Axle lengths in the kit.
  - Go-ahead for code changes:
    - the NXT compaction stroke (rotate to 184° and back), a full stroke for the dump, and the roller off during the
      dump;
    - speed ramps;
    - the outline/footprint update;
    - CORNER_POKE_CM 17→16 (it would win back clearance);
    - SENSOR_SPACING;
    - re-running the auto tuner.

## 2026-11-19: one roller shaft with the axle joiner; user prefers a deflector roof

- **User:** kit No.18 (×6) is the axle joiner. They think a hinged plate is unrealistic (light pom-poms would get
  swept out on the return stroke). They suggest an inclined roof that turns thrown pom-poms toward the back, or a
  guiding ramp.
- **Done (model v5.1):** the roller is on one shaft: 6L (motor output → wheel 1 → joiner at x 2.6–4.2) + 9L
  (joiner → wheel 2 → inner bearing fully, outer bearing halfway, ends x 10.6). 63 parts, 98 pins/axles, check passes.
  7L + 8L or 8L + 7L also fit. Assembly order matters: the 6L and joiner go into wheel 1 before the wheels go in the
  tube. Renders 10 and 11 regenerated; `docs/robot_3d.html` rebuilt.
- **Roof/ramp assessment (numbers for the reply):**
  - Throw height straight up = v²/2g at the pom-pom centre (r 3.65): 900 dps → 1.7 cm, 1200 → 2.9 cm, 1500 → 4.7 cm.
  - At the current guide end (+25°) the throw goes 42 % forward, so the guide must end at about 0° to −10°.
  - The roof must rise toward the back to send pom-poms back; one that falls toward the back sends them forward.
  - With the NXT 1.6 cm back and the rod pinned at the sled top, the crank and rod stay behind z ≈ −2.8, which leaves
    z −5 to −2.9 for a roof.
  - A ramp the flaps push pom-poms up only works below the axis height (above it the flaps move forward).
- **Next:** build v6 (guide end ~0°, roof, NXT moved back, sled-top pivot), or bench-test first.

## 2026-11-19: V-angled flaps in; tread conveyor ruled out

- **User:** a single tread is too narrow, and its drums (rims) are taken by the wheels and roller hubs, so no
  conveyor. V flaps: yes.
- **Done:** V-angled flaps in the 3D page (`renders/12_v_flaps.png`, build steps in `renders/10_roller_build.png`).
  The model is v5.1.
- **Pusher search with the rod pinned at the top of the sled (y 5.4), plate post resting at z −0.6:** clears all
  the card by ~0.3 cm with the NXT hub 1.6 cm further back (crank 4.0, rod 6.4) or 3.2 back (crank 4.8, rod 5.6).
  Not built yet: the NXT deck would need re-mounting.
- **Idea for continuous collection:** a hinged ("ratchet") plate that swings up over pom-poms on the return stroke,
  so short strokes can run while driving. Needs the user's go-ahead.

## 2026-11-19 (latest): user inputs on capacity and ideas (waiting on a direction)

- **User:** there may be more than 50 pom-poms (≤ 16 per tile, so up to ~110–190 on 7–12 tile streets); they would
  rather not stop during collection; short pusher strokes while driving would drop pom-poms in front of the plate.
- **User ideas:**
  - A roof on top of the plate. Good, but it can only be ~3 cm long (the space between the tray mouth at z −1.3 and
    the guide at z −4.5). So strokes while driving are limited to ~3 cm. It doesn't fix the crank going through
    the chute.
  - Funnel wings in front with a narrow centre roller. Not recommended: it adds ≥ 5–6 cm of reach (corner budget),
    the wings bulldoze pom-poms along the table, and the EV3 Medium doesn't fit in a ~9 cm roller. Suggested instead:
    V-angled flaps (ends lead, centre trails) so the roller moves pom-poms to the middle. That also helps the chute,
    which narrows from 19.2 to 8.6 cm in 3.4 cm (a jam point I hadn't flagged).
- **Proposed alternative:** the lab-4 rubber-tread conveyor (kit No.59 ×2) as the tray floor. It runs slowly while
  collecting, so the tray fills from the back without stopping; it dumps by running fast; there is no plate, roof or
  crank in the chute; and its front can sit lower, which helps the hand-off. Unknowns: loop length (fixes the drum
  spacing; ~9 cm in the lab-4 photos), tread width, what to use as drums (the 4 rims are in use), and where the NXT
  goes.

## 2026-11-19 (after v5): user questions on capacity, pusher, guide, roller shaft. Three v5 problems found

- **One roller shaft (tried, then reverted):** a 3L + 12L meeting inside wheel 1's hub. Reverted because the rim's
  cross hole is only 1 stud (0.8 cm) deep (LDraw `axlehole.dat` spans 0.8), so each axle would grip 4 mm; and moving
  wheel 1 to make room put its tyre against the motor face (x −0.60). Options: a 16L from the motor output (4 mm past
  the 22.0 width) or an axle joiner, depending on the kit. The model is back on v5's 2 × 4L.
- **Problem 1, the pusher hits the card** (user spotted it). `robot3d_src/pusher_card_check.py`: the crank goes through
  the hood deflector end (crank 201.6°) and the funnel floor (239.8°), and the rod goes through the funnel floor
  (241.9°). Overlap ~0.36 cm, i.e. the full beam half-width. Searched every crank 2.4–6.4, rod 5.6–9.6, NXT hub z 0–4,
  y 8.4–11.2: best gap −0.05 cm, so it doesn't fit. Cause: the crank and rod live in the same space as the chute
  (above the tray front).
- **Problem 2, the hand-off from the roller.** The guide ends at +25° on the back side; flaps there move up and 25°
  forward, and a pom-pom on a flap slides inward. It either rides over the top or gets thrown up and forward into
  the hood. A flap only tips a pom-pom off backward below the axis (≈ −10° to −30°), i.e. at y ≈ 2.2–3.6. The tray
  mouth is at y 5.6 (floor 3.4 + plate 2.0 + 0.2), which is set by the caster under the floor.
- **Problem 3, capacity.** Gross tray 8.6 × 12.2 × 3.0 = 315 cm³; usable to the plate top ≈ 8.2 × 11.4 × 2.0 ≈ 190 cm³
  → ~95 pom-poms of 1.27 cm (or ~70 of 1.4 cm), at a loose packing fraction ≈ 0.55. That assumes they spread evenly.
  They land at the mouth and don't roll, so the front pile reaches the mouth after a rough ~15–25. The requirements
  doc says 50 pom-poms total. Fix: short pusher strokes during collection (code), plus a bench test.
- **Next:** user decision on the front/tray/pusher rework; bench-test the hand-off with the real roller and card mock-ups.

## 2026-11-19: 3D model v5: roller held at both ends, rear caster, guided pusher

- **User decisions:** option B (19.2 cm roller, both ends supported, width 22.0); guide the pusher; rear caster with
  most of the mass on the wheels; roller from paper, card and tape only (no glue).
- **Files:** `docs/robot_3d.html` (v5); `docs/renders/1–11` (new: `8_rear_caster.png` replaces `8_rear_frame.png`,
  `10_roller_build.png`, `11_right_frame_hubs.png`); `docs/robot_connections.md` (v5); `docs/robot3d_src/`
  (`assembly.py` v5, `assembly_v4.py`, `robot3d_v4.html`, `sim_outline_v5.py`, `roller_build_fig.py`).
- **Check:** 62 parts, 98 pins/axles, one connected structure, no bad joints; box-overlap hits are false positives only.
- **Kit:** black pins **60 of 61** (tight), blue 3L 26/52, L-beams 13/14, 13-hole 7/10, 15-hole 2/2, rims/tyres 4/4,
  caster 1/1. Axles 2×4L, 4×5L, 4×8L, 2×12L.
- **Changes from v4:**
  - Roller 19.2 cm (x −10.0..9.2). Right frame in two planes (x 9.8 / 10.6): 2 L-beams pinned together form the
    bearing at the roller axis; 7-hole bottom beam → 8L axle to the right rail; 13-hole top beam → 5L axle to the
    right outer stem L-beam. 6 black pins, 2 per overlap.
  - Hubs: the 2 spare wheels (rim + tyre, 5.6 cm). Card tube 5.6 inside, taped over the tyres. Wheel 1 on a 4L axle
    from the EV3 Medium (drives); wheel 2 on a 4L axle through both right bearing holes. No.26 and No.25 unused.
  - Flaps 1.15 cm (tip diameter still 8.1).
  - Rear ball caster on the centre line at z 7.2, under a tray floor raised to y 3.2–3.4, held by 3 × 9-hole beams
    from the sensor posts. Ball ~3 mm above the table → ~2.4° lean back, roller tips ~0.7 cm up. Measure.
  - Pusher: 11-hole rod, pivot 1.2 cm above the floor; card sled (plate + 2 cheeks) under guide lips on both walls;
    L-beam runner on the floor. Stroke 0.66 → 10.86 cm behind the tray front.
  - Rear flap is passive (pushed open by the pom-poms). Leak risk with the lean: suggest a 5 mm sill and/or a weight.
- **Mass estimate:** ~875 g, CG 2.57 cm behind the axle → wheels 64 % / caster 36 %. Battery forward to z ≈ 1.2 gives
  ~74/26. Battery mass (150 g) is assumed; weigh it.
- **Simulator (3 maps × 4 seeds, noise 2, CORNER_POKE 16, half-width 9.6, reach 12.85):** 686/899 = **76.3 %**,
  0 contacts, min clearance **1.85 cm** (`/tmp/v5_sim.log`). Corner sweep in the 3D page: 3.00 cm.
- **Not done (code, needs the user's go-ahead):** `CORNER_POKE_CM` 17 → 16; `ROBOT_OUTLINE[2]` and
  `sim/world.py` footprint / `INTAKE_HALF_WIDTH` 9.6 / `INTAKE_REACH` 12.85; `SENSOR_SPACING` ≈ 19.3; re-run the auto
  tuner (track ≈ 19.2).

## 2026-11-19 (later): open problems in v4 raised by the user (all addressed in v5, see above)

- **Roller held from one side only:**
  - The "1–2 mm play" figure was a guess, not measured.
  - Options for supporting both ends:
    - (A) right frame in a 2nd plane outside: width 22.8;
    - (B) roller 19.2 cm, with the right frame in planes x 9.8/10.6: width stays 22.0, about −2.4% snow (recommended).
- **Roller hubs:** kit No.26 (black 4-armed piece) was never positively identified, and its centre hole may not be a
  cross hole. Proposed instead: the 2 spare wheel rims (No.16), which have a known axle hole and are 2.6 cm wide, built
  up with wrapped card to fit the tube's 5.0 cm inside diameter.
- **Pusher (slider-crank):** nothing guides the L-beam post. Only the rod pin at y 4.2 holds it, plus gravity on the
  runner. A moment balance shows the plate tips back about its bottom corner under load, because the rod's push acts
  3.6 cm above the floor. Proposed fix: a sled with 2 cross axles riding in horizontal guides, which makes it a true
  slider.
- **No rear support:**
  - No caster is modelled. A rough estimate puts the centre of mass ~3 cm behind the drive axle, so the robot would sit
    back on the tray's rear corner.
  - The caster can't go under the tray (floor 0.4 cm off the table) or behind z ≈ 8: during the dump the rear overhangs
    the table edge at z ≈ 9.6.
  - Proposed: the steel ball caster on the centre line at z ≈ 6, with the tray floor raised to the rail tops (y 3.2).
    Measure the caster's height.

## 2026-11-19 (latest): 3D model v4, with a 20 cm roller and 22 cm overall width

- **User decision:** 20 cm roller, 22 cm overall width. They want to know how the EV3 Medium drives and holds the
  roller, and the roller's dimensions so they can rebuild it.
- **Files:**
  - `docs/robot_3d.html` (v4), `docs/renders/1–9` (v4; `9_left_frame.png` replaces `9_motor_stems.png`).
  - `docs/robot_connections.md` (v4 guide with the roller rebuild table).
  - `docs/robot3d_src/assembly.py` (v4), `assembly_v3.py` (kept), `sim_outline_v4.py`.
- **Check:** 51 parts, 83 pins/axles, one connected structure, no bad joints. The box-overlap check finds no real hits,
  including the moving pusher.
- **Kit:** black pins 56 of 61, 15-hole beams 2 of 2, 13-hole 6 of 10, L-beams 11 of 14.
- **Changes from v3:**
  - Wheels in to ±9.6 (track 19.2), tyre faces at ±11.0.
  - New back crossbar: a 13-hole beam on two 7-hole back beams, which have 3 pins per motor. This is the stiff
    motor-to-motor tie. A low front cross axle is not possible: it needs 13L.
  - Roller axis moved to z −8.8 (from −8.0). In v3 the flaps hit the rail tips by 0.27 cm.
  - EV3 Medium inside the tube (output at x −0.6) on 4 × 13-hole beams. One left frame plane at x −10.6:
    - L-beams lock the beams (no parallelogram wobble; v3 actually had that wobble);
    - spacers and 3L pins;
    - top and bottom crosspieces → cross blocks → 5L axle to the stem L-beam and 8L axle to the rail.
  - No right frame: it doesn't fit in 22 cm. The tube is carried by 2 cross hubs on a 12L axle from the motor, plus a
    bush.
  - Sensor posts stay at ±8.0, because the motor cable socket blocks ±7.2. Eyes are unchanged at ±9.7, 10.4 behind.
- **Roller:** 20.0 cm long, inside diameter ≥ 5.0 (stationary parts need 4.66), outside ≈ 5.6. 8 rows of 1.25 cm
  flaps, tip diameter 8.1, 3.5 mm above the table. Axis 4.4 up.
- **Simulator (3 maps × 4 seeds, noise 2; outline from part vertices; intake half-width 10.0, reach 12.85):**
  - As the code stands: 707/899 = **78.6%**, 0 contacts, min clearance **0.87 cm**. The low clearance comes from
    `CORNER_POKE_CM = 17`, whose own comment gives a maximum of 30.48 − reach − 1.5 = 16.1 for this reach.
  - With `CORNER_POKE_CM = 16`: 706/899 = **78.5%**, 0 contacts, min clearance **1.84 cm**.
  - Corner-arc sweep in the 3D page: min clearance 3.00 cm.
- **Not done yet (code, needs the user's go-ahead):**
  - `CORNER_POKE_CM` 17 → 16.
  - `ROBOT_OUTLINE[2]` and `sim/world.py` `FOOTPRINT_P2_SQUARE` / `INTAKE_HALF_WIDTH` 10.0 / `INTAKE_REACH` 12.85.
  - `SENSOR_SPACING` ≈ 19.3. Track ≈ 19.2: re-run the auto tuner after the rebuild.
- **Note:** 78.5% is below the professor's ~90%. The simulator gives about 3% of the snow per cm of roller.

## 2026-11-19: why a 24 cm roller collects only ~92%, and the width trade-off

- **Where the 75 missed pom-poms go** (`/tmp/miss.py`, 3 maps × 4 seeds, roller half-width 12):
  - 47 of 899 (5.2%) were further than 11.7 cm from the street centre line, beyond the roller. On walled tiles
    pom-pom centres reach ±12.5 cm. On the first/last tiles and tiles with a missing house they can be up to ~14.6.
  - 28 (3.1%) were inside the roller's reach but missed by the path (turns, start/end of the street, drift).
  - By tile type (missed beyond the roller + missed within it):
    - straight tiles: 22 + 18
    - first/last tiles: 18 + 6
    - corner tiles: 7 + 4
- **Wall margin** (each side, driving centred) = (30.48 − overall width) / 2.
  - Overall width ≈ roller + 3.3 cm of side frames; the wheels alone are 23 cm.
  - 24 cm roller: 1.6 cm per side. 21.6 cm: 2.8. 18 cm: 4.6.
- **Handout:** it says only "without touching the street boundaries" in Phase 2. It gives no point values for touches
  or for snow percentage; ask the TA.

## 2026-11-18 (latest): size vs snow coverage, and v3's motor-to-motor tie is weak

- **User input:**
  - Professor: about 90% of the snow can get full marks.
  - The user thinks v3 is too big and risks touching houses.
  - The drive motors must be tied together stiffly for odometry.
- **Simulator (3 maps × 4 seeds, noise 2, v3 outline with the roller width varied):** snow collected vs roller width.

  | Roller width | Collected |
  |---|---|
  | 24 cm | 824/899 = 91.7% |
  | 21.6 cm (v3) | 757/899 = 84.2% |
  | 18 cm | 71.6% (71.7% with reach 10.5) |
  | 15 cm | 59.3% |

  - That is about 3% of the snow per cm of roller width.
  - No contacts in any run. Min clearance 0.76 cm at 21.6, ~1.7–1.9 cm at 18 and 15.
  - Conclusion: with one pass down each street centre, 90% needs about the full 24 cm roller. v3's 21.6 cm roller is
    already below 90%.
- **Swing radius:** the v3 outline reaches 16.2 cm from the axle centre at the roller ends and 15.8 cm at the rear
  sensors. Half a street is 15.24 cm, so v3 cannot spin in place on a street centre line.
- **v3 tie (weak):** the motors are tied only through the top frame, i.e. one pin per top rail plus two 12L axles held
  by friction in round holes. The rails and roller frames each attach to one side only. The rear crossbar was removed
  for the pusher rod, and that was the wrong trade.
- **Next (pending user choice):** v4 with a stiff low frame (back crossbar plus a cross axle through both rails), a
  24 cm roller, and a shorter rear (sensors inboard).

## 2026-11-18 (late night): 3D model v3, every part pinned on real holes

- **User correction:** in v2 the parts were not properly connected (motors, sensors, motor to motor, base). v3 fixes
  this.
- **Method:** `docs/robot3d_src/assembly.py` places each LDraw part on its real hole positions (`holes.py`) and checks
  every pin and axle. Each must engage at least 2 parts coaxially, and no pin may sit in a cross hole.
  - **Result:** 51 parts and 83 pins/axles, all passing, in one connected structure. Cardboard is taped.
  - A box-overlap check flagged no real collisions; all hits were motor bounding boxes, flush pins or axles into hubs.
  - The pusher was checked at 21 stroke positions.
- **Layout changes from v2:**
  - Motors moved from x ±7.4 to ±6.4. They are joined by the rails, the cross axles and the top frame. There is no
    rear crossbar.
  - Tray inside is 8.6 × 12.2 cm. Track is 20.2 cm.
  - The roller axis is now (y 4.4, z −8.0): reach 12.1 cm, tube 21.6 cm.
  - The pusher is a 7-hole crank rocking about 130° on the NXT hub, with a 9-hole rod. Plate travel is 10.3 cm.
  - The NXT and the Pi sit on top rails. The Pi case is pinned to 4 L-beam ears; its hole positions are assumed.
  - **Colour sensors:** hung on posts behind the motors, eyes at x = ±9.7, 10.4 cm behind the axle, 0.8 cm off the
    table. The code currently has `SENSOR_SPACING` 12 and `SENSOR_OFFSET` −5, so measure the real robot before
    changing it.
- **Clearance checks:**
  - Simulator with the v3 outline (3 maps × 6 seeds, noise 2): 0 contacts in 18 runs, min clearance 0.73 cm.
  - The page's corner sweep gives a min of 1.55 cm (v2: 1.14).
- **Kit use:** black pins 58 of 61 (tight), blue 3L pins 13 of 52, 15-hole beams 2 of 2, L-beams 10 of 14, cross blocks
  2 of 8. Axles needed: 3L, 4L, 2× 5L, 6L, 7L, 2× 8L, 2× 10L, 2× 12L. The kit's axle lengths are still not identified.
- **Files:**
  - `docs/robot_3d.html` (v3).
  - `docs/robot_connections.md`: a joint-by-joint guide.
  - `docs/renders/1-9_*.png`; 8 and 9 are new close-ups.
  - `docs/robot3d_src/`.
  - v2 is kept at `/tmp/robot3d/robot3d_v2.html` only.
- **Open items:**
  - Measure the tube's inside diameter (needs ≥ ~4.7 cm for the motor and its 4 beams).
  - Confirm the wheel size: 5.6 cm, while the code default is `WHEEL_RADIUS` 2.1 (calibration overrides it).
  - Check the BrickPi case's hole positions.
  - The funnel is 2.5 cm long.
  - The ball caster and the ultrasonic turret are not modelled.

## 2026-11-18 (night): 3D model v2, EV3 Medium inside the roller, kit parts only

- `docs/robot_3d.html` and `docs/renders/*.png` were redrawn. The previous version is kept at
  `/tmp/robot3d/robot3d_v1.html` only.
- **Kit parts:**
  - Identified from the checklist photo `20260922_134826(1).jpg`: rim No.16 + tyre No.17 are a 56 mm wheel (LDraw
    56908 + 41897), confirmed in the video. Beams No.42-46 are 7/9/11/13/15-hole (labelled on the sheet).
    No.38 looks like a 5-hole beam.
  - The panel in the 3D page counts the parts drawn against the kit counts: 13-hole 9 of 10, 15-hole 2 of 2,
    9-hole 5 of 14, 7-hole 6 of 20, 11-hole 2 of 6, 5-hole 5 of 18, NXT 1 of 2.
  - Pins and short axles are not all drawn; the axle lengths in the kit are not identified.
- **Wheels:** if they are really 5.6 cm, the code default `WHEEL_RADIUS` = 2.10 is wrong.
  `calibration_params.json` from `auto_tuner.py` overrides it anyway.
- **EV3 Medium motor:**
  - From the LDraw model: its only pin holes are within 2.8 cm of the output face (side holes 0.4 cm behind the
    face, at +-0.8 cm), and the cable socket is on the back end.
  - Mount: four 9-hole beams pinned to those side holes run out through the open end of the tube to an 11-hole
    side frame. The far corner is 2.3 cm from the shaft and the beam corners are 2.26 cm, so the tube needs
    an inside diameter of at least ~4.8 cm.
- **Roller:** the tube is 5.3 cm across (outside) with 1.4 cm flaps, so the tips reach 4.05 cm from the shaft
  (was 3.0). It is 23.2 cm long, between the side frames.
- **Front reach sweep** (proposed body, 3 seeds x 3 maps, noise 2):

  | Reach | Runs with contact | Min clearance |
  |---|---|---|
  | 11.6 cm | 0/9 | 0.73 cm |
  | 12.6 cm | 0/9 | 0.73 cm |
  | 13.6 cm | 0/9 | 0.10 cm |
  | 14.6 cm | 9/9 | hits |

  The model uses 12.6. The ideal corner path in the 3D scene leaves 1.14 cm.
- **Consequence:** the funnel is now only 2.7 cm long (was ~4), so the funnel risk for the tray-between-motors
  design got worse.
- **Pusher:** moved to the NXT motor lying on the deck. The crank is 5.6 cm (stroke 11.2 cm) so the carriage stays
  on the 15-hole rails pinned to the motors. The tray is 9.1 x 12.2 cm (~55 pom-poms in one layer). Robot is about
  21 cm tall.

## 2026-11-18 (late): roller is now direct drive (team change)

- Team info: the roller motor now drives the roller shaft directly, because the gears skipped teeth.
- Sim, Phase 2 outline, 2 seeds x 3 maps, noise 2:

  | Outline | Runs with contact | Min clearance |
  |---|---|---|
  | 25 cm square (motor within the width) | 0/6 | 0.89 cm |
  | 24 cm roller + NXT on the right end (reaches 16.6 cm from centre; the house is at 15.24) | 6/6 | hits |
  | 20 cm roller + NXT on the end, 25 cm total | 0/6 | 0.89 cm |

  The last case leaves a ~5 cm strip unswept on the motor side, since pom-poms lie up to 13.2 cm from the lane
  centre.
- Proposed fix: put the EV3 Medium motor inside the roller tube. Its output is on its end face, so it lines up with
  the roller shaft. This needs a tube inner diameter of about 5 cm or more (the motor is 2.4 x 3.1 cm, with a far
  corner 2.3 cm from the shaft). The NXT then does the dump or pusher. Waiting on the tube's measured inner
  diameter. The 3D model still shows the old gear drive.

## 2026-11-18 (later): 3D model of the tray-between-motors idea + review of the Nov 17 footage

- **`docs/robot_3d.html`** (new, 1.6 MB, works offline): a three.js scene built from real LDraw parts that match the
  kit checklist (EV3 Large x2, EV3 Medium, NXT, colour sensor, 43.2 mm wheels, Technic beams, 24/40-tooth gears).
  Cardboard parts are drawn to our sizes. Three scenes:
  - Robot, with an animated slider-crank stroke.
  - Street corner, the 14 cm arc with a live clearance readout.
  - Drop-off: reverse to the edge, two strokes, pom-poms fall into the bucket.
  - Still images are in `docs/renders/`; the source and build scripts are in `docs/robot3d_src/`.
- **Layout drawn:**
  - Track 21.6 cm, so the tray inside is 10.5 x 13.5 cm, enough for about 70 pom-poms in one layer.
  - The colour sensors move behind the motors, with their eyes 11.2 cm behind the axle.
  - Rear outline is 12.7 cm behind the axle (8.6 cm each side of centre).
  - Pusher: the medium motor turns a 9-hole crank with a 13-hole rod, for a 12.8 cm stroke.
  - The roller is driven by the NXT motor through 24-40-24 gears placed inside the left side plate.
- **Sim with this outline**, 6 seeds x 3 maps, noise 2: 0 contacts in 18 runs, minimum clearance 0.69 cm. The ideal
  corner path in the 3D scene leaves 1.5 cm.
- **Main risk:** the funnel narrows from 24 cm to 10.5 cm in about 4 cm. Test it with the existing roller and
  cardboard walls before building anything else.
- **Nov 17 footage** (`Nov17_progress/`):
  - The side-tipping tray (one motor) empties about 100 pom-poms in about 2 s, so the dump works.
  - The hand-turned flap roller throws pom-poms high over the back sheet, so it needs a closed hood.
  - The roller motor sits outside the roller's end, which adds width.

## 2026-11-18: team idea, tray between the drive motors with a rear pusher (not built, no code yet)

- Idea (user sketch): roller and ramp at the front; the tray sits low between the two drive motors; a plate pushes
  the pom-poms backward and out the rear into the bucket. This replaces the tipping tray (no 190-250 deg swing
  needed, and no clash between the tray and the wheels).
- Sim check, Phase 2 body as a full-width 25 cm box, 3 seeds x 3 maps, noise 2, current code:

  | Rear of body behind the axle | Contacts | Min clearance |
  |---|---|---|
  | 9 cm | 0 | 0.71 cm |
  | 13 cm | 0 | 0.17 cm |
  | 15 cm (1 run only) | 0 | 0.10 cm |
  | 17 cm | 9/9 runs | hits (in-place turns late in Phase 2) |
  | 21 cm | 9/9 runs | hits |

  So keep the rear within about 13 cm of the axle.
- Open questions:
  - The track is 12 cm now, which leaves only a few cm between the motors. Widen it to about 20-22 cm.
  - The colour sensors (behind the axle, 12 cm apart) sit where the tray would go; they need a new spot.
  - The rear needs a lip or flap so pom-poms don't spill while braking or reversing.
- Suggested pusher: a slider-crank (one turn = push out and come back). Other options: a rack and pinion, or a
  conveyor floor like lab 4.
- If the team builds it, the code changes are: reverse up to the edge for drop-off (`DUMP_POINT_FORWARD_CM`
  becomes negative), a push-stroke dump routine, a new outline, and new sensor constants.

## 2026-11-12 (end of day): bring-up tools, ultrasonic sideways fix off

- **`docs/BRINGUP.md`**: the order to bring the robot up (measure -> bench tests -> auto_tuner -> phases) and every
  constant to tune, with how to find each one.
- **`bench_tests.py`** (new, on the robot): 10 guided tests (colour threshold, drive direction, turret/US, turns,
  straight run, square loop, side-US accuracy, roller + catch, dump-arm jog, corner arc). Each prints the value to
  set. `sim/test_bench.py` runs all 10 in the simulator with scripted answers: all pass, 0 contacts.
- **`USE_US_LATERAL_FIX = False`** (was True). Sim, 6 seeds x 3 maps, noise 2: identical with and without it (maps
  18/18, Phase 1 home error mean 0.57 vs 0.59 cm). Real side readings near house corners/edges can be off, so the risk
  isn't worth it. Side glances are still used to *detect* openings. Turn the fix on only if bench test 7 shows < 1 cm.
- **`DRIVE_POLARITY`** constant (core) replaces the hard-coded minus signs; `auto_tuner.py` now imports the ports,
  polarity and `LINE_THRESHOLD` from `snowplow_core` instead of keeping its own copies.
- **`run_sim.py --set MOD.NAME=VALUE`** overrides any constant (MOD = core, p1, p2); the result line now shows
  Phase 1 min clearance.
- Regression (3 seeds x 3 maps, noise 2): all maps OK, 0 contacts, P2 min clearance 0.89 cm.
- **Open:** at noise 4 (wheel slip 1.6 %, colour noise sd 10, US sd 2.4 cm) mapping fails in 8/18 runs with or without
  the US fix. Probably false line hits from colour noise; worth a look once real sensor noise is measured (bench 1).

## 2026-11-12 (later still): web viewer on the Pi, late lane shift before corners

- **Viewer runs on the robot.** `visualizer.py --web [PORT]` serves the same view as a web page (stdlib HTTP
  server; the Python Scene draws into a command list that the page replays on a canvas). No screen, tkinter or
  laptop IP needed. The phase scripts start it automatically (`vizlog.AUTO_WEB_VIEWER`, niced, 6 fps, keeps
  running between phases) and print `[VIZ] live view: http://<robot-ip>:8080`. Telemetry now goes to
  127.0.0.1:5005 by default. `--web` also works with `--replay` (play/pause/speed/seek in the page) and
  `--sim`. Frame cost here 1.7 ms / 24 KB. Stop it with `pkill -f "visualizer.py --web"`.
  Env: `SNOWPLOW_VIZ=host:port` (send elsewhere) / `none`, `SNOWPLOW_WEBVIZ=0` (no auto-start).
- **Lane shift only near the corner.** The 1.24 cm shift toward the inside of the next arc used to apply to the
  whole run (visible as the robot sitting off-centre all along the street); now only the last 45 cm
  (`end_lane_offset_cm`). Sim: catch +1.3 points on every map (long 90.4 %, sample 91.9 %, u_reversed 92.4 %),
  0 contacts, min clearance 0.78 cm, same times.

## 2026-11-12 (late night): square intake + arc corners, 2 cm pom-pom rule

**Team info:** pom-poms are at least 2 cm from a house, so they lie up to ~13.2 cm from the lane centre and the
roller has to be full width (24 cm). **Correction:** the "funnel wings" in the first 3D model pointed the wrong
way (narrow in front, swept back) and would push pom-poms outward toward the houses. Dropped. The intake is now a
square 25 cm front (24 cm roller + side plates) reaching 11.6 cm.

### Changes
- **Arc corners in Phase 2** (`ARC_CORNERS`, `core.drive_arc`, `ARC_RADIUS_CM = 14`). A square 25 cm front sweeps a
  17 cm circle when spinning, more than the 15.24 cm to the houses. At street corners the run now stops on the
  corner tile's entry edge (1.24 cm toward the inside), then drives a 90 deg arc round the inside house corner (axle
  on a 14 cm radius, heading + radius feedback) and the next run starts on the exit edge.
- **Corner poke** (`CORNER_POKE_CM = 17`): before each arc, sweep straight into the corner tile and back to pick up
  its outer part, which the arc skips. Skipped when less than 8 s is spare.
- Sim: square footprint by default (`--intake chamfer` for the old shape), `--no-arcs`, pom-poms placed >= 2 cm
  from houses. `ROBOT_OUTLINE[2]` is the square shape.
- `docs/robot_model.html`: wings and chamfer removed; "At a street corner" shows the arc vs spinning in place with
  live clearance.

### Results (6 seeds x 3 maps, noise 2, calibrated; catch = sim's geometric catch, not real roller physics)
| Setup | Contacts | Min clearance | Catch long / sample / u_rev | P2 time long / sample / u_rev |
|---|---|---|---|---|
| square, spin in place | 18/18 runs | 0 | 91.6 / 92.8 / 92.9 % | 97-101 / 81-84 / 95-99 s |
| square, arcs | 0 | 0.63 cm | 85.5 / 88.3 / 87.4 % | 92-96 / 78-81 / 90-94 s |
| **square, arcs + poke (default)** | **0** | **0.63 cm** | **89.0 / 90.6 / 91.1 %** | **104-107 / 84-87 / 102-105 s** |
The model's ideal arc clearance is 1.5 cm; tracking errors take ~0.5-0.9 cm of it.

### Open (mechanical)
- **Tray vs wheels:** with the roller at 11.6 cm, only ~2 cm is left between its guide (~4.7 cm ahead of the axle)
  and the wheels (2.1 cm). A low 4.5 cm deep tray cannot fit. Options: raise the tray over the wheels/motors and
  release pom-poms near the top of the roller (brick moves back/up), or a U-tray around the wheels. Moving the
  roller forward kills the arc clearance (reach 14 -> ~0.5 cm).
- Remaining misses are mostly the ~1 cm strip next to each house that a 24 cm roller doesn't reach.

## 2026-11-12 (night): direct drop, nearest origin corner, sensors behind, tuner test, 3D model

### Changes
- **Direct drop-off** (`DROP_STYLES = ("direct", "along")`). The robot drives straight onto the drop tile facing
  the bucket, stops 3 cm past centre, turns 90 deg in place and reverses 6 cm along the edge. The planner picks the
  cheaper style, so the long map now goes exit (0,2) -> drop (-1,2) -> home with no detour to (-1,3), saving ~8 s.
  The in-place turn swings the corners ~1.5 cm over the bucket rim (the bucket is lower, so no contact).
- **Origin: nearest of the 4 tiles around it** (`ORIGIN_TILES`, `plan_home`, `choose_start_tile`) for leaving
  and returning, instead of always (0,0); leaving adds the cost of turns.
- **Colour sensors 5 cm behind the axle in both phases** (`SENSOR_OFFSET_BY_PHASE = {1: -5, 2: -5}`; the team
  agreed to build it that way).
- **Heading back-correction on snaps.** When two-sensor snaps measure a heading error, the lateral error it
  caused since the last turn or snap (capped at 60 cm of travel) is also undone. Uncalibrated contacts went
  from 3/18 to 2/18.
- **auto_tuner.py**: 3 spins, the width from each spin feeds the next, and only spins within 8 deg of 350 deg
  are averaged. `sim/test_auto_tuner.py` runs the real tuner in the sim: radius within 0.07% and track within
  0.5% over 6 seeds (one seed was off by 1.5% before the fix).
- **Sim defaults to a calibrated robot** (true values within ~0.1% radius and 0.5% track); `--uncalibrated`
  for the old behaviour.
- **`docs/robot_model.html`**: interactive 3D model of both phases (subsystem toggles, street/drop-off context,
  intake width, chamfer, reach and dump-arm sliders, live fit check). Open it in any browser.

### Results (calibrated, noise 2, 6 seeds per map)
| Map | Phase 1 | Phase 2 | Min clearance |
|---|---|---|---|
| long | 67.8-70.3 s (was 90.8) | 97.3-100.8 s (was 110.5) | 1.87 cm |
| sample | 63.2-65.4 s (was 70.5) | 80.8-83.6 s (was 88.3) | 1.86 cm |
| u_reversed | 82.5-85.7 s | 95.3-98.8 s | 1.87 cm |
All 18 OK, 0 contacts, 0 off-table. Uncalibrated: 2/18 runs touch a house, so **run auto_tuner.py first**.
Pour point: long (direct) ~1.7 cm along the edge from target; sample/u_reversed (along) within 0.5 cm.
Long map Phase 2 now sweeps all 12 tiles (no watchdog cut; was 10), then drop-off and home in ~99 s.

### Mechanical findings (from the model; to confirm on the build)
- A full 24 cm roller reaching 11.6 cm has a 16.7 cm turning radius and touched houses in 6/6 sim runs. The
  sim's shape (5 cm chamfers, 13.7 cm) needs the chamfers to be real: a ~14 cm roller plus angled card wings
  out to 24 cm, or a shorter roller.
- With a 14 cm roller the tray (13 x 6.9 cm for one layer of 50) overlaps the wheels and drive motors by
  ~4.9 cm: notch it, raise it or move the roller forward.
- The bucket is at table height, so the tray must swing past the table edge: ~190 deg (side, pivot 13 cm up on
  the edge side) or ~250 deg (rear overshot, robot backed up). `DUMP_TILT_DEG = 110` is a placeholder. The
  code's drop pose assumes a side pour; a single-sided arm means `DUMP_SIDES = ("RIGHT",)`.

### Next
- Build: decide the roller length/wings and the dump arm; measure `DUMP_TILT_DEG`, `DUMP_POINT_FORWARD_CM`,
  `EDGE_LANE_SHIFT_CM`, `SENSOR_SPACING`.
- On the robot: `python3 auto_tuner.py` (sensors ~3 cm before a line, 2 clear tiles ahead), then phase 1.

## 2026-11-12 (evening): drop-off lane fix, scan rays in the visualizer

- **Drop-off lane shift was incomplete on short final runs.** The 3 cm shift toward the edge was steered
  during the final run only; a 1-tile final run reached 1.5 of 3 cm and tipped while still angled 2.5 deg
  (long map). Now every run on the drop line uses the edge lane, and the run that turns onto that line stops
  already shifted. Sim, at the dump: within 0.5 cm of the target lane and 0.4 deg of the edge on all 3 maps.
- Visualizer draws each stationary L/F/R scan (2.5 s) and each side glance (1 s) as labelled rays
  (red = wall < 25 cm, green dashed = open). Before, scans were invisible, which looked like "no side check"
  at the street exit. Screenshot: `docs/visualizer_checks.png`.
- Regression batch (6 seeds x 3 maps, noise 2): 18/18 maps OK, 0 contacts, 0 off-table, times unchanged.
- Noticed: on the 12-tile `long` street the Phase 2 watchdog stops the sweep 2 tiles early (needs ~53 s for
  drop-off + home, actual ~49 s). Faster sweeping or a tighter estimate would recover them.

## 2026-11-12 (later): forward-watch mapping + visualizer (still untested on the real robot)

**Team info added:** no houses between origin and the gate, between the street end and the origin,
or between the street end and the bucket.

### Changes
- **Mapping = look forward, glance sideways** (`EXPLORE_MODE = "forward"`, default). During a run the turret
  points forward; a close reading (median of 3 < `FORWARD_STOP_CM` = 45 cm) stops the robot on the tile
  before the house (that is a corner). Once per tile, mid-tile, the turret flicks to one side (~0.5 s, while
  still driving) so the robot also stops where the side houses end (the street exit into open board, which a
  forward-only look can't see). At every stop: one F/L/R scan decides. Only close readings are trusted.
  The old side-watch mode is still there (`EXPLORE_MODE = "side"`).
- **Transit has no stop-and-look checks.** Phase 1 transit keeps the turret forward while driving
  (`OBSTACLE_MODE = "watch"`, no time cost) as a safety net, and confirms with one reading before marking a
  tile blocked. Phase 2 has no checks. `infer_houses()`: every tile touching the street (except approach and
  exit) is treated as a house when planning, in both phases.
- **Bug fixed: U-turns.** `turn_to` could exit immediately on a 180 deg turn (the wrapped error flips sign at
  +-180, which looked like an overshoot); the run then steered an arc off the table (sim: long map seed 4).
  Now it tracks the rotation made; `drive_run` re-turns if the heading is still >10 deg off.
- **Telemetry** (`vizlog.py`, stdlib): every phase writes `logs/phase{1,2}_<time>.jsonl` (pose 15 Hz,
  ultrasonic 8 Hz, scans, snaps, glances, plans, map, dump, console lines). Optional live UDP:
  `SNOWPLOW_VIZ=<laptop-ip>:5005` (or `vizlog.DEFAULT_UDP`). Never raises into robot code.
- **Visualizer** (`visualizer.py`, tkinter only): `--live` (UDP, saves `logs/live_*.jsonl`), `--replay
  logs/...jsonl` (play/pause/speed/slider), `--sim MAP [--seed N --speed 3]` (runs the sim and shows true pose,
  true houses, pom-poms), `--png out.png` (headless render, needs Pillow). Screenshots:
  `docs/visualizer_phase1.png`, `docs/visualizer_phase2.png`. The laptop needs `python3-tk` for the window
  (system python here lacks it; the uv python 3.10 has it).
- Sim: `--explore`, `--seed-start`, `--viz-udp`, `--viz-log`, `--realtime`; truth/pom-pom telemetry.

### Sim results (8 seeds per map, noise 2.0)
| Map | Mode | Phase 1 | Maps OK | Phase 2 | P2 min clearance | Contacts / off-table |
|---|---|---|---|---|---|---|
| sample | forward | 70.5-71.7 s | 8/8 | 88.2-89.8 s | 1.36 cm | 0 / 0 |
| u_reversed | forward | 90.5-91.7 s | 8/8 | 108.5-110.0 s | 0.72 cm | 0 / 0 |
| long | forward | 90.7-94.4 s | 8/8 | 105.5-110.7 s | 0.75 cm | 0 / 0 |
| sample | side | 69.5-72.5 s | 8/8 | 88.2-89.9 s | 1.33 cm | 0 / 0 |
| u_reversed | side | 90.7-91.9 s | 8/8 | 108.6-109.9 s | 0.64 cm | 0 / 0 |
| long | side | 93.5-96.2 s | 8/8 | 105.6-110.8 s | 0.60 cm | 0 / 0 |

Forward mode is about as fast (2 s faster on long streets) and never relies on long readings.
Phase 1 time is mostly driving: ~15 s to the gate, ~2.6 s per street tile, ~28 s scouting the drop-off + home.
`SCOUT_DROPOFF_ON_RETURN` could be turned off now that end->bucket is known to be clear (Phase 1 only).

### Risks to test on the robot
- The real ultrasonic beam is wider than the sim's +-7 deg: driving down a street, the forward watch may see a
  side house's front face as "close ahead". Effect: an extra stop + scan (the scan decides), not a wrong map.
  Watch for it in the visualizer (orange hit dots in front of the robot); raise/lower `FORWARD_STOP_CM`.
- Glance timing (`GLANCE_SEC_EST`, turret speed) and whether side readings at 90 deg are stable while moving.

### Next steps
1. Bench: turret zero/direction, then a straight run with `SNOWPLOW_VIZ` streaming to the laptop.
2. Same as below (measure constants, auto_tuner, roller catch rate, tray capacity).

## 2026-11-12: speed rework + laptop simulator (code untested on the real robot)

**Goal:** both phases reliably under 120 s, no house contact in Phase 2, before robot time.

### Confirmed rules (from the team, beyond the DRAFT handout)
Hardware swap between phases is allowed. Bucket is about 30 x 30 cm at table height on the map edge.
At most 16 pom-poms per tile (some tiles 0). No dead ends. 2 min per phase. Materials: kit + paper +
cardboard + tape. A floor-touching skirt is fine. Gyro drifts too much (not used). The requirements doc is
already submitted (the .md in this folder is an old version).

### What changed in the code
- `snowplow_core.py`: straight **runs** instead of tile-by-tile stops; **flying line snaps** (heading from the
  distance between left/right sensor hits, no pivoting); cross-track steering back to the lane centre; gated
  ultrasonic lateral correction; a path planner that also counts turns; obstacle checks with the turret
  *before* turning; only close readings mark tiles blocked; time watchdog plans the whole remaining mission.
  Bugs fixed: a `None` encoder read flipped the pose; the path search crashed with `StopIteration`;
  a long diagonal "step to origin" could cut through houses.
- `phase1_mapping.py`: one L/F/R scan per decision point, then a run as long as the forward reading allows
  (max 5 tiles) while the turret watches a side wall. Heads for whichever side of s1-s2 is cheaper to reach
  and senses which side is the street. The table edge counts as a wall. The exit tile is not part of the street.
  Drives through the drop-off on the way home so Phase 2 reuses a checked path. Prints an ASCII map.
- `phase2_snow_removal.py`: sweeps in runs; watchdog includes drive to drop-off + dump + home; drop-off =
  drive along the edge row into the drop tile shifted 3 cm toward the edge, stop with the pour point
  centred, tip the tray sideways (`DUMP_SIDES` = which way the tray can tip).
- `auto_tuner.py`: line 0 no longer counted as line 1 (old code doubled `WHEEL_RADIUS`: sim shows 4.23 vs
  true 2.11 cm); track width measured from one exact 360 turn (sim: within ~0.4%).
- `demo_runner.py`: separate diagnostics for Phase 1 and Phase 2 hardware; hardware-swap prompt.
- `utils/` copied from `lab4/utils` (the code imports it).
- Originals kept in `archive/v0-before-speed-rework/`.

### Simulator results (`python3 sim/run_sim.py --map all --seeds 5 --noise 2.0`)
| Map | Phase 1 | Map correct | Phase 2 | P2 min clearance | Contacts |
|---|---|---|---|---|---|
| sample (Figure 1, 7 tiles) | 73-74 s | 5/5 | 88-90 s | 1.5-2.1 cm | 0 |
| u_reversed (9 tiles, gate guess wrong) | 91-92 s | 5/5 | 108-110 s | 0.7-1.7 cm | 0 |
| long (12 tiles along the table edge) | 95-96 s | 5/5 | 105-111 s | 0.7-1.7 cm | 0 |

Geometric pom-pom coverage of a centred pass: ~75-85 %. The edges are missed, and the sim does not model
catch rate. The tightest clearances come from spinning in place with ~1 cm position error (roller v2
footprint radius ~13.7 cm in a 15.24 cm half-street).
The old code, by the same timing model: ~150 s Phase 1 and ~160 s Phase 2 on the sample map.

### MEASURE on the robot before trusting anything
`SENSOR_OFFSET`, `SENSOR_SPACING`, `US_FORWARD_OFFSET_CM`, `US_FACE_CM`, turret zero/direction, roller
direction sign, `DUMP_TILT_DEG` sign, `DUMP_POINT_FORWARD_CM`, `EDGE_LANE_SHIFT_CM` (wheel margin to the edge).
Then run `auto_tuner.py` and tune `TURN_DPS_MAX`/`KP_TURN` (overshoot), `EST_SEC_PER_*` from real logs.

### Next steps
1. Bench: roller v2 at 19/20 catches; pick `SWEEP_DPS` from that (time budget allows ~170 dps).
2. Measure the constants above; run `auto_tuner.py`; check one straight run and one 90 turn.
3. Real ultrasonic: compare side readings while driving against the sim (the user worries they're noisy).
   If bad, set `USE_US_LATERAL_FIX = False`; mapping still works from stationary scans.
4. Full runs on a cardboard-box street; log times; adjust `EST_SEC_PER_*`.
5. Tray capacity: up to 16/tile means 100+ pom-poms on a long street; a 50-pom-pom one-layer tray may fill.
