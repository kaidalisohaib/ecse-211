#!/usr/bin/env python3
"""
ECSE 211 Final Design Project - Core Navigation, Turret & Localization Engine
=============================================================================
Shared core library for Phase 1 (Mapping) and Phase 2 (Snow Removal).

Coordinate frame (same as labs 2-4):
  - x, y in cm, origin at grid intersection (0, 0); theta in degrees,
    0 = +Y (North), 90 = +X (East), 180 = -Y, 270 = -X. Positive turns are clockwise.
  - Tile (Tx, Ty) has its lower-left corner at intersection (Tx, Ty) and its
    centre at ((Tx + 0.5) * 30.48, (Ty + 0.5) * 30.48). The robot drives on tile centres.

Motion model (v1, "runs"):
  - A *run* is a straight line through several tiles, driven in one motion
    (`drive_run`). The robot only stops at the end of a run (corners, scan points,
    the drop-off), never at every tile.
  - Every grid line crossed during a run is used for a *flying snap*: the distance
    driven between the two colour sensors hitting the line gives the heading error,
    and the line position fixes the along-track coordinate. No stopping, no pivoting.
  - A cross-track controller steers back onto the lane (tile centre line + offset).
  - Optional ultrasonic side readings during a run correct the cross-track
    coordinate (wall-follower style, but as a gated odometry correction so a bad
    reading is ignored instead of steering the robot into a wall).

Everything marked MEASURE must be measured on the real robot; TUNE values need
real runs. The laptop simulator in sim/ runs this file unchanged.
"""
from __future__ import annotations

from collections import deque
import json
import math
import os
import statistics
import time
from utils.brick import EV3ColorSensor, EV3UltrasonicSensor, Motor, reset_brick
import vizlog

# ==================== HARDWARE PORT CONFIGURATION ====================
# Ports as wired on the robot (lab, 2026-11-22)
LEFT_DRIVE_PORT = "C"         # BrickPi port of the left wheel motor
RIGHT_DRIVE_PORT = "D"        # BrickPi port of the right wheel motor
TURRET_PORT = "B"             # Phase 1: ultrasonic turret (the belt's port: the belt motor isn't plugged in then)
DUMP_PORT = "A"               # Phase 2: bin tilt (dump)
ROLLER_MOTOR_PORT = "B"       # Phase 2: pinch belt (conveyor)
DRIVE_POLARITY = -1           # -1 = drive motors mounted backwards (positive dps drives in reverse). bench_tests [2]

LEFT_COLOR_PORT = 1           # Downward left colour sensor
RIGHT_COLOR_PORT = 2          # Downward right colour sensor
ULTRASONIC_PORT = 4           # Turret ultrasonic (Phase 1) / fixed ultrasonic (Phase 2, optional)

# ==================== GEOMETRY & CALIBRATION CONSTANTS ====================
TILE_SIZE = 30.48             # Size of one grid tile (cm), 12 in
WHEEL_RADIUS = 2.10           # Overridden by calibration_params.json
LEFT_WHEEL_SCALE = 1.0000     # Left wheel distance multiplier (1 = nominal; the auto-tuner writes it to calibration_params.json)
RIGHT_WHEEL_SCALE = 1.0000    # Right wheel distance multiplier (1 = nominal; the auto-tuner writes it to calibration_params.json)
TRACK_WIDTH = 13.45           # measured 2026-11-24: tyre centre-to-centre ((11.4 inner + 15.5 outer) / 2).
TRACK_WIDTH_CW = 13.45        # calibration_params.json overrides these three if present
TRACK_WIDTH_CCW = 13.45       # Track width used for counter-clockwise (left) turns (cm)

# Colour sensors, relative to the wheel-axle midpoint (cm).
# Colour sensors: distance ahead of the wheel axle, per phase (MEASURE). Negative = behind the axle,
# which is where the team is mounting them (behind the intake, on swept floor). The snap math works either way.
SENSOR_OFFSET_BY_PHASE = {1: -5.0, 2: -5.0}   # measured 2026-11-24: 5 cm behind the axle
SENSOR_OFFSET = SENSOR_OFFSET_BY_PHASE[1]   # set by init_hardware(phase)
SENSOR_SPACING = 14.0         # measured 2026-11-24: 14 cm centre-to-centre between the two colour sensors
SENSOR_SKEW_DEG = 0.0         # Colour sensor bar angle bias (deg): + = left sensor triggers ahead of right
                              # (fixes steady LEFT drift after snaps); - = right triggers ahead (fixes RIGHT drift)
LINE_THRESHOLD = 30           # Reflected red below this = black line
LINE_WINDOW_CM = 7.0          # Only look for a line within +-7.0 cm of where odometry expects it
LINE_SQUARE_MODE = "flying"   # "flying" = keep driving and estimate heading from the travel gap between hits (the
                              # last mode tested on the robot, 2026-11-24; sim ~4 s faster per phase than "stop");
                              # "stop" = stop when one colour sensor hits and pivot until the other sees the line
LINE_SQUARE_DPS = 65          # Wheel speed (dps) while pivoting to align the second sensor onto the line
LINE_SQUARE_TIMEOUT_SEC = 1.5 # Max time (s) spent searching for the second sensor during stop-and-square
MAX_SNAP_GAP_CM = 4.0         # Second sensor must hit within 4 cm of travel (= ~18 deg); else single-sensor snap
MAX_SNAP_HEADING_DEG = 18.0   # Reject heading snaps / stop-square pivots larger than this (pom-pom or seam, not a line).
                              # Must stay above the worst real error: an uncalibrated U-turn is easily 10 deg off.

# ==================== MOTION PROFILE & CONTROL GAINS (TUNE) ====================
CRUISE_DPS = 360              # Transit speed (~13.2 cm/s)
EXPLORE_DPS = 320             # Phase 1 runs inside the street
SWEEP_DPS = 170               # Phase 2 sweeping speed (roller catch rate decides this; bench-test it)
MIN_DRIVE_DPS = 50            # Launch / final creep speed
LINE_CROSS_MAX_DPS = 280      # Speed cap while a colour sensor is inside a line window (snap precision)
ACCEL_RAMP_SEC = 0.30         # Time (s) to ramp from standstill up to full speed at the start of a run
DECEL_DIST_CM = 7.0           # Decelerate over the last 7 cm of a run (only at run ends now)

TURN_DPS_MAX = 240            # Peak in-place turn wheel speed (~84 deg/s of robot rotation)
TURN_DPS_MIN = 40             # Slowest wheel speed (deg/s) in an in-place turn, so the end of the turn doesn't stall
KP_TURN = 5.0                 # wheel dps per degree of heading error
TURN_TOLERANCE_DEG = 0.8      # An in-place turn is done when the heading is within this many degrees
KP_DRIVE_HEADING = 2.5        # wheel dps of differential per degree of heading error
MAX_HEADING_CORR_DPS = 45     # Most wheel speed difference (deg/s) used to hold the heading while driving straight
K_LATERAL_DEG_PER_CM = 1.5    # cross-track: degrees of heading correction per cm off the lane
MAX_LATERAL_STEER_DEG = 6.0   # Most the robot steers (deg) to get back onto its lane

# ==================== ULTRASONIC & TURRET (MEASURE / TUNE) ====================
USE_TURRET = True             # Phase 1 hardware has the turret (TURRET_PORT)
TURRET_GEAR_RATIO = 1.0       # motor degrees per turret degree
TURRET_LEFT_DEG = -110.0      # Turret angle (deg) when looking left (replaces -90; tune for gear slack / cable)
TURRET_RIGHT_DEG = 110.0      # Turret angle (deg) when looking right (replaces +90)
TURRET_DPS = 400              # Turret motor speed (deg/s)
TURRET_SETTLE_TOL_DEG = 2.0   # The turret counts as arrived when within this many degrees of its target
TURRET_MOVE_TIMEOUT_SEC = 1.0 # Stop waiting for the turret after this long (s), even if it isn't there yet
GLANCE_WHILE_DRIVING = True   # side glances don't pause the drive loop (steering, odometry, line checks keep
                              # running while the turret swings out and back); False = old blocking glance
US_FORWARD_OFFSET_CM = 8.0    # MEASURE: turret axis (or fixed sensor) ahead of the wheel axle
US_FACE_CM = 2.5              # MEASURE: turret axis to sensor face (fixed side mount: centreline to face)
WALL_THRESHOLD_CM = 25.0      # Forward reading below this = house in the next tile
SIDE_WALL_THRESHOLD_CM = 22.0 # Side reading (glance or standing) below this = house beside the tile. A house beside
                              # reads ~13-17 cm (sim box houses: at most 19.6); at an exit, the corner of the last
                              # house (diagonally behind) can echo at ~24-26 cm, under the old shared 25 cm
US_SCAN_SAMPLES = 5           # stationary scans: median of 5 = right even if 2 of the 5 are bad
US_SCAN_EXTRA = 4             # ...and if those 5 disagree about wall/open (some below the house threshold, some
                              # above), take up to this many more before deciding (costs time only when unsure)
US_GLANCE_SAMPLES = 3         # side glance while driving: 3 so the median can outvote one bad reading
US_GLANCE_EXTRA = 2           # ...and up to this many more if those 3 disagree about house/open
US_FORWARD_WINDOW = 5         # forward watch: rolling median of the last N single readings
US_SPACING_SEC = 0.015        # MEASURE (bench [3]): time between samples; less than the sensor's own update
                              # period just reads the same ping twice
USE_US_LATERAL_FIX = False    # Correct sideways position from side-wall readings. Off: no gain in the sim, and
                              # real readings near house corners can be off. Turn on only if bench_tests [7] is < 1 cm.
USE_US_FORWARD_FIX = False    # Use forward wall reading at stops to correct along-track (needs offsets measured)
US_FIX_GATE_CM = 4.0          # Ignore US corrections that disagree with odometry by more than this
US_LATERAL_BLEND_STOPPED = 0.7# Side-wall position fix, standing: fraction of the measured error applied (0-1)
US_LATERAL_BLEND_MOVING = 0.15# Side-wall position fix, while driving: fraction applied per reading (0-1)
SIDE_SAMPLE_WINDOW = (-10.0, 4.0)  # Sample a tile's side wall while the sensor is this far (cm) from its centre
MAX_RUN_TILES = 5             # "side" explore mode: never trust one forward reading for more than this many tiles
FORWARD_STOP_CM = 45.0        # Forward watch: a reading below this means "house within the next tile or so";
                              # stop on the tile centre just before it. Only close readings are trusted.
GLANCE_SEC_EST = 0.6          # Rough duration of one side glance (turret out and back)
GLANCE_AT_CM = 4.0            # Side glance while driving: the turret axis is this far past the tile centre when it
                              # reads. The swing starts early by the distance the robot drives while the turret
                              # swings out (learned from the glances). It used to read ~11 cm past the centre: 2 cm
                              # from the ~5 cm gap between two 25.4 cm boxes (later still with a slower turret).
                              # Not 0: at an exit tile the last house's corner (behind) is then ~23 cm away, 26 at 4
# Robot outline for the visualizer, (lateral +right, forward) cm from the axle centre. Measure yours.
ROBOT_OUTLINE = {
    1: [(-8, -9), (8, -9), (8, 10.5), (-8, 10.5)],
    # Phase 2: pinch belt + V-funnel, inside a 15 cm circle, 24 cm wide (sim/world.py FOOTPRINT_P2_FUNNEL)
    2: [(12, 9), (12, -9), (11.98, -9.03), (11.15, -10.04), (10.23, -10.97), (9.23, -11.82), (8.17, -12.58),
        (7.04, -13.24), (5.86, -13.81), (4.64, -14.27), (3.37, -14.62), (2.09, -14.85), (0.79, -14.98),
        (-0.52, -14.99), (-1.83, -14.89), (-3.12, -14.67), (-4.39, -14.34), (-5.62, -13.91), (-6.81,
        -13.37), (-7.95, -12.72), (-9.03, -11.98), (-10.04, -11.15), (-10.97, -10.23), (-11.82, -9.23),
        (-12, -9), (-12, 9), (-2.25, 1), (2.25, 1)],
}
OBSTACLE_MODE = "watch"       # Phase 1 transit: "watch" = turret looks forward while driving (no stops),
                              # "precheck" = stop-and-look down each run before turning into it.
FIXED_US_ANGLE = None         # Phase 2: angle of a fixed ultrasonic (+90 right, -90 left), for the visualizer

# ==================== PHASE 2 INTAKE & DUMP SETTINGS ====================
ROLLER_SWEEP_POWER = -50      # Belt motor power (%) while sweeping: lab-tested 2026-11-22 (hardware tester "p -50").
                              # Sign = intake direction. None = run at ROLLER_SWEEP_DPS instead
ROLLER_SWEEP_DPS = 900        # only used when ROLLER_SWEEP_POWER is None
ROLLER_POWER_LIMIT = 85       # Not used by the code at the moment
DUMP_TILT_DEG = 140           # Motor A rotation that tips the bin (lab 2026-11-22: -140 empties it)
DUMP_PLUS = None              # Where the bin pours when motor A turns +: "RIGHT", "LEFT", "BACK" or None (bench test 9)
DUMP_MINUS = "LEFT"           # Where it pours when motor A turns - (user 2026-11-24: our robot always dumps on the left)
                              # Phase 2 parks for whichever of the two is quicker; set one to None to force the other.
DUMP_HOLD_SEC = 1.2           # Time (s) the bin stays tipped so the pom-poms fall out
DUMP_SHAKES = 2               # after tipping: back DUMP_SHAKE_DEG toward rest and down again, this many times
DUMP_SHAKE_DEG = 50           # (lab: +50 / -50 shakes the last pom-poms out)

# ==================== TIME BUDGET (TUNE with real runs) ====================
PHASE_TIME_LIMIT_SEC = 120.0  # Hard time limit of each phase (s)
TIME_SAFETY_MARGIN_SEC = 8.0  # The watchdog keeps this many seconds spare when deciding whether there is time left
EST_SEC_PER_TILE = 2.9        # Average transit time per tile inside a run
EST_SEC_PER_RUN = 1.6         # Extra per run (accel/decel)
EST_SEC_PER_TURN = 1.8        # Time estimate (s) for one 90 deg turn
EST_DUMP_SEC = 6.0            # Time estimate (s) for the whole dump (tip, shake, back)
EST_HOME_STEP_SEC = 5.0       # Tile (0,0) centre -> origin intersection

# Table size in tiles (tiles are named by their lower-left intersection). One table: 9 x 5 tiles, X -1..7,
# Y -1..3, origin one tile in from the lower-left corner. Two tables side by side: TILE_X_MAX = 16; one above
# the other: TILE_Y_MAX = 8. Keep the MINs at -1 (the origin stays at the lower-left).
TILE_X_MIN = -1                # one per line: the dashboard (and its Constants panel) only reads NAME = value
TILE_X_MAX = 7                # Rightmost tile column on the table
TILE_Y_MIN = -1               # Bottom tile row on the table
TILE_Y_MAX = 3                # Top tile row on the table

_HERE = os.path.dirname(os.path.abspath(__file__))
CALIBRATION_FILE = os.path.join(_HERE, "calibration_params.json")# Where the auto-tuner saves wheel radius, scales and track width
MAP_FILE = os.path.join(_HERE, "street_map.json")# Where Phase 1 saves the street map for Phase 2

# ==================== GLOBAL HARDWARE & ODOMETRY STATE ====================
left_motor = Motor(LEFT_DRIVE_PORT)
right_motor = Motor(RIGHT_DRIVE_PORT)
turret_motor = Motor(TURRET_PORT)
dump_motor = Motor(DUMP_PORT)
roller_motor = Motor(ROLLER_MOTOR_PORT)

left_color = EV3ColorSensor(LEFT_COLOR_PORT, mode="red")      # red mode from the start: get_red() would
right_color = EV3ColorSensor(RIGHT_COLOR_PORT, mode="red")    # otherwise switch modes (and wait) on the first read
us_sensor = EV3UltrasonicSensor(ULTRASONIC_PORT)

x = 0.0
y = 0.0
theta = 0.0
travel_cm = 0.0               # Cumulative signed distance driven by the axle midpoint (for snaps)
# Heading-error back-correction: where the current heading error most likely started (travel_cm), and
# how much of it to trust (1.0 after a turn, where the error appears all at once; 0.5 after a snap,
# where any new error built up gradually).
_hdg_ref_travel = 0.0
_hdg_ref_weight = 1.0
HEADING_BACKFIX_MAX_CM = 60.0 # When a grid line corrects the heading, the position is also corrected for at most this many cm driven

prev_left_deg = None
prev_right_deg = None
turret_present = False
turret_zero_enc = 0
turret_angle = 0.0            # Last commanded turret angle (deg, + = right)
glance_lead_cm = None         # cm driven between a side glance's swing start and the turret being ready to read

HEADING_TO_DELTA = {
    0: (0, 1),
    90: (1, 0),
    180: (0, -1),
    270: (-1, 0),
}


def load_calibration_params():
    """Loads auto-tuned physical constants from calibration_params.json if available."""
    global WHEEL_RADIUS, LEFT_WHEEL_SCALE, RIGHT_WHEEL_SCALE
    global TRACK_WIDTH, TRACK_WIDTH_CW, TRACK_WIDTH_CCW

    if not os.path.exists(CALIBRATION_FILE):
        print("[CALIB] No calibration_params.json found; using default constants.")
        return
    try:
        with open(CALIBRATION_FILE, "r") as f:
            data = json.load(f)
        WHEEL_RADIUS = float(data.get("WHEEL_RADIUS", WHEEL_RADIUS))
        LEFT_WHEEL_SCALE = float(data.get("LEFT_WHEEL_SCALE", LEFT_WHEEL_SCALE))
        RIGHT_WHEEL_SCALE = float(data.get("RIGHT_WHEEL_SCALE", RIGHT_WHEEL_SCALE))
        TRACK_WIDTH = float(data.get("TRACK_WIDTH", TRACK_WIDTH))
        TRACK_WIDTH_CW = float(data.get("TRACK_WIDTH_CW", TRACK_WIDTH_CW))
        TRACK_WIDTH_CCW = float(data.get("TRACK_WIDTH_CCW", TRACK_WIDTH_CCW))
        print(
            f"[CALIB] Loaded calibration: R={WHEEL_RADIUS:.4f}cm, "
            f"Scale=({LEFT_WHEEL_SCALE:.4f}, {RIGHT_WHEEL_SCALE:.4f}), "
            f"Track=(CW:{TRACK_WIDTH_CW:.3f}cm, CCW:{TRACK_WIDTH_CCW:.3f}cm)"
        )
    except Exception as e:
        print(f"[CALIB] Warning: Failed to load calibration file ({e}); using defaults.")


def _read_drive_encoders():
    """Returns (left_deg, right_deg) with drive polarity applied, or None if a read failed."""
    raw_l = left_motor.get_position()
    raw_r = right_motor.get_position()
    if raw_l is None or raw_r is None:
        return None
    return DRIVE_POLARITY * float(raw_l), DRIVE_POLARITY * float(raw_r)


class SensorMissing(RuntimeError):
    """A sensor the job needs sends no data (not plugged in, or the wrong port in the port settings)."""


SENSOR_READY_TIMEOUT_SEC = 5.0   # an EV3 sensor sends data ~0.5-2 s after it is set up; none by then = not plugged in


def wait_sensors_ready(need_colour: bool = True, need_us: bool = False, timeout: float = SENSOR_READY_TIMEOUT_SEC):
    """
    Waits (at most `timeout` s in all) for the colour sensors and the ultrasonic to send data, printing each
    port's result. A port that never does is named in the console instead of hanging the program (the course's
    wait_ready_sensors() waits forever, e.g. for an ultrasonic left unplugged with the Phase 2 hardware on).
    Raises SensorMissing if a sensor this job needs (need_colour / need_us) is missing.
    """
    ports = [(LEFT_COLOR_PORT, "left colour", left_color, need_colour),
             (RIGHT_COLOR_PORT, "right colour", right_color, need_colour),
             (ULTRASONIC_PORT, "ultrasonic", us_sensor, need_us)]
    t0 = time.time()
    status, ready_at = {}, {}
    while True:
        for port, _, sensor, _ in ports:
            if port in ready_at:
                continue
            try:
                status[port] = str(sensor.get_status())
            except Exception as e:      # SPI/brick error: report it, keep trying until the time is up
                status[port] = f"error {type(e).__name__}"
            if status[port] == "VALID_DATA":
                ready_at[port] = time.time() - t0
        if len(ready_at) == len(ports) or time.time() - t0 >= timeout:
            break
        time.sleep(0.02)
    missing = []
    for port, name, _, needed in ports:
        if port in ready_at:
            print(f"[INIT] S{port} {name}: ready ({ready_at[port]:.1f} s)")
        else:
            print(f"[INIT] S{port} {name}: NO DATA after {timeout:.0f} s (status {status.get(port)}): "
                  f"check the cable is in S{port}" + ("" if needed else "; not needed for this job, carrying on"))
            if needed:
                missing.append(f"S{port} {name}")
    if missing:
        raise SensorMissing(f"no data from {', '.join(missing)}: plug it in (or fix the port in snowplow_core.py)")


def init_hardware(phase: int = 1, strict: bool = True, need_us: bool | None = None):
    """
    Initializes sensors, latches encoders, loads calibration. phase=1 enables the turret (TURRET_PORT).
    strict: stop (SensorMissing) if a needed sensor sends nothing: the colour sensors, and the ultrasonic when
    need_us (default: Phase 1). strict=False only warns (small actions from the dashboard).
    """
    global prev_left_deg, prev_right_deg, turret_zero_enc, turret_present, turret_angle
    global x, y, theta, travel_cm, SENSOR_OFFSET
    SENSOR_OFFSET = SENSOR_OFFSET_BY_PHASE.get(phase, SENSOR_OFFSET)
    if need_us is None:
        need_us = phase == 1
    wait_sensors_ready(need_colour=strict, need_us=strict and need_us)
    load_calibration_params()

    enc = None
    for _ in range(20):
        enc = _read_drive_encoders()
        if enc is not None:
            break
        time.sleep(0.01)
    prev_left_deg, prev_right_deg = enc if enc is not None else (None, None)

    turret_present = bool(USE_TURRET and phase == 1)
    turret_zero_enc = turret_motor.get_position() or 0
    turret_angle = 0.0
    if turret_present:
        turret_motor.set_limits(power=80, dps=TURRET_DPS)

    x, y, theta, travel_cm = 0.0, 0.0, 0.0, 0.0
    print(f"[INIT] Phase {phase} hardware ready (turret={'on' if turret_present else 'off'}). "
          f"Pose (0, 0), heading 0 (+Y).")


# ==================== COORDINATE & GEOMETRY HELPERS ====================
def tile_to_center_cm(tx: int, ty: int) -> tuple[float, float]:
    return (tx + 0.5) * TILE_SIZE, (ty + 0.5) * TILE_SIZE


def cm_to_nearest_tile(x_cm: float, y_cm: float) -> tuple[int, int]:
    return int(math.floor(x_cm / TILE_SIZE)), int(math.floor(y_cm / TILE_SIZE))


def is_tile_in_bounds(tx: int, ty: int) -> bool:
    return (TILE_X_MIN <= tx <= TILE_X_MAX) and (TILE_Y_MIN <= ty <= TILE_Y_MAX)


def all_tiles() -> list[tuple[int, int]]:
    return [(x, y) for x in range(TILE_X_MIN, TILE_X_MAX + 1) for y in range(TILE_Y_MIN, TILE_Y_MAX + 1)]


def tile_step(tile: tuple[int, int], heading: int, n: int = 1) -> tuple[int, int]:
    dx, dy = HEADING_TO_DELTA[int(heading) % 360]
    return tile[0] + dx * n, tile[1] + dy * n


def heading_between_adjacent_tiles(t_from: tuple[int, int], t_to: tuple[int, int]) -> int:
    d = (t_to[0] - t_from[0], t_to[1] - t_from[1])
    for h, delta in HEADING_TO_DELTA.items():
        if delta == d:
            return h
    raise ValueError(f"Tiles {t_from} and {t_to} are not cardinally adjacent!")


def _along_of_xy(heading: int, px: float, py: float) -> float:
    """Coordinate that increases in the direction of `heading`."""
    heading = int(heading) % 360
    return {0: py, 90: px, 180: -py, 270: -px}[heading]


def get_along(heading: int) -> float:
    return _along_of_xy(heading, x, y)


def set_along(heading: int, value: float):
    global x, y
    heading = int(heading) % 360
    if heading == 0:
        y = value
    elif heading == 90:
        x = value
    elif heading == 180:
        y = -value
    else:
        x = -value


def get_lateral(heading: int) -> float:
    """Cross-track coordinate for `heading`, positive to the robot's right."""
    return get_along((int(heading) + 90) % 360)


def set_lateral(heading: int, value: float):
    set_along((int(heading) + 90) % 360, value)


def tile_center_along(tile: tuple[int, int], heading: int) -> float:
    return _along_of_xy(heading, *tile_to_center_cm(*tile))


def tile_center_lateral(tile: tuple[int, int], heading: int) -> float:
    return _along_of_xy((int(heading) + 90) % 360, *tile_to_center_cm(*tile))


def parse_gate_geometry(p1: tuple[int, int], p2: tuple[int, int]) -> dict:
    """
    Given two adjacent grid intersections p1, p2 (s1,s2 or d1,d2), returns the midpoint
    and the two tiles sharing that edge as {approach_tile, first_tile, heading} candidates.
    """
    x1, y1 = p1
    x2, y2 = p2
    mid = (0.5 * (x1 + x2) * TILE_SIZE, 0.5 * (y1 + y2) * TILE_SIZE)
    if x1 == x2:
        y_low = min(y1, y2)
        west_tile, east_tile = (x1 - 1, y_low), (x1, y_low)
        return {
            "orientation": "vertical",
            "mid_cm": mid,
            "candidates": [
                {"approach_tile": west_tile, "first_tile": east_tile, "heading": 90},
                {"approach_tile": east_tile, "first_tile": west_tile, "heading": 270},
            ],
        }
    if y1 == y2:
        x_low = min(x1, x2)
        south_tile, north_tile = (x_low, y1 - 1), (x_low, y1)
        return {
            "orientation": "horizontal",
            "mid_cm": mid,
            "candidates": [
                {"approach_tile": south_tile, "first_tile": north_tile, "heading": 0},
                {"approach_tile": north_tile, "first_tile": south_tile, "heading": 180},
            ],
        }
    raise ValueError(f"Gate points {p1} and {p2} must share an axis!")


# ==================== ODOMETRY & MOTOR CONTROL ====================
def minimal_angle_diff(target_deg: float, current_deg: float) -> float:
    """Signed minimal angle difference in [-180, 180). Positive = target is clockwise of current."""
    return ((target_deg - current_deg + 180.0) % 360.0) - 180.0


def update_odometer():
    """Updates (x, y, theta, travel_cm) from wheel encoder deltas. Skips a cycle on a failed read."""
    global x, y, theta, travel_cm, prev_left_deg, prev_right_deg

    enc = _read_drive_encoders()
    if enc is None:
        return
    curr_l, curr_r = enc
    if prev_left_deg is None or prev_right_deg is None:
        prev_left_deg, prev_right_deg = curr_l, curr_r
        return

    d_l = curr_l - prev_left_deg
    d_r = curr_r - prev_right_deg
    prev_left_deg, prev_right_deg = curr_l, curr_r

    dist_l = math.radians(d_l) * WHEEL_RADIUS * LEFT_WHEEL_SCALE
    dist_r = math.radians(d_r) * WHEEL_RADIUS * RIGHT_WHEEL_SCALE
    dist_c = 0.5 * (dist_l + dist_r)
    raw_diff = dist_l - dist_r

    # Direction-specific track widths only describe in-place spins (wheels opposite).
    if d_l * d_r < 0.0:
        eff_track = TRACK_WIDTH_CW if raw_diff >= 0.0 else TRACK_WIDTH_CCW
    else:
        eff_track = TRACK_WIDTH
    d_theta_rad = raw_diff / eff_track

    mid = math.radians(theta) + 0.5 * d_theta_rad
    x += dist_c * math.sin(mid)
    y += dist_c * math.cos(mid)
    theta = (theta + math.degrees(d_theta_rad)) % 360.0
    travel_cm += dist_c
    vizlog.pose(x, y, theta)


def drive(left_dps: float, right_dps: float):
    """Commands drive motors with inverted polarity matching physical wiring."""
    left_motor.set_dps(DRIVE_POLARITY * int(round(left_dps)))
    right_motor.set_dps(DRIVE_POLARITY * int(round(right_dps)))


def stop_drive_motors():
    left_motor.set_power(0)
    right_motor.set_power(0)


def stop_all_motors():
    left_motor.set_power(0)
    right_motor.set_power(0)
    dump_motor.set_power(0)
    turret_motor.set_power(0)
    roller_motor.set_power(0)


def drive_with_heading_hold(base_dps: float, target_heading: float):
    """Drives at base_dps while steering toward target_heading (P control)."""
    err = minimal_angle_diff(target_heading, theta)
    corr = max(-MAX_HEADING_CORR_DPS, min(MAX_HEADING_CORR_DPS, KP_DRIVE_HEADING * err))
    drive(base_dps + corr, base_dps - corr)


def turn_to(target_heading: float):
    """Rotates in place to target_heading with a soft start and proportional slowdown."""
    target_heading = target_heading % 360.0
    update_odometer()
    diff = minimal_angle_diff(target_heading, theta)
    if abs(diff) < TURN_TOLERANCE_DEG:
        return

    # Track the rotation actually made instead of re-wrapping the error: near 180 deg the wrapped
    # error flips sign on the slightest wobble, which used to look like an overshoot and end the turn.
    initial_dir = 1 if diff > 0 else -1
    goal = diff
    turned = 0.0
    prev = theta
    t_start = time.time()
    while True:
        update_odometer()
        turned += minimal_angle_diff(theta, prev)
        prev = theta
        diff = goal - turned
        if abs(diff) < TURN_TOLERANCE_DEG or (diff * initial_dir) < 0:
            break
        if time.time() - t_start > 6.0:
            print("  [TURN] timeout")
            break
        direction = 1 if diff > 0 else -1
        ramp = min(1.0, (time.time() - t_start) / 0.15)
        raw_speed = min(TURN_DPS_MAX, max(TURN_DPS_MIN, KP_TURN * abs(diff)))
        speed = max(TURN_DPS_MIN, raw_speed * ramp)
        drive(direction * speed, -direction * speed)
        time.sleep(0.008)

    drive(0, 0)
    time.sleep(0.08)
    update_odometer()
    stop_drive_motors()
    _mark_heading_reference(1.0)


def _mark_heading_reference(weight: float):
    global _hdg_ref_travel, _hdg_ref_weight
    _hdg_ref_travel, _hdg_ref_weight = travel_cm, weight


def compute_trapezoidal_dps(elapsed_sec: float, remaining_cm: float, ceiling_dps: float) -> float:
    accel = MIN_DRIVE_DPS + (ceiling_dps - MIN_DRIVE_DPS) * min(1.0, max(0.0, elapsed_sec) / ACCEL_RAMP_SEC)
    decel = MIN_DRIVE_DPS + (ceiling_dps - MIN_DRIVE_DPS) * min(1.0, max(0.0, remaining_cm) / DECEL_DIST_CM)
    return max(MIN_DRIVE_DPS, min(ceiling_dps, accel, decel))


def drive_straight_distance(distance_cm: float, forward: bool = True, hold_heading: float | None = None,
                            speed_dps: float = CRUISE_DPS):
    """Drives straight by distance_cm (no line snaps). Used for short off-grid moves."""
    if distance_cm <= 0.1:
        return
    update_odometer()
    start_travel = travel_cm
    lock_heading = theta if hold_heading is None else (hold_heading % 360.0)
    t_start = time.time()
    # Twice the time at the cruise speed, plus 3 s: wheels blocked or encoders not counting stop here, not never.
    t_limit = 3.0 + 2.0 * distance_cm / max(1.0, math.radians(speed_dps) * WHEEL_RADIUS)
    while True:
        update_odometer()
        remaining = distance_cm - abs(travel_cm - start_travel)
        if remaining <= 0.0:
            break
        if time.time() - t_start > t_limit:
            print(f"  [DRIVE] timeout after {t_limit:.1f} s: moved {distance_cm - remaining:.1f} of {distance_cm:.1f} cm "
                  "(wheels blocked, or the encoders aren't counting: check the drive ports)")
            break
        mag = compute_trapezoidal_dps(time.time() - t_start, remaining, ceiling_dps=speed_dps)
        drive_with_heading_hold(mag if forward else -mag, lock_heading)
        time.sleep(0.008)
    drive(0, 0)
    time.sleep(0.08)
    update_odometer()
    stop_drive_motors()


# ==================== ORIGIN <-> TILE (0,0) TRANSITION ====================
# The origin is a grid intersection shared by four tiles; leave and come back through whichever is best.
ORIGIN_TILES = [(0, 0), (-1, 0), (0, -1), (-1, -1)]# The four tiles around the origin (never houses)


def _origin_step_heading(tile) -> float:
    tx, ty = tile_to_center_cm(*tile)
    return math.degrees(math.atan2(tx, ty)) % 360.0


def plan_home(tile, blocked, allowed=None, start_heading=None, tight=None) -> list[tuple[int, int]]:
    """Cheapest path from `tile` to any of the four tiles around the origin (or [] if none)."""
    best, best_cost = [], None
    for t in ORIGIN_TILES:
        if t in blocked:
            continue
        p = plan_path(tile, t, blocked, allowed, start_heading, tight)
        if not p:
            continue
        cost = estimate_path_seconds(p, start_heading)
        if best_cost is None or cost < best_cost:
            best, best_cost = p, cost
    return best


def choose_start(goals, blocked, allowed=None, tight=None, keep_route=False):
    """(tile, first heading or None): which of the four origin tiles to start from, given where the robot is
    heading next, counting the way off the origin (arcs when the first run leads away from the origin, else the
    diagonal step). The diagonal step turns in place on the tile: one in `tight` (a house beside it) is a last
    resort then.
    keep_route (Phase 1): score routes as before the origin arcs (as if leaving on the diagonal). Phase 1 doesn't
    know where the street is; a route that is a little cheaper off the origin can run through it and reach the
    gate from inside (sim random-113: the gate check then reads the street as outside)."""
    best, best_cost = ((0, 0), None), None
    for t in ORIGIN_TILES:
        if t in blocked:
            continue
        for g in goals:
            if keep_route:
                diag = _origin_step_heading(t)
                p = plan_path(t, g, blocked, allowed, diag, tight)
                if p:
                    cost = abs(minimal_angle_diff(diag, 0.0)) / 90.0 * EST_SEC_PER_TURN + estimate_path_seconds(p, diag)
                    if best_cost is None or cost < best_cost:
                        best = (t, heading_between_adjacent_tiles(p[0], p[1]) if len(p) > 1 else None)
                        best_cost = cost
                continue
            p = plan_path(t, g, blocked, allowed, None, tight)
            if not p:
                continue
            h1 = heading_between_adjacent_tiles(p[0], p[1]) if len(p) > 1 else None
            dep = origin_departure_seconds(t, h1, room_ahead=True)     # p[1] is a known free tile
            arcs = ORIGIN_ARCS and h1 is not None and h1 in _origin_ways(t)[0]
            if tight and t in tight and not arcs:
                dep += 1000.0
            cost = dep + estimate_path_seconds(p, h1)
            if best_cost is None or cost < best_cost:
                best, best_cost = (t, h1), cost
    return best


def choose_start_tile(goals, blocked, allowed=None, tight=None) -> tuple[int, int]:
    return choose_start(goals, blocked, allowed, tight)[0]


def step_origin_to_start_tile(start_tile: tuple[int, int] = (0, 0), first_heading=None, room_ahead=False):
    """Moves from the origin intersection onto `start_tile`: on arcs if the first run (first_heading) leads away
    from the origin, else one diagonal step to the tile centre. room_ahead: the next tile along first_heading is
    known to be free (Phase 2), so the S-curve may run into it."""
    if leave_origin_on_arc(start_tile, first_heading, room_ahead):
        return
    tx, ty = tile_to_center_cm(*start_tile)
    update_odometer()
    dist = math.hypot(tx - x, ty - y)
    heading = math.degrees(math.atan2(tx - x, ty - y)) % 360.0
    print(f"[NAV] Origin -> Tile {start_tile} centre ({tx:.1f}, {ty:.1f} cm)")
    turn_to(heading)
    drive_straight_distance(dist, forward=True, hold_heading=heading, speed_dps=EXPLORE_DPS)


def step_tile_to_origin(face_north: bool = False):
    """
    Moves from the centre of one of the four origin tiles onto the origin intersection.
    Refuses long moves: a straight line from elsewhere could cut through houses.
    """
    if not face_north and reach_origin_on_arc():
        return
    update_odometer()
    dist = math.hypot(x, y)
    if dist > 30.0:
        print(f"[NAV] Not next to the origin ({dist:.0f} cm away); not taking a straight line through the board.")
        return
    if dist >= 0.5:
        heading = math.degrees(math.atan2(-x, -y)) % 360.0
        print(f"[NAV] Stepping onto Origin (dist={dist:.1f} cm @ {heading:.1f} deg)")
        turn_to(heading)
        drive_straight_distance(dist, forward=True, hold_heading=heading, speed_dps=EXPLORE_DPS)
    if face_north:
        turn_to(0.0)
    print(f"[NAV] At Origin: X={x:.2f} cm, Y={y:.2f} cm, theta={theta:.1f}")


# ==================== ULTRASONIC & TURRET SUBSYSTEM ====================
def get_us_cm(samples: int = 3, spacing_sec: float | None = None, extra: int = 0,
              threshold: float | None = None) -> float:
    """
    Median ultrasonic distance in cm (255.0 = no echo / nothing read).

    The median of n readings is right as long as fewer than half of them are bad (2 of 5), and against a
    threshold it is a majority vote. So every real answer votes, "no echo" (255 = nothing in range) included;
    only failed reads (exception, None, 0) are dropped. Dropping the 255s instead lets one ghost echo decide on
    an open side: of [255, 255, 16] only the 16 would be left (the sim's stress test found exactly that).
    Ties (even count) take the lower value: "house" is the safer call.
    If `threshold` is given and the readings disagree about which side of it they are on, up to `extra` more are
    taken before deciding.
    """
    spacing = US_SPACING_SEC if spacing_sec is None else spacing_sec
    readings = []

    def read_one():
        try:
            val = us_sensor.get_cm()
            if val is not None and val > 0.5:
                readings.append(min(255.0, float(val)))
        except Exception:
            pass

    for i in range(samples):
        read_one()
        if samples > 1 and i < samples - 1:
            time.sleep(spacing)
    if threshold is not None:
        for _ in range(extra):
            below = sum(r < threshold for r in readings)
            if below == 0 or below == len(readings):
                break                     # all agree (or nothing valid): no point asking again
            time.sleep(spacing)
            read_one()
    result = statistics.median_low(readings) if readings else 255.0
    vizlog.us(turret_angle if turret_present else FIXED_US_ANGLE, result, force=samples > 1)
    return result


def wall_threshold(direction) -> float:
    """House / open threshold (cm) for a reading toward `direction` ("L" / "F" / "R", or degrees from forward)."""
    if isinstance(direction, str):
        direction = {"L": -90.0, "F": 0.0, "R": 90.0}[direction]
    return WALL_THRESHOLD_CM if abs(float(direction)) < 45.0 else SIDE_WALL_THRESHOLD_CM


def glance_cm() -> float:
    """One side glance reading (turret already pointing sideways)."""
    return get_us_cm(samples=US_GLANCE_SAMPLES, spacing_sec=0.01, extra=US_GLANCE_EXTRA,
                     threshold=SIDE_WALL_THRESHOLD_CM)


def turret_target_for(rel_angle_deg: float) -> float:
    """Maps a logical scan direction (-90 left, 0 forward, +90 right) to the configured turret range."""
    if abs(rel_angle_deg) <= 1.0:
        return 0.0
    if abs(rel_angle_deg - 90.0) <= 1.0:
        return float(TURRET_RIGHT_DEG)
    if abs(rel_angle_deg + 90.0) <= 1.0:
        return float(TURRET_LEFT_DEG)
    lo, hi = min(TURRET_LEFT_DEG, TURRET_RIGHT_DEG), max(TURRET_LEFT_DEG, TURRET_RIGHT_DEG)
    return max(lo, min(hi, float(rel_angle_deg)))


def set_turret_angle(rel_angle_deg: float, wait: bool = True):
    """Points the turret within [TURRET_LEFT_DEG, TURRET_RIGHT_DEG] (0 = forward) and waits until it gets there."""
    global turret_angle
    if not turret_present:
        return
    lo, hi = min(TURRET_LEFT_DEG, TURRET_RIGHT_DEG), max(TURRET_LEFT_DEG, TURRET_RIGHT_DEG)
    angle = max(lo, min(hi, float(rel_angle_deg)))
    target_enc = int(round(turret_zero_enc + angle * TURRET_GEAR_RATIO))
    turret_motor.set_limits(power=80, dps=TURRET_DPS)
    turret_motor.set_position(target_enc)
    turret_angle = angle
    if not wait:
        return
    t0 = time.time()
    tol = TURRET_SETTLE_TOL_DEG * abs(TURRET_GEAR_RATIO)
    while time.time() - t0 < TURRET_MOVE_TIMEOUT_SEC:
        pos = turret_motor.get_position()
        if pos is not None and abs(pos - target_enc) <= tol:
            break
        time.sleep(0.01)
    time.sleep(0.04)  # let the sensor take a fresh ping at the new angle


def turret_at_target() -> bool:
    """True once the turret is within TURRET_SETTLE_TOL_DEG of the angle last set (for non-blocking moves)."""
    if not turret_present:
        return True
    pos = turret_motor.get_position()
    target = turret_zero_enc + turret_angle * TURRET_GEAR_RATIO
    return pos is not None and abs(pos - target) <= TURRET_SETTLE_TOL_DEG * abs(TURRET_GEAR_RATIO)


def check_direction_distance(rel_angle_deg: float, restore: bool = True) -> float:
    """
    Distance (cm) in direction rel_angle_deg (-90 left, 0 forward, +90 right).
    Turret if present (mapped to TURRET_LEFT_DEG / 0 / TURRET_RIGHT_DEG), else pivots the whole robot.
    """
    if turret_present:
        set_turret_angle(turret_target_for(rel_angle_deg))
        dist = get_us_cm(samples=US_SCAN_SAMPLES, extra=US_SCAN_EXTRA, threshold=wall_threshold(rel_angle_deg))
        if restore and abs(rel_angle_deg) > 1.0:
            set_turret_angle(0.0)
        return dist
    if abs(rel_angle_deg) < 1.0:
        return get_us_cm(samples=US_SCAN_SAMPLES, extra=US_SCAN_EXTRA, threshold=WALL_THRESHOLD_CM)
    saved = theta
    turn_to((saved + rel_angle_deg) % 360.0)
    dist = get_us_cm(samples=US_SCAN_SAMPLES, extra=US_SCAN_EXTRA, threshold=wall_threshold(rel_angle_deg))
    turn_to(saved)
    return dist


def free_tiles_ahead(dist_cm: float, rel_angle_deg: float = 0.0) -> int:
    """
    Free tiles in the direction the reading was taken (rel_angle_deg from the robot's
    heading: 0 = forward, +-90 = sideways), for a robot standing on a tile centre.
    """
    if dist_cm >= 254.0:
        return 99
    reach = dist_cm + US_FACE_CM + US_FORWARD_OFFSET_CM * math.cos(math.radians(rel_angle_deg))
    blocked_index = int(round(reach / TILE_SIZE + 0.5))   # 1 = adjacent tile is blocked
    return max(0, blocked_index - 1)


def _side_reading_to_lateral(side: int, dist_cm: float, heading_err_deg: float) -> float:
    """Converts a side reading into the robot's offset from the lane centre (cm, + = right)."""
    perp = dist_cm * math.cos(math.radians(heading_err_deg)) + US_FACE_CM
    if side < 0:   # left wall sits at -T/2
        return perp - TILE_SIZE / 2.0
    return TILE_SIZE / 2.0 - perp


def apply_us_lateral_fix(heading: int, tile: tuple[int, int], side: int, dist_cm: float, blend: float) -> bool:
    """Gated correction of the cross-track coordinate from one side wall reading."""
    if not USE_US_LATERAL_FIX or dist_cm >= SIDE_WALL_THRESHOLD_CM:
        return False
    err = minimal_angle_diff(theta, heading)
    measured = tile_center_lateral(tile, heading) + _side_reading_to_lateral(side, dist_cm, err)
    current = get_lateral(heading)
    if abs(measured - current) > US_FIX_GATE_CM:
        return False
    set_lateral(heading, current + blend * (measured - current))
    return True


def scan_walls(heading: int, tile: tuple[int, int]) -> dict:
    """
    Stationary L/F/R scan at a tile centre. Orders the turret moves to minimise travel
    and leaves the turret pointing at one side (the side monitored by the next run).
    Applies the gated lateral fix when both side walls are seen.
    Returns {"L": cm, "F": cm, "R": cm}.
    """
    if abs(turret_angle) < 45:
        order = [0.0, -90.0, 90.0] if turret_angle <= 0 else [0.0, 90.0, -90.0]
    else:
        order = [-90.0, 0.0, 90.0] if turret_angle < 0 else [90.0, 0.0, -90.0]
    names = {-90.0: "L", 0.0: "F", 90.0: "R"}
    out = {}
    for ang in order:
        out[names[ang]] = check_direction_distance(ang, restore=False)
    if out["L"] < SIDE_WALL_THRESHOLD_CM and out["R"] < SIDE_WALL_THRESHOLD_CM and USE_US_LATERAL_FIX:
        err = minimal_angle_diff(theta, heading)
        offset = 0.5 * (_side_reading_to_lateral(-1, out["L"], err) + _side_reading_to_lateral(1, out["R"], err))
        measured = tile_center_lateral(tile, heading) + offset
        current = get_lateral(heading)
        if abs(measured - current) <= US_FIX_GATE_CM:
            set_lateral(heading, current + US_LATERAL_BLEND_STOPPED * (measured - current))
    if USE_US_FORWARD_FIX and out["F"] < WALL_THRESHOLD_CM:
        face = tile_center_along(tile, heading) + TILE_SIZE / 2.0
        measured = face - (out["F"] + US_FORWARD_OFFSET_CM + US_FACE_CM)
        if abs(measured - get_along(heading)) <= US_FIX_GATE_CM:
            set_along(heading, measured)
    vizlog.emit("scan", tile=list(tile), h=heading, L=round(out["L"], 1), F=round(out["F"], 1), R=round(out["R"], 1))
    return out


# ==================== COLOUR SENSORS ====================
def get_color_red(sensor) -> float:
    try:
        val = sensor.get_red()
        return val if val is not None else 100
    except Exception:
        return 100


# ==================== RUNS: CONTINUOUS DRIVING WITH FLYING SNAPS ====================
def _emit_snap(heading: int, line_u: float, kind: str, e: float | None = None):
    axis = "y" if heading in (0, 180) else "x"
    world = line_u if heading in (0, 90) else -line_u
    vizlog.emit("snap", axis=axis, v=round(world, 2), how=kind, e=None if e is None else round(e, 2))


def _snap_heading_and_along(heading: int, line_u: float, first: str, gap_cm: float) -> bool:
    """Both sensors crossed line_u; first sensor 'L' or 'R', gap_cm of travel between the hits."""
    global theta, x, y
    e = math.degrees(math.atan2(gap_cm, SENSOR_SPACING))
    if first == "R":
        e = -e
    e -= SENSOR_SKEW_DEG
    if abs(e) > MAX_SNAP_HEADING_DEG:
        return False
    # The heading was off by `delta` since the last turn (or snap): odometry put every cm driven since
    # then in a slightly wrong direction. Move the pose by the difference (only the cross-track part
    # survives: the along-track part is overwritten by the line position just below).
    delta = minimal_angle_diff(heading + e, theta)
    d = max(0.0, min(HEADING_BACKFIX_MAX_CM, travel_cm - _hdg_ref_travel)) * _hdg_ref_weight
    tb, tt = math.radians(theta), math.radians(theta + delta)
    x += d * (math.sin(tt) - math.sin(tb))
    y += d * (math.cos(tt) - math.cos(tb))
    theta = (heading + e) % 360.0
    _mark_heading_reference(0.5)
    _emit_snap(heading, line_u, "both", e)
    er = math.radians(e)
    sigma_second = -1.0 if first == "L" else 1.0   # +1 = left sensor, -1 = right sensor
    lead = SENSOR_SPACING * math.tan(math.radians(SENSOR_SKEW_DEG))
    fwd_second = SENSOR_OFFSET + 0.5 * sigma_second * lead
    u_axle = line_u - fwd_second * math.cos(er) - sigma_second * (SENSOR_SPACING / 2.0) * math.sin(er)
    set_along(heading, u_axle)
    return True


def _snap_along_single(heading: int, line_u: float, which: str, travel_since_hit: float):
    """Only one sensor crossed: fix along-track with the current heading estimate."""
    er = math.radians(minimal_angle_diff(theta, heading))
    sigma = 1.0 if which == "L" else -1.0
    u_at_hit = line_u - SENSOR_OFFSET * math.cos(er) - sigma * (SENSOR_SPACING / 2.0) * math.sin(er)
    set_along(heading, u_at_hit + travel_since_hit * math.cos(er))
    _emit_snap(heading, line_u, "single")


def _stop_and_square_line(heading: int, line_u: float, hit_l: bool, hit_r: bool,
                          seen_at: float | None = None) -> bool:
    """
    Stops when one colour sensor detects a grid line and pivots the other wheel until the second sensor also
    detects it, which leaves the robot nearly square to `heading`, then snaps heading and along-track position.
    The first sensor coasts into the line while the robot brakes, and the second stops at the line's near edge,
    so the robot is not exactly square: the heading is taken from that coast (odometry), not assumed. Assuming
    square left every pivot ~1.5 deg off (0.36 cm on the 14 cm bar, sim) and Phase 2 drifted into houses.
    seen_at: travel_cm where the hit sensor(s) reached the line (default: now). A sensor checked every few
    millimetres at line-crossing speed is seen up to that late, and the angle comes from the difference.
    """
    global theta, x, y
    update_odometer()
    theta_pre = theta
    travel_pre = travel_cm
    sensors = {"L": left_color, "R": right_color}
    seen_at = travel_cm if seen_at is None else seen_at
    hit_at = {s: seen_at for s, h in (("L", hit_l), ("R", hit_r)) if h}   # travel when each sensor reached it

    def brake(sec):
        drive(0, 0)                            # active brake; note a sensor that reaches the line meanwhile
        t = time.time()
        look = travel_cm
        while time.time() - t < sec:
            update_odometer()
            for s in "LR":
                if s not in hit_at and get_color_red(sensors[s]) < LINE_THRESHOLD:
                    hit_at[s] = 0.5 * (look + travel_cm)
            look = travel_cm
            time.sleep(0.005)
        update_odometer()

    brake(0.08)

    # Only one sensor hit and it coasted past the thin line while braking: back both wheels just behind the line
    # and re-approach slowly (at LINE_SQUARE_DPS it coasts < 0.1 cm), so it stops on the leading edge.
    if len(hit_at) == 1:
        first = next(iter(hit_at))
        if get_color_red(sensors[first]) >= LINE_THRESHOLD:
            drive(-LINE_SQUARE_DPS, -LINE_SQUARE_DPS)
            t_back = time.time()
            tr0 = travel_cm
            saw_line = False
            while time.time() - t_back < 0.7 and abs(travel_cm - tr0) < 2.0:
                update_odometer()
                red = get_color_red(sensors[first])
                if red < LINE_THRESHOLD:
                    saw_line = True
                elif saw_line and red >= LINE_THRESHOLD + 5:
                    break
                time.sleep(0.005)
            drive(0, 0)
            time.sleep(0.03)
            update_odometer()
            hit_at.clear()
            drive(LINE_SQUARE_DPS, LINE_SQUARE_DPS)
            t_re = time.time()
            tr1 = travel_cm
            while time.time() - t_re < 0.9 and abs(travel_cm - tr1) < 2.5 and not hit_at:
                update_odometer()
                for s in "LR":
                    if get_color_red(sensors[s]) < LINE_THRESHOLD:
                        hit_at[s] = travel_cm
                time.sleep(0.005)
            brake(0.05)
            if not hit_at:
                return False

    # Both sensors crossed while the robot drove straight or braked: the travel between the hits gives the angle.
    if len(hit_at) == 2:
        first = "L" if hit_at["L"] <= hit_at["R"] else "R"
        t_second = max(hit_at.values())
        if _snap_heading_and_along(heading, line_u, first, abs(hit_at["L"] - hit_at["R"])):
            er = math.radians(minimal_angle_diff(theta, heading))
            set_along(heading, get_along(heading) + (travel_cm - t_second) * math.cos(er))
            return True
        _snap_along_single(heading, line_u, first, travel_cm - min(hit_at.values()))
        return False

    # One sensor on the line: hold its wheel and pivot the other forward until the second sensor sees it.
    first = next(iter(hit_at))
    second = "R" if first == "L" else "L"
    depth = max(0.0, travel_cm - hit_at[first])   # how far the first sensor went into the line (braking)
    if first == "L":
        drive(0, LINE_SQUARE_DPS)
    else:
        drive(LINE_SQUARE_DPS, 0)
    turned = 0.0
    prev_th = theta
    t0 = time.time()
    found = False
    theta_det = u_det = 0.0
    while time.time() - t0 < LINE_SQUARE_TIMEOUT_SEC:
        update_odometer()
        turned += minimal_angle_diff(theta, prev_th)
        prev_th = theta
        if get_color_red(sensors[second]) < LINE_THRESHOLD:
            found = True
            theta_det, u_det = theta, get_along(heading)
            break
        if abs(turned) > MAX_SNAP_HEADING_DEG:
            break
        time.sleep(0.005)
    drive(0, 0)
    time.sleep(0.04)
    update_odometer()

    if not found:
        # False trigger (e.g. pom-pom/seam) or missed line: undo the single-wheel pivot so heading isn't ruined.
        if abs(turned) > 0.5:
            if first == "L":
                drive(0, -LINE_SQUARE_DPS)
            else:
                drive(-LINE_SQUARE_DPS, 0)
            t_undo = time.time()
            undone = 0.0
            prev_th = theta
            while time.time() - t_undo < LINE_SQUARE_TIMEOUT_SEC:
                update_odometer()
                undone += minimal_angle_diff(theta, prev_th)
                prev_th = theta
                if abs(undone) >= abs(turned):
                    break
                time.sleep(0.005)
            drive(0, 0)
            time.sleep(0.04)
            update_odometer()
        return False

    # When the second sensor reached the line's near edge, the first sat `depth` cm into it: the sensor bar was
    # at e to the line (+ = left sensor ahead), less SENSOR_SKEW_DEG (left sensor mounted ahead by that angle).
    e = math.degrees(math.atan2(depth, SENSOR_SPACING)) * (1.0 if first == "L" else -1.0) - SENSOR_SKEW_DEG
    offset = minimal_angle_diff((heading + e) % 360.0, theta_det)   # odometry's heading error before this line
    if abs(offset) > MAX_SNAP_HEADING_DEG:
        return False
    er = math.radians(e)
    sigma_second = 1.0 if second == "L" else -1.0
    fwd_second = SENSOR_OFFSET + 0.5 * sigma_second * SENSOR_SPACING * math.tan(math.radians(SENSOR_SKEW_DEG))
    u_axle_det = line_u - fwd_second * math.cos(er) - sigma_second * (SENSOR_SPACING / 2.0) * math.sin(er)
    du = get_along(heading) - u_det                 # moved since the second hit (the pivot's coast)
    # Odometry tracked the pivot, so `offset` is the heading error that accumulated before this line:
    # back-correct cross-track drift just like flying snaps do.
    d = max(0.0, min(HEADING_BACKFIX_MAX_CM, travel_pre - _hdg_ref_travel)) * _hdg_ref_weight
    tb, tt = math.radians(theta_pre), math.radians(theta_pre + offset)
    x += d * (math.sin(tt) - math.sin(tb))
    y += d * (math.cos(tt) - math.cos(tb))
    theta = (theta + offset) % 360.0
    _mark_heading_reference(0.5)
    set_along(heading, u_axle_det + du)
    _emit_snap(heading, line_u, "square", -offset)
    return True


def compile_runs(path: list[tuple[int, int]]) -> list[tuple[int, int]]:
    """Turns a tile path into [(heading, n_tiles), ...] straight runs."""
    runs = []
    for a, b in zip(path, path[1:]):
        h = heading_between_adjacent_tiles(a, b)
        if runs and runs[-1][0] == h:
            runs[-1] = (h, runs[-1][1] + 1)
        else:
            runs.append((h, 1))
    return runs


def drive_run(
    start_tile: tuple[int, int],
    heading: int,
    n_tiles: int,
    speed_dps: float = CRUISE_DPS,
    lane_offset_cm: float = 0.0,
    end_along_offset_cm: float = 0.0,
    side_us: int | None = None,
    stop_on_side_open: bool = False,
    tile_hook=None,
    square_lines: bool = True,
    watch_forward: bool = False,
    side_glance: int | None = None,
    end_lane_offset_cm: float | None = None,
    end_lane_lead_cm: float = 45.0,
    lane_gain: float | None = None,
    max_steer_deg: float | None = None,
    approach_cm: float = 0.0,
    approach_dps: float | None = None,
) -> dict:
    """
    Drives n_tiles tiles from start_tile along heading in one motion and stops on the
    centre of the last tile (+ end_along_offset_cm).

    side_us: -1 / +1 = sample the ultrasonic on the left / right side during the run
             (turret pointed there in Phase 1, or a fixed side-mounted sensor).
             Readings feed the gated lateral fix and per-tile side-open detection.
    stop_on_side_open: stop at the first tile (index >= 1) whose monitored side is open.
    end_lane_offset_cm: switch to this lane offset for the last end_lane_lead_cm of the run
             (Phase 2 shifts toward the inside of the corner arc only there).
    lane_gain / max_steer_deg: stronger cross-track steering than K_LATERAL_DEG_PER_CM / MAX_LATERAL_STEER_DEG,
             for a short run that must reach its lane offset within a tile (Phase 2 drop-off approach).
    approach_dps: slow to this for the last approach_cm of the run (eased over DECEL_DIST_CM before it), and use
             lane_gain / max_steer_deg only there: a long drop-off run cruises until its last tile.
    tile_hook(idx, tile) -> bool: called as the axle passes each tile centre; return
             True to stop at the next tile centre (used by the time watchdog).

    watch_forward: (Phase 1 turret) keep the ultrasonic pointed forward and stop on the tile
             centre just before a house that shows up within FORWARD_STOP_CM.
    side_glance: -1 / +1 = while watching forward, flick the turret to that side once per tile
             (mid-tile, between grid lines) to notice where the side houses end.

    Returns {"tile", "tiles" (entered, in order), "side_open_idx", "side_status",
             "stop_reason" ("end", "wall", "side_open", "watchdog", "timeout"), "wall_ahead" (tile or None)}.
    """
    heading = int(heading) % 360
    n_tiles = max(0, int(n_tiles))
    tiles = [tile_step(start_tile, heading, k) for k in range(n_tiles + 1)]
    if n_tiles == 0:
        return {"tile": start_tile, "tiles": [], "side_open_idx": None, "side_status": {},
                "stop_reason": "end", "wall_ahead": None}

    turn_to(float(heading))
    if abs(minimal_angle_diff(heading, theta)) > 10.0:
        print(f"  [RUN] heading {theta:.1f} is far from {heading}; turning again")
        turn_to(float(heading))
    watch_forward = watch_forward and turret_present
    if watch_forward:
        set_turret_angle(0.0)
    elif side_us is not None and turret_present:
        set_turret_angle(turret_target_for(90.0 * side_us))
    vizlog.emit("run", tile=list(start_tile), h=heading, n=n_tiles)

    u0 = tile_center_along(start_tile, heading)
    u_goal = u0 + n_tiles * TILE_SIZE + end_along_offset_cm
    v_lane = tile_center_lateral(start_tile, heading) + lane_offset_cm
    lines = [u0 + (k + 0.5) * TILE_SIZE for k in range(n_tiles)]
    snapped = set()
    hits = {}                      # k -> {"L": travel_at_hit, "R": ...}
    side_samples = {}              # tile idx -> [cm, ...]
    side_status = {}               # tile idx -> "open" | "wall"
    side_open_idx = None
    hooked = 0
    stop_requested = False
    stop_reason = "end"
    wall_ahead = None
    fwd_hist = []
    fwd_t = 0.0
    glanced = set()
    glance = None                  # non-blocking glance in progress: {"j", "stage" ("out"/"back"), "t", "at", "u"}
    global glance_lead_cm
    if side_glance is not None and glance_lead_cm is None:
        # first glance: turret swing time (plus the settle and the readings) at line-crossing speed
        swing = abs(turret_target_for(90.0 * side_glance)) / TURRET_DPS + 0.1
        glance_lead_cm = math.radians(min(speed_dps, LINE_CROSS_MAX_DPS)) * WHEEL_RADIUS * swing

    def glance_result(j, d_side):
        nonlocal side_open_idx, u_goal, stop_reason
        side_tile = tile_step(tiles[j], (heading + 90 * side_glance) % 360)
        if not is_tile_in_bounds(*side_tile):
            side_status[j] = "edge"
            return
        side_status[j] = "open" if d_side >= SIDE_WALL_THRESHOLD_CM else "wall"
        vizlog.emit("glance", tile=list(tiles[j]), side=side_glance, d=round(d_side, 1),
                    open=side_status[j] == "open")
        apply_us_lateral_fix(heading, tiles[j], side_glance, d_side, 0.3)
        if side_status[j] == "open" and side_open_idx is None:
            side_open_idx = j
            if stop_on_side_open:
                u_goal = min(u_goal, u0 + j * TILE_SIZE + end_along_offset_cm)
                stop_reason = "side_open"

    update_odometer()
    t_start = time.time()
    t_ramp = t_start
    sq_look = None                            # (line, travel) at the last stop-mode look at the colour sensors
    while True:
        update_odometer()
        u = get_along(heading)
        remaining = u_goal - u
        if remaining <= 0.0:
            break
        if time.time() - t_start > 4.0 + n_tiles * 8.0:
            print("  [RUN] timeout")
            stop_reason = "timeout"
            break

        # ---- line correction (stop-and-square or flying snaps) ----
        in_window = False
        if square_lines:
            u_s = u + SENSOR_OFFSET
            for k, line_u in enumerate(lines):
                if k in snapped:
                    continue
                d = u_s - line_u
                if d < -LINE_WINDOW_CM:
                    break                      # lines are ordered; the rest are further ahead
                if LINE_SQUARE_MODE == "stop":
                    if abs(d) <= LINE_WINDOW_CM:
                        in_window = True
                        hit_l = get_color_red(left_color) < LINE_THRESHOLD
                        hit_r = get_color_red(right_color) < LINE_THRESHOLD
                        if hit_l or hit_r:
                            # the line was reached somewhere since the last look: midway, on average
                            seen_at = 0.5 * (sq_look[1] + travel_cm) if sq_look and sq_look[0] == k else travel_cm
                            _stop_and_square_line(heading, line_u, hit_l, hit_r, seen_at)
                            snapped.add(k)
                            t_ramp = time.time()
                            sq_look = None
                            break
                        sq_look = (k, travel_cm)
                    elif d > LINE_WINDOW_CM:
                        snapped.add(k)         # missed this line entirely; give up on it
                    continue
                h = hits.setdefault(k, {})
                if abs(d) <= LINE_WINDOW_CM:
                    in_window = True
                    if "L" not in h and get_color_red(left_color) < LINE_THRESHOLD:
                        h["L"] = travel_cm
                    if "R" not in h and get_color_red(right_color) < LINE_THRESHOLD:
                        h["R"] = travel_cm
                if "L" in h and "R" in h:
                    first = "L" if h["L"] <= h["R"] else "R"
                    gap = abs(h["L"] - h["R"])
                    if not _snap_heading_and_along(heading, line_u, first, gap):
                        _snap_along_single(heading, line_u, first, travel_cm - h[first])
                    snapped.add(k)
                elif h:
                    which = "L" if "L" in h else "R"
                    if travel_cm - h[which] > MAX_SNAP_GAP_CM or d > LINE_WINDOW_CM:
                        _snap_along_single(heading, line_u, which, travel_cm - h[which])
                        snapped.add(k)
                elif d > LINE_WINDOW_CM:
                    snapped.add(k)             # missed this line entirely; give up on it
            u = get_along(heading)
            remaining = u_goal - u

        # ---- ultrasonic side monitoring ----
        if side_us is not None:
            u_us = u + US_FORWARD_OFFSET_CM
            idx = int(round((u_us - u0) / TILE_SIZE))
            if 0 <= idx <= n_tiles:
                rel = u_us - (u0 + idx * TILE_SIZE)
                reading = get_us_cm(samples=1)
                if SIDE_SAMPLE_WINDOW[0] <= rel <= SIDE_SAMPLE_WINDOW[1]:
                    side_samples.setdefault(idx, []).append(reading)
                apply_us_lateral_fix(heading, tiles[idx], side_us, reading, US_LATERAL_BLEND_MOVING)
            for j, samples in side_samples.items():
                if j in side_status:
                    continue
                if u_us - (u0 + j * TILE_SIZE) > SIDE_SAMPLE_WINDOW[1] and samples:
                    side_tile = tile_step(tiles[j], (heading + 90 * side_us) % 360)
                    if not is_tile_in_bounds(*side_tile):
                        side_status[j] = "edge"      # table edge: nothing to see, not an opening
                        continue
                    side_status[j] = "open" if statistics.median(samples) >= SIDE_WALL_THRESHOLD_CM else "wall"
                    if side_status[j] == "open" and j >= 1 and side_open_idx is None:
                        side_open_idx = j
                        if stop_on_side_open:
                            u_goal = min(u_goal, u0 + j * TILE_SIZE + end_along_offset_cm)
                            stop_reason = "side_open"
            remaining = u_goal - u

        # ---- forward watch: stop before a close house ----
        if watch_forward:
            # Turret faces forward (not mid-glance); samples at least US_SPACING_SEC apart, so each is a new ping
            if glance is None and time.time() - fwd_t >= US_SPACING_SEC:
                fwd_t = time.time()
                fwd_hist.append(get_us_cm(samples=1))
                del fwd_hist[:-US_FORWARD_WINDOW]
            if glance is None and len(fwd_hist) == US_FORWARD_WINDOW:
                d = statistics.median(fwd_hist)
                if d < FORWARD_STOP_CM:
                    face_u = u + US_FORWARD_OFFSET_CM + US_FACE_CM + d
                    b = int(round((face_u - u0) / TILE_SIZE + 0.5))
                    stop_idx = max(0, min(n_tiles, b - 1))
                    new_goal = u0 + stop_idx * TILE_SIZE + end_along_offset_cm
                    if new_goal < u_goal - 0.5:
                        u_goal = new_goal
                        stop_reason = "wall"
                        wall_ahead = tile_step(start_tile, heading, b)
                        vizlog.emit("wall_ahead", tile=list(wall_ahead), d=round(d, 1))
                    remaining = u_goal - u

            # ---- side glance (once per tile, reading with the turret beside the middle of the side house) ----
            if side_glance is not None and glance is None:
                # axle position (from the tile centre) to start the swing at
                glance_start = GLANCE_AT_CM - US_FORWARD_OFFSET_CM - glance_lead_cm
                j = int(math.floor((u - u0 - glance_start) / TILE_SIZE))
                rel = u - (u0 + j * TILE_SIZE)
                # The blocking glance pauses the loop: not while a colour sensor waits for a line
                if (1 <= j <= n_tiles and j not in glanced and glance_start <= rel <= glance_start + 10.0
                        and u_goal > u0 + j * TILE_SIZE + 1.0 and (GLANCE_WHILE_DRIVING or not in_window)):
                    glanced.add(j)
                    if GLANCE_WHILE_DRIVING:
                        set_turret_angle(turret_target_for(90.0 * side_glance), wait=False)
                        glance = {"j": j, "stage": "out", "t": time.time(), "at": None, "u": u}
                    else:
                        set_turret_angle(turret_target_for(90.0 * side_glance))
                        d_side = glance_cm()
                        set_turret_angle(0.0)
                        fwd_hist.clear()
                        update_odometer()
                        glance_result(j, d_side)
                    u = get_along(heading)
                    remaining = u_goal - u
            elif glance is not None:
                arrived = turret_at_target() or time.time() - glance["t"] > TURRET_MOVE_TIMEOUT_SEC
                if arrived and glance["at"] is None:
                    glance["at"] = time.time()
                if glance["at"] is not None and time.time() - glance["at"] >= 0.04:   # fresh ping at the angle
                    if glance["stage"] == "out":
                        if "ready" not in glance:          # learn how far the robot drives while the turret swings
                            glance["ready"] = u
                            glance_lead_cm = 0.5 * glance_lead_cm + 0.5 * max(0.0, min(15.0, u - glance["u"]))
                        if not in_window:                  # the readings pause the loop ~50 ms: not while a
                            glance_result(glance["j"], glance_cm())  # colour sensor waits for a line
                            set_turret_angle(0.0, wait=False)
                            glance.update(stage="back", t=time.time(), at=None)
                    else:
                        glance = None
                        fwd_hist.clear()
                u = get_along(heading)
                remaining = u_goal - u

        # ---- per-tile hook (watchdog) ----
        while tile_hook is not None and hooked < n_tiles and u >= u0 + (hooked + 1) * TILE_SIZE:
            hooked += 1
            if not stop_requested and hooked < n_tiles and tile_hook(hooked, tiles[hooked]):
                stop_requested = True
                stop_reason = "watchdog"
                u_goal = min(u_goal, u0 + (hooked + 1) * TILE_SIZE)
                remaining = u_goal - u

        if remaining <= 0.0:
            break

        # ---- speed + cross-track steering ----
        ceiling = speed_dps
        if approach_dps is not None:
            ceiling = approach_dps + (speed_dps - approach_dps) * min(1.0, max(0.0, (remaining - approach_cm)
                                                                             / DECEL_DIST_CM))
        ceiling = min(ceiling, LINE_CROSS_MAX_DPS) if in_window else ceiling
        cmd = compute_trapezoidal_dps(time.time() - t_ramp, remaining, ceiling)
        lane = v_lane
        if end_lane_offset_cm is not None and remaining < end_lane_lead_cm:
            lane = tile_center_lateral(start_tile, heading) + end_lane_offset_cm
        v_err = get_lateral(heading) - lane
        strong = approach_dps is None or remaining < approach_cm
        cap = MAX_LATERAL_STEER_DEG if max_steer_deg is None or not strong else max_steer_deg
        gain = K_LATERAL_DEG_PER_CM if lane_gain is None or not strong else lane_gain
        steer = max(-cap, min(cap, -gain * v_err))
        drive_with_heading_hold(cmd, (heading + steer) % 360.0)
        time.sleep(0.006)

    stop_drive_motors()
    if glance is not None:                    # run ended mid-glance: finish it standing still
        if glance["stage"] == "out":
            set_turret_angle(turret_target_for(90.0 * side_glance))
            glance_result(glance["j"], glance_cm())
        set_turret_angle(0.0)
    time.sleep(0.05)
    update_odometer()
    if stop_reason == "wall" and get_along(heading) - u_goal > 1.0:
        # Already past the stop point (the run started beyond the tile centre, e.g. after the compact S-curve
        # from the origin): back onto it, so the next turn in place doesn't swing the body into the house.
        drive_straight_distance(get_along(heading) - u_goal, forward=False, hold_heading=heading, speed_dps=SWEEP_DPS)
        update_odometer()
    idx_final =int(round((get_along(heading) - end_along_offset_cm - u0) / TILE_SIZE))
    idx_final = max(0, min(n_tiles, idx_final))
    return {
        "tile": tiles[idx_final],
        "tiles": tiles[1:idx_final + 1],
        "side_open_idx": side_open_idx,
        "side_status": side_status,
        "stop_reason": stop_reason,
        "wall_ahead": wall_ahead,
    }


# ==================== CORNER ARCS (Phase 2) ====================
# Phase 2 can't turn in place next to a house, so it drives round the inside house corner on an arc. Slightly
# inside the tile centreline: the funnel's outer tip and round back swing outward during the turn. Funnel,
# ideal arc: clearance inside = radius - 12, outside 3.0 / 2.5 / 1.8 cm at 14 / 14.5 / 15.24; sim lowest 1.69 cm at 14.5.
ARC_RADIUS_CM = 14.5          # axle path radius around the inside house corner (15.24 = tile centreline)
ARC_K_RADIAL_DEG_PER_CM = 4.0 # heading correction per cm off the arc
ARC_MAX_CORR_DEG = 15.0       # Most heading correction (deg) used to stay on an arc
ARC_KP_HEADING = 4.0          # wheel dps of differential per degree of heading error on the arc


def arc_lane_offset(h_in: int, h_out: int, radius: float | None = None) -> float:
    """Lateral offset (+ = right) to hold on the run into a corner so the arc starts on its path."""
    radius = ARC_RADIUS_CM if radius is None else radius
    s = 1 if minimal_angle_diff(h_out, h_in) > 0 else -1
    return s * (TILE_SIZE / 2.0 - radius)


def drive_arc(corner_tile: tuple[int, int], h_in: int, h_out: int, speed_dps: float,
              radius: float | None = None) -> tuple[int, int]:
    """
    Drives a 90 deg arc through corner_tile, from its entry edge (heading h_in) to its exit edge
    (heading h_out), round the inside house corner. Starts and ends stopped on the tile edges.
    """
    radius = ARC_RADIUS_CM if radius is None else radius
    cx, cy = tile_to_center_cm(*corner_tile)
    half = TILE_SIZE / 2.0
    a, b = math.radians(h_in), math.radians(h_out)
    px = cx - half * math.sin(a) + half * math.sin(b)          # inside house corner
    py = cy - half * math.cos(a) + half * math.cos(b)
    vizlog.emit("arc", tile=list(corner_tile), h_in=int(h_in), h_out=int(h_out), r=radius,
                p=[round(px, 1), round(py, 1)])
    drive_arc_about(px, py, radius, h_in, h_out, speed_dps)
    return corner_tile


def drive_arc_about(px: float, py: float, radius: float, h_in: float, h_out: float, speed_dps: float,
                    then_cm: float = 0.0, t_start: float | None = None, stop: bool = True):
    """
    Follows a circle of `radius` round the pivot (px, py), turning from heading h_in to h_out (less than 180 deg
    either way), and ends where the heading reaches h_out: the radius to the pivot is then square to h_out.
    Closed loop on the distance to the pivot and the bearing from it (see the comment above ARC_RADIUS_CM).
    then_cm / t_start / stop: chain arcs without stopping in between (the speed ramp sees the rest of the path).
    """
    s = 1 if minimal_angle_diff(h_out, h_in) > 0 else -1      # +1 = right (clockwise)
    turn = abs(minimal_angle_diff(h_out, h_in))
    b = math.radians(h_out)
    ff = 0.5 * TRACK_WIDTH / radius                            # wheel speed split for the curvature
    update_odometer()
    t0 = time.time() if t_start is None else t_start
    t_limit = time.time() + 8.0
    while True:
        update_odometer()
        dx, dy = x - px, y - py
        along_out = dx * math.sin(b) + dy * math.cos(b)        # < 0 until the heading reaches h_out
        if along_out >= 0.0 or time.time() > t_limit:
            break
        phi = math.degrees(math.atan2(dx, dy))                 # compass bearing from the pivot to the robot
        r_err = math.hypot(dx, dy) - radius                    # + = too far out
        target = phi + 90.0 * s + s * max(-ARC_MAX_CORR_DEG, min(ARC_MAX_CORR_DEG, ARC_K_RADIAL_DEG_PER_CM * r_err))
        err = minimal_angle_diff(target % 360.0, theta)
        swept = abs(minimal_angle_diff(theta, h_in))
        remaining = radius * math.radians(max(0.0, turn - swept)) + then_cm
        base = compute_trapezoidal_dps(time.time() - t0, max(remaining, 1.0), speed_dps)
        corr = max(-MAX_HEADING_CORR_DPS, min(MAX_HEADING_CORR_DPS, ARC_KP_HEADING * err))
        drive(base * (1 + s * ff) + corr, base * (1 - s * ff) - corr)
        time.sleep(0.006)
    if stop:
        drive(0, 0)
        time.sleep(0.06)
        update_odometer()
        stop_drive_motors()
        _mark_heading_reference(1.0)


# ==================== ORIGIN ARCS ====================
# The origin is a grid corner, not a tile centre. Instead of the diagonal step (turn 45, crawl 21.6 cm, turn
# back: ~6 s), leave and come back on arcs of radius half a tile, round pivots inside the origin tiles (always
# free): the robot never turns in place on the start tile and never stops on its centre.
ORIGIN_ARCS = True            # False = the old diagonal step to / from the tile centre
ORIGIN_S_CURVE = False        # Phase 2, first run straight on from the origin: leave on the S-curve (two 60 deg arcs).
                              # Off (user 2026-11-25: on the robot it ended off its lane): turn in place on the
                              # origin and take the quarter arc onto the tile instead, as Phase 1 does


def _u(h):
    return math.sin(math.radians(h)), math.cos(math.radians(h))


def _origin_ways(tile):
    """(outward, inward): the two headings leading away from the origin through `tile`, and the two back."""
    tx, ty = tile_to_center_cm(*tile)
    out = {90 if tx > 0 else 270, 0 if ty > 0 else 180}
    return out, {(h + 180) % 360 for h in out}


def origin_departure_seconds(tile, first_heading, room_ahead=False) -> float:
    """Rough time from the origin onto `tile`, ready for a run heading first_heading (for choosing the tile)."""
    if ORIGIN_ARCS and first_heading is not None and int(first_heading) % 360 in _origin_ways(tile)[0]:
        if room_ahead and ORIGIN_S_CURVE and abs(minimal_angle_diff(first_heading, 0.0)) < 1:
            return 2.6                                         # S-curve, no turn
        other = (_origin_ways(tile)[0] - {int(first_heading) % 360}).pop()
        return 2.0 + abs(minimal_angle_diff(other, 0.0)) / 90.0 * EST_SEC_PER_TURN
    diag = _origin_step_heading(tile)
    turns = abs(minimal_angle_diff(diag, 0.0))
    if first_heading is not None:
        turns += abs(minimal_angle_diff(first_heading, diag))
    return 2.7 + turns / 90.0 * EST_SEC_PER_TURN


def leave_origin_on_arc(start_tile, first_heading, room_ahead=False, speed_dps=None) -> bool:
    """
    From the origin intersection onto start_tile's lane, heading first_heading, without stopping on the tile:
      - first run straight on, the next tile known free (room_ahead, Phase 2) and ORIGIN_S_CURVE: S-curve (two 60 deg arcs,
        radius 15.24) shifts half a tile sideways in 26.4 cm and ends on the lane, 11 cm past the tile centre (the
        front then reaches ~8 cm into the next tile, so Phase 1, which doesn't know that tile yet, can't use it);
      - otherwise, first run away from the origin: turn in place on the origin if needed, then a quarter arc that
        ends on the tile centre already heading first_heading. (A compact S-curve that stops short of the next
        tile was tried for Phase 1: its 9.4 cm arcs end 1.3 cm / 7 deg off, clearance 1.7 -> 0.5 cm, for ~1 s.)
    False (nothing done) if the first run heads back across the origin: use the diagonal step.
    """
    if not ORIGIN_ARCS or first_heading is None:
        return False
    update_odometer()
    if math.hypot(x, y) > 2.0:
        return False
    h1 = int(first_heading) % 360
    out = _origin_ways(start_tile)[0]
    if h1 not in out:
        return False
    hs = (out - {h1}).pop()                                    # the other way out: the side to shift to
    R = TILE_SIZE / 2.0
    speed = CRUISE_DPS if speed_dps is None else speed_dps
    print(f"[NAV] Origin -> Tile {start_tile} on arcs, leaving heading {h1}")
    if room_ahead and ORIGIN_S_CURVE and abs(minimal_angle_diff(h1, theta)) < 20.0:
        # Two arcs of radius r turning `ang` each way shift d = 2r(1 - cos ang) sideways over L = 2r sin ang.
        r, ang = R, 60.0                                       # d = 15.24, L = 26.4
        s = 1 if minimal_angle_diff(hs, h1) > 0 else -1        # first arc turns toward the side
        ux, uy = _u(h1 + 90 * s)
        p1 = (r * ux, r * uy)                                  # pivot of the first arc
        h_mid = (h1 + ang * s) % 360
        bx, by = _u(h_mid - 90 * s)
        e1 = (p1[0] + r * bx, p1[1] + r * by)                  # end of the first arc
        p2 = (e1[0] + r * bx, e1[1] + r * by)                  # pivot of the second arc (other side)
        arc_cm = r * math.radians(ang)
        t0 = time.time()
        drive_arc_about(p1[0], p1[1], r, h1, h_mid, speed, then_cm=arc_cm, t_start=t0, stop=False)
        drive_arc_about(p2[0], p2[1], r, h_mid, h1, speed, t_start=t0)
        return True
    turn_to(float(hs))
    s = 1 if minimal_angle_diff(h1, hs) > 0 else -1
    ux, uy = _u(hs + 90 * s)
    drive_arc_about(R * ux, R * uy, R, hs, h1, speed)
    return True


def reach_origin_on_arc(speed_dps=None) -> bool:
    """On an origin tile's centre facing toward the origin's side: one quarter arc ends on the intersection.
    False (nothing done) otherwise: use the diagonal step."""
    if not ORIGIN_ARCS:
        return False
    update_odometer()
    tile = cm_to_nearest_tile(x, y)
    if tile not in ORIGIN_TILES:
        return False
    cx, cy = tile_to_center_cm(*tile)
    h = int(round(theta / 90.0)) * 90 % 360
    inward = _origin_ways(tile)[1]
    if math.hypot(x - cx, y - cy) > 3.0 or h not in inward or abs(minimal_angle_diff(h, theta)) > 12.0:
        return False
    hs = (inward - {h}).pop()
    s = 1 if minimal_angle_diff(hs, h) > 0 else -1
    R = TILE_SIZE / 2.0
    ux, uy = _u(h + 90 * s)
    print(f"[NAV] Tile {tile} -> Origin on a quarter arc (heading {h} -> {hs})")
    vizlog.emit("arc", tile=list(tile), h_in=int(h), h_out=int(hs), r=R, p=[round(cx + R * ux, 1), round(cy + R * uy, 1)])
    drive_arc_about(cx + R * ux, cy + R * uy, R, h, hs, CRUISE_DPS if speed_dps is None else speed_dps)
    print(f"[NAV] At Origin: X={x:.2f} cm, Y={y:.2f} cm, theta={theta:.1f}")
    return True


def step_one_tile_center(current_tile: tuple[int, int], heading_deg: int, speed_dps: float = CRUISE_DPS,
                         enable_line_square: bool = True) -> tuple[int, int]:
    """Backward-compatible single-tile move (now just a 1-tile run)."""
    return drive_run(current_tile, heading_deg, 1, speed_dps, square_lines=enable_line_square)["tile"]


# ==================== BFS PATHFINDER & NAVIGATION ====================
def find_shortest_tile_path(
    start_tile: tuple[int, int],
    goal_tile: tuple[int, int],
    blocked_tiles: set[tuple[int, int]],
    allowed_tiles: set[tuple[int, int]] | None = None,
    start_heading: float | None = None,
    tight: set[tuple[int, int]] | None = None,
    tile_cost=None,
) -> list[tuple[int, int]]:
    """
    Fastest 4-connected tile path [start, ..., goal] (or [] if none). Costs tiles AND
    turns, so among equally short paths it picks the one with the fewest stops.
    tile_cost(tile) -> extra seconds for entering that tile (Phase 1: how likely an unknown tile hides a house).
    tight: tiles with a house directly beside them (Phase 2: no room to turn in place). The path never
    U-turns on one, and never turns on the start tile if it is one; a 90 deg turn on one is driven as an arc.
    """
    import heapq
    if start_tile == goal_tile:
        return [start_tile]

    def ok(t):
        if not is_tile_in_bounds(*t):
            return False
        if t == goal_tile:
            return True
        return t not in blocked_tiles and (allowed_tiles is None or t in allowed_tiles)

    turn_cost = EST_SEC_PER_TURN + EST_SEC_PER_RUN
    h0 = None if start_heading is None else int(round((start_heading % 360.0) / 90.0)) * 90 % 360
    start_key = (start_tile, h0)
    best = {start_key: 0.0}
    parent = {start_key: None}
    pq = [(0.0, 0, start_key)]
    count = 0
    while pq:
        cost, _, key = heapq.heappop(pq)
        tile, h = key
        if tile == goal_tile:
            path = []
            while key is not None:
                path.append(key[0])
                key = parent[key]
            return path[::-1]
        if cost > best.get(key, float("inf")):
            continue
        for nh, (dx, dy) in HEADING_TO_DELTA.items():
            nxt = (tile[0] + dx, tile[1] + dy)
            if not ok(nxt):
                continue
            step = EST_SEC_PER_TILE + (tile_cost(nxt) if tile_cost else 0.0)
            if tight and h is not None and nh != h and tile in tight and (
                    (nh - h) % 360 == 180 or tile == start_tile):
                continue
            if h is not None and nh != h:
                step += 2 * turn_cost if (nh - h) % 360 == 180 else turn_cost
            nkey = (nxt, nh)
            if cost + step < best.get(nkey, float("inf")):
                best[nkey] = cost + step
                parent[nkey] = key
                count += 1
                heapq.heappush(pq, (cost + step, count, nkey))
    return []


def plan_path(start, goal, blocked, allowed=None, start_heading=None, tight=None, tile_cost=None) -> list[tuple[int, int]]:
    """Path restricted to `allowed` if possible, otherwise through any unblocked tile. With `tight`, a path
    that respects it is preferred (see find_shortest_tile_path) over one that doesn't; known tiles come first
    (an unknown tile may hide a house)."""
    for area in ((allowed, None) if allowed is not None else (None,)):
        for t in ((tight, None) if tight else (None,)):
            path = find_shortest_tile_path(start, goal, blocked, area, start_heading, t, tile_cost)
            if path:
                return path
    return []


def estimate_path_seconds(path: list[tuple[int, int]], start_heading: float | None = None) -> float:
    if len(path) < 2:
        return 0.0
    runs = compile_runs(path)
    turns = len(runs) - 1
    if start_heading is not None and abs(minimal_angle_diff(runs[0][0], start_heading)) > 5:
        turns += 1
    return (len(path) - 1) * EST_SEC_PER_TILE + len(runs) * EST_SEC_PER_RUN + turns * EST_SEC_PER_TURN


def infer_houses(street, approach_tile=None, exit_tile=None) -> set[tuple[int, int]]:
    """
    Every on-table tile touching the street that is not street, approach, exit or one of the four
    origin tiles is a house: a street is one tile wide with houses on both sides, and the four
    tiles around the origin never hold houses.
    """
    street_set = {tuple(t) for t in street}
    keep = street_set | set(ORIGIN_TILES) | {tuple(t) for t in (approach_tile, exit_tile) if t is not None}
    houses = set()
    for t in street_set:
        for h in (0, 90, 180, 270):
            nb = tile_step(t, h)
            if nb not in keep and is_tile_in_bounds(*nb):
                houses.add(nb)
    return houses


def navigate_tile_with_obstacle_avoidance(
    current_tile: tuple[int, int],
    target_tile,
    blocked_tiles: set[tuple[int, int]],
    speed_dps: float = CRUISE_DPS,
    check_obstacles: bool = True,
    allowed_tiles: set[tuple[int, int]] | None = None,
    traversed: set[tuple[int, int]] | None = None,
) -> tuple[int, int]:
    """
    Drives to target_tile as a sequence of straight runs along tile centres.
    With check_obstacles (Phase 1 turret), the turret looks down each run *before* the
    robot turns into it. Only a house in the very next tile is recorded as blocked
    (close readings are reliable); a far reading just shortens the run so the robot
    re-checks from closer, because long readings in a corridor often hit a house corner.
    target_tile may be a list of tiles: the robot heads for whichever is cheapest to
    reach given what it knows so far, and stops at the first one it reaches.
    """
    targets = [tuple(target_tile)] if isinstance(target_tile[0], int) else [tuple(t) for t in target_tile]
    if traversed is not None:
        traversed.add(current_tile)
    watch = check_obstacles and turret_present and OBSTACLE_MODE == "watch"

    false_alarm_at = None
    for _ in range(40):
        if current_tile in targets:
            break
        update_odometer()
        path = []
        for goal in targets:
            p = plan_path(current_tile, goal, blocked_tiles, allowed_tiles, theta)
            if len(p) >= 2 and (not path or estimate_path_seconds(p, theta) < estimate_path_seconds(path, theta)):
                path = p
        if len(path) < 2:
            print(f"  [NAV] No path from {current_tile} to {targets}; stopping here.")
            break
        vizlog.emit("plan", path=vizlog.tiles(path))
        heading, n = compile_runs(path)[0]
        if check_obstacles and turret_present and not watch:
            rel = minimal_angle_diff(heading, theta)
            if abs(rel) > 95.0:
                turn_to(float(heading))
                rel = 0.0
            dist = check_direction_distance(rel, restore=False)
            free = free_tiles_ahead(dist, rel)
            if free == 0:
                blocked_tile = tile_step(current_tile, heading)
                if blocked_tile not in ORIGIN_TILES:
                    print(f"  [OBSTACLE] {dist:.0f} cm toward {heading} -> Tile {blocked_tile} blocked.")
                    blocked_tiles.add(blocked_tile)
                    if blocked_tile in targets:
                        targets.remove(blocked_tile)
                        if not targets:
                            print("  [NAV] Target tile is blocked; stopping here.")
                            break
                    continue
                n = min(n, 1)
            else:
                n = min(n, free)
        # Never drive more than 1 tile with watch_forward off: a stationary check only guarantees
        # the very next tile is clear (e.g. 38 cm ahead = 1 tile free, wall in the 2nd tile).
        skip_watch = (false_alarm_at == current_tile)
        if skip_watch:
            n = min(n, 1)
            false_alarm_at = None
        res = drive_run(current_tile, heading, n, speed_dps, watch_forward=watch and not skip_watch)
        if traversed is not None:
            traversed.update(res["tiles"])
        current_tile = res["tile"]
        if res["stop_reason"] == "wall":
            # Stopped one tile short of a close reading. Confirm from here (close readings are reliable).
            nxt = tile_step(current_tile, heading)
            dist = check_direction_distance(0.0, restore=False)
            free = free_tiles_ahead(dist, 0.0)
            if free == 0 and nxt not in ORIGIN_TILES:
                print(f"  [OBSTACLE] {dist:.0f} cm ahead -> Tile {nxt} blocked.")
                blocked_tiles.add(nxt)
                vizlog.emit("blocked", tile=list(nxt))
                if nxt in targets:
                    targets.remove(nxt)
                    if not targets:
                        print("  [NAV] Target tile is blocked; stopping here.")
                        break
            else:
                reason = "origin tile (never a house)" if nxt in ORIGIN_TILES else f"{free} tile(s) clear ahead"
                print(f"  [NAV] Forward reading {dist:.0f} cm at {current_tile} ({reason}): stepping 1 tile to re-check.")
                false_alarm_at = current_tile
    return current_tile


def time_remaining(phase_start_time: float) -> float:
    return PHASE_TIME_LIMIT_SEC - (time.time() - phase_start_time)


def should_abort_for_time(phase_start_time: float, seconds_needed: float) -> bool:
    """True when finishing the plan from here would not leave TIME_SAFETY_MARGIN_SEC spare."""
    remaining = time_remaining(phase_start_time)
    if remaining <= seconds_needed + TIME_SAFETY_MARGIN_SEC:
        print(f"\n[WATCHDOG] {remaining:.1f}s left, ~{seconds_needed:.1f}s needed to finish -> wrapping up.")
        return True
    return False


# ==================== DROP-OFF PLANNING ====================
DUMP_DIRECTIONS = ("RIGHT", "LEFT", "BACK")# Names the code accepts for where the bin pours


POUR_HOUSE_MARGIN_CM = 1.0    # a pour shifted toward a house along the edge stops with the robot this far from its line


def pour_shift(dropoff_tile, final_heading, blocked, pour_fwd_cm) -> float:
    """Side pour: where the axle stops along final_heading, in cm from the drop-off tile centre, so the pour
    opening (pour_fwd_cm ahead of the axle, - = behind) is level with the bucket centre: -pour_fwd_cm. Cut short
    when the tile it reaches into along the edge holds a house (the robot's front / rear stays
    POUR_HOUSE_MARGIN_CM from that tile's line) or, behind, is off the table. Off the table ahead is fine: only
    the front overhangs the side edge, the wheels stay on."""
    want = -float(pour_fwd_cm)
    if abs(want) < 0.5:
        return want
    sign = 1 if want > 0 else -1
    t = tile_step(dropoff_tile, final_heading, sign)
    if not is_tile_in_bounds(*t) and sign > 0:
        return want
    if is_tile_in_bounds(*t) and t not in blocked:
        return want
    fwd = [p[1] for p in ROBOT_OUTLINE[2]]
    reach = max(fwd) if sign > 0 else -min(fwd)
    return sign * min(abs(want), max(0.0, TILE_SIZE / 2.0 - reach - POUR_HOUSE_MARGIN_CM))


def plan_dump_approach(dropoff_tile, outward_heading, blocked, sides=("RIGHT", "LEFT"),
                       styles=("direct", "along"), pour_fwd_cm=0.0, max_run_tiles=1) -> list[dict]:
    """
    Ways to end up on dropoff_tile with the pouring side toward the bucket (outward_heading points at it).
    RIGHT / LEFT (tray tips sideways, robot ends parallel to the table edge):
      "direct": drive straight in toward the bucket from the tile next to it, stop, turn 90 deg in place
                to be parallel, pour. Short, but the in-place turn swings the corners ~1.5 cm over the
                table edge, and needs the neighbouring tile along the edge to be free.
      "along":  drive into the drop tile along the edge (from a neighbouring edge tile), already parallel.
      "turn":   drive in along the edge the other way (from the next edge tile on the far side), turn 180 deg
                in place, pour. For a robot that pours one side only when the "along" tile is off the table or
                a house.
      All put the pour opening (pour_fwd_cm ahead of the axle) level with the tile centre: the axle stops
      "shift" cm past the centre along final_heading (pour_shift: cut short next to a house, which costs a
      penalty of 3 s per cm the pour ends off the bucket centre).
      max_run_tiles 2: "along" / "turn" also as a 2-tile last run along the edge ("via" = the tile between
      pre_tile and the drop-off): the edge lane is reached before a turn in place, and a U-turn needed to line
      up happens one tile further out.
    BACK (tray tips over the rear, robot ends with its back to the bucket, then reverses to the edge):
      "direct": drive in toward the bucket, turn 180 deg in place.
      "along":  drive in along the edge (either way), turn 90 deg in place.
    Each option: {"style", "side", "heading" (last run), "final_heading" (when pouring), "pre_tile",
    "tilt_sign" (+1 right, -1 left, 0 back), "turns" (90 deg turns in place on the drop tile)}.
    """
    options = []
    o = int(outward_heading) % 360
    for side in sides:
        side = str(side).upper()
        if side == "BACK":
            final = (o + 180) % 360
            if "direct" in styles:
                pre = tile_step(dropoff_tile, o, -1)
                if is_tile_in_bounds(*pre) and pre not in blocked:
                    options.append({"style": "direct", "side": side, "heading": o, "final_heading": final,
                                    "pre_tile": pre, "tilt_sign": 0, "turns": 2})
            if "along" in styles:
                for h_par in ((o - 90) % 360, (o + 90) % 360):
                    pre = tile_step(dropoff_tile, h_par, -1)
                    if is_tile_in_bounds(*pre) and pre not in blocked:
                        options.append({"style": "along", "side": side, "heading": h_par, "final_heading": final,
                                        "pre_tile": pre, "tilt_sign": 0, "turns": 1})
            continue
        if side not in ("RIGHT", "LEFT"):
            print(f"[DUMP] unknown dump direction {side!r} (use RIGHT, LEFT or BACK), ignored")
            continue
        h_par = (o - 90) % 360 if side == "RIGHT" else (o + 90) % 360
        tilt = 1 if side == "RIGHT" else -1
        shift = pour_shift(dropoff_tile, h_par, blocked, pour_fwd_cm)
        common = {"side": side, "final_heading": h_par, "tilt_sign": tilt, "shift": shift,
                  "penalty": 3.0 * abs(shift + pour_fwd_cm)}
        for k in range(1, max(1, max_run_tiles) + 1):
            for style, sign, heading, turns in (("along", -1, h_par, 0), ("turn", 1, (h_par + 180) % 360, 2)):
                if style not in styles:
                    continue
                run = [tile_step(dropoff_tile, h_par, sign * j) for j in range(k, 0, -1)]
                if all(is_tile_in_bounds(*t) and t not in blocked for t in run):
                    options.append(dict(common, style=style, heading=heading, pre_tile=run[0], via=run[1:],
                                        turns=turns, run_tiles=k))
        if "direct" in styles:
            pre = tile_step(dropoff_tile, o, -1)
            if is_tile_in_bounds(*pre) and pre not in blocked:
                options.append(dict(common, style="direct", heading=o, pre_tile=pre, turns=1))
    return options


def dump_direction(d) -> str:
    """"R" / "right" -> "RIGHT", "L" -> "LEFT", "B" -> "BACK"."""
    d = str(d).strip().upper()
    for name in DUMP_DIRECTIONS:
        if d and name.startswith(d):
            return name
    raise ValueError(f"dump direction {d!r}: use RIGHT, LEFT or BACK")


def dump_directions() -> list[str]:
    """Directions the tray can pour, from DUMP_PLUS / DUMP_MINUS (motor + first)."""
    out = []
    for d in (DUMP_PLUS, DUMP_MINUS):
        if d is not None and dump_direction(d) not in out:
            out.append(dump_direction(d))
    return out


def dump_sign(direction) -> int:
    """Motor A sign for a pour direction ("+" / "-" pass straight through)."""
    if direction in ("+", "-"):
        return 1 if direction == "+" else -1
    d = dump_direction(direction)
    for sign, setting in ((1, DUMP_PLUS), (-1, DUMP_MINUS)):
        if setting is not None and dump_direction(setting) == d:
            return sign
    raise ValueError(f"the tray can't pour {d} (DUMP_PLUS = {DUMP_PLUS!r}, DUMP_MINUS = {DUMP_MINUS!r})")


# ==================== PHASE 2 INTAKE & DUMP ACTUATORS ====================
def start_intake_roller():
    if ROLLER_SWEEP_POWER:
        print(f"[INTAKE] Belt on (port {ROLLER_MOTOR_PORT}, power {ROLLER_SWEEP_POWER}).")
        roller_motor.set_power(ROLLER_SWEEP_POWER)
    else:
        print(f"[INTAKE] Belt on (port {ROLLER_MOTOR_PORT}, {ROLLER_SWEEP_DPS} dps).")
        roller_motor.set_dps(ROLLER_SWEEP_DPS)


def stop_intake_roller():
    roller_motor.set_power(0)


def _move_aux_and_wait(target_enc: int, timeout: float = 1.5):
    dump_motor.set_position(int(target_enc))
    t0 = time.time()
    while time.time() - t0 < timeout:
        pos = dump_motor.get_position()
        if pos is not None and abs(pos - target_enc) <= 4:
            break
        time.sleep(0.01)


def dump_snow_payload(direction="+"):
    """Turns motor A by DUMP_TILT_DEG to tip the tray, shakes it, and brings it back.
    direction: "+" / "-" (motor sign), or where to pour ("RIGHT" / "LEFT" / "BACK", via DUMP_PLUS / DUMP_MINUS)."""
    sign = dump_sign(direction)
    where = DUMP_PLUS if sign > 0 else DUMP_MINUS
    if where is None:
        # The bin can't tip that way (e.g. + would swing it forward into the belt): don't move.
        print(f"[DUMP] {'DUMP_PLUS' if sign > 0 else 'DUMP_MINUS'} is None: the bin can't tip that way; not moving.")
        return
    d = dump_direction(where)
    vizlog.emit("dump", side={"RIGHT": 1, "LEFT": -1, "BACK": 0}[d], dir=d)
    print(f"[DUMP] Motor {DUMP_PORT} {sign * DUMP_TILT_DEG:+.0f} deg: bin pours {d.lower()}...")
    base = dump_motor.get_position() or 0
    dump_motor.set_limits(power=85, dps=300)
    full = base + sign * DUMP_TILT_DEG
    _move_aux_and_wait(full)
    for _ in range(DUMP_SHAKES):
        _move_aux_and_wait(full - sign * DUMP_SHAKE_DEG, 0.4)
        _move_aux_and_wait(full, 0.4)
    time.sleep(DUMP_HOLD_SEC)
    _move_aux_and_wait(base)
    dump_motor.set_power(0)
    print("[DUMP] Done.")
