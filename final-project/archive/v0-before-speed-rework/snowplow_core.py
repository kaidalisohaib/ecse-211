#!/usr/bin/env python3
"""
ECSE 211 Final Design Project - Core Navigation, Turret & Localization Engine
=============================================================================
Shared core library for Phase 1 (Mapping) and Phase 2 (Snow Removal).

Key Capabilities:
  1. Loads auto-tuned parameters from `calibration_params.json` (if present).
  2. Tile-Center Coordinate Frame:
     - Grid intersections span X in [-1..8], Y in [-1..4] (tile size = 30.48 cm).
     - Each tile (Tx, Ty) has its lower-left corner at intersection (Tx, Ty)
       and its physical center at ((Tx + 0.5) * 30.48, (Ty + 0.5) * 30.48).
     - Driving along tile centers keeps the robot 15.24 cm away from house walls
       and crosses perpendicular black grid lines every 30.48 cm for auto-squaring.
  3. Ultrasonic Turret Controller (Port A) for fast [-90°, 0°, +90°] wall checks.
  4. Guarded Dual Color-Sensor Line Squaring (Ports 4 & 2) with spatial windowing
     so dark pom-poms or mid-tile debris never cause false alignments.
  5. Obstacle-Aware BFS Tile Pathfinder & 120-Second Overtime Watchdog.
"""

from collections import deque
import json
import math
import os
import statistics
import time
from utils.brick import EV3ColorSensor, EV3UltrasonicSensor, Motor, reset_brick, wait_ready_sensors

# ==================== HARDWARE PORT CONFIGURATION ====================
LEFT_DRIVE_PORT = "B"
RIGHT_DRIVE_PORT = "C"
TURRET_OR_DUMP_PORT = "A"     # Phase 1: Ultrasonic Turret | Phase 2: Dump Gate/Pusher
ROLLER_MOTOR_PORT = "D"       # Phase 2: Front Intake Sweeper Roller

LEFT_COLOR_PORT = 4           # Downward left color sensor
RIGHT_COLOR_PORT = 2          # Downward right color sensor
ULTRASONIC_PORT = 3           # Forward / turret-mounted ultrasonic sensor

# ==================== GEOMETRY & CALIBRATION CONSTANTS ====================
TILE_SIZE = 30.48             # Grid square width/length in cm (1 ft)
WHEEL_RADIUS = 2.10           # Default wheel radius in cm (overridden by calibration_params.json)
LEFT_WHEEL_SCALE = 1.0000
RIGHT_WHEEL_SCALE = 1.0000
TRACK_WIDTH = 12.00           # Nominal wheel-to-wheel track width in cm
TRACK_WIDTH_CW = 12.00
TRACK_WIDTH_CCW = 12.00

# Color sensor mounting offset relative to wheel axle midpoint (cm):
# Positive = in front of wheel axle; Negative = behind wheel axle.
SENSOR_OFFSET = 7.0
LINE_THRESHOLD = 30           # Reflected red intensity threshold for black grid lines
LINE_WINDOW_CM = 5.5          # Spatial gate (+-5.5 cm around true grid lines) to reject pom-pom triggers
MAX_ALIGN_SWEEP_DEG = 15.0    # Max pivot angle during line squaring
MAX_ALIGN_TIME_SEC = 1.0      # Timeout guard on line squaring

# ==================== MOTION PROFILE & CONTROL GAINS ====================
CRUISE_DPS = 260              # Fast transit speed on open/known segments (deg/s)
EXPLORE_DPS = 200             # Phase 1 corridor exploration speed (deg/s)
SWEEP_DPS = 140               # Phase 2 snow sweeping speed (deg/s) for high roller catch rate
MIN_DRIVE_DPS = 45            # Minimum crawl speed (deg/s)
LINE_APPROACH_DPS = 75        # Speed when crossing expected grid line for squaring
ALIGN_DPS = 45                # Single-wheel pivot speed during line squaring

ACCEL_RAMP_SEC = 0.25         # Soft acceleration ramp time (s)
DECEL_DIST_CM = 8.0           # Deceleration distance before stopping target (cm)
LINE_SLOWDOWN_DIST_CM = 5.5   # Pre-line slowdown distance (cm)

TURN_DPS_MAX = 110            # Peak in-place turn speed (deg/s)
TURN_DPS_MIN = 32             # Minimum crawl turn speed (deg/s)
KP_TURN = 2.8                 # Proportional turn gain
KP_DRIVE_HEADING = 2.2        # Active yaw heading-hold gain while driving straight
MAX_HEADING_CORR_DPS = 28     # Maximum differential steering correction (deg/s)

# ==================== ULTRASONIC & TURRET SETTINGS ====================
USE_TURRET = True             # True = rotate turret motor on Port A; False = pivot robot to check sides
TURRET_GEAR_RATIO = 1.0       # Ratio from motor degrees to turret degrees (1.0 if direct drive)
TURRET_DPS = 300              # Turret rotation speed (deg/s)
WALL_THRESHOLD_CM = 25.0      # Distance (cm) below which a tile face is considered blocked by a house wall
CORNER_STOP_DIST_CM = 18.0    # Forward ultrasonic distance (cm) at which robot stops in corner tile

# ==================== PHASE 2 INTAKE & DUMP SETTINGS ====================
ROLLER_SWEEP_DPS = 500        # Roller motor speed (deg/s) during snow sweeping (~83 RPM, or gear up)
ROLLER_POWER_LIMIT = 85
DUMP_GATE_OPEN_DEG = 110      # Motor A rotation (deg) to open rear trapdoor / push ejector paddle
DUMP_DURATION_SEC = 2.5       # Time (s) to hold gate open / eject snow at drop-off edge

# ==================== TIME WATCHDOG & BOARD BOUNDS ====================
PHASE_TIME_LIMIT_SEC = 120.0  # Strict 2-minute hard cap per phase
TIME_SAFETY_MARGIN_SEC = 14.0 # Trigger emergency return when remaining time <= return_est + 14s

# Tile index bounds: intersections span X in [-1..8], Y in [-1..4],
# so the 9x5 grid of tiles has lower-left corner indices:
TILE_X_MIN, TILE_X_MAX = -1, 7
TILE_Y_MIN, TILE_Y_MAX = -1, 3

CALIBRATION_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibration_params.json")
MAP_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "street_map.json")

# ==================== GLOBAL HARDWARE & ODOMETRY STATE ====================
left_motor = Motor(LEFT_DRIVE_PORT)
right_motor = Motor(RIGHT_DRIVE_PORT)
aux_motor_a = Motor(TURRET_OR_DUMP_PORT)
roller_motor = Motor(ROLLER_MOTOR_PORT)

left_color = EV3ColorSensor(LEFT_COLOR_PORT)
right_color = EV3ColorSensor(RIGHT_COLOR_PORT)
us_sensor = EV3UltrasonicSensor(ULTRASONIC_PORT)

# Pose in cm and degrees: Origin (0, 0) at intersection (0, 0), theta = 0 deg along +Y (North)
x = 0.0
y = 0.0
theta = 0.0

prev_left_deg = 0.0
prev_right_deg = 0.0
turret_zero_enc = 0
ignore_red_counter = 0

# Heading constants (0=+Y North, 90=+X East, 180=-Y South, 270=-X West)
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


def init_hardware():
    """Initializes sensors, latches motor encoders, and loads calibration parameters."""
    global prev_left_deg, prev_right_deg, turret_zero_enc, x, y, theta
    wait_ready_sensors(True)
    load_calibration_params()

    prev_left_deg = -(left_motor.get_position() or 0.0)
    prev_right_deg = -(right_motor.get_position() or 0.0)
    turret_zero_enc = aux_motor_a.get_position() or 0
    aux_motor_a.set_limits(power=80, dps=TURRET_DPS)
    roller_motor.set_limits(power=ROLLER_POWER_LIMIT, dps=ROLLER_SWEEP_DPS)

    x, y, theta = 0.0, 0.0, 0.0
    print("[INIT] Hardware & odometry initialized at Origin (0.0, 0.0), heading=0.0° (+Y).")


# ==================== COORDINATE & GEOMETRY HELPERS ====================
def tile_to_center_cm(tx: int, ty: int) -> tuple[float, float]:
    """Converts tile index (Tx, Ty) to real-world center coordinates (X_cm, Y_cm)."""
    return (tx + 0.5) * TILE_SIZE, (ty + 0.5) * TILE_SIZE


def cm_to_nearest_tile(x_cm: float, y_cm: float) -> tuple[int, int]:
    """Converts real-world coordinates (X_cm, Y_cm) to containing tile index (Tx, Ty)."""
    tx = int(math.floor(x_cm / TILE_SIZE))
    ty = int(math.floor(y_cm / TILE_SIZE))
    return tx, ty


def is_tile_in_bounds(tx: int, ty: int) -> bool:
    """Returns True if tile (Tx, Ty) lies within the 9x5 table grid."""
    return (TILE_X_MIN <= tx <= TILE_X_MAX) and (TILE_Y_MIN <= ty <= TILE_Y_MAX)


def parse_gate_geometry(
    p1: tuple[int, int], p2: tuple[int, int]
) -> dict:
    """
    Given two adjacent grid intersections p1=(x1,y1) and p2=(x2,y2) (such as s1,s2 or d1,d2),
    computes the midpoint in cm and the two tiles sharing that boundary edge.
    """
    x1, y1 = p1
    x2, y2 = p2
    mid_x_cm = 0.5 * (x1 + x2) * TILE_SIZE
    mid_y_cm = 0.5 * (y1 + y2) * TILE_SIZE

    if x1 == x2:
        # Vertical gate segment along X = x1, spanning Y in [min_y, max_y]
        y_low = min(y1, y2)
        west_tile = (x1 - 1, y_low)
        east_tile = (x1, y_low)
        return {
            "orientation": "vertical",
            "mid_cm": (mid_x_cm, mid_y_cm),
            "candidates": [
                {"approach_tile": west_tile, "first_tile": east_tile, "heading": 90},
                {"approach_tile": east_tile, "first_tile": west_tile, "heading": 270},
            ],
        }
    elif y1 == y2:
        # Horizontal gate segment along Y = y1, spanning X in [min_x, max_x]
        x_low = min(x1, x2)
        south_tile = (x_low, y1 - 1)
        north_tile = (x_low, y1)
        return {
            "orientation": "horizontal",
            "mid_cm": (mid_x_cm, mid_y_cm),
            "candidates": [
                {"approach_tile": south_tile, "first_tile": north_tile, "heading": 0},
                {"approach_tile": north_tile, "first_tile": south_tile, "heading": 180},
            ],
        }
    else:
        raise ValueError(f"Gate points {p1} and {p2} must share an axis!")


# ==================== ODOMETRY & MOTOR CONTROL ====================
def minimal_angle_diff(target_deg: float, current_deg: float) -> float:
    """Signed minimal angle difference in [-180.0, 180.0]. Positive = CW, Negative = CCW."""
    return ((target_deg - current_deg + 180.0) % 360.0) - 180.0


def update_odometer():
    """Updates global pose (x, y, theta) from wheel encoder deltas."""
    global x, y, theta, prev_left_deg, prev_right_deg

    curr_left_deg = -(left_motor.get_position() or prev_left_deg)
    curr_right_deg = -(right_motor.get_position() or prev_right_deg)

    d_left_deg = curr_left_deg - prev_left_deg
    d_right_deg = curr_right_deg - prev_right_deg
    prev_left_deg = curr_left_deg
    prev_right_deg = curr_right_deg

    dist_l = math.radians(d_left_deg) * WHEEL_RADIUS * LEFT_WHEEL_SCALE
    dist_r = math.radians(d_right_deg) * WHEEL_RADIUS * RIGHT_WHEEL_SCALE

    dist_c = 0.5 * (dist_l + dist_r)
    raw_diff = dist_l - dist_r
    eff_track = TRACK_WIDTH_CW if raw_diff >= 0.0 else TRACK_WIDTH_CCW
    d_theta_rad = raw_diff / eff_track

    mid_theta_rad = math.radians(theta) + 0.5 * d_theta_rad
    x += dist_c * math.sin(mid_theta_rad)
    y += dist_c * math.cos(mid_theta_rad)
    theta = (theta + math.degrees(d_theta_rad)) % 360.0


def drive(left_dps: float, right_dps: float):
    """Commands drive motors with inverted polarity matching physical wiring."""
    left_motor.set_dps(-int(round(left_dps)))
    right_motor.set_dps(-int(round(right_dps)))


def stop_drive_motors():
    left_motor.set_power(0)
    right_motor.set_power(0)


def stop_all_motors():
    left_motor.set_power(0)
    right_motor.set_power(0)
    aux_motor_a.set_power(0)
    roller_motor.set_power(0)


def drive_with_heading_hold(base_dps: float, target_heading: float):
    """Drives straight at base_dps while holding target_heading via proportional control."""
    err = minimal_angle_diff(target_heading, theta)
    corr = max(-MAX_HEADING_CORR_DPS, min(MAX_HEADING_CORR_DPS, KP_DRIVE_HEADING * err))
    drive(base_dps + corr, base_dps - corr)


def turn_to(target_heading: float):
    """Rotates robot in place to target_heading (deg) with soft start & proportional slowdown."""
    target_heading = target_heading % 360.0
    update_odometer()
    diff = minimal_angle_diff(target_heading, theta)
    if abs(diff) < 1.0:
        return

    initial_dir = 1 if diff > 0 else -1
    t_start = time.time()

    while True:
        update_odometer()
        diff = minimal_angle_diff(target_heading, theta)
        if abs(diff) < 0.45 or (initial_dir == 1 and diff < 0) or (initial_dir == -1 and diff > 0):
            break

        direction = 1 if diff > 0 else -1
        ramp_scale = min(1.0, (time.time() - t_start) / 0.15)
        raw_speed = min(TURN_DPS_MAX, max(TURN_DPS_MIN, KP_TURN * abs(diff)))
        turn_speed = max(TURN_DPS_MIN, raw_speed * ramp_scale)

        drive(direction * turn_speed, -direction * turn_speed)
        time.sleep(0.008)

    stop_drive_motors()
    time.sleep(0.12)
    update_odometer()


def compute_trapezoidal_dps(elapsed_sec: float, remaining_cm: float, ceiling_dps: float) -> float:
    """Computes trapezoidal velocity command (deg/s) given elapsed time and remaining distance."""
    accel_dps = MIN_DRIVE_DPS + (ceiling_dps - MIN_DRIVE_DPS) * min(1.0, max(0.0, elapsed_sec) / ACCEL_RAMP_SEC)
    decel_dps = MIN_DRIVE_DPS + (ceiling_dps - MIN_DRIVE_DPS) * min(1.0, max(0.0, remaining_cm) / DECEL_DIST_CM)
    return max(MIN_DRIVE_DPS, min(ceiling_dps, accel_dps, decel_dps))


def drive_straight_distance(distance_cm: float, forward: bool = True, hold_heading: float | None = None, speed_dps: float = CRUISE_DPS):
    """Drives straight by distance_cm without line sensor triggers."""
    if distance_cm <= 0.1:
        return

    update_odometer()
    start_x, start_y = x, y
    lock_heading = theta if hold_heading is None else (hold_heading % 360.0)
    t_start = time.time()

    while True:
        update_odometer()
        traveled = math.hypot(x - start_x, y - start_y)
        remaining = distance_cm - traveled
        if remaining <= 0.0:
            break

        mag_dps = compute_trapezoidal_dps(time.time() - t_start, remaining, ceiling_dps=speed_dps)
        signed_dps = mag_dps if forward else -mag_dps
        drive_with_heading_hold(signed_dps, lock_heading)
        time.sleep(0.008)

    stop_drive_motors()
    time.sleep(0.1)
    update_odometer()


# ==================== ORIGIN <-> TILE (0,0) TRANSITION ====================
def step_origin_to_start_tile(start_tile: tuple[int, int] = (0, 0)):
    """
    Moves the robot from the starting grid intersection (0, 0) onto the center
    of `start_tile` (default Tile (0, 0) at (+15.24 cm, +15.24 cm)) via a single
    smooth 45° diagonal step of 21.55 cm.
    """
    target_x, target_y = tile_to_center_cm(start_tile[0], start_tile[1])
    update_odometer()
    dx = target_x - x
    dy = target_y - y
    dist = math.hypot(dx, dy)
    diag_heading = math.degrees(math.atan2(dx, dy)) % 360.0

    print(f"[NAV] Stepping from Origin (0,0) to Tile {start_tile} center ({target_x:.1f}, {target_y:.1f} cm)...")
    turn_to(diag_heading)
    drive_straight_distance(dist, forward=True, hold_heading=diag_heading, speed_dps=EXPLORE_DPS)


def step_tile_to_origin():
    """
    Moves the robot from its current position back onto the exact origin
    intersection (0.0, 0.0) at the end of a phase.
    """
    update_odometer()
    dx = 0.0 - x
    dy = 0.0 - y
    dist = math.hypot(dx, dy)
    if dist >= 0.5:
        home_heading = math.degrees(math.atan2(dx, dy)) % 360.0
        print(f"[NAV] Stepping onto Origin (0,0) (dist={dist:.1f} cm @ {home_heading:.1f}°)...")
        turn_to(home_heading)
        drive_straight_distance(dist, forward=True, hold_heading=home_heading, speed_dps=EXPLORE_DPS)
    turn_to(0.0)
    print(f"[NAV] Arrived at Origin: X={x:.2f} cm, Y={y:.2f} cm, θ={theta:.1f}°")


# ==================== ULTRASONIC & TURRET SUBSYSTEM ====================
def get_us_cm(samples: int = 3) -> float:
    """Returns median ultrasonic distance reading in cm (255.0 if open/none)."""
    readings = []
    for _ in range(samples):
        try:
            val = us_sensor.get_cm()
            if val is not None and 0.5 < val < 255.0:
                readings.append(float(val))
        except Exception:
            pass
        if samples > 1:
            time.sleep(0.01)
    if not readings:
        return 255.0
    return statistics.median(readings)


def set_turret_angle(rel_angle_deg: float):
    """
    Rotates the ultrasonic turret on Port A to rel_angle_deg relative to forward (0°):
      -90° = Left, 0° = Forward, +90° = Right.
    """
    if not USE_TURRET:
        return
    target_enc = int(round(turret_zero_enc + rel_angle_deg * TURRET_GEAR_RATIO))
    aux_motor_a.set_limits(power=80, dps=TURRET_DPS)
    aux_motor_a.set_position(target_enc)
    time.sleep(0.22)


def check_direction_distance(rel_angle_deg: float) -> float:
    """
    Measures distance (cm) in direction `rel_angle_deg` (-90=Left, 0=Forward, +90=Right).
    Uses the turret if USE_TURRET=True, or temporarily pivots the robot if USE_TURRET=False.
    """
    if abs(rel_angle_deg) < 1.0:
        if USE_TURRET:
            set_turret_angle(0.0)
        return get_us_cm(samples=3)

    if USE_TURRET:
        set_turret_angle(rel_angle_deg)
        dist = get_us_cm(samples=5)
        set_turret_angle(0.0)
        return dist
    else:
        saved_heading = theta
        turn_to((saved_heading + rel_angle_deg) % 360.0)
        dist = get_us_cm(samples=5)
        turn_to(saved_heading)
        return dist


# ==================== LINE SQUARING ON TILE BOUNDARIES ====================
def get_color_red(sensor) -> int:
    try:
        val = sensor.get_red()
        return val if val is not None else 100
    except Exception:
        return 100


def align_on_cross_line(nominal_heading: float, line_coord_cm: float, is_y_axis: bool):
    """
    Squares the robot across a perpendicular black grid line at `line_coord_cm`
    and snaps longitudinal odometry + yaw heading.
    """
    global x, y, theta, ignore_red_counter

    left_hit = get_color_red(left_color) < LINE_THRESHOLD
    right_hit = get_color_red(right_color) < LINE_THRESHOLD

    start_th = theta
    t0 = time.time()
    aborted = False

    if left_hit and not right_hit:
        drive(0, ALIGN_DPS)
        while get_color_red(right_color) >= LINE_THRESHOLD:
            update_odometer()
            if abs(minimal_angle_diff(theta, start_th)) > MAX_ALIGN_SWEEP_DEG or (time.time() - t0) > MAX_ALIGN_TIME_SEC:
                aborted = True
                break
            time.sleep(0.01)
    elif right_hit and not left_hit:
        drive(ALIGN_DPS, 0)
        while get_color_red(left_color) >= LINE_THRESHOLD:
            update_odometer()
            if abs(minimal_angle_diff(theta, start_th)) > MAX_ALIGN_SWEEP_DEG or (time.time() - t0) > MAX_ALIGN_TIME_SEC:
                aborted = True
                break
            time.sleep(0.01)

    stop_drive_motors()
    ignore_red_counter = 45

    if aborted:
        turn_to(nominal_heading)
        return

    # Snap heading & longitudinal coordinate to exact grid line
    theta = nominal_heading % 360.0
    positive_dir = nominal_heading in (0.0, 90.0)
    signed_offset = SENSOR_OFFSET if positive_dir else -SENSOR_OFFSET

    if is_y_axis:
        y = line_coord_cm - signed_offset
    else:
        x = line_coord_cm - signed_offset

    print(f"  [LINE SNAP] Squared on line {line_coord_cm / TILE_SIZE:.0f} -> Pose: ({x:.1f}, {y:.1f} cm, {theta:.1f}°)")


def step_one_tile_center(
    current_tile: tuple[int, int],
    heading_deg: int,
    speed_dps: float = CRUISE_DPS,
    enable_line_square: bool = True,
) -> tuple[int, int]:
    """
    Drives 1 tile (30.48 cm) from the center of `current_tile` to the center of the
    adjacent tile in `heading_deg` (0, 90, 180, 270).
    Halfway through the move (at 15.24 cm), the robot crosses the perpendicular grid line
    separating the two tiles, squares its chassis, and snaps its odometry!
    """
    global ignore_red_counter
    heading_deg = int(heading_deg) % 360
    dx, dy = HEADING_TO_DELTA[heading_deg]
    next_tile = (current_tile[0] + dx, current_tile[1] + dy)

    turn_to(float(heading_deg))

    start_x, start_y = tile_to_center_cm(current_tile[0], current_tile[1])
    target_x, target_y = tile_to_center_cm(next_tile[0], next_tile[1])

    is_y_axis = heading_deg in (0, 180)
    positive_dir = heading_deg in (0, 90)

    # Exact coordinate of the perpendicular grid line between current_tile and next_tile:
    cross_line_cm = 0.5 * ((start_y + target_y) if is_y_axis else (start_x + target_x))
    target_coord = target_y if is_y_axis else target_x

    t_start = time.time()
    squared_this_step = False
    ignore_red_counter = 0

    while True:
        update_odometer()
        ignore_red_counter -= 1

        curr_coord = y if is_y_axis else x
        signed_offset = SENSOR_OFFSET if positive_dir else -SENSOR_OFFSET
        sensor_coord = curr_coord + signed_offset

        # Check if color sensors are within the valid spatial window around cross_line_cm
        if enable_line_square and not squared_this_step and ignore_red_counter <= 0:
            if abs(sensor_coord - cross_line_cm) <= LINE_WINDOW_CM:
                if get_color_red(left_color) < LINE_THRESHOLD or get_color_red(right_color) < LINE_THRESHOLD:
                    align_on_cross_line(float(heading_deg), cross_line_cm, is_y_axis)
                    squared_this_step = True
                    curr_coord = y if is_y_axis else x
                    t_start = time.time()

        remaining_cm = (target_coord - curr_coord) if positive_dir else (curr_coord - target_coord)
        if remaining_cm <= 0.0:
            break

        # Slow down slightly right before hitting the cross-line for crisp squaring
        dist_to_line = (cross_line_cm - sensor_coord) if positive_dir else (sensor_coord - cross_line_cm)
        if enable_line_square and not squared_this_step and 0.0 < dist_to_line < LINE_SLOWDOWN_DIST_CM:
            ceiling = LINE_APPROACH_DPS + (speed_dps - LINE_APPROACH_DPS) * (dist_to_line / LINE_SLOWDOWN_DIST_CM)
        else:
            ceiling = speed_dps

        cmd_dps = compute_trapezoidal_dps(time.time() - t_start, remaining_cm, ceiling_dps=ceiling)
        drive_with_heading_hold(cmd_dps, float(heading_deg))
        time.sleep(0.008)

    stop_drive_motors()
    time.sleep(0.05)
    update_odometer()
    return next_tile


# ==================== BFS PATHFINDER & OVERTIME WATCHDOG ====================
def find_shortest_tile_path(
    start_tile: tuple[int, int],
    goal_tile: tuple[int, int],
    blocked_tiles: set[tuple[int, int]],
    allowed_tiles: set[tuple[int, int]] | None = None,
) -> list[tuple[int, int]]:
    """
    Computes shortest Manhattan tile path from start_tile to goal_tile using BFS.
    Returns list of tiles [start_tile, ..., goal_tile].
    """
    if start_tile == goal_tile:
        return [start_tile]

    queue = deque([[start_tile]])
    visited = {start_tile}

    while queue:
        path = queue.popleft()
        cx, cy = path[-1]

        for heading, (dx, dy) in HEADING_TO_DELTA.items():
            nx, ny = cx + dx, cy + dy
            nxt = (nx, ny)
            if nxt == goal_tile:
                return path + [nxt]
            if not is_tile_in_bounds(nx, ny):
                continue
            if nxt in blocked_tiles or nxt in visited:
                continue
            if allowed_tiles is not None and nxt not in allowed_tiles:
                continue
            visited.add(nxt)
            queue.append(path + [nxt])

    return [start_tile, goal_tile]


def navigate_tile_with_obstacle_avoidance(
    current_tile: tuple[int, int],
    target_tile: tuple[int, int],
    blocked_tiles: set[tuple[int, int]],
    speed_dps: float = CRUISE_DPS,
    check_obstacles: bool = True,
) -> tuple[int, int]:
    """
    Navigates from `current_tile` to `target_tile` along tile centers.
    If `check_obstacles` is True, checks forward ultrasonic distance before each step
    and dynamically reroutes around any unexpected exterior house boxes!
    """
    while current_tile != target_tile:
        path = find_shortest_tile_path(current_tile, target_tile, blocked_tiles)
        if len(path) < 2:
            break

        next_tile = path[1]
        dx = next_tile[0] - current_tile[0]
        dy = next_tile[1] - current_tile[1]
        desired_heading = next(h for h, d in HEADING_TO_DELTA.items() if d == (dx, dy))

        turn_to(float(desired_heading))

        if check_obstacles:
            fwd_dist = check_direction_distance(0.0)
            if fwd_dist < WALL_THRESHOLD_CM:
                print(f"  [OBSTACLE] Detected box at Tile {next_tile} ({fwd_dist:.1f} cm)! Rerouting...")
                blocked_tiles.add(next_tile)
                continue

        current_tile = step_one_tile_center(current_tile, desired_heading, speed_dps=speed_dps)

    return current_tile


def should_abort_for_time(phase_start_time: float, current_tile: tuple[int, int]) -> bool:
    """
    Real-Time Overtime Watchdog:
    Estimates time required to return from `current_tile` to Origin (0,0) and returns True
    if remaining phase time is within TIME_SAFETY_MARGIN_SEC of the 120s cap.
    """
    elapsed = time.time() - phase_start_time
    remaining = PHASE_TIME_LIMIT_SEC - elapsed

    manhattan_tiles = abs(current_tile[0] - 0) + abs(current_tile[1] - 0)
    linear_speed_cm_s = math.radians(CRUISE_DPS) * WHEEL_RADIUS
    est_return_sec = (manhattan_tiles * TILE_SIZE) / max(5.0, linear_speed_cm_s) + 6.0

    if remaining <= (est_return_sec + TIME_SAFETY_MARGIN_SEC):
        print(
            f"\n[WATCHDOG] Time Alert! Elapsed={elapsed:.1f}s, Remaining={remaining:.1f}s, "
            f"Est. Return={est_return_sec:.1f}s -> Triggering Safe Return to Origin!"
        )
        return True
    return False


# ==================== PHASE 2 INTAKE & DUMP ACTUATORS ====================
def start_intake_roller():
    """Starts the front sweeper roller motor on Port D."""
    print("[INTAKE] Starting front sweeper roller (Port D)...")
    roller_motor.set_dps(ROLLER_SWEEP_DPS)


def stop_intake_roller():
    """Stops the front sweeper roller motor."""
    roller_motor.set_power(0)


def dump_snow_payload():
    """
    Actuates the discharge mechanism (Port A rear trapdoor/ejector + Port D roller reverse/assist)
    to deposit collected pom-poms off the table edge into the flush bucket.
    """
    print("[DUMP] Actuating rear discharge mechanism into table-edge bucket...")
    base_enc = aux_motor_a.get_position() or 0
    aux_motor_a.set_limits(power=85, dps=250)
    aux_motor_a.set_position(int(base_enc + DUMP_GATE_OPEN_DEG))
    time.sleep(DUMP_DURATION_SEC)

    # Close gate back to initial position
    aux_motor_a.set_position(int(base_enc))
    time.sleep(0.6)
    aux_motor_a.set_power(0)
    print("[DUMP] Payload discharge complete!")
