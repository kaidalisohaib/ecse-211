import math
import time
from utils.brick import Motor, EV3ColorSensor, wait_ready_sensors, reset_brick

# ==================== DEMO WAYPOINTS ====================
# Enter the 4 waypoints specified by the TA (in grid units, starting from (0, 0)).
# One foam cube will be delivered automatically at each of the 4 waypoints.
WAYPOINTS = [(2, 2), (1, 1), (0, 1), (2, 2)]

# ==================== DELIVERY PLACEMENT MODE ====================
# False -> Robot stops on the waypoint (<= 2 cm) and drops the cube in place (~12 cm away at back-left).
# True  -> Robot stops on the waypoint first, then positions the back-left conveyor chute directly
#          over the (gx, gy) intersection so the cube lands dead-center on the coordinate, waits
#          for the cube to be cleared, and reverses back onto (gx, gy) for the next leg.
DROP_CUBE_EXACTLY_ON_WAYPOINT = False

# Physical offsets (cm) from wheel-axle midpoint to where the center of a dropped cube lands:
# (Measure with a ruler from wheel-axle center to dropped cube center if fine-tuning mode=True)
DROP_OFFSET_BACK_CM = 8.5     # Distance (cm) BEHIND the wheel axle where the cube lands
DROP_OFFSET_LEFT_CM = 8.5     # Distance (cm) LEFT of robot centerline where the cube lands (negative if right)
RETURN_TO_WAYPOINT_ON_LAST_CUBE = False  # Keep False so robot doesn't back into Cube #4 at the end

# ==================== HARDWARE CONFIG ====================
left_motor = Motor("B")
right_motor = Motor("C")
conveyor_motor = Motor("D")   # Rear conveyor belt motor on Port MD

# Two downward-facing color sensors mounted symmetrically on left (Port 4) and right (Port 2)
left_color = EV3ColorSensor(4)
right_color = EV3ColorSensor(2)

# ==================== ROBOT & CONTROL CONSTANTS ====================
WHEEL_RADIUS = 2.1            # Radius of drive wheels in cm
LEFT_WHEEL_SCALE = 1.00       # Fine-tune (e.g. 1.01) if robot systematically curves right on straight drives
RIGHT_WHEEL_SCALE = 1.00      # Fine-tune (e.g. 1.01) if robot systematically curves left on straight drives

TRACK_WIDTH = 12.00           # Nominal distance between wheel contact points in cm
TRACK_WIDTH_CW = TRACK_WIDTH  # Effective track width for clockwise turns (increase if CW turns stop short)
TRACK_WIDTH_CCW = TRACK_WIDTH # Effective track width for counter-clockwise turns (increase if CCW turns stop short)

TILE_SIZE = 30.48             # Grid tile spacing in cm (30.48 cm per Lab 4 handout)
LINE_THRESHOLD = 30           # Reflected red light threshold for black line detection
SENSOR_OFFSET = 7.0           # Distance in cm from wheel axle to color sensors

# Speeds, trapezoidal motion profiling & closed-loop yaw control gains
FORWARD_DPS = 150             # Cruise speed (deg/s)
MIN_DRIVE_DPS = 45            # Gentle launch & final creep speed (deg/s) for zero-skid stops
LINE_APPROACH_DPS = 75        # Reduced speed right before hitting an expected black grid line
ALIGN_DPS = 50                # Single-wheel squaring crawl speed on black line
ACCEL_RAMP_SEC = 0.25         # Soft acceleration ramp duration (s) to prevent wheel spin & cube jolt
DECEL_DIST_CM = 8.0           # Distance (cm) before target to smoothly decelerate down to MIN_DRIVE_DPS
LINE_SLOWDOWN_DIST_CM = 6.0   # Distance (cm) before expected grid line to ease down to LINE_APPROACH_DPS

TURN_DPS_MAX = 100            # Peak turn speed (deg/s)
TURN_DPS_MIN = 32             # Minimum crawl speed near target heading to prevent overshoot
KP_TURN = 2.8                 # Proportional slow-down gain for turns (starts slowing within ~25 deg)
KP_DRIVE_HEADING = 2.0        # Active heading-hold gain during straight segments (fights skid drag)
MAX_HEADING_CORR_DPS = 25     # Max differential steering correction while driving straight

# Parallel-line & false-trigger protection
IGNORE_RED_CTR_START = 80
LINE_WINDOW_CM = 6.5          # Only accept line hits within +-6.5 cm of an expected perpendicular grid line
MAX_ALIGN_SWEEP_DEG = 16.0    # Abort single-wheel pivot if robot rotates >16 deg (prevents spinning on parallel line)
MAX_ALIGN_TIME_SEC = 1.2      # Timeout guard on single-wheel line alignment

# Conveyor delivery settings (tested: -90 deg per cube on Port MD)
CONVEYOR_DEG_PER_CUBE = -90   # Motor D rotation in degrees to deploy 1 foam cube
CONVEYOR_POWER_LIMIT = 80     # Max power (%) for smooth cube deployment
CONVEYOR_DPS_LIMIT = 250      # Speed limit (deg/s) so cubes drop gently without bouncing
WAYPOINT_ARRIVAL_PAUSE_SEC = 1.0  # Brief pause on waypoint before offset maneuver (when enabled)
POST_DROP_WAIT_SEC = 2.5      # Pause (s) after drop so cube settles & TA can clear it from board

# Safe wheel-axle bounds on the 4x4 tile wooden board (tiles span [-1, 3] in grid units)
BOARD_MIN_CM = -20.0
BOARD_MAX_CM = 3.0 * TILE_SIZE - 8.0

# ==================== ODOMETRY & CONVEYOR STATE ====================
# Origin (0, 0) on the 4x4 grid, facing +y (0 deg)
x = 0.0                       # cm
y = 0.0                       # cm
theta = 0.0                   # Degrees: 0deg is +y, 90deg is +x, 180deg is -y, 270deg is -x

prev_encoder_l = 0
prev_encoder_r = 0
odometer_initialized = False
ignore_red_counter = 0
odom_print_counter = 0

conveyor_target_deg = 0
conveyor_initialized = False


# ==================== FAST ODOMETRY ====================
def update_odometer(log_sensors: bool = False):
    """
    Updates (x, y, theta) from drive motor encoders at high frequency.
    Avoids unnecessary color-sensor SPI polling and throttles console printing
    so loop latency never causes turn overshoot or discretization drift.
    """
    global x, y, theta, prev_encoder_l, prev_encoder_r, odometer_initialized, odom_print_counter

    raw_l = left_motor.get_encoder()
    raw_r = right_motor.get_encoder()

    if raw_l is None or raw_r is None:
        return

    # Drive motors are mounted inverted
    enc_l = -raw_l
    enc_r = -raw_r

    if not odometer_initialized:
        prev_encoder_l = enc_l
        prev_encoder_r = enc_r
        odometer_initialized = True
        return

    d_l_deg = enc_l - prev_encoder_l
    d_r_deg = enc_r - prev_encoder_r

    prev_encoder_l = enc_l
    prev_encoder_r = enc_r

    # Arc distances traveled by each wheel in cm (with optional left/right wheel scale calibration)
    dist_l = (d_l_deg * math.pi / 180.0) * (WHEEL_RADIUS * LEFT_WHEEL_SCALE)
    dist_r = (d_r_deg * math.pi / 180.0) * (WHEEL_RADIUS * RIGHT_WHEEL_SCALE)
    delta_dist = (dist_l + dist_r) / 2.0

    # Directional effective track width (accounts for asymmetric rear-skid friction CW vs CCW)
    eff_track = TRACK_WIDTH_CW if (dist_l - dist_r) >= 0 else TRACK_WIDTH_CCW
    delta_theta_rad = (dist_l - dist_r) / eff_track
    delta_theta_deg = math.degrees(delta_theta_rad)

    # Exact midpoint RK2 integration (or arc integration)
    avg_theta_rad = math.radians(theta + (delta_theta_deg / 2.0))

    # Convention: 0 deg = +y, 90 deg = +x
    x += delta_dist * math.sin(avg_theta_rad)
    y += delta_dist * math.cos(avg_theta_rad)
    theta = (theta + delta_theta_deg) % 360.0

    odom_print_counter += 1
    if odom_print_counter >= 20:
        odom_print_counter = 0
        if log_sensors:
            l_red = left_color.get_red()
            r_red = right_color.get_red()
            print(f"X: {x:6.2f} cm | Y: {y:6.2f} cm | θ: {theta:5.1f}deg | L_RED: {l_red} | R_RED: {r_red}")
        else:
            print(f"X: {x:6.2f} cm | Y: {y:6.2f} cm | θ: {theta:5.1f}deg")


# ==================== MOTION & CLOSED-LOOP YAW HELPERS ====================
def drive(left_dps: float, right_dps: float):
    """Drives wheels with inverted polarity matching physical motor orientation."""
    left_motor.set_dps(-int(round(left_dps)))
    right_motor.set_dps(-int(round(right_dps)))


def stop_drive_motors():
    """Stops drive wheels while keeping conveyor motor holding its position."""
    left_motor.set_power(0)
    right_motor.set_power(0)


def stop_all_motors():
    """Stops drive wheels and conveyor motor."""
    left_motor.set_power(0)
    right_motor.set_power(0)
    conveyor_motor.set_power(0)


def minimal_angle_diff(target_deg: float, current_deg: float) -> float:
    """
    Calculates minimal signed rotation angle from current_deg to target_deg
    in the range [-180.0, 180.0].
    Positive = clockwise turn, Negative = counter-clockwise turn.
    """
    return ((target_deg - current_deg + 180.0) % 360.0) - 180.0


def drive_with_heading_hold(base_dps: float, target_heading: float):
    """
    Applies a proportional heading-hold controller on top of base_dps (works in
    both forward and reverse) to counteract yaw drift from rear-skid drag.
    """
    err = minimal_angle_diff(target_heading, theta)
    corr = max(-MAX_HEADING_CORR_DPS, min(MAX_HEADING_CORR_DPS, KP_DRIVE_HEADING * err))
    drive(base_dps + corr, base_dps - corr)


def turn_to(target_heading: float):
    """
    Rotates in place to target_heading using the minimal angle with:
      1. Soft acceleration ramp (prevents initial tire slip against rear skid), and
      2. Proportional deceleration near target (prevents rotational inertia overshoot).
    Color sensors are completely ignored during turns.
    """
    target_heading = target_heading % 360.0
    update_odometer()
    diff = minimal_angle_diff(target_heading, theta)

    if abs(diff) < 1.2:
        return

    initial_dir = 1 if diff > 0 else -1
    t_start = time.time()

    while True:
        update_odometer()
        diff = minimal_angle_diff(target_heading, theta)

        # Stop when within tight tolerance or if we just crossed 0
        if abs(diff) < 0.4 or (initial_dir == 1 and diff < 0) or (initial_dir == -1 and diff > 0):
            break

        direction = 1 if diff > 0 else -1
        # Soft start ramp over 0.15s to prevent static-friction wheel slip
        ramp_scale = min(1.0, (time.time() - t_start) / 0.15)
        raw_speed = min(TURN_DPS_MAX, max(TURN_DPS_MIN, KP_TURN * abs(diff)))
        turn_speed = max(TURN_DPS_MIN, raw_speed * ramp_scale)

        drive(direction * turn_speed, -direction * turn_speed)
        time.sleep(0.008)

    stop_drive_motors()
    time.sleep(0.2)
    update_odometer()


def compute_trapezoidal_dps(elapsed_sec: float, remaining_cm: float, ceiling_dps: float = FORWARD_DPS) -> float:
    """
    Computes a smooth trapezoidal velocity profile combining:
      - Soft time-based acceleration ramp from MIN_DRIVE_DPS -> ceiling_dps, and
      - Smooth distance-based deceleration from ceiling_dps -> MIN_DRIVE_DPS near the stop.
    """
    accel_dps = MIN_DRIVE_DPS + (ceiling_dps - MIN_DRIVE_DPS) * min(1.0, max(0.0, elapsed_sec) / ACCEL_RAMP_SEC)
    decel_dps = MIN_DRIVE_DPS + (ceiling_dps - MIN_DRIVE_DPS) * min(1.0, max(0.0, remaining_cm) / DECEL_DIST_CM)
    return max(MIN_DRIVE_DPS, min(ceiling_dps, accel_dps, decel_dps))


def drive_straight_distance(distance_cm: float, forward: bool = True, hold_heading: float | None = None):
    """
    Drives straight forward or backward by distance_cm with:
      - Smooth trapezoidal acceleration & deceleration (no sudden starts/stops), and
      - Active yaw-hold P-control.
    Color sensors are completely ignored so crossing lines during drop maneuvers
    can never trigger a false line alignment.
    """
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

        mag_dps = compute_trapezoidal_dps(time.time() - t_start, remaining)
        signed_dps = mag_dps if forward else -mag_dps
        drive_with_heading_hold(signed_dps, lock_heading)
        time.sleep(0.01)

    stop_drive_motors()
    time.sleep(0.2)
    update_odometer()


# ==================== CONVEYOR DELIVERY HELPERS ====================
def init_conveyor():
    """Latches the current conveyor encoder position and configures speed/power limits."""
    global conveyor_target_deg, conveyor_initialized
    enc = conveyor_motor.get_encoder()
    if enc is None:
        conveyor_motor.reset_encoder()
        conveyor_target_deg = 0
    else:
        conveyor_target_deg = int(enc)
    conveyor_motor.set_limits(power=CONVEYOR_POWER_LIMIT, dps=CONVEYOR_DPS_LIMIT)
    conveyor_initialized = True


def actuate_conveyor_one_cube(cube_number: int):
    """Advances Motor D by CONVEYOR_DEG_PER_CUBE (-90 deg) using drift-free cumulative targets."""
    global conveyor_target_deg
    if not conveyor_initialized:
        init_conveyor()

    print(f">>> [CUBE #{cube_number}] Rotating conveyor MD by {CONVEYOR_DEG_PER_CUBE} deg...")
    time.sleep(0.3)

    conveyor_target_deg += CONVEYOR_DEG_PER_CUBE
    conveyor_motor.set_limits(power=CONVEYOR_POWER_LIMIT, dps=CONVEYOR_DPS_LIMIT)
    conveyor_motor.set_position(int(conveyor_target_deg))

    time.sleep(POST_DROP_WAIT_SEC)
    print(f">>> [CUBE #{cube_number}] Delivery complete!\n")


def select_safe_step_heading(xd: float, yd: float, step_radius: float, current_th: float) -> float:
    """
    Picks a forward step direction phi so stepping step_radius cm from (xd, yd) keeps
    the robot safely inside the 4x4 wooden board boundaries.
    Prefers current_th first (0 deg extra turn), then nearest cardinal grid axis,
    and falls back to pointing toward the board center for extreme outer corners.
    """
    cardinals = sorted([0.0, 90.0, 180.0, 270.0], key=lambda c: abs(minimal_angle_diff(c, current_th)))
    center_x, center_y = 1.0 * TILE_SIZE, 1.0 * TILE_SIZE
    to_center_deg = math.degrees(math.atan2(center_x - xd, center_y - yd)) % 360.0

    candidates = [current_th % 360.0] + cardinals + [to_center_deg]
    for cand in candidates:
        rx = xd + step_radius * math.sin(math.radians(cand))
        ry = yd + step_radius * math.cos(math.radians(cand))
        if BOARD_MIN_CM <= rx <= BOARD_MAX_CM and BOARD_MIN_CM <= ry <= BOARD_MAX_CM:
            return cand

    return to_center_deg


def deliver_at_waypoint(cube_number: int, waypoint: tuple[int, int], is_last_waypoint: bool = False):
    """
    Delivers 1 foam cube at `waypoint` according to DROP_CUBE_EXACTLY_ON_WAYPOINT:
      - False: Drops cube immediately while wheel center sits on (gx, gy).
      - True:  Pauses on (gx, gy), steps forward by R = hypot(L, B) along a safe direction phi,
               pivots by -atan2(L, B) so the rear-left conveyor opening is directly over (gx, gy),
               drops the cube dead-center on (gx, gy), waits for board clearing, and reverses back
               onto (gx, gy) with heading-hold active (color sensors disabled throughout!).
    """
    target_x = waypoint[0] * TILE_SIZE
    target_y = waypoint[1] * TILE_SIZE
    pos_err = math.hypot(target_x - x, target_y - y)
    print(
        f"\n>>> [CUBE #{cube_number}] Reached Waypoint {waypoint} "
        f"(Odom: X={x:.2f} cm, Y={y:.2f} cm, Est. Error={pos_err:.2f} cm)"
    )

    drop_radius = math.hypot(DROP_OFFSET_LEFT_CM, DROP_OFFSET_BACK_CM)

    # Mode 1: Drop in-place while robot's wheel center sits at the waypoint
    if not DROP_CUBE_EXACTLY_ON_WAYPOINT or drop_radius < 0.5:
        actuate_conveyor_one_cube(cube_number)
        return

    # Mode 2: Exact-intersection cube placement maneuver (light sensors completely ignored here)
    time.sleep(WAYPOINT_ARRIVAL_PAUSE_SEC)

    phi = select_safe_step_heading(target_x, target_y, drop_radius, theta)
    beta_deg = math.degrees(math.atan2(DROP_OFFSET_LEFT_CM, DROP_OFFSET_BACK_CM))
    theta_drop = (phi - beta_deg) % 360.0

    print(
        f">>> [CUBE #{cube_number}] Positioning chute over {waypoint}: "
        f"step {drop_radius:.1f} cm @ {phi:.1f}deg, pivot to {theta_drop:.1f}deg"
    )
    turn_to(phi)
    drive_straight_distance(drop_radius, forward=True, hold_heading=phi)

    turn_to(theta_drop)
    actuate_conveyor_one_cube(cube_number)

    if (not is_last_waypoint) or RETURN_TO_WAYPOINT_ON_LAST_CUBE:
        print(f">>> [CUBE #{cube_number}] Returning wheel axle to Waypoint {waypoint}...")
        turn_to(phi)
        drive_straight_distance(drop_radius, forward=False, hold_heading=phi)


# ==================== LINE ALIGNMENT & SAFEST/SHORTEST NAVIGATION ====================
def align_on_line(nominal_heading: float, line_axis_val: float, is_y_axis: bool):
    """
    Squares the robot against a perpendicular grid line using both color sensors.
    Includes a max-angle sweep guard (MAX_ALIGN_SWEEP_DEG) so if a single sensor
    accidentally grazed a PARALLEL grid line instead of a perpendicular cross-line,
    it immediately aborts the pivot and restores nominal_heading instead of spinning
    onto the parallel line!
    """
    global theta, x, y, ignore_red_counter

    left_hit = (left_color.get_red() or 100) < LINE_THRESHOLD
    right_hit = (right_color.get_red() or 100) < LINE_THRESHOLD

    start_align_th = theta
    start_time = time.time()
    aborted_parallel = False

    if left_hit and not right_hit:
        drive(0, ALIGN_DPS)
        while (right_color.get_red() or 100) >= LINE_THRESHOLD:
            update_odometer()
            if (
                abs(minimal_angle_diff(theta, start_align_th)) > MAX_ALIGN_SWEEP_DEG
                or (time.time() - start_time) > MAX_ALIGN_TIME_SEC
            ):
                aborted_parallel = True
                break
            time.sleep(0.02)
    elif right_hit and not left_hit:
        drive(ALIGN_DPS, 0)
        while (left_color.get_red() or 100) >= LINE_THRESHOLD:
            update_odometer()
            if (
                abs(minimal_angle_diff(theta, start_align_th)) > MAX_ALIGN_SWEEP_DEG
                or (time.time() - start_time) > MAX_ALIGN_TIME_SEC
            ):
                aborted_parallel = True
                break
            time.sleep(0.02)

    stop_drive_motors()
    ignore_red_counter = IGNORE_RED_CTR_START

    if aborted_parallel:
        print("[GUARD] Parallel-line graze detected during alignment! Aborting snap & restoring heading.")
        turn_to(nominal_heading)
        return

    # Valid perpendicular cross-line: snap heading and axis coordinate
    theta = nominal_heading % 360.0

    if is_y_axis:
        y = line_axis_val - SENSOR_OFFSET if nominal_heading == 0 else line_axis_val + SENSOR_OFFSET
    else:
        x = line_axis_val - SENSOR_OFFSET if nominal_heading == 90 else line_axis_val + SENSOR_OFFSET

    # Glide gently off the black tape (~1.8 cm at LINE_APPROACH_DPS, safely < SENSOR_OFFSET = 7.0 cm)
    push_start = time.time()
    while time.time() - push_start < 0.55:
        update_odometer()
        drive_with_heading_hold(LINE_APPROACH_DPS, nominal_heading)
        time.sleep(0.01)


def is_expected_cross_line(
    sensor_coord: float, start_coord: float, target_coord: float, positive_dir: bool
) -> tuple[bool, float]:
    """
    Returns (is_valid, est_line) checking two spatial gates:
      1. sensor_coord is within +-LINE_WINDOW_CM of a true grid multiple k * TILE_SIZE
         (ignores middle-of-tile parallel line grazing).
      2. est_line lies strictly AHEAD of start_coord up to target_coord
         (never snaps backward onto the intersection we just departed from).
    """
    est_line = round(sensor_coord / TILE_SIZE) * TILE_SIZE
    if abs(sensor_coord - est_line) > LINE_WINDOW_CM:
        return False, est_line

    half_tile = 0.5 * TILE_SIZE
    if positive_dir:
        valid_range = (start_coord + half_tile) <= est_line <= (target_coord + half_tile)
    else:
        valid_range = (target_coord - half_tile) <= est_line <= (start_coord - half_tile)

    return valid_range, est_line


def compute_pre_line_ceiling_dps(
    sensor_coord: float, start_coord: float, target_coord: float, positive_dir: bool
) -> float:
    """
    Smoothly eases speed from FORWARD_DPS down to LINE_APPROACH_DPS when the front
    sensors get within LINE_SLOWDOWN_DIST_CM (6 cm) of an upcoming grid line, so
    hitting the black tape never causes an abrupt high-speed brake/nose-dive.
    """
    if positive_dir:
        next_line = math.ceil((sensor_coord - 1.0) / TILE_SIZE) * TILE_SIZE
        dist_to_line = next_line - sensor_coord
    else:
        next_line = math.floor((sensor_coord + 1.0) / TILE_SIZE) * TILE_SIZE
        dist_to_line = sensor_coord - next_line

    half_tile = 0.5 * TILE_SIZE
    if positive_dir:
        in_segment = (start_coord + half_tile) <= next_line <= (target_coord + half_tile)
    else:
        in_segment = (target_coord - half_tile) <= next_line <= (start_coord - half_tile)

    if not in_segment or dist_to_line >= LINE_SLOWDOWN_DIST_CM:
        return FORWARD_DPS
    if dist_to_line <= 0.0:
        return LINE_APPROACH_DPS

    ratio = dist_to_line / LINE_SLOWDOWN_DIST_CM
    return LINE_APPROACH_DPS + (FORWARD_DPS - LINE_APPROACH_DPS) * ratio


def drive_axis_segment(start_coord: float, target_coord: float, nominal_heading: float, is_y_axis: bool):
    """
    Drives straight along an axis (X or Y) from start_coord to target_coord using:
      - Smooth trapezoidal acceleration & waypoint deceleration,
      - Predictive slowdown before each expected black grid line,
      - Active closed-loop heading hold between grid lines,
      - Spatial window filtering + max-sweep guard to reject parallel-line false triggers,
      - Dual color-sensor squaring & coordinate snapping at valid cross-lines.
    """
    global ignore_red_counter
    ignore_red_counter = 0
    positive_dir = nominal_heading in (0.0, 90.0)
    ramp_start_time = time.time()

    while True:
        update_odometer(log_sensors=True)
        ignore_red_counter -= 1

        if is_y_axis:
            curr_pos = y
            sensor_coord = y + SENSOR_OFFSET if positive_dir else y - SENSOR_OFFSET
        else:
            curr_pos = x
            sensor_coord = x + SENSOR_OFFSET if positive_dir else x - SENSOR_OFFSET

        val_l = left_color.get_red() or 100
        val_r = right_color.get_red() or 100

        if ignore_red_counter <= 0 and (val_l < LINE_THRESHOLD or val_r < LINE_THRESHOLD):
            valid_cross_line, est_line = is_expected_cross_line(
                sensor_coord, start_coord, target_coord, positive_dir
            )
            if valid_cross_line:
                align_on_line(nominal_heading, est_line, is_y_axis)
                # Refresh position & restart smooth acceleration ramp from LINE_APPROACH_DPS
                curr_pos = y if is_y_axis else x
                ramp_start_time = time.time()

        remaining_cm = (target_coord - curr_pos) if positive_dir else (curr_pos - target_coord)
        if remaining_cm <= 0.0:
            break

        line_ceiling = (
            compute_pre_line_ceiling_dps(sensor_coord, start_coord, target_coord, positive_dir)
            if ignore_red_counter <= 0
            else FORWARD_DPS
        )
        cmd_dps = compute_trapezoidal_dps(time.time() - ramp_start_time, remaining_cm, ceiling_dps=line_ceiling)
        drive_with_heading_hold(cmd_dps, nominal_heading)
        time.sleep(0.01)

    stop_drive_motors()
    time.sleep(0.2)
    update_odometer()


def navigate_and_deliver_safest(waypoints):
    """
    Safest Path Navigation + Cube Delivery (Recommended for Lab 4):
    Travels strictly along X and Y grid lines using dual light sensor correction
    and closed-loop heading hold to guarantee <= 2 cm accuracy at every waypoint.
    """
    init_conveyor()
    time.sleep(1.0)
    update_odometer()

    current_gx, current_gy = 0, 0
    total = len(waypoints)

    for idx, (target_gx, target_gy) in enumerate(waypoints, start=1):
        print(f"\n--- [Waypoint {idx}/{total}] Navigating (Safest) to ({target_gx}, {target_gy}) ---")
        dx_grid = target_gx - current_gx
        dy_grid = target_gy - current_gy

        heading_x = 90.0 if dx_grid > 0 else 270.0
        heading_y = 0.0 if dy_grid > 0 else 180.0

        # Choose the axis requiring the smaller initial turn first to minimize turns
        y_first = True
        if dx_grid != 0 and dy_grid != 0:
            turn_for_x = abs(minimal_angle_diff(heading_x, theta))
            turn_for_y = abs(minimal_angle_diff(heading_y, theta))
            y_first = turn_for_y <= turn_for_x

        if y_first:
            if dy_grid != 0:
                turn_to(heading_y)
                drive_axis_segment(
                    current_gy * TILE_SIZE, target_gy * TILE_SIZE, heading_y, is_y_axis=True
                )
                current_gy = target_gy
            if dx_grid != 0:
                turn_to(heading_x)
                drive_axis_segment(
                    current_gx * TILE_SIZE, target_gx * TILE_SIZE, heading_x, is_y_axis=False
                )
                current_gx = target_gx
        else:
            if dx_grid != 0:
                turn_to(heading_x)
                drive_axis_segment(
                    current_gx * TILE_SIZE, target_gx * TILE_SIZE, heading_x, is_y_axis=False
                )
                current_gx = target_gx
            if dy_grid != 0:
                turn_to(heading_y)
                drive_axis_segment(
                    current_gy * TILE_SIZE, target_gy * TILE_SIZE, heading_y, is_y_axis=True
                )
                current_gy = target_gy

        stop_drive_motors()
        update_odometer()

        # Deliver 1 foam cube at this waypoint
        deliver_at_waypoint(idx, (target_gx, target_gy), is_last_waypoint=(idx == total))

    stop_all_motors()
    print("Finished Safest Path Delivery for all waypoints!")


def navigate_and_deliver_shortest(waypoints):
    """
    Shortest Path Navigation + Cube Delivery:
    Travels directly to each waypoint in a straight Euclidean line after rotating
    by the minimal angle in [-180, 180], using closed-loop heading hold while driving.
    """
    init_conveyor()
    time.sleep(1.0)
    update_odometer()

    total = len(waypoints)

    for idx, (target_gx, target_gy) in enumerate(waypoints, start=1):
        print(f"\n--- [Waypoint {idx}/{total}] Navigating (Shortest) to ({target_gx}, {target_gy}) ---")
        target_x = target_gx * TILE_SIZE
        target_y = target_gy * TILE_SIZE

        update_odometer()
        dx = target_x - x
        dy = target_y - y
        target_dist = math.hypot(dx, dy)

        if target_dist >= 0.5:
            target_theta = math.degrees(math.atan2(dx, dy)) % 360.0
            turn_to(target_theta)
            drive_straight_distance(target_dist, forward=True, hold_heading=target_theta)

        # Deliver 1 foam cube at this waypoint (even if consecutive waypoints match)
        deliver_at_waypoint(idx, (target_gx, target_gy), is_last_waypoint=(idx == total))

    stop_all_motors()
    print("Finished Shortest Path Delivery for all waypoints!")


# ==================== MAIN ENTRY POINT ====================
if __name__ == "__main__":
    wait_ready_sensors(True)

    try:
        placement_str = (
            f"EXACT ON WAYPOINT (back={DROP_OFFSET_BACK_CM}cm, left={DROP_OFFSET_LEFT_CM}cm)"
            if DROP_CUBE_EXACTLY_ON_WAYPOINT
            else "IN-PLACE AT WAYPOINT"
        )
        print("\n=== ECSE 211 Lab 4: Autonomous Cube Delivery ===")
        print(f"Configured Waypoints ({len(WAYPOINTS)}): {WAYPOINTS}")
        print(f"Cube Placement Mode: {placement_str}")
        mode = input(
            "Select mode ('1'/'s' = Safest Path [Recommended], '2'/'d' = Shortest Path, "
            "'c' = Test 1 Cube Drop, 'p' = Test Exact-Point Drop @ (0,0)): "
        ).strip().lower()

        if mode in ("1", "s", "safest", ""):
            print("Starting Safest Path Navigation & Delivery...")
            navigate_and_deliver_safest(WAYPOINTS)
        elif mode in ("2", "d", "shortest"):
            print("Starting Shortest Path Navigation & Delivery...")
            navigate_and_deliver_shortest(WAYPOINTS)
        elif mode == "c":
            init_conveyor()
            actuate_conveyor_one_cube(1)
        elif mode == "p":
            DROP_CUBE_EXACTLY_ON_WAYPOINT = True
            init_conveyor()
            deliver_at_waypoint(1, (0, 0), is_last_waypoint=False)
        else:
            print("Invalid mode selected.")
    except (KeyboardInterrupt, SystemExit, BaseException) as e:
        print(f"\nTerminated: {e}")
    finally:
        stop_all_motors()
        reset_brick()
