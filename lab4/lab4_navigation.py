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

# ==================== ROBOT & DELIVERY CONSTANTS ====================
WHEEL_RADIUS = 2.1            # Radius of drive wheels in cm
TRACK_WIDTH = 12.00           # Distance between wheel contact points in cm
TILE_SIZE = 30.48             # Grid tile spacing in cm (30.48 cm per Lab 4 handout)
LINE_THRESHOLD = 30           # Reflected red light threshold for black line detection
SENSOR_OFFSET = 7.0           # Distance in cm from wheel axle to color sensors

FORWARD_DPS = 150
ALIGN_DPS = 60
TURN_DPS = 100
IGNORE_RED_CTR_START = 100

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

conveyor_target_deg = 0
conveyor_initialized = False


# ==================== ODOMETRY ====================
def update_odometer():
    """Updates (x, y, theta) from drive motor encoders and prints state to console."""
    global x, y, theta, prev_encoder_l, prev_encoder_r, odometer_initialized

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

    # Arc distances traveled by each wheel in cm
    dist_l = (d_l_deg * math.pi / 180.0) * WHEEL_RADIUS
    dist_r = (d_r_deg * math.pi / 180.0) * WHEEL_RADIUS
    delta_dist = (dist_l + dist_r) / 2.0

    # Angular change in degrees (positive = clockwise, matching compass convention)
    delta_theta_rad = (dist_l - dist_r) / TRACK_WIDTH
    delta_theta_deg = math.degrees(delta_theta_rad)

    # Average heading for linear displacement approximation
    avg_theta_rad = math.radians(theta + (delta_theta_deg / 2.0))

    # Convention: 0 deg = +y, 90 deg = +x
    x += delta_dist * math.sin(avg_theta_rad)
    y += delta_dist * math.cos(avg_theta_rad)

    # Ensure theta stays within [0.0, 360.0)
    theta = (theta + delta_theta_deg) % 360.0

    left_red = left_color.get_red()
    right_red = right_color.get_red()
    print(f"X: {x:6.2f} cm | Y: {y:6.2f} cm | θ: {theta:5.1f}deg | L_RED: {left_red} | R_RED: {right_red}")


# ==================== MOTION HELPERS ====================
def drive(left_dps, right_dps):
    """Drives wheels with inverted polarity matching physical motor orientation."""
    left_motor.set_dps(-left_dps)
    right_motor.set_dps(-right_dps)


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


def turn_to(target_heading: float):
    """Rotates the robot in place to target_heading using the minimal angle."""
    target_heading = target_heading % 360.0
    update_odometer()
    diff = minimal_angle_diff(target_heading, theta)

    # Skip turn if already aligned
    if abs(diff) < 2.0:
        return

    direction = 1 if diff > 0 else -1
    drive(direction * TURN_DPS, -direction * TURN_DPS)

    while True:
        update_odometer()
        diff = minimal_angle_diff(target_heading, theta)
        # Stop when within tolerance or if we just crossed the target angle
        if abs(diff) < 0.5 or (direction == 1 and diff < 0) or (direction == -1 and diff > 0):
            break
        time.sleep(0.01)

    stop_drive_motors()
    time.sleep(0.2)
    update_odometer()


def drive_straight_distance(distance_cm: float, forward: bool = True):
    """Drives straight forward or backward by distance_cm using encoder odometry."""
    if distance_cm <= 0.1:
        return

    update_odometer()
    start_x, start_y = x, y
    dps = FORWARD_DPS if forward else -FORWARD_DPS
    drive(dps, dps)

    while True:
        update_odometer()
        if math.hypot(x - start_x, y - start_y) >= distance_cm:
            break
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
               onto (gx, gy) so grid-line navigation continues seamlessly.
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

    # Mode 2: Exact-intersection cube placement maneuver
    # 1. Pause on the waypoint first so TA sees robot reached (gx, gy) within <= 2 cm
    time.sleep(WAYPOINT_ARRIVAL_PAUSE_SEC)

    # 2. Step forward by R along a board-safe direction phi
    phi = select_safe_step_heading(target_x, target_y, drop_radius, theta)
    beta_deg = math.degrees(math.atan2(DROP_OFFSET_LEFT_CM, DROP_OFFSET_BACK_CM))
    theta_drop = (phi - beta_deg) % 360.0

    print(
        f">>> [CUBE #{cube_number}] Positioning chute over {waypoint}: "
        f"step {drop_radius:.1f} cm @ {phi:.1f}deg, pivot to {theta_drop:.1f}deg"
    )
    turn_to(phi)
    drive_straight_distance(drop_radius, forward=True)

    # 3. Pivot so (-DROP_OFFSET_LEFT_CM, -DROP_OFFSET_BACK_CM) sits directly over (target_x, target_y)
    turn_to(theta_drop)
    actuate_conveyor_one_cube(cube_number)

    # 4. Return to (target_x, target_y) so robot is back on the grid line for the next leg
    if (not is_last_waypoint) or RETURN_TO_WAYPOINT_ON_LAST_CUBE:
        print(f">>> [CUBE #{cube_number}] Returning wheel axle to Waypoint {waypoint}...")
        turn_to(phi)
        drive_straight_distance(drop_radius, forward=False)


# ==================== NAVIGATION + DELIVERY STRATEGIES ====================
def align_on_line(nominal_heading: float, line_axis_val: float, is_y_axis: bool):
    """
    Stops the wheel that detected the line first and crawls the other wheel
    forward until it also detects the line, squaring the robot and snapping
    odometry coordinates.
    """
    global theta, x, y, ignore_red_counter

    left_hit = (left_color.get_red() or 100) < LINE_THRESHOLD
    right_hit = (right_color.get_red() or 100) < LINE_THRESHOLD

    if left_hit and not right_hit:
        drive(0, ALIGN_DPS)
        while (right_color.get_red() or 100) >= LINE_THRESHOLD:
            update_odometer()
            time.sleep(0.05)
    elif right_hit and not left_hit:
        drive(ALIGN_DPS, 0)
        while (left_color.get_red() or 100) >= LINE_THRESHOLD:
            update_odometer()
            time.sleep(0.05)

    ignore_red_counter = IGNORE_RED_CTR_START
    stop_drive_motors()

    # Snap heading and axis coordinate based on known grid line geometry and sensor offset
    theta = nominal_heading % 360.0

    if is_y_axis:
        # Facing 0deg (+y) or 180deg (-y)
        y = line_axis_val - SENSOR_OFFSET if nominal_heading == 0 else line_axis_val + SENSOR_OFFSET
    else:
        # Facing 90deg (+x) or 270deg (-x)
        x = line_axis_val - SENSOR_OFFSET if nominal_heading == 90 else line_axis_val + SENSOR_OFFSET

    # Push forward off the black tape (~2.5 cm, safely less than SENSOR_OFFSET = 7.0 cm)
    drive(FORWARD_DPS, FORWARD_DPS)
    push_start = time.time()
    while time.time() - push_start < 0.45:
        update_odometer()
        time.sleep(0.01)


def drive_axis_segment(target_coord: float, nominal_heading: float, is_y_axis: bool):
    """
    Drives straight along an axis (X or Y) to target_coord, correcting heading
    and position on every grid line intercepted along the way.
    """
    global ignore_red_counter
    ignore_red_counter = 0

    drive(FORWARD_DPS, FORWARD_DPS)

    while True:
        update_odometer()
        ignore_red_counter -= 1

        val_l = left_color.get_red() or 100
        val_r = right_color.get_red() or 100

        if ignore_red_counter <= 0 and (val_l < LINE_THRESHOLD or val_r < LINE_THRESHOLD):
            # Calculate the sensor's coordinate to snap to the nearest grid line
            if is_y_axis:
                sensor_coord = y + SENSOR_OFFSET if nominal_heading == 0 else y - SENSOR_OFFSET
            else:
                sensor_coord = x + SENSOR_OFFSET if nominal_heading == 90 else x - SENSOR_OFFSET

            est_line = round(sensor_coord / TILE_SIZE) * TILE_SIZE
            align_on_line(nominal_heading, est_line, is_y_axis)
            drive(FORWARD_DPS, FORWARD_DPS)

        # Check if wheel axle has reached target_coord along the travel direction
        if is_y_axis:
            reached = (y >= target_coord) if nominal_heading == 0 else (y <= target_coord)
        else:
            reached = (x >= target_coord) if nominal_heading == 90 else (x <= target_coord)

        if reached:
            break

        time.sleep(0.01)

    stop_drive_motors()
    time.sleep(0.2)
    update_odometer()


def navigate_and_deliver_safest(waypoints):
    """
    Safest Path Navigation + Cube Delivery (Recommended for Lab 4):
    Travels strictly along X and Y grid lines using dual light sensor correction
    to guarantee <= 2 cm stopping accuracy at every waypoint, delivering 1 cube
    per waypoint.
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
                drive_axis_segment(target_gy * TILE_SIZE, heading_y, is_y_axis=True)
                current_gy = target_gy
            if dx_grid != 0:
                turn_to(heading_x)
                drive_axis_segment(target_gx * TILE_SIZE, heading_x, is_y_axis=False)
                current_gx = target_gx
        else:
            if dx_grid != 0:
                turn_to(heading_x)
                drive_axis_segment(target_gx * TILE_SIZE, heading_x, is_y_axis=False)
                current_gx = target_gx
            if dy_grid != 0:
                turn_to(heading_y)
                drive_axis_segment(target_gy * TILE_SIZE, heading_y, is_y_axis=True)
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
    by the minimal angle in [-180, 180], delivering 1 cube per waypoint.
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
            # 0 deg = +y, 90 deg = +x -> atan2(dx, dy)
            target_theta = math.degrees(math.atan2(dx, dy)) % 360.0
            turn_to(target_theta)

            drive_straight_distance(target_dist, forward=True)

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

