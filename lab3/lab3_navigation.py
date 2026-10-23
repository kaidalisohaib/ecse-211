import math
import time
from utils.brick import Motor, EV3ColorSensor, wait_ready_sensors, reset_brick

# ==================== DEMO WAYPOINTS ====================
# Enter the 4 waypoints specified by the TA (in grid units, starting from (0, 0))
WAYPOINTS = [(2, 2), (1, 1), (0, 1), (2, 2)]

# ==================== HARDWARE CONFIG ====================
left_motor = Motor("B")
right_motor = Motor("C")

# Two downward-facing color sensors mounted symmetrically on left (Port 2) and right (Port 4)
left_color = EV3ColorSensor(4)
right_color = EV3ColorSensor(2)

# ==================== ROBOT CONSTANTS (FROM LAB 2) ====================
WHEEL_RADIUS = 2.1        # Radius of wheels in cm (calibrated in Lab 2)
TRACK_WIDTH = 12.00        # Distance between wheel contact points in cm (calibrated in Lab 2)
TILE_SIZE = 30.48         # Grid tile spacing in cm (30.48 cm per Lab 3 handout; use 30.0 if using 30 cm grid)
LINE_THRESHOLD = 30       # Reflected red light threshold for black line detection
SENSOR_OFFSET = 7.0      # Distance in cm from wheel axle to color sensors (calibrated in Lab 2) # For shortest, 7.4 seems better result

FORWARD_DPS = 150
ALIGN_DPS = 60
TURN_DPS = 100
IGNORE_RED_CTR_START = 100

# ==================== ODOMETRY STATE ====================
# Origin (0, 0) on the 4x4 grid, facing +y (0 deg)
x = 0.0                   # cm
y = 0.0                   # cm
theta = 0.0               # Degrees: 0deg is +y, 90deg is +x, 180deg is -y, 270deg is -x

prev_encoder_l = 0
prev_encoder_r = 0
odometer_initialized = False
ignore_red_counter = 0


def update_odometer():
    """Updates (x, y, theta) from motor encoders and prints state to console."""
    global x, y, theta, prev_encoder_l, prev_encoder_r, odometer_initialized

    raw_l = left_motor.get_encoder()
    raw_r = right_motor.get_encoder()

    if raw_l is None or raw_r is None:
        return

    # Motors are mounted inverted (same polarity as Lab 2)
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
    """Drives motors with inverted polarity matching physical motor orientation."""
    left_motor.set_dps(-left_dps)
    right_motor.set_dps(-right_dps)


def stop_motors():
    left_motor.set_power(0)
    right_motor.set_power(0)


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

    stop_motors()
    time.sleep(0.2)
    update_odometer()


def align_on_line(nominal_heading: float, line_axis_val: float, is_y_axis: bool):
    """
    Stops the wheel that detected the line first and crawls the other wheel
    forward until it also detects the line, squaring the robot and snapping
    odometry coordinates (using Lab 2 logic).
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
    stop_motors()

    # Snap heading and axis coordinate based on known grid line geometry and sensor offset
    theta = nominal_heading % 360.0

    if is_y_axis:
        # Facing 0deg (+y) or 180deg (-y)
        y = line_axis_val - SENSOR_OFFSET if nominal_heading == 0 else line_axis_val + SENSOR_OFFSET
    else:
        # Facing 90deg (+x) or 270deg (-x)
        x = line_axis_val - SENSOR_OFFSET if nominal_heading == 90 else line_axis_val + SENSOR_OFFSET

    # Push forward slightly off the line while continuing to update odometry
    drive(FORWARD_DPS, FORWARD_DPS)
    push_start = time.time()
    while time.time() - push_start < 0.8:
        update_odometer()
        time.sleep(0.01)


# ==================== NAVIGATION STRATEGIES ====================
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

    stop_motors()
    time.sleep(0.2)
    update_odometer()


def navigate_safest(waypoints):
    """
    Safest Path Navigation:
    Travels strictly along X and Y grid lines using Lab 2 light sensor correction.
    Chooses the axis requiring the smaller initial turn first (e.g., Y-first when
    starting at (0, 0) facing +y, matching Figure 1a in the Lab 3 handout).
    """
    time.sleep(1.0)
    update_odometer()

    current_gx, current_gy = 0, 0

    for target_gx, target_gy in waypoints:
        print(f"\n--- Navigating (Safest) to Waypoint ({target_gx}, {target_gy}) ---")
        dx_grid = target_gx - current_gx
        dy_grid = target_gy - current_gy

        heading_x = 90.0 if dx_grid > 0 else 270.0
        heading_y = 0.0 if dy_grid > 0 else 180.0

        # Determine whether to move along Y first or X first to minimize turning
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

        time.sleep(0.3)

    stop_motors()
    print("Finished Safest Path Navigation.")


def navigate_shortest(waypoints):
    """
    Shortest Path Navigation:
    Travels directly to each waypoint in a straight Euclidean line after rotating
    by the minimal angle in [-180, 180].
    """
    time.sleep(1.0)
    update_odometer()

    for target_gx, target_gy in waypoints:
        print(f"\n--- Navigating (Shortest) to Waypoint ({target_gx}, {target_gy}) ---")
        target_x = target_gx * TILE_SIZE
        target_y = target_gy * TILE_SIZE

        update_odometer()
        dx = target_x - x
        dy = target_y - y
        target_dist = math.hypot(dx, dy)

        if target_dist < 0.5:
            continue

        # 0 deg = +y, 90 deg = +x -> atan2(dx, dy)
        target_theta = math.degrees(math.atan2(dx, dy)) % 360.0
        turn_to(target_theta)

        start_x = x
        start_y = y
        drive(FORWARD_DPS, FORWARD_DPS)

        while True:
            update_odometer()
            traveled = math.hypot(x - start_x, y - start_y)
            if traveled >= target_dist:
                break
            time.sleep(0.01)

        stop_motors()
        time.sleep(0.3)
        update_odometer()

    stop_motors()
    print("Finished Shortest Path Navigation.")


# ==================== MAIN ENTRY POINT ====================
if __name__ == "__main__":
    wait_ready_sensors(True)

    try:
        mode = input("Select mode ('1' or 's' for Safest Path, '2' or 'd' for Shortest Path): ").strip().lower()
        if mode in ('1', 's', 'safest'):
            print("Starting Safest Path Navigation...")
            navigate_safest(WAYPOINTS)
        elif mode in ('2', 'd', 'shortest'):
            print("Starting Shortest Path Navigation...")
            navigate_shortest(WAYPOINTS)
        else:
            print("Invalid mode selected.")
    except (KeyboardInterrupt, SystemExit, BaseException) as e:
        print(f"\nTerminated: {e}")
    finally:
        stop_motors()
        reset_brick()
