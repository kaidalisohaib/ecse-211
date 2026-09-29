

import math
import time
from utils.brick import Motor, EV3ColorSensor, wait_ready_sensors, reset_brick

from utils.brick import EV3ColorSensor
s = EV3ColorSensor(4)
print("Port 4 Status:", s.get_status())
wait_ready_sensors()
reset_brick()
exit()
# ----------------- HARDWARE SETUP -----------------
left_motor = Motor("B")
right_motor = Motor("C")

# Two downward-facing color sensors mounted symmetrically on left and right
left_color = EV3ColorSensor(1)
right_color = EV3ColorSensor(2)

# ----------------- ROBOT CONSTANTS -----------------
# Calibrate these constants with your actual physical measurements
WHEEL_RADIUS = 2.1        # Radius of wheels in cm (standard EV3 drive wheel ~ 2.16 cm)
TRACK_WIDTH = 12.0         # Distance between wheel contact points in cm
TILE_SIZE = 30.0          # Standard grid tile spacing in cm
LINE_THRESHOLD = 30        # Reflected light intensity threshold for black line detection
SENSOR_OFFSET = 3.0        # Distance in cm from wheel axle to color sensors

FORWARD_DPS = 150
ALIGN_DPS = 60
TURN_DPS = 100

# ----------------- ODOMETRY STATE -----------------
# S-point initial coordinates (adjust according to your starting tile convention)
x = 0.0
y = 0.0
theta = 0.0                # Degrees: 0deg is +y, 90deg is +x, 180deg is -y, 270deg is -x

prev_encoder_l = 0
prev_encoder_r = 0
odometer_initialized = False


def update_odometer():
    """Updates (x, y, theta) from motor encoders and prints to console."""
    global x, y, theta, prev_encoder_l, prev_encoder_r, odometer_initialized

    enc_l = left_motor.get_encoder()
    enc_r = right_motor.get_encoder()

    if enc_l is None or enc_r is None:
        return

    if not odometer_initialized:
        prev_encoder_l = enc_l
        prev_encoder_r = enc_r
        odometer_initialized = True
        return

    # Invert sign if your motors are oriented backwards
    d_l_deg = enc_l - prev_encoder_l
    d_r_deg = enc_r - prev_encoder_r

    prev_encoder_l = enc_l
    prev_encoder_r = enc_r

    # Arc distances traveled by each wheel in cm
    dist_l = (d_l_deg * math.pi / 180.0) * WHEEL_RADIUS
    dist_r = (d_r_deg * math.pi / 180.0) * WHEEL_RADIUS
    delta_dist = (dist_l + dist_r) / 2.0

    # Angular change in degrees (positive = clockwise, matching convention)
    delta_theta_rad = (dist_l - dist_r) / TRACK_WIDTH
    delta_theta_deg = math.degrees(delta_theta_rad)

    # Average heading for linear displacement approximation
    avg_theta_rad = math.radians(theta + (delta_theta_deg / 2.0))

    # Convention: 0 deg = +y, 90 deg = +x
    x += delta_dist * math.sin(avg_theta_rad)
    y += delta_dist * math.cos(avg_theta_rad)

    # Ensure theta stays within [0.0, 359.9]
    theta = (theta + delta_theta_deg) % 360.0

    print(f"X: {x:6.2f} cm | Y: {y:6.2f} cm | θ: {theta:5.1f}deg")


def float_motors():
    """Required by TA to manually test encoder tracking."""
    left_motor.float_motor()
    right_motor.float_motor()

    while True:
        update_odometer()
        time.sleep(0.01)


# ----------------- MOTION HELPERS -----------------
def drive(left_dps, right_dps):
    left_motor.set_dps(left_dps)
    right_motor.set_dps(right_dps)


def stop_motors():
    left_motor.set_power(0)
    right_motor.set_power(0)


def turn_90_degrees_clockwise():
    """Turns the robot 90 degrees clockwise in place while updating odometry."""
    target_theta = (theta + 90.0) % 360.0
    drive(TURN_DPS, -TURN_DPS)

    # Turn until close to target
    while True:
        update_odometer()
        diff = (target_theta - theta) % 360.0
        # When facing target, diff is near 0 or 360
        if diff < 3.0 or diff > 357.0:
            break
        time.sleep(0.01)

    stop_motors()
    time.sleep(0.2)


def align_on_line(nominal_heading: float, line_axis_val: float, is_y_axis: bool):
    """
    Stops the wheel that detected the line first and crawls the other wheel
    forward until it also detects the line, re-aligning the robot.
    """
    global theta, x, y

    # Let the remaining wheel catch up to square up with the line
    left_hit = (left_color.get_red() or 100) < LINE_THRESHOLD
    right_hit = (right_color.get_red() or 100) < LINE_THRESHOLD

    if left_hit and not right_hit:
        drive(0, ALIGN_DPS)
        while (right_color.get_red() or 100) >= LINE_THRESHOLD:
            update_odometer()
            time.sleep(0.005)
    elif right_hit and not left_hit:
        drive(ALIGN_DPS, 0)
        while (left_color.get_red() or 100) >= LINE_THRESHOLD:
            update_odometer()
            time.sleep(0.005)

    stop_motors()

    # Snap heading and axis coordinate based on known grid line geometry
    theta = nominal_heading % 360.0

    if is_y_axis:
        # Facing 0deg or 180deg: heading along Y
        y = line_axis_val - SENSOR_OFFSET if nominal_heading == 0 else line_axis_val + SENSOR_OFFSET
    else:
        # Facing 90deg or 270deg: heading along X
        x = line_axis_val - SENSOR_OFFSET if nominal_heading == 90 else line_axis_val + SENSOR_OFFSET

    # Push forward slightly off the line so sensors don't trigger again immediately
    drive(FORWARD_DPS, FORWARD_DPS)
    time.sleep(0.3)


def drive_leg(distance_cm: float, nominal_heading: float, is_y_axis: bool):
    """Drives forward for distance_cm, correcting on lines along the way."""
    start_x = x
    start_y = y
    drive(FORWARD_DPS, FORWARD_DPS)

    while True:
        update_odometer()
        traveled = math.hypot(x - start_x, y - start_y)

        # Check line detection for odometry correction
        val_l = left_color.get_red() or 100
        val_r = right_color.get_red() or 100

        if val_l < LINE_THRESHOLD or val_r < LINE_THRESHOLD:
            # Estimate which grid line was intercepted (nearest 30.48 cm multiple)
            curr_coord = y if is_y_axis else x
            est_line = round(curr_coord / TILE_SIZE) * TILE_SIZE
            align_on_line(nominal_heading, est_line, is_y_axis)
            drive(FORWARD_DPS, FORWARD_DPS)

        if traveled >= distance_cm:
            break

        time.sleep(0.01)

    stop_motors()
    time.sleep(0.2)


def square_driver():
    """Drives a 90 cm square (3-by-3 tiles) trajectory while correcting with light sensors."""
    time.sleep(1.0)
    side_length = 3.0 * TILE_SIZE  # 90 cm

    # 4 legs of the square: (nominal heading, moving along Y axis?)
    legs = [
        (0.0, True),    # Leg 1: +y
        (90.0, False),  # Leg 2: +x
        (180.0, True),  # Leg 3: -y
        (270.0, False)  # Leg 4: -x
    ]

    for heading, is_y in legs:
        drive_leg(side_length, heading, is_y)
        turn_90_degrees_clockwise()

    stop_motors()
    print("Finished 90cm square trajectory.")


# ----------------- MAIN ENTRY POINT -----------------
if __name__ == "__main__":
    wait_ready_sensors(True)
    try:
        # Prompt for demo mode:
        # 'f' for float_motors demo, 's' for square_driver demo
        mode = input("Enter mode ('f' for float_motors, 's' for square_driver): ").strip().lower()
        if mode == 'f':
            float_motors()
        else:
            square_driver()
    except (KeyboardInterrupt, SystemExit, BaseException) as e:
        print(f"\nTerminated: {e}")
    finally:
        stop_motors()
        reset_brick()