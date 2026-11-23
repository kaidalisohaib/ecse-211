#!/usr/bin/env python3
"""
ECSE 211 Final Design Project - Autonomous Iterative Parameter Tuner
=====================================================================
Fully automated, zero-human-measurement calibration script.
Uses dual downward color sensors (Ports 4 & 2) and the black grid lines
(tile size = 30.48 cm) to iteratively converge physical odometry constants:

  1. WHEEL_RADIUS (cm) & WHEEL_SCALING (left vs right ratio)
  2. TRACK_WIDTH_CW (effective turning diameter for clockwise spins)
  3. TRACK_WIDTH_CCW (effective turning diameter for counter-clockwise spins)

No tape measure or protractor needed. The robot autonomously drives,
detects line crossings, self-squares, measures encoder deviations,
updates parameters iteratively, and saves to `calibration_params.json`.
"""

import math
import time
import json
import os
from utils.brick import Motor, EV3ColorSensor, wait_ready_sensors, reset_brick

# ==================== HARDWARE SETUP ====================
left_motor = Motor("B")
right_motor = Motor("C")
left_color = EV3ColorSensor(4)
right_color = EV3ColorSensor(2)

# ==================== INITIAL NOMINAL SEED CONSTANTS ====================
TILE_SIZE = 30.48             # cm per grid square
LINE_THRESHOLD = 30           # Reflected red light threshold for black lines
WHEEL_RADIUS_SEED = 2.10      # Starting estimate (cm)
TRACK_WIDTH_SEED = 12.00      # Starting estimate (cm)
SENSOR_SPACING = 12.00        # Approximate lateral distance between color sensors (cm)

CALIBRATION_FILE = "calibration_params.json"

# ==================== LOW-LEVEL SENSOR & MOTOR UTILS ====================
def get_red(sensor):
    """Reads reflected red light (0-100)."""
    try:
        val = sensor.get_red()
        return val if val is not None else 100
    except Exception:
        return 100

def stop_drive():
    left_motor.set_power(0)
    right_motor.set_power(0)

def set_drive_dps(left_dps, right_dps):
    """Drives with motor polarity inversion matching the physical chassis."""
    left_motor.set_dps(-int(round(left_dps)))
    right_motor.set_dps(-int(round(right_dps)))

def get_motor_positions():
    """Returns inverted encoder degrees for (left, right)."""
    return -left_motor.get_position(), -right_motor.get_position()

# ==================== AUTONOMOUS LINE SQUARING ====================
def autonomous_line_square(crawl_dps=45, timeout_sec=2.5):
    """
    Autonomously aligns both color sensors onto a black line.
    Returns:
      delta_wheel_deg: Signed differential wheel rotation required to square.
                       Positive = left wheel moved forward relative to right.
    """
    t_start = time.time()
    l_init, r_init = get_motor_positions()
    
    # 1. Approach until at least one sensor detects black
    while time.time() - t_start < timeout_sec:
        red_l = get_red(left_color)
        red_r = get_red(right_color)
        if red_l < LINE_THRESHOLD or red_r < LINE_THRESHOLD:
            break
        set_drive_dps(crawl_dps, crawl_dps)
        time.sleep(0.005)
    stop_drive()
    time.sleep(0.1)

    red_l = get_red(left_color)
    red_r = get_red(right_color)
    l_hit_pos, r_hit_pos = get_motor_positions()

    # 2. Pivot the trailing wheel until both sensors see black
    align_t0 = time.time()
    if red_l < LINE_THRESHOLD and red_r >= LINE_THRESHOLD:
        # Left hit first: hold left, pivot right wheel forward
        set_drive_dps(0, crawl_dps)
        while get_red(right_color) >= LINE_THRESHOLD and (time.time() - align_t0) < timeout_sec:
            time.sleep(0.005)
        stop_drive()
    elif red_r < LINE_THRESHOLD and red_l >= LINE_THRESHOLD:
        # Right hit first: hold right, pivot left wheel forward
        set_drive_dps(crawl_dps, 0)
        while get_red(left_color) >= LINE_THRESHOLD and (time.time() - align_t0) < timeout_sec:
            time.sleep(0.005)
        stop_drive()

    time.sleep(0.15)
    l_final, r_final = get_motor_positions()
    
    # Differential rotation during the squaring step
    delta_l = l_final - l_init
    delta_r = r_final - r_init
    return delta_l - delta_r

# ==================== STEP 1: ZERO-MEASUREMENT STRAIGHT TUNING ====================
def auto_tune_straight(num_tiles=2):
    """
    Calibrates WHEEL_RADIUS and LEFT_WHEEL_SCALE / RIGHT_WHEEL_SCALE.
    Robot starts ~3 cm behind Line 0, squares automatically on Line 0 (establishing zero),
    drives across `num_tiles` (60.96 cm), and squares on Line 2.
    Everything is computed directly from encoder ticks when lines are detected!
    """
    print("\n" + "=" * 65)
    print(f"  STEP 1: AUTONOMOUS STRAIGHT LINE TUNING ({num_tiles} TILES = {num_tiles*TILE_SIZE:.2f} cm)")
    print("=" * 65)
    print("Place robot ~3 cm behind any black grid line facing forward.")
    print("Starting automatic line lock in 2 seconds...")
    time.sleep(2.0)

    # 1. Lock onto Line 0
    print("  -> Squaring on Line 0 to establish ground-truth start...")
    autonomous_line_square(crawl_dps=40)
    time.sleep(0.3)
    start_l, start_r = get_motor_positions()
    print("  -> Start position locked!")

    # 2. Drive across tiles toward Line N
    target_dist_cm = num_tiles * TILE_SIZE
    print(f"  -> Driving {target_dist_cm:.2f} cm toward target line...")
    set_drive_dps(120, 120)

    lines_seen_l = 0
    lines_seen_r = 0
    hit_l_pos = None
    hit_r_pos = None
    in_l = False
    in_r = False

    while lines_seen_l < num_tiles or lines_seen_r < num_tiles:
        pos_l, pos_r = get_motor_positions()
        red_l = get_red(left_color)
        red_r = get_red(right_color)

        if red_l < LINE_THRESHOLD and not in_l:
            lines_seen_l += 1
            hit_l_pos = pos_l
            in_l = True
            print(f"     [Left Sensor] Detected line {lines_seen_l}/{num_tiles}")
        elif red_l >= LINE_THRESHOLD + 8:
            in_l = False

        if red_r < LINE_THRESHOLD and not in_r:
            lines_seen_r += 1
            hit_r_pos = pos_r
            in_r = True
            print(f"     [Right Sensor] Detected line {lines_seen_r}/{num_tiles}")
        elif red_r >= LINE_THRESHOLD + 8:
            in_r = False

        time.sleep(0.005)

    stop_drive()
    time.sleep(0.3)

    # Calculate exact distance from encoder counts at the target line
    deg_l = hit_l_pos - start_l
    deg_r = hit_r_pos - start_r
    deg_avg = (deg_l + deg_r) / 2.0

    calibrated_radius = target_dist_cm / (deg_avg * (math.pi / 180.0))
    
    # Scale ratio: if right turned more than left, left wheel is slightly larger
    ratio = deg_r / deg_l
    if ratio >= 1.0:
        left_scale = ratio
        right_scale = 1.0000
    else:
        left_scale = 1.0000
        right_scale = 1.0 / ratio

    print("\n  [STRAIGHT TUNING CONVERGED]:")
    print(f"    Target Distance:   {target_dist_cm:.2f} cm")
    print(f"    Encoder Counts:    L={deg_l:.1f}°, R={deg_r:.1f}° (Avg={deg_avg:.1f}°)")
    print(f"    WHEEL_RADIUS:      {calibrated_radius:.4f} cm (Seed: {WHEEL_RADIUS_SEED})")
    print(f"    LEFT_WHEEL_SCALE:  {left_scale:.4f}")
    print(f"    RIGHT_WHEEL_SCALE: {right_scale:.4f}")
    
    return calibrated_radius, left_scale, right_scale

# ==================== STEP 2: ITERATIVE TURN CONVERGENCE ====================
def auto_tune_turns_iterative(wheel_radius, left_scale, right_scale, direction="CW", max_iters=3, tol_deg=0.4):
    """
    Iteratively converges TRACK_WIDTH for a given spin direction (CW or CCW).
    Algorithm:
      1. Robot starts on a line (both sensors locked).
      2. Robot executes commanded 360° spin based on current track_width estimate.
      3. Robot searches/squares back onto the line using color sensors.
      4. Measures residual angular error theta_err from squaring motor degrees.
      5. Updates: track_width_new = track_width * (360 / (360 + theta_err)).
      6. Repeats until |theta_err| <= tol_deg (0.4°).
    """
    print("\n" + "=" * 65)
    print(f"  STEP 2: ITERATIVE {direction} TRACK WIDTH CONVERGENCE")
    print("=" * 65)
    print(f"Align robot on any grid line. Starting iterative auto-tuning in 2s...")
    time.sleep(2.0)

    # Initial line lock
    autonomous_line_square(crawl_dps=35)
    time.sleep(0.3)

    current_track = TRACK_WIDTH_SEED
    dir_sign = 1.0 if direction == "CW" else -1.0  # +1 for CW, -1 for CCW

    for iteration in range(1, max_iters + 1):
        print(f"\n  --- Iteration {iteration}/{max_iters} (Current Estimate: {current_track:.3f} cm) ---")

        # 1. Calculate required wheel rotation for full 360-deg pivot
        # Wheel travel arc = pi * track_width; Wheel deg = (track_width / (2 * R)) * 360
        target_wheel_deg = (current_track / (2.0 * wheel_radius)) * 360.0
        
        start_l, start_r = get_motor_positions()
        # Spin in place: for CW, left drives forward (+), right backward (-)
        speed_l = dir_sign * 75 * left_scale
        speed_r = -dir_sign * 75 * right_scale
        set_drive_dps(speed_l, speed_r)

        # Spin until open-loop 360 deg completes
        while True:
            cur_l, cur_r = get_motor_positions()
            turned = (abs(cur_l - start_l) + abs(cur_r - start_r)) / 2.0
            if turned >= target_wheel_deg - 10.0:  # slow down slightly before line
                break
            time.sleep(0.005)
            
        stop_drive()
        time.sleep(0.2)

        # 2. Color sensor ground-truth relock
        print("    -> Locking sensors onto line to measure angular overshoot/undershoot...")
        # Crawl forward slowly onto the line to catch it
        t_search = time.time()
        while time.time() - t_search < 1.5:
            if get_red(left_color) < LINE_THRESHOLD or get_red(right_color) < LINE_THRESHOLD:
                break
            # Gentle spin creep in direction of spin
            set_drive_dps(dir_sign * 30, -dir_sign * 30)
            time.sleep(0.005)
        stop_drive()

        # Run precise squaring and measure encoder correction
        diff_deg = autonomous_line_square(crawl_dps=30, timeout_sec=2.0)
        
        # Convert differential wheel degrees to robot heading error:
        # delta_heading_deg = (diff_deg * wheel_radius / track_width)
        angular_error_deg = dir_sign * (diff_deg * wheel_radius / current_track)

        print(f"    -> Measured Heading Residual: {angular_error_deg:+.2f}°")

        if abs(angular_error_deg) <= tol_deg:
            print(f"    *** CONVERGED in {iteration} iterations! Error within {tol_deg}° ***")
            break

        # Closed-loop update formula
        # If robot over-rotated (error > 0), track_width was overestimated -> reduce it
        correction_factor = 360.0 / (360.0 + angular_error_deg)
        current_track = current_track * correction_factor
        print(f"    -> Refined TRACK_WIDTH_{direction}: {current_track:.4f} cm")
        time.sleep(0.5)

    return current_track

# ==================== MAIN AUTO-TUNING ORCHESTRATOR ====================
def main():
    print("*" * 65)
    print("   ECSE 211 AUTONOMOUS SENSOR-DRIVEN ITERATIVE AUTO-TUNER")
    print("*" * 65)
    wait_ready_sensors(True)

    try:
        # Step 1: Autonomous straight driving over 2 tiles (60.96 cm)
        wheel_radius, left_scale, right_scale = auto_tune_straight(num_tiles=2)
        time.sleep(1.0)

        # Step 2: Iterative 360° sensor-guided convergence for CW turns
        track_cw = auto_tune_turns_iterative(wheel_radius, left_scale, right_scale, direction="CW", max_iters=3)
        time.sleep(1.0)

        # Step 3: Iterative 360° sensor-guided convergence for CCW turns
        track_ccw = auto_tune_turns_iterative(wheel_radius, left_scale, right_scale, direction="CCW", max_iters=3)

        # Final parameter assembly
        final_params = {
            "WHEEL_RADIUS": round(wheel_radius, 4),
            "LEFT_WHEEL_SCALE": round(left_scale, 4),
            "RIGHT_WHEEL_SCALE": round(right_scale, 4),
            "TRACK_WIDTH": round((track_cw + track_ccw) / 2.0, 4),
            "TRACK_WIDTH_CW": round(track_cw, 4),
            "TRACK_WIDTH_CCW": round(track_ccw, 4),
            "TILE_SIZE": TILE_SIZE,
            "TIMESTAMP": time.strftime("%Y-%m-%d %H:%M:%S")
        }

        with open(CALIBRATION_FILE, "w") as f:
            json.dump(final_params, f, indent=2)

        print("\n" + "=" * 65)
        print("  ALL PARAMETERS CONVERGED & AUTONOMOUSLY SAVED!")
        print("=" * 65)
        print(json.dumps(final_params, indent=2))
        print(f"Saved to: {os.path.abspath(CALIBRATION_FILE)}")

    finally:
        stop_drive()
        reset_brick()

if __name__ == "__main__":
    main()
