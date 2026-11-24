#!/usr/bin/env python3
"""
ECSE 211 Final Design Project - Grid-Line Odometry Calibrator
=============================================================
Uses the two downward colour sensors and the black grid lines (30.48 cm apart)
as ground truth:

  1. Straight: square on line 0, drive across N tiles, read the encoders when each
     sensor reaches line N  ->  WHEEL_RADIUS and the left/right wheel scale.
  2. Turns: square on a line, spin ~350 deg, back off 3 cm, square on the same line
     again. The robot has now turned exactly 360 deg, so the total wheel travel
     difference gives the effective track width directly:
         W = (dist_left - dist_right) / (2*pi)     (clockwise; swapped for CCW)
     Each spin aims with the latest track-width estimate. Spins that stopped far from 360 deg
     (so the final squaring had to pivot a lot, which is less exact) are not averaged.
  Works with the colour sensors ahead of or behind the axle.

Saves calibration_params.json next to this file (where snowplow_core.py reads it).
"""
from __future__ import annotations

import json
import math
import os
import statistics
import time
from utils.brick import reset_brick
import snowplow_core as core   # same ports, motor polarity and line threshold as the main code

left_motor, right_motor = core.left_motor, core.right_motor
left_color, right_color = core.left_color, core.right_color
POL = core.DRIVE_POLARITY

TILE_SIZE = core.TILE_SIZE
LINE_THRESHOLD = core.LINE_THRESHOLD
LINE_CLEAR = LINE_THRESHOLD + 8      # Hysteresis: must read above this to count as "off the line"
WHEEL_RADIUS_SEED = core.WHEEL_RADIUS   # starting guesses: snowplow_core's constants (before calibration_params.json)
TRACK_WIDTH_SEED = core.TRACK_WIDTH     # the first spin aims with it: 12 on a 13.75 cm robot stops ~45 deg short
TURN_REPEATS = 3
MAX_SPIN_RESIDUAL_DEG = 8.0          # Average only spins that ended within this of a full turn

CALIBRATION_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibration_params.json")


def get_red(sensor):
    try:
        val = sensor.get_red()
        return val if val is not None else 100
    except Exception:
        return 100


def stop_drive():
    left_motor.set_power(0)
    right_motor.set_power(0)


def set_drive_dps(left_dps, right_dps):
    left_motor.set_dps(POL * int(round(left_dps)))
    right_motor.set_dps(POL * int(round(right_dps)))


def get_motor_positions():
    """Encoder degrees (left, right) with drive polarity applied. Retries failed reads."""
    for _ in range(20):
        l, r = left_motor.get_position(), right_motor.get_position()
        if l is not None and r is not None:
            return POL * l, POL * r
        time.sleep(0.005)
    raise IOError("Could not read drive encoders")


def autonomous_line_square(crawl_dps=45, timeout_sec=3.0):
    """Creeps forward until a sensor sees black, then pivots the other wheel until both do."""
    t0 = time.time()
    while time.time() - t0 < timeout_sec:
        if get_red(left_color) < LINE_THRESHOLD or get_red(right_color) < LINE_THRESHOLD:
            break
        set_drive_dps(crawl_dps, crawl_dps)
        time.sleep(0.005)
    stop_drive()
    time.sleep(0.1)

    red_l, red_r = get_red(left_color), get_red(right_color)
    t1 = time.time()
    if red_l < LINE_THRESHOLD <= red_r:
        set_drive_dps(0, crawl_dps)
        while get_red(right_color) >= LINE_THRESHOLD and time.time() - t1 < timeout_sec:
            time.sleep(0.005)
    elif red_r < LINE_THRESHOLD <= red_l:
        set_drive_dps(crawl_dps, 0)
        while get_red(left_color) >= LINE_THRESHOLD and time.time() - t1 < timeout_sec:
            time.sleep(0.005)
    stop_drive()
    time.sleep(0.15)


def drive_encoder_distance(cm, wheel_radius, dps):
    """Drives straight (both wheels same speed) for |cm|; sign of dps sets direction."""
    target_deg = abs(cm) / wheel_radius * 180.0 / math.pi
    l0, r0 = get_motor_positions()
    set_drive_dps(dps, dps)
    t0, limit = time.time(), 3.0 + 2.0 * target_deg / max(1.0, abs(dps))   # wheels blocked: give up, don't hang
    while time.time() - t0 < limit:
        l, r = get_motor_positions()
        if (abs(l - l0) + abs(r - r0)) / 2.0 >= target_deg:
            break
        time.sleep(0.005)
    else:
        print(f"    [TUNER] the wheels didn't turn {target_deg:.0f} deg in {limit:.0f} s (blocked? drive ports?)")
    stop_drive()
    time.sleep(0.15)


def auto_tune_straight(num_tiles=2):
    print("\n" + "=" * 65)
    print(f"  STEP 1: STRAIGHT ({num_tiles} tiles = {num_tiles * TILE_SIZE:.2f} cm)")
    print("=" * 65)
    print("Place the robot ~3 cm behind a black grid line, facing it. Starting in 2 s...")
    time.sleep(2.0)

    autonomous_line_square(crawl_dps=40)
    time.sleep(0.3)
    start_l, start_r = get_motor_positions()

    set_drive_dps(120, 120)
    seen_l = seen_r = 0
    hit_l = hit_r = None
    # Both sensors start ON line 0: they must leave it before any line counts.
    on_l = on_r = True
    t0 = time.time()
    while (seen_l < num_tiles or seen_r < num_tiles) and time.time() - t0 < 30:
        pos_l, pos_r = get_motor_positions()
        red_l, red_r = get_red(left_color), get_red(right_color)
        if red_l < LINE_THRESHOLD and not on_l:
            seen_l += 1
            hit_l = pos_l
            on_l = True
            print(f"     [Left] line {seen_l}/{num_tiles}")
        elif red_l >= LINE_CLEAR:
            on_l = False
        if red_r < LINE_THRESHOLD and not on_r:
            seen_r += 1
            hit_r = pos_r
            on_r = True
            print(f"     [Right] line {seen_r}/{num_tiles}")
        elif red_r >= LINE_CLEAR:
            on_r = False
        time.sleep(0.005)
    stop_drive()
    time.sleep(0.3)
    if hit_l is None or hit_r is None or seen_l < num_tiles or seen_r < num_tiles:
        raise RuntimeError("Did not see the target line with both sensors.")

    deg_l, deg_r = hit_l - start_l, hit_r - start_r
    deg_avg = (deg_l + deg_r) / 2.0
    dist = num_tiles * TILE_SIZE
    radius = dist / math.radians(deg_avg)
    ratio = deg_r / deg_l
    left_scale, right_scale = (ratio, 1.0) if ratio >= 1.0 else (1.0, 1.0 / ratio)

    print(f"    Encoders: L={deg_l:.1f}, R={deg_r:.1f} deg  ->  WHEEL_RADIUS={radius:.4f} cm "
          f"(seed {WHEEL_RADIUS_SEED}), scales L={left_scale:.4f} R={right_scale:.4f}")
    return radius, left_scale, right_scale


def measure_track_width(wheel_radius, left_scale, right_scale, direction="CW", track_guess=TRACK_WIDTH_SEED):
    """One exact 360-degree turn between two squarings on the same line -> (track width, spin residual deg)."""
    sign = 1.0 if direction == "CW" else -1.0
    autonomous_line_square(crawl_dps=35)
    time.sleep(0.3)
    l0, r0 = get_motor_positions()

    # Open-loop spin to ~350 deg using the seed track width.
    target_wheel_deg = (track_guess / (2.0 * wheel_radius)) * 350.0
    set_drive_dps(sign * 90, -sign * 90)
    t0, limit = time.time(), 3.0 + 2.0 * target_wheel_deg / 90.0
    while True:
        l, r = get_motor_positions()
        if (abs(l - l0) + abs(r - r0)) / 2.0 >= target_wheel_deg:
            break
        if time.time() - t0 > limit:
            stop_drive()
            raise RuntimeError(f"the spin didn't finish in {limit:.0f} s (wheels blocked, or the drive ports are wrong)")
        time.sleep(0.005)
    stop_drive()
    time.sleep(0.2)
    lo, ro = get_motor_positions()

    # Back off so both sensors are behind the line, then square on it again.
    drive_encoder_distance(3.0, wheel_radius, -40)
    autonomous_line_square(crawl_dps=30)
    l1, r1 = get_motor_positions()

    dist_l = math.radians(l1 - l0) * wheel_radius * left_scale
    dist_r = math.radians(r1 - r0) * wheel_radius * right_scale
    width = sign * (dist_l - dist_r) / (2.0 * math.pi)
    # How far the open-loop spin alone turned, using the width just measured (aiming was for 350).
    spun = sign * math.degrees((math.radians(lo - l0) * left_scale - math.radians(ro - r0) * right_scale)
                               * wheel_radius / width)
    residual = 350.0 - spun
    print(f"    {direction}: dL={dist_l:.2f} cm, dR={dist_r:.2f} cm -> TRACK_WIDTH_{direction}={width:.3f} cm "
          f"(spin stopped {residual:+.1f} deg from the aim)")
    return width, residual


def main():
    print("*" * 65)
    print("   ECSE 211 GRID-LINE ODOMETRY CALIBRATOR")
    print("*" * 65)
    core.wait_sensors_ready(need_colour=True)     # never hangs: names a port that sends nothing
    try:
        radius, ls, rs = auto_tune_straight(num_tiles=2)
        time.sleep(1.0)
        print("\n  STEP 2: TURNS. Leave the robot behind a grid line (it will find it).")
        widths = {}
        guess = TRACK_WIDTH_SEED
        for direction in ("CW", "CCW"):
            vals = []
            last = guess
            for i in range(TURN_REPEATS):
                print(f"\n  --- {direction} spin {i + 1}/{TURN_REPEATS} ---")
                width, residual = measure_track_width(radius, ls, rs, direction, guess)
                last = guess = width
                if abs(residual) <= MAX_SPIN_RESIDUAL_DEG:
                    vals.append(width)
                time.sleep(0.5)
            if not vals:
                vals = [last]
            widths[direction] = statistics.mean(vals)
            print(f"  {direction} mean {widths[direction]:.3f} cm (spread {max(vals) - min(vals):.3f})")

        params = {
            "WHEEL_RADIUS": round(radius, 4),
            "LEFT_WHEEL_SCALE": round(ls, 4),
            "RIGHT_WHEEL_SCALE": round(rs, 4),
            "TRACK_WIDTH": round((widths["CW"] + widths["CCW"]) / 2.0, 4),
            "TRACK_WIDTH_CW": round(widths["CW"], 4),
            "TRACK_WIDTH_CCW": round(widths["CCW"], 4),
            "TILE_SIZE": TILE_SIZE,
            "TIMESTAMP": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        with open(CALIBRATION_FILE, "w") as f:
            json.dump(params, f, indent=2)
        print("\n" + json.dumps(params, indent=2))
        print(f"Saved to: {CALIBRATION_FILE}")
    finally:
        stop_drive()
        reset_brick()


if __name__ == "__main__":
    main()
