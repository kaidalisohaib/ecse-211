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
CRAWL_DPS = 60                       # strong enough not to stall EV3 motors, slow enough for clean squaring
TURN_REPEATS = 3
BACKOFF_CM = 5.5                     # back off before re-squaring (clears 14 cm sensor bar up to ~25 deg spin error)
MAX_SPIN_RESIDUAL_DEG = 10.0         # Average only spins that ended within this of a full 360 deg turn

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


def _square_after_hit(hit_l, hit_r, crawl_dps=CRAWL_DPS, timeout_sec=4.0):
    """
    Called the instant at least one colour sensor sees black:
      1. Actively brakes (set_dps(0)) while polling both sensors for 80 ms so a near-simultaneous hit isn't missed.
      2. If the hit sensor(s) coasted past the thin line during braking, backs BOTH wheels just behind the line
         and creeps forward at slow crawl_dps (so coasting is < 0.1 cm and the hit sensor stays ON the line).
      3. If only one sensor hit, holds its wheel at 0 and creeps the other wheel forward until the second hits.
      4. With sensors behind the axle (SENSOR_OFFSET < 0), straightening from a large angle swings the first
         sensor slightly *behind* the line by |offset|*(1 - cos theta): if it left the line, creep its wheel
         forward a few mm back onto the leading edge.
    Returns True if both sensors locked onto the line, False on timeout.
    """
    set_drive_dps(0, 0)
    t_brake = time.time()
    while time.time() - t_brake < 0.08:
        if not hit_l and get_red(left_color) < LINE_THRESHOLD:
            hit_l = True
        if not hit_r and get_red(right_color) < LINE_THRESHOLD:
            hit_r = True
        time.sleep(0.005)

    if not hit_l and not hit_r:
        stop_drive()
        return False

    # Step 2: if the hit sensor(s) coasted past the thin line during braking, back up slightly behind the
    # line and re-approach slowly so the triggered sensor stops right on the leading edge.
    coasted = (hit_l and get_red(left_color) >= LINE_THRESHOLD) or (hit_r and get_red(right_color) >= LINE_THRESHOLD)
    if coasted:
        check_s = left_color if hit_l and get_red(left_color) >= LINE_THRESHOLD else right_color
        l0, r0 = get_motor_positions()
        set_drive_dps(-crawl_dps, -crawl_dps)
        t_back = time.time()
        saw = False
        while time.time() - t_back < 1.2:
            l, r = get_motor_positions()
            if 0.5 * (abs(l - l0) + abs(r - r0)) > 90.0:
                break
            red = get_red(check_s)
            if red < LINE_THRESHOLD:
                saw = True
            elif saw and red >= LINE_CLEAR:
                break
            time.sleep(0.005)
        set_drive_dps(0, 0)
        time.sleep(0.05)
        # Slowly re-approach until at least one sensor hits the leading edge.
        hit_l = hit_r = False
        set_drive_dps(crawl_dps, crawl_dps)
        t_re = time.time()
        while time.time() - t_re < timeout_sec:
            hit_l = get_red(left_color) < LINE_THRESHOLD
            hit_r = get_red(right_color) < LINE_THRESHOLD
            if hit_l or hit_r:
                break
            time.sleep(0.005)
        set_drive_dps(0, 0)
        t_b2 = time.time()
        while time.time() - t_b2 < 0.06:
            if not hit_l and get_red(left_color) < LINE_THRESHOLD:
                hit_l = True
            if not hit_r and get_red(right_color) < LINE_THRESHOLD:
                hit_r = True
            time.sleep(0.005)
        if not (hit_l or hit_r):
            stop_drive()
            return False

    # Step 3: if only one sensor hit, pivot the opposite wheel forward until the second sensor hits.
    first = None
    if hit_l and not hit_r:
        first = "L"
        set_drive_dps(0, crawl_dps)
        t1 = time.time()
        while time.time() - t1 < timeout_sec:
            if get_red(right_color) < LINE_THRESHOLD:
                hit_r = True
                break
            time.sleep(0.005)
        set_drive_dps(0, 0)
        time.sleep(0.05)
    elif hit_r and not hit_l:
        first = "R"
        set_drive_dps(crawl_dps, 0)
        t1 = time.time()
        while time.time() - t1 < timeout_sec:
            if get_red(left_color) < LINE_THRESHOLD:
                hit_l = True
                break
            time.sleep(0.005)
        set_drive_dps(0, 0)
        time.sleep(0.05)

    if not (hit_l and hit_r):
        stop_drive()
        return False

    # Step 4: after a large pivot with rear-mounted sensors, the first sensor can swing a few mm BEHIND the
    # line. Gently nudge its wheel forward back onto the leading edge (bounded to ~1.5 cm).
    if first is not None:
        first_sensor = left_color if first == "L" else right_color
        if get_red(first_sensor) >= LINE_THRESHOLD:
            l0, r0 = get_motor_positions()
            set_drive_dps(crawl_dps if first == "L" else 0, 0 if first == "L" else crawl_dps)
            t_nudge = time.time()
            while time.time() - t_nudge < 0.8:
                l, r = get_motor_positions()
                moved = abs((l - l0) if first == "L" else (r - r0))
                if moved > 45.0 or get_red(first_sensor) < LINE_THRESHOLD:
                    break
                time.sleep(0.005)
            set_drive_dps(0, 0)
            time.sleep(0.04)

    time.sleep(0.08)
    stop_drive()
    return True


def autonomous_line_square(crawl_dps=CRAWL_DPS, timeout_sec=5.0):
    """Creeps forward until a sensor sees black, then squares both sensors onto the leading edge of the line."""
    t0 = time.time()
    hit_l = hit_r = False
    set_drive_dps(crawl_dps, crawl_dps)
    while time.time() - t0 < timeout_sec:
        hit_l = get_red(left_color) < LINE_THRESHOLD
        hit_r = get_red(right_color) < LINE_THRESHOLD
        if hit_l or hit_r:
            break
        time.sleep(0.005)
    if not (hit_l or hit_r):
        set_drive_dps(0, 0)
        time.sleep(0.06)
        stop_drive()
        return False
    return _square_after_hit(hit_l, hit_r, crawl_dps=crawl_dps, timeout_sec=timeout_sec)


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
    set_drive_dps(0, 0)
    time.sleep(0.1)
    stop_drive()


def auto_tune_straight(num_tiles=2):
    print("\n" + "=" * 65)
    print(f"  STEP 1: STRAIGHT ({num_tiles} tiles = {num_tiles * TILE_SIZE:.2f} cm)")
    print("=" * 65)
    print("Place the robot ~3 cm behind a black grid line, facing it. Starting in 2 s...")
    time.sleep(2.0)

    if not autonomous_line_square(crawl_dps=CRAWL_DPS):
        raise RuntimeError("Could not square on the starting grid line (check LINE_THRESHOLD and placement).")
    time.sleep(0.25)
    start_l, start_r = get_motor_positions()

    set_drive_dps(120, 120)
    seen_l = seen_r = 0
    # Both sensors start ON line 0: they must leave it before any line counts.
    on_l = on_r = True
    hit_l = hit_r = False
    t0 = time.time()
    while time.time() - t0 < 30:
        red_l, red_r = get_red(left_color), get_red(right_color)
        if red_l < LINE_THRESHOLD and not on_l:
            seen_l += 1
            on_l = True
            print(f"     [Left] line {seen_l}/{num_tiles}")
        elif red_l >= LINE_CLEAR:
            on_l = False
        if red_r < LINE_THRESHOLD and not on_r:
            seen_r += 1
            on_r = True
            print(f"     [Right] line {seen_r}/{num_tiles}")
        elif red_r >= LINE_CLEAR:
            on_r = False
        if seen_l >= num_tiles or seen_r >= num_tiles:
            hit_l = seen_l >= num_tiles
            hit_r = seen_r >= num_tiles
            break
        time.sleep(0.005)

    if not (hit_l or hit_r):
        set_drive_dps(0, 0)
        time.sleep(0.1)
        stop_drive()
        raise RuntimeError("Did not see the target line.")

    # Square physically onto line `num_tiles` so start and end headings are both 0 deg:
    # then left wheel distance == right wheel distance == num_tiles * TILE_SIZE regardless
    # of motor speed mismatch, sensor spacing, or track width.
    if not _square_after_hit(hit_l, hit_r, crawl_dps=CRAWL_DPS):
        raise RuntimeError("Could not square both sensors on the target grid line.")
    time.sleep(0.25)
    end_l, end_r = get_motor_positions()

    deg_l, deg_r = end_l - start_l, end_r - start_r
    if deg_l <= 0 or deg_r <= 0:
        raise RuntimeError(f"Unexpected encoder deltas L={deg_l:.1f}, R={deg_r:.1f} deg (check DRIVE_POLARITY).")
    dist = num_tiles * TILE_SIZE
    r_left = dist / math.radians(deg_l)
    r_right = dist / math.radians(deg_r)
    radius = 0.5 * (r_left + r_right)
    left_scale = r_left / radius
    right_scale = r_right / radius

    print(f"    Encoders: L={deg_l:.1f}, R={deg_r:.1f} deg  ->  WHEEL_RADIUS={radius:.4f} cm "
          f"(seed {WHEEL_RADIUS_SEED}), scales L={left_scale:.4f} R={right_scale:.4f}")
    return radius, left_scale, right_scale


def measure_track_width(wheel_radius, left_scale, right_scale, direction="CW", track_guess=TRACK_WIDTH_SEED):
    """One exact 360-degree turn between two squarings on the same line -> (track width, spin residual deg, ok)."""
    sign = 1.0 if direction == "CW" else -1.0
    if not autonomous_line_square(crawl_dps=CRAWL_DPS):
        print(f"    {direction}: could not square on the line before spinning")
        return track_guess, 999.0, False
    time.sleep(0.25)
    l0, r0 = get_motor_positions()

    # Open-loop spin to 360 deg using the latest track-width estimate and calibrated wheel scales.
    target_wheel_deg = (track_guess / (2.0 * wheel_radius)) * 360.0
    set_drive_dps(sign * 90 / left_scale, -sign * 90 / right_scale)
    t0, limit = time.time(), 3.0 + 2.0 * target_wheel_deg / 90.0
    while True:
        l, r = get_motor_positions()
        avg_scaled = 0.5 * (abs(l - l0) * left_scale + abs(r - r0) * right_scale)
        if avg_scaled >= target_wheel_deg:
            break
        if time.time() - t0 > limit:
            set_drive_dps(0, 0)
            stop_drive()
            raise RuntimeError(f"the spin didn't finish in {limit:.0f} s (wheels blocked, or the drive ports are wrong)")
        time.sleep(0.005)
    set_drive_dps(0, 0)
    time.sleep(0.15)
    lo, ro = get_motor_positions()

    # Back off so both sensors are behind the line (even for +-25 deg spin error), then square on it again.
    drive_encoder_distance(BACKOFF_CM, wheel_radius, -60)
    ok = autonomous_line_square(crawl_dps=CRAWL_DPS, timeout_sec=5.0)
    l1, r1 = get_motor_positions()
    if not ok:
        print(f"    {direction}: failed to re-square on the line after the spin (skipping)")
        return track_guess, 999.0, False

    dist_l = math.radians(l1 - l0) * wheel_radius * left_scale
    dist_r = math.radians(r1 - r0) * wheel_radius * right_scale
    width = sign * (dist_l - dist_r) / (2.0 * math.pi)
    if width <= 1.0:
        print(f"    {direction}: invalid width {width:.3f} cm (skipping)")
        return track_guess, 999.0, False
    # How far the open-loop spin alone turned, using the width just measured (aiming was for 360).
    spun = sign * math.degrees((math.radians(lo - l0) * left_scale - math.radians(ro - r0) * right_scale)
                               * wheel_radius / width)
    residual = 360.0 - spun
    print(f"    {direction}: dL={dist_l:.2f} cm, dR={dist_r:.2f} cm -> TRACK_WIDTH_{direction}={width:.3f} cm "
          f"(spin stopped {residual:+.1f} deg from 360)")
    return width, residual, True


def main():
    print("*" * 65)
    print("   ECSE 211 GRID-LINE ODOMETRY CALIBRATOR")
    print("*" * 65)
    core.wait_sensors_ready(need_colour=True)     # never hangs: names a port that sends nothing
    try:
        radius, ls, rs = auto_tune_straight(num_tiles=2)
        time.sleep(1.0)
        print("\n  STEP 2: TURNS. Leave the robot on/behind the grid line (it will back off and re-square).")
        widths = {}
        guess = TRACK_WIDTH_SEED
        for direction in ("CW", "CCW"):
            vals = []
            last = guess
            for i in range(TURN_REPEATS):
                print(f"\n  --- {direction} spin {i + 1}/{TURN_REPEATS} ---")
                # Make sure both sensors start behind the line before squaring for this spin.
                drive_encoder_distance(BACKOFF_CM, radius, -60)
                width, residual, ok = measure_track_width(radius, ls, rs, direction, guess)
                if ok:
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
