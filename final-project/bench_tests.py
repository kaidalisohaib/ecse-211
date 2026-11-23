#!/usr/bin/env python3
"""
Step-by-step bench tests for the snowplow, in the order to run them on a new build.
Each test says what to look at and which constant in snowplow_core.py / phase*.py to change.

  python3 bench_tests.py          (menu)
  python3 bench_tests.py 3        (run test 3 directly)

Tile positions below are board tiles; the robot is told where it is, so put it there by hand
(axle over the tile centre, facing the given direction). Watch it live at http://<robot-ip>:8080.
"""
from __future__ import annotations
import math
import statistics
import sys
import time

import snowplow_core as core
import vizlog
from utils.brick import reset_brick

T = core.TILE_SIZE


def ask(msg):
    return input(f"\n>>> {msg} ").strip().lower()


def set_pose(tile, heading):
    """Tells odometry the robot is on tile's centre facing heading (0 = +Y, 90 = +X)."""
    core.update_odometer()
    core.x, core.y = core.tile_to_center_cm(*tile)
    core.theta = float(heading)
    core._mark_heading_reference(1.0)


def pose_str():
    core.update_odometer()
    return f"x {core.x:6.1f}  y {core.y:6.1f}  heading {core.theta:5.1f}"


def red_samples(n=40):
    L, R = [], []
    for _ in range(n):
        L.append(core.get_color_red(core.left_color))
        R.append(core.get_color_red(core.right_color))
        time.sleep(0.01)
    return statistics.median(L), statistics.median(R)


# ------------------------------------------------------------------ 1
def t_colour():
    """Colour sensors: pick LINE_THRESHOLD."""
    core.init_hardware(phase=2)
    ask("Put BOTH colour sensors over plain table (no line). Enter")
    tl, tr = red_samples()
    ask("Put BOTH colour sensors on a black grid line. Enter")
    ll, lr = red_samples()
    print(f"   table: L {tl:.0f}  R {tr:.0f}     line: L {ll:.0f}  R {lr:.0f}")
    thr = 0.5 * (min(tl, tr) + max(ll, lr))
    print(f"   -> LINE_THRESHOLD = {thr:.0f}   (now {core.LINE_THRESHOLD}; snowplow_core.py)")
    if min(tl, tr) - max(ll, lr) < 15:
        print("   !! Table and line are less than 15 apart: lower the sensors (about 0.5-1 cm above the table).")
    if ask("Optional: put a DARK pom-pom under the left sensor (Enter to skip, y = measure)") == "y":
        pl, _ = red_samples()
        print(f"   pom-pom: {pl:.0f} -> {'reads as a LINE, snaps may be fooled' if pl < thr else 'ok, above the threshold'}")


# ------------------------------------------------------------------ 2
def t_drive_direction():
    """Drive motor polarity and left/right ports."""
    core.init_hardware(phase=2)
    ask("Robot on the floor with 20 cm free in front. It will drive FORWARD 10 cm slowly. Enter")
    core.drive_straight_distance(10.0, True, hold_heading=0.0, speed_dps=120)
    print(f"   odometry after: {pose_str()}  (expect y ~ +10)")
    a = ask("Did it go forward? (y/n)")
    if a == "n":
        print(f"   -> flip DRIVE_POLARITY (now {core.DRIVE_POLARITY}) in snowplow_core.py, then rerun this test.")
        return
    ask("Now it turns RIGHT (clockwise seen from above) by 90 deg. Enter")
    core.turn_to(90.0)
    print(f"   odometry after: {pose_str()}  (expect heading ~ 90)")
    if ask("Did it turn clockwise? (y/n)") == "n":
        print("   -> swap LEFT_DRIVE_PORT and RIGHT_DRIVE_PORT (or the cables).")
    core.turn_to(0.0)


# ------------------------------------------------------------------ 3
def t_turret_and_us():
    """Turret direction (Phase 1 hardware) and ultrasonic accuracy."""
    core.init_hardware(phase=1)
    print("   The turret's zero is wherever it points now: it must point straight ahead.")
    for ang, name in ((90, "RIGHT"), (-90, "LEFT"), (0, "FORWARD")):
        core.set_turret_angle(ang)
        print(f"   turret -> {ang:+d} (should point {name}), US {core.get_us_cm(samples=3):.1f} cm")
        time.sleep(0.8)
    if ask("Did +90 point RIGHT? (y/n)") == "n":
        print(f"   -> TURRET_GEAR_RATIO = {-core.TURRET_GEAR_RATIO} (sign flip; also set the gear ratio if geared)")
    print("   Distance check: put a box face square to the sensor; enter the true distance from the")
    print("   sensor face in cm (q to stop).")
    while True:
        a = ask("true distance cm (q = done):")
        if a in ("q", ""):
            break
        try:
            true = float(a)
        except ValueError:
            continue
        r = [core.get_us_cm(samples=1) for _ in range(15)]
        med = statistics.median(r)
        print(f"   read median {med:.1f} (spread {min(r):.1f}-{max(r):.1f}), error {med - true:+.1f} cm")
    print("   A constant error goes into US_FACE_CM. Houses are ~12.7 cm away when centred in a street:")
    print("   if readings there are off by more than ~1 cm or jump around, set USE_US_LATERAL_FIX = False.")
    # How often the sensor really pings: reading faster than that just returns the same ping again, so a
    # "median of 5" would only be a median of 1 or 2 real readings.
    ask("Update rate: wave a hand slowly 10-40 cm in front of the sensor for 3 s. Enter to start")
    vals, t0 = [], time.time()
    while time.time() - t0 < 3.0:
        try:
            vals.append((time.time(), core.us_sensor.get_cm()))
        except Exception:
            pass
    changes = [t for (t, v), (_, prev) in zip(vals[1:], vals) if v != prev]
    gaps = sorted(b - a for a, b in zip(changes, changes[1:]))
    if len(gaps) < 5:
        print(f"   {len(vals)} reads, only {len(changes)} changes: move the hand more and run [3] again")
        return
    ping = gaps[len(gaps) // 2]
    print(f"   {len(vals) / 3:.0f} reads/s, value changed {len(changes) / 3:.0f} times/s, median gap {1000 * ping:.0f} ms")
    print(f"   -> US_SPACING_SEC = {max(0.01, round(ping, 3))} (now {core.US_SPACING_SEC}); a scan of "
          f"{core.US_SCAN_SAMPLES} then takes about {1000 * core.US_SCAN_SAMPLES * max(ping, 0.01):.0f} ms per direction")


# ------------------------------------------------------------------ 4
def t_turns():
    """In-place turns (after auto_tuner.py)."""
    core.init_hardware(phase=2)
    ask("Robot on a tile centre, sensors clear of lines, facing +Y. It turns 90, 180, 270, 0. Enter")
    set_pose((0, 0), 0)
    for h in (90.0, 180.0, 270.0, 0.0):
        t0 = time.time()
        core.turn_to(h)
        print(f"   target {h:5.1f}: odometry {core.theta:5.1f}, took {time.time() - t0:.1f} s")
        time.sleep(0.6)
    print("   Check the final heading against a grid line. Off by > 2 deg after 360: rerun auto_tuner.py.")
    print("   Overshoot/wobble: lower KP_TURN or TURN_DPS_MAX. Slow to finish: raise TURN_DPS_MIN.")


# ------------------------------------------------------------------ 5
def t_straight_run(phase=2, speed=None):
    """Straight run with flying line snaps: where does it stop?"""
    core.init_hardware(phase=phase)
    speed = speed or (core.SWEEP_DPS if phase == 2 else core.EXPLORE_DPS)
    ask("Axle over a tile centre, facing +Y, 3 free tiles ahead. Enter")
    set_pose((0, 0), 0)
    vizlog.start(phase, log_dir=None, outline=core.ROBOT_OUTLINE[phase])
    res = core.drive_run((0, 0), 0, 3, speed)
    print(f"   stopped: {pose_str()}  (target x 15.2, y {3.5 * T:.1f}), reason {res['stop_reason']}")
    vizlog.close()
    print("   Measure: axle vs the tile centre. Along error > 1.5 cm: check SENSOR_OFFSET_BY_PHASE (axle to")
    print("   sensors, - = behind). Sideways error > 1.5 cm or a slow S: KP_DRIVE_HEADING / K_LATERAL_DEG_PER_CM.")
    print("   No snaps in the log: LINE_THRESHOLD, LINE_CROSS_MAX_DPS (slower = surer).")


# ------------------------------------------------------------------ 6
def t_square_loop():
    """Odometry + snaps over a loop: home error."""
    core.init_hardware(phase=2)
    ask("Axle over a tile centre facing +Y with a free 3 x 3 tile area ahead and to the right. Enter")
    set_pose((0, 0), 0)
    vizlog.start(2, log_dir=None, outline=core.ROBOT_OUTLINE[2])
    for h, tile in ((0, (0, 0)), (90, (0, 2)), (180, (2, 2)), (270, (2, 0))):
        core.drive_run(tile, h, 2, core.CRUISE_DPS)
    core.turn_to(0.0)
    print(f"   back home: {pose_str()}  (odometry thinks x 15.2, y 15.2)")
    vizlog.close()
    print("   Measure the true axle position: > 2 cm off means calibration (auto_tuner.py) or snaps need work.")


# ------------------------------------------------------------------ 7
def t_side_us_fix():
    """Is the side-wall ultrasonic good enough to correct sideways position?"""
    core.init_hardware(phase=1)
    print("   Build one street tile: two boxes 30.48 cm apart. Put the axle on the tile centre, facing +Y.")
    rows = []
    while True:
        a = ask("true sideways offset of the axle, cm (+ = right of centre; q = done):")
        if a in ("q", ""):
            break
        try:
            true = float(a)
        except ValueError:
            continue
        set_pose((0, 0), 0)
        core.USE_US_LATERAL_FIX = False
        L = core.check_direction_distance(-90.0)
        R = core.check_direction_distance(90.0)
        est = 0.5 * (core._side_reading_to_lateral(-1, L, 0.0) + core._side_reading_to_lateral(1, R, 0.0))
        rows.append(est - true)
        print(f"   L {L:.1f}  R {R:.1f}  -> ultrasonic says {est:+.1f} cm, error {est - true:+.1f} cm")
    if rows:
        print(f"   mean error {statistics.mean(rows):+.1f} cm, worst {max(rows, key=abs):+.1f} cm")
        print("   Worst > 1 cm: set USE_US_LATERAL_FIX = False (Phase 1 then relies on odometry + snaps, which")
        print("   the sim shows is enough). A constant offset: fix US_FACE_CM first.")
    core.set_turret_angle(0.0)


# ------------------------------------------------------------------ 8
def t_roller():
    """Roller direction, speed and catch."""
    core.init_hardware(phase=2)
    speed = f"power {core.ROLLER_SWEEP_POWER}" if core.ROLLER_SWEEP_POWER else f"{core.ROLLER_SWEEP_DPS} dps"
    ask(f"Belt runs 3 s at {speed}: its underside must move BACKWARD (pom-poms carried up into the bin). Enter")
    core.start_intake_roller()
    time.sleep(3.0)
    core.stop_intake_roller()
    if ask("Did the underside move toward the robot? (y/n)") == "n":
        if core.ROLLER_SWEEP_POWER:
            print(f"   -> ROLLER_SWEEP_POWER = {-core.ROLLER_SWEEP_POWER}")
        else:
            print(f"   -> ROLLER_SWEEP_DPS = {-core.ROLLER_SWEEP_DPS}")
        return
    if ask("Catch test: scatter 20 pom-poms over 2 tiles ahead (some 2 cm from the side). Run? (y/n)") == "y":
        set_pose((0, 0), 0)
        core.start_intake_roller()
        core.drive_run((0, 0), 0, 2, core.SWEEP_DPS)
        time.sleep(0.8)
        core.stop_intake_roller()
        print(f"   Count what's in the tray. < 19/20: slow SWEEP_DPS (now {core.SWEEP_DPS}) or change")
        print(f"   the belt speed ({speed}); jams: check the funnel throat / lip height.")


# ------------------------------------------------------------------ 9
def t_dump():
    """Find DUMP_TILT_DEG and where motor + / - pours (DUMP_PLUS / DUMP_MINUS) by jogging the arm."""
    core.init_hardware(phase=2)
    base = core.dump_motor.get_position() or 0
    core.dump_motor.set_limits(power=85, dps=200)
    print("   Enter an arm angle in motor degrees (relative to now), + or -; q = done.")
    print(f"   Now: DUMP_PLUS = {core.DUMP_PLUS!r}, DUMP_MINUS = {core.DUMP_MINUS!r}, DUMP_TILT_DEG = {core.DUMP_TILT_DEG}")
    seen = {}
    while True:
        a = ask("angle (q = done):")
        if a in ("q", ""):
            break
        try:
            last = int(float(a))
        except ValueError:
            continue
        core._move_aux_and_wait(base + last, timeout=3.0)
        print(f"   at {last:+d}")
        if last:
            way = ask("Which way does the tray pour here? (r)ight, (l)eft, (b)ack, (n)owhere / Enter = skip:")
            if way in ("r", "l", "b", "n"):
                seen[1 if last > 0 else -1] = ({"r": "RIGHT", "l": "LEFT", "b": "BACK", "n": None}[way], abs(last))
        core._move_aux_and_wait(base, timeout=3.0)
    core._move_aux_and_wait(base, timeout=3.0)
    core.dump_motor.set_power(0)
    for sign, (where, deg) in sorted(seen.items(), reverse=True):
        name = "DUMP_PLUS" if sign > 0 else "DUMP_MINUS"
        print(f"   -> {name} = {where!r}  (now {getattr(core, name)!r}); tipped at {deg} deg")
    if seen:
        print(f"   -> DUMP_TILT_DEG = {max(d for _, d in seen.values())}  (now {core.DUMP_TILT_DEG}): one angle for "
              f"both ways, so the larger of the angles you said empty the tray")
    if any(w == "BACK" for w, _ in seen.values()):
        print("   Pours back: also measure DUMP_BACK_LIP_CM (axle to rear lip) and DUMP_BACK_SUPPORT_CM (axle to the")
        print("   rearmost ground contact) in phase2_snow_removal.py.")
    way = ask("Run the full dump now? (+) or (-), Enter = skip:")
    if way in ("+", "-"):
        core.dump_snow_payload(way)


# ------------------------------------------------------------------ 10
def t_corner_arc():
    """One street corner on an arc (Phase 2 hardware, boxes around a corner)."""
    core.init_hardware(phase=2)
    d = ask("Corner turning (r)ight or (l)eft?")
    out = 90 if d != "l" else 270
    print("   Robot on the tile centre BEFORE the corner tile, facing +Y; the corner tile is the next one.")
    ask("Houses placed around the corner as on the board. Enter to drive")
    set_pose((0, 0), 0)
    vizlog.start(2, log_dir=None, outline=core.ROBOT_OUTLINE[2])
    core.start_intake_roller()
    core.drive_run((0, 0), 0, 1, core.SWEEP_DPS, end_lane_offset_cm=core.arc_lane_offset(0, out),
                   end_along_offset_cm=-T / 2.0)
    if ask("Poke into the corner first? (y/n)") == "y":
        core.drive_straight_distance(17.0, True, hold_heading=0, speed_dps=core.SWEEP_DPS)
        core.drive_straight_distance(17.0, False, hold_heading=0, speed_dps=core.CRUISE_DPS)
    core.drive_arc((0, 1), 0, out, core.SWEEP_DPS)
    core.drive_run((0, 1), out, 1, core.SWEEP_DPS)
    core.stop_intake_roller()
    vizlog.close()
    print(f"   ended: {pose_str()}")
    print("   Brushed a house on the outside of the turn: ARC_RADIUS_CM smaller (e.g. 14.5 -> 14). Inside: larger.")
    print("   Wobbles on the arc: lower ARC_KP_HEADING; drifts off the arc: raise ARC_K_RADIAL_DEG_PER_CM.")


TESTS = [
    ("Colour sensors -> LINE_THRESHOLD", t_colour),
    ("Drive direction + left/right ports", t_drive_direction),
    ("Turret direction + ultrasonic accuracy (Phase 1 hw)", t_turret_and_us),
    ("In-place turns (run auto_tuner.py first)", t_turns),
    ("Straight 3-tile run with line snaps", t_straight_run),
    ("Square loop, home error", t_square_loop),
    ("Side ultrasonic: is the lateral fix worth it? (Phase 1 hw)", t_side_us_fix),
    ("Roller direction + catch test (Phase 2 hw)", t_roller),
    ("Dump arm angle (Phase 2 hw)", t_dump),
    ("Street corner on an arc (Phase 2 hw)", t_corner_arc),
]


def main():
    if len(sys.argv) > 1:
        picks = [sys.argv[1]]
    else:
        print("=" * 60 + "\n  SNOWPLOW BENCH TESTS (run them in this order on a new build)\n" + "=" * 60)
        for i, (name, _) in enumerate(TESTS, 1):
            print(f"  [{i:2d}] {name}")
        print("  Between 2 and 4: run  python3 auto_tuner.py")
        picks = [input("\nTest number: ").strip()]
    try:
        for p in picks:
            name, fn = TESTS[int(p) - 1]
            print(f"\n--- {name} ---")
            fn()
    except (KeyboardInterrupt, EOFError):
        print("\nStopped.")
    finally:
        core.stop_all_motors()
        vizlog.close()
        reset_brick()


if __name__ == "__main__":
    main()
