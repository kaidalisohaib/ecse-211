#!/usr/bin/env python3
"""
ECSE 211 Final Design Project - Unified Demo Runner & Hardware Tester
=====================================================================
Provides a single menu to:
  [1] Run Full Two-Phase Demo (Phase 1 Mapping -> Pause at (0,0) -> Phase 2 Snow Removal)
  [2] Run Phase 1 Only (Street Mapping -> saves `street_map.json`)
  [3] Run Phase 2 Only (Snow Removal -> loads `street_map.json`)
  [4] Hardware Diagnostic & Port Tester (Turret, Roller, Dump Gate, Color & US Sensors)
"""

import time
from phase1_mapping import run_phase1
from phase2_snow_removal import run_phase2
from snowplow_core import (
     check_direction_distance,
    dump_snow_payload,
    get_color_red,
    get_us_cm,
    init_hardware,
    left_color,
    reset_brick,
    right_color,
    start_intake_roller,
    stop_all_motors,
    stop_intake_roller,
)


def run_hardware_diagnostics():
    """Interactive bench test for sensors, turret, roller, and dump actuator."""
    init_hardware()
    print("\n--- HARDWARE DIAGNOSTICS ---")
    print("1. Reading Sensors (5 iterations):")
    for i in range(5):
        rl = get_color_red(left_color)
        rr = get_color_red(right_color)
        us = get_us_cm(samples=3)
        print(f"   [{i+1}/5] Left Color Red={rl:3d} | Right Color Red={rr:3d} | Ultrasonic={us:5.1f} cm")
        time.sleep(0.3)

    ans = input("\nTest Turret [-90°, 0°, +90°] scan on Port A? (y/n): ").strip().lower()
    if ans == "y":
        d_l = check_direction_distance(-90.0)
        d_f = check_direction_distance(0.0)
        d_r = check_direction_distance(90.0)
        print(f"   Turret Scan -> Left: {d_l:.1f} cm | Forward: {d_f:.1f} cm | Right: {d_r:.1f} cm")

    ans = input("\nTest Intake Roller on Port D for 2 seconds? (y/n): ").strip().lower()
    if ans == "y":
        start_intake_roller()
        time.sleep(2.0)
        stop_intake_roller()

    ans = input("\nTest Dump Gate / Ejector on Port A? (y/n): ").strip().lower()
    if ans == "y":
        dump_snow_payload()

    print("Diagnostics complete!")


def main():
    print("=" * 65)
    print("  ECSE 211 AUTONOMOUS SNOWPLOW - MASTER CONTROL MENU")
    print("=" * 65)
    print("  [1] Full Demo (Phase 1 Mapping -> Standby Pause -> Phase 2 Removal)")
    print("  [2] Phase 1 Only (Summer Mapping -> saves street_map.json)")
    print("  [3] Phase 2 Only (Winter Snow Removal -> loads street_map.json)")
    print("  [4] Hardware & Sensor Diagnostics")
    choice = input("\nSelect option [1/2/3/4, default=1]: ").strip()

    try:
        if choice in ("1", ""):
            run_phase1()
            print("\n" + "*" * 65)
            print("  PHASE 1 FINISHED. Wait for TA to place snow in the street.")
            print("*" * 65)
            run_phase2()
        elif choice == "2":
            run_phase1()
        elif choice == "3":
            run_phase2()
        elif choice == "4":
            run_hardware_diagnostics()
        else:
            print("Unknown selection.")
    except (KeyboardInterrupt, SystemExit, BaseException) as e:
        print(f"\n[ABORTED]: {e}")
    finally:
        stop_all_motors()
        reset_brick()


if __name__ == "__main__":
    main()
