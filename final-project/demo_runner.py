#!/usr/bin/env python3
"""
ECSE 211 Final Design Project - Demo Runner & Hardware Tester
=============================================================
  [1] Full demo: Phase 1 -> standby (swap hardware, TA adds snow) -> Phase 2
  [2] Phase 1 only (saves street_map.json)
  [3] Phase 2 only (loads street_map.json)
  [4] Diagnostics with Phase 1 hardware (sensors, turret)
  [5] Diagnostics with Phase 2 hardware (sensors, roller, dump tray)
Fallback over SSH: the usual way to run all of this is the page of `python3 dashboard.py`.
"""

import time
import traceback
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


def read_sensors(n=5):
    for i in range(n):
        print(f"   [{i + 1}/{n}] L red={get_color_red(left_color):5.1f} | R red={get_color_red(right_color):5.1f} "
              f"| US={get_us_cm(samples=3):5.1f} cm")
        time.sleep(0.3)


def diagnostics(phase: int):
    init_hardware(phase=phase)
    print(f"\n--- PHASE {phase} HARDWARE DIAGNOSTICS ---")
    read_sensors()
    if phase == 1 and input("\nTurret scan L/F/R? (y/n): ").strip().lower() == "y":
        d_l = check_direction_distance(-90.0)
        d_f = check_direction_distance(0.0)
        d_r = check_direction_distance(90.0)
        print(f"   Left {d_l:.1f} | Forward {d_f:.1f} | Right {d_r:.1f} cm")
    if phase == 2:
        if input("\nRoller for 2 s? (y/n): ").strip().lower() == "y":
            start_intake_roller()
            time.sleep(2.0)
            stop_intake_roller()
        side = input("\nTip tray with motor A (+) or (-), or Enter to skip: ").strip()
        if side in ("+", "-"):
            dump_snow_payload(side)
    print("Diagnostics complete.")


def main():
    print("=" * 65)
    print("  ECSE 211 AUTONOMOUS SNOWPLOW - MASTER CONTROL MENU")
    print("=" * 65)
    print("  [1] Full demo (Phase 1 -> standby -> Phase 2)")
    print("  [2] Phase 1 only")
    print("  [3] Phase 2 only")
    print("  [4] Diagnostics, Phase 1 hardware")
    print("  [5] Diagnostics, Phase 2 hardware")
    choice = input("\nSelect [1-5, default 1]: ").strip()
    try:
        if choice in ("1", ""):
            run_phase1()
            stop_all_motors()
            print("\n" + "*" * 65)
            print("  PHASE 1 DONE. Swap to Phase 2 hardware while the TA adds snow.")
            print("*" * 65)
            run_phase2()
        elif choice == "2":
            run_phase1()
        elif choice == "3":
            run_phase2()
        elif choice == "4":
            diagnostics(1)
        elif choice == "5":
            diagnostics(2)
        else:
            print("Unknown selection.")
    except BaseException as e:
        print(f"\n[ABORTED]: {type(e).__name__}: {e}")
        traceback.print_exc()
    finally:
        stop_all_motors()
        reset_brick()


if __name__ == "__main__":
    main()
