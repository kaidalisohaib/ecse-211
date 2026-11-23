#!/usr/bin/env python3
"""
ECSE 211 Final Design Project - Phase 1: Autonomous Street Mapping
==================================================================
Mission Procedure (<= 120 seconds):
  1. Accepts runtime coordinates for street entrance (s1, s2) and drop-off (d1, d2).
  2. Departs from Origin (0,0), steps onto Tile (0,0) center, and navigates with
     ultrasonic obstacle avoidance to the street entrance between s1 and s2.
  3. Explores the 1-tile-wide street corridor along tile centerlines:
     - Uses dual color sensors to square on every perpendicular grid line.
     - Uses forward ultrasonic + Port A turret [-90°, +90°] to detect corners & walls.
  4. Saves the complete street trajectory & wall layout to `street_map.json`.
  5. Returns autonomously to Origin (0,0) before the 120s watchdog deadline and
     waits for Phase 2 snow placement.
"""

import json
import time
from snowplow_core import (
    CRUISE_DPS,
    EXPLORE_DPS,
    HEADING_TO_DELTA,
    MAP_FILE,
    WALL_THRESHOLD_CM,
    check_direction_distance,
    find_shortest_tile_path,
    init_hardware,
    is_tile_in_bounds,
    navigate_tile_with_obstacle_avoidance,
    parse_gate_geometry,
    reset_brick,
    should_abort_for_time,
    step_one_tile_center,
    step_origin_to_start_tile,
    step_tile_to_origin,
    stop_all_motors,
    turn_to,
)

# Default coordinates from Figure 1 (Sample Map)
DEFAULT_S1 = (1, 3)
DEFAULT_S2 = (1, 2)
DEFAULT_D1 = (2, -1)
DEFAULT_D2 = (3, -1)


def prompt_point(label: str, default_pt: tuple[int, int]) -> tuple[int, int]:
    """Prompts operator for grid intersection coordinates 'x y' (or Enter for default)."""
    raw = input(f"Enter {label} coordinates as 'x y' [default {default_pt[0]} {default_pt[1]}]: ").strip()
    if not raw:
        return default_pt
    parts = raw.replace(",", " ").split()
    return int(parts[0]), int(parts[1])


def resolve_dropoff_pose(d1: tuple[int, int], d2: tuple[int, int]) -> tuple[tuple[int, int], int]:
    """
    Determines which in-bounds tile borders the drop-off segment (d1, d2)
    and the outward heading facing the collection bucket on the table edge.
    """
    gate = parse_gate_geometry(d1, d2)
    for cand in gate["candidates"]:
        # If first_tile is OUTSIDE the board and approach_tile is INSIDE the board,
        # then approach_tile is the drop-off tile and cand["heading"] points into the bucket!
        if is_tile_in_bounds(*cand["approach_tile"]) and not is_tile_in_bounds(*cand["first_tile"]):
            return cand["approach_tile"], cand["heading"]

    # Fallback: pick candidate whose approach_tile is inside bounds
    for cand in gate["candidates"]:
        if is_tile_in_bounds(*cand["approach_tile"]):
            return cand["approach_tile"], cand["heading"]
    return (2, -1), 180


def select_entrance_candidate(s1: tuple[int, int], s2: tuple[int, int]) -> dict:
    """
    Selects the primary entrance approach tile and entry heading for gate (s1, s2).
    Prefers the approach tile closest in Manhattan distance to Tile (0,0).
    """
    gate = parse_gate_geometry(s1, s2)
    valid = [
        c for c in gate["candidates"]
        if is_tile_in_bounds(*c["approach_tile"]) and is_tile_in_bounds(*c["first_tile"])
    ]
    if not valid:
        valid = [c for c in gate["candidates"] if is_tile_in_bounds(*c["first_tile"])]
    valid.sort(key=lambda c: abs(c["approach_tile"][0]) + abs(c["approach_tile"][1]))
    return valid[0]


def explore_street_corridor(
    start_tile: tuple[int, int],
    initial_heading: int,
    approach_tile: tuple[int, int],
    blocked_tiles: set[tuple[int, int]],
    phase_start_time: float,
) -> list[tuple[int, int]]:
    """
    Traverses the contiguous 1-tile street starting at `start_tile` with `initial_heading`.
    Records each street tile in order and identifies corners/walls using the ultrasonic turret.
    """
    street_tiles = [start_tile]
    visited_street = {approach_tile, start_tile}
    current_tile = start_tile
    current_heading = initial_heading % 360

    print(f"\n[MAP] Entered Street at Tile {current_tile}, heading={current_heading}°")

    max_street_steps = 35
    for _ in range(max_street_steps):
        if should_abort_for_time(phase_start_time, current_tile):
            break

        turn_to(float(current_heading))

        # Check Left (-90°), Forward (0°), and Right (+90°) using turret
        d_fwd = check_direction_distance(0.0)
        d_right = check_direction_distance(90.0)
        d_left = check_direction_distance(-90.0)

        fwd_open = d_fwd >= WALL_THRESHOLD_CM
        right_open = d_right >= WALL_THRESHOLD_CM
        left_open = d_left >= WALL_THRESHOLD_CM

        # Record detected adjacent house walls into blocked_tiles
        h_right = (current_heading + 90) % 360
        h_left = (current_heading - 90) % 360
        tile_fwd = (
            current_tile[0] + HEADING_TO_DELTA[current_heading][0],
            current_tile[1] + HEADING_TO_DELTA[current_heading][1],
        )
        tile_right = (
            current_tile[0] + HEADING_TO_DELTA[h_right][0],
            current_tile[1] + HEADING_TO_DELTA[h_right][1],
        )
        tile_left = (
            current_tile[0] + HEADING_TO_DELTA[h_left][0],
            current_tile[1] + HEADING_TO_DELTA[h_left][1],
        )

        if not fwd_open and is_tile_in_bounds(*tile_fwd):
            blocked_tiles.add(tile_fwd)
        if not right_open and is_tile_in_bounds(*tile_right):
            blocked_tiles.add(tile_right)
        if not left_open and is_tile_in_bounds(*tile_left):
            blocked_tiles.add(tile_left)

        print(
            f"  [SCAN @ Tile {current_tile} | θ={current_heading}°] "
            f"L={d_left:.0f}cm ({'OPEN' if left_open else 'WALL'}), "
            f"F={d_fwd:.0f}cm ({'OPEN' if fwd_open else 'WALL'}), "
            f"R={d_right:.0f}cm ({'OPEN' if right_open else 'WALL'})"
        )

        # Exit condition 1: If we stepped out of the walled street (both Left and Right have no walls)
        if len(street_tiles) > 1 and left_open and right_open:
            print(f"[MAP] Both side walls ended at Tile {current_tile} -> Street exit reached!")
            break

        # Decision 1: Keep going straight if forward is open and inside table bounds
        if fwd_open and is_tile_in_bounds(*tile_fwd) and tile_fwd not in visited_street:
            current_tile = step_one_tile_center(current_tile, current_heading, speed_dps=EXPLORE_DPS)
            street_tiles.append(current_tile)
            visited_street.add(current_tile)
            continue

        # Decision 2: Forward is blocked (or out of bounds) -> 90° Corner! Check Right then Left
        if right_open and is_tile_in_bounds(*tile_right) and tile_right not in visited_street:
            print(f"  [CORNER] Turning RIGHT (90° CW) at Tile {current_tile} -> {tile_right}")
            current_heading = h_right
            current_tile = step_one_tile_center(current_tile, current_heading, speed_dps=EXPLORE_DPS)
            street_tiles.append(current_tile)
            visited_street.add(current_tile)
            continue

        if left_open and is_tile_in_bounds(*tile_left) and tile_left not in visited_street:
            print(f"  [CORNER] Turning LEFT (90° CCW) at Tile {current_tile} -> {tile_left}")
            current_heading = h_left
            current_tile = step_one_tile_center(current_tile, current_heading, speed_dps=EXPLORE_DPS)
            street_tiles.append(current_tile)
            visited_street.add(current_tile)
            continue

        # Exit condition 2: No further unvisited open corridor directions
        print(f"[MAP] Reached terminal end of street at Tile {current_tile}!")
        break

    return street_tiles


def save_street_map(
    s1: tuple[int, int],
    s2: tuple[int, int],
    d1: tuple[int, int],
    d2: tuple[int, int],
    approach_tile: tuple[int, int],
    entry_heading: int,
    street_tiles: list[tuple[int, int]],
    blocked_tiles: set[tuple[int, int]],
    dropoff_tile: tuple[int, int],
    dropoff_heading: int,
):
    """Serializes the discovered street topology to street_map.json for Phase 2."""
    payload = {
        "s1": list(s1),
        "s2": list(s2),
        "d1": list(d1),
        "d2": list(d2),
        "approach_tile": list(approach_tile),
        "entry_heading": entry_heading,
        "street_tiles": [list(t) for t in street_tiles],
        "blocked_tiles": [list(t) for t in sorted(blocked_tiles)],
        "dropoff_tile": list(dropoff_tile),
        "dropoff_heading": dropoff_heading,
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    with open(MAP_FILE, "w") as f:
        json.dump(payload, f, indent=2)
    print(f"\n[SAVE] Saved {len(street_tiles)} street tiles to '{MAP_FILE}'.")


def run_phase1():
    print("=" * 65)
    print("  ECSE 211 FINAL PROJECT - PHASE 1: SUMMER STREET MAPPING")
    print("=" * 65)

    s1 = prompt_point("Entrance s1", DEFAULT_S1)
    s2 = prompt_point("Entrance s2", DEFAULT_S2)
    d1 = prompt_point("Drop-off d1", DEFAULT_D1)
    d2 = prompt_point("Drop-off d2", DEFAULT_D2)

    ent = select_entrance_candidate(s1, s2)
    approach_tile = ent["approach_tile"]
    first_street_tile = ent["first_tile"]
    entry_heading = ent["heading"]
    dropoff_tile, dropoff_heading = resolve_dropoff_pose(d1, d2)

    print(f"\n[PLAN] Entrance Gate: {s1}-{s2} -> Approach Tile {approach_tile}, First Street Tile {first_street_tile} @ {entry_heading}°")
    print(f"[PLAN] Drop-off Edge: {d1}-{d2} -> Drop-off Tile {dropoff_tile} facing {dropoff_heading}°")
    input("Place robot at Origin (0,0) facing +Y (North) and press Enter to start Phase 1...")

    init_hardware()
    phase_start_time = time.time()
    blocked_tiles: set[tuple[int, int]] = set()

    # 1. Step from Origin (0,0) intersection onto Tile (0,0) center
    current_tile = (0, 0)
    step_origin_to_start_tile(current_tile)

    # 2. Navigate safely to the entrance approach tile
    print(f"\n[NAV] Navigating to Entrance Approach Tile {approach_tile}...")
    current_tile = navigate_tile_with_obstacle_avoidance(
        current_tile, approach_tile, blocked_tiles, speed_dps=CRUISE_DPS, check_obstacles=True
    )

    # 3. Step through entrance gate (s1, s2) into first_street_tile
    print(f"\n[NAV] Entering street through ({s1}, {s2}) into Tile {first_street_tile}...")
    current_tile = step_one_tile_center(current_tile, entry_heading, speed_dps=EXPLORE_DPS)

    # 4. Explore the street corridor
    street_tiles = explore_street_corridor(
        start_tile=current_tile,
        initial_heading=entry_heading,
        approach_tile=approach_tile,
        blocked_tiles=blocked_tiles,
        phase_start_time=phase_start_time,
    )
    current_tile = street_tiles[-1]

    # 5. Save discovered street map for Phase 2
    save_street_map(
        s1, s2, d1, d2,
        approach_tile, entry_heading,
        street_tiles, blocked_tiles,
        dropoff_tile, dropoff_heading,
    )

    # 6. Return to Origin (0,0) via shortest path
    print(f"\n[RETURN] Navigating from Tile {current_tile} back to Origin Tile (0,0)...")
    # Ensure street tiles are never marked blocked
    blocked_tiles.difference_update(street_tiles)
    current_tile = navigate_tile_with_obstacle_avoidance(
        current_tile, (0, 0), blocked_tiles, speed_dps=CRUISE_DPS, check_obstacles=True
    )
    step_tile_to_origin()

    total_time = time.time() - phase_start_time
    print("\n" + "=" * 65)
    print(f"  PHASE 1 COMPLETE in {total_time:.1f}s (Limit: 120.0s)!")
    print("  Robot is in STANDBY at Origin (0,0). Ready for Phase 2.")
    print("=" * 65)


if __name__ == "__main__":
    try:
        run_phase1()
    except (KeyboardInterrupt, SystemExit, BaseException) as e:
        print(f"\n[PHASE 1 TERMINATED]: {e}")
    finally:
        stop_all_motors()
        reset_brick()
