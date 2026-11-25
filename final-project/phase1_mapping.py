#!/usr/bin/env python3
"""
ECSE 211 Final Design Project - Phase 1: Autonomous Street Mapping
==================================================================
  1. Reads s1, s2, d1, d2 at runtime.
  2. Origin -> tile (0,0) -> the tile just outside the s1-s2 gate (forward-ultrasonic
     obstacle checks; runs are shortened when a house is in the way).
  3. Checks which side of the gate is the street (houses on both sides = street).
  4. Explores the street in straight *runs* (EXPLORE_MODE "forward", default): the turret
     looks forward while driving and the robot stops one tile short of a close house
     (that is where the street turns). Once per tile it glances to one side, so it also
     stops where the side houses end (the street exit). At each stop: one L/F/R scan.
     EXPLORE_MODE "side" = the older scheme: the turret watches a side wall during runs
     and forward distance comes from the stationary scans.
  5. Saves street_map.json (houses next to the street are inferred), then returns home via
     the drop-off tile so Phase 2 can reuse a path that has been driven.
"""
from __future__ import annotations

import json
import time
import traceback
import snowplow_core as core
import vizlog
from snowplow_core import (
    CRUISE_DPS,
    EXPLORE_DPS,
    MAP_FILE,
    MAX_RUN_TILES,
    WALL_THRESHOLD_CM,
    drive_run,
    estimate_path_seconds,
    free_tiles_ahead,
    init_hardware,
    is_tile_in_bounds,
    navigate_tile_with_obstacle_avoidance,
    parse_gate_geometry,
    plan_dump_approach,
    plan_path,
    reset_brick,
    scan_walls,
    should_abort_for_time,
    step_origin_to_start_tile,
    step_tile_to_origin,
    stop_all_motors,
    heading_between_adjacent_tiles,
    tile_step,
    turn_to,
)

# Sample-map coordinates (Figure 1). Only used if the operator just presses Enter.
DEFAULT_S1 = (1, 3)
DEFAULT_S2 = (1, 2)
DEFAULT_D1 = (2, -1)
DEFAULT_D2 = (3, -1)

SCOUT_DROPOFF_ON_RETURN = False  # Drive through the drop-off tile on the way home (if time allows). Not needed while
                                 # Phase 2's TRUST_MAP is on (it plans on any tile that isn't a street wall); turn
                                 # it on if TRUST_MAP is off. No stop there when the way home passes through it
EXPLORE_MODE = "forward"         # "forward" (watch ahead + per-tile side glance) or "side" (watch a side wall)
MAX_FORWARD_RUN_TILES = 8        # "forward" mode: longest run before a precautionary stop
SKIP_KNOWN_READINGS = True       # at stops, only measure directions whose answer isn't known yet, and turn
                                 # without looking when only one way is possible. At the gate, one side reading
                                 # tells which tile is the street. False = full L/F/R scans
STREET_RULES = False             # If True, assumes no street tile can be in the outer ring of tiles (blindly
                                 # declares "exit" on edge tiles like y=-1 without scanning L/R). Keep False so
                                 # outer-ring corners/walls (e.g. bottom-right corner) are always scanned properly.
RETRY_ONE_TILE_STREET = True     # a one-tile street means the gate check guessed the wrong side: explore again
                                 # from the other gate tile
ALWAYS_FINISH_STREET = True      # map the whole street even if time runs short: a half map leaves Phase 2 a
                                 # partial street, and the clear way home starts at the street's end. The
                                 # watchdog then only decides whether to scout the drop-off. False = old cut-off


def prompt_point(label: str, default_pt: tuple[int, int]) -> tuple[int, int]:
    while True:
        raw = input(f"Enter {label} as 'x y' [Enter = sample map {default_pt[0]} {default_pt[1]}]: ").strip()
        if not raw:
            return default_pt
        parts = raw.replace(",", " ").split()
        try:
            return int(parts[0]), int(parts[1])
        except (ValueError, IndexError):
            print("  Please type two integers, e.g. '1 3'.")


def resolve_dropoff_pose(d1: tuple[int, int], d2: tuple[int, int]) -> tuple[tuple[int, int], int]:
    """In-bounds tile bordering the d1-d2 edge, and the outward heading toward the bucket."""
    gate = parse_gate_geometry(d1, d2)
    for cand in gate["candidates"]:
        if is_tile_in_bounds(*cand["approach_tile"]) and not is_tile_in_bounds(*cand["first_tile"]):
            return cand["approach_tile"], cand["heading"]
    for cand in gate["candidates"]:
        if is_tile_in_bounds(*cand["approach_tile"]):
            return cand["approach_tile"], cand["heading"]
    raise ValueError(f"Drop-off {d1}-{d2} does not border any tile on the board.")


def select_entrance_candidate(s1: tuple[int, int], s2: tuple[int, int]) -> dict:
    """
    First guess at which side of the s1-s2 edge is outside the street (closest to the origin).
    The guess is verified on arrival by `verify_entrance_side`.
    """
    gate = parse_gate_geometry(s1, s2)
    valid = [c for c in gate["candidates"]
             if is_tile_in_bounds(*c["approach_tile"]) and is_tile_in_bounds(*c["first_tile"])]
    if not valid:
        valid = [c for c in gate["candidates"] if is_tile_in_bounds(*c["first_tile"])]
    valid.sort(key=lambda c: abs(c["approach_tile"][0]) + abs(c["approach_tile"][1]))
    return valid[0]


def record_scan(tile, heading, scan, blocked):
    """Marks in-bounds neighbours seen as walls in the scan as blocked (the 4 origin tiles are never houses)."""
    for key, rel in (("F", 0), ("R", 90), ("L", -90)):
        if scan[key] < WALL_THRESHOLD_CM:
            t = tile_step(tile, (heading + rel) % 360)
            if is_tile_in_bounds(*t) and t not in core.ORIGIN_TILES:
                blocked.add(t)


REL = {"L": -90.0, "F": 0.0, "R": 90.0}
FREE = set()   # tiles known to have no house (set by run_phase1): origin tiles, the drop-off tile, tiles driven
NEVER_STREET = set()   # the origin tiles and the drop-off tile (the robot starts / pours there)


def could_be_open(t, current, street, blocked, visited) -> bool:
    """Can tile t (beside `current`) be open, i.e. not a house? Not if it is off the table, a house already
    seen, already driven, or touching an earlier street tile: the street is one tile wide with houses on both
    sides, so a tile beside it is a house (the same rule as core.infer_houses)."""
    if not is_tile_in_bounds(*t) or t in blocked or t in visited:
        return False
    return all(tile_step(t, h) == current or tile_step(t, h) not in street for h in (0, 90, 180, 270))


def on_edge(t) -> bool:
    """Tile in the outer ring of the table (a neighbour is off the table)."""
    return any(not is_tile_in_bounds(*tile_step(t, h)) for h in (0, 90, 180, 270))


def look_around(current, heading, street, blocked, visited, known, full=False):
    """
    Decides what to do at a stop. known = {"L"/"F"/"R": cm} already known (the forward watch saw a house ahead,
    the side glance saw an opening). Measures only what the decision needs, in the order the old scan used it:
    exit (both sides open), else forward, right, left. A direction that is the only one left is taken without
    looking (the next run's forward watch stops within a centimetre if that's wrong).
    Returns (choice "exit" | "F" | "R" | "L" | None, readings {dir: cm}, blind).
    """
    tiles = {d: tile_step(current, (heading + int(REL[d])) % 360) for d in REL}
    if full:                                   # after a wrong guess: only the table edge rules anything out
        # (and the street already mapped: open, but behind us, so it doesn't make this tile the exit)
        can_open = {d: is_tile_in_bounds(*tiles[d]) and tiles[d] not in street for d in REL}
        can_street = {d: can_open[d] and tiles[d] not in visited for d in REL}
    else:
        can_open = {d: could_be_open(tiles[d], current, street, blocked, visited) for d in REL}
        can_street = {d: can_open[d] and tiles[d] not in NEVER_STREET
                      and not (STREET_RULES and on_edge(tiles[d])) for d in REL}
    read = dict(known)
    if not full:
        for d in ("L", "F", "R"):
            if tiles[d] in FREE and tiles[d] not in street and d not in read:
                read[d] = 255.0                # known to have no house: open without looking
                can_open[d] = True
    measured = {}

    def maybe(cand, d):
        return cand[d] and read.get(d, 255.0) >= WALL_THRESHOLD_CM

    def is_open(cand, d, blind_ok=False):
        if not cand[d]:
            return False
        if d not in read:
            if blind_ok:
                return True
            read[d] = measured[d] = core.check_direction_distance(REL[d], restore=False)
        return read[d] >= WALL_THRESHOLD_CM

    if full:
        for d in ("F", "L", "R"):
            if d not in read:
                read[d] = measured[d] = core.check_direction_distance(REL[d], restore=False)
    exit_possible = len(street) > 1 and maybe(can_open, "L") and maybe(can_open, "R")
    # Street running along the table edge (one side off-table, the other in-bounds): no house can sit off-table,
    # so if BOTH the in-bounds side AND forward are open (not a corner with a wall ahead), the street has ended.
    edge_side = None
    if len(street) > 1 and not exit_possible:
        l_in, r_in = is_tile_in_bounds(*tiles["L"]), is_tile_in_bounds(*tiles["R"])
        if l_in != r_in:
            s = "L" if l_in else "R"
            if maybe(can_open, s) and maybe(can_open, "F"):
                edge_side = s
    wall_ahead = read.get("F", 255.0) < WALL_THRESHOLD_CM
    if STREET_RULES and not full and wall_ahead:
        exit_possible = False                  # a house straight ahead: this is a corner, not the street's end
    if STREET_RULES and not full and len(street) > 1 and not wall_ahead and on_edge(current):
        choice = "exit"                        # this tile can't be a street tile (outer ring): the street ended
    elif exit_possible and is_open(can_open, "L") and is_open(can_open, "R"):
        choice = "exit"
    elif edge_side is not None and is_open(can_open, edge_side) and is_open(can_open, "F"):
        choice = "exit"
    else:
        choice = None
        order = ("F", "R", "L")
        for i, d in enumerate(order):
            others = [o for o in order[i + 1:] if maybe(can_street, o)]
            if is_open(can_street, d, blind_ok=not others):
                choice = d
                break
    if measured:
        vizlog.emit("scan", tile=list(current), h=heading, **{d: round(v, 1) for d, v in measured.items()})
    if choice is None and not full:
        # Everything ruled out: the assumptions don't fit this board. Look at every direction properly.
        print(f"  [LOOK] nothing possible at {current} under the street rules; looking properly.")
        return look_around(current, heading, street, blocked, visited, {"F": read["F"]} if "F" in known else {},
                           full=True)
    blind = choice in REL and choice not in read
    return choice, read, blind


def explore_street(start_tile, heading, scan, street, visited, blocked, traversed, phase_start) -> tuple[list, tuple | None]:
    """
    Explores from start_tile (robot stopped there, facing `heading`, `scan` taken there).
    Returns (street_tiles, exit_tile). The exit tile (first tile with no houses on
    either side) is not part of the street.
    """
    current = start_tile
    fwd_dist, fwd_rel = scan["F"], 0.0
    exit_tile = None

    def hook(idx, tile):
        home = core.plan_home(tile, blocked)
        return should_abort_for_time(phase_start, estimate_path_seconds(home, heading) + core.EST_HOME_STEP_SEC)

    if ALWAYS_FINISH_STREET:
        hook = None

    print(f"\n[MAP] Exploring from Tile {current} heading {heading}")

    forward_mode = EXPLORE_MODE == "forward" and core.turret_present
    blind_run = None
    for _ in range(30):
        if forward_mode:
            # The forward reading only has to say the next tile is open; the run watches ahead itself.
            k = MAX_FORWARD_RUN_TILES if free_tiles_ahead(fwd_dist, fwd_rel) > 0 else 0
        else:
            k = min(free_tiles_ahead(fwd_dist, fwd_rel), MAX_RUN_TILES)
        n = 0
        while n < k:
            t = tile_step(current, heading, n + 1)
            if not is_tile_in_bounds(*t) or t in visited:
                break
            n += 1
        if n == 0:
            print(f"[MAP] Nothing ahead of Tile {current}; stopping.")
            break

        # Watch a side that has a house next to the first tile of the run (not the table edge).
        first = tile_step(current, heading)
        prefer = -1 if core.turret_angle < 0 else 1
        side = prefer
        for cand in (prefer, -prefer):
            nb = tile_step(first, (heading + 90 * cand) % 360)
            if is_tile_in_bounds(*nb):
                side = cand
                break
        if forward_mode:
            res = drive_run(current, heading, n, EXPLORE_DPS, stop_on_side_open=True, tile_hook=hook,
                            watch_forward=True, side_glance=side)
        else:
            res = drive_run(current, heading, n, EXPLORE_DPS, side_us=side, stop_on_side_open=True, tile_hook=hook)
        for t in res["tiles"]:
            street.append(t)
            visited.add(t)
            traversed.add(t)
        run_tiles = [current] + res["tiles"]
        for j, status in res["side_status"].items():
            if status == "wall" and 1 <= j < len(run_tiles):
                wt = tile_step(run_tiles[j], (heading + 90 * side) % 360)
                if is_tile_in_bounds(*wt) and wt not in core.ORIGIN_TILES:
                    blocked.add(wt)
        current = res["tile"]
        emit_map(street, blocked)

        home_path = core.plan_home(current, blocked)
        needed = estimate_path_seconds(home_path, heading) + core.EST_HOME_STEP_SEC
        if not ALWAYS_FINISH_STREET and should_abort_for_time(phase_start, needed):
            break

        if SKIP_KNOWN_READINGS and forward_mode:
            known = {}
            if res["stop_reason"] == "wall":
                # The forward watch stopped us one tile short of a house. Confirm it standing still (turret
                # already forward, ~0.1 s): a ghost echo while driving must not become a house on the map.
                known["F"] = core.check_direction_distance(0.0, restore=False)
                if known["F"] >= WALL_THRESHOLD_CM:
                    print(f"  [LOOK] forward watch saw a house ahead of {current}, standing still it reads "
                          f"{known['F']:.0f} cm: no house")
            if res["stop_reason"] == "side_open":
                known["R" if side > 0 else "L"] = 255.0      # the glance just saw this side open
            full = blind_run is not None and not res["tiles"]
            if full:
                # A turn taken without looking ran straight into a house: face the way we came in again,
                # mark both that turn and the way ahead as houses, and look at everything else properly.
                heading, turned = blind_run
                print(f"  [LOOK] the {turned} side at {current} is a house after all; looking properly.")
                turn_to(float(heading))
                known = {"F": 0.0, turned: 0.0}
        h_right, h_left = (heading + 90) % 360, (heading - 90) % 360
        t_f, t_r, t_l = tile_step(current, heading), tile_step(current, h_right), tile_step(current, h_left)
        if SKIP_KNOWN_READINGS and forward_mode:
            choice, read, blind = look_around(current, heading, street, blocked, visited, known, full)
            record_scan(current, heading, {d: read.get(d, 255.0) for d in REL}, blocked)
            txt = " ".join((f"{d}={read[d]:.0f}" if d not in known else
                            f"{d}={'open' if read[d] >= WALL_THRESHOLD_CM else 'house'}(seen)")
                           for d in ("L", "F", "R") if d in read)
            print(f"  [LOOK @ {current} h={heading}] {txt or 'nothing to measure'}"
                  + (f" -> {choice}" if choice else "") + (" (only way left, not measured)" if blind else ""))
        else:
            scan = scan_walls(heading, current)
            record_scan(current, heading, scan, blocked)
            l_open = scan["L"] >= WALL_THRESHOLD_CM and is_tile_in_bounds(*t_l)
            f_open = scan["F"] >= WALL_THRESHOLD_CM and is_tile_in_bounds(*t_f)
            r_open = scan["R"] >= WALL_THRESHOLD_CM and is_tile_in_bounds(*t_r)
            print(f"  [SCAN @ {current} h={heading}] L={scan['L']:.0f} F={scan['F']:.0f} R={scan['R']:.0f}")
            read, blind = scan, False
            if len(street) > 1 and l_open and r_open:
                choice = "exit"
            elif f_open and t_f not in visited:
                choice = "F"
            elif r_open and t_r not in visited:
                choice = "R"
            elif l_open and t_l not in visited:
                choice = "L"
            else:
                choice = None
        blind_run = (heading, choice) if blind else None      # heading before the turn, and the side taken

        if choice == "exit":
            exit_tile = current
            if street and street[-1] == current:
                street.pop()
            print(f"[MAP] No houses on either side at Tile {current} -> street ended.")
            break
        if choice == "F":
            fwd_dist, fwd_rel = read.get("F", 255.0), 0.0
        elif choice == "R":
            print(f"  [CORNER] right at {current}")
            heading, fwd_dist, fwd_rel = h_right, read.get("R", 255.0), 90.0
        elif choice == "L":
            print(f"  [CORNER] left at {current}")
            heading, fwd_dist, fwd_rel = h_left, read.get("L", 255.0), -90.0
        else:
            print(f"[MAP] No unvisited open direction at Tile {current}; street ended.")
            break

    return street, exit_tile


def emit_map(street, blocked, exit_tile=None, approach_tile=None):
    vizlog.emit("map", street=vizlog.tiles(street), blocked=vizlog.tiles(sorted(blocked)),
                exit=list(exit_tile) if exit_tile else None, approach=list(approach_tile) if approach_tile else None)


def save_street_map(payload: dict):
    payload["timestamp"] = time.strftime("%Y-%m-%d %H:%M:%S")
    with open(MAP_FILE, "w") as f:
        json.dump(payload, f, indent=2)
    print(f"\n[SAVE] {len(payload['street_tiles'])} street tiles -> '{MAP_FILE}'.")


def print_ascii_map(street, blocked, exit_tile, dropoff_tile):
    print("\n  Map (S = street, # = house seen, E = exit, D = drop-off, O = origin tile):")
    for ty in range(core.TILE_Y_MAX, core.TILE_Y_MIN - 1, -1):
        row = ""
        for tx in range(core.TILE_X_MIN, core.TILE_X_MAX + 1):
            t = (tx, ty)
            ch = "."
            if t in blocked:
                ch = "#"
            if t in street:
                ch = "S"
            if t == exit_tile:
                ch = "E"
            if t == dropoff_tile:
                ch = "D"
            if t == (0, 0) and ch == ".":
                ch = "O"
            row += ch + " "
        print(f"  {ty:>2} | {row}")


def run_phase1(s1=None, s2=None, d1=None, d2=None):
    print("=" * 65)
    print("  ECSE 211 FINAL PROJECT - PHASE 1: SUMMER STREET MAPPING")
    print("=" * 65)

    s1 = s1 or prompt_point("Entrance s1", DEFAULT_S1)
    s2 = s2 or prompt_point("Entrance s2", DEFAULT_S2)
    d1 = d1 or prompt_point("Drop-off d1", DEFAULT_D1)
    d2 = d2 or prompt_point("Drop-off d2", DEFAULT_D2)

    ent = select_entrance_candidate(s1, s2)
    approach_tile, first_tile, entry_heading = ent["approach_tile"], ent["first_tile"], ent["heading"]
    dropoff_tile, dropoff_heading = resolve_dropoff_pose(d1, d2)
    print(f"\n[PLAN] Gate {s1}-{s2}: approach {approach_tile} -> first tile {first_tile} @ {entry_heading}")
    print(f"[PLAN] Drop-off {d1}-{d2}: tile {dropoff_tile}, bucket toward {dropoff_heading}")
    input("Place robot at Origin (0,0) facing +Y and press Enter to start Phase 1...")

    init_hardware(phase=1)
    vizlog.start(1, s1=list(s1), s2=list(s2), d1=list(d1), d2=list(d2), dropoff=list(dropoff_tile),
                 outline=core.ROBOT_OUTLINE[1], us_fwd=core.US_FORWARD_OFFSET_CM, us_face=core.US_FACE_CM,
                 sensor_offset=core.SENSOR_OFFSET, sensor_spacing=core.SENSOR_SPACING)
    phase_start = time.time()
    blocked: set = set()
    # Which side of the gate is outside, if the rules already say (no reading needed): an origin / drop-off
    # tile, or a tile with one beside it, can't be the street, nor (STREET_RULES) one on the table's outer
    # ring. Then head for the outside tile only: the street's first tile is only reachable through it (or
    # through the whole street from its far end).
    gate_tiles = [approach_tile, first_tile]
    never = set(core.ORIGIN_TILES) | {tuple(dropoff_tile)}
    axis = heading_between_adjacent_tiles(approach_tile, first_tile)

    def ruled_out(t):
        return t in never or any(tile_step(t, (axis + d) % 360) in never for d in (90, 270)) or (
            STREET_RULES and on_edge(t))
    outside_known = [t for t in gate_tiles if ruled_out(t)]
    targets = outside_known if len(outside_known) == 1 else gate_tiles
    if len(outside_known) == 1:
        print(f"[PLAN] {outside_known[0]} can't be a street tile: heading for it (the outside of the gate)")
    start_tile, first_heading = core.choose_start(targets, blocked, keep_route=True)
    traversed: set = {start_tile}

    step_origin_to_start_tile(start_tile, first_heading)
    street: list = []
    # Head for whichever side of the gate is cheaper to reach, then work out which side is the street: s1-s2 is
    # only an edge, and the tile nearer the origin isn't always the outside one (sim map u_reversed).
    current = navigate_tile_with_obstacle_avoidance(
        start_tile, targets, blocked, speed_dps=CRUISE_DPS, check_obstacles=True, traversed=traversed)
    if current not in (approach_tile, first_tile):
        raise RuntimeError(f"Could not reach the gate {s1}-{s2} (stopped at {current}).")
    other = first_tile if current == approach_tile else approach_tile
    toward_gate = heading_between_adjacent_tiles(current, other)
    turn_to(float(toward_gate))
    visited = {current, other}
    sides = [d for d in ("L", "R") if is_tile_in_bounds(*tile_step(current, (toward_gate + int(REL[d])) % 360))]
    FREE.clear()
    FREE.update(core.ORIGIN_TILES, [dropoff_tile], traversed)
    NEVER_STREET.clear()
    NEVER_STREET.update(core.ORIGIN_TILES, [dropoff_tile])

    def not_street_reason(t):
        # A street tile has a house on at least one side (across the gate axis; both unless the street turns
        # right away), and the side that isn't a house is the next street tile. An origin / drop-off tile beside
        # it can be neither, so it rules it out, as does being one. A tile the robot drove through proves
        # nothing: the way here may have run through the street itself (from its far end).
        # One step further: if the robot drove through the tile beside it (so not a house) and t were the street,
        # the street would turn there and that tile would be the next street tile. It can't be one if it touches
        # an origin / drop-off tile or (STREET_RULES) is on the outer ring: then t isn't the street either.
        if t in NEVER_STREET:
            return "it is an origin / drop-off tile"
        for d in ("L", "R"):
            b = tile_step(t, (toward_gate + int(REL[d])) % 360)
            if b in NEVER_STREET:
                return f"{b} beside it has no house (origin / drop-off tile)"
            if b in traversed and b not in (current, other):
                n = next((tile_step(b, h) for h in (0, 90, 180, 270)
                          if tile_step(b, h) != t and tile_step(b, h) in NEVER_STREET), None)
                if n is not None:
                    return f"{b} beside it was driven through and touches {n}, so it can't be the next street tile"
                if STREET_RULES and on_edge(b):
                    return f"{b} beside it was driven through and is on the outer ring"
        return None
    why_cur, why_other = not_street_reason(current), not_street_reason(other)
    if SKIP_KNOWN_READINGS and (why_cur is None) != (why_other is None):
        inside = why_cur is None
        scan = {"F": 255.0}
        print(f"  [LOOK @ {current}] {'the other tile' if inside else 'this tile'} can't be the street: "
              f"{why_other if inside else why_cur} -> {'inside' if inside else 'outside'} (nothing to measure)")
    elif SKIP_KNOWN_READINGS and STREET_RULES and on_edge(current) != on_edge(other):
        # A street tile can't be in the outer ring of tiles: the one that isn't is the street.
        inside = not on_edge(current)
        scan = {"F": 255.0}
        print(f"  [LOOK @ {current}] {'the other tile' if inside else 'this tile'} is on the table's outer ring "
              f"-> {'inside' if inside else 'outside'} the street (nothing to measure)")
    elif SKIP_KNOWN_READINGS and sides:
        # One side is enough: a house right beside this tile means it is the street's first tile.
        # A house beside this tile doesn't settle it: the tile outside the gate can have one too (the walls of
        # the street's other arm in a U, or of its second tile when it turns right after the gate). Only a house
        # on both sides does; one open side means outside (if that's wrong, the one-tile check below catches it).
        d = min(sides, key=lambda d: abs(core.turret_target_for(REL[d]) - core.turret_angle))
        readings = {d: core.check_direction_distance(REL[d], restore=False)}
        if readings[d] < WALL_THRESHOLD_CM and len(sides) == 2:
            o = "R" if d == "L" else "L"
            readings[o] = core.check_direction_distance(REL[o], restore=False)
        vizlog.emit("scan", tile=list(current), h=toward_gate, **{k: round(v, 1) for k, v in readings.items()})
        record_scan(current, toward_gate, {"L": 255.0, "F": 255.0, "R": 255.0, **readings}, blocked)
        inside = all(v < WALL_THRESHOLD_CM for v in readings.values())
        print(f"  [LOOK @ {current} h={toward_gate}] " + " ".join(f"{k}={v:.0f}" for k, v in readings.items())
              + f" -> {'inside' if inside else 'outside'} the street")
        scan = {"F": 255.0}                    # the gate side is open by definition; the run watches ahead itself
    else:
        scan = scan_walls(toward_gate, current)
        record_scan(current, toward_gate, scan, blocked)
        inside = scan["L"] < WALL_THRESHOLD_CM and scan["R"] < WALL_THRESHOLD_CM
    if inside:
        # Houses beside us: we are standing in the first street tile; the street runs away from the gate.
        print(f"[MAP] Houses beside {current}: first street tile; street runs away from the gate.")
        approach_tile, first_tile = other, current
        heading = (toward_gate + 180) % 360
        street.append(current)
        turn_to(float(heading))
        if not SKIP_KNOWN_READINGS:
            scan = scan_walls(heading, current)
            record_scan(current, heading, scan, blocked)
    else:
        # We are outside; the street starts on the other side of the gate.
        approach_tile, first_tile = current, other
        heading = toward_gate
        visited.discard(other)
    entry_heading = heading if street else toward_gate
    gate_pair = (approach_tile, first_tile)

    street, exit_tile = explore_street(current, heading, scan, street, visited, blocked, traversed, phase_start)
    current = exit_tile if exit_tile is not None else (street[-1] if street else current)
    if len(street) == 1 and RETRY_ONE_TILE_STREET:
        # A one-tile "street" means the gate check picked the wrong side: when the street turns right after the
        # gate, both gate tiles have one house beside them, and the readings there can look the same. Explore
        # again from the other side of the gate.
        approach_tile, first_tile = gate_pair[1], gate_pair[0]
        print(f"[MAP] Only one street tile found: {gate_pair[1]} was the outside. Exploring again from "
              f"{approach_tile} into {first_tile}.")
        current = navigate_tile_with_obstacle_avoidance(current, approach_tile, blocked, CRUISE_DPS, True, None,
                                                        traversed)
        if current == approach_tile:
            heading = entry_heading = heading_between_adjacent_tiles(approach_tile, first_tile)
            turn_to(float(heading))
            blocked.discard(first_tile)
            visited.discard(first_tile)
            visited.add(approach_tile)
            street, exit_tile = explore_street(current, heading, {"F": 255.0}, [], visited, blocked, traversed,
                                               phase_start)
            current = exit_tile if exit_tile is not None else (street[-1] if street else current)
    blocked.difference_update(street)
    blocked.difference_update(core.ORIGIN_TILES)
    if exit_tile is not None:
        blocked.discard(exit_tile)
    # A street is one tile wide: every other tile touching it is a house (never the 4 origin tiles).
    blocked |= core.infer_houses(street, approach_tile, exit_tile)
    blocked.difference_update(core.ORIGIN_TILES)
    emit_map(street, blocked, exit_tile, approach_tile)

    payload = {
        "s1": list(s1), "s2": list(s2), "d1": list(d1), "d2": list(d2),
        "approach_tile": list(approach_tile),
        "entry_heading": entry_heading,
        "street_tiles": [list(t) for t in street],
        "exit_tile": list(exit_tile) if exit_tile is not None else None,
        "blocked_tiles": [list(t) for t in sorted(blocked)],
        "traversed_tiles": [list(t) for t in sorted(traversed)],
        "dropoff_tile": list(dropoff_tile),
        "dropoff_heading": dropoff_heading,
    }
    save_street_map(payload)
    print_ascii_map(street, blocked, exit_tile, dropoff_tile)

    # Return home, scouting the drop-off approach Phase 2 will use if there is time.
    on_the_way = dropoff_tile in core.plan_home(current, blocked, None, core.theta)
    if SCOUT_DROPOFF_ON_RETURN and on_the_way:
        print(f"\n[SCOUT] The way home passes through the drop-off tile {dropoff_tile}: no detour, no stop.")
    elif SCOUT_DROPOFF_ON_RETURN:
        options = plan_dump_approach(dropoff_tile, dropoff_heading, blocked)
        best = None
        for opt in options:
            p1 = plan_path(current, opt["pre_tile"], blocked)
            p2 = core.plan_home(dropoff_tile, blocked, None, opt["heading"])
            if p1 and p2:
                cost = estimate_path_seconds(p1 + [dropoff_tile]) + estimate_path_seconds(p2, opt["heading"])
                if best is None or cost < best[0]:
                    best = (cost, opt)
        if best and core.time_remaining(phase_start) > best[0] + core.EST_HOME_STEP_SEC + core.TIME_SAFETY_MARGIN_SEC:
            opt = best[1]
            print(f"\n[SCOUT] Via drop-off: {current} -> {opt['pre_tile']} -> {dropoff_tile}")
            current = navigate_tile_with_obstacle_avoidance(
                current, opt["pre_tile"], blocked, CRUISE_DPS, True, None, traversed)
            if current == opt["pre_tile"]:
                current = navigate_tile_with_obstacle_avoidance(
                    current, dropoff_tile, blocked, CRUISE_DPS, True, None, traversed)
            payload["blocked_tiles"] = [list(t) for t in sorted(blocked)]
            payload["traversed_tiles"] = [list(t) for t in sorted(traversed)]
            save_street_map(payload)
        else:
            print("\n[SCOUT] Skipping drop-off scouting (time or no route).")

    print(f"\n[RETURN] {current} -> Origin")
    home_tiles = [t for t in core.ORIGIN_TILES if t not in blocked]
    current = navigate_tile_with_obstacle_avoidance(
        current, home_tiles, blocked, CRUISE_DPS, True, None, traversed)
    step_tile_to_origin()
    payload["traversed_tiles"] = [list(t) for t in sorted(traversed)]
    payload["blocked_tiles"] = [list(t) for t in sorted(blocked)]
    save_street_map(payload)

    total = time.time() - phase_start
    print("\n" + "=" * 65)
    print(f"  PHASE 1 COMPLETE in {total:.1f}s (limit 120 s). Standby at origin.")
    print("=" * 65)
    vizlog.emit("end", total=round(total, 1))
    vizlog.close()
    return total


if __name__ == "__main__":
    try:
        run_phase1()
    except BaseException as e:
        print(f"\n[PHASE 1 TERMINATED]: {type(e).__name__}: {e}")
        traceback.print_exc()
    finally:
        vizlog.close()
        stop_all_motors()
        reset_brick()
