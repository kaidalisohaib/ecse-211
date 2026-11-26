#!/usr/bin/env python3
"""
ECSE 211 Final Design Project - Phase 2: Autonomous Snow Removal
================================================================
  1. Loads street_map.json from Phase 1 (street tiles, houses seen, tiles already driven).
  2. Origin -> approach tile outside the gate, preferring tiles Phase 1 actually drove.
  3. Roller on; sweeps the street as straight runs (stops only at corners), with flying
     line snaps and optional side-ultrasonic wall correction.
  4. The watchdog plans the whole remaining mission (street -> drop-off -> dump -> home)
     and cuts the sweep short if that would not fit in 120 s.
  5. Drop-off: drives along the table edge into the drop-off tile, shifted toward the
     edge, stops with the tray's pour point centred on the bucket, and tips the tray
     sideways into it (the bucket is at table height, so the snow only goes sideways).
  6. Returns to the origin.
"""
from __future__ import annotations

import json
import math
import os
import time
import traceback
import snowplow_core as core
import vizlog
from snowplow_core import (
    CRUISE_DPS,
    MAP_FILE,
    SWEEP_DPS,
    compile_runs,
    drive_run,
    dump_snow_payload,
    estimate_path_seconds,
    init_hardware,
    plan_dump_approach,
    plan_path,
    reset_brick,
    should_abort_for_time,
    start_intake_roller,
    step_origin_to_start_tile,
    step_tile_to_origin,
    stop_all_motors,
    stop_intake_roller,
    tile_step,
    turn_to,
)

# ---- Phase 2 hardware / strategy settings ----
# Which way the tray pours is set by core.DUMP_PLUS / core.DUMP_MINUS (the cheapest of the two is used).
DUMP_BACK_LIP_CM = 9.0            # MEASURE (BACK only): axle to the rear edge the tray pours over
DUMP_BACK_OVERHANG_CM = 2.0       # BACK: wanted overhang of that lip past the table edge
DUMP_BACK_SUPPORT_CM = 8.0        # MEASURE (BACK only): axle to the rearmost ground contact (caster / skid)
DUMP_BACK_EDGE_MARGIN_CM = 2.0    # BACK: that contact stays at least this far inside the table edge
EDGE_LANE_SHIFT_CM = 5.0          # Drive this much closer to the table edge on the drop-off row (MEASURE wheel margin)
DUMP_POINT_FORWARD_CM = -10.0     # MEASURE: middle of the bin's pour opening ahead of the wheel axle (- = behind):
                                  # centres the pour on the bucket. Robot photos 2026-11-22: bin ~10 cm behind
DROP_APPROACH_DPS = 220       # Slower wheel speed (deg/s) for the end of the last run into the drop-off
DROP_APPROACH_CM = core.TILE_SIZE  # ...over the last run's last this many cm (it cruises before that)
DROP_LANE_GAIN = 5.0              # last drop-off run: cross-track deg per cm (drive_run's default 1.5 needs ~40 cm)
DROP_MAX_STEER_DEG = 15.0         # ...and its steering limit (default 6)
SPIN_SHIFT_CM = 3.0               # turn in place this far from the centre, away from a house beside the tile
ARC_CORNERS = True                # drive street corners on an arc (square intake can't spin there)
ARC_EVERY_TURN = True             # transit: arc every 90 deg turn too (sim: -5 s P2, same home error); False = only next to houses
ARC_RADIUS_MIN_CM = 12.0          # an arc sized to end on a drop-off lane (arc_radius_for): limits when no house is
ARC_RADIUS_MAX_CM = 19.0          # on the inside diagonal / outside the corner (wheels stay 5 cm inside the tile)
CORNER_POKE_CM = 0.0              # before each arc, sweep straight into the corner's outer part and back (0 = off).
                                  # Off since 2026-11-21: the funnel can't keep what it pokes at when backing out,
                                  # and it costs 6-12 s. Max = 30.48 - intake reach (9) - 1.5 cm to the far house.
CORNER_POKE_SPARE_SEC = 8.0       # skip the poke unless this much time is spare
DROP_STYLES = ("direct", "along", "turn")  # "direct" = drive in facing the bucket, turn 90 in place, pour;
                                   # "along" = arrive along the edge already parallel; "turn" = arrive along the
                                   # edge the other way, turn 180 in place (side pours). Cheapest is used.
ROLLER_ON_DURING_TRANSIT = True   # Keep the roller turning until the dump so nothing falls back out
ROLLER_FLUSH_SEC = 0.5        # After the sweep, pause this long (s) with the belt running so the last pom-poms reach the bin
TURN_LANE_RUN_CM = 25.0           # side-pour drop-off: the last run is at least this long (reverses first if not)
                                  # to reach its lane: the edge lane, or the lane off a house beside the tile
LANE_MISS_CM = 1.0                # ...when it starts more than this off its lane (the arc onto it couldn't end there)
DROP_REVERSE_EST_SEC = 3.0        # time estimate for that reverse (sim: 1.8 s back + the longer run)
POUR_SHIFT_DPS = 120              # "direct": the move along the edge after the 90 deg turn
BACK_OUT_DPS = 220                # reversing straight out of the street when the watchdog cuts the sweep short
BACK_OUT_SEC_PER_TILE = 4.6       # time estimate for that reverse (slower than EST_SEC_PER_TILE)

TRUST_MAP = True  # the only houses are the street's walls: the tiles beside the street plus corner boxes (a tile
                  # diagonal to a street tile with a house on both tiles between them). Everything else is open
                  # floor, so Phase 2 plans anywhere. False = only tiles Phase 1 drove (needs its drop-off visit)

TIGHT = set()   # tiles with a house directly beside them (set by run_phase2): no turning in place there
PHASE2_SIDE_US = None             # None, -1 (left) or +1 (right): fixed side ultrasonic for wall correction
PHASE2_US_FORWARD_OFFSET_CM = None  # MEASURE if PHASE2_SIDE_US is used (sensor ahead of the axle)
PHASE2_US_FACE_CM = None            # MEASURE if PHASE2_SIDE_US is used (centreline to sensor face)


def load_street_map() -> dict:
    if not os.path.exists(MAP_FILE):
        raise FileNotFoundError(f"Map file '{MAP_FILE}' not found! Run phase1_mapping.py first.")
    with open(MAP_FILE, "r") as f:
        data = json.load(f)
    print(f"[LOAD] '{MAP_FILE}': {len(data['street_tiles'])} street tiles.")
    return data


def finish_seconds(tile, plan, blocked, known) -> float:
    """Estimated time from `tile` to finish: drive to the drop-off, dump, go home."""
    if plan is None:
        home = core.plan_home(tile, blocked, known, tight=TIGHT)
        return estimate_path_seconds(home) + core.EST_HOME_STEP_SEC
    to_pre = plan_path(tile, plan["pre_tile"], blocked, known, tight=TIGHT)
    turn = core.EST_SEC_PER_TURN * plan.get("turns", 1 if plan["style"] == "direct" else 0)
    if plan.get("side") == "BACK":
        turn += 1.5                        # reversing to the edge
    drop, h = tuple(plan["dropoff_tile"]), plan["final_heading"]
    t_home = estimate_path_seconds(core.plan_home(drop, blocked, known, h, tight=TIGHT), h)
    back = tile_step(drop, h, -1)
    if drop in TIGHT and core.is_tile_in_bounds(*back) and back not in blocked and back not in TIGHT:
        via_back = core.plan_home(back, blocked, known, h, tight=TIGHT)   # leave_dropoff may back out first
        if via_back:
            t_home = min(t_home, BACK_OUT_SEC_PER_TILE + estimate_path_seconds(via_back, h))
    path = to_pre + list(plan.get("via", [])) + [plan["dropoff_tile"]]
    return estimate_path_seconds(path) + turn + core.EST_DUMP_SEC + t_home + core.EST_HOME_STEP_SEC


def _cm_per_sec(dps):
    return math.radians(dps) * core.WHEEL_RADIUS


def drop_extra_seconds(path, plan, blocked) -> float:
    """What estimate_path_seconds leaves out of a drop-off: the last run's last tile at DROP_APPROACH_DPS,
    the reverse before a short last run that an arc couldn't put on its lane (drive_to_dump_and_pour),
    "direct"'s move along the edge after its turn. Only for choosing between drop-off plans: finish_seconds (the
    watchdog) leaves it out, its EST_ constants are already ~10 s pessimistic over a whole Phase 2 (sim)."""
    runs = compile_runs(path) if len(path) >= 2 else []
    if not runs:
        return 0.0
    h_last, n_last = runs[-1]
    lane, last_end = last_run_lane(plan, blocked, h_last)
    slow_cm = min(DROP_APPROACH_CM, n_last * core.TILE_SIZE + last_end)
    extra = slow_cm * (1.0 / _cm_per_sec(DROP_APPROACH_DPS) - 1.0 / _cm_per_sec(CRUISE_DPS))
    if (lane_matters(plan) and n_last == 1 and len(runs) >= 2 and core.TILE_SIZE / 2.0 + last_end < TURN_LANE_RUN_CM
            and abs(abs(core.minimal_angle_diff(h_last, runs[-2][0])) - 90) < 1):
        h_in = runs[-2][0]
        r = arc_radius_for(path[-2], h_in, h_last, lane, blocked) or core.ARC_RADIUS_CM
        s = 1 if core.minimal_angle_diff(h_last, h_in) > 0 else -1
        if abs(lane - s * (core.TILE_SIZE / 2.0 - r)) > LANE_MISS_CM:     # the lane that arc ends on
            extra += DROP_REVERSE_EST_SEC
    if plan["style"] == "direct" and plan.get("side") != "BACK":
        extra += abs(plan.get("shift", 0.0)) / _cm_per_sec(POUR_SHIFT_DPS) + 0.8
    return extra


def lane_matters(plan) -> bool:
    """The last drop-off run must reach its lane before the pour: a side pour's lane is the edge lane (where the
    bin tips), except "direct"'s, which a move along the edge after its turn replaces; but next to a house the
    lane is what clears it (spin_shift). BACK pours reverse to the edge afterwards (unchanged)."""
    if plan.get("side") == "BACK":
        return False
    return plan["style"] != "direct" or tuple(plan["dropoff_tile"]) in TIGHT


def choose_dump_plan(from_tile, dropoff_tile, dropoff_heading, blocked, known):
    options = plan_dump_approach(dropoff_tile, dropoff_heading, blocked, core.dump_directions(), DROP_STYLES,
                                 DUMP_POINT_FORWARD_CM, max_run_tiles=2)
    best = None
    for opt in options:
        opt = dict(opt, dropoff_tile=dropoff_tile)
        cost = finish_seconds(from_tile, opt, blocked, known)
        cost += opt.get("penalty", 0.0)
        path = plan_path(from_tile, opt["pre_tile"], blocked, known, tight=TIGHT)
        cost += drop_extra_seconds(path + list(opt.get("via", [])) + [dropoff_tile], opt, blocked)
        if opt.get("turns") and tuple(dropoff_tile) in TIGHT and not drop_spin_clear(opt, dropoff_tile, blocked):
            cost += 1000.0      # turns in place on the drop-off tile next to a house: last resort
        if path and len(path) >= 2 and (core.heading_between_adjacent_tiles(path[-2], path[-1])
                                         - opt["heading"]) % 360 == 180:
            # The way to pre_tile ends heading away from the drop-off (often through it): a U-turn in place there
            cost += 2 * core.EST_SEC_PER_TURN + (1000.0 if tuple(opt["pre_tile"]) in TIGHT else 0.0)
        if path and (best is None or cost < best[0]):
            best = (cost, opt)
    return best[1] if best else None


def drop_spin_clear(plan, dropoff_tile, blocked) -> bool:
    """A drop-off plan that turns in place next to a house does it off the tile centre, clear of the house:
      turn:   the 180 is on the edge lane, EDGE_LANE_SHIFT_CM from an inland house.
      direct: the 90 is SPIN_SHIFT_CM from a house beside the tile along the edge (spin_shift), EDGE_LANE_SHIFT_CM
              toward the bucket; the inland side is the tile it came from. Not with a house on both sides.
    Both reverse first when the last run is too short to reach that lane (TURN_LANE_RUN_CM, LANE_MISS_CM)."""
    if plan["style"] == "turn":
        return True
    if plan["style"] == "direct":
        return spin_shift(dropoff_tile, plan["heading"], blocked)[1] != 0.0
    return False


def last_run_lane(plan, blocked, h_last):
    """(lane cm, + = right of h_last; stop cm past the drop-off tile centre) of the last drop-off run.
    A plan that turns in place on the drop-off tile does it EDGE_LANE_SHIFT_CM toward the bucket from the
    tile centre (the bucket side is open), and SPIN_SHIFT_CM away from a house beside the tile."""
    shift_along, shift_lat = spin_shift(plan["dropoff_tile"], h_last, blocked)
    if plan["side"] == "BACK" and plan["style"] == "along":
        outward = (plan["final_heading"] + 180) % 360
        return EDGE_LANE_SHIFT_CM * (1 if outward == (plan["heading"] + 90) % 360 else -1), shift_along
    if plan["style"] == "direct":
        return shift_lat, EDGE_LANE_SHIFT_CM
    if plan["style"] == "turn":                        # arrives facing away from final_heading, turns 180
        return -EDGE_LANE_SHIFT_CM * plan["tilt_sign"], -plan.get("shift", -DUMP_POINT_FORWARD_CM)
    return EDGE_LANE_SHIFT_CM * plan["tilt_sign"], plan.get("shift", -DUMP_POINT_FORWARD_CM)   # along


def drop_runs(current, plan, blocked, known, heading):
    """
    The way from `current` into the drop-off tile: (runs, starts, lanes, ends, last_end).
    Edge lane: the final run drives EDGE_LANE_SHIFT_CM off the tile centre line toward the bucket. Every run on
    that same line uses the lane too (sign flips when driving the opposite way), a run that turns 90 deg onto
    the line stops already shifted (ends), and an arc onto the line is sized to end on it (arc_radius_for): a
    short final run (often one tile) has no room to steer 3 cm sideways.
    """
    path = (plan_path(current, plan["pre_tile"], blocked, known, heading, TIGHT) + list(plan.get("via", []))
            + [plan["dropoff_tile"]])
    runs = compile_runs(path)
    h_last = runs[-1][0] if runs else 0
    lane, last_end = last_run_lane(plan, blocked, h_last)
    drop_line = core.tile_center_lateral(plan["dropoff_tile"], h_last)
    starts, t = [], current
    for h, n in runs:
        starts.append(t)
        t = tile_step(t, h, n)
    starts.append(t)
    lanes = [0.0] * len(runs)
    for i, (h, n) in enumerate(runs):
        on_line = abs(core.tile_center_lateral(starts[i], h_last) - drop_line) < 1.0
        if on_line and h == h_last:
            lanes[i] = lane
        elif on_line and h == (h_last + 180) % 360:
            lanes[i] = -lane
    ends = [0.0] * len(runs)
    for i in range(len(runs) - 1):
        h, hn = runs[i][0], runs[i + 1][0]
        if lanes[i + 1] and h == (hn + 90) % 360:
            ends[i] = lanes[i + 1]
        elif lanes[i + 1] and h == (hn - 90) % 360:
            ends[i] = -lanes[i + 1]
    return path, runs, starts, lanes, ends, last_end


def arc_radius_for(corner, h_in, h_out, lane_out, blocked):
    """Arc radius that ends the arc on lane_out (cm, + = right of h_out) instead of the usual arc lane, within
    what the houses allow. A house on the inside keeps it >= ARC_RADIUS_CM, one on the outside <= ARC_RADIUS_CM:
    the corner tile's outside neighbours (the funnel swings outward) and the houses beside the tile before the
    corner (the run in ends shifted by 15.24 - radius: in a street there is only +-3 cm of room). None = default."""
    if not lane_out:
        return None
    s = 1 if core.minimal_angle_diff(h_out, h_in) > 0 else -1
    half = core.TILE_SIZE / 2.0
    prev = tile_step(corner, (h_in + 180) % 360)               # the run into the arc ends on this tile's edge,
    inside = (tile_step(tile_step(corner, h_out), (h_in + 180) % 360),  # already shifted (arc_lane_offset)
              tile_step(prev, (h_in + 90 * s) % 360))
    outside = (tile_step(corner, h_in), tile_step(corner, (h_out + 180) % 360), tile_step(prev, (h_in - 90 * s) % 360))
    lo = ARC_RADIUS_MIN_CM if not any(t in blocked for t in inside) else core.ARC_RADIUS_CM
    hi = ARC_RADIUS_MAX_CM if not any(t in blocked for t in outside) else core.ARC_RADIUS_CM
    return max(lo, min(hi, half - s * lane_out))


def drive_to_dump_and_pour(current, plan, blocked, known, _tries=3):
    """
    Path to the pre-drop tile, then the last run into the drop-off tile, then pour.
    The axle has to end EDGE_LANE_SHIFT_CM closer to the edge than the tile centre, with the pour point
    (DUMP_POINT_FORWARD_CM ahead of the axle, - = behind) level with the tile centre along the edge.
      along:  last run is parallel to the edge -> lane offset = edge shift, stops that far short of / past the
              centre.
      direct: last run heads at the bucket on the tile centre line, stops past the centre by the edge shift,
              turns 90 deg in place, then moves along the edge so the pour point is level with the centre.
    """
    path, runs, starts, lanes, ends, last_end = drop_runs(current, plan, blocked, known, core.theta)
    vizlog.emit("plan", path=vizlog.tiles(path))
    tile = current
    for i, (heading, n) in enumerate(runs):
        last = i == len(runs) - 1
        if last and lane_matters(plan):
            # The pour needs the edge lane, a turn in place next to a house its lane (on the centre line it passes
            # 0.24 cm from the house). An arc onto this run (the street exit's, or one hemmed in by houses) can
            # leave ~5 cm to reach it: reverse first so the run has TURN_LANE_RUN_CM.
            core.update_odometer()
            stop_at = core.tile_center_along(plan["dropoff_tile"], heading) + last_end
            miss = core.get_lateral(heading) - core.tile_center_lateral(tile, heading) - lanes[i]
            if stop_at - core.get_along(heading) < TURN_LANE_RUN_CM and abs(miss) > LANE_MISS_CM:
                if abs(core.minimal_angle_diff(heading, core.theta)) > 10.0:
                    turn_to(float(heading))        # drive_run would turn here first anyway
                behind = tile_step(tile, heading, -1)
                room = core.is_tile_in_bounds(*behind) and behind not in blocked   # else not past the centre
                end = stop_at - TURN_LANE_RUN_CM - core.tile_center_along(tile, heading)
                back_out(tile, heading, lane_offset_cm=lanes[i], end_along_offset_cm=end if room else max(0.0, end))
        if last:
            # The last run is often one tile: steer harder so the lane offset is reached before the stop. A longer
            # one cruises until its last tile.
            res = drive_run(tile, heading, n, CRUISE_DPS, lane_offset_cm=lanes[i],
                            end_along_offset_cm=last_end, lane_gain=DROP_LANE_GAIN,
                            max_steer_deg=DROP_MAX_STEER_DEG, approach_cm=DROP_APPROACH_CM,
                            approach_dps=DROP_APPROACH_DPS)
        elif _arc_here(starts[i + 1], heading, runs[i + 1][0]):
            nxt = runs[i + 1][0]
            r = arc_radius_for(starts[i + 1], heading, nxt, lanes[i + 1], blocked)
            res = drive_run(tile, heading, n, CRUISE_DPS, lane_offset_cm=lanes[i],
                            end_lane_offset_cm=core.arc_lane_offset(heading, nxt, r),
                            end_along_offset_cm=-core.TILE_SIZE / 2.0)
            if res["stop_reason"] == "end":
                tile = core.drive_arc(starts[i + 1], heading, nxt, CRUISE_DPS, radius=r)
                continue
            res["tile"] = core.cm_to_nearest_tile(core.x, core.y)
        else:
            res = drive_run(tile, heading, n, CRUISE_DPS, lane_offset_cm=lanes[i], end_along_offset_cm=ends[i])
        tile = res["tile"]
        if not last and res["stop_reason"] != "end" and _tries > 0:
            # Run cut short (timeout): the later runs count tiles from their planned starts, so plan again
            # from here instead of driving them from the wrong tile.
            print(f"[DROP-OFF] run stopped early at {tile} ({res['stop_reason']}): planning again")
            return drive_to_dump_and_pour(tile, plan, blocked, known, _tries - 1)
    if plan["side"] == "BACK":
        turn_to(float(plan["final_heading"]))
        back_to_edge(plan["dropoff_tile"], (plan["final_heading"] + 180) % 360)
    elif plan["style"] in ("direct", "turn"):
        turn_to(float(plan["final_heading"]))
        # Pour point DUMP_POINT_FORWARD_CM ahead of the axle: put the axle plan["shift"] past the tile centre.
        # Measured from where the turn left it ("direct" turns SPIN_SHIFT_CM away from a house beside the tile).
        core.update_odometer()
        h = plan["final_heading"]
        off = core.tile_center_along(plan["dropoff_tile"], h) + plan.get("shift", 0.0) - core.get_along(h)
        if abs(off) > 0.5:
            core.drive_straight_distance(abs(off), forward=off > 0, hold_heading=h, speed_dps=POUR_SHIFT_DPS)
    stop_intake_roller()
    dump_snow_payload(plan["side"])
    # After a BACK pour the robot faces inland and turns in place next: stop EDGE_LANE_SHIFT_CM toward the table
    # edge (that side is open), away from the inland tile, which Phase 1 may never have seen (sim u_reversed:
    # an unseen house there, the funnel hit it turning on the centre).
    recentre_along(plan["dropoff_tile"], plan["final_heading"], blocked,
                   -EDGE_LANE_SHIFT_CM if plan["side"] == "BACK" else None)
    return tile


def spin_shift(tile, heading, blocked):
    """(along, lateral) cm, relative to `heading` (+ = forward / right), that moves the axle away from a house
    directly beside `tile` before turning in place there: a house beside the centre leaves 0.24 cm."""
    def axis(h):
        a = tile_step(tile, h) in blocked
        b = tile_step(tile, (h + 180) % 360) in blocked
        return 0.0 if a == b else (-SPIN_SHIFT_CM if a else SPIN_SHIFT_CM)
    return axis(int(heading) % 360), axis((int(heading) + 90) % 360)


def recentre_along(tile, heading, blocked, offset_cm=None):
    """After pouring the axle can be ~6 cm off the tile centre along `heading` (pour point / edge reverse).
    Drive straight back to the centre line (or SPIN_SHIFT_CM from it, away from a house beside the tile) before
    the way home: a turn there would carry that offset sideways into the next run, next to whatever houses it
    passes. offset_cm: stop this far from the centre along `heading` instead."""
    core.update_odometer()
    shift = spin_shift(tile, heading, blocked)[0] if offset_cm is None else offset_cm
    off = (core.tile_center_along(tile, heading) + shift) - core.get_along(heading)
    if abs(off) > 0.5:
        core.drive_straight_distance(abs(off), forward=off > 0, hold_heading=heading, speed_dps=150)


def leave_dropoff(tile, heading, blocked, known, home_tiles):
    """On a drop-off tile with a house beside it, don't start the way home with a turn in place there: if the
    planner can't avoid one, drive straight on or back out one tile first and turn there."""
    tile = tuple(tile)
    if tile not in TIGHT:
        return tile

    def way_home(start):
        best = None
        for goal in home_tiles:
            p = plan_path(start, goal, blocked, known, heading, TIGHT)
            if p and (best is None or len(p) < len(best)):
                best = p
        return best

    best = way_home(tile)
    if best and (len(best) < 2 or core.heading_between_adjacent_tiles(best[0], best[1]) == int(heading) % 360):
        # The way home leaves straight ahead. A one-side pour can leave the robot facing away from home on an
        # edge lined with houses: that "straight ahead" may run to the first free tile and back. Backing out
        # one tile to a tile it can turn on can be much shorter (sim random-9x10-35: 29 s -> ~10 s).
        back = tile_step(tile, heading, -1)
        if core.is_tile_in_bounds(*back) and back not in blocked and back not in TIGHT:
            via_back = way_home(back)
            if via_back and (BACK_OUT_SEC_PER_TILE + estimate_path_seconds(via_back, heading)
                             < estimate_path_seconds(best, heading)):
                print(f"[RETURN] {tile} has a house beside it: back to {back} before turning")
                return back_out(back, heading)
        return tile
    for step in (1, -1):
        nxt = tile_step(tile, heading, step)
        if core.is_tile_in_bounds(*nxt) and nxt not in blocked and (nxt not in TIGHT or step == -1):
            print(f"[RETURN] {tile} has a house beside it: {'forward' if step > 0 else 'back'} to {nxt} "
                  f"before turning")
            if step > 0:
                return drive_run(tile, heading, 1, CRUISE_DPS)["tile"]
            return back_out(nxt, heading)
    return tile


def back_to_edge(dropoff_tile, outward):
    """BACK dump: facing away from the bucket, reverse until the tray's rear lip is DUMP_BACK_OVERHANG_CM past
    the table edge, but never so far that the rearmost ground contact gets within DUMP_BACK_EDGE_MARGIN_CM."""
    core.update_odometer()
    edge = core.tile_center_along(dropoff_tile, outward) + core.TILE_SIZE / 2.0
    want = DUMP_BACK_LIP_CM - DUMP_BACK_OVERHANG_CM            # axle to edge
    floor = DUMP_BACK_SUPPORT_CM + DUMP_BACK_EDGE_MARGIN_CM
    if want < floor:
        print(f"[DUMP] back: lip would end {floor - DUMP_BACK_LIP_CM:+.1f} cm from the edge (support limit); "
              f"stopping there")
    dist = (edge - max(want, floor)) - core.get_along(outward)
    if dist > 0.1:
        core.drive_straight_distance(dist, forward=False, hold_heading=(outward + 180) % 360, speed_dps=120)


def pour_without_plan(tile, outward):
    """No approach option fits: pour from the drop-off tile centre in the first allowed direction."""
    side = (core.dump_directions() or ["RIGHT"])[0]
    turn_to(float({"RIGHT": outward - 90, "LEFT": outward + 90, "BACK": outward + 180}[side] % 360))
    if side == "BACK":
        back_to_edge(tile, outward)
    stop_intake_roller()
    dump_snow_payload(side)


def _arc_here(corner, h_in, h_out) -> bool:
    """A 90 deg turn on a tile with a house beside it is driven as an arc instead of a turn in place
    (ARC_EVERY_TURN: every 90 deg transit turn)."""
    return ((ARC_EVERY_TURN or tuple(corner) in TIGHT)
            and abs(abs(core.minimal_angle_diff(h_out, h_in)) - 90) < 1)


def travel(current, targets, blocked, known, speed_dps=CRUISE_DPS, then_heading=None):
    """Phase 2 transit (no obstacle checks: the map is known). Like navigate_tile_with_obstacle_avoidance,
    but never turns in place on a tile with a house beside it: the planner avoids U-turns and starting turns
    there, and 90 deg turns there are driven as arcs. targets: a tile or a list of tiles.
    then_heading: the heading the robot leaves the target in (the street entrance): a 90 deg turn there on a
    tile with a house beside it is driven as an arc too, ending on the way out."""
    targets = [tuple(targets)] if isinstance(targets[0], int) else [tuple(t) for t in targets]
    for _ in range(4):                       # re-plan if a run ends early (timeout)
        if current in targets:
            break
        core.update_odometer()
        path = []
        for goal in targets:
            p = plan_path(current, goal, blocked, known, core.theta, TIGHT)
            if len(p) >= 2 and (not path or estimate_path_seconds(p, core.theta)
                                < estimate_path_seconds(path, core.theta)):
                path = p
        if len(path) < 2:
            print(f"  [NAV] No path from {current} to {targets}; stopping here.")
            break
        vizlog.emit("plan", path=vizlog.tiles(path))
        runs = compile_runs(path)
        tile, early = current, False
        for i, (heading, n) in enumerate(runs):
            corner = tile_step(tile, heading, n)
            nxt = runs[i + 1][0] if i + 1 < len(runs) else then_heading
            if nxt is not None and _arc_here(corner, heading, nxt):
                res = drive_run(tile, heading, n, speed_dps, end_lane_offset_cm=core.arc_lane_offset(heading, nxt),
                                end_along_offset_cm=-core.TILE_SIZE / 2.0)
                if res["stop_reason"] != "end":
                    tile, early = core.cm_to_nearest_tile(core.x, core.y), True
                    break
                tile = core.drive_arc(corner, heading, nxt, speed_dps)
            else:
                res = drive_run(tile, heading, n, speed_dps)
                tile = res["tile"]
                if res["stop_reason"] != "end":
                    early = True
                    break
        current = tile
        if not early:
            break
    return current


def corner_boxes(street, blocked) -> set:
    """Tiles that may hold a corner box: diagonal to a street tile, with a house on both tiles between them
    (the outside of a street corner, e.g. (6,3) on the handout's sample map). Phase 1 never sees these.
    The 4 origin tiles never hold houses."""
    street = set(street)
    origin = set(core.ORIGIN_TILES)
    out = set()
    for sx, sy in street:
        for dx in (-1, 1):
            for dy in (-1, 1):
                d = (sx + dx, sy + dy)
                if (d not in street and d not in origin and core.is_tile_in_bounds(*d)
                        and (sx + dx, sy) in blocked and (sx, sy + dy) in blocked):
                    out.add(d)
    return out


def spin_safe(tile, blocked) -> bool:
    """A turn in place at this tile's centre clears the houses: none directly beside it (a house beside the
    centre is 15.24 cm away, the robot sweeps a 15 cm circle; a diagonal house corner is 21.5 cm away)."""
    return all(tile_step(tile, h) not in blocked for h in (0, 90, 180, 270))


def back_out(start_tile, heading, speed_dps=BACK_OUT_DPS, lane_offset_cm=0.0, end_along_offset_cm=0.0):
    """Reverses along `heading` to start_tile's centre on the tile centre line, without turning (the way out of
    a street when there's no room to turn round). Lane hold like drive_run's, with the steering sign flipped.
    lane_offset_cm / end_along_offset_cm: hold that lane (+ = right of heading), stop that far past the centre."""
    heading = int(heading) % 360
    core.update_odometer()
    u_goal = core.tile_center_along(start_tile, heading) + end_along_offset_cm
    lane = core.tile_center_lateral(start_tile, heading) + lane_offset_cm
    t0 = time.time()
    vizlog.emit("back_out", tile=list(start_tile), h=heading)
    while True:
        core.update_odometer()
        remaining = core.get_along(heading) - u_goal
        if remaining <= 0.0 or time.time() - t0 > 30.0:
            break
        cmd = core.compute_trapezoidal_dps(time.time() - t0, remaining, speed_dps)
        v_err = core.get_lateral(heading) - lane
        # Backing up, the robot moves toward the side its nose points away from: steer the other way.
        steer = max(-core.MAX_LATERAL_STEER_DEG, min(core.MAX_LATERAL_STEER_DEG, core.K_LATERAL_DEG_PER_CM * v_err))
        core.drive_with_heading_hold(-cmd, (heading + steer) % 360.0)
        time.sleep(0.008)
    core.stop_drive_motors()
    time.sleep(0.05)
    core.update_odometer()
    return start_tile


def run_phase2():
    print("=" * 65)
    print("  ECSE 211 FINAL PROJECT - PHASE 2: WINTER SNOW REMOVAL")
    print("=" * 65)

    m = load_street_map()
    approach_tile = tuple(m["approach_tile"])
    street = [tuple(t) for t in m["street_tiles"]]
    exit_tile = tuple(m["exit_tile"]) if m.get("exit_tile") else None
    blocked = {tuple(t) for t in m.get("blocked_tiles", [])}
    blocked.difference_update(street)
    blocked |= core.infer_houses(street, approach_tile, exit_tile)   # older maps may lack them
    blocked.difference_update(core.ORIGIN_TILES)                     # the 4 origin tiles never hold houses
    traversed = {tuple(t) for t in m.get("traversed_tiles", [])}
    known = traversed | set(street) | set(core.ORIGIN_TILES) | {approach_tile} | ({exit_tile} if exit_tile else set())
    if TRUST_MAP:
        blocked |= corner_boxes(street, blocked) - known
        blocked.difference_update(core.ORIGIN_TILES)
        known = {t for t in core.all_tiles() if t not in blocked}
    dropoff_tile = tuple(m["dropoff_tile"])
    dropoff_heading = int(m["dropoff_heading"])
    global TIGHT
    TIGHT = {t for t in core.all_tiles() if t not in blocked and not spin_safe(t, blocked)}

    if PHASE2_US_FORWARD_OFFSET_CM is not None:
        core.US_FORWARD_OFFSET_CM = PHASE2_US_FORWARD_OFFSET_CM
    if PHASE2_US_FACE_CM is not None:
        core.US_FACE_CM = PHASE2_US_FACE_CM

    sweep_end = street[-1] if street else approach_tile
    if street and exit_tile and abs(exit_tile[0] - street[-1][0]) + abs(exit_tile[1] - street[-1][1]) == 1:
        sweep_end = exit_tile
    plan = choose_dump_plan(sweep_end, dropoff_tile, dropoff_heading, blocked, known)
    print(f"  Approach {approach_tile}, street ({len(street)}): {street}")
    print(f"  Drop-off {dropoff_tile}: " + (f"{plan['style']}: enter from {plan['pre_tile']} heading {plan['heading']}, "
                                           f"pour {plan['side']} facing {plan['final_heading']}"
                                           if plan else "no edge-parallel approach, will pour from the tile centre"))
    input("Install Phase 2 hardware, place robot at Origin facing +Y, press Enter to start Phase 2...")

    init_hardware(phase=2, need_us=PHASE2_SIDE_US is not None)
    core.FIXED_US_ANGLE = None if PHASE2_SIDE_US is None else 90.0 * PHASE2_SIDE_US
    vizlog.start(2, s1=m.get("s1"), s2=m.get("s2"), d1=m.get("d1"), d2=m.get("d2"),
                 dropoff=list(dropoff_tile), outline=core.ROBOT_OUTLINE[2], us_fwd=core.US_FORWARD_OFFSET_CM, us_face=core.US_FACE_CM,
                 sensor_offset=core.SENSOR_OFFSET, sensor_spacing=core.SENSOR_SPACING)
    vizlog.emit("map", street=vizlog.tiles(street), blocked=vizlog.tiles(sorted(blocked)),
                exit=list(exit_tile) if exit_tile else None, approach=list(approach_tile))
    phase_start = time.time()

    # Reach the entrance without driving through the street (it would push the snow around, and arrive at
    # the entrance facing the wrong way), unless there's no other way.
    to_gate = blocked | set(street)
    start_tile, first_heading = core.choose_start([approach_tile], to_gate, known, TIGHT)
    if not plan_path(start_tile, approach_tile, to_gate, known, None, TIGHT):
        to_gate = blocked
        start_tile, first_heading = core.choose_start([approach_tile], blocked, known, TIGHT)
    step_origin_to_start_tile(start_tile, first_heading, room_ahead=True)   # first_heading: from a known path
    first_heading = core.heading_between_adjacent_tiles(approach_tile, street[0]) if street else None
    current = travel(start_tile, approach_tile, to_gate, known, then_heading=first_heading)

    route = []
    if street:
        lead_in = plan_path(current, street[0], blocked, known, core.theta, TIGHT)
        if lead_in and len(lead_in) <= 3:
            route = lead_in + street[1:]
            # Sweep on to the exit tile: a street that ends in a corner then gets an arc there too, and the
            # robot leaves the street before it turns in place.
            if (exit_tile and exit_tile not in blocked and core.is_tile_in_bounds(*exit_tile)
                    and abs(exit_tile[0] - street[-1][0]) + abs(exit_tile[1] - street[-1][1]) == 1):
                route.append(exit_tile)
        else:
            print(f"[SWEEP] Could not reach the street entrance from {current}; skipping the sweep.")
    for t in (route[:1] + route[-1:]):
        if not spin_safe(t, blocked):
            print(f"[SWEEP] Warning: {t} has a house right beside it; turning in place there is tight.")
    if route:
        start_intake_roller()

    runs = compile_runs(route) if route else []
    starts = [route[0]] if route else []
    for heading, n in runs:
        starts.append(tile_step(starts[-1], heading, n))
    nexts = [runs[i + 1][0] for i in range(len(runs) - 1)] + [None] if runs else []
    radii = [None] * len(runs)
    if runs and plan is not None and ARC_CORNERS:
        # Leaving the street toward the drop-off turns 90 deg at its last tile: an arc too (that tile is the exit,
        # or the last street tile, which then gets the corner poke like any street corner).
        h_end = runs[-1][0]
        dp = drop_runs(route[-1], plan, blocked, known, h_end)
        if dp[1] and _arc_here(route[-1], h_end, dp[1][0][0]):
            nexts[-1] = dp[1][0][0]
            radii[-1] = arc_radius_for(route[-1], h_end, nexts[-1], dp[3][0], blocked)
    arcs = [ARC_CORNERS and nexts[i] is not None and abs(abs(core.minimal_angle_diff(nexts[i], h)) - 90) < 1
            for i, (h, n) in enumerate(runs)]
    if route:
        end_finish = finish_seconds(route[-1], plan, blocked, known)
        back_ok = spin_safe(route[0], blocked)
        start_finish = finish_seconds(route[0], plan, blocked, known) if back_ok else None

    def way_out(i, k):
        """("forward" | "back", seconds) cheapest way to finish from k tiles into run i, never turning in place
        inside the street: drive the rest of the street at cruise speed, or (still on the first straight)
        reverse out to where the sweep started."""
        tiles = runs[i][1] - k + sum(n for _, n in runs[i + 1:])
        n_runs = (1 if runs[i][1] > k else 0) + len(runs) - i - 1
        fwd = (tiles * core.EST_SEC_PER_TILE + n_runs * core.EST_SEC_PER_RUN
               + sum(arcs[i:]) * core.EST_SEC_PER_TURN + end_finish)
        if i == 0 and back_ok:            # k == 0: still on the start tile, i.e. skip the street
            back = (k * BACK_OUT_SEC_PER_TILE + core.EST_SEC_PER_RUN if k else 0.0) + start_finish
            if back < fwd:
                return "back", back
        return "forward", fwd

    state = {"i": 0}

    def hook(idx, tile):
        # The run stops one tile further on (at tiles[idx + 1]) when this says so.
        return should_abort_for_time(phase_start, way_out(state["i"], idx + 1)[1])

    def drive_runs(i0, speed, hook_fn, sweeping):
        """Drives runs[i0:] (arcs at corners). Returns None when done, else the run index it stopped in."""
        for i in range(i0, len(runs)):
            heading, n = runs[i]
            state["i"] = i
            if hook_fn is not None and should_abort_for_time(phase_start, way_out(i, 0)[1]):
                return i
            # Every run starts from its planned start tile: when the wrap-up re-drives a run the robot stopped
            # in, drive_run just carries on from where it is (lines already passed are skipped).
            if not arcs[i]:
                res = drive_run(starts[i], heading, n, speed, side_us=PHASE2_SIDE_US, tile_hook=hook_fn)
                if res["stop_reason"] != "end":
                    return i
                continue
            nxt = nexts[i]
            # Stop on the corner tile's entry edge, already on the arc's lane, then drive the arc.
            res = drive_run(starts[i], heading, n, speed,
                            end_lane_offset_cm=core.arc_lane_offset(heading, nxt, radii[i]),
                            end_along_offset_cm=-core.TILE_SIZE / 2.0, side_us=PHASE2_SIDE_US, tile_hook=hook_fn)
            if res["stop_reason"] != "end":
                return i
            corner = starts[i + 1]
            rest = way_out(i + 1, 0)[1] if i + 1 < len(runs) else end_finish
            if sweeping and CORNER_POKE_CM > 0 and corner in street and not should_abort_for_time(
                    phase_start, rest + CORNER_POKE_SPARE_SEC):
                # The arc cuts across the corner tile; this picks up what lies in its outer part.
                core.drive_straight_distance(CORNER_POKE_CM, True, hold_heading=heading, speed_dps=SWEEP_DPS)
                core.drive_straight_distance(CORNER_POKE_CM, False, hold_heading=heading, speed_dps=CRUISE_DPS)
            core.drive_arc(corner, heading, nxt, speed, radius=radii[i])
        return None

    print(f"\n[SWEEP] {max(0, len(route) - 1)} tiles at {SWEEP_DPS} dps")
    vizlog.emit("plan", path=vizlog.tiles(route), sweep=True)
    if runs:
        stopped = drive_runs(0, SWEEP_DPS, hook, True)
        if stopped is None:
            current = route[-1]
        else:
            # Cut short inside the street: leave it without turning in place (no room between the houses).
            i = stopped
            core.update_odometer()
            k = int(round((core.get_along(runs[i][0]) - core.tile_center_along(starts[i], runs[i][0]))
                          / core.TILE_SIZE))
            k = max(0, min(runs[i][1], k))
            how, secs = way_out(i, k)
            print(f"[SWEEP] Wrapping up from run {i + 1}/{len(runs)}, {k} tile(s) in: "
                  + (("backing out to " + str(route[0]) if k else "not entering the street") if how == "back"
                     else "driving the rest of the street")
                  + f" (~{secs:.0f} s to finish)")
            if how == "back":
                current = back_out(route[0], runs[0][0]) if k else route[0]
            else:
                left = drive_runs(i, CRUISE_DPS, None, False)
                current = route[-1] if left is None else core.cm_to_nearest_tile(core.x, core.y)
    time.sleep(ROLLER_FLUSH_SEC)
    if not ROLLER_ON_DURING_TRANSIT:
        stop_intake_roller()

    if tuple(current) != tuple(sweep_end):
        # The sweep was cut short (backed out): the plan was chosen for the street's end. A one-side pour may
        # now be a U-turn away from it, next to a house: choose again from here.
        plan = choose_dump_plan(current, dropoff_tile, dropoff_heading, blocked, known) or plan
    if plan is not None:
        print(f"\n[DROP-OFF] {current} -> {plan['pre_tile']} -> {dropoff_tile}")
        current = drive_to_dump_and_pour(current, plan, blocked, known)
    else:
        current = travel(current, dropoff_tile, blocked, known)
        pour_without_plan(dropoff_tile, dropoff_heading)

    print(f"\n[RETURN] {current} -> Origin")
    home_tiles = [t for t in core.ORIGIN_TILES if t not in blocked]
    # The last step onto the origin turns in place on the home tile: avoid one with a house beside it.
    home_tiles = [t for t in home_tiles if t not in TIGHT] or home_tiles
    current = leave_dropoff(current, int(round(core.theta / 90.0)) * 90 % 360, blocked, known, home_tiles)
    current = travel(current, home_tiles, blocked, known)
    step_tile_to_origin()

    total = time.time() - phase_start
    print("\n" + "=" * 65)
    print(f"  PHASE 2 COMPLETE in {total:.1f}s (limit 120 s).")
    print("=" * 65)
    vizlog.emit("end", total=round(total, 1))
    vizlog.close()
    return total


if __name__ == "__main__":
    try:
        run_phase2()
    except BaseException as e:
        print(f"\n[PHASE 2 TERMINATED]: {type(e).__name__}: {e}")
        traceback.print_exc()
    finally:
        vizlog.close()
        stop_all_motors()
        reset_brick()
