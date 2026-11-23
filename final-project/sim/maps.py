"""
Test boards. Tiles are named by their lower-left intersection (same as the robot code).
`street` is the ground-truth street (without the exit tile) used to score the mapping.
"""

MAPS = {
    # Figure 1 of the project handout.
    "sample": {
        "boxes": [(x, 3) for x in range(1, 7)] + [(6, y) for y in range(0, 3)]
                 + [(x, 1) for x in range(1, 5)] + [(4, 0)],
        "s1": (1, 3), "s2": (1, 2), "d1": (2, -1), "d2": (3, -1),
        "street": [(1, 2), (2, 2), (3, 2), (4, 2), (5, 2), (5, 1), (5, 0)],
    },
    # U-shaped street whose gate is on the side nearer the origin, so the first guess of
    # "which side is outside" is wrong and the robot has to detect it.
    "u_reversed": {
        "boxes": [(1, 0), (1, 1), (1, 2), (1, 3), (3, 0), (3, 1), (4, 0), (4, 1), (5, 0), (5, 1),
                  (7, 0), (7, 1), (7, 2), (7, 3), (2, 3), (3, 3), (4, 3), (5, 3), (6, 3)],
        "s1": (2, 0), "s2": (3, 0), "d1": (4, -1), "d2": (5, -1),
        "street": [(2, 0), (2, 1), (2, 2), (3, 2), (4, 2), (5, 2), (6, 2), (6, 1), (6, 0)],
    },
    # 12-tile street along the bottom edge, up, and back west; drop-off on the left edge.
    "long": {
        "boxes": [(1, 0), (2, 0), (3, 0), (4, 0), (6, -1), (6, 0), (6, 1), (6, 2), (4, 1),
                  (3, 1), (2, 1), (1, 1), (1, 3), (2, 3), (3, 3), (4, 3), (5, 3)],
        "s1": (1, -1), "s2": (1, 0), "d1": (-1, 2), "d2": (-1, 3),
        "street": [(1, -1), (2, -1), (3, -1), (4, -1), (5, -1), (5, 0), (5, 1), (5, 2),
                   (4, 2), (3, 2), (2, 2), (1, 2)],
    },
}


# ------------------------------------------------------------------ table size and random boards
# Tables are given in tiles: (x_min, x_max, y_min, y_max), the same as snowplow_core.TILE_X_MIN.. One table is
# 9 x 5 tiles with the origin one tile in from the lower-left corner. Two tables side by side: 18 x 5 (x_max 16);
# one above the other: 9 x 10 (y_max 8).
DEFAULT_TABLE = (-1, 7, -1, 3)
ORIGIN_TILES = [(0, 0), (-1, 0), (0, -1), (-1, -1)]
DIRS = [(0, 1), (1, 0), (0, -1), (-1, 0)]


def table_of(spec):
    return tuple(spec.get("table", DEFAULT_TABLE))


def table_for_size(w, h):
    """w x h tiles, lower-left tile (-1, -1)."""
    return (-1, w - 2, -1, h - 2)


def _add(a, b, k=1):
    return (a[0] + k * b[0], a[1] + k * b[1])


def _in(t, tb):
    return tb[0] <= t[0] <= tb[1] and tb[2] <= t[1] <= tb[3]


def _on_ring(t, tb):
    return t[0] in (tb[0], tb[1]) or t[1] in (tb[2], tb[3])


def _edge_points(a, b):
    """Grid intersections at the ends of the edge between adjacent tiles a and b (tiles are named by their
    lower-left intersection)."""
    if a[1] == b[1]:
        x = max(a[0], b[0])
        return (x, a[1] + 1), (x, a[1])
    y = max(a[1], b[1])
    return (a[0], y), (a[0] + 1, y)


def _reachable(start_tiles, free, tb):
    seen, todo = set(), [t for t in start_tiles if t in free]
    seen.update(todo)
    while todo:
        t = todo.pop()
        for d in DIRS:
            n = _add(t, d)
            if _in(n, tb) and n in free and n not in seen:
                seen.add(n)
                todo.append(n)
    return seen


def _direct_route(starts, goal, free):
    """A house-free shortest route (staircase, only moving toward the goal) from one of `starts` to `goal`:
    the strict reading of "no houses between A and B"."""
    for a in starts:
        if a not in free:
            continue
        sx = 1 if goal[0] >= a[0] else -1
        sy = 1 if goal[1] >= a[1] else -1
        ok = {a}
        for i in range(abs(goal[0] - a[0]) + 1):
            for j in range(abs(goal[1] - a[1]) + 1):
                t = (a[0] + sx * i, a[1] + sy * j)
                if t == a or t not in free:
                    continue
                if (t[0] - sx, t[1]) in ok or (t[0], t[1] - sy) in ok:
                    ok.add(t)
        if goal in ok:
            return True
    return False


def random_map(seed, table=DEFAULT_TABLE, edge_streets=False, corner_box_p=0.7, min_len=4, max_len=None,
               strict=True):
    """
    A random board under the project rules, as the robot code assumes them:
      - the street is one tile wide (no two street tiles touch unless they follow each other), no dead ends,
        houses on every table tile beside it; edge_streets=False keeps it off the outer ring of tiles (room for
        a house on both sides), True lets the table edge stand in for houses there (like the "long" map);
      - the outside corner of a street corner has a house with probability corner_box_p;
      - the tile before the gate and the tile after the street's end are open, with no house beside the exit;
      - no other houses; the origin tiles and the drop-off tile are free; "no houses between" the origin and
        the gate, the street's end and the origin, the street's end and the drop-off: strict=True means a
        direct (staircase) route between them is house-free, strict=False only that some route is.
    """
    import random
    rng = random.Random(seed)
    tb = tuple(table)
    n_tiles = (tb[1] - tb[0] + 1) * (tb[3] - tb[2] + 1)
    max_len = max_len or max(min_len, min(18, n_tiles // 3))
    allowed = {(x, y) for x in range(tb[0], tb[1] + 1) for y in range(tb[2], tb[3] + 1)
               if (edge_streets or not _on_ring((x, y), tb)) and (x, y) not in ORIGIN_TILES}
    for _attempt in range(4000):
        target = rng.randint(min_len, max_len)
        street = [rng.choice(sorted(allowed))]
        d = rng.choice(DIRS)
        while len(street) < target:
            opts = []
            for nd in DIRS:
                if nd == (-d[0], -d[1]):
                    continue
                n = _add(street[-1], nd)
                if n not in allowed or n in street:
                    continue
                if any(_add(n, dd) in street[:-1] for dd in DIRS):
                    continue                    # would touch an earlier street tile: two tiles wide
                opts.append((3.0 if nd == d else 1.0, nd))
            if not opts:
                break
            r, acc = rng.uniform(0, sum(w for w, _ in opts)), 0.0
            for w_, nd in opts:
                acc += w_
                if r <= acc:
                    d = nd
                    break
            street.append(_add(street[-1], d))
        if len(street) < min_len:
            continue
        sset = set(street)

        def ends(tile, inner):
            """Open tiles just outside a street end: not on the street, not beside another street tile."""
            out = []
            for dd in DIRS:
                n = _add(tile, dd)
                if n == inner or not _in(n, tb) or n in sset or n in ORIGIN_TILES:
                    continue
                if any(_add(n, k) in sset and _add(n, k) != tile for k in DIRS):
                    continue
                straight = (tile[0] - inner[0], tile[1] - inner[1]) == dd
                out.append((3.0 if straight else 1.0, n))
            return out

        def pick(opts):
            r, acc = rng.uniform(0, sum(w for w, _ in opts)), 0.0
            for w_, n in opts:
                acc += w_
                if r <= acc:
                    return n
            return opts[-1][1]

        a_opts, e_opts = ends(street[0], street[1]), ends(street[-1], street[-2])
        if not a_opts or not e_opts:
            continue
        approach, exit_tile = pick(a_opts), pick(e_opts)
        if approach == exit_tile:
            continue
        houses = set()
        for i, t in enumerate(street):
            for dd in DIRS:
                n = _add(t, dd)
                if not _in(n, tb) or n in sset:
                    continue
                if (i == 0 and n == approach) or (i == len(street) - 1 and n == exit_tile):
                    continue
                houses.add(n)
        if houses & set(ORIGIN_TILES) or approach in houses or exit_tile in houses:
            continue
        ex_dir = (exit_tile[0] - street[-1][0], exit_tile[1] - street[-1][1])
        if any(_add(exit_tile, (ex_dir[1] * s, -ex_dir[0] * s)) in houses for s in (1, -1)):
            continue                            # a house beside the exit tile: the street wouldn't end there
        for i in range(1, len(street) - 1):     # outside corners
            a, b, c = street[i - 1], street[i], street[i + 1]
            d1, d2 = (b[0] - a[0], b[1] - a[1]), (c[0] - b[0], c[1] - b[1])
            if d1 != d2:
                corner = _add(_add(b, d1), (-d2[0], -d2[1]))
                beside_end = any(_add(corner, k) in (approach, exit_tile) for k in DIRS)
                if (_in(corner, tb) and corner not in sset and corner not in (approach, exit_tile)
                        and corner not in ORIGIN_TILES and not beside_end and rng.random() < corner_box_p):
                    houses.add(corner)
        free = {(x, y) for x in range(tb[0], tb[1] + 1) for y in range(tb[2], tb[3] + 1)} - houses - sset
        # drop-off: a boundary edge whose table tile is free and reachable from the street's end
        reach_exit = _reachable([exit_tile], free, tb)
        if not set(ORIGIN_TILES) & reach_exit or approach not in _reachable(ORIGIN_TILES, free, tb):
            continue
        if strict and not (_direct_route(ORIGIN_TILES, approach, free)
                           and any(_direct_route([exit_tile], o, free) for o in ORIGIN_TILES)):
            continue
        drops = []
        for t in sorted(reach_exit):
            if t in ORIGIN_TILES or t in (approach, exit_tile):
                continue
            if strict and not _direct_route([exit_tile], t, free):
                continue
            for dd in DIRS:
                if not _in(_add(t, dd), tb):
                    drops.append((t, dd))
        if not drops:
            continue
        drop_t, drop_d = rng.choice(drops)
        s1, s2 = _edge_points(approach, street[0])
        d1, d2 = _edge_points(drop_t, _add(drop_t, drop_d))
        if rng.random() < 0.5:
            s1, s2 = s2, s1
        if rng.random() < 0.5:
            d1, d2 = d2, d1
        straight_in = (street[0][0] - approach[0], street[0][1] - approach[1]) == (street[1][0] - street[0][0],
                                                                                  street[1][1] - street[0][1])
        straight_out = (exit_tile[0] - street[-1][0], exit_tile[1] - street[-1][1]) == (
            street[-1][0] - street[-2][0], street[-1][1] - street[-2][1])
        spec = {"boxes": sorted(houses), "s1": s1, "s2": s2, "d1": d1, "d2": d2, "street": street,
                "table": tb, "approach": approach, "exit": exit_tile, "dropoff": drop_t,
                "turn_at_gate": not straight_in, "turn_at_exit": not straight_out}
        return spec
    raise RuntimeError(f"no valid random map for seed {seed} on table {tb}")


def parse_random_name(name):
    """'random-SEED', 'random-WxH-SEED', optionally with '-edge' (street may use the outer ring) and / or
    '-loose' (only some house-free route between origin, gate, street end and drop-off, not a direct one)
    before the seed. Returns (seed, table, edge, strict) or None."""
    parts = name.split("-")
    if parts[0] != "random" or len(parts) < 2 or not parts[-1].isdigit():
        return None
    table, edge, strict = DEFAULT_TABLE, False, True
    for p in parts[1:-1]:
        if p == "edge":
            edge = True
        elif p == "loose":
            strict = False
        elif "x" in p and all(q.isdigit() for q in p.split("x")) and p.count("x") == 1:
            w, h = (int(q) for q in p.split("x"))
            if w < 4 or h < 4 or w * h > 400:
                return None
            table = table_for_size(w, h)
        else:
            return None
    return int(parts[-1]), table, edge, strict


def is_map_name(name):
    return name in MAPS or parse_random_name(str(name)) is not None


_cache = {}


def get_map(name):
    """A fixed map from MAPS, or a random one named random[-WxH][-edge][-loose]-SEED (same name = same board)."""
    if name in MAPS:
        return MAPS[name]
    if name not in _cache:
        p = parse_random_name(name)
        if p is None:
            raise KeyError(f"unknown map {name!r}: one of {sorted(MAPS)} or random[-WxH][-edge]-SEED")
        seed, table, edge, strict = p
        _cache[name] = random_map(seed, table, edge, strict=strict)
    return _cache[name]


def render(spec):
    """ASCII picture of a board (top row first): # house, digits street order, A approach, E exit, D drop-off,
    O origin tiles."""
    tb = table_of(spec)
    street = {t: i for i, t in enumerate(spec["street"])}
    houses = set(map(tuple, spec["boxes"]))
    rows = []
    for y in range(tb[3], tb[2] - 1, -1):
        row = ""
        for x in range(tb[0], tb[1] + 1):
            t = (x, y)
            if t in houses:
                c = "#"
            elif t in street:
                c = "0123456789abcdefghijklmnopqrstuvwxyz"[street[t] % 36]
            elif t == tuple(spec.get("approach", ())):
                c = "A"
            elif t == tuple(spec.get("exit", ())):
                c = "E"
            elif t == tuple(spec.get("dropoff", ())):
                c = "D"
            elif t in ORIGIN_TILES:
                c = "O"
            else:
                c = "."
            row += c + " "
        rows.append(f"{y:>3} {row}")
    return "\n".join(rows)
