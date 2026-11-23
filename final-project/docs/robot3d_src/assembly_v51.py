"""Snowplow robot v5 (19.2 cm roller held at both ends, 22 cm wide, rear ball caster, guided pusher): every LEGO part placed on real LDraw hole positions, joined by explicit pins/axles.

Frame: cm, x right, y up, z BACKWARD (forward is -z), origin = drive-axle line on the table (y=0).
Run: python3 assembly.py [out.json]   -> prints the joint check, kit tally, writes placements for robot3d.html.
"""
import sys, json, math
import numpy as np
sys.path.insert(0, "/tmp/robot3d")
import holes as HL

X, Y, Z = np.eye(3)
V = lambda *a: np.array(a, float)
PARTS, BY, CONN, TAPE = [], {}, [], []
BEAM = {3: "32523", 5: "32316", 7: "32524", 9: "40490", 11: "32525", 13: "41239", 15: "32278"}
AXLE = {2: "3704", 3: "4519", 4: "3705", 5: "32073", 6: "3706", 7: "44294", 8: "3707", 9: "60485", 10: "3737", 12: "3708"}
KITNAME = {"32523": "3-hole beam (No.39)", "32316": "5-hole beam (No.38)", "32524": "7-hole beam (No.42)",
           "40490": "9-hole beam (No.43)", "32525": "11-hole beam (No.44)", "41239": "13-hole beam (No.45)",
           "32278": "15-hole beam (No.46)", "32526": "L-beam 3x5 (No.48)", "6536": "cross block (No.30)",
           "95658": "EV3 Large motor", "99455": "EV3 Medium motor", "53787": "NXT motor", "95650": "EV3 colour sensor",
           "56908": "wheel rim (No.16)", "41897": "tyre (No.17)", "2780": "black pin (No.1)", "6558": "blue 3L pin (No.8)",
           "3713": "bush (No.58)", "59443": "axle joiner (No.18)", "92911c01": "ball caster (steel ball)"}
KIT = {"3-hole beam (No.39)": 16, "5-hole beam (No.38)": 18, "7-hole beam (No.42)": 20, "9-hole beam (No.43)": 14,
       "11-hole beam (No.44)": 6, "13-hole beam (No.45)": 10, "15-hole beam (No.46)": 2, "L-beam 3x5 (No.48)": 14,
       "cross block (No.30)": 8, "EV3 Large motor": 2, "EV3 Medium motor": 1, "NXT motor": 2, "EV3 colour sensor": 2,
       "wheel rim (No.16)": 4, "tyre (No.17)": 4, "black pin (No.1)": 61, "blue 3L pin (No.8)": 52, "bush (No.58)": 11, "axle joiner (No.18)": 6, "ball caster (steel ball)": 1}
COL = {"beam": 71, "L": 0, "6536": 71, "2780": 0, "6558": 1, "axle": 0, "3713": 0, "dev": 15, "rim": 71, "tyre": 0}


def Rb(ex, ey, ez):
    R = np.column_stack([ex, ey, ez]).astype(float)
    assert abs(np.linalg.det(R) - 1) < 1e-6, R
    return R


def add(name, f, R, t, col, group="static", holes=None):
    p = dict(name=name, f=f, R=np.array(R, float), t=np.array(t, float), col=col, group=group)
    if holes is None:
        p["holes"] = [(p["R"] @ c + p["t"], p["R"] @ a, k) for c, a, k, _ in HL.holes(f + ".dat")]
        if f == "6536": p["holes"].append((p["t"], p["R"] @ X, "axle"))     # LDraw marks only the face of this axle hole
    else:
        p["holes"] = [(V(*c), V(*a), k) for c, a, k in holes]
    assert name not in BY, name
    PARTS.append(p); BY[name] = p
    return p


def beam(name, n, a, u, h0, group="static"):
    """n-hole straight beam, hole 0 at h0, extending along u, holes along a."""
    a, u = V(*a), V(*u)
    return add(name, BEAM[n], Rb(np.cross(a, u), a, u), V(*h0) + (n - 1) / 2 * 0.8 * u, COL["beam"], group)


def lbeam(name, corner, long_dir, short_dir, group="static"):
    """3x5 L-beam: corner hole at `corner`, 5-hole arm toward long_dir, 3-hole arm toward short_dir."""
    ez, ex = V(*long_dir), V(*short_dir)
    return add(name, "32526", Rb(ex, np.cross(ez, ex), ez), V(*corner) + 3.2 * ez, COL["L"], group)


def conn(kind, centre, axis, group="static", why=""):
    """kind '2780' (2L pin), '6558' (3L pin), 'axleN'. Centred at `centre` along `axis`."""
    u = V(*axis) / np.linalg.norm(axis)
    L = {"2780": 1.6, "6558": 2.4}.get(kind) or 0.8 * int(kind[4:])
    ref = Y if abs(u @ Y) < 0.9 else Z
    ey = ref - (ref @ u) * u; ey /= np.linalg.norm(ey)
    c = dict(kind=kind, f=AXLE[int(kind[4:])] if kind.startswith("axle") else kind, centre=V(*centre), u=u, L=L,
             R=Rb(u, ey, np.cross(u, ey)), col=COL["axle"] if kind.startswith("axle") else COL[kind], group=group, why=why)
    CONN.append(c)
    return c


def axle_span(n, a, b, group="static", why=""):
    a, b = V(*a), V(*b)
    assert abs(np.linalg.norm(b - a) - 0.8 * n) < 1e-6, (n, a, b)
    return conn(f"axle{n}", (a + b) / 2, b - a, group, why)


def tape(a, b): TAPE.append((a, b))


# =====================================================================================================
HUB_Y, MX = 2.8, 6.4                 # drive hub height (56 mm wheels), motor centre x
WX = 9.6                             # wheel centre x: tyre outer face at 11.0
YC, ZC = 4.4, -8.8                   # roller axis (ZC far enough forward that the flaps clear the rail tips)
TUBE_L, TUBE_R = -10.0, 9.2          # tube from -10.0 to 9.2 (19.2 cm); right bearing frame in planes x 9.8 / 10.6
FX = 10.6                            # left roller frame plane (outer face at -11.0)
for s, side in ((-1, "L"), (1, "R")):
    add(f"motor_{side}", "95658", np.eye(3), (s * MX, HUB_Y, 0), COL["dev"])
    # wheel: rim + tyre on an 8L axle through the rail, the hub and the rim
    Rw = Rb(V(0, 0, -s), Y, s * X)
    add(f"rim_{side}", "56908", Rw, (s * WX, HUB_Y, 0), COL["rim"])
    add(f"tyre_{side}", "41897", Rw, (s * WX, HUB_Y, 0), COL["tyre"]); tape(f"tyre_{side}", f"rim_{side}")
    axle_span(8, (s * 4.4, HUB_Y, 0), (s * 10.8, HUB_Y, 0), why="drive axle: rail -> hub -> rim")
    # rail: 15-hole beam on edge, pinned to the motor foot (z 5.6, 7.2); carries the card tray
    beam(f"rail_{side}", 15, X, Z, (s * 4.8, HUB_Y, -4.0))
    for z in (5.6, 7.2): conn("6558", (s * 5.6, HUB_Y, z), X, why=f"rail_{side} -> motor foot")
    # back face, layer 1: 7-hole (top, 3 pins) and 3-hole (bottom, 2 pins) beams on the motor's back holes
    beam(f"backT_{side}", 7, Z, s * X, (s * 4.0, 6.8, 8.8))
    beam(f"backB_{side}", 3, Z, s * X, (s * 6.4, 3.6, 8.8))
    for x in (5.6, 6.4, 7.2): conn("2780", (s * x, 6.8, 8.4), Z, why=f"back beams_{side} -> motor back")
    for x in (6.4, 7.2): conn("2780", (s * x, 3.6, 8.4), Z, why=f"back beams_{side} -> motor back")
    # layer 2: sensor post at x 8.0 (clear of the motor's cable socket) pinned to both back beams
    beam(f"post_{side}", 11 if s > 0 else 7, Z, Y, (s * 8.0, 2.8, 9.6))
    for y in (3.6, 6.8): conn("2780", (s * 8.0, y, 9.2), Z, why=f"post_{side} -> back beams")
    # colour sensor: hangs on the post, eye down 0.8 cm above the table, body outward
    add(f"colour_{side}", "95650", Rb(V(0, 0, -s), s * X, V(0, -1, 0)), (s * 8.0, 3.6, 10.4), COL["dev"])
    conn("2780", (s * 8.0, 4.4, 10.0), Z, why=f"colour_{side} -> post")
    conn("6558", (s * 8.0, 2.8, 9.6), Z, why=f"caster bar_{side} -> post -> colour_{side}")
    # stem L-beams: both sides of the 1-stud stem above the hub, joined by 3L pins through the stem
    lbeam(f"stemIn_{side}", (s * 5.6, 5.2, 1.6), Y, Z)
    lbeam(f"stemOut_{side}", (s * 7.2, 5.2, 1.6), Y, Z)
    for z in (1.6, 3.2): conn("6558", (s * 6.4, 5.2, z), X, why=f"stem L-beams_{side} -> motor stem")
    # upright: 7-hole beam on the inner L, up to the top rails
    beam(f"upright_{side}", 7, X, Y, (s * 4.8, 6.8, 1.6))
    for y in ((6.8, 7.6) if s > 0 else (6.8, 7.6, 8.4)): conn("6558", (s * 5.6, y, 1.6), X, why=f"upright_{side} -> stem L")
    # top rail: 9-hole along z at y 10.8 (right one also takes the NXT)
    beam(f"top_{side}", 9, X, Z, (s * 4.0, 10.8, 1.6))
    conn("2780", (s * 4.4, 10.8, 1.6), X, why=f"top_{side} -> upright")
    # Pi-case ears: L-beams standing on the top rails
    lbeam(f"earF_{side}", (s * 4.8, 10.8, 2.4), Y, Z)
    lbeam(f"earR_{side}", (s * 4.8, 10.8, 8.0), Y, -Z)
    for z in (2.4, 4.0, 8.0): conn("2780", (s * 4.4, 10.8, z), X, why=f"ears_{side} -> top rail")
    for y in (12.4, 13.2):
        for z in (2.4, 8.0): conn("2780", (s * 4.4, y, z), X, why=f"Pi case -> ear_{side}")

# layer 2 crossbar: 13-hole across the back, 2 pins into each top back beam -> the two motors are one block
beam("backX", 13, Z, X, (-4.8, 6.8, 9.6))
for x in (-4.8, -4.0, 4.0, 4.8): conn("2780", (x, 6.8, 9.2), Z, why="back crossbar -> top back beams")

# NXT (pusher motor): lies on the top rails, pinned at the back to a 9-hole beam on the right post
add("nxt", "53787", np.eye(3), (2.4, 10.0, 0.0), COL["dev"])
beam("nxtBack", 9, Z, X, (1.6, 10.0, 8.8))
for x in (1.6, 2.4, 3.2): conn("2780", (x, 10.0, 8.4), Z, why="NXT back -> nxtBack beam")
conn("2780", (8.0, 10.0, 9.2), Z, why="nxtBack -> right post")
axle_span(5, (2.0, 8.4, 1.6), (6.0, 8.4, 1.6), why="NXT front stem -> right upright -> right stem L")
conn("2780", (3.6, 10.8, 6.4), X, why="NXT rear box -> right top rail")
for z in (5.6, 7.2): axle_span(12, (-4.8, 10.8, z), (4.8, 10.8, z), why="cross axle: left top rail -> NXT -> right top rail")

# Pi case (BrickPi3 acrylic case, Technic holes on its long sides) -- not an LDraw part
PI = dict(x=4.4, y0=11.2, y1=15.6, z0=1.2, z1=9.6)
add("pi_case", "PI", np.eye(3), (0, 0, 0), 0,
    holes=[((s * 4.0, y, z), (1, 0, 0), "pin") for s in (-1, 1) for y in (12.4, 13.2) for z in (2.4, 8.0)])

# ---------------- roller module ----------------
# EV3 Medium inside the tube, output face at x -0.6 pointing right; 4 x 13-hole beams carry it out of the left end
OUT_X = -0.6
add("medium", "99455", Rb(Y, Z, X), (OUT_X, YC, ZC), COL["dev"])
for y, yf in ((6.0, 5.6), (2.8, 3.2)):
    for z in (ZC - 0.8, ZC + 0.8):
        nm = f"mbeam_{'T' if y > YC else 'B'}{'F' if z < ZC else 'R'}"
        beam(nm, 13, Y, -X, (OUT_X - 0.4, y, z))
        conn("2780", (OUT_X - 0.4, yf, z), Y, why=f"{nm} -> EV3 Medium side hole")
# left frame, one plane at x -10.6: L-beams make the beam pairs rigid, crosspieces + cross blocks tie it to the chassis
for y, tag in ((5.2, "T"), (3.6, "B")):
    lbeam(f"frameL{tag}", (-FX, y, ZC - 0.8), Z, X)          # short arm runs 3 holes into the tube under/over the front beam
beam("frameSp", 3, Y, Z, (-FX, 4.4, ZC + 0.8))                 # spacer between the two L-beams
beam("frameSp2", 3, Y, Z, (-FX, 6.0, ZC + 1.6))                # spacer between the top L-beam and the top crosspiece
beam("frameT", 13, Y, Z, (-FX, 6.8, ZC + 0.8))                 # top crosspiece, runs back over the wheel
beam("frameB", 7, Y, Z, (-FX, 2.0, ZC - 0.8))                  # bottom crosspiece
conn("2780", (-FX, 5.6, ZC - 0.8), Y, why="top L-beam -> front top motor beam")
conn("6558", (-FX, 6.0, ZC + 0.8), Y, why="top crosspiece -> rear top motor beam -> top L-beam")
conn("6558", (-FX, 6.0, ZC + 2.4), Y, why="top crosspiece -> spacer -> top L-beam")
conn("2780", (-FX, 6.4, ZC + 1.6), Y, why="top crosspiece -> spacer")
for z in (ZC - 0.8, ZC + 0.8):
    conn("6558", (-FX, 2.8, z), Y, why="bottom crosspiece -> bottom motor beam -> bottom L-beam")
conn("2780", (-FX + 0.8, 5.6, ZC - 0.8), Y, why="top L-beam -> front top motor beam (2nd pin)")
conn("2780", (-FX + 0.8, 3.2, ZC - 0.8), Y, why="bottom L-beam -> front bottom motor beam (2nd pin)")
conn("6558", (-FX, 4.4, ZC + 1.6), Y, why="bottom L-beam -> spacer -> top L-beam")
conn("2780", (-FX, 4.0, ZC + 2.4), Y, why="bottom L-beam -> spacer")
R6536 = Rb(-X, Z, Y)                        # pin hole vertical, axle hole along x, 0.8 behind the pin hole
add("xblock_low", "6536", R6536, (-FX, 2.8, -4.0), COL["6536"])
conn("2780", (-FX, 2.4, -4.8), Y, why="cross block -> bottom crosspiece")
axle_span(8, (-10.8, 2.8, -4.0), (-4.4, 2.8, -4.0), why="left frame (cross block) -> left rail")
add("xblock_high", "6536", R6536, (-FX, 7.6, 1.6), COL["6536"])
conn("2780", (-FX, 7.2, 0.8), Y, why="cross block -> top crosspiece")
axle_span(5, (-10.8, 7.6, 1.6), (-6.8, 7.6, 1.6), why="left frame (cross block) -> left outer stem L")
# right bearing frame: two beam planes (x 9.8 and 10.6) just past the tube end, tied to the right rail and stem L-beam
B = (YC, ZC)
lbeam("rfL", (9.8, 2.8, ZC), Y, Z)                 # inner plane: long arm up through the bearing hole, short arm back
beam("rfB", 7, X, Z, (10.6, 2.8, ZC))               # outer plane: bottom beam to the rail axle
lbeam("rfV", (10.6, 6.8, ZC), -Y, Z)                # outer plane: long arm down through the bearing hole, short arm back
beam("rfT", 13, X, Z, (9.8, 6.8, ZC + 0.8))         # inner plane: top beam back over the wheel to the stem axle
for y, z in ((2.8, ZC + 0.8), (2.8, ZC + 1.6)): conn("2780", (10.2, y, z), X, why="right frame: L-beam -> bottom beam")
for y in (3.6, 6.0): conn("2780", (10.2, y, ZC), X, why="right frame: L-beam -> upright L-beam")
for z in (ZC + 0.8, ZC + 1.6): conn("2780", (10.2, 6.8, z), X, why="right frame: upright L-beam -> top beam")
axle_span(8, (4.4, 2.8, -4.0), (10.8, 2.8, -4.0), why="right rail -> right frame bottom beam")
axle_span(5, (6.8, 6.8, 1.6), (10.8, 6.8, 1.6), why="right outer stem L -> right frame top beam")
# roller: card tube held on the two spare wheels (rim No.16 + tyre No.17) by the tyres' grip; no glue
W1, W2 = 1.0, TUBE_R - 1.4                          # rim hub starts: wheel 1 next to the motor face (tyre 2 mm clear), wheel 2 flush with the right end
Rh = Rb(V(0, 0, -1), Y, X)                          # rim axis along +x (axle holes at c and c + 0.8)
for nm, c in (("hub1", W1), ("hub2", W2)):
    add(f"{nm}_rim", "56908", Rh, (c, YC, ZC), COL["rim"], group="roller")
    add(f"{nm}_tyre", "41897", Rh, (c, YC, ZC), COL["tyre"], group="roller"); tape(f"{nm}_tyre", f"{nm}_rim")
add("roller", "ROLLER", np.eye(3), (0, 0, 0), 0, group="roller", holes=[])
tape("roller", "hub1_tyre"); tape("roller", "hub2_tyre")
# one shaft from the motor to the right frame: 6L (motor -> wheel 1 -> joiner) + axle joiner No.18 + 9L (joiner -> wheel 2 -> bearings)
J_X = OUT_X + 4.0                                   # joiner centre, between wheel 1's tyre (ends x 2.4) and wheel 2's tyre (starts 6.4)
add("joiner", "59443", Rh, (J_X, YC, ZC), COL["axle"], group="roller")
axle_span(6, (OUT_X - 0.8, YC, ZC), (J_X, YC, ZC), group="roller", why="EV3 Medium output -> wheel 1 -> axle joiner (6L)")
axle_span(9, (J_X, YC, ZC), (J_X + 7.2, YC, ZC), group="roller", why="axle joiner -> wheel 2 -> right-frame bearings (9L)")

# ---------------- rear ball caster: on the centre line, 7.2 cm behind the axle, under the raised tray floor ----------------
beam("casterA", 9, Z, X, (-8.0, 2.8, 8.8))          # left post -> centre (holes along z)
beam("casterB", 9, Z, X, (1.6, 2.8, 8.8))           # centre -> right post
beam("casterC", 9, Z, X, (-3.2, 2.8, 8.0))          # splice in front of A and B
for x in (-3.2, -2.4, 2.4, 3.2): conn("6558", (x, 2.8, 8.8), Z, why="caster splice beam -> caster bars")
add("caster", "92911c01", Rb(X, Z, -Y), (0.0, 1.2, 7.2), COL["dev"])
for x in (-0.8, 0.8): conn("6558", (x, 2.8, 8.0), Z, why="ball caster -> splice beam")

# ---------------- pusher: NXT crank (7-hole), rod (11-hole), L-beam sled post with the card plate ----------------
HUB = V(0, 10.0, 0.0)                     # NXT hub (y, z) = (10.0, 0.0)
R_CR, L_ROD, Q_Y = 4.8, 8.0, 4.6        # Q_Y: rod pivot 1.2 cm above the raised tray floor (3.4)


def kin(th):
    C = V(0, HUB[1] + R_CR * math.sin(th), HUB[2] + R_CR * math.cos(th))
    dy = C[1] - Q_Y
    zq = C[2] + math.sqrt(L_ROD ** 2 - dy ** 2)
    return C, zq


def solve(target, lo, hi):
    for _ in range(80):
        mid = (lo + hi) / 2
        if (kin(mid)[1] - target) * (kin(lo)[1] - target) <= 0: hi = mid
        else: lo = mid
    return (lo + hi) / 2


Z_REST, Z_PUSH = -1.0, 10.2                 # plate face from 0.66 cm behind the tray front to 10.86 (tray floor ends at 11.0)
TH_REST = solve(Z_REST, math.radians(150), math.radians(180))
TH_PUSH = solve(Z_PUSH, math.radians(270), math.radians(330))
MOVING = []


def pusher(th, tag):
    C, zq = kin(th)
    Q = V(0, Q_Y, zq)
    g = lambda nm: nm + tag
    beam(g("crank"), 7, X, V(0, math.sin(th), math.cos(th)), (0, HUB[1], HUB[2]), group="crank" + tag)
    axle_span(5, (-0.4, HUB[1], HUB[2]), (3.6, HUB[1], HUB[2]), group="crank" + tag, why="crank -> NXT hub")
    add(g("bush"), "3713", Rb(Z, -Y, X), (0.8, HUB[1], HUB[2]), COL["3713"], group="crank" + tag)
    d = (Q - C) / L_ROD
    beam(g("rod"), 11, X, d, (-0.8, C[1], C[2]), group="rod" + tag)
    conn("2780", (-0.4, C[1], C[2]), X, group="rod" + tag, why="crank -> rod")
    lbeam(g("pusherL"), (-1.6, 3.8, zq), -Z, Y, group="post" + tag)
    conn("2780", (-1.2, Q_Y, zq), X, group="post" + tag, why="rod -> pusher L-beam")
    return zq


pusher(TH_REST, "")
tape("pusherL", "plate")

# =====================================================================================================
def engaged(c):
    out = {}
    a0 = c["centre"] - c["u"] * c["L"] / 2
    for p in PARTS:
        for h, a, k in p["holes"]:
            if abs(abs(a @ c["u"]) - 1) > 1e-3: continue
            r = h - a0; s = r @ c["u"]
            if np.linalg.norm(r - s * c["u"]) < 0.06 and 0.05 <= s <= c["L"] - 0.05:
                out.setdefault(p["name"], set()).add(k)
    return out


def check(verbose=True):
    bad, parent = [], {p["name"]: p["name"] for p in PARTS}
    find = lambda a: a if parent[a] == a else find(parent[a])
    for c in CONN:
        if c["kind"] == "3713": continue
        e = engaged(c)
        ok = len(e) >= 2
        if c["kind"] in ("2780", "6558"):
            for nm, ks in e.items():
                if "pin" not in ks: ok = False; bad.append(f"pin in a cross hole: {nm} ({c['why']})")
        if not ok: bad.append(f"{c['kind']} at {np.round(c['centre'], 2)} engages {sorted(e)} ({c['why']})")
        names = sorted(e)
        for n in names[1:]: parent[find(n)] = find(names[0])
        c["engaged"] = names
    for a, b in TAPE:
        if a in parent and b in parent: parent[find(b)] = find(a)
    # overlapping connectors in one hole
    for i, a in enumerate(CONN):
        for b in CONN[i + 1:]:
            if a["group"] != b["group"] or abs(abs(a["u"] @ b["u"]) - 1) > 1e-3: continue
            r = b["centre"] - a["centre"]; s = r @ a["u"]
            if np.linalg.norm(r - s * a["u"]) < 0.06 and abs(s) < (a["L"] + b["L"]) / 2 - 0.05:
                bad.append(f"overlap: {a['kind']} ({a['why']}) and {b['kind']} ({b['why']})")
    comps = {}
    for p in PARTS: comps.setdefault(find(p["name"]), []).append(p["name"])
    if verbose:
        print(f"{len(PARTS)} parts, {len(CONN)} pins/axles, {len(comps)} connected group(s)")
        for b in bad: print("  BAD:", b)
        if len(comps) > 1:
            for k, v in comps.items(): print("  group:", v)
    return bad, comps


def tally():
    t = {}
    for p in PARTS:
        k = KITNAME.get(p["f"])
        if k: t[k] = t.get(k, 0) + 1
    axles = {}
    for c in CONN:
        k = KITNAME.get(c["f"])
        if k: t[k] = t.get(k, 0) + 1
        if c["kind"].startswith("axle"): axles[c["kind"][4:] + "L"] = axles.get(c["kind"][4:] + "L", 0) + 1
    return t, axles


def M4(R, t):
    m = np.eye(4); m[:3, :3] = R; m[:3, 3] = t
    return [round(float(v), 5) for v in m.T.flatten()]          # column-major for three.js


# ---------------- mass estimate (grams) -> centre of mass ----------------
MASS = {"95658": 76, "99455": 48, "53787": 80, "95650": 18, "56908": 6, "41897": 12, "92911c01": 28, "32526": 1.8,
        "6536": 0.6, "2780": 0.3, "6558": 0.45, "3713": 0.3}
BEAM_G_PER_HOLE = 0.25
CARD = [("tray + walls + sled", 18, (0, 4.5, 5.0)), ("roller tube + flaps", 18, (-0.4, YC, ZC)),
        ("guide + hood + funnel", 15, (0, 5.5, -6.0))]
PI_G, BATT_G = 125, 150          # Pi 3 + BrickPi3 + case; battery pack (ASSUMED, weigh yours)


def mass_cg():
    import pack
    tot, mom, rows = 0.0, np.zeros(3), {}
    def put(name, m, p):
        nonlocal tot, mom
        tot += m; mom += m * np.asarray(p, float); rows[name] = rows.get(name, 0) + m
    for p in PARTS:
        f = p["f"]
        if f in ("PI", "ROLLER"): continue
        m = MASS.get(f) or (BEAM_G_PER_HOLE * [k for k, v in BEAM.items() if v == f][0] if f in BEAM.values() else 0)
        a = np.array(pack.bbox(f + ".dat")) * 0.04 * np.array([1, -1, -1])
        put(KITNAME.get(f, f), m, p["R"] @ ((a.min(0) + a.max(0)) / 2) + p["t"])
    for c in CONN:
        m = MASS.get(c["kind"]) or 0.15 * int(c["kind"][4:])
        put("pins + axles", m, c["centre"])
    for nm, m, pos in CARD: put(nm, m, pos)
    put("Pi 3 + BrickPi3 + case", PI_G, ((0, (PI["y0"] + PI["y1"]) / 2, (PI["z0"] + PI["z1"]) / 2)))
    put("battery (assumed 150 g)", BATT_G, (0, PI["y1"] + 1.2, (PI["z0"] + PI["z1"]) / 2))
    return tot, mom / tot, rows


if __name__ == "__main__":
    bad, comps = check()
    t, axles = tally()
    M_TOT, CGv, _ = mass_cg()
    CG = [round(float(v), 2) for v in CGv]
    CASTER_Z = 7.2
    print(f"mass ~{M_TOT:.0f} g, centre of mass x {CG[0]} y {CG[1]} z {CG[2]} (z = cm behind the drive axle); "
          f"caster carries {100 * CG[2] / CASTER_Z:.0f}%, wheels {100 - 100 * CG[2] / CASTER_Z:.0f}%")
    print("kit use:"); [print(f"  {k:24s} {n:3d} of {KIT.get(k, '?')}") for k, n in t.items()]
    print("axles:", dict(sorted(axles.items(), key=lambda kv: int(kv[0][:-1]))))
    print(f"pusher: rest {math.degrees(TH_REST):.1f} deg (post z {Z_REST}), push {math.degrees(TH_PUSH):.1f} deg (post z {Z_PUSH})")
    # moving parts across the stroke
    samples = []
    N = 40
    static_names = {p["name"] for p in PARTS}
    for i in range(N + 1):
        th = TH_REST + (TH_PUSH - TH_REST) * i / N
        n0, c0 = len(PARTS), len(CONN)
        zq = pusher(th, f"#{i}")
        new_p, new_c = PARTS[n0:], CONN[c0:]
        b, _ = check(verbose=False)
        b = [x for x in b if "#" in x or any(f"#{i}" in n for n in x.split())]
        mv = [dict(f=p["f"], c=p["col"], m=M4(p["R"], p["t"])) for p in new_p] + \
             [dict(f=c["f"], c=c["col"], m=M4(c["R"], c["centre"])) for c in new_c]
        samples.append(dict(z=round(zq, 3), parts=mv))
        for p in new_p: BY.pop(p["name"])
        del PARTS[n0:]; del CONN[c0:]
        assert all(len(c.get("engaged", [])) >= 2 for c in new_c), (i, [c["engaged"] for c in new_c])
    out = dict(
        static=[dict(f=p["f"], c=p["col"], m=M4(p["R"], p["t"]), n=p["name"]) for p in PARTS
                if p["group"] == "static" and p["f"] not in ("PI",)] +
               [dict(f=c["f"], c=c["col"], m=M4(c["R"], c["centre"]), n=c["why"]) for c in CONN if c["group"] == "static"],
        roller=[dict(f=c["f"], c=c["col"], m=M4(c["R"], c["centre"])) for c in CONN if c["group"] == "roller"] +
               [dict(f=p["f"], c=p["col"], m=M4(p["R"], p["t"])) for p in PARTS if p["group"] == "roller" and p["f"] != "ROLLER"],
        pusher=samples,
        kit={k: [n, KIT.get(k)] for k, n in t.items()}, axles=axles,
        dims=dict(YC=YC, ZC=ZC, TUBE_L=TUBE_L, TUBE_R=TUBE_R, OUT_X=OUT_X, PI=PI, Q_Y=Q_Y, WX=WX, FX=FX, W1=W1, W2=W2, CG=CG),
        joints=sorted({c["why"] for c in CONN}),
    )
    json.dump(out, open(sys.argv[1] if len(sys.argv) > 1 else "/tmp/robot3d/asm.json", "w"))
    print("parts drawn:", len(out["static"]), "static,", len(out["roller"]), "roller,", len(samples[0]["parts"]), "moving")
