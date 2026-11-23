"""v6: moving pusher LEGO (crank, rod) vs every card surface, in the side profile (z, y), over the full stroke.
Also: does the rod ever reach the sliding roof, and how long a compaction stroke the roof allows.
Beams are capsules between their end-hole centres with radius 0.36; card sheets are 0.15 thick (centre line drawn).
Run: python3 pusher_card_check.py   (v5's version: pusher_card_check_v5.py)"""
import math, numpy as np
ZC, YC = -8.8, 4.4                         # roller axis
HUB, R_CR, L_ROD, Q_Y = (0.0, 10.0), 3.2, 9.6, 2.8
Z_REST, Z_PUSH = 0.2, 10.2                 # sled post z
FLOOR, PT, ROOF_LEN, ROOF_FRONT = 0.8, 2.1, 5.56, -4.7
HW = 0.36 + 0.075
P = lambda a, r: (ZC + r * math.cos(math.radians(a)), YC + r * math.sin(math.radians(a)))
GE = P(-5.0, 4.4)                          # guide end
LIP_TOP = 2.5
LIP_Z = GE[0] + (GE[1] - LIP_TOP) / math.tan(math.radians(29))
arc = lambda r, a0, a1, n=40: [P(a, r) for a in np.linspace(a0, a1, n)]
CARD = {
    "guide": arc(4.4, 262, 355), "chute": [(GE[0], GE[1] - 0.075), (LIP_Z, LIP_TOP - 0.075)],
    "hood": arc(4.85, 125, 30, 16), "ceiling": [P(30, 4.85), (-2.0, 5.2)],
    "floor": [(-4.7, FLOOR - 0.1), (11.0, FLOOR - 0.1)],
}
LEGO = {"back crossbar": [(9.2, 6.4), (10.0, 6.4), (10.0, 7.2), (9.2, 7.2), (9.2, 6.4)], "Pi case": [(1.2, 11.2), (9.6, 11.2)],
        "cross axle z5.6": [(5.36, 10.8), (5.84, 10.8)], "cross axle z7.2": [(6.96, 10.8), (7.44, 10.8)]}
def segd(p, a, b):
    ab = b - a; t = min(1, max(0, (p - a) @ ab / (ab @ ab))); return float(np.linalg.norm(p - (a + t * ab)))
def seg_seg(a, b, c, d):
    a, b, c, d = map(lambda v: np.asarray(v, float), (a, b, c, d))
    x = lambda o, p, q: (p[0] - o[0]) * (q[1] - o[1]) - (p[1] - o[1]) * (q[0] - o[0])
    if x(c, d, a) * x(c, d, b) < 0 and x(a, b, c) * x(a, b, d) < 0: return 0.0
    return min(segd(a, c, d), segd(b, c, d), segd(c, a, b), segd(d, a, b))
def kin(th):
    C = np.array([HUB[0] + R_CR * math.cos(th), HUB[1] + R_CR * math.sin(th)])
    dy = C[1] - Q_Y
    return C, (None if abs(dy) >= L_ROD else C[0] + math.sqrt(L_ROD ** 2 - dy ** 2))
def solve(target, lo, hi):
    for _ in range(80):
        m = (lo + hi) / 2
        if (kin(m)[1] - target) * (kin(lo)[1] - target) <= 0: hi = m
        else: lo = m
    return (lo + hi) / 2
if __name__ == "__main__":
    th0, th1 = solve(Z_REST, math.radians(136), math.radians(170)), solve(Z_PUSH, math.radians(280), math.radians(320))
    print(f"crank {math.degrees(th0):.1f} -> {math.degrees(th1):.1f} deg; lip at z {LIP_Z:.2f}")
    worst, roof_hits = {}, []
    for th in np.linspace(th0, th1, 300):
        C, zq = kin(th); Q = np.array([zq, Q_Y]); hub = np.array(HUB); tip = hub + R_CR * (C - hub) / R_CR
        for sn, (a, b) in {"crank": (hub, tip), "rod": (C, Q)}.items():
            for cn, poly in {**CARD, **LEGO}.items():
                d = min(seg_seg(a, b, poly[i], poly[i + 1]) for i in range(len(poly) - 1)) - HW
                if (sn, cn) not in worst or d < worst[(sn, cn)][0]: worst[(sn, cn)] = (d, math.degrees(th), zq)
        for zr in np.linspace(zq + 0.66 - ROOF_LEN, zq + 0.66, 200):
            if segd(np.array([zr, PT + 0.075]), C, Q) < HW + 0.05: roof_hits.append(zr - zq)
    for (sn, cn), (d, th, zq) in sorted(worst.items(), key=lambda kv: kv[1][0]):
        print(f"  {sn:5s} vs {cn:16s}: min gap {d:5.2f} cm at crank {th:5.1f} deg (sled post z {zq:5.2f})")
    print("rod reaches the sled roof:", "never" if not roof_hits else f"between post {min(roof_hits):+.2f} and {max(roof_hits):+.2f} cm (notch it)")
    print(f"compaction stroke with the roof still under the lip: {ROOF_LEN - (Z_REST + 0.66 - LIP_Z):.2f} cm")
