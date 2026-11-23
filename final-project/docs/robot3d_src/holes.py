"""Extract technic hole/axle-hole primitives from LDraw parts, in robot-model coords (cm, three.js axes: x, y up, z)."""
import re, sys
import numpy as np
sys.path.insert(0, "/tmp/robot3d")
import pack
HOLE = re.compile(r"^(peghole|beamhole|beamhol2|connhole|npeghol\d*|confric\d*|axlehole|axlehol\d*|axl\dhol\w*|axl\dhole)\.dat$")
CONV = np.diag([0.04, -0.04, -0.04, 1.0])          # LDraw LDU -> cm with y up, z flipped (holder rot x pi)
_cache = {}
def _walk(name, M, out):
    k, t = pack.load(name)
    for ln in t.splitlines():
        p = ln.split()
        if len(p) >= 15 and p[0] == "1":
            v = list(map(float, p[2:14]))
            T = np.array([[v[3], v[4], v[5], v[0]], [v[6], v[7], v[8], v[1]], [v[9], v[10], v[11], v[2]], [0, 0, 0, 1]])
            f = " ".join(p[14:]).lower().replace("\\", "/").split("/")[-1]
            MT = M @ T
            if HOLE.match(f):
                out.append((MT, f))
            elif not f.startswith(("4-4", "2-4", "1-4", "3-4", "1-8", "rect", "box", "stud", "ring", "edge", "cyl", "disc", "ndis", "con")):
                _walk(" ".join(p[14:]), MT, out)
def holes(part):
    if part in _cache: return _cache[part]
    raw = []
    _walk(part, np.eye(4), raw)
    res = []
    for MT, f in raw:
        W = CONV @ MT
        c = W[:3, 3]
        a = W[:3, 1]; L = np.linalg.norm(a); a = a / L if L else a
        kind = "axle" if f.startswith("axl") else "pin"
        res.append((c, a, kind, f))
    # merge duplicates: same axis line, centres within 0.85 cm along it -> keep distinct points (rounded)
    uniq = {}
    for c, a, kind, f in res:
        key = (tuple(np.round(c, 2)), tuple(np.round(np.abs(a), 2)))
        uniq.setdefault(key, (c, a, kind, f))
    _cache[part] = list(uniq.values())
    return _cache[part]
if __name__ == "__main__":
    for p in sys.argv[1:]:
        hs = holes(p + ".dat")
        print(f"== {p}: {len(hs)} hole primitives")
        for c, a, kind, f in sorted(hs, key=lambda h: (round(abs(h[1][0])), round(abs(h[1][1])), h[0][2], h[0][1], h[0][0])):
            ax = "xyz"[int(np.argmax(np.abs(a)))]
            print(f"  {kind:4s} axis {ax}  at ({c[0]:6.2f}, {c[1]:6.2f}, {c[2]:6.2f})  {f}")
