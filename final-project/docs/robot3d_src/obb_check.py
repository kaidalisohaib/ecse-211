# Run from a copy of robot3d_src in /tmp/robot3d (see README).
"""LEGO-vs-LEGO interference: OBBs for beams/pins/axles/small parts, surface point clouds for motors/sensors/caster.
Pinned/axled neighbours may touch; reports overlaps deeper than TOL. Checks the rest pose and the pusher stroke."""
import sys, math, numpy as np
sys.path.insert(0, "/tmp/robot3d"); sys.argv = ["x"]
import assembly as A, pack
TOL = 0.06
COMPLEX = {"95658", "99455", "53787", "95650", "92911c01", "56908", "41897"}
_tri = {}
def tris(f):
    if f not in _tri:
        from proj import polys  # noqa
    return _tri[f]
def surface_pts(f, step=0.15):
    key = f
    if key in _tri: return _tri[key]
    import importlib.util
    spec = importlib.util.spec_from_file_location("projm", "/tmp/robot3d/proj.py")
    src = open("/tmp/robot3d/proj.py").read().split("part = sys.argv[1]")[0]
    ns = {}; exec(src, ns)
    P = ns["polys"](f + ".dat")
    pts = []
    for p in P:
        p = np.asarray(p)
        for i in range(1, len(p) - 1):
            a, b, c = p[0], p[i], p[i + 1]
            n = max(2, int(max(np.linalg.norm(b - a), np.linalg.norm(c - a)) / step) + 1)
            for u in np.linspace(0, 1, n):
                for v in np.linspace(0, 1 - u, max(1, int(n * (1 - u)))):
                    pts.append(a + u * (b - a) + v * (c - a))
    _tri[key] = np.array(pts)
    return _tri[key]
def local_box(f):
    a = np.array(pack.bbox(f + ".dat")) * 0.04 * np.array([1, -1, -1])
    return (a.min(0) + a.max(0)) / 2, (a.max(0) - a.min(0)) / 2
def solids(parts, conns):
    out = []
    for p in parts:
        if p["f"] in ("PI", "ROLLER"): continue
        c, h = local_box(p["f"])
        out.append(dict(name=p["name"], f=p["f"], R=p["R"], t=p["t"], c=p["R"] @ c + p["t"], h=h, cx=p["f"] in COMPLEX, eng=set()))
    for i, cn in enumerate(conns):
        c, h = local_box(cn["f"])
        if cn["kind"] in ("2780", "6558"): h = np.array([cn["L"] / 2, 0.24, 0.24])     # shaft only (collars are inside holes)
        out.append(dict(name=f"{cn['kind']}:{cn['why']}", f=cn["f"], R=cn["R"], t=cn["centre"], c=cn["R"] @ c + cn["centre"], h=h,
                        cx=False, eng=set(cn.get("engaged", []))))
    return out
def obb_overlap(a, b):
    """SAT on two OBBs (shrunk by TOL): True if they overlap."""
    Ra, Rb_ = a["R"], b["R"]; ha, hb = a["h"] - TOL, b["h"] - TOL
    if (ha <= 0).any() or (hb <= 0).any(): return False
    T = b["c"] - a["c"]
    axes = [Ra[:, i] for i in range(3)] + [Rb_[:, i] for i in range(3)] + \
           [np.cross(Ra[:, i], Rb_[:, j]) for i in range(3) for j in range(3)]
    for L in axes:
        n = np.linalg.norm(L)
        if n < 1e-6: continue
        L = L / n
        ra = sum(ha[i] * abs(Ra[:, i] @ L) for i in range(3)); rb = sum(hb[i] * abs(Rb_[:, i] @ L) for i in range(3))
        if abs(T @ L) > ra + rb: return False
    return True
def pts_in(a, pts):
    loc = (pts - a["c"]) @ a["R"]
    return int((np.abs(loc) < a["h"] - TOL).all(1).sum())
def world_pts(s): return (s["R"] @ surface_pts(s["f"]).T).T + s["t"]
def check(S, only=None):
    bad = []
    for i, a in enumerate(S):
        for b in S[i + 1:]:
            if only and not (only(a) or only(b)): continue
            if a["name"] in b["eng"] or b["name"] in a["eng"]: continue
            if np.linalg.norm(a["c"] - b["c"]) > np.linalg.norm(a["h"]) + np.linalg.norm(b["h"]): continue
            if a["cx"] and b["cx"]:
                n = pts_in(dict(a, h=a["h"] * 0.6), world_pts(b))   # crude for complex pairs
                hit = n > 3
            elif a["cx"] or b["cx"]:
                cx, sm = (a, b) if a["cx"] else (b, a)
                if not obb_overlap(a, b): continue
                n = pts_in(sm, world_pts(cx)); hit = n > 2
            else:
                hit = obb_overlap(a, b); n = 0
            if hit: bad.append((a["name"], b["name"], n))
    return bad
if __name__ == "__main__":
    A.check(verbose=False)
    S = solids(A.PARTS, A.CONN)
    bad = check(S)
    print(f"rest pose: {len(S)} solids, {len(bad)} overlaps")
    for x in bad: print("  ", x)
    static = [s for s in S if not any(k in s["name"] for k in ("crank", "rod", "pusherL", "adaptor", "NXT hub", "sled", "drives the crank"))]
    worst = {}
    for i in range(41):
        th = A.TH_REST + (A.TH_PUSH - A.TH_REST) * i / 40
        n0, c0 = len(A.PARTS), len(A.CONN)
        A.pusher(th, f"#{i}")
        A.check(verbose=False)
        mv = solids(A.PARTS[n0:], A.CONN[c0:])
        for p in A.PARTS[n0:]: A.BY.pop(p["name"])
        del A.PARTS[n0:]; del A.CONN[c0:]
        for b in check(static + mv, only=lambda s: "#" in s["name"] or any(k in s["name"] for k in ("drives the crank", "crank -> rod", "rod -> sled", "NXT hub ->"))):
            if "#" in b[0] or "#" in b[1] or True:
                k = tuple(sorted((b[0].split("#")[0], b[1].split("#")[0])))
                worst.setdefault(k, []).append(i)
    print("stroke overlaps (moving vs static):", "none" if not worst else "")
    for k, v in worst.items(): print("  ", k, "samples", v[:6], "..." if len(v) > 6 else "")
