"""Pack LDraw parts + all sub-files into one MPD text for three.js LDrawLoader; print bounding boxes (cm)."""
import os, sys, json
import numpy as np
LIB = "/tmp/ldraw/ldraw"
DIRS = ["parts", "p", "models", "unofficial/parts", "unofficial/p"]
def find(name):
    n = name.replace("\\", "/").lower()
    for d in DIRS:
        p = os.path.join(LIB, d, n)
        if os.path.exists(p): return p
    raise FileNotFoundError(name)
cache = {}
def load(name):
    key = name.replace("\\", "/").lower()
    if key not in cache:
        with open(find(key), encoding="utf-8", errors="replace") as f:
            cache[key] = f.read().replace("\r", "")
    return key, cache[key]
order = []
def collect(name, seen):
    key, txt = load(name)
    if key in seen: return
    seen.add(key)
    for ln in txt.splitlines():
        t = ln.split()
        if len(t) >= 15 and t[0] == "1":
            collect(" ".join(t[14:]), seen)
    order.append(key)
def bbox(name, M=np.eye(4), acc=None):
    key, txt = load(name)
    acc = acc if acc is not None else []
    for ln in txt.splitlines():
        t = ln.split()
        if not t: continue
        if t[0] == "1" and len(t) >= 15:
            v = list(map(float, t[2:14]))
            T = np.array([[v[3], v[4], v[5], v[0]], [v[6], v[7], v[8], v[1]], [v[9], v[10], v[11], v[2]], [0, 0, 0, 1]])
            bbox(" ".join(t[14:]), M @ T, acc)
        elif t[0] in ("3", "4"):
            n = 3 if t[0] == "3" else 4
            pts = np.array(list(map(float, t[2:2 + 3 * n]))).reshape(n, 3)
            acc.extend((M @ np.c_[pts, np.ones(n)].T).T[:, :3])
    return acc
if __name__ == "__main__":
    specs = [p.split(":") for p in sys.argv[1].split(",")]
    parts = [p[0] for p in specs]
    cols = [p[1] if len(p) > 1 else "16" for p in specs]
    out = sys.argv[2]
    seen = set()
    for p in dict.fromkeys(parts): collect(p + ".dat", seen)
    cfg = open(os.path.join(LIB, "LDConfig.ldr")).read()
    colours = "\n".join(l for l in cfg.splitlines() if "!COLOUR" in l)
    main = "0 FILE main.ldr\n0 Main\n0 !LDRAW_ORG Model\n" + colours + "\n" + \
        "\n".join(f"1 {c} 0 0 0 1 0 0 0 1 0 0 0 1 {p}.dat" for p, c in zip(parts, cols)) + "\n"
    fname = lambda k: "parts/" + k if k.startswith("s/") else "p/" + k if k.startswith("48/") else k
    body = [main] + [f"0 FILE {fname(k)}\n{cache[k]}\n" for k in order]
    txt = "\n".join(body)
    open(out, "w").write(txt)
    info = {}
    for p in dict.fromkeys(parts):
        a = np.array(bbox(p + ".dat"))
        info[p] = {"min": (a.min(0) * 0.04).round(2).tolist(), "max": (a.max(0) * 0.04).round(2).tolist()}
        print(p, "min", info[p]["min"], "max", info[p]["max"], "(cm, LDraw axes: -y up)")
    print(len(order), "files,", round(len(txt) / 1e6, 2), "MB")
