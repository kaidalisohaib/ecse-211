"""Quick orthographic silhouettes of an LDraw part (model frame cm) with holes marked."""
import sys; sys.path.insert(0, '/tmp/robot3d')
import pack, holes, numpy as np, matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
def polys(name, M=np.eye(4), acc=None):
    key, txt = pack.load(name); acc = [] if acc is None else acc
    for ln in txt.splitlines():
        t = ln.split()
        if not t: continue
        if t[0] == "1" and len(t) >= 15:
            v = list(map(float, t[2:14]))
            T = np.array([[v[3], v[4], v[5], v[0]], [v[6], v[7], v[8], v[1]], [v[9], v[10], v[11], v[2]], [0, 0, 0, 1]])
            polys(" ".join(t[14:]), M @ T, acc)
        elif t[0] in ("3", "4"):
            n = 3 if t[0] == "3" else 4
            pts = np.array(list(map(float, t[2:2 + 3 * n]))).reshape(n, 3)
            acc.append((M @ np.c_[pts, np.ones(n)].T).T[:, :3] * np.array([0.04, -0.04, -0.04]))
    return acc
part = sys.argv[1]
P = polys(part + '.dat'); H = holes.holes(part + '.dat')
fig, axs = plt.subplots(1, 3, figsize=(18, 6))
for ax, (i, j, name) in zip(axs, [(2, 1, 'side: z right, y up (from +x)'), (0, 2, 'top: x right, z up'), (0, 1, 'back: x right, y up')]):
    ax.add_collection(PolyCollection([p[:, [i, j]] for p in P], facecolor=(0.5, 0.5, 0.6, 0.08), edgecolor=(0, 0, 0, 0.15), lw=0.3))
    for c, a, kind, f in H:
        ax.plot(c[i], c[j], 'o' if kind == 'pin' else 's', color='r' if kind == 'pin' else 'b', ms=4)
    ax.autoscale(); ax.set_aspect('equal'); ax.grid(True, lw=0.3); ax.set_title(name)
    ax.xaxis.set_major_locator(plt.MultipleLocator(0.8)); ax.yaxis.set_major_locator(plt.MultipleLocator(0.8)); ax.tick_params(labelsize=6)
plt.tight_layout(); plt.savefig(sys.argv[2], dpi=70)
