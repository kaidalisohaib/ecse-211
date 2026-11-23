import math, numpy as np
ZC, YC = -8.8, 4.4
def arc(r, a0, a1, n=40):
    return [(ZC + r*math.cos(math.radians(a)), YC + r*math.sin(math.radians(a))) for a in np.linspace(a0, a1, n)]
guide = arc(4.4, 265, 385)
hood = arc(4.85, 125, 60, 12) + [(-4.4, 8.2)]
rel = guide[-1]
funnel = [rel, (rel[0]+0.1, rel[1]), (-1.3, 5.6)]
CARD = {"guide": guide, "hood": hood, "funnel floor": funnel}
def segd(p, a, b):
    p, a, b = map(np.array, (p, a, b)); ab = b-a; t = np.clip((p-a)@ab/(ab@ab), 0, 1); return np.linalg.norm(p-(a+t*ab))
def seg_seg(a, b, c, d):
    # min distance between segments (sampled)
    return min(min(segd(a+(b-a)*t, c, d) for t in np.linspace(0,1,40)), min(segd(np.array(c)+(np.array(d)-np.array(c))*t, a, b) for t in np.linspace(0,1,40)))
def check(hub, r, L, qy, th0, th1, label):
    hub = np.array(hub); worst = {}
    for th in np.linspace(th0, th1, 200):
        C = hub + r*np.array([math.cos(th), math.sin(th)])
        dy = C[1]-qy
        if abs(dy) > L: return None
        Q = np.array([C[0] + math.sqrt(L*L-dy*dy), qy])
        u = (C-hub)/r; ext = hub + (r+0.4)*u; tail = hub - 0.4*u
        v = (Q-C)/L
        segs = {"crank": (tail, ext), "rod": (C-0.4*v, Q+0.4*v)}
        for sn, (a, b) in segs.items():
            for cn, poly in CARD.items():
                d = min(seg_seg(a, b, np.array(poly[i]), np.array(poly[i+1])) for i in range(len(poly)-1)) - 0.36
                k = (sn, cn)
                if k not in worst or d < worst[k][0]: worst[k] = (d, math.degrees(th), tuple(np.round(C,2)), round(Q[0],2))
    print(label)
    for k, v in sorted(worst.items(), key=lambda kv: kv[1][0]):
        print(f"   {k[0]:5s} vs {k[1]:12s}: min gap {v[0]:6.2f} cm at crank {v[1]:6.1f} deg, pin(z,y) {v[2]}, plate post z {v[3]}")
if __name__ == "__main__": check((0.0, 10.0), 4.8, 8.0, 4.6, math.radians(157.7), math.radians(298.5), "v5 pusher (hub z0 y10, crank 4.8, rod 8.0, pivot y4.6):")
