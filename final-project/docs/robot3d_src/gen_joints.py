import sys; sys.path.insert(0, "/tmp/robot3d"); sys.argv = ["x"]
import assembly as A
A.check(verbose=False)
NM = {"2780": "black pin", "6558": "blue 3L pin"}
rows = {}
for c in A.CONN:
    k = NM.get(c["kind"]) or f"axle {c['kind'][4:]}L"
    r = rows.setdefault(c["why"], [{}, set()])
    r[0][k] = r[0].get(k, 0) + 1
    r[1].update(c.get("engaged", []))
print("| Joint | Uses | Parts in the joint |\n|---|---|---|")
for why, (kinds, parts) in rows.items():
    print(f"| {why} | {', '.join(f'{n}x {k}' for k, n in kinds.items())} | {', '.join(sorted(parts))} |")
