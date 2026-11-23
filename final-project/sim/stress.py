#!/usr/bin/env python3
"""
Stress test: Phase 1 + Phase 2 on many random boards (sim/maps.py random_map), in parallel.

  python3 sim/stress.py                          # 60 random one-table boards
  python3 sim/stress.py --n 200 --kinds all      # one table, edge streets, two tables wide / tall
  python3 sim/stress.py --kinds 18x5 --n 30 --noise 2
  python3 sim/stress.py --show random-1234       # print one board

A run fails if the map is wrong, a phase goes over 120 s, Phase 2 touches a house, a wheel leaves the table,
the robot ends > 3 cm from the origin, or the code crashes. Pom-poms below --min-pompoms are listed too.
Re-run one board with: python3 sim/run_sim.py --map NAME -v
"""
from __future__ import annotations
import argparse
import multiprocessing as mp
import re
import os
import sys
import time

SIM_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SIM_DIR)
import maps  # noqa: E402

KINDS = {"1t": "random", "loose": "random-loose", "edge": "random-edge", "18x5": "random-18x5",
         "9x10": "random-9x10"}
LIMIT = 120.0


def _job(args):
    name, seed, noise, overrides = args
    os.environ["SNOWPLOW_VIZ_LOGDIR"] = "none"
    os.environ["SNOWPLOW_WEBVIZ"] = "0"
    os.environ["SNOWPLOW_VIZ"] = "none"
    import run_sim
    t0 = time.time()
    r = run_sim.run_once(name, seed, noise, False, overrides=overrides)
    r["wall"] = time.time() - t0
    spec = maps.get_map(name)
    r["len"] = len(spec["street"])
    r["tags"] = "".join(t for t, k in ((" turn-at-gate", "turn_at_gate"), (" turn-at-exit", "turn_at_exit"))
                        if spec.get(k))
    r.pop("log", None) if "error" not in r else None
    return r


def problems(r, min_pompoms):
    out = []
    if "error" in r:
        return [f"crash: {r['error']}"]
    if not r.get("map_ok"):
        out.append("map wrong")
    if r.get("p1_time", 0) > LIMIT:
        out.append(f"P1 {r['p1_time']:.0f}s > {LIMIT:.0f}")
    if r.get("p2_time", 0) > LIMIT:
        out.append(f"P2 {r['p2_time']:.0f}s > {LIMIT:.0f}")
    if r["p2_contacts"]:
        out.append(f"P2 touched a house ({r['p2_contacts']})")
    if r["offtable"]:
        out.append(f"off the table ({r['offtable']})")
    for ph in (1, 2):
        if r.get(f"p{ph}_end_err", 0) > 3.0:
            out.append(f"P{ph} home error {r[f'p{ph}_end_err']:.1f} cm")
    got, total = (int(v) for v in r["pompoms"].split("/"))
    if total and got / total < min_pompoms:
        out.append(f"pom-poms {got}/{total}")
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=60, help="boards per kind")
    ap.add_argument("--kinds", default="1t", help=f"comma list of {', '.join(KINDS)} or 'all'")
    ap.add_argument("--seed-start", type=int, default=0)
    ap.add_argument("--noise", type=float, default=1.0)
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 2))
    ap.add_argument("--min-pompoms", type=float, default=0.85)
    ap.add_argument("--set", action="append", default=[], metavar="MOD.NAME=VALUE")
    ap.add_argument("--show", metavar="NAME", help="print a board and exit")
    ap.add_argument("--out", default=os.path.join(SIM_DIR, "session", "stress_results.json"))
    a = ap.parse_args()
    if a.show:
        spec = maps.get_map(a.show)
        print(a.show, {k: spec[k] for k in ("s1", "s2", "d1", "d2")}, "street", len(spec["street"]))
        print(maps.render(spec))
        return
    kinds = list(KINDS) if a.kinds == "all" else a.kinds.split(",")
    names = [f"{KINDS[k]}-{s}" for k in kinds for s in range(a.seed_start, a.seed_start + a.n)]
    tasks = [(n, i % 7, a.noise, a.set) for i, n in enumerate(names)]
    t0 = time.time()
    rows = []
    with mp.get_context("fork").Pool(a.jobs, maxtasksperchild=1) as pool:
        for i, r in enumerate(pool.imap_unordered(_job, tasks), 1):
            rows.append(r)
            bad = problems(r, a.min_pompoms)
            if bad:
                print(f"[{i:>4}/{len(tasks)}] {r['map']:<22} seed {r['seed']} len {r['len']:>2}{r['tags']}: " + "; ".join(bad),
                      flush=True)
    import json
    with open(a.out, "w") as f:
        json.dump([{k: v for k, v in r.items() if k not in ("log", "trace")} | {"problems": problems(r, a.min_pompoms)}
                   for r in rows], f, indent=0, default=str)
    print(f"\n{len(rows)} runs in {time.time() - t0:.0f} s ({a.jobs} processes, noise {a.noise})")
    for k in kinds:
        rs = [r for r in rows if r["map"].rsplit("-", 1)[0] == KINDS[k]]
        ok = [r for r in rs if "error" not in r]
        fails = [r for r in rs if problems(r, a.min_pompoms)]
        hard = [r for r in rs if [p for p in problems(r, a.min_pompoms) if not p.startswith("pom-poms")]]
        if not ok:
            print(f"  {k:>5}: {len(rs)} runs, all crashed")
            continue

        def stat(key):
            v = sorted(r.get(key, 0) for r in ok)
            return f"{v[0]:.0f} / {v[len(v) // 2]:.0f} / {v[-1]:.0f}"
        pp = [int(r["pompoms"].split("/")[0]) / max(1, int(r["pompoms"].split("/")[1])) for r in ok]
        print(f"  {k:>5}: {len(rs) - len(hard)}/{len(rs)} clean ({len(fails) - len(hard)} only short on pom-poms) | "
              f"P1 s min/med/max {stat('p1_time')} | P2 {stat('p2_time')} | pom-poms mean {100 * sum(pp) / len(pp):.0f}% "
              f"| street len {stat('len')} | min P2 clearance {min(r['p2_min_clear'] for r in ok):.2f} cm")
    cats = {}
    for r in rows:
        for p in problems(r, a.min_pompoms):
            key = re.sub(r"\d+(\.\d+)?", "N", p.split(":")[0] if p.startswith("crash") else p)
            key = key.replace("PN", p[:2]) if key.startswith("PN") else key
            cats.setdefault(key, []).append(r["map"])
    if cats:
        print("\nProblems:")
        for k, v in sorted(cats.items(), key=lambda kv: -len(kv[1])):
            print(f"  {len(v):>3} x {k}: {', '.join(sorted(v)[:8])}{' ...' if len(v) > 8 else ''}")


if __name__ == "__main__":
    main()
