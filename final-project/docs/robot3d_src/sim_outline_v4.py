import sys, os
sys.path.insert(0, "sim"); sys.path.insert(1, ".")
os.environ.update(SNOWPLOW_WEBVIZ="0", SNOWPLOW_VIZ="none", SNOWPLOW_VIZ_LOGDIR="none")
import world as wm, run_sim as rs
# v4 plan outline (x lateral, y forward), from the LDraw part vertices + roller sweep
fp = [(-10.76, -11.6), (10.76, -11.6), (10.76, -8.6), (8.4, -8.6), (8.4, -2.8), (11.0, -2.8), (11.0, 2.8), (10.0, 2.8),
      (10.0, 12.85), (-10.0, 12.85), (-10.0, 10.0), (-11.0, 10.0), (-11.0, -2.8), (-8.4, -2.8), (-8.4, -8.6), (-10.76, -8.6)]
wm.FOOTPRINT_P2_SQUARE = fp
wm.INTAKE_HALF_WIDTH = 10.0
wm.INTAKE_REACH = 12.85
ov = (f"p2.CORNER_POKE_CM={sys.argv[2]}",) if len(sys.argv) > 2 else ()
seeds = int(sys.argv[1]) if len(sys.argv) > 1 else 4
rows = [rs.run_once(m, s, 2.0, False, overrides=ov) for m in ("sample", "long", "u_reversed") for s in range(seeds)]
ok = [r for r in rows if "error" not in r]
tot = sum(r.get('p2_picked', 0) for r in ok)
print("v4:", "contact runs", sum(1 for r in ok if r['p2_contacts']), "/", len(ok),
      "min clr", round(min(r['p2_min_clear'] for r in ok), 2), "errors", len(rows) - len(ok))
for r in ok: print(rs.fmt(r))
import re
num = sum(int(re.search(r"pompoms\s+(\d+)/(\d+)", rs.fmt(r)).group(1)) for r in ok); den = sum(int(re.search(r"pompoms\s+(\d+)/(\d+)", rs.fmt(r)).group(2)) for r in ok)
print(f"snow {num}/{den} = {100*num/den:.1f}%")
