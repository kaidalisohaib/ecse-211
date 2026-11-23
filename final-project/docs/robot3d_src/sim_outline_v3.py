import sys, os
sys.path.insert(0, "sim"); sys.path.insert(1, ".")
os.environ.update(SNOWPLOW_WEBVIZ="0", SNOWPLOW_VIZ="none", SNOWPLOW_VIZ_LOGDIR="none")
import world as wm, run_sim as rs
# v3 outline in sim coords (x right, y forward): roller tips 12.05 ahead, frames +/-12.45, wheels +/-11.5, rear +/-10.8 to 11.6 behind
fp = [(-10.8, -11.6), (10.8, -11.6), (10.8, -2.8), (11.5, -2.8), (11.5, -2.0), (12.45, -2.0), (12.45, 9.2), (10.8, 9.2),
      (10.8, 12.05), (-10.8, 12.05), (-10.8, 9.2), (-12.45, 9.2), (-12.45, -2.0), (-11.5, -2.0), (-11.5, -2.8), (-10.8, -2.8)]
wm.FOOTPRINT_P2_SQUARE = fp
rows = [rs.run_once(m, s, 2.0, False) for m in ("sample", "long", "u_reversed") for s in range(6)]
ok = [r for r in rows if "error" not in r]
for r in rows: print(rs.fmt(r))
print("TOTAL contacts", sum(r['p2_contacts'] for r in ok), "runs with contact", sum(1 for r in ok if r['p2_contacts']), "/", len(ok),
      "min clr", round(min(r['p2_min_clear'] for r in ok), 2), "errors", len(rows) - len(ok))
