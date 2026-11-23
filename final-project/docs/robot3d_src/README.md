Source for `docs/robot_3d.html`, the **v6** model: 19.2 cm roller held at both ends, low tray (floor 0.8 cm up) fed by a
29° gravity chute, crank pusher driven through a No.3 adaptor, sled with a roof, right-rear ball caster plus a left
anti-tip foot, 22 cm wide. Every part is pinned. Older versions:
- `assembly_v51.py` / `robot3d_v51.html` (v5.1)
- `assembly_v5.py` (v5)
- `assembly_v4.py` / `robot3d_v4.html` (v4)
- `assembly_v3.py` / `robot3d_v3.html` (v3)

It is one offline file, with three.js, the LDraw parts and the placements inlined.

Copy this folder to `/tmp/robot3d/` first: the scripts import each other from there.

1. Get the LDraw library: `curl -O https://library.ldraw.org/library/updates/complete.zip`, then unzip it to
   `/tmp/ldraw/`. This sets `LIB` in pack.py.
2. Put the three.js 0.170 files in `/tmp/three/`: three.module.min.js, LDrawLoader.js and OrbitControls.js, from unpkg.
3. `python3 assembly.py asm.json`
   - Places every part on real LDraw hole positions (`holes.py` reads the hole primitives).
   - Checks that every pin and axle engages at least 2 parts, with no pin in a cross hole, and that all parts are connected.
   - Prints the kit tally and writes the placements, including the pusher stroke samples.
4. Pack the parts:

   ```
   python3 pack.py 95658:15,99455:15,53787:15,95650:15,56908:71,41897:0,32523:71,32316:71,32524:71,40490:71,32525:71,41239:71,32278:71,32526:0,6536:71,2780:0,6558:1,3713:0,4519:0,3705:0,32073:0,3706:0,44294:0,3707:0,3737:0,3708:0,60485:0,59443:0,92911c01:0,60483:0 parts.mpd
   ```
5. `python3 build.py robot3d.html parts.mpd ../robot_3d.html asm.json`
6. Renders: `./shot.sh <abs path>/robot_3d.html out.png 1300 850 "scene=robot&t=0.4&view=iso&noui"`
   - Hash options: `scene=robot|corner|dump`, `view=iso|side|top|front|rear|roller`, `cam=x,y,z,tx,ty,tz`, `t=<freeze time>`.
   - Display flags: `nodeck`, `xray`, `nolabels`, `noui`; `clip=<x>` cuts the view at x (section view, e.g. `clip=-0.45`).
   - `t=` freezes the timeline: in the robot scene, 1.0–3.4 s and 4.0–6.4 s are 3 cm compaction strokes and 7.5–12.5 s is a full stroke.
   - `sweep` puts the corner minimum clearance in the page title.
7. Other tools:
   - `python3 proj.py <part> out.png` draws a part's silhouettes with its holes marked.
   - `sim_outline_v3.py` … `_v6.py` run the simulator with each version's outline (run from final-project/).
     `python3 docs/robot3d_src/sim_outline_v5.py 4 16.0` = 4 seeds per map with `CORNER_POKE_CM` = 16.
   - `roller_build_fig.py` draws `renders/10_roller_build.png` (roller section and tape-only build steps; matplotlib).
   - `python3 assembly.py` also prints the estimated mass, centre of mass, wheel and caster loads, the lean, and the tipping margin.
   - `python3 pusher_card_check.py`: the moving crank and rod vs every card surface over the full stroke.
     - It also reports whether the rod reaches the sled roof and how long a compaction stroke the roof allows.
     - `pusher_card_check_v5.py` is the version that found the v5 collision.
   - `python3 obb_check.py`: LEGO-vs-LEGO interference at rest and over the stroke.
   - `python3 gen_joints.py`: the joint table in `robot_connections.md`.
   - `sim_outline_v6.py`: the simulator with the v6 outline.
