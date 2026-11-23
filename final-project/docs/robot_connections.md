# How every part of the v6 model is connected (low tray + gravity chute, roofed pusher sled, right-rear caster)

This guide goes with `robot_3d.html` (v6) and `docs/robot3d_src/assembly.py`. Older versions are kept as
`robot3d_src/assembly_v5.py` (v5.1 = `assembly_v51.py`), `assembly_v4.py` and `assembly_v3.py`. The script places each
LEGO part on the real hole positions from its LDraw model, then checks every pin and axle: each one must sit coaxially
in the holes of at least two parts, no pin may sit in a cross (axle) hole, no two connectors may share a hole, and all
parts must form one connected structure.

- **Result:** 62 parts and 98 pins/axles, all passing, one connected structure. Paper, cardboard and tape only (no
  glue); card parts are taped and not counted.
- **Collision checks:**
  - LEGO vs LEGO (`robot3d_src/obb_check.py`): nothing at rest or anywhere in the pusher stroke. The only hits are 3
    axle ends sitting half-way into a hole on purpose (the 12L cross axles in the rear ears, the 9L in the outer bearing).
  - Moving LEGO vs card (`robot3d_src/pusher_card_check.py`): over the full stroke the closest approach is
    **1.01 cm** (rod vs chute ceiling, at rest). The rod never reaches the sliding roof.
- **Simulator** with the v6 outline (`sim_outline_v6.py 4`, 12 runs):
  - 76.2 % of the snow (v5: 76.3 %), no contact runs.
  - Minimum house clearance 0.88 cm (v5: 1.85), because the caster now fills part of the notch behind the right wheel.
- **Coordinates:** cm from the drive-axle line on the table; x = right, y = up, z = backward. Everything is given in the
  "level" frame; the real robot leans back 2.1° onto the caster (section 8).

## What changed from v5.1, and why
1. **Pom-poms reach the tray by gravity, not by being thrown.** In v5 the tray floor sat on the rails (y 3.4) with
   the mouth at 5.6, above the roller axis (4.4). The flaps only shed pom-poms backward below the axis, so they would have
   had to be thrown over the mouth. That needs ≈ 1500 °/s at the best release angle; the code runs the roller at 900 and
   the EV3 Medium tops out at about 1440–1500 with no load. A sloped roof can't fix it: fluffy pom-poms lose their speed
   when they hit card. v6 drops the floor to 0.8 cm above the table and ends the guide 5° *below* the axis, so a 29°
   chute runs downhill from there into the tray.
2. **The crank is actually driven.** The NXT output is a cross hole and beams have round holes, so v5's crank would
   have just spun loose on its axle. v6 adds a 1x2 liftarm with a cross hole and a pin hole (kit **No.3**, LDraw 60483)
   between the NXT and the crank. The axle turns it, and its pin turns the crank.
3. **The crank and rod no longer touch the card.** The crank is now 5-hole and the rod 13-hole, pinned into the top
   hole of the sled's L-beam. The NXT doesn't move. The closest approach to any card is 1.01 cm.
4. **The sled has a roof** (your idea). See section 7.
5. **The caster moved to the right-rear corner.** A floor 0.8 cm off the table leaves no room for a caster under it.
6. **The lean is now drawn.** The 18 mm ball is 3 mm short when its pins are at hub height. This was already true in
   v5, but now the page tilts the robot back onto the ball.

## Overall size
| | v6 | v5.1 |
|---|---|---|
| Width | 22.0 (tyres and both roller frames inside ±11.0) | 22.0 |
| Roller | 19.2 (x −10.0 to +9.2), held at both ends | same |
| Front reach (flap tips ahead of the axle) | 12.85 | 12.85 |
| Rear (colour sensors) | 11.6 | 11.6 |
| Rear support | **ball caster at x 8.8, z 8.0** (right rear, behind the right wheel) + anti-tip foot on the left | caster on the centre line, z 7.2 |
| Tray floor | **0.8 cm** above the table | 3.4 |
| Plan outline change | the caster fills part of the notch behind the right wheel (x 8.4–10.0, z 6.8–8.6) | |

## 1. Drive motors, tied together at the back (unchanged)
- **Motors:** EV3 Large at x = ±6.4, hub 2.8 cm up (56 mm wheels). Wheels at x = ±9.6 on an 8L axle per side
  (rail → motor hub → rim).
- **Back beams:** top row (y 6.8) a 7-hole beam per motor with 3 black pins into the motor's 3 back holes; bottom row
  (y 3.6) a 3-hole beam with 2 black pins.
- **Back crossbar:** a 13-hole beam across both motors at y 6.8, with 2 black pins into each top back beam.

## 2. Colour sensors (position unchanged)
- **Posts:** vertical beam per side at x = ±8.0 (7-hole left, 11-hole right), pinned to both back beams.
- **Sensor:** pinned through its side tab into the post with 2 pins. The lower pin no longer carries a caster bar:
  - left: a blue 3L pin (its front 0.8 cm sticks out into free space);
  - right: a black pin, because the caster rail ends just in front of it.
- **Eye:** 0.8 cm above the table in the level frame, **≈ 0.4 cm once the robot leans back** (z 10.4, at x = ±9.7).
  Calibrate the colour thresholds on the real robot.

## 3. Rails, motor stems, top (unchanged from v3)
- Rails: 15-hole beam on edge, 2 blue 3L pins into each motor foot.
- Stems: a 3x5 L-beam on each side of each motor's stem, 2 blue 3L pins through the stem.
- Top: uprights on the inner stem L-beams, 9-hole top rails at y 10.8, NXT pinned front/side/back, two 12L cross axles
  through left rail → NXT → right rail, Pi case on 4 L-beam ears.

## 4. Roller module
### Left frame: holds the EV3 Medium (unchanged from v4)
- The motor sits **inside the tube**, output face at x = −0.6, pointing right, on the roller axis (y 4.4, z −8.8).
- **4 × 13-hole beams**, one black pin each into the motor's 4 side pin holes next to the output, running left to the
  frame plane at x = −10.6 (just outside the tube's left end).
- **Left frame plane:** two 3x5 L-beams (each locked to its beam pair with 2 pins, so the frame can't shear like a
  parallelogram), two 3-hole spacers, a 13-hole top crosspiece → cross block → 5L axle into the left outer stem L-beam,
  and a 7-hole bottom crosspiece → cross block → 8L axle into the left rail's front hole.

### Right frame: the second bearing (new in v5)
Two beam planes just past the tube's right end, inner at x = 9.8 and outer at x = 10.6 (outer face 11.0, same as the
tyre). Shortening the roller to 19.2 cm is what makes room for them.
- **Inner plane (x 9.8):**
  - `rfL`: 3x5 L-beam, long arm vertical from y 2.8 up to 6.0, short arm running back along the bottom.
    Its hole at y 4.4 is the **bearing**.
  - `rfT`: 13-hole beam at y 6.8 running back over the right wheel (0.8 cm above the tyre). Its rear hole takes a
    **5L axle** into the right outer stem L-beam (mirror of the left frame).
- **Outer plane (x 10.6):**
  - `rfV`: 3x5 L-beam, long arm hanging down from y 6.8 to 3.6, short arm running back along the top.
    Its hole at y 4.4 is the **second bearing**, in line with the first.
  - `rfB`: 7-hole beam at y 2.8 running back to z −4.0, where an **8L axle** goes through it into the right rail's front
    hole (mirror of the left frame).
- **Pins (all black, across the two planes):** 2 at the bottom (rfL short arm ↔ rfB), 2 on the vertical
  (rfL long arm ↔ rfV long arm, at y 3.6 and 6.0), 2 at the top (rfV short arm ↔ rfT). Each pair of overlapping
  pieces has 2 pins, so no joint can hinge.
- The bearing is two round holes 0.8 cm apart in two parts that are pinned together, so the roller's right end is
  held in a fixed line.

### The roller itself (spare wheels as hubs; paper, card and tape only)
- **Hubs = the 2 spare wheels** (rim No.16 + tyre No.17). The tyre is 5.6 cm across, so a card tube rolled to a snug
  fit over the tyre grips it without glue. Tape wrapped around the outside of the tube over each wheel squeezes the
  card onto the rubber.
  - **One shaft, motor to right frame:** a **6L axle** from the EV3 Medium's output through wheel 1's hub
    (x = 1.0–1.8, tyre 2 mm clear of the motor face) into the **axle joiner (No.18)** at x = 2.6–4.2, then a **9L axle**
    from the joiner through wheel 2's hub and the right-frame bearing holes. The 9L ends at x = 10.6, halfway through the
    outer layer, so it is fully in the inner layer and half in the outer one. Both wheels are on one straight shaft
    held at both ends: the motor's own bearings on the left and the two-layer frame on the right.
  - **Why the joiner:** the rim's cross hole is only 1 stud (0.8 cm) deep, so two axles can't share it. And the motor
    output to the outer face is 12.4 cm, which no single axle fits.
  - **Other lengths that work** (the joiner sits between the wheels either way): 6L + 9L (modelled), 7L + 8L, or
    8L + 7L. Use whichever your kit has.
  - Tape over the outer bearing hole so the 9L can't slide out.
- The kit's black four-armed piece (No.26, which I wrongly called "the X") is **no longer used**, and neither is
  No.25.

| | |
|---|---|
| Tube length | **19.2 cm** (x −10.0 to +9.2) |
| Tube inside diameter | **5.6 cm** = the tyre's outside diameter (snug sliding fit) |
| Tube outside diameter | ≈ 5.8 cm (2 wraps of thin card) |
| Flaps | 8 rows, 1.15 cm tall, every 45°, **V-angled** (each half at ~25° to the axis, ends leading the centre by ~4.4 cm around the tube), slit every ~2.7 cm |
| Flap tip diameter | 8.1 cm; tips 3.5 mm above the table in the level frame, ≈ 6.7 mm with the 2.1° lean (section 8) |
| Wheel 1 / wheel 2 hubs | 11.0–11.8 cm / 17.8–18.6 cm from the tube's left end |
| Axis | 4.4 cm above the table, 8.8 cm ahead of the drive axle |
| Left end | open: the motor's 4 beams and the L-beam arms go in there |

**Build steps** (also drawn in `renders/10_roller_build.png`):
1. Roll thin card (cereal-box weight) twice around a spare wheel so it is a snug sliding fit. Tape the seam along its
   length. Cut to 19.2 cm.
2. Flaps (V-angled): 16 strips of card, 10.5 × ~2.2 cm, two per row. Fold each lengthwise into an L (1 cm foot,
   1.15 cm flap). Draw 8 V lines on the tube 45° apart: each half runs straight from the tube's middle to its end, with
   the end ~4.4 cm further round in the turning direction (~25° to the axis). Tape the feet along the lines and slit
   the flaps every ~2.7 cm. Ends lead and the centre trails, so the flaps push pom-poms toward the middle
   (`renders/12_v_flaps.png`).
3. Shaft first: push a 6L axle through wheel 1's hub so 2.4 cm sticks out on one side, and push the joiner (No.18)
   onto the other end. Push wheel 1 (stub pointing left) into the tube from the right until its centre is 11 cm from
   the left end. Push wheel 2 in flush with the right end. (The joiner can't be added once both wheels are in.)
   Wrap 2–3 turns of tape around the tube over each wheel.
4. Slide the tube onto the motor from the right (it passes over the motor and its 4 beams) and turn it until the 6L
   stub drops into the EV3 Medium's output.
5. Push a 9L axle in from outside through both right-frame bearing holes and wheel 2's hub into the joiner. Tape over
   the outer hole.

## 5. Chute, hood and ceiling (card; replaces v5's funnel floor and deflector)
- **Guide** (blue, under the roller): an arc 4.4 cm from the axis, from the table up to **5° below the axis**
  (z −4.42, y 4.02). There the flaps still move up and slightly back. A short skirt at the front reaches the table,
  because the 2.1° lean lifts the arc's bottom by about 3 mm.
- **Chute** (blue): a flat sheet from the guide end down at **29°** to the **lip** at z −1.68, top surface y 2.5. It is
  the full roller width at the top and narrows to the tray width (±4.3) by z −3.2.
- **Funnel walls** (grey): straight from the roller ends (x −10.0 / +9.2 at z −4.42) to ±4.3 at z −3.2, then along the
  tray walls. Up to z −3.2 the chute surface is still above the rails (y 3.2) and the frame axles at z −4.0 (top 3.04).
  Behind that it stays between the rails.
- **Hood** (yellow): an arc 4.85 cm from the axis, from 125° over the top to 30°. A flat **ceiling** then runs from there
  (z −4.6, y 6.8) to z −2.0, y 5.2, so pom-poms that bounce up come back down onto the chute.
- **Mouth:** 2.7 cm tall at z −2.0 (between the chute and the ceiling).
- **Risk:** the funnel walls are steep (≈ 77° to the direction of travel). The V-flaps move pom-poms about 2.8 cm toward
  the middle while they ride up the guide. Pom-poms caught at the very ends of the roller can still pile against the
  walls. Watch for this in the bench test.

## 6. Tray (card)
- **Floor:** 0.8 cm above the table (0.6–0.8), hanging between the rails, from z −4.7 (under the chute) to 11.0.
  - Use stiff card, two layers or corrugated. The rod pushes the sled down hard near the start of each stroke, and the
    floor must not sag onto the table. With the lean, the floor's back edge is only ≈ 0.2 cm above the table.
- **Inside:** 8.6 wide × 12.7 long × 5.6 tall behind the lip.
- **Walls:** taped to the rails' inner faces (x ±4.3–4.4).
- **Lips** (yellow, 0.6 cm shelves) on both walls at y 2.35–2.5, from the chute lip back to z 8.0. They hold the
  sled's roof down.
- **Rear flap:** hinged with tape along its top edge and passive, as in v5. Pom-poms leaking past it while driving is
  still a risk; the fixes are a ~5 mm card sill and/or a coin taped to the flap. Bench-test it.

## 7. Pusher: NXT → No.3 adaptor → 5-hole crank → 13-hole rod → L-beam sled (plate + cheeks + roof)
- **Crank drive:**
  - The 5L axle runs in the NXT output (cross hole) and the adaptor's cross hole, and through the crank's round end hole.
  - The adaptor's pin hole takes a black pin into the crank's second hole, so the adaptor drives the crank.
  - Order along x: rod (−0.8), crank (0), adaptor (0.8), NXT hub (1.2–3.6). The NXT is not moved.
- **Rod:** 13-hole beam, from the crank's end hole (r 3.2) to the **top hole of the sled L-beam** (y 2.8).
- **Sled:** a 3x5 L-beam at x −1.6. The long arm lies forward on the floor as a runner; the short arm stands up
  (holes at y 1.2 / 2.0 / 2.8). Taped to it:
  - the **plate**: 8.3 wide, top at **2.1**, 0.36–0.66 behind the post;
  - two **cheeks** along the walls;
  - the **roof**: a flat sheet on the plate top reaching **5.6 cm forward**. At rest its front edge is at z −4.7, under
    the chute; it has a 0.9 × 0.8 cm notch where the L-beam comes through.
- **Strokes** (the NXT is position-controlled, so the crank can stop part-way):
  - **Compaction stroke, 3.0 cm** (crank 140.9° → 184.0°), during collection with the roller running. The roof still
    reaches under the chute lip the whole time, so pom-poms coming down the chute land on the roof or behind the plate,
    never in front of it. On the way back the roof slides under the lip, which stops anything sitting on the roof. As the
    plate passes under them they drop off the back edge into the tray.
  - **Full stroke** (crank 140.9° → 302.5°): plate face from 0.86 to 10.86 (the floor ends at 11.0). This is the dump.
    Stop the roller first: beyond 3 cm the roof no longer covers the landing zone.
- **Capacity:** the floor behind the plate (8.6 × 10 cm) holds about 45 pom-poms in one layer. Compaction strokes push
  that layer back so the front stays clear, and the pile can stack up to the 6.4 cm walls. Bench-test how many fit
  before the mouth chokes (expect well over 50).

## 8. Rear ball caster and the lean
- The kit's steel-ball caster (92911 socket + 18 mm ball), ball centre at **x 8.8, z 8.0**: behind the right wheel, in
  front of the right colour sensor (gap 0.04 cm to its case).
- **Mounting:** a 7-hole beam (`casterRail`) lies along z at x 8.0, y 2.8, against the outer face of the right motor's
  foot. It is held by a black pin at z 5.6 and a blue 3L pin at z 7.2. The 3L pin goes through the motor foot, the rail
  and the caster's front hole. A black pin at z 8.8 takes the caster's rear hole.
- **Lean:** the ball hangs 1.6 cm below its pin holes, so with the holes at hub height its bottom is 0.3 cm above the
  table. The robot leans back asin(0.3/8.0) = **2.1°** until the ball touches. Effects:
  - roller tips +0.33 cm (lowest tip ≈ 0.67 cm above the table, still below a pom-pom's middle);
  - colour eyes ≈ 0.4 cm above the table;
  - floor back edge ≈ 0.2 cm above the table.

  A centred caster would lean the same way. Only a different wheel or ball size removes it.
- **Loads** (estimate, 875 g, battery assumed 150 g on top of the Pi): **left wheel 47 %, right wheel 21 %, caster 32 %**.
  The light right wheel still has ≈ 1.8 N of grip, which is plenty for driving. Hard accelerations are the concern.
- **Tipping:** the centre of mass is 1.6 cm inside the line from the left wheel to the caster. Forward acceleration
  above ≈ **0.21 g (2 m/s²)** would rock the robot back onto that line, lifting the right wheel. So:
  - the **anti-tip foot** is an L-beam pinned to the outer face of the left motor foot (2 black pins), short arm down to
    0.8 cm (≈ 0.5 cm above the table once leaned). It catches the robot after ≈ 4° of rock;
  - the code should ramp speed changes rather than step them (not changed; needs your go-ahead).

## Kit use (drawn)
| Part | Used | Kit |
|---|---|---|
| black pin (No.1) | **60** | 61 |
| blue 3L pin (No.8) | 26 | 52 |
| 15 / 13 / 11 / 9 / 7 / 5 / 3-hole beams | 2 / 8 / 1 / 3 / 8 / 1 / 4 | 2 / 10 / 6 / 14 / 20 / 18 / 16 |
| L-beam 3x5 (No.48) | **14** | 14 |
| 1x2 liftarm with cross hole + pin hole (No.3, 60483) | 1 | 8 (please confirm No.3 is this part) |
| cross block (No.30) | 2 | 8 |
| bush (No.58) | 0 | 11 |
| wheel rim / tyre (No.16 / No.17) | 4 / 4 | 4 / 4 |
| ball caster | 1 | 1 |
| axle joiner (No.18) | 1 | 6 |
| axles | 4× 5L, 1× 6L, 4× 8L, 1× 9L, 2× 12L | lengths in the kit not identified yet |

- The ear pins are now blue 3L pins (6 of them), which brought the black pins back under the kit count (1 spare).
- All 14 L-beams are used.

## Every joint (generated by assembly.py)
| Joint | Uses | Parts in the joint |
|---|---|---|
| drive axle: rail -> hub -> rim | 2x axle 8L | motor_L, motor_R, rail_L, rail_R, rim_L, rim_R |
| rail_L -> motor foot | 2x blue 3L pin | motor_L, rail_L |
| back beams_L -> motor back | 5x black pin | backB_L, backT_L, motor_L |
| post_L -> back beams | 2x black pin | backB_L, backT_L, post_L |
| colour_L -> post | 1x black pin | colour_L, post_L |
| post_L -> colour_L (lower pin) | 1x blue 3L pin | colour_L, post_L |
| stem L-beams_L -> motor stem | 2x blue 3L pin | motor_L, stemIn_L, stemOut_L |
| upright_L -> stem L | 3x blue 3L pin | stemIn_L, upright_L |
| top_L -> upright | 1x black pin | top_L, upright_L |
| ears_L -> top rail | 3x blue 3L pin | earF_L, earR_L, top_L |
| Pi case -> ear_L | 4x black pin | earF_L, earR_L, pi_case |
| rail_R -> motor foot | 2x blue 3L pin | motor_R, rail_R |
| back beams_R -> motor back | 5x black pin | backB_R, backT_R, motor_R |
| post_R -> back beams | 2x black pin | backB_R, backT_R, post_R |
| colour_R -> post | 1x black pin | colour_R, post_R |
| post_R -> colour_R (lower pin) | 1x black pin | colour_R, post_R |
| stem L-beams_R -> motor stem | 2x blue 3L pin | motor_R, stemIn_R, stemOut_R |
| upright_R -> stem L | 2x blue 3L pin | stemIn_R, upright_R |
| top_R -> upright | 1x black pin | top_R, upright_R |
| ears_R -> top rail | 3x blue 3L pin | earF_R, earR_R, top_R |
| Pi case -> ear_R | 4x black pin | earF_R, earR_R, pi_case |
| back crossbar -> top back beams | 4x black pin | backT_L, backT_R, backX |
| NXT back -> nxtBack beam | 3x black pin | nxt, nxtBack |
| nxtBack -> right post | 1x black pin | nxtBack, post_R |
| NXT front stem -> right upright -> right stem L | 1x axle 5L | nxt, stemIn_R, upright_R |
| NXT rear box -> right top rail | 1x black pin | nxt, top_R |
| cross axle: left top rail -> NXT -> right top rail | 2x axle 12L | nxt, top_L, top_R |
| mbeam_TF -> EV3 Medium side hole | 1x black pin | mbeam_TF, medium |
| mbeam_TR -> EV3 Medium side hole | 1x black pin | mbeam_TR, medium |
| mbeam_BF -> EV3 Medium side hole | 1x black pin | mbeam_BF, medium |
| mbeam_BR -> EV3 Medium side hole | 1x black pin | mbeam_BR, medium |
| top L-beam -> front top motor beam | 1x black pin | frameLT, mbeam_TF |
| top crosspiece -> rear top motor beam -> top L-beam | 1x blue 3L pin | frameLT, frameT, mbeam_TR |
| top crosspiece -> spacer -> top L-beam | 1x blue 3L pin | frameLT, frameSp2, frameT |
| top crosspiece -> spacer | 1x black pin | frameSp2, frameT |
| bottom crosspiece -> bottom motor beam -> bottom L-beam | 2x blue 3L pin | frameB, frameLB, mbeam_BF, mbeam_BR |
| top L-beam -> front top motor beam (2nd pin) | 1x black pin | frameLT, mbeam_TF |
| bottom L-beam -> front bottom motor beam (2nd pin) | 1x black pin | frameLB, mbeam_BF |
| bottom L-beam -> spacer -> top L-beam | 1x blue 3L pin | frameLB, frameLT, frameSp |
| bottom L-beam -> spacer | 1x black pin | frameLB, frameSp |
| cross block -> bottom crosspiece | 1x black pin | frameB, xblock_low |
| left frame (cross block) -> left rail | 1x axle 8L | rail_L, xblock_low |
| cross block -> top crosspiece | 1x black pin | frameT, xblock_high |
| left frame (cross block) -> left outer stem L | 1x axle 5L | stemOut_L, xblock_high |
| right frame: L-beam -> bottom beam | 2x black pin | rfB, rfL |
| right frame: L-beam -> upright L-beam | 2x black pin | rfL, rfV |
| right frame: upright L-beam -> top beam | 2x black pin | rfT, rfV |
| right rail -> right frame bottom beam | 1x axle 8L | rail_R, rfB |
| right outer stem L -> right frame top beam | 1x axle 5L | rfT, stemOut_R |
| EV3 Medium output -> wheel 1 -> axle joiner (6L) | 1x axle 6L | hub1_rim, joiner, medium |
| axle joiner -> wheel 2 -> right-frame bearings (9L) | 1x axle 9L | hub2_rim, joiner, rfL |
| caster rail -> right motor foot | 1x black pin | casterRail, motor_R |
| right motor foot -> caster rail -> ball caster | 1x blue 3L pin | caster, casterRail, motor_R |
| caster rail -> ball caster | 1x black pin | caster, casterRail |
| anti-tip foot -> left motor foot | 2x black pin | motor_L, tipFoot |
| NXT hub -> adaptor -> crank (5L) | 1x axle 5L | adaptor, crank, nxt |
| adaptor pin hole -> crank hole 2 (drives the crank) | 1x black pin | adaptor, crank |
| crank -> rod | 1x black pin | crank, rod |
| rod -> sled L-beam (top hole) | 1x black pin | pusherL, rod |
