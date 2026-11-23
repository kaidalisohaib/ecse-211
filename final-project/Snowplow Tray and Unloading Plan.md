# Snowplow Tray and Unloading Plan

Nov 2, 2026 · @Sohaib Kaidali

The catch tray needs about 22 × 4.5 cm of floor; its volume barely matters. A sideways conveyor floor for unloading is worth trying only if the drop location is on the floor, and only after the roller catches 19 of 20 on the bench.

## The dumpster tray

Size the tray by floor area, not volume: it needs room for one layer of all 50 pom-poms, about 22 × 4.5 cm.

### Why floor area beats volume

Fifty pom-poms at about 1.4 cm are only about 72 cm³ solid, or about 130 cm³ loose. They don't roll, so they pile up where they land, right behind the roller. Once the pile reaches the release point (about 1.3 cm off the floor in v2), new pom-poms get pushed back into the roller and it jams.

One layer of all 50 needs about 80–100 cm². The 3 cm deep tray in the first v2 drawing holds roughly 30 before it starts to choke, so it is too small.

### Recommended shape

| Part | Size or setting | Why |
| --- | --- | --- |
| Inside floor | 22 cm wide × 4.5 cm deep, flat, about 3 mm above the table | One layer of all 50; 1 cm narrower than the roller |
| Front lip | About 1.5 cm high | Stays below the release point |
| Guide end | About 205° instead of 220° | Raises the release point to about 2 cm, leaving room for a layer plus some |
| Back wall | 4–5 cm tall, leaning back about 15° | Contents fall out cleanly when the arm turns the tray over |
| Lid | Curved card, from the top of the hood to the top of the back wall | Pom-poms hit it and drop near the back, so the tray fills back to front |
| Ends and top | Closed, except the entry slot | Nothing spills during the swing over the robot |

The deflector lid matters as much as the tray size: without it the front fills first and jams early.

### Where the extra depth comes from

- **Move the roller about 1.4 cm forward**, to a front reach of about 13 cm. With the front corners cut back 6 cm instead of 5, the turn checker still gives 1.8 cm of clearance at corners. Spinning in place drops to 0.7 cm, so turn only at corners.
- **Or notch the back of the tray** around the wheels and motors. Measure that space on the robot first; the drawing on the design page guessed at it.

## Conveyor belt for unloading

A sideways belt floor beats the swinging arm only when the drop location is a spot on the floor, so don't build it before the roller works.

### What it looks like

The tray's floor is a belt running across the robot. At the drop point, the belt runs and the pom-poms walk out a door at one end. They land beside the robot, outside the wheel tracks, so the robot drives away without touching the pile.

### Belt vs. swinging arm

|  | Sideways belt floor | Swinging arm (overshot) |
| --- | --- | --- |
| Tray height | Stays low | Swings about 20 cm over the robot |
| Spill or tip risk | Low | Higher: more can catch, spill or tip |
| Drop | Gentle, at floor level, in short bursts if needed | Drops behind the robot |
| Drop position | About 12–15 cm to one side of the robot's centre | Behind the robot |
| Drop into a box or bin | No, can't get over the edge | Yes |
| Build risk | High: belt tracking, slip, edges catching on side walls | Moderate |
| Motor ports | Same as the arm (roller + one more) | Roller + arm |

The belt can also run slowly while sweeping to spread pom-poms across the tray, which helps the floor-area problem. The load is tiny (about 100 g), which makes a paper belt more forgiving.

### Recommendation

1. Get the roller and tray catching 19 of 20 on the bench first; that's where the points are.
2. Ask the professor what the drop location physically is.
3. If it's a marked spot on the floor, try the sideways belt as a separate tray module you can swap with the arm version. If it's a container, use the arm.

Open question: is the drop location a floor spot or a container?
