# ECSE 211 Final Design Project: Autonomous Snowplow System
## System Architecture, Mechanism Engineering & Requirements Document

---

## 1. Executive Summary & Problem Formulation

### 1.1 Client Problem Statement
Municipal roadway maintenance authorities require an autonomous robotic solution capable of clearing winter snow accumulation from urban street networks without human intervention. The urban environment is represented by a grid of 30.48 x 30.48 cm (1 ft x 1 ft) tiles bounded by street walls (30.48 x 30.48 x 30.48 cm weighted boxes representing residential structures). 

The operation is strictly bifurcated into two distinct phases, each allocated a hard maximum operational time of 2 minutes (120 seconds):
1. Phase 1 (Summer Mapping): Operating in a pristine, snow-free environment, the robot must depart from the origin (0, 0), locate and enter the designated street through entrance points s1 and s2, autonomously discover and map the complete topological geometry and dimensions of the street network (with physical wall contact permitted), and return safely to (0, 0).
2. Phase 2 (Winter Snow Removal): Given the map generated in Phase 1, the robot departs from (0, 0), enters the street through (s1, s2), collects 50 uniform pom-poms (12.7 mm diameter) distributed along the street corridor without touching any street walls and without dragging snow or containers across the surface, transports the collected volume to the designated table-edge drop-off zone (d1, d2), ejects the payload cleanly into the table-height collection receptacle, and returns to (0, 0).

Crucially, two separate, optimized hardware configurations are permitted between Phase 1 and Phase 2.

```
                      SAMPLE MAP TOPOLOGY & WORKSPACE
  Y ^
  4 | [ ][ ][ ][ ][ ][ ][ ][ ][ ]
  3 | [ ][ ](s1)==================+ [ ]   <-- Street Entrance at (s1, s2)
  2 | [ ][ ] ||   STREET CORRIDOR | [ ]   <-- One tile wide (30.48 cm)
  1 | [ ][ ](s2)====+      +======+ [ ]   <-- Bounded by 30 cm house cubes
  0 | [R]   [ ]     |      |        [ ]
 -1 | [ ][ ][ ]    (d1)====(d2)     [ ]   <-- Drop-off zone at table edge
    +----------------------------------> X
      -1  0  1   2   3   4  5   6 7  8
     (R = Origin at (0,0); Bucket rim flush with table edge at d1-d2)
```

---

## 2. Mechanical Design Architectures

### 2.1 Geometric Envelope & Turning Clearance Analysis
The street corridor has a constant nominal width of W_street = 30.48 cm. In Phase 2, touching any wall incurs a severe penalty. When navigating a 90° street bend or executing a turnaround, the robot rotates about its center of differential drive (wheel axle midpoint).

* Maximum In-Place Clearance Radius:
  R_max < W_street / 2 = 30.48 cm / 2 = 15.24 cm

* Safety Margin Buffer: Allowing a positional tracking error margin Delta_safe = 1.74 cm, the maximum physical radius of any component from the axle center must satisfy:
  R_robot <= 13.50 cm

* Chassis & Sweeper Chamfering:
  A rectangular profile of width W = 22 cm (half-width y = 11 cm) and front overhang L = 11 cm has a square corner radius of:
  R_corner = sqrt(11^2 + 11^2) = 15.56 cm > 15.24 cm  -->  CRITICAL COLLISION!

  Geometric Fix: The front left and right corners of the intake scoop/bumper must be chamfered at 45° with a minimum 4.5 cm cutback.
  With a 4.5 cm chamfer, the corner coordinates become (x=11, y=6.5) and (x=6.5, y=11):
  R_chamfer = sqrt(11^2 + 6.5^2) = sqrt(121 + 42.25) = 12.78 cm <= 13.50 cm  -->  100% CLEARANCE GUARANTEED (>= 2.0 cm clearance on all sides).

```
                       TURNING ENVELOPE (TOP VIEW)
                     +---------------------------+  <-- Wall (30.48 cm apart)
                     |       Safety Margin       |
                     |         (1.98 cm)         |
                     |     /-----------------\   |
                     |    /  Chamfered Front  \  |
                     |   |   Sweeper Intake    | |
                     |   |     (Width 22cm)    | |
                     |   |                     | |
                     |  [W]======(AXLE)======[W] |  <-- Track Width = 12 cm
                     |   |    Drive Wheels     | |
                     |    \     Chamfered     /  |
                     |     \    Rear Gate    /   |
                     |                           |
                     +---------------------------+  <-- Wall
```

---

### 2.2 Phase 1: Exploration & Mapping Mechanisms

In Phase 1, no snow collection hardware is required, and wall contact is permitted. The robot should be lightweight, agile, and mechanically simple to achieve mapping within the 120 s limit.

```
+--------------------------------------------------------------------------------------------------+
| OPTION 1A: ACTIVE TURRET SCANNER (RECOMMENDED FOR ACCURACY)                                      |
| Configuration:                                                                                   |
| - Drive: 2 EV3 Large Motors (Ports B & C)                                                        |
| - Turret: EV3 Medium Motor (Port A) driving EV3 Ultrasonic Sensor (Port S3)                      |
| - Odometry Ground Sensors: Dual EV3 Color Sensors (Ports S4 & S2) facing downward                |
| Operational Principle:                                                                           |
| At each grid tile center (x_c, y_c), the turret rotates [-90°, 0°, +90°] in 0.35 s, pinging left,  |
| forward, and right. Distance < 25 cm indicates a wall; distance > 30 cm indicates an open street.|
| Pros: Zero physical wall dragging; precise topology discovery; perfectly non-destructive.       |
| Cons: Requires brief stop (0.4 s) at each node to take pings.                                   |
+--------------------------------------------------------------------------------------------------+
| OPTION 1B: WHISKER / TOUCH BUMPER SYSTEM (RECOMMENDED FOR MAXIMUM SPEED)                         |
| Configuration:                                                                                   |
| - Drive: 2 EV3 Large Motors (Ports B & C)                                                        |
| - Forward Wall Sensing: Fixed Forward Ultrasonic Sensor (Port S3)                                |
| - Lateral Wall Sensing: Dual Low-Friction Cardboard/Beam Whiskers with EV3 Touch Sensors (S1, S2)|
| Operational Principle:                                                                           |
| Continuous driving without stopping. Whisker switches compress when entering walled corridors.  |
| Pros: High speed (~15 cm/s); explores the entire street in under 45 seconds.                    |
| Cons: Whiskers can snag on rough cardboard box edges if tape is imperfect.                       |
+--------------------------------------------------------------------------------------------------+
| OPTION 1C: FIXED DUAL-SENSOR RIG (HYBRID CONTINUOUS)                                             |
| Configuration:                                                                                   |
| - Forward Ultrasonic (Port S3), Side Ultrasonic/Touch (Port S1), Gyro (Port S2), Down Color (S4)|
| Operational Principle:                                                                           |
| Fixed sensors looking straight ahead and 90° right. Robot performs right-hand rule exploration  |
| continuously without stopping for turret panning.                                                |
+--------------------------------------------------------------------------------------------------+
```

---

### 2.3 Phase 2: Snow Harvesting & Drop-Off Mechanisms

#### Snow Pellet Properties:
* Quantity: 50 pom-poms.
* Diameter: 12.7 mm (0.5 in).
* Individual Volume: 1.07 cm³; Total Solid Volume: 53.5 cm³.
* Loose Bulk Volume (packing fraction ~0.45): ~120-130 cm³.
* Total Mass: ~25-35 g (virtually zero structural load).
* Physics: Non-rolling, high inter-fiber friction, tend to snag and form clumps. Must be actively ingested and cannot be pushed along the floor (rule violation: "must not be dragged on the floor").

#### Drop-Off Target Breakthrough:
Per Section 2 of the project specification, the snow drop-off bucket is level with the table top at the edge between d1 and d2.

> [!IMPORTANT]
> Because the bucket rim is flush with the table edge, no heavy overhead vertical crane or tall lift arm is required! A horizontal discharge through a rear trapdoor, ejector paddle, or reverse conveyor belt directly sweeps the snow off the rear of the robot into the bucket.

```
                      MECHANISM OPTION COMPARISON MATRIX
+----------------------------+-----------------------+-----------------------+-----------------------+
| Criterion                  | Option 2A: Flap Roller| Option 2B: Front Scoop| Option 2C: Slat-Belt  |
|                            | + Rear Trapdoor       | Dustpan with Tilt Arm | Continuous Bed        |
+----------------------------+-----------------------+-----------------------+-----------------------+
| Ingestion Reliability      | 5/5 (95%+)            | 3/5 (75% - plows)     | 4/5 (85%)             |
| Anti-Dragging Compliance   | 5/5 (Instant lift)    | 3/5 (Pushes before)   | 5/5 (Instant lift)    |
| Discharge Simplicity       | 5/5 (Rear door drop)  | 3/5 (Over-axle dump)  | 5/5 (Motor reverse)   |
| Wall Clearance Envelope    | 4/5 (Chamfered hood)  | 2/5 (Wide swing)      | 4/5 (Compact)         |
| Mechanism Jam Risk         | Very Low              | Moderate              | High (fiber snagging) |
| Build Difficulty (LEGO/Card)| Easy (Tube + Tape)   | Moderate (Linkage)    | Hard (Tension/Track)  |
+----------------------------+-----------------------+-----------------------+-----------------------+
```

#### Detailed Architecture of Recommended Option 2A:
1. Front Roller Sweeper: 
   * A 22 cm wide cardboard tube or LEGO axle shaft fitted with 8 rows of flexible stationery-tape flaps (14 mm radial projection with 3 cm slit intervals).
   * Driven by Motor Port D via a 1:1 or 1:1.5 step-up gear ratio running at ~150-200 RPM.
   * Rotates upward into the intake mouth, pinching pom-poms against a curved bottom guide (6 mm clearance) and flicking them up and over into the containment tray.
2. Containment Hopper (Floor Tray):
   * Dimensions: 22 cm wide x 5 cm deep x 4.5 cm high (effective volume ~495 cm³, more than 3.5x the loose pom-pom volume).
   * Curved internal deflector cowl directs airborne pom-poms toward the rear of the hopper, preventing ingestion bottlenecks.
   * Internal floor sits 4 mm above the grid surface (zero dragging on floor).
3. Rear Gravity Dump Gate (Port A Motor):
   * When arriving at table edge (d1, d2), the robot squares perpendicular to the edge with its rear wheels 4 cm from the edge.
   * Motor A rotates a lightweight cardboard rear flap upward/open, and an internal hinged paddle sweeps backward, discharging all 50 pom-poms horizontally off the table edge into the flush collection bucket in < 2.0 seconds.

```
                 OPTION 2A: ROLLER INTAKE & REAR DISCHARGE
                         (SIDE ELEVATION PROFILE)

          [Curved Deflector Cowl]
                  |
     (Flap Roller)|            [Storage Hopper Tray]
       O===)      v               (Vol = 495 cm³)
     // | \\  .--------.       +--------------------+
    //  |  \\/  Curved  \      | 50 Snow Pom-Poms   |  [Rear Trapdoor]
   ||   v   ||   Ramp    \     |    (12.7 mm)       |      |
    \\     // \__________/     +--------------------+      v
      =====                     \                  /    | | | (Motor A)
   Intake Mouth                  \   Floor 4mm    /     | | |  Lifts
   (Lifts off floor)              \  above table /      /   \
  ======================================================     \ ---- Table Edge
  TABLE SURFACE                                               \   [BUCKET]
                                                               v
```

---

## 3. Software Architecture & Algorithmic Strategy

```
                               SOFTWARE SUBSYSTEM PIPELINE
  +---------------------------------------------------------------------------------------+
  |                                   HARDWARE LAYER                                      |
  |  Motors B&C (Drive) | Motor D (Intake) | Motor A (Turret/Gate) | Sensors S1,S2,S3,S4  |
  +---------------------------------------------------------------------------------------+
                                             ^
                                             | Sensor Poll & Motor Commands
                                             v
  +---------------------------------------------------------------------------------------+
  |                           ODOMETRY & MOTION CONTROL ENGINE                            |
  | - Differential Wheel Odometry (100 Hz Loop)                                            |
  | - Trapezoidal Velocity Profiling (Accel Ramp, Decel Crawl)                            |
  | - Closed-Loop Heading Hold (KP_DRIVE_HEADING = 2.0)                                   |
  | - Black Line Intersection Localization & Auto-Squaring (dual EV3 Color Sensors)       |
  +---------------------------------------------------------------------------------------+
                                             ^
                                             | Pose (x, y, theta) & Tile Arrivals
                                             v
  +---------------------------------------------------------------------------------------+
  |                             STREET TOPOLOGY & GRAPH ENGINE                            |
  | - Discretized Tile Grid Representation: (Gx, Gy) in [-1..8, -1..4]                    |
  | - Phase 1: Entry Alignment at (s1,s2) -> DFS / Right-Hand Rule Maze Exploration       |
  | - Wall State Persistence -> Generates street_map.json                                 |
  | - Phase 2: Graph Path Optimization (BFS Shortest Path) -> Sweeping Run -> Deposit Path|
  +---------------------------------------------------------------------------------------+
```

### 3.1 Odometry & Line-Squaring Subsystem
The navigation core provides high-accuracy movement:
* Grid Coordinate Mapping:
  Tile index (Gx, Gy) maps to real-world table coordinates:
  Xc = Gx * 30.48 + 15.24 cm
  Yc = Gy * 30.48 + 15.24 cm

* Trapezoidal Velocity Profile:
  Straight movements accelerate smoothly over 0.25 s to cruise speed (300 dps ~ 11.0 cm/s), then decelerate over 8.0 cm to crawl speed (45 dps ~ 1.6 cm/s).

* Perpendicular Grid Line Gating:
  When passing over grid lines, downward color sensors detect the black line (LINE_THRESHOLD = 30). Line squaring executes only within a valid spatial window (|Delta| <= 6.0 cm) to reject false triggers from pom-poms or shadows. The robot squares in < 0.8 s and resets its longitudinal position and heading drift to zero.

### 3.2 Phase 1 Exploration Algorithm (Graph Discovery)
1. Transit to Entrance: Navigate from (0, 0) to the midpoint of entrance line segment between s1 and s2:
   P_entry = ((x_s1 + x_s2) / 2, (y_s1 + y_s2) / 2)

2. Corridor Traversal (Right-Hand Exploration / DFS):
   * At each tile center (Gx, Gy), ping the 3 available relative directions (Left, Straight, Right).
   * Mark current tile as VISITED and register passable adjacent edges in graph G = (V, E).
   * Follow Right-Hand Priority: Turn Right if open; else Straight if open; else Turn Left; else Turn 180° (Dead-end turnaround).
   * Continue until the street terminates or all connected street tiles are explored.

3. Map Serialization: Write discovered street topology to street_map.json:
   ```json
   {
     "entrance": [[1, 2], [1, 3]],
     "street_tiles": [[1, 2], [2, 2], [3, 2], [4, 2], [4, 1]],
     "dropoff_target": [[2, -1], [3, -1]]
   }
   ```

4. Return Home: Execute Dijkstra / BFS shortest path from current terminal tile back to (0, 0). Total Phase 1 elapsed time: ~65-80 s (well under 120 s limit).

### 3.3 Phase 2 Snow Clearing & Drop-off Execution
1. Pre-computation: Load street_map.json. Generate an optimal coverage path that traverses every street tile containing snow.
2. Transit & Intake Activation:
   * Drive from (0, 0) to (s1, s2) at 350 dps (12.8 cm/s).
   * Activate Motor D (sweeper roller) at 180 RPM prior to passing through (s1, s2).
3. Corridor Sweeping:
   * Cruise along the centerline of each street tile at 100 dps (3.7 cm/s).
   * The 22 cm intake sweeps the central 72% of the tile width, while flexible side air/deflector curtains funnel adjacent pom-poms into the roller.
   * Line squaring at tile junctions eliminates cumulative odometry drift without stopping.
4. Transit to Drop-Off:
   * After traversing the street, execute BFS path to the drop-off waypoint adjacent to (d1, d2).
   * Align perpendicular to the table edge facing outward or backward.
5. Ejection Sequence:
   * Motor A actuates rear trapdoor and internal discharge paddle.
   * Run discharge sequence for 2.5 s. 100% of collected snow falls off the table edge into the bucket.
6. Return Home: Navigate back to origin (0, 0) and sound completion tone. Total Phase 2 elapsed time: ~85-95 s (well under 120 s limit).

---

## 4. Formal Requirements Document (Client Specifications)

### 4.1 Engineering Methodology: Solution Independence
In accordance with engineering design principles taught in ECSE 211, the requirements document must remain **solution-independent**:
* **Functions (What the system MUST DO):** Defined strictly in terms of active transformations, behaviors, inputs, and outputs. They must never specify implementation hardware (e.g., stating *"detect visual contrast transitions on the surface"* rather than *"use EV3 color sensors"*, or *"ingest snow pellets from the surface"* rather than *"spin a roller with tape flaps"*).
* **Objectives (HOW WELL the system should perform):** Continuous, quantifiable performance criteria to be maximized or minimized (e.g., maximizing collection yield, minimizing execution time, minimizing positional drift).
* **Constraints (Non-negotiable BOUNDARIES):** Hard limits, regulatory restrictions, and environmental boundaries imposed by the client prompt, rules, physical laws, and permitted materials.

---

### 4.2 Functional Requirements (20 FRs)
* **FR01 (Autonomous Locomotion):** The system must propel and steer itself across the planar grid surface without human intervention or physical guidance.
* **FR02 (Coordinate Acquisition):** The system must accept and store user-provided spatial coordinates for the street entrance (s1, s2) and drop-off zone (d1, d2) at runtime.
* **FR03 (Trajectory Generation):** The system must compute collision-free navigation trajectories between arbitrary waypoints within the operating workspace.
* **FR04 (Position & Heading Tracking):** The system must track its instantaneous position coordinates and heading orientation relative to the map origin.
* **FR05 (Surface Boundary Detection):** The system must detect visual contrast transitions or grid boundaries on the operating surface to support positional correction.
* **FR06 (Street Barrier Detection):** The system must detect the physical presence and location of street boundary walls along its path.
* **FR07 (Corridor Entry Alignment):** The system must align its position and heading with the designated street entrance line segment between s1 and s2.
* **FR08 (Corridor Traversal):** The system must follow the longitudinal contour of the street corridor through straight sections and turns.
* **FR09 (Corner & Junction Identification):** The system must identify changes in corridor direction, including 90° bends and corridor terminations.
* **FR10 (Topological Mapping):** The system must construct an internal topological representation of all discovered street segments and their connectivity during Phase 1.
* **FR11 (Map Data Persistence):** The system must retain the constructed topological map in non-volatile memory across operational phases.
* **FR12 (Map Recall & Replay):** The system must recall the stored map to plan optimal clearing routes during Phase 2 without re-exploring the corridor.
* **FR13 (Origin Return Navigation):** The system must navigate autonomously from any point in the workspace back to the initial origin (0, 0).
* **FR14 (Phase Transition Standby):** The system must halt locomotion and enter a stationary standby state at (0, 0) upon completing Phase 1 to await operator authorization.
* **FR15 (Snow Ingestion):** The system must lift loose snow pellets off the operating surface into an onboard containment volume.
* **FR16 (Snow Containment):** The system must securely retain all collected snow pellets within its onboard storage volume during high-speed driving and turns.
* **FR17 (Discharge Alignment):** The system must position and align its payload discharge interface relative to the designated drop-off boundary (d1, d2).
* **FR18 (Snow Deposition):** The system must discharge the stored snow payload from onboard containment into the external collection receptacle.
* **FR19 (Autonomous Mission Initiation):** The system must initiate execution sequences upon receiving a single start trigger.
* **FR20 (Operational Status Signaling):** The system must indicate its operational state (e.g., active mapping, standby pause, clearing, mission complete) to external observers.

---

### 4.3 Performance Objectives (20 OBJs)
* **OBJ01 (Maximize Snow Collection Yield):** The system should capture at least 80% (target: >= 90%, at least 45 of 50) of all snow pellets present in the street corridor.
* **OBJ02 (Minimize Snow Deposition Loss):** The system should fail to deposit less than 10% (target: <= 5%) of collected snow pellets outside the collection receptacle.
* **OBJ03 (Minimize Phase 1 Execution Time):** The system should minimize total mapping duration (target: <= 75 s; constraint threshold: <= 120 s).
* **OBJ04 (Minimize Phase 2 Execution Time):** The system should minimize total snow clearing duration (target: <= 85 s; constraint threshold: <= 120 s).
* **OBJ05 (Minimize Positional Drift):** The system should minimize cumulative uncorrected odometry drift over travel distance (target: <= 1.5° yaw drift per meter).
* **OBJ06 (Maximize Corridor Wall Clearance):** The system should maximize lateral clearance between its outermost perimeter and boundary walls during transit (target: >= 1.5 cm buffer).
* **OBJ07 (Minimize Swept Turning Radius):** The system should minimize the maximum swept outer radius during in-place rotational turns (target: <= 13.50 cm).
* **OBJ08 (Maximize Effective Clearing Swath):** The system should maximize the lateral ingestion swath cleared in a single longitudinal pass (target: >= 70% of corridor width).
* **OBJ09 (Maximize Volumetric Storage Margin):** The system should maximize the volume of the onboard storage compartment relative to loose snow bulk volume (target: >= 2.0x loose volume).
* **OBJ10 (Minimize Payload Discharge Duration):** The system should minimize the time required to complete snow payload ejection at the drop-off site (target: <= 3.0 s).
* **OBJ11 (Maximize Translational Cruise Velocity):** The system should maximize steady-state translational speed on unobstructed path segments without wheel slippage (target: >= 11.0 cm/s).
* **OBJ12 (Minimize Surface Alignment Duration):** The system should minimize the time consumed while correcting position against surface grid features (target: <= 0.8 s per feature).
* **OBJ13 (Minimize Origin Arrival Offset):** The system should minimize final radial positioning error upon returning to origin (0, 0) (target: <= 2.0 cm).
* **OBJ14 (Maximize Mapping Completeness):** The system should discover and map 100% of all contiguous navigable street corridor tiles during Phase 1.
* **OBJ15 (Minimize Energy Depletion):** The system should minimize electrical energy consumption across both operational phases (target: <= 20% battery capacity depletion).
* **OBJ16 (Maximize Ingestion Throughput):** The system should maximize the instantaneous intake rate of snow pellets without mechanical jamming (target: >= 8 pellets/second).
* **OBJ17 (Minimize Acceleration Jerk):** The system should minimize motion jerk and wheel slip during transitions between acceleration, cruising, and braking.
* **OBJ18 (Minimize Residual Retention):** The system should minimize snow pellets remaining stuck inside the storage container following discharge (target: 0 pellets remaining).
* **OBJ19 (Maximize Hardware Reconfigurability):** The system should minimize the time and tool complexity required to reconfigure hardware between Phase 1 and Phase 2.
* **OBJ20 (Maximize Path Efficiency):** The system should minimize total travel distance during snow clearing and drop-off by generating optimal shortest paths.

---

### 4.4 Design & Implementation Constraints (20 CONs)
* **CON01 (Phase 1 Duration Constraint):** Total runtime for Phase 1 shall not exceed 120.0 seconds under penalty of disqualification.
* **CON02 (Phase 2 Duration Constraint):** Total runtime for Phase 2 shall not exceed 120.0 seconds under penalty of disqualification.
* **CON03 (Workspace Boundary Containment):** The system must remain entirely within the physical footprint of the 9x5 grid table and shall not fall off table edges.
* **CON04 (Zero Wall Contact in Phase 2):** The system must not make physical contact with the street boundary walls at any point during Phase 2.
* **CON05 (Zero Ground Dragging):** Neither collected snow pellets nor any part of the snow containment or collection structure shall drag along the table floor surface during transit.
* **CON06 (Material Composition Restriction):** The physical structure of the robot must be fabricated exclusively from components in the assigned lab kit, cardboard (thickness <= 2.5 mm), and clear stationery tape (width <= 18 mm).
* **CON07 (Equipment Preservation):** The system must not inflict damage, permanent alteration, or adhesive residue on lab equipment, sensors, or table surfaces.
* **CON08 (Information Input Restriction):** No external spatial or geometric information about the street layout—other than the provided entrance coordinates (s1, s2) and drop-off coordinates (d1, d2)—shall be inputted into the system.
* **CON09 (Initial Starting Location):** The system must initiate both Phase 1 and Phase 2 from the designated origin coordinate (0, 0).
* **CON10 (Final Termination Location):** The system must terminate its mission by returning its physical footprint to the origin coordinate (0, 0).
* **CON11 (Corridor Entrance Passage):** Entry into the street network must occur strictly through the boundary segment defined between entrance points s1 and s2.
* **CON12 (Designated Drop-Off Interface):** Discharge of collected snow pellets must occur exclusively into the collection receptacle located between drop-off points d1 and d2.
* **CON13 (Flush Receptacle Interface):** The system must interface with a drop-off collection receptacle whose upper opening is flush with the table surface.
* **CON14 (Single Corridor Width):** The system must operate within a street corridor having a uniform lateral width of exactly one grid square (30.48 cm).
* **CON15 (Snow Pellet Dimension Specification):** The ingestion and containment mechanisms must be designed for discrete spherical pom-poms with a nominal diameter of 12.7 mm (0.5 in).
* **CON16 (Total Payload Quantity):** The system must be capable of processing a total snow quantity of 50 discrete pellets.
* **CON17 (Autonomous Operation Mandate):** Once initiated, the system must execute all sensing, decision-making, navigation, and manipulation without human telemetry or intervention.
* **CON18 (Actuator Port Limitation):** The system shall utilize no more than the 4 independent motor drive channels provided by the controller architecture.
* **CON19 (Sensor Port Limitation):** The system shall utilize no more than the 4 independent sensor input channels provided by the controller architecture.
* **CON20 (Self-Contained Power Source):** The system must operate entirely on an onboard DC battery power supply without external tethering.

---

## 5. Verification & Compliance Matrix

| Requirement ID | Verification Method | Pass / Fail Criteria | Lab 3/4 Validation Baseline |
| :--- | :--- | :--- | :--- |
| FR02, CON15 | Code Inspection & CLI Run | Accepts runtime inputs for s1, s2, d1, d2; no hardcoded layout | Verified via argparser/terminal prompt |
| FR04, OBJ18 | Functional Demonstration | Discovers 100% of street tiles in Phase 1 | Verified via grid DFS graph traversal |
| FR08, FR11 | File System Inspection | Creates street_map.json after Phase 1; loads in Phase 2 | Standard JSON serialize/deserialize |
| FR13, OBJ03 | Empirical Bench Test | Ingests >= 45/50 pom-poms in single pass over test strip | Flap roller bench test benchmark |
| FR17, OBJ16 | Timed Bench Test | Discharges payload into table-edge bucket in <= 3.0 s | Rear motorized gate gravity dump |
| OBJ01, OBJ02 | Stopwatch Timing | Phase 1 <= 120 s (target 75 s); Phase 2 <= 120 s (target 85 s) | Calibrated straight speeds (350 dps) |
| OBJ05, CON09 | Video & Proximity Log | Zero contact with cardboard wall boxes in Phase 2 | 12.78 cm chamfered radius vs 15.24 cm gap |
| OBJ07, OBJ09 | Odometry Log Analysis | Drift <= 1.5°/m; line squaring <= 0.8 s | Validated in Lab 4 line-squaring engine |
| CON04, CON05 | Physical Inspection | Cardboard <= 2.5 mm, tape <= 18 mm, zero adhesive residue | Caliper measurement & kit audit |
| CON06, CON07 | Port Allocation Audit | Exactly 4 motors (A,B,C,D) and 4 sensors (S1,S2,S3,S4) utilized | Ports verified on BrickPi3 schematic |

---

## 6. Grid-Based Auto-Tuning Calibration Subsystem (Zero Human Measurement)

Odometry errors stem from three mechanical parameters:
1. Nominal Wheel Radius (R): Causes systematic distance errors on straight segments.
2. Wheel Radius Asymmetry (E_d = R_R / R_L): Causes systematic curving (left or right) on straight drives.
3. Effective Track Width (W or b): Scuffing and tire scrubbing cause actual rotation per wheel degree to differ from physical contact distance, differing between Clockwise (W_cw) and Counter-Clockwise (W_ccw) turns.

### 6.1 Fully Autonomous Sensor-Driven Iterative Architecture
Instead of manual measurement or human eyeballing, the auto-tuning script (auto_tuner.py) uses the dual downward color sensors (Ports 4 & 2) and the black grid lines (D_tile = 30.48 cm) as an absolute closed-loop feedback loop:

```
                  AUTONOMOUS ITERATIVE CONVERGENCE PIPELINE
  +-------------------------------------------------------------------------------+
  | STEP 1: Autonomous Straight 2-Tile Calibration (60.96 cm)                     |
  | 1. Robot locks onto Line 0 using dual color sensors (resets start encoder).   |
  | 2. Cruises 2 tiles (60.96 cm); color sensors trigger on Line 2.               |
  | 3. Computes: R_wheel = 60.96 / (deg_avg * pi/180) and LEFT_SCALE / RIGHT_SCALE |
  +-------------------------------------------------------------------------------+
                                         |
                                         v
  +-------------------------------------------------------------------------------+
  | STEP 2: Iterative Closed-Loop Turn Calibration (CW & CCW)                     |
  | 1. Start locked squarely on a grid line.                                      |
  | 2. Execute commanded 360° in-place spin based on current Track Width estimate.|
  | 3. Relock onto grid line using dual color sensors; record squaring diff deg.  |
  | 4. Compute residual error: delta_theta = (diff_deg * R) / W.                  |
  | 5. Update: W_new = W_old * (360° / (360° + delta_theta)).                     |
  | 6. Repeat loop until |delta_theta| <= 0.4° (converges in 2-3 iterations!).    |
  +-------------------------------------------------------------------------------+
                                         |
                                         v
  +-------------------------------------------------------------------------------+
  | STEP 3: Auto-Serialization                                                   |
  | Saves converged parameters to calibration_params.json with zero user input.   |
  +-------------------------------------------------------------------------------+
```

### 6.2 Closed-Form Parameter Update Derivations
* Distance & Wheel Scaling:
  Given nominal distance D_true = 60.96 cm and measured average wheel degrees deg_avg = (deg_L + deg_R) / 2:
  R_actual = D_true / (deg_avg * (pi / 180))

  If the left sensor detects the line before the right sensor by distance offset delta_skew over sensor spacing W_sensor = 12.0 cm:
  Scale Ratio S_L / S_R = deg_R / deg_L

* Effective Track Width (CW & CCW):
  After a commanded 360° spin, the robot measures angular deviation delta_theta against the ground-truth line intersection:
  W_effective = W_nominal * (360.0° / (360.0° + delta_theta))

---

## 7. Coordinate Frames, Grid Intersections & Non-Empty Table Routing

### 7.1 Intersections vs. Tile Centers
* The Grid Lines: Black grid lines define the boundaries of each 30.48 x 30.48 cm coordinate cell.
* The Houses (Cardboard Boxes): Each 30.48 x 30.48 x 30.48 cm box sits inside a grid square (between 4 grid lines).
* The Street: The street corridor is a contiguous sequence of open grid squares. Its physical centerline runs through the tile centers:
  X_center = Gx * 30.48 + 15.24 cm
  Y_center = Gy * 30.48 + 15.24 cm

* Starting Pose at (0, 0): 
  The robot starts with its wheel axle midpoint stationed at grid line intersection (0, 0).
* Entering the Street at (s1, s2):
  The entrance is specified by two adjacent grid intersections s1 and s2. Because s1 and s2 span one grid tile edge, their midpoint is precisely the corridor centerline:
  P_entry = ((x_s1 + x_s2) / 2, (y_s1 + y_s2) / 2)

### 7.2 Non-Empty Space Safe Routing to Entrance (s1, s2)
> [!WARNING]
> Do not assume the space between (0, 0) and (s1, s2) is empty. Walled street sections or exterior houses may occupy grid cells outside the designated street segment.

Safe Approach Strategy:
1. From (0, 0), navigate orthogonally along grid axes (e.g., along row y=0 or column x=0) rather than attempting a blind diagonal traverse.
2. The forward ultrasonic sensor continuously monitors path clearance. If an unmapped obstacle/wall box is detected within < 25 cm on the approach corridor, the planner routes via the alternate open grid axis (Manhattan routing with obstacle avoidance).

---

## 8. Fast "No Dead Ends" Navigation & Real-Time Time Safeguards

### 8.1 High-Speed Continuous Corridor Traversal ("No Dead Ends")
Because the client specification guarantees no dead ends, the street corridor is a non-branching continuous channel with only straight segments and 90° bends.

Optimized Reactive State Machine:
1. Cruise Mode: Drive straight along the tile centerline at high cruise speed (325 deg/s ~ 12.0 cm/s) with active closed-loop heading hold. Dual downward color sensors square over intermediate tile lines on the fly without stopping.
2. Corner Lookahead Trigger: Continuously poll forward ultrasonic distance. When d_forward <= 25.0 cm:
   * Decelerate smoothly to crawl speed (50 deg/s).
   * Align wheel axle on the corner tile center.
   * Query open direction with a single check:
     * Ping Right: If d_right > 28.0 cm --> execute 90° Turn Right.
     * Else --> execute 90° Turn Left.
   * Resume full Cruise Mode immediately!
3. Mapping Speed Benefit: Eliminates stopped 3-way turret sweeps. Traverses a 15-tile street in under 35 seconds!

### 8.2 Real-Time Dynamic Time Budgeting & Overtime Safeguard
The competition imposes a strict 120-second hard cap per phase. Exceeding 120 seconds results in an immediate zero score.

```
                           DYNAMIC TIME WATCHDOG
  +-------------------------------------------------------------------------------+
  | Total Phase Time: T_MAX = 120.0 s                                             |
  | Hard Safety Margin: T_MARGIN = 15.0 s  -->  Abort Limit = 105.0 s             |
  +-------------------------------------------------------------------------------+
                                         |
                                         v
  At each tile arrival:
  1. Measure elapsed time: t_elapsed = time.time() - t_start
  2. Estimate return time:
     t_return = (Manhattan_Dist_to_Origin / V_cruise) + (N_turns * t_turn) + 5.0 s
  3. Safety Evaluation:
     IF (t_elapsed + t_return) >= (T_MAX - T_MARGIN):
         ENGAGE EMERGENCY RETURN TO ORIGIN
         Abort remaining exploration/sweeping;
         Execute high-speed shortest path back to (0, 0);
         Guarantee arrival at origin at t <= 112 s.
```
