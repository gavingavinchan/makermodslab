<!-- Copyright 2026 MakerMods. All rights reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
-->

# LANDEN 75 aquarium: waterproof Viscous arm environment

Offline model for `viscous_waterproof_01`, supporting haptics goals A1a/A3 and
aquarium sim-to-real goal B4. Gavin (2026-11-07): "create a new environment for
the aquarium, aka something like urdf for simulations ... so that you know the
limits of the aquarium walls. we may or may not need to put up real time
boundary limits to the follower arm. lets see".

`tank.json` is the geometry source. `aquarium_scene.xml` and `previews/*.png`
are generated artifacts. `offline-validation.json` records the offline results.
The script imports no Lab, driver, serial or CAN code and reads no user state.
No real-time boundary is installed or enabled by this work.

## Numbers and coordinate conventions

All JSON distances are **mm**, in the repo URDF's **base_link frame**; MJCF uses
metres. Every parameter carries its source and status. No tank placement has
been tape measured. The arm angle readback was supplied by Gavin; this tool
never reads the arm.

| Parameter | Current value | Provenance/status |
|---|---:|---|
| Outer length × width × height | 750 × 450 × 450 mm | LANDEN Amazon listing supplied in brief; nominal estimate |
| Uniform glass thickness | 8 mm | Unconfirmed third-party LANDEN 75H listing; estimate |
| Near inner face (+x, behind J1) | x = +40 mm | Prior base-back-edge design assumption; estimate |
| Far inner face (−x, arm reach direction) | x = −694 mm | 40 − (750 − 16); derived estimate |
| Left/right inner faces | y = +217 / −217 mm | J1 centred in 434 mm inner width; estimate |
| J1 offset from inner width midpoint | 0 mm | Estimate; positive means J1 toward the +y face |
| Base orientation relative to box | roll/pitch/yaw = 0/0/0° | Estimate; current implementation rejects nonzero rotations |
| Floor inner surface | z = 0 mm | Lowest base collision-mesh z, assuming direct glass support |
| Mat/spacer height | 0 mm | Estimate; a spacer of height h gives floor z = mesh minimum − h |
| Glass rim | z = 442 mm | Floor + outer height − bottom glass thickness |
| Optional water height | 283 mm above floor | Design in `docs/shackle-sim2real.md`; not filled; `null` disables |
| Wall and rim margins | 30 mm each | Follow-up brief design target; moving links only |
| Floor margin | 0 mm | Follow-up brief: moving geometry must not go below glass floor |

Inner length/width are outer minus **two** panes: 734 × 434 mm. The requested
open-top floor-to-rim height subtracts **one** bottom pane, hence 442 mm
(outer-minus-two-glass nominal height would be 434 mm). There is **no lid**.
The six half-space clearances bound the inner box; `rim` reports upper
headroom and is not an overhead physical collision surface. `water_line`
reports headroom below the proposed water surface, for a fully submerged arm;
it is informational and excluded from the default glass sweep.

J1–J6 use `configs/viscous/model/motor_to_urdf.json`:
**URDF angle = sign × motor_deg + offset_deg**, then convert to radians.
J5 wrist_yaw is −1 (Gavin's top-down photo inference 2026-11-07; hand test pending).
J1–J4 are +1 and photo-consistent in the supplied side view; J6 +1 is assumed,
unverified. All offsets are 0°; a photo match does not verify isolated motor signs.
Shoulder_pan retains its URDF −Z axis; motor angles are never wrapped.
The gravity model (`makermodslab/arm_gravity.py`) still uses direct mapping and needs the same sign check.

Gripper uses an **approximate** linear map: 0° closed, −115° → 52.4 mm displacement
per finger, clamped to [0, 52.4] mm. Thus readback −117.9° is fully open.
The CAD slide limit is 52.4125 mm. The geometry is the repo's CAD collision
model, not a new measurement of waterproof seals, wiring or other hardware.

## What Gavin must measure

Use the **J1 axis**, not the edge of the base plate, as the horizontal datum:

1. J1 axis to the **near short-wall inner glass face**, perpendicular distance
   in mm. This replaces `placement.near_inner_x_mm` (positive +x).
2. J1 axis to **each long-wall inner face**, separately. Mark which physical
   front/back face corresponds to base +y (called left) and −y (called right).
   If those distances are L and R, inner width = L + R and
   `j1_y_from_inner_midline_mm = (R − L)/2`. Confirm the arm's long direction
   is perpendicular to the short wall; measure base yaw if not aligned.
3. Glass thickness, including whether bottom and side panes have the same
   thickness. Current schema assumes uniform thickness; split pane thicknesses
   in the schema/code if they differ. Confirm actual outer length/width/height
   (or measure J1 to the far inner face to cross-check inner length).
4. Does the base plate sit **directly on the inner glass floor**, or on a
   mat/spacer? Measure spacer height under the lowest base collision surface.
   Set spacer height h and `floor_z_mm = base mesh minimum − h`.
5. The black rim-clamped **support tower footprint and height** near the base:
   measure its x/y extents relative to J1, lowest/highest z relative to the
   inner floor, and any clamp overhang into the tank. Record photos/handedness
   so it can be added as an obstacle; it is currently absent.
6. When filled, measure the water height above the inner glass floor.

For each replaced parameter set `status: "measured"` and record date, datum
and method in `source`. Recompute `derived_inner_bounds_mm` and `rim_z_mm`;
`load_tank` rejects stale derived values. Regenerate scene/previews and rerun
tests/clearance/sweeps after changing measurements. For a base yaw/tilt, extend
the plane transforms first: silently treating rotated walls as axis-aligned
would be incorrect.

## Offline commands

Run from repository root. Core commands need only the existing repo Python
and numpy; tests additionally use its installed Pinocchio. `-o` writes JSON
results; sweep also prints a Markdown table. Flags override the selected
reference pose; `--angles` accepts a complete seven-angle JSON object or file.
`clearance` returns `moving_links`, an informational `static_mount`, and moving
margin `violations`. Floor is reported with its own margin; the base is never a
violation. `sweep --margin-mm` overrides only walls/rim; `--floor-margin-mm`
independently overrides the floor. Defaults come from `surface_margins_mm` in
`tank.json`, each with source/status. `--include-water` optionally applies the
rim margin to water headroom; water is otherwise informational.

```bash
.venv/bin/python scripts/viscous_aquarium_env.py clearance --pose read -o /tmp/aquarium-read.json
.venv/bin/python scripts/viscous_aquarium_env.py clearance --pose folded -o /tmp/aquarium-folded.json
.venv/bin/python scripts/viscous_aquarium_env.py clearance --shoulder-pan=182.02 --gripper=-117.9
.venv/bin/python scripts/viscous_aquarium_env.py sweep -o /tmp/aquarium-sweep.json
.venv/bin/python scripts/viscous_aquarium_env.py sweep --margin-mm 0 -o /tmp/aquarium-zero-margin.json
.venv/bin/python -m unittest discover -s tests -p test_viscous_aquarium_env.py -v
```

Importable, load-once interface (no shared mutable FK workspace):

```python
from scripts.viscous_aquarium_env import load_tank, load_model, clearance
tank = load_tank()
model = load_model(tank)
pose = tank["reference_poses_motor_deg"]["read"]["value"]
distances = clearance(model, tank, pose)  # moving links, surface -> {distance_mm, link}
mount = clearance(model, tank, pose, static_mount=True)  # info only, never a violation
```

Only scene generation/check/render use the separate MuJoCo environment:

```bash
~/dreamtrial/.venv/bin/python scripts/viscous_aquarium_env.py scene -o /tmp/aquarium-scene.json
MUJOCO_GL=egl ~/dreamtrial/.venv/bin/python scripts/viscous_aquarium_env.py render -o /tmp/aquarium-render.json
```

MuJoCo is **not** added to Lab's dependencies. MJCF copies URDF joint origins,
axes, inertias, collision geometry and CAD limits, plus the gripper mimic
constraint; mesh paths are relative to the scene. Moving-link masks 1/2 and
glass masks 2/1 enable moving-arm/glass contacts and disable arm self contacts.
Static base_link geoms explicitly have masks 0/0. The read/folded keyframes
use the same sign map, and scene checks verify saved qpos before mj_forward.
Water is a visual-only finite plane (thin box). The scene's read/folded
keyframes are kinematic snapshots; **mj_forward only**, no dynamics/actuation
validation. CAD joint limits differ from the waterproof motor soft limits;
these snapshots may exceed CAD limits. Sweep uses the committed waterproof
`evidence/lab-hardware-profile.json` **follower.joint_limits**, without
substituting the rounded calibration ranges or reading installed state.

## Results and interpretation (2026-11-07 follow-up)

Gavin: "seems like the j5 is tilted the wrong way". Applying the provisional
J5 −1 sign gives read tool0 **(−520.150, +101.963, 42.820) mm**; folded
**(207.979, 82.008, 148.941) mm**. Independently assembling the mapped joint
vector in Pinocchio 3.7.0 confirms both (numpy disagreement <2e−13 mm).
For comparison the direct-map read was (−526.041, −65.063, 42.820) mm.
MuJoCo 3.16.0 loads 8 qpos/175 geoms, saved mapped keyframes agree with numpy
FK within 3e−10 mm, and reports **0 read / 28 folded near-pane contacts**.
Base contacts are disabled. Deepest folded pane contact is −32.022 mm;
half-space clearance can be worse when a link is already beyond a finite pane.

Checks select **all descendants of shoulder_pan**, including **yaw_link**,
fixed tool links and both fingers. Static base_link is informational only.
Floor uses its own **0 mm** margin, separately from the **30 mm** walls/rim.
Positive means inside/below rim; negative means outside/below floor.

| Surface | Margin mm | Read mm / closest link | Folded mm / closest link |
|---|---:|---|---|
| near | 30 | 8.979 / yaw_link | -201.280 / gripper_right |
| far | 30 | 116.580 / gripper_left | 513.697 / upper_arm_link |
| left | 30 | 50.564 / gripper_right | 78.974 / wrist_roll_link |
| right | 30 | 178.642 / upper_arm_link | 99.975 / forearm_link |
| floor | 0 | 4.551 / wrist_yaw_link | 57.400 / yaw_link |
| rim | 30 | 301.038 / yaw_link | 173.206 / wrist_pitch_link |
| water_line | info only | 142.038 / yaw_link | 14.206 / wrist_pitch_link |

Both poses violate only the near **margin**. The read pose has no glass
penetration; its moving yaw_link near distance is **8.979 mm**, short of 30 mm
by **21.021 mm**. Folded penetrates the near half-space by **201.280 mm**.
Static mount distances, identical at both poses (mm): near 0, far 654,
left/right 142, floor 0, rim 382, water 223. These never enter violations.

**The default sweep still has no passing samples for J1–J6**, even after
removing the static base and separating floor margin. The moving yaw_link is
too close to the assumed near face. Near fails at every sampled angle for
each joint (J1: 316/316; J2: 199/199; J3: 250/250; J4: 206/206;
J5: 176/176; J6: 307/307). No moving link is silently exempted.

| Joint | Soft limits (motor deg) | Passing samples (motor deg) | Contiguous around read (motor deg) |
|---|---:|---:|---:|
| J1 shoulder_pan | -126.85 to 187.53 | none | none |
| J2 shoulder_lift | -199.45 to -2.81 | none | none |
| J3 elbow_flex | 2.86 to 250.04 | none | none |
| J4 wrist_flex | -122.27 to 82.10 | none | none |
| J5 wrist_yaw | -86.06 to 87.32 | none | none |
| J6 wrist_roll | -154.99 to 150.00 | none | none |

A separate **diagnostic** sweep with walls/rim 0 mm, floor still 0 mm gives:

| Joint | Soft limits (motor deg) | Passing samples (motor deg) | Contiguous around read (motor deg) |
|---|---:|---:|---:|
| J1 shoulder_pan | -126.85 to 187.53 | 157.02 to 187.53 | 157.02 to 187.53 |
| J2 shoulder_lift | -199.45 to -2.81 | -199.45 to -161.83 | none |
| J3 elbow_flex | 2.86 to 250.04 | 170.96 to 235.96 | 170.96 to 235.96 |
| J4 wrist_flex | -122.27 to 82.10 | 38.42 to 82.10 | 38.42 to 82.10 |
| J5 wrist_yaw | -86.06 to 87.32 | -54.78 to 87.32 | -54.78 to 87.32 |
| J6 wrist_roll | -154.99 to 150.00 | -14.46 to 16.54 | -14.46 to 16.54 |

Samples use motor degrees and each read angle's 1° grid plus exact soft-limit
endpoints. The contiguous range is the passing sampled component containing
the read angle, not another disconnected island. J2 read −201.83° is below
its −199.4466° profile limit, so even its diagnostic in-limit samples cannot
contain read. Gripper −117.9° is also below −115.1072°. Other joints stay at
read even when outside their limits; none of these tables grants hardware
permission or proves a continuous swept-volume path.

Read moving-clearance benchmark, 1,000 calls: **0.435 ms median / 0.438 ms p95 /
0.448 ms max**; folded **0.439 / 0.445 / 0.454 ms**. Loading and the separate
static-mount report are excluded. Every distinct vertex is retained: **9,059
moving + 565 static = 9,624**; no geometric decimation. Observed timing meets
the 2 ms target, without a hard deadline guarantee or deployed 100 Hz guard.

17 offline tests pass, including independent Pinocchio all-link transforms,
map signs/offsets, static separation, independent floor margin, disconnected
sweep intervals and saved scene masks/keyframes. Ruff check/format pass.
`tank.json`, the sign map, scene, script, tests and previews are fingerprinted
in `offline-validation.json` alongside raw results and test output.

Previews regenerated with the mapped read keyframe: `previews/read_top.png`,
`read_side.png`, `read_three_quarter.png` (each under 1 MB, visually reviewed).
Glass outlines mark inner faces. Placement and glass thickness are still
estimates; no new measurement or hand sign test was performed. Tower, wiring,
fixtures, self-collisions, water forces and dynamics remain outside this model.
No hardware, service/session, installed-state, network or push operations.
