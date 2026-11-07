# Sculpted shackle cradle and independent pin cup — goal B4

Gavin rejected the block-and-towers holder and asked for integrated Prusa MK3
zip-tie channels and a roughly 3 cm cup from which the robot picks up the pin.
This redesign uses a continuous crescent cradle blended into a low elliptical
foot. Two short curved passages follow the lower bow stock. The separate,
coordinated cup can be positioned by the environment thread. No adhesive,
tank attachment, robot motion or paid inference is involved.

**Selected CAD pose: holder 85°; cup 85°, tip 80 mm above the floor.**
The whole-arm kinematic gate selected bore midpoint **600 mm** ahead of J1 and
cup tip **565 mm** radially from J1, 15° toward sim +X. The original 575/500 mm
placement failed extraction/reach and water checks. **No CAD revision or reprint
is needed**: move the same two pieces. The recommended placement completed the
1,246-sample offline sequence; forces, friction, torque and physical trials
remain untested. See [the gate report](../shackle85_gate/README.md).

## Reproduce and coordinate frames

```sh
uv run --script scripts/viscous-dream1/shackle_tied_holder.py --tilt 85 --cup-tilt 85 --pin-tip-height 80 --cup-tip-forward 565 --cup-azimuth 15 --shackle-forward 600
uv run --script scripts/viscous-dream1/shackle_fixture_clearance.py
```

Dependencies are pinned in the script. No hardware, billed calls, environment
edits or commits are made by this generator. `--no-render` skips previews;
`--output-dir` redirects outputs; `--cup-foot-offset` changes the foot centre
relative to the tip's floor projection (default -48 mm toward the arm).

**Current exports are in `tilt_85deg/`:**

- `shackle_tied_holder.stl` / `.3mf` and `pin_cup.stl` / `.3mf`:
  millimetres, each one connected print, flat foot at Z=0.
- `shackle_tied_holder_body_frame.obj` and `zip_tie_envelope_body_frame.obj`:
  metres, original shackle body frame. Apply the body pose once.
- `pin_cup_floor_frame.obj`, `pin_seated_in_cup_floor_frame.obj`,
  `pin_pickup_corridor_floor_frame.obj`, `pin_pickup_wide_envelope_floor_frame.obj`:
  metres, **local** cup floor frame. +X is radially away from J1, +Y across,
  +Z up. Origin is the floor projection of the pin tip, NOT foot centre.
- `cup_checks.json`: exact `pin_tip_floor_frame_mm`, extraction axis,
  tip-to-floor matrix, foot-centre offset, dimensions and checks.
- `holder_checks.json`: unchanged holder geometry checks and body transform.
- `cup_with_pin.png`, `cup_side.png`, `cup_holder_layout.png`, plus holder
  `assembled.png`, `holder_only.png`, `side.png`, `tie_section.png`.

The local tip is `[0,0,80]` mm. Rotate the local cup frame +15° about Z and
translate its origin to `[565*cos(15°),565*sin(15°),0]` mm in (s,w,h).
This places the tip at approximately `[545.748,146.233,80]` mm.
JSON includes a **suggested** placement transform and the generator checks and
renders that placement; **no globally placed scene mesh is exported**. Parent
owns the independent global scene and gripper audit. Holder print frame retains
bore midpoint s=0; its OBJ remains in original body frame as before.

`tilt_75deg/` preserves the previously committed, rejected first-form review
files, not the selected print. Unselected intermediate exports were discarded;
the current source can regenerate other angles. Round-2 evidence is at 85°.

## Shape and routing

The holder is **146 × 112 × 32.19 mm** at 85°. A tapered elliptical foot blends
into both sides of a curved, open cradle, with a relief reaching through the
foot so the steel's lowest bow point rests directly on the tank floor. Added
bow height is **zero**. Nominal forged-stock allowance is **0.7 mm**; the crown
curve remains estimated and must be dry-fitted to the real shackle.

Two stations, ±55° around the lowest crown point, each have a **6.2 mm axial ×
2.8 mm radial passage**, with a **14.5 mm centreline bend radius**, following
a 190° covered stock return (210° cutter sweep), continuing tangentially to accessible
mouths. They wrap the local ~19.41 mm stock, not the entire shackle. Nominal
radial plastic on each side of the passage is about **2.6 mm** in its 12 mm-wide
continuous shell before edge
rounding; this is a geometric allowance, not a strength certification.

Assumed ties are up to **4.8 × 1.5 mm**. The illustrated loose loop is ~91 mm;
use a sufficiently long tie (e.g. 150 mm) for its head and trimming tail. Feed
before seating the steel. The orange mesh deliberately includes clearance
around the stock and **7 × 7 × 5 mm locking-head bounds** at the +W mouths.
The head sits in an 8 × 8 × 6 mm clearance pocket.
These are envelope assumptions; confirm the actual ties and put trimmed tails
away from the grasp corridor. All exposed convex rims receive nominal **R0.5**
treatment, with 0.5 mm bed-edge chamfers. Tangential slot mouths are eased by
the same rounding operation.

### Primary references, reference-only licensing

Inspected Prusa MK3 revision `902885a41340eea19c19c10d36dfc4af8e209d47`:

- [y-rod-holder.scad](https://github.com/prusa3d/Original-Prusa-i3/blob/902885a41340eea19c19c10d36dfc4af8e209d47/Printed-Parts/scad/y-rod-holder.scad):
  3.2 mm-wide, 2 mm-thick short bottom passage, curved corners (2/4 mm radii)
  and angled mouths illustrate a feedable continuous route.
- [bearing.scad](https://github.com/prusa3d/Original-Prusa-i3/blob/902885a41340eea19c19c10d36dfc4af8e209d47/Printed-Parts/scad/bearing.scad):
  annular 9–11.5 mm-radius channels, 4 mm wide, follow the retained stock.
- [cable-holder.scad](https://github.com/prusa3d/Original-Prusa-i3/blob/902885a41340eea19c19c10d36dfc4af8e209d47/Printed-Parts/scad/cable-holder.scad):
  paired mouths and a 4.6–6 mm-radius return guide a tie around cables.

Those sources are GPLv3. No Prusa code or mesh is copied or distributed here.
Our original annular sweep and tangential-mouth implementation uses the routing
principle with independently selected dimensions for this shackle.

## Raised cup and pickup

“About 3 cm” now means **30 mm axial tube length**, not floor-to-top height.
The raised pedestal is additional height required by the underwater wrist
layout. The cylindrical bore is 21.5 mm, the lead-in 23.5 mm, and the axial
backstop 3 mm thick: **27 mm tip-to-mouth insertion**. The tube points 85°
from vertical, head toward the arm and slightly upward. Pin extraction is
along its axis, not vertical. The actual threaded pin mesh is used in checks.

The 96 × 76 mm elliptical foot is biased 48 mm toward the arm, beneath the
pin's overhanging mass. A tapered oval pedestal blends into the tube; a curved
clearance relief under the fingers prevents the pedestal crowding the pickup.
Convex edges receive nominal R0.5 rounding, with a flat chamfered bed retained.
The part is one print, without hardware, glue, ballast or separate pieces.

The 6 mm backstop drain connects the interior to the rear; a 5 mm downward
bore near the bottom of the cup joins transverse floor outlets. These are
finite-volume material cuts, not just lines or zero-thickness openings.
The loose thread/bore clearance and rear hole vent the seated pin; physical
water flow and any residual puddle still require a print trial.

The cup pickup grip/TCP reference is **68 mm from the pin tip**. Fingertips
extend 34.3 mm toward the tip, so their foremost plane is at 33.7 mm,
**6.7 mm beyond the cup lip**. The generator checks the legacy 60 mm diameter
corridor and additionally a conservative 42 × 162 mm finger cross-section
(fully open) over axial distances 33.7–118 mm. A radius sqrt(80²+4²) mm
cylinder over 118–178 mm bounds the palm's complete roll sweep, including
0–120°. These solids deliberately bound the supplied envelope; they do not
replace actual gripper meshes, approach planning or collision dynamics.

## Verification and limits

Current `tilt_85deg` reports are the round-2 evidence. The holder checks retain
single watertight connected mesh, millimetre STL/3MF round trips, zero steel
intersection, nine vertical shackle drop-in samples, 33 pin-axis samples,
complete ties/heads clear of steel and holder, 28 material probes at the tie
channel walls, and the 60 × 300 mm head-side corridor clear of holder/ties.
The wide insertion/regrip palm and finger checks are in the parent's independent
`fixture_clearance_checks.json`, described below.

Cup checks assert one watertight outward-wound connected mesh, flat bed,
STL/3MF and metre OBJ bounds round trips, actual seated pin and **61 axial
extraction samples over 120 mm**, clear narrow and wide pickup envelopes,
and continuous drain cores inset 0.05 mm after edge rounding. The nominal
drain cutter has separately reported triangle-chord residual from 0.002 mm
mesh simplification, bounded by tool surface area × 0.002 mm; the inset flow
core must have zero overlap. Dry/wet gravity calculations use
the actual convex bed polygon, with and without the pin, modeled steel density
7.9 g/cm³ and 40%-material PETG at 1.27 g/cm³. Wet calculations assume flooded
print voids and displace only material volume. They establish gravity support
only, **not** robot-load capacity, sliding resistance or print strength.

At the selected pose the cup print bounds are approximately 100.15 × 76 ×
96.24 mm (tube and pedestal included); its nominal foot is 96 × 76 mm.
Estimated 40%-material PETG mass is 64.4 g and modeled pin mass is 240.7 g.
Cup support margins are 28.30 mm empty, 37.48 mm dry with pin and 37.26 mm
flooded with pin. Minimum restoring moments are **0.00380 N m flooded empty**
and **0.08186 N m flooded with pin**. The empty cup can therefore be displaced
by quite small external loads despite being gravity-stable.

The selected off-line placement yields zero cup/holder, cup/shackle and
pin/holder intersection in this generator's check. The separate whole-arm gate
also passes the recommended placement. Print geometry and all local mesh frames
are unchanged from `0de8facd`; only the placed meshes and metadata move.

Print feet flat. The tilted tube roof and drain require slicer inspection;
PETG, 0.2 mm layers, five walls and 40% infill are starting assumptions, not a
validated profile. No physical print, water-flow or load trial has been done.
The gate resolved the earlier seated clear-ear/fingertip collision by changing
the **head grip**, not the part: TCP **120.15 mm from the pin tip**, fingertips
2 mm above the collar, gripping with the distal ~22 mm of the fingers. Do not
reuse the nominal mid-head grip (96.9 mm from the tip), which intersects the
collar and clear ear. Tightening uses **90° strokes and 18 mm-per-side resets**.
The tab's flats must face the jaws in the cup. The strength of this distal
grip on the real fingers still needs a physical trial.

The kinematic sequence passes water, wall, elbow, limits and self-collision:
minimum water margin **15.8 mm**, wall **3.4 mm**, elbow **110 mm** ahead of J1,
joint-limit margin **10.2°**, cup/finger clearance **6.67 mm**. This is not a
dynamic or force qualification. The overhead camera remains provisional.

## Independent fixture placement and hand envelope review

Run `uv run --script scripts/viscous-dream1/shackle_fixture_clearance.py` from
the repository root after generating the selected CAD. It places the exact
holder, tie/head envelopes and cup at the recommended 600 mm / 565 mm / 15°
layout, then checks 121 static roll angles from 0–120°, four jaw openings and
six task poses using the corrected head grip. Palm bounds include ±80 mm jaw
width, ±4 mm thickness and
53.4–110 mm behind TCP. The published finger bounds are represented by two
conservative boxes, with **0.5 mm extra padding** around every hand dimension.

**Result: clear with explicit motion constraints**, across 26,136 tested
hand/fixture pairs (1,224 Boolean intersection tests after bounding-box
filtering). Required operating states have zero overlaps:

- Cup pickup/extraction at **roll 0°**, before reorienting away from the cup.
- Insertion start and high head-regrip pose: all checked rolls and openings.
- Threads-catching and seated poses: **at most 26 mm opening per side**
  through rolls 0–120°. Open enough to release the pin; avoid full splay there.

The unrestricted stress screen **does find 477 conservative overlaps**: full
52.4125 mm-per-side opening near the seated holder, and rolling while the
fingers still surround the cup pedestal. These are retained in the evidence,
not counted as passing. Some boxes enclose voids in the actual finger CAD;
they are deliberate bounds, not proof that every flagged state collides.
The motion planner must honour the constraints or establish clearance with
the actual meshes. This box screen excludes held-pin/shackle contact and the rest
of the arm; the separate gate checks those meshes across the full sequence.

`layout_manifest.json` records exact placement and file hashes. The three
`layout_*_plane_frame.obj` files are already positioned in **metres** in the
shared plane frame: +S forward, +W toward sim +X, +H up from tank floor. To
put those files into sim world coordinates, apply once:
`world = (0.305 + W, 0.240 - S, 0.760 + H)`.
Do not also apply the local holder or cup transforms to these placed meshes.

The Mac did not respond to the milestone SSH check; Linux exports and git
history are the handoff until that checkout can be synchronised. The local
arm-model and LeRobot checkouts have no changes or commits ahead of upstream.
