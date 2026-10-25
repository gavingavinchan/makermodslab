# Enlarged propeller: bench motion and underwater replay

Goal B3a/B3b, implementing [Gavin's scaling/presentation brief](BRIEF_PROPELLER_LARGE.md).
The propeller is **960 mm tip diameter, three times the original 320 mm in all
linear blade dimensions**. The Viscous arms and **52 mm cleaning disc** retain
their original dimensions. “Remove the disk” was interpreted as removing the
**desk** for presentation; the abrasive tool is retained.

There are two explicitly different views of the **same robot/task/trajectory**:

1. **Temporary table motion scene:** original Dream table, relocated overhead
   camera, original Maker-style wrist cameras, two arm risers and fixed shaft
   stand. All feasibility measurements come from this MuJoCo dynamics run.
2. **Underwater presentation:** exact recorded arm joints, finger joints and
   free-tool poses replayed with a procedural vessel stern/shaft, submerged
   service frame, blue-green fog, seabed, work lights and particles. Desk,
   risers, camera pole and tabletop shaft stand are removed. This is a fixed
   submerged service rig, **not a model of a floating ROV**, and not an
   underwater dynamics or Dream result.

## Review artifacts

All new evidence is in [`runs/propeller_large`](runs/propeller_large/):

- [`ready_front.png`](runs/propeller_large/ready_front.png): full bench setup.
- [`ready_detail.png`](runs/propeller_large/ready_detail.png): real outer-edge
  hold and pad contact on the same left blade.
- [`scripted_feasibility.mp4`](runs/propeller_large/scripted_feasibility.mp4):
  15 seconds of servo-driven physics, with all three full policy frames.
- [`ready_top.png`](runs/propeller_large/ready_top.png),
  [`ready_left.png`](runs/propeller_large/ready_left.png),
  [`ready_right.png`](runs/propeller_large/ready_right.png): exact 640×360 policy
  images. `*_top_sensor_full.png` additionally preserves the 640×480 sensor
  render before the established policy pipeline's central 360-row crop.
- [`underwater/underwater_hero.png`](runs/propeller_large/underwater/underwater_hero.png),
  [`underwater_wide.png`](runs/propeller_large/underwater/underwater_wide.png),
  [`underwater_detail.png`](runs/propeller_large/underwater/underwater_detail.png):
  actual MuJoCo presentation renders, not generated concept images.
- [`underwater/underwater_preview.mp4`](runs/propeller_large/underwater/underwater_preview.mp4):
  short replay of the same measured motion; no new motion is invented for it.
- [`report.md`](runs/propeller_large/report.md), `check.json`, `geometry_check.json`,
  `trajectory.jsonl`, `physics_replay.npz` and `underwater/replay_check.json`:
  measurements, raw commands/states, replay provenance and verification.

The original [`propeller_cardinal`](PROPELLER_CARDINAL_README.md) code, procedural
assets and `runs/propeller_cardinal` evidence remain unchanged and reproducible.

## Layout and physical model

World +X is screen-left from the arms (+Y) looking toward -Y. Four blade axes
remain +X, +Z, -X and -Z. Blade 0 (+X) is the target. The new MJCF builder reuses
all repo-local cardinal blade meshes, explicitly applying `scale="3 3 3"` to
**both the four visible meshes and all 192 convex collision cells**. Hub and
nose dimensions are also scaled. Thickness grows from 6 to 18 mm; blade pitch
and normalized curvature are unchanged. No arm meshes, arm limits or tool
meshes are scaled. The compiled collision and visual world bounds are measured
independently in the geometry check.

The hub moves to `[-0.420, -0.120, 1.290]` m. It is 530 mm above the original
760 mm tabletop; the lowest blade has **50 mm clearance**. The table's footprint
and geometry are unchanged. Arms sit on substantial temporary risers at
`[+0.220, +0.240, 1.092]` and `[-0.150, +0.240, 1.092]` m (370 mm spacing).
This raises the working height 320 mm, while moving the hub right in the work
view keeps the outer blade reachable. The right base moves 70 mm inward to
avoid the elbow limit; shrinking the propeller was unnecessary. The overhead
camera is relocated to see the full enlarged task. The inherited camera pole
is removed from this layout; the overhead camera represents an external mount.

The left pregrasp targets normalized blade radius parameter `t=0.97` and closes
from an 24 mm jaw half-opening. Its measured contact lies in the outermost
10% of the 480 mm radius, not at the root. The right pad targets `t=0.80` and
commands two smooth sinusoidal cycles with amplitude `0.05` of the blade's
root-to-tip span (39.3 mm commanded radial peak-to-peak). The actual pad travel
is separately measured. The 117 g free tool, 52 mm pad, 10 N left / 40 N right
simulation gripper drive caps, friction and original 5 Nm arm caps are inherited
from the small scene; these are not wet-hardware calibrations.

## Limits and measurement contract

`neutral_*` is a separate neutral initialization. `pregrasp_*` is an explicitly
**initialized pregrasp**, not an executed pickup or approach. The tool is placed
between the right jaws once, then the jaws close over 0.6 seconds and settle to
2 seconds. All subsequent recorded motion advances ordinary servo control and
MuJoCo physics at 1 ms substeps with 20 Hz command rows. No weld, attachment,
external force, mocap drive or object pose setter drives the physical demo.
The hub is fixed, so the bilateral left hold does not prove required
antirotation torque against a freely rotating shaft.

The audit checks actual contacts at every 1 ms substep, including both left
jaws on blade 0, both right jaws on the handle, and the abrasive pad on blade 0.
It also records unwanted collisions, penetration, pad force, actual motion,
IK residual and **arm-cap occupancy at every substep**. The stricter reversal
metric resets on any 50 ms row with a contact break; radial-only reversals are
also reported. A separate release check opens the right jaws after the recorded
clip and verifies that the free tool falls.

The left arm remains at its **5 Nm simulation cap**; this is a torque-margin
limitation, even with good contact/IK results. The existing dry-hardware planning
caution still applies. Neither presentation nor contact success validates
hardware, cleaning, material removal, abrasive spin, buoyancy, hydrodynamic drag,
ROV stabilization or policy success. **Zero new Dream calls and no hardware use.**

## Reproduce

Use the existing Linux simulation environment and versions documented in
[the cardinal README](PROPELLER_CARDINAL_README.md); no package edits or sibling
repository are required. From `scripts/viscous-dream1`:

```sh
mkdir -p .cache/tmp
MUJOCO_GL=egl TMPDIR="$PWD/.cache/tmp" \
  "$HOME/dreamtrial/.venv/bin/python" propeller_large_demo.py
MUJOCO_GL=egl TMPDIR="$PWD/.cache/tmp" \
  "$HOME/dreamtrial/.venv/bin/python" propeller_large_check.py
MUJOCO_GL=egl TMPDIR="$PWD/.cache/tmp" \
  "$HOME/dreamtrial/.venv/bin/python" propeller_large_underwater.py
MUJOCO_GL=egl TMPDIR="$PWD/.cache/tmp" \
  "$HOME/dreamtrial/.venv/bin/python" propeller_large_media_check.py
```

`propeller_large_demo.py --no-render` omits the bench video but retains stills,
metrics and physics replay. `propeller_large_underwater.py --stills-only` renders
only the three underwater stills. Run the demo first: replay verifies identical
joint order and compares arm/tool world transforms against the bench model.
The underwater script writes recorded poses for rendering only, executes no
physics steps, and hashes the exact source replay archive in its report.

All additions are procedural code using already-vendored source meshes. The
cinematic sibling project was a visual reference only; no runtime paths or
assets from it are needed. Rendering copies to the Linux scratch directory are
individual files; the parent task owns repository commits/pushes and the goal
record in `docs/viscous-haptics.md`.
