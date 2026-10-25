# Four-blade propeller: environment and scripted feasibility

**Reviewable scene and physical scrub demo complete. Zero Dream calls.**
The original table remains, with a curved pitched bronze **320 mm four-blade
propeller** on a fixed horizontal shaft. The left arm pinches the outer edge of
the screen-left blade; the right arm presses and translates a circular abrasive
pad on that same blade. This is a scripted feasibility demonstration, not a
policy result or simulated dirt removal.

Review: [neutral front](neutral_front.png), [ready front](ready_front.png),
[ready contact detail](ready_detail.png), [15-second clip](scripted_feasibility.mp4).
Policy images: [overhead](ready_top.png), [left wrist](ready_left.png),
[right wrist](ready_right.png). [Sampled video frames](video_review.jpg).

The selected blade is **blade 0 / world +X**. From the arms on +Y looking toward
-Y, that is screen-left. +Z is up. Tip axes are +X, +Z, -X, -Z, despite the
intermediate sweep. Center is `[-0.10, -0.12, 0.97]` m, table top Z = 0.760 m,
shaft height = 210 mm, bottom-blade clearance = 50 mm. The 52 mm abrasive pad
has a 32 × 70 mm handle and a total tool mass of 117 g.

Both arms use the original Viscous CAD, joint limits, 5 Nm arm limits, servo,
1 ms physics and Maker-style wrist-camera pipeline. This new task uses **440 mm
base spacing**, with bases `[+0.22, 0.24, 0.772]` and `[-0.22, 0.24, 0.772]` m.
The old task's 610 mm spacing and three-blade geometry remain unchanged.
The left jaw drive remains 10 N; the right is 40 N to prevent the cylindrical
tool pivoting/slipping. No installed packages or shared task files were changed.

Measured over the **12-second scrub segment**, sampled every physics substep:

- Both left jaws on the selected blade's outer edge: **100%**.
- Both right jaws on the handle: **100%**.
- Actual abrasive-pad contact on that same blade, with both holds simultaneous:
  **99.458%**.
- Actual radial pad travel: **26.14 mm peak-to-peak**;
  **3 radial reversals** with 10 mm hysteresis.
  The stricter fully continuous-contact metric records **2 reversals** and
  **97.7 mm tangential travel**. Brief
  contact losses reset that stricter count.
- Left contact radius: **135.1–160.0 mm**
  on the 160 mm radius blade, confirming an outer-edge hold.
- Maximum intended contact penetration: **0.774 mm**;
  no unintended contact pairs or penetration in the recorded segment/start.
- Maximum pad normal force: **21.69 N**; values are simulation
  contact forces, not calibrated abrasion loads.
- Maximum scrub IK error: **1.387 mm / 1.049°**.
  Both simultaneous initialization fits are below numerical micrometre scale.
- **Zero welds or attachments.** Opening the right jaws after the clip lets the
  free tool fall **231.4 mm**.

The clip starts from an **explicitly initialized pregrasp**. It does not show
pickup, blade acquisition or a collision-free path from the separate neutral
scene. Jaws close gradually, settle for two seconds, then the ordinary servos
execute two requested stroke cycles. There are no object teleports during the
clip; IK scratch configurations are restored before stepping physics.

The left arm stays at the **5 Nm actuator cap for 100% of scrub rows** and the
right reaches a cap for **0.42%**. The fixed blade
provides support through real contact. This does not establish free-propeller
load capacity, torque margin or hardware feasibility. The fixed shaft also
means the hold is a verified bilateral contact, not proof that it prevents an
otherwise free propeller rotating. Friction/compliance are engineering simulation
settings (pad sliding friction 0.30), and no abrasive spin or material removal
is modelled.

The first original-spacing layout was rejected for forearm/table contact and
load-induced loss of pad contact ([diagnostic](rejected_setup_check.json)).
The final narrower layout and downward tool approach resolve the geometric
collision. A gradual 0.6-second jaw closure removes the initial closing impact.

Validation passed: four watertight blade meshes, exact cardinal tips, 192 convex
blade contact cells, retained table/top camera attributes, three 640 × 360 policy
images, simultaneous ready contact, no welds, physical release, source hashes,
and MP4 decode (150 frames, 10 fps, 1600 × 940, 15 seconds).
[Machine-readable summary](check.json), [geometry/contact checks](geometry_check.json),
[video check](video_check.json), [per-row measurements](trajectory.jsonl).

Implementation and reproduction: [PROPELLER_CARDINAL_README.md](../../PROPELLER_CARDINAL_README.md).
The required arm XML/229 meshes are vendored with revision, hashes and original
notices, and validation succeeds with the legacy external arm path deliberately
invalidated. The parent-owned [reference brief](../../BRIEF_PROPELLER_BIMANUAL.md)
records the commercial cleaning references; the exact left-hold/right-scrub
coordination is Gavin's design, not a claim about those clips.
