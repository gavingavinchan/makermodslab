# 960 mm propeller: enlarged bench feasibility and underwater preview

Completed 2026-10-24 for goal B3a/B3b. The propeller is **3× larger in linear
geometry**, from 320 to 960 mm diameter. All four visible blades and 192 physical
collision cells scale together. The original arms and 52 mm cleaning disc are
unchanged. Original small-scene code and evidence remain in `propeller_cardinal`.

Start with [the bench overview](ready_front.png), [working detail](ready_detail.png),
[15-second physical demo](scripted_feasibility.mp4),
[underwater hero](underwater/underwater_hero.png),
[underwater close-up](underwater/underwater_detail.png) and
[15-second underwater preview](underwater/underwater_preview.mp4).
Full task/reproduction notes: [PROPELLER_LARGE_README.md](../../PROPELLER_LARGE_README.md).

## Physical run

The motion scene retains the original 1.30 × 1.10 m table, top Z = 0.760 m.
The hub at `[-0.420, -0.120, 1.290]` m puts the lowest blade **50 mm above the
table**. Temporary arm risers keep the working patch reachable; base locations
are `[0.220, 0.240, 1.092]` and `[-0.150, 0.240, 1.092]` m. The inward move of
the right base removes its elbow-limit conflict without shrinking the propeller.
The overhead camera is relocated and its obsolete pole removed; original
Maker-style wrist mounting and the full three-camera policy pipeline are kept.

The left jaws contact the **outer edge of the left blade**, at radii
**453.59–473.70 mm**, or **94.5–98.7%** of the 480 mm tip radius. The right hand
holds the free 117 g cleaning tool and scrubs that same blade. The shaft/hub is
fixed, so this hold does not demonstrate required antirotation load on a free
propeller.

The final initialized pregrasp has **zero contacts or penetrations**. Its left
jaw half-opening is 24 mm, wide enough for the enlarged 18 mm blade skin.
This is an explicit initialization, **not an executed approach or pickup**.
Jaws close gradually over 0.6 seconds and settle for a total of 2 seconds.
The 12-second scrub and final 1-second hold then use ordinary servo physics.
There are no welds, attachments, external forces, mocap drives or pose resets
in the physical trajectory. IK uses scratch poses and restores the simulation
before stepping.

From the final `check.json`, audited at all 12,000 scrub substeps:

- Left bilateral blade hold: **100%**.
- Right bilateral handle retention: **100%**.
- Abrasive pad/blade contact: **100%**.
- All three conditions simultaneously: **100%**.
- Actual pad radial travel: **34.14 mm peak-to-peak**; Y/Z travel
  11.29/16.62 mm as the pitched, curved surface is followed.
- Strict continuous-contact reversals: **4**, with 10 mm hysteresis.
  Radial-only reversal count: also **4**.
- Maximum unintended penetration: **0 mm**; no unwanted contact pairs.
- Maximum intended contact penetration: **0.772 mm**.
- Maximum pad normal force: **4.83 N**.
- Maximum scrub IK residual: **0.0000372 mm / 0.0000284°**.
  This numerical target-fit residual does not imply equivalent physical accuracy.
- Opening the right jaws after the clip lets the tool fall **559.51 mm**;
  weld count is **zero**.

**Torque limitation remains:** the left arm reaches its **5.00 Nm cap at 100%
of scrub physics substeps**. The right arm reaches 4.755 Nm maximum and has
zero cap occupancy. The simulation passes the contact/geometry feasibility
criteria, but has no demonstrated left-arm torque margin and is not hardware
ready. The prior dry-arm safety guidance still applies. These are simulation
actuator forces, not measured hardware loads.

## Underwater presentation

The presentation model removes the desk, temporary risers, camera pole and
bench shaft stand. A submerged service frame supports the same arm-base
transforms; procedural stern/shaft geometry contextualizes the stationary
bronze propeller. Blue-green fog, broad work lights, particles and seabed stones
provide the water setting. Disabling harsh shadow maps removes distracting
floor triangles and surface speckle from the preview.

The preview replays **the exact recorded qpos/qvel**, including both finger
pairs and the free tool. Every rendered state is compared against the bench
model's arm/tool world transforms. It runs **no additional dynamics**. This is
not a validated wet-dynamics model or a floating BlueROV2 model. It does not
claim buoyancy, drag, vehicle station keeping, abrasive spin, material removal,
cleaning success or Dream success. **No Dream calls and no hardware activity.**

## Verification and artifacts

- `geometry_check.json`: compiled collision extent 959.99998 mm and visual
  extent 959.99998 mm in both X and Z; 192 convex contact parts; 50.00001 mm
  table clearance; unchanged 52 mm pad; verified vendored arm source hashes;
  no sibling checkout required; clean pregrasp and actual ready contacts.
- `check.json` and `trajectory.jsonl`: final source hashes, versions, commands,
  actual states, contacts, forces, reversals and saturation.
- `physics_replay.npz`: 300 physical states sampled at 20 Hz, with free-tool pose.
- `underwater/replay_check.json`: exact replay archive SHA-256, identical
  arm/tool transforms, presentation-only declaration and clip metadata.
- `media_check.json`: full decode of both videos, **150 frames each**, 10 fps,
  15 seconds; bench 1600×940, underwater 1280×720; SHA-256 for each video.
- `neutral_*`, `pregrasp_*`, `ready_*`, `scrub_*`, `final_*`: actual scene and
  full policy frames. `*_top_sensor_full.png` preserves the complete 640×480
  sensor image in addition to the unchanged 640×360 policy image pipeline.

The code uses repo-local procedural geometry and the previously vendored arm
meshes. It does not load assets from the cinematic sibling project. All five
new modules compile; final geometry/contact checks and complete video decoding
passed. Only task-owned new source, readme and artifacts are added. The parent
owns the goal-document update and both repositories' commit/push actions.
