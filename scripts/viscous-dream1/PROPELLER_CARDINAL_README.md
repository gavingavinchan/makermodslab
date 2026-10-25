# Four-blade bimanual propeller environment

This is the simulation-only B3a environment-first revisit described in
[BRIEF_PROPELLER_BIMANUAL.md](BRIEF_PROPELLER_BIMANUAL.md). Task name:
`propeller_cardinal`. The existing `propeller` three-blade task is unchanged.
There is no Dream client construction, live mode, ledger access, hardware access,
or service control in these entrypoints.

## Reproduce

From `scripts/viscous-dream1`, using the established Linux simulation venv:

```sh
mkdir -p .cache/tmp
MUJOCO_GL=egl TMPDIR="$PWD/.cache/tmp" \
  "$HOME/dreamtrial/.venv/bin/python" propeller_cardinal_demo.py
MUJOCO_GL=egl TMPDIR="$PWD/.cache/tmp" \
  "$HOME/dreamtrial/.venv/bin/python" propeller_cardinal_check.py
```

`--no-render` omits MP4 encoding, but still writes inspection/policy stills.
`propeller_cardinal_scene.generate_assets()` regenerates all four blade visuals
and 192 convex contact cells locally with NumPy/Trimesh. No downloaded propeller
model is required. The generator's dimensions and pitch are the source of truth;
after changing them, explicitly regenerate before running.

The checked environment used Python 3.12, `dreamlabs==1.0.1`,
`dreamlabs_client==1.0.3`, `mujoco==3.16.0`, `numpy==2.2.6`, `scipy==1.18.1`,
`trimesh==5.1.1`, `imageio==2.38.1`, `imageio-ffmpeg==0.6.0`, `Pillow==13.0.0`.
These dependencies include Dream's original table, textures, lighting and camera
builder. No installed package is edited. No Dream credentials are needed.

## Coordinates and dimensions

World +Z is up. The arms sit on the +Y side and work toward -Y. Thus **world +X
is screen-left** in the front work view. Blade 0 points +X and is the selected
LEFT blade. Blade axes are exactly +X, +Z, -X, -Z, in that order; sweep occurs
between root and tip and does not rotate those four tip axes.

- Propeller: 320 mm tip diameter, 6 mm blade skin thickness, curved camber and
  sweep, 30° root pitch tapering to 16° at the tip. Bronze visual mesh, separately
  decomposed convex physical contacts. Center `[-0.10, -0.12, 0.97]` metres.
- The face is vertical in X/Z, with a horizontal Y shaft. A 68 mm hub is fixed
  to a steel stand on the original 1.30 × 1.10 m table (top Z = 0.760 m).
  Axis height above the table is 210 mm, and the bottom blade clears it by 50 mm.
- This named task moves the arm bases to `[+0.22, 0.24, 0.772]` and
  `[-0.22, 0.24, 0.772]` m: **440 mm separation**, versus B0's 610 mm.
  The closer spacing makes the shared blade reachable. Original arm joint
  limits, 5 Nm arm actuator limits, gravity compensation, 1 ms physics steps,
  20 Hz commands and all three policy cameras are reused.
- Tool: 52 mm diameter circular abrasive pad, a 32 × 70 mm cylindrical handle,
  short metal neck and backing disc, total 117 g. The handle is physically
  squeezed by the right jaws. The pad is a compliant ellipsoid contact envelope;
  this is a translating scrub, with no powered spin or material-removal model.

The left gripper retains the baseline 10 N generalized drive cap. Only the
right gripper uses 40 N in this task, below the existing flange tooling's
160 N ideal CAD-linkage estimate. This is a simulation setting, not a calibrated
hardware force. The pad's sliding friction is 0.30, explicitly priority 1 to
avoid inheriting the blade's higher friction. These contact values are not
measured wet-abrasive properties.

## Start and measurement contract

`neutral_*` shows the table environment with arms at zero and tool on the table.
`pregrasp_*` is a **separately initialized pose**, not an executed approach or
pickup. Local least-squares IK is reused from the flange tooling, without
constructing its native Dream client. The tool is initialized between the right
jaws once; the left jaws straddle the outer blade edge. The jaws close gradually
for 0.6 seconds, then settle. `ready_*` is captured at 2 seconds of physics.

All following movement uses the ordinary actuator servo and MuJoCo dynamics.
There are **no welds, attachments, mocap drives, external forces or object pose
setters during the recorded motion**. IK scratch evaluations restore the
simulation configuration before each physics step. The hub is intentionally
fixed: the left hold verifies bilateral outer-edge contact, not an independently
necessary antirotation load or a free-propeller grasp.

Every 1 ms substep audits actual contact pairs. A hold needs both jaw bodies
contacting the selected blade or tool handle simultaneously. Pad contact must
be the actual abrasive geom against blade 0; proximity does not count.
Unwanted arm/table, arm/arm, backing/blade and other-blade contacts are reported.
The trajectory records commands, actual joints, pad/TCP positions, forces,
penetration and contact counts for every 50 ms row.

`radial_motion_reversals_10mm` measures the actual pad's radial travel, with
10 mm reversal hysteresis. `scrub_reversals` is stricter: it uses projected
actual motion and resets whenever any of that row's 50 substeps lacks pad
contact. Consequently brief contact breaks can split one physical stroke
into separate uninterrupted bouts. Both counts and raw duty fractions are kept.
Feasibility requires at least two such strict reversals, >90% simultaneous
contact, <1 mm unwanted penetration, <2 mm IK position error and valid start
poses. It does not mean Dream success, cleaning success or hardware readiness.
A separate post-clip release check opens the right jaws and verifies the free
tool falls, rather than remaining attached.

## Self-contained arm source

`assets/propeller_cardinal/arm/` contains the source arm XML and **only its 229
referenced meshes**, about 20.9 MB, copied unchanged from
`gavingavinchan/viscous-arm-simulation` commit
`836bb0028b692d491f6485455841896358dde5a0`. The manifest preserves SHA-256 hashes;
`THIRD_PARTY_NOTICES.md` is copied verbatim. The source states that the Viscous
CAD was supplied by Gavin with permission to publish, and that no additional
project-wide license was selected; this copy does not invent a broader license.
The new implementation code is Apache-2.0 under the MakerMods copyright.

The new builder resolves mesh files beside itself. The geometry check
intentionally points the legacy `cross_scenes.SCENE` to a nonexistent file to
verify that no separate `~/Documents/viscous-arm-model` checkout is needed.
The existing `wrist_mount()` is pure numeric mounting data; it does not load
Maker/Metal assets. Existing B0 renderer/servo methods and local IK are reused,
without modifying any shared baseline or concurrent flange task.

See [the rendered report](runs/propeller_cardinal/report.md),
[check.json](runs/propeller_cardinal/check.json) and
[geometry_check.json](runs/propeller_cardinal/geometry_check.json).
