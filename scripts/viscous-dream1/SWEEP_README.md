# Sweep runner

Run from ~/dreamtrial/viscous_sim with MUJOCO_GL=egl, TMPDIR=$PWD/.cache/tmp, and ~/dreamtrial/.venv/bin/python.

- One configuration: `sweep.py --config tip_single_r24 --task both --episodes 6 --live`.
- Full staged experiment / resume: `sweep.py --stages --live`.
- No-call checks: `sweep.py --check`, `sweep_runner_check.py`, `sweep_baseline_check.py`.
- Regenerate report: `sweep_report.py`.
- Without --live, no inference is requested. Existing yam_baseline.py modes remain unchanged.

Configuration names combine tip, single, r24, table with underscores; B0 is all defaults.

B0 uses both arms, unchanged Maker-style wrist views, the original pad TCP, 48 executed rows, and no region.
`tip` uses viscous_arm_fingertip.json, a copy of the original with only tcp changed. At grip 0, visual_043 and
visual_042 are the closed finger blades. Transform each CAD mesh into wrist_roll_link coordinates, select the
furthest face along approach (100 micrometre tolerance), take each face bounding-box centre, and average the two.
The new point is (0.166308001117, -0.000000000081, -0.027090638862) m, 3.433 cm forward along approach. Axes are
unchanged. MuJoCo's site_sameframe optimization is disabled for the moved diagnostic site so its FK actually moves.
The cameras and physical meshes do not move. Real hardware needs Gavin's dreamcli tcp calibration.

`single` removes the entire right body tree, camera, contacts, equality and actuators. It sends arm-file rest joints
with an open grip for the right state, black right wrist frames, and ignores the right chunk. Left base is unchanged.
`r24` executes the first 24 of 48 rows at 20 Hz before requesting again.
`table` uses documented dream/client_ik.py ik.region.left: world x [-.08,.40], y [-.32,.16], z [.785,1.06] m.
Table z=.76, so the box is 2.5 to 30 cm above it and covers object spawn jitter and targets in front of the arm.
In left base coordinates: lo [.08,-.385,.013], hi [.56,.095,.288] m. The client clips targets inward by its collision
margin and adds a region cost on arm centreline points; its hard fence is off by default. IK still determines pose
reachability. The dry test directly verifies the native solver's target clipping.

Seeds: 20261020 + episode index for apple, 20271020 + index for box. Uniform independent x/y jitter ±2 cm, paired
across configurations. Fresh physics scene per episode. Existing geometry/rest success check must pass at two
consecutive call endpoints; the confirming call is included in calls to success. Grasp means >3 cm centre-height
increase from settled spawn. Contact rows count any finger/table contact once per executed row. Requested-hand
proximity uses raw model positions against the object's position immediately before each row, for executed rows.

Every request reserves a ledger slot, atomically written and fsynced before get_action. Only explicit DA-2000
pool_warming refunds it. Unknown outcomes remain charged conservatively. A global file lock prevents concurrent
runner processes. Chunks are fsynced before execution. On restart, saved joint chunks replay from the seeded scene
with no requests; completed episodes skip. A new client resumes from that recovered scene, resetting seam history
for the next chunk. A finished episode whose video is missing is reconstructed. Only log.json, chunks.jsonl, run.mp4
and a first-episode contact sheet are saved inside episode folders.

Stage selection aggregates successes across tasks in that stage, breaking ties by total calls to confirmed success.
Stage 1 exact tie keeps B0; stage 2 exact tie carries single as requested. Stage 3 compares equal seed prefixes when
shortened and combines knobs only when each improves that score. Stage 4 reuses existing episodes of its selected
configuration, keeping six total per configuration/task; it budgets conservatively for missing episodes at full caps.

Validation artifacts: runs/sweep/validation/checks.json (32 scene/task combinations, four client FK/clamp variants,
ten FK samples each), runner_check.console.log (fake-client two-call stop and interrupted/finished resume),
baseline_check.console.log (all 18 previous modes dry), final_audit.json (counts, caps, files, video checks, hashes).
Validation used zero inference calls. Original viscous_arm.json SHA256:
cbbd055358a98b8ef39eecde33df87db265ed2ebd7b06f8fd46f32674fe0d2b0.

One-arm follow-up (`onearm_sweep.py --live`): apple only, six paired episodes of
`single_copy` and `parked`; conditionally test the winner and `single` in a left-side
layout when neither reaches 5/6. `onearm_check.py` runs the compatibility and full
18-call-length no-inference checks. `onearm_report.py` regenerates the report.
The follow-up authorizes exactly 440 additional calls over 872 already used, so
its one-time locked ledger migration sets cap=1312 (not 1340). Re-runs do not add
budget. Both stage plans reserve episode counts conservatively at full caps.

New configuration fields default to the previous behavior: `wrist_r='normal'`,
`arms='both'`. `single_copy` is `arms='single', wrist_r='copy'`: remove the right
arm, duplicate the left RGB frame unchanged at every capture (and therefore its
encoded JPEG), keep right rest/open state, and discard right actions.
`parked` is `arms='parked'`: retain the right arm at B0's base; constrain its six
joints and two fingers at arm-file rest/open, servo there, and remove finite
constraint residuals after each 1 ms physics step. This ideal position hold is
immune to settling/contact displacement at observation and control endpoints;
logs include maximum measured and pre-projection solver deviations. Its wrist
camera and measured joint state remain real, and right actions are discarded.
Neither arm JSON file is edited. First-episode contact sheets for new modes show
all three actual encoded call-1 policy frames, with labels outside the images.
