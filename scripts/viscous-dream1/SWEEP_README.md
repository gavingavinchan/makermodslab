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

Isolation follow-up: `onearm_isolate_check.py` runs zero-inference checks and
full dry episodes; `onearm_isolate_sweep.py --live` runs six paired apple episodes
per control under an additional hard 220-call ceiling from ledger usage 1029.
The existing global cap stays 1312. `single_fixedcam` selects `arms='single',
wrist_r='fixedcam'`: no right body tree, a world-fixed camera with the parked
rest/open wrist's exact position, orientation, 77.2-degree vertical FOV and
640x360 rendering, right rest/open state and ignored right actions. Removing the
arm also removes its wrist-view fingers and its shadows/reflections.
`parked_blind` selects `arms='parked', wrist_r='blind'`: all parked physics/state
and visible geometry stay intact, but every right policy image is black.
Default `wrist_r='normal'` and all previous configuration names are unchanged.
`onearm_isolate_contacts.py` replays saved parked/blind chunks without inference
and audits right-body contacts after every 1 ms physics step.
`onearm_isolate_report.py` regenerates the final isolation report and checks
paired spawns, accounting, protected hashes, permitted files, and video metadata.

Propeller exploration (2026-10-23): `propeller_sweep.py` runs four 30-chunk local
stub episodes without inference; `propeller_sweep.py --live` runs Stage A (two
paired seeds per P1–P4), selects tool-contact rows then reversals (stable P1–P4
tie order), and runs three more Viscous plus three YAM vision episodes with that
prompt. It calls the existing `sweep.episode` runner, ledger, fsynced chunks and
seeded replay; `sweep.py --task propeller --prompt-id P1 --episodes 1` is also
available. Legacy task caps and stage iteration are unchanged. Live startup
migrates cap 1312→1912 exactly once under the ledger lock, recording Gavin's
600-call authorization. The full experiment needs at most 420 calls.

`propeller_scene.py` generates three twisted bronze blades under assets/propeller,
with six convex collision segments per blade, a fixed hub and stand, and a free
9×6×4 cm yellow/green sponge. All fruit, bottle, bowl and plate are removed in this
task only. Table, cameras, lights and arm bases are unchanged; no brush is added.
Per-row contact booleans collect every physics substep. Grasp means >3 cm lift
and sponge centre between distal fingers. Scrub path projects row displacement
onto the nearest blade triangle plane while actual sponge/finger collision
surfaces are within 1 cm; reversals require >1 cm out then >1 cm back along an
initial tangent with hysteresis, resetting outside each contact bout. This is a
motion signal, not cleanliness. `path_in_contact_m` sums projected travel only
in rows with physical contact by that effector; `path_within_1cm_m` also includes
the proximity band used for reversals. Requested-hand proximity is point-to-surface
against the propeller and oriented sponge before execution, for both hands.
Each episode runs 30 calls with no success stop. Video shows top and left wrist;
first-episode sheets contain the exact three encoded call-1 policy JPEGs.

No-call checks: `propeller_ik_check.py` (native client IK, three points over a 3 cm
surface pass; multistart seed), `propeller_check.py` (measurements, locked cap
migration fixture, four full stub episodes and deterministic replay), and all
previous checks. Results live in runs/propeller/validation.

Flange scene preview (2026-10-23, **local only**):
`sweep.py --task flange --config B0 --episodes 1` executes four 48-row hold-stub
chunks through the existing runner, including fsync, row logs, replay, policy
sheet and video. Outputs go to `runs/flange_v2/dry/`; the ledger is never accessed.
The flange task rejects live execution before constructing a scene/client.
Legacy task caps, stage selection, modes, station cameras/lights and arm JSONs
are unchanged. `flange_render.py` makes the three 1280×720 inspection renders;
`flange_demo.py` records a contact-only IK attempt (no object setters or grasp
welds), with top + inspection view. The current attempt lifts the nut but fails
to lift the bolt; it does not prove full assembly feasibility.

`flange_scene.py` adds fixed 0.6 m coated pipes/grey eight-hole weld-neck-style
flanges with seven paired-nut studs, real convex-decomposed clearance holes, and free M30-size fasteners.
Its 1 ms ideal thread projection engages only near the tip, within 3 mm radial
and 10 degrees axial error; rotation advances the nut 3.5 mm/revolution. Reverse
rotation or sufficient attempted withdrawal releases it. The bolt is never
snapped or constrained to the holes. This is an ideal thread surrogate, not a
thread torque or cross-threading simulation. See `runs/flange_v2/flange_v2_report.md`; `runs/flange/` is preserved as the before.
`flange_check.py`, `flange_ik_check.py`, and `flange_runner_check.py` are zero-call
physics, native IK and exact local-stub replay checks. Per-row logs include
insertion depth, nut engagement/turns/advance, lift/between-finger/contact grasp,
object/fixture/gripper/table contacts, and requested-hand distances.

Subsea v2 preview: `flange_assets_v2.py` authors local procedural meshes/PNG textures;
`flange_clearance_check.py` verifies sampled joint paths against both arms, table and full
pipework. `flange_visibility_check.py` records actual object pixels and target-mouth
occlusion in unchanged call-1 cameras. All current flange outputs default to `runs/flange_v2/`.
