Real Viscous Dream-1 executor: read-only research, 7 November 2026

This is goal B3 research. No DreamLabs request, hardware access, checkout edit, or commit was made. Only source, package metadata, saved JSON, and independent NumPy/XML calculations were
used. Public JAX documentation was checked for Mac wheel support. No credentials file was opened.

Evidence path abbreviations: S = /home/gavin/dreamtrial/.venv/lib/python3.12/site-packages
R = /home/gavin/Documents/makermodslab-viscous-tankbox
D = /home/gavin/dreamtrial/viscous_sim/runs
T = /tmp/tankbox/research
Every abbreviated file:line below expands using these roots. Units are explicit.

1. Client contract and IK

Use DreamClient.policy with checkpoint dreamlabs/dream-1-beta, the YAM embodiment camera/part map, ik={arm: <Viscous JSON>, head: eef}, action_hz=20. The custom arm defaults to
other_embodiment. The proven setup uses timeout_ms=45000, strict=false, _record=false. Evidence: R/scripts/viscous-dream1/yam_baseline.py:68,207; R/scripts/viscous-dream1/sweep.py:153.

Call set_observation(frames, state, timestamp=<monotonic seconds>), then get_action(commanded=...). Frames have keys video.overhead, video.wrist_l, video.wrist_r. Each value is one base64
JPEG string. The client keeps 9 frames per camera at 5 Hz, oldest first, spanning 1.6 s once full. A lower-level PolicyClient.get_action(observation, commanded=...) accepts the windows
directly. Fewer than 9 frames are allowed at episode startup. The client does not decode or resize JPEGs. The successful makerwrist/B0 scripts use RGB images resized to 320 x 176 pixels,
JPEG quality 85. That size is the demonstrated preprocessing convention, not a size check enforced by this client. Evidence: S/dream/facade.py:503,544; S/dream/client.py:294;
S/dream/_window.py:34; S/dream_examples/yam_mujoco/dream_control.py:164,177,183.

For EACH side send state.left_arm / state.right_arm: six measured joint angles in radians. Order: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_yaw, wrist_roll. Send
state.left_grip / state.right_grip: one number in a list, normalized 0 closed, 1 open. Send prompt as the instruction string, e.g. "put apple into the bowl". Do not supply
state.<side>_eef_xyz or state.<side>_eef_rot6 with ik enabled: the client computes them. These FK anchors are [3] metres and [6] rotation values, the first two rotation-matrix columns. The
frame is each arm's selected URDF base: x forward, y left, z up. There is no common world frame. For the YAM-trained model, the outgoing state.<side>_arm becomes six zeros;
state.<side>_gripper receives the normalized grip, and the FK anchors accompany it. The serve withholds ALL state from the model in other_embodiment mode. Anchors compose its tool-pose
answer. Evidence: S/dream/client_ik.py:309; S/dream/arm_file.py:6; S/dream/client.py:307; R/scripts/viscous-dream1/viscous_dream_sim.py:36,439.

The reply has per_key with action.<side>_eef_xyz [48,3] in metres, action.<side>_eef_rot6 [48,6], and action.<side>_grip [48,1] (or [48]). These are absolute tool poses in each arm's base
frame, after the service composes the answer on the anchor. action.left_arm and action.right_arm are None in this mode. Do not execute the YAM joint head. Client IK adds q_l_chunk and
q_r_chunk: [48,7], six absolute joint targets in radians, then normalized grip. Grip outputs can exceed [0,1]; the existing executors explicitly clip them. They are not gripper motor
degrees. At 20 Hz, one row is 50 ms and 48 rows are 2.4 s. execution_horizon defaults to 30 rows (1.5 s). It controls async replacement timing, not reply length. Sync executes all 48 rows,
then holds during inference. The default sync threshold is median recent RTT >700 ms; the median uses 5 calls. The makerwrist and B0 experiments execute 48 rows per call. The early
viscous_dream_sim uses 30. Evidence: S/dream/client.py:307,330; S/dream/client_ik.py:390,459; S/dream_ik/decode.py:132; S/dream/pacing.py:8,48;
R/scripts/viscous-dream1/hand_path_tools.py:102; R/scripts/viscous-dream1/viscous_dream_sim.py:43; R/scripts/viscous-dream1/sweep.py:26,173.

commanded is local only: q_l_chunk/q_r_chunk -> the last actual commanded row (six joints, optional grip). It anchors chunk takeover to the command, rather than lagging measured joints. It
is not sent to the service. IK, pose filtering, and seam smoothing run on the caller's CPU, thus on the Mac in a Mac executor. The solver uses vendored PyRoKi/jaxls with JAX CPU. It does not
use Pinocchio or a local neural model/GPU. Evidence: S/dream/client.py:324,433; S/dream_ik/solver.py:19,32; S/dream_ik/__init__.py:11.

IK is a deterministic numerical procedure for fixed targets, initial joints, history, spec, and solver mode. It warm-starts least squares from the previous solution, rather than sampling
random restarts. This is not a promise of identical whole-client replies: seam logic uses history/RTT, CPU numerical results can differ, and startup uses tick until background compilation of
traj finishes. Dream inference is remote. Select solve=tick explicitly for one fixed mode, or wait for traj readiness before an executor accepts motion. A traj warm-up compilation failure
logs an exception and keeps tick. Ordinary solve exceptions propagate. Evidence: S/dream_ik/solver.py:261,291,713; S/dream/client_ik.py:208,234,244.

Limits: a stiff cost pushes inward; the solver clips its solution to URDF bounds with a 2 degree margin, then scales the step under per-joint speed limits. For this URDF, velocity=2 rad/s
=114.6 deg/s. An initial joint already outside its margin expands the solve bounds to include that initial angle. Thus the general "inside limits" claim assumes a valid start; the code can
retain an out-of-range start. Pose misses are diagnostics, not an automatic failure/hold threshold. The client discards the solve diagnostics. Malformed reply shapes raise DreamActionError.
There is no reliable non-convergence-to-safe-hold contract. The numeric part reader checks shape, not finiteness. Validate finite outputs and tracking error in the executor. Seam smoothing
runs after IK, so solver speed/limit clipping alone is not a final outgoing-command guarantee. Evidence: S/dream_ik/solver.py:184,323,334,730,768; S/dream_ik/decode.py:123;
S/dream/client_ik.py:455,943,961; S/dream/client.py:433; S/dream/smoothing.py:100.

Arm-file overrides: allowed fields are urdf, base, flange, tcp, approach, jaw, rest; optional fixed and tcp_calibration. Extra fields are rejected. There is no base rotation, limit, or speed
field. base selects an existing URDF link; the client cuts its chain to flange. rest overrides the soft preferred pose, not the measured start, and registration rejects rest values outside
the raw URDF bounds. Limits and speeds come from the URDF; velocity is capped at 3 rad/s if larger or missing. Internal ArmSpec can set rest_pose, rest_weight, max_speed, limit_margin,
tools, and the URDF path. It has no independent joint-bound array or base-pose transform. Embodiment.side_base is translation only. Public ik config accepts arm/head and optionally
solve/keep_out/reach/region, not arbitrary ArmSpec overrides. Evidence: S/dream/arm_file.py:48,100,198,238,313; S/dream_ik/specs.py:99,141; S/dream/client_ik.py:530. Existing TCP/rest:
R/scripts/viscous-dream1/viscous_arm.json:1.

180 degree tank frame: yes through URDF kinematics; no through an extra JSON rotation field. Let B be the physical base and D be Dream's base, rotated +180 degrees about z from B. Then
p_D=Rz(-pi)*p_B, and R_Dtool=Rz(-pi)*R_Btool. A frame rotation alone does NOT change J1's number. With motor J1=+185 degrees, the old [-3.235,2.227] rad bound still excludes it. A coordinate
change cannot establish that an excluded physical pose is mechanically safe. Original chain evidence: /home/gavin/Documents/viscous-arm-model/model/viscous_arm.urdf:1393.

Proposal: ship a separate, self-contained IK URDF and arm JSON, with an explicit dream_base fixed root. Set dream_base -> physical base_link to yaw +pi and select dream_base in the JSON. For
centered IK coordinates use q_IK1=q_motor1-pi; apply the inverse q_motor1=q_IK1+pi on output. Also give shoulder_pan's origin yaw +pi in the physical base, retaining its +z axis and
translation. Those two pi rotations cancel in the kinematic chain. Motor +182..+191 degrees becomes IK +2..+11 degrees, and the resulting FK is the physical tool expressed in D. This
preserves the base's explicit physical meaning. Set J1 limits to VERIFIED unwrapped motor bounds minus pi, and rest to the desired motor working pose minus pi. Keep the other joints'
mappings and the flange-local TCP/axes unchanged. Transform cameras/objects consistently. Do not merely subtract pi from the old URDF limits: they would still exclude this start after
conversion. An equivalent old angle for +185 is -175 degrees, but blindly wrapping outputs risks a full physical revolution. Keep one unwrapped motor branch throughout the executor, driver,
logs, and external guard. The frame may instead be represented by shifting the first origin alone, but that hides what base_link means. The explicit root plus documented J1 conversion is
cleaner than patching the installed solver or disguising bounds. If retaining motor coordinates in IK, use only the rotated root, valid motor-coordinate limits, and motor-frame rest. The
existing JSON's ../../Documents/... URDF path assumes the dreamtrial layout; fix it for a portable Mac bundle. These are proposed asset changes, not changes made here. Hardware bound
verification remains unresolved: R/docs/viscous-haptics.md:1834 documents several J1 frames and warns the real zero/limits may have changed. Frame algebra check at motor J1=182,185,191 degrees: transform error <1e-12 (T/frame_check.py:1; T/frame_check.json:1).

2. One working arm

The custom-arm registration always creates both right and left sides. This path needs both state/camera slots. Use the working arm as left, as in the experiments. Ignore q_r_chunk for
hardware actuation. For single/parked experiments the right state is [0,-1.895,2.094,-1.608,0,0] rad and grip=[1]. Single has no physical right arm but still reports that synthetic PARK
state. Parked has the actual fixed arm there. The client sends FK(PARK) as right XYZ/rot6, six zeros as right_arm, normalized right_gripper=[1], and the 9-frame video.wrist_r window. The
supplied state.right_grip also remains in the sent dictionary. A copied/blacked image changes that window, not the right state contract. Also pass the fixed right commanded row when
maintaining the parked context. Never run its returned targets. Evidence: S/dream/arm_file.py:68,133; S/dream/client_ik.py:341; R/scripts/viscous-dream1/sweep_scene.py:11,36,134,140;
R/scripts/viscous-dream1/sweep.py:173,346.

A saved PARK anchor is right_eef_xyz=(0.174703,0,0.100003) m; right_eef_rot6=(-2.57e-8,1,0,1,-2.57e-8,0.000320653).
Evidence: D/sweep/parked/apple/ep_00/chunks.jsonl:1. It changes if the right-arm model/frame/rest changes.

Measured apple outcomes: both active arms 6/6; single black wrist 2/6; left copied to right 2/6; fixed bare camera 1/6; parked arm with black wrist 4/6; parked arm with its own wrist camera
6/6. The parked arm did not touch the task. Its visible body and its wrist view both contributed in this small sample. A camera plus dummy gripper is untested. One real arm can satisfy the
API but has no equally reliable measured setup. An unpowered parked arm must be physically supported; the Viscous has no brakes. Evidence: R/docs/viscous-haptics.md:1065,1078;
R/docs/viscous-arm.md:135.

3. macOS arm64 and credentials

The client/IK path should run on Apple Silicon CPU. This is source/package evidence, not a Mac execution test. Both installed Dream distributions are pure Python wheels (py3-none-any; each
dist-info/WHEEL:3). dreamlabs advertises Linux in METADATA:16, but no Linux-only requirement was found in the client/IK core. Python 3.12 is the sensible common version: dreamlabs requires
>=3.11,<3.14; client requires >=3.10. Pinned jaxlib 0.7.1 has a CPython 3.12 macosx_11_0_arm64 wheel: [PyPI release files](https://pypi.org/project/jaxlib/0.7.1/#files). JAX CPU officially supports Mac ARM:
[JAX installation documentation](https://docs.jax.dev/en/latest/installation.html). The client forces CPU, so Metal/CUDA is unnecessary. Reinstall native dependencies on the Mac. Do not copy this Linux venv or its x86_64
.so files.

Declared dependencies, exactly from S/dreamlabs-1.0.1.dist-info/METADATA:22-62: av>=15; boto3>=1.36; click<8.2.0; dm-env==1.6; dreamlabs-client[datasets,live]>=1.0.1,<2; einops>=0.7;
jax-dataclasses==1.6.3; jax==0.7.1; jaxlib==0.7.1; jaxlie==1.5.0; jaxtyping; loguru; mink>=0.0.13; mujoco>=3.3.5; numpy==2.2.6; opencv-python-headless>=4.10; pillow>=10; portal==3.7.3;
prompt-toolkit>=3.0,<4; pyarrow>=17; pydantic>=2; python-can[serial]==4.5.0; pyyaml>=6.0; rich>=13; scipy>=1.11; termcolor; threadpoolctl==3.6.0; tqdm; trimesh; typing-extensions>=4.5;
tyro==0.9.26; viser>=0.2.0; websockets>=13; yourdfpy==0.0.60. pyrealsense2>=2.55 is Linux-only and Python<3.13 only; it is skipped on macOS. The so101 extra adds lerobot[feetech]>=0.5 on
Python>=3.12; it is not needed for this arm. Most IK-related lines carry a Python>=3.11 marker, already satisfied by dreamlabs' Python requirement. Client 1.0.3 itself requires numpy>=1.24
and websockets>=13; datasets adds av/boto3/pyarrow; live adds av/opencv-python-headless. Evidence: S/dreamlabs_client-1.0.3.dist-info/METADATA:22.

Dream/dream_ik/PyRoKi/jaxls have no .so here; PyRoKi and jaxls are bundled by dreamlabs, not extra pip packages. Native dependencies include jaxlib/XLA, numpy, scipy, MuJoCo, PyAV/FFmpeg,
OpenCV, Pillow, PyArrow, pydantic-core, ml_dtypes, dm-tree, PyYAML, and some geometry dependencies. No pin/Pinocchio distribution is installed, and the IK imports do not use it.
T/native_inventory.json lists all 35 installed distributions with native files. Evidence: S/dream_ik/solver.py:19; S/dreamlabs-1.0.1.dist-info/RECORD:42; each listed package's WHEEL/RECORD.
Linux-only TCP_INFO diagnostics degrade to empty/None elsewhere; they do not block calls (S/dream/transport.py:94,113). The sim harness sets MUJOCO_GL=egl and Linux cache paths; those are
not requirements of a real camera executor (R/scripts/viscous-dream1/yam_baseline.py:34). Use the Mac's existing serial CAN driver, not Linux SocketCAN names.

DreamClient takes an explicit api_key first; otherwise DREAM_API_KEY in the environment, then ~/.dreamlabs/credentials (or DREAM_CREDENTIALS). URLs also come from that configuration. The sim
settings helper additionally checks ~/.config/dream-yam-sim/credentials.json, between env and credentials. Evidence: S/dream/facade.py:93; S/dream/credentials.py:111,136,231;
S/dream_examples/yam_mujoco/dream_control.py:25; S/dream_examples/yam_mujoco/credentials.py:20. No key or credential contents were read, printed, copied, or included in the scratch outputs.

4. Saved motion and latency

T/analyze_chunks.py is the offline calculation. T/manifest.json records input paths, hashes, and exclusions. The snapshot enumerated 177 chunk files, rather than the brief's 191. The
directory was changing during research. 111 primary Viscous files were included: 2,294 chunks, 107,808 executed rows per side. Excluded: 19 empty files, 35 fixtures/dry runs/replays/legacy
copies, 12 non-Viscous or unidentified files. Included sets cover makerwrist, B0 apple/box, one-arm variants, propeller, flange, shackle, and confirm_1107. This is a mixed set of
tasks/configurations, not a safety envelope or a success-rate estimate.

For each row delta=q_target-previous_executed_target; implied speed=abs(delta)*20 in deg/s. The first row of each episode uses the saved observed start. Chunk boundaries use the
prior executed final target. No angle wrapping is applied. Execution arrays take precedence; baseline trajectory.json supplies actual commands. The fallback is the saved solved chunk
truncated to executed_rows, with grip clipped to [0,1]. Counts below exclude the gripper: 15 deg/s permits 0.75 degree/row; 30 deg/s permits 1.5 degrees/row. A row is slowed if ANY arm joint
exceeds that step. A chunk is slowed if any of its rows is slowed. A gripper-motor slew fraction needs its real motor calibration; normalized grip alone is insufficient. These are nominal-step/time-stretch counts. Recursive target clipping can affect later rows too; these percentages are not
a simulated closed-loop slew-limiter outcome, and are not measured physical motor speeds. T/per_row_deltas.jsonl contains every analyzed joint target, signed delta, speed, grip, and source
file:line.

| Executed left-arm set | Chunks / rows | 15 deg/s: rows / chunks | 30 deg/s: rows / chunks |
|---|---:|---:|---:|
| makerwrist apple | 20 / 960 | 27.50% / 60.00% | 13.23% / 45.00% |
| makerwrist box | 20 / 960 | 27.19% / 75.00% | 3.23% / 20.00% |
| B0 apple (6 episodes) | 78 / 3,744 | 32.40% / 76.92% | 9.86% / 32.05% |
| B0 box (6 episodes) | 144 / 6,912 | 14.76% / 38.89% | 3.59% / 12.50% |
| parked apple (6 episodes) | 55 / 2,640 | 38.75% / 81.82% | 9.32% / 34.55% |
| other sweep Viscous | 1,259 / 58,128 | 19.54% / 46.47% | 7.44% / 24.46% |
| flange Viscous | 300 / 14,400 | 4.03% / 11.33% | 0.56% / 2.00% |
| shackle dream | 90 / 4,320 | 1.34% / 3.33% | 0.00% / 0.00% |
| confirm_1107 | 312 / 14,976 | 19.97% / 41.67% | 8.02% / 18.27% |
| legacy wrist apple | 15 / 720 | 28.33% / 53.33% | 1.81% / 20.00% |
| ALL included Viscous | 2,294 / 107,808 | 17.60% / 41.33% | 6.16% / 19.57% |

For either arm together, makerwrist apple becomes 29.69%/65.00% at 15 deg/s, 13.23%/45.00% at 30 deg/s. B0 apple is unchanged. The parked right arm has zero joint motion and grip=1
throughout. Across all included runs, either-arm fractions are 18.47%/43.07% and 6.55%/20.40% respectively. Per-joint table: degrees for min/max target and absolute delta; deg/s for speed.
P95 pools executed rows.

| Set / side / joint | Target min..max deg | abs delta P95 / max deg | Speed P95 / max deg/s |
|---|---:|---:|---:|
| MA L J1 pan | -53.04..61.92 | 1.62 / 4.69 | 32.32 / 93.75 |
| MA L J2 lift | -145.91..-1.93 | 1.26 / 2.97 | 25.24 / 59.31 |
| MA L J3 elbow | 1.95..155.17 | 1.14 / 2.28 | 22.75 / 45.52 |
| MA L J4 flex | -86.52..-3.03 | 1.49 / 4.42 | 29.90 / 88.39 |
| MA L J5 yaw | -0.58..60.95 | 0.99 / 4.83 | 19.83 / 96.60 |
| MA L J6 roll | -95.06..0.54 | 1.62 / 4.70 | 32.35 / 93.95 |
| MB L J1 pan | -3.74..55.77 | 0.66 / 1.90 | 13.14 / 37.91 |
| MB L J2 lift | -154.20..-1.98 | 0.89 / 2.51 | 17.86 / 50.23 |
| MB L J3 elbow | 1.99..157.21 | 1.07 / 2.21 | 21.49 / 44.29 |
| MB L J4 flex | -107.73..2.04 | 0.83 / 1.45 | 16.67 / 28.90 |
| MB L J5 yaw | -16.76..32.94 | 0.77 / 2.30 | 15.33 / 46.06 |
| MB L J6 roll | -55.51..7.06 | 0.69 / 1.22 | 13.83 / 24.49 |
| B0 L J1 pan | -44.53..71.64 | 0.89 / 1.84 | 17.79 / 36.80 |
| B0 L J2 lift | -145.93..0.00 | 1.39 / 3.60 | 27.87 / 71.97 |
| B0 L J3 elbow | 0.00..161.80 | 1.64 / 3.84 | 32.79 / 76.80 |
| B0 L J4 flex | -84.42..13.77 | 1.23 / 2.20 | 24.59 / 43.97 |
| B0 L J5 yaw | -7.21..60.76 | 0.72 / 2.04 | 14.34 / 40.82 |
| B0 L J6 roll | -72.74..32.65 | 0.97 / 1.86 | 19.40 / 37.15 |
| B0 R J1 pan | -5.95..11.47 | 0.20 / 0.54 | 4.04 / 10.81 |
| B0 R J2 lift | -17.81..0.00 | 0.19 / 0.43 | 3.77 / 8.62 |
| B0 R J3 elbow | 0.00..18.89 | 0.15 / 0.36 | 2.92 / 7.14 |
| B0 R J4 flex | -14.56..6.68 | 0.15 / 0.29 | 3.08 / 5.83 |
| B0 R J5 yaw | -16.86..6.23 | 0.22 / 0.53 | 4.34 / 10.57 |
| B0 R J6 roll | -7.58..0.71 | 0.05 / 0.13 | 0.94 / 2.59 |
| All L J1 pan | -107.84..84.35 | 0.61 / 7.62 | 12.26 / 152.44 |
| All L J2 lift | -174.90..0.00 | 1.05 / 43.32 | 20.91 / 866.43 |
| All L J3 elbow | 0.00..221.90 | 1.24 / 32.92 | 24.88 / 658.43 |
| All L J4 flex | -119.72..90.19 | 0.98 / 23.32 | 19.56 / 466.37 |
| All L J5 yaw | -50.21..84.48 | 0.55 / 35.99 | 11.05 / 719.70 |
| All L J6 roll | -151.31..51.00 | 0.70 / 65.69 | 13.94 / 1313.88 |
| All R J1 pan | -84.24..64.18 | 0.22 / 5.08 | 4.31 / 101.61 |
| All R J2 lift | -152.24..0.00 | 0.24 / 3.43 | 4.76 / 68.55 |
| All R J3 elbow | 0.00..174.07 | 0.24 / 5.75 | 4.78 / 115.06 |
| All R J4 flex | -92.13..48.19 | 0.22 / 4.97 | 4.39 / 99.39 |
| All R J5 yaw | -75.15..74.96 | 0.22 / 5.55 | 4.34 / 110.96 |
| All R J6 roll | -89.54..43.22 | 0.09 / 2.78 | 1.84 / 55.70 |

MA=makerwrist apple; MB=makerwrist box. T/statistics.json also has medians and signed delta extrema for both sides. Executed grip ranges, left/right: MA 0.376..0.993 / 0.148..0.826; MB
0.055..0.971 / 0.092..0.811; B0 apple 0.091..0.987 / 0.174..0.830. All executed grips span 0..1 on each side. Raw solved grips span -0.086..1.028 left and -0.041..1.010 right across all
runs. Keep explicit grip clipping. The extreme aggregate speeds mainly represent chunk-boundary target jumps, not sustained motion. Within-chunk left maxima J1..J6 are
152.44,140.40,129.77,118.45,134.20,152.20 deg/s (T/boundaries.json). The 1,313.88 deg/s J6 boundary is D/sweep/tip/box/ep_01/chunks.jsonl:15, a 65.69 degree target jump. This supports an
independent final slew cap even with client IK/smoothing enabled. Dataset evidence: D/viscous_apple_makerwrist_live1/chunks.jsonl:1; D/viscous_box_makerwrist_live1/chunks.jsonl:1;
D/sweep/B0/apple/ep_00/chunks.jsonl:1. Recording semantics: R/scripts/viscous-dream1/hand_path_tools.py:46,69; R/scripts/viscous-dream1/yam_baseline.py:276,300;
R/scripts/viscous-dream1/sweep.py:173,294,418.

Per-call successful latency, seconds (min / median / P95 / max):

| Set | log.json whole get_action | Saved client network RTT |
|---|---|---|
| makerwrist apple | 0.87 / 1.43 / 1.68 / 1.72 | 0.76 / 1.41 / 1.63 / 1.65 |
| makerwrist box | 0.92 / 1.25 / 1.59 / 1.66 | 0.89 / 1.22 / 1.57 / 1.64 |
| B0 apple (6 episodes) | not recorded | 0.69 / 1.56 / 2.73 / 6.12 |
| parked apple (6 episodes) | not recorded | 0.75 / 1.76 / 2.41 / 2.88 |
| ALL included Viscous | 0.87 / 1.26 / 1.65 / 1.72 | 0.62 / 1.50 / 2.66 / 6.12 |

The aggregate whole-call column has only 55 calls from the three legacy/baseline runs, not 2,294 calls. Sweep log.json records no timing. Its chunks preserve client_ik.rtt_s: measured after
the HTTP reply, before local IK/seam work. Thus B0's 6.12 s maximum is a lower bound on the required hold for that call. Makerwrist log RTT includes local processing and is rounded to 0.01
s. Failed/cold-start waits are not bounded by these successful-call statistics. The configured request timeout alone is 45 s. At native rate a B0 cycle is 2.4 s of motion plus roughly 1.56 s
median network hold plus local IK overhead. The real executor must keep CAN service and fresh camera observations running throughout the hold. Evidence:
D/viscous_apple_makerwrist_live1/log.json:4; D/viscous_box_makerwrist_live1/log.json:4; R/scripts/viscous-dream1/hand_path_tools.py:60; S/dream/client.py:412,427,433.

5. Cameras in makerwrist and sweep B0

Both use the same station overhead and Maker-style wrist mounts. The bench defaults in viscous_dream_sim are a different setup and should not be used as makerwrist camera coordinates. Below,
B is the unrotated URDF arm base. Wrist poses move with the joints; there is no single fixed base-relative pose. The sim bases share yaw -90 degrees in world. Left is (0.305,0.240,0.772) m;
right is (-0.305,0.240,0.772) m. Overhead world position is (-0.0090,0.4065,1.5543) m, optical direction (0,-0.57357,-0.81916). Relative to left B: eye=(-0.16649,-0.31400,0.78232) m; optical
up=(0.81916,0,0.57357). Relative to right B: eye=(-0.16649,+0.29600,0.78232) m; same optical up and direction in B. Look-at on the table plane: left B=(0.38969,-0.31400,-0.01200) m; right
B=(0.38969,+0.29600,-0.01200) m. In either B, optical direction=(0.57357,0,-0.81916): 55 degrees down from horizontal. Top intrinsics: fx=392.196, fy=391.722, cx=318.389, cy=237.876 pixels
at 640 x 480. FOV approximately 78.42 degrees horizontal, 62.99 degrees native vertical, 49.36 degrees after vertical crop. Render 640 x 480; retain rows 60:420 ->640 x 360; resize to 320 x
176 for the model. Evidence: S/dream_examples/yam_mujoco/yam_env.py:14,19,58,254,415; S/dream_examples/yam_mujoco/assets/sim-eval-yam.json:1;
R/scripts/viscous-dream1/cross_scenes.py:67,115,165.

Each wrist camera, relative to that arm's tool0: eye=(0,0.07934769,-0.08749375) m. Optical direction=(0,-0.342020,+0.939693); up=(0,+0.939693,+0.342020). A look-at point 1 m ahead is
(0,-0.262672,+0.852199) m in tool0. It sits 79.35 mm above the fingers and 87.49 mm behind the TCP, pitched 20 degrees down from their direction. VFOV=77.2 degrees; HFOV=109.66 degrees at
16:9. Render 640 x 360; resize to 320 x 176 without the top crop. At q=(0,0,0,0,0,0), eye in its own B=(0.148751,0,0.222367) m, direction=(0.872480,0,-0.488650). At the JSON rest
q=(0,-1.895,2.094,-1.608,0,0), eye=(0.254022,0,0.187522) m, direction=(-0.341719,0,-0.939802). Both wrists have this same transform in their respective bases. The 20 degree mount is
CAD-derived and image-matched, not a measured real optical calibration. Evidence: R/scripts/viscous-dream1/supported_arms.py:147; R/scripts/viscous-dream1/cross_scenes.py:120;
D/viscous_apple_makerwrist_live1/configuration.json:1. Calculations: T/cameras.py:1; T/cameras.json:1.

For the proposed tank frame, rotate positions/rays by Rz(pi) into physical B when mounting cameras. For example, the left-slot overhead eye becomes (+0.16649,+0.31400,+0.78232) m in physical
B, looking toward (-0.38969,+0.31400,-0.01200) m on the sim table plane. That side offset comes from a two-arm station; it is not a recommendation to assume the aquarium is that wide. Set
tank height/framing from the real floor and working region. Preserve wrist finger visibility and image orientation.

6. Proposed minimum executor, not an implementation

- One Mac controller process owns the existing calibrated Viscous serial-CAN connection and its session lock. A separate inference process owns DreamClient and CPU IK. A camera worker
  captures/encodes all three slots. Exchange timestamped measured state, last sent commands, and bounded chunks through local IPC.
- Keep motor service independent of network/JAX. Accept targets on a monotonic 20 Hz schedule. Start with synchronous chunks: finish or retime the selected rows, hold the last accepted
  target, request again. Keep cameras live during holds. Send actual last commands back to the client; never synthetic elapsed sim time.
- Validate chunk keys, [N,7] shape, finiteness, joint order, units, and age. Clip normalized grip explicitly. Convert IK J1 to the chosen unwrapped motor branch and apply the 15 or 30 deg/s
  final target slew policy. Call the separately supplied tank guard on EVERY final candidate before the CAN write, including intermediate slew steps, startup, holds, and stop commands. A
  whole-chunk check can be an extra preflight, not a substitute. Reject/stop on a guard failure. Do not send a different unguarded fallback or wrap through another branch.
- Before enabling: require current calibration, verified angle mapping/limits, valid guard/tank frame, fresh cameras, a supported stable base, and measured pose close to the initial command.
  Initialize targets from measurements. Compile IK first and provide an explicit operator start; do not send the JSON rest pose as a startup move.
- Provide local stop plus worker-death, stale-state, CAN-failure and thermal stop handling. Discard late replies after stop. Use a guard-checked controlled stop/hold; release torque only
  when physically supported (there are no brakes). Do not automatically return along an unchecked path. The operator's power switch remains the independent hard stop.
- Log local camera timestamps/images, prompts, model/client versions, asset hashes, coordinate conversions, measured joints, request start/end and timing breakdown, raw per_key, solved
  chunks, final sent targets, limiter/guard decisions, temperatures, faults, and stop cause. Keep secrets/authorization headers out of logs. Use _record=false if only local recording is
  intended. Preserve enough data for a fully offline replay.
