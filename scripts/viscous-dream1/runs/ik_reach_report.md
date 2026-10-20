# Hand intent versus IK reach: live2 and offline replay

The live Viscous failure is **(a): wrong requested hand positions**, not (b)'s proposed inability of client IK to reach sensible apple-directed requests. The closest left request remains 19.05 cm from the apple; solved and measured hands closely follow those wrong requests. This establishes wrong intent, but does not isolate appearance confusion from differences in pose anchors or vision-only policy behavior.

There is also an independent execution problem on the transplanted YAM path. Viscous's command FK can follow its grasp phase closely, but the physical fingers collide with the table. The arm never grasps/lifts the apple and later contacts the bowl. This is not evidence that the apple target is outside the kinematic workspace or that rest pull caused the live failure.

## Calls and execution

All commands used `MUJOCO_GL=egl`, workspace `TMPDIR`, and `/home/gavin/dreamtrial/.venv/bin/python`:

```text
yam_baseline.py --mode vision --live --max-calls 15 --out runs/yam_vision_live2
yam_baseline.py --mode vision --rig viscous --live --max-calls 15 --out runs/viscous_apple_vision_live2
replay_hand_path.py
```

YAM: **15 billed**. Viscous: **15 billed**. **30 total**. No warming refusals or other retries. Replay and contact-check replay: **zero inference/network calls**. Each run executes 15 chunks × 48 rows = 720 rows at 20 Hz.

YAM first meets the conservative bowl/rest success check at call 10, row 36 (zero-based). Final success is true, and frames show a grasp, lift, and placement. Viscous has zero apple-contact rows, no success, and minimum apple-to-bowl XY distance 0.208648 m. Replay has no success, maximum apple rise 0.005667 m, one apple-contact row, and minimum apple-to-bowl XY distance 0.107324 m: the apple is displaced, not picked up or placed.

## Main measurements

Median / 95th percentile over all 720 executed rows of each side. Position and rotation errors compare Dream FK of the **executed joint commands after client smoothing/seam and joint-limit clipping** against the **raw per_key requested pose**, with Dream's own tool point. Rotation is the SO(3) angle after Dream's rot6 Gram–Schmidt decoding. Apple distance uses the object's current world position transformed into that arm's own base frame before the row executes.

| Run | Arm | FK position error median / p95 (m) | Rotation median / p95 (deg) | Minimum requested xyz → apple (m) |
|---|---|---:|---:|---:|
| YAM live2 | left | 0.004949 / 0.021362 | 0.682 / 2.047 | 0.002916 |
| YAM live2 | right | 0.004601 / 0.014093 | 0.561 / 1.263 | 0.251294 |
| Viscous live2 | left | 0.006711 / 0.016839 | 1.071 / 4.053 | 0.190538 |
| Viscous live2 | right | 0.005860 / 0.011419 | 0.770 / 1.461 | 0.448143 |
| YAM → Viscous replay | left | 0.011651 / 0.050890 | 2.534 / 17.317 | 0.006636 |
| YAM → Viscous replay | right | 0.004622 / 0.009235 | 0.691 / 2.204 | 0.378858 |

Using the *initial stationary apple* instead of the moving apple, minimum requested distances are YAM left/right 0.006645 / 0.463063 m, Viscous left/right 0.190539 / 0.448149 m, replay left/right 0.006640 / 0.463059 m. YAM's smaller moving-apple distance partly occurs while carrying it. In Viscous live2, the closest left request is call 3, row 44: requested base xyz [0.208997, -0.016261, 0.126908], apple [0.134933, -0.154994, 0.019333], solved FK [0.221126, -0.008663, 0.130611]. It asks to stay substantially above and beside the apple, and the solve follows within 14.8 mm.

The filtered-target left position errors (median / p95) are YAM 0.001298 / 0.020624 m, Viscous 0.002662 / 0.014790 m, replay 0.010302 / 0.049104 m. Small raw errors partly reflect the client's intended filtering and seam, not inability to reach.

## Replay, physical tracking, and seam

Replay feeds the raw `per_key` poses/grips directly to `ClientIK.solve`, then the same `SeamSmoother.smooth` and executor validation as live vision. It does not translate, re-anchor, or transform YAM poses to accommodate Viscous. Each pose stays numerically identical in its own side's base frame. Both Viscous arms start neutral/open and receive the same 33-row camera-window warmup as live. Each chunk starts from Viscous's own measured joints or prior Viscous command history; copying YAM joint starts would be invalid. The source client's measured RTT is reused for the seam clock. All subsequent takeover skips are zero because every prior chunk was fully executed. Replay waits for the default trajectory solve to compile; this matches Viscous live2, which used `traj` for every call. YAM used `tick` for calls 1–2 and `traj` thereafter.

During calls 4–6 (grasp), replay command FK has per-call median position errors 5.1, 4.2, and 4.0 mm and median rotation errors 0.66°, 0.59°, and 0.55°. Yet physical measured-joint FK has 83.1 mm median / 93.8 mm p95 position error over that phase. Whole replay left measured-FK errors are 87.3 / 273.1 mm and 11.50° / 24.96°; the commanded-joint table above must not be mistaken for physical tracking. Closest measured left TCP to apple is 36.0 mm (command FK: 11.6 mm).

A second offline contact-check replay records gripper/table collision beginning call 3, row 45, gripper/table contact on 155 rows, and gripper/bowl contact on 357 rows. This independently supports physical interference during the grasp and subsequent transfer. This repeat differs slightly (maximum left command difference 0.006413 rad; measured difference 0.018312 rad), so contact counts belong to that diagnostic repeat rather than the canonical replay. No live calls were added. See `replay_contact_analysis.json` and the diagnostic trajectory's `robot_contacts`.

Dream FK from measured Viscous joints agrees with the recorded physical TCP position to <0.083 mm, consistent with the trajectory's 0.1 mm position rounding. The larger replay errors are not a tool-point coordinate mismatch.

## Limits and rest

“Near a limit” below means within 0.02 rad of Dream's margined solver bound, evaluated on executed commands. A neutral initial shoulder/elbow can lie outside the solver margin while still being at a valid URDF limit; those initial rows are not counted as being within 0.02 rad of the margined bound.

Viscous live2 left: shoulder_lift is near its upper bound −0.034907 rad for 18.33% of rows; elbow_flex near its lower bound +0.034907 for 31.67%; wrist_flex near its upper bound +1.571093 for 6.25% (45 rows in calls 12–15). The other joints never approach a solver bound. Right: shoulder_lift upper 14.17%, elbow_flex lower 4.44%; no other saturation. Neither side has a row exceeding the diagnostic miss threshold (position >5 cm or rotation >15°). Thus these limit encounters do not explain failing to reach apple-directed poses: no such requests were made.

Rest is [0, −1.895, 2.094, −1.608, 0, 0] rad, with Dream rest residual weights [0.15, 0.3, 0.3, 0.3, 0.3, 0.3]. The live left elbow only reaches 1.199 rad, far from rest 2.094; wrist_flex reaches +1.571 while its rest is −1.608. Shoulder_lift passes near rest briefly (2.22% within 0.1 rad). The zero-rest pan/yaw/roll naturally spend some time near zero, but proximity alone cannot identify a rest-pull cause. The logged postures do not support the proposed retreat into the rest pose.

Replay has 114 left rows above that miss threshold. On those rows **no commanded joint is within 0.02 rad of either solver limit**. Median distances to rest for pan, lift, elbow, wrist flex, yaw, roll are [0.632, 0.496, 0.639, 0.549, 0.524, 0.444] rad. Only lift (8.77%) and pan (2.63%) ever lie within 0.1 rad of rest on those rows. Rest cost is active by construction, but the data do not establish rest pull as the reason for the replay miss; contacts and measured-command divergence provide direct evidence of another constraint.

## Artifacts and verification

- Changed `yam_baseline.py`: vision-only instance wrappers capture exact returned anchors; write raw hand/grip heads, solved chunks, per-row command-FK errors, measured arm joints, and apple base coordinates. Viscous clips finite grip commands to [0,1] with notices, preserving the raw reply. Station execution remains unchanged.
- Added `hand_path_tools.py`: shared read-only tracing, Dream FK errors, frame conversion, and grip clipping.
- Added `replay_hand_path.py`: offline replay with frames, contact sheet, video, row errors, measured joints, and contacts. No Dream client/network setup.
- Added `hand_path_check.py`: offline transparency/FK/grip regression and replay fixture.
- Added `analyse_hand_paths.py`: numerical comparisons, measured-FK diagnostics, joint limits/rest statistics, and plot data.
- Added `plot_hand_paths.py`: static plot using the system matplotlib; all plotting caches/temp files remain in the workspace.

All seven existing dry mode/task variants passed: YAM station apple/orange/cube; YAM vision apple/orange/cube; Viscous vision apple. The offline regression confirms wrapped observation and solve outputs are identical to their originals, checks 48 aligned FK rows, and accepts a −0.089 grip by clipping without mutating raw values. The zero-network replay smoke test passed. No installed package or `viscous_arm.json` was edited. No hardware access.

Numerical source: [hand_path_analysis.json](hand_path_analysis.json). [Comparison plot](hand_path_comparison.png).

[YAM frames](yam_vision_live2/contact_sheet.jpg) · [Viscous frames](viscous_apple_vision_live2/contact_sheet.jpg) · [Replay frames](replay_yam_path_on_viscous/contact_sheet.jpg) · [Replay video](replay_yam_path_on_viscous/run.mp4).
