# Larger scrubbing strokes, same propeller and scene

Goal B3a, Gavin's refinement after viewing the corrected propeller (2026-10-24):
"i want larger motions, the side way motions is not enough."

The corrected 960 mm marine screw propeller remains the task object. Keep its
geometry, bench, arm mounts, cameras, cleaning pad, contact settings and actuator
limits unchanged. Change the motion so the right arm visibly sweeps across a
substantial part of the same left blade while the left hand keeps its outer-edge
hold. This remains the motion-review stage before the BlueROV/ship environment.

The baseline actual radial travel is only **31.506 mm**. Target **100–150 mm
actual in-contact travel per stroke**, with repeated back-and-forth sweeps.
Move the working patch inward, follow the true curved surface and slow down as
needed. A longer real-time clip is acceptable. Measure actual stroke spans;
accumulated path length, camera movement or accelerated playback do not establish
larger motion. If that target is unreachable with the fixed setup, report the
largest stable tested stroke and its limiting constraint explicitly.

Use the existing IK/servo/contact pipeline and free tool. Check both bilateral
holds, actual pad contact, left edge-hold radius, reversals, individual stroke
amplitudes, unwanted collisions, penetration, actuator saturation and release.
Preserve the smaller-motion result and prove the physics model is unchanged.
Show a clean contact-view clip as well as diagnostic camera footage and a concise
comparison against the baseline. No scenery edits, hardware activity, new Dream
calls or autonomy claim. The initialized pregrasp, fixed shaft and left-arm
5 Nm saturation remain limitations where still present.
