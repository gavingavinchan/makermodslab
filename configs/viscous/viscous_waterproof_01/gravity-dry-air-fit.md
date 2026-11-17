<!-- Copyright 2026 MakerMods. All rights reserved. Licensed under the Apache License, Version 2.0. -->
# Waterproof arm empirical dry-air gravity fit

619 static windows/holds, 14 CV groups. Provisional: True.
Provisional feedforward requires --allow-provisional-fit; environment installer refuses it. Gripper locked at Pinocchio neutral.
Zero-centered ridge; CAD inertials appear only in the reference error. Directions are signed last motor motion.

| Joint | R² | RMS Nm | Held-out Nm | CAD Nm | Max residual Nm | Coulomb Nm |
|---|---:|---:|---:|---:|---:|---:|
| shoulder_pan | -0.012 | 0.222 | 0.222 | 0.222 | 0.838 | 0.000 |
| shoulder_lift | 0.662 | 0.808 | 0.970 | 1.551 | 3.204 | 0.000 |
| elbow_flex | 0.705 | 0.919 | 1.057 | 1.639 | 3.195 | 0.000 |
| wrist_flex | -0.203 | 0.973 | 0.998 | 0.854 | 3.348 | 0.000 |
| wrist_yaw | 0.324 | 0.230 | 0.336 | 0.319 | 0.845 | 0.000 |
| wrist_roll | -2.370 | 0.171 | 0.170 | 0.096 | 0.553 | 0.000 |

Static rank 10/24; augmented rank 10.
The JSON records every identified right-singular combination and scale. Individual masses/COMs are not identifiable.
Basis is recomputed using training data only in each CV fold. Coulomb signs/offsets can be confounded in one-direction logs; signed estimates are not physical friction certificates.

Coverage (motor degrees):
- shoulder_pan: 148.75 to 183.32
- shoulder_lift: -137.98 to -58.26
- elbow_flex: 27.73 to 123.86
- wrist_flex: -59.22 to 31.70
- wrist_yaw: -12.25 to 4.89
- wrist_roll: -16.50 to 15.46

Everything outside these ranges is unsampled, especially wrist yaw/roll and combinations of wide shoulder/elbow/flex angles. Narrow logs do not validate extrapolation.

RS00 telemetry versus kp*(command-position), using Viscous defaults with runner overrides:

- shoulder_pan: kp 60, slope 0.9339472794057954, r 0.9954672480704551, RMS difference 0.063 Nm
- shoulder_lift: kp 90, slope 0.9887793173130983, r 0.9190423161861488, RMS difference 0.568 Nm
- elbow_flex: kp 90, slope 1.004847946938638, r 0.9097688831608116, RMS difference 0.727 Nm
- wrist_flex: kp 30, slope 1.0484989401104738, r 0.9122452326883951, RMS difference 0.369 Nm
- wrist_yaw: kp 30, slope 0.9039666348537226, r 0.9680221026013329, RMS difference 0.087 Nm
- wrist_roll: kp 30, slope 0.6682260749847946, r 0.7114947570768785, RMS difference 0.074 Nm

Sources (hashes and selected counts in JSON):
- live-20261117T024118Z/ticks.jsonl: 31
- live-20261117T024948Z/ticks.jsonl: 47
- live-20261117T025317Z/ticks.jsonl: 46
- live-20261117T030355Z/ticks.jsonl: 41
- live-20261117T030700Z/ticks.jsonl: 12
- live-20261117T054216Z/ticks.jsonl: 91
- live-20261117T073809Z/ticks.jsonl: 59
- live-20261117T074056Z/ticks.jsonl: 56
- live-20261117T075837Z/ticks.jsonl: 55
- live-20261117T082639Z/ticks.jsonl: 42
- live-20261117T082827Z/ticks.jsonl: 32
- rehearse-live-20261117T023712Z/ticks.jsonl: 35
- rehearse-live-20261117T024005Z/ticks.jsonl: 36
- rehearse-live-20261117T090749Z/ticks.jsonl: 36
