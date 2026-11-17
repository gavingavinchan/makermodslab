<!-- Copyright 2026 MakerMods. All rights reserved. Licensed under the Apache License, Version 2.0. -->
# Waterproof arm empirical dry-air gravity fit

339 static windows/holds, 8 CV groups. Provisional: True.
Provisional feedforward requires --allow-provisional-fit; environment installer refuses it. Gripper locked at Pinocchio neutral.
Zero-centered ridge; CAD inertials appear only in the reference error. Directions are signed last motor motion.

| Joint | R² | RMS Nm | Held-out Nm | CAD Nm | Max residual Nm | Coulomb Nm |
|---|---:|---:|---:|---:|---:|---:|
| shoulder_pan | 0.312 | 0.154 | 0.160 | 0.212 | 0.583 | 0.123 |
| shoulder_lift | 0.880 | 0.567 | 0.663 | 1.630 | 2.296 | 0.199 |
| elbow_flex | 0.865 | 0.688 | 0.818 | 1.826 | 2.594 | 0.136 |
| wrist_flex | 0.442 | 0.570 | 0.666 | 0.710 | 1.970 | 0.323 |
| wrist_yaw | 0.588 | 0.195 | 0.304 | 0.397 | 0.818 | -0.013 |
| wrist_roll | -4.873 | 0.170 | 0.186 | 0.071 | 0.576 | -0.066 |

Static rank 10/24; augmented rank 22.
The JSON records every identified right-singular combination and scale. Individual masses/COMs are not identifiable.
Basis is recomputed using training data only in each CV fold. Coulomb signs/offsets can be confounded in one-direction logs; signed estimates are not physical friction certificates.

Coverage (motor degrees):
- shoulder_pan: 154.20 to 179.89
- shoulder_lift: -75.85 to -59.47
- elbow_flex: 45.71 to 59.38
- wrist_flex: -44.94 to -35.77
- wrist_yaw: -12.25 to 4.58
- wrist_roll: -11.71 to 5.44

Everything outside these ranges is unsampled, especially wrist yaw/roll and combinations of wide shoulder/elbow/flex angles. Narrow logs do not validate extrapolation.

RS00 telemetry versus kp*(command-position), using Viscous defaults with runner overrides:

- shoulder_pan: kp 60, slope 1.0364139290441372, r 0.9944180722118973, RMS difference 0.062 Nm
- shoulder_lift: kp 90, slope 0.9726934563169102, r 0.9998913983260679, RMS difference 0.087 Nm
- elbow_flex: kp 90, slope 0.979875924182325, r 0.999910509215869, RMS difference 0.086 Nm
- wrist_flex: kp 30, slope 0.9735163448714125, r 0.9998896509398764, RMS difference 0.040 Nm
- wrist_yaw: kp 30, slope 0.9421948868830279, r 0.9992748830069107, RMS difference 0.037 Nm
- wrist_roll: kp 30, slope 0.7744429401445825, r 0.9879121902897928, RMS difference 0.038 Nm

Sources (hashes and selected counts in JSON):
- live-20261117T024118Z/ticks.jsonl: 31
- live-20261117T024948Z/ticks.jsonl: 47
- live-20261117T025317Z/ticks.jsonl: 46
- live-20261117T030355Z/ticks.jsonl: 41
- live-20261117T030700Z/ticks.jsonl: 12
- live-20261117T054216Z/ticks.jsonl: 91
- rehearse-live-20261117T023712Z/ticks.jsonl: 35
- rehearse-live-20261117T024005Z/ticks.jsonl: 36
