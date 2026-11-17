<!-- Copyright 2026 MakerMods. All rights reserved. Licensed under the Apache License, Version 2.0. -->
# Waterproof arm empirical dry-air gravity fit

583 static windows/holds, 13 CV groups. Provisional: True.
Provisional feedforward requires --allow-provisional-fit; environment installer refuses it. Gripper locked at Pinocchio neutral.
Zero-centered ridge; CAD inertials appear only in the reference error. Directions are signed last motor motion.

| Joint | R² | RMS Nm | Held-out Nm | CAD Nm | Max residual Nm | Coulomb Nm |
|---|---:|---:|---:|---:|---:|---:|
| shoulder_pan | -0.015 | 0.228 | 0.228 | 0.228 | 0.838 | 0.000 |
| shoulder_lift | 0.652 | 0.834 | 1.011 | 1.590 | 3.188 | 0.000 |
| elbow_flex | 0.687 | 0.952 | 1.100 | 1.660 | 3.174 | 0.000 |
| wrist_flex | -0.413 | 0.939 | 0.956 | 0.766 | 3.230 | 0.000 |
| wrist_yaw | 0.312 | 0.235 | 0.344 | 0.327 | 0.852 | 0.000 |
| wrist_roll | -2.868 | 0.175 | 0.174 | 0.091 | 0.554 | 0.000 |

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

- shoulder_pan: kp 60, slope 0.940843228882909, r 0.9954856073214378, RMS difference 0.063 Nm
- shoulder_lift: kp 90, slope 0.9723898315705323, r 0.9998574293953022, RMS difference 0.085 Nm
- elbow_flex: kp 90, slope 0.9806612407654794, r 0.9998900526942196, RMS difference 0.084 Nm
- wrist_flex: kp 30, slope 0.9745909094216791, r 0.999894896277747, RMS difference 0.040 Nm
- wrist_yaw: kp 30, slope 0.9468919696784441, r 0.9991093414034684, RMS difference 0.037 Nm
- wrist_roll: kp 30, slope 0.8029361710014908, r 0.9930677184196693, RMS difference 0.038 Nm

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
