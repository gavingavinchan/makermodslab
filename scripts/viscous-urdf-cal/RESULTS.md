<!--
# Copyright 2026 MakerMods. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
-->

# Waterproof Viscous calibration · offline findings, 2026-11-18

**The strongest known discrepancy is the choice of TCP.** The dry-run tank model uses the pad centre. A real November 8 run used the CAD fingertip. No measured target coordinates for this physical arm accompany either file. The offline work produced no real-unit zero correction.

## Geometry and frames

| Quantity in `wrist_roll_link` | X, Y, Z (mm) | Evidence |
|---|---|---|
| Pad centre | 132.425, 0, −21.571 | Source `tool_fixed` and dry-run tank arm JSON |
| Distal fingertip centre | 166.308, 0, −27.091 | Mean of Lab’s two extreme mesh patches with jaws closed |
| Archived live TCP | 166.308, 0, −27.091 | Run `live-20261108T213138Z`, human-confirmed Dream TCP record, digest `6514c61d3eb7c893` |

The two candidate physical points are **34.330 mm apart along the fingers**. How much of that difference becomes vertical error depends on wrist orientation. Even the correct centre can sit above a finger edge that already touches glass. The dry-run JSON, the station TCP confirmation and the lowest collision vertex answer three different measurement questions. [Vendored geometry](../../configs/viscous/model/README.md) · [Dry-run arm](../viscous-dream1/real/tank_model/viscous_waterproof_tank_arm.json) · [Archived live arm](../viscous-dream1/real/runs/live-20261108T213138Z/arm/viscous-waterproof-tank.json)

The local source package at `~/Documents/viscous-arm-model` contains Gavin's 2026-09-18 STEP/F3Z exports of the shorter Viscous assembly. The STEP SHA-256 is `3c375f41702958ddd5586fa65e4a26df0340ae77f06e5ab0aa07092e01d02d43`, matching the intake and model manifest. Its generated URDF and the vendored URDF have identical bytes, SHA-256 `c1e6c3876a67ce2a2e229bdd8a80960e68f085e08833d699536b1feddefb6ff7`. Comparing the manifest’s six parent-relative joint origins with the vendored XML gave a largest difference below 0.000001 mm. Translation lengths, in chain order, are 57.400, 53.600, 160.000, 150.000, 86.922 and 60.264 mm. In particular the forearm’s 138.000 mm X and 58.788 mm Z components give its 150 mm length. All 169 referenced collision mesh files and all 60 referenced visuals matched the upstream export byte-for-byte. The review generator finds the visuals in the repo’s propeller-cardinal arm assets.

These comparisons verify the CAD export chain. They do not independently measure the waterproof unit’s axes, finger shape, installed lengths, cable envelope, flex or deflection in water. The upstream manifest identifies the pad centre and calls the slider transmission provisional. Grace’s sim/IK checks and the shoulder torque finding in B3 concern simulation consistency and load tracking. They supply no physical encoder-zero or fingertip measurements. [CAD pin and limitations](../../configs/viscous/model/README.md) · [Project evidence](../../docs/viscous-haptics.md)

**Zeros and signs.** All six source offsets are 0°. J1–J4 have only photo-consistent signs. J5 is −1 and J6 is +1 from a physical leader test of commanded follower direction. The waterproof follower's October 11 J1 flash re-zero moved neutral by 94.48° while its profile limits remained in the old frame. The November 7 value of 185.8° is a *leader alignment* for pointing down the tank. The separate November 5 left-mount re-zero concerns unit 1. None of those values is a surveyed waterproof URDF offset. The generator retains the source tank convention, **IK pan = sign × motor pan + fitted offset − 180°**, with `dream_base` rotated π above `base_link`. Rest and limits are transformed once. [Motor map](../../configs/viscous/model/motor_to_urdf.json) · [Waterproof evidence](../../configs/viscous/viscous_waterproof_01/evidence) · [Tank generator](../viscous-dream1/real/make_tank_arm.py)

## What a contact session can identify

The fit uses mm in `base_link`, a free horizontal translation, a level sheet, independently measured plane heights, and the six URDF axes. Each numbered touch supplies XYZ. A bare floor touch supplies Z only. Before-fit errors already include a best sheet registration. After-fit errors compare measured targets to the corrected closed centre. Residuals use sheet coordinates, with positive Z meaning the model reports the tip too high. Soft-L1 limits the influence of a bad touch while both reported RMS values retain every observation at full weight.

There are two exact ambiguities. Turning the arm about J1 while rotating the sheet and its origin together preserves every contact. Turning J6 while rotating the fitted TCP back in the flange also preserves every contact. Free-sheet mode holds J1. An independently surveyed sheet heading enables J1 estimation. The optional level-jaw observations constrain J6 by measuring the vertical component of the jaw-opening axis at several postures. Otherwise J6 stays at its prior value. The latter choice calibrates an effective TCP for the retained flange convention. It cannot certify flange orientation or the roll of the rest of the hand.

For the 25-target example, the two smallest touch-only singular values are below 3 × 10⁻¹⁵. With the surveyed heading and level observations the active spectrum spans 1.41–61.05 (mm of residual per degree or mm of parameter). The script reports the spectrum in every training fold and refuses active directions below 0.5 in these units. At 1.5 mm coordinate noise, a singular value of 0.5 corresponds to about 3 units of linearized one-sigma uncertainty along that direction. The gate and the externally measured datums both have finite precision. A datum heading error of 0.5° alone gives about 4.4 mm lateral error at 500 mm reach.

## Synthetic recovery

The generator solves contact poses inside the profile limits with independent URDF FK derivatives checked against Lab FK. Truth is +3, −2.5, +3, −2, +2 and +2.5° in J1–J6, TCP delta (+8, 0, +6) mm, and a 5 mm translation from the rough origin. Independent isotropic Gaussian noise has σ = 1.5 mm **per coordinate**. The three level observations have 0.3 mm height noise over 60 mm. The synthetic heading and surface heights are known. Two elbow/wrist postures per ID, three block contacts, three floor contacts and three level witnesses make 2N + 9 readings. Folds hold out every reading of an ID together, with all bare-floor points forming a separate fold. Five deterministic seeds (18–22) produced these medians.

| Targets | Touches | Largest J1–J6 error (°) | TCP error (mm) | Sheet origin error (mm) | Held-out RMS (mm) |
|---:|---:|---:|---:|---:|---:|
| 5 | 19 | 0.460 | 0.794 | 1.669 | 2.628 |
| 10 | 29 | 0.347 | 0.695 | 1.490 | 2.526 |
| 15 | 39 | 0.349 | 0.465 | 1.542 | 2.625 |
| 20 | 49 | 0.330 | 0.411 | 1.807 | 2.608 |
| 25 | 59 | 0.365 | 0.477 | 1.712 | 2.611 |

The 15–25 target sets resolved the injected 10 mm TCP error to about 0.4–0.5 mm median vector error. Held-out RMS remains near the 3D noise level (about 2.6 mm). These runs support 15 as a practical starting count for spatial coverage and repeated postures. They do not establish a universal minimum or monotonic improvement from adding points. The 25-target seed-18 fit reduced RMS **14.72 → 2.40 mm**, with **2.52 mm held out** and a largest held-out point of **5.22 mm**. Its closest reversed-sign combination scored 119.15 mm RMS. A deliberately reversed J3 in the regression suite is identified and produces only diagnostics. A separate simulation injected +4.5/−4.5 mm link corrections at 15 targets. The default held-out RMS was 4.15 mm, and enabling links recovered +4.09/−4.22 mm with 2.72 mm held out. The optional fit moves joint origins along their original link vectors. Review physical mesh lengths separately before using such a change for collision checking.

## November 8 real trace

Recomputed both finger meshes at all **857** unwrapped follower samples using the current map. Trace SHA-256 `2b7a460f44ba1976323711f7d9c8d5099f02aa1f4aebb922761343d671421f5b`. The file contains two automatic marks both labelled “right,” no surveyed XY and no contact switch. Gavin described floor, +Y wall, −Y wall, and base movement. We fitted robust, axis-aligned **plane offsets** to the low floor interval and wall sweeps, treating each time window separately. The window choice uses the observed extrema and is retrospective. [Original trace](../../configs/viscous/viscous_waterproof_01/evidence/tank-trace-20261108T044404Z.jsonl)

| Sweep window, seconds from trace start | Samples | Fitted plane (mm) | Within-window RMS (mm) | Extreme range (mm) |
|---|---:|---:|---:|---:|
| Floor, 18.85–19.60 | 7 | −4.13 | 1.14 | −6.16 to −2.72 |
| +Y, 27–42 | 135 | +207.81 | 8.70 | +177.96 to +217.31 |
| −Y, 45–57 | 108 | −229.43 | 11.36 | −255.98 to −207.35 |

On the floor interval the median **pad TCP** was +23.53 mm, the median **distal midpoint** −0.71 mm, and the lowest finger vertex reached −6.16 mm. Using the extrema over the whole trace gives the same minimum. The wall estimates imply about 437 mm width, subject to the loose base and moving on/off contact. The earlier B3 analysis selected tighter wall plateaus and found about 434 mm. Running Lab’s four-face fitter on the two recorded capture labels refuses the fit: missing two left, one near and one floor contact. This evidence supports a tool-point distinction and old-mount floor envelope. There is no absolute surveyed floor or fixed mount to separate mount height, zero error, sag and contact geometry. Later base/desk heights refer to later setups. No joint correction or new glass plane for tonight follows from this trace.

## Reproduce offline

From repo root with the calibration requirement installed, run:

```sh
C=scripts/viscous-urdf-cal
D=$(mktemp -d)
.venv/bin/python "$C/synthetic.py" --session "$D/synthetic.jsonl"
.venv/bin/python "$C/fit.py" "$D/synthetic.jsonl" "$D/fit"
.venv/bin/python "$C/make_tank_arm.py" "$D/fit" "$D/fit/model"
.venv/bin/python "$C/analyze_trace.py"
.venv/bin/python -m pytest -q tests/test_viscous_urdf_cal.py
ruff check "$C" tests/test_viscous_urdf_cal.py
ruff format --check "$C" tests/test_viscous_urdf_cal.py
```

The supplied PDF and PNG files are ready to print. Regenerating them with `print_targets.py` additionally requires ReportLab, Pillow and Poppler’s `pdftoppm`. The rendered 100 mm bar spans 1,000 pixels at the embedded 254 dpi on both sizes.

Synthetic candidates carry `robot: synthetic` in their provenance. Tonight's real fit needs a new session directory. Review the physical survey, level witnesses, five caliper gaps and fresh checks over the working volume. A small point RMS alone cannot establish worst-case glass clearance, powered sag, accuracy of cables/jaw edges or the mount registration. [Mac procedure](PROCEDURE.md)
