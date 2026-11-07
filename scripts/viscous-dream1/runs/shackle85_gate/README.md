# Pre-print gate: 85° shackle layout, whole arm, end to end (2026-11-07)

Offline only: no hardware, printing, uploads or billed calls. Fixtures are the holder
session's committed `tilt_85deg` exports (`0de8facd`). Script:
`scripts/viscous-dream1/shackle85_gate.py` (python-fcl exact triangle-mesh distances,
MuJoCo arm model, least-squares IK with J5 = 0). Kinematic, not dynamic: no forces,
friction, torque or drag. The pin is moved kinematically: held means rigid to the
hand, released means fixed, screwing means rotation about its axis plus 2.5 mm pitch.

## Verdict

- **CAD: no revision needed.** The holder, ties and cup clear the whole arm. The hand
  never comes within 20 mm of the holder or ties at any sample. The closest hand
  approach to the cup is 6.67 mm (the fingers at the shank grasp), the designed lip
  clearance.
- **Committed placement (cup tip 500 mm, bore s 575): FAIL.**
  - The arm cannot pull the pin out of the cup. Axial extraction stops after 29 of
    40 mm, the tip only 2 mm past the 27 mm bore, with the wrist pitch (J4) at its
    1.606 rad limit.
  - Every pre-grasp from which the hand can descend onto the shank breaks the water
    line (40 mm above the grasp: 4.1 mm over).
  - Placement only; the parts don't change.
- **Recommended placement: PASS** (1246 samples, sequence completed).
  - Cup pin tip at **565 mm** from J1, azimuth +15° (toward sim +X): plane
    (s, w) = (545.75, 146.23) mm, tip 80 mm up, cup floor origin at
    (545.75, 146.23, 0) mm, rotation +15°.
  - Shackle bore midpoint at **s 600**, h 98.48. The holder and ties move with it.
  - Margins:
    - water line: 15.8 mm under at worst (during the transfer);
    - wall: 3.4 mm;
    - elbow: at least 110 mm ahead of J1;
    - joints: at least 10.2° from every limit;
    - no self-collision.

## Placement window (`placement_sweep.json`, `--summarise`)

| cup tip (holder at s 600) | verdict | binding |
|---|---|---|
| 500 | FAIL | extraction at the J4 limit; pre-grasp 4.1 mm above the water |
| 525 | PASS | J4 margin only 1.4° at extraction |
| 540 / 565 / 590 / 600 | PASS | J4 5.4° / 10.2° / 10.2° / 10.2° |

| bore s (cup at 565) | verdict | water margin (worst) |
|---|---|---|
| 550 / 560 | FAIL | -4.0 / -0.6 mm (above the water during transfer and insert) |
| 565 / 575 / 590 | PASS | 1.1 / 4.9 / 11.1 mm |
| 600 / 610 / 625 | PASS | 15.8 / 17.6 / 17.6 mm |
| 635 / 650 | FAIL | can't finish the shank push (J4 at its limit) |

So the holder tolerates bore s 565–625: -35/+25 mm around 600. The cup tolerates
tip 525–600 (540+ for comfortable J4). Mark both spots in the tank.

The orientation session's screen had 500 mm reachable and 625 mm not seatable. Both
differ here because this gate runs the whole sequence (extraction, transfer) with the
grips the fingers actually allow (below).

## Grip findings (gripper strategy, not CAD)

- **The nominal seated head grip collides with the pin itself.** That grip puts the
  TCP at the middle of the head, 96.9 mm from the tip. The flat finger faces run 34.3
  mm past the TCP, and closed on the 9.2 mm tab they sit inside the collar's 13.8 mm
  radius. Result: fingertips 21.2 mm below the collar top, 4.44 mm into the collar,
  1.7 mm into the clear ear (`nominal_head_grip_seated` in `gate_report.json`).
- **Gated head grip:** TCP 120.15 mm from the tip, fingertips 2 mm above the collar.
  The distal ~22 mm of the flat finger faces hold the tab; the pad is 12 mm beyond
  the pin's end. The fingertips then stay above the clear ear at every depth. Is
  that grip strong enough on the real fingers? Gavin to judge.
- **Cup:** grasp the shank 68 mm from the tip, roll 0. Fingertips sit 6.67 mm off the
  cup (the designed lip clearance).
  - Come down onto the shank from 40 mm straight above, open 26 mm per side. An axial
    approach from beyond the pin head would start about 75 mm back, past J4's limit.
  - Pull out 40 mm along the axis at roll 0 before any other motion.
- **Shank push:** the fingertips stop 2.97 mm off the clear ear's face (3 mm designed).
  The pin clears the shackle's own bore by 0.30 mm when exactly coaxial, so alignment
  matters on the real part.
- **Tightening:** 90° strokes (roll 0→90) with resets at 18 mm per side. The tab is
  2-fold symmetric, so the jaws alternate between its flats (9.2 mm) and its edges
  (24 mm). That needs no roll past 90° and no full opening near the holder.
  - The pin's tab should start with its flats facing the jaws: set it so in the cup.
  - Open fingers clear the tab by 3.57 mm during resets.
  - 31 strokes from the tapped mouth to seated.

## Cameras (provisional)

At the start pose (TCP 460 mm out, 120 mm up, J1 -20°, hand level, arm top 262.6 mm):
- The wrist camera sees all of the shackle and 78% of the cup.
- The overhead camera is the old Dream-station camera, where the parts are about
  33 × 54 and 23 × 48 px in a 640 × 360 image (half that at Dream's 320 × 176). It
  gets replaced by the real camera poses measured at the tank.

## Files

- `cup_r565_bore_s600/`: recommended placement. `gate_report.json`,
  `gate_samples.json.gz` (every sample: joints, arm top, wall, every clearance) and
  `contact_sheet.jpg` (side, 3/4, wrist, overhead at each stage, plus the nominal
  head grip).
- `cup_r500_bore_s575/`: the committed placement, failing.
- `cup_r*_bore_s*/`: the sweep.
