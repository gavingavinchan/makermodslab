<!-- Copyright 2026 MakerMods. All rights reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
-->

# Tank autocal: the arm finds the aquarium by pressing on it

Gavin (2026-11-18): "Auto-calibrate the tank walls ... touch the bottom, touch the sides left and
right ... the back post will always be flush to the back of the glass ... I want it done
autonomously ... keep it a separate piece: it is not necessary once we know the exact tank size
from a measuring tape. Not a very important tool, just in case."

So this is a **standalone, opt-in convenience tool**. Nothing in the runner, the astra harness or
the tank guard imports it (a test enforces that). Its only output is a **candidate** tank geometry
and a **proposed** keep-out, both marked `pending Gavin's approval`, written into its run folder.
It never touches `tank.json`, the keep-out config or the Lab's tank measurement. The tape-measure
path (or the hand-guided **Measure tank** flow in the Lab) remains the primary way; this tool is a
cross-check or a convenience when the mount has moved.

## How it works

**The method (Gavin's design, 2026-11-18).** "You don't have to detect the exact point of contact.
Press on it, and since the arm is compliant (MIT position control), it bends against the surface.
Through the URDF you can see the arm is hitting something ... Touch a few points and you know the
plane."

1. **Probe point: the closed jaw's fingertips**, the same probe as the hand-guided measurement.
   They are the arm's mesh-modelled extremity, rigid with the wrist when closed, carry no cables
   (their padding is 0, every other link's is the unmeasured 40 mm), and the existing fit already
   reasons in them. The *contact point* of a press is the finger vertex deepest along the surface
   normal (measured joints, CAD mesh), not the fixed tip point, because a tilted jaw touches with
   its edge (Gavin's 2026-11-08 trace). A jaw face would add the gripper slide's uncertainty; a
   printed probe would need a new mount and URDF entry.
2. **Back plane: not probed.** The truss post is flush on the back glass (CAD: J1 118 mm from the
   glass, the post's bearing point at y = +10 mm). It goes to `fit_tank` as one synthetic `near`
   touch labelled as such.
3. **Probe plan** (`planner.py`). 4 floor touches spread over ~160 x 80 mm, 3 per long wall spread
   200 mm along the tank (fit_tank needs 150 mm), and 3 held-out checks (`check:floor`,
   `check:left`, `check:right`), plus spares. For each, the planner searches approach tilts
   (0-45 deg off the normal) and azimuths, solves the whole line (lift pose, standoff, nodes every
   4 mm to the deepest allowed command) with IK on the guard's own FK, and refuses unless:
   - every node is inside the soft limits and joint-continuous;
   - the lift and standoff pass the **normal** keep-out and the harness's own 15 deg/s route
     planner can get there from the hub (the arm file's rest) and back;
   - the press line passes the **probe keep-out**: the normal one, except that against the probed
     face only, the fingers may be commanded up to the window plus the deep press past the plane,
     and every other link must keep its cable padding + 15 mm in the real (stopped) arm;
   - the fingers lead every other link plus padding toward the glass by >= 15 mm, and stay >= 15 mm
     off every other face (a wall probe must not find the floor);
   - the arm is soft enough along the normal (see Force below).
   Each probe is re-planned from the touches before it: the standoff and the search window are the
   plane uncertainty + 15 mm (J1 +-30 mm off centre, +-3 deg yaw, +-10 mm floor at first, a few mm
   once a wall has a touch).
4. **One press** (`runner.py`, `press.py`), all at <= 15 deg/s and every trip on:
   - harness route to the lift pose (60 mm above the standoff);
   - **sag compensation**: hold, measure the MIT standing error (cmd - measured), add it to the
     command, twice; then down to the standoff and once more. Wet sag is 27-53 mm at the hand, so
     without this a floor "standoff" would already be on the glass;
   - **baseline**: 1 s hold at the standoff, then the first 6 mm of the approach: the free-space
     divergence and torques at this very pose;
   - **approach at 3 mm/s** along the normal. Primary detection: the divergence
     `n . (p_cmd - p_meas)` over the baseline by max(3 mm, 6 sigma) for 5 ticks AND rising >= 0.5
     mm per mm of command. Secondary: any joint torque 1.5 Nm over its baseline stops the advance
     (a contact that does not stop the jaw is a false contact). Nothing within the window
     (expected plane + window): the probe is abandoned, never pushed further;
   - **two presses**: d1 then d2 past the onset (8 / 14 mm, less where the arm is stiff), each
     held 1.2 s; every settled measured-joint sample is a point on the plane;
   - **verdict**: rigid glass gives the same plane point at both depths (<= 1.5 mm apart along the
     normal) while the divergence grows with the extra depth (>= 0.6x); a cable, an object or a
     link other than the jaw gives way or moves. The torque-implied force must agree with
     stiffness x divergence (0.25-4x). Lateral sliding along the glass is fine and is recorded;
   - retreat at 8 mm/s, standoff, lift (compensation removed), harness route to the hub.
5. **Fit** (`fitting.py`): the existing `makermodslab.tank_geometry.fit_tank` (planes, shared yaw,
   held-out checks, its 3 mm residual and width gates), plus a leave-worst-out pass: a touch the
   fit cannot put on the glass (a rigid object passes the press test) is dropped and re-probed at
   a spare location. Then the placement report (each plane, the yaw, the residuals, where J1 sits:
   off the centre line, to the back glass, above the floor), flags (J1 > 15 mm off centre, yaw >
   1.5 deg, any plane > 10 mm from the current keep-out), and the two candidate files.

**Force.** A press commanded d past rigid glass pushes with about K_n x d, where K_n is the MIT
joint stiffness mapped to the contact point along the normal (`geometry.normal_stiffness`:
1 / n^T J K^-1 J^T n). The planner picks d2 = min(14 mm, 8 N / K_n) and refuses a pose where that
is under 6 mm (the arm stretched along the normal: near-singular, K_n blows up). Probe-session
gains are softened for this tool only: pan 30/2, shoulder_lift and elbow 60/3 (Gavin has run 60/3
in water), wrists 30/2. If the divergence detector failed entirely, the torque backstop stops the
advance at 1.5 Nm on the most loaded joint: the plan reports that force per probe.

**Every exit is the harness's stop path**: a trip, an exception, a refused move, the end of the
run, or the red button (`stop_button.py --pid-file`, which now also recognises `autocal.py run`;
nothing new signals the process). Re-anchor, gripper limp, legs back along the trail (lift,
standoff, hub, start) at <= 5 deg/s with each leg's path checked, all torque off. A leg that is not
provably clear, a limp or a silent motor HOLDS until the second press, as everywhere else.

## Commands

From the repo root (the branch's worktree).

```bash
# Dry plan: every probe pose and its verdict against the nominal tank. No arm. Lab venv is fine.
.venv/bin/python scripts/viscous-tank-autocal/autocal.py plan [--spares] [--out /tmp/plan.json]

# Simulation (gavin-linux only; MuJoCo lives in ~/dreamtrial/.venv, never the Lab venv)
MUJOCO_GL=egl ~/dreamtrial/.venv/bin/python scripts/viscous-tank-autocal/autocal.py sim \
  --backend mujoco --tanks 20 --workers 20 --video /tmp/autocal.mp4
MUJOCO_GL=egl ~/dreamtrial/.venv/bin/python scripts/viscous-tank-autocal/autocal.py sim \
  --backend mujoco --scenario cable rock floor_trap buckle model_error red_button second_signal stale limp missing_wall
# The numpy fake (fast, any venv): --backend fake

# Tests (Lab venv)
.venv/bin/python -m pytest tests/test_viscous_tank_autocal.py -q
```

**The real arm** (Gavin's Mac; Claude starts it only with Gavin's explicit go). Arm supported at
rest lying on the tank floor, torque off, the post flush on the back glass, the Lab idle. Red
button first, then:

```bash
cd "$HOME/wt/tank-autocal"
source "$HOME/Documents/makermodslab-viscous/.venv/bin/activate"
export MAKERMODSLAB_HOME="$HOME/.makermods/makermodslab-viscous"
python scripts/viscous-dream1/real/stop_button.py --pid-file /tmp/viscous-autocal.pid &

# 1. Unarmed (default): opens the bus without the energizing handshake, reads the start pose,
#    checks the start gate, plans every probe from there, closes. No torque.
python scripts/viscous-tank-autocal/autocal.py run --pid-file /tmp/viscous-autocal.pid

# 2. Armed, after reviewing step 1:
python scripts/viscous-tank-autocal/autocal.py run --armed --pid-file /tmp/viscous-autocal.pid
```

Output: `~/.makermods/makermodslab-viscous/run-data/autocal-<UTC>/` with `autocal-result.json`
(placement report, flags, touches, rejections, peaks), `probes.json` (every probe's plan, sag
offsets, baseline, detection and both presses), `ticks.jsonl` (every reading), `stop.json`, and,
for a valid fit, `candidate-tank.PENDING-APPROVAL.json` and
`candidate-keepout.PENDING-APPROVAL.json`.

## Before a powered run, Gavin must check

1. **The URDF and the TCP feed straight into the planes.** The planes are FK of measured joints:
   every joint-zero, link-length or mesh error moves them one-for-one, and nothing here can see
   it. The arm file's TCP (132 mm out along the flange) is 34 mm short of the CAD fingertip the
   probe uses (166 mm); this tool uses the CAD fingertip (as the hand-guided measurement does),
   but if the CAD fingers are not the real fingers, every plane is off by that error along its
   normal. In sim, 0.5 deg zero errors on three joints moved the planes by [SIM_MODEL_ERROR].
   Cross-check the result against a tape measure once before trusting it.
2. **The wet sag.** The compensation assumes the standing error is steady over the 60 mm descent
   and the approach. It is in the sim; in the water it has been 27-53 mm at the hand and has
   changed with the heat. Watch the first probe's `offset_deg` in `probes.json` (refused over
   10 deg).
3. **Structural flex and backlash are invisible to the encoders** and bias every press point
   by roughly force / structural stiffness at the lever. In sim (300-600 Nm/rad, 0.1 deg play)
   that was [SIM_FLEX]. Unknown on the real arm.
4. **False contacts**: a soft or movable thing is rejected by the two-depth test; a RIGID thing
   (the post, a heavy object) passes it and is caught only by the fit (leave-worst-out) or by
   the geometry checks. Clear the tank of objects and keep cables off the walls.
5. **Wall-contact torque is near the detection floor.** A 3-4 N wall press makes only
   [SIM_WALL_TORQUE] on the pan joint against ~0.1 Nm static friction and ~0.2 Nm model error;
   that is why the divergence, not the torque, is the primary signal, and why the torque check
   is only a loose cross-check (0.25-4x).
6. **The probe gains (pan 30/2) have not run on hardware**; 60/3 on lift/elbow has. The return
   uses the same gains.
7. **The post must really be flush.** The back plane is never measured; if the post stands off
   the glass, the near plane (and with it the fitted yaw only slightly) is off by that gap.

## Relation to the tape-measure path

The tape measure (or the Lab's hand-guided **Measure tank**, torque off) is the primary path and
needs none of this. Use autocal after the mount moves, to cross-check a tape measurement, or
when no one is there to guide the arm. Either way the numbers reach the keep-out only by hand,
after Gavin approves them.
