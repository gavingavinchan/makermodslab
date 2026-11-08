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
"""The Dream-1 runner's route from the tank rest up to the start pose and back.

Pure geometry against the merged tank keep-out guard: no hardware, no network.
Gavin (2026-11-08): "for start position, you probably need to lift the elbow
up first". Leg 1 lifts shoulder and elbow only, leg 2 goes to the start.

Minimum slack along the route at the TRACE_POSE rest (mm, clearance minus the
guard's requirement; floor slack is above floor bound - 10 mm press):
near 9.4 (upper-arm heel, the rest's own), floor 16.0 (forearm, just off the
rest, rising from there), rim 43.3 (wrist pitch, at the start pose itself),
left 46.0, right 64.6, far 55.5. Over the perturbed rests the worst are near
7.5, floor 10.9, left 8.0, all at the rest itself: the route never takes the
heel closer to the back wall, or any link lower, than the rest already is.

Guard stream (retimed at 15 deg/s): every substep is `clamped`, never
`rejected`. The gripper rests at -0.45, outside its -2.92 limit, so the guard
clips it on every substep; arm joints are clipped only in leg 1's leading
substeps (shoulder_pan down to 187.5, shoulder_lift up to -199.4).
Run: .venv/bin/python -m pytest tests/test_viscous_dream_approach.py -q
"""

import json
import sys
from pathlib import Path

import pytest

from makermodslab import tank_guard as tg

REAL = Path(__file__).resolve().parents[1] / "scripts" / "viscous-dream1" / "real"
sys.path.insert(0, str(REAL))

import dream_real  # noqa: E402
import dream_real_core as core  # noqa: E402

ARM_DEG_S, GRIP_DEG_S = 15.0, 60.0  # the runner's caps (README)
START = json.loads((REAL / "runs/start_pose/20261108T183949Z.json").read_text())["motor_deg"]
TRACE = dict(dream_real.TRACE_POSE)
# Rests a few degrees off the trace's, all inside the 5 deg start tolerance and
# accepted by the start gate (lower lift/elbow combinations put a link through
# the floor, which a resting arm cannot be).
DELTAS = (
    {
        "shoulder_pan": 3.0,
        "shoulder_lift": 3.0,
        "elbow_flex": -4.0,
        "wrist_flex": 4.0,
        "wrist_yaw": -5.0,
        "wrist_roll": 5.0,
    },
    {
        "shoulder_pan": -4.0,
        "shoulder_lift": 2.0,
        "elbow_flex": 4.0,
        "wrist_flex": -4.0,
        "wrist_yaw": 5.0,
        "wrist_roll": -5.0,
    },
    {
        "shoulder_pan": 3.4,
        "shoulder_lift": -2.5,
        "elbow_flex": 3.0,
        "wrist_flex": -3.0,
        "wrist_yaw": -5.0,
        "wrist_roll": -5.0,
    },
    {
        "shoulder_pan": -5.0,
        "shoulder_lift": 5.0,
        "elbow_flex": 5.0,
        "wrist_flex": 5.0,
        "wrist_yaw": 5.0,
        "wrist_roll": 5.0,
    },
    {
        "shoulder_pan": -3.0,
        "shoulder_lift": -2.5,
        "elbow_flex": 5.0,
        "wrist_flex": -5.0,
        "wrist_yaw": -5.0,
        "wrist_roll": 3.0,
    },
    {
        "shoulder_pan": 2.0,
        "shoulder_lift": 4.0,
        "elbow_flex": -5.0,
        "wrist_flex": -5.0,
        "wrist_yaw": 5.0,
        "wrist_roll": -3.0,
    },
)
RESTS = {"trace": TRACE} | {
    f"perturbed{i}": {m: TRACE[m] + d.get(m, 0.0) for m in core.MOTORS} for i, d in enumerate(DELTAS)
}


@pytest.fixture(scope="module")
def guard():
    return tg.TankGuard.from_file()


def legs(route):
    return list(zip(route, route[1:], strict=False))


def face_slack(guard, poses):
    out = {}
    for q in poses:
        for (_, face), slack in guard.clearances(q).items():
            out[face] = min(out.get(face, float("inf")), slack)
    return out


def stream(guard, route):
    """Send the route through GuardedTargets as the runner does; (decisions, leg index per decision)."""
    targets = tg.GuardedTargets(guard, route[0])
    decisions, leg_of = [], []
    for i, (a, b) in enumerate(legs(route)):
        for step in core.retime(a, b, arm_deg_s=ARM_DEG_S, grip_deg_s=GRIP_DEG_S):
            decisions.append(targets.step(step))
            leg_of.append(i)
    return targets, decisions, leg_of


@pytest.mark.parametrize("name", RESTS)
def test_route_shape_and_ends(name):
    rest = RESTS[name]
    route = core.approach_waypoints(rest, START)
    assert 1 <= len(route) <= 4
    assert route[-1] == {**{m: START[m] for m in core.ARM}, "gripper": rest["gripper"]}
    assert all(wp["gripper"] == rest["gripper"] for wp in route)
    lift = route[0]
    # Leg 1 lifts the elbow: shoulder up, elbow folded, pan and wrist held
    # (only clipped into the limits), so nothing turns while the hand is down.
    assert lift["shoulder_lift"] - rest["shoulder_lift"] == pytest.approx(core.APPROACH_LIFT_DEG)
    assert rest["elbow_flex"] - lift["elbow_flex"] == pytest.approx(core.APPROACH_FOLD_DEG)
    limits = core.keepout_limits()
    for m in ("shoulder_pan", "wrist_flex", "wrist_yaw", "wrist_roll"):
        lo, hi = limits[m]
        assert lift[m] == pytest.approx(min(hi - 1, max(lo + 1, rest[m])))
    assert all(limits[m][0] + 1 - 1e-9 <= lift[m] <= limits[m][1] - 1 + 1e-9 for m in core.ARM)


@pytest.mark.parametrize("name", RESTS)
def test_every_leg_is_proven_clear(guard, name):
    rest = RESTS[name]
    assert guard.start_gate(rest).ok
    route = [rest, *core.approach_waypoints(rest, START)]
    for a, b in legs(route):
        verdict = guard.check_segment(a, b)
        assert verdict.ok, (name, verdict)


@pytest.mark.parametrize("name", RESTS)
def test_route_never_closer_to_back_wall_or_floor_than_rest(guard, name):
    rest = RESTS[name]
    route = [rest, *core.approach_waypoints(rest, START)]
    poses = [q for a, b in legs(route) for q in core.retime(a, b, arm_deg_s=ARM_DEG_S, grip_deg_s=GRIP_DEG_S)]
    along, at_rest = face_slack(guard, poses), face_slack(guard, [rest])
    assert along["near"] >= at_rest["near"] - 0.5
    assert along["floor"] >= at_rest["floor"]  # leg 1 lifts off; nothing presses deeper than the rest
    assert min(along.values()) > 5.0


def test_trace_rest_slack_per_face(guard):
    route = [TRACE, *core.approach_waypoints(TRACE, START)]
    poses = [q for a, b in legs(route) for q in core.retime(a, b, arm_deg_s=ARM_DEG_S, grip_deg_s=GRIP_DEG_S)]
    slack = face_slack(guard, poses)
    # The numbers the module docstring states (sampled at the 20 Hz substeps).
    assert slack["near"] > 9.0
    assert slack["floor"] > 15.5
    assert slack["rim"] > 43.0
    assert slack["left"] > 45.0
    assert slack["right"] > 60.0
    assert slack["far"] > 50.0
    # The lift waypoint itself is well clear of the floor and the rim.
    lifted = face_slack(guard, [route[1]])
    assert lifted["floor"] > 100.0 and lifted["rim"] > 100.0


@pytest.mark.parametrize("name", RESTS)
def test_guard_stream_never_rejects(guard, name):
    rest = RESTS[name]
    route = [rest, *core.approach_waypoints(rest, START)]
    targets, decisions, leg_of = stream(guard, route)
    assert targets.counts["rejected"] == 0
    assert {d.reason for d in decisions} <= {"", "joint_limit"}
    # Arm joints clip only for the rest's own limit overshoot: a run of leading
    # substeps in leg 1, never later. The gripper (resting beyond its limit)
    # clips throughout, which is all a `clamped` after that means.
    arm_clipped = [i for i, d in enumerate(decisions) if set(d.clipped_joints) - {"gripper"}]
    assert arm_clipped == list(range(len(arm_clipped)))
    assert all(leg_of[i] == 0 for i in arm_clipped)
    assert {j for d in decisions for j in d.clipped_joints} <= {"shoulder_pan", "shoulder_lift", "gripper"}
    # The arm ends at the start pose (gripper as the guard clipped it).
    assert all(targets.target[m] == pytest.approx(START[m]) for m in core.ARM)


def test_trace_first_reading_rest_needs_the_guards_partial_grant(guard):
    # The trace's first torque-off reading (pan 191.2, yaw 18.1) sits 0.66 mm
    # inside the left wall's requirement, so no straight leg from it can pass
    # check_segment's sample allowance. The guard still grants partial steps
    # ("keepout" clamps) and is clear within a dozen substeps: never rejected.
    rest = {
        "shoulder_pan": 191.209,
        "shoulder_lift": -201.738,
        "elbow_flex": 168.043,
        "wrist_flex": 46.871,
        "wrist_yaw": 18.122,
        "wrist_roll": -3.77,
        "gripper": -0.517,
    }
    route = [rest, *core.approach_waypoints(rest, START)]
    targets, decisions, _ = stream(guard, route)
    assert targets.counts["rejected"] == 0
    keepout = [i for i, d in enumerate(decisions) if d.reason == "keepout"]
    # Within the first ~0.75 s: pan now eases inward at the leg's pace instead of
    # snapping to its limit (tank_guard, 2026-11-08), so the partial grants last
    # longer than with the old snap, but they still end early and never reject.
    assert keepout and max(keepout) < 15
    assert all(targets.target[m] == pytest.approx(START[m]) for m in core.ARM)


@pytest.mark.parametrize("name", RESTS)
def test_return_is_the_approach_reversed(guard, name):
    rest = RESTS[name]
    start = {**START, "gripper": -10.0}  # a different grip at the start: the return keeps it
    there = [rest, *core.approach_waypoints(rest, start)]
    back = [start, *core.return_waypoints(rest, start)]
    assert [{m: q[m] for m in core.ARM} for q in back] == [{m: q[m] for m in core.ARM} for q in there[::-1]]
    assert all(wp["gripper"] == -10.0 for wp in back)
    # With the same grip at both ends the return is the approach reversed, exactly.
    same = {**START, "gripper": rest["gripper"]}
    assert core.return_waypoints(rest, same) == [*core.approach_waypoints(rest, same)[-2::-1], rest]
    targets, _, _ = stream(guard, back)
    assert targets.counts["rejected"] == 0


def test_rest_already_at_start_is_one_leg():
    start = {**START, "gripper": -10.0}
    assert core.approach_waypoints(start, start) == [start]
    assert core.return_waypoints(start, start) == [start]
