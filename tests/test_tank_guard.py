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
"""Aquarium keep-out: walls refused, floor allowed, paths proven, jumps refused."""

import copy
import json
import math

import numpy as np
import pytest

from makermodslab import tank_geometry as geo, tank_guard as tg

# Resting in the tank at the end of the 2026-11-07 trace (gripper on the floor).
REST = {
    "shoulder_pan": 189.0,
    "shoulder_lift": -201.8,
    "elbow_flex": 169.6,
    "wrist_flex": 47.3,
    "wrist_yaw": 31.3,
    "wrist_roll": -7.2,
    "gripper": -0.5,
}
# Raised and pointing down the tank, clear of everything.
WORK = {
    "shoulder_pan": 182.0,
    "shoulder_lift": -150.0,
    "elbow_flex": 120.0,
    "wrist_flex": 30.0,
    "wrist_yaw": 0.0,
    "wrist_roll": 0.0,
    "gripper": -10.0,
}


@pytest.fixture(scope="module")
def raw():
    return json.loads(tg.DEFAULT_CONFIG.read_text())


@pytest.fixture(scope="module")
def guard(raw):
    return tg.TankGuard(raw)


def variant(raw, guard, **changes):
    config = copy.deepcopy(raw)
    for key, value in changes.items():
        config[key] = value
    return tg.TankGuard(config, guard.model)


def mesh_points(guard, q):
    transforms = geo.forward_kinematics(guard.model, q)
    return {
        link: (guard.model.vertices[link] @ transforms[link][0].T + transforms[link][1]) * 1000
        for link in guard.links
    }


def dense_ok(guard, q0, q1, step_deg=0.05):
    span = max(abs(q1[m] - q0[m]) for m in tg.MOTORS)
    n = max(1, math.ceil(span / step_deg))
    return all(guard.check_pose(tg._lerp(q0, q1, k / n)).ok for k in range(n + 1))


def test_config_loads_with_documented_rules(guard):
    assert all(v is None for v in guard.required["yaw_link"].values())
    assert guard.required["forearm_link"]["left"] == 70.0  # 30 margin + 40 cable padding
    assert guard.required["gripper_left"]["left"] == 30.0  # fingers carry no cables
    # 66 margin from the 106 post face = the heel's same 40 mm behind J1 (2026-11-13 mount)
    assert guard.required["upper_arm_link"]["near"] == 66.0
    assert guard.required["forearm_link"]["near"] == 70.0  # the elbow keeps the full near rule
    assert all(guard.required[link]["floor"] == -10.0 for link in guard.links if link != "yaw_link")


def test_rest_and_work_poses_may_start(guard):
    assert guard.start_gate(REST).ok
    assert guard.start_gate(WORK).ok
    assert guard.outside_limits(REST) == ["shoulder_pan", "shoulder_lift", "gripper"]


def test_start_far_outside_a_limit_is_refused(guard):
    verdict = guard.start_gate({**WORK, "shoulder_pan": 195.0})
    assert not verdict.ok and verdict.face == "joint_limit:shoulder_pan"


def test_a_rest_outside_the_limits_moves_back_in_without_snapping(guard):
    # REST lies outside three soft limits; holding it is not a clip, so the first
    # target never jumps to the limit (every motion <= 15 deg/s, Gavin 2026-11-08).
    assert set(guard.outside_limits(REST)) == {"shoulder_pan", "shoulder_lift", "gripper"}
    hold = guard.limit(REST, REST)
    assert hold.status == "ok" and hold.target == REST
    inward = {**REST, "shoulder_pan": REST["shoulder_pan"] - 0.5}
    assert guard.limit(REST, inward).target == inward
    # Never further out than the accepted pose.
    outward = guard.limit(REST, {**REST, "shoulder_pan": REST["shoulder_pan"] + 1.0})
    assert outward.status == "clamped" and outward.target["shoulder_pan"] == REST["shoulder_pan"]


def test_guarded_targets_may_return_exactly_to_the_start(guard):
    run = tg.GuardedTargets(guard, REST)
    away = {**REST, "shoulder_pan": 186.0, "shoulder_lift": -196.0}
    for _ in range(3):
        run.step(away)
    assert run.target == away
    back = run.step(REST)
    assert back.status == "ok" and back.target == REST
    # Inside the limits, the plain limit still clips.
    near_top = {**WORK, "wrist_roll": 145.0}
    clipped = guard.limit(near_top, {**WORK, "wrist_roll": 155.0}, allowed=WORK)
    assert clipped.status == "clamped" and clipped.target["wrist_roll"] == guard.limits["wrist_roll"][1]


def test_walls_refuse_but_floor_allows_contact(guard):
    # J1 up swings the hand toward +y (left), down toward -y (right).
    assert guard.check_pose(WORK).ok
    assert guard.check_pose({**WORK, "shoulder_pan": 200.0}).face == "left"
    assert guard.check_pose({**WORK, "shoulder_pan": 160.0}).face == "right"
    assert not guard.check_pose({**WORK, "shoulder_pan": 160.0}).ok
    # Resting on the floor, as at the end of the trace: allowed.
    floor_slack = min(v for (link, face), v in guard.clearances(REST).items() if face == "floor")
    # The 11-07 trace's rest; the floor bound moved down 25 mm when Gavin raised the base (2026-11-08).
    assert floor_slack < 45
    assert guard.check_pose(REST).ok


def test_floor_press_is_bounded(guard, raw):
    lowest = min(p[:, 2].min() for p in mesh_points(guard, REST).values())
    floor = guard.planes["floor"]
    near_floor = variant(raw, guard, planes_mm={**raw["planes_mm"], "floor": {"value": lowest - 5.0}})
    assert near_floor.check_pose(REST).ok  # 5 mm above the floor bound: fine
    pressing = variant(raw, guard, planes_mm={**raw["planes_mm"], "floor": {"value": lowest + 30.0}})
    verdict = pressing.check_pose(REST)
    assert not verdict.ok and verdict.face == "floor"  # 30 mm into the floor: refused
    assert floor == -28.0  # the jaws resting on the glass, live run 2026-11-08T213138Z


def test_limit_clamps_at_the_wall_and_the_granted_path_is_clear(guard):
    # Find a J1 swing that reaches the keep-out from a clear pose.
    target = {**WORK, "shoulder_pan": WORK["shoulder_pan"] - 19.0}
    assert not guard.check_pose(target).ok
    decision = guard.limit(WORK, target)
    assert decision.status == "clamped" and decision.reason == "keepout"
    assert 0 < decision.fraction < 1
    assert dense_ok(guard, WORK, decision.target)
    assert guard.check_pose(decision.target).ok


def test_jump_and_wrap_are_refused_not_slid(guard):
    wrapped = {**WORK, "shoulder_pan": WORK["shoulder_pan"] - 360.0}
    decision = guard.limit(WORK, wrapped)
    assert decision.status == "rejected" and decision.reason.startswith("jump:")
    assert decision.target == WORK
    decision = guard.limit(WORK, {**WORK, "wrist_flex": WORK["wrist_flex"] + 25.0})
    assert decision.status == "rejected" and decision.reason == "jump:wrist_flex"


def test_path_through_the_keepout_is_caught_between_clear_endpoints(raw, guard):
    # Far wall pulled in to 500 mm, side walls out of reach: an extended arm
    # at J1 135 and 235 is clear, but at 185 (straight down the tank) it is not.
    planes = {
        **raw["planes_mm"],
        "near": {"value": 500.0},
        "far": {"value": -500.0},
        "left": {"value": 2000.0},
        "right": {"value": -2000.0},
    }
    limits = copy.deepcopy(raw["joint_limits_deg"])
    limits["value"]["shoulder_pan"] = [100.0, 260.0]
    g = variant(raw, guard, planes_mm=planes, joint_limits_deg=limits, max_jump_deg={"value": 200.0})
    reach = {**WORK, "shoulder_lift": -175.0, "elbow_flex": 160.0, "wrist_flex": 15.0}
    q0, q1 = {**reach, "shoulder_pan": 135.0}, {**reach, "shoulder_pan": 235.0}
    assert g.check_pose(q0).ok and g.check_pose(q1).ok
    assert not g.check_pose({**reach, "shoulder_pan": 185.0}).ok
    verdict = g.check_segment(q0, q1)
    assert not verdict.ok and verdict.face == "far"
    decision = g.limit(q0, q1)
    assert decision.status == "clamped" and decision.target["shoulder_pan"] < 185.0
    assert dense_ok(g, q0, decision.target)


def test_motion_bound_covers_real_vertex_motion(guard):
    rng = np.random.default_rng(3)
    for _ in range(40):
        q0 = {m: WORK[m] + rng.uniform(-20, 20) for m in tg.MOTORS}
        q1 = {m: q0[m] + rng.uniform(-3, 3) for m in tg.MOTORS}
        a, b = mesh_points(guard, q0), mesh_points(guard, q1)
        moved = max(np.linalg.norm(a[k] - b[k], axis=1).max() for k in a)
        assert moved <= guard.motion_bound_mm(q0, q1) + 1e-6


def test_accepted_segments_are_dense_clear(guard):
    rng = np.random.default_rng(11)
    accepted = 0
    for _ in range(25):
        q0 = {m: WORK[m] + rng.uniform(-8, 8) for m in tg.MOTORS}
        if not guard.check_pose(q0).ok:
            continue
        q1 = {m: q0[m] + rng.uniform(-15, 15) for m in tg.MOTORS}
        if guard.check_segment(q0, q1).ok:
            accepted += 1
            assert dense_ok(guard, q0, q1, step_deg=0.25)
    assert accepted >= 5


def test_unsafe_previous_and_bad_inputs_are_refused(guard):
    inside_wall = {**WORK, "shoulder_pan": 170.0}
    assert not guard.check_pose(inside_wall).ok
    assert guard.limit(inside_wall, WORK).reason == "previous_unsafe"
    with pytest.raises(ValueError):
        guard.check_pose({k: v for k, v in WORK.items() if k != "gripper"})
    with pytest.raises(ValueError):
        guard.check_pose({**WORK, "elbow_flex": float("nan")})
    with pytest.raises(ValueError):
        tg.GuardedTargets(guard, inside_wall)


def test_bad_config_is_refused(raw, guard):
    with pytest.raises(ValueError):
        variant(raw, guard, planes_mm={**raw["planes_mm"], "left": {"value": -5.0}})
    with pytest.raises(ValueError):
        variant(raw, guard, margins_mm={"value": {**raw["margins_mm"]["value"], "left": -1.0}})
    with pytest.raises(ValueError):
        variant(raw, guard, link_rules={"no_such_link": {"exempt_faces": ["near"]}})


def test_monitor_allows_only_the_tracking_allowance(guard):
    for q in (WORK, REST, {**WORK, "shoulder_pan": 165.0}, {**WORK, "shoulder_pan": 150.0}):
        slack = guard.check_pose(q).slack_mm
        assert guard.monitor(q).ok == (slack >= -guard.tracking_allowance_mm)


def test_guarded_targets_tracks_the_last_accepted_target(guard):
    run = tg.GuardedTargets(guard, WORK)
    first = run.step({**WORK, "elbow_flex": WORK["elbow_flex"] + 2})
    assert first.status == "ok" and run.target == first.target
    wild = run.step({**WORK, "shoulder_pan": WORK["shoulder_pan"] + 300})
    assert wild.reason == "jump:shoulder_pan"  # judged before limit clipping
    assert run.counts == {"ok": 1, "clamped": 0, "rejected": 1}
    assert run.target == first.target


def on_the_line(guard, q, joint, blocked):
    """q with `joint` moved toward `blocked` (a value inside the keep-out) until it sits on the line."""
    clear, inside = q[joint], blocked
    for _ in range(80):
        mid = (clear + inside) / 2
        clear, inside = (mid, inside) if guard.check_pose({**q, joint: mid}).ok else (clear, mid)
    return {**q, joint: clear}


def face_slack(guard, q, face):
    return min(v for (_link, f), v in guard.clearances(q).items() if f == face)


def test_a_pose_on_a_wall_line_can_move_away_but_not_closer(guard):
    # Live 215535Z: a target clamped onto the back-wall line froze the arm, because the grid asked even
    # the starting sample to clear by the gap it covered (up to 2 mm), whichever way the segment went.
    line = on_the_line(guard, WORK, "shoulder_pan", 160.0)
    verdict = guard.check_pose(line)
    assert verdict.face == "right" and 0 <= verdict.slack_mm < 1e-6
    away = {**line, "shoulder_pan": line["shoulder_pan"] + 4.0}
    assert guard.check_segment(line, away).ok
    decision = guard.limit(line, away)
    assert decision.status == "ok" and decision.target == away
    assert dense_ok(guard, line, away)
    closer = {**line, "shoulder_pan": line["shoulder_pan"] - 1.0}
    assert not guard.check_segment(line, closer).ok
    decision = guard.limit(line, closer)
    assert decision.status == "rejected" and decision.reason == "keepout" and decision.target == line


def test_a_mixed_move_leaves_one_face_toward_another_clear_one(guard):
    # Away from the right wall the hand swings toward the left one, which stays clear; the wrist moves too.
    line = on_the_line(guard, WORK, "shoulder_pan", 160.0)
    mixed = {**line, "shoulder_pan": line["shoulder_pan"] + 6.0, "wrist_flex": line["wrist_flex"] - 5.0}
    assert face_slack(guard, mixed, "right") > face_slack(guard, line, "right")
    assert 0 < face_slack(guard, mixed, "left") < face_slack(guard, line, "left")
    assert guard.check_segment(line, mixed).ok
    assert guard.limit(line, mixed).status == "ok"
    assert dense_ok(guard, line, mixed)
    # Away from the right wall but INTO the rim: cut short of the rim, never past it.
    up = {**line, "shoulder_pan": line["shoulder_pan"] + 6.0, "shoulder_lift": line["shoulder_lift"] + 19.0}
    assert guard.check_pose(up).face == "rim" and not guard.check_pose(up).ok
    decision = guard.limit(line, up)
    assert decision.status == "clamped" and 0 < decision.fraction < 1
    assert dense_ok(guard, line, decision.target)


def test_guarded_targets_slide_off_a_wall_they_were_clamped_onto(guard):
    run = tg.GuardedTargets(guard, WORK)
    toward = {**WORK, "shoulder_pan": WORK["shoulder_pan"] - 19.0}
    first = run.step(toward)
    assert first.status == "clamped" and first.reason == "keepout"
    on_line = dict(run.target)
    assert 0 <= guard.check_pose(on_line).slack_mm < 1.0
    again = run.step(toward)  # pressing on: nothing (or a sliver) more, never past the line
    assert again.fraction < 0.05 and guard.check_pose(run.target).ok
    back = run.step({**run.target, "shoulder_pan": run.target["shoulder_pan"] + 3.0})
    assert back.status == "ok" and back.fraction == 1.0


def test_a_start_inside_a_requirement_is_still_refused(guard):
    # No sample can prove a pose already inside a margin does not first get worse (the bound lets every
    # vertex move toward the glass from s = 0), so the stop path holds from there, as it did before.
    inside = {**WORK, "shoulder_pan": 165.0}
    assert guard.check_pose(inside).face == "right" and not guard.check_pose(inside).ok
    away = {**inside, "shoulder_pan": 170.0}
    assert not guard.check_segment(inside, away).ok
