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

from makermodslab import tank_geometry as geo
from makermodslab import tank_guard as tg

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
    assert guard.required["upper_arm_link"]["near"] == 0.0
    assert guard.required["forearm_link"]["near"] == 70.0  # the elbow keeps the full near rule
    assert all(guard.required[link]["floor"] == -10.0 for link in guard.links if link != "yaw_link")


def test_rest_and_work_poses_may_start(guard):
    assert guard.start_gate(REST).ok
    assert guard.start_gate(WORK).ok
    assert guard.outside_limits(REST) == ["shoulder_pan", "shoulder_lift", "gripper"]


def test_start_far_outside_a_limit_is_refused(guard):
    verdict = guard.start_gate({**WORK, "shoulder_pan": 195.0})
    assert not verdict.ok and verdict.face == "joint_limit:shoulder_pan"


def test_first_target_from_rest_is_clipped_into_limits(guard):
    decision = guard.limit(REST, REST)
    assert decision.status == "clamped"
    assert set(decision.clipped_joints) == {"shoulder_pan", "shoulder_lift", "gripper"}
    assert guard.outside_limits(decision.target) == []


def test_walls_refuse_but_floor_allows_contact(guard):
    # J1 up swings the hand toward +y (left), down toward -y (right).
    assert guard.check_pose(WORK).ok
    assert guard.check_pose({**WORK, "shoulder_pan": 200.0}).face == "left"
    assert guard.check_pose({**WORK, "shoulder_pan": 160.0}).face == "right"
    assert not guard.check_pose({**WORK, "shoulder_pan": 160.0}).ok
    # Resting on the floor, as at the end of the trace: allowed.
    floor_slack = min(v for (link, face), v in guard.clearances(REST).items() if face == "floor")
    assert floor_slack < 20  # the lowest vertex is within ~5 mm of the floor bound
    assert guard.check_pose(REST).ok


def test_floor_press_is_bounded(guard, raw):
    lowest = min(p[:, 2].min() for p in mesh_points(guard, REST).values())
    floor = guard.planes["floor"]
    near_floor = variant(raw, guard, planes_mm={**raw["planes_mm"], "floor": {"value": lowest - 5.0}})
    assert near_floor.check_pose(REST).ok  # 5 mm above the floor bound: fine
    pressing = variant(raw, guard, planes_mm={**raw["planes_mm"], "floor": {"value": lowest + 30.0}})
    verdict = pressing.check_pose(REST)
    assert not verdict.ok and verdict.face == "floor"  # 30 mm into the floor: refused
    assert floor == -6.0


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
