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

from contextlib import suppress
from types import SimpleNamespace

import pytest

from makermodslab.joint_posture_mapping import (
    JOINTS,
    JointPostureMapper,
    PostureWarmup,
    install,
    prepare_test,
)


def angles(value=0.0):
    return dict.fromkeys(JOINTS, value)


def mapper():
    return JointPostureMapper(angles(-1.25), angles(2.0), dict.fromkeys(JOINTS, (-120.0, 120.0)))


def test_unit_angular_gain_and_fixed_offsets_do_not_depend_on_start_pose():
    first, requested, _ = mapper().map(angles(10.0), {"gripper.pos": -7.0})
    second, _, _ = mapper().map(angles(20.0), {"gripper.pos": -7.0})
    for name in JOINTS:
        assert first[f"{name}.pos"] == requested[name] == pytest.approx(-8.0)
        assert second[f"{name}.pos"] - first[f"{name}.pos"] == pytest.approx(-10.0)
    assert first["gripper.pos"] == -7.0


def test_independent_leader_unwrap_is_not_changed_by_shifted_follower_limits():
    m = JointPostureMapper(angles(1), angles(0), dict.fromkeys(JOINTS, (-32.0, 282.0)))
    output, requested, limited = m.map({**angles(), "shoulder_pan": 252.9}, {})
    assert requested["shoulder_pan"] == pytest.approx(-107.1)
    assert output["shoulder_pan.pos"] == -32.0
    assert "shoulder_pan" in limited


def test_warmup_ends_at_five_seconds_even_with_stalled_follower_without_reanchoring():
    now = [100.0]
    warmup = PostureWarmup(clock=lambda: now[0])
    targets = {f"{name}.pos": 80.0 for name in JOINTS}
    for elapsed in (0, 1, 4.999, 5, 10, 1000):
        now[0] = 100.0 + elapsed
        output = warmup.action(targets, angles(), angles(80), [])
        assert output == targets
        assert warmup.status["phase"] == ("warming_up" if elapsed < 5 else "tracking")
        assert warmup.status["max_error_deg"] == 80.0


def test_alignment_does_not_shorten_five_second_warmup_or_restart_it_afterward():
    now = [0.0]
    warmup = PostureWarmup(clock=lambda: now[0])
    targets = {f"{name}.pos": 10.0 for name in JOINTS}
    warmup.action(targets, angles(9), angles(10), [])
    now[0] = 0.6
    assert warmup.action(targets, angles(9), angles(10), []) == targets
    assert warmup.status["phase"] == "warming_up"
    now[0] = 5.0
    assert warmup.action(targets, angles(9), angles(10), []) == targets
    assert warmup.status["phase"] == "tracking"
    now[0] = 6.0
    assert warmup.action(targets, angles(-40), angles(10), []) == targets
    assert warmup.status["phase"] == "tracking"
    assert warmup.status["max_error_deg"] == 50.0


def test_limited_target_error_remains_visible_after_warmup():
    now = [0.0]
    warmup = PostureWarmup(clock=lambda: now[0])
    for t in (0, 2, 5):
        now[0] = t
        warmup.action({f"{n}.pos": 5.0 for n in JOINTS}, angles(), angles(10), ["elbow_flex"])
        assert warmup.status["phase"] == ("warming_up" if t < 5 else "tracking")
        assert warmup.status["max_error_deg"] == 5.0
        assert warmup.status["max_requested_error_deg"] == 10.0
        assert warmup.status["limited_joints"] == ["elbow_flex"]


def test_joint_limit_does_not_keep_warmup_stuck_after_legal_target_is_reached():
    now = [0.0]
    warmup = PostureWarmup(clock=lambda: now[0])
    targets = {f"{n}.pos": 3.0 for n in JOINTS}
    warmup.action(targets, angles(3), angles(0), ["elbow_flex"])
    now[0] = 5.0
    warmup.action(targets, angles(3), angles(0), ["elbow_flex"])
    assert warmup.status["phase"] == "tracking"
    assert warmup.status["max_error_deg"] == 0.0
    assert warmup.status["max_requested_error_deg"] == 3.0
    assert warmup.status["limited_joints"] == ["elbow_flex"]


def test_status_identifies_the_joint_still_preventing_alignment():
    warmup = PostureWarmup()
    targets = {f"{n}.pos": 0.0 for n in JOINTS}
    warmup.action(targets, {**angles(), "shoulder_lift": -7.5}, angles(), [])
    assert warmup.status["worst_joint"] == "shoulder_lift"
    assert warmup.status["max_error_deg"] == 7.5


def test_nonfinite_or_missing_readings_cannot_become_targets():
    for raw in ({}, angles(float("nan")), angles(None)):
        with pytest.raises(ValueError, match="fresh finite"):
            mapper().map(raw, {})


def test_trial_corrects_j1_frame_in_ram_and_forces_slow_cap():
    original = {
        **dict.fromkeys(JOINTS, (-100, 100)),
        "shoulder_pan": (-126.85285584196426, 187.52908634631845),
    }
    request = SimpleNamespace(arm_type="viscous", mode="single", leader_kind="star", speed_limit_deg_s=None)
    robot = SimpleNamespace(config=SimpleNamespace(id="viscous_waterproof_01", joint_limits=original))
    prepare_test(request, robot)
    assert robot.config.joint_limits["shoulder_pan"] == pytest.approx(
        (-32.37285584196425, 282.00908634631844)
    )
    assert original["shoulder_pan"][0] == -126.85285584196426
    assert request.speed_limit_deg_s == 15
    prepare_test(request, robot)  # Already shifted ranges are not shifted again.
    robot.config.id = "viscous_01"
    with pytest.raises(ValueError, match="commissioned waterproof"):
        prepare_test(request, robot)


def test_leader_keeps_existing_read_and_cache_behavior_without_extra_polls():
    calls = []

    def existing_read():
        calls.append("existing read")
        raise OSError("transient missed reply")

    def forbidden_monitor(_ids):
        raise AssertionError("The posture mapper must not poll the leader separately")

    leader = SimpleNamespace(
        config=SimpleNamespace(
            joint_directions=angles(1), joint_offsets=angles(), joint_ids={n: i for i, n in enumerate(JOINTS)}
        ),
        bus=SimpleNamespace(sync_monitor=forbidden_monitor),
        _read_raw_positions=existing_read,
        _last_raw_positions=angles(10),
        is_locked=False,
    )

    def legacy_action():
        with suppress(OSError):
            leader._read_raw_positions()
        # The unchanged driver uses its last known angles after a missed read.
        return {f"{n}.pos": 50.0 for n in JOINTS}

    leader.get_action = legacy_action
    robot = SimpleNamespace(
        config=SimpleNamespace(joint_limits=dict.fromkeys(JOINTS, (-120, 120)), startup_sync_speed_deg=1),
        get_observation=lambda: {f"{n}.pos": 0.0 for n in JOINTS},
    )
    install(robot, leader)
    result = leader.get_action()
    assert calls == ["existing read"]
    assert leader._read_raw_positions is existing_read
    assert result["shoulder_pan.pos"] == pytest.approx(10.0)
    assert leader.posture_mapping_status["phase"] == "warming_up"


def test_session_option_reaches_teleop_request():
    from makermodslab.schemas.sessions import TeleoperationOptions
    from makermodslab.sessions import _build_teleoperation_request

    record = {
        "name": "test",
        "mode": "single",
        "leader_port": "l",
        "follower_port": "f",
        "leader_config": "l",
        "follower_config": "f",
        "right_leader_port": "",
        "right_follower_port": "",
        "right_leader_config": "",
        "right_follower_config": "",
        "arm_type": "viscous",
        "leader_kind": "star",
    }
    request = _build_teleoperation_request(record, TeleoperationOptions(joint_posture_mapping=True))
    assert request.joint_posture_mapping


def test_installed_trial_seeks_absolute_posture_and_preserves_gripper():
    config = SimpleNamespace(
        joint_directions=angles(-1), joint_offsets=angles(), joint_ids={n: i for i, n in enumerate(JOINTS)}
    )
    leader = SimpleNamespace(config=config, is_locked=False, _read_raw_positions=lambda: angles(10))
    leader.bus = SimpleNamespace(
        sync_monitor=lambda ids: {i: SimpleNamespace(reliable=True, angle_deg=10.0) for i in ids}
    )

    def old_action():
        raw = leader._read_raw_positions()
        leader._last_raw_positions = raw
        return {**{f"{n}.pos": -1.4 * raw[n] for n in JOINTS}, "gripper.pos": -9.0}

    leader.get_action = old_action
    robot = SimpleNamespace(
        config=SimpleNamespace(joint_limits=dict.fromkeys(JOINTS, (-120, 120)), startup_sync_speed_deg=1),
        get_observation=lambda: {f"{n}.pos": 0.0 for n in JOINTS},
    )
    install(robot, leader)
    output = leader.get_action()
    assert output["gripper.pos"] == -9.0
    # It must converge toward the fixed -10-degree target, not declare the
    # current zero-degree follower posture to be the matched reference.
    for name in JOINTS:
        assert output[f"{name}.pos"] == pytest.approx(-10.0)
    assert leader.posture_mapping_status["max_error_deg"] == pytest.approx(10.0)
    assert leader.posture_mapping_status["phase"] == "warming_up"
    assert robot.config.startup_sync_speed_deg is None


def test_stalled_follower_can_build_lifting_error_at_the_existing_speed_limit():
    from makermodslab.can_speed_limit import RateLimitedRobstrideBus

    now, writes = [0.0], []
    gains = {n: {"kp": 30.0, "kd": 2.0} for n in JOINTS}
    base = SimpleNamespace(
        _gains=gains,
        _get_motors_list=lambda motors: list(JOINTS),
        _get_motor_name=lambda name: name,
        _query_status_via_clear_fault=lambda name, timeout: (False, SimpleNamespace(data=bytes(8))),
        _decode_motor_state=lambda data: None,
        read=lambda data_name, name: 0.0,
        write=lambda data_name, name, value: writes.append((data_name, name, value)),
        enable_torque=lambda motors, num_retry: None,
    )
    robot = SimpleNamespace(bus=base, config=SimpleNamespace(joint_limits=dict.fromkeys(JOINTS, (-120, 120))))
    bus = RateLimitedRobstrideBus(robot, 15.0, clock=lambda: now[0])
    bus.enable_torque()
    warmup = PostureWarmup(clock=lambda: now[0])
    targets = {f"{n}.pos": 80.0 for n in JOINTS}
    for i in range(1, 53):
        now[0] = i * 0.1
        # Measured position remains zero, simulating gravity/stiction.
        output = warmup.action(targets, angles(), angles(80), [])
        bus.sync_write("Goal_Position", {n: output[f"{n}.pos"] for n in JOINTS})
        latest = {name: value for kind, name, value in writes if kind == "Goal_Position"}
        for name in JOINTS:
            assert latest[name] == pytest.approx(15.0 * now[0])
    assert latest["shoulder_lift"] > 1.0
    assert gains["shoulder_lift"] == {"kp": 30.0, "kd": 2.0}
    assert warmup.status["phase"] == "tracking"
