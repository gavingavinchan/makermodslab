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

"""Hardware-free checks of outgoing target bounds and pre-enable sequencing."""

from types import SimpleNamespace

import pytest
from pydantic import ValidationError

from makermodslab.can_speed_limit import RateLimitedRobstrideBus


class Bus:
    def __init__(self):
        self.motors = {"a": object(), "b": object()}
        self.positions = {"a": 10.0, "b": -10.0}
        self._gains = {name: {"kp": 20.0, "kd": 0.5} for name in self.motors}
        self.events = []
        self.failed_read = None
        self.failed_write = False

    def _get_motors_list(self, motors):
        return list(self.motors) if motors is None else [motors] if isinstance(motors, str) else motors

    def _get_motor_name(self, motor):
        if motor not in self.motors:
            raise KeyError(motor)
        return motor

    def _query_status_via_clear_fault(self, name, timeout):
        self.events.append(("read", name))
        return False, None if name == self.failed_read else SimpleNamespace(data=bytes(8))

    def _decode_motor_state(self, data):
        pass

    def read(self, data_name, name):
        return self.positions[name]

    def write(self, data_name, name, value):
        if self.failed_write:
            raise OSError("transmit failed")
        self.events.append((data_name, name, value))

    def enable_torque(self, motors, num_retry=0):
        self.events.append(("enable", motors))

    def sync_write(self, data_name, values):
        self.events.append((data_name, values))


def setup():
    base = Bus()
    now = [0.0]
    robot = SimpleNamespace(
        bus=base,
        config=SimpleNamespace(joint_limits={"a": (-90, 90), "b": (-90, 90)}),
        _turn_offset={"a": 0, "b": 0},
    )
    bus = RateLimitedRobstrideBus(robot, 7.5, clock=lambda: now[0])
    return bus, base, robot, now


def test_every_pose_is_read_before_preloading_and_enabling():
    bus, base, _, _ = setup()
    bus.enable_torque()
    assert base.events == [
        ("read", "a"),
        ("read", "b"),
        ("Goal_Position", "a", 10.0),
        ("Goal_Position", "b", -10.0),
        ("enable", None),
    ]


@pytest.mark.parametrize("elapsed,step", [(0, 0), (0.02, 0.15), (1 / 30, 0.25), (10, 0.75)])
def test_7_5_degree_limit_and_no_jump_after_a_pause(elapsed, step):
    bus, base, _, now = setup()
    bus.enable_torque()
    now[0] = elapsed
    bus.sync_write("Goal_Position", {"a": 80, "b": -80})
    assert base.events[-2:] == [
        ("Goal_Position", "a", pytest.approx(10 + step)),
        ("Goal_Position", "b", pytest.approx(-10 - step)),
    ]


def test_fast_updates_and_reversals_cannot_accumulate_extra_travel():
    bus, base, _, now = setup()
    bus.enable_torque()
    targets = []
    for i in range(1, 101):
        now[0] = i * 0.001
        bus.write("Goal_Position", "a", 80)
        targets.append(base.events[-1][2])
    assert targets[-1] == pytest.approx(10.75)
    now[0] += 0.02
    bus.write("Goal_Position", "a", -80)
    assert base.events[-1][2] == pytest.approx(10.6)


def test_missing_start_reply_prevents_all_preloads_and_enables():
    bus, base, _, _ = setup()
    base.failed_read = "b"
    with pytest.raises(ConnectionError, match="fresh"):
        bus.enable_torque()
    assert base.events == [("read", "a"), ("read", "b")]


@pytest.mark.parametrize("start", [92.9, 120.0])
def test_start_outside_a_limit_eases_inside_without_a_jump(start):
    bus, base, _, now = setup()
    base.positions["a"] = start
    bus.enable_torque()
    assert ("Goal_Position", "a", start) in base.events
    now[0] = 0.02
    bus.write("Goal_Position", "a", 90)
    assert base.events[-1][2] == pytest.approx(start - 0.15)


def test_full_turn_offset_is_accounted_for_without_changing_motor_coordinates():
    bus, base, robot, now = setup()
    base.positions["a"] = 370
    robot._turn_offset["a"] = -360
    bus.enable_torque()
    now[0] = 0.02
    bus.write("Goal_Position", "a", 400)
    assert base.events[-1][2] == pytest.approx(370.15)


def test_failed_transmit_does_not_advance_last_sent_target():
    bus, base, _, now = setup()
    bus.enable_torque()
    now[0] = 0.02
    base.failed_write = True
    with pytest.raises(OSError):
        bus.write("Goal_Position", "a", 80)
    base.failed_write = False
    bus.write("Goal_Position", "a", 80)
    assert base.events[-1][2] == pytest.approx(10.15)


@pytest.mark.parametrize("batch", [False, True])
def test_normal_gripper_still_requires_fresh_initialization(batch):
    bus, base, robot, _ = setup()
    base.motors["gripper"] = object()
    base.positions["gripper"] = -10.0
    base._gains["gripper"] = {"kp": 20, "kd": 0.5}
    robot.config.joint_limits["gripper"] = (-120, 0)
    with pytest.raises(RuntimeError, match="fresh measured pose"):
        if batch:
            bus.sync_write("Goal_Position", {"gripper": -90})
        else:
            bus.write("Goal_Position", "gripper", -90)
    assert not base.events


def test_cleanup_zero_effort_is_allowed_and_velocity_bypass_refused():
    bus, base, _, _ = setup()
    for name in base.motors:
        base._gains[name] = {"kp": 0, "kd": 0}
    bus.sync_write("Goal_Position", {"a": 0, "b": 0})
    assert base.events[-1] == ("Goal_Position", "b", 0)
    with pytest.raises(ValueError, match="bypass"):
        bus.write("Goal_Velocity", "a", 100)
    with pytest.raises(ValueError, match="bypass"):
        bus.sync_write("Goal_Velocity", {"a": 100})


def test_invalid_batch_is_refused_before_any_motion_write():
    bus, base, _, now = setup()
    bus.enable_torque()
    count = len(base.events)
    now[0] = 0.02
    with pytest.raises(ValueError, match="Non-finite"):
        bus.sync_write("Goal_Position", {"a": 80, "b": float("nan")})
    assert len(base.events) == count


@pytest.mark.parametrize("speed", [0, -1, 31, float("nan"), float("inf")])
def test_invalid_speed_is_rejected_by_both_public_schemas(speed):
    from makermodslab.schemas.sessions import TeleoperationOptions
    from makermodslab.teleoperate import TeleoperateRequest

    with pytest.raises(ValidationError):
        TeleoperationOptions(speed_limit_deg_s=speed)
    with pytest.raises(ValidationError):
        TeleoperateRequest(
            leader_port="a",
            follower_port="b",
            leader_config="a",
            follower_config="b",
            speed_limit_deg_s=speed,
        )


def test_speed_option_reaches_the_teleop_request():
    from makermodslab.schemas.sessions import TeleoperationOptions
    from makermodslab.sessions import _build_teleoperation_request

    record = {
        "name": "test",
        "leader_port": "a",
        "follower_port": "b",
        "leader_config": "a",
        "follower_config": "b",
        "mode": "single",
        "right_leader_port": "",
        "right_follower_port": "",
        "right_leader_config": "",
        "right_follower_config": "",
        "arm_type": "viscous",
        "leader_kind": "star",
    }
    request = _build_teleoperation_request(record, TeleoperationOptions(speed_limit_deg_s=7.5))
    assert request.speed_limit_deg_s == 7.5


def test_slow_return_uses_same_cap_and_longer_time_budget(monkeypatch):
    from makermodslab import maker_rest_pose
    from makermodslab.arms.viscous import VISCOUS

    calls = []
    device = SimpleNamespace(
        bus=SimpleNamespace(speed_limit_deg_s=7.5), config=SimpleNamespace(joint_limits={"a": (-180, 180)})
    )
    monkeypatch.setattr(maker_rest_pose, "return_maker_arms_to_rest", lambda *a, **kw: calls.append(kw))
    VISCOUS.return_to_rest([(device, {"a": 0})])
    assert calls[0]["speed_deg_s"] == 7.5
    assert calls[0]["ceiling_s"] >= 48
