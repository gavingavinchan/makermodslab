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
"""Upstream CAN behaviour that must stay OFF on the Viscous arm (2026-11-13 sync).

ViscousFamily subclasses CanArmFamily and ViscousFollower subclasses
MakerFollower, so upstream's Maker/Metal changes reach this arm with no
textual conflict. Each test pins one opt-out; docs/viscous-upstream-sync-2026-11.md
lists how to turn each one on and what to test at the arm first. No hardware
is opened: the follower is constructed, never connected.
"""

from types import SimpleNamespace

import pytest

from makermodslab import gravity_feedforward, maker_can, maker_rest_pose, remote_can, teleoperate
from makermodslab.arms.maker import MAKER
from makermodslab.arms.metal import METAL
from makermodslab.arms.viscous import VISCOUS, ViscousFamily


def test_family_flags_opt_out_while_maker_and_metal_keep_upstreams():
    assert VISCOUS.requires_verified_rest is False
    assert VISCOUS.rest_return_ensures_target is False
    assert VISCOUS.hosted_gripper_skips_speed_cap is False
    assert VISCOUS.gravity_feedforward_scale == 0.0
    assert VISCOUS.supports_gripper_effort_control is False
    assert VISCOUS.supports_gripper_current_control is False
    # The opt-outs are Viscous's own; the shared CAN defaults stay upstream's.
    for family in (MAKER, METAL):
        assert family.requires_verified_rest is True
        assert family.rest_return_ensures_target is True
        assert family.hosted_gripper_skips_speed_cap is True
    assert MAKER.gravity_feedforward_scale == 1.0


def test_teleop_and_leader_following_hooks_install_nothing():
    # Maker's hook installs MakerTeleopSafety (it rebinds send_action and sets
    # teleop_safety); Metal's wraps the bus in its follower speed cap.
    assert ViscousFamily.prepare_teleoperation is not type(MAKER).prepare_teleoperation
    assert ViscousFamily.prepare_leader_following is not type(METAL).prepare_leader_following
    bus, send = object(), object()
    robot = SimpleNamespace(bus=bus, send_action=send, config=SimpleNamespace(type="viscous_follower"))
    assert VISCOUS.prepare_teleoperation(robot) is None
    assert VISCOUS.prepare_leader_following(robot) is None
    assert robot.bus is bus and robot.send_action is send
    assert not hasattr(robot, "teleop_safety")


@pytest.fixture
def maker_hooks_installed(monkeypatch):
    # Every Lab process entry point calls this; the Maker family asks for gravity.
    monkeypatch.setattr(MAKER, "gravity_feedforward_scale", 1.0)
    maker_can.install()


def test_maker_gravity_feedforward_is_not_attached_to_the_viscous_follower(maker_hooks_installed):
    from lerobot.robots.viscous_follower import ViscousFollower, ViscousFollowerConfig

    robot = ViscousFollower(ViscousFollowerConfig(port="/dev/fake"))
    assert not hasattr(robot, "gravity_feedforward")
    assert "write" not in vars(robot.bus)


def test_maker_gripper_open_limit_leaves_the_viscous_gripper_range(maker_hooks_installed):
    from lerobot.robots.maker_follower import MakerFollowerConfig
    from lerobot.robots.viscous_follower import ViscousFollowerConfig, ViscousFollowerConfigBase

    measured = ViscousFollowerConfigBase.__dataclass_fields__["joint_limits"].default_factory()
    assert ViscousFollowerConfig(port="/dev/fake").joint_limits["gripper"] == measured["gripper"]
    # The hook did run: the Maker config is narrowed.
    assert MakerFollowerConfig(port="/dev/fake").joint_limits["gripper"][0] == -96.0


def test_bench_gravity_switch_never_stacks_on_upstreams_feedforward(monkeypatch):
    monkeypatch.setenv(gravity_feedforward.ENV_FIT, "never-read.json")

    def load_model(path):
        raise AssertionError("must refuse before loading a fit")

    monkeypatch.setattr(gravity_feedforward, "load_model", load_model)
    bus = SimpleNamespace(_mit_control=None, _gains={})
    robot = SimpleNamespace(bus=bus, _last_positions={}, gravity_feedforward=object())
    assert gravity_feedforward.install_from_env(robot, "viscous_01") is None


def test_hosted_gripper_stays_under_the_speed_cap():
    assert remote_can.hosted_speed_cap_exempt(VISCOUS) == ()
    assert remote_can.hosted_speed_cap_exempt(MAKER) == remote_can.GRIPPERS
    previous = {"shoulder_pan.pos": 0.0, "gripper.pos": 0.0}
    action = {"shoulder_pan.pos": 50.0, "gripper.pos": -50.0}
    capped = remote_can.limit_action(
        previous, action, 0.1, exempt=remote_can.hosted_speed_cap_exempt(VISCOUS)
    )
    assert capped == {"shoulder_pan.pos": pytest.approx(3.0), "gripper.pos": pytest.approx(-3.0)}


@pytest.mark.parametrize("speed_capped", [False, True])
def test_rest_return_does_not_force_a_ramp(monkeypatch, speed_capped):
    seen = []
    monkeypatch.setattr(
        maker_rest_pose,
        "return_maker_arms_to_rest",
        lambda poses, abort, **kwargs: seen.append(kwargs) or [(True, "ok")],
    )
    device = SimpleNamespace(
        bus=SimpleNamespace(speed_limit_deg_s=10.0 if speed_capped else None),
        config=SimpleNamespace(joint_limits={"shoulder_pan": (-10.0, 10.0)}),
    )
    assert VISCOUS.return_to_rest([(device, {"shoulder_pan": 0.0})]) == [(True, "ok")]
    assert MAKER.return_to_rest([(device, {"shoulder_pan": 0.0})]) == [(True, "ok")]
    assert [kwargs["ensure_target"] for kwargs in seen] == [False, True]


def test_failed_teleop_landing_still_releases_torque(monkeypatch):
    # requires_verified_rest=True would hold the bus energized until an
    # explicit release; Viscous keeps "stopped means de-energized".
    monkeypatch.setattr(ViscousFamily, "return_to_rest", lambda self, poses, abort: [(False, "stalled")])
    device = SimpleNamespace(bus=SimpleNamespace(motors={"shoulder_pan": 1}))
    assert teleoperate._return_before_release(VISCOUS, [(device, {"shoulder_pan": 0.0})], None) is None
    monkeypatch.setattr(type(MAKER), "return_to_rest", lambda self, poses, abort: [(False, "stalled")])
    assert "Automatic torque release is blocked" in teleoperate._return_before_release(
        MAKER, [(device, {"shoulder_pan": 0.0})], None
    )


def test_gripper_holding_adapter_skips_a_viscous_follower():
    from makermodslab.metal_gripper import install_metal_gripper

    bus = object()
    robot = SimpleNamespace(bus=bus, config=SimpleNamespace(type="viscous_follower"))
    install_metal_gripper(robot, "")
    assert robot.bus is bus


def test_viscous_leader_keeps_the_signed_star_mapping():
    from makermodslab.star_gripper import STAR_TRIGGER_LEADER_KIND, TriggerJointDirections

    assert STAR_TRIGGER_LEADER_KIND not in {option.id for option in VISCOUS.leader_options()}
    leader = VISCOUS.single_leader_config("/dev/fake", "")
    assert not isinstance(leader.joint_directions, TriggerJointDirections)
