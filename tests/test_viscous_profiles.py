"""Commissioned per-arm Viscous ranges must reach the actual Lab config builders."""

import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
from makermodslab.arms.viscous import VISCOUS
from makermodslab.utils import config as cfg


def write_profiles(home, follower_id="waterproof", leader_id="waterproof_star"):
    follower = VISCOUS.single_follower_config("/dev/can", "viscous_01")
    limits = dict(follower.joint_limits)
    limits["shoulder_pan"] = (-126.9, 187.5)
    limits["shoulder_lift"] = (-199.4, -2.8)
    limits["wrist_flex"] = (-122.3, 82.1)
    leader = VISCOUS.single_leader_config("/dev/star", "star_viscous")
    profile = {
        "follower": {"joint_limits": limits},
        "leader": {
            "joint_ranges": {k: [math.ceil(lo), math.floor(hi)] for k, (lo, hi) in limits.items()},
            "joint_directions": leader.joint_directions,
            "joint_offsets": dict.fromkeys(limits, 0.0),
            "max_start_offset_deg": 30.0,
        },
    }
    root = Path(home) / "viscous_profiles"
    root.mkdir()
    for name in (follower_id, leader_id):
        (root / f"{name}.json").write_text(json.dumps(profile))
    return profile, root


def test_profiles_reach_lab_pair_and_follower_only(tmp_lerobot_home):
    profile, _ = write_profiles(cfg.MAKERMODSLAB_HOME)
    request = SimpleNamespace(follower_port="/dev/can", leader_port="/dev/star")
    follower, leader = VISCOUS.build_single_configs(request, None, "waterproof_star", "waterproof")
    assert follower.joint_limits["shoulder_lift"] == (-199.4, -2.8)
    assert leader.joint_ranges["shoulder_pan"] == [-126, 187]
    assert leader.joint_offsets["gripper"] == 0
    assert follower.startup_sync_speed_deg == 1
    assert follower.temp_stop_c == 85
    assert VISCOUS.single_follower_config("/dev/can", "waterproof").joint_limits == follower.joint_limits
    old_follower, old_leader = VISCOUS.build_single_configs(request, None, "star_viscous", "viscous_01")
    assert old_follower.joint_limits["shoulder_lift"] == (-176.4, -2.4)
    assert old_leader.joint_offsets["gripper"] == -33


def test_mismatched_old_leader_refuses_before_hardware(tmp_lerobot_home):
    write_profiles(cfg.MAKERMODSLAB_HOME)
    request = SimpleNamespace(follower_port="/dev/can", leader_port="/dev/star")
    with pytest.raises(ValueError, match="do not match"):
        VISCOUS.build_single_configs(request, None, "star_viscous", "waterproof")


@pytest.mark.parametrize("bad", ["missing_joint", "reversed_range", "nonfinite"])
def test_invalid_follower_profile_refuses(tmp_lerobot_home, bad):
    profile, root = write_profiles(cfg.MAKERMODSLAB_HOME)
    limits = profile["follower"]["joint_limits"]
    if bad == "missing_joint":
        del limits["gripper"]
    elif bad == "reversed_range":
        limits["gripper"] = [0, -120]
    else:
        limits["gripper"] = [-120, float("inf")]
    (root / "waterproof.json").write_text(json.dumps(profile))
    with pytest.raises(ValueError, match="ranges"):
        VISCOUS.single_follower_config("/dev/can", "waterproof")


def test_missing_offsets_refuses_old_gripper_fallback(tmp_lerobot_home):
    profile, root = write_profiles(cfg.MAKERMODSLAB_HOME)
    del profile["leader"]["joint_offsets"]
    (root / "waterproof_star.json").write_text(json.dumps(profile))
    with pytest.raises(ValueError, match="Incomplete"):
        VISCOUS.single_leader_config("/dev/star", "waterproof_star")


def test_profile_id_cannot_escape_directory(tmp_lerobot_home):
    with pytest.raises(ValueError, match="Invalid Viscous calibration id"):
        VISCOUS.single_follower_config("/dev/can", "../unsafe")


def test_one_sided_factories_load_selected_profile(tmp_lerobot_home):
    from makermodslab.utils.robot_factory import (
        build_follower_config,
        build_leader_config,
    )

    write_profiles(cfg.MAKERMODSLAB_HOME)
    for directory, name in (
        (VISCOUS.follower_calibration_dir(), "waterproof"),
        (VISCOUS.leader_calibration_dir(), "waterproof_star"),
    ):
        path = Path(directory)
        path.mkdir(parents=True, exist_ok=True)
        (path / f"{name}.json").write_text("{}")
    follower = build_follower_config(
        SimpleNamespace(arm_type="viscous", follower_port="/dev/can", follower_config="waterproof")
    )
    leader = build_leader_config(
        SimpleNamespace(arm_type="viscous", leader_port="/dev/star", leader_config="waterproof_star")
    )
    assert follower.joint_limits["shoulder_lift"] == (-199.4, -2.8)
    assert leader.joint_ranges["shoulder_lift"] == [-199, -3]
    assert leader.joint_offsets["gripper"] == 0


def test_missing_profile_cannot_use_unit_one_limits(tmp_lerobot_home):
    with pytest.raises(FileNotFoundError, match="Missing commissioned"):
        VISCOUS.single_follower_config("/dev/can", "waterproof")
    with pytest.raises(FileNotFoundError, match="Missing commissioned"):
        VISCOUS.single_leader_config("/dev/star", "waterproof_star")
