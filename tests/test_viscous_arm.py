"""Viscous wiring through Lab's existing workflows; no hardware is opened."""

import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from makermodslab.api_errors import ApiError
from makermodslab.arm_capabilities import arm_type_from_robot_type, require_known_arm_type
from makermodslab.arms.viscous import VISCOUS
from makermodslab.utils import config as cfg
from makermodslab.utils.robot_factory import build_follower_config, build_single_configs


def test_config_first_import_registers_viscous():
    # A fresh server imports config before arms; pytest's shared imports can
    # otherwise hide a registration-time circular import.
    subprocess.run(
        [
            sys.executable,
            "-c",
            "from makermodslab.utils.config import arm_registry; assert 'viscous' in arm_registry.ids()",
        ],
        check=True,
        capture_output=True,
        text=True,
    )


@pytest.fixture
def commissioned(tmp_lerobot_home):
    for directory, name in (
        (VISCOUS.follower_calibration_dir(), "viscous_01"),
        (VISCOUS.leader_calibration_dir(), "star_viscous"),
    ):
        path = Path(directory)
        path.mkdir(parents=True, exist_ok=True)
        (path / f"{name}.json").write_text("{}")
    return SimpleNamespace(
        arm_type="viscous",
        mode="single",
        follower_port="/dev/can",
        leader_port="/dev/star",
        follower_config="viscous_01",
        leader_config="star_viscous",
    )


def test_teleop_and_recording_use_viscous_presets_and_preserve_calibrations(commissioned):
    from lerobot.robots.viscous_follower import ViscousFollowerConfig
    from lerobot.teleoperators.rebot_102_leader import RebotArm102LeaderViscousTeleopConfig

    paths = [
        Path(VISCOUS.follower_calibration_dir()) / "viscous_01.json",
        Path(VISCOUS.leader_calibration_dir()) / "star_viscous.json",
    ]
    before = [p.read_bytes() for p in paths]
    for cameras in (None, {}):
        follower, leader = build_single_configs(commissioned, cameras)
        assert isinstance(follower, ViscousFollowerConfig)
        assert isinstance(leader, RebotArm102LeaderViscousTeleopConfig)
        assert follower.id == "viscous_01"
        assert leader.id == "star_viscous"
        assert follower.temp_stop_c == 85
        assert follower.gains["shoulder_lift"] == (30, 2)
    assert [p.read_bytes() for p in paths] == before


def test_missing_calibration_refuses_before_device_construction(commissioned):
    (Path(VISCOUS.follower_calibration_dir()) / "viscous_01.json").unlink()
    with pytest.raises(FileNotFoundError):
        build_single_configs(commissioned)
    with pytest.raises(FileNotFoundError):
        build_follower_config(commissioned)


def test_follower_only_and_inference_use_same_driver(commissioned):
    from lerobot.robots import make_robot_from_config
    from makermodslab.rollout import InferenceRequest, _arm_count_mismatch, _single_robot_args

    follower = build_follower_config(commissioned)
    robot = make_robot_from_config(follower)
    assert robot.name == "viscous_follower"
    from lerobot.utils.feature_utils import hw_to_dataset_features

    state = hw_to_dataset_features(robot.observation_features, "observation")["observation.state"]
    assert state["shape"] == (7,)
    assert state["names"] == list(robot.action_features)
    assert _arm_count_mismatch("single", state["shape"][0], "viscous") is None
    assert set(robot.motor_models.values()) == {"O0"}
    request = InferenceRequest(
        arm_type="viscous", follower_port="/dev/can", follower_config="viscous_01", policy_ref="test"
    )
    assert "--robot.type=viscous_follower" in _single_robot_args(request, "viscous_01")
    assert arm_type_from_robot_type(robot.name) == "viscous"


def test_follower_zero_is_refused_without_opening_a_bus(monkeypatch):
    build = Mock(side_effect=AssertionError("must not construct a device"))
    monkeypatch.setattr(VISCOUS, "single_follower_config", build)
    with pytest.raises(ValueError, match="Re-zeroing"):
        VISCOUS.open_for_calibration("robot", "/dev/can", "new-zero")
    with pytest.raises(ValueError, match="Re-zeroing"):
        VISCOUS.calibrate(Mock(), "robot", Mock())
    build.assert_not_called()


def test_manifest_and_robot_record_accept_viscous(client, tmp_lerobot_home):
    entry = next(a for a in client.get("/api/v1/arms").json()["arms"] if a["id"] == "viscous")
    assert entry["joints_per_arm"] == 7
    assert entry["robot_types"] == ["viscous_follower"]
    assert entry["supports_bimanual"] is False
    assert entry["telemetry_kind"] == "degrees"
    assert entry["capabilities"]["supports_port_probe"] is True
    response = client.post(
        "/api/v1/robots/viscous?create=true", json={"arm_type": "viscous", "mode": "single"}
    )
    assert response.status_code == 200, response.text
    assert cfg.get_robot_record("viscous")["arm_type"] == "viscous"


def test_bimanual_refused_at_api_and_builder(client, tmp_lerobot_home):
    response = client.post(
        "/api/v1/robots/viscous?create=true", json={"arm_type": "viscous", "mode": "bimanual"}
    )
    assert response.status_code == 400
    assert cfg.get_robot_record("viscous") is None
    with pytest.raises(ApiError, match="single-arm"):
        require_known_arm_type("viscous", mode="bimanual")
    with pytest.raises(ValueError, match="single follower"):
        VISCOUS.build_bimanual_configs(None, None, "", "", "")
