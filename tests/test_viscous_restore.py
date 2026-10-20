"""The recovery bundles restore safely, including machine-specific ports."""

import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts/restore-viscous-waterproof.py"
SPEC = importlib.util.spec_from_file_location("viscous_restore", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_restore_builds_actual_matched_lab_pair(tmp_lerobot_home, monkeypatch):
    from makermodslab.utils import config as cfg
    from makermodslab.utils.robot_factory import build_single_configs

    calibration_home = tmp_lerobot_home / "restored-calibration"
    monkeypatch.setattr(cfg, "CALIBRATION_BASE_PATH_ROBOTS", str(calibration_home / "robots"))
    monkeypatch.setattr(cfg, "CALIBRATION_BASE_PATH_TELEOP", str(calibration_home / "teleoperators"))
    monkeypatch.setattr(
        cfg, "MAKER_LEADER_CONFIG_PATH", str(calibration_home / "teleoperators/rebot_102_leader")
    )
    paths = MODULE.restore(
        MODULE.DEFAULT_BUNDLE,
        cfg.MAKERMODSLAB_HOME,
        calibration_home,
        "/dev/new-can",
        "/dev/new-star",
    )
    before = {path: path.read_bytes() for path in paths}
    record = json.loads(paths[-1].read_text())
    follower, leader = build_single_configs(SimpleNamespace(**record))
    assert follower.port == "/dev/new-can"
    assert leader.port == "/dev/new-star"
    assert follower.joint_limits["wrist_flex"] == (-122.26919348091766, 82.10485204224697)
    assert leader.joint_offsets["gripper"] == 0
    assert follower.startup_sync_speed_deg == 1
    MODULE.restore(
        MODULE.DEFAULT_BUNDLE,
        cfg.MAKERMODSLAB_HOME,
        calibration_home,
        "/dev/new-can",
        "/dev/new-star",
    )
    assert {path: path.read_bytes() for path in paths} == before


def test_conflict_refuses_entire_restore(tmp_path):
    lab, cal = tmp_path / "lab", tmp_path / "cal"
    conflict = lab / "viscous_profiles/viscous_waterproof_01.json"
    conflict.parent.mkdir(parents=True)
    conflict.write_text('{"existing": true}')
    with pytest.raises(FileExistsError, match="Existing file differs"):
        MODULE.restore(MODULE.DEFAULT_BUNDLE, lab, cal, "can", "star")
    assert not cal.exists()
    assert not (lab / "robots").exists()
    assert conflict.read_text() == '{"existing": true}'


def test_dry_run_does_not_create_destinations(tmp_path):
    lab, cal = tmp_path / "lab", tmp_path / "cal"
    paths = MODULE.restore(MODULE.DEFAULT_BUNDLE, lab, cal, "can", "star", dry_run=True)
    assert len(paths) == 5
    assert not lab.exists() and not cal.exists()


def test_changed_bundle_refuses_before_writing(tmp_path):
    import shutil

    bundle = tmp_path / "bundle"
    shutil.copytree(MODULE.DEFAULT_BUNDLE, bundle)
    (bundle / "robot-record-template.json").write_text("{}")
    with pytest.raises(ValueError, match="checksum failed"):
        MODULE.restore(bundle, tmp_path / "lab", tmp_path / "cal", "can", "star")
    assert not (tmp_path / "lab").exists()
    assert not (tmp_path / "cal").exists()


UNIT_1 = MODULE.BUNDLES / "viscous_01"


def test_waterproof_default_plan_is_unchanged(tmp_path):
    lab, cal = tmp_path / "lab", tmp_path / "cal"
    paths = MODULE.restore(MODULE.DEFAULT_BUNDLE, lab, cal, "can", "star", dry_run=True)
    assert MODULE.DEFAULT_BUNDLE.name == "viscous_waterproof_01"
    assert set(paths) == {
        cal / "robots/viscous_follower/viscous_waterproof_01.json",
        cal / "teleoperators/rebot_102_leader/star_viscous_waterproof_01.json",
        lab / "viscous_profiles/viscous_waterproof_01.json",
        lab / "viscous_profiles/star_viscous_waterproof_01.json",
        lab / "robots/viscous_waterproof_01.json",
    }
    assert paths[-1] == lab / "robots/viscous_waterproof_01.json"


def test_unit_1_plan_has_no_profiles(tmp_path):
    lab, cal = tmp_path / "lab", tmp_path / "cal"
    paths = MODULE.restore(UNIT_1, lab, cal, "can", "star", dry_run=True)
    assert paths == [
        cal / "robots/viscous_follower/viscous_01.json",
        cal / "teleoperators/rebot_102_leader/star_viscous.json",
        lab / "robots/viscous_01.json",
    ]
    assert not lab.exists() and not cal.exists()


def test_unit_1_restore_builds_built_in_preset_pair(tmp_lerobot_home, monkeypatch):
    from lerobot.robots.viscous_follower import ViscousFollowerConfig
    from lerobot.teleoperators.rebot_102_leader import RebotArm102LeaderViscousTeleopConfig
    from makermodslab.utils import config as cfg
    from makermodslab.utils.robot_factory import build_single_configs

    calibration_home = tmp_lerobot_home / "restored-calibration"
    monkeypatch.setattr(cfg, "CALIBRATION_BASE_PATH_ROBOTS", str(calibration_home / "robots"))
    monkeypatch.setattr(cfg, "CALIBRATION_BASE_PATH_TELEOP", str(calibration_home / "teleoperators"))
    monkeypatch.setattr(
        cfg, "MAKER_LEADER_CONFIG_PATH", str(calibration_home / "teleoperators/rebot_102_leader")
    )
    lab = Path(cfg.MAKERMODSLAB_HOME)
    paths = MODULE.restore(UNIT_1, lab, calibration_home, "/dev/new-can", "/dev/new-star")
    assert not (lab / "viscous_profiles").exists()
    record = json.loads(paths[-1].read_text())
    assert [camera["camera_index"] for camera in record["cameras"]] == [0, 2]
    follower, leader = build_single_configs(SimpleNamespace(**record))
    assert follower.port == "/dev/new-can"
    assert leader.port == "/dev/new-star"
    assert follower.joint_limits == ViscousFollowerConfig(port="x").joint_limits
    assert leader.joint_ranges == RebotArm102LeaderViscousTeleopConfig(port="x").joint_ranges
    for path, source in zip(paths[:2], ("robots", "teleoperators"), strict=True):
        bundled = next((UNIT_1 / "calibration" / source).rglob("*.json"))
        assert json.loads(path.read_text()) == json.loads(bundled.read_text())


def test_unlisted_bundle_file_refuses_before_writing(tmp_path):
    import shutil

    bundle = tmp_path / "bundle"
    shutil.copytree(UNIT_1, bundle)
    (bundle / "viscous_profiles").mkdir()
    (bundle / "viscous_profiles/viscous_01.json").write_text("{}")
    with pytest.raises(ValueError, match="not in its manifest"):
        MODULE.restore(bundle, tmp_path / "lab", tmp_path / "cal", "can", "star")
    assert not (tmp_path / "lab").exists()
    assert not (tmp_path / "cal").exists()


def test_bundle_manifest_must_name_its_robot(tmp_path):
    import shutil

    bundle = tmp_path / "bundle"
    shutil.copytree(UNIT_1, bundle)
    manifest = json.loads((bundle / "manifest.json").read_text())
    manifest["arm"] = "viscous_waterproof_01"
    (bundle / "manifest.json").write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match="does not match its robot"):
        MODULE.restore(bundle, tmp_path / "lab", tmp_path / "cal", "can", "star", dry_run=True)
