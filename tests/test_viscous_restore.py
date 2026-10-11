"""The recovery bundle restores safely, including machine-specific ports."""

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
