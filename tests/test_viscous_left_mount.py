"""Offline tests of the future operator tool; every hardware operation is faked."""

import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

SPEC = importlib.util.spec_from_file_location(
    "left_mount", Path(__file__).parents[1] / "scripts/viscous_left_mount.py"
)
mount = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(mount)


@pytest.mark.parametrize("value", [-270, -90, 0, 79.9, 100.1, 450, float("nan")])
def test_wrong_sign_branch_or_nonfinite_aborts(value):
    with pytest.raises(ValueError):
        mount.check_before([value] * 5)


def test_new_limits_use_actual_shift_and_leader_neutral_stays_zero():
    profile = mount.profiles(93.2)
    assert profile["follower"]["joint_limits"][mount.J1] == pytest.approx([-278.6, 34.4])
    assert profile["leader"]["joint_ranges"][mount.J1] == [-154, 34]
    assert profile["leader"]["joint_offsets"][mount.J1] == 0
    assert profile["leader"]["joint_offsets"]["gripper"] == -33
    assert profile["follower"]["joint_limits"]["wrist_flex"] == [-120.8, 92]
    assert mount.table_degrees({mount.J1: 0, "elbow_flex": 30}, 93.2) == {mount.J1: 93.2, "elbow_flex": 30}


@pytest.mark.parametrize("shift", [80.0, 90.0, 100.0])
def test_actual_driver_unwrap_cannot_jump_across_star_travel_or_whole_turn_aliases(shift):
    from lerobot.teleoperators.rebot_102_leader.rebot_102_leader import RebotArm102Leader

    profile = mount.profiles(shift)["leader"]
    lo, hi = profile["joint_ranges"][mount.J1]
    gain = profile["joint_directions"][mount.J1]
    previous = -float("inf")
    for step in range(501):
        raw = -107.1 + (110.3 + 107.1) * step / 500
        for turns in range(-2, 3):
            unwrapped, _ = RebotArm102Leader._round_to_valid_range(raw + turns * 360, lo / gain, hi / gain)
            assert unwrapped == pytest.approx(raw)
        target = max(lo, min(hi, unwrapped * gain))
        assert target >= previous - 1e-9
        previous = target


def test_unverified_receipt_cannot_produce_profiles(tmp_path):
    with pytest.raises(ValueError):
        mount.stage_release(tmp_path, {"activation_ready": False})
    assert not list(tmp_path.iterdir())


def test_staged_pair_is_accepted_by_existing_unit_profile_loader(tmp_path, monkeypatch):
    from makermodslab.arms.viscous import VISCOUS
    from makermodslab.utils import config

    receipt = {
        "activation_ready": True,
        "unit": "viscous_01",
        "zero_write_attempted": True,
        "final_torque_disabled": True,
        "physical_mount_verified_by_operator": True,
        "readings_before_deg": [92.0] * 5,
        "readings_after_deg": [0.01] * 5,
    }
    mount.stage_release(tmp_path, receipt)
    monkeypatch.setattr(config, "MAKERMODSLAB_HOME", str(tmp_path))
    follower = VISCOUS.single_follower_config("fake-CAN", "viscous_01")
    leader = VISCOUS.single_leader_config("fake-Star", "star_viscous")
    assert follower.joint_limits[mount.J1] == pytest.approx([-277.4, 35.6])
    assert leader.joint_ranges[mount.J1] == [-154, 35]
    assert leader.joint_offsets[mount.J1] == 0
    assert leader.joint_directions[mount.J1] == 1.439742


@pytest.mark.parametrize("failure", [None, "before", "after", "cleanup"])
def test_only_motor_one_zeroed_and_activation_requires_verification(tmp_path, monkeypatch, failure):
    from lerobot.motors import robstride

    calls = []

    class Bus:
        is_connected = False

        def __init__(self, port, motors, **kwargs):
            assert list(motors) == [mount.J1]
            assert motors[mount.J1].id == 1
            self.last_feedback_time = {mount.J1: None}
            self.zeroed = False
            self.disables = 0

        def connect(self, *, handshake):
            assert handshake is False
            self.is_connected = True
            calls.append("connect")

        def disable_torque(self, motors):
            assert motors == [mount.J1]
            self.disables += 1
            calls.append("disable")
            if failure == "cleanup" and self.disables == 3:
                raise RuntimeError("release failed")

        def read(self, field, motor):
            assert field == "Present_Position" and motor == mount.J1
            if self.zeroed:
                return 20 if failure == "after" else 0.01
            return -270 if failure == "before" else 92.0

        def set_zero_position(self, motors):
            assert motors == [mount.J1]
            calls.append("zero")
            self.zeroed = True

        def disconnect(self, *, disable_torque):
            assert disable_torque is False
            self.is_connected = False
            calls.append("disconnect")

    monkeypatch.setattr(robstride, "RobstrideMotorsBus", Bus)
    monkeypatch.setattr(mount.subprocess, "run", lambda *a, **kw: SimpleNamespace(stdout="inactive\n"))
    monkeypatch.setattr(mount.time, "sleep", lambda _: None)
    monkeypatch.setattr(mount.signal, "signal", lambda *a: None)
    monkeypatch.setattr("builtins.input", lambda _: "LEFT J1 NEUTRAL VERIFIED")
    output = tmp_path / "release"
    args = SimpleNamespace(port="/dev/serial/by-id/unit-207E37A55445", output=output, execute=True)
    if failure:
        with pytest.raises((ValueError, RuntimeError)):
            mount.operate(args)
    else:
        mount.operate(args)
    evidence = json.loads((output / "j1-rezero.json").read_text())
    assert evidence["activation_ready"] is (failure is None)
    assert calls[:2] == ["connect", "disable"]
    assert calls[-1] == "disconnect"
    assert ("zero" in calls) is (failure != "before")
    assert (output / "viscous_profiles").exists() is (failure is None)


def test_dry_run_never_calls_hardware(monkeypatch, capsys):
    monkeypatch.setattr("sys.argv", ["viscous_left_mount.py", "--dry-run"])
    monkeypatch.setattr(mount, "operate", lambda _: pytest.fail("Hardware path reached"))
    mount.main()
    assert json.loads(capsys.readouterr().out)["hardware_opened"] is False
