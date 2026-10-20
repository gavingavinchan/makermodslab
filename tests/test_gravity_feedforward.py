import json
import math

import pytest

pytest.importorskip("pinocchio")

from makermodslab import gravity_feedforward as ff  # noqa: E402
from makermodslab.arm_gravity import GravityModel, file_sha256  # noqa: E402
from tests.test_arm_gravity import JOINTS, urdf  # noqa: E402,F401  (fixture)


class FakeBus:
    def __init__(self):
        self._gains = {m: {"kp": 30.0, "kd": 2.0} for m in (*JOINTS, "gripper")}
        self.frames, self.other = [], []

    def _get_motor_name(self, motor):
        return motor

    def _mit_control(self, motor, kp, kd, position, velocity, torque):
        self.frames.append((motor, kp, kd, position, velocity, torque))

    def write(self, data_name, motor, value):
        self.other.append((data_name, motor, value))


class FakeRobot:
    def __init__(self, positions):
        self.bus = FakeBus()
        self._last_positions = dict(positions)
        self._turn_offset = dict.fromkeys(positions, 0.0)


POSE = {"pan": 0.0, "lift": -60.0, "elbow": 90.0, "gripper": -30.0}


def drive(robot, clock, t):
    clock[0] = t
    robot.bus.frames.clear()
    for motor, value in POSE.items():
        robot.bus.write("Goal_Position", motor, value)
    return {m: tau for m, _, _, _, _, tau in robot.bus.frames}


def test_feedforward_ramps_in_scales_and_matches_the_model(urdf):  # noqa: F811
    model = GravityModel(urdf, JOINTS)
    robot, clock = FakeRobot(POSE), [0.0]
    ff.install(robot, model, scale=0.5, clock=lambda: clock[0])
    expected = model.torque({k: POSE[k] for k in JOINTS})
    first = drive(robot, clock, 10.0)
    assert first["lift"] == 0.0  # ramp starts at the first command
    half = drive(robot, clock, 11.0)
    assert half["lift"] == pytest.approx(0.5 * 0.5 * expected["lift"])
    full = drive(robot, clock, 13.0)
    assert full["elbow"] == pytest.approx(0.5 * expected["elbow"])
    assert full["gripper"] == 0.0  # not in the model: no feedforward
    motor, kp, kd, position, velocity, _ = robot.bus.frames[1]
    assert (motor, kp, kd, position, velocity) == ("lift", 30.0, 2.0, -60.0, 0)


def test_feedforward_is_clamped_and_other_writes_pass_through(urdf, monkeypatch):  # noqa: F811
    model = GravityModel(urdf, JOINTS, params=GravityModel(urdf, JOINTS).prior * 100)
    robot, clock = FakeRobot(POSE), [0.0]
    ff.install(robot, model, clock=lambda: clock[0])
    drive(robot, clock, 0.0)
    out = drive(robot, clock, 5.0)
    assert max(abs(v) for v in out.values()) == ff.CLAMP_NM
    robot.bus.write("Kp", "lift", 10.0)
    assert robot.bus.other == [("Kp", "lift", 10.0)]


def test_unreadable_pose_holds_with_stiffness_alone(urdf):  # noqa: F811
    model = GravityModel(urdf, JOINTS)
    robot, clock = FakeRobot({**POSE, "lift": math.nan}), [0.0]
    ff.install(robot, model, clock=lambda: clock[0])
    drive(robot, clock, 0.0)
    assert all(tau == 0.0 for tau in drive(robot, clock, 5.0).values())


def _fit_file(tmp_path, urdf_path, robot="viscous_01"):  # noqa: F811
    model = GravityModel(urdf_path, JOINTS)
    path = tmp_path / "fit.json"
    path.write_text(
        json.dumps(
            {
                "robot": robot,
                "urdf": str(urdf_path),
                "urdf_sha256": file_sha256(urdf_path),
                "joints": list(JOINTS),
                "params": model.prior.tolist(),
            }
        )
    )
    return path


def test_env_switch_applies_only_to_the_named_viscous_robot(urdf, tmp_path, monkeypatch):  # noqa: F811
    robot = FakeRobot(POSE)
    original = robot.bus.write
    assert ff.install_from_env(robot, "viscous_01") is None  # unset: untouched
    monkeypatch.setenv(ff.ENV_FIT, str(_fit_file(tmp_path, urdf)))
    assert ff.install_from_env(robot, "viscous_02") is None
    feetech = FakeRobot(POSE)
    del feetech.bus._gains  # not an MIT-mode bus
    assert ff.install_from_env(feetech, "viscous_01") is None
    assert robot.bus.write == original
    monkeypatch.setenv(ff.ENV_SCALE, "0.5")
    status = ff.install_from_env(robot, "viscous_01")
    assert status["scale"] == 0.5


def test_a_changed_urdf_is_refused(urdf, tmp_path):  # noqa: F811
    path = _fit_file(tmp_path, urdf)
    urdf.write_text(urdf.read_text().replace('value="0.5"', 'value="0.6"'))
    with pytest.raises(ValueError, match="changed"):
        ff.load_model(path)


def test_scale_outside_unit_range_is_refused(urdf):  # noqa: F811
    with pytest.raises(ValueError):
        ff.install(FakeRobot(POSE), GravityModel(urdf, JOINTS), scale=1.5)


def test_the_committed_unit_1_fit_loads_from_a_fresh_clone():
    # Repo-relative paths only: nothing outside the checkout is needed.
    fit, model = ff.load_model("configs/viscous/viscous_01/gravity-air-fit.json")
    assert fit["robot"] == "viscous_01" and fit["condition"] == "air"
    assert not fit["urdf"].startswith("/")
    assert model.urdf_sha256 == fit["urdf_sha256"]
