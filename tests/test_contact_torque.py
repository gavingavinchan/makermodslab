import math

import pytest

pytest.importorskip("pinocchio")

from makermodslab.arm_gravity import GravityModel  # noqa: E402
from makermodslab.contact_torque import ContactEstimator  # noqa: E402
from makermodslab.gravity_feedforward import load_model  # noqa: E402
from tests.test_arm_gravity import JOINTS, urdf  # noqa: E402,F401


def test_contact_removes_gravity_and_offsets_and_preserves_push(urdf):  # noqa: F811
    model = GravityModel(urdf, JOINTS, offsets=[0.1, -0.2, 0.3])
    estimator = ContactEstimator(model)
    pose = dict(zip(JOINTS, [0, -60, 90], strict=True))
    torque = model.torque(pose)
    assert estimator.estimate(pose, torque) == pytest.approx(dict.fromkeys(JOINTS, 0))
    torque["elbow"] += 1.5
    assert estimator.estimate(pose, torque)["elbow"] == pytest.approx(1.5)


@pytest.mark.parametrize("reading", [None, math.nan, math.inf, -math.inf, True, False, "1"])
def test_bad_or_missing_torque_is_nan(urdf, reading):  # noqa: F811
    estimator = ContactEstimator(GravityModel(urdf, JOINTS))
    pose = dict.fromkeys(JOINTS, 0)
    assert math.isnan(estimator.estimate(pose, {"lift": reading})["lift"])
    assert math.isnan(estimator.estimate(pose, {})["elbow"])


def test_committed_fit_offsets_are_only_used_for_telemetry():
    path = "configs/viscous/viscous_01/gravity-air-fit.json"
    fit, command = load_model(path)
    estimator = ContactEstimator.from_fit(path)
    pose = dict.fromkeys(command.joints, 0)
    expected = command.torque(pose)
    reported = estimator.model.torque(pose)
    assert [reported[k] - expected[k] for k in command.joints] == pytest.approx(fit["offsets_nm"])
    assert estimator.estimate(pose, reported) == pytest.approx(dict.fromkeys(command.joints, 0))


@pytest.fixture
def check_script(monkeypatch):
    import importlib.util
    from pathlib import Path

    scripts = Path(__file__).resolve().parents[1] / "scripts"
    monkeypatch.syspath_prepend(str(scripts))
    spec = importlib.util.spec_from_file_location("contact_check", scripts / "viscous-contact-check.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_scale_ramp_sends_exactly_one_tick_per_step(check_script):
    from types import SimpleNamespace

    status = {"scale": 0.0}
    values = []
    run = SimpleNamespace(command={}, _tick=lambda target: values.append(status["scale"]))
    check_script.ramp_scale(run, status, 1.0)
    assert values == pytest.approx([(i + 1) / 50 for i in range(50)])


@pytest.mark.parametrize("ticks", [0, 1, 5])
def test_analysis_accepts_aborted_run_without_holds_or_push(check_script, ticks, capsys):
    estimator = ContactEstimator.from_fit("configs/viscous/viscous_01/gravity-air-fit.json")
    arm = check_script.ARM
    pose = dict.fromkeys(arm, 0)
    torque = estimator.model.torque(pose)
    columns = ["t", "phase", "scale"] + [f"{k}.{f}" for f in ("cmd", "pos", "torque", "ff") for k in arm]
    rows = [[i * 0.02, "move:start", 0] + [0] * 12 + [torque[k] for k in arm] + [0] * 6 for i in range(ticks)]
    check_script.analyze(
        {
            "robot": "viscous_01",
            "started": "test",
            "outcome": "interrupted",
            "gains": {k: [30, 2] for k in arm},
            "holds": [],
            "push": None,
            "ticks": {"columns": columns, "rows": rows},
        },
        estimator,
    )
    assert "interrupted" in capsys.readouterr().out
