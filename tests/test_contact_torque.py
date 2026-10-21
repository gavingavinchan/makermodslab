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


@pytest.fixture
def friction_estimator(urdf):  # noqa: F811
    return ContactEstimator(
        GravityModel(urdf, JOINTS),
        {k: {"coulomb_nm": 0.2, "viscous_nm_per_deg_s": 0.01, "v_smooth_deg_s": 3.0} for k in JOINTS},
    )


def test_friction_zero_at_rest_odd_and_smooth(friction_estimator):
    estimator = friction_estimator
    assert estimator.friction_torque(dict.fromkeys(JOINTS, 0)) == dict.fromkeys(JOINTS, 0)
    positive = estimator.friction_torque(dict.fromkeys(JOINTS, 6))
    negative = estimator.friction_torque(dict.fromkeys(JOINTS, -6))
    assert positive == pytest.approx({k: 0.2 * math.tanh(2) + 0.06 for k in JOINTS})
    assert negative == pytest.approx({k: -v for k, v in positive.items()})
    eps = 1e-6
    small = estimator.friction_torque(dict.fromkeys(JOINTS, eps))
    assert small == pytest.approx(dict.fromkeys(JOINTS, (0.2 / 3 + 0.01) * eps))
    assert estimator.friction_torque({}) == dict.fromkeys(JOINTS, 0)


def test_friction_is_opt_in_and_observation_passes_velocity(friction_estimator):
    estimator = friction_estimator
    pose = dict.fromkeys(JOINTS, 0)
    velocity = dict.fromkeys(JOINTS, -6)
    friction = estimator.friction_torque(velocity)
    torque = {k: v + friction[k] for k, v in estimator.model.torque(pose).items()}
    torque["elbow"] += 1.5
    expected = {**dict.fromkeys(JOINTS, 0), "elbow": 1.5}
    assert estimator.estimate(pose, torque, velocity) == pytest.approx(expected)
    assert estimator.estimate(pose, torque) == pytest.approx({k: expected[k] + friction[k] for k in JOINTS})
    obs = {**{f"{k}.pos": pose[k] for k in JOINTS}, **{f"{k}.torque": torque[k] for k in JOINTS}}
    assert estimator.from_observation(obs, velocity) == pytest.approx(expected)
    assert estimator.from_observation(obs) == pytest.approx(estimator.estimate(pose, torque))


def test_old_fit_has_no_friction_even_with_velocity():
    estimator = ContactEstimator.from_fit("configs/viscous/viscous_01/gravity-air-fit.json")
    pose = dict.fromkeys(estimator.joints, 0)
    torque = estimator.model.torque(pose)
    assert estimator.friction == {}
    assert estimator.estimate(pose, torque, dict.fromkeys(estimator.joints, 15)) == pytest.approx(
        dict.fromkeys(estimator.joints, 0)
    )


def test_v2_loads_from_a_relocated_clone(tmp_path, monkeypatch):
    import json
    import shutil
    from pathlib import Path

    from makermodslab import arm_gravity

    root = Path(__file__).resolve().parents[1]
    relative = Path("configs/viscous/viscous_01/gravity-air-fit-v2.json")
    fit = json.loads((root / relative).read_text())
    paths = [str(relative), fit["urdf"], *fit["runs"], *fit["friction_identification"]["runs"]]
    clone = tmp_path / "clone"
    for name in paths:
        assert not Path(name).is_absolute()
        assert ".." not in Path(name).parts
        target = clone / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(root / name, target)
    monkeypatch.setattr(arm_gravity, "REPO_ROOT", clone)
    monkeypatch.chdir(tmp_path)
    estimator = ContactEstimator.from_fit(relative)
    assert set(estimator.friction) == set(estimator.joints)
    pose = dict.fromkeys(estimator.joints, 0)
    assert estimator.estimate(
        pose, estimator.model.torque(pose), dict.fromkeys(estimator.joints, 0)
    ) == pytest.approx(dict.fromkeys(estimator.joints, 0))
    # The command model must still exclude telemetry offsets and friction.
    _, command = load_model(relative)
    assert [
        estimator.model.torque(pose)[k] - command.torque(pose)[k] for k in estimator.joints
    ] == pytest.approx(fit["offsets_nm"])


@pytest.fixture
def fit_script(monkeypatch):
    import importlib.util
    from pathlib import Path

    scripts = Path(__file__).resolve().parents[1] / "scripts"
    monkeypatch.syspath_prepend(str(scripts))
    spec = importlib.util.spec_from_file_location("gravity_fit", scripts / "viscous-gravity-fit.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_hold_loader_keeps_file_pose_groups_and_checks_provenance(fit_script):
    from pathlib import Path

    from makermodslab.arm_gravity import VISCOUS_URDF

    evidence = Path(__file__).resolve().parents[1] / "configs/viscous/viscous_01/evidence"
    paths = [evidence / f"contact-check-20261020-{stamp}.json" for stamp in ("181208", "182410")]
    _, runs, rows = fit_script.load_runs(paths, VISCOUS_URDF)
    assert len(rows) == 24
    assert {run["condition"] for run in runs} == {"air"}
    assert len({group for group, _, _ in rows}) == 6
    assert {group for group, _, _ in rows[:12]}.isdisjoint({group for group, _, _ in rows[12:]})
    for group in {group for group, _, _ in rows}:
        samples = [s for g, s, _ in rows if g == group]
        assert len({s["pose"] for s in samples}) == 1
        assert {(s["scale"], s["approach"]) for s in samples} == {
            (scale, approach) for scale in (0, 1) for approach in ("above", "below")
        }


def test_hold_loader_rejects_another_urdf(fit_script, tmp_path):
    from pathlib import Path

    path = (
        Path(__file__).resolve().parents[1]
        / "configs/viscous/viscous_01/evidence/contact-check-20261020-181208.json"
    )
    from makermodslab.arm_gravity import VISCOUS_URDF

    changed = tmp_path / "changed.urdf"
    changed.write_text(VISCOUS_URDF.read_text() + "\n")
    with pytest.raises(ValueError, match="URDF hash"):
        fit_script.load_runs([path], changed)


def test_friction_identification_excludes_push_fold_and_acceleration(check_script):
    import copy
    import json
    from pathlib import Path

    import numpy as np
    from viscous_contact_analysis import fit_friction, motion_arrays

    path = (
        Path(__file__).resolve().parents[1]
        / "configs/viscous/viscous_01/evidence/contact-check-20261020-181208.json"
    )
    data = json.loads(path.read_text())
    phases, col, velocity, mask = motion_arrays(data)
    assert mask.sum() > 2000
    assert np.all(col("shoulder_lift.pos")[mask] < -15)
    assert np.all(col("elbow_flex.pos")[mask] > 15)
    assert all(p.startswith(("move:", "approach:")) for p in phases[mask])
    assert all(np.max(np.abs(np.gradient(v, col("t"))[mask])) <= 50 for v in velocity.values())
    estimator = ContactEstimator.from_fit("configs/viscous/viscous_01/gravity-air-fit-v2.json")
    fit, _ = fit_friction([data], estimator)
    changed = copy.deepcopy(data)
    indices = [changed["ticks"]["columns"].index(f"{k}.torque") for k in check_script.ARM]
    for i, row in enumerate(changed["ticks"]["rows"]):
        if not mask[i]:
            for j in indices:
                row[j] += 1000
    other, _ = fit_friction([changed], estimator)
    assert fit == other
    assert fit["shoulder_pan"]["coulomb_nm"] == 0
    assert fit["shoulder_pan"]["viscous_nm_per_deg_s"] == 0


@pytest.mark.parametrize("velocity", [None, math.nan, math.inf, -math.inf, True, "1"])
def test_bad_velocity_cannot_report_finite_contact(friction_estimator, velocity):
    pose = dict.fromkeys(JOINTS, 0)
    torque = friction_estimator.model.torque(pose)
    assert math.isnan(friction_estimator.estimate(pose, torque, {"elbow": velocity})["elbow"])


def test_analysis_compensation_matches_estimator_with_partial_friction(check_script, urdf):  # noqa: F811
    import numpy as np
    from viscous_contact_analysis import compensate

    estimator = ContactEstimator(
        GravityModel(urdf, JOINTS),
        {"elbow": {"coulomb_nm": 0.2, "viscous_nm_per_deg_s": 0.01, "v_smooth_deg_s": 3}},
    )
    contact = {k: np.array([1.5]) for k in JOINTS}
    velocity = {k: np.array([-6.0]) for k in JOINTS}
    corrected = compensate(contact, velocity, estimator.friction)
    pose = dict.fromkeys(JOINTS, 0)
    torque = {k: v + 1.5 for k, v in estimator.model.torque(pose).items()}
    expected = estimator.estimate(pose, torque, dict.fromkeys(JOINTS, -6.0))
    assert {k: v[0] for k, v in corrected.items()} == pytest.approx(expected)


def test_analysis_recomputes_saved_hold_contact_for_selected_fit(check_script, capsys):
    import json
    from pathlib import Path

    path = (
        Path(__file__).resolve().parents[1]
        / "configs/viscous/viscous_01/evidence/contact-check-20261020-181208.json"
    )
    data = json.loads(path.read_text())
    data["ticks"]["rows"] = []
    for hold in data["holds"]:
        hold["contact_nm"] = dict.fromkeys(check_script.ARM, 999)
    estimator = ContactEstimator.from_fit("configs/viscous/viscous_01/gravity-air-fit-v2.json")
    check_script.analyze(data, estimator)
    assert "999" not in capsys.readouterr().out
    assert data["holds"][0]["contact_nm"]["shoulder_lift"] == 999


def test_friction_is_held_at_the_fastest_identified_speed(urdf):  # noqa: F811
    terms = {"coulomb_nm": 0.2, "viscous_nm_per_deg_s": 0.02, "v_smooth_deg_s": 2.0, "max_speed_deg_s": 16.0}
    estimator = ContactEstimator(GravityModel(urdf, JOINTS), {"lift": terms})
    at_limit = estimator.friction_torque({"lift": 16.0})["lift"]
    assert estimator.friction_torque({"lift": 60.0})["lift"] == pytest.approx(at_limit)
    assert estimator.friction_torque({"lift": -60.0})["lift"] == pytest.approx(-at_limit)
    assert estimator.friction_torque({"lift": 8.0})["lift"] < at_limit


def test_the_committed_v2_fit_compensates_no_weakly_excited_joint():
    import json
    from pathlib import Path

    fit = json.loads(Path("configs/viscous/viscous_01/gravity-air-fit-v2.json").read_text())
    for name in ("shoulder_pan", "wrist_yaw", "wrist_roll"):
        assert fit["friction"][name]["coulomb_nm"] == 0.0
        assert fit["friction"][name]["viscous_nm_per_deg_s"] == 0.0
