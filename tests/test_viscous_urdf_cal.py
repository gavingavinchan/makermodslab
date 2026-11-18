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
"""Offline touch calibration, including the motor-to-tip integration boundary."""

import json
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("scipy", reason="Install scripts/viscous-urdf-cal/requirements.txt for calibration tests")

CAL = Path(__file__).resolve().parents[1] / "scripts/viscous-urdf-cal"
sys.path.insert(0, str(CAL))
from capture import Hold, fresh, parse_label  # noqa: E402
from fit import Problem, Session, fit_file  # noqa: E402
from kinematics import ARM_FILE, JOINTS, MAP, REAL, URDF, Arm, core, digest, read  # noqa: E402
from synthetic import TRUTH, generate  # noqa: E402


@pytest.fixture(scope="module")
def sample():
    return generate(n=15)


def write(path, meta, rows):
    path.write_text("\n".join(json.dumps(r) for r in [meta, *rows]) + "\n")
    return path


def test_recovery_outputs_and_regeneration(sample, tmp_path):
    meta, rows = sample
    inputs = {
        path: digest(path) for path in [URDF, ARM_FILE, MAP, REAL / "tank_model/viscous_waterproof_tank.urdf"]
    }
    source = write(tmp_path / "touch.jsonl", meta, rows)
    out = tmp_path / "fit"
    report = fit_file(source, out)
    assert report["issues"] == []
    assert report["loto_rms_mm"] < 3 < report["before_rms_mm"]
    assert len(report["points"]) == len(rows) and len(report["folds"]) == 16
    assert len(report["sign_trials"]) == 15
    mapping = read(out / "motor_to_urdf.pending.json")
    assert mapping["provenance"]["sha256"]["session"] == digest(source)
    assert mapping["provenance"]["status"] == "pending Gavin's approval"
    actual = [mapping["joints"][j]["offset_deg"] - read(MAP)["joints"][j]["offset_deg"] for j in JOINTS]
    np.testing.assert_allclose(actual, TRUTH[:6], atol=1)
    tcp = read(out / "tcp.pending.json")
    np.testing.assert_allclose(np.array(tcp["tcp"]) * 1000 - Arm().tcp, TRUTH[6:9], atol=2)
    assert abs(report["parameters"]["sheet_x_mm"] - TRUTH[9]) < 3
    args = [sys.executable, str(CAL / "make_tank_arm.py"), str(out), str(out / "model")]
    subprocess.run(args, check=True)
    candidate = out / "model/viscous_waterproof_tank.urdf"
    arm = read(out / "model/viscous_waterproof_tank_arm.json")
    jmap = core.JointMap.load(out / "model/motor_to_urdf.pending.json")
    q = dict(zip(JOINTS, jmap.to_ik(rows[0]["motor_deg"]), strict=True))
    joints = core.parse_urdf(candidate.read_bytes())
    flange = core.fk(joints, arm["base"], arm["flange"], q)
    tool = core.fk(joints, arm["base"], "tool0", q)
    np.testing.assert_allclose(flange @ [*tcp["tcp"], 1], tool[:, 3], atol=1e-9)
    assert all(
        (candidate.parent / n.get("filename")).is_file() for n in ET.parse(candidate).findall(".//mesh")
    )
    assert read(candidate.with_suffix(".provenance.json"))["approval"]["status"] == "pending_gavin_approval"
    assert subprocess.run(args, capture_output=True).returncode != 0
    with pytest.raises(FileExistsError):
        fit_file(source, out)
    assert inputs == {path: digest(path) for path in inputs}


def test_free_sheet_and_roll_gauges(sample):
    meta, rows = sample
    meta = meta.copy()
    del meta["survey_yaw_deg"]
    data = Session(meta, [r | {"jaw_level": False} for r in rows])
    arm = Arm()
    problem = Problem(arm, data, arm.sign)
    p, _, check = problem.solve(np.ones(len(rows), bool))
    assert p[0] == p[5] == 0
    assert abs(p[11] - (TRUTH[11] + TRUTH[0])) < 1
    assert max(check["touch_only_free_all_singular_values"][-2:]) < 1e-8
    assert not check["issues"]


def test_fk_and_jacobian_against_lab(sample):
    from makermodslab import tank_geometry as geo

    _, rows = sample
    m = np.array([[r["motor_deg"][j] for j in JOINTS] for r in rows[:3]])
    arm = Arm()
    p = np.zeros(14)
    tip, jac = arm.tip(m, p)
    model = geo.load_model(geo.load_tank())
    for r, point in zip(rows[:3], tip, strict=True):
        rot, pos = geo.forward_kinematics(model, r["motor_deg"])["wrist_roll_link"]
        np.testing.assert_allclose(point, rot @ arm.tcp + pos * 1000, atol=1e-7)
    for k in range(14):
        dp = np.eye(14)[k] * 1e-5
        fd = (arm.tip(m, p + dp)[0] - arm.tip(m, p - dp)[0]) / 2e-5
        np.testing.assert_allclose(fd, jac[:, :, k], atol=1e-6)


def test_wrong_sign_and_poor_data_refuse(sample, tmp_path):
    meta, rows = sample
    wrong = [r | {"motor_deg": r["motor_deg"] | {"elbow_flex": -r["motor_deg"]["elbow_flex"]}} for r in rows]
    out = tmp_path / "wrong"
    r = fit_file(write(tmp_path / "wrong.jsonl", meta, wrong), out)
    assert any("sign" in issue for issue in r["issues"])
    assert min(r["sign_trials"], key=lambda trial: trial["rms_mm"])["reversed"] == ["elbow_flex"]
    assert list(out.iterdir()) == [out / "residuals.json"]
    data = Session(meta, [r | {"motor_deg": rows[0]["motor_deg"]} for r in rows])
    problem = Problem(Arm(), data, Arm().sign, links=True)
    assert problem.solve(np.ones(len(rows), bool))[2]["issues"]
    assert data.coverage()


def test_stillness_and_gripper(sample, tmp_path):
    from gripper import fit, opening_mm

    p = sample[1][0]["motor_deg"]
    hold = Hold(p)
    assert not hold.update(p, True, 0)
    q = p | {"wrist_roll": p["wrist_roll"] + 5}
    for t in np.arange(0.25, 3, 0.25):
        assert not hold.update(q, True, t)
    assert not hold.update(None, False, 3)
    for t in np.arange(4, 7, 0.25):
        assert not hold.update(q, True, t)
    assert hold.update(q, True, 7)
    assert fresh({"positions": q, "stale": True}) is None
    assert parse_label("8@20", "A4") == (8, 20)
    with pytest.raises(ValueError):
        parse_label("49", "A4")
    source = tmp_path / "calipers"
    curve = [(-100, 83), (-75, 64), (-50, 45), (-25, 21), (0, 0)]
    source.write_text("\n".join(json.dumps({"motor_deg": x, "opening_mm": y}) for x, y in curve))
    fit(source, tmp_path / "gap.json")
    gaps = read(tmp_path / "gap.json")
    assert gaps["lab_one_finger_slide_mm"][-1] == 0
    assert opening_mm(gaps["piecewise_linear_deg_full_gap_mm"], -50) == 45
    with pytest.raises(ValueError):
        opening_mm(curve, -110)


def test_lengths_only_on_supported_improvement(monkeypatch, tmp_path):
    import synthetic

    truth = TRUTH.copy()
    truth[-2:] = [4.5, -4.5]
    monkeypatch.setattr(synthetic, "TRUTH", truth)
    meta, rows = synthetic.generate(n=15)
    report = fit_file(write(tmp_path / "contacts", meta, rows), tmp_path / "links", links=True)
    assert report["issues"] == []
    assert report["loto_rms_mm"] < 3 < report["baseline_loto_rms_mm"]
    lengths = read(tmp_path / "links/tcp.pending.json")["link_length_delta_mm"]
    np.testing.assert_allclose(lengths, truth[-2:], atol=0.6)


def test_manual_capture_uses_only_lab_session(monkeypatch, tmp_path, sample):
    import io
    from types import SimpleNamespace

    import capture

    calls = []
    state = {
        "active": True,
        "robot_name": "unit",
        "trace_path": "session-a",
        "still": True,
        "stale": False,
        "positions": sample[1][0]["motor_deg"],
    }

    def fake_lab(action, body=None):
        calls.append(action)
        return state

    monkeypatch.setattr(capture, "call", fake_lab)
    monkeypatch.setattr(capture.subprocess, "run", lambda *a, **kw: SimpleNamespace(returncode=1))
    stdin = io.StringIO("level:12@30\nfloor\ndone\n")
    monkeypatch.setattr(capture.sys, "stdin", stdin)
    monkeypatch.setattr(capture.select, "select", lambda *a: ([stdin], [], []))
    path = tmp_path / "capture"
    args = SimpleNamespace(
        output=path,
        watch=None,
        level_targets=None,
        sheet="A4",
        robot="unit",
        origin_mm=[10, 20],
        floor_z_mm=-10,
        sheet_z_mm=-9,
        survey_yaw_deg=None,
        roll_datum_deg=None,
    )
    capture.run(args)
    header, raised, floor = [json.loads(line) for line in path.read_text().splitlines()]
    assert header["schema"] == "viscous-touch-v1"
    assert (raised["target"], raised["height_mm"], raised["jaw_level"]) == (12, 30, True)
    assert floor["target"] == "floor" and floor["utc"] and floor["motor_deg"] == state["positions"]
    assert calls == ["start", "status", "status", "status", "stop"]
    with pytest.raises(FileExistsError):
        capture.run(args)
