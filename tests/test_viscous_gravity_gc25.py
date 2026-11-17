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
"""Replay the archived gc25 incident, without constructing a hardware device."""

import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from makermodslab import gravity_feedforward as ff
from makermodslab.arm_gravity import GravityModel, repo_path

REAL = Path(__file__).resolve().parents[1] / "scripts/viscous-dream1/real"
FIXTURE = Path(__file__).resolve().parent / "fixtures/viscous_gravity/gc25"
sys.path.insert(0, str(REAL))
import gravity_comp as gc  # noqa: E402
import gravity_comp_report as report  # noqa: E402
from gravity_fit import KP  # noqa: E402


def ticks(name):
    return [json.loads(line) for line in (FIXTURE / f"{name}.jsonl").read_text().splitlines()]


def settled(name):
    hold = [r for r in ticks(name) if r["phase"] == "hold"]
    return [r for r in hold if r["t"] >= hold[0]["t"] + 1]


def test_archived_sag_and_signed_effort_balance():
    expected = {
        "baseline": [(1.977, 3.016, 0), (2.967, 4.584, 0), (1.071, 0.522, 0)],
        "gc25": [(0.483, 3.093, 2.421), (1.274, 5.000, 3.076), (2.588, 2.886, 1.566)],
    }
    for run, values in expected.items():
        data = report.report(FIXTURE / f"{run}.jsonl")
        rows = settled(run)
        for n, (error, torque, feedforward) in zip(list(KP)[1:4], values, strict=True):
            joint = data["joints"][n]
            assert joint["cmd_minus_pos_deg"] == pytest.approx(error, abs=0.001)
            assert np.mean([r["torque"][n] for r in rows]) == pytest.approx(torque, abs=0.001)
            assert joint["mean_ff_nm"] == pytest.approx(feedforward, abs=0.001)
            assert abs(joint["mean_torque_minus_effort_nm"]) < 0.1
    # Even the gripper configuration differed: these are not matched physical-load trials.
    for run, angle in (("baseline", -47.326), ("gc25", -5.880)):
        assert np.mean([r["pos"]["gripper"] for r in settled(run)]) == pytest.approx(angle, abs=0.001)
    wrist = report.report(FIXTURE / "gc25.jsonl")["joints"]["wrist_flex"]
    # Reversing ff would leave a +3.09 Nm mismatch, not improve the measured effort balance.
    assert wrist["mean_torque_minus_effort_nm"] + 2 * wrist["mean_ff_nm"] > 3
    # Before the ramp completes the wrist is already sagging more than in the baseline. During
    # the ramp, positive ff moves it toward the goal. Cross-run sag alone cannot identify its sign.
    hold = [r for r in ticks("gc25") if r["phase"] == "hold"]
    assert hold[0]["cmd"]["wrist_flex"] - hold[0]["pos"]["wrist_flex"] > 3
    assert hold[0]["pos"]["wrist_flex"] < settled("gc25")[-1]["pos"]["wrist_flex"]


def test_replay_original_gravity_terms_through_the_actual_command_hook():
    fit = json.loads((FIXTURE / "original-fit.json").read_text())
    # Deliberately construct the historical model for diagnosis: the command loader must refuse it.
    model = GravityModel(
        repo_path(fit["urdf"]), fit["joints"], params=fit["params"], joint_map=fit["joint_map"]
    )
    model.region_deg, model.fade_deg = fit["region_deg"], fit["fade_deg"]
    frames, now = [], [0.0]
    bus = SimpleNamespace(
        write=lambda *args: None,
        _get_motor_name=lambda n: n,
        _gains={n: {"kp": kp, "kd": 2} for n, kp in KP.items()},
        _mit_control=lambda *args: frames.append(args),
    )
    robot = SimpleNamespace(bus=bus, _last_positions=ticks("gc25")[0]["pos"], _turn_offset={})
    status = ff.install(robot, model, scale=0.25, clock=lambda: now[0])
    bus.write("Goal_Position", "shoulder_lift", -68.5)
    assert frames[-1][-1] == 0  # ramp starts at zero
    replay, logged, predictions = [], [], []
    first = ticks("gc25")[0]["t"]
    for row in settled("gc25"):
        if row["t"] < first + 3:
            continue  # full ramp only; archive has no per-frame send timestamps
        now[0] = row["t"]
        robot._last_positions = row["pos"]
        for n in KP:
            bus.write("Goal_Position", n, row["cmd"][n])
            assert frames[-1][-1] == status["feedforward_nm"][n]
        replay.append([status["feedforward_nm"][n] for n in list(KP)[1:4]])
        logged.append([row["ff"][n] for n in list(KP)[1:4]])
        predictions.append([model.torque(row["pos"])[n] for n in list(KP)[1:4]])
    # Reading precedes sending; the log is not a packet capture. The settled replay nevertheless
    # matches the archived command to 0.02 Nm, including the measured-pose cosine weight.
    np.testing.assert_allclose(replay, logged, atol=0.02, rtol=0)
    np.testing.assert_allclose(np.mean(predictions, axis=0), [10.387, 13.195, 6.717], atol=0.02, rtol=0)
    assert np.mean([r["w"] for r in settled("gc25")]) == pytest.approx(0.933, abs=0.001)


def test_original_fit_is_refused_at_both_command_entry_points(tmp_path, monkeypatch):
    path = FIXTURE / "original-fit.json"
    with pytest.raises(ValueError, match="offsets exceed"):
        ff.load_model(path)
    settings = SimpleNamespace(gravity_comp=0.25, sweep=False, gravity_fit=path, allow_provisional_fit=True)
    with pytest.raises(ValueError, match="offsets exceed"):
        gc.load(settings, "viscous_waterproof_01_dry_air", "dry_air")
    # Even clearing provisional must not bypass the same protection in Lab's env installer.
    fit = json.loads(path.read_text())
    fit["provisional"] = False
    path = tmp_path / "bad-fit.json"
    path.write_text(json.dumps(fit))
    monkeypatch.setenv(ff.ENV_FIT, str(path))
    robot = SimpleNamespace(bus=SimpleNamespace(_mit_control=None, _gains={}), _last_positions={})
    with pytest.raises(ValueError, match="offsets exceed"):
        ff.install_from_env(robot, fit["robot"])
    # Telemetry/contact prediction keeps the original offsets available for offline analysis.
    _, telemetry_model = ff.load_model(path, telemetry_offsets=True)
    assert telemetry_model.offsets[3] == pytest.approx(-5.543954476866824)


def test_corrected_fit_reduces_commanded_gravity_without_changing_joint_signs():
    cfg = SimpleNamespace(gravity_comp=0.25, sweep=False, gravity_fit=None, allow_provisional_fit=True)
    _, model = gc.load(cfg, "viscous_waterproof_01_dry_air", "dry_air")
    for name in ("baseline", "gc25"):
        rows = settled(name)
        pose = {n: float(np.median([r["pos"][n] for r in rows])) for n in KP}
        tau = model.torque(pose)
        assert 0 < tau["shoulder_lift"] < 4
        assert 0 < tau["elbow_flex"] < 6
        assert 0 < tau["wrist_flex"] < 2
    assert [model.joint_map[n]["sign"] for n in KP] == [1, 1, 1, 1, -1, 1]


@pytest.mark.parametrize("joint", list(KP))
def test_positive_mit_torque_is_encoded_positive_for_every_joint(joint):
    from lerobot.motors.robstride.robstride import RobstrideMotorsBus
    from lerobot.motors.robstride.tables import MotorType

    # No __init__, serial port, bus, robot, or connection. Encoder is the actual installed driver.
    bus = object.__new__(RobstrideMotorsBus)
    packet = bus._encode_mit_packet(MotorType.O0, KP[joint], 2, 0, 0, 1.0)
    tau = (((packet[6] & 15) << 8) | packet[7]) * 28 / 4095 - 14
    assert tau == pytest.approx(1.0, abs=28 / 4095)
    assert math.isfinite(tau)
