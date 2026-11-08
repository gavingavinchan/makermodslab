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
"""Offline probe/FK, noisy identifiable fits and export compatibility."""

import importlib.util
import json
import sys

import numpy as np
import pytest

from makermodslab import tank_geometry as g


@pytest.fixture(scope="module")
def offline():
    spec = importlib.util.spec_from_file_location(
        "tank_offline_reference", g.ROOT / "scripts/viscous_aquarium_env.py"
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def tank():
    return json.loads(g.DEFAULT_TANK.read_text())


def synthetic(yaw=0.0, near=40.0, left=217.0, right=-217.0, floor=0.0, noise=0.0, seed=7):
    rng = np.random.default_rng(seed)
    points = {
        "left": [[-60, left, 100], [-410, left, 180]],
        "right": [[-60, right, 130], [-410, right, 160]],
        "near": [[near, 110, 180], [near, -110, 180]],
        "floor": [[-200, 0, floor]],
    }
    return [
        {
            "label": face,
            "point_mm": (np.array(p) @ g.tank_axes(yaw) + rng.normal(0, noise, 3)).tolist(),
            "positions": {},
            "timestamp": "2026-11-07T00:00:00+00:00",
        }
        for face, ps in points.items()
        for p in ps
    ]


@pytest.mark.parametrize("pose", ["read", "folded"])
def test_fk_agrees_with_offline_every_link_and_probe(offline, tank, pose):
    ours, theirs = g.load_model(tank), offline.load_model(tank)
    angles = tank["reference_poses_motor_deg"][pose]["value"]
    actual = g.forward_kinematics(ours, angles)
    expected = offline.forward_kinematics(theirs, angles)
    for link in expected:
        np.testing.assert_allclose(actual[link][1] * 1000, expected[link][1] * 1000, atol=0.5)
        np.testing.assert_allclose(actual[link][0], expected[link][0], atol=1e-12)
    # Independently select the mesh extreme in the reference transforms.
    axis = next(j.t for j in theirs.joints if j.name == "tool_fixed")
    axis /= np.linalg.norm(axis)
    tips = []
    for link in ("gripper_left", "gripper_right"):
        v = theirs.vertices[link]
        dot = v @ axis
        tip = v[dot >= max(dot) - 0.00001].mean(0)
        r, t = expected[link]
        tips.append((r @ tip + t) * 1000)
    np.testing.assert_allclose(g.probe_point(ours, actual), np.mean(tips, axis=0), atol=0.5)
    target = [-520.150, 101.963, 42.820] if pose == "read" else [207.979, 82.008, 148.941]
    np.testing.assert_allclose(actual["tool0"][1] * 1000, target, atol=0.001)


def test_tip_extremes_and_opening(tank):
    model = g.load_model(tank)
    tips = g.probe_local_points(model)
    np.testing.assert_allclose(np.mean(list(tips.values()), axis=0) * 1000, [166.308, 0, -27.091], atol=0.001)
    pose = tank["reference_poses_motor_deg"]["read"]["value"]
    assert g.gripper_opening_mm(model, pose) == pytest.approx(52.4)
    assert g.gripper_opening_mm(model, {**pose, "gripper": 0}) == 0


def test_noisy_random_tank_recovery(tank):
    rng = np.random.default_rng(123)
    for seed in range(40):
        yaw = rng.uniform(-5, 5)
        near, left, right, floor = np.array([40, 217, -217, 0]) + rng.uniform(-30, 30, 4)
        fit = g.fit_tank(synthetic(yaw, near, left, right, floor, noise=0.5, seed=seed), tank)
        assert fit["yaw_deg"] == pytest.approx(yaw, abs=0.3)
        for face, expected in zip(
            ("near", "left", "right", "floor"), (near, left, right, floor), strict=True
        ):
            assert fit["offsets_mm"][face] == pytest.approx(expected, abs=1.5)


@pytest.mark.parametrize("case", ["missing", "span", "width", "residual", "boundary"])
def test_gates_refuse_bad_measurements(tank, case):
    touches = synthetic()
    if case == "missing":
        touches.pop(0)
    if case == "span":
        touches[1]["point_mm"][0] = -70
    if case == "width":
        touches = synthetic(left=240)
    if case == "residual":
        touches.append({"label": "check:near", "point_mm": [55, 0, 100]})
    if case == "boundary":
        touches = synthetic(yaw=24)
    fit = g.fit_tank(touches, tank)
    assert not fit["valid"]
    assert fit["failures"]
    with pytest.raises(ValueError):
        g.export_tank(tank, fit, touches, None, {}, "now")


def test_check_is_held_out_and_change_direction(tank):
    touches = synthetic()
    before = g.fit_tank(touches, tank)
    previous = {"measured": before}
    touches = synthetic(near=30)
    touches.append({"label": "check:near", "point_mm": [32, 0, 100]})
    fit = g.fit_tank(touches, tank, previous)
    assert fit["offsets_mm"]["near"] == pytest.approx(30, abs=1e-5)
    assert fit["residuals"][-1]["residual_mm"] == pytest.approx(2, abs=1e-5)
    assert fit["change"]["toward"] == "near"
    assert fit["change"]["translation_mm"] == pytest.approx(10, abs=1e-5)


def test_export_loads_at_zero_yaw(offline, tank, tmp_path):
    fit = g.fit_tank(synthetic(left=219, right=-218, floor=-3), tank)
    fit["yaw_deg"] = 0.0
    exported = g.export_tank(tank, fit, synthetic(), None, {}, "2026-11-07T00:00:00+00:00")
    path = tmp_path / "tank.json"
    path.write_text(json.dumps(exported))
    assert offline.load_tank(path)["placement"]["near_inner_x_mm"]["status"] == "measured"
    assert exported["measured"]["far_status"] == "estimated"
    assert exported["placement"]["base_rpy_deg"]["value"] == [0, 0, 0]
    fit["yaw_deg"] = 2.0
    assert g.export_tank(tank, fit, [], None, {}, "now")["placement"]["base_rpy_deg"]["value"] == [0, 0, 2]


def test_tower_padding_and_all_moving_link_clearances(tank):
    model = g.load_model(tank)
    touches = synthetic()
    fit = g.fit_tank(touches, tank)
    for label, p in [
        ("front", [10, 0, 50]),
        ("top", [20, 0, 150]),
        ("left", [20, 40, 50]),
        ("right", [20, -40, 50]),
    ]:
        touches.append({"label": "tower_" + label, "point_mm": p})
    tower = g.tower_box(touches, fit)
    np.testing.assert_allclose(tower["min_mm"], [10, -40, 0], atol=1e-5)
    pose = tank["reference_poses_motor_deg"]["read"]["value"]
    touches.append({"label": "envelope:yaw_link:near", "positions": pose})
    pads = g.cable_paddings(model, touches, fit, {})
    assert pads["yaw_link"]["status"] == "measured"
    assert pads["yaw_link"]["padding_mm"] == pytest.approx(8.979, abs=0.002)
    assert pads["forearm_link"] == {"padding_mm": 40.0, "status": "default"}
    result = g.live_clearances(model, tank, pose, fit, pads, tower)
    assert set(result["links"]) == model.moving_links & model.vertices.keys()
    assert result["floor_informational"]
    assert result["links"]["yaw_link"]["distances_mm"]["near"] == pytest.approx(0, abs=1e-5)


def test_nonfinite_touch_is_refused_including_check_points(tank):
    touches = synthetic() + [{"label": "check:near", "point_mm": [float("nan"), 0, 0]}]
    with pytest.raises(ValueError, match="finite"):
        g.fit_tank(touches, tank)
