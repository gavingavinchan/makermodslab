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
"""Offline checks of signed static equivalence, CAD accounting and guard meshes."""

import json
import sys
from pathlib import Path

import numpy as np
import pytest

from makermodslab import arm_gravity, tank_geometry as geo

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts/viscous-buoyancy"))

from hydro import CORNERS, JOINTS, OUT, RHO, URDF, block_load, mass_loads  # noqa: E402
from static_urdf import xml_output  # noqa: E402


def sidecar(name):
    return json.loads((OUT / f"{name}.json").read_text())


@pytest.mark.parametrize("name", ["current", "foamed", "wrist_alternative"])
def test_static_urdf_matches_independent_archimedes_wrenches(name):
    side = sidecar(name)
    assert side["schema"] == "viscous-buoyancy/1"
    assert arm_gravity.file_sha256(URDF) == side["urdf_sha256"]
    assert_static_equivalence(side)


def assert_static_equivalence(side):
    pytest.importorskip("pinocchio")
    model = geo.load_model(geo.load_tank())
    rng = np.random.default_rng(20261118)
    poses = rng.uniform(-180, 180, (192, 7))
    poses[:, 6] = model.gripper_motor_closed_deg
    path = ROOT / side["gravity_only_urdf"]
    pin = arm_gravity.GravityModel(path, JOINTS, joint_map=model.motor_to_urdf)
    actual = np.array([list(pin.torque(dict(zip(JOINTS, q[:6], strict=True))).values()) for q in poses])
    dry, water = mass_loads(model, poses, side["links"], submerged=True)
    expected = dry - water @ np.array([RHO * link["volume_m3"] for link in side["links"].values()])
    for b in side["foam_blocks"]:
        expected += block_load(model, poses, b, submerged=True) * b["volume"]
    np.testing.assert_allclose(actual, expected, atol=1e-6, rtol=0)
    return np.max(abs(actual - expected))


def test_exactly_neutral_and_positive_buoyancy_keep_first_moments(tmp_path):
    side = sidecar("current")
    # A neutral body can still exert a couple when COM and COB differ.
    for n, factor in [("upper_arm_link", 1.0), ("forearm_link", 1.8), ("gripper_left", 1.15)]:
        link = side["links"][n]
        link["volume_m3"] = factor * link["mass_kg"] / RHO
        assert np.linalg.norm(np.array(link["com_m"]) - link["cob_m"]) > 1e-4
    side["gravity_only_urdf"] = str(tmp_path / "signed.urdf")
    xml_output(side)
    assert_static_equivalence(side)


@pytest.mark.parametrize("variant", ["foamed", "wrist_alternative"])
def test_cad_first_moments_and_guard_geometry(variant):
    current = sidecar("current")
    side = sidecar(variant)
    assert side["links"] == current["links"]
    assert current["foam_blocks"] == []
    guard = geo.load_model(geo.load_tank() | {"arm_urdf": side["gravity_only_urdf"]})
    for link in side["links"].values():
        np.testing.assert_allclose(np.mean(link["volume_samples"], axis=0), link["cob_m"], rtol=0, atol=1e-12)
    for b in side["foam_blocks"]:
        assert b["volume"] == pytest.approx(np.prod(b["size_m"]))
        assert b["mass"] == pytest.approx(b["density"] * b["volume"])
        vertices = geo.stl_vertices(ROOT / b["mesh"])
        np.testing.assert_allclose(np.ptp(vertices, axis=0), b["size_m"], atol=1e-8)
        corners = CORNERS * b["size_m"] + b["center_m"]
        for vertex in corners:
            assert np.linalg.norm(guard.vertices[b["link"]] - vertex, axis=1).min() < 1e-8
    if variant == "wrist_alternative":
        assert guard.vertices["wrist_roll_link"][:, 1].max() == pytest.approx(0.1275)
