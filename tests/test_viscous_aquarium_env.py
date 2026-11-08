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
"""Hardware-free geometry tests; unittest avoids importing Lab's pytest fixtures.

Run: .venv/bin/python -m unittest discover -s tests -p test_viscous_aquarium_env.py -v
Also collectable by pytest. No MuJoCo dependency.
"""

import copy
import json
import math
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import patch

import numpy as np

from scripts import viscous_aquarium_env as env


class AquariumGeometryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tank = env.load_tank()
        cls.model = env.load_model(cls.tank)
        cls.read = cls.tank["reference_poses_motor_deg"]["read"]["value"]
        cls.folded = cls.tank["reference_poses_motor_deg"]["folded"]["value"]

    def test_tank_bounds_consistent_and_provenanced(self):
        b = self.tank["derived_inner_bounds_mm"]["value"]
        self.assertEqual(b, env.derive_bounds(self.tank))
        self.assertEqual(
            b, {"x_min": -694, "x_max": 40, "y_min": -217, "y_max": 217, "z_min": 0, "z_max": 442}
        )
        for value in self.tank["placement"].values():
            self.assertIn(value["status"], ("estimate", "measured"))
            self.assertTrue(value["source"])
        self.assertEqual(self.tank["water_height_mm"]["status"], "design")

    def test_stale_derived_bounds_rejected(self):
        tank = copy.deepcopy(self.tank)
        tank["placement"]["near_inner_x_mm"]["value"] = 50
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "tank.json"
            path.write_text(json.dumps(tank))
            with self.assertRaisesRegex(ValueError, "Stale derived bound"):
                env.load_tank(path)

    def test_floor_is_collision_base_minimum_minus_spacer(self):
        low = min(self.model.vertices["base_link"][:, 2]) * 1000
        self.assertAlmostEqual(low, self.tank["placement"]["floor_z_mm"]["value"])
        tank = copy.deepcopy(self.tank)
        tank["placement"]["base_spacer_height_mm"]["value"] = 5
        with self.assertRaisesRegex(ValueError, "floor_z_mm disagrees"):
            env.load_model(tank)

    def test_read_pose_tool0_within_5_mm(self):
        tool = env.forward_kinematics(self.model, self.read)["tool0"][1] * 1000
        self.assertLess(np.linalg.norm(tool - [-526, -65, 43]), 5)

    def test_folded_pose_tool0_within_5_mm(self):
        tool = env.forward_kinematics(self.model, self.folded)["tool0"][1] * 1000
        self.assertLess(np.linalg.norm(tool - [185, 125, 149]), 5)

    def test_fk_matches_independent_pinocchio_all_links(self):
        import pinocchio as pin

        model = pin.buildModelFromUrdf(str(self.model.path))
        data = model.createData()
        for pose in (self.read, self.folded):
            q = pin.neutral(model)
            for name, value in env.joint_positions(self.model, pose).items():
                q[model.idx_qs[model.getJointId(name)]] = value
            pin.framesForwardKinematics(model, data, q)
            for name, (r, t) in env.forward_kinematics(self.model, pose).items():
                actual = data.oMf[model.getFrameId(name)]
                np.testing.assert_allclose(t, actual.translation, atol=1e-10)
                np.testing.assert_allclose(r, actual.rotation, atol=1e-10)

    def test_read_pose_no_penetration_and_all_moving_links_positive(self):
        # Strictly positive floor clearance is impossible for a base on the floor.
        # Preserve the full-arm result; test those intentional tangencies explicitly.
        c = env.clearance(self.model, self.tank, self.read)
        for wall, result in c.items():
            self.assertGreaterEqual(result["distance_mm"], -env.EPS_MM, wall)
        for wall in ("floor", "near"):
            self.assertEqual(c[wall]["link"], "base_link")
            self.assertAlmostEqual(c[wall]["distance_mm"], 0, delta=env.EPS_MM)
        moving = env.clearance(self.model, self.tank, self.read, exclude_links=("base_link",))
        self.assertTrue(all(v["distance_mm"] > 0 for v in moving.values()))

    def test_folded_target_penetrates_near_wall(self):
        c = env.clearance(self.model, self.tank, self.folded)
        self.assertLess(c["near"]["distance_mm"], -170)
        self.assertEqual(c["near"]["link"], "gripper_right")

    def test_gripper_linear_mapping_clamps_overtravel(self):
        for degrees, expected in ((-117.9, 0.0524), (-115, 0.0524), (-57.5, 0.0262), (0, 0), (10, 0)):
            q = env.joint_positions(self.model, {**self.read, "gripper": degrees})
            self.assertAlmostEqual(q["gripper_left_joint"], expected)
            self.assertEqual(q["gripper_left_joint"], q["gripper_right_joint"])
        self.assertAlmostEqual(
            env.joint_positions(self.model, self.read)["shoulder_pan"], math.radians(182.02)
        )

    def test_invalid_motor_angles_rejected(self):
        for pose in ({}, {**self.read, "wrist_roll": float("nan")}, {**self.read, "extra": 0}):
            with self.assertRaisesRegex(ValueError, "six arm joints"):
                env.clearance(self.model, self.tank, pose)

    def test_optional_waterline_is_visual_headroom(self):
        tank = copy.deepcopy(self.tank)
        tank["water_height_mm"]["value"] = None
        c = env.clearance(self.model, tank, self.read)
        self.assertNotIn("water_line", c)
        self.assertEqual(set(c), set(env.WALLS))

    def test_default_margin_sweep_is_empty_and_flags_reference_limits(self):
        result = env.sweep(self.model, self.tank, self.read, 30)
        self.assertTrue(all(row["safe_sample_intervals_deg"] == [] for row in result["rows"]))
        self.assertEqual(set(result["reference_outside_limits"]), {"shoulder_lift", "gripper"})
        self.assertEqual(result["excluded_links"], [])
        self.assertNotIn("water_line", result["constraints"])

    def test_zero_margin_sweep_stops_before_floor_penetration(self):
        # A small real J4 interval straddles the floor crossing. Other joint
        # intervals are kept tiny so this path test remains fast.
        limits = {j: [self.read[j], self.read[j]] for j in (*env.JOINTS, "gripper")}
        limits["wrist_flex"] = [36.42, 42.42]
        with patch.object(env, "soft_limits", return_value=limits):
            result = env.sweep(self.model, self.tank, self.read, 0)
        row = next(row for row in result["rows"] if row["joint"] == "wrist_flex")
        np.testing.assert_allclose(row["safe_sample_intervals_deg"], [[38.42, 42.42]])

    def test_scene_assets_are_relative_and_glass_contact_masks_match(self):
        scene = env.ENV_DIR / "aquarium_scene.xml"
        root = ET.parse(scene).getroot()
        for mesh in root.findall("asset/mesh"):
            filename = Path(mesh.attrib["file"])
            self.assertFalse(filename.is_absolute())
            self.assertTrue((scene.parent / filename).is_file())
        glass = [g for g in root.findall("worldbody/geom") if g.get("name", "").startswith("glass_")]
        self.assertEqual(len(glass), 5)
        for g in glass:
            self.assertEqual((g.get("contype"), g.get("conaffinity")), ("2", "1"))
        water = root.find("worldbody/geom[@name='water_line']")
        self.assertEqual((water.get("contype"), water.get("conaffinity")), ("0", "0"))


if __name__ == "__main__":
    unittest.main()
