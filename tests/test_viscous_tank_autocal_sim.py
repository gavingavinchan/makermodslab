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
"""Exercise actual MuJoCo stop paths. Run with unittest in the separate physics venv."""

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parents[1] / "scripts/viscous-tank-autocal"


@unittest.skipUnless(importlib.util.find_spec("mujoco"), "requires the separate MuJoCo environment")
class SimAbortTest(unittest.TestCase):
    def test_faults_release_and_false_contacts_never_generate_candidates(self):
        sys.path.insert(0, str(HERE))
        self.addCleanup(sys.path.remove, str(HERE))
        import autocal

        expected = {
            "red_button": ("red button", "at the rest"),
            "second_signal": ("red button", "second signal"),
            "stale": ("No fresh feedback", "held"),
            "limp": ("limp", "held"),
            "wrist_post": ("limp", "held"),
            "missing_wall": ("done", "at the rest"),
            "floor_trap": ("done", "at the rest"),
        }
        with tempfile.TemporaryDirectory() as directory:
            for scenario, (reason, returned) in expected.items():
                with self.subTest(scenario=scenario):
                    run = autocal.simulate(
                        {"seed": 2, "backend": "mujoco", "scenario": scenario, "out": directory}
                    )
                    self.assertIn(reason, run["reason"])
                    self.assertIn(returned, run["stop"]["returned"])
                    self.assertTrue(run["released"])
                    self.assertIsNone(run["stop"]["release_problems"])
                    self.assertFalse(list(Path(run["folder"]).glob("candidate*.json")))
                    if scenario == "wrist_post":
                        self.assertGreater(run["contacts"]["nonfinger_contact_ticks"], 0)
                        self.assertEqual(run["touches"], 0)
                    if scenario == "missing_wall":
                        self.assertTrue(
                            any("no contact within the window" in why for why in run["abandoned"])
                        )
                        self.assertEqual(run["contacts"]["peak_force_n"], 0.0)
                    if scenario == "floor_trap":
                        self.assertIn("right", run["unplannable"])
                        self.assertTrue(all(label == "floor" for label, _ in run["touch_errors_mm"]))
