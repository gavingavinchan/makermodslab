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
"""Import paths and the offline harness model shared by plan, sim and the tests.

The astra operator harness (scripts/viscous-astra) is reused as is: its Model (tank URDF, motor
map, the station keep-out guard, the 15 deg/s route planner) and, through it, dream_real's trips
and stop path. Nothing here opens a port.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
ASTRA = ROOT / "scripts/viscous-astra"
for path in (str(ROOT), str(ASTRA), str(HERE)):
    if path not in sys.path:
        sys.path.insert(0, path)

import model as astra_model  # noqa: E402
from model import core, real  # noqa: E402


def profile_limits() -> dict:
    from make_tank_arm import PROFILE

    return json.loads(PROFILE.read_text())["follower"]["joint_limits"]


def offline_model(folder: Path, arm_file: Path | None = None, limits: dict | None = None):
    """The harness Model on the committed tank arm file (lab=False: no station approval check)."""
    folder.mkdir(parents=True, exist_ok=True)
    return astra_model.Model(
        arm_file or real.DEFAULT_ARM_FILE,
        real.ENVIRONMENTS["tank"],
        folder,
        limits or profile_limits(),
        False,
    )


def arm_file_rest(model) -> dict:
    return {**model.map.to_motor(model.spec["rest"]), "gripper": core.GRIP_CLOSED_DEG}
