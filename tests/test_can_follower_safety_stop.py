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

"""The CAN follower's own session-ending stops must reach the teleop loop.

On 2026-11-05 a Viscous shoulder held torque to 101 C because the thermal stop
was raised inside the joint-broadcast read and logged as a read error.
"""

from unittest.mock import Mock

import pytest

from makermodslab.arms.maker import MAKER
from makermodslab.teleoperate import get_can_joint_data, read_can_observation

# The exact wording lerobot's maker_follower raises (the Viscous follower inherits it).
THERMAL = (
    "viscous_01 ViscousFollower thermal stop: shoulder_lift at 85.0 C for 3 consecutive readings "
    "(temp_stop_c=85 C). Ending the session; torque is released and the arm has no brakes, so it drops."
)
SILENT = (
    "viscous_01 ViscousFollower has no CAN feedback from any motor: elbow_flex (30 cycles) — check "
    "arm power / CAN adapter."
)


@pytest.mark.parametrize("message", [THERMAL, SILENT])
def test_follower_safety_stops_propagate(message):
    robot = Mock()
    robot.get_observation.side_effect = RuntimeError(message)
    with pytest.raises(RuntimeError, match="thermal stop|no CAN feedback from any motor"):
        read_can_observation(robot)
    with pytest.raises(RuntimeError):
        get_can_joint_data(robot, MAKER, False, 0.0)


def test_other_read_errors_still_hold_pose():
    robot = Mock()
    robot.get_observation.side_effect = RuntimeError("camera wrist timed out")
    assert read_can_observation(robot) == {}
    data = get_can_joint_data(robot, MAKER, False, 0.0)
    assert data["joints_deg"] == {}


def test_supplied_observation_is_not_reread():
    robot = Mock()
    get_can_joint_data(robot, MAKER, False, 0.0, observation={"shoulder_pan.pos": 1.0})
    robot.get_observation.assert_not_called()
