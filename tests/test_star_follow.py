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

from types import SimpleNamespace

import pytest

from makermodslab.joint_posture_mapping import JOINTS, JointPostureMapper
from makermodslab.star_follow import StallWatch, StarFollower, StarFollowMapper, slew

# The waterproof pair's leader profile (viscous_profiles/star_viscous_waterproof_01.json).
DIRECTIONS = {
    "shoulder_pan": 1.446099,
    "shoulder_lift": -0.981723,
    "elbow_flex": -0.912081,
    "wrist_flex": -1.040071,
    "wrist_yaw": -1.24642,
    "wrist_roll": 0.945691,
    "gripper": -1.702354,
}
RANGES = {
    "shoulder_pan": [-126, 187],
    "shoulder_lift": [-199, -3],
    "elbow_flex": [3, 250],
    "wrist_flex": [-122, 82],
    "wrist_yaw": [-86, 87],
    "wrist_roll": [-154, 150],
    "gripper": [-115, 0],
}
OFFSETS = dict.fromkeys(DIRECTIONS, 0.0)


def mapper():
    return StarFollowMapper(DIRECTIONS, OFFSETS, RANGES)


def test_targets_round_trip_through_the_posture_mapping_inside_star_travel():
    forward = JointPostureMapper(DIRECTIONS, OFFSETS, dict.fromkeys(JOINTS, (-400.0, 400.0)))
    follower = {
        "shoulder_pan": 40.0,
        "shoulder_lift": -100.0,
        "elbow_flex": 200.0,
        "wrist_flex": -30.0,
        "wrist_yaw": 20.0,
        "wrist_roll": -60.0,
        "gripper": -50.0,
    }
    raw = mapper().leader_targets(follower)
    mapped, _, _ = forward.map(raw, {})
    for name in JOINTS:
        assert mapped[f"{name}.pos"] == pytest.approx(follower[name])
    # The elbow needs the multi-turn frame: past -180 deg of servo angle.
    assert raw["elbow_flex"] == pytest.approx(-200.0)
    # The gripper keeps the scaled stock preset.
    assert raw["gripper"] * DIRECTIONS["gripper"] == pytest.approx(-50.0)


def test_targets_clamp_to_star_travel_where_the_follower_reaches_further():
    m = mapper()
    raw = m.leader_targets({**dict.fromkeys(DIRECTIONS, 0.0), "shoulder_pan": 250.0, "wrist_yaw": -86.0})
    # Pan travel is 187 / 1.446 = 129.3 deg, minus the margin.
    assert raw["shoulder_pan"] == pytest.approx(187 / 1.446099 - 5.0)
    assert raw["wrist_yaw"] == pytest.approx(86 / 1.24642 - 5.0)
    assert m.travel["shoulder_pan"][1] < 250.0


def test_non_finite_follower_positions_are_refused():
    with pytest.raises(ValueError, match="shoulder_lift"):
        mapper().leader_targets({**dict.fromkeys(DIRECTIONS, 0.0), "shoulder_lift": float("nan")})


def test_single_turn_readings_unwrap_into_the_target_window():
    assert mapper().unwrap("elbow_flex", 160.0) == pytest.approx(-200.0)
    assert mapper().unwrap("shoulder_pan", 10.0) == pytest.approx(10.0)


def test_slew_bounds_each_step_both_ways():
    assert slew(0.0, 10.0, 3.0) == 3.0
    assert slew(0.0, -10.0, 3.0) == -3.0
    assert slew(0.0, 1.0, 3.0) == 1.0


def test_stall_watch_trips_only_after_a_sustained_lag():
    now = [0.0]
    watch = StallWatch(threshold_deg=20.0, hold_s=0.5, clock=lambda: now[0])
    assert watch.check({"elbow_flex": 25.0}) is None
    now[0] = 0.4
    assert watch.check({"elbow_flex": 25.0}) is None
    now[0] = 0.45
    assert watch.check({"elbow_flex": 5.0}) is None
    now[0] = 0.5
    assert watch.check({"elbow_flex": -25.0}) is None
    now[0] = 1.0
    assert watch.check({"elbow_flex": -25.0}) == "elbow_flex"


class FakeBus:
    def __init__(self, raw, multi_turn=None):
        self.raw, self.multi_turn = raw, multi_turn or raw
        self.sent, self.stops = [], []

    def read_angle(self, motor_id, multi_turn=True):
        return SimpleNamespace(raw_deg=self.multi_turn[motor_id])

    def set_angle(self, motor_id, angle, multi_turn=False, interval_ms=0):
        assert multi_turn
        self.sent.append((motor_id, angle))

    def set_stop_mode(self, motor_id, mode, power):
        self.stops.append((motor_id, mode, power))


def fake_leader(raw_by_name, multi_turn_by_name=None):
    ids = {name: i for i, name in enumerate(DIRECTIONS)}
    bus = FakeBus(
        {ids[n]: v for n, v in raw_by_name.items()},
        {ids[n]: v for n, v in (multi_turn_by_name or raw_by_name).items()},
    )
    leader = SimpleNamespace(config=SimpleNamespace(joint_ids=ids), bus=bus)
    leader._read_raw_positions = lambda: {n: bus.raw[ids[n]] for n in ids}
    return leader


def test_engage_refuses_a_multi_turn_frame_reset_away_from_the_fold():
    folded = dict.fromkeys(DIRECTIONS, 0.0)
    leader = fake_leader({**folded, "elbow_flex": 160.0}, {**folded, "elbow_flex": 160.0})
    with pytest.raises(RuntimeError, match="elbow_flex"):
        StarFollower(leader, mapper()).engage()
    assert leader.bus.sent == []


def test_step_slews_from_the_engaged_pose_and_release_frees_every_joint():
    now = [0.0]
    leader = fake_leader(dict.fromkeys(DIRECTIONS, 0.0))
    follower = StarFollower(leader, mapper(), speed_deg_s=30.0, clock=lambda: now[0])
    follower.engage()
    now[0] = 0.1
    follower.step({**dict.fromkeys(DIRECTIONS, 0.0), "shoulder_pan": 90.0})
    sent = dict(leader.bus.sent)
    assert sent[0] == pytest.approx(3.0)
    # Shoulder lift's travel excludes 0, so it slews toward its edge too, no faster.
    assert all(abs(angle) <= 3.0 + 1e-9 for angle in sent.values())
    assert follower.release()
    assert {stop[0] for stop in leader.bus.stops} == set(range(7))
    assert all(stop[1:] == (0x10, 0) for stop in leader.bus.stops)


def test_speed_is_bounded():
    with pytest.raises(ValueError):
        StarFollower(fake_leader(dict.fromkeys(DIRECTIONS, 0.0)), mapper(), speed_deg_s=200.0)


def test_travel_windows_sit_inside_the_posture_unwrap_windows():
    m = mapper()
    for name in JOINTS:
        lo, hi = m.travel[name]
        assert m.unwrap(name, lo) == pytest.approx(lo)
        assert m.unwrap(name, hi) == pytest.approx(hi)
