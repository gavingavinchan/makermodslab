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
"""Gripper haptics: the pure loop with fake buses and an injected clock, and
the raw Star bus's release-on-close from a worker thread."""

import struct
import threading
import time
from types import SimpleNamespace

import pytest

from makermodslab.fashionstar_raw import MONITOR, PING, RELEASE, STOP, Monitor
from makermodslab.gripper_haptics import (
    CALIB_S,
    MAX_LEAD_DEG,
    GripperHaptics,
    RawStarBus,
)

SCALE, OFFSET = -1.778452, -33.0  # the stock Viscous gripper mapping
LIMITS = (-120.0, -2.8)


class FakeRaw:
    def __init__(self):
        self.commands = []
        self.guard = None

    def angle(self, motor_id, target, duration_ms, power_mw, travel):
        travel.check(target)
        self.commands.append(("angle", motor_id, target, duration_ms, power_mw))

    def stop(self, motor_id, mode, power_mw=0):
        self.commands.append(("stop", motor_id, mode))


class Rig:
    """A follower gripper and a Star gripper the test moves by hand."""

    def __init__(self):
        self.t = 0.0
        self.state = {"position": OFFSET - 30.0, "velocity": 0.0, "torque": 0.1, "temp_mos": 30.0}
        bus = SimpleNamespace(
            last_feedback_time={"gripper": time.time()}, _last_known_states={"gripper": self.state}
        )
        self.robot = SimpleNamespace(
            bus=bus, config=SimpleNamespace(joint_limits={"gripper": LIMITS}), _turn_offset={"gripper": 0.0}
        )
        self.raw = FakeRaw()
        leader_bus = SimpleNamespace(monitors={}, fresh={}, raw=self.raw)
        self.leader = SimpleNamespace(
            bus=leader_bus,
            config=SimpleNamespace(
                joint_ids={"gripper": 6},
                joint_directions={"gripper": SCALE},
                joint_offsets={"gripper": OFFSET},
            ),
        )
        self.star(10.0)
        self.haptics = GripperHaptics(self.robot, self.leader, clock=lambda: self.t)

    def star(self, angle, temp=30.0, fresh=True):
        self.leader.bus.monitors[6] = Monitor(6, 12000, 40, 400, 2000, 0, angle, 0, temp)
        self.leader.bus.fresh[6] = fresh

    def tick(self, star, follower=None, velocity=0.0, torque=0.1, dt=0.02):
        """One teleop tick: the Star reads ``star``; the jaws end at ``follower``."""
        self.t += dt
        self.star(star)
        action = self.haptics.before_send({"gripper.pos": star * SCALE + OFFSET, "wrist_roll.pos": 1.0})
        self.state.update(
            position=action["gripper.pos"] if follower is None else follower, velocity=velocity, torque=torque
        )
        self.robot.bus.last_feedback_time["gripper"] = time.time()
        self.haptics.after_send()
        return action

    def calibrate(self):
        """Open and close freely (20+ deg of Star travel), then sit still mid-travel."""
        for i in range(int(CALIB_S / 0.02) + 30):
            star = 10.0 + 25.0 * (i % 100 < 50)
            self.tick(star, velocity=0.0, torque=0.1 + 0.01 * (i % 3))
        assert self.haptics.phase == "on"


def test_only_the_gripper_target_changes_and_never_past_the_closed_stop():
    rig = Rig()
    # The Star a little past closed maps beyond the follower's real stop.
    action = rig.tick(-5.0)
    assert action["gripper.pos"] <= OFFSET
    assert action["wrist_roll.pos"] == 1.0


def test_squeeze_cap_applies_only_while_the_jaws_are_still():
    rig = Rig()
    rig.haptics.phase = "on"
    rig.state.update(position=-80.0, velocity=0.0)
    rig.robot.bus.last_feedback_time["gripper"] = time.time()
    still = rig.haptics.before_send({"gripper.pos": -40.0})
    assert still["gripper.pos"] == pytest.approx(-80.0 + MAX_LEAD_DEG)
    rig.state.update(velocity=200.0)
    moving = rig.haptics.before_send({"gripper.pos": -40.0})
    assert moving["gripper.pos"] == -40.0


def test_calibration_waits_for_travel_and_measures_a_threshold():
    rig = Rig()
    for _ in range(int(CALIB_S / 0.02) + 30):
        rig.tick(10.0)
    assert rig.haptics.phase == "calibrating"  # no travel yet
    rig.calibrate()
    assert rig.haptics.threshold >= 0.3
    assert rig.raw.guard is not None
    assert not any(c[0] == "angle" for c in rig.raw.commands)


def test_contact_renders_a_wall_and_releases_when_the_hand_opens():
    rig = Rig()
    rig.calibrate()
    wall = 20.0 * SCALE + OFFSET  # the jaws stop on an object at Star 20 deg
    # A single squeezed tick is not contact (persistence).
    rig.tick(20.0, follower=wall, torque=3.0)
    assert not rig.haptics.latched
    for _ in range(4):
        rig.tick(20.0, follower=wall, torque=3.0)
    assert rig.haptics.latched
    # The hand closes the Star 5 deg past the wall: the Star pushes back.
    for _ in range(10):
        rig.tick(15.0, follower=wall, torque=3.0)
    angles = [c for c in rig.raw.commands if c[0] == "angle"]
    assert angles and all(c[1] == 6 for c in angles)
    assert angles[-1][2] == pytest.approx(20.0, abs=0.2)
    assert all(100 <= c[4] <= 600 for c in angles)
    powers = [c[4] for c in angles]
    assert powers == sorted(powers)  # power only rises while pressed
    # The hand opens past the wall: released at once, contact ends.
    rig.tick(24.0, follower=wall, torque=3.0)
    assert rig.raw.commands[-1] == ("stop", 6, RELEASE)
    assert not rig.haptics.latched
    assert rig.leader.gripper_haptics_status == {
        "phase": "on",
        "contact": False,
        "power_mw": 0,
        "reason": None,
    }


def test_no_contact_at_the_closed_stop():
    rig = Rig()
    rig.calibrate()
    for _ in range(20):
        rig.tick(0.0, follower=OFFSET - 1.0, torque=3.0)
    assert not rig.haptics.latched


def test_stale_star_reading_releases_and_a_hot_servo_turns_haptics_off():
    rig = Rig()
    rig.calibrate()
    wall = 20.0 * SCALE + OFFSET
    for _ in range(5):
        rig.tick(20.0, follower=wall, torque=3.0)
    rig.tick(15.0, follower=wall, torque=3.0)
    assert rig.haptics.star_on
    rig.star(15.0, fresh=False)
    rig.haptics.after_send()
    assert not rig.haptics.star_on and rig.raw.commands[-1] == ("stop", 6, RELEASE)
    rig.star(15.0, temp=56.0)
    rig.haptics.after_send()
    assert rig.haptics.phase == "off"
    assert "Star gripper at 56.0 C" in rig.leader.gripper_haptics_status["reason"]


def test_a_fault_turns_haptics_off_without_raising():
    rig = Rig()
    rig.calibrate()

    def broken(*_args, **_kwargs):
        raise RuntimeError("serial gone")

    rig.raw.angle = broken
    wall = 20.0 * SCALE + OFFSET
    for _ in range(5):
        rig.tick(20.0, follower=wall, torque=3.0)
    rig.tick(15.0, follower=wall, torque=3.0)
    assert rig.haptics.phase == "off"
    assert "serial gone" in rig.leader.gripper_haptics_status["reason"]


def test_close_writes_the_tick_log(tmp_path):
    rig = Rig()
    rig.haptics.log_dir = str(tmp_path)
    rig.tick(10.0)
    rig.haptics.close()
    assert rig.haptics.phase == "off"
    assert len(list(tmp_path.glob("gripper-haptics-*.csv.gz"))) == 1


class FakeSerial:
    def __init__(self, *args, **kwargs):
        self.writes, self.rx, self.closed = [], b"", False

    def reset_input_buffer(self):
        self.rx = b""

    def write(self, packet):
        self.writes.append(packet)
        command, motor_id = packet[2], packet[4]
        if command in (PING, MONITOR):
            payload = (
                bytes((motor_id,))
                if command == PING
                else struct.pack("<BHHHHBih", motor_id, 12000, 30, 360, 1947, 0, 0, 0)
            )
            body = b"\x05\x1c" + bytes((command, len(payload))) + payload
            self.rx = body + bytes((sum(body) % 256,))
        return len(packet)

    def read(self, _size):
        data, self.rx = self.rx[:1], self.rx[1:]
        return data

    def close(self):
        self.closed = True


def test_raw_star_bus_releases_all_seven_when_closed_from_a_worker_thread():
    serial = FakeSerial()
    bus = RawStarBus("/dev/fake", 1_000_000, serial_factory=lambda *a, **kw: serial)
    assert bus.ping(6)
    assert bus.sync_monitor([0, 6])[6].angle_deg == 0.0 and bus.fresh[6]
    worker = threading.Thread(target=bus.close)
    worker.start()
    worker.join()
    released = [p[4] for p in serial.writes if p[2] == STOP and p[5] == RELEASE]
    assert sorted(released) == sorted(list(range(7)) * 2)
    assert serial.closed and not bus.is_open


def test_raw_star_bus_refuses_holding_any_servo():
    serial = FakeSerial()
    bus = RawStarBus("/dev/fake", 1_000_000, serial_factory=lambda *a, **kw: serial)
    with pytest.raises(Exception, match="only releases"):
        bus.set_stop_mode(3, 0x11, 200)
    bus.close()


def test_haptic_star_leader_reads_all_seven_joints_and_releases_on_disconnect(tmp_path):
    from lerobot.teleoperators.rebot_102_leader import RebotArm102LeaderViscousTeleopConfig
    from makermodslab.gripper_haptics import HapticStarLeader

    serial = FakeSerial()
    config = RebotArm102LeaderViscousTeleopConfig(
        port="/dev/fake", id="star_viscous", calibration_dir=tmp_path
    )
    leader = HapticStarLeader(config, serial_factory=lambda *a, **kw: serial)
    leader.connect(calibrate=False)
    action = leader.get_action()
    assert set(action) == {f"{name}.pos" for name in config.joint_ids}
    assert action["gripper.pos"] == pytest.approx(-33.0)  # Star closed (0 deg) lands on the closed stop
    assert leader.bus.fresh[6]
    leader.disconnect()
    released = [p[4] for p in serial.writes if p[2] == STOP and p[5] == RELEASE]
    assert released.count(6) >= 3  # configure, disconnect, then the bus's two-pass close
    assert serial.closed and leader.bus is None


def test_a_stalled_start_is_not_measured_as_free_motion():
    """2026-10-28: the follower started 27 deg from the Star's pose, stalled, and
    the stall's 2.75 N m became the baseline. Calibration neither caps the jaws
    nor counts a jaw straining toward a far target."""
    rig = Rig()
    rig.state.update(position=-60.0)
    rig.robot.bus.last_feedback_time["gripper"] = time.time()
    action = rig.haptics.before_send({"gripper.pos": -33.0})
    assert action["gripper.pos"] == -33.0  # no squeeze cap while calibrating
    for _ in range(100):
        rig.tick(0.5, follower=-54.0, torque=2.75)  # stuck far from its command
    assert rig.haptics._still_tau == []
    rig.calibrate()
    assert rig.haptics.baseline < 0.5
