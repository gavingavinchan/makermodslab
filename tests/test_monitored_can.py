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

"""Real local socket and pinned driver, simulated CAN hardware only."""

import json
import math
import tempfile
import threading
from pathlib import Path
from types import SimpleNamespace

import can
import pytest

from makermodslab.monitored_can import MonitorCanBridge, MonitorCanClient, supervised_speed_limit


@pytest.fixture
def bridge(tmp_path):
    socket_dir = tempfile.TemporaryDirectory(prefix="can-test-", dir="/tmp")
    sent, records = [], []
    health = [True]
    server = MonitorCanBridge(
        Path(socket_dir.name) / "can.sock",
        tmp_path / "bridge.json",
        port="CAN",
        profile="test",
        send=lambda motor, data: sent.append((motor, data)),
        healthy=lambda: health[0],
        record=lambda event, **fields: records.append((event, fields)),
    )
    stop = threading.Event()

    def worker():
        while not stop.wait(0.001):
            server.pump()

    thread = threading.Thread(target=worker)
    thread.start()
    metadata = json.loads((tmp_path / "bridge.json").read_text())
    yield server, metadata, sent, records, health
    stop.set()
    thread.join(timeout=2)
    server.close()
    socket_dir.cleanup()


def test_only_one_serial_owner_and_fault_queries_stay_with_monitor(bridge):
    server, metadata, sent, _, _ = bridge
    client = MonitorCanClient(metadata, 7.5)
    try:
        packet = bytes.fromhex("FFFFFFFFFFFF00FB")
        client.send(can.Message(arbitration_id=7, is_extended_id=False, data=packet))
        assert sent == [(7, packet)]
        server.receive(0xFD, False, bytes.fromhex("0700000000"))
        assert client.recv(timeout=0.01) is None
        server.receive(0xFD, False, bytes.fromhex("077FFF7FF7FF0104"))
        reply = client.recv(timeout=0.1)
        assert bytes(reply.data) == bytes.fromhex("077FFF7FF7FF0104")
    finally:
        client.shutdown()


def test_unhealthy_status_refuses_motion_but_still_allows_disable(bridge):
    server, metadata, sent, _, health = bridge
    client = MonitorCanClient(metadata, 7.5)
    try:
        health[0] = False
        with pytest.raises(can.CanOperationError, match="fresh"):
            client.send(can.Message(arbitration_id=7, is_extended_id=False, data=bytes(8)))
        assert not sent
        disable = bytes.fromhex("FFFFFFFFFFFFFFFD")
        client.send(can.Message(arbitration_id=7, is_extended_id=False, data=disable))
        assert sent == [(7, disable)]
    finally:
        client.shutdown()


def test_zeroing_is_refused_without_transmitting(bridge):
    _, metadata, sent, _, _ = bridge
    client = MonitorCanClient(metadata, 7.5)
    try:
        with pytest.raises(can.CanError):
            client.send(
                can.Message(arbitration_id=7, is_extended_id=False, data=bytes.fromhex("FFFFFFFFFFFFFFFE"))
            )
        assert not sent
    finally:
        client.socket.close()


def test_old_browser_uses_slow_bridge_before_connecting_any_hardware(bridge, monkeypatch):
    from makermodslab import teleoperate
    from makermodslab.can_speed_limit import RateLimitedRobstrideBus

    server, _, _, _, _ = bridge
    monkeypatch.setenv("MAKERMODSLAB_CAN_BRIDGE", str(server.metadata_path))
    config = SimpleNamespace(port="CAN", id="test")

    class Robot:
        def __init__(self):
            self.config = config
            self.bus = object()

        def connect(self, **kwargs):
            assert isinstance(self.bus, RateLimitedRobstrideBus)
            assert self.bus.speed_limit_deg_s == 15

    robot = Robot()
    leader = SimpleNamespace(connect=lambda **kwargs: None)
    monkeypatch.setattr(teleoperate, "build_single_configs", lambda req: (config, object()))
    monkeypatch.setattr(teleoperate, "make_robot_from_config", lambda cfg: robot)
    monkeypatch.setattr(teleoperate, "make_teleoperator_from_config", lambda cfg: leader)
    request = teleoperate.TeleoperateRequest(
        arm_type="viscous",
        follower_port="CAN",
        follower_config="test",
        leader_port="LEADER",
        leader_config="leader",
    )
    result = teleoperate._connect_can(request)
    assert result[:2] == (robot, leader)
    assert request.speed_limit_deg_s == 15


def test_supervised_port_cannot_fall_back_on_wrong_profile_or_fast_mode(bridge, monkeypatch):
    server, _, sent, _, _ = bridge
    monkeypatch.setenv("MAKERMODSLAB_CAN_BRIDGE", str(server.metadata_path))
    with pytest.raises(ValueError, match="different commissioned arm"):
        MonitorCanClient.discover("CAN", "wrong", 7.5)
    with pytest.raises(ValueError, match="15"):
        supervised_speed_limit("CAN", "test", 30)
    assert supervised_speed_limit("CAN", "test", 2) == 2
    assert supervised_speed_limit("OTHER", "test", None) is None
    assert not sent


@pytest.mark.parametrize("speed", [7.5, 15.0])
def test_pinned_driver_reads_preloads_and_limits_targets_over_shared_connection(bridge, monkeypatch, speed):
    from lerobot.motors import Motor, MotorNormMode
    from lerobot.motors.robstride.robstride import RobstrideMotorsBus
    from makermodslab.can_speed_limit import RateLimitedRobstrideBus

    server, metadata, sent, _, _ = bridge
    positions = dict.fromkeys(range(1, 8), 10.0)

    def hardware(motor, data):
        sent.append((motor, data))
        if data[:6] != bytes([255] * 6):
            q = (data[0] << 8) | data[1]
            positions[motor] = math.degrees(-12.57 + q * 25.14 / 65535)
        q = int((math.radians(positions[motor]) + 12.57) / 25.14 * 65535)
        server.receive(0xFD, False, bytes((motor, q >> 8, q & 255, 0x7F, 0xF7, 0xFF, 1, 4)))

    server.send = hardware
    metadata_path = str(server.metadata_path)
    monkeypatch.setenv("MAKERMODSLAB_CAN_BRIDGE", metadata_path)
    motors = {("gripper" if i == 7 else f"m{i}"): Motor(i, "O0", MotorNormMode.DEGREES) for i in range(1, 8)}
    for motor in motors.values():
        motor.recv_id = motor.id
    base = RobstrideMotorsBus(port="CAN", motors=motors, can_interface="slcan", use_can_fd=False)
    monkeypatch.setattr(base, "connect", lambda **kw: pytest.fail("Must not open a second serial connection"))
    robot = SimpleNamespace(
        bus=base,
        config=SimpleNamespace(port="CAN", id="test", joint_limits=dict.fromkeys(motors, (-90, 90))),
        _turn_offset=dict.fromkeys(motors, 0),
    )
    now = [0.0]
    bus = RateLimitedRobstrideBus(robot, speed, clock=lambda: now[0])
    bus.connect()
    try:
        bus.enable_torque()
        first_enable = next(
            i for i, (_, data) in enumerate(sent) if data == bytes.fromhex("FFFFFFFFFFFFFFFC")
        )
        preloads = [row for row in sent[:first_enable] if row[1][:6] != bytes([255] * 6)]
        assert len(preloads) == 7
        initial = positions[1]
        now[0] = 0.02
        bus.write("Goal_Position", "m1", 80)
        assert positions[1] - initial == pytest.approx(speed * 0.02, abs=0.03)  # MIT encoder quantization
        bus.sync_write("Goal_Position", {"gripper": 80})
        assert positions[7] == pytest.approx(80, abs=0.03)  # Normal gripper target, no extra slew cap.
        assert server.enabled == set(range(1, 8))
        base.disable_torque()
        assert not server.enabled
        assert not server.alarm
    finally:
        base.disconnect()
