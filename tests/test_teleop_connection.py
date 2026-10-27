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
"""Descriptor snapshots and real driver handshakes on fake wire transports."""

import errno
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from makermodslab import teleop_connection as connection, usb_ports
from makermodslab.api_errors import ApiError
from makermodslab.arms.maker import MAKER
from makermodslab.arms.metal import METAL
from makermodslab.arms.so101 import SO101
from makermodslab.arms.viscous import VISCOUS

ROOT = Path(__file__).resolve().parents[1]
DRY = "207E37A55445"
WET = "205038953846"


def port(path, serial=DRY, model=connection.CANABLE_MODEL, **kwargs):
    return SimpleNamespace(device=path, serial_number=serial, vid=model[0], pid=model[1], **kwargs)


def record(config="viscous_01"):
    return {
        "mode": "single",
        "follower_config": config,
        "leader_config": "star_viscous",
        "follower_port": "/dev/stale",
        "leader_port": "/dev/stale-star",
    }


def resolve(rec=None, ports=None, **kwargs):
    return connection.resolve_start_ports(
        rec or record(),
        VISCOUS,
        enumerate_ports=lambda: (
            ports if ports is not None else [port("new-can"), port("new-star", "", connection.STAR_MODEL)]
        ),
        **kwargs,
    )


@pytest.mark.parametrize("saved", ["/dev/tty.usbmodem-old", "/dev/serial/by-id/usb-old", ""])
def test_every_click_reenumerates_and_never_opens_saved_path(saved):
    rec = record()
    rec["follower_port"] = saved
    live = [port("/dev/cu.usbmodem-new"), port("/dev/cu.wchusbserial-new", "", connection.STAR_MODEL)]
    assert resolve(rec, live)["follower_port"] == live[0].device
    assert resolve(rec, live)["leader_port"] == live[1].device
    live[0].device = "renamed-can"
    assert resolve(rec, live)["follower_port"] == "renamed-can"
    assert rec["follower_port"] == saved


def test_aliases_collapse_without_following_symlinks():
    live = [
        port("/dev/tty.can"),
        port("/dev/cu.can"),
        port("/dev/serial/by-id/can", location="1-2", hwid="LINK=/dev/ttyACM0"),
        port("/dev/ttyACM0", location="1-2", hwid="USB"),
        port("star", "", connection.STAR_MODEL),
    ]
    unique = usb_ports.unique_ports(live)
    assert len(unique) == 3
    assert "/dev/ttyACM0" in [p.device for p in unique]
    assert len(usb_ports.unique_ports(live[:2])) == 1


@pytest.mark.parametrize(
    "live,code",
    [
        ([], "robot.follower_usb_missing"),
        ([port("can")], "robot.leader_usb_missing"),
        (
            [
                port("can"),
                port("star-a", "", connection.STAR_MODEL),
                port("star-b", "", connection.STAR_MODEL),
            ],
            "robot.connection_ambiguous",
        ),
        (
            [port("can-a"), port("can-b"), port("star", "", connection.STAR_MODEL)],
            "robot.connection_ambiguous",
        ),
    ],
)
def test_missing_and_ambiguous_exact_codes(live, code):
    with pytest.raises(ApiError) as caught:
        resolve(ports=live)
    assert caught.value.code == code
    assert caught.value.detail == connection.MESSAGES[code]


def test_enumeration_failure_never_uses_stale_port():
    with pytest.raises(ApiError) as caught:
        connection.resolve_start_ports(record(), VISCOUS, enumerate_ports=lambda: None)
    assert caught.value.code == "robot.connection_failed"
    assert caught.value.details == {"reason": "usb_enumeration_failed"}


def test_linux_snapshot_mismatch_override_is_per_start_and_does_not_change_settings():
    path = (
        ROOT
        / "configs/viscous/state-snapshots/2026-10-27/gavin-linux/.makermods/makermodslab-viscous/robots/viscous_waterproof_01.json"
    )
    rec = json.loads(path.read_text())
    before = dict(rec)
    for _ in range(2):
        with pytest.raises(ApiError) as caught:
            resolve(rec)
        error = caught.value
        assert error.code == "robot.unit_mismatch"
        assert error.details["observed_unit"] == "viscous_01"
        assert error.details["settings_unit"] == "viscous_waterproof_01"
        assert DRY in error.detail and "viscous_waterproof_01" in error.detail
        assert resolve(rec, override=True)["follower_config"] == "viscous_waterproof_01"
    assert rec == before


def test_unenrolled_adapter_requires_explicit_override():
    live = [port("replacement", "unknown"), port("star", "", connection.STAR_MODEL)]
    with pytest.raises(ApiError) as caught:
        resolve(ports=live)
    assert caught.value.code == "robot.identity_unverified"
    assert caught.value.details["observed_unit"] is None
    assert caught.value.details["settings_unit"] == "viscous_01"
    assert resolve(ports=live, override=True)["follower_port"] == "replacement"


def test_enrollment_serials_match_committed_mac_snapshots():
    for unit, entry in VISCOUS.teleop_adapter_units().items():
        rec = json.loads(
            (
                ROOT
                / f"configs/viscous/state-snapshots/2026-10-27/mac/.makermods/makermodslab-viscous/robots/{unit}.json"
            ).read_text()
        )
        assert entry["follower_usb_serial"] == rec["port_identities"]["follower_port"]["serial"]


def test_saved_serial_selects_among_models_and_unrelated_ports_are_ignored():
    live = [
        port("other-can", WET),
        port("desired"),
        port("star", "", connection.STAR_MODEL),
        port("unrelated", "x", (123, 456)),
    ]
    rec = record()
    rec["follower_port"] = "old"
    resolved = resolve(rec, live, identities={"follower_port": {"serial": DRY}})
    assert resolved["follower_port"] == "desired"
    assert resolved["leader_port"] == "star"
    assert (
        connection.resolve_start_ports(
            rec, MAKER, identities={"follower_port": {"serial": WET}}, enumerate_ports=lambda: live
        )["follower_port"]
        == "other-can"
    )
    assert (
        connection.resolve_start_ports(
            rec, SO101, enumerate_ports=lambda: pytest.fail("SO-101 must not enumerate")
        )
        is rec
    )
    assert (
        connection.resolve_start_ports(
            rec, METAL, enumerate_ports=lambda: pytest.fail("Damiao must not enumerate/retry")
        )
        is rec
    )


class Clock:
    def __init__(self):
        self.value = 0.0

    def now(self):
        return self.value

    def sleep(self, seconds):
        self.value += seconds


def wait_for(clock, cancelled=lambda: False):
    phases = []
    return connection.ConnectWait(cancelled, phases.append, clock=clock.now, sleep=clock.sleep), phases


def follower(monkeypatch, tmp_path, clock, ready=0, missing=(), fault=None):
    """Actual pinned Maker connect/RobStride wire stack; only transport is fake."""
    import can

    from lerobot.motors.robstride import robstride as driver
    from lerobot.robots.maker_follower import maker_follower as maker
    from lerobot.robots.viscous_follower import ViscousFollower, ViscousFollowerConfig

    monkeypatch.setattr(
        driver, "time", SimpleNamespace(time=clock.now, sleep=clock.sleep, perf_counter=clock.now)
    )
    monkeypatch.setattr(
        maker, "time", SimpleNamespace(time=clock.now, sleep=clock.sleep, perf_counter=clock.now)
    )
    frames, handles = [], []

    class FakeCAN:
        def __init__(self, **kwargs):
            self.ready = clock.now() >= ready
            self.queue = []
            self.closed = False
            handles.append(self)

        def send(self, msg):
            frames.append((clock.now(), msg.arbitration_id, bytes(msg.data)))
            if self.ready and msg.arbitration_id not in missing:
                payload = bytes([msg.arbitration_id, 0x80, 0, 0x80, 8, 0, 25, 25])
                if msg.arbitration_id == fault:
                    payload = bytes([msg.arbitration_id, 1, 0, 0, 0, 0, 0, 0])
                self.queue.append(
                    can.Message(arbitration_id=msg.arbitration_id, data=payload, is_extended_id=False)
                )

        def recv(self, timeout):
            if self.queue:
                return self.queue.pop(0)
            clock.sleep(timeout)
            return None

        def shutdown(self):
            self.closed = True

    monkeypatch.setattr(driver.can.interface, "Bus", FakeCAN)
    robot = ViscousFollower(ViscousFollowerConfig(port="fake-can", id="test", calibration_dir=tmp_path))
    return robot, frames, handles


@pytest.mark.parametrize("ready", [5, 60])
def test_follower_late_uses_same_attempt_until_ready(monkeypatch, tmp_path, ready):
    clock = Clock()
    robot, frames, handles = follower(monkeypatch, tmp_path, clock, ready=ready)
    wait, phases = wait_for(clock)
    wait.until_ready(lambda: connection.robstride_attempt(robot, wait), "follower")
    assert ready <= clock.now() < 65
    assert phases and set(phases) == {"waiting_follower"}
    assert all(payload == b"\xff" * 7 + b"\xfb" for when, _id, payload in frames if when < ready)
    assert any(payload == b"\xff" * 7 + b"\xfc" for _when, _id, payload in frames)
    assert all(h.closed for h in handles[:-1])
    robot.disconnect()


def test_follower_never_ready_has_no_enable_or_target(monkeypatch, tmp_path):
    clock = Clock()
    robot, frames, handles = follower(monkeypatch, tmp_path, clock, ready=1000)
    wait, _ = wait_for(clock)
    with pytest.raises(ApiError) as caught:
        wait.until_ready(lambda: connection.robstride_attempt(robot, wait), "follower")
    assert caught.value.code == "robot.follower_not_ready"
    assert 65 <= clock.now() < 65.2
    assert all(payload == b"\xff" * 7 + b"\xfb" for _when, _id, payload in frames)
    assert all(h.closed for h in handles)


@pytest.mark.parametrize(
    "missing,fault,code", [((7,), None, "robot.feedback_missing"), ((), 2, "robot.motor_fault")]
)
def test_partial_feedback_and_faults_are_not_retried(monkeypatch, tmp_path, missing, fault, code):
    clock = Clock()
    robot, frames, handles = follower(monkeypatch, tmp_path, clock, missing=missing, fault=fault)
    wait, _ = wait_for(clock)
    with pytest.raises(ApiError) as caught:
        wait.until_ready(lambda: connection.robstride_attempt(robot, wait), "follower")
    assert caught.value.code == code
    assert len(handles) == 1 and handles[0].closed
    assert all(payload == b"\xff" * 7 + b"\xfb" for _when, _id, payload in frames)


def test_stop_during_wait_cancels_before_any_activation(monkeypatch, tmp_path):
    from makermodslab import teleoperate

    clock = Clock()
    robot, frames, handles = follower(monkeypatch, tmp_path, clock, ready=5)
    monkeypatch.setattr(teleoperate, "_starting", True)
    monkeypatch.setattr(teleoperate, "teleoperation_active", True)
    import threading

    cancel = threading.Event()
    monkeypatch.setattr(teleoperate, "_start_cancel", cancel)
    wait, _ = wait_for(clock, cancel.is_set)

    def sleep(seconds):
        clock.sleep(seconds)
        teleoperate.handle_stop_teleoperation()

    wait.sleep = sleep
    with pytest.raises(connection.ConnectionCancelledError):
        wait.until_ready(lambda: connection.robstride_attempt(robot, wait), "follower")
    assert teleoperate.teleoperation_active  # Mutex remains held until worker cleanup.
    assert clock.now() < 5
    assert all(h.closed for h in handles)
    assert all(payload[-1] == 0xFB for _when, _id, payload in frames)


class FakeSerial:
    def __init__(self, clock, ready, missing=()):
        self.clock, self.ready, self.missing = clock, ready, missing
        self.queue = b""
        self.frames = []
        self.closed = False

    def reset_input_buffer(self):
        self.queue = b""

    def write(self, packet):
        from makermodslab.fashionstar_raw import PING, frame

        motor_id = packet[4]
        assert packet == frame(PING, bytes([motor_id]))
        self.frames.append(packet)
        if self.ready and motor_id not in self.missing:
            response = b"\x05\x1c\x01\x01" + bytes([motor_id])
            self.queue = response + bytes([sum(response) % 256])
        return len(packet)

    def read(self, size):
        result, self.queue = self.queue[:size], self.queue[size:]
        if not result:
            self.clock.sleep(0.02)
        return result

    def close(self):
        self.closed = True


def test_star_late_uses_only_existing_ping_frames():
    clock = Clock()
    wait, phases = wait_for(clock)
    handles = []
    config = SimpleNamespace(port="fake-star", baudrate=1000000, joint_ids={f"j{i}": i for i in range(7)})

    def factory(*args, **kwargs):
        assert kwargs["exclusive"] and kwargs["timeout"] == 0.02
        handle = FakeSerial(clock, clock.now() >= 8)
        handles.append(handle)
        return handle

    wait.until_ready(lambda: connection.star_attempt(config, wait, factory=factory), "leader")
    assert 8 <= clock.now() < 10
    assert set(phases) == {"waiting_leader"}
    assert all(h.closed for h in handles)


def test_star_partial_is_not_retried():
    clock = Clock()
    wait, _ = wait_for(clock)
    config = SimpleNamespace(port="fake", baudrate=1000000, joint_ids={f"j{i}": i for i in range(7)})
    handle = FakeSerial(clock, True, missing=(6,))
    with pytest.raises(ApiError) as caught:
        wait.until_ready(
            lambda: connection.star_attempt(config, wait, factory=lambda *a, **k: handle), "leader"
        )
    assert caught.value.code == "robot.leader_not_ready"
    assert caught.value.details["partial"] is True
    assert handle.closed


@pytest.mark.parametrize(
    "err,code",
    [(OSError(errno.EBUSY, "busy"), "robot.port_busy"), (ValueError("opaque"), "robot.connection_failed")],
)
def test_port_failure_types_not_prose(err, code):
    wrapped = RuntimeError("the message can change")
    wrapped.__cause__ = err
    result = connection.port_error(wrapped, "port")
    assert result.code == code
    assert result.detail == connection.MESSAGES[code]
    assert result.details["error"] == "the message can change"


def test_stop_inside_handshake_unwraps_typed_cancellation(monkeypatch, tmp_path):
    clock = Clock()
    robot, frames, handles = follower(monkeypatch, tmp_path, clock, ready=1000)
    wait, _ = wait_for(clock, lambda: clock.now() >= 0.2)
    with pytest.raises(connection.ConnectionCancelledError):
        connection.robstride_attempt(robot, wait)
    assert all(h.closed for h in handles)
    assert all(payload[-1] == 0xFB for _when, _id, payload in frames)


def test_stop_after_handshake_cannot_enable(monkeypatch, tmp_path):
    clock = Clock()
    robot, frames, _handles = follower(monkeypatch, tmp_path, clock)
    wait, _ = wait_for(clock)
    # Connect has finished reading but has not enabled yet.
    robot._detect_full_turn_offsets = lambda: setattr(wait, "cancelled", lambda: True)
    with pytest.raises(connection.ConnectionCancelledError):
        connection.robstride_attempt(robot, wait)
    assert not any(payload[-1] == 0xFC for _when, _id, payload in frames)


def test_sessions_rebind_before_readiness_and_attach_lease_before_worker(monkeypatch):
    from makermodslab import sessions
    from makermodslab.schemas.sessions import SessionStartBody
    from makermodslab.utils import config as cfg

    clock = Clock()
    tracker = sessions.SessionTracker(clock=clock.now)
    monkeypatch.setattr(sessions, "tracker", tracker)
    monkeypatch.setattr(sessions, "_held_by", lambda: None)
    monkeypatch.setattr(sessions, "_ensure_watchdog", lambda: None)
    rec = cfg._empty_record("bench") | record() | {"arm_type": VISCOUS.id, "leader_kind": "star"}
    monkeypatch.setattr(sessions, "get_robot_record", lambda name: rec)
    live = [port("rebound-can"), port("rebound-star", "", connection.STAR_MODEL)]
    monkeypatch.setattr(usb_ports, "serial_ports", lambda: live)

    def readiness(resolved, **kwargs):
        assert resolved["follower_port"] == "rebound-can"
        assert resolved["leader_port"] == "rebound-star"
        return True

    monkeypatch.setattr(sessions, "is_robot_record_clean", readiness)
    captured = []

    def start(kind, request, manager):
        assert not request._start_gate.is_set()
        assert request._ports_resolved
        tracker.observe({"session": {"kind": kind, "active": True, "phase": "starting"}})
        captured.append(request)
        return {"success": True}

    monkeypatch.setattr(sessions, "_dispatch_start", start)
    result = sessions.handle_start_session(SessionStartBody(kind="teleoperation", robot="bench", owner="tab"))
    assert captured[0]._start_gate.is_set()
    assert result["session"]["phase"] == "starting"
    assert result["session"]["lease"]["owner"] == "tab"
    clock.sleep(20)
    renewed = sessions.handle_heartbeat_session(result["session"]["id"], "tab")
    assert renewed["session"]["lease"]["expires_in_s"] == 60
    tracker.observe({"session": {"kind": "teleoperation", "active": True, "phase": "waiting_follower"}})
    assert tracker.current()["phase"] == "waiting_follower"


def test_wait_phase_is_visible_in_typed_status(monkeypatch):
    from makermodslab import teleoperate
    from makermodslab.schemas.sessions import TeleoperationStatusResponse

    monkeypatch.setattr(teleoperate, "teleoperation_active", True)
    monkeypatch.setattr(teleoperate, "connection_phase", "waiting_follower")
    status = TeleoperationStatusResponse.model_validate(teleoperate.handle_teleoperation_status())
    assert status.status_sentence == "Waiting for the arm to power up..."
    assert status.phase == "waiting_follower"


def test_feature_ci_triggers_and_profile_script_filters():
    import yaml

    for name in ("tests", "quality", "frontend"):
        workflow = yaml.load((ROOT / f".github/workflows/{name}.yml").read_text(), Loader=yaml.BaseLoader)
        assert "feat/viscous-arm" in workflow["on"]["push"]["branches"]
        if name == "tests":
            for trigger in ("push", "pull_request"):
                assert {"configs/**", "scripts/**", "uv.lock"} <= set(workflow["on"][trigger]["paths"])


def test_lease_expiry_cancels_wait_through_the_existing_stop(monkeypatch):
    import threading

    from makermodslab import sessions, teleoperate

    clock = Clock()
    tracker = sessions.SessionTracker(clock=clock.now)
    monkeypatch.setattr(sessions, "tracker", tracker)
    tracker.observe({"session": {"kind": "teleoperation", "active": True, "phase": "waiting_leader"}})
    tracker.attach_lease("teleoperation", owner="tab", timeout_s=10)
    cancel = threading.Event()
    monkeypatch.setattr(teleoperate, "_start_cancel", cancel)
    monkeypatch.setattr(teleoperate, "_starting", True)
    monkeypatch.setattr(teleoperate, "teleoperation_active", True)
    clock.sleep(10)
    assert sessions.check_expiry() is not None
    assert cancel.is_set()
    assert teleoperate.teleoperation_active  # Cleanup, not Stop, releases ownership.
    assert sessions.check_expiry() is None


def test_damiao_partial_connect_runs_recovery_once_without_retry(monkeypatch):
    from makermodslab import gravity_feedforward, metal_gripper, teleoperate

    attempts, recoveries = [], []

    def connect(**kwargs):
        attempts.append(kwargs)
        raise ConnectionError("partial Damiao handshake")

    robot = SimpleNamespace(connect=connect)
    config = SimpleNamespace(port="fake")
    monkeypatch.setattr(teleoperate, "build_single_configs", lambda request: (config, config))
    monkeypatch.setattr(teleoperate, "make_robot_from_config", lambda config: robot)
    monkeypatch.setattr(teleoperate, "make_teleoperator_from_config", lambda config: object())
    monkeypatch.setattr(teleoperate, "de_energize_can_device", lambda *args: recoveries.append(args) or [])
    monkeypatch.setattr(teleoperate, "_cleanup_after_setup_failure", lambda *args: None)
    monkeypatch.setattr(gravity_feedforward, "install_from_env", lambda *args: None)
    monkeypatch.setattr(metal_gripper, "install_metal_gripper", lambda *args: None)
    request = teleoperate.TeleoperateRequest(
        arm_type=METAL.id,
        leader_kind="star",
        leader_port="fake-star",
        follower_port="fake-can",
        leader_config="L",
        follower_config="F",
    )
    with pytest.raises(RuntimeError, match="Could not connect"):
        teleoperate._connect_can(request)
    assert len(attempts) == 1 and len(recoveries) == 1
    assert not METAL.teleop_usb_discovery


def test_star_never_ready_stops_at_shared_deadline():
    clock = Clock()
    wait, _ = wait_for(clock)
    config = SimpleNamespace(port="fake", baudrate=1000000, joint_ids={f"j{i}": i for i in range(7)})
    handles = []

    def factory(*args, **kwargs):
        handle = FakeSerial(clock, False)
        handles.append(handle)
        return handle

    with pytest.raises(ApiError) as caught:
        wait.until_ready(lambda: connection.star_attempt(config, wait, factory=factory), "leader")
    assert caught.value.code == "robot.leader_not_ready"
    assert 65 <= clock.now() < 65.2
    assert all(h.closed for h in handles)


def test_missing_enrollment_still_warns_instead_of_silently_starting(monkeypatch):
    monkeypatch.setattr(VISCOUS, "teleop_adapter_units", lambda: {})
    with pytest.raises(ApiError) as caught:
        resolve()
    assert caught.value.code == "robot.identity_unverified"
    assert resolve(override=True)["follower_config"] == "viscous_01"
