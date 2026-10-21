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

import importlib.util
import signal
import struct
from dataclasses import replace
from pathlib import Path

import pytest

from makermodslab.fashionstar_raw import (
    ANGLE,
    DAMPING,
    HOLD,
    IDS,
    MONITOR,
    PING,
    RELEASE,
    STOP,
    Guard,
    RawBus,
    SafetyError,
    Travel,
    angle_packet,
    decode_monitor,
    decode_response,
    frame,
    stop_packet,
)

# Derived from the pinned vendor SDK cited in docs/star-servo-protocol.md:
# Packet.pack/calc_checksum:24-58; <BhHH basic angle:489-491;
# <BHHHHBih monitor + NTC:653-658; <BBH stop:673-689.
# These are literal vectors, not round trips through our encoder.
MONITOR_RESPONSE = bytes.fromhex("05 1c 16 10 03 0c 30 1e 00 71 01 9b 07 00 e8 ff ff ff ff ff 9b")


@pytest.fixture(autouse=True)
def restore_signal_handlers():
    previous = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    yield
    for sig, handler in previous.items():
        signal.signal(sig, handler)


def test_vendor_packet_vectors():
    assert frame(PING, b"\x03") == bytes.fromhex("12 4c 01 01 03 63")
    assert frame(MONITOR, b"\x03") == bytes.fromhex("12 4c 16 01 03 78")
    assert angle_packet(3, -12.3, 1000, 200) == bytes.fromhex("12 4c 08 07 03 85 ff e8 03 c8 00 a7")
    assert stop_packet(3, DAMPING, 200) == bytes.fromhex("12 4c 18 04 03 12 c8 00 57")
    assert stop_packet(3, RELEASE) == bytes.fromhex("12 4c 18 04 03 10 00 00 8d")
    # 0.1 degree units, positive sign, u16 little endian duration and final power.
    assert angle_packet(3, 2.5, 513, 1000)[4:-1] == bytes.fromhex("03 19 00 01 02 e8 03")


def test_monitor_vendor_vector_and_ntc():
    result = decode_monitor(decode_response(MONITOR_RESPONSE, MONITOR, 3))
    assert (result.id, result.angle_deg, result.turns) == (3, -2.4, -1)
    assert (result.voltage_mv, result.current_ma, result.power_mw) == (12300, 30, 369)
    assert result.temperature_adc == 1947
    assert result.temperature_c == pytest.approx(27.576641, abs=0.001)
    assert decode_monitor(MONITOR_RESPONSE[4:-3]).turns is None
    with pytest.raises(ValueError, match="NTC"):
        decode_monitor(struct.pack("<BHHHHBih", 3, 12000, 30, 360, 0, 0, 0, 0))


@pytest.mark.parametrize("mutation", ["checksum", "id", "command", "header", "length"])
def test_response_validation(mutation):
    data = bytearray(MONITOR_RESPONSE)
    index = {"checksum": -1, "id": 4, "command": 2, "header": 0, "length": 3}[mutation]
    data[index] ^= 1
    if mutation != "checksum":
        data[-1] = sum(data[:-1]) % 256
    with pytest.raises(ValueError):
        decode_response(bytes(data), MONITOR, 3)


@pytest.mark.parametrize("power", [0, -1, 1001, 65536, True, 0.5])
def test_energizing_power_rejected(power):
    with pytest.raises(ValueError, match="power"):
        angle_packet(3, 1, 1000, power)
    for mode in (HOLD, DAMPING):
        with pytest.raises(ValueError, match="power"):
            stop_packet(3, mode, power)


def test_explicit_power_override_and_other_encoding_guards():
    angle_packet(3, 1, 1000, 1200, max_power_mw=1200)
    for angle in (float("nan"), float("inf"), 180.1, -180.1):
        with pytest.raises(ValueError):
            angle_packet(3, angle, 1000, 100)
    for duration in (0, -1, 65536, 1.0):
        with pytest.raises(ValueError):
            angle_packet(3, 0, duration, 100)
    for motor_id in (7, 255, -1, True):
        with pytest.raises(ValueError):
            stop_packet(motor_id, RELEASE)
    with pytest.raises(ValueError):
        frame(0x0D, b"")  # intentionally not implemented
    with pytest.raises(ValueError):
        stop_packet(3, 0)


def test_travel_and_deflection_guards():
    travel = Travel(-84.95298, 110.80823)
    travel.check(-79.9)
    travel.check(105.8)
    for target in (-80, 105.9, float("nan")):
        with pytest.raises(SafetyError):
            travel.check(target)
    with pytest.raises(SafetyError, match="single-turn"):
        Travel(-300, 300).check(190)
    guard = Guard(3, -2.4, 10)
    sample = decode_monitor(MONITOR_RESPONSE[4:-1])
    guard.check(replace(sample, angle_deg=17.6))
    for monitor in (
        replace(sample, angle_deg=17.7),
        replace(sample, angle_deg=-22.5),
        replace(sample, id=0, temperature_c=60),
    ):
        with pytest.raises(SafetyError):
            guard.check(monitor)
    guard.check(replace(sample, id=0, angle_deg=100))


class FakeSerial:
    """Fresh protocol replies, short reads, and injected faults; no hardware or sleeps."""

    def __init__(self, *args, **kwargs):
        self.writes, self.rx, self.closed = [], b"", False
        self.fail_reads = 0
        self.fail_stop_id = None
        self.angle_deg = -2.4
        self.is_open = False
        self.options = kwargs

    def open(self):
        self.is_open = True

    def reset_input_buffer(self):
        self.rx = b""

    def write(self, packet):
        self.writes.append(packet)
        command, motor_id = packet[2], packet[4]
        if command == STOP and motor_id == self.fail_stop_id:
            raise OSError("injected release write failure")
        if command == ANGLE:
            self.angle_deg = struct.unpack("<h", packet[5:7])[0] / 10
        if command in (PING, MONITOR):
            if self.fail_reads:
                self.fail_reads -= 1
                return len(packet)
            payload = (
                bytes((motor_id,))
                if command == PING
                else struct.pack(
                    "<BHHHHBih",
                    motor_id,
                    12300,
                    30,
                    369,
                    1947,
                    0,
                    round(self.angle_deg * 10) if motor_id == 3 else 0,
                    -1,
                )
            )
            body = b"\x05\x1c" + bytes((command, len(payload))) + payload
            self.rx = b"noise" + body + bytes((sum(body) % 256,))
        return len(packet)

    def read(self, _size):
        data, self.rx = self.rx[:1], self.rx[1:]
        return data

    def close(self):
        self.closed = True
        self.is_open = False


def releases(port):
    return [p for p in port.writes if p[2] == STOP and p[5] == RELEASE]


@pytest.mark.parametrize("reason", ["normal", "exception", "sigint", "sigterm", "atexit"])
def test_every_powered_exit_releases_twice_each_and_closes(reason):
    port, events = FakeSerial(), []
    bus = RawBus(port, read_only=False, emit=lambda kind, **kw: events.append((kind, kw)))
    try:
        with bus:
            if reason == "exception":
                raise RuntimeError("injected")
            if reason in ("sigint", "sigterm"):
                check.interrupt(signal.SIGINT if reason == "sigint" else signal.SIGTERM, None)
            if reason == "atexit":
                bus.close()  # exact callback registered with atexit
    except (RuntimeError, KeyboardInterrupt):
        pass
    assert [packet[4] for packet in releases(port)] == list(IDS) * 2
    assert port.closed and bus.closed
    assert len(bus.released["readback"]) == 7
    assert all(r["looks_released"] for r in bus.released["readback"])
    assert any(e[0] == "release" for e in events)
    bus.close()
    assert len(releases(port)) == 14


def test_failed_release_write_continues_other_servos_and_second_pass():
    port = FakeSerial()
    port.fail_stop_id = 2
    with RawBus(port, read_only=False) as bus:
        pass
    assert len(releases(port)) == 14
    assert len(bus.released["write_errors"]) == 2
    assert port.closed


def test_read_retries_no_cached_fallback_and_cleanup():
    port = FakeSerial()
    with pytest.raises(SafetyError, match="3 consecutive"), RawBus(port, read_only=False) as bus:
        port.fail_reads = 3
        bus.monitor(3)
    assert len(releases(port)) == 14
    assert port.closed
    port = FakeSerial()
    port.fail_reads = 2
    with RawBus(port) as bus:
        assert bus.monitor(3).current_ma == 30
    assert len(port.writes) == 3


@pytest.mark.parametrize("reason", ["normal", "exception", "sigterm", "read_failure"])
def test_read_only_has_no_state_writes_even_on_exit(reason):
    port = FakeSerial()
    try:
        with RawBus(port) as bus:
            bus.request(PING, 3)
            bus.monitor(3)
            bus.guard = Guard(3, 0, 10)
            for action in (
                lambda: bus.angle(3, 0, 1000, 100, Travel(-80, 110)),
                lambda: bus.stop(3, DAMPING, 100),
                lambda: bus.release_all(),
            ):
                with pytest.raises(SafetyError, match="Read-only"):
                    action()
            if reason == "exception":
                raise RuntimeError("injected")
            if reason == "sigterm":
                check.interrupt(signal.SIGTERM, None)
            if reason == "read_failure":
                port.fail_reads = 3
                bus.monitor(3)
    except (RuntimeError, KeyboardInterrupt):
        pass
    assert all(p[2] in (PING, MONITOR) for p in port.writes)
    assert port.closed


def test_bus_refuses_other_joint_and_out_of_envelope_commands():
    port = FakeSerial()
    with RawBus(port, read_only=False) as bus:
        bus.guard = Guard(3, 0, 10)
        with pytest.raises(SafetyError, match="chosen"):
            bus.angle(2, 0, 1000, 100, Travel(-80, 110))
        with pytest.raises(SafetyError, match="chosen"):
            bus.stop(2, HOLD, 100)
        with pytest.raises(SafetyError, match="envelope"):
            bus.angle(3, 21, 1000, 100, Travel(-80, 110))
        assert port.writes == []


spec = importlib.util.spec_from_file_location(
    "star_actuation_check", Path(__file__).parents[1] / "scripts/star-actuation-check.py"
)
check = importlib.util.module_from_spec(spec)
spec.loader.exec_module(check)


def test_default_plan_and_unsafe_inputs():
    args = check.parser().parse_args([])
    powers, args.maximum = check.validate(args)
    config, travel = check.configuration(args)
    assert config.joint_ids["wrist_flex"] == 3 and config.baudrate == 1000000
    assert (travel.low, travel.high) == pytest.approx((-84.95298037, 110.80823527))
    plan = check.phase_plan(args, 3, -2.4, travel, powers)
    assert len(plan) == 13
    assert plan[4]["targets"] == [7.6, -2.4, -12.4, -2.4]
    assert plan[-1]["targets"] == [0.1, -2.4]
    assert plan[-1]["power_mw"] == 400
    for phase in plan:
        for packet in phase["commands"]:
            raw = bytes.fromhex(packet)
            assert sum(raw[:-1]) % 256 == raw[-1]
            assert struct.unpack("<H", raw[-3:-1])[0] == phase["power_mw"]
    with pytest.raises(SafetyError):
        check.phase_plan(args, 3, 105, travel, powers)
    for options in (
        ["--powers", "0"],
        ["--powers", "1001"],
        ["--move-deg", "21"],
        ["--move-s", "nan"],
        ["--move-s", "0.00001"],
        ["--phase-s", "0"],
    ):
        with pytest.raises(ValueError):
            check.validate(check.parser().parse_args(options))


@pytest.mark.parametrize("dry_run", [True, False])
def test_cli_with_fake_serial_complete_plan_and_evidence(tmp_path, monkeypatch, dry_run):
    import serial

    port = FakeSerial()
    monkeypatch.setattr(serial, "Serial", lambda **_kwargs: port)
    monkeypatch.setattr(check, "REPO", tmp_path)
    monkeypatch.setattr(check.sys.stdin, "isatty", lambda: False)
    now = [0.0]

    def clock():
        return now[0]

    def sleep(seconds):
        now[0] += max(seconds, 0.001)

    real_window = check.sample_window
    monkeypatch.setattr(check, "sample_window", lambda *a: real_window(*a, clock=clock, sleep=sleep))
    arguments = (
        ["--dry-run"] if dry_run else ["--yes", "--powers", "100", "--phase-s", "0.04", "--move-s", "0.04"]
    )
    assert check.main(arguments) == 0
    assert port.closed
    paths = list((tmp_path / check.EVIDENCE).glob("*.json"))
    assert len(paths) == 1
    import json

    data = json.loads(paths[0].read_text())
    assert len(data["preflight"]) == 7
    assert not Path(data["path"]).is_absolute()
    assert not list(paths[0].parent.glob("*.tmp"))
    if dry_run:
        assert len(port.writes) == 14
        assert {p[2] for p in port.writes} == {PING, MONITOR}
    else:
        assert data["outcome"] == "completed"
        assert data["samples"]
        assert data["summary"]["100"]["peak_current_ma"] == 30
        assert data["summary"]["100"]["hold_deflection_vs_time"]
        assert all(row["command_power_mw"] == 100 and row["power_mw"] == 369 for row in data["samples"])
        assert len(releases(port)) == 14 * 6  # startup, four phase boundaries, final cleanup
        assert all(e["id"] == 3 for e in data["events"] if e["kind"] == "tx" and e["command"] == ANGLE)


def test_stop_test_no_serial(monkeypatch, capsys):
    def signal_during_sleep(_seconds):
        check.interrupt(signal.SIGTERM, None)

    monkeypatch.setattr(check.time, "sleep", signal_during_sleep)
    monkeypatch.setattr(check, "configuration", lambda _: pytest.fail("must not load hardware config"))
    assert check.main(["--stop-test"]) == 0
    assert "Stop test passed: received SIGTERM; no hardware opened" in capsys.readouterr().out


@pytest.mark.parametrize("fault", ["temperature", "deflection", "open"])
def test_guard_or_open_fault_releases_all(fault):
    port = FakeSerial()
    with pytest.raises((SafetyError, OSError)), RawBus(port, read_only=False) as bus:
        if fault == "open":
            raise OSError("injected port-open failure")
        bus.guard = Guard(3, 0, 10)
        if fault == "deflection":
            port.angle_deg = 21
            bus.monitor(3)
        else:
            original = bus.request
            bus.request = lambda command, motor_id: replace(original(command, motor_id), temperature_c=60)
            bus.monitor(0)  # thermal trip on a different servo
    assert len(releases(port)) == 14
    assert port.closed


def test_evidence_fault_still_sends_all_emergency_releases():
    port = FakeSerial()

    def broken_evidence(*args, **kwargs):
        raise OSError("injected evidence disk failure")

    with pytest.raises(OSError), RawBus(port, read_only=False, emit=broken_evidence) as bus:
        bus.guard = Guard(3, 0, 10)
        bus.angle(3, 0, 1000, 100, Travel(-80, 110))
    assert len(releases(port)) == 14
    assert port.closed
