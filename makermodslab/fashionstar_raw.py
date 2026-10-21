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
"""Minimal FashionStar bench transport; never imports or opens motorbridge.

Wire formats independently implemented from the manufacturer SDK pinned in
``docs/star-servo-protocol.md`` (uservo.py:24-58, 403-491, 653-689).
Only ping, monitor, basic single-turn angle+time+power and stop are supported.
Read-only means ping/monitor request bytes, NEVER a servo-state write, even on exit.
"""

from __future__ import annotations

import atexit
import math
import signal
import struct
import time
from dataclasses import asdict, dataclass

IDS = tuple(range(7))
PING, MONITOR, ANGLE, STOP = 0x01, 0x16, 0x08, 0x18
RELEASE, HOLD, DAMPING = 0x10, 0x11, 0x12


class SafetyError(RuntimeError):
    """A read, temperature, travel or deflection limit requires release."""


def servo_id(value):
    if type(value) is not int or value not in IDS:
        raise ValueError("Only individual servo IDs 0-6 are permitted")
    return value


def power_limit(power_mw, max_power_mw=1000):
    if (
        type(power_mw) is not int
        or type(max_power_mw) is not int
        or not 1 <= power_mw <= max_power_mw <= 65535
    ):
        raise ValueError("Energizing power must be 1..max_power_mw (0 selects the protection ceiling)")
    return power_mw


def frame(command, payload):
    if command not in (PING, MONITOR, ANGLE, STOP):
        raise ValueError("Unsupported command")
    body = b"\x12\x4c" + bytes((command, len(payload))) + payload
    return body + bytes((sum(body) % 256,))


def decode_response(data, command, motor_id):
    if (
        len(data) < 6
        or data[:2] != b"\x05\x1c"
        or data[2] != command
        or len(data) != data[3] + 5
        or sum(data[:-1]) % 256 != data[-1]
        or data[4] != motor_id
    ):
        raise ValueError("Invalid response header, command, length, checksum or servo ID")
    return data[4:-1]


def angle_packet(motor_id, angle_deg, duration_ms, power_mw, *, max_power_mw=1000):
    power_limit(power_mw, max_power_mw)
    if not math.isfinite(angle_deg) or not -180 <= angle_deg <= 180:
        raise ValueError("Single-turn angle must be finite and within [-180, 180]")
    if type(duration_ms) is not int or not 1 <= duration_ms <= 65535:
        raise ValueError("Duration must be 1..65535 ms")
    # Quantize to signed 0.1 degree units; never silently clamp.
    return frame(
        ANGLE, struct.pack("<BhHH", servo_id(motor_id), round(angle_deg * 10), duration_ms, power_mw)
    )


def stop_packet(motor_id, mode, power_mw=0, *, max_power_mw=1000):
    if mode not in (RELEASE, HOLD, DAMPING):
        raise ValueError("Unknown stop mode")
    if mode == RELEASE:
        if power_mw != 0:
            raise ValueError("Release uses power 0 only")
    else:
        power_limit(power_mw, max_power_mw)
    return frame(STOP, struct.pack("<BBH", servo_id(motor_id), mode, power_mw))


@dataclass(frozen=True)
class Monitor:
    id: int
    voltage_mv: int
    current_ma: int
    power_mw: int
    temperature_adc: int
    status: int
    angle_deg: float
    turns: int | None
    temperature_c: float


def decode_monitor(payload):
    # Older firmware lacks the final signed turn count; no cached telemetry is used.
    if len(payload) not in (14, 16):
        raise ValueError("Monitor must have 14 or 16 payload bytes")
    motor_id, voltage, current, power, adc, status, angle = struct.unpack("<BHHHHBi", payload[:14])
    servo_id(motor_id)
    if not 0 < adc < 4096:
        raise ValueError("Invalid NTC ADC")
    temp = 1 / (math.log(adc / (4096 - adc)) / 3435 + 1 / 298.15) - 273.15
    turns = struct.unpack("<h", payload[14:])[0] if len(payload) == 16 else None
    return Monitor(motor_id, voltage, current, power, adc, status, angle / 10, turns, temp)


@dataclass(frozen=True)
class Travel:
    low: float
    high: float
    margin: float = 5.0

    def check(self, target):
        if not math.isfinite(target) or not self.low + self.margin <= target <= self.high - self.margin:
            raise SafetyError(f"Target {target} outside Star travel with {self.margin} deg margin: {self}")
        if not -180 <= target <= 180:
            raise SafetyError("Target outside the basic single-turn frame; no wrapping is permitted")
        return target


@dataclass
class Guard:
    motor_id: int
    start: float
    move_deg: float

    def check(self, monitor, *, energized=True):
        if monitor.temperature_c >= 60:
            raise SafetyError(f"Servo {monitor.id}: temperature {monitor.temperature_c:.2f} C >= 60 C")
        if energized and monitor.id == self.motor_id:
            lo, hi = self.start - self.move_deg - 10, self.start + self.move_deg + 10
            if not math.isfinite(monitor.angle_deg) or not lo <= monitor.angle_deg <= hi:
                raise SafetyError(f"Servo {monitor.id}: angle {monitor.angle_deg} outside [{lo}, {hi}]")


class RawBus:
    """Own a pyserial handle. Read-only policy is enforced at the final write boundary.

    The caller must open with exclusive=True and finite read/write timeouts.
    Each fresh read has up to three attempts, never a cached fallback.
    Cleanup is best effort: two independent release passes, then readback, then close.
    """

    def __init__(self, port, *, read_only=True, max_power_mw=1000, emit=None, clock=time.monotonic):
        self.port, self.read_only, self.max_power_mw = port, read_only, max_power_mw
        self.emit = emit or (lambda *a, **kw: None)
        self.clock = clock
        self.closed = False
        self.guard = None
        self.released = None

    def __enter__(self):
        atexit.register(self.close)
        return self

    def __exit__(self, *_exc):
        self.close()

    def _emit(self, kind, **fields):
        self.emit(kind, **fields)

    def _send(self, packet):
        if self.read_only and (packet[2] not in (PING, MONITOR) or packet[3] != 1):
            raise SafetyError("Read-only transport refuses servo-state writes")
        count = self.port.write(packet)
        if count != len(packet):
            raise OSError("Short serial write")
        try:
            self._emit("tx", frame=packet.hex(" "), command=packet[2], id=packet[4])
        except Exception as exc:
            # An evidence failure aborts actuation. Only emergency release may continue.
            if packet[2] != STOP or packet[5] != RELEASE:
                raise
            print(f"Evidence write failed during release: {exc}", flush=True)

    def request(self, command, motor_id):
        if command not in (PING, MONITOR):
            raise ValueError("Only ping/monitor reads are supported")
        for attempt in range(1, 4):
            try:
                self.port.reset_input_buffer()
                self._send(frame(command, bytes((servo_id(motor_id),))))
                # Bounded resynchronization: ignore noise, reject malformed frames.
                deadline = self.clock() + 0.12
                previous = b""
                while self.clock() < deadline:
                    byte = self.port.read(1)
                    if not byte:
                        raise TimeoutError("No response header")
                    if previous + byte == b"\x05\x1c":
                        break
                    previous = byte
                else:
                    raise TimeoutError("Response header deadline exceeded")
                header = self._read_exact(2)
                if header[0] != command or header[1] not in ((1,) if command == PING else (14, 16)):
                    raise ValueError("Unexpected response command/length")
                data = b"\x05\x1c" + header + self._read_exact(header[1] + 1)
                payload = decode_response(data, command, motor_id)
                result = decode_monitor(payload) if command == MONITOR else payload
                if command == PING and payload != bytes((motor_id,)):
                    raise ValueError("Invalid ping payload")
                self._emit("rx", frame=data.hex(" "), command=command, id=motor_id)
                return result
            except (OSError, ValueError) as exc:
                self._emit("read_error", command=command, id=motor_id, attempt=attempt, error=str(exc))
                if attempt == 3:
                    raise SafetyError(f"Servo {motor_id}: 3 consecutive read failures: {exc}") from exc
        raise AssertionError("unreachable")

    def _read_exact(self, size):
        result = b""
        while len(result) < size:
            part = self.port.read(size - len(result))
            if not part:
                raise TimeoutError("Truncated response")
            result += part
        return result

    def monitor(self, motor_id):
        result = self.request(MONITOR, motor_id)
        self._emit("monitor", **asdict(result))
        if self.guard:
            self.guard.check(result)
        elif result.temperature_c >= 60:
            raise SafetyError(f"Servo {motor_id}: temperature >= 60 C")
        return result

    def angle(self, motor_id, target, duration_ms, power_mw, travel):
        if self.guard is None or motor_id != self.guard.motor_id:
            raise SafetyError("Only the guarded chosen servo may be energized")
        travel.check(target)
        lo = self.guard.start - self.guard.move_deg - 10
        hi = self.guard.start + self.guard.move_deg + 10
        if not lo <= target <= hi:
            raise SafetyError("Command outside start deflection envelope")
        self._send(angle_packet(motor_id, target, duration_ms, power_mw, max_power_mw=self.max_power_mw))

    def stop(self, motor_id, mode, power_mw=0):
        if mode != RELEASE and (self.guard is None or motor_id != self.guard.motor_id):
            raise SafetyError("Only the guarded chosen servo may be energized")
        self._send(stop_packet(motor_id, mode, power_mw, max_power_mw=self.max_power_mw))

    def release_all(self):
        if self.read_only:
            raise SafetyError("Read-only transport refuses release")
        errors = []
        for _ in range(2):
            for motor_id in IDS:
                try:
                    self.stop(motor_id, RELEASE)
                except Exception as exc:
                    errors.append({"id": motor_id, "error": str(exc)})
        reports = []
        for motor_id in IDS:
            try:
                # Bypass active deflection/thermal guard during emergency cleanup.
                monitor = self.request(MONITOR, motor_id)
                reports.append({**asdict(monitor), "looks_released": monitor.current_ma <= 45})
            except Exception as exc:
                reports.append({"id": motor_id, "looks_released": None, "error": str(exc)})
        self.released = {"write_errors": errors, "readback": reports}
        print(
            "RELEASE readback (<=45 mA near 30 mA rest; NOT proof of torque-off):", self.released, flush=True
        )
        self._emit("release", **self.released)
        return self.released

    def close(self):
        if self.closed:
            return
        # Ignore repeat stop signals while trying every release, even if a read/write fails.
        previous = {}
        try:
            for sig in (signal.SIGINT, signal.SIGTERM):
                previous[sig] = signal.signal(sig, signal.SIG_IGN)
            if not self.read_only:
                self.release_all()
        finally:
            try:
                self.port.close()
            finally:
                self.closed = True
                atexit.unregister(self.close)
                for sig, handler in previous.items():
                    signal.signal(sig, handler)
