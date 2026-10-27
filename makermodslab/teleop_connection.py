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
"""Local RobStride Teleop discovery and cancellable connection policy.

Only descriptors discover endpoints. Readiness reuses the fork's existing
0xFB query and Star ping, never a new protocol/identity command. Adapter
identity is deliberately a replaceable seam, not proof of motor identity.
"""

from __future__ import annotations

import errno
import time
from contextlib import ExitStack, contextmanager

from . import usb_ports
from .api_errors import ApiError, ErrorCode

MESSAGES = {
    ErrorCode.ROBOT_FOLLOWER_USB_MISSING: "Connect the large arm’s USB cable.",
    ErrorCode.ROBOT_LEADER_USB_MISSING: "Connect the small arm’s USB cable.",
    ErrorCode.ROBOT_FOLLOWER_NOT_READY: "Turn on the large arm’s power switch.",
    ErrorCode.ROBOT_LEADER_NOT_READY: "Connect the small arm’s power supply.",
    ErrorCode.ROBOT_FEEDBACK_MISSING: "Reconnect the large arm’s power cable.",
    ErrorCode.ROBOT_CONNECTION_AMBIGUOUS: "Unplug the extra arm connection.",
    ErrorCode.ROBOT_UNIT_MISMATCH: "The connected adapter belongs to another arm; review the settings before continuing.",
    ErrorCode.ROBOT_IDENTITY_UNVERIFIED: "The connected adapter is not enrolled to this arm; review the settings before continuing.",
    ErrorCode.ROBOT_PORT_BUSY: "Close the other arm session and press Teleop again.",
    ErrorCode.ROBOT_CONNECTION_FAILED: "Reconnect the arm’s USB and power cables, then press Teleop.",
    ErrorCode.ROBOT_MOTOR_FAULT: "Turn off the large arm’s power switch.",
    ErrorCode.ROBOT_RELEASE_UNCERTAIN: "Turn off the large arm’s power switch.",
}
CANABLE_MODEL = (5840, 4478)
STAR_MODEL = (6790, 29987)


def failure(code, **details):
    return ApiError(409, MESSAGES[code], code=code, details=details)


def resolve_start_ports(record, family, *, override=False, identities=None, enumerate_ports=None):
    """Resolve a single descriptor snapshot, refusing stale-path fallback.

    An enrolled/saved serial selects among multiple CANables; a sole different
    serial is an identity warning. Never choose among indistinguishable Stars.
    No port opens, symlink reads or record writes occur here.
    """
    if not family.teleop_usb_discovery:
        return record
    try:
        ports = (enumerate_ports or usb_ports.serial_ports)()
        if ports is None:
            raise failure(ErrorCode.ROBOT_CONNECTION_FAILED, reason="usb_enumeration_failed")
        ports = usb_ports.unique_ports(ports)
        enrollments = family.teleop_adapter_units()
    except ApiError:
        raise
    except Exception as exc:
        raise failure(ErrorCode.ROBOT_CONNECTION_FAILED, reason="discovery_failed", error=str(exc)) from exc
    result = dict(record)
    identities = identities or {}
    slots = [""] if record.get("mode", "single") == "single" else ["", "right_"]
    used = set()
    for prefix in slots:
        field = prefix + "follower_port"
        config = record.get(prefix + "follower_config", "").removesuffix(".json")
        expected = next(
            (unit for unit, entry in enrollments.items() if config in entry["follower_calibrations"]), None
        )
        saved = identities.get(field, {})
        serial = saved.get("serial")
        # Old Linux records have only a by-id locator. Only committed serials
        # are recognized inside it; the path itself is never opened or trusted.
        if not serial:
            serial = next(
                (
                    entry["follower_usb_serial"]
                    for entry in enrollments.values()
                    if entry["follower_usb_serial"] in record.get(field, "")
                ),
                None,
            )
        if not serial and expected:
            serial = enrollments[expected]["follower_usb_serial"]
        candidates = [p for p in ports if (p.vid, p.pid) == CANABLE_MODEL and p.device not in used]
        matches = [p for p in candidates if serial and p.serial_number == serial]
        if matches:
            candidates = matches
        if not candidates:
            raise failure(ErrorCode.ROBOT_FOLLOWER_USB_MISSING, side=field)
        if len(candidates) != 1:
            raise failure(
                ErrorCode.ROBOT_CONNECTION_AMBIGUOUS,
                side=field,
                candidates=[usb_ports.identity(p) | {"port": p.device} for p in candidates],
            )
        found = candidates[0]
        observed = next(
            (
                unit
                for unit, entry in enrollments.items()
                if entry["follower_usb_serial"] == found.serial_number
            ),
            None,
        )
        # Maker has no per-unit enrollment yet; saved serials still rebind it.
        if (
            (enrollments or family.teleop_requires_adapter_enrollment)
            and (expected is None or observed != expected)
            and not override
        ):
            code = (
                ErrorCode.ROBOT_UNIT_MISMATCH
                if observed and expected
                else ErrorCode.ROBOT_IDENTITY_UNVERIFIED
            )
            warning = failure(
                code,
                expected_unit=expected,
                observed_unit=observed,
                adapter=usb_ports.identity(found),
                port=found.device,
                settings_unit=expected,
                follower_config=config,
                identity_kind="usb_adapter_only",
                override_option="allow_unit_override",
            )
            warning.detail = (
                f"Adapter {found.serial_number or 'without a serial'} belongs to "
                f"{observed or 'an unenrolled unit'}; continuing uses {expected or config} settings."
            )
            raise warning
        result[field] = found.device
        used.add(found.device)
        leader_field = prefix + "leader_port"
        candidates = [p for p in ports if (p.vid, p.pid) == STAR_MODEL and p.device not in used]
        if not candidates:
            raise failure(ErrorCode.ROBOT_LEADER_USB_MISSING, side=leader_field)
        if len(candidates) != 1 or len(slots) > 1:
            raise failure(
                ErrorCode.ROBOT_CONNECTION_AMBIGUOUS,
                side=leader_field,
                candidates=[p.device for p in candidates],
            )
        result[leader_field] = candidates[0].device
        used.add(candidates[0].device)
    return result


class ConnectionCancelledError(Exception):
    """Stop/lease expiry cancelled setup; cleanup remains with its worker."""


class ConnectWait:
    """One shared deadline with an injected monotonic clock and cancellable sleep."""

    def __init__(self, cancelled, phase, *, clock=time.monotonic, sleep=time.sleep, timeout=65.0):
        self.cancelled = cancelled
        self.phase = phase
        self.clock = clock
        self.sleep = sleep
        self.deadline = clock() + timeout

    def check(self):
        if self.cancelled():
            raise ConnectionCancelledError()

    def until_ready(self, attempt, side):
        code = ErrorCode.ROBOT_FOLLOWER_NOT_READY if side == "follower" else ErrorCode.ROBOT_LEADER_NOT_READY
        while True:
            self.check()
            if self.clock() >= self.deadline:
                raise failure(code, deadline_s=65, side=side)
            self.phase("waiting_" + side)
            if attempt():
                self.check()
                return
            until = min(self.clock() + 1.0, self.deadline)
            while self.clock() < until:
                self.check()
                self.sleep(min(0.25, until - self.clock()))


def port_error(exc, port):
    """Use OS error types/errno, never driver prose, across wrapped causes."""
    cause = exc
    while cause is not None:
        if isinstance(cause, OSError) and cause.errno in {errno.EBUSY, errno.EACCES, errno.EAGAIN}:
            return failure(ErrorCode.ROBOT_PORT_BUSY, port=port, error=str(exc))
        cause = cause.__cause__ or cause.__context__
    return failure(ErrorCode.ROBOT_CONNECTION_FAILED, port=port, error=str(exc))


@contextmanager
def _observe_method(bus, name, replacement):
    original = getattr(bus, name)
    setattr(bus, name, replacement)
    try:
        yield
    finally:
        setattr(bus, name, original)


def robstride_attempt(device, wait):
    """Observe the existing handshake at the query source, without prose parsing.

    The fork wraps handshake exceptions in ConnectionError; the local ledger
    survives that wrapper. Only an all-silent handshake can be retried. Any
    reply, fault, post-handshake exception, or failed close is terminal.
    """
    arms = [device.left_arm, device.right_arm] if hasattr(device, "left_arm") else [device]
    results = []
    try:
        with ExitStack() as stack:
            for arm in arms:
                query = arm.bus._query_status_via_clear_fault

                def observed(motor, timeout=None, *, _query=query):
                    wait.check()
                    remaining = wait.deadline - wait.clock()
                    if remaining <= 0:
                        raise failure(ErrorCode.ROBOT_FOLLOWER_NOT_READY, reason="deadline")
                    try:
                        fault, msg = _query(motor, timeout=min(timeout or 0.1, 0.1, remaining))
                    except (ValueError, IndexError) as exc:
                        raise failure(ErrorCode.ROBOT_FEEDBACK_MISSING, motor=motor, error=str(exc)) from exc
                    results.append((motor, fault, msg is not None))
                    return fault, msg

                stack.enter_context(_observe_method(arm.bus, "_query_status_via_clear_fault", observed))
                enable = arm.bus._enable_motor

                def guarded_enable(motor, *, _enable=enable):
                    wait.check()
                    if wait.clock() >= wait.deadline:
                        raise failure(ErrorCode.ROBOT_FOLLOWER_NOT_READY, reason="deadline_before_enable")
                    return _enable(motor)

                stack.enter_context(_observe_method(arm.bus, "_enable_motor", guarded_enable))
            device.connect(calibrate=False)
        return True
    except Exception as exc:
        missing = [motor for motor, _fault, answered in results if not answered]
        faults = [motor for motor, fault, _answered in results if fault]
        expected_count = sum(len(arm.bus.motors) for arm in arms)
        all_silent = len(results) == expected_count and len(missing) == expected_count and not faults
        # A failed bus handshake never reached enable; close its leaked
        # transport even on partial feedback/cancellation. Other failures still
        # use the caller's existing de-energize + disconnect recovery.
        for arm in arms:
            if not arm.bus.is_connected:
                try:
                    if arm.bus.canbus is not None:
                        arm.bus.canbus.shutdown()
                        arm.bus.canbus = None
                except Exception as close_error:
                    raise failure(ErrorCode.ROBOT_RELEASE_UNCERTAIN, error=str(close_error)) from exc
        source = exc
        while source is not None:
            if isinstance(source, (ApiError, ConnectionCancelledError)):
                raise source from None
            source = source.__cause__ or source.__context__
        if all_silent:
            wait.check()
            return False
        if faults:
            raise failure(ErrorCode.ROBOT_MOTOR_FAULT, faults=faults, missing=missing) from exc
        if missing:
            raise failure(ErrorCode.ROBOT_FEEDBACK_MISSING, missing=missing) from exc
        raise port_error(exc, device.config.port) from exc


def star_attempt(config, wait, *, factory=None):
    """Bounded existing Star pings, before connect can unlock/reset servos.

    RawBus enforces ping/monitor-only writes; use its existing packet checks
    with explicit pyserial timeouts instead of an opaque native ping budget.
    The handle is closed directly (RawBus's bench signal handler is main-thread
    only and is unnecessary for these non-energizing reads).
    """
    from .fashionstar_raw import PING, RawBus, SafetyError

    if factory is None:
        from serial import Serial

        factory = Serial
    transport = None
    try:
        transport = factory(
            config.port, baudrate=config.baudrate, timeout=0.02, write_timeout=0.1, exclusive=True
        )
        bus = RawBus(transport, read_only=True, clock=wait.clock)
        answered = []
        missing = []
        for name, motor_id in config.joint_ids.items():
            wait.check()
            if wait.clock() >= wait.deadline:
                raise failure(ErrorCode.ROBOT_LEADER_NOT_READY, reason="deadline")
            try:
                bus.request(PING, motor_id)
                answered.append(name)
            except SafetyError as exc:
                if not isinstance(exc.__cause__, TimeoutError):
                    raise failure(ErrorCode.ROBOT_LEADER_NOT_READY, motor=name, error=str(exc)) from exc
                missing.append(name)
        if missing and answered:
            raise failure(ErrorCode.ROBOT_LEADER_NOT_READY, missing=missing, answered=answered, partial=True)
        return not missing
    except (ApiError, ConnectionCancelledError):
        raise
    except Exception as exc:
        raise port_error(exc, config.port) from exc
    finally:
        if transport is not None:
            transport.close()
