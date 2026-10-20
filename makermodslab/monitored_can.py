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

"""Local CAN sharing: one serial owner keeps checking faults during slow teleop.

The experiment monitor runs MonitorCanBridge in its existing polling loop.
Lab uses MonitorCanClient in place of opening the physical adapter. The
socket is local and private; no motor zero/parameter writes are accepted.
"""

from __future__ import annotations

import json
import os
import select
import socket
import time
from collections import deque
from pathlib import Path

import can


def bridge_metadata(port, profile):
    path = os.environ.get("MAKERMODSLAB_CAN_BRIDGE")
    if not path:
        return None
    metadata = json.loads(Path(path).read_text())
    if metadata["port"] != port:
        return None
    if metadata["profile"] != profile:
        raise ValueError("The monitored CAN adapter belongs to a different commissioned arm")
    os.kill(metadata["pid"], 0)
    return metadata


def supervised_speed_limit(port, profile, requested):
    """Older browser tabs must also use the monitored arm's slow connection."""
    if bridge_metadata(port, profile) is None:
        return requested
    if requested is None:
        return 15.0
    if not 0 < requested <= 15.0:
        raise ValueError("The monitored arm requires a target speed limit no greater than 15 deg/s")
    return requested


class MonitorCanBridge:
    def __init__(self, path, metadata_path, *, port, profile, send, healthy, record):
        self.path = Path(path)
        self.metadata_path = Path(metadata_path)
        self.port, self.profile = port, profile
        self.send, self.healthy, self.record = send, healthy, record
        self.client = None
        self.authenticated = False
        self.enabled = set()
        self.alarm = ""
        self.input = bytearray()
        self.output = bytearray()
        self.path.unlink(missing_ok=True)
        self.listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.listener.bind(str(self.path))
        os.chmod(self.path, 0o600)
        self.listener.listen(1)
        self.listener.setblocking(False)
        metadata = {
            "socket": str(self.path),
            "port": port,
            "profile": profile,
            "pid": os.getpid(),
            "protocol": 1,
        }
        self.metadata_path.write_text(json.dumps(metadata) + "\n")
        os.chmod(self.metadata_path, 0o600)
        self.record("teleop_bridge_ready", **metadata)

    @property
    def connected(self):
        return self.client is not None

    def _queue(self, row):
        self.output.extend(json.dumps(row).encode() + b"\n")
        if len(self.output) > 262144:
            raise ConnectionError("Teleop stopped consuming CAN feedback")

    def _drop(self, reason):
        if self.client:
            self.client.close()
        self.client = None
        self.authenticated = False
        self.input.clear()
        self.output.clear()
        if self.enabled:
            self.alarm = "Teleoperation control connection lost. Motors may still hold position."
        self.record("teleop_bridge_disconnected", reason=reason, possibly_enabled_ids=sorted(self.enabled))

    def _command(self, command):
        op = command.get("op")
        if op == "hello" and not self.authenticated:
            speed = command.get("speed_limit_deg_s")
            if (
                command.get("port") != self.port
                or command.get("profile") != self.profile
                or not isinstance(speed, (int, float))
                or not 0 < speed <= 15.0
                or not self.healthy()
                or self.enabled
            ):
                raise ValueError(
                    "Bridge requires the commissioned healthy arm and a cap no greater than 15 deg/s"
                )
            self.authenticated = True
            self.alarm = ""
            self._queue({"op": "hello", "protocol": 1})
            self.record("teleop_bridge_claimed", speed_limit_deg_s=speed, profile=self.profile)
            return
        if not self.authenticated:
            raise ValueError("Teleop bridge handshake required")
        if op == "shutdown":
            self._drop("Lab client shut down")
            return
        if op != "send":
            raise ValueError("Unknown bridge command")
        motor = command.get("can_id")
        data = bytes.fromhex(command.get("data", ""))
        if command.get("extended") or motor not in range(1, 8) or len(data) != 8:
            raise ValueError("Only eight-byte standard frames to IDs 1-7 are allowed")
        if data == bytes.fromhex("FFFFFFFFFFFFFFFE"):
            raise ValueError("Motor zeroing is forbidden on the teleop bridge")
        disable = data == bytes.fromhex("FFFFFFFFFFFFFFFD")
        query = data == bytes.fromhex("FFFFFFFFFFFFFFFB")
        if not disable and not query and not self.healthy():
            self._queue({"op": "error", "message": "Joint status not fresh and zero-fault; command refused"})
            # Keep the socket available for the client's cleanup disables.
            return
        self.send(motor, data)
        if data == bytes.fromhex("FFFFFFFFFFFFFFFC"):
            self.enabled.add(motor)
        elif disable:
            self.enabled.discard(motor)
        self._queue({"op": "sent"})

    def pump(self):
        if self.client is None:
            try:
                self.client, _ = self.listener.accept()
                self.client.setblocking(False)
            except BlockingIOError:
                return
        try:
            try:
                chunk = self.client.recv(65536)
            except BlockingIOError:
                chunk = None
            if chunk == b"":
                self._drop("Socket closed")
                return
            if chunk:
                self.input.extend(chunk)
                if len(self.input) > 262144:
                    raise ValueError("Teleop command buffer overflow")
            for _ in range(64):
                if b"\n" not in self.input:
                    break
                end = self.input.index(b"\n")
                line = bytes(self.input[:end])
                del self.input[: end + 1]
                self._command(json.loads(line))
                if self.client is None:
                    return
            if self.output:
                try:
                    sent = self.client.send(self.output)
                    del self.output[:sent]
                except BlockingIOError:
                    pass
        except (OSError, ValueError, TypeError, KeyError) as exc:
            self.record("teleop_bridge_error", error=repr(exc))
            self._drop(str(exc))

    def receive(self, can_id, extended, data):
        # Five-byte non-clearing fault replies belong to the monitor. The
        # pinned SDK expects eight-byte encoder feedback and must not consume
        # the monitor's status reply as if it were a position sample.
        if self.authenticated and len(data) == 8:
            try:
                self._queue({"op": "frame", "can_id": can_id, "extended": extended, "data": data.hex()})
            except ConnectionError as exc:
                self._drop(str(exc))

    def close(self):
        if self.client:
            self._drop("Monitor closing")
        self.listener.close()
        self.path.unlink(missing_ok=True)
        self.metadata_path.unlink(missing_ok=True)


class MonitorCanClient:
    def __init__(self, metadata, speed_deg_s):
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.socket.settimeout(2)
        self.buffer = bytearray()
        self.pending = deque()
        self.channel_info = "Supervised local CAN bridge"
        try:
            self.socket.connect(metadata["socket"])
            self._send(
                {
                    "op": "hello",
                    "port": metadata["port"],
                    "profile": metadata["profile"],
                    "speed_limit_deg_s": speed_deg_s,
                }
            )
            hello = self._read(2)
            if hello != {"op": "hello", "protocol": 1}:
                raise ConnectionError("Monitor refused teleop bridge claim")
        except BaseException:
            self.socket.close()
            raise

    @classmethod
    def discover(cls, port, profile, speed_deg_s):
        metadata = bridge_metadata(port, profile)
        if metadata is None:
            return None
        return cls(metadata, speed_deg_s)

    def _send(self, row):
        self.socket.sendall(json.dumps(row).encode() + b"\n")

    def _read(self, timeout):
        deadline = time.monotonic() + max(0, timeout)
        while True:
            if b"\n" in self.buffer:
                end = self.buffer.index(b"\n")
                line = bytes(self.buffer[:end])
                del self.buffer[: end + 1]
                return json.loads(line)
            remaining = max(0, deadline - time.monotonic())
            if not select.select([self.socket], [], [], remaining)[0]:
                return None
            chunk = self.socket.recv(65536)
            if not chunk:
                raise can.CanOperationError("Connection monitor bridge closed")
            self.buffer.extend(chunk)

    def send(self, message, timeout=None):
        self._send(
            {
                "op": "send",
                "can_id": message.arbitration_id,
                "extended": message.is_extended_id,
                "data": bytes(message.data).hex(),
            }
        )
        deadline = time.monotonic() + (1 if timeout is None else max(timeout, 0.05))
        while time.monotonic() < deadline:
            row = self._read(max(0, deadline - time.monotonic()))
            if row is None:
                break
            if row.get("op") == "sent":
                return
            if row.get("op") == "error":
                raise can.CanOperationError(row["message"])
            if row.get("op") == "frame":
                if len(self.pending) >= 4096:
                    raise can.CanOperationError("Teleop feedback queue overflow")
                self.pending.append(row)
            else:
                raise can.CanOperationError("Invalid bridge acknowledgement")
        raise can.CanOperationError("Timed out waiting for monitored CAN transmission")

    def recv(self, timeout=None):
        row = self.pending.popleft() if self.pending else self._read(0.05 if timeout is None else timeout)
        if row is None:
            return None
        if row.get("op") != "frame":
            raise can.CanOperationError(row.get("message", "Unexpected monitor bridge response"))
        return can.Message(
            arbitration_id=row["can_id"],
            is_extended_id=row["extended"],
            data=bytes.fromhex(row["data"]),
            timestamp=time.time(),
        )

    def shutdown(self):
        try:
            self._send({"op": "shutdown"})
        finally:
            self.socket.close()
