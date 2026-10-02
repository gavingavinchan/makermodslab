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
"""Descriptor-only USB serial discovery and saved adapter rebinding.

Never open a port here. A USB serial identifies a unit across sockets. For
adapters without a serial, model matching is allowed only when both the rig
and the live enumeration contain exactly one adapter of that model.
"""

import logging

logger = logging.getLogger(__name__)
PORT_FIELDS = ("leader_port", "follower_port", "right_leader_port", "right_follower_port")


def serial_ports():
    from serial.tools import list_ports

    try:
        return [p for p in list_ports.comports() if p.vid is not None and p.pid is not None]
    except OSError:
        logger.warning("Could not enumerate USB serial adapters", exc_info=True)
        return None


def same_port(left: str, right: str) -> bool:
    # pyserial enumerates macOS's callout node; old records used the tty alias.
    return left.replace("/dev/tty.", "/dev/cu.") == right.replace("/dev/tty.", "/dev/cu.")


def identity(port) -> dict:
    return {"vid": port.vid, "pid": port.pid, "serial": port.serial_number or ""}


def capture_identities(record: dict, previous: dict, changed: set[str], ports: list) -> dict:
    result = dict(previous)
    for field in PORT_FIELDS:
        if field in changed:
            result.pop(field, None)
        # Preserve the identity of an unplugged adapter when unrelated settings
        # are saved. Only an explicitly assigned port can replace that identity.
        if field in result:
            continue
        port = record.get(field, "")
        matches = [p for p in ports if same_port(p.device, port)]
        if len(matches) == 1:
            result[field] = identity(matches[0])
    return result


def resolve_ports(record: dict, identities: dict, ports: list) -> dict:
    result = dict(record)
    for field in PORT_FIELDS:
        saved = identities.get(field)
        if not isinstance(saved, dict) or not record.get(field):
            continue
        if not isinstance(saved.get("vid"), int) or not isinstance(saved.get("pid"), int):
            continue
        model = (saved["vid"], saved["pid"])
        serial = saved.get("serial", "")
        candidates = [p for p in ports if (p.vid, p.pid) == model]
        if serial:
            candidates = [p for p in candidates if p.serial_number == serial]
        else:
            slots = [
                key
                for key, value in identities.items()
                if key in PORT_FIELDS
                and record.get(key)
                and isinstance(value, dict)
                and (value.get("vid"), value.get("pid")) == model
            ]
            if len(slots) != 1:
                candidates = []
        # Missing, duplicate serials and indistinguishable adapters all refuse:
        # never fall back to a saved path that now belongs to another unit.
        result[field] = candidates[0].device if len(candidates) == 1 else ""
    return result
