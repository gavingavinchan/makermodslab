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
"""Bounded one-command socket client adapted from metal-arm-harness operator.send; no arm imports."""

import json
import socket
from pathlib import Path


def address(tcp):
    host, sep, port = tcp.partition(":")
    if host != "127.0.0.1" or not sep or not 0 <= int(port) <= 65535:
        raise ValueError("--tcp requires 127.0.0.1:PORT")
    return host, int(port)


def send(folder, words):
    endpoint = json.loads((Path(folder) / "connection.json").read_text())
    tcp = endpoint.get("tcp")
    with socket.socket(socket.AF_INET if tcp else socket.AF_UNIX) as sock:
        sock.connect(address(tcp) if tcp else endpoint["socket"])
        with sock.makefile("rwb") as stream:
            stream.write(json.dumps({"token": endpoint["token"], "command": words}).encode() + b"\n")
            stream.flush()
            line = stream.readline(131073)
            if not line or len(line) > 131072:
                raise ConnectionError("operator disconnected or oversized reply; inspect the session log")
            return json.loads(line)
