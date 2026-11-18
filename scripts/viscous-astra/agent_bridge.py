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
"""Narrow stdio MCP adapter for the MakerMods-style op client and its two returned images."""

import base64
import json
import shlex
import sys
from pathlib import Path

# Start this trusted client with Python -I, outside the agent's writable workspace.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from transport import send  # noqa: E402


def tool(name, description, properties):
    return {
        "name": name,
        "description": description,
        "inputSchema": {
            "type": "object",
            "properties": properties,
            "required": list(properties),
            "additionalProperties": False,
        },
    }


class Bridge:
    def __init__(self, folder):
        self.folder = folder
        self.frames = None

    def call(self, name, args):
        if name == "frames" and not args and self.frames:
            batch = []
            for camera in ("head", "left"):
                path = Path(self.frames[camera]).resolve(strict=True)
                if path.parent != self.folder / "frames" / camera or path.suffix != ".jpg":
                    raise ValueError("invalid camera frame in operator reply")
                batch.extend(
                    [
                        {"type": "text", "text": camera},
                        {
                            "type": "image",
                            "mimeType": "image/jpeg",
                            "data": base64.b64encode(path.read_bytes()).decode(),
                        },
                    ]
                )
            self.frames = None
            return batch
        if name != "op" or set(args) != {"command"}:
            raise ValueError("use op, then frames")
        words = shlex.split(args["command"])
        if not words or words[0] not in (
            "observe",
            "status",
            "tip",
            "nudge",
            "gripper",
            "open",
            "close",
            "rest",
        ):
            raise ValueError("unknown agent command")
        if self.frames:
            raise ValueError("open the two pending JPEGs with frames first")
        reply = send(self.folder, words)
        self.frames = reply.get("frames")
        return [{"type": "text", "text": json.dumps(reply)}]

    def handle(self, msg):
        if "id" not in msg:
            return None
        reply = {"jsonrpc": "2.0", "id": msg["id"]}
        method = msg["method"]
        if method == "initialize":
            result = {
                "protocolVersion": msg["params"]["protocolVersion"],
                "capabilities": {"tools": {}},
                "serverInfo": {"name": "viscous-operator", "version": "1"},
            }
        elif method == "tools/list":
            result = {
                "tools": [
                    tool("op", "Issue exactly one operator command.", {"command": {"type": "string"}}),
                    tool("frames", "Open the head and wrist JPEGs from the preceding reply.", {}),
                ]
            }
        elif method == "tools/call":
            try:
                params = msg["params"]
                result = {"content": self.call(params["name"], params.get("arguments", {}))}
            except (ValueError, KeyError, OSError) as exc:
                result = {"isError": True, "content": [{"type": "text", "text": str(exc)}]}
        elif method == "ping":
            result = {}
        else:
            return {**reply, "error": {"code": -32601, "message": "unknown method"}}
        return {**reply, "result": result}


if __name__ == "__main__":
    bridge = Bridge(Path(sys.argv[1]).resolve(strict=True))
    for line in sys.stdin:
        reply = bridge.handle(json.loads(line))
        if reply is not None:
            print(json.dumps(reply), flush=True)
