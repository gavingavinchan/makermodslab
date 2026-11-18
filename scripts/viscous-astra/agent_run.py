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
"""Bound the agent's operator turns, adapted from the metal-arm-harness op/observe loop."""

import json
import subprocess
import sys
from pathlib import Path


def run(folder, command):
    budget = json.loads((folder / "connection.json").read_text())["budget"]
    # Each command has a paired image turn; count failed MCP calls too. Codex
    # does not expose an individual provider-request event for internal retries.
    counts = {"command": 0, "frames": 0, "turn": 0}
    with (
        (folder / "astra-transcript.jsonl").open("xb") as transcript,
        (folder / "astra-stderr.log").open("xb") as errors,
        subprocess.Popen(command, stdout=subprocess.PIPE, stderr=errors) as agent,
    ):
        for line in agent.stdout:
            transcript.write(line)
            transcript.flush()
            event = json.loads(line)
            item = event.get("item", {})
            if event["type"] == "turn.started":
                counts["turn"] += 1
            elif event["type"] == "item.started" and item.get("type") in (
                "mcp_tool_call",
                "command_execution",
                "web_search",
                "file_change",
            ):
                kind = (
                    "frames"
                    if item.get("server") == "viscous" and item.get("tool") == "frames"
                    else "command"
                )
                counts[kind] += 1
            if max(counts.values()) > budget:
                # End the agent only. The supervisor owns the red button and
                # serve keeps polling trips and its idle deadline until stopped.
                print(f"Agent budget exhausted: {counts}", file=sys.stderr)
                agent.terminate()
                agent.wait()
                return 2
        return agent.wait()


if __name__ == "__main__":
    sys.exit(run(Path(sys.argv[1]), sys.argv[2:]))
