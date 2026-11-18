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
"""Exercise the operator, wall clock, stop path and JSON-lines sim contract with a local child."""

import base64
import io
import json
import os
import select
import signal
import statistics
import subprocess
import sys
import time
from pathlib import Path

import pytest
from PIL import Image

HERE = Path(__file__).resolve().parents[1] / "scripts/viscous-astra"
sys.path.insert(0, str(HERE))
from agent_bridge import Bridge  # noqa: E402
from transport import send  # noqa: E402


def jpeg(color):
    out = io.BytesIO()
    Image.new("RGB", (8, 8), color).save(out, format="JPEG")
    return base64.b64encode(out.getvalue()).decode()


def stub_child():
    assert len(sys.argv) == 1 and os.environ["MUJOCO_GL"] == "egl"
    images = {"head": jpeg("green"), "left": jpeg("purple")}
    energized = False
    torque_at = float("inf")
    for line in sys.stdin:
        req = json.loads(line)
        op = req["op"]
        reply = {"ok": True}
        if op == "init":
            assert set(req) == {"op", "run_dir", "variant", "task", "arm_file", "start"}
            assert Path(req["arm_file"]).is_absolute() and Path(req["arm_file"]).is_file()
            folder = Path(req["run_dir"])
            assert folder.is_absolute()
            pos = req["start"]
        elif op == "engage":
            energized = True
            reply["pos"] = pos
        elif op == "read":
            torque = dict.fromkeys(pos, 0.5)
            if time.monotonic() >= torque_at:
                torque["shoulder_lift"] = 13
            reply.update(pos=pos, torque=torque, temp=dict.fromkeys(pos, 31))
        elif op == "goal":
            assert energized and req["target"].keys() == pos.keys()
            pos = req["target"]
        elif op == "frames":
            reply.update(images)
        elif op == "mark":
            if req["label"] == "inject torque":
                torque_at = time.monotonic() + 0.85
        elif op == "truth":
            reply.update(object=[-0.27, 0.02, 0.001], success=False)
        elif op == "release":
            energized = False
        elif op not in ("gripper_off", "close"):
            raise AssertionError(op)
        with (folder / "stub.jsonl").open("a") as log:
            log.write(json.dumps({"t": time.monotonic(), **req}) + "\n")
        print(json.dumps(reply), flush=True)
        if op == "close":
            return


@pytest.fixture
def sim(tmp_path):
    from model import real

    def start(*options):
        # Replace only the child executable/script. Run the ordinary serve parser and endpoint.
        command = (
            "import sys; from pathlib import Path; sys.path.insert(0, sys.argv.pop(1)); "
            "import sim_arm, astra_op; sim_arm.SIM_PYTHON = sys.executable; "
            "sim_arm.SIM_CHILD = sys.argv.pop(1); sys.exit(astra_op.main())"
        )
        server = subprocess.Popen(
            [
                sys.executable,
                "-c",
                command,
                str(HERE),
                str(Path(__file__).resolve()),
                "serve",
                "--backend",
                "sim",
                "--arm-file",
                str(real.DEFAULT_ARM_FILE),
                "--pid-file",
                str(tmp_path / "serve.pid"),
                "--run-root",
                str(tmp_path / "runs"),
                "--tcp",
                "127.0.0.1:0",
                *options,
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        servers.append(server)
        assert select.select([server.stdout], [], [], 15)[0], "sim server startup timed out"
        line = server.stdout.readline()
        assert line.startswith("READY "), line
        return server, Path(line.strip()[6:])

    servers = []
    yield start
    for server in servers:
        if server.poll() is None:
            server.kill()
            server.wait()
        server.stdout.close()


def calls(folder):
    return [json.loads(line) for line in (folder / "stub.jsonl").read_text().splitlines()]


def test_sim_wall_time_motion_frames_and_idle_return(sim):
    server, folder = sim(
        "--armed", "--sim-task", "ball", "--sim-variant", "foamed", "--idle-deadline-s", "0.7"
    )
    bridge = Bridge(folder)
    for command in ("truth", "mark label", "quit"):
        with pytest.raises(ValueError, match="unknown agent command"):
            bridge.call("op", {"command": command})
    assert send(folder, ["truth"])["object"] == [-0.27, 0.02, 0.001]
    # This near-floor-independent gripper move takes over one deadline to execute.
    # Idle is charged from the completed reply; a later command must still work.
    reply = json.loads(bridge.call("op", {"command": "gripper 0.12"})[0]["text"])
    assert reply["ok"] and reply["sequence"] == 1
    assert reply["clearance"] == reply["commanded_clearance"]
    assert reply["sag_mm"] == pytest.approx(0)
    assert [item["data"] for item in bridge.call("frames", {}) if item["type"] == "image"] == [
        jpeg("green"),
        jpeg("purple"),
    ]
    assert send(folder, ["nudge", "up=0.015"])["ok"]
    assert server.wait(timeout=10) == 0
    stop = json.loads((folder / "stop.json").read_text())
    assert stop["reason"] == "idle deadline" and stop["returned"] == "at the rest"
    trace = calls(folder)
    ops = [r["op"] for r in trace]
    assert trace[0]["variant"] == "foamed" and trace[0]["task"] == "ball"
    assert ops[-2:] == ["release", "close"]
    assert ops.index("engage") < ops.index("goal") < ops.index("gripper_off")
    returning = ops[ops.index("gripper_off") :]
    assert "frames" not in returning and "goal" in returning
    assert returning.count("read") >= 2


def test_no_commands_needed_for_idle_torque_trip(sim):
    server, folder = sim("--armed")
    assert send(folder, ["mark", "inject torque"])["ok"]
    assert server.wait(timeout=8) != 0
    stop = json.loads((folder / "stop.json").read_text())
    assert "torque" in stop["reason"] and stop["returned"] == "at the rest"
    trace = calls(folder)
    marked = next(r["t"] for r in trace if r["op"] == "mark")
    ticks = [json.loads(line) for line in (folder / "ticks.jsonl").read_text().splitlines()]
    tripped = next(t for t in ticks if t["trips"])
    assert tripped["phase"] == "operator" and "torque" in tripped["trips"][0]
    idle = [r["t"] for r in trace if r["op"] == "read" and marked <= r["t"] <= tripped["t"]]
    assert len(idle) >= 10
    assert 0.045 < statistics.median(b - a for a, b in zip(idle, idle[1:], strict=False)) < 0.080
    assert not any(
        "request" in json.loads(line) for line in (folder / "commands.jsonl").read_text().splitlines()
    )


def test_unarmed_sim_closes_child_after_idle(sim):
    server, folder = sim("--idle-deadline-s", "0.25")
    assert server.wait(timeout=5) == 0
    assert json.loads((folder / "stop.json").read_text())["passive"]
    assert {r["op"] for r in calls(folder)} == {"init", "read", "frames", "close"}


def test_launcher_failure_leaves_armed_sim_for_one_red_button_press(sim, tmp_path):
    server, folder = sim("--armed")
    # Exercise the real launcher, including its EXIT behavior. No model process runs.
    stand_in = tmp_path / "codex"
    stand_in.write_text("#!/bin/sh\nexit 19\n")
    stand_in.chmod(0o700)
    result = subprocess.run(
        ["bash", HERE / "run_astra.sh", folder, "offline launcher test"],
        env={**os.environ, "PATH": f"{tmp_path}{os.pathsep}{os.environ['PATH']}"},
        capture_output=True,
        timeout=8,
        check=False,
    )
    assert result.returncode == 19
    assert not (folder / "stop.json").exists()
    assert send(folder, ["observe"])["sequence"] == 1
    pid = int((tmp_path / "serve.pid").read_text())
    assert pid == server.pid
    os.kill(pid, signal.SIGTERM)
    assert server.wait(timeout=6) == 0
    stop = json.loads((folder / "stop.json").read_text())
    assert stop["reason"] == "red button" and stop["returned"] == "at the rest"


if __name__ == "__main__":
    stub_child()
