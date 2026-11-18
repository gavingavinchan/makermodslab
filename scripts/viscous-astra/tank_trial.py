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
"""Run one reserved tank-simulation attempt, through the same operator and red button.

Use the Lab Python. Keep the root across restarts. Each (variant, task, repeat)
slot can launch Astra once, even if the launcher or this coordinator crashes.
"""

import argparse
import base64
import io
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

from agent_bridge import Bridge
from make_foamed_arm import SIM_PROFILES
from PIL import Image
from transport import send

HERE = Path(__file__).resolve().parent
TASKS = {
    "ball": "Pick up the white golf ball and lift it at least 50 mm above its starting position.",
    "screwdriver": "Pick up the screwdriver by its handle and lift it visibly clear of the floor.",
    "cube": "Pick up the 25 mm cube and place it on the red 40 mm square, within 20 mm of its center. Open and withdraw.",
}
HAND_BACK = (
    "Use one op and one paired frames read at a time. Briefly state what changed between moves. "
    "When done or unable to proceed, give your final report and hand back for the red button."
)
SMOKE = (
    "Approval test only. Use op observe followed by frames. Identify the two pictures. "
    "End your turn immediately after describing them. Leave the server up for the red button."
)


def save(path, data):
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")


def wait_for_ready(server, pid_file):
    pointer = Path(str(pid_file) + ".session")
    deadline = time.monotonic() + 100
    while time.monotonic() < deadline:
        if server.poll() is not None:
            raise RuntimeError(f"sim server exited {server.returncode} before READY (see server.log)")
        if pointer.is_file():
            return Path(pointer.read_text().strip())
        time.sleep(0.05)
    raise TimeoutError("sim server did not become ready")


def proof_route(task):
    # Known fixture coordinates are used by this deterministic check only.
    # The production model receives TASKS[task] and camera observations.
    pick_z = {"ball": -0.009, "screwdriver": -0.017, "cube": -0.017}[task]
    yield "observe"
    yield "gripper 0.32"
    yield "gripper 0.64"
    yield from [
        (0.457, -0.012, 0.115, -31),
        (0.420, -0.020, 0.100, -38),
        (0.397, -0.030, 0.090, -44),
        (0.377, -0.040, 0.080, -50),
        (0.354, -0.050, 0.065, -57),
        (0.330, -0.060, 0.055, -64),
        (0.325, -0.060, 0.035, -69),
        (0.325, -0.060, 0.014, -72),
        (0.325, -0.060, pick_z, -72),
    ]
    yield "gripper 0.32"
    yield "close"
    yield (0.325, -0.060, 0.005, -72)
    # Progressively level the tool as it rises; a steep tip at this reach
    # runs out of wrist travel. These are commanded poses in dream_base.
    yield from [
        "tip .341 -.061 .039 --pitch -64",
        "tip .340 -.061 .047 --pitch -61",
        "tip .340 -.061 .065 --pitch -55",
        "tip .340 -.061 .084 --pitch -49",
        "tip .345 -.061 .091 --pitch -46",
    ]
    if task == "cube":
        yield from [
            "tip .348 -.034 .083 --pitch -48",
            "tip .360 -.010 .080 --pitch -47",
            "tip .361 -.007 .055 --pitch -56",
            "tip .362 -.008 .027 --pitch -64",
            "tip .360 -.007 .018 --pitch -66",
            "gripper 0.32",
            "gripper 0.64",
            "tip .362 -.008 .039 --pitch -60",
            "tip .360 -.008 .057 --pitch -55",
        ]


def scripted(run, commands):
    bridge = Bridge(run)
    result = {}
    for command in commands:
        if isinstance(command, tuple):
            # Advance the measured TCP. A bounded one-step correction compensates
            # for PD holding error, and still goes through the ordinary planner.
            measured, goal = result["tip"]["xyz_m"], result["commanded_tip"]["xyz_m"]
            gain = 0 if command[0] > 0.35 else min(1.0, max(0.0, (0.060 - command[2]) / 0.045))
            coords = [
                v + gain * max(-0.065, min(0.065, c - m))
                for v, c, m in zip(command[:3], goal, measured, strict=True)
            ]
            pitch_err = result["commanded_tip"]["pitch_deg"] - result["tip"]["pitch_deg"]
            pitch = command[3] + gain * max(-8, min(8, pitch_err))
            command = "tip " + " ".join(f"{v:.5f}" for v in coords) + f" --pitch {pitch:.2f}"
        result = json.loads(bridge.call("op", {"command": command})[0]["text"])
        images = bridge.call("frames", {})
        if sum(item["type"] == "image" and bool(item["data"]) for item in images) != 2:
            raise RuntimeError("missing camera images")
        print(command, result["ok"], result["guard"], result["gripper"], flush=True)
        if not result["ok"]:
            return 1
    return 0


def astra(run, prompt, slot, budget):
    # Reserve before exec, never overwrite a previous launch record.
    with (slot / "model-launch.json").open("x") as f:
        json.dump({"utc_epoch": time.time(), "budget": budget, "task": prompt}, f)
    with (slot / "launcher.log").open("w") as log:
        return subprocess.run(
            ["bash", str(HERE / "run_astra.sh"), str(run), prompt],
            stdout=log,
            stderr=subprocess.STDOUT,
            check=False,
        ).returncode


def approved_images(run):
    """Only an executed observation followed by two actual JPEGs verifies tool approval."""
    observed = False
    with (run / "astra-transcript.jsonl").open() as stream:
        for line in stream:
            event = json.loads(line)
            item = event.get("item", {})
            if event["type"] != "item.completed" or item.get("server") != "viscous":
                continue
            result = item.get("result") or {}
            if item.get("error") or result.get("isError"):
                continue
            content = result.get("content", [])
            if item.get("tool") == "op" and item.get("arguments") == {"command": "observe"}:
                observed = bool(content and json.loads(content[0]["text"]).get("ok"))
            if observed and item.get("tool") == "frames":
                if [part["text"] for part in content if part["type"] == "text"] != ["head", "left"]:
                    return False
                images = [part for part in content if part["type"] == "image"]
                for part in images:
                    with Image.open(io.BytesIO(base64.b64decode(part["data"], validate=True))) as img:
                        if img.format != "JPEG":
                            return False
                        img.verify()
                return len(images) == 2
    return False


def attempt(args):
    mode, task, variant = args.mode, args.task, args.variant
    if mode == "campaign":
        receipt = args.root / "approval-ok.json"
        if not receipt.is_file():
            raise ValueError("the approval trial must return one successful observation and both JPEGs")
    slot = args.root / (f"{variant}-{task}-{args.repeat}" if mode == "campaign" else args.label or mode)
    slot.mkdir(parents=True, exist_ok=False)
    budget = 40
    pid_file = slot / "serve.pid"
    record = {"variant": variant, "task": task, "mode": mode, "start_epoch": time.time()}
    backend = (
        ["--backend", "fake"]
        if mode == "approval"
        else [
            "--backend",
            "sim",
            "--armed",
            "--sim-variant",
            variant,
            "--sim-task",
            task,
        ]
    )
    cmd = [
        sys.executable,
        str(HERE / "astra_op.py"),
        "serve",
        *backend,
        "--arm-file",
        str(SIM_PROFILES[variant]),
        "--run-root",
        str(slot / "runs"),
        "--pid-file",
        str(pid_file),
        "--tcp",
        "127.0.0.1:0",
        "--budget",
        str(budget),
        "--idle-deadline-s",
        "45" if variant == "current" else "240",
    ]
    with (slot / "server.log").open("w") as log:
        server = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT)
        try:
            run = wait_for_ready(server, pid_file)
            record.update(
                run=str(run), ready_epoch=time.time(), epoch_minus_monotonic=time.time() - time.monotonic()
            )
            if mode != "approval":
                initial = send(run, ["truth"])
                record["physics_epoch"] = time.time() - initial["t_s"]
            save(slot / "trial.json", record)
            if mode == "proof":
                record["operator_rc"] = scripted(
                    run, json.loads(args.commands.read_text()) if args.commands else proof_route(task)
                )
            else:
                prompt = SMOKE if mode == "approval" else TASKS[task] + " " + HAND_BACK
                record["operator_rc"] = astra(run, prompt, slot, budget)
        finally:
            record["handback_epoch"] = time.time()
            # The owner process and pid file must agree. Never signal a different session.
            if server.poll() is None and pid_file.is_file() and int(pid_file.read_text()) == server.pid:
                os.kill(server.pid, signal.SIGTERM)
                record["red_button"] = True
            try:
                record["server_rc"] = server.wait(timeout=180)
            except subprocess.TimeoutExpired:
                # No person can support a HOLDING arm tonight. Isolated simulator only.
                # Record an incomplete stop, then reap the simulator server (stdin closes).
                record["stop_incomplete"] = True
                server.kill()
                record["server_rc"] = server.wait()
            record["end_epoch"] = time.time()
            save(slot / "trial.json", record)
    if mode == "approval" and record["operator_rc"] == 0 and approved_images(run):
        save(args.root / "approval-ok.json", {"run": str(run), "verified": ["observe", "head", "left"]})
    return record.get("operator_rc", 1) or int(bool(record.get("stop_incomplete") or record["server_rc"]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["proof", "approval", "campaign"])
    parser.add_argument("--root", required=True, type=Path)
    parser.add_argument("--variant", choices=SIM_PROFILES, default="current")
    parser.add_argument("--task", choices=TASKS, default="ball")
    parser.add_argument("--repeat", type=int, choices=range(1, 4), default=1)
    parser.add_argument("--label", help="unique directory for scripted development runs")
    parser.add_argument("--commands", type=Path)
    args = parser.parse_args()
    if args.label and (args.mode != "proof" or Path(args.label).name != args.label):
        parser.error("--label must be a single directory name for a scripted run")
    args.root = args.root.expanduser().resolve()
    return attempt(args)


if __name__ == "__main__":
    raise SystemExit(main())
