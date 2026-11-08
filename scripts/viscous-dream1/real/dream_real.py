#!/usr/bin/env python3
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
"""Dream-1 on the REAL Viscous arm (waterproof unit, robot record viscous_waterproof_01). Goal B3.

Three modes (README.md has the preconditions):

  --dry-run  (default) The whole loop on FAKES: a fake arm, fake camera frames, and Dream's real client
             offline (its HTTP channel replaced by a local stub, network denied) or, without the client
             installed, a tiny internal stub. Prints the plan and its plan id. Opens no port, no camera,
             no network. `--scenario all` fires every safety trip once and shows the stop path.
  --preview  No motion. Opens the record's CAMERAS (only when Gavin runs it; --fake-cameras otherwise),
             runs Dream's real client offline, saves the frames as sent and prints the 12-hex inputs id
             that `python -m dream.input_check <id>` records. No CAN/serial, no motor, no billed call.
  --live     Billed Dream calls on the real arm. RefusedError unless every precondition holds (tool point
             set by `dreamcli tcp`, inputs checked, tank model approved, unverified joint signs
             acknowledged, the dry run's --plan-id, Lab idle, tank guard present, credentials).

Never run `python -m dream.check` on this arm: it moves it (hand circles, 10 deg joint moves).
Run with the repo venv; Dream's client comes from --dream-venv (default ~/viscous-dream-real/.venv):
  .venv/bin/python scripts/viscous-dream1/real/dream_real.py --dry-run --overhead top --scenario all
"""

from __future__ import annotations

import argparse
import base64
import collections
import contextlib
import dataclasses
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
import urllib.request
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import dream_real_core as core  # noqa: E402

REPO = core.ROOT
DEFAULT_ARM_FILE = HERE / "tank_model/viscous_waterproof_tank_arm.json"
BOX_FILE = HERE / "workspace_box_viscous_waterproof_01.json"
SRC_URDF = REPO / "configs/viscous/model/viscous_arm.urdf"
LEDGER = HERE / "runs/ledger.json"
LEDGER_CAP = 100  # total billed calls across all real-arm runs; raise it by editing the ledger file
CHECKPOINT = "dreamlabs/dream-1-beta"
YAM_EMBODIMENT = {
    "state": {"right_arm": [0, 6], "right_gripper": [6, 7], "left_arm": [7, 13], "left_gripper": [13, 14]},
    "action": {"right_arm": [0, 6], "right_gripper": [6, 7], "left_arm": [7, 13], "left_gripper": [13, 14]},
    "video": {role: f"observation.images.{role}" for role in core.SLOTS},
}
MIN_CLIENT = (1, 1, 3)
START_LIMIT_TOLERANCE_DEG = 5.0  # the arm rests a few degrees outside soft limits in the tank
SCENARIOS = ("nominal", "jump", "workspace", "lag", "torque", "sustained", "temperature", "stale", "signal")
# The pose the tank-measure trace last read (2026-11-08T04:44Z, torque off): the dry run's fake start.
TRACE_POSE = {
    "shoulder_pan": 189.03,
    "shoulder_lift": -201.76,
    "elbow_flex": 169.65,
    "wrist_flex": 47.31,
    "wrist_yaw": 31.31,
    "wrist_roll": -7.22,
    "gripper": -0.45,
}


class RefusedError(Exception):
    pass


class TripStopError(Exception):
    def __init__(self, trips):
        super().__init__("; ".join(f"{t.name}: {t.detail}" for t in trips))
        self.trips = trips


# --- environment: Lab home, record, profile, Dream venv -------------------------------


def lab_home(arg: str | None) -> Path:
    if arg:
        return Path(arg).expanduser()
    if os.environ.get("MAKERMODSLAB_HOME"):
        return Path(os.environ["MAKERMODSLAB_HOME"]).expanduser()
    branch = Path("~/.makermods/makermodslab-viscous").expanduser()  # this branch's service home
    return branch if branch.is_dir() else Path("~/.makermods/makermodslab").expanduser()


def load_record(home: Path, name: str) -> dict:
    path = home / "robots" / f"{name}.json"
    if not path.is_file():
        raise RefusedError(f"no robot record {path}")
    record = json.loads(path.read_text())
    if record.get("arm_type") != "viscous" or record.get("mode", "single") != "single":
        raise RefusedError(f"{name} is not a single Viscous arm")
    return record


def follower_limits(home: Path, record: dict) -> tuple[dict, str]:
    """The follower's soft limits as the Lab builds them (Viscous profile), read only."""
    os.environ["MAKERMODSLAB_HOME"] = str(home)
    try:
        from makermodslab.arms import registry

        config = registry.get("viscous").single_follower_config(
            record["follower_port"], record["follower_config"]
        )
        return {m: tuple(map(float, config.joint_limits[m])) for m in core.MOTORS}, "Lab ViscousFamily config"
    except ImportError:
        path = home / "viscous_profiles" / f"{record['follower_config']}.json"
        limits = json.loads(path.read_text())["follower"]["joint_limits"]
        return {m: tuple(map(float, limits[m])) for m in core.MOTORS}, str(path)


def add_dream_venv(venv: Path) -> str | None:
    """Append (never prepend) the Dream venv's site-packages, so the arm stack keeps its own packages."""
    site = next(iter(sorted(venv.expanduser().glob("lib/python3.*/site-packages"))), None)
    if site is None:
        return f"no site-packages under {venv}"
    if str(site) not in sys.path:
        sys.path.append(str(site))
    try:
        import importlib.metadata as md

        import dream  # noqa: F401
        import dream.input_check  # noqa: F401

        version = md.version("dreamlabs-client")
        if tuple(int(x) for x in version.split(".")[:3]) < MIN_CLIENT:
            return f"dreamlabs-client {version} is older than {'.'.join(map(str, MIN_CLIENT))}"
        from dream import client as dream_client

        if "_first_request" not in dream_client.PolicyClient.__init__.__code__.co_names:
            return "this client has no first-request hook; the inputs gate cannot be installed"
    except Exception as error:  # noqa: BLE001
        return f"cannot import Dream's client: {error}"
    return None


# --- Lab-idle and port checks (read-only) --------------------------------------------


def lab_holder(url: str, allow_down: bool) -> str | None:
    holders = []
    try:
        with urllib.request.urlopen(f"{url}/api/v1/sessions/current", timeout=3) as r:
            session = json.loads(r.read()).get("session")
        if session:
            holders.append(f"session {session.get('kind')} ({session.get('robot')})")
        with (
            contextlib.suppress(Exception),
            urllib.request.urlopen(f"{url}/api/v1/tank-measure/status", timeout=3) as r,
        ):
            if json.loads(r.read()).get("active"):
                holders.append("tank measurement")
    except Exception as error:  # noqa: BLE001
        if not allow_down:
            holders.append(
                f"Lab server at {url} not reachable ({error}); pass --lab-down-ok if it is stopped"
            )
    sessions = sys.modules.get("makermodslab.sessions")  # only when imported into a Lab process
    if sessions is not None and (held := sessions.held_by()):
        holders.append(f"in-process flag {held}")
    return "; ".join(holders) or None


def port_holders(port: str) -> list[str]:
    if not port or not Path(port).exists():
        return []
    try:
        out = subprocess.run(["lsof", "-t", port], capture_output=True, text=True, timeout=5).stdout.split()
    except Exception:  # noqa: BLE001
        return []
    return [f"pid {pid}" for pid in out if pid != str(os.getpid())]


# --- tank guard (makermodslab/tank_guard.py, branch feat/viscous-dream1-tank-box) ---------------


class Guard:
    """Adapter over the vertex-level tank keep-out guard, or a stand-in reporting its absence."""

    def __init__(self):
        self.problem = None
        self.guard = self.targets = None
        try:
            from makermodslab import tank_guard

            self.module = tank_guard
            self.guard = tank_guard.TankGuard.from_file()
        except Exception as error:  # noqa: BLE001
            self.problem = (
                f"makermodslab.tank_guard is not available ({error.__class__.__name__}: {error}); "
                "merge branch feat/viscous-dream1-tank-box first"
            )

    @property
    def available(self):
        return self.guard is not None

    def start(self, pose) -> str | None:
        if not self.available:
            return None
        try:
            self.targets = self.module.GuardedTargets(self.guard, pose)
        except ValueError as error:
            return str(error)
        return None

    def step(self, target):
        if not self.available:
            return "unguarded", target, ""
        d = self.targets.step(target)
        return d.status, d.target, d.reason

    def monitor(self, measured) -> str | None:
        if not self.available:
            return None
        v = self.guard.monitor(measured)
        return (
            None
            if v.ok
            else f"measured pose eats into the keep-out ({v.link} vs {v.face}, {v.slack_mm:.1f} mm)"
        )

    def path_clear(self, a, b) -> bool:
        return True if not self.available else self.guard.check_segment(a, b).ok


# --- the arm: a fake for dry runs, the Lab's CAN machinery for live --------------------------


@dataclasses.dataclass
class Reading:
    pos: dict
    torque: dict
    temp: dict
    frames: dict


def synthetic_frame(name: str, t: float) -> np.ndarray:
    """A 640x480 RGB test card, upright, with a dark bar at the bottom where the jaws enter."""
    h, w = 480, 640
    y, x = np.mgrid[0:h, 0:w]
    img = np.stack([(x * 255 // w), (y * 255 // h), np.full_like(x, 96 + 60 * (hash(name) % 3))], -1)
    img = img.astype(np.uint8)
    img[400:, 250:390] = 30  # "fingers" rising from the bottom
    img[20:40, int(20 + (t * 40) % 560) : int(60 + (t * 40) % 560)] = 255
    return img


class FakeArm:
    """Tracks targets with a first-order lag; scenarios inject faults. No hardware."""

    def __init__(self, start, cameras, clock, scenario="nominal"):
        self.pos = dict(start)
        self.target = dict(start)
        self.cameras, self.clock, self.scenario = list(cameras), clock, scenario
        self.ticks = 0
        self.events = []
        self.energized = False

    def connect(self):
        self.events.append("connect (fake: no port opened)")

    def read(self) -> Reading:
        self.ticks += 1
        if self.scenario == "stale" and self.ticks > 70:
            raise ConnectionError("No fresh feedback from motor 'elbow_flex' within 50 ms (fake)")
        if self.energized:
            for m in core.MOTORS:
                self.pos[m] += 0.6 * (self.target[m] - self.pos[m])
            if self.scenario == "lag" and self.ticks > 70:
                self.pos["elbow_flex"] -= 0.8 * (self.ticks - 70)  # the joint sags away from its target
        torque = {m: 0.4 + 0.1 * math.sin(self.ticks / 7 + i) for i, m in enumerate(core.MOTORS)}
        temp = dict.fromkeys(core.MOTORS, 31.0 + 0.002 * self.ticks)
        if self.scenario == "torque" and self.ticks > 70:
            torque["shoulder_lift"] = 6.2
        if self.scenario == "sustained" and self.ticks > 70:
            torque["shoulder_lift"] = 4.1
        if self.scenario == "temperature" and self.ticks > 50:
            temp["shoulder_lift"] = 31.0 + (self.ticks - 50)
        return Reading(
            dict(self.pos), torque, temp, {c: synthetic_frame(c, self.clock()) for c in self.cameras}
        )

    def engage(self):
        self.energized = True
        self.events.append("engage: torque on at the measured pose")
        return dict(self.pos)

    def send(self, target, previous):
        self.target = dict(target)

    def return_to_rest(self, pose, abort, speed):
        travel = max(abs(self.pos[m] - pose[m]) for m in pose)
        for m, v in pose.items():
            self.pos[m] = self.target[m] = v
        self.events.append(
            f"return to rest: {travel:.1f} deg max travel at {speed:.0f} deg/s, gripper left as is"
        )
        return True, "fake arm at rest"

    def release(self):
        self.energized = False
        self.events.append("release: zero-effort MIT frames, then torque off on every motor")
        return None


class LabArm:
    """The real follower through the Lab's CAN machinery. NEVER exercised by an agent.

    Mirrors remote_host.py's CAN path: rate-limit backstop installed before connect,
    connect(calibrate=False) then neutralize + torque off, fresh-feedback reads
    (remote_can.get_observation), engage = remote_can.engage, writes through
    remote_can.send_action, return via maker_rest_pose on fresh feedback, release
    via remote_can.release (neutralize + de_energize_can_device + cameras off),
    which is also the recovery when connect fails part-way.
    """

    def __init__(self, home: Path, record: dict, backstop_deg_s: float):
        self.home, self.record, self.backstop = home, record, backstop_deg_s
        self.robot = None

    def connect(self):
        os.environ["MAKERMODSLAB_HOME"] = str(self.home)
        from types import SimpleNamespace

        from lerobot.robots import make_robot_from_config
        from makermodslab import can_speed_limit, remote_can
        from makermodslab.camera_identity import resolve_cv2_index
        from makermodslab.maker_rest_pose import maker_follower_arms
        from makermodslab.record import _build_camera_configs, _platform_backend
        from makermodslab.utils.config import session_camera_config
        from makermodslab.utils.robot_factory import build_follower_config

        cameras = {}
        for entry in self.record.get("cameras") or []:
            cam = session_camera_config(entry)
            index = resolve_cv2_index(entry.get("unique_id"), cam.get("camera_index", 0))
            if index is None:
                raise RefusedError(f"camera {entry.get('name')!r} ({entry.get('unique_id')}) is not attached")
            cameras[entry["name"]] = {**cam, "camera_index": index}
        request = SimpleNamespace(
            robot_name=self.record["name"],
            arm_type="viscous",
            mode="single",
            leader_kind=None,
            follower_port=self.record["follower_port"],
            follower_config=self.record["follower_config"],
            leader_port="",
            leader_config="",
        )
        config = build_follower_config(request, cameras=_build_camera_configs(cameras, _platform_backend()))
        robot = make_robot_from_config(config)
        can_speed_limit.install(robot, self.backstop)
        remote_can.prepare(robot)
        self.robot = robot
        try:
            robot.connect(calibrate=False)
            # Connect can energize RobStride motors: park limp before anything else.
            remote_can.neutralize(robot)
            for arm, _label in maker_follower_arms(robot):
                arm.bus.disable_torque()
        except Exception:
            problem = remote_can.release(robot)
            print(f"connect failed; de-energize recovery: {problem or 'done'}", flush=True)
            self.robot = None
            raise

    def read(self) -> Reading:
        from makermodslab import remote_can

        obs = remote_can.get_observation(self.robot)  # raises on a stale or faulted motor
        frames = {c["name"]: obs[c["name"]] for c in self.record.get("cameras") or []}
        return Reading(
            {m: float(obs[f"{m}.pos"]) for m in core.MOTORS},
            {m: float(obs[f"{m}.torque"]) for m in core.MOTORS},
            {m: float(obs[f"{m}.temp"]) for m in core.MOTORS},
            frames,
        )

    def engage(self):
        from makermodslab import remote_can

        pose = remote_can.engage(self.robot)
        return {m: pose[f"{m}.pos"] for m in core.MOTORS}

    def send(self, target, previous):
        from makermodslab import remote_can

        remote_can.send_action(
            self.robot,
            {f"{m}.pos": v for m, v in target.items()},
            {f"{m}.pos": v for m, v in previous.items()},
        )

    def return_to_rest(self, pose, abort, speed):
        from makermodslab import remote_can
        from makermodslab.maker_rest_pose import MAKER_RETURN_CEILING_S, return_maker_to_pose

        robot = self.robot

        class Fresh:  # the rest-return helper on fresh CAN feedback (remote_can's pattern)
            def get_observation(self):
                return remote_can.get_observation(robot)

            def send_action(self, action):
                return robot.send_action(action)

        spans = [hi - lo for lo, hi in robot.config.joint_limits.values()]
        return return_maker_to_pose(
            Fresh(),
            pose,
            abort_event=abort,
            speed_deg_s=speed,
            ceiling_s=max(MAKER_RETURN_CEILING_S, max(spans) / speed + 3),
        )

    def release(self):
        from makermodslab import remote_can

        return remote_can.release(self.robot) if self.robot is not None else None


# --- Dream: the real client (offline stub or live), or an internal stub ------------------------------


class InputsNotCheckedError(Exception):
    pass


class PreflightDoneError(Exception):
    pass


class StubTransport:
    """Stands in for the client's HTTP channel (as viscous_dream_sim.py's dry run): records
    the request, answers a small synthetic hand motion around the anchor. Nothing leaves."""

    def __init__(self, real, scenario="nominal"):
        self.real, self.scenario, self.requests, self.calls = real, scenario, [], 0
        self.last_exchange = getattr(real, "last_exchange", None)

    def request(self, request, headers, **kwargs):
        if kwargs.get("path"):
            return {"status": {"code": "DA-0000"}}
        obs = request["observation"]
        self.requests.append(request)
        self.calls += 1
        rows, s = 48, np.linspace(0.0, 1.0, 48)[:, None]
        sign = 1.0 if self.calls % 2 else -1.0
        delta = (
            np.array([0.0, 0.25, 0.0])
            if self.scenario == "workspace"
            else sign * np.array([0.02, 0.0, -0.02])
        )
        per_key = {}
        for side in core.SIDES:
            xyz = np.asarray(obs[f"state.{side}_eef_xyz"], float)
            rot6 = np.asarray(obs[f"state.{side}_eef_rot6"], float)
            grip = float(np.asarray(obs[f"state.{side}_grip"], float).reshape(-1)[0])
            moving = side == "left"
            per_key[f"action.{side}_eef_xyz"] = (
                xyz + s * delta if moving else np.repeat(xyz[None], rows, 0)
            ).tolist()
            per_key[f"action.{side}_eef_rot6"] = np.repeat(rot6[None], rows, 0).tolist()
            per_key[f"action.{side}_grip"] = (
                grip + s * (0.5 - grip) if moving else np.full((rows, 1), grip)
            ).tolist()
        return {"status": {"code": "DA-0000"}, "session_id": "dry-run", "actions": {"per_key": per_key}}

    def close(self):
        self.real.close()


def deny_network():
    def hook(event, _args):
        if event in {"socket.connect", "socket.getaddrinfo", "socket.sendto"}:
            raise RuntimeError(f"network is disabled in this mode ({event})")

    sys.addaudithook(hook)


class DreamReal:
    """Dream's own client. offline=True patches the model lookup and the transport (no key, no network)."""

    def __init__(self, arm_file: Path, log_dir: Path, *, offline: bool, scenario="nominal", api_ok=False):
        os.environ["DREAM_LOG_DIR"] = str(log_dir / "dream_logs")
        os.environ.setdefault(
            "JAX_COMPILATION_CACHE_DIR", str(Path("~/viscous-dream-real/.cache/jax").expanduser())
        )
        import warnings

        import dream

        warnings.filterwarnings("ignore", category=dream.DreamSlowNetworkWarning)
        self.dream = dream
        self.config = {
            "checkpoint": CHECKPOINT,
            "embodiment": YAM_EMBODIMENT,
            "ik": {"arm": str(arm_file), "head": "eef"},
            "timeout_ms": 45000,
            "strict": False,
            "action_hz": 20,
            "_record": False,
        }
        self.client = dream.DreamClient(api_key="dry-run-no-key" if offline else None)
        if offline:
            from unittest.mock import patch

            from dream.models import Model

            model = Model(
                id=CHECKPOINT,
                owner="public",
                base_model=CHECKPOINT,
                embodiment="yam",
                servable=True,
                created_at=None,
                cameras=list(core.SLOTS),
            )
            with patch("dream.models._record", return_value=model):
                self.client.policy(self.config)
            self.stub = StubTransport(self.client._policy._transport, scenario)
            self.client._policy._transport = self.stub
        else:
            self.client.policy(self.config)
            self.stub = None
        self.inputs_id = None
        self.inputs_checked = None
        self.sent_observation = None
        self.mode = "gate"  # gate | record | preflight

    def install_gate(self, mode: str):
        """Replace the client's first-request reminder. It runs on the observation AS SENT, just before
        the transport (dream/client.py `_post`), so raising there sends nothing."""
        from dream import input_check

        self.mode = mode

        def gate(obs):
            self.sent_observation = obs
            self.inputs_id = input_check.inputs_id(obs)
            self.inputs_checked = input_check.checked(self.inputs_id)
            if not self.inputs_checked:
                input_check.remind(obs)  # Dream's own warning, verbatim: never suppressed
                print(
                    "  (On this arm use the runner's --preview to look, never `python -m dream.check`: it moves "
                    "the arm.)",
                    file=sys.stderr,
                )
            if self.mode == "preflight":
                raise PreflightDoneError()
            if self.mode == "gate" and not self.inputs_checked:
                raise InputsNotCheckedError(self.inputs_id)

        self.client._policy._first_request = gate

    def set_observation(self, frames, state, t):
        self.client.set_observation(frames, state, timestamp=t)

    def request(self, commanded):
        return self.client.get_action(commanded=commanded)

    def close(self):
        with contextlib.suppress(Exception):
            self.client.close()


class DreamInternalStub:
    """When Dream's client is not installed: joint rows straight from the commanded row (no IK)."""

    def __init__(self, jmap, scenario="nominal"):
        self.jmap, self.scenario, self.calls = jmap, scenario, 0
        self.inputs_id, self.inputs_checked, self.state = None, None, None

    def set_observation(self, frames, state, t):
        self.state = state

    def request(self, commanded):
        self.calls += 1
        base = np.asarray(commanded["q_l_chunk"], float)
        s = np.linspace(0, 1, 48)[:, None]
        sign = 1.0 if self.calls % 2 else -1.0
        delta = np.array([0, 0.03, -0.03, 0.02, 0, 0, -0.2]) * sign
        if self.scenario == "workspace":
            delta = np.array([-0.9, 0, 0, 0, 0, 0, 0])  # pan swings the hand sideways
        rows = base + s * delta
        return {
            "q_l_chunk": rows.tolist(),
            "q_r_chunk": np.repeat(np.asarray(commanded["q_r_chunk"])[None], 48, 0),
        }

    def close(self):
        pass


class Worker:
    """One get_action at a time, off the control thread (live), or completed after a virtual latency (dry)."""

    def __init__(self, dream, clock, *, threaded, latency_s=1.5, ledger=None, run_id="", max_calls=10):
        self.dream, self.clock, self.threaded, self.latency = dream, clock, threaded, latency_s
        self.ledger, self.run_id, self.max_calls = ledger, run_id, max_calls
        self.calls = 0
        self.busy = False
        self.result = None
        self.ready_at = None

    def _call(self, commanded):
        request = self.ledger.reserve(self.run_id, self.max_calls) if self.ledger else None
        while True:
            try:
                reply = self.dream.request(commanded)
                if request:
                    self.ledger.finish(request["id"], "billed")
                return reply, None, request
            except InputsNotCheckedError as error:
                if request:
                    self.ledger.finish(request["id"], "not_sent")
                return None, f"inputs not checked (id {error}); nothing was sent", request
            except Exception as error:  # noqa: BLE001
                if getattr(error, "reason", None) == "pool_warming" and request:
                    self.ledger.finish(request["id"], "pool_warming")
                    time.sleep(min(getattr(error, "retry_after_s", None) or 10, 30))
                    request = self.ledger.reserve(self.run_id, self.max_calls)
                    continue
                return None, f"{error.__class__.__name__}: {error}", request

    def start(self, commanded):
        self.busy, self.result = True, None
        self.calls += 1
        if self.threaded:

            def run():
                self.result = self._call(commanded)

            threading.Thread(target=run, daemon=True).start()
        else:
            self.result = self._call(commanded)
            self.ready_at = self.clock() + self.latency

    def take(self):
        if not self.busy or self.result is None or (not self.threaded and self.clock() < self.ready_at):
            return None
        self.busy = False
        return self.result


# --- the episode ---------------------------------------------------------------------


@dataclasses.dataclass
class Settings:
    prompt: str
    rows: int
    max_calls: int
    speed_deg_s: float
    grip_deg_s: float
    max_jump_deg: float
    limit_margin_deg: float
    trips: core.TripLimits
    slots: dict


class Episode:
    """The control loop. Every exit goes through stop(): return to rest, then torque off."""

    def __init__(
        self,
        arm,
        dream,
        worker,
        jmap,
        limits,
        box,
        guard,
        settings,
        clock,
        sleep,
        log_dir,
        stop_event,
        abort_event,
        mutate=None,
    ):
        self.arm, self.dream, self.worker, self.jmap, self.limits = arm, dream, worker, jmap, limits
        self.box, self.guard, self.s, self.clock, self.sleep = box, guard, settings, clock, sleep
        self.log_dir, self.stop_event, self.abort_event, self.mutate = (
            log_dir,
            stop_event,
            abort_event,
            mutate,
        )
        self.monitor = core.TripMonitor(settings.trips, clock)
        arm_file = (
            json.loads(Path(dream.config["ik"]["arm"]).read_text()) if hasattr(dream, "config") else None
        )
        rest = arm_file["rest"] if arm_file else json.loads(DEFAULT_ARM_FILE.read_text())["rest"]
        self.right_rest = [float(v) for v in rest]
        self.summary = {
            "trips": [],
            "calls": 0,
            "rows_planned": 0,
            "substeps_sent": 0,
            "clipped": 0,
            "guard": collections.Counter(),
            "stretched_rows": 0,
        }
        self.energized = False
        self.start_pose = None
        self.commanded = None
        self.ticks = log_dir / "ticks.jsonl"
        self.chunks = log_dir / "chunks.jsonl"

    def _write(self, path, obj):
        with path.open("a") as f:
            f.write(json.dumps(obj, default=float) + "\n")

    def start_gate(self, pose) -> list[str]:
        problems = []
        for m in core.MOTORS:
            lo, hi = self.limits[m]
            if not lo - START_LIMIT_TOLERANCE_DEG <= pose[m] <= hi + START_LIMIT_TOLERANCE_DEG:
                problems.append(
                    f"{m} at {pose[m]:.1f} is more than {START_LIMIT_TOLERANCE_DEG} deg outside "
                    f"[{lo:.1f}, {hi:.1f}]"
                )
        problems += [f"workspace box: {v}" for v in self.box.violations(pose)]
        if g := self.guard.start(pose):
            problems.append(f"tank guard: {g}")
        return problems

    def plan_chunk(self, reply) -> list[dict]:
        rows = np.array(reply["q_l_chunk"], float)[: self.s.rows]
        rows[:, 6] = np.clip(rows[:, 6], 0.0, 1.0)
        if self.mutate:
            rows = self.mutate(rows)
        out, prev = [], dict(self.commanded)
        for i, row in enumerate(rows):
            if not np.isfinite(row).all() or row.shape != (7,):
                raise TripStopError([core.Trip("bad_chunk", f"row {i} is {row.tolist()}")])
            target = core.ik_row_to_motor(row, self.jmap)
            if bad := core.jumps(prev, target, self.s.max_jump_deg):
                raise TripStopError(
                    [
                        core.Trip(
                            "jump",
                            f"row {i}: {', '.join(f'{m} {target[m] - prev[m]:+.1f} deg' for m in bad)}",
                        )
                    ]
                )
            target, clipped = core.clamp_to_limits(target, self.limits, self.s.limit_margin_deg)
            self.summary["clipped"] += bool(clipped)
            steps = core.retime(prev, target, arm_deg_s=self.s.speed_deg_s, grip_deg_s=self.s.grip_deg_s)
            self.summary["stretched_rows"] += len(steps) > 1
            for step in steps:
                if bad := self.box.violations(step):
                    raise TripStopError([core.Trip("workspace", f"row {i}: {bad[0]}")])
                status, step, reason = self.guard.step(step)
                self.summary["guard"][status] += 1
                if status == "rejected":
                    raise TripStopError([core.Trip("tank_guard", f"row {i}: {reason}")])
                out.append(step)
                prev = step
        self.summary["rows_planned"] += len(rows)
        return out

    def run(self) -> dict:
        frames_fed, queue = 0, collections.deque()
        reason = "finished"
        self.arm.connect()
        try:
            reading = self.arm.read()
            self.start_pose = dict(reading.pos)
            if problems := self.start_gate(self.start_pose):
                raise RefusedError("start pose refused: " + "; ".join(problems))
            self.commanded = self.arm.engage()
            self.energized = True
            next_tick = next_frame = self.clock()
            while True:
                now = self.clock()
                try:
                    reading = self.arm.read()
                except Exception as error:  # noqa: BLE001 - stale reply, motor fault, camera read
                    raise TripStopError(
                        [core.Trip("feedback", f"{error.__class__.__name__}: {error}")]
                    ) from error
                trips = self.monitor.update(reading.pos, self.commanded, reading.torque, reading.temp)
                if g := self.guard.monitor(reading.pos):
                    trips.append(core.Trip("tank_guard_measured", g))
                self._write(
                    self.ticks,
                    {
                        "t": now,
                        "pos": reading.pos,
                        "cmd": self.commanded,
                        "torque": reading.torque,
                        "temp": reading.temp,
                        "trips": [t.name for t in trips],
                    },
                )
                if trips:
                    raise TripStopError(trips)
                if now >= next_frame:
                    frames = core.slot_frames(reading.frames, self.s.slots)
                    state = core.dream_state(reading.pos, self.jmap, self.right_rest, self.s.prompt)
                    self.dream.set_observation(frames, state, now)
                    frames_fed += 1
                    next_frame += core.FRAME_PERIOD_S
                if self.stop_event.is_set():
                    reason = "stop signal"
                    break
                previous = self.commanded
                if queue:
                    self.commanded = queue.popleft()
                    self.summary["substeps_sent"] += 1
                self.arm.send(self.commanded, previous)  # an empty queue re-sends the hold target
                if not queue and not self.worker.busy and frames_fed >= core.FRAMES_IN_WINDOW:
                    if self.worker.calls >= self.s.max_calls:
                        reason = f"call cap {self.s.max_calls} reached"
                        break
                    row = core.motor_to_ik_row(self.commanded, self.jmap)
                    self.worker.start({"q_l_chunk": row, "q_r_chunk": [*self.right_rest, 1.0]})
                result = self.worker.take()
                if result is not None:
                    reply, error, request = result
                    if error:
                        raise TripStopError([core.Trip("dream", error)])
                    self.summary["calls"] += 1
                    plan = self.plan_chunk(reply)
                    self._write(
                        self.chunks,
                        {
                            "call": self.summary["calls"],
                            "ledger": request,
                            "q_l_chunk": np.asarray(reply["q_l_chunk"]).tolist(),
                            "per_key": {
                                k: np.asarray(v).tolist() for k, v in (reply.get("per_key") or {}).items()
                            },
                            "substeps": plan,
                        },
                    )
                    queue.extend(plan)
                next_tick += core.ROW_DT_S
                self.sleep(max(0.0, next_tick - self.clock()))
        except TripStopError as stop:
            reason = "trip: " + str(stop)
            self.summary["trips"] = [dataclasses.asdict(t) for t in stop.trips]
        except RefusedError as refused:
            reason = f"refused: {refused}"
        except Exception as error:  # noqa: BLE001
            reason = f"error: {error.__class__.__name__}: {error}"
        finally:
            self.summary["stop"] = self.stop(reason)
        self.summary.update(
            reason=reason,
            peak_torque_nm=self.monitor.peak_torque,
            peak_temp_c=self.monitor.peak_temp,
            peak_lag_deg=self.monitor.peak_lag,
            guard=dict(self.summary["guard"]),
        )
        return self.summary

    def stop(self, reason) -> dict:
        """Every exit: return to the start pose (arm joints; the gripper keeps its grip) at the
        speed cap, then torque off. A second signal aborts the return, then releases."""
        out = {"reason": reason, "returned": None, "release_problems": None}
        try:
            if self.energized:
                pose = {m: self.start_pose[m] for m in core.ARM}
                here = dict(self.commanded)
                if not self.guard.path_clear(here, {**pose, "gripper": here["gripper"]}):
                    print(
                        "RETURN PATH NOT CLEAR of the tank keep-out: HOLDING. Support the arm, then signal "
                        "again (Ctrl-C / kill -TERM) to release torque.",
                        flush=True,
                    )
                    while not self.abort_event.is_set():
                        self.sleep(0.1)
                    out["returned"] = "held, released on the second signal"
                else:
                    ok, message = self.arm.return_to_rest(pose, self.abort_event, self.s.speed_deg_s)
                    out["returned"] = {"ok": ok, "message": message}
        finally:
            out["release_problems"] = self.arm.release()
            self.energized = False
        return out


def effective_arm_file(source: Path, keep_rest: bool, out: Path) -> tuple[Path, str]:
    """The arm file the client loads. `dreamcli tcp` writes `rest` as zeros (a straight, level arm), and
    `rest` is outside the tool point's digest, so unless --keep-arm-rest a run-local COPY is written in the
    run folder: the tool keys and the tcp_calibration record byte-for-byte unchanged, the URDF copied
    byte-for-byte beside it, only `rest` taken from the tank model. Gavin's own file is never edited."""
    arm = json.loads(source.read_text())
    tank_rest = json.loads(DEFAULT_ARM_FILE.read_text())["rest"]
    if keep_rest or arm["rest"] == tank_rest:
        return source, f"rest {arm['rest']} (the arm file's own)"
    if arm.get("base") != "dream_base":
        return source, f"rest {arm['rest']} (the arm file's own; not a tank model, so not overridden)"
    dest = out / "arm"
    dest.mkdir(parents=True, exist_ok=True)
    (dest / arm["urdf"]).write_bytes((source.parent / arm["urdf"]).read_bytes())
    copy = {**arm, "rest": tank_rest}
    (dest / source.name).write_text(json.dumps(copy, indent=1) + "\n")
    urdf = (dest / arm["urdf"]).read_bytes()
    if core.tool_point_problem(arm, urdf) != core.tool_point_problem(copy, urdf):
        raise RefusedError("the rest-only copy changed the tool point verdict; refusing")
    return (
        dest / source.name,
        f"rest {tank_rest} from the tank model, in a run-local copy {dest / source.name}",
    )


# --- plan ---------------------------------------------------------------------------


def joint_plan(m, bounds, ik, jmap, settings):
    lo, hi = bounds
    grip = f"grip 0..1 -> motor {core.GRIP_CLOSED_DEG:.1f} (closed) .. {core.GRIP_OPEN_DEG:.1f} (open)"
    return {
        "motor_limits_deg": [round(lo, 2), round(hi, 2)],
        "commanded_within_deg": [
            round(lo + settings.limit_margin_deg, 2),
            round(hi - settings.limit_margin_deg, 2),
        ],
        "ik_limits_deg": [round(v, 2) for v in ik[m]] if m in ik else grip,
        "sign_to_urdf": jmap.sign.get(m),
        "sign_status": jmap.status.get(m, "n/a"),
    }


def build_plan(
    args,
    record,
    limits,
    limits_source,
    arm_path,
    arm,
    urdf_bytes,
    provenance,
    slots,
    missing,
    jmap,
    settings,
    box_config,
    guard,
    dream_status,
):
    cameras = {
        c["name"]: {k: c.get(k) for k in ("camera_index", "unique_id", "width", "height", "fps", "rotation")}
        for c in record.get("cameras") or []
    }
    urdf_joints = {j.name: j for j in core.parse_urdf(urdf_bytes).values()}
    ik = {m: (math.degrees(urdf_joints[m].lower), math.degrees(urdf_joints[m].upper)) for m in core.ARM}
    plan = {
        "robot": record["name"],
        "follower_port": record["follower_port"],
        "follower_config": record["follower_config"],
        "joints_that_move": {m: joint_plan(m, limits[m], ik, jmap, settings) for m in core.MOTORS},
        "limits_source": limits_source,
        "right_arm": "none on the rig; reported to Dream parked at the arm file's rest, grip 1 (open); its answers are dropped",
        "speed": {
            "arm_target_deg_s": settings.speed_deg_s,
            "gripper_deg_s": settings.grip_deg_s,
            "retime": "each row split into equal 20 Hz substeps, all joints arriving together",
            "bus_backstop_deg_s": min(30.0, 2 * settings.speed_deg_s),
            "rows_per_chunk": settings.rows,
            "max_calls": settings.max_calls,
        },
        "arm_file": {
            "path": str(arm_path),
            "base": arm["base"],
            "flange": arm["flange"],
            "tcp": arm["tcp"],
            "rest_ik_rad": arm["rest"],
            "urdf_sha256": core.sha256_bytes(urdf_bytes),
            "tcp_problem": core.tool_point_problem(arm, urdf_bytes),
            "approval_problem": core.approval_problem(urdf_bytes, provenance, arm["base"]),
        },
        "cameras_in_record": cameras,
        "dream_slots": {
            slot: (
                src
                if src in ("black",) or src.startswith("copy:")
                else f"{src} (rotation {cameras[src].get('rotation') or 0} applied by lerobot at capture; "
                f"{cameras[src].get('width')}x{cameras[src].get('height')} -> centre-crop 16:9 -> 320x176 RGB JPEG q85)"
            )
            for slot, src in slots.items()
        },
        "missing_slots": missing,
        "prompt": settings.prompt,
        "trips": {
            "joint_clamp": f"every target clipped {settings.limit_margin_deg} deg inside the motor limits (a notice, not a stop)",
            "jump": f"any arm joint target > {settings.max_jump_deg} deg from the last -> refuse chunk, stop",
            "workspace_box": f"{BOX_FILE.name}: tcp/wrist/wrist_pitch/elbow boxes on every substep -> refuse chunk, stop",
            "tank_guard": "every substep through makermodslab.tank_guard GuardedTargets.step (rejected -> stop) and "
            "every measured pose through monitor()"
            if guard.available
            else f"NOT AVAILABLE: {guard.problem}",
            "lag": f"measured vs commanded > {settings.trips.lag_deg} deg on any arm joint for {settings.trips.lag_ticks} ticks",
            "torque": f"|torque| > {settings.trips.torque_trip_nm} Nm for {settings.trips.torque_trip_ticks} ticks, or > "
            f"{settings.trips.torque_cap_nm} Nm for {settings.trips.torque_cap_s} s",
            "temperature": f">= {settings.trips.temp_trip_c} C (warn {settings.trips.temp_warn_c})",
            "feedback": "no fresh reply / motor fault / camera read error -> stop",
            "dream": "Dream error or a malformed / non-finite chunk -> stop",
            "inputs_unchecked": "first request refused locally (nothing sent) unless ~/.dreamlabs/inputs_checked holds its id",
            "start_gate": f"start pose within {START_LIMIT_TOLERANCE_DEG} deg of limits, inside the box and the guard",
            "stop": "SIGINT/SIGTERM/SIGHUP: return to the start pose at the speed cap (guard-checked path), then "
            "torque off; a second signal aborts the return and releases. Every exit path does this.",
        },
        "box_config_sha256": core.sha256_bytes(BOX_FILE.read_bytes()),
        "box_points_mm": {k: v["box_mm"] for k, v in box_config["points"].items()},
        "dream_client": dream_status,
    }
    safety = {
        k: plan[k]
        for k in (
            "robot",
            "follower_port",
            "joints_that_move",
            "speed",
            "dream_slots",
            "prompt",
            "trips",
            "box_config_sha256",
        )
    }
    safety["arm_urdf"] = plan["arm_file"]["urdf_sha256"]
    safety["arm_tool"] = {k: arm[k] for k in core.TOOL_KEYS}
    plan["plan_id"] = core.canonical_id(safety)
    return plan


def print_plan(plan):
    print(f"\n=== Dream-1 real-arm plan (plan id {plan['plan_id']}) ===")
    print(f"robot {plan['robot']}  port {plan['follower_port']}  limits from {plan['limits_source']}")
    print("Joints that may move (ALL SEVEN): motor limits / commanded window / IK limits / sign")
    for m, j in plan["joints_that_move"].items():
        print(
            f"  {m:13s} {j['motor_limits_deg']} -> {j['commanded_within_deg']}  IK {j['ik_limits_deg']}  "
            f"sign {j['sign_to_urdf']} ({j['sign_status']})"
        )
    print(f"Right arm: {plan['right_arm']}")
    sp = plan["speed"]
    print(
        f"Speed: arm targets <= {sp['arm_target_deg_s']} deg/s, gripper <= {sp['gripper_deg_s']} deg/s "
        f"({sp['retime']}); bus backstop {sp['bus_backstop_deg_s']} deg/s; {sp['rows_per_chunk']} rows/chunk, "
        f"<= {sp['max_calls']} calls"
    )
    a = plan["arm_file"]
    print(
        f"Arm file: {a['path']}\n  base {a['base']} flange {a['flange']} tcp {a['tcp']} rest {a['rest_ik_rad']}"
    )
    print(f"  tool point: {a['tcp_problem'] or 'OK (set by a person)'}")
    print(f"  tank model approval: {a['approval_problem'] or 'OK'}")
    print("Dream slots:")
    for slot, src in plan["dream_slots"].items():
        print(f"  video.{slot:9s} <- {src}")
    for m in plan["missing_slots"]:
        print(f"  MISSING: {m}")
    print(f"Prompt: {plan['prompt']!r}")
    print(
        "Workspace box (physical base frame, mm): "
        + "; ".join(f"{k} x{v['x']} y{v['y']} z{v['z']}" for k, v in plan["box_points_mm"].items())
    )
    print("Safety trips:")
    for k, v in plan["trips"].items():
        print(f"  {k:17s} {v}")
    print(f"Dream client: {plan['dream_client']}")


# --- modes ------------------------------------------------------------------------------------


class VirtualClock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t

    def sleep(self, dt):
        self.t += max(0.0, dt)


def jump_mutation(rows):
    rows = rows.copy()
    rows[10:, 0] += math.radians(30)
    return rows


def run_dry(args, ctx) -> int:
    deny_network()  # after the read-only Lab check: nothing below may reach the network
    results = []
    scenarios = SCENARIOS if args.scenario == "all" else (args.scenario,)
    for scenario in scenarios:
        clock = VirtualClock()
        out = ctx["out"] / scenario
        out.mkdir(parents=True, exist_ok=True)
        start = {**TRACE_POSE, **(args.start_motor_deg or {})}
        arm = FakeArm(
            start, [c["name"] for c in ctx["record"].get("cameras") or []] + ["(fake)"], clock, scenario
        )
        if ctx["dream_problem"] is None and not args.internal_stub:
            dream = DreamReal(ctx["arm_path"], out, offline=True, scenario=scenario)
            dream.install_gate("record")
        else:
            dream = DreamInternalStub(ctx["jmap"], scenario)
        stop_event, abort_event = threading.Event(), threading.Event()
        worker = Worker(dream, clock, threaded=False, latency_s=1.5, max_calls=ctx["settings"].max_calls)

        def sleep(dt, clock=clock, stop_event=stop_event, scenario=scenario):
            clock.sleep(dt)
            if scenario == "signal" and clock.t > 1006.0:
                stop_event.set()

        episode = Episode(
            arm,
            dream,
            worker,
            ctx["jmap"],
            ctx["limits"],
            ctx["box"],
            ctx["guard"],
            ctx["settings"],
            clock,
            sleep,
            out,
            stop_event,
            abort_event,
            mutate=jump_mutation if scenario == "jump" else None,
        )
        summary = episode.run()
        summary["fake_arm_events"] = arm.events
        summary["inputs_id"] = getattr(dream, "inputs_id", None)
        dream.close()
        core.atomic_json(out / "summary.json", summary)
        results.append((scenario, summary))
    print("\n=== Dry run on fakes (virtual clock; no port, camera, network or billed call) ===")
    for scenario, s in results:
        stop = s["stop"]
        print(f"- {scenario:11s} -> {s['reason'][:110]}")
        print(
            f"    calls {s['calls']}, rows {s['rows_planned']}, substeps sent {s['substeps_sent']}, "
            f"rows stretched by the speed cap {s['stretched_rows']}, clipped {s['clipped']}, guard {s['guard'] or 'n/a'}"
        )
        print(f"    stop path: return {stop['returned']}; release problems: {stop['release_problems']}")
        print(
            f"    peak lag {max(s['peak_lag_deg'].values()):.1f} deg, peak torque "
            f"{max(s['peak_torque_nm'].values()):.2f} Nm, peak temp {max(s['peak_temp_c'].values()):.1f} C"
        )
    if results and results[0][1].get("inputs_id"):
        print(
            f"(inputs id of these FAKE frames: {results[0][1]['inputs_id']}; the --preview id with the real "
            "cameras is the one Gavin records)"
        )
    print(f"logs: {ctx['out']}")
    return 0


def open_cameras(record, fake):
    """Real lerobot cameras with the record's config (rotation applied at capture), or fakes."""
    names = [c["name"] for c in record.get("cameras") or []]
    if fake:
        return names, (lambda: {n: synthetic_frame(n, time.monotonic()) for n in names}), (lambda: None)
    from lerobot.cameras.utils import make_cameras_from_configs
    from makermodslab.camera_identity import resolve_cv2_index
    from makermodslab.record import _build_camera_configs, _platform_backend
    from makermodslab.utils.config import session_camera_config

    configs = {}
    for entry in record.get("cameras") or []:
        cam = session_camera_config(entry)
        index = resolve_cv2_index(entry.get("unique_id"), cam.get("camera_index", 0))
        if index is None:
            raise RefusedError(f"camera {entry['name']!r} ({entry.get('unique_id')}) is not attached")
        configs[entry["name"]] = {**cam, "camera_index": index}
    cameras = make_cameras_from_configs(_build_camera_configs(configs, _platform_backend()))
    for cam in cameras.values():
        cam.connect()

    def close():
        for cam in cameras.values():
            with contextlib.suppress(Exception):
                cam.disconnect()

    return names, (lambda: {n: c.read_latest() for n, c in cameras.items()}), close


def run_preview(args, ctx) -> int:
    if ctx["dream_problem"]:
        print(f"--preview needs Dream's client: {ctx['dream_problem']}")
        return 2
    deny_network()
    out = ctx["out"]
    out.mkdir(parents=True, exist_ok=True)
    dream = DreamReal(ctx["arm_path"], out, offline=True)
    dream.install_gate("record")
    _, grab, close = open_cameras(ctx["record"], args.fake_cameras)
    rest = [float(v) for v in ctx["arm"]["rest"]]
    motor = {**ctx["jmap"].to_motor(rest), "gripper": core.GRIP_OPEN_DEG}
    if args.start_motor_deg:
        motor.update(args.start_motor_deg)
    state = core.dream_state(motor, ctx["jmap"], rest, ctx["settings"].prompt)
    try:
        t0 = time.monotonic()
        for i in range(core.FRAMES_IN_WINDOW):
            dream.set_observation(
                core.slot_frames(grab(), ctx["settings"].slots), state, t0 + i * core.FRAME_PERIOD_S
            )
            time.sleep(core.FRAME_PERIOD_S)
        dream.request(
            {"q_l_chunk": [*state["state.left_arm"], state["state.left_grip"][0]], "q_r_chunk": [*rest, 1.0]}
        )
    finally:
        close()
        dream.close()
    sent = dream.sent_observation
    from PIL import Image

    tiles = []
    for slot in core.SLOTS:
        value = sent[f"video.{slot}"]
        jpeg = base64.b64decode(value[-1] if isinstance(value, list) else value)
        (out / f"{slot}.jpg").write_bytes(jpeg)
        tiles.append(np.asarray(Image.open(out / f"{slot}.jpg").convert("RGB")))
    Image.fromarray(np.concatenate(tiles, axis=1)).save(out / "sheet.jpg", quality=90)
    values = {k: v for k, v in sent.items() if not k.startswith("video.")}
    core.atomic_json(
        out / "observation_as_sent.json",
        {"values": values, "inputs_id": dream.inputs_id, "slots": ctx["settings"].slots},
    )
    print("\n=== Inputs preview (no motor, no CAN/serial, no network, no billed call) ===")
    print(
        "frames as sent (oldest->newest, newest saved): "
        + ", ".join(str(out / f"{s}.jpg") for s in core.SLOTS)
    )
    print(f"side by side: {out / 'sheet.jpg'}  (overhead | wrist_l | wrist_r)")
    for slot, src in ctx["settings"].slots.items():
        print(f"  video.{slot:9s} <- {src}")
    for m in ctx["missing"]:
        print(f"  MISSING: {m}")
    print(
        "state handed to the client (radians and a 0..1 grip; the arm file's rest, NOT read from the arm: the "
        "preview opens no CAN):"
    )
    for k, v in state.items():
        print(f"  {k}: {v}")
    print(
        "as sent by the client (for other_embodiment it zeroes the joint slots and adds each hand's pose in "
        "dream_base, metres):"
    )
    for k, v in values.items():
        print(f"  {k}: {np.round(np.asarray(v, float), 4).tolist() if k.startswith('state.') else v}")
    print(f"\ninputs id: {dream.inputs_id}  (already recorded on this computer: {dream.inputs_checked})")
    if args.fake_cameras:
        print("These are FAKE camera frames: do not record this id.")
    else:
        print(
            "If every frame is upright, on the right key, in natural colours, and the state is in radians, "
            f"Gavin (never an agent) records it:\n  ~/viscous-dream-real/.venv/bin/python -m dream.input_check {dream.inputs_id}"
        )
    return 0


def live_facts(args, ctx, ledger) -> core.LiveFacts:
    arm, urdf = ctx["arm"], ctx["urdf_bytes"]
    dream_tcp = None
    if ctx["dream_problem"] is None:
        from dream import arm_file

        dream_tcp = arm_file.tool_point_problem(arm, urdf)  # Dream's own verdict as well as the mirror
    return core.LiveFacts(
        tcp_problem=core.tool_point_problem(arm, urdf) or dream_tcp,
        approval_problem=core.approval_problem(urdf, ctx["provenance"], arm["base"]),
        unverified_signs=ctx["jmap"].unverified(),
        acked_signs=args.ack_unverified_sign or [],
        plan_id=ctx["plan"]["plan_id"],
        plan_id_given=args.plan_id,
        lab_holder=lab_holder(args.lab_url, args.lab_down_ok),
        port_holders=port_holders(ctx["record"]["follower_port"]),
        missing_slots=ctx["missing"],
        guard_problem=ctx["guard"].problem,
        dream_problem=ctx["dream_problem"],
        ledger_remaining=ledger.remaining(),
        max_calls=args.max_calls,
        arm_file_explicit=args.arm_file is not None,
        credentials_present=bool(os.environ.get("DREAM_API_KEY"))
        or Path("~/.dreamlabs/credentials").expanduser().is_file(),
    )


def run_live(args, ctx) -> int:
    arm_path, arm = ctx["arm_path"], ctx["arm"]
    ledger = core.Ledger(LEDGER, LEDGER_CAP)
    facts = live_facts(args, ctx, ledger)
    if refusals := core.live_refusals(facts):
        print("\n--live REFUSED:")
        for r in refusals:
            print(f"  - {r}")
        return 2
    out = ctx["out"]
    out.mkdir(parents=True, exist_ok=True)
    core.atomic_json(out / "plan.json", ctx["plan"])
    if args.pid_file:
        Path(args.pid_file).write_text(f"{os.getpid()}\n")
    stop_event, abort_event = threading.Event(), threading.Event()

    def on_signal(signum, _frame):
        (abort_event if stop_event.is_set() else stop_event).set()
        print(f"signal {signum}: {'abort the return' if abort_event.is_set() else 'stopping'}", flush=True)

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, on_signal)

    # Preflight, before any CAN traffic: the observation as it would be sent, refused locally.
    dream = DreamReal(arm_path, out, offline=False)
    dream.install_gate("preflight")
    _, grab, close = open_cameras(ctx["record"], fake=False)
    rest = [float(v) for v in arm["rest"]]
    state = core.dream_state(
        {**ctx["jmap"].to_motor(rest), "gripper": core.GRIP_OPEN_DEG},
        ctx["jmap"],
        rest,
        ctx["settings"].prompt,
    )
    try:
        t0 = time.monotonic()
        for i in range(core.FRAMES_IN_WINDOW):
            dream.set_observation(core.slot_frames(grab(), ctx["settings"].slots), state, t0 + i * 0.2)
            time.sleep(0.2)
        with contextlib.suppress(PreflightDoneError):
            dream.request({"q_l_chunk": [*state["state.left_arm"], 1.0], "q_r_chunk": [*rest, 1.0]})
    finally:
        close()
        dream.close()
    if not dream.inputs_checked:
        print(
            f"\n--live REFUSED: inputs id {dream.inputs_id} is not in ~/.dreamlabs/inputs_checked. Run --preview, "
            "look, and Gavin records it with python -m dream.input_check."
        )
        return 2
    print(
        f"inputs id {dream.inputs_id} checked. Energizing next: ALL SEVEN joints may move (plan "
        f"{ctx['plan']['plan_id']}). Stop: Ctrl-C or kill -TERM {os.getpid()}; power switch is the hard stop.",
        flush=True,
    )
    dream = DreamReal(arm_path, out, offline=False)
    dream.install_gate("gate")
    worker = Worker(
        dream, time.monotonic, threaded=True, ledger=ledger, run_id=out.name, max_calls=args.max_calls
    )
    arm_io = LabArm(ctx["home"], ctx["record"], min(30.0, 2 * ctx["settings"].speed_deg_s))
    episode = Episode(
        arm_io,
        dream,
        worker,
        ctx["jmap"],
        ctx["limits"],
        ctx["box"],
        ctx["guard"],
        ctx["settings"],
        time.monotonic,
        time.sleep,
        out,
        stop_event,
        abort_event,
    )
    try:
        summary = episode.run()
    finally:
        dream.close()
    core.atomic_json(out / "summary.json", summary)
    print(json.dumps({k: summary[k] for k in ("reason", "calls", "stop")}, indent=1, default=str))
    return 0 if not summary["trips"] else 1


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = p.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true", help="the default")
    mode.add_argument(
        "--preview", action="store_true", help="no motion: cameras + Dream client offline, print inputs id"
    )
    mode.add_argument("--live", action="store_true", help="billed; refused unless every precondition holds")
    p.add_argument("--robot", default="viscous_waterproof_01")
    p.add_argument(
        "--lab-home", help="MAKERMODSLAB_HOME (default: env, else ~/.makermods/makermodslab-viscous)"
    )
    p.add_argument(
        "--arm-file", type=Path, help="live: the arm file dreamcli tcp wrote (its URDF copy beside it)"
    )
    p.add_argument("--prompt", default="put apple into the bowl")
    p.add_argument("--overhead", help="record camera name for video.overhead (none in the record yet)")
    p.add_argument(
        "--wrist-l", default="wrist", help="record camera name for video.wrist_l (the working arm)"
    )
    p.add_argument(
        "--wrist-r", default="black", help="black | copy | camera:<name> (no second arm on the rig)"
    )
    p.add_argument("--max-calls", type=int, default=10)
    p.add_argument("--rows", type=int, default=48)
    p.add_argument("--speed-deg-s", type=float, default=15.0)
    p.add_argument("--grip-deg-s", type=float, default=60.0)
    p.add_argument("--max-jump-deg", type=float, default=20.0)
    p.add_argument("--limit-margin-deg", type=float, default=1.0)
    p.add_argument("--lag-trip-deg", type=float, default=10.0)
    p.add_argument("--torque-trip-nm", type=float, default=5.0)
    p.add_argument("--torque-cap-nm", type=float, default=3.5)
    p.add_argument("--torque-cap-s", type=float, default=2.0)
    p.add_argument("--temp-trip-c", type=float, default=80.0)
    p.add_argument(
        "--scenario", default="nominal", choices=(*SCENARIOS, "all"), help="dry run: fault to inject"
    )
    p.add_argument("--internal-stub", action="store_true", help="dry run without Dream's client")
    p.add_argument("--start-motor-deg", type=json.loads, help="dry run/preview: override the fake start pose")
    p.add_argument(
        "--fake-cameras", action="store_true", help="preview with synthetic frames (agents use this)"
    )
    p.add_argument("--dream-venv", type=Path, default=Path("~/viscous-dream-real/.venv"))
    p.add_argument("--lab-url", default="http://127.0.0.1:8000")
    p.add_argument("--lab-down-ok", action="store_true")
    p.add_argument("--plan-id", help="live: the plan id the dry run printed for exactly these settings")
    p.add_argument(
        "--ack-unverified-sign",
        action="append",
        metavar="JOINT",
        help="live: acknowledge a joint sign never hand-tested (wrist_yaw = J5, wrist_roll = J6)",
    )
    p.add_argument(
        "--keep-arm-rest",
        action="store_true",
        help="use the arm file's own rest (dreamcli tcp writes zeros: a straight, level arm)",
    )
    p.add_argument("--pid-file")
    p.add_argument("--out", type=Path)
    args = p.parse_args(argv)
    sys.stdout.reconfigure(line_buffering=True)
    if not 1 <= args.max_calls <= 20 or not 1 <= args.rows <= 48:
        p.error("--max-calls 1..20 and --rows 1..48")
    if not 0 < args.speed_deg_s <= 30 or not 0 < args.grip_deg_s <= 90 or not 0 < args.max_jump_deg <= 20:
        p.error("--speed-deg-s (0, 30], --grip-deg-s (0, 90], --max-jump-deg (0, 20]")
    kind = "live" if args.live else "preview" if args.preview else "dry"

    home = lab_home(args.lab_home)
    record = load_record(home, args.robot)
    limits, limits_source = follower_limits(home, record)
    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    out = args.out or (HERE / "runs" / f"{kind}-{stamp}")
    source_arm = (args.arm_file or DEFAULT_ARM_FILE).expanduser().resolve()
    arm_path, rest_note = effective_arm_file(source_arm, args.keep_arm_rest, out)
    arm = json.loads(arm_path.read_text())
    urdf_path = arm_path.parent / arm["urdf"]
    urdf_bytes = urdf_path.read_bytes()
    print(f"arm file: {source_arm}\n  {rest_note}")
    sidecars = [
        source_arm.parent / Path(arm["urdf"]).with_suffix(".provenance.json"),
        HERE / "tank_model/viscous_waterproof_tank.provenance.json",
    ]
    provenance = next((json.loads(s.read_text()) for s in sidecars if s.is_file()), None)
    jmap = core.JointMap.load()
    box = core.WorkspaceBox.load(BOX_FILE, SRC_URDF, jmap, tcp_m=arm["tcp"])
    slots, missing = core.resolve_slots(
        [c["name"] for c in record.get("cameras") or []], args.overhead, args.wrist_l, args.wrist_r
    )
    trips = core.TripLimits(
        lag_deg=args.lag_trip_deg,
        torque_trip_nm=args.torque_trip_nm,
        torque_cap_nm=args.torque_cap_nm,
        torque_cap_s=args.torque_cap_s,
        temp_trip_c=args.temp_trip_c,
    )
    trips.check()
    settings = Settings(
        args.prompt,
        args.rows,
        args.max_calls,
        args.speed_deg_s,
        args.grip_deg_s,
        args.max_jump_deg,
        args.limit_margin_deg,
        trips,
        slots,
    )
    dream_problem = add_dream_venv(args.dream_venv)
    guard = Guard()
    plan = build_plan(
        args,
        record,
        limits,
        limits_source,
        arm_path,
        arm,
        urdf_bytes,
        provenance,
        slots,
        missing,
        jmap,
        settings,
        json.loads(BOX_FILE.read_text()),
        guard,
        dream_problem or f"Dream's client from {args.dream_venv}",
    )
    print_plan(plan)
    ctx = {
        "home": home,
        "record": record,
        "limits": limits,
        "arm_path": arm_path,
        "arm": arm,
        "urdf_bytes": urdf_bytes,
        "provenance": provenance,
        "jmap": jmap,
        "box": box,
        "missing": missing,
        "settings": settings,
        "guard": guard,
        "dream_problem": dream_problem,
        "plan": plan,
        "out": out,
    }
    if kind == "dry":
        print(
            f"lab: {lab_holder(args.lab_url, True) or 'idle'}; follower port holders: "
            f"{port_holders(record['follower_port']) or 'none'}"
        )
        out.mkdir(parents=True, exist_ok=True)
        core.atomic_json(out / "plan.json", plan)
        refusals = core.live_refusals(live_facts(args, ctx, core.Ledger(LEDGER, LEDGER_CAP)))
        print(
            "Live readiness: "
            + ("would proceed to the preflight" if not refusals else "--live would REFUSE:")
        )
        for r in refusals:
            print(f"  - {r}")
        return run_dry(args, ctx)
    if kind == "preview":
        return run_preview(args, ctx)
    return run_live(args, ctx)


if __name__ == "__main__":
    raise SystemExit(main())
