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
  --rehearse With --live: the live hardware path WITHOUT Dream (no client, no billed call, no cameras):
             connect, start gate on the rest, engage, approach the Dream start pose, hold there until a
             stop signal or --hold-s, then the stop path. Alone (or with --dry-run): the same on FakeArm.

Start: the arm lies flat on the tank floor (its rest). After engage it lifts the elbow and goes to the
Dream start pose (core.approach_waypoints) before the first Dream call. Stop, on every exit: the arm
joints' goals re-anchored at their measured pose (no spring-back), gripper torque off at once (it may
drop what it holds), arm joints back along the route to the measured rest from where they are, then all
torque off. stop_button.py is the big red button (SIGTERM to the --pid-file's pid).

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
# Gavin 2026-11-08: "as long as we go slow, must keep the 15deg/s limits". Every arm motion the runner
# commands (Dream substeps, the approach, every return leg) and the bus backstop under it are capped here.
HARD_ARM_SPEED_DEG_S = 15.0
# MIT (kp, kd) overrides for this runner only (the Lab's defaults are untouched). The follower's shoulder/
# elbow (30, 2) are untuned soft placeholders: at the start pose, in water, the shoulder sat 9.5 deg and
# the elbow 5.4 deg below target, the hand ~12 cm under its command, and Dream chased that (live
# 2026-11-08T211253Z). 60/3 halved it (4.7 / 2.9 deg, hand ~7 cm low, rehearsal 211903Z); Gavin then
# chose 90/4. Torque to hold is unchanged.
FOLLOWER_GAINS = {"shoulder_lift": (90.0, 4.0), "elbow_flex": (90.0, 4.0)}
DEFAULT_HOLD_S = 120.0  # --rehearse: how long to hold at the Dream start pose before stopping by itself
SCENARIOS = (
    "nominal",
    "jump",
    "workspace",
    "lag",
    "torque",
    "sustained",
    "temperature",
    "stale",
    "signal",
    "overspeed",
    "push",
    "far",
)
# The shoulder's standing error holding the Dream start pose (powered rehearsal, 2026-11-08).
START_POSE_SAG = {"shoulder_lift": 7.45}
# The stop path re-anchors each arm joint's goal at its measured pose plus at most this much standing
# error (core.HoldError); can_speed_limit.rebase refuses more.
REANCHOR_MAX_OFFSET_DEG = 10.0
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
    """Tracks targets with a first-order lag; scenarios inject faults. No hardware.

    `overspeed`: shoulder_lift loses its hold (the 2026 dry-shoulder failure) and falls at 45 deg/s, in
    the loop and on through the return. `push`: a visitor pushes shoulder_pan away at 18 deg/s (under
    the speed trip) until the lag trip (or the tank guard); the stop's re-anchor is where they let go.
    `lag`: elbow_flex droops at 12 deg/s (under the speed trip). `far` (with sag=START_POSE_SAG): the arm
    settles short of its goals as the real one does, and the run stops far from the start pose. A return leg that starts with a joint's
    goal more than 5 deg (half the lag trip) from where it is records a SPRING-BACK (a real arm would
    snap toward it).
    """

    FALL_DEG_PER_TICK = 2.25  # 45 deg/s at 20 Hz
    PUSH_DEG_PER_TICK = 0.9  # 18 deg/s

    # A real MIT-held joint settles short of its goal by its standing error: `sag` (deg, per arm joint)
    # is that error, e.g. {"shoulder_lift": 7.45} as measured at the start pose on 2026-11-08.
    def __init__(self, start, cameras, clock, scenario="nominal", sag=None):
        self.pos = dict(start)
        self.target = dict(start)
        self.sag = dict.fromkeys(core.MOTORS, 0.0) | dict(sag or {})
        self.home = dict(start)  # lying on the tank floor, supported: no sag there
        self.cameras, self.clock, self.scenario = list(cameras), clock, scenario
        self.ticks = 0
        self.events = []
        self.energized = False
        self.gripper_limp = False
        self.limp = set()  # arm joints that lost their hold
        self.pushing = scenario == "push"
        self.stopping = False  # from the re-anchor on, injected faults stop (a lost hold does not)
        self.returns = []  # (pose, speed) per return leg
        self.leg_starts = []  # the measured pose each return leg started from

    def connect(self):
        self.events.append("connect (fake: no port opened)")

    def _pushed(self):
        return self.pushing and self.ticks > 70

    def _drooping(self):
        return self.scenario == "lag" and self.ticks > 70 and not self.stopping

    def _settle(self, m, goal):
        """Where a joint held at `goal` comes to rest: short by its sag, phased in over the first 20 deg
        away from the floor rest (lying there it is supported)."""
        return goal - self.sag[m] * min(1.0, abs(goal - self.home[m]) / 20.0)

    def read(self) -> Reading:
        self.ticks += 1
        if self.scenario == "stale" and self.ticks > 70:
            raise ConnectionError("No fresh feedback from motor 'elbow_flex' within 50 ms (fake)")
        if self.energized:
            if self.scenario == "overspeed" and self.ticks > 70:
                self.limp.add("shoulder_lift")
            for m in core.ARM if self.gripper_limp else core.MOTORS:
                if (
                    m in self.limp
                    or (m == "shoulder_pan" and self._pushed())
                    or (m == "elbow_flex" and self._drooping())
                ):
                    continue
                self.pos[m] += 0.6 * (self._settle(m, self.target[m]) - self.pos[m])
            if "shoulder_lift" in self.limp:
                self.pos["shoulder_lift"] -= self.FALL_DEG_PER_TICK
            if self._pushed():
                self.pos["shoulder_pan"] -= self.PUSH_DEG_PER_TICK  # the visitor overpowers the joint
            if self._drooping():
                self.pos["elbow_flex"] -= 0.6  # cannot follow (weak or blocked): droops at 12 deg/s
        torque = {m: 0.4 + 0.1 * math.sin(self.ticks / 7 + i) for i, m in enumerate(core.MOTORS)}
        temp = dict.fromkeys(core.MOTORS, 31.0 + 0.002 * self.ticks)
        if self.scenario == "torque" and self.ticks > 70:
            torque["shoulder_lift"] = 12.8  # over the 12 Nm instant tier
        if self.scenario == "sustained" and self.ticks > 70:
            torque["shoulder_lift"] = 5.5  # over the 5 Nm sustained tier, for 5 s
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

    def release_gripper(self):
        self.gripper_limp = True
        self.events.append(
            "release gripper: zero-effort MIT frame, then torque off on the gripper motor only"
        )
        return None

    def reanchor(self, offsets):
        """can_speed_limit.rebase on the fake: the goal becomes the measured pose plus the offset, at once."""
        if any(abs(offsets[m]) > REANCHOR_MAX_OFFSET_DEG for m in core.ARM):
            raise ValueError("re-anchor offset too large")
        self.pushing = False  # the visitor lets go: nothing is left to pull the arm back
        self.stopping = True
        moved = max(abs(self.target[m] - self.pos[m] - offsets[m]) for m in core.ARM)
        for m in core.ARM:
            self.target[m] = self.pos[m] + offsets[m]
            if self.sag[m]:  # where the joint settles under that goal: still where it is
                self.pos[m] = self._settle(m, self.target[m])
        self.events.append(
            f"re-anchor: arm goals at the measured pose (moved {moved:.1f} deg from the old goals)"
        )
        return {m: self.pos[m] for m in core.ARM}

    def return_to_rest(self, pose, abort, speed, watch=None):
        """A straight ramp from the MEASURED pose at `speed`, 20 Hz, each step's pose through `watch`
        (the stop path's speed trip): a trip ends the leg where it is. Limp joints keep falling."""
        if "gripper" in pose:
            raise AssertionError("the return must leave the (limp) gripper alone")
        start = {m: self.pos[m] for m in pose}
        self.leg_starts.append(dict(start))
        if spring := [m for m in pose if abs(self._settle(m, self.target[m]) - self.pos[m]) > 5.0]:
            self.events.append(f"SPRING-BACK: {', '.join(spring)} goal far from where the arm is")
        travel = max(abs(start[m] - pose[m]) for m in pose)
        self.returns.append((dict(pose), speed))
        steps = max(1, math.ceil(travel / (speed * core.ROW_DT_S)))
        for i in range(1, steps + 1):
            if hasattr(self.clock, "sleep"):
                self.clock.sleep(core.ROW_DT_S)
            for m, v in pose.items():
                if m in self.limp:
                    self.pos[m] -= self.FALL_DEG_PER_TICK
                else:
                    self.target[m] = start[m] + (v - start[m]) * i / steps
                    self.pos[m] = start[m] + (self._settle(m, v) - start[m]) * i / steps
            if watch is not None and (trips := watch(dict(self.pos))):
                self.events.append(f"return leg: ended by {trips[0].name} after {i}/{steps} steps")
                return False, f"{trips[0].name}: {trips[0].detail}"
        self.events.append(
            f"return leg: {travel:.1f} deg max travel at {speed:.0f} deg/s, gripper limp: {self.gripper_limp}"
        )
        # The Lab helper's verdict (maker_rest_pose): within 2 deg arrived, within 6 settled, else short.
        motor, off = max(((m, abs(self.pos[m] - v)) for m, v in pose.items()), key=lambda kv: kv[1])
        if off <= 2.0:
            return True, ""
        if off <= 6.0:
            return True, "settled"
        return False, f"{motor} still {off:.1f} deg away"

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
    which is also the recovery when connect fails part-way. release_gripper
    disables the ONE gripper motor at the start of every stop (see there).
    cameras=False (--rehearse) opens the follower without the record's cameras.
    """

    def __init__(self, home: Path, record: dict, backstop_deg_s: float, cameras: bool = True):
        if not 0 < backstop_deg_s <= HARD_ARM_SPEED_DEG_S:
            raise RefusedError(f"bus backstop {backstop_deg_s} deg/s is above the {HARD_ARM_SPEED_DEG_S} cap")
        self.home, self.record, self.backstop = home, record, backstop_deg_s
        self.use_cameras = cameras
        self.robot = None
        self.gripper_released = False

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
        for entry in (self.record.get("cameras") or []) if self.use_cameras else []:
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
        config.gains = {**config.gains, **FOLLOWER_GAINS}
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
        names = [c["name"] for c in self.record.get("cameras") or []] if self.use_cameras else []
        frames = {n: obs[n] for n in names}
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

        # MakerFollower.send_action writes only the motors in the action, so a released
        # gripper gets no further MIT frames from here.
        motors = core.ARM if self.gripper_released else core.MOTORS
        remote_can.send_action(
            self.robot,
            {f"{m}.pos": target[m] for m in motors},
            {f"{m}.pos": v for m, v in previous.items()},
        )

    def release_gripper(self) -> str | None:
        """Gripper motor only, at once (Gavin: it may drop what it holds). First a zero-effort MIT frame
        (Kp = Kd = 0 cached for the gripper, as remote_can.neutralize does for all seven), so the motor
        makes no torque even if the disable frame is lost; then RobstrideMotorsBus.disable_torque("gripper")
        (`_disable_motor`: the MIT-protocol disable frame, FF x7 + CAN_CMD_DISABLE, to that one CAN id).
        The gains stay zero in the bus cache, so no later frame can stiffen it; release() restores the
        cache. The other six keep holding their setpoints. Never raises: the return follows regardless."""
        problems = []
        bus = self.robot.bus
        try:
            bus.sync_write("Kp", {"gripper": 0.0})
            bus.sync_write("Kd", {"gripper": 0.0})
            bus.sync_write("Goal_Position", {"gripper": 0.0})  # zero gains: a no-torque frame
        except Exception as error:  # noqa: BLE001
            problems.append(f"zero-effort gripper frame failed: {error}")
        try:
            bus.disable_torque("gripper")
            self.gripper_released = True
        except Exception as error:  # noqa: BLE001
            problems.append(f"gripper disable failed: {error}")
        return "; ".join(problems) or None

    def reanchor(self, offsets) -> dict:
        """The six arm joints' goals to their fresh measured pose plus `offsets` (each joint's standing
        error, core.HoldError), written at once: can_speed_limit.rebase reads the motors itself, refuses
        an offset over REANCHOR_MAX_OFFSET_DEG, and rebases the bus backstop's last-sent target so the
        return slews from here, never from the old goal. Returns the measured arm pose."""
        from makermodslab import can_speed_limit

        return can_speed_limit.rebase(
            self.robot, {m: offsets[m] for m in core.ARM}, max_offset_deg=REANCHOR_MAX_OFFSET_DEG
        )

    def return_to_rest(self, pose, abort, speed, watch=None):
        """maker_rest_pose.return_maker_to_pose on fresh CAN feedback. It plans each leg from its own
        fresh reading (the measured pose, not the cached command). The adapter also reads the arm before
        every setpoint and hands it to `watch` (the stop path's measured-speed trip): a trip raises, which
        the helper (never-raises) turns into a leg that did not arrive, so the stop releases there. A
        missed reply before a setpoint is skipped, not raised: ending the ramp on one lost frame would
        release torque mid-air."""
        from makermodslab import remote_can
        from makermodslab.maker_rest_pose import MAKER_RETURN_CEILING_S, return_maker_to_pose

        robot = self.robot
        speed = min(speed, HARD_ARM_SPEED_DEG_S)  # never the helper's 30 deg/s default
        tripped = []

        class Fresh:  # the rest-return helper on fresh CAN feedback (remote_can's pattern)
            def _read(self):
                if tripped:
                    raise TripStopError(tripped)
                obs = remote_can.get_observation(robot)
                if watch is not None and (trips := watch({m: float(obs[f"{m}.pos"]) for m in core.ARM})):
                    tripped.extend(trips)
                    raise TripStopError(trips)
                return obs

            def get_observation(self):
                for attempt in range(3):  # the helper's first read plans the leg: a miss there releases
                    try:
                        return self._read()
                    except TripStopError:
                        raise
                    except Exception:  # noqa: BLE001
                        if attempt == 2:
                            raise
                return None

            def send_action(self, action):
                try:
                    self._read()
                except TripStopError:
                    raise
                except Exception:  # noqa: BLE001 - a missed reply: the next setpoint reads again
                    pass
                return robot.send_action(action)

        spans = [hi - lo for lo, hi in robot.config.joint_limits.values()]
        # Doubled: every ramp step now reads seven motors as well, and a ramp cut by the ceiling ends
        # the return (torque off) wherever it is. The second signal still ends it at once.
        return return_maker_to_pose(
            Fresh(),
            pose,
            abort_event=abort,
            speed_deg_s=speed,
            ceiling_s=max(MAKER_RETURN_CEILING_S, 2 * max(spans) / speed + 3),
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


def make_floor_slide(arm: dict) -> tuple[core.FloorSlide | None, str | None]:
    """The floor slide for this arm file's tool point, on the tank guard's own mesh and floor line."""
    try:
        from makermodslab import tank_guard

        model = tank_guard.TankGuard.from_file().model
    except Exception as error:  # noqa: BLE001 - the plan reports it; the guard's own absence refuses --live
        return None, f"no tank model ({error.__class__.__name__}: {error})"
    if arm.get("flange") != "wrist_roll_link":
        return None, f"flange {arm.get('flange')!r} is not the tank model's wrist_roll_link"
    return (
        core.FloorSlide(
            core.HandMesh.from_model(model),
            core.tool_rotation(arm["approach"], arm["jaw"]),
            np.asarray(arm["tcp"], float),
            core.keepout_floor_mm(),
        ),
        None,
    )


class SlideTransport:
    """Wraps the client's transport: BEFORE the client's IK, clips Dream's left hand targets into the tcp box
    (core.clamp_eef_rows), then raises any whose hand would come lower than the floor slide allows
    (core.FloorSlide), so the hand slides along the limits instead of the chunk being cut there."""

    KEY = "action.left_eef_xyz"
    ROT = "action.left_eef_rot6"
    GRIP = "action.left_grip"

    def __init__(self, real, box_mm, floor: core.FloorSlide | None = None):
        self.real, self.box_mm, self.floor = real, box_mm, floor
        self.last_exchange = getattr(real, "last_exchange", None)
        self.clamped_rows = []  # per reply: rows whose hand target was moved onto a limit
        self.lifts_mm = []  # per reply: each row's floor lift (mm), [] without a floor slide

    def request(self, request, headers, **kwargs):
        reply = self.real.request(request, headers, **kwargs)
        per_key = (reply or {}).get("actions", {}).get("per_key") if isinstance(reply, dict) else None
        if per_key and self.KEY in per_key:
            rows, changed = core.clamp_eef_rows(per_key[self.KEY], self.box_mm)
            lift = np.zeros(len(rows))
            if self.floor is not None:
                rows, lift = self.floor.lift_rows(rows, per_key[self.ROT], per_key[self.GRIP])
            per_key[self.KEY] = rows.tolist()
            self.clamped_rows.append(changed)
            self.lifts_mm.append(lift.tolist())
        return reply

    def __getattr__(self, name):
        return getattr(self.real, name)

    def close(self):
        self.real.close()


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
        floor, self.floor_problem = make_floor_slide(json.loads(Path(arm_file).read_text()))
        self.slide = SlideTransport(self.client._policy._transport, core.tcp_box_mm(BOX_FILE), floor)
        self.client._policy._transport = self.slide
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
    hold_s: float = DEFAULT_HOLD_S
    pause_s: float = 0.0  # hold at the start pose before the first Dream call (Gavin places the object)
    max_stuck_calls: int = 3  # stop after this many stuck calls in a row (each one is billed)
    # A call is stuck when it executes at most this many substeps: in live 215535Z every call was cut at row 0
    # after ONE substep, so a count of empty calls only never fired and 19 billed calls went by in ~12 s.
    stuck_substeps: int = 2
    # What Dream is told the arm is at (see Episode.dream_pose): "commanded" (the last target, gripper as
    # measured) or "measured" (the reading, as before 2026-11-08).
    dream_state: str = "commanded"


class Episode:
    """The control loop. Phases: connect, start gate, engage, approach (rest -> Dream start pose), then
    dream (Dream chunks) or, with dream=None (--rehearse), hold at the start pose until a stop signal or
    hold_s. Every exit goes through stop(): gripper torque off, return legs to the rest, all torque off."""

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
        arm_rest=None,
        say=None,
    ):
        if not 0 < settings.speed_deg_s <= HARD_ARM_SPEED_DEG_S:
            raise RefusedError(
                f"arm speed {settings.speed_deg_s} deg/s is above the hard {HARD_ARM_SPEED_DEG_S} deg/s cap"
            )
        self.arm, self.dream, self.worker, self.jmap, self.limits = arm, dream, worker, jmap, limits
        self.box, self.guard, self.s, self.clock, self.sleep = box, guard, settings, clock, sleep
        self.log_dir, self.stop_event, self.abort_event, self.mutate = (
            log_dir,
            stop_event,
            abort_event,
            mutate,
        )
        self.say = say
        self.monitor = core.TripMonitor(settings.trips, clock)
        self.hold_error = core.HoldError(limit_deg=min(settings.trips.lag_deg, REANCHOR_MAX_OFFSET_DEG))
        self.measured = None  # the last fresh reading's positions
        if arm_rest is None:
            arm_file = (
                json.loads(Path(dream.config["ik"]["arm"]).read_text()) if hasattr(dream, "config") else None
            )
            arm_rest = arm_file["rest"] if arm_file else json.loads(DEFAULT_ARM_FILE.read_text())["rest"]
        # The arm file's rest (IK radians, the tank model's): the parked right arm reported to Dream, and,
        # through the joint map, the pose the approach brings the working arm to before the first call.
        self.right_rest = [float(v) for v in arm_rest]
        self.summary = {
            "trips": [],
            "calls": 0,
            "rows_planned": 0,
            "approach_substeps": 0,
            "substeps_sent": 0,
            "clipped": 0,
            "guard": collections.Counter(),
            "stretched_rows": 0,
            "phases": [],
        }
        self.energized = False
        self.phase = "connect"
        self.start_pose = None  # the measured rest, lying on the tank floor
        self.dream_start = None  # the arm file's rest in motor degrees, gripper as measured
        self.trail = []  # approach leg ends reached, from the rest: the way back from a stop mid-approach
        self.commanded = None
        self.planned = None  # the last planned substep: the hard speed cap is checked against it
        self.t0 = clock()
        self.ticks = log_dir / "ticks.jsonl"
        self.chunks = log_dir / "chunks.jsonl"

    def _write(self, path, obj):
        with path.open("a") as f:
            f.write(json.dumps(obj, default=float) + "\n")

    def _phase(self, name, detail=""):
        self.summary["phases"].append(name)
        if self.say:
            self.say(f"[{self.clock() - self.t0:6.1f} s] phase: {name}{f' - {detail}' if detail else ''}")

    def dream_pose(self, measured) -> dict:
        """The pose Dream is told the arm is at (its state, and the row the call carries). With "commanded":
        the six arm joints at the last commanded target, the gripper as measured. The arm sags under its
        command (the hand ~5 cm low at the start pose with 90/4 gains), and told the measured joints Dream
        "corrected" that sag with its first move every run, lifting the hand and folding the arm back toward
        the back wall (live 215535Z); the gripper is measured so Dream sees a grasp that blocks the jaws.
        Only what Dream is told: every trip, the lag, the guard monitor, the start gate and the stop keep
        using the measured pose."""
        if self.s.dream_state == "measured":
            return dict(measured)
        return {**{m: float(self.commanded[m]) for m in core.ARM}, "gripper": float(measured["gripper"])}

    def start_target(self, rest) -> dict:
        return {**self.jmap.to_motor(self.right_rest), "gripper": float(rest["gripper"])}

    def start_gate(self, rest, start) -> list[str]:
        """The measured rest: limits (with the resting tolerance) and the tank guard. The workspace box
        does not apply to it (lying flat, its tcp can sit below the box floor); it applies to the Dream
        start pose (and every Dream substep), which must also be strictly inside the limits."""
        problems = []
        for m in core.MOTORS:
            lo, hi = self.limits[m]
            if not lo - START_LIMIT_TOLERANCE_DEG <= rest[m] <= hi + START_LIMIT_TOLERANCE_DEG:
                problems.append(
                    f"{m} at {rest[m]:.1f} is more than {START_LIMIT_TOLERANCE_DEG} deg outside "
                    f"[{lo:.1f}, {hi:.1f}]"
                )
        if g := self.guard.start(rest):
            problems.append(f"tank guard: {g}")
        for m in core.ARM:
            lo, hi = self.limits[m]
            if not lo <= start[m] <= hi:
                problems.append(f"Dream start pose: {m} at {start[m]:.1f} is outside [{lo:.1f}, {hi:.1f}]")
        problems += [f"Dream start pose, workspace box: {v}" for v in self.box.violations(start)]
        return problems

    def plan_approach(self, rest, start) -> list[list[dict]]:
        """Rest -> core.approach_waypoints legs, each retimed at the arm speed cap and every substep
        through the tank guard (rejected -> stop), all planned before the arm is energized. A substep
        the guard clips (a rest outside a limit) is re-retimed from the last one, so no planned substep
        moves an arm joint faster than the cap; the clipped segment is the one the guard checked."""
        legs, prev = [], dict(rest)
        for k, waypoint in enumerate(core.approach_waypoints(rest, start), 1):
            steps = []
            for step in core.retime(
                prev, waypoint, arm_deg_s=self.s.speed_deg_s, grip_deg_s=self.s.grip_deg_s
            ):
                status, granted, reason = self.guard.step(step)
                self.summary["guard"][status] += 1
                if status == "rejected":
                    raise TripStopError([core.Trip("tank_guard", f"approach leg {k}: {reason}")])
                steps += core.retime(
                    prev, granted, arm_deg_s=self.s.speed_deg_s, grip_deg_s=self.s.grip_deg_s
                )
                prev = granted
            legs.append(steps)
        return legs

    def plan_chunk(self, reply) -> list[dict]:
        """Dream's rows -> 20 Hz substeps at <= 15 deg/s, every substep through the workspace box and the
        tank guard. A row that cannot be reached (box, or the keep-out cut it) is SKIPPED and the arm heads
        for Dream's later rows from where it is, instead of the chunk being dropped there: Dream's chunks often
        open by continuing a motion into a limit and only then turn away (live 2026-11-08T223929Z: rows 0-10
        folded the forearm 29 mm past the back wall, rows 11-47 reached out to the ball). Gavin: follow Dream as
        far as possible, "it shouldn't just reject or give up". Every sent substep is still guard-proven and
        speed-capped; IK flips are judged row to row on Dream's own chunk. Jumps, bad rows and other guard
        refusals still stop the run."""
        rows = np.array(reply["q_l_chunk"], float)[: self.s.rows]
        rows[:, 6] = np.clip(rows[:, 6], 0.0, 1.0)
        if self.mutate:
            rows = self.mutate(rows)
        out, prev, ref, skipped = [], dict(self.commanded), dict(self.commanded), []
        for i, row in enumerate(rows):
            if not np.isfinite(row).all() or row.shape != (7,):
                raise TripStopError([core.Trip("bad_chunk", f"row {i} is {row.tolist()}")])
            target = core.ik_row_to_motor(row, self.jmap)
            if bad := core.jumps(ref, target, self.s.max_jump_deg):
                raise TripStopError(
                    [
                        core.Trip(
                            "jump",
                            f"row {i}: {', '.join(f'{m} {target[m] - ref[m]:+.1f} deg' for m in bad)}",
                        )
                    ]
                )
            ref = dict(target)
            target, clipped = core.clamp_to_limits(target, self.limits, self.s.limit_margin_deg)
            self.summary["clipped"] += bool(clipped)
            steps = core.retime(prev, target, arm_deg_s=self.s.speed_deg_s, grip_deg_s=self.s.grip_deg_s)
            self.summary["stretched_rows"] += len(steps) > 1
            why = None
            for step in steps:
                if bad := self.box.violations(step):
                    why = f"workspace {bad[0]}"
                    break
                status, granted, reason = self.guard.step(step)
                self.summary["guard"][status] += 1
                if status == "rejected":
                    if reason == "keepout":
                        why = "tank keep-out"
                        break
                    raise TripStopError([core.Trip("tank_guard", f"row {i}: {reason}")])
                # A guard clamp ends short of the line; re-retime to it so the next substep stays under
                # the cap (just `granted` when nothing was clamped).
                subs = core.retime(prev, granted, arm_deg_s=self.s.speed_deg_s, grip_deg_s=self.s.grip_deg_s)
                cut = next(
                    (k for k, sub in enumerate(subs if granted != step else ()) if self.box.violations(sub)),
                    None,
                )
                if cut is not None:
                    out += subs[:cut]
                    prev = subs[cut - 1] if cut else prev
                    why = "workspace (after a clamp)"
                    break
                out += subs
                prev = granted
                if status == "clamped" and reason == "keepout":
                    why = "tank keep-out (cut at the line)"
                    break
            if why:
                skipped.append((i, why))
        self.summary["rows_planned"] += len(rows)
        if skipped:
            first, why = skipped[0]
            note = f"{len(skipped)} of {len(rows)} rows skipped (first: row {first}, {why}), {len(out)} substeps kept"
            self.summary.setdefault("overrides", []).append({"call": self.summary["calls"], "why": note})
            self._phase("override", f"call {self.summary['calls']}: {note}")
        return out

    def _check_speed(self, target):
        """The hard cap on what is about to be sent, whatever planned it (a clamp is on the guard's line)."""
        cap = HARD_ARM_SPEED_DEG_S * core.ROW_DT_S + 1e-6
        if fast := [m for m in core.ARM if abs(target[m] - self.planned[m]) > cap]:
            raise TripStopError(
                [core.Trip("speed", f"{', '.join(fast)} would move faster than {HARD_ARM_SPEED_DEG_S} deg/s")]
            )
        self.planned = dict(target)

    def run(self) -> dict:
        frames_fed, queue, approach = 0, collections.deque(), collections.deque()
        reason = "finished"
        hold_until = None
        dream_after = 0.0
        stuck = 0
        self._phase("connect")
        self.arm.connect()
        try:
            reading = self.arm.read()
            self.start_pose = dict(reading.pos)
            self.dream_start = self.start_target(self.start_pose)
            self._phase("start gate", "measured rest (limits + tank guard) and the Dream start pose (+ box)")
            if problems := self.start_gate(self.start_pose, self.dream_start):
                measured = ", ".join(f"{m} {v:.1f}" for m, v in self.start_pose.items())
                raise RefusedError(f"start pose refused (measured {measured}): " + "; ".join(problems))
            legs = self.plan_approach(self.start_pose, self.dream_start)
            for k, steps in enumerate(legs, 1):
                approach.extend((k, i == len(steps) - 1, step) for i, step in enumerate(steps))
            if self.stop_event.is_set():
                raise RefusedError("stop signal before energizing")
            self._phase("engage", "torque on, holding the measured rest")
            self.commanded = self.arm.engage()
            self.energized = True
            self.planned = dict(self.start_pose)  # the approach was planned from the gate's reading
            self.trail = [dict(self.start_pose)]
            self.phase = "approach"
            self._phase(
                "approach",
                f"{len(legs)} legs, {len(approach)} substeps, {len(approach) * core.ROW_DT_S:.1f} s at <= "
                f"{self.s.speed_deg_s:g} deg/s to the Dream start pose",
            )
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
                if not any(t.name == "bad_reading" for t in trips):
                    self.measured = dict(reading.pos)
                if not trips:  # the tick that trips (a push, a drop) must not teach the stop its error
                    self.hold_error.update(reading.pos, self.commanded, now)
                self._write(
                    self.ticks,
                    {
                        "t": now,
                        "phase": self.phase,
                        "pos": reading.pos,
                        "cmd": self.commanded,
                        "torque": reading.torque,
                        "temp": reading.temp,
                        "trips": [t.name for t in trips],
                    },
                )
                if trips:
                    raise TripStopError(trips)
                if self.dream is not None and now >= next_frame:
                    frames = core.slot_frames(reading.frames, self.s.slots)
                    state = core.dream_state(
                        self.dream_pose(reading.pos), self.jmap, self.right_rest, self.s.prompt
                    )
                    self.dream.set_observation(frames, state, now)
                    frames_fed += 1
                    next_frame += core.FRAME_PERIOD_S
                if self.stop_event.is_set():
                    reason = "stop signal"
                    break
                if self.phase == "hold" and now >= hold_until:
                    reason = f"held {self.s.hold_s:g} s at the Dream start pose"
                    break
                previous = self.commanded
                if approach:
                    _leg, leg_end, self.commanded = approach.popleft()
                    self._check_speed(self.commanded)
                    self.summary["approach_substeps"] += 1
                    if leg_end:
                        self.trail.append(dict(self.commanded))
                elif queue:
                    self.commanded = queue.popleft()
                    self._check_speed(self.commanded)
                    self.summary["substeps_sent"] += 1
                self.arm.send(self.commanded, previous)  # an empty queue re-sends the hold target
                if self.phase == "approach" and not approach:
                    if self.dream is None:
                        self.phase, hold_until = "hold", now + self.s.hold_s
                        self._phase(
                            "hold", f"at the Dream start pose until a stop signal or {self.s.hold_s:g} s"
                        )
                    else:
                        self.phase, dream_after = "dream", now + self.s.pause_s
                        if self.s.pause_s > 0:
                            self._phase(
                                "pause", f"holding at the start pose {self.s.pause_s:g} s before Dream"
                            )
                        self._phase("dream", "start pose reached; Dream calls from here")
                if (
                    self.phase == "dream"
                    and not queue
                    and not self.worker.busy
                    and frames_fed >= core.FRAMES_IN_WINDOW
                    and now >= dream_after
                ):
                    if self.worker.calls >= self.s.max_calls:
                        reason = f"call cap {self.s.max_calls} reached"
                        break
                    # The client reads only the arm joints of this row (the IK seed and the seam join), and
                    # they are the command in either mode; with "commanded" the grip is Dream's state's too.
                    told = (
                        self.commanded if self.s.dream_state == "measured" else self.dream_pose(reading.pos)
                    )
                    row = core.motor_to_ik_row(told, self.jmap)
                    self.worker.start({"q_l_chunk": row, "q_r_chunk": [*self.right_rest, 1.0]})
                result = self.worker.take() if self.phase == "dream" else None
                if result is not None:
                    reply, error, request = result
                    if error:
                        raise TripStopError([core.Trip("dream", error)])
                    self.summary["calls"] += 1
                    slide = getattr(self.dream, "slide", None)
                    slid = slide.clamped_rows[-1] if slide is not None and slide.clamped_rows else 0
                    lifts = slide.lifts_mm[-1] if slide is not None and slide.lifts_mm else []
                    lifted = sum(v > 0 for v in lifts)
                    if slid or lifted:
                        self.summary.setdefault("slid", []).append(
                            {
                                "call": self.summary["calls"],
                                "rows": slid,
                                "floor_rows": lifted,
                                "max_floor_lift_mm": round(max(lifts, default=0.0), 1),
                            }
                        )
                        self._phase(
                            "slide",
                            f"call {self.summary['calls']}: {slid} hand targets moved onto the box, {lifted} "
                            f"raised off the floor (up to {max(lifts, default=0.0):.0f} mm)",
                        )
                    try:
                        plan = self.plan_chunk(reply)
                    except TripStopError as trip:
                        self._write(
                            self.chunks,
                            {
                                "call": self.summary["calls"],
                                "ledger": request,
                                "q_l_chunk": np.asarray(reply["q_l_chunk"]).tolist(),
                                "refused": str(trip),
                            },
                        )
                        raise
                    self._write(
                        self.chunks,
                        {
                            "call": self.summary["calls"],
                            "ledger": request,
                            "q_l_chunk": np.asarray(reply["q_l_chunk"]).tolist(),
                            "per_key": {
                                k: np.asarray(v).tolist() for k, v in (reply.get("per_key") or {}).items()
                            },
                            "floor_lift_mm": [round(v, 1) for v in lifts],
                            "substeps": plan,
                        },
                    )
                    queue.extend(plan)
                    stuck = 0 if len(plan) > self.s.stuck_substeps else stuck + 1
                    if stuck >= self.s.max_stuck_calls:
                        reason = (
                            f"stuck: {stuck} calls in a row executed <= {self.s.stuck_substeps} substeps "
                            "(each is billed)"
                        )
                        break
                next_tick += core.ROW_DT_S
                # Never catch up: after a slow tick (planning a chunk takes up to ~160 ms) the loop would send
                # the queued substeps back to back, faster than the 15 deg/s the plan was retimed for. Restart
                # the 20 Hz clock instead, so every substep keeps its 50 ms.
                next_tick = max(next_tick, self.clock())
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
            peak_speed_deg_s=self.monitor.speed.peak,
            guard=dict(self.summary["guard"]),
        )
        return self.summary

    def return_legs(self) -> list[dict]:
        """Where the stop goes, in order, ending at the measured rest. Mid-approach: back along the
        approach legs travelled, in reverse. Past it: straight to the Dream start pose, then the
        approach route reversed (core.return_waypoints)."""
        if self.phase == "approach":
            return list(reversed(self.trail))
        return [self.dream_start, *core.return_waypoints(self.start_pose, self.dream_start)]

    def legs_from(self, here) -> list[dict]:
        """return_legs() from the MEASURED pose `here`. An arm holding a leg end sits off it by its
        standing sag (7.4 deg on the shoulder at the start pose), and a leg back up to it would stop
        short of it (the return helper settles within 6 deg) and release there, so within the lag trip
        of the first leg end counts as at it. The last leg (to the rest) always runs."""
        legs = self.return_legs()
        if len(legs) > 1 and max(abs(legs[0][m] - here[m]) for m in core.ARM) <= self.s.trips.lag_deg:
            legs = legs[1:]
        return legs

    def reanchor(self, out) -> dict:
        """Before any return motion: every arm joint's goal to its fresh measured pose plus its standing
        error (HoldError), written at once and rebased in the bus backstop. Whatever tripped, a pushed or
        blocked arm is then held where it is, never pulled back by the old goal's error. Returns the GOAL
        the motors now hold (measured + offset): the guard judges targets (the measured pose, which sags
        below its goal, is policed by monitor() with its tracking allowance), so the return's path checks
        and leg choice start from it. On failure the old goal is still held: the command."""
        offsets = self.hold_error.value()
        self._phase("stop: re-anchor", "arm goals to the measured pose (+ standing error), no slew")
        try:
            measured = self.arm.reanchor(offsets)
            goal = {m: float(measured[m]) + offsets[m] for m in core.ARM}
            out["reanchor"] = {
                "measured": {m: round(measured[m], 2) for m in core.ARM},
                "offset_deg": {m: round(offsets[m], 2) for m in core.ARM},
                "goal": {m: round(goal[m], 2) for m in core.ARM},
            }
            return goal
        except Exception as error:  # noqa: BLE001 - the return and the release follow regardless
            out["reanchor"] = f"{error.__class__.__name__}: {error}"
            return {m: float(self.commanded[m]) for m in core.ARM}

    def stop(self, reason) -> dict:
        """Every exit: (a) re-anchor the six arm joints at their measured pose (no spring-back), (b)
        gripper torque off at once, (c) the arm joints back to the measured rest leg by leg, from the
        measured pose, at the speed cap on fresh feedback, each leg's straight path checked by the guard
        and every reading through the measured-speed trip; a leg that stops short within the lag trip
        counts as arrived, (d) all torque off, at the rest. NEVER a mid-air release: a path not clear, an
        overspeed, no fresh reading or a leg ending beyond the lag trip HOLDS (re-anchored, torque on)
        until the second signal. A second signal aborts the return (or the hold) and releases."""
        out = {
            "reason": reason,
            "reanchor": None,
            "gripper_release": None,
            "legs": [],
            "returned": None,
            "release_problems": None,
        }
        try:
            if self.energized:
                here = self.reanchor(out)  # the goal now held: path checks and the leg choice start here
                self._phase("stop: gripper torque off")
                try:
                    out["gripper_release"] = self.arm.release_gripper() or "ok"
                except Exception as error:  # noqa: BLE001 - the return and the release follow regardless
                    out["gripper_release"] = f"{error.__class__.__name__}: {error}"
                grip = self.commanded["gripper"]  # where the limp gripper was last held, for the guard
                here = {**here, "gripper": grip}
                legs = self.legs_from(here)
                # A fresh speed monitor: motion before the re-anchor must not end the return.
                lim = self.s.trips
                speed = core.SpeedMonitor(lim.overspeed_deg_s, lim.overspeed_ticks)
                return_trips = []

                def watch(pos):
                    trips = speed.update(pos, self.clock())
                    return_trips.extend(trips)
                    return trips

                out["returned"] = "at the rest"
                for k, leg in enumerate(legs, 1):
                    target = {**{m: float(leg[m]) for m in core.ARM}, "gripper": grip}
                    last = k == len(legs)
                    if self.abort_event.is_set():
                        out["returned"] = f"aborted by the second signal before leg {k}"
                        break
                    if not self.guard.path_clear(here, target):
                        self._hold_mid_air(out, f"before leg {k}: its path is not clear of the tank keep-out")
                        break
                    travel = max(abs(target[m] - here[m]) for m in core.ARM)
                    self._phase(
                        f"return leg {k}/{len(legs)}", f"{travel:.1f} deg max at {self.s.speed_deg_s:g} deg/s"
                    )
                    ok, message = self.arm.return_to_rest(
                        {m: target[m] for m in core.ARM}, self.abort_event, self.s.speed_deg_s, watch
                    )
                    if return_trips:  # the helper reports a raised trip as a plain failure
                        ok, message = False, "; ".join(f"{t.name}: {t.detail}" for t in return_trips)
                        out["return_trips"] = [dataclasses.asdict(t) for t in return_trips]
                    entry = {"leg": k, "ok": ok, "message": message}
                    out["legs"].append(entry)
                    if self.abort_event.is_set():  # the second signal (the helper reports "cut-short")
                        out["returned"] = f"leg {k} cut short by the second signal; released there"
                        break
                    if return_trips:
                        self._hold_mid_air(out, f"during leg {k}: {message}", reanchor=True)
                        break
                    pos = self._fresh_arm(watch)  # where the arm really is, for the verdict and the next leg
                    if return_trips:
                        self._hold_mid_air(out, f"after leg {k}: {return_trips[0].detail}", reanchor=True)
                        break
                    if pos is None and ok and last:
                        # The helper judged arrival at the rest on its own fresh readings (within 6 deg):
                        # lying there, torque off is the end the return was for.
                        out["returned"] = "at the rest (the helper's verdict; no reading after it)"
                        break
                    if pos is None:
                        self._hold_mid_air(out, f"after leg {k}: no fresh reading", reanchor=True)
                        break
                    off = max(abs(pos[m] - target[m]) for m in core.ARM)
                    if any(abs(pos[m] - target[m]) > self.s.trips.lag_for(m) for m in core.ARM):
                        # Further off than the lag trip (whatever the helper said): never a mid-air release.
                        why = f"leg {k} ended {off:.1f} deg off its target ({message or 'helper: arrived'})"
                        self._hold_mid_air(out, why, reanchor=True)
                        break
                    if not ok:
                        # Stopped short within the lag trip: the standing sag under load (7.4 deg on the
                        # shoulder at the start pose) is more than the helper's 6 deg settle band. Arrived.
                        entry["arrived"] = f"within the lag trip ({off:.1f} deg off)"
                        if last:
                            out["returned"] = f"at the rest ({off:.1f} deg off)"
                    # The next leg's guard check starts from the goal this reading implies (it sags below).
                    offsets = self.hold_error.value()
                    here = {**target, **{m: pos[m] + offsets[m] for m in core.ARM}}
        finally:
            self._phase("release all torque")
            out["release_problems"] = self.arm.release()
            self.energized = False
        return out

    def _fresh_arm(self, watch, attempts: int = 3) -> dict | None:
        """A fresh reading of the arm joints (retried: one lost frame is not a failed read), through the
        speed watch; None when no fresh reading can be had."""
        for _ in range(attempts):
            try:
                pos = self.arm.read().pos
            except Exception:  # noqa: BLE001
                continue
            watch(pos)
            return {m: float(pos[m]) for m in core.ARM}
        return None

    def _hold_mid_air(self, out, why: str, reanchor: bool = False) -> None:
        """Never release an arm that is up: HOLD (torque on, gripper already limp) until the second
        signal. With `reanchor`, the goals go to the fresh measured pose plus the hold error first, so the
        arm is held where it is (a ramp cut short leaves its goal elsewhere); while no fresh reading can be
        had the last goal is held and the re-anchor retried. The second signal is the only mid-air release."""
        self._phase("stop: HOLDING", why)
        # Any signal from here releases. A stop that began without a signal (call cap, trip) left stop_event
        # clear, so the handler would have spent the next press on "stop" and the hold kept waiting: Gavin
        # pressed once, supported the arm, and torque stayed on (live 2026-11-08T215535Z).
        self.stop_event.set()
        print(
            f"RETURN HELD ({why}): HOLDING (gripper already limp). Support the arm, then press the stop "
            "button again (or Ctrl-C / kill -TERM) to release all torque.",
            flush=True,
        )
        anchored, attempts = not reanchor, 0
        while not self.abort_event.is_set():
            if not anchored:
                attempts += 1
                try:
                    measured = self.arm.reanchor(self.hold_error.value())
                    anchored = True
                    out["hold_reanchor"] = {
                        "measured": {m: round(float(measured[m]), 2) for m in core.ARM},
                        "attempts": attempts,
                    }
                except Exception as error:  # noqa: BLE001 - keep holding the last goal, retry
                    out["hold_reanchor"] = f"attempt {attempts}: {error.__class__.__name__}: {error}"
            self.sleep(0.1)
        out["returned"] = f"held {why}; released on the second signal"


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
        "follower_gains_override": {m: list(g) for m, g in FOLLOWER_GAINS.items()},
        "limits_source": limits_source,
        "right_arm": "none on the rig; reported to Dream parked at the arm file's rest, grip 1 (open); its answers are dropped",
        "dream_state": "the six arm joints at the last COMMANDED target (the arm sags ~5 cm under it), the gripper "
        "as measured; every trip, the guard monitor, the start gate and the stop use the measured pose"
        if settings.dream_state == "commanded"
        else "the measured joints and gripper",
        "speed": {
            "arm_target_deg_s": settings.speed_deg_s,
            "gripper_deg_s": settings.grip_deg_s,
            "retime": "each row and approach leg split into equal 20 Hz substeps, all joints arriving together",
            "hard_arm_cap_deg_s": HARD_ARM_SPEED_DEG_S,
            "bus_backstop_deg_s": settings.speed_deg_s,
            "return_deg_s": settings.speed_deg_s,
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
            "slide": "BEFORE Dream's client IK, every row of Dream's left hand target: the tcp clipped into the "
            f"box's tcp limits {core.SLIDE_MARGIN_MM} mm inside (core.clamp_eef_rows), then its z raised just "
            "enough that the lowest mesh vertex of " + "/".join(core.HAND_LINKS) + " (at Dream's requested "
            "rot6 and grip; wrist_yaw_link bounded over every roll) sits at the keep-out floor line "
            f"({core.keepout_floor_mm():g} mm) + {core.FLOOR_BUFFER_MM:g} mm + the predicted sag "
            f"{core.SAG_MM_PER_M2:g} mm/m^2 x tcp reach^2 (core.FloorSlide); x, y, orientation and grip as Dream "
            "asked, so the hand follows Dream down to the floor line, then slides along it"
            if guard.available
            else f"box clip only; NO floor slide: {guard.problem}",
            "workspace_box": f"{BOX_FILE.name}: tcp/wrist/wrist_pitch/elbow boxes on every substep -> override: "
            "a row that cannot be reached is skipped and the arm heads for Dream's later rows; if none is reachable it holds until the next call",
            "tank_guard": "every substep through makermodslab.tank_guard GuardedTargets.step (keep-out -> override as "
            "above; any other rejection -> stop) and "
            "every measured pose through monitor()"
            if guard.available
            else f"NOT AVAILABLE: {guard.problem}",
            "lag": f"measured vs commanded > {settings.trips.lag_deg} deg on any arm joint "
            f"({settings.trips.shoulder_lag_deg} on shoulder_lift) for {settings.trips.lag_ticks} ticks",
            "torque": f"|torque| > {settings.trips.torque_trip_nm} Nm for {settings.trips.torque_trip_ticks} ticks, or > "
            f"{settings.trips.torque_cap_nm} Nm for {settings.trips.torque_cap_s} s",
            "temperature": f">= {settings.trips.temp_trip_c} C (warn {settings.trips.temp_warn_c})",
            "overspeed": f"measured speed of any arm joint > {settings.trips.overspeed_deg_s} deg/s for "
            f"{settings.trips.overspeed_ticks} readings (successive fresh positions over their timestamps; "
            "gripper excluded; independent of the commands and the guard), in the loop and on every return leg",
            "feedback": "no fresh reply / motor fault / camera read error -> stop",
            "dream": "Dream error or a malformed / non-finite chunk -> stop",
            "inputs_unchecked": "first request refused locally (nothing sent) unless ~/.dreamlabs/inputs_checked holds its id",
            "start_gate": f"measured rest within {START_LIMIT_TOLERANCE_DEG} deg of limits and through the guard; "
            "the Dream start pose inside the limits and the box",
            "speed": f"any planned arm substep faster than {HARD_ARM_SPEED_DEG_S} deg/s -> stop",
            "stop": "every exit (trips, SIGINT/SIGTERM/SIGHUP, stop_button.py): the six arm joints re-anchored at "
            "their measured pose (+ standing error, no slew: no spring-back), gripper torque off at once, then "
            "the arm joints back to the measured rest leg by leg from the measured pose at the speed cap "
            "(guard-checked legs; stopped short within the lag trip = arrived), then all torque off at the rest; "
            "never released mid-air: path not clear, overspeed, no fresh reading or a leg ending beyond the lag "
            "trip -> re-anchor and HOLD until the second signal, which aborts the return or the hold and releases.",
        },
        "run": "rehearse: approach, hold at the start pose, stop; no Dream"
        if args.rehearse
        else "dream: approach, then Dream chunks",
        "approach": {
            "from": "the measured rest (lying on the tank floor)",
            "route": "core.approach_waypoints: lift shoulder/fold elbow first, then straight to the start pose",
            "dream_start_motor_deg": {m: round(v, 2) for m, v in jmap.to_motor(arm["rest"]).items()},
            "gripper": "as measured at the rest",
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
            "run",
            "approach",
            "dream_state",
        )
    }
    safety["arm_urdf"] = plan["arm_file"]["urdf_sha256"]
    safety["arm_tool"] = {k: arm[k] for k in core.TOOL_KEYS}
    # The follower gains and the tank keep-out shape every motion too (both were left out until 2026-11-08,
    # so a floor move and a gain change kept the old plan id).
    safety["follower_gains_override"] = plan["follower_gains_override"]
    safety["keepout_sha256"] = core.sha256_bytes(core.KEEPOUT_FILE.read_bytes())
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
    print(f"Dream state: {plan['dream_state']}")
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
    print(f"Run: {plan['run']}")
    ap = plan["approach"]
    print(f"Approach: from {ap['from']}, {ap['route']}; start pose (motor deg) {ap['dream_start_motor_deg']}")
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


def far_mutation():
    """Scenario `far`: the first chunk also turns wrist_roll 12 deg (0.25 deg a row), so the run stops
    more than the lag trip from the Dream start pose and the stop's first leg goes back to it."""
    done = []

    def mutate(rows):
        if done:
            return rows
        done.append(True)
        rows = rows.copy()
        rows[:, 5] += np.linspace(0.0, math.radians(12.0), len(rows))
        return rows

    return mutate


def run_dry(args, ctx) -> int:
    deny_network()  # after the read-only Lab check: nothing below may reach the network
    results = []
    scenarios = SCENARIOS if args.scenario == "all" else (args.scenario,)
    for scenario in scenarios:
        clock = VirtualClock()
        out = ctx["out"] / scenario
        out.mkdir(parents=True, exist_ok=True)
        start = {**TRACE_POSE, **(args.start_motor_deg or {})}
        cameras = (
            [] if args.rehearse else [c["name"] for c in ctx["record"].get("cameras") or []] + ["(fake)"]
        )
        arm = FakeArm(start, cameras, clock, scenario, sag=START_POSE_SAG if scenario == "far" else None)
        if args.rehearse:
            dream = None
        elif ctx["dream_problem"] is None and not args.internal_stub:
            dream = DreamReal(ctx["arm_path"], out, offline=True, scenario=scenario)
            dream.install_gate("record")
        else:
            dream = DreamInternalStub(ctx["jmap"], scenario)
        stop_event, abort_event = threading.Event(), threading.Event()
        # `far` stops at the call cap right after its rolled chunk (> 10 deg from the start pose).
        settings = dataclasses.replace(ctx["settings"], max_calls=1) if scenario == "far" else ctx["settings"]
        worker = (
            Worker(dream, clock, threaded=False, latency_s=1.5, max_calls=settings.max_calls)
            if dream is not None
            else None
        )

        def sleep(dt, clock=clock, stop_event=stop_event, abort_event=abort_event, scenario=scenario):
            clock.sleep(dt)
            if scenario == "signal" and clock.t > 1006.0:
                stop_event.set()
            if clock.t > 1900.0:  # a dry run never waits forever (a held return): the fake RELEASE ALL
                stop_event.set()
                abort_event.set()

        episode = Episode(
            arm,
            dream,
            worker,
            ctx["jmap"],
            ctx["limits"],
            ctx["box"],
            ctx["guard"],
            settings,
            clock,
            sleep,
            out,
            stop_event,
            abort_event,
            mutate={"jump": jump_mutation, "far": far_mutation()}.get(scenario),
            arm_rest=ctx["arm"]["rest"],
            say=print if args.rehearse else None,
        )
        summary = episode.run()
        summary["fake_arm_events"] = arm.events
        summary["inputs_id"] = getattr(dream, "inputs_id", None)
        if dream is not None:
            dream.close()
        core.atomic_json(out / "summary.json", summary)
        results.append((scenario, summary))
    what = "Rehearsal" if args.rehearse else "Dry run"
    print(f"\n=== {what} on fakes (virtual clock; no port, camera, network or billed call) ===")
    for scenario, s in results:
        stop = s["stop"]
        print(f"- {scenario:11s} -> {s['reason'][:110]}")
        print(
            f"    approach substeps {s['approach_substeps']}, calls {s['calls']}, rows {s['rows_planned']}, "
            f"substeps sent {s['substeps_sent']}, rows stretched by the speed cap {s['stretched_rows']}, "
            f"clipped {s['clipped']}, guard {s['guard'] or 'n/a'}"
        )
        print(f"    phases: {' > '.join(s['phases'])}")
        anchor = stop.get("reanchor")
        if isinstance(anchor, dict):
            anchor = "ok, offsets " + ", ".join(f"{m} {v:+.1f}" for m, v in anchor["offset_deg"].items())
        print(
            f"    stop path: re-anchor {anchor}; gripper {stop['gripper_release']}; legs {len(stop['legs'])} "
            f"({', '.join(str(leg['ok']) for leg in stop['legs']) or 'none'}); {stop['returned']}; "
            f"release problems: {stop['release_problems']}"
        )
        print(f"    fake arm: {' | '.join(e.split(':')[0] for e in s['fake_arm_events'])}")
        print(
            f"    peak lag {max(s['peak_lag_deg'].values()):.1f} deg, peak torque "
            f"{max(s['peak_torque_nm'].values()):.2f} Nm, peak temp {max(s['peak_temp_c'].values()):.1f} C, "
            f"peak measured arm speed {max(s['peak_speed_deg_s'].values()):.1f} deg/s"
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


def rehearse_refusals(f: core.LiveFacts) -> list[str]:
    """--rehearse --live: the refusals that matter for the hardware (tank model approved, joint signs,
    plan id, Lab idle, port free, tank guard), with the Dream-only facts (tool point, cameras, client,
    ledger, credentials, an explicit dreamcli arm file) taken as satisfied: no Dream call is made."""
    return core.live_refusals(
        dataclasses.replace(
            f,
            tcp_problem=None,
            missing_slots=[],
            dream_problem=None,
            ledger_remaining=1,
            max_calls=1,
            credentials_present=True,
            arm_file_explicit=True,
        )
    )


def install_stop_signals(pid_file):
    """SIGINT/SIGTERM/SIGHUP (handlers installed, so nohup cannot swallow them): the first stops, a
    second aborts the return and releases. stop_button.py sends SIGTERM to the pid written here."""
    if pid_file:
        Path(pid_file).write_text(f"{os.getpid()}\n")
    stop_event, abort_event = threading.Event(), threading.Event()

    def on_signal(signum, _frame):
        (abort_event if stop_event.is_set() else stop_event).set()
        print(f"signal {signum}: {'abort the return' if abort_event.is_set() else 'stopping'}", flush=True)

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, on_signal)
    return stop_event, abort_event


def remove_pid_file(pid_file):
    with contextlib.suppress(OSError):
        if pid_file and Path(pid_file).read_text().strip() == str(os.getpid()):
            Path(pid_file).unlink()


def run_live(args, ctx) -> int:
    arm_path, arm = ctx["arm_path"], ctx["arm"]
    ledger = core.Ledger(LEDGER, LEDGER_CAP)
    facts = live_facts(args, ctx, ledger)
    refusals = rehearse_refusals(facts) if args.rehearse else core.live_refusals(facts)
    if refusals:
        print(f"\n{'--rehearse ' if args.rehearse else ''}--live REFUSED:")
        for r in refusals:
            print(f"  - {r}")
        return 2
    out = ctx["out"]
    out.mkdir(parents=True, exist_ok=True)
    core.atomic_json(out / "plan.json", ctx["plan"])
    stop_event, abort_event = install_stop_signals(args.pid_file)
    try:
        return _run_live(args, ctx, arm_path, arm, ledger, out, stop_event, abort_event)
    finally:
        remove_pid_file(args.pid_file)


def _run_live(args, ctx, arm_path, arm, ledger, out, stop_event, abort_event) -> int:
    dream = worker = None
    if not args.rehearse:
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
        print(f"inputs id {dream.inputs_id} checked.", flush=True)
        dream = DreamReal(arm_path, out, offline=False)
        dream.install_gate("gate")
        worker = Worker(
            dream, time.monotonic, threaded=True, ledger=ledger, run_id=out.name, max_calls=args.max_calls
        )
    print(
        f"{'REHEARSAL (no Dream, no billed call). ' if args.rehearse else ''}Energizing next: ALL SEVEN joints may "
        f"move (plan {ctx['plan']['plan_id']}), arm <= {ctx['settings'].speed_deg_s:g} deg/s. Stop: the red button "
        f"(stop_button.py --pid-file {args.pid_file or '<none>'}), Ctrl-C or kill -TERM {os.getpid()}; a second press "
        "releases all torque at once; the power switch is the hard stop.",
        flush=True,
    )
    speed = ctx["settings"].speed_deg_s
    arm_io = LabArm(ctx["home"], ctx["record"], speed, cameras=not args.rehearse)
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
        arm_rest=arm["rest"],
        say=lambda line: print(line, flush=True),
    )
    try:
        summary = episode.run()
    finally:
        if dream is not None:
            dream.close()
    core.atomic_json(out / "summary.json", summary)
    print(json.dumps({k: summary[k] for k in ("reason", "calls", "phases", "stop")}, indent=1, default=str))
    return 0 if not summary["trips"] else 1


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = p.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true", help="the default")
    mode.add_argument(
        "--preview", action="store_true", help="no motion: cameras + Dream client offline, print inputs id"
    )
    mode.add_argument("--live", action="store_true", help="billed; refused unless every precondition holds")
    p.add_argument(
        "--rehearse",
        action="store_true",
        help="no Dream: approach, hold, stop. Alone = on FakeArm; with --live = the real arm (no billed call)",
    )
    p.add_argument(
        "--hold-s", type=float, default=DEFAULT_HOLD_S, help="--rehearse: hold time at the start pose"
    )
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
    p.add_argument(
        "--pause-s",
        type=float,
        default=0.0,
        help="hold at the start pose this long before the first Dream call",
    )
    p.add_argument(
        "--max-stuck-calls",
        type=int,
        default=3,
        help="stop after this many stuck calls in a row (see --stuck-substeps)",
    )
    p.add_argument(
        "--stuck-substeps",
        type=int,
        default=2,
        help="a call that executes at most this many substeps counts as stuck",
    )
    p.add_argument(
        "--dream-state",
        choices=("commanded", "measured"),
        default="commanded",
        help="the arm joints Dream is told: the last command (default; the arm sags under it) or the reading",
    )
    p.add_argument("--rows", type=int, default=48)
    p.add_argument("--speed-deg-s", type=float, default=HARD_ARM_SPEED_DEG_S, help="arm cap, at most 15")
    p.add_argument("--grip-deg-s", type=float, default=60.0)
    p.add_argument("--max-jump-deg", type=float, default=20.0)
    p.add_argument("--limit-margin-deg", type=float, default=1.0)
    p.add_argument("--lag-trip-deg", type=float, default=10.0)
    p.add_argument("--shoulder-lag-trip-deg", type=float, default=20.0)
    p.add_argument("--torque-trip-nm", type=float, default=12.0, help="instant tier (2 ticks), at most 14")
    p.add_argument("--torque-cap-nm", type=float, default=5.0, help="sustained tier, with --torque-cap-s")
    p.add_argument("--torque-cap-s", type=float, default=5.0)
    p.add_argument("--temp-trip-c", type=float, default=80.0)
    p.add_argument("--overspeed-deg-s", type=float, default=30.0, help="measured arm joint speed trip")
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
    if not 0 < args.speed_deg_s <= HARD_ARM_SPEED_DEG_S:
        p.error(f"--speed-deg-s must be in (0, {HARD_ARM_SPEED_DEG_S:g}]: the hard arm speed cap")
    if not 0 < args.grip_deg_s <= 90 or not 0 < args.max_jump_deg <= 20 or not 0 < args.hold_s <= 600:
        p.error("--grip-deg-s (0, 90], --max-jump-deg (0, 20], --hold-s (0, 600]")
    if args.max_stuck_calls < 1 or args.stuck_substeps < 0:
        p.error("--max-stuck-calls >= 1 and --stuck-substeps >= 0")
    if args.rehearse and args.preview:
        p.error("--rehearse goes with --dry-run (the default) or --live, not --preview")
    kind = "live" if args.live else "preview" if args.preview else "dry"
    kind = f"rehearse-{kind}" if args.rehearse else kind

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
        shoulder_lag_deg=args.shoulder_lag_trip_deg,
        torque_trip_nm=args.torque_trip_nm,
        torque_cap_nm=args.torque_cap_nm,
        torque_cap_s=args.torque_cap_s,
        temp_trip_c=args.temp_trip_c,
        overspeed_deg_s=args.overspeed_deg_s,
    )
    try:
        trips.check()
    except ValueError as error:
        p.error(str(error))
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
        args.hold_s,
        args.pause_s,
        args.max_stuck_calls,
        stuck_substeps=args.stuck_substeps,
        dream_state=args.dream_state,
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
    if not args.live and not args.preview:
        print(
            f"lab: {lab_holder(args.lab_url, True) or 'idle'}; follower port holders: "
            f"{port_holders(record['follower_port']) or 'none'}"
        )
        out.mkdir(parents=True, exist_ok=True)
        core.atomic_json(out / "plan.json", plan)
        facts = live_facts(args, ctx, core.Ledger(LEDGER, LEDGER_CAP))
        refusals = rehearse_refusals(facts) if args.rehearse else core.live_refusals(facts)
        live = "--rehearse --live" if args.rehearse else "--live"
        print(f"{live} readiness: " + ("would proceed" if not refusals else f"{live} would REFUSE:"))
        for r in refusals:
            print(f"  - {r}")
        return run_dry(args, ctx)
    if args.preview:
        return run_preview(args, ctx)
    return run_live(args, ctx)


if __name__ == "__main__":
    raise SystemExit(main())
