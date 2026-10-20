"""Bench helpers shared by the Viscous follower scripts in this directory.

Imported by ``viscous-gravity-collect.py`` and ``viscous-contact-check.py``
(run as scripts, so this directory is on ``sys.path``). Everything here talks
to one single-arm Viscous follower outside a Lab session.
"""

import contextlib
import json
import os
import time
from pathlib import Path

import numpy as np

from makermodslab.arm_gravity import interpolate

ARM = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll")
GRAVITY_JOINTS = ("shoulder_lift", "elbow_flex", "wrist_flex")
HUB = {"shoulder_lift": -90.0, "elbow_flex": 90.0, "wrist_flex": 0.0, "wrist_yaw": 0.0, "wrist_roll": 0.0}
# Near-balanced upright pose: CAD gravity under 0.1 N m on every joint.
COOL = {"shoulder_lift": -92.0, "elbow_flex": 160.0, "wrist_flex": 20.0, "wrist_yaw": 0.0, "wrist_roll": 0.0}
TICK_S = 0.02


class RunAbortError(RuntimeError):
    pass


def _interrupt(signum, frame):
    # Explicit, because a process launched in the background (nohup, &) starts
    # with SIGINT ignored and Python then installs no Ctrl-C handler at all.
    raise KeyboardInterrupt


def load_record(name):
    path = Path(os.environ["MAKERMODSLAB_HOME"]) / "robots" / f"{name}.json"
    record = json.loads(path.read_text())
    if record.get("arm_type") != "viscous" or record.get("mode", "single") != "single":
        raise SystemExit(f"{name} is not a single Viscous arm")
    return record


def build_robot(record):
    from lerobot.robots import make_robot_from_config
    from makermodslab.arms import registry

    config = registry.get("viscous").single_follower_config(
        record["follower_port"], record["follower_config"]
    )
    return make_robot_from_config(config)


def read_pose_readonly(robot):
    """Present joint angles without enabling torque (the RobStride handshake only pings)."""
    try:
        robot.bus.connect()
        robot._detect_full_turn_offsets()
        return robot._read_present_positions()
    finally:
        with contextlib.suppress(Exception):
            robot.bus.disconnect(disable_torque=False)


def clipped(pose, limits):
    return {k: min(limits[k][1], max(limits[k][0], v)) for k, v in pose.items()}


class Runner:
    def __init__(self, robot, args, gripper):
        self.robot, self.args, self.gripper = robot, args, gripper
        self.command = None
        self.last_obs = None
        self.on_tick = None  # called with (target, obs) after every tick

    def _tick(self, target):
        started = time.monotonic()
        self.robot.send_action({**{f"{k}.pos": v for k, v in target.items()}, "gripper.pos": self.gripper})
        self.command = dict(target)
        obs = self.robot.get_observation()
        self.last_obs = obs
        if self.on_tick is not None:
            self.on_tick(target, obs)
        hot = {k: obs[f"{k}.temp"] for k in (*ARM, "gripper") if obs[f"{k}.temp"] >= self.args.max_temp}
        if hot:
            raise RunAbortError(f"Too hot: {hot}")
        lag = {k: target[k] - obs[f"{k}.pos"] for k in ARM}
        worst = max(lag, key=lambda k: abs(lag[k]))
        if abs(lag[worst]) > self.args.max_lag:
            raise RunAbortError(f"{worst} lags its command by {lag[worst]:+.1f} deg: blocked or overloaded")
        time.sleep(max(0.0, TICK_S - (time.monotonic() - started)))
        return obs

    def move(self, target, speed):
        for p in interpolate(self.command, target, speed * TICK_S):
            self._tick(p)
            self.command = p

    def hold(self, seconds):
        frames, end = [], time.monotonic() + seconds
        while time.monotonic() < end:
            frames.append(self._tick(self.command))
        return frames


def summarize(frames, key):
    values = np.array([[f[f"{k}.{key}"] for k in ARM] for f in frames])
    return dict(zip(ARM, values.mean(axis=0).tolist(), strict=True)), dict(
        zip(ARM, values.std(axis=0).tolist(), strict=True)
    )


def write_json(path, data):
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(data, indent=1))
    tmp.replace(path)
