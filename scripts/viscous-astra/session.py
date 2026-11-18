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
"""Single-owner arm lifecycle adapted from metal-arm-harness operator/control/executor.

The hardware writer and the complete stop/hold/release state machine belong to dream_real.
"""

import json
import math
import shutil
import time
from collections import deque

import numpy as np
from model import DT, LEG, SLOW, ZONE, RefusalError, core, mix, real, travel
from PIL import Image


def number(value):
    try:
        result = float(value)
    except (OverflowError, ValueError) as exc:
        raise RefusalError(f"expected a finite number, got {value!r}") from exc
    if not math.isfinite(result):
        raise RefusalError("expected a finite number")
    return result


class PassiveLabArm(real.LabArm):
    """LabArm reads, without its energizing follower handshake or any actuation/disable writes.

    The human starts with torque off. Use the tank measurement's bus-only connection
    and turn correction. LabArm.read uses remote_can's fresh-status requests.
    """

    def connect(self):
        from lerobot.robots import make_robot_from_config
        from makermodslab.arms import registry
        from makermodslab.tank_measure import full_turn_offsets

        family = registry.get("viscous")
        self.robot = make_robot_from_config(
            family.single_follower_config(self.record["follower_port"], self.record["follower_config"])
        )
        self.robot.bus.connect(handshake=False)
        try:
            raw = family._read_fresh_positions(self.robot)
            self.robot._turn_offset = full_turn_offsets(raw, self.robot.config)
            _, self.grab, self.close_cameras = real.open_cameras(self.record, fake=False)
        except BaseException:
            self.robot.bus.disconnect(False)
            raise

    def read(self):
        reading = super().read()
        reading.frames = self.grab()
        return reading

    def release(self):
        try:
            self.close_cameras()
        finally:
            self.robot.bus.disconnect(False)


class ReplayArm(real.FakeArm):
    """Paired recorded views, optionally a 25 mm contact fixture at a known tank location."""

    def __init__(self, model, folder, cube, clock):
        start = {
            **model.map.to_motor(json.loads(real.DEFAULT_ARM_FILE.read_text())["rest"]),
            "gripper": core.GRIP_CLOSED_DEG,
        }
        super().__init__(start, [], clock, heat_per_tick=0)
        self.model = model
        self.frames = folder
        self.names = sorted(
            p.name for p in (folder / "head").glob("*.jpg") if (folder / "left" / p.name).is_file()
        )
        if not self.names:
            raise ValueError(f"need paired recorded head/left JPEGs in {folder}")
        self.cube = None if cube is None else np.array([*cube, model.guard.planes["floor"] / 1000 + 0.0125])
        self.held = False

    def read(self):
        reading = super().read()
        if self.cube is not None and self.energized and not self.gripper_limp:
            tip = np.array(self.model.pose(reading.pos)["xyz_m"])
            contact = core.grip_to_motor(25 / (2000 * self.model.guard.model.gripper_slide_open_m))
            if self.target["gripper"] < contact:
                self.held = False
            elif self.held or np.linalg.norm(tip - self.cube) < 0.004:
                self.held = True
                self.cube = tip
                self.pos["gripper"] = reading.pos["gripper"] = contact
                reading.torque["gripper"] = 0.8
        return reading


class ReturnGuard(real.Guard):
    def __init__(self, model):
        self.guard = model.guard
        self.targets = None


class Operator(real.Episode):
    def __init__(self, arm, model, folder, env, armed, events, pid_file, budget):
        self.model, self.folder, self.armed = model, folder, armed
        self.number, self.budget = 0, budget
        self.trace = (folder / "commands.jsonl").open("a", buffering=1)
        self.floor_sample = None
        self.grip_history = deque(maxlen=4)
        self.closing = False
        self.quit = False
        self.finished = False
        self.connected = False
        self.goal_tip = None
        self.last_poll = 0.0
        self.reading = None
        fake = isinstance(arm, ReplayArm)
        clock = arm.clock if fake else time.monotonic
        settings = real.Settings("", 0, 0, SLOW, 15, LEG, 0, core.TripLimits(), {})
        super().__init__(
            arm,
            None,
            None,
            model.map,
            model.limits,
            None,
            ReturnGuard(model),
            settings,
            clock,
            clock.sleep if fake else time.sleep,
            folder,
            *events,
            status_path=str(pid_file) + core.STATUS_SUFFIX,
            environment=env,
            arm_rest=model.spec["rest"],
        )

    def log(self, **entry):
        self.trace.write(json.dumps({"time": time.time(), **entry}, allow_nan=False) + "\n")

    def begin(self):
        self.arm.connect()
        self.connected = True
        r = self.arm.read()
        self.start_pose = self.commanded = dict(r.pos)
        self.trail = [dict(r.pos)]
        v = self.model.guard.start_gate(r.pos)
        if not v.ok:
            raise ValueError(f"start gate: {v}")
        for m, (lo, hi) in self.limits.items():
            if not lo - 5 <= r.pos[m] <= hi + 5:
                raise ValueError(f"start outside unit profile: {m}")
        self.poll()
        if self.armed:
            if self.stop_event.is_set():
                raise RuntimeError("red button before engage")
            self.commanded = self.arm.engage()
            self.energized = True
            if travel(r.pos, self.commanded) > 1 or not self.model.guard.start_gate(self.commanded).ok:
                raise RuntimeError("arm moved during engage")
        self._phase("operator", "holding at measured rest" if self.armed else "passive telemetry")
        self.poll()

    def poll(self):
        if self.stop_event.is_set():
            raise RuntimeError("red button")
        if isinstance(self.arm, ReplayArm):
            self.clock.sleep(DT)
        r = self.arm.read()
        self.reading = r
        if not isinstance(self.arm, ReplayArm):
            for name in ("head", "left"):
                image = r.frames.get(name)
                if (
                    not isinstance(image, np.ndarray)
                    or image.ndim != 3
                    or image.shape[2] != 3
                    or image.dtype != np.uint8
                ):
                    raise RuntimeError(f"missing or invalid RGB frame from {name}")
        reference = self.commanded if self.armed else r.pos
        trips = self.monitor.update(r.pos, reference, r.torque, r.temp)
        if problem := self.guard.monitor(r.pos):
            trips.append(core.Trip("tank_guard_measured", problem))
        self.last_state = self._state(r, reference, self.phase)
        self._tick("operator", r, reference, trips)
        self._status(r)
        self.grip_history.append(r.pos["gripper"])
        self.last_poll = time.monotonic()
        if trips:
            self.log(trips=[vars(t) for t in trips])
            raise real.TripStopError(trips)
        self.hold_error.update(r.pos, reference, self.clock())

    def heartbeat(self):
        if self.stop_event.is_set() or time.monotonic() - self.last_poll >= DT:
            self.poll()

    def legs_from(self, here):
        # Include the beginning of a partly executed leg. The runner checks each
        # segment again after re-anchoring and holds on any impossible return.
        legs = []
        for dest in reversed(self.trail):
            count = max(1, math.ceil(travel(here, dest) / LEG))
            legs.extend(mix(here, dest, i / count) for i in range(1, count + 1))
            here = dest
        return legs

    def end(self, reason):
        if self.finished:
            return self.stop_result
        self.finished = True
        if self.reading and self.commanded:
            self.commanded = {**self.commanded, "gripper": self.reading.pos["gripper"]}
        # Leave the two signal stages to install_stop_signals and Episode.stop.
        # HOLDING arms the supported release itself, including after a normal quit.
        if self.connected and (self.armed or isinstance(self.arm, PassiveLabArm)):
            self.stop_result = self.stop(reason)
        else:
            self.stop_result = {"reason": reason, "passive": True, "release_problems": None}
        if isinstance(self.arm, ReplayArm):
            self.stop_result["fake_events"] = self.arm.events
        self.log(stop=self.stop_result)
        core.atomic_json(self.folder / "stop.json", self.stop_result)
        return self.stop_result

    def execute(self, goals, returning=False):
        steps = self.model.plan(self.commanded, goals, self.start_pose, self.heartbeat)
        # A stalled gripper can be appreciably wider than its target. Prove the
        # arm route at the sensed aperture too (finger slides are linear).
        grip = self.reading.pos["gripper"]
        if abs(grip - self.commanded["gripper"]) > 1 and any(travel(self.commanded, g) > 0 for g in goals):
            self.model.plan(
                {**self.commanded, "gripper": grip},
                [{**g, "gripper": grip} for g in goals],
                self.start_pose,
                self.heartbeat,
            )
        measured = self.reading.pos
        if self.armed and self.model.clearance(measured)["hand_floor_mm"] < 0 and measured != self.commanded:
            # Check the requested recovery from the sensed height as well as from
            # the last commanded height. A refusal here leaves every motor alone.
            offset = {m: measured[m] - self.commanded[m] for m in core.MOTORS}
            sensed_route = [{m: g[m] + offset[m] for m in core.MOTORS} for g in goals]
            self.model.plan(measured, sensed_route, self.start_pose, self.heartbeat)
        self.log(waypoints=len(steps), guards=[v for _, _, _, v in steps])
        last_leg = None
        for target, origin, speed, _ in steps:
            if origin is not last_leg and not returning and self.trail[-1] != origin:
                self.trail.append(dict(origin))
            last_leg = origin
            if self.armed:
                self.poll()
                measured = self.reading.pos
                height = self.model.clearance(measured)["hand_floor_mm"]
                if height < 0:
                    projected = {m: measured[m] + target[m] - self.commanded[m] for m in core.MOTORS}
                    if self.model.clearance(projected)["hand_floor_mm"] < height - 1e-7:
                        raise RuntimeError("measured hand below floor; next waypoint would deepen contact")
                # Re-time on measured near-floor sag while preserving the approved line.
                ticks = 1
                if speed > SLOW and height <= ZONE + self.model.guard.motion_bound_mm(self.commanded, target):
                    ticks = max(1, math.ceil(travel(self.commanded, target) / (SLOW * DT)))
                before = self.commanded
                for i in range(1, ticks + 1):
                    if i > 1:
                        self.poll()
                    if self.stop_event.is_set():
                        raise RuntimeError("red button")
                    step = mix(before, target, i / ticks)
                    self.arm.send(step, self.commanded)
                    self.commanded = step
                    self.sleep(DT)
            else:
                self.heartbeat()
                self.commanded = target
            if (
                returning
                and len(self.trail) > 1
                and all(abs(target[m] - self.trail[-1][m]) < 1e-8 for m in core.ARM)
            ):
                self.trail.pop()
        if returning:
            self.trail = [self.start_pose]
        if self.armed:
            for _ in range(4):
                self.poll()
                self.sleep(DT)
        return f"{len(steps)} waypoints approved" + ("" if self.armed else " (preview only)")

    def command(self, words):
        if not words:
            raise RefusalError("empty command")
        cmd, *args = words
        if cmd == "tip":
            if len(args) < 3:
                raise RefusalError("tip X Y Z [--pitch DEG] [--yaw DEG]")
            xyz = [number(v) for v in args[:3]]
            flags = args[3:]
            if (
                len(flags) % 2
                or any(k not in ("--pitch", "--yaw") for k in flags[::2])
                or len(set(flags[::2])) != len(flags[::2])
            ):
                raise RefusalError("unknown or duplicate tip option")
            opt = dict(zip(flags[::2], map(number, flags[1::2]), strict=True))
            pitch, yaw = opt.get("--pitch", -90), opt.get("--yaw")
        elif cmd == "nudge":
            opts = {}
            for arg in args:
                k, _, val = arg.partition("=")
                if k not in ("forward", "left", "up", "pitch") or k in opts:
                    raise RefusalError("use forward=, left=, up=, pitch=")
                opts[k] = number(val)
            if not opts:
                raise RefusalError("nudge needs a delta")
            current = self.model.pose(self.commanded)
            xyz, pitch, yaw = self.goal_tip or (current["xyz_m"], current["pitch_deg"], None)
            xyz = np.asarray(xyz) + [opts.get(k, 0) for k in ("forward", "left", "up")]
            pitch += opts.get("pitch", 0)
        elif cmd == "gripper" or cmd in ("open", "close") and not args:
            if cmd == "gripper" and len(args) != 1:
                raise RefusalError("gripper OPEN_FRACTION between 0 and 1")
            fraction = number(args[0]) if cmd == "gripper" else float(cmd == "open")
            if not 0 <= fraction <= 1:
                raise RefusalError("opening must be within [0,1]")
            goal = {**self.commanded, "gripper": core.grip_to_motor(fraction)}
            closing = goal["gripper"] > self.reading.pos["gripper"] + 1
            result = self.execute([goal])
            self.closing = closing
            return result
        elif args:
            raise RefusalError("unexpected command arguments")
        elif cmd in ("observe", "status"):
            return "fresh observation"
        elif cmd == "quit":
            self.quit = True
            return "final pre-stop observation"
        elif cmd == "rest":
            goals = [{**q, "gripper": self.commanded["gripper"]} for q in reversed(self.trail)]
            result = self.execute(goals, returning=True)
            self.goal_tip = None
            return result
        elif cmd == "calibrate-floor":
            if self.armed:
                raise RefusalError("floor touches require a separate torque-off session")
            distances = self.model.clearance(self.reading.pos)
            if (
                distances["jaw_floor_mm"] > distances["hand_floor_mm"] + 1
                or core.motor_to_grip(self.reading.pos["gripper"]) > 0.05
            ):
                raise RefusalError("floor touch needs closed jaws as the lowest part of the hand")
            z = self.model.guard.planes["floor"] + distances["jaw_floor_mm"]
            if self.floor_sample is None:
                self.floor_sample = z
                return "first touch saved; lift, then re-seat the jaws and sample again"
            a, self.floor_sample = self.floor_sample, None
            if abs(a - z) > 3:
                raise RefusalError(f"floor touches differ by {abs(a - z):.1f} mm; repeat both")
            result = {
                "floor_mm": (a + z) / 2,
                "samples_mm": [a, z],
                "urdf_sha256": self.model.sha,
                "backend": "fake" if isinstance(self.arm, ReplayArm) else "lab",
                "environment": self.environment.name,
            }
            previous = self.model.guard.planes["floor"]
            self.model.guard.planes["floor"] = result["floor_mm"]
            if not self.model.guard.check_pose(self.reading.pos).ok:
                self.model.guard.planes["floor"] = previous
                raise RefusalError("touch would place another link outside the tank keep-out")
            core.atomic_json(self.folder / "floor.json", result)
            return f"session floor {result['floor_mm']:.1f} mm; measurement saved in floor.json"
        else:
            raise RefusalError("unknown operator command")
        goal = self.model.solve(self.commanded, xyz, pitch, yaw, poll=self.heartbeat)
        result = self.execute([goal])
        self.goal_tip = (list(xyz), pitch, yaw)
        return result

    def reply(self, words):
        self.number += 1
        self.log(request=words, sequence=self.number)
        self.poll()
        try:
            if self.number >= self.budget and words != ["quit"]:
                raise RefusalError("command budget reached; stopping")
            note = self.command(words)
            ok = True
        except RefusalError as exc:
            ok, note = False, str(exc)
        self.poll()
        reply = {"ok": ok, "guard": note, **self.observation()}
        if self.quit or self.number >= self.budget:
            reply["observation_phase"] = "before stop; cameras close before return"
            reply["stop"] = self.end("quit" if self.quit else "budget")
            reply["ok"] = ok and not reply["stop"].get("release_problems")
        self.log(reply=reply)
        return reply

    def observation(self):
        r = self.reading
        frames = {}
        for name in ("head", "left"):
            dest = self.folder / "frames" / name / f"{self.number:04}.jpg"
            dest.parent.mkdir(parents=True, exist_ok=True)
            if isinstance(self.arm, ReplayArm):
                shutil.copyfile(
                    self.arm.frames / name / self.arm.names[(self.number - 1) % len(self.arm.names)], dest
                )
            else:
                Image.fromarray(r.frames[name]).save(dest, quality=88)
            frames[name] = str(dest)
        fraction = core.motor_to_grip(r.pos["gripper"])
        holding = (
            self.energized
            and self.closing
            and self.commanded["gripper"] - r.pos["gripper"] > 3
            and len(self.grip_history) == 4
            and max(self.grip_history) - min(self.grip_history) < 0.5
            and abs(r.torque["gripper"]) >= 0.4
        )
        reply = {
            "armed": self.armed,
            "sequence": self.number,
            "joints_motor_deg": r.pos,
            "tip": self.model.pose(r.pos),
            "commanded_tip": self.model.pose(self.commanded),
            "gripper": {
                "fraction": fraction,
                "opening_mm": fraction * 2000 * self.model.guard.model.gripper_slide_open_m,
                "holding_something": holding,
            },
            "clearance": self.model.clearance(r.pos),
            "floor_z_mm": self.model.guard.planes["floor"],
            "urdf_sha256": self.model.sha,
            "frames": frames,
        }
        if isinstance(self.arm, ReplayArm) and self.arm.cube is not None:
            reply["fixture_cube_m"] = self.arm.cube.tolist()
        return reply
