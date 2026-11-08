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
"""Pure helpers for driving the real Viscous arm with Dream-1 (goal B3).

Nothing here touches hardware, the network or the Dream client: numpy, PIL and
the standard library only, so every safety rule can be tested offline with
injected clocks. `dream_real.py` wires these to the arm, the cameras and Dream.

Units: motor degrees (what the follower reports, after its full-turn unwrap)
for everything the arm sees; IK radians for everything Dream sees. The tank
IK model (`tank_model/`) turns Dream's base 180 deg about z, so

    IK angle (deg) = sign * motor + offset           (motor_to_urdf.json)
    IK shoulder_pan = URDF shoulder_pan - PAN_SHIFT_DEG

and Dream's "forward" (+x of `dream_base`) points down the tank, where the
waterproof arm works (pan reads about +182..186 there).
"""

from __future__ import annotations

import base64
import fcntl
import hashlib
import io
import json
import math
import os
import time
import xml.etree.ElementTree as ET
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
ARM = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll")
MOTORS = (*ARM, "gripper")
SIDES = ("left", "right")
SLOTS = ("overhead", "wrist_l", "wrist_r")
PAN_SHIFT_DEG = 180.0
FRAME_SIZE = (320, 176)  # what Dream's YAM example and every sim run sent
JPEG_QUALITY = 85
FRAMES_IN_WINDOW, FRAME_PERIOD_S = 9, 0.2  # the client's window: 9 frames at 5 Hz
ROW_DT_S = 0.05  # Dream rows are 20 Hz
TOOL_KEYS = ("base", "flange", "tcp", "approach", "jaw")  # dream.arm_file.TOOL_KEYS
MOTOR_TO_URDF = ROOT / "configs/viscous/model/motor_to_urdf.json"
UNVERIFIED_MARKERS = ("pending", "unverified", "assumed")

# Gripper motor degrees for Dream's normalized grip (0 closed, 1 open). Open is
# the follower profile's lower limit; closed is the keep-out config's inner
# upper limit (configs/viscous/environments/aquarium_landen_75/
# keepout_viscous_waterproof_01.json, branch feat/viscous-dream1-tank-box),
# 2.9 deg inside the profile's -0.03 stop. Same sense as the Maker arm
# (closed near 0, open near -119). Confirm on the arm before a grasp trial.
GRIP_CLOSED_DEG = -2.9220847551671985
GRIP_OPEN_DEG = -115.10722104178238


# --- small utilities ---------------------------------------------------------


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def normalized_urdf_sha256(data: bytes) -> str:
    """The URDF's hash ignoring trailing newlines: `dreamcli tcp`'s copy adds one."""
    return sha256_bytes(data.rstrip(b"\n"))


def canonical_id(obj, n: int = 10) -> str:
    return sha256_bytes(json.dumps(obj, sort_keys=True, default=str).encode())[:n]


def atomic_json(path: Path, obj) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    with tmp.open("w") as f:
        json.dump(obj, f, indent=1, allow_nan=False, default=str)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


# --- URDF forward kinematics (numpy) ------------------------------------------


def _rpy(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def _axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    a = axis / np.linalg.norm(axis)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * (k @ k)


@dataclass(frozen=True)
class Joint:
    name: str
    kind: str
    parent: str
    child: str
    xyz: tuple
    rpy: tuple
    axis: tuple
    lower: float | None
    upper: float | None


def parse_urdf(text: str | bytes) -> dict[str, Joint]:
    """Joints by CHILD link name (each link has one parent joint)."""
    root = ET.fromstring(text)
    joints = {}
    for j in root.findall("joint"):
        origin = j.find("origin")
        axis = j.find("axis")
        limit = j.find("limit")
        xyz = tuple(map(float, (origin.get("xyz", "0 0 0") if origin is not None else "0 0 0").split()))
        rpy = tuple(map(float, (origin.get("rpy", "0 0 0") if origin is not None else "0 0 0").split()))
        joints[j.find("child").get("link")] = Joint(
            name=j.get("name"),
            kind=j.get("type"),
            parent=j.find("parent").get("link"),
            child=j.find("child").get("link"),
            xyz=xyz,
            rpy=rpy,
            axis=tuple(map(float, axis.get("xyz").split())) if axis is not None else (1.0, 0.0, 0.0),
            lower=float(limit.get("lower")) if limit is not None and limit.get("lower") else None,
            upper=float(limit.get("upper")) if limit is not None and limit.get("upper") else None,
        )
    return joints


def chain(joints: Mapping[str, Joint], root: str, link: str) -> list[Joint]:
    out = []
    while link != root:
        if link not in joints:
            raise ValueError(f"{link!r} does not hang below {root!r}")
        out.append(joints[link])
        link = joints[link].parent
    return out[::-1]


def fk(joints: Mapping[str, Joint], root: str, link: str, q_rad: Mapping[str, float]) -> np.ndarray:
    """4x4 pose of `link` in `root`, revolute joints at `q_rad` (missing = 0)."""
    t = np.eye(4)
    for j in chain(joints, root, link):
        step = np.eye(4)
        step[:3, :3] = _rpy(*j.rpy)
        step[:3, 3] = j.xyz
        t = t @ step
        if j.kind in ("revolute", "continuous"):
            rot = np.eye(4)
            rot[:3, :3] = _axis_angle(np.array(j.axis), float(q_rad.get(j.name, 0.0)))
            t = t @ rot
    return t


# --- motor degrees <-> Dream's IK coordinates ---------------------------------


@dataclass(frozen=True)
class JointMap:
    sign: dict
    offset_deg: dict
    status: dict
    pan_shift_deg: float = PAN_SHIFT_DEG

    @classmethod
    def load(cls, path: Path = MOTOR_TO_URDF, pan_shift_deg: float = PAN_SHIFT_DEG) -> JointMap:
        data = json.loads(Path(path).read_text())["joints"]
        sign = {m: float(data[m]["sign"]) for m in ARM}
        if any(s not in (1.0, -1.0) for s in sign.values()):
            raise ValueError("motor_to_urdf signs must be +1 or -1")
        return cls(
            sign,
            {m: float(data[m]["offset_deg"]) for m in ARM},
            {m: data[m]["status"] for m in ARM},
            pan_shift_deg,
        )

    def unverified(self) -> list[str]:
        """Joints whose sign was never hand-tested (J5 photo-inferred, J6 assumed)."""
        return [m for m in ARM if any(word in self.status[m].lower() for word in UNVERIFIED_MARKERS)]

    def urdf_deg(self, motor: Mapping[str, float]) -> dict:
        return {m: self.sign[m] * motor[m] + self.offset_deg[m] for m in ARM}

    def to_ik(self, motor: Mapping[str, float]) -> list[float]:
        urdf = self.urdf_deg(motor)
        urdf["shoulder_pan"] -= self.pan_shift_deg
        return [math.radians(urdf[m]) for m in ARM]

    def to_motor(self, ik_rad) -> dict:
        deg = [math.degrees(float(v)) for v in ik_rad]
        out = {}
        for m, value in zip(ARM, deg, strict=True):
            if m == "shoulder_pan":
                value += self.pan_shift_deg
            out[m] = (value - self.offset_deg[m]) / self.sign[m]
        return out

    def ik_limits_deg(self, motor_limits: Mapping[str, tuple]) -> dict:
        out = {}
        for m in ARM:
            ends = [self.sign[m] * v + self.offset_deg[m] for v in motor_limits[m]]
            if m == "shoulder_pan":
                ends = [v - self.pan_shift_deg for v in ends]
            out[m] = (min(ends), max(ends))
        return out


def grip_to_motor(grip: float) -> float:
    g = min(1.0, max(0.0, float(grip)))
    return GRIP_CLOSED_DEG + g * (GRIP_OPEN_DEG - GRIP_CLOSED_DEG)


def motor_to_grip(motor_deg: float) -> float:
    g = (float(motor_deg) - GRIP_CLOSED_DEG) / (GRIP_OPEN_DEG - GRIP_CLOSED_DEG)
    return min(1.0, max(0.0, g))


def ik_row_to_motor(row, jmap: JointMap) -> dict:
    """One Dream joint row (6 IK rad + normalized grip) -> seven motor degrees."""
    row = [float(v) for v in row]
    if len(row) != 7 or not all(math.isfinite(v) for v in row):
        raise ValueError(f"Dream row must be 7 finite numbers, got {row!r}")
    out = jmap.to_motor(row[:6])
    out["gripper"] = grip_to_motor(row[6])
    return out


def motor_to_ik_row(motor: Mapping[str, float], jmap: JointMap) -> list[float]:
    return [*jmap.to_ik(motor), motor_to_grip(motor["gripper"])]


# --- per-row limits, speed, jumps ---------------------------------------------


def clamp_to_limits(target: Mapping[str, float], limits: Mapping[str, tuple], margin_deg: float = 1.0):
    """Clip every joint into its limits shrunk by margin; returns (clipped, names clipped)."""
    out, clipped = {}, []
    for m in MOTORS:
        lo, hi = limits[m]
        lo, hi = lo + margin_deg, hi - margin_deg
        value = min(hi, max(lo, float(target[m])))
        if value != target[m]:
            clipped.append(m)
        out[m] = value
    return out, clipped


def jumps(previous: Mapping[str, float], target: Mapping[str, float], max_jump_deg: float) -> list[str]:
    """Arm joints whose target moved more than max_jump_deg from the last one (a J1 wrap, an IK flip)."""
    return [m for m in ARM if abs(target[m] - previous[m]) > max_jump_deg]


def retime(
    previous: Mapping[str, float],
    target: Mapping[str, float],
    *,
    arm_deg_s: float,
    grip_deg_s: float,
    dt: float = ROW_DT_S,
) -> list[dict]:
    """Split one row into equal 20 Hz substeps so no joint exceeds its speed cap.

    All joints arrive together (a straight joint-space line), which keeps the
    tank guard's straight-path proof valid; a per-joint clip would bend it.
    """
    if not (arm_deg_s > 0 and grip_deg_s > 0 and dt > 0):
        raise ValueError("speeds and dt must be positive")
    arm = max(abs(target[m] - previous[m]) for m in ARM) / (arm_deg_s * dt)
    grip = abs(target["gripper"] - previous["gripper"]) / (grip_deg_s * dt)
    n = max(1, math.ceil(max(arm, grip) - 1e-9))
    return [{m: previous[m] + (target[m] - previous[m]) * (i / n) for m in MOTORS} for i in range(1, n + 1)]


# --- route between the tank rest and the start pose ---------------------------

KEEPOUT_FILE = ROOT / "configs/viscous/environments/aquarium_landen_75/keepout_viscous_waterproof_01.json"
# Leg 1 of the approach (Gavin 2026-11-08: "you probably need to lift the elbow
# up first"): shoulder_lift up and elbow_flex folded by these, nothing else
# moving, lifts the forearm and hand nearly straight up off the floor (the
# lowest point rises about 2.3 mm per 0.9 mm outward at the TRACE_POSE rest),
# elbow ~110 mm higher, hand ~75 mm higher, the arm's lowest point ~95 mm
# above the floor bound, 3 s at 15 deg/s.
APPROACH_LIFT_DEG = 40.0
APPROACH_FOLD_DEG = 45.0
ROUTE_LIMIT_MARGIN_DEG = 1.0  # the runner's limit_margin_deg


def keepout_limits(path: Path = KEEPOUT_FILE) -> dict:
    """Motor joint limits (deg) from the tank keep-out config, the ones the guard clips to."""
    data = json.loads(Path(path).read_text())["joint_limits_deg"]
    return {m: tuple(map(float, v)) for m, v in data.get("value", data).items()}


def approach_waypoints(rest: Mapping[str, float], start: Mapping[str, float]) -> list[dict]:
    """Motor-degree waypoints (all seven motors) from rest to start: rest excluded, start included.
    Each consecutive pair (rest -> wp[0] -> ... -> start) is a straight joint-space leg.

    Two legs. Leg 1 lifts the elbow: shoulder_lift and elbow_flex move up to
    APPROACH_LIFT_DEG / APPROACH_FOLD_DEG toward the start, every other joint
    holds (the hand lies on the floor, so turning the pan or the wrist there
    would scrub it) except that the arm joints come 1 deg inside the keep-out's
    limits (the rest lies a few degrees outside some), gradually over the leg.
    Leg 2, in the air, goes straight to the start. Relative to the given rest, so a rest a few degrees
    off still lifts the same way. The gripper keeps the rest's value throughout.
    """
    limits = keepout_limits()
    lift = {"shoulder_lift": APPROACH_LIFT_DEG, "elbow_flex": APPROACH_FOLD_DEG}
    up = {}
    for m in ARM:
        value = float(rest[m])
        if m in lift:
            value += max(-lift[m], min(lift[m], float(start[m]) - value))
        lo, hi = limits[m]
        # A joint resting outside a soft limit comes back in during this leg, at the
        # leg's pace (the guard no longer snaps it): held at the rest instead, the
        # trace's first reading (pan 191.2, 0.66 mm off the left-wall margin) is refused.
        up[m] = min(hi - ROUTE_LIMIT_MARGIN_DEG, max(lo + ROUTE_LIMIT_MARGIN_DEG, value))
    grip = float(rest["gripper"])
    end = {**{m: float(start[m]) for m in ARM}, "gripper": grip}
    up["gripper"] = grip
    return [end] if up == end else [up, end]


def return_waypoints(rest: Mapping[str, float], start: Mapping[str, float]) -> list[dict]:
    """The same route back: from start to rest, start excluded, rest included (the approach reversed).

    The arm joints retrace approach_waypoints exactly; the gripper keeps the
    start's value throughout (the runner returns arm joints and leaves the grip).
    """
    grip = float(start["gripper"])
    back = [*approach_waypoints(rest, start)[-2::-1], rest]
    return [{**{m: float(wp[m]) for m in ARM}, "gripper": grip} for wp in back]


# --- workspace box -----------------------------------------------------------


@dataclass
class WorkspaceBox:
    """Axis-aligned boxes, per arm point, in the physical J1 base frame (mm).

    A coarse second layer under the vertex-level tank guard: it keeps the
    hand inside a task region and the wrist/elbow off the walls and rim.
    """

    points: dict  # name -> {"link": str, "offset_m": [x,y,z], "box_mm": {"x":[lo,hi],...}}
    urdf: dict
    jmap: JointMap
    root: str = "base_link"
    config: dict = field(default_factory=dict)

    @classmethod
    def load(cls, path: Path, urdf_path: Path, jmap: JointMap, tcp_m=None) -> WorkspaceBox:
        config = json.loads(Path(path).read_text())
        points = {}
        for name, p in config["points"].items():
            offset = tcp_m if p.get("offset") == "tcp" else p.get("offset_m", [0, 0, 0])
            if offset is None:
                raise ValueError(f"point {name} needs the arm file's tcp")
            points[name] = {"link": p["link"], "offset_m": list(map(float, offset)), "box_mm": p["box_mm"]}
        return cls(
            points, parse_urdf(Path(urdf_path).read_bytes()), jmap, config.get("root", "base_link"), config
        )

    def positions_mm(self, motor: Mapping[str, float]) -> dict:
        q = {}
        urdf_deg = self.jmap.urdf_deg(motor)
        for joint in self.urdf.values():
            if joint.name in urdf_deg:
                q[joint.name] = math.radians(urdf_deg[joint.name])
        out = {}
        for name, p in self.points.items():
            t = fk(self.urdf, self.root, p["link"], q)
            out[name] = (t[:3, :3] @ np.array(p["offset_m"]) + t[:3, 3]) * 1000.0
        return out

    def violations(self, motor: Mapping[str, float]) -> list[str]:
        bad = []
        for name, xyz in self.positions_mm(motor).items():
            for axis, value in zip("xyz", xyz, strict=True):
                lo, hi = self.points[name]["box_mm"][axis]
                if not lo <= value <= hi:
                    bad.append(f"{name}.{axis}={value:.0f}mm outside [{lo}, {hi}]")
        return bad


# --- trips on measured state ---------------------------------------------------


@dataclass
class TripLimits:
    lag_deg: float = 10.0  # the bench rule; 25 was far too late
    lag_ticks: int = 2
    torque_trip_nm: float = 5.0  # RS00 continuous rating, instantaneous
    torque_trip_ticks: int = 2
    torque_cap_nm: float = 3.5  # the dry-arm planned-load cap
    torque_cap_s: float = 2.0
    temp_warn_c: float = 70.0
    temp_trip_c: float = 80.0  # driver stops at 85, scripts abort below 82 and cool at 80

    def check(self) -> None:
        if not 0 < self.lag_deg <= 20:
            raise ValueError("lag trip must be in (0, 20] deg")
        if not 0 < self.torque_cap_nm <= self.torque_trip_nm <= 14:
            raise ValueError("need 0 < torque cap <= torque trip <= 14 Nm (RS00 peak)")
        if not 0 < self.temp_trip_c <= 82:
            raise ValueError("temperature trip must be at most 82 C")


@dataclass(frozen=True)
class Trip:
    name: str
    detail: str


class TripMonitor:
    """Per-tick checks of the measured arm against the commanded target.

    Clock injected; never sleeps. The gripper is left out of the lag trip (it
    stalls on whatever it holds) but not out of torque or temperature.
    """

    def __init__(self, limits: TripLimits, clock: Callable[[], float]):
        limits.check()
        self.limits = limits
        self.clock = clock
        self._lag = dict.fromkeys(ARM, 0)
        self._torque = dict.fromkeys(MOTORS, 0)
        self._over_cap_since: dict[str, float | None] = dict.fromkeys(MOTORS, None)
        self.peak_torque = dict.fromkeys(MOTORS, 0.0)
        self.peak_temp = dict.fromkeys(MOTORS, 0.0)
        self.peak_lag = dict.fromkeys(ARM, 0.0)
        self.warnings: list[str] = []

    def update(self, measured, commanded, torque, temp) -> list[Trip]:
        lim, now, trips = self.limits, self.clock(), []
        for name, values in (("position", measured), ("torque", torque), ("temperature", temp)):
            for m in MOTORS:
                v = values.get(m)
                if not isinstance(v, (int, float)) or not math.isfinite(v):
                    trips.append(Trip("bad_reading", f"{m} {name} is {v!r}"))
        if trips:
            return trips
        for m in ARM:
            lag = abs(measured[m] - commanded[m])
            self.peak_lag[m] = max(self.peak_lag[m], lag)
            self._lag[m] = self._lag[m] + 1 if lag > lim.lag_deg else 0
            if self._lag[m] >= lim.lag_ticks:
                trips.append(Trip("lag", f"{m} {lag:.1f} deg behind its target (> {lim.lag_deg})"))
        for m in MOTORS:
            tau = abs(torque[m])
            self.peak_torque[m] = max(self.peak_torque[m], tau)
            self._torque[m] = self._torque[m] + 1 if tau > lim.torque_trip_nm else 0
            if self._torque[m] >= lim.torque_trip_ticks:
                trips.append(Trip("torque", f"{m} {tau:.2f} Nm (> {lim.torque_trip_nm})"))
            if tau > lim.torque_cap_nm:
                since = self._over_cap_since[m] = (
                    self._over_cap_since[m] if self._over_cap_since[m] is not None else now
                )
                if now - since >= lim.torque_cap_s:
                    trips.append(
                        Trip("torque_sustained", f"{m} over {lim.torque_cap_nm} Nm for {now - since:.1f} s")
                    )
            else:
                self._over_cap_since[m] = None
            t = temp[m]
            self.peak_temp[m] = max(self.peak_temp[m], t)
            if t >= lim.temp_trip_c:
                trips.append(Trip("temperature", f"{m} at {t:.1f} C (>= {lim.temp_trip_c})"))
            elif t >= lim.temp_warn_c and f"{m}-hot" not in self.warnings:
                self.warnings.append(f"{m}-hot")
        return trips


# --- refusals ----------------------------------------------------------------


def tool_point_problem(arm: Mapping, urdf: bytes) -> str | None:
    """Mirror of dream.arm_file.tool_point_problem (dreamlabs 1.0.37), stricter:
    the record must carry the digest, since the tank URDF postdates digests."""
    if not any(arm.get("tcp", [])):
        return "its tool center point is the flange itself (tcp [0, 0, 0])"
    record = arm.get("tcp_calibration")
    if record is None:
        return "no tcp_calibration record says a person set its tool center point; Gavin runs `dreamcli tcp`"
    if not isinstance(record, dict) or not {"status", "at"} <= set(record) <= {"status", "at", "digest"}:
        return f"its tcp_calibration record is not one dreamcli writes: {record!r}"
    if record["status"] != "calibrated":
        return f"its tool point was not set (tcp_calibration status {record['status']!r})"
    if "digest" not in record:
        return "its tcp_calibration record has no digest, so it cannot prove it matches this URDF"
    tool = json.dumps({k: arm[k] for k in TOOL_KEYS}, sort_keys=True)
    if record["digest"] != sha256_bytes(urdf + b"\0" + tool.encode())[:16]:
        return f"its tcp, approach, jaw, base, flange or URDF changed since the tool point was set ({record['at']})"
    return None


def approval_problem(urdf: bytes, provenance: Mapping | None, base: str = "dream_base") -> str | None:
    """Whether Gavin approved THIS tank URDF (its sidecar, matched by normalized hash)."""
    if base != "dream_base":
        return (
            f"the arm file's base is {base!r}, not the tank model's 'dream_base': run dreamcli tcp on the "
            "approved tank model (make_tank_arm.py --export), not on the original URDF"
        )
    if provenance is None:
        return "no provenance sidecar for the arm file's URDF: it is not a generated tank model"
    approval = provenance.get("approval") or {}
    if approval.get("status") != "approved":
        return f"tank model approval is {approval.get('status')!r}, not 'approved' (Gavin edits the sidecar)"
    if not str(approval.get("by") or "").strip():
        return "tank model approval names no approver"
    if approval.get("urdf_sha256_normalized") != normalized_urdf_sha256(urdf):
        return "tank model approval is for a different URDF than the arm file's copy"
    return None


@dataclass
class LiveFacts:
    tcp_problem: str | None
    approval_problem: str | None
    unverified_signs: list
    acked_signs: list
    plan_id: str
    plan_id_given: str | None
    lab_holder: str | None
    port_holders: list
    missing_slots: list
    guard_problem: str | None
    dream_problem: str | None
    ledger_remaining: int
    max_calls: int
    arm_file_explicit: bool
    credentials_present: bool


def live_refusals(f: LiveFacts) -> list[str]:
    """Every reason --live must not start. Empty means it may proceed to the preflight."""
    out = []
    if not f.arm_file_explicit:
        out.append(
            "--live needs --arm-file naming the arm file `dreamcli tcp` wrote (no default is used live)"
        )
    if f.tcp_problem:
        out.append(f"tool point: {f.tcp_problem}")
    if f.approval_problem:
        out.append(f"arm model: {f.approval_problem}")
    missing = [m for m in f.unverified_signs if m not in f.acked_signs]
    if missing:
        out.append(
            "joint signs never hand-tested: "
            + ", ".join(missing)
            + " (motor_to_urdf.json); pass --ack-unverified-sign for each after reading the README"
        )
    if f.plan_id_given != f.plan_id:
        out.append(f"--plan-id must be {f.plan_id}, the id the dry run printed for exactly this plan")
    if f.lab_holder:
        out.append(f"the Lab is busy: {f.lab_holder}")
    if f.port_holders:
        out.append("the follower port is open in another process: " + ", ".join(f.port_holders))
    if f.missing_slots:
        out.append("no camera for Dream slot(s): " + ", ".join(f.missing_slots))
    if f.guard_problem:
        out.append(f"tank keep-out guard: {f.guard_problem}")
    if f.dream_problem:
        out.append(f"Dream client: {f.dream_problem}")
    if f.ledger_remaining < 1:
        out.append("the billed-call ledger has no calls left")
    if not 1 <= f.max_calls <= 20:
        out.append("--max-calls must be 1..20")
    if not f.credentials_present:
        out.append("no Dream credentials (~/.dreamlabs/credentials, 0600, written by Gavin)")
    return out


# --- billed-call ledger ---------------------------------------------------------


class BudgetEndError(Exception):
    pass


class Ledger:
    """Crash-safe call accounting (sweep.py's idea). A call is reserved BEFORE it is
    sent; an uncertain reservation counts as billed. `pool_warming` and
    `not_sent` (refused locally before any byte left) are free."""

    FREE = ("pool_warming", "not_sent")

    def __init__(self, path: Path, cap: int, clock: Callable[[], float] = time.time):
        self.path, self.cap, self.clock = Path(path), int(cap), clock
        self.lock_path = self.path.with_suffix(".lock")

    def _update(self, fn):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self.lock_path.open("a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            data = (
                json.loads(self.path.read_text()) if self.path.exists() else {"cap": self.cap, "requests": []}
            )
            # The file's cap wins: raising it is a deliberate edit, never an argument.
            self.cap = int(data["cap"])
            result = fn(data)
            data["budget_used"] = sum(r["status"] not in self.FREE for r in data["requests"])
            atomic_json(self.path, data)
            return result

    def remaining(self) -> int:
        if not self.path.exists():  # read-only: a refused run leaves no ledger behind
            return self.cap
        return self._update(lambda d: d["cap"] - sum(r["status"] not in self.FREE for r in d["requests"]))

    def reserve(self, run: str, run_cap: int) -> dict:
        def reserve(d):
            used = [r for r in d["requests"] if r["status"] not in self.FREE]
            mine = sum(r["run"] == run for r in used)
            if len(used) >= d["cap"] or mine >= run_cap:
                raise BudgetEndError(f"ledger {len(used)}/{d['cap']}, this run {mine}/{run_cap}")
            r = {
                "id": len(d["requests"]) + 1,
                "run": run,
                "call": mine + 1,
                "status": "reserved",
                "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(self.clock())),
            }
            d["requests"].append(r)
            return dict(r)

        return self._update(reserve)

    def finish(self, request_id: int, status: str) -> None:
        if status not in ("billed", *self.FREE):
            raise ValueError(status)
        self._update(lambda d: d["requests"][request_id - 1].update(status=status))


# --- images ------------------------------------------------------------------


def to_model_frame(rgb: np.ndarray) -> np.ndarray:
    """Centre-crop to 16:9 (640x480 -> rows 60:420, as the sim's top camera) and
    resize to Dream's 320x176. Input and output are RGB uint8 HxWx3."""
    from PIL import Image

    rgb = np.asarray(rgb)
    if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8:
        raise ValueError(f"frame must be HxWx3 uint8 RGB, got {rgb.shape} {rgb.dtype}")
    h, w = rgb.shape[:2]
    want_h = round(w * 9 / 16)
    if h > want_h:
        top = (h - want_h) // 2
        rgb = rgb[top : top + want_h]
    elif h < want_h:
        want_w = round(h * 16 / 9)
        left = (w - want_w) // 2
        rgb = rgb[:, left : left + want_w]
    return np.asarray(Image.fromarray(rgb).resize(FRAME_SIZE, Image.BILINEAR))


def jpeg_b64(rgb: np.ndarray) -> str:
    """Encode RGB with PIL (never cv2.imencode, which expects BGR and would tint skin blue)."""
    from PIL import Image

    buffer = io.BytesIO()
    Image.fromarray(np.asarray(rgb)).save(buffer, format="JPEG", quality=JPEG_QUALITY)
    return base64.b64encode(buffer.getvalue()).decode("ascii")


def black_frame() -> np.ndarray:
    return np.zeros((FRAME_SIZE[1], FRAME_SIZE[0], 3), np.uint8)


def slot_frames(frames_by_camera: Mapping[str, np.ndarray], slots: Mapping[str, str]) -> dict:
    """{video.<slot>: base64 JPEG} from camera frames and the slot map.

    A slot maps to a camera name, "black", or "copy:<slot>" (another slot's frame).
    """
    model = {}
    for slot in SLOTS:
        source = slots[slot]
        if source == "black":
            model[slot] = black_frame()
        elif source.startswith("copy:"):
            continue
        else:
            model[slot] = to_model_frame(frames_by_camera[source])
    for slot in SLOTS:
        if slots[slot].startswith("copy:"):
            model[slot] = model[slots[slot][5:]]
    return {f"video.{slot}": jpeg_b64(model[slot]) for slot in SLOTS}


def resolve_slots(camera_names, overhead: str | None, wrist_l: str | None, wrist_r: str) -> tuple[dict, list]:
    """Slot map and the slots no camera feeds. wrist_r: black | copy | camera:<name>."""
    names, slots, missing = set(camera_names), {}, []
    for slot, name in (("overhead", overhead), ("wrist_l", wrist_l)):
        if name and name in names:
            slots[slot] = name
        else:
            slots[slot] = "black"
            missing.append(
                f"{slot} (camera {name!r} is not in the record)" if name else f"{slot} (none chosen)"
            )
    if wrist_r == "black":
        slots["wrist_r"] = "black"
    elif wrist_r == "copy":
        slots["wrist_r"] = "copy:wrist_l"
    elif wrist_r.startswith("camera:") and wrist_r[7:] in names:
        slots["wrist_r"] = wrist_r[7:]
    else:
        slots["wrist_r"] = "black"
        missing.append(f"wrist_r (camera {wrist_r[7:]!r} is not in the record)")
    return slots, missing


def dream_state(measured: Mapping[str, float], jmap: JointMap, right_rest_rad, prompt: str) -> dict:
    """The state set_observation takes: the working arm on the LEFT, the right arm
    reported parked at the arm file's rest with its gripper open (the sim's
    parked arm), the prompt. Radians and a 0..1 grip, never degrees."""
    return {
        "state.left_arm": jmap.to_ik(measured),
        "state.left_grip": [motor_to_grip(measured["gripper"])],
        "state.right_arm": [float(v) for v in right_rest_rad],
        "state.right_grip": [1.0],
        "prompt": prompt,
    }
