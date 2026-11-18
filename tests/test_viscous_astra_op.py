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
"""Offline operator contract adapted from metal-arm-harness safety, calibration and control tests."""

import base64
import contextlib
import json
import subprocess
import sys
import threading
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

HERE = Path(__file__).resolve().parents[1] / "scripts/viscous-astra"
sys.path.insert(0, str(HERE))
from agent_bridge import Bridge  # noqa: E402
from model import DT, FAST, SLOW, Model, RefusalError, check_leg, core, real, travel  # noqa: E402
from session import Operator, ReplayArm  # noqa: E402
from stop_button import StopLogic  # noqa: E402
from transport import send  # noqa: E402


@pytest.fixture
def model(tmp_path):
    return Model(real.DEFAULT_ARM_FILE, real.TANK_ENVIRONMENT, tmp_path, core.keepout_limits())


def make_frames(folder):
    for name in ("head", "left"):
        (folder / name).mkdir(parents=True)
        Image.new("RGB", (8, 8), "red" if name == "head" else "blue").save(folder / name / "000.jpg")
    return folder


@pytest.fixture
def operator(model, tmp_path):
    arm = ReplayArm(model, make_frames(tmp_path / "source"), None, real.VirtualClock())
    op = Operator(
        arm,
        model,
        tmp_path,
        real.TANK_ENVIRONMENT,
        True,
        (threading.Event(), threading.Event()),
        tmp_path / "pid",
        40,
    )
    op.begin()
    yield op
    op.abort_event.set()  # release during teardown even when a test injected an unsafe pose
    op.end("completed")
    op.trace.close()


@pytest.mark.parametrize("use_yaw", [False, True])
def test_ik_round_trip_and_physical_guard_share_urdf(operator, use_yaw):
    m = operator.model
    q = {**operator.start_pose, "elbow_flex": 100, "wrist_yaw": -7}
    tip = m.pose(q)
    solved = m.solve(operator.start_pose, tip["xyz_m"], tip["pitch_deg"], tip["yaw_deg"] if use_yaw else None)
    actual = m.pose(solved)
    assert np.linalg.norm(np.subtract(actual["xyz_m"], tip["xyz_m"])) <= 0.002
    assert abs(actual["pitch_deg"] - tip["pitch_deg"]) <= 2
    if use_yaw:
        assert abs(actual["yaw_deg"] - tip["yaw_deg"]) <= 2
    from makermodslab import tank_geometry as geo

    transforms = geo.forward_kinematics(m.guard.model, q)
    r, t = transforms[m.spec["flange"]]
    assert np.allclose(np.diag([-1, -1, 1]) @ (r @ m.tcp + t), tip["xyz_m"], atol=1e-9)
    with pytest.raises(RefusalError, match="IK refused"):
        m.solve(q, [1.3, 0, 0.1], -90)


def test_late_wall_refusal_is_atomic_and_session_survives(operator, monkeypatch):
    q = operator.start_pose
    monkeypatch.setattr(operator.model, "solve", lambda *a, **kw: {**q, "shoulder_pan": 155.0})
    writes = []
    monkeypatch.setattr(operator.arm, "send", lambda *args: writes.append(args))
    refused = operator.reply(["tip", ".3", "0", ".1"])
    assert not refused["ok"] and "right" in refused["guard"]
    assert not writes and operator.commanded == q and operator.arm.energized
    assert operator.reply(["observe"])["ok"]


def test_post_limit_on_interpolated_segment(operator):
    q = operator.start_pose
    # Place the post just outside the current requirement, then turn the heel toward it.
    guard = operator.model.guard
    guard.planes["near"] -= guard.check_pose(q).slack_mm - 0.15
    for m in ("shoulder_pan", "shoulder_lift"):
        for d in (-6, 6):
            try:
                operator.model.plan(q, [{**q, m: q[m] + d}], q)
            except RefusalError as e:
                if "near" in str(e):
                    return
    pytest.fail("expected post refusal")


def test_slow_zone_recovery_speed_excursion_and_joint_limits(operator):
    m = operator.model
    a = operator.start_pose
    b = {**a, "shoulder_lift": a["shoulder_lift"] + 1}
    with pytest.raises(RefusalError, match="speed"):
        m.check_step(a, b, FAST, 100, 100, a)
    with pytest.raises(RefusalError, match="speed"):
        m.check_step(a, a, FAST, 75, 75, a)
    with pytest.raises(RefusalError, match="excursion"):
        check_leg(a, {**a, "wrist_roll": a["wrist_roll"] + 35.01})
    with pytest.raises(RefusalError, match="soft limit"):
        m.plan(a, [{**a, "gripper": a["gripper"] + 1}], a)
    a = m.solve(a, [0.29, 0, 0.01], -70)
    m.guard.planes["floor"] += m.clearance(a)["hand_floor_mm"] + 2
    low = m.clearance(a)["hand_floor_mm"]
    assert low == pytest.approx(-2)
    origin = a
    for sign in (1, -1):
        a = origin
        goal = {**a, "shoulder_lift": a["shoulder_lift"] + sign}
        if m.clearance(goal)["hand_floor_mm"] > low:
            steps = m.plan(a, [goal], a)
            for wp, _, speed, _ in steps:
                assert speed == SLOW and travel(a, wp) <= DT * SLOW + 1e-8
                a = wp
        else:
            with pytest.raises(RefusalError, match="floor"):
                m.plan(a, [goal], a)


def test_unarmed_has_no_actuation_paths_and_calibrates(tmp_path, model, monkeypatch):
    arm = ReplayArm(model, make_frames(tmp_path / "source"), None, real.VirtualClock())
    for name in ("engage", "send", "reanchor", "release_gripper", "return_to_rest"):
        monkeypatch.setattr(arm, name, lambda *_: pytest.fail("unarmed motor write"))
    # Read-only backend release is a close without any servo commands.
    monkeypatch.setattr(arm, "release", lambda: None)
    op = Operator(
        arm,
        model,
        tmp_path,
        real.TANK_ENVIRONMENT,
        False,
        (threading.Event(), threading.Event()),
        tmp_path / "pid",
        40,
    )
    try:
        op.begin()
        assert not op.reply(["open"])["ok"]  # the full 112-degree stroke exceeds the command cap
        q = op.reply(["gripper", "0.3"])
        assert q["ok"] and q["gripper"]["fraction"] == 0
        assert op.reply(["nudge", "up=0.008"])["ok"]
        assert op.commanded != arm.pos
        assert op.reply(["rest"])["ok"]
        arm.pos = model.solve(arm.pos, [0.29, 0, 0.01], -70)
        op.monitor.speed = core.SpeedMonitor(30, 2)  # hand-placed between samples
        for _ in range(2):
            assert op.reply(["calibrate-floor"])["ok"]
        assert (tmp_path / "floor.json").exists()
    finally:
        op.end("quit")
        op.trace.close()


@pytest.mark.parametrize("trip", ["lag", "torque", "temperature", "overspeed", "limp"])
def test_telemetry_trips_enter_stop(operator, trip):
    arm = operator.arm
    arm.scenario = trip
    arm.ticks = 72
    # A real limp forces the existing supported-release state. Acknowledge that
    # second stage on the injected Event so the fixture can finish offline.
    try:
        with pytest.raises(real.TripStopError, match=trip):
            for _ in range(90):
                operator.poll()
    finally:
        if trip == "limp":
            operator.abort_event.set()
        arm.scenario = "nominal"
        if trip == "overspeed":
            operator.abort_event.set()


@pytest.mark.parametrize("tcp,button", [(False, True), (True, False)])
def test_socket_frames_reply_budget_stop_and_button(tmp_path, tcp, button):
    root = tmp_path / "runs"
    args = [
        sys.executable,
        str(HERE / "astra_op.py"),
        "serve",
        "--backend",
        "fake",
        "--armed",
        "--frames",
        str(make_frames(tmp_path / "source")),
        "--run-root",
        str(root),
        "--pid-file",
        str(tmp_path / "pid"),
        "--budget",
        "4",
    ]
    if tcp:
        args.extend(["--tcp", "127.0.0.1:0"])
    else:
        # macOS sockaddr_un.sun_path is only 104 bytes.
        import tempfile

        sock = Path(tempfile.gettempdir()) / f"astra-{tmp_path.stat().st_ino}.sock"
        args.extend(["--socket", str(sock)])
    with subprocess.Popen(args, stdout=subprocess.PIPE, text=True) as proc:
        line = proc.stdout.readline()
        assert line.startswith("READY ")
        folder = Path(line.strip()[6:])
        bridge = Bridge(folder)
        try:
            r = json.loads(bridge.call("op", {"command": "observe"})[0]["text"])
            for k in ("joints_motor_deg", "tip", "gripper", "clearance", "guard", "frames", "urdf_sha256"):
                assert k in r
            assert set(r["frames"]) == {"head", "left"} and r["ok"]
            with pytest.raises(ValueError, match="pending JPEG"):
                bridge.call("op", {"command": "observe"})
            images = bridge.call("frames", {})
            for camera, block in zip(("head", "left"), images[1::2], strict=True):
                assert base64.b64decode(block["data"]) == Path(r["frames"][camera]).read_bytes()
            error = send(folder, ["nudge", "up=nan"])
            assert not error["ok"] and error["frames"] != r["frames"]
            assert send(folder, ["nudge", "up=0.005"])["ok"]
            if button:
                assert "STOP sent" in StopLogic(tmp_path / "pid").press()
            else:
                denied = send(folder, ["open"])
                assert not denied["ok"] and denied["stop"]["reason"] == "budget"
        finally:
            if proc.poll() is None and not (folder / "stop.json").exists() and not button:
                with contextlib.suppress(OSError):
                    send(folder, ["quit"])
            proc.wait(timeout=10)
    stop = json.loads((folder / "stop.json").read_text())
    assert stop["returned"] == "at the rest" and stop["release_problems"] is None
    events = stop["fake_events"]
    stages = [
        next(i for i, e in enumerate(events) if e.startswith(s))
        for s in ("re-anchor", "release gripper", "return leg", "release:")
    ]
    assert stages == sorted(stages)
    assert not (tmp_path / "pid").exists()


def test_passive_lab_lifecycle_only_connects_bus_without_handshake_and_reads(tmp_path, monkeypatch):
    from types import SimpleNamespace

    from session import PassiveLabArm

    from lerobot import robots
    from makermodslab import remote_can
    from makermodslab.arms import registry

    events = []
    bus = SimpleNamespace(
        connect=lambda *, handshake: events.append(("connect", handshake)),
        disconnect=lambda disable: events.append(("close", disable)),
    )
    record = {
        "follower_port": "fixture",
        "follower_config": "fixture",
        "cameras": [{"name": "head"}, {"name": "left"}],
    }
    config = SimpleNamespace(joint_limits=core.keepout_limits())
    device = SimpleNamespace(bus=bus, config=config)
    family = SimpleNamespace(
        single_follower_config=lambda port, profile: config,
        _read_fresh_positions=lambda robot: real.TRACE_POSE,
    )
    monkeypatch.setattr(robots, "make_robot_from_config", lambda config: device)
    monkeypatch.setattr(registry, "get", lambda name: family)
    cameras = {k: np.zeros((2, 2, 3), dtype=np.uint8) for k in ("left", "head")}
    monkeypatch.setattr(
        real,
        "open_cameras",
        lambda record, fake: (list(cameras), lambda: cameras, lambda: events.append("cameras closed")),
    )
    observation = {
        f"{m}.{k}": v
        for m in core.MOTORS
        for k, v in (("pos", real.TRACE_POSE[m]), ("torque", 0), ("temp", 30))
    }
    monkeypatch.setattr(remote_can, "get_observation", lambda device: observation)
    passive = PassiveLabArm(tmp_path, record, 15)
    passive.connect()
    assert passive.read().pos == real.TRACE_POSE
    assert passive.read().frames.keys() == cameras.keys()
    passive.release()
    assert events == [("connect", False), "cameras closed", ("close", False)]


def test_floor_touch_disagreement_is_rejected(operator):
    op = operator
    op.armed = False
    op.arm.energized = False
    before = op.model.guard.planes["floor"]
    for z in (0.01, 0.025):
        op.arm.pos = op.model.solve(op.arm.pos, [0.29, 0, z], -70)
        op.monitor.speed = core.SpeedMonitor(30, 2)
        reply = op.reply(["calibrate-floor"])
    assert not reply["ok"] and "touches differ" in reply["guard"]
    assert op.model.guard.planes["floor"] == before
    assert not (op.folder / "floor.json").exists()


def test_travel_sums_joints_and_legs_before_writing_and_exempts_rest(operator, monkeypatch):
    q = operator.start_pose
    operator.max_command_travel_deg = 4.0
    both = {**q, "wrist_yaw": q["wrist_yaw"] + 2.1, "wrist_roll": q["wrist_roll"] + 2}
    roll = {**q, "wrist_roll": q["wrist_roll"] + 3}
    with monkeypatch.context() as patch:
        patch.setattr(operator.arm, "send", lambda *_: pytest.fail("wrote a goal before refusal"))
        for goals in ([both], [roll, q], [{**q, "gripper": q["gripper"] - 5}]):
            with pytest.raises(RefusalError, match="command travel"):
                operator.execute(goals)
    operator.execute([roll])
    operator.max_command_travel_deg = 1.0
    assert operator.reply(["rest"])["ok"] and operator.commanded == q


def test_projected_floor_refusal_while_measured_hand_is_still_above_glass(operator, monkeypatch):
    model = operator.model
    q = model.solve(operator.start_pose, [0.29, 0, 0.01], -70)

    # Put the floor midway between the sagging hand and its projected endpoint.
    def lower(q, amount):
        return min(
            ({**q, "shoulder_lift": q["shoulder_lift"] + sign * amount} for sign in (-1, 1)),
            key=lambda goal: model.clearance(goal)["hand_floor_mm"],
        )

    measured = lower(q, 2)
    goal, projected = lower(q, 1), lower(measured, 1)
    floor_shift = (
        model.clearance(measured)["hand_floor_mm"] + model.clearance(projected)["hand_floor_mm"]
    ) / 2
    model.guard.planes["floor"] += floor_shift
    assert model.clearance(measured)["hand_floor_mm"] > 0 > model.clearance(projected)["hand_floor_mm"]
    assert model.clearance(goal)["hand_floor_mm"] > 0
    monkeypatch.setattr(operator, "commanded", q)
    monkeypatch.setattr(operator.reading, "pos", measured)
    monkeypatch.setattr(operator, "heartbeat", lambda: None)
    monkeypatch.setattr(operator.arm, "send", lambda *_: pytest.fail("wrote a goal before refusal"))
    observation = operator.observation()
    assert observation["sag_mm"] == pytest.approx(
        model.clearance(q)["hand_floor_mm"] - model.clearance(measured)["hand_floor_mm"]
    )
    assert observation["sag_mm"] > 0
    with pytest.raises(RefusalError, match="standing error projects hand"):
        operator.execute([goal])
