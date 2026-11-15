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
"""Dream lifecycle gates and file serving. All subprocesses and approvals are fakes."""

import importlib
import json
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from makermodslab import dream, session_events, sessions
from makermodslab.api_errors import ApiError
from makermodslab.arms import registry
from makermodslab.schemas.dream import DreamInputsApproval, DreamPlanBody, DreamSettings
from makermodslab.schemas.sessions import SessionStartBody
from makermodslab.utils import config as cfg


@pytest.fixture(autouse=True)
def isolated(tmp_lerobot_home, tmp_path, monkeypatch):
    monkeypatch.setattr(cfg, "DREAM_RUN_DATA", tmp_path / "run-data")
    monkeypatch.setattr(cfg, "DREAM_REPLAY", None)
    for name, value in {
        "dream_active": False,
        "checking_inputs": False,
        "_proc": None,
        "_pid_file": None,
        "_button": None,
        "_thread": None,
        "_term_sent": False,
        "_stop_requested": False,
        "_error": None,
        "_latest_run": None,
        "_latest_robot": None,
        "_plans": {},
        "_previews": {},
    }.items():
        monkeypatch.setattr(dream, name, value)
    sessions.tracker.reset()
    yield
    sessions.tracker.reset()


@pytest.fixture
def robot():
    record = {
        "name": "tank",
        "arm_type": "viscous",
        "mode": "single",
        "follower_port": "/fake",
        "follower_config": "viscous_01",
        "cameras": [],
    }
    Path(cfg.ROBOTS_PATH, "tank.json").write_text(json.dumps(record))
    return cfg.get_robot_record("tank", resolve_usb=False)


def test_capability_is_viscous_only():
    assert {family.id for family in registry.families() if family.supports_dream} == {"viscous"}


@pytest.mark.parametrize("kind", ["so101", "maker", "metal"])
def test_other_families_refused_before_any_subprocess(robot, monkeypatch, kind):
    record = dict(robot, arm_type=kind)
    monkeypatch.setattr(cfg, "get_robot_record", lambda *a, **kw: record)
    run = Mock(side_effect=AssertionError("must not launch"))
    monkeypatch.setattr(dream, "_run", run)
    with pytest.raises(ApiError, match="Viscous"):
        dream.plan(DreamPlanBody(robot="tank"))
    run.assert_not_called()


@pytest.mark.parametrize(
    "module,attr,value,kind",
    [
        ("teleoperate", "teleoperation_active", True, "teleoperation"),
        ("record", "recording_active", True, "recording"),
        ("rollout", "inference_active", True, "inference"),
        ("remote_inference", "remote_inference_active", True, "remote_inference"),
        ("replay", "replay_active", True, "replay"),
        ("calibrate", "calibration_is_active", lambda: True, "calibration"),
        ("auto_calibrate", "auto_calibration_is_active", lambda: True, "auto_calibration"),
        ("wiggle", "wiggle_active", True, "wiggle"),
        ("remote_host", "hosting_active", True, "hosting"),
        ("remote_teleoperate", "remote_teleoperation_active", True, "remote_teleoperation"),
        ("jobs", "training_is_active", lambda: "training job", "training"),
    ],
)
def test_dream_refuses_every_peer_before_claim(monkeypatch, module, attr, value, kind):
    monkeypatch.setattr(importlib.import_module(f"makermodslab.{module}"), attr, value)
    with pytest.raises(ApiError) as error:
        dream.handle_start_dream(dream.DreamRequest("not-even-resolved", "0123456789"))
    assert error.value.code == f"robot.busy.{kind}"
    assert not dream.dream_active


@pytest.mark.parametrize(
    "module,handler",
    [
        ("teleoperate", "handle_start_teleoperation"),
        ("record", "handle_start_recording"),
        ("rollout", "handle_start_inference"),
        ("replay", "handle_start_replay"),
        ("remote_host", "handle_start_hosting"),
        ("remote_teleoperate", "handle_start_remote_teleoperation"),
        ("remote_inference", "handle_start_remote_inference"),
    ],
)
def test_peer_reciprocal_early_refusal(monkeypatch, module, handler):
    monkeypatch.setattr(dream, "dream_active", True)
    # A bare request proves the gate runs before anything touches hardware.
    result = getattr(importlib.import_module(f"makermodslab.{module}"), handler)(SimpleNamespace())
    assert result["code"] == "robot.busy.dream"


def test_calibration_training_and_preview_reciprocal_gates(monkeypatch, client):
    monkeypatch.setattr(dream, "dream_active", True)
    from makermodslab import jobs, step_calibrate

    assert step_calibrate._other_feature_busy()["code"] == "robot.busy.dream"
    assert jobs.job_registry._robot_busy() == "a Dream session"
    # Must refuse BEFORE camera identity resolution or cv2 access.
    response = client.get("/api/v1/camera-preview/0")
    assert response.status_code == 409
    assert response.json()["code"] == "robot.busy.dream"


def test_plan_uses_live_settings_and_preserves_movement_verbatim(robot, monkeypatch):
    calls = []
    movement = "Joints that may move (ALL SEVEN): motor limits / commanded window / IK limits / sign\n  exact spacing\n"

    def run(args, **kwargs):
        calls.append(args)
        Path(args[args.index("--lab-report-file") + 1]).write_text(
            json.dumps({"plan_id": "0123456789", "movement": movement, "readiness": ["unapproved inputs"]})
        )

    monkeypatch.setattr(dream, "_run", run)
    body = DreamPlanBody(
        robot="tank",
        prompt="pick up the screwdriver",
        overhead="head",
        wrist_l="left",
        wrist_r="copy",
        max_calls=6,
        pause_s=10,
    )
    result = dream.plan(body)
    assert result == {"plan_id": "0123456789", "movement": movement, "readiness": ["unapproved inputs"]}
    args = calls[0]
    assert "--dry-run" in args and "--live" not in args
    for flag, value in [
        ("--max-calls", "6"),
        ("--pause-s", "10.0"),
        ("--overhead", "head"),
        ("--wrist-l", "left"),
        ("--wrist-r", "copy"),
        ("--prompt", body.prompt),
    ]:
        assert args[args.index(flag) + 1] == value
    assert dream._plans[result["plan_id"]]["settings"].model_dump() == body.model_dump(exclude={"robot"})


def test_start_waits_for_attributed_leased_session(robot, monkeypatch):
    request = dream.DreamRequest("tank", "0123456789")
    settings = DreamSettings(overhead="head", wrist_l="left")
    dream._plans[request.plan_id] = {
        "robot": "tank",
        "settings": settings,
        "record": dream._digest(robot),
        "created": time.monotonic(),
    }
    thread = Mock()
    monkeypatch.setattr(dream.threading, "Thread", Mock(return_value=thread))
    monkeypatch.setattr(sessions, "is_robot_record_clean", lambda *a, **k: True)
    monkeypatch.setattr(sessions, "_ensure_watchdog", lambda: None)
    monkeypatch.setitem(sessions._REQUEST_BUILDERS, "dream", lambda *a: request)

    def before_gate():
        assert dream.dream_active
        assert sessions.tracker.current()["kind"] == "dream"
        assert not request.start_gate.is_set()

    thread.start.side_effect = before_gate
    result = sessions.handle_start_session(
        SessionStartBody(kind="dream", robot="tank", owner="browser", options={"plan_id": request.plan_id})
    )
    assert request.start_gate.is_set()
    assert result["session"]["lease"]["owner"] == "browser"
    assert result["session"]["robot"] == "tank"
    assert request.plan_id not in dream._plans


def test_changed_or_expired_plan_cannot_start(robot):
    dream._plans["0123456789"] = {
        "robot": "tank",
        "record": "changed",
        "created": time.monotonic(),
        "settings": DreamSettings(),
    }
    with pytest.raises(ApiError, match="changed"):
        dream.handle_start_dream(dream.DreamRequest("tank", "0123456789"))
    assert not dream.dream_active
    dream._plans["0123456789"]["created"] = 0
    with pytest.raises(ApiError, match="expired"):
        dream.handle_start_dream(dream.DreamRequest("tank", "0123456789"))


def test_repeated_stop_and_lease_expiry_only_send_one_term(monkeypatch, tmp_path):
    proc = Mock()
    proc.poll.return_value = None
    monkeypatch.setattr(dream, "_proc", proc)
    monkeypatch.setattr(dream, "dream_active", True)
    monkeypatch.setattr(dream, "_pid_file", tmp_path / "runner.pid")
    session_events.notify_session_changed("dream", True, "running")
    snap = sessions.tracker.attach_lease("dream", "browser", 10)
    dream.handle_stop_dream()
    # Simulate another phase hint while returning: signal idempotence must
    # hold even if the lease watchdog sees a non-canonical runner phase.
    session_events.notify_session_changed("dream", True, "return leg 1/3")
    sessions.check_expiry(snap["lease"]["deadline"] + 1)
    dream.handle_stop_dream()
    dream.stop_for_shutdown()
    assert len(list(tmp_path.glob("runner.pid.press/*.json"))) == 1
    proc.send_signal.assert_not_called()
    assert dream.dream_active  # no release until child cleanup
    proc.kill.assert_not_called()
    proc.terminate.assert_not_called()


def test_stop_before_spawn_prevents_hardware_launch(monkeypatch, tmp_path):
    request = dream.DreamRequest("tank", "0123456789")
    request.start_gate.set()
    monkeypatch.setattr(dream, "dream_active", True)
    session_events.notify_session_changed("dream", True, "starting")
    popen = Mock(side_effect=AssertionError("must not launch"))
    monkeypatch.setattr(dream.subprocess, "Popen", popen)
    dream.handle_stop_dream()
    dream._worker(request, DreamSettings(), "live-test", tmp_path)
    popen.assert_not_called()
    assert not dream.dream_active and sessions.tracker.current() is None


def test_missing_display_button_refuses_before_runner_launch(monkeypatch, tmp_path):
    from makermodslab.camera_preview import camera_preview_manager

    request = dream.DreamRequest("tank", "0123456789")
    request.start_gate.set()
    monkeypatch.setattr(dream, "dream_active", True)
    session_events.notify_session_changed("dream", True, "starting")
    monkeypatch.setattr(camera_preview_manager, "stop_all", lambda: None)
    button = Mock()
    button.poll.return_value = 1
    popen = Mock(return_value=button)
    monkeypatch.setattr(dream.subprocess, "Popen", popen)
    dream._worker(request, DreamSettings(), "live-test", tmp_path)
    assert popen.call_count == 1
    assert "stop_button.py" in popen.call_args.args[0][1]
    assert "Start it by hand" in dream._error
    assert "arm has not been started" in dream._error
    assert not dream.dream_active and sessions.tracker.current() is None


def test_camera_check_reserves_and_releases_previews_before_child(robot, monkeypatch):
    from makermodslab.camera_preview import camera_preview_manager

    order = []
    monkeypatch.setattr(camera_preview_manager, "stop_all", lambda: order.append("release"))
    core, module = dream._helpers()

    def run(args, **kwargs):
        assert order == ["release"] and dream.checking_inputs
        assert "--preview" in args and "--live" not in args
        view = module.DreamView(
            Path(args[args.index("--dream-view-dir") + 1]),
            run="preview-test",
            mode="preview",
            prompt="x",
            max_calls=10,
            slots=dict.fromkeys(core.SLOTS, "black"),
        )
        view.observation(
            {f"video.{s}": core.jpeg_b64(core.black_frame()) for s in core.SLOTS},
            inputs_id="abcdef123456",
            inputs_checked=False,
        )

    monkeypatch.setattr(dream, "_run", run)
    preview = dream.preview(DreamPlanBody(robot="tank"))
    assert preview["inputs_id"] == "abcdef123456"
    assert not dream.checking_inputs
    assert len(dream._previews) == 1


def test_preview_failure_cleans_reservation(robot, monkeypatch):
    from makermodslab.camera_preview import camera_preview_manager

    monkeypatch.setattr(camera_preview_manager, "stop_all", lambda: None)
    monkeypatch.setattr(dream, "_run", Mock(side_effect=RuntimeError("camera refused")))
    with pytest.raises(RuntimeError):
        dream.preview(DreamPlanBody(robot="tank"))
    assert not dream.checking_inputs
    assert "camera refused" in dream._error


def test_approval_is_explicit_bound_to_a_recent_preview(robot, monkeypatch):
    token = "a" * 32
    dream._previews[token] = {
        "inputs_id": "abcdef123456",
        "robot": "tank",
        "record": dream._digest(robot),
        "created": time.monotonic(),
    }
    run = Mock()
    monkeypatch.setattr(dream, "_run", run)
    with pytest.raises(ApiError):
        dream.approve_inputs(DreamInputsApproval(preview_id=token, inputs_id="111111111111"))
    run.assert_not_called()
    assert dream.approve_inputs(DreamInputsApproval(preview_id=token, inputs_id="abcdef123456"))["approved"]
    assert run.call_args.args[0] == [
        str(cfg.DREAM_VENV / "bin/python"),
        "-m",
        "dream.input_check",
        "abcdef123456",
    ]
    assert token not in dream._previews
    with pytest.raises(ApiError):
        dream.approve_inputs(DreamInputsApproval(preview_id=token, inputs_id="abcdef123456"))
    assert run.call_count == 1


def test_status_and_image_routes_are_read_only_no_store(client, monkeypatch):
    core, module = dream._helpers()
    folder = dream.run_root() / "preview-test/dream_view"
    view = module.DreamView(
        folder,
        run="preview-test",
        mode="preview",
        prompt="x",
        max_calls=10,
        slots=dict.fromkeys(core.SLOTS, "black"),
    )
    sent = {f"video.{s}": core.jpeg_b64(core.black_frame()) for s in core.SLOTS}
    view.observation(sent)
    monkeypatch.setattr(dream, "_latest_run", "preview-test")
    monkeypatch.setattr(dream.subprocess, "Popen", Mock(side_effect=AssertionError("no subprocess")))
    r = client.get("/api/v1/dream/status")
    assert r.status_code == 200 and r.headers["cache-control"] == "no-store"
    r = client.get("/api/v1/dream/view/preview-test/overhead")
    assert r.headers["content-type"] == "image/jpeg"
    assert r.headers["cache-control"] == "no-store"
    assert r.content == (folder / "overhead.jpg").read_bytes()
    assert client.get("/api/v1/dream/view/preview-test/not-a-slot").status_code == 404
    assert client.get("/dream/status").status_code == 404
    outside = folder.parent.parent / "outside"
    outside.mkdir()
    (dream.run_root() / "evil").symlink_to(outside, target_is_directory=True)
    assert client.get("/api/v1/dream/view/evil/overhead").status_code == 404
    (folder / "overhead.jpg").unlink()
    (folder / "overhead.jpg").symlink_to(outside / "anything")
    assert client.get("/api/v1/dream/view/preview-test/overhead").status_code == 404


@pytest.fixture
def replay_bundle(tmp_path, monkeypatch):
    import numpy as np
    from PIL import Image

    source = tmp_path / "run-data" / "live-test"
    (source / "frames/head").mkdir(parents=True)
    (source / "run").mkdir()
    for index in range(2):
        rgb = np.zeros((48, 64, 3), dtype=np.uint8)
        rgb[:, :, index] = np.arange(48)[:, None] * 5
        Image.fromarray(rgb).save(source / f"frames/head/{index}.jpg")
    (source / "frames/index.jsonl").write_text(
        "\n".join(json.dumps({"t": 104.7 + i, "frames": {"head": f"head/{i}.jpg"}}) for i in range(2))
    )
    (source / "console.log").write_text(
        "[0.0 s] phase: connect\n[4.7 s] phase: approach - 2 legs\n[11.1 s] phase: pause - 10 s\n[11.1 s] phase: dream - ready\n[59.0 s] phase: release all torque\n"
    )
    (source / "ticks.jsonl").write_text(
        json.dumps(
            {
                "t": 115,
                "pos": {"wrist_roll": 2},
                "cmd": {"wrist_roll": 3},
                "trips": [],
                "torque": {"wrist_roll": 1},
                "temp": {"wrist_roll": 22},
            }
        )
    )
    (source / "chunks.jsonl").write_text("")
    (source / "run/plan.json").write_text(
        json.dumps(
            {
                "prompt": "test task",
                "dream_slots": {"overhead": "head (rotation 180)", "wrist_l": "black", "wrist_r": "black"},
                "speed": {"max_calls": 10},
                "pause_s": 10,
            }
        )
    )
    monkeypatch.setattr(cfg, "DREAM_REPLAY", source)
    return source


def test_replay_uses_saved_frames_and_same_crop_without_claiming(client, monkeypatch, replay_bundle):
    import base64

    import numpy as np
    from PIL import Image

    monkeypatch.setattr(dream.subprocess, "Popen", Mock(side_effect=AssertionError("replay must not spawn")))
    response = client.get("/api/v1/dream/replay?at_s=15")
    assert response.status_code == 200, response.text
    r = response.json()
    assert r["status"]["mode"] == "replay"
    assert r["status"]["phase"].startswith("pause")
    assert not dream.dream_active and sessions.tracker.current() is None
    frames, _, _, _, _, slots, start, _ = dream._load_replay(str(cfg.DREAM_REPLAY))
    row = [x for x in frames if x["t"] <= start + 15][-1]
    raw = {
        name: np.asarray(Image.open(cfg.DREAM_REPLAY / "frames" / path).convert("RGB"))
        for name, path in row["frames"].items()
    }
    core, _ = dream._helpers()
    encoded = core.slot_frames(raw, slots)
    for slot in core.SLOTS:
        image = client.get(f"/api/v1/dream/view/{r['run']}/{slot}")
        assert image.content == base64.b64decode(encoded[f"video.{slot}"])
    assert client.get("/api/v1/dream/replay?at_s=nan").status_code == 422


@pytest.mark.parametrize("reserved", ["dream_active", "checking_inputs"])
def test_all_calibration_and_wiggle_paths_respect_dream(monkeypatch, reserved):
    import asyncio

    from makermodslab import auto_calibrate, calibrate, can_wiggle, wiggle

    monkeypatch.setattr(dream, reserved, True)
    assert calibrate.CalibrationManager().start_calibration(SimpleNamespace())["code"] == "robot.busy.dream"
    assert auto_calibrate._AutoCalArmRunner().start(SimpleNamespace())["code"] == "robot.busy.dream"
    assert auto_calibrate.AutoCalibrationBatchManager().start(SimpleNamespace())["code"] == "robot.busy.dream"
    assert can_wiggle._busy_refusal()["code"] == "robot.busy.dream"
    assert asyncio.run(wiggle.wiggle_gripper("/fake"))["code"] == "robot.busy.dream"


def test_button_or_runner_return_already_in_flight_cannot_release_all(monkeypatch):
    proc = Mock()
    proc.poll.return_value = None
    monkeypatch.setattr(dream, "_proc", proc)
    monkeypatch.setattr(dream, "dream_active", True)
    for state in ("returning", "holding", "releasing", "released"):
        monkeypatch.setattr(dream, "_term_sent", False)
        monkeypatch.setattr(dream, "_read_view", lambda run, state=state: {"state": state})
        dream.handle_stop_dream()
    proc.send_signal.assert_not_called()


def test_lease_attribution_failure_cancels_before_releasing_the_launch_gate(robot, monkeypatch):
    request = dream.DreamRequest("tank", "0123456789")
    dream._plans[request.plan_id] = {
        "robot": "tank",
        "settings": DreamSettings(),
        "record": dream._digest(robot),
        "created": time.monotonic(),
    }
    monkeypatch.setattr(dream.threading, "Thread", Mock(return_value=Mock()))
    monkeypatch.setattr(sessions, "is_robot_record_clean", lambda *a, **k: True)
    monkeypatch.setitem(sessions._REQUEST_BUILDERS, "dream", lambda *a: request)
    monkeypatch.setattr(sessions.tracker, "attribute", Mock(side_effect=RuntimeError("attribution failed")))
    with pytest.raises(RuntimeError, match="attribution failed"):
        sessions.handle_start_session(
            SessionStartBody(
                kind="dream", robot="tank", owner="browser", options={"plan_id": request.plan_id}
            )
        )
    assert request.start_gate.is_set() and dream._stop_requested


def test_replay_default_uses_run_data_and_never_changes_live_pointer(client, monkeypatch, replay_bundle):
    monkeypatch.setattr(cfg, "DREAM_REPLAY", None)
    monkeypatch.setattr(dream, "_latest_run", "live-other")
    response = client.get("/api/v1/dream/replay?at_s=15")
    assert response.status_code == 200, response.text
    assert dream._latest_run == "live-other"
    assert dream._bundle(response.json()["run"]).is_relative_to(dream.run_root())


@pytest.mark.parametrize("method", ["construct", "start"])
def test_failed_supervisor_creation_releases_claim(robot, monkeypatch, method):
    request = dream.DreamRequest("tank", "0123456789")
    dream._plans[request.plan_id] = {
        "robot": "tank",
        "settings": DreamSettings(),
        "record": dream._digest(robot),
        "created": time.monotonic(),
    }
    factory = Mock()
    if method == "construct":
        factory.side_effect = RuntimeError("no worker")
    else:
        factory.return_value.start.side_effect = RuntimeError("no worker")
    monkeypatch.setattr(dream.threading, "Thread", factory)
    with pytest.raises(RuntimeError, match="no worker"):
        dream.handle_start_dream(request)
    assert not dream.dream_active
    assert sessions.tracker.current() is None
    assert sessions.tracker.last_ended()["phase"] == "error"
    assert "no worker" in dream._error


@pytest.mark.parametrize("change", [{"arms": "leader"}, {"mode": "bimanual"}])
def test_dream_requires_one_local_follower(robot, monkeypatch, change):
    monkeypatch.setattr(cfg, "get_robot_record", lambda *a, **k: dict(robot, **change))
    launch = Mock(side_effect=AssertionError("must not launch"))
    monkeypatch.setattr(dream, "_run", launch)
    with pytest.raises(ApiError, match="single Viscous follower"):
        dream.plan(DreamPlanBody(robot="tank"))
    launch.assert_not_called()


def test_stop_button_spawn_failure_explains_manual_recovery_without_launching_runner(monkeypatch, tmp_path):
    from makermodslab.camera_preview import camera_preview_manager

    request = dream.DreamRequest("tank", "0123456789")
    request.start_gate.set()
    monkeypatch.setattr(dream, "dream_active", True)
    session_events.notify_session_changed("dream", True, "starting")
    monkeypatch.setattr(camera_preview_manager, "stop_all", lambda: None)
    spawn = Mock(side_effect=OSError("cannot spawn button"))
    monkeypatch.setattr(dream.subprocess, "Popen", spawn)
    dream._worker(request, DreamSettings(), "live-test", tmp_path)
    assert spawn.call_count == 1
    assert "Start it by hand" in dream._error and "arm has not been started" in dream._error
    assert not dream.dream_active and sessions.tracker.current() is None


@pytest.mark.parametrize("order", ["page-key", "page-click", "button-page", "all-sources"])
def test_remote_stop_sequences_use_real_button_logic(monkeypatch, tmp_path, order):
    import signal

    import stop_button

    pid_file = tmp_path / "runner.pid"
    pid_file.write_text("4242")
    now, sent = [0.0], []
    logic = stop_button.StopLogic(
        pid_file,
        command=lambda pid: "python dream_real.py",
        clock=lambda: now[0],
        kill=lambda pid, sig: sent.append((pid, sig)),
    )
    monkeypatch.setattr(dream, "_pid_file", pid_file)
    monkeypatch.setattr(dream, "dream_active", True)
    session_events.notify_session_changed("dream", True, "running")
    lease = sessions.tracker.attach_lease("dream", "browser", 10)
    if order == "button-page":
        logic.press(from_key=True)
    if order == "all-sources":
        sessions.check_expiry(lease["lease"]["deadline"] + 1)
    dream.handle_stop_dream()
    if order == "all-sources":
        dream.stop_for_shutdown()
    messages = logic.poll_remote()
    assert len(messages) == 1 and "REMOTE" in messages[0]
    assert sent == [(4242, signal.SIGTERM)]
    logic.press(from_key=True)
    assert len(sent) == 1
    if order == "page-click":
        logic.press()  # double click still locked out
        assert len(sent) == 1
        now[0] = 1.0
        logic.press()
        assert sent == [(4242, signal.SIGTERM)] * 2


def test_lab_has_no_runner_signal_paths():
    import inspect

    source = inspect.getsource(dream)
    assert "send_signal" not in source and "os.kill" not in source
    assert "proc.terminate(" not in source and "proc.kill(" not in source
