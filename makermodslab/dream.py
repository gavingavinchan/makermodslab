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
"""Dream-1 subprocess lifecycle for the Viscous page (goal B3).

The runner alone owns hardware and safety. This module owns plans, camera
reservations, the session lease's single graceful signal, and read-only files.
"""

from __future__ import annotations

import contextlib
import hashlib
import importlib
import json
import logging
import re
import subprocess
import sys
import threading
import time
import uuid
from dataclasses import dataclass, field
from functools import lru_cache
from pathlib import Path

from . import dream_process, session_events
from .api_errors import ApiError, ErrorCode
from .arms import registry
from .schemas.dream import DreamPlanBody, DreamSettings
from .utils import config as cfg

logger = logging.getLogger(__name__)
REPO = Path(__file__).resolve().parents[1]
REAL = REPO / "scripts/viscous-dream1/real"
PLAN_TTL_S = 1200
PREVIEW_TTL_S = 600
_lock = threading.RLock()
_operations = threading.Lock()
dream_active = False
checking_inputs = False
_proc = None
_button = None
_thread = None
_stop_requested = False
_term_sent = False
_pid_file = None
_adopted = None
_latest_run = None
_latest_robot = None
_error = None
_plans = {}
_previews = {}


def root() -> Path:
    return Path(cfg.MAKERMODSLAB_HOME) / cfg.DREAM_DIRECTORY


def run_root() -> Path:
    if cfg.DREAM_RUN_DATA is not None:
        return Path(cfg.DREAM_RUN_DATA)
    _helpers()  # adds the source checkout's pure runner helpers to sys.path
    return importlib.import_module("run_data").RUN_DATA


def refuse(message, code=ErrorCode.ROBOT_NOT_READY, status=409):
    raise ApiError(status_code=status, detail=message, code=code)


def _active_path():
    return root() / "active.json"


def _save_active(proc, button, pid_file, name, robot, session_id):
    record = {
        "runner": dream_process.identity(proc.pid),
        "button": dream_process.identity(button.pid),
        "pid_file": str(pid_file),
        "status_path": str(_view_file(name, "status.json")),
        "session_id": session_id,
        "run": name,
        "robot": robot,
    }
    path = _active_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(record))
    tmp.replace(path)


def adopt_active():
    """Reconcile on startup and every mutex check, including after abrupt Lab death."""
    global _adopted, _latest_run, _latest_robot, _pid_file, _term_sent, _stop_requested
    from . import sessions

    with _lock:
        if dream_active:  # this server's worker still owns the launch/cleanup
            return True
        try:
            record = json.loads(_active_path().read_text())
        except FileNotFoundError:
            return False
        except (OSError, ValueError):
            logger.exception("Cannot read Dream active record; refusing other robot work")
            return True
        try:
            if not dream_process.alive(record["runner"]):
                _active_path().unlink(missing_ok=True)  # only after original runner exits
                if _adopted is not None:
                    session_events.notify_session_changed("dream", False, phase="exited")
                _adopted = None
                return False
            if not dream_process.alive(record["runner"], REAL / "dream_real.py"):
                logger.error(
                    "Dream active record has a live identity with unexpected command; refusing robot work"
                )
                return True
        except Exception:
            logger.exception("Cannot verify Dream runner identity; refusing other robot work")
            return True
        if _adopted != record:
            logger.warning(
                "Adopting surviving Dream runner %s and red button %s; no shutdown escalation",
                record["runner"]["pid"],
                record["button"]["pid"],
            )
            _term_sent = _stop_requested = False
        _adopted = record
        _latest_run, _latest_robot, _pid_file = record["run"], record["robot"], Path(record["pid_file"])
        sessions.tracker.adopt_dream(record["session_id"], record["robot"])
        return True


def dream_is_active() -> bool:
    return dream_active or checking_inputs or adopt_active()


def open_idle_preview(index, key, rotation, *, open_stream):
    # Same reservation lock used when a Dream check/run claims the cameras.
    # open_stream acquires the capture before returning its generator.
    with _lock:
        if dream_is_active():
            refuse("Dream owns the cameras. Use the Dream-view tiles.", ErrorCode.ROBOT_BUSY_DREAM)
        return open_stream(index, key, rotation)


def busy_response():
    if dream_is_active():
        return {
            "success": False,
            "status_code": 409,
            "code": ErrorCode.ROBOT_BUSY_DREAM,
            "message": "Dream is using the robot or checking its cameras. Stop it first.",
        }
    return None


def _other_busy():
    from . import jobs, sessions

    held = sessions.held_by()
    if held:
        refuse(f"Stop {held.replace('_', ' ')} before using Dream.", f"robot.busy.{held}")
    if jobs.training_is_active() is not None:
        refuse("Stop local training before using Dream.", ErrorCode.ROBOT_BUSY_TRAINING)


def _record(robot):
    from .arm_capabilities import require_known_arm_type

    record = cfg.get_robot_record(robot, resolve_usb=False) if cfg.is_valid_robot_name(robot) else None
    if record is None:
        refuse("Robot record not found.", ErrorCode.ROBOT_NOT_FOUND, 404)
    require_known_arm_type(record.get("arm_type"))
    if (
        not registry.get(record["arm_type"]).supports_dream
        or record.get("mode", "single") != "single"
        or record.get("arms", "both") == "leader"
    ):
        refuse("Dream requires a supported single Viscous follower.", status=400)
    return record


def _digest(record):
    return hashlib.sha256(json.dumps(record, sort_keys=True).encode()).hexdigest()


def _new_run(mode):
    name = f"{mode}-{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime()).lower()}-{uuid.uuid4().hex[:8]}"
    work = root() / "work" / name
    work.mkdir(parents=True)
    return name, work


def _argv(robot, settings, name, work):
    args = [
        sys.executable,
        str(REAL / "dream_real.py"),
        "--robot",
        robot,
        "--lab-home",
        str(cfg.MAKERMODSLAB_HOME),
        "--lab-url",
        cfg.DREAM_LAB_URL,
        "--dream-venv",
        str(cfg.DREAM_VENV),
        "--arm-file",
        str(cfg.DREAM_ARM_FILE),
        "--out",
        str(work),
        "--run-data",
        str(run_root()),
        "--dream-view-dir",
        str(run_root() / name / "dream_view"),
        "--prompt",
        settings.prompt,
        "--wrist-l",
        settings.wrist_l,
        "--wrist-r",
        settings.wrist_r,
        "--max-calls",
        str(settings.max_calls),
        "--pause-s",
        str(settings.pause_s),
    ]
    if settings.overhead is not None:
        args += ["--overhead", settings.overhead]
    return args


def _run(args, timeout=180):
    try:
        result = subprocess.run(args, cwd=REPO, capture_output=True, text=True, timeout=timeout, check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        refuse(f"Dream command could not finish: {exc}", status=503)
    if result.returncode:
        refuse(f"Dream refused: {(result.stdout + result.stderr)[-6000:]}", status=400)
    return result


def plan(body: DreamPlanBody):
    if not _operations.acquire(blocking=False):
        refuse("Another Dream check is in progress.", ErrorCode.ROBOT_BUSY_DREAM)
    try:
        with _lock:
            _other_busy()
        record = _record(body.robot)
        settings = DreamSettings.model_validate(body.model_dump(exclude={"robot"}))
        name, work = _new_run("dry")
        report = work / "lab-report.json"
        _run([*_argv(body.robot, settings, name, work), "--dry-run", "--lab-report-file", str(report)])
        data = json.loads(report.read_text())
        plan_id = data["plan_id"]
        with _lock:
            # Bounded cache; plans never survive a server restart or outlive
            # the robot record that produced them.
            while len(_plans) >= 16:
                _plans.pop(next(iter(_plans)))
            _plans[plan_id] = {
                "robot": body.robot,
                "settings": settings,
                "record": _digest(record),
                "created": time.monotonic(),
            }
        return {"plan_id": plan_id, "movement": data["movement"], "readiness": data["readiness"]}
    finally:
        _operations.release()


def preview(body: DreamPlanBody):
    global checking_inputs, _latest_run, _latest_robot, _error
    if not _operations.acquire(blocking=False):
        refuse("Another Dream check is in progress.", ErrorCode.ROBOT_BUSY_DREAM)
    try:
        record = _record(body.robot)
        settings = DreamSettings.model_validate(body.model_dump(exclude={"robot"}))
        with _lock:
            _other_busy()
            checking_inputs = True
            _previews.clear()
        # Reserve BEFORE releasing captures so other preview requests cannot
        # immediately reopen the devices the child is about to use.
        from .camera_preview import camera_preview_manager

        camera_preview_manager.stop_all()
        name, work = _new_run("preview")
        with _lock:
            _latest_run, _latest_robot, _error = name, body.robot, None
        _run([*_argv(body.robot, settings, name, work), "--preview"], timeout=90)
        status = _read_view(name)
        if not status or not re.fullmatch(r"[a-f0-9]{12}", status.get("inputs_id") or ""):
            refuse("The camera preview did not produce an inputs id.", status=503)
        token = uuid.uuid4().hex
        with _lock:
            _previews.clear()
            _previews[token] = {
                "inputs_id": status["inputs_id"],
                "run": name,
                "robot": body.robot,
                "record": _digest(record),
                "created": time.monotonic(),
            }
        return {
            "preview_id": token,
            "inputs_id": status["inputs_id"],
            "inputs_checked": bool(status["inputs_checked"]),
            "run": name,
        }
    except Exception as exc:
        with _lock:
            _error = str(getattr(exc, "detail", exc))
        raise
    finally:
        with _lock:
            checking_inputs = False
        _operations.release()


def approve_inputs(body):
    if not _operations.acquire(blocking=False):
        refuse("Another Dream check is in progress.", ErrorCode.ROBOT_BUSY_DREAM)
    try:
        with _lock:
            _other_busy()
            item = _previews.get(body.preview_id)
            if not item or item["inputs_id"] != body.inputs_id:
                refuse("Check cameras again before approving these inputs.", status=400)
            if time.monotonic() - item["created"] > PREVIEW_TTL_S:
                refuse("Camera check expired. Check cameras again.", status=400)
            if _digest(_record(item["robot"])) != item["record"]:
                refuse("The robot changed. Check cameras again.", status=400)
        # This is the ONLY call site. The route represents Gavin's explicit
        # 'Inputs look right' click. Never called by planning or startup.
        _run([str(cfg.DREAM_VENV / "bin/python"), "-m", "dream.input_check", body.inputs_id], timeout=30)
        with _lock:
            _previews.pop(body.preview_id, None)
        return {"inputs_id": body.inputs_id, "approved": True}
    finally:
        _operations.release()


@dataclass
class DreamRequest:
    robot: str
    plan_id: str
    start_gate: threading.Event = field(default_factory=threading.Event)


def build_request(record, opts):
    return DreamRequest(record["name"], opts.plan_id)


def handle_start_dream(request):
    global dream_active, _stop_requested, _term_sent, _error, _latest_run, _latest_robot, _thread
    if not _operations.acquire(blocking=False):
        refuse("Another Dream check is in progress.", ErrorCode.ROBOT_BUSY_DREAM)
    try:
        with _lock:
            _other_busy()
            record = _record(request.robot)
            item = _plans.get(request.plan_id)
            if not item or item["robot"] != request.robot or time.monotonic() - item["created"] > PLAN_TTL_S:
                refuse("Plan expired or belongs to another robot. Make a new plan.", status=400)
            if item["record"] != _digest(record):
                refuse("Robot settings changed since the plan. Make a new plan.", status=400)
            name, work = _new_run("live")
            dream_active, _stop_requested, _term_sent = True, False, False
            _latest_run, _latest_robot, _error = name, request.robot, None
            session_events.notify_session_changed("dream", True, phase="starting")
            _plans.pop(request.plan_id)
            try:
                _thread = threading.Thread(
                    target=_worker,
                    args=(request, item["settings"], name, work),
                    name="dream-runner",
                    daemon=True,
                )
                _thread.start()
            except Exception as exc:
                _error = f"Could not start Dream supervision: {exc}"
                dream_active = False
                session_events.notify_session_changed("dream", False, phase="error")
                raise
        return {"success": True}
    finally:
        _operations.release()


def _button_problem(pid_file):
    return (
        "The standalone red stop button could not stay open (a desktop display and tkinter are required). "
        f"Start it by hand on the arm's desktop: {sys.executable} {REAL / 'stop_button.py'} "
        f"--pid-file {pid_file}. The arm has not been started. Retry with a working desktop display."
    )


def _worker(request, settings, name, work):
    global _proc, _button, _error, dream_active, _pid_file
    from . import sessions
    from .camera_preview import camera_preview_manager

    proc = button = None
    phase = "starting"
    try:
        request.start_gate.wait()  # wrapper attaches identity + lease FIRST
        with _lock:
            session = sessions.tracker.current()
            if _stop_requested:
                return
            if not session or session["kind"] != "dream":
                raise RuntimeError("Dream's Lab session was lost before launch.")
            session_id = session["id"]
        camera_preview_manager.stop_all()
        pid_file = work / "runner.pid"
        with _lock:
            _pid_file = pid_file
        log_path = work / "supervisor.log"
        try:
            button = dream_process.spawn(
                [sys.executable, str(REAL / "stop_button.py"), "--pid-file", str(pid_file)],
                cwd=REPO,
                log=log_path,
                result=work / "button-exit.json",
            )
        except OSError as exc:
            raise RuntimeError(f"{_button_problem(pid_file)} ({exc})") from exc
        with _lock:
            _button = button
        with contextlib.suppress(subprocess.TimeoutExpired):
            button.wait(timeout=1.0)
        if button.poll() is not None:
            raise RuntimeError(_button_problem(pid_file))
        button_identity = dream_process.identity(button.pid)
        with _lock:
            if _stop_requested:
                return
            if button.poll() is not None:
                raise RuntimeError(_button_problem(pid_file))
            proc = dream_process.spawn(
                [
                    *_argv(request.robot, settings, name, work),
                    "--live",
                    "--stop-button-pid",
                    str(button.pid),
                    "--stop-button-start-time",
                    str(button_identity["started"]),
                    "--plan-id",
                    request.plan_id,
                    "--pid-file",
                    str(pid_file),
                    "--lab-session-id",
                    session_id,
                ],
                cwd=REPO,
                log=log_path,
                result=work / "runner-exit.json",
            )
            _proc = proc
            _save_active(proc, button, pid_file, name, request.robot, session_id)
            logger.warning(
                "Dream runner %s and button %s are reparented outside Lab shutdown trees",
                proc.pid,
                button.pid,
            )
        button_gone = False
        while proc.poll() is None:
            status = _read_view(name)
            if status and status["phase"] != phase:
                phase = status["phase"]
                session_events.notify_session_changed("dream", True, phase=phase)
            if button.poll() is not None and not button_gone:
                button_gone = True
                with _lock:
                    _error = (
                        "The standalone stop button closed. Return to rest requested. "
                        "If the runner is holding, support the arm and reopen the button: "
                        f"{sys.executable} {REAL / 'stop_button.py'} --pid-file {pid_file}"
                    )
                handle_stop_dream(source="button died")
            with contextlib.suppress(subprocess.TimeoutExpired):
                proc.wait(timeout=0.2)
        if proc.returncode:
            raise RuntimeError(
                f"Dream runner exited with code {proc.returncode}. "
                + (work / "supervisor.log").read_text()[-5000:]
            )
    except Exception as exc:
        logger.exception("Dream runner failed")
        with _lock:
            _error = str(exc)
        # If supervision itself fails, stop ONCE and retain the claim until
        # the runner finishes its return or the person releases using the button.
        handle_stop_dream(source="supervision error")
        if proc is not None:
            proc.wait()  # deliberately no timeout escalation, never SIGKILL
        phase = "error"
    finally:
        # Neither process belongs to Lab's shutdown tree. Leave the button
        # open even after failure, and never escalate a held arm's stop.
        with _lock:
            if proc is not None and proc.poll() is not None:
                _active_path().unlink(missing_ok=True)
            _proc = None
            _button = None
            dream_active = False
            session_events.notify_session_changed("dream", False, phase="error" if _error else "released")


def handle_stop_dream(source="page/lease"):
    global _stop_requested, _term_sent
    dream_is_active()  # adopt a surviving runner before routing its stop
    with _lock:
        first = not _stop_requested
        _stop_requested = True
        if _pid_file is not None and not _term_sent:
            # Only the button signals the runner. Its real key-press logic
            # serializes physical and remote requests, with no release path.
            _helpers()
            target_pid = _proc.pid if _proc is not None else _adopted["runner"]["pid"] if _adopted else None
            if target_pid is None:
                with contextlib.suppress(OSError, ValueError, IndexError):
                    target_pid = int(_pid_file.read_text().split()[0])
            if target_pid is not None:
                importlib.import_module("stop_button").request_remote_press(_pid_file, source, target_pid)
                _term_sent = True
        if dream_active and first:
            session_events.notify_session_changed("dream", True, phase="stopping")
    return {"success": True, "message": "Return to rest requested. The standalone button owns release-all."}


def stop_for_shutdown():
    if dream_is_active():
        handle_stop_dream(source="Lab shutdown")
        logger.warning(
            "Dream continues its stop independently under the red button; not waiting or escalating."
        )


def _bundle(run):
    if not re.fullmatch(r"[a-zA-Z0-9-]{1,90}", run):
        refuse("Unknown Dream run.", status=404)
    base = run_root() / ".dream-replay-cache" if run.startswith("replay-") else run_root()
    path = base / run
    if base.is_symlink() or path.is_symlink() or path.resolve().parent != base.resolve():
        refuse("Unknown Dream run.", status=404)
    return path


def _view_file(run, name):
    folder = (
        Path(_adopted["status_path"]).parent
        if _adopted is not None and _adopted["run"] == run
        else _bundle(run) / "dream_view"
    )
    path = folder / name
    if folder.is_symlink() or path.is_symlink():
        refuse("Unknown Dream view.", status=404)
    return path


def _read_view(run):
    try:
        return json.loads(_view_file(run, "status.json").read_text()) if run else None
    except (OSError, ValueError):
        return None


def status():
    is_active = dream_is_active()
    with _lock:
        active, checking, run, error = is_active, checking_inputs, _latest_run, _error
        button_alive = _button is not None and _button.poll() is None
        if _adopted is not None:
            button_alive = dream_process.alive(_adopted["button"], REAL / "stop_button.py")
        if active and _pid_file is not None:
            _helpers()
            pending = importlib.import_module("stop_button").request_directory(_pid_file)
            try:
                unresponsive = any(
                    time.time() - path.stat().st_mtime >= 1.0 for path in pending.glob("*.json")
                )
            except OSError:
                unresponsive = True
            if unresponsive:
                warning = "the red button is not responding: use it directly or cut power"
                error = f"{warning}\n{error}" if error else warning
    data = _read_view(run)
    stale = bool(active and (not data or time.time() - data.get("written", 0) > 1.0))
    return {
        "active": active,
        "checking_inputs": checking,
        "run": run,
        "robot": _latest_robot,
        "status": data,
        "error": error,
        "stale": stale,
        "button_alive": button_alive,
    }


def image_path(run, slot):
    if slot not in ("overhead", "wrist_l", "wrist_r"):
        refuse("Unknown Dream slot.", status=404)
    path = _view_file(run, f"{slot}.jpg")
    if not path.is_file():
        refuse("No Dream image yet.", status=404)
    return path


@lru_cache(maxsize=1)
def _helpers():
    # Source checkout required, like the runner itself. Import only its pure
    # image helpers and observational writer, never dream_real / hardware.
    sys.path.insert(0, str(REAL))
    return importlib.import_module("dream_real_core"), importlib.import_module("dream_view")


@lru_cache(maxsize=2)
def _load_replay(source):
    source = Path(source)
    frames = [json.loads(line) for line in (source / "frames/index.jsonl").read_text().splitlines()]
    ticks = [json.loads(line) for line in (source / "ticks.jsonl").read_text().splitlines()]
    chunks = [json.loads(line) for line in (source / "chunks.jsonl").read_text().splitlines()]
    phases = []
    for line in (source / "console.log").read_text().splitlines():
        match = re.match(r"\[\s*([\d.]+) s\] phase: (.*)", line)
        if match:
            phases.append((float(match[1]), match[2]))
    plan = json.loads((source / "run/plan.json").read_text())
    # Raw frames already have the record rotation applied at capture.
    slots = {k: v.split(" (rotation")[0] for k, v in plan["dream_slots"].items()}
    if not frames or not ticks:
        raise ValueError("bundle has no recorded camera frames or ticks")
    approach_t = next((t for t, line in phases if line.startswith("approach")), 0.0)
    start_t = frames[0]["t"] - approach_t
    duration = max([t for t, _ in phases] + [ticks[-1]["t"] - start_t])
    return frames, ticks, chunks, phases, plan, slots, start_t, duration


def replay(at_s):
    """Reconstruct saved raw frames with the runner's SAME pure crop/JPEG path.

    No camera, subprocess, session claim or Dream client. Replay outputs use
    separate bundles and can never replace the live run's latest pointer.
    Historical raw JPEGs were compressed once by run_data; these reconstructed
    tiles are labelled replay, never claimed to be original request bytes.
    """
    import bisect
    import datetime as dt

    import numpy as np
    from PIL import Image

    source = cfg.DREAM_REPLAY
    if source is None:
        # No bundled media. The default source is the latest complete run
        # in the runner's existing ring, chosen without opening devices.
        _helpers()
        candidates = importlib.import_module("run_data").bundles(run_root())
        source = next(
            (
                b["path"]
                for b in candidates
                if not b["active"]
                and all(
                    (b["path"] / f).is_file()
                    for f in (
                        "frames/index.jsonl",
                        "ticks.jsonl",
                        "chunks.jsonl",
                        "console.log",
                        "run/plan.json",
                    )
                )
            ),
            None,
        )
        if source is None:
            refuse(
                "No saved camera run available. Set MAKERMODSLAB_DREAM_REPLAY to a run-data bundle.",
                status=404,
            )
    try:
        frames, ticks, chunks, phases, plan, slots, t0, duration = _load_replay(str(source))
    except (OSError, ValueError, KeyError) as exc:
        refuse(
            f"Replay bundle unavailable: {exc}. Set MAKERMODSLAB_DREAM_REPLAY to a saved bundle.", status=404
        )
    elapsed = min(max(float(at_s), 0.0), duration)
    now = t0 + elapsed
    row = frames[max(0, bisect.bisect_right([r["t"] for r in frames], now) - 1)]
    images = {}
    for camera, relative in row["frames"].items():
        path = (source / "frames" / relative).resolve()
        if not path.is_relative_to((source / "frames").resolve()):
            refuse("Unsafe replay frame path.", status=400)
        with Image.open(path) as img:
            images[camera] = np.asarray(img.convert("RGB"))
    core, view_module = _helpers()
    # The output name depends on its frame index, so concurrent viewers
    # scrubbing different times cannot overwrite each other's JPEGs.
    run = f"replay-{hashlib.sha256(str(source).encode()).hexdigest()[:8]}-{frames.index(row)}"
    view = view_module.DreamView(
        _bundle(run) / "dream_view",
        run=run,
        mode="replay",
        prompt=plan["prompt"],
        max_calls=plan["speed"]["max_calls"],
        slots=slots,
    )
    view.observation(core.slot_frames(images, slots))
    history = [line for t, line in phases if t <= elapsed]
    phase = history[-1] if history else "connect"
    pause = float(plan.get("pause_s", 10.0))
    pause_start = next((t for t, line in phases if line.startswith("pause")), None)
    if pause_start is not None and pause_start <= elapsed < pause_start + pause:
        phase = "pause - holding before the first Dream call"
    tick_history = [r for r in ticks if r["t"] <= now]
    tick = tick_history[-1] if tick_history else {}
    # Align the saved ledger's wall times to the first possible Dream call.
    # Exact hardware telemetry stays aligned to its recorded monotonic clock.
    pause = float(plan.get("pause_s", 10.0))
    first_call = next((t for t, line in phases if line.startswith("dream")), duration) + pause
    if chunks:

        def utc(c):
            return dt.datetime.fromisoformat(c["ledger"]["utc"].replace("Z", "+00:00")).timestamp()

        calls = [c for c in chunks if first_call + utc(c) - utc(chunks[0]) <= elapsed]
    else:
        calls = []
    call = max(
        [c["call"] for c in calls]
        + [int(m.group(1)) for line in history if (m := re.search(r"call (\d+):", line))]
        + [0]
    )
    last_chunk = next((c for c in reversed(chunks) if c["call"] <= call), {})
    requested = last_chunk.get("per_key", {}).get("action.left_eef_xyz", [])
    reasons = [str(t) for r in tick_history for t in r.get("trips", [])]
    state = (
        "released"
        if elapsed >= duration
        else "returning"
        if "return" in phase or phase.startswith("stop")
        else "running"
    )
    view.update(
        pid=None,
        phase=phase,
        state=state,
        call=last_chunk.get("call", 0),
        measured=tick.get("pos"),
        commanded=tick.get("cmd"),
        requested_tcp_path=requested,
        last_reason=reasons[-1] if reasons else last_chunk.get("refused"),
        peak_torque_nm=max(
            (abs(v) for r in tick_history for v in (r.get("torque") or {}).values()), default=0.0
        ),
        peak_temp_c=max((v for r in tick_history for v in (r.get("temp") or {}).values()), default=0.0),
        message=phase,
    )
    return {
        "run": run,
        "duration_s": duration,
        "elapsed_s": elapsed,
        "status": _read_view(run),
        "console": history[-8:],
    }
