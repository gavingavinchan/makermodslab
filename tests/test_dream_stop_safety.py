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
"""Offline stop/release regressions: real handlers, real StopLogic, FakeArm only."""

import json
import signal
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

REAL = Path(__file__).resolve().parents[1] / "scripts/viscous-dream1/real"
sys.path.insert(0, str(REAL))
import dream_real  # noqa: E402
import dream_real_core as core  # noqa: E402
import stop_button  # noqa: E402


def handlers(monkeypatch, supervised=True):
    installed = {}
    monkeypatch.setattr(dream_real.signal, "signal", lambda sig, fn: installed.update({sig: fn}))
    events = dream_real.install_stop_signals(None, supervised)
    return events, installed


@pytest.mark.parametrize(
    "source",
    ["button died", "page after button died", "call cap", "trip", "NaN status", "unwritable status", "key"],
)
def test_hold_cannot_release_without_click(monkeypatch, tmp_path, source, capsys):
    from tests.test_viscous_dream_real import EpisodeOnFakesTest, NoGuard

    events, installed = handlers(monkeypatch)
    stop, abort = events
    pid_file = tmp_path / "runner.pid"
    pid_file.write_text("4242")
    status_path = Path(f"{pid_file}.status.json")
    trial, sent, stages = EpisodeOnFakesTest(), [], []
    logic = None

    def kill(pid, sig):
        sent.append((pid, sig))
        installed[sig](sig, None)

    def after(clock, _stop, _abort):
        nonlocal logic
        ep = trial.ep
        if ep.run_state != "holding":
            return
        if not stages:
            assert ep.arm.energized and not abort.is_set()
            if source == "NaN status":
                ep._status(
                    {
                        "torque": {m: float("nan") for m in core.ARM},
                        "temp": {m: float("inf") for m in core.ARM},
                    }
                )
                data = json.loads(status_path.read_text())
                assert all(j["torque_nm"] is None and j["temp_c"] is None for j in data["joints"].values())
            if source == "unwritable status":
                monkeypatch.setattr(core, "write_status", Mock(side_effect=OSError("disk unwritable")))
                ep._status()
                assert not status_path.exists()
            # A reopened button, or the first poll immediately after an internal
            # hold: deliberately never call view(), reproducing the old 350 ms gap.
            logic = stop_button.StopLogic(
                pid_file,
                kill=kill,
                clock=clock,
                wall=clock,
                command=lambda pid: "python dream_real.py",
                started=lambda pid: 999,
                button_start_time=clock() if source in ("button died", "page after button died") else 999,
            )
            assert not logic.holding
            if source == "key":
                logic.press(from_key=True)
            else:
                if source not in ("button died", "page after button died"):
                    stop_button.request_remote_press(pid_file, source, 4242, wall=clock)
                logic.poll_remote()
            expected = [] if source in ("button died", "page after button died") else [(4242, signal.SIGTERM)]
            assert sent == expected
            assert not abort.is_set() and ep.arm.energized
            # Even a second raw STOP cannot abort supervised return/hold.
            installed[signal.SIGTERM](signal.SIGTERM, None)
            assert not abort.is_set()
            stages.append(clock())
        elif clock() - stages[0] >= 1.0 and len(stages) == 1:
            if source in ("button died", "page after button died"):
                assert sent == []  # old request never acts on the reopened button
                data = json.loads(status_path.read_text())
                data["pid"] = 4242  # status belongs to the injected runner identity
                status_path.write_text(json.dumps(data))
                logic.view()  # a first physical click while holding releases immediately
                assert logic.holding
            else:
                logic.press(from_key=True)
                assert sent == [(4242, signal.SIGTERM)]
            assert not abort.is_set() and ep.arm.energized
            logic.press()  # the deliberate physical click is the ONLY abort
            assert sent[-1] == (4242, signal.SIGUSR1) and abort.is_set()
            stages.append(clock())

    def button_lost():
        if trial.ep.clock() < 1009:
            return True
        # Lab queues a stop after button death; nobody consumes it until reopen.
        stop_button.request_remote_press(pid_file, source, 4242, wall=trial.ep.clock)
        return False

    alive = button_lost if source in ("button died", "page after button died") else None
    summary, arm = trial.episode(
        scenario="overspeed" if source == "trip" else "nominal",
        max_calls=5 if source == "trip" else 1,
        guard=NoGuard(clear=False),
        after=after,
        stop_button_alive=alive,
        status_path=status_path,
        signal_events=events,
    )
    assert len(stages) == 2 and not arm.energized and trial.release_after_abort
    assert "held" in summary["stop"]["returned"]
    if source == "trip":
        assert [trip["name"] for trip in summary["trips"]] == ["overspeed"]
    if source == "call cap":
        assert summary["reason"] == "call cap 1 reached"
    if source in ("button died", "page after button died"):
        assert summary["reason"] == "stop button gone"
    if source == "unwritable status":
        assert "STATUS WRITE FAILED (no status)" in capsys.readouterr().out
        assert "no status" in logic.view()[0]


def test_page_stop_then_click_uses_separate_release_signal(monkeypatch, tmp_path):
    events, installed = handlers(monkeypatch)
    pid_file = tmp_path / "runner.pid"
    pid_file.write_text("4242")
    now, sent = [1000.0], []

    def kill(pid, sig):
        sent.append(sig)
        installed[sig](sig, None)

    logic = stop_button.StopLogic(
        pid_file,
        kill=kill,
        clock=lambda: now[0],
        command=lambda pid: "dream_real.py",
        started=lambda pid: 999,
        button_start_time=999,
    )
    stop_button.request_remote_press(pid_file, "page", 4242, wall=lambda: now[0])
    logic.poll_remote()
    assert events[0].is_set() and not events[1].is_set()
    logic.press()
    assert sent == [signal.SIGTERM]
    now[0] += 1
    logic.press()
    assert sent == [signal.SIGTERM, signal.SIGUSR1] and events[1].is_set()


def test_terminal_keeps_two_sigterm_release_on_fake_arm(monkeypatch):
    from tests.test_viscous_dream_real import EpisodeOnFakesTest

    events, installed = handlers(monkeypatch, supervised=False)
    pressed = []

    def after(clock, stop, abort):
        if clock() > 1009 and not pressed:
            installed[signal.SIGTERM](signal.SIGTERM, None)
            assert stop.is_set() and not abort.is_set()
            installed[signal.SIGTERM](signal.SIGTERM, None)
            assert abort.is_set()
            pressed.append(True)

    trial = EpisodeOnFakesTest()
    _, arm = trial.episode(after=after, signal_events=events)
    assert pressed and not arm.energized and trial.release_after_abort


def test_lookup_gap_does_not_reset_stop_count(tmp_path):
    pid_file = tmp_path / "runner.pid"
    pid_file.write_text("4242")
    command, sent = ["dream_real.py"], []
    logic = stop_button.StopLogic(
        pid_file, command=lambda pid: command[0], kill=lambda pid, sig: sent.append(sig)
    )
    logic.press(from_key=True)
    command[0] = None
    logic.view()
    pid_file.unlink()
    logic.view()
    assert logic.presses == 1
    pid_file.write_text("4242")
    command[0] = "dream_real.py"
    logic.view()
    logic.press(from_key=True)
    assert sent == [signal.SIGTERM]
    pid_file.write_text("4343")
    logic.view()
    assert logic.presses == 0
    logic.press(from_key=True)
    assert sent == [signal.SIGTERM, signal.SIGTERM]


@pytest.mark.parametrize("pid,written", [(9999, 1001), (4242, 998)])
def test_mailbox_rejects_other_runner_or_pre_start_request(tmp_path, pid, written):
    pid_file = tmp_path / "runner.pid"
    pid_file.write_text("4242")
    sent = []
    logic = stop_button.StopLogic(
        pid_file,
        command=lambda pid: "dream_real.py",
        started=lambda pid: 999,
        kill=lambda *args: sent.append(args),
    )
    path = stop_button.request_remote_press(pid_file, "old page", pid, wall=lambda: written)
    assert "ignored" in logic.poll_remote()[0]
    assert not path.exists() and not sent


def test_heartbeat_identity_age_and_corruption(monkeypatch, tmp_path):
    import psutil

    pid_file = tmp_path / "runner.pid"
    logic = stop_button.StopLogic(pid_file, wall=lambda: 1002, clock=lambda: 50)
    logic.heartbeat()
    path = Path(f"{pid_file}.button.json")
    beat = json.loads(path.read_text())
    assert set(beat) == {"pid", "start_time", "wall", "mono"}
    button = Mock()
    button.create_time.return_value = beat["start_time"]
    button.status.return_value = "running"
    monkeypatch.setattr(psutil, "Process", Mock(return_value=button))
    now = [50.0]
    # A real creation time must precede the synthetic heartbeat wall.
    beat["wall"] = beat["start_time"] + 2
    path.write_text(json.dumps(beat))
    alive = dream_real.button_liveness(beat["pid"], beat["start_time"], pid_file, clock=lambda: now[0])
    assert alive()
    now[0] = 52
    assert alive()
    path.write_text("corrupt")
    now[0] = 52.01
    assert not alive() and alive.reason == "stop button not responding"
    # Before its first heartbeat, a reused pid must still fail immediately.
    button.create_time.return_value += 1
    assert not dream_real.button_liveness(beat["pid"], beat["start_time"], pid_file)()


def test_frozen_button_stops_fake_arm_internally(monkeypatch, tmp_path):
    import psutil

    from tests.test_viscous_dream_real import EpisodeOnFakesTest

    button = Mock()
    button.create_time.return_value = 999
    button.status.return_value = "running"
    monkeypatch.setattr(psutil, "Process", Mock(return_value=button))
    path = tmp_path / "runner.pid"
    Path(f"{path}.button.json").write_text(
        json.dumps({"pid": 42, "start_time": 999, "wall": 1000, "mono": 1000})
    )
    trial = EpisodeOnFakesTest()
    alive = dream_real.button_liveness(42, 999, path, clock=lambda: trial.ep.clock())
    summary, arm = trial.episode(rehearse=True, stop_button_alive=alive)
    assert summary["reason"] == "stop button not responding"
    assert not trial.ep.stop_event.is_set() and not trial.ep.abort_event.is_set()
    trial.assert_stopped_safely(arm)


def test_status_sanitizes_all_nonfinite_fields(tmp_path):
    path = tmp_path / "status.json"
    core.write_status(
        path,
        {
            "state": "holding",
            "joints": {"a": {"torque_nm": float("nan")}},
            "other": [float("inf"), {"nested": -float("inf")}],
        },
    )
    assert json.loads(path.read_text()) == {
        "state": "holding",
        "joints": {"a": {"torque_nm": None}},
        "other": [None, {"nested": None}],
    }


def test_app_nap_assertion_lifetime(monkeypatch, tmp_path):
    process, token, order = Mock(), object(), []
    process.beginActivityWithOptions_reason_.side_effect = lambda *args: (order.append("begin"), token)[1]
    process.endActivity_.side_effect = lambda value: order.append(("end", value))
    monkeypatch.setitem(
        sys.modules,
        "Foundation",
        SimpleNamespace(
            NSActivityUserInitiated=123, NSProcessInfo=SimpleNamespace(processInfo=lambda: process)
        ),
    )
    monkeypatch.setattr(sys, "platform", "darwin")
    monkeypatch.setattr(stop_button, "App", lambda logic: SimpleNamespace(run=lambda: order.append("run")))
    stop_button.main(["--pid-file", str(tmp_path / "runner.pid")])
    assert order == ["begin", "run", ("end", token)]
