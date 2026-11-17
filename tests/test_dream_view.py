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
"""Additive runner outputs on fakes: no port, camera, Dream network or approval."""

import base64
import contextlib
import io
import json
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

REAL = Path(__file__).resolve().parents[1] / "scripts/viscous-dream1/real"
sys.path.insert(0, str(REAL))
import dream_real  # noqa: E402
import dream_real_core as core  # noqa: E402
from dream_view import DreamView, ViewTransport  # noqa: E402


def make_view(tmp_path, mode="live"):
    return DreamView(
        tmp_path / "dream_view",
        run="test-run",
        mode=mode,
        prompt="test prompt",
        max_calls=2,
        slots=dict.fromkeys(core.SLOTS, "black"),
        wall=lambda: 123.0,
    )


def test_every_request_publishes_the_newest_jpeg_bytes_without_reencoding(tmp_path):
    view = make_view(tmp_path)
    transport = Mock()
    path = [[0.1, 0.2, 0.3]]
    reply = {"actions": {"per_key": {"action.left_eef_xyz": path}}}
    transport.request.return_value = reply
    observer = ViewTransport(transport, view)
    for call in range(2):
        payloads = {slot: f"exact JPEG bytes for {slot} call {call}".encode() for slot in core.SLOTS}
        obs = {
            f"video.{slot}": [base64.b64encode(b"older").decode(), base64.b64encode(data).decode()]
            for slot, data in payloads.items()
        }
        request, headers = {"observation": obs}, {"header": "unchanged"}
        assert observer.request(request, headers) is reply
        assert transport.request.call_args.args == (request, headers)
        for slot, data in payloads.items():
            assert (view.directory / f"{slot}.jpg").read_bytes() == data
    status = json.loads((view.directory / "status.json").read_text())
    assert status["requested_tcp_path"] == path
    assert status["images_written"] == 123.0
    assert not list(view.directory.glob("*.tmp"))


def test_failed_output_cannot_interrupt_transport_or_status(tmp_path, monkeypatch):
    view = make_view(tmp_path)
    monkeypatch.setattr(view, "_atomic", Mock(side_effect=OSError("disk full")))
    view.update(phase="return leg 1/2")
    transport = Mock()
    transport.request.return_value = {"actions": {}}
    request = {"observation": {f"video.{s}": core.jpeg_b64(core.black_frame()) for s in core.SLOTS}}
    assert ViewTransport(transport, view).request(request, {}) == {"actions": {}}
    assert transport.request.call_count == 1


@pytest.mark.parametrize(
    "session,expected",
    [
        ({"id": "claimed", "kind": "dream"}, False),
        ({"id": "other", "kind": "dream"}, True),
        ({"id": "claimed", "kind": "teleoperation"}, True),
        (None, False),
    ],
)
def test_idle_exemption_accepts_only_the_exact_dream_session(monkeypatch, session, expected):
    def response(url, **kwargs):
        data = {"session": session} if url.endswith("current") else {"active": False}
        return contextlib.closing(io.BytesIO(json.dumps(data).encode()))

    monkeypatch.setattr(dream_real.urllib.request, "urlopen", response)
    from makermodslab import sessions

    monkeypatch.setattr(sessions, "held_by", lambda: None)
    assert bool(dream_real.lab_holder("http://local", False, "claimed")) is expected
    if session:
        assert dream_real.lab_holder("http://local", False)


def test_tank_measurement_still_refuses_with_the_claimed_session(monkeypatch):
    def response(url, **kwargs):
        data = (
            {"session": {"id": "claimed", "kind": "dream"}} if url.endswith("current") else {"active": True}
        )
        return contextlib.closing(io.BytesIO(json.dumps(data).encode()))

    monkeypatch.setattr(dream_real.urllib.request, "urlopen", response)
    from makermodslab import sessions

    monkeypatch.setattr(sessions, "held_by", lambda: None)
    assert "tank measurement" in dream_real.lab_holder("http://local", False, "claimed")


def test_episode_outputs_follow_the_stop_without_changing_its_actions(tmp_path, monkeypatch):
    from tests import test_viscous_dream_real as fixtures

    baseline = fixtures.EpisodeOnFakesTest()
    summary, arm = baseline.episode()
    expected_events = arm.events
    expected_sent = baseline.sent
    view = make_view(tmp_path, "dry")
    init = dream_real.Episode.__init__

    def with_view(self, *args, **kwargs):
        init(self, *args, **kwargs, view=view)

    monkeypatch.setattr(dream_real.Episode, "__init__", with_view)
    trial = fixtures.EpisodeOnFakesTest()
    observed, arm = trial.episode()
    assert arm.events == expected_events
    assert trial.sent == expected_sent
    assert observed == summary
    status = json.loads((view.directory / "status.json").read_text())
    assert status["state"] == "released"
    assert status["phase"] == "released"
    assert status["call"] == 2
    assert status["last_reason"] == "call cap 2 reached"
    assert status["measured"]
    assert status["commanded"]
    assert status["peak_temp_c"] > 0


def test_preview_exports_exact_bytes_to_chosen_directory_and_never_approves(tmp_path, monkeypatch):
    view = make_view(tmp_path, "preview")
    jpeg = core.jpeg_b64(np.zeros((176, 320, 3), dtype=np.uint8))
    sent = {f"video.{s}": [jpeg] * 9 for s in core.SLOTS}
    fake = SimpleNamespace(
        sent_observation=sent,
        inputs_id="abcdef123456",
        inputs_checked=False,
        install_gate=Mock(),
        set_observation=Mock(),
        close=Mock(),
    )

    def request(*args):
        view.observation(sent, inputs_id=fake.inputs_id, inputs_checked=False)

    fake.request = request
    monkeypatch.setattr(dream_real, "DreamReal", lambda *a, **k: fake)
    monkeypatch.setattr(dream_real, "deny_network", lambda: None)
    monkeypatch.setattr(dream_real.time, "sleep", lambda _: None)
    close = Mock()
    monkeypatch.setattr(dream_real, "open_cameras", lambda record, fake: ([], lambda: {}, close))
    monkeypatch.setattr(core, "slot_frames", lambda *a: sent)
    jmap = core.JointMap.load()
    ctx = {
        "dream_problem": None,
        "environment": dream_real.TANK_ENVIRONMENT,
        "out": tmp_path,
        "arm_path": REAL / "tank_model/viscous_waterproof_tank_arm.json",
        "record": {},
        "arm": {"rest": [0] * 6},
        "jmap": jmap,
        "settings": SimpleNamespace(prompt="test", slots=dict.fromkeys(core.SLOTS, "black")),
        "view": view,
        "missing": [],
    }
    assert dream_real.run_preview(SimpleNamespace(fake_cameras=True, start_motor_deg=None), ctx) == 0
    for slot in core.SLOTS:
        assert (view.directory / f"{slot}.jpg").read_bytes() == base64.b64decode(jpeg)
    assert json.loads((view.directory / "status.json").read_text())["inputs_id"] == fake.inputs_id
    close.assert_called_once()


@pytest.mark.parametrize("environment", ["tank", "dry_air"])
def test_new_cli_flags_report_the_verbatim_plan_without_changing_it(tmp_path, monkeypatch, environment):
    from tests.test_viscous_dream_real import LIMITS

    record = {
        "name": "fake_viscous",
        "arm_type": "viscous",
        "mode": "single",
        "follower_port": "/no-port",
        "follower_config": "fake",
        "cameras": [],
    }
    monkeypatch.setattr(dream_real, "load_record", lambda *a: record)
    monkeypatch.setattr(dream_real, "follower_limits", lambda *a: (LIMITS, "fake limits"))
    monkeypatch.setattr(dream_real, "add_dream_venv", lambda *a: "test has no Dream client")
    monkeypatch.setattr(dream_real, "lab_holder", lambda *a: None)
    monkeypatch.setattr(dream_real, "port_holders", lambda *a: [])
    monkeypatch.setattr(dream_real, "LEDGER", tmp_path / "ledger.json")
    # The mode dispatcher must stay dry. No transport/arm/network is constructed.
    dry = Mock(return_value=0)
    monkeypatch.setattr(dream_real, "run_dry", dry)

    class Output(io.StringIO):
        def reconfigure(self, **kwargs):
            pass

    capture = Output()
    monkeypatch.setattr(dream_real.sys, "stdout", capture)
    args = [
        "--environment",
        environment,
        "--dry-run",
        "--no-run-data",
        "--out",
        str(tmp_path / "run"),
        "--overhead",
        "black",
        "--wrist-l",
        "black",
    ]
    assert dream_real.main(args) == 0
    baseline = json.loads((tmp_path / "run/plan.json").read_text())
    assert (
        dream_real.main(
            [
                *args,
                "--lab-session-id",
                "claimed",
                "--dream-view-dir",
                str(tmp_path / "chosen"),
                "--lab-report-file",
                str(tmp_path / "report.json"),
            ]
        )
        == 0
    )
    report = json.loads((tmp_path / "report.json").read_text())
    assert report["plan_id"] == baseline["plan_id"]
    assert report["movement"] in capture.getvalue()
    assert report["movement"].startswith(f"Environment: {environment}\n")
    assert "Joints that may move (ALL SEVEN)" in report["movement"]
    assert "gripper" in report["movement"]
    assert any("plan-id" in item for item in report["readiness"])
    assert json.loads((tmp_path / "chosen/status.json").read_text())["environment"] == environment
    assert dry.call_count == 2
    assert dry.call_args.args[0].lab_session_id == "claimed"


def test_transport_end_session_delegates_and_closes_unchanged(tmp_path):
    view = make_view(tmp_path)
    real = Mock()
    real.request.return_value = {"ended": True}
    transport = ViewTransport(real, view)
    # Dream._end_session has no observation; this must reach the transport.
    request, headers = {"end_session": True}, {"session": "fake"}
    assert transport.request(request, headers, timeout=2) == {"ended": True}
    transport.close()
    real.request.assert_called_once_with(request, headers, timeout=2)
    real.close.assert_called_once()
    assert view.status["images_written"] is None


def test_control_updates_do_not_wait_for_jpeg_writes(tmp_path, monkeypatch):
    import threading

    view = make_view(tmp_path)
    entered, finish, updated = threading.Event(), threading.Event(), threading.Event()
    atomic = view._atomic

    def slow_write(name, data):
        if name.endswith(".jpg"):
            entered.set()
            assert finish.wait(5)
        atomic(name, data)

    monkeypatch.setattr(view, "_atomic", slow_write)
    request = {f"video.{s}": core.jpeg_b64(core.black_frame()) for s in core.SLOTS}
    writer = threading.Thread(target=view.observation, args=(request,))
    writer.start()
    try:
        assert entered.wait(5)
        tick = threading.Thread(target=lambda: (view.update(phase="returning"), updated.set()))
        tick.start()
        assert updated.wait(1), "control status waited behind JPEG IO"
        tick.join(1)
        assert view.status["phase"] == "returning"
    finally:
        finish.set()
        writer.join(5)
    assert json.loads((view.directory / "status.json").read_text())["phase"] == "returning"


def test_nonfinite_peaks_do_not_freeze_later_status(tmp_path):
    from tests.test_viscous_dream_real import EpisodeOnFakesTest

    trial = EpisodeOnFakesTest()
    trial.episode(rehearse=True, hold_s=0.1)
    ep = trial.ep
    ep.view = make_view(tmp_path)
    ep._status(
        {
            "torque": {"a": float("inf"), "b": float("nan"), "c": -2},
            "temp": {"a": float("nan"), "b": float("inf"), "c": 30},
        }
    )
    ep._status(state="released", message="finished")
    status = json.loads((ep.view.directory / "status.json").read_text())
    assert status["state"] == "released"
    assert status["peak_torque_nm"] == 2 and status["peak_temp_c"] == 30


def test_entire_view_status_block_cannot_interrupt_release(tmp_path, monkeypatch, capsys):
    from tests.test_viscous_dream_real import EpisodeOnFakesTest

    class BrokenView:
        # Fail during argument evaluation, before update() can swallow errors.
        @property
        def status(self):
            raise RuntimeError("broken view state")

        def update(self, **values):
            pass

    init = dream_real.Episode.__init__

    def with_view(self, *args, **kwargs):
        init(self, *args, **kwargs, view=BrokenView())

    monkeypatch.setattr(dream_real.Episode, "__init__", with_view)
    trial = EpisodeOnFakesTest()
    _, arm = trial.episode(rehearse=True, hold_s=0.1)
    trial.assert_stopped_safely(arm)
    assert capsys.readouterr().out.count("dream-view unavailable") == 1


def test_deadman_disappearance_during_identity_read_is_an_internal_stop(monkeypatch):
    import psutil

    button = Mock()
    button.is_running.return_value = True
    button.status.side_effect = psutil.NoSuchProcess(42)
    monkeypatch.setattr(psutil, "Process", Mock(return_value=button))
    assert dream_real.button_liveness(42)() is False
    monkeypatch.setattr(psutil, "Process", Mock(side_effect=psutil.NoSuchProcess(42)))
    assert dream_real.button_liveness(42)() is False
