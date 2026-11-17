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
"""Fake arm/Dream, virtual latency; thread synchronization uses Events, never sleeps.

The client seam API is injected here; offline CLI evidence also runs the installed
client's real seam_join. Unit tests require neither that optional package nor a key.
"""

import collections
import json
import threading
import time
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import numpy as np
import pytest

from tests import test_viscous_dream_real as fixtures

POSE, NoGuard, core, dream_real = fixtures.POSE, fixtures.NoGuard, fixtures.core, fixtures.dream_real


class SeamAPI:
    """Deterministic stand-in: verifies the runner supplies the *actual* seam motion."""

    RAMP_ROWS = 16

    def __init__(self):
        self.windows = []

    def fit_motion(self, rows, times):
        self.old = rows.copy(), times.copy()
        return (rows[1], rows[1] - rows[0]) if len(rows) > 1 else (rows[0], np.zeros(7))

    def velocity_blend(self, now, velocity, plan):
        self.windows.append((now.copy(), velocity.copy(), plan.copy()))
        return np.linspace(now, plan[-1], len(plan))


def run_episode(latencies=(0.6, 0.6), *, overlap=True, guard=None, after=None, changes=None, ledger=None):
    runner = fixtures.EpisodeOnFakesTest()
    seam = SeamAPI()
    init = dream_real.Episode.__init__
    start = dream_real.Worker.start
    index = 0

    def episode_init(*args, **kwargs):
        kwargs["seam_join"] = seam
        return init(*args, **kwargs)

    def worker_start(worker, commanded):
        nonlocal index
        worker.latency = latencies[min(index, len(latencies) - 1)]
        if ledger is not None:
            worker.ledger, worker.run_id = ledger, "overlap-test"
        index += 1
        return start(worker, commanded)

    with (
        mock.patch.object(dream_real.Episode, "__init__", episode_init),
        mock.patch.object(dream_real.Worker, "start", worker_start),
    ):
        summary, arm = runner.episode(
            max_calls=len(latencies),
            guard=guard,
            after=after,
            settings_changes={"overlap": overlap, "dream_state": "measured", **(changes or {})},
            environment=dream_real.ENVIRONMENTS["dry_air"],
        )
    if runner.ep.worker.cleanup_thread:
        runner.ep.worker.cleanup_thread.join(timeout=1)
        assert not runner.ep.worker.cleanup_thread.is_alive()
    assert not summary["trips"], summary["reason"]
    return runner, summary, arm, seam


def test_continuous_seam_drops_latency_rows_and_replans_after_median_lead():
    runner, summary, _, seam = run_episode()
    first, second = summary["pacing_timeline"]
    assert first["dropped_rows"] == 0
    assert second["executed_rows"] == 18  # 30 - ceil(.6 * 20)
    assert second["remaining_substeps"] == 30
    assert second["dropped_rows"] == 12
    assert second["idle_s"] == 0
    assert len(seam.windows) == 1
    # The first row after install continues exactly where the guarded queue went next.
    now = seam.windows[0][0]
    commands = [
        core.motor_to_ik_row(t["cmd"], runner.ep.jmap) for t in runner.tick_rows if t["phase"] == "dream"
    ]
    at = next(i for i, q in enumerate(commands) if np.allclose(q, now, atol=1e-10))
    assert not np.allclose(commands[at - 1][:6], commands[at][:6], atol=1e-8)
    ticks = [t["t"] for t in runner.tick_rows if t["phase"] == "dream"]
    assert np.allclose(np.diff(ticks), core.ROW_DT_S)
    assert summary["reason"] == "call cap 2 reached"
    assert summary["rows_planned"] == 48 + 36


def test_slow_median_falls_back_to_full_synchronous_chunks():
    _, summary, _, seam = run_episode((0.8, 0.8, 0.8))
    assert "pacing" in summary["phases"]
    assert not seam.windows
    for call in summary["pacing_timeline"][1:]:
        assert call["remaining_substeps"] == 0
        assert call["executed_rows"] == 48
        assert call["dropped_rows"] == 0
        assert call["idle_s"] >= 0.8


def test_late_reply_holds_last_command_then_splices_from_the_hold():
    _, summary, _, seam = run_episode((0.6, 2.0, 0.6))
    second = summary["pacing_timeline"][1]
    assert second["idle_s"] == pytest.approx(0.5)
    assert second["dropped_rows"] == 40
    assert np.allclose(seam.windows[0][1], 0)  # exhausted old queue = at rest


def test_entirely_stale_chunk_is_dropped_without_executing_it():
    _, summary, _, _ = run_episode((0.6, 3.0))
    assert summary["pacing_timeline"][1]["dropped_rows"] == 48
    assert summary["rows_planned"] == 48


def test_median_window_switches_both_ways_and_700ms_is_still_async():
    pace = dream_real.OverlapPacing(30)
    pace.recent.extend([0.1, 0.7, 0.7, 0.7, 2.0])
    assert not pace.synchronous
    assert not pace.due(15, 20)
    assert pace.due(16, 20)
    pace.recent.extend([0.8, 0.8, 0.8])
    assert pace.synchronous
    assert not pace.due(48, 1)
    assert pace.due(0, 0)
    pace.recent.extend([0.2, 0.2, 0.2])
    assert not pace.synchronous
    assert pace.due(26, 20)


def test_row_horizon_counts_completed_model_rows_not_stretched_substeps():
    def stretch(rows):
        out = rows.copy()
        out[:, 1] += np.linspace(0, 0.5, len(out))
        return out

    plan_chunk = dream_real.Episode.plan_chunk

    def slow_plan(ep, reply):
        ep.mutate = stretch
        return plan_chunk(ep, reply)

    with mock.patch.object(dream_real.Episode, "plan_chunk", slow_plan):
        _, summary, _, _ = run_episode(changes={"speed_deg_s": 5})
    assert summary["stretched_rows"] > 0
    assert summary["pacing_timeline"][1]["executed_rows"] == 18
    assert summary["pacing_timeline"][1]["remaining_substeps"] > 30


def test_spliced_rows_pass_workspace_guard_jump_and_speed_checks():
    class RecordingGuard(NoGuard):
        def __init__(self):
            super().__init__()
            self.seen = []

        def step(self, q):
            self.seen.append(dict(q))
            return super().step(q)

    guard = RecordingGuard()
    runner, summary, _, _ = run_episode(guard=guard)
    dream_commands = [t["cmd"] for t in runner.tick_rows if t["phase"] == "dream"]
    for command in dream_commands:
        assert not runner.ep.box.violations(command)
        assert any(command == q for q in guard.seen)
    assert summary["substeps_sent"] > 60
    # Force a jump in the NEW spliced window. The real plan_chunk must trip on it.
    real_splice = dream_real.Episode.splice_reply

    def bad_splice(ep, reply, queue, moving):
        reply, dropped = real_splice(ep, reply, queue, moving)
        if moving:
            reply["q_l_chunk"][0][0] += 1
        return reply, dropped

    with (
        mock.patch.object(dream_real.Episode, "splice_reply", bad_splice),
        pytest.raises(AssertionError, match="jump"),
    ):
        run_episode()


def test_seam_jump_reference_is_actual_command_not_previous_chunk_end():
    jmap = core.JointMap.load()
    ep = SimpleNamespace(
        s=SimpleNamespace(rows=48),
        clock=lambda: 10.6,
        worker=SimpleNamespace(started_at=10),
        commanded=dict(POSE),
        jmap=jmap,
        seam_join=SeamAPI(),
    )
    future = [{**POSE, "shoulder_pan": POSE["shoulder_pan"] + i} for i in (1, 2, 3)]
    rows = np.repeat(np.asarray(core.motor_to_ik_row(POSE, jmap))[None], 48, axis=0)
    rows[:, 0] += 0.1
    reply, dropped = dream_real.Episode.splice_reply(ep, {"q_l_chunk": rows}, collections.deque(future), True)
    assert dropped == 12
    np.testing.assert_allclose(reply["q_l_chunk"][0], core.motor_to_ik_row(future[0], jmap))


def test_every_sent_request_is_in_ledger_including_an_uncollected_stop(tmp_path):
    ledger = core.Ledger(tmp_path / "ledger.json", cap=10)

    def stop_in_request(clock, stop, abort):
        # Stop on call 2 (it starts at ~1010, well before cap).
        if clock.t >= 1011.1:
            stop.set()

    _, summary, _, _ = run_episode((0.6, 4.0, 0.6), ledger=ledger, after=stop_in_request)
    requests = json.loads(ledger.path.read_text())["requests"]
    assert len(requests) == 2
    assert ledger.remaining() == 8
    assert summary["calls"] == 1
    assert summary["reason"] == "stop signal"


def test_ledger_cap_still_stops_before_extra_request(tmp_path):
    ledger = core.Ledger(tmp_path / "ledger.json", cap=1)
    with pytest.raises(AssertionError, match="BudgetEndError"):
        run_episode(ledger=ledger)
    assert ledger.remaining() == 0


def test_thread_shutdown_returns_while_request_is_blocked_then_joins_and_accounts(tmp_path):
    entered, unblock, closed = threading.Event(), threading.Event(), threading.Event()
    ledger = core.Ledger(tmp_path / "ledger.json", cap=2)

    def request(commanded):
        entered.set()
        assert unblock.wait(timeout=3)
        return {"q_l_chunk": []}

    stub = SimpleNamespace(request=request, close=closed.set)
    worker = dream_real.Worker(stub, time.monotonic, threaded=True, ledger=ledger, run_id="stop")
    worker.start({})
    assert entered.wait(timeout=1)
    assert ledger.remaining() == 1
    # A non-cooperative call must not hold the arm's stop path hostage.
    worker.shutdown()
    assert worker.thread.is_alive()
    assert not closed.is_set()
    unblock.set()
    worker.cleanup_thread.join(timeout=1)
    assert not worker.thread.is_alive()
    assert not worker.cleanup_thread.is_alive()
    assert closed.is_set()
    assert ledger.remaining() == 1
    assert json.loads(ledger.path.read_text())["requests"][0]["status"] == "billed"


def test_thread_reserve_failure_is_reported_instead_of_waiting_forever(tmp_path):
    ledger = core.Ledger(tmp_path / "ledger.json", cap=0)
    stub = SimpleNamespace(request=mock.Mock(), close=mock.Mock())
    worker = dream_real.Worker(stub, time.monotonic, threaded=True, ledger=ledger)
    worker.start({})
    worker.thread.join(timeout=1)
    reply, error, _ = worker.take()
    assert reply is None and "BudgetEndError" in error
    stub.request.assert_not_called()


def test_cancel_wakes_the_transport_socket_without_calling_close_or_waiting_on_policy():
    sock = mock.Mock()
    client = SimpleNamespace(_policy=SimpleNamespace())
    adapter = object.__new__(dream_real.DreamReal)
    adapter.client = client
    adapter.slide = SimpleNamespace(real=SimpleNamespace(_connection=SimpleNamespace(sock=sock)))
    adapter.cancel_request()
    sock.shutdown.assert_called_once()


def test_tank_refuses_overlap_at_cli_and_episode_boundary():
    with pytest.raises(SystemExit):
        dream_real.main(["--overlap", "--internal-stub", "--no-run-data"])
    settings = dream_real.Settings("test", 48, 2, 15, 60, 20, 1, core.TripLimits(), {}, overlap=True)
    with pytest.raises(dream_real.RefusedError, match="dry_air"):
        dream_real.Episode(*[None] * 7, settings, lambda: 0, None, Path("."), None, None)


def test_plan_binds_both_flags_and_preserves_pinned_tank_id(capsys):
    fixture = fixtures.EnvironmentTest()
    sync = fixture.plan("dry_air")
    overlap = fixture.plan("dry_air", overlap=True)
    changed = fixture.plan("dry_air", overlap=True, replan_after=24)
    assert len({sync["plan_id"], overlap["plan_id"], changed["plan_id"]}) == 3
    assert fixture.plan()["plan_id"] == "c6c40272db"
    with pytest.raises(dream_real.RefusedError, match="dry_air"):
        fixture.plan("tank", overlap=True)
    with pytest.raises(dream_real.RefusedError, match="dry_air"):
        fixture.plan("tank", replan_after=24)
    dream_real.print_plan(overlap)
    assert "pacing: overlapped, replan after 30 rows" in capsys.readouterr().out
    dream_real.print_plan(sync)
    assert "pacing: synchronous" in capsys.readouterr().out


def test_stateful_guard_restarts_at_commanded_seam_preserving_start_gate():
    guard = dream_real.Guard(dream_real.ENVIRONMENTS["dry_air"])
    resume = guard.resume_from
    seen = []

    def observe(commanded):
        start, relax = guard.targets.start, guard.targets.relax
        old = dict(guard.targets.target)
        resume(commanded)
        assert guard.targets.target == commanded
        assert guard.targets.start is start
        assert guard.targets.relax is relax
        seen.append((old, dict(commanded)))

    guard.resume_from = observe
    _, summary, _, _ = run_episode(guard=guard)
    assert len(seen) == 2
    assert seen[1][0] != seen[1][1]  # the unexecuted old tail was ahead of the arm
    assert summary["guard"]["ok"] > 0


def test_guard_rejection_of_a_spliced_substep_stops_the_episode():
    class RejectSeam(NoGuard):
        resumes = 0

        def resume_from(self, commanded):
            self.resumes += 1

        def step(self, target):
            if self.resumes == 2:
                return "rejected", target, "injected-seam-refusal"
            return super().step(target)

    with pytest.raises(AssertionError, match="tank_guard.*injected-seam-refusal"):
        run_episode(guard=RejectSeam())


def test_stop_mid_threaded_episode_releases_arm_before_request_unblocks(tmp_path):
    entered, unblock, arm_released = threading.Event(), threading.Event(), threading.Event()
    original_start, original_init = dream_real.Worker.start, dream_real.Worker.__init__
    original_episode = dream_real.Episode.__init__
    ledger = core.Ledger(tmp_path / "ledger.json", cap=3)
    worker_holder = []
    runner = fixtures.EpisodeOnFakesTest()

    def worker_init(worker, *args, **kwargs):
        kwargs.update(threaded=True, ledger=ledger, run_id="threaded-stop")
        original_init(worker, *args, **kwargs)
        worker_holder.append(worker)

    def request(stub, commanded):
        entered.set()
        assert unblock.wait(timeout=3)
        return {"q_l_chunk": []}

    def start(worker, commanded):
        original_start(worker, commanded)
        assert entered.wait(timeout=1)

    def episode_init(*args, **kwargs):
        kwargs["seam_join"] = SeamAPI()
        original_episode(*args, **kwargs)

    def patch_arm(arm):
        release = arm.release

        def released():
            assert entered.is_set() and not unblock.is_set()
            arm_released.set()
            release()

        arm.release = released

    def stop(clock, stop, abort):
        if entered.is_set():
            stop.set()

    try:
        with (
            mock.patch.object(dream_real.Worker, "__init__", worker_init),
            mock.patch.object(dream_real.Worker, "start", start),
            mock.patch.object(dream_real.Episode, "__init__", episode_init),
            mock.patch.object(fixtures.RecordingStub, "request", request),
        ):
            summary, _ = runner.episode(
                patch=patch_arm,
                after=stop,
                settings_changes={"overlap": True},
                environment=dream_real.ENVIRONMENTS["dry_air"],
            )
        assert summary["reason"] == "stop signal"
        assert arm_released.is_set()
        assert ledger.remaining() == 2
        assert worker_holder[0].thread.is_alive()
    finally:
        unblock.set()
        for worker in worker_holder:
            if worker.cleanup_thread:
                worker.cleanup_thread.join(timeout=1)
                assert not worker.cleanup_thread.is_alive()
            assert not worker.thread.is_alive()
