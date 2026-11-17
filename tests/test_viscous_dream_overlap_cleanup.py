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
"""Safe expectations from REVIEW-overlap's three adversarial reproductions.

No hardware/network/client instance; Events synchronize real thread teardown.
The transport regression uses the optional installed Dream transport with fake HTTP.
"""

import http.client
import json
import socket
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import pytest

from tests import test_viscous_dream_real as fixtures
from tests.test_viscous_dream_overlap import SeamAPI

r, core = fixtures.dream_real, fixtures.core


@pytest.fixture(autouse=True)
def forbid_network(monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError("network forbidden in overlap cleanup regression")

    monkeypatch.setattr(socket, "create_connection", forbidden)
    monkeypatch.setattr(socket.socket, "connect", forbidden)


def test_cleanup_thread_start_failure_still_stops_arm_and_logs(capsys):
    runner = fixtures.EpisodeOnFakesTest()
    init, start = r.Episode.__init__, threading.Thread.start

    def episode_init(*args, **kwargs):
        kwargs["seam_join"] = SeamAPI()
        init(*args, **kwargs)

    def no_cleanup_thread(thread):
        if thread.name == "dream-request-join":
            raise RuntimeError("can't start new thread")
        return start(thread)

    with (
        mock.patch.object(r.Episode, "__init__", episode_init),
        mock.patch.object(threading.Thread, "start", no_cleanup_thread),
        mock.patch.object(r.DreamInternalStub, "close") as close,
    ):
        summary, arm = runner.episode(
            max_calls=1, settings_changes={"overlap": True}, environment=r.ENVIRONMENTS["dry_air"]
        )
    assert not arm.energized and not runner.ep.energized
    for action in ("re-anchor:", "release gripper:", "release:"):
        assert any(event.startswith(action) for event in arm.events)
    close.assert_not_called()  # no synchronous fallback on the arm thread
    assert summary["reason"] == "call cap 1 reached"
    assert "can't start new thread" in summary["cleanup_errors"][0]
    assert "can't start new thread" in capsys.readouterr().err


def test_take_returns_during_thread_exit_and_old_handles_are_reaped():
    local = threading.local()
    teardown, unblock, taken, closed = (threading.Event() for _ in range(4))
    calls = 0

    class DelayedThreadExit:
        def __del__(self):
            teardown.set()
            unblock.wait(3)

    def request(commanded):
        nonlocal calls
        calls += 1
        if calls == 1:
            local.value = DelayedThreadExit()
        return {"call": calls}

    worker = r.Worker(SimpleNamespace(request=request, close=closed.set), time.monotonic, threaded=True)
    collector, first = None, None
    results = []
    try:
        worker.start({})
        first = worker.thread
        assert teardown.wait(1) and worker.result is not None

        def collect():
            results.append(worker.take())
            taken.set()

        collector = threading.Thread(target=collect)
        collector.start()
        assert taken.wait(1), "take() waited for thread termination after result publication"
        assert results[0][0] == {"call": 1}
        assert first.is_alive()
        worker.start({})  # keep the still-exiting old handle when starting another request
        worker.thread.join(1)  # the test thread may wait; the control loop must not
        assert worker.take()[0] == {"call": 2}
        assert first in worker.threads
        worker.shutdown()
        assert not closed.is_set()  # close cannot race either request thread's teardown
    finally:
        unblock.set()
        if first:
            first.join(1)
        if collector:
            collector.join(1)
        if worker.cleanup_thread:
            worker.cleanup_thread.join(1)
    assert closed.is_set()
    assert not worker.cleanup_thread.is_alive()
    assert worker.threads == []


def test_stop_does_not_retry_sent_post_under_one_ledger_reservation(tmp_path, monkeypatch):
    # Append, preserving the repo's pinned packages. No DreamClient or credentials are read.
    sites = sorted(Path("~/viscous-dream-real/.venv").expanduser().glob("lib/python3.*/site-packages"))
    monkeypatch.setattr(sys, "path", [*sys.path, *map(str, sites)])
    transport_module = pytest.importorskip("dream.transport")
    sent, interrupted, release, joining = (threading.Event() for _ in range(4))
    sends = []

    class Sock:
        def settimeout(self, value):
            pass

        def getsockopt(self, *args):
            raise OSError("fake socket")

        def shutdown(self, how):
            interrupted.set()

    class Connection:
        def __init__(self, first):
            self.first, self.sock, self.timeout = first, Sock(), 45

        def request(self, method, path, body, headers):
            sends.append((method, path, body))
            sent.set()

        def getresponse(self):
            if self.first:
                assert release.wait(3)
                if interrupted.is_set():
                    raise http.client.RemoteDisconnected("shutdown before status line")
            return SimpleNamespace(status=200, read=lambda: b'{"q_l_chunk": []}')

        def close(self):
            pass

    first, second = Connection(True), Connection(False)
    transport = transport_module.Transport("regression.invalid", 1, 45000)
    transport._connection, transport._reused = first, True

    def connect():
        if transport._connection is None:
            transport._connection = second
        return transport._connection

    transport._connect = connect
    adapter = object.__new__(r.DreamReal)
    adapter.slide = SimpleNamespace(real=transport)
    # Old runner cancels this socket; fixed runner must never call the hook.
    cancel = mock.Mock(side_effect=lambda: getattr(adapter, "cancel_request", lambda: None)())
    dream = SimpleNamespace(
        request=lambda cmd: transport.request({}, {}), close=transport.close, cancel_request=cancel
    )
    ledger = core.Ledger(tmp_path / "ledger.json", cap=1)
    worker = r.Worker(dream, time.monotonic, threaded=True, ledger=ledger, run_id="stop", max_calls=1)
    try:
        worker.start({})
        assert sent.wait(1)
        join = worker.thread.join

        def observed_join(*args, **kwargs):
            joining.set()
            return join(*args, **kwargs)

        with mock.patch.object(worker.thread, "join", observed_join):
            worker.shutdown()
            assert joining.wait(1)  # reaper has passed any cancellation and is awaiting the request
            assert worker.thread.is_alive()
            assert ledger.remaining() == 0  # still reserved and chargeable before the reply
            release.set()
            worker.cleanup_thread.join(1)
        assert not worker.cleanup_thread.is_alive()
        assert len(sends) == 1, "cancel triggered an unaccounted stale-connection POST retry"
        assert not interrupted.is_set()
        cancel.assert_not_called()
        requests = json.loads(ledger.path.read_text())["requests"]
        assert len(requests) == 1 and requests[0]["status"] == "billed"
    finally:
        release.set()
        worker.thread.join(1)
        if worker.cleanup_thread:
            worker.cleanup_thread.join(1)
