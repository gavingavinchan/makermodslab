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
"""Hardware-free terminal camera reservation and Lab handoff contracts."""

import io
import json
import select
import signal
import subprocess
import sys
import urllib.error
from pathlib import Path
from types import SimpleNamespace

import psutil
import pytest

from makermodslab import dream_camera_lock as lock

REAL = Path(__file__).resolve().parents[1] / "scripts/viscous-dream1/real"
sys.path.insert(0, str(REAL))
import dream_real  # noqa: E402


def process(pid):
    assert pid == 123
    return SimpleNamespace(
        is_running=lambda: True,
        create_time=lambda: 42.0,
        cmdline=lambda: ["python", "/runner/dream_real.py", "--preview"],
    )


@pytest.mark.parametrize("ending", ["normal", "trip", "exception"])
def test_marker_atomic_and_cleanup(tmp_path, ending):
    path = tmp_path / "marker.json"

    def run():
        with lock.terminal_run("preview", "example", path=path, pid=123, process=process, clock=lambda: 99):
            assert lock.terminal_run_active(path, process=process)
            assert json.loads(path.read_text()) == {
                "pid": 123,
                "process_start_time": 42.0,
                "mode": "preview",
                "run_name": "example",
                "started_wall_time": 99,
            }
            assert list(tmp_path.iterdir()) == [path]
            if ending == "exception":
                raise RuntimeError("injected")
            return 1 if ending == "trip" else 0

    if ending == "exception":
        with pytest.raises(RuntimeError, match="injected"):
            run()
    else:
        assert run() == (1 if ending == "trip" else 0)
    assert not path.exists()


@pytest.mark.parametrize("stale", ["dead", "reused", "other_script", "malformed", "bad_pid"])
def test_stale_markers_cleaned(tmp_path, stale):
    path = tmp_path / "marker.json"
    marker = {"pid": 123, "process_start_time": 42.0}
    if stale == "reused":
        marker["process_start_time"] = 1.0
    if stale == "bad_pid":
        marker["pid"] = -1
    path.write_text("{" if stale == "malformed" else json.dumps(marker))

    def inspect(pid):
        if stale == "dead":
            raise psutil.NoSuchProcess(pid)
        proc = process(pid)
        if stale == "other_script":
            proc.cmdline = lambda: ["python", "/runner/not_dream_real.py"]
        return proc

    assert not lock.terminal_run_active(path, process=inspect)
    assert not path.exists()


def test_cleanup_does_not_remove_another_marker(tmp_path):
    path = tmp_path / "marker.json"
    with lock.terminal_run("live", "one", path=path, pid=123, process=process):
        path.write_text("replacement")
    assert path.read_text() == "replacement"


@pytest.mark.parametrize("result", [{"success": False, "closed": 0}, {"success": True}, {}])
def test_release_refuses_unconfirmed_response(result):
    with pytest.raises(dream_real.RefusedError, match="close the camera previews in the Lab GUI"):
        dream_real.release_lab_previews(
            "http://lab",
            True,
            urlopen=lambda *a, **kw: io.BytesIO(json.dumps(result).encode()),
            sleep=lambda _: pytest.fail("must not wait after failed release"),
        )


def test_release_post_and_os_settle_order():
    calls = []

    def urlopen(request, timeout):
        calls.append((request.method, request.full_url, timeout))
        return io.BytesIO(b'{"success": true, "closed": 2}')

    dream_real.release_lab_previews("http://lab", False, urlopen=urlopen, sleep=lambda s: calls.append(s))
    assert calls == [("POST", "http://lab/api/v1/camera-preview/release", 5), 2.0]


@pytest.mark.parametrize("allow_down", [True, False])
@pytest.mark.parametrize("responding", [True, False])
def test_only_down_lab_can_skip_release(allow_down, responding):
    def urlopen(*a, **kw):
        if responding:
            raise urllib.error.HTTPError("http://lab", 500, "failed", {}, None)
        raise urllib.error.URLError("refused")

    if allow_down and not responding:
        dream_real.release_lab_previews("http://lab", allow_down, urlopen=urlopen)
    else:
        with pytest.raises(dream_real.RefusedError, match="close the camera previews"):
            dream_real.release_lab_previews("http://lab", allow_down, urlopen=urlopen)


@pytest.mark.parametrize("ending", [0, 1, "exception", "release_failure"])
@pytest.mark.parametrize("mode", ["preview", "live", "rehearse-live"])
def test_runner_claims_before_release_and_open_and_cleans(tmp_path, monkeypatch, mode, ending):
    from makermodslab.utils.config import dream_terminal_run_path

    path = dream_terminal_run_path(tmp_path)
    args = SimpleNamespace(
        preview=mode == "preview",
        live=mode != "preview",
        rehearse=mode == "rehearse-live",
        fake_cameras=False,
        lab_url="http://lab",
        lab_down_ok=False,
        pid_file=None,
        stop_button_pid=None,
    )
    ctx = {"home": tmp_path, "out": tmp_path / "run", "arm_path": None, "arm": {}, "plan": {}}
    calls = []

    def release(*a):
        assert path.exists()
        calls.append("release")
        if ending == "release_failure":
            raise dream_real.RefusedError("injected release failure")

    def opened(*a):
        assert path.exists()
        calls.append("arm" if args.rehearse else "cameras")
        if ending == "exception":
            raise RuntimeError("injected open failure")
        return 1 if ending == "release_failure" else ending

    monkeypatch.setattr(dream_real, "release_lab_previews", release)
    monkeypatch.setattr(dream_real, "_run_preview", opened)
    monkeypatch.setattr(dream_real, "_run_live", opened)
    monkeypatch.setattr(dream_real, "live_facts", lambda *a: None)
    monkeypatch.setattr(dream_real.core, "live_refusals", lambda *a: [])
    monkeypatch.setattr(dream_real, "rehearse_refusals", lambda *a: [])
    # Stubbed function only: never the --live CLI or any device path.
    run = dream_real.run_preview if args.preview else dream_real.run_live
    old = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP, signal.SIGUSR1)}
    try:
        if ending == "exception" or (ending == "release_failure" and not args.rehearse):
            with pytest.raises((RuntimeError, dream_real.RefusedError), match="injected"):
                run(args, ctx)
        else:
            assert run(args, ctx) == (1 if ending == "release_failure" else ending)
    finally:
        for sig, handler in old.items():
            signal.signal(sig, handler)
    assert calls == (
        ["arm"] if args.rehearse else ["release"] if ending == "release_failure" else ["release", "cameras"]
    )
    assert not path.exists()


@pytest.mark.parametrize("mode", ["preview", "live"])
def test_sigterm_unwinds_marker_in_spawned_fake_runner(tmp_path, mode):
    # Signal ONLY this test's child. No CLI hardware modes or Dream imports.
    script = f"""
import sys, threading
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0, {str(REAL)!r})
import dream_real as d
home = Path({str(tmp_path)!r})
args = SimpleNamespace(rehearse=False, live={mode == "live"}, fake_cameras=False,
    lab_url='unused', lab_down_ok=False, pid_file=None, stop_button_pid=None)
ctx = dict(home=home, out=home/'run', arm_path=None, arm={{}}, plan={{}})
d.release_lab_previews = lambda *a: None
d.live_facts = lambda *a: None
d.core.live_refusals = lambda *a: []
def preview(*a):
    print('ready', flush=True)
    threading.Event().wait(30)
def live(*a):
    print('ready', flush=True)
    assert a[-2].wait(30)
    from makermodslab.utils.config import dream_terminal_run_path
    assert dream_terminal_run_path(home).exists()  # still held during stop
    return 0
d._run_preview = preview
d._run_live = live
raise SystemExit(d.run_{mode}(args, ctx))
"""
    from makermodslab.utils.config import dream_terminal_run_path

    child = subprocess.Popen(
        [sys.executable, "-c", script], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
    )
    try:
        assert select.select([child.stdout], [], [], 15)[0], "child did not become ready"
        assert child.stdout.readline().strip() == "ready"
        assert dream_terminal_run_path(tmp_path).exists()
        child.terminate()
        _, errors = child.communicate(timeout=15)
        assert child.returncode == (128 + signal.SIGTERM if mode == "preview" else 0), errors
        assert not dream_terminal_run_path(tmp_path).exists()
    finally:
        if child.poll() is None:
            child.kill()
            child.communicate()


def test_atexit_removes_marker_even_without_context_unwind(tmp_path):
    from makermodslab.utils.config import dream_terminal_run_path

    script = f"""
from pathlib import Path
from makermodslab.dream_camera_lock import terminal_run
from makermodslab.utils.config import dream_terminal_run_path
claim = terminal_run('preview', 'test', path=dream_terminal_run_path(Path({str(tmp_path)!r})))
claim.__enter__()
"""
    result = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stderr
    assert not dream_terminal_run_path(tmp_path).exists()


@pytest.mark.parametrize("failure", [RuntimeError("connect failed"), SystemExit(143)])
def test_partial_camera_open_releases_before_marker_can_unwind(monkeypatch, failure):
    from lerobot.cameras import utils
    from makermodslab import camera_identity, record
    from makermodslab.utils import config

    closed = []
    first = SimpleNamespace(connect=lambda: None, disconnect=lambda: closed.append("first"))

    def fail():
        raise failure

    second = SimpleNamespace(connect=fail, disconnect=lambda: closed.append("second"))
    monkeypatch.setattr(utils, "make_cameras_from_configs", lambda *a: {"a": first, "b": second})
    monkeypatch.setattr(camera_identity, "resolve_cv2_index", lambda *a: 0)
    monkeypatch.setattr(config, "session_camera_config", lambda *a: {})
    monkeypatch.setattr(record, "_build_camera_configs", lambda *a: {})
    with pytest.raises(type(failure)):
        dream_real.open_cameras({"cameras": [{"name": "a"}, {"name": "b"}]}, fake=False)
    assert closed == ["first", "second"]
