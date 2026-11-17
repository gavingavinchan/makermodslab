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
"""Cross-process camera reservation owned by dream_real.py, never by Lab.

PID alone is unsafe (reuse). Readers verify both process birth time and the
script name, and discard stale markers. No process is ever signalled here.
"""

import atexit
import contextlib
import json
import os
import tempfile
import time
from pathlib import Path

import psutil

from .utils.config import dream_terminal_run_path


def _remove_if_unchanged(path: Path, raw: bytes) -> None:
    with contextlib.suppress(OSError):
        if path.read_bytes() == raw:
            path.unlink()


def terminal_run_active(path=None, *, process=psutil.Process) -> bool:
    path = dream_terminal_run_path() if path is None else Path(path)
    try:
        raw = path.read_bytes()
    except FileNotFoundError:
        return False
    try:
        marker = json.loads(raw)
        pid = marker["pid"]
        if not isinstance(pid, int) or isinstance(pid, bool) or pid <= 0:
            raise ValueError("invalid pid")
        proc = process(pid)
        valid = (
            proc.is_running()
            and proc.create_time() == marker["process_start_time"]
            and any(Path(arg).name == "dream_real.py" for arg in proc.cmdline())
        )
    except psutil.AccessDenied:
        # A reader unable to verify identity must not open a competing camera.
        return True
    except (psutil.NoSuchProcess, ValueError, TypeError, KeyError):
        valid = False
    if not valid:
        _remove_if_unchanged(path, raw)
    return valid


@contextlib.contextmanager
def terminal_run(mode, run_name, *, path=None, pid=None, process=psutil.Process, clock=time.time):
    """Publish atomically before any device open; retain through the entire stop.

    The finally covers normal returns, trips and exceptions. atexit also covers
    interpreter exits; callers retain their existing graceful signal handling.
    """
    path = dream_terminal_run_path() if path is None else Path(path)
    if terminal_run_active(path, process=process):
        raise RuntimeError("A Dream run already owns the cameras")
    pid = os.getpid() if pid is None else pid
    raw = json.dumps(
        {
            "pid": pid,
            "process_start_time": process(pid).create_time(),
            "mode": mode,
            "run_name": run_name,
            "started_wall_time": clock(),
        },
        allow_nan=False,
    ).encode()
    path.parent.mkdir(parents=True, exist_ok=True)

    def cleanup():
        _remove_if_unchanged(path, raw)

    atexit.register(cleanup)
    try:
        with tempfile.NamedTemporaryFile(dir=path.parent, delete=False) as f:
            temporary = Path(f.name)
            try:
                f.write(raw)
                f.flush()
                os.fsync(f.fileno())
                os.replace(temporary, path)
            finally:
                temporary.unlink(missing_ok=True)
        yield
    finally:
        cleanup()
        atexit.unregister(cleanup)
