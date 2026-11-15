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
"""Reparent Dream processes before returning to Lab; never signal them.

Fork only in a fresh helper interpreter, never in Lab's multithreaded server.
A detached monitor reaps the child and records its exit code. No Lab shutdown
snapshot or process-group signal can reach either monitor or child.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import time
from pathlib import Path

import psutil


def identity(pid):
    return {"pid": pid, "started": psutil.Process(pid).create_time()}


def alive(record, script=None):
    try:
        process = psutil.Process(record["pid"])
        if process.create_time() != record["started"] or process.status() == psutil.STATUS_ZOMBIE:
            return False
        if script is not None:
            return any(Path(arg).resolve() == Path(script).resolve() for arg in process.cmdline()[1:2])
        return process.is_running()
    except psutil.NoSuchProcess:
        return False
    # Permission errors intentionally propagate: an unreadable identity is
    # not evidence that the recorded runner has exited.


class DetachedProcess:
    def __init__(self, record, result):
        self.record, self.result = record, Path(result)
        self.pid = record["pid"]
        self.returncode = None

    def poll(self):
        if self.returncode is None:
            try:
                self.returncode = json.loads(self.result.read_text())["returncode"]
            except FileNotFoundError:
                if not alive(self.record):
                    self.returncode = 1  # detached monitor lost; never presume success
        return self.returncode

    def wait(self, timeout=None):
        start = time.monotonic()
        while self.poll() is None:
            if timeout is not None and time.monotonic() - start >= timeout:
                raise subprocess.TimeoutExpired(str(self.pid), timeout)
            time.sleep(0.02)
        return self.returncode


def spawn(args, *, cwd, log, result):
    payload = {"args": args, "cwd": str(cwd), "log": str(log), "result": str(result)}
    launched = subprocess.run(
        [sys.executable, "-m", "makermodslab.dream_process", json.dumps(payload)],
        cwd=cwd,
        capture_output=True,
        text=True,
        check=True,
    )
    record = json.loads(launched.stdout)
    if "error" in record:
        raise OSError(record["error"])
    return DetachedProcess(record, result)


def main(payload):
    read_fd, write_fd = os.pipe()
    first = os.fork()
    if first:
        os.close(write_fd)
        os.waitpid(first, 0)
        with os.fdopen(read_fd) as pipe:
            print(pipe.read(), flush=True)
        return
    os.close(read_fd)
    os.setsid()
    intermediate_pid = os.getpid()
    if os.fork():
        os._exit(0)
    # Do not create the button/runner until reparenting is complete, even
    # if this grandchild is scheduled before the intermediate parent exits.
    while os.getppid() == intermediate_pid:
        time.sleep(0.001)
    # No pipes back to subprocess.run remain open in the detached monitor.
    with open(os.devnull) as null, open(payload["log"], "a") as log:
        os.dup2(null.fileno(), 0)
        os.dup2(log.fileno(), 1)
        os.dup2(log.fileno(), 2)
        with os.fdopen(write_fd, "w") as pipe:
            try:
                proc = subprocess.Popen(
                    payload["args"],
                    cwd=payload["cwd"],
                    stdin=null,
                    stdout=log,
                    stderr=log,
                    start_new_session=True,
                )
                record = identity(proc.pid)
            except Exception as error:
                pipe.write(json.dumps({"error": str(error)}))
                pipe.flush()
                os._exit(1)
            pipe.write(json.dumps(record))
        code = proc.wait()
        path = Path(payload["result"])
        tmp = path.with_suffix(".tmp")
        tmp.write_text(json.dumps({"returncode": code}))
        tmp.replace(path)
    os._exit(0)


if __name__ == "__main__":
    main(json.loads(sys.argv[1]))
