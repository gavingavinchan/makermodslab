#!/usr/bin/env python3
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
"""The big red button for dream_real.py (Gavin: "one big red button to stop").

A small always-on-top window, a separate process from the runner, so it works even if the runner's
loop is stuck. It sends SIGTERM to the pid in the runner's --pid-file:

  first press   STOP: the runner releases the gripper at once, walks the arm back to its rest at
                <= 15 deg/s, then releases all torque.
  second press  RELEASE ALL TORQUE NOW (arm drops): aborts the return and releases everything.

Space, Enter and Escape press STOP while the window has focus. They never give the second press: a
held key auto-repeats, and that must not skip the return. The second press is a click on the button,
taken only after ARM_SECOND_S, so a double click cannot do it either. Before signalling it checks the
pid is alive and is a dream_real.py process (a stale pid file can name an unrelated process).

  .venv/bin/python scripts/viscous-dream1/real/stop_button.py [--pid-file /tmp/dream_real.pid]

The power switch is still the hard stop. Fallback without a display: kill -TERM "$(cat /tmp/dream_real.pid)".
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import time
from pathlib import Path

DEFAULT_PID_FILE = "/tmp/dream_real.pid"
ARM_SECOND_S = 1.0  # the release-all press is refused this soon after STOP
RUNNER = "dream_real.py"


def runner_command(pid: int) -> str | None:
    """The process's command line, or None if it is gone."""
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return None
    except PermissionError:
        pass  # alive, another user's
    try:
        out = subprocess.run(
            ["ps", "-p", str(pid), "-o", "command="], capture_output=True, text=True, timeout=2
        )
    except Exception:  # noqa: BLE001
        return ""
    return out.stdout.strip() or None


class StopLogic:
    """Everything but the window: pid lookup, the two stages, the lockout. Injected kill/clock for tests."""

    def __init__(self, pid_file, *, kill=os.kill, clock=time.monotonic, command=runner_command):
        self.pid_file = Path(pid_file)
        self.kill, self.clock, self.command = kill, clock, command
        self.pid = None  # the runner the presses so far went to
        self.presses = 0
        self.first_at = None
        self.message = ""

    def target(self) -> tuple[int | None, str]:
        """(pid, status line). pid is None unless it is a live dream_real.py."""
        try:
            pid = int(self.pid_file.read_text().split()[0])
        except (OSError, ValueError, IndexError):
            return None, f"no runner: {self.pid_file} missing or empty"
        cmd = self.command(pid)
        if cmd is None:
            return None, f"pid {pid} is not running (runner exited)"
        if cmd and RUNNER not in cmd:
            return None, f"pid {pid} is not {RUNNER} ({cmd[:60]}); not signalling it"
        return pid, f"runner pid {pid} alive"

    def refresh(self) -> str:
        pid, status = self.target()
        if pid != self.pid:
            self.pid, self.presses, self.first_at = pid, 0, None  # runner gone or a new run: back to STOP
        return status

    def press(self, *, from_key=False) -> str:
        pid, status = self.target()
        if pid is None:
            self.message = f"NOT SENT: {status}"
            return self.message
        if pid != self.pid:
            self.pid, self.presses, self.first_at = pid, 0, None
        if self.presses >= 1:
            if from_key:
                self.message = "keys only STOP; click the button to release all torque"
                return self.message
            if self.clock() - self.first_at < ARM_SECOND_S:
                self.message = "release-all ignored: too soon after STOP (click again)"
                return self.message
        try:
            self.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            self.message = f"NOT SENT: pid {pid} exited"
            return self.message
        self.presses += 1
        if self.presses == 1:
            self.first_at = self.clock()
            self.message = f"STOP sent (SIGTERM to {pid}): gripper off, returning to rest, then torque off"
        else:
            self.message = f"RELEASE ALL sent (SIGTERM #{self.presses} to {pid}): all torque off now"
        return self.message

    @property
    def label(self) -> str:
        return "RELEASE ALL TORQUE NOW\n(arm drops)" if self.presses >= 1 else "STOP"


class App:
    def __init__(self, logic: StopLogic):
        import tkinter as tk

        self.logic = logic
        self.root = tk.Tk()
        self.root.title("Dream-1 STOP")
        self.root.attributes("-topmost", True)
        self.root.geometry("420x340+40+40")
        self.root.configure(bg="black")
        # A Label, not a Button: macOS's native buttons ignore background colours.
        self.button = tk.Label(
            self.root,
            text=logic.label,
            bg="#d00000",
            fg="white",
            font=("Helvetica", 40, "bold"),
            relief="raised",
            bd=6,
            cursor="hand2",
        )
        self.button.pack(fill="both", expand=True, padx=12, pady=(12, 6))
        self.status = tk.Label(self.root, text="", bg="black", fg="white", font=("Helvetica", 13))
        self.status.pack(fill="x", padx=12)
        self.note = tk.Label(
            self.root, text="", bg="black", fg="#ffcc00", font=("Helvetica", 12), wraplength=396
        )
        self.note.pack(fill="x", padx=12, pady=(0, 10))
        self.button.bind("<ButtonPress-1>", lambda _e: self.press(from_key=False))
        for key in ("<space>", "<Return>", "<KP_Enter>", "<Escape>"):
            self.root.bind(key, lambda _e: self.press(from_key=True))
        self.root.focus_force()
        self.tick()

    def press(self, from_key):
        self.note.configure(text=self.logic.press(from_key=from_key))
        self.redraw()

    def redraw(self):
        second = self.logic.presses >= 1
        self.button.configure(
            text=self.logic.label,
            bg="#7a0000" if second else "#d00000",
            font=("Helvetica", 24 if second else 40, "bold"),
        )

    def tick(self):
        status = self.logic.refresh()
        self.status.configure(text=status, fg="#66ff66" if "alive" in status else "#ff6666")
        self.redraw()
        self.root.after(500, self.tick)

    def run(self):
        self.root.mainloop()


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--pid-file", default=DEFAULT_PID_FILE, help="the runner's --pid-file")
    args = p.parse_args(argv)
    App(StopLogic(args.pid_file)).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
