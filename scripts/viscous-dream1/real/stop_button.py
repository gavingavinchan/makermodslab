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

Under the button, the runner's own status (`<pid-file>.status.json`, written every tick and in every
phase, the stop path and the hold included), not just "pid alive": a joint gone limp (Gavin, 2026-11-14:
"the arm will suddenly die, but the line under the big stop button still says alive, so thats wrong"),
HOLDING mid-air, no fresh data from the run, or released. While the runner HOLDS, the button is the
RELEASE press (a click; keys never release). A missing or corrupt status file falls back to the pid check.

The power switch is still the hard stop. Fallback without a display: kill -TERM "$(cat /tmp/dream_real.pid)".
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import time
from pathlib import Path

DEFAULT_PID_FILE = "/tmp/dream_real.pid"
ARM_SECOND_S = 1.0  # the release-all press is refused this soon after STOP
RUNNER = "dream_real.py"
STATUS_SUFFIX = ".status.json"  # dream_real_core.STATUS_SUFFIX
# The runner writes its status every 50 ms tick (10 Hz while holding, ~30 Hz during the return); a stop's
# re-anchor or the release can take a few hundred ms. Past this, with the pid alive, the run is not
# reporting: the loop is stuck, or the arm stopped answering.
STALE_S = 1.0
JOINT_WORDS = {
    "shoulder_pan": "SHOULDER PAN",
    "shoulder_lift": "SHOULDER",
    "elbow_flex": "ELBOW",
    "wrist_flex": "WRIST FLEX",
    "wrist_yaw": "WRIST YAW",
    "wrist_roll": "WRIST ROLL",
}
COLOURS = {  # (background, text) of the status panel
    "ok": ("#0b3d0b", "#66ff66"),
    "info": ("#1a1a1a", "#dddddd"),
    "warn": ("#b35c00", "white"),
    "alarm": ("#d00000", "white"),
    "nodata": ("#6a00a8", "white"),
}


def read_status(pid_file) -> dict | None:
    """The runner's status file, or None when it is missing, unreadable or not a status."""
    try:
        status = json.loads(Path(f"{pid_file}{STATUS_SUFFIX}").read_text())
    except (OSError, ValueError):
        return None
    return status if isinstance(status, dict) and "state" in status else None


def status_view(pid: int | None, pid_line: str, status: dict | None, now: float) -> tuple[str, str, str]:
    """(headline, level, detail) for the panel under the button: a pure function of the live runner pid
    (None: none alive), the pid check's own line, the status file (None: missing or corrupt) and the
    wall clock. A status from another pid is ignored (a stale file from an earlier run)."""
    if status is not None and pid is not None and status.get("pid") not in (None, pid):
        status = None
    if status is None:
        return ("RUNNER ALIVE (no status)" if pid else "NO RUNNER"), "info", pid_line
    state = status.get("state")
    message = str(status.get("message") or "")[:160]
    joints = status.get("joints") or {}
    limp = [m for m in status.get("limp") or [] if isinstance(m, str)]
    facts = "; ".join(
        f"{m} {j.get('torque_nm')} Nm {j.get('temp_c')} C"
        for m, j in joints.items()
        if isinstance(j, dict) and j.get("torque_nm") is not None
    )
    if state == "released":
        dead = f" ({', '.join(limp)} went limp)" if limp else ""
        return "RELEASED: torque off", "ok", f"{message}{dead}"
    if pid is None:
        return "RUNNER GONE: torque state unknown, check the arm", "alarm", f"last: {state}: {message}"
    if limp:
        words = " + ".join(JOINT_WORDS.get(m, m.upper()) for m in limp)
        return f"{words} WENT LIMP: SUPPORT THE ARM, then RELEASE", "alarm", f"{message} | {facts}"
    written = status.get("written")
    last = status.get("last_reading")
    if not isinstance(written, (int, float)) or now - written > STALE_S:
        age = f"{now - written:.1f} s" if isinstance(written, (int, float)) else "?"
        return "NO DATA FROM THE RUN", "nodata", f"status {age} old ({state}: {message})"
    energized = state in ("running", "returning", "holding")
    if energized and (not isinstance(last, (int, float)) or now - last > STALE_S):
        age = f"{now - last:.1f} s" if isinstance(last, (int, float)) else "never"
        return "NO DATA FROM THE RUN", "nodata", f"no fresh reading from the arm for {age} ({state})"
    if state == "holding":
        return "HOLDING MID-AIR: support the arm, then press RELEASE", "warn", f"{message} | {facts}"
    if state == "returning":
        return "RETURNING TO REST", "warn", message
    if state == "releasing":
        return "RELEASING TORQUE", "warn", message
    if state == "starting":
        return "STARTING (not energized yet)", "info", message
    return f"RUNNING: {status.get('phase')}", "ok", facts or message


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

    def __init__(
        self, pid_file, *, kill=os.kill, clock=time.monotonic, command=runner_command, wall=time.time
    ):
        self.pid_file = Path(pid_file)
        self.kill, self.clock, self.command, self.wall = kill, clock, command, wall
        self.pid = None  # the runner the presses so far went to
        self.presses = 0
        self.first_at = None
        self.message = ""
        self.holding = False  # the runner's status says it HOLDS: the next signal releases all torque

    def view(self) -> tuple[str, str, str]:
        """Refresh, then (headline, level, detail) for the panel. Never raises: the button must keep working."""
        pid_line = self.refresh()
        try:
            status = read_status(self.pid_file)
            view = status_view(self.pid, pid_line, status, self.wall())
        except Exception as error:  # noqa: BLE001 - a bad status falls back to the pid check
            status, view = (
                None,
                (
                    "RUNNER ALIVE (status unreadable)" if self.pid else "NO RUNNER",
                    "info",
                    f"{pid_line}; {error}",
                ),
            )
        self.holding = bool(
            self.pid and status and status.get("pid") in (None, self.pid) and status.get("state") == "holding"
        )
        return view

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
        if self.presses >= 1 or self.holding:
            if from_key:
                self.message = "keys only STOP; click the button to release all torque"
                return self.message
            if self.first_at is not None and self.clock() - self.first_at < ARM_SECOND_S:
                self.message = "release-all ignored: too soon after STOP (click again)"
                return self.message
        try:
            self.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            self.message = f"NOT SENT: pid {pid} exited"
            return self.message
        self.presses += 1
        if self.presses == 1 and not self.holding:
            self.first_at = self.clock()
            self.message = f"STOP sent (SIGTERM to {pid}): gripper off, returning to rest, then torque off"
        else:
            self.message = f"RELEASE ALL sent (SIGTERM #{self.presses} to {pid}): all torque off now"
        return self.message

    @property
    def label(self) -> str:
        return "RELEASE ALL TORQUE NOW\n(arm drops)" if self.presses >= 1 or self.holding else "STOP"


class App:
    def __init__(self, logic: StopLogic):
        import tkinter as tk

        self.logic = logic
        self.root = tk.Tk()
        self.root.title("Dream-1 STOP")
        self.root.attributes("-topmost", True)
        self.root.geometry("580x480+40+40")
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
        self.headline = tk.Label(
            self.root, text="", bg="black", fg="white", font=("Helvetica", 22, "bold"), wraplength=540
        )
        self.headline.pack(fill="x", padx=12, ipady=8)
        self.status = tk.Label(
            self.root, text="", bg="black", fg="white", font=("Helvetica", 13), wraplength=540
        )
        self.status.pack(fill="x", padx=12)
        self.note = tk.Label(
            self.root, text="", bg="black", fg="#ffcc00", font=("Helvetica", 12), wraplength=540
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
        try:
            headline, level, detail = self.logic.view()
        except Exception as error:  # noqa: BLE001 - never let the panel stop the button
            headline, level, detail = "STATUS ERROR", "info", str(error)
        bg, fg = COLOURS.get(level, COLOURS["info"])
        if level == "alarm" and int(time.monotonic() * 2) % 2:  # blink: it must be seen from the tank
            bg, fg = "white", "#d00000"
        self.headline.configure(text=headline, bg=bg, fg=fg)
        self.status.configure(text=detail)
        self.redraw()
        self.root.after(250, self.tick)

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
