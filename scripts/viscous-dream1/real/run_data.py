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
"""Raw run bundles for dream_real.py, outside git, kept as a ring of the last RING runs.

Gavin (2026-11-14): "all run from now to the future, keep all the last 10 run logs, like all the raw
data, if possible even camera in the future, so we can always go back in time to figure out what is
wrong. then discard by default unless needed to be kept".

Every run of dream_real.py (dry, rehearse, preview, live) writes RUN_DATA/<run-name>/:

  meta.json      command line, git SHA, `git diff --stat`, host, python, MAKERMODSLAB_HOME, ports, cameras
  console.log    everything the run printed (stdout and stderr)
  ticks.jsonl    every tick at full rate, the stop path and the hold included (per dry-run scenario:
  chunks.jsonl   <scenario>/ticks.jsonl); every Dream call's request metadata and reply chunk
  inputs/        the keep-out, workspace box, arm file + URDF and robot record the run used
  run/           a copy of the run's folder under runs/ at the end (plan.json, summary.json, Dream logs)
  frames/        live only: the camera frames the runner already reads for Dream (5 Hz), one JPEG each,
                 written by a background thread; skipped when the disk is short (see FrameRecorder)
  ACTIVE         while the run is in progress (its pid); KEEP when it must never be pruned

Retention: at the START of each run the oldest bundles beyond the newest RING are deleted. A bundle
with a KEEP file (optional reason line) is kept forever and does not count. Prune never deletes an
ACTIVE bundle whose pid is alive, a bundle touched in the last RECENT_S, a symlink, or anything that is
not a direct child directory of RUN_DATA.

  .venv/bin/python scripts/viscous-dream1/real/run_data.py list | keep <run> [reason] | unkeep <run> | prune
"""

from __future__ import annotations

import argparse
import contextlib
import json
import os
import queue
import re
import shutil
import socket
import subprocess
import sys
import threading
import time
from collections.abc import Callable
from pathlib import Path

# The Viscous service home's run-data (not the Lab's MAKERMODSLAB_HOME, which --lab-home may point
# elsewhere): one place on this machine for every run's raw data.
RUN_DATA = Path("~/.makermods/makermodslab-viscous/run-data").expanduser()
RING = 10
KEEP, ACTIVE = "KEEP", "ACTIVE"
RECENT_S = 120.0  # a bundle touched this recently is never pruned (a run between mkdir and ACTIVE)
STAMP = re.compile(r"(\d{8}T\d{6}Z)")
REPO = Path(__file__).resolve().parents[3]


def pid_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    except (OverflowError, ValueError):
        return False
    return True


def _safe_child(root: Path, name: str) -> Path | None:
    """root/name when it is a plain directory directly under root; never a symlink, never outside."""
    if not name or name in (".", "..") or "/" in name or "\\" in name or name.startswith("."):
        return None
    path = root / name
    if path.is_symlink() or not path.is_dir():
        return None
    try:
        if path.resolve().parent != root.resolve():
            return None
    except OSError:
        return None
    return path


def _active_pid(path: Path) -> int | None:
    try:
        return int(path.joinpath(ACTIVE).read_text().split()[0])
    except (OSError, ValueError, IndexError):
        return None


def _mtime(path: Path) -> float:
    """When the run last wrote into its bundle: its newest file (a KEEP file is a person, not the run), or
    the folder itself while it is still empty."""
    try:
        files = [p for p in path.iterdir() if p.name != KEEP]
        return max(p.stat().st_mtime for p in files) if files else path.stat().st_mtime
    except OSError:
        return 0.0


def bundles(root: Path = RUN_DATA, *, alive: Callable[[int], bool] = pid_alive) -> list[dict]:
    """Every bundle under root, newest first: name, path, kept (reason or None), active, order key."""
    root = Path(root)
    if not root.is_dir():
        return []
    out = []
    for child in root.iterdir():
        path = _safe_child(root, child.name)
        if path is None:
            continue
        keep = path / KEEP
        reason = None
        if keep.is_file():
            with contextlib.suppress(OSError):
                reason = keep.read_text().strip() or "kept"
            reason = reason or "kept"
        pid = _active_pid(path)
        m = STAMP.search(path.name)
        out.append(
            {
                "name": path.name,
                "path": path,
                "kept": reason,
                "active": pid is not None and alive(pid),
                "pid": pid,
                "mtime": _mtime(path),
                "key": (m.group(1) if m else time.strftime("%Y%m%dT%H%M%SZ", time.gmtime(_mtime(path)))),
            }
        )
    return sorted(out, key=lambda b: (b["key"], b["mtime"], b["name"]), reverse=True)


def prune(
    root: Path = RUN_DATA,
    ring: int = RING,
    *,
    reserve: int = 0,
    clock: Callable[[], float] = time.time,
    alive: Callable[[int], bool] = pid_alive,
) -> list[str]:
    """Delete the oldest unkept bundles beyond the newest `ring - reserve` (reserve 1 at a run's start,
    so with the new run there are `ring`). Kept bundles do not count; an active or recently touched one
    counts but is never deleted. Returns the names deleted."""
    root = Path(root)
    unkept = [b for b in bundles(root, alive=alive) if not b["kept"]]
    removed = []
    for b in unkept[max(0, ring - reserve) :]:
        if b["active"] or clock() - b["mtime"] < RECENT_S:
            continue
        path = _safe_child(root, b["name"])  # checked again, right before deleting
        if path is None or (path / KEEP).exists():
            continue
        shutil.rmtree(path, ignore_errors=True)
        removed.append(b["name"])
    return removed


def keep(root: Path, name: str, reason: str = "") -> Path:
    path = _safe_child(Path(root), name)
    if path is None:
        raise FileNotFoundError(f"no run bundle {name!r} under {root}")
    stamp = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    (path / KEEP).write_text(f"{reason.strip() or 'kept'}\n(kept {stamp})\n")
    return path


def unkeep(root: Path, name: str) -> Path:
    path = _safe_child(Path(root), name)
    if path is None:
        raise FileNotFoundError(f"no run bundle {name!r} under {root}")
    with contextlib.suppress(FileNotFoundError):
        (path / KEEP).unlink()
    return path


def _size(path: Path) -> int:
    total = 0
    for p in path.rglob("*"):
        with contextlib.suppress(OSError):
            if p.is_file() and not p.is_symlink():
                total += p.stat().st_size
    return total


# --- writing a bundle --------------------------------------------------------------------------


class _Tee:
    """A text stream that writes to the original and to the console log; the log never breaks the run."""

    def __init__(self, stream, log):
        self._stream, self._log = stream, log

    def write(self, text):
        with contextlib.suppress(Exception):
            self._log.write(text)
        return self._stream.write(text)

    def flush(self):
        with contextlib.suppress(Exception):
            self._log.flush()
        self._stream.flush()

    def __getattr__(self, name):
        return getattr(self._stream, name)


def _git(*args) -> str:
    try:
        done = subprocess.run(
            ["git", "-C", str(REPO), *args], capture_output=True, text=True, timeout=5, check=False
        )
    except Exception as error:  # noqa: BLE001
        return f"unavailable: {error.__class__.__name__}: {error}"
    return (
        done.stdout.strip() if done.returncode == 0 else f"git {' '.join(args)} failed: {done.stderr.strip()}"
    )


class RunBundle:
    """One run's raw bundle. Never raises into the run: every write is best effort and says so once."""

    def __init__(self, path: Path):
        self.dir = Path(path)
        self.problems: list[str] = []
        self._log = None
        self._streams = None
        self.frames: FrameRecorder | None = None

    @classmethod
    def start(
        cls, name: str, root: Path = RUN_DATA, *, ring: int = RING, meta: dict | None = None
    ) -> RunBundle:
        """Prune the ring (oldest first, keeping room for this run), then create root/name with ACTIVE
        and meta.json."""
        root = Path(root).expanduser()
        root.mkdir(parents=True, exist_ok=True)
        pruned = prune(root, ring, reserve=1)
        path, n = root / name, 1
        while path.exists():
            n += 1
            path = root / f"{name}-{n}"
        path.mkdir()
        (path / ACTIVE).write_text(f"{os.getpid()} {socket.gethostname()}\n")
        bundle = cls(path)
        bundle.write_json(
            "meta.json",
            {
                "argv": sys.argv,
                "cwd": os.getcwd(),
                "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                "host": socket.gethostname(),
                "pid": os.getpid(),
                "python": sys.executable,
                "git_sha": _git("rev-parse", "HEAD"),
                "git_branch": _git("rev-parse", "--abbrev-ref", "HEAD"),
                "git_diff_stat": _git("diff", "--stat", "HEAD"),
                "env": {k: v for k, v in os.environ.items() if k in ("MAKERMODSLAB_HOME", "DREAM_LOG_DIR")},
                "pruned": pruned,
                **(meta or {}),
            },
        )
        return bundle

    def _note(self, what: str, error: Exception) -> None:
        line = f"run-data: {what}: {error.__class__.__name__}: {error}"
        self.problems.append(line)
        with contextlib.suppress(Exception):
            sys.__stderr__.write(line + "\n")

    def write_json(self, name: str, obj) -> None:
        try:
            (self.dir / name).write_text(json.dumps(obj, indent=1, default=str))
        except Exception as error:  # noqa: BLE001
            self._note(f"writing {name}", error)

    def tee(self) -> None:
        """Everything printed from here on (stdout and stderr) also goes to console.log."""
        try:
            self._log = (self.dir / "console.log").open("a", buffering=1)
        except Exception as error:  # noqa: BLE001
            self._note("opening console.log", error)
            return
        self._streams = (sys.stdout, sys.stderr)
        sys.stdout, sys.stderr = _Tee(sys.stdout, self._log), _Tee(sys.stderr, self._log)

    def copy_input(self, src: Path, name: str | None = None) -> None:
        try:
            dest = self.dir / "inputs" / (name or Path(src).name)
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dest)
        except Exception as error:  # noqa: BLE001
            self._note(f"copying {src}", error)

    def mirror_dir(self, sub: str | None = None) -> Path:
        """Where an Episode mirrors its ticks.jsonl / chunks.jsonl as it writes them."""
        path = self.dir / sub if sub else self.dir
        path.mkdir(parents=True, exist_ok=True)
        return path

    def record_frames(self, **kwargs):
        """The frame sink an Episode calls (FrameRecorder.offer), or a no-op if it could not start."""
        self.frames = FrameRecorder.open(self.dir / "frames", **kwargs)
        return self.frames.offer if self.frames is not None else (lambda t, frames: None)

    def finish(self, run_dir: Path | None = None) -> None:
        """Copy the run's folder in (everything but the mirrored tick and chunk logs), stop the frame
        writer and the tee, drop ACTIVE. Safe to call twice."""
        if self.frames is not None:
            self.write_json("frames/stats.json", self.frames.close())
            self.frames = None
        if run_dir is not None and Path(run_dir).is_dir():
            try:
                shutil.copytree(
                    run_dir,
                    self.dir / "run",
                    dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns("ticks.jsonl", "chunks.jsonl"),
                )
            except Exception as error:  # noqa: BLE001
                self._note(f"copying {run_dir}", error)
        if self.problems:
            self.write_json("problems.json", self.problems)
        if self._streams is not None:
            with contextlib.suppress(Exception):
                sys.stdout.flush()
                sys.stderr.flush()
            sys.stdout, sys.stderr = self._streams
            self._streams = None
        if self._log is not None:
            with contextlib.suppress(Exception):
                self._log.close()
            self._log = None
        with contextlib.suppress(OSError):
            (self.dir / ACTIVE).unlink()


class FrameRecorder:
    """The frames the runner already reads, as JPEGs, written by a background thread.

    offer() never blocks the 20 Hz loop: a full queue drops the frame (counted). It adds no camera
    read: the runner hands it the frames it read for Dream (5 Hz in the live loop). Capped by bytes
    per run and by free disk space (it stops, and says so, when either runs out).
    """

    def __init__(
        self, path: Path, *, quality=80, max_bytes=2 * 1024**3, min_free_bytes=4 * 1024**3, maxsize=32
    ):
        self.dir = Path(path)
        self.quality, self.max_bytes, self.min_free = int(quality), int(max_bytes), int(min_free_bytes)
        self.dir.mkdir(parents=True, exist_ok=True)
        self.queue: queue.Queue = queue.Queue(maxsize=maxsize)
        self.stats = {"offered": 0, "written": 0, "dropped": 0, "bytes": 0, "stopped": None}
        self._thread = threading.Thread(target=self._run, name="frame-recorder", daemon=True)
        self._thread.start()

    @classmethod
    def open(cls, path: Path, **kwargs) -> FrameRecorder | None:
        try:
            return cls(path, **kwargs)
        except Exception as error:  # noqa: BLE001 - no frames, the run goes on
            print(f"run-data: no frames ({error.__class__.__name__}: {error})")
            return None

    def offer(self, t: float, frames: dict) -> None:
        if self.stats["stopped"] or not frames:
            return
        self.stats["offered"] += 1
        try:
            self.queue.put_nowait((t, dict(frames)))
        except queue.Full:
            self.stats["dropped"] += 1

    def _run(self) -> None:
        from PIL import Image

        index = None
        while True:
            item = self.queue.get()
            if item is None:
                break
            if self.stats["stopped"]:
                continue
            t, frames = item
            try:
                if shutil.disk_usage(self.dir).free < self.min_free:
                    self.stats["stopped"] = "free disk under the floor"
                    continue
                if index is None:
                    index = (self.dir / "index.jsonl").open("a", buffering=1)
                names = {}
                for camera, rgb in frames.items():
                    folder = self.dir / str(camera)
                    folder.mkdir(exist_ok=True)
                    name = f"{t:.3f}.jpg"
                    Image.fromarray(rgb).save(folder / name, quality=self.quality)
                    self.stats["bytes"] += (folder / name).stat().st_size
                    self.stats["written"] += 1
                    names[camera] = f"{camera}/{name}"
                index.write(json.dumps({"t": t, "frames": names}) + "\n")
                if self.stats["bytes"] > self.max_bytes:
                    self.stats["stopped"] = f"over {self.max_bytes} bytes this run"
            except Exception as error:  # noqa: BLE001 - recording must never touch the run
                self.stats["stopped"] = f"{error.__class__.__name__}: {error}"
        if index is not None:
            index.close()

    def close(self, timeout_s: float = 10.0) -> dict:
        with contextlib.suppress(Exception):
            self.queue.put(None, timeout=timeout_s)
        self._thread.join(timeout_s)
        return dict(self.stats)


# --- the CLI ---------------------------------------------------------------------------------


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--root", type=Path, default=RUN_DATA, help=f"default {RUN_DATA}")
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list")
    k = sub.add_parser("keep")
    k.add_argument("run")
    k.add_argument("reason", nargs="*")
    u = sub.add_parser("unkeep")
    u.add_argument("run")
    pr = sub.add_parser("prune")
    pr.add_argument("--ring", type=int, default=RING)
    args = p.parse_args(argv)
    root = args.root.expanduser()
    if args.cmd == "list":
        found = bundles(root)
        print(f"{root}: {len(found)} bundles ({sum(not b['kept'] for b in found)} in the ring of {RING})")
        for b in found:
            flags = [f"KEEP: {b['kept']}" if b["kept"] else "", "ACTIVE" if b["active"] else ""]
            size = _size(b["path"]) / 1e6
            print(f"  {b['name']:40s} {size:8.1f} MB  {' '.join(f for f in flags if f)}")
        return 0
    try:
        if args.cmd == "keep":
            print(f"kept: {keep(root, args.run, ' '.join(args.reason))}")
        elif args.cmd == "unkeep":
            print(f"no longer kept (the ring may prune it): {unkeep(root, args.run)}")
        else:
            removed = prune(root, args.ring)
            print(f"pruned {len(removed)}: {', '.join(removed) or 'nothing'}")
    except FileNotFoundError as error:
        print(error)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
