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
"""Offline tests for the Dream-1 runner's raw run bundles (scripts/viscous-dream1/real/run_data.py).

tmp dirs, injected clocks and pid checks; no hardware, no network, no sleeps.
Run: .venv/bin/python -m pytest tests/test_viscous_run_data.py -q
"""

import json
import os
import sys
import time
from pathlib import Path

import numpy as np
import pytest

REAL = Path(__file__).resolve().parents[1] / "scripts" / "viscous-dream1" / "real"
sys.path.insert(0, str(REAL))

import run_data  # noqa: E402

OLD = 1_000_000.0  # an mtime long before NOW
NOW = 2_000_000.0


def make(root, name, *, kept=None, active=None, mtime=OLD):
    path = root / name
    path.mkdir()
    (path / "ticks.jsonl").write_text("{}\n")
    if kept is not None:
        (path / run_data.KEEP).write_text(kept)
    if active is not None:
        (path / run_data.ACTIVE).write_text(f"{active} host\n")
    for p in (*path.iterdir(), path):
        os.utime(p, (mtime, mtime))
    return path


def names(root):
    return sorted(p.name for p in root.iterdir())


def prune(root, ring=3, reserve=0, alive=lambda pid: pid == 4242):
    return run_data.prune(root, ring, reserve=reserve, clock=lambda: NOW, alive=alive)


def test_the_ring_keeps_the_newest_and_deletes_oldest_first(tmp_path):
    for day in range(1, 7):
        make(tmp_path, f"live-2026110{day}T120000Z")
    removed = prune(tmp_path)
    assert removed == ["live-20261103T120000Z", "live-20261102T120000Z", "live-20261101T120000Z"]
    assert names(tmp_path) == [f"live-2026110{d}T120000Z" for d in (4, 5, 6)]
    # At a run's start one slot is reserved for the new run.
    assert prune(tmp_path, reserve=1) == ["live-20261104T120000Z"]


def test_names_from_every_kind_sort_by_their_stamp(tmp_path):
    for name in ("dry-20261103T000000Z", "rehearse-live-20261101T000000Z", "live-20261102T000000Z"):
        make(tmp_path, name)
    assert prune(tmp_path, ring=1) == ["live-20261102T000000Z", "rehearse-live-20261101T000000Z"]


def test_kept_runs_never_go_and_do_not_count(tmp_path):
    make(tmp_path, "live-20261101T000000Z", kept="the shoulder cut out\n")
    for day in (2, 3, 4):
        make(tmp_path, f"live-2026110{day}T000000Z")
    assert prune(tmp_path) == []  # three unkept fill the ring of 3; the kept one is extra
    make(tmp_path, "live-20261105T000000Z")
    assert prune(tmp_path) == ["live-20261102T000000Z"]
    assert "live-20261101T000000Z" in names(tmp_path)
    found = {b["name"]: b for b in run_data.bundles(tmp_path)}
    assert found["live-20261101T000000Z"]["kept"] == "the shoulder cut out"


def test_keep_and_unkeep(tmp_path):
    make(tmp_path, "live-20261101T000000Z")
    for day in (2, 3, 4):
        make(tmp_path, f"live-2026110{day}T000000Z")
    run_data.keep(tmp_path, "live-20261101T000000Z", "limp shoulder")
    assert (tmp_path / "live-20261101T000000Z" / run_data.KEEP).read_text().startswith("limp shoulder\n")
    assert prune(tmp_path) == []
    run_data.unkeep(tmp_path, "live-20261101T000000Z")
    assert prune(tmp_path) == ["live-20261101T000000Z"]
    for bad in ("nope", "../x", "", ".", "a/b"):
        with pytest.raises(FileNotFoundError):
            run_data.keep(tmp_path, bad)


def test_an_active_or_just_touched_run_is_never_pruned(tmp_path):
    make(tmp_path, "live-20261101T000000Z", active=4242)  # its pid is alive
    make(tmp_path, "live-20261102T000000Z", active=999)  # a crashed run: pid dead, a partial folder
    make(tmp_path, "live-20261103T000000Z", mtime=NOW - 10)  # between mkdir and ACTIVE
    make(tmp_path, "live-20261104T000000Z")
    assert prune(tmp_path, ring=1) == ["live-20261102T000000Z"]
    assert names(tmp_path) == ["live-20261101T000000Z", "live-20261103T000000Z", "live-20261104T000000Z"]


def test_prune_never_leaves_its_root(tmp_path):
    root = tmp_path / "run-data"
    root.mkdir()
    outside = tmp_path / "precious-20261101T000000Z"
    outside.mkdir()
    (outside / "f").write_text("x")
    (root / "link-20261101T000000Z").symlink_to(outside, target_is_directory=True)
    (root / "a-file-20261101T000000Z").write_text("not a folder")
    (root / ".tmp-20261101T000000Z").mkdir()
    make(root, "live-20261102T000000Z")
    make(root, "live-20261103T000000Z")
    assert prune(root, ring=1) == ["live-20261102T000000Z"]
    assert (outside / "f").read_text() == "x"
    assert (root / "link-20261101T000000Z").is_symlink()
    assert (root / "a-file-20261101T000000Z").is_file() and (root / ".tmp-20261101T000000Z").is_dir()
    assert run_data.prune(tmp_path / "missing", 1) == []


def test_start_prunes_first_makes_room_and_marks_itself_active(tmp_path, monkeypatch):
    for day in range(1, 5):
        make(tmp_path, f"live-2026110{day}T000000Z")
    bundle = run_data.RunBundle.start("live-20261105T000000Z", tmp_path, ring=3)
    assert names(tmp_path) == [f"live-2026110{d}T000000Z" for d in (3, 4, 5)]
    assert (bundle.dir / run_data.ACTIVE).read_text().split()[0] == str(os.getpid())
    meta = json.loads((bundle.dir / "meta.json").read_text())
    assert meta["pruned"] == ["live-20261102T000000Z", "live-20261101T000000Z"]
    again = run_data.RunBundle.start("live-20261105T000000Z", tmp_path, ring=10)  # same name: suffixed
    assert again.dir.name == "live-20261105T000000Z-2"
    bundle.finish()
    again.finish()
    assert not (bundle.dir / run_data.ACTIVE).exists()


def test_frames_are_written_in_the_background_and_never_block(tmp_path):
    rec = run_data.FrameRecorder(tmp_path / "frames", maxsize=2, min_free_bytes=0)
    frame = np.zeros((48, 64, 3), np.uint8)
    t0 = time.perf_counter()
    for i in range(50):  # far faster than the writer: the queue fills and frames are dropped, not waited on
        rec.offer(i * 0.2, {"left": frame, "wrist": frame})
    assert time.perf_counter() - t0 < 0.5
    stats = rec.close()
    assert stats["offered"] == 50 and stats["dropped"] + stats["written"] // 2 == 50
    index = [json.loads(line) for line in (tmp_path / "frames" / "index.jsonl").read_text().splitlines()]
    assert len(index) == stats["written"] // 2 >= 1
    assert (tmp_path / "frames" / index[0]["frames"]["left"]).is_file()


def test_frames_stop_when_the_disk_is_short(tmp_path):
    rec = run_data.FrameRecorder(tmp_path / "frames", min_free_bytes=1 << 62)
    rec.offer(0.0, {"left": np.zeros((8, 8, 3), np.uint8)})
    stats = rec.close()
    assert stats["written"] == 0 and "free disk" in stats["stopped"]


def test_the_cli(tmp_path, capsys):
    make(tmp_path, "live-20261101T000000Z")
    assert run_data.main(["--root", str(tmp_path), "keep", "live-20261101T000000Z", "limp", "shoulder"]) == 0
    assert run_data.main(["--root", str(tmp_path), "list"]) == 0
    assert "KEEP: limp shoulder" in capsys.readouterr().out
    assert run_data.main(["--root", str(tmp_path), "unkeep", "live-20261101T000000Z"]) == 0
    assert run_data.main(["--root", str(tmp_path), "prune", "--ring", "0"]) == 0
    assert names(tmp_path) == []
    assert run_data.main(["--root", str(tmp_path), "keep", "nope"]) == 2
