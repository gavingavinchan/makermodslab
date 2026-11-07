# Copyright 2026 MakerMods. All rights reserved.
"""Write episodes as a LeRobot v2.1 dataset, the only version Dream's fine-tune service takes.

Lab and current lerobot record v3.0; Dream's validator (`shared/datasets/validation.py`, installed client 1.0.3)
reads v2.1 only, so sim demos are written straight to v2.1 here rather than converted. The layout is LeRobot's
standard: `meta/{info.json,episodes.jsonl,tasks.jsonl}`, `data/chunk-000/episode_NNNNNN.parquet`,
`videos/chunk-000/<video key>/episode_NNNNNN.mp4` (H.264 via PyAV's libx264, yuv420p; the service also takes HEVC,
AV1 and VP9). Each frame carries its own `task_index`, so one episode can switch instruction per phase ("pick up the
pin", "insert it", "screw it in"). The rate must be 30, 15, 10 or 5 fps, and every episode at least 1.6 s.

Check a written dataset locally, free and with no key, before any upload:

    ~/dreamtrial/.venv/bin/python lerobot_v21.py --selftest /tmp/v21_selftest
"""

import json
import sys
from pathlib import Path

import av
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

CHUNK = "chunk-000"
CHUNKS_SIZE = 1000


class V21Writer:
    def __init__(self, root, fps: int, state_names: list[str], action_names: list[str], cameras: dict[str, tuple]):
        """`cameras`: video key (`observation.images.overhead`) to its (height, width)."""
        if fps not in (30, 15, 10, 5):
            raise ValueError(f"fps {fps}: Dream trains only 30, 15, 10 or 5")
        self.root, self.fps = Path(root), fps
        self.state_names, self.action_names, self.cameras = state_names, action_names, cameras
        self.tasks: dict[str, int] = {}
        self.episodes: list[dict] = []
        self.frames = 0
        (self.root / "meta").mkdir(parents=True, exist_ok=True)

    def _task(self, text: str) -> int:
        return self.tasks.setdefault(text, len(self.tasks))

    def add_episode(self, state, action, tasks: list[str], images: dict[str, np.ndarray]):
        """state (T, S), action (T, A) floats; tasks: one instruction per frame; images: key -> (T, H, W, 3) uint8."""
        state, action = np.asarray(state, np.float32), np.asarray(action, np.float32)
        n = len(state)
        if len(action) != n or len(tasks) != n or any(len(v) != n for v in images.values()):
            raise ValueError("state, action, tasks and every camera need one row per frame")
        if state.shape[1] != len(self.state_names) or action.shape[1] != len(self.action_names):
            raise ValueError("vector widths do not match the names")
        if set(images) != set(self.cameras):
            raise ValueError(f"cameras {sorted(images)} != {sorted(self.cameras)}")
        ep = len(self.episodes)
        task_idx = np.array([self._task(t) for t in tasks], np.int64)
        table = pa.table(
            {
                "observation.state": pa.array(list(state), type=pa.list_(pa.float32())),
                "action": pa.array(list(action), type=pa.list_(pa.float32())),
                "timestamp": pa.array(np.arange(n, dtype=np.float32) / self.fps),
                "frame_index": pa.array(np.arange(n, dtype=np.int64)),
                "episode_index": pa.array(np.full(n, ep, np.int64)),
                "index": pa.array(np.arange(self.frames, self.frames + n, dtype=np.int64)),
                "task_index": pa.array(task_idx),
            }
        )
        data = self.root / "data" / CHUNK / f"episode_{ep:06d}.parquet"
        data.parent.mkdir(parents=True, exist_ok=True)
        pq.write_table(table, data)
        for key, frames in images.items():
            self._write_video(self.root / "videos" / CHUNK / key / f"episode_{ep:06d}.mp4", frames)
        seen = list(dict.fromkeys(tasks))
        self.episodes.append({"episode_index": ep, "tasks": seen, "length": n})
        self.frames += n

    def _write_video(self, path: Path, frames: np.ndarray):
        path.parent.mkdir(parents=True, exist_ok=True)
        h, w = frames.shape[1:3]
        with av.open(str(path), "w") as out:
            stream = out.add_stream("libx264", rate=self.fps)
            stream.width, stream.height, stream.pix_fmt = w, h, "yuv420p"
            stream.options = {"crf": "20", "preset": "veryfast", "g": str(2 * self.fps)}
            for img in frames:
                for packet in stream.encode(av.VideoFrame.from_ndarray(np.ascontiguousarray(img), format="rgb24")):
                    out.mux(packet)
            for packet in stream.encode():
                out.mux(packet)

    def finish(self, robot_type: str = "viscous_bimanual"):
        features = {
            key: {
                "dtype": "video",
                "shape": [h, w, 3],
                "names": ["height", "width", "channel"],
                "info": {
                    "video.height": h,
                    "video.width": w,
                    "video.codec": "h264",
                    "video.pix_fmt": "yuv420p",
                    "video.is_depth_map": False,
                    "video.fps": self.fps,
                    "video.channels": 3,
                    "has_audio": False,
                },
            }
            for key, (h, w) in self.cameras.items()
        }
        features["observation.state"] = {
            "dtype": "float32",
            "shape": [len(self.state_names)],
            "names": self.state_names,
        }
        features["action"] = {"dtype": "float32", "shape": [len(self.action_names)], "names": self.action_names}
        for name, dtype in (
            ("timestamp", "float32"),
            ("frame_index", "int64"),
            ("episode_index", "int64"),
            ("index", "int64"),
            ("task_index", "int64"),
        ):
            features[name] = {"dtype": dtype, "shape": [1], "names": None}
        info = {
            "codebase_version": "v2.1",
            "robot_type": robot_type,
            "total_episodes": len(self.episodes),
            "total_frames": self.frames,
            "total_tasks": len(self.tasks),
            "total_videos": len(self.episodes) * len(self.cameras),
            "total_chunks": 1,
            "chunks_size": CHUNKS_SIZE,
            "fps": self.fps,
            "splits": {"train": f"0:{len(self.episodes)}"},
            "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
            "video_path": "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4",
            "features": features,
        }
        meta = self.root / "meta"
        (meta / "info.json").write_text(json.dumps(info, indent=2) + "\n")
        (meta / "episodes.jsonl").write_text("".join(json.dumps(e) + "\n" for e in self.episodes))
        (meta / "tasks.jsonl").write_text(
            "".join(json.dumps({"task_index": i, "task": t}) + "\n" for t, i in self.tasks.items())
        )


def selftest(out: Path):
    """Two synthetic episodes through the writer, then Dream's own local validator against our body config."""
    import shutil

    import dream

    import dream_body

    shutil.rmtree(out, ignore_errors=True)
    session = out / "viscous_selftest"
    cams = {key: (176, 320) for key in dream_body.DEFAULT_CAMERAS.values()}
    names = dream_body.names()
    writer = V21Writer(session, 30, names, names, cams)
    rng = np.random.default_rng(0)
    rot6 = np.array([1, 0, 0, 0, 1, 0], np.float32)  # identity, a valid rotation
    for n in (60, 75):
        vec = rng.normal(0, 0.1, (n, dream_body.WIDTH)).astype(np.float32)
        for side in range(2):
            base = side * dream_body.PER_ARM
            vec[:, base + 6] = rng.uniform(0, 1, n)
            vec[:, base + 10 : base + 16] = rot6
        tasks = ["pick up the pin"] * (n // 2) + ["screw the pin into the shackle"] * (n - n // 2)
        images = {k: rng.integers(0, 255, (n, h, w, 3), np.uint8) for k, (h, w) in cams.items()}
        writer.add_episode(vec, vec, tasks, images)
    writer.finish()
    report = dream.DreamClient.validate_dataset(None, str(out), dream_body.config(dream_body.DEFAULT_CAMERAS))
    print(report)
    return report.ok


if __name__ == "__main__":
    if len(sys.argv) == 3 and sys.argv[1] == "--selftest":
        sys.exit(0 if selftest(Path(sys.argv[2])) else 1)
    print(__doc__)
