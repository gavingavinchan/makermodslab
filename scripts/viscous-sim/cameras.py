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
"""Synchronized EGL views. The renderer owns separate MjData and never stalls physics."""

import base64
import json
import math
import threading

import av
import cv2
import mujoco as mj
import numpy as np
from scene import PLANES, WATER

WIDTH, HEIGHT = 480, 360
N_WATER = 1.333


class Cameras:
    def __init__(self, tank, directory):
        self.model = tank.model
        self.directory = directory
        self.condition = threading.Condition()
        self.pending = None
        self.stopping = False
        self.error = None
        self.latest = {}
        self.dropped = 0
        self.rendered = 0
        self.thread = threading.Thread(target=self.run, name="tank-render")
        self.thread.start()

    def offer(self, tank):
        with self.condition:
            self.dropped += self.pending is not None
            self.pending = (
                tank.data.qpos.copy(),
                float(tank.data.time),
                tank.label,
                float(tank.data.actuator_force[1]),
                bool(tank.success),
            )
            self.condition.notify_all()

    def get(self, wait=False):
        with self.condition:
            if wait:
                self.condition.wait_for(lambda: self.latest or self.error, timeout=45)
            if self.error:
                raise RuntimeError(f"camera/video failure: {self.error}") from self.error
            if not self.latest:
                raise RuntimeError("camera pair unavailable")
            return self.latest.copy()

    def close(self):
        with self.condition:
            self.stopping = True
            self.condition.notify_all()
        self.thread.join()
        if self.error:
            raise RuntimeError(f"camera/video failure: {self.error}") from self.error

    def run(self):
        try:
            self.capture_loop()
        except Exception as exc:
            with self.condition:
                self.error = exc
                self.condition.notify_all()

    def capture_loop(self):
        cv2.setNumThreads(1)
        model = self.model
        d = mj.MjData(model)
        headid = model.camera("head").id
        wristid = model.camera("left").id
        y, x = np.mgrid[:HEIGHT, :WIDTH].astype(np.float32)
        dx, dy = x - (WIDTH - 1) / 2, y - (HEIGHT - 1) / 2
        f = float(HEIGHT / (2 * np.tan(np.radians(model.cam_fovy[headid]) / 2)))
        r = np.hypot(dx, dy) / f
        # tan(theta_water)/tan(theta_air) from Snell at the flat horizontal interface.
        bend = 1 / np.sqrt(N_WATER**2 + (N_WATER**2 - 1) * r * r)
        wrist_f = float(HEIGHT / (2 * np.tan(np.radians(model.cam_fovy[wristid]) / 2)))
        wr = np.hypot(dx, dy) / wrist_f
        port_scale = 1 / np.sqrt(N_WATER**2 + (N_WATER**2 - 1) * wr * wr)
        last_index = -1
        last_rgb = None
        with (
            mj.Renderer(model, HEIGHT, WIDTH) as renderer,
            av.open(str(self.directory / "sim-video.mp4"), "w") as video,
            (self.directory / "sim-video-frames.jsonl").open("x") as ledger,
        ):
            video_stream = video.add_stream("libx264", rate=20)
            video_stream.width = WIDTH * 2
            video_stream.height = HEIGHT
            video_stream.pix_fmt = "yuv420p"
            video_stream.options = {"preset": "ultrafast", "crf": "22", "threads": "1"}

            def encode(rgb):
                frame = av.VideoFrame.from_ndarray(rgb, format="rgb24")
                for pkt in video_stream.encode(frame):
                    video.mux(pkt)

            while True:
                with self.condition:
                    self.condition.wait_for(lambda: self.pending is not None or self.stopping)
                    if self.pending is None:
                        break
                    pose, t, label, tau, success = self.pending
                    self.pending = None
                d.qpos[:] = pose
                mj.mj_kinematics(model, d)
                mj.mj_comPos(model, d)
                mj.mj_camlight(model, d)
                renderer.update_scene(d, camera="head")
                head = renderer.render()
                renderer.enable_depth_rendering()
                depth = renderer.render()
                renderer.disable_depth_rendering()
                air = float(d.cam_xpos[headid, 2] - WATER)
                sx = d.cam_xpos[headid, 0] + dx * (air / f)
                sy = d.cam_xpos[headid, 1] - dy * (air / f)
                through_water = (
                    (sx >= PLANES["far"])
                    & (sx <= PLANES["near"] + 0.012)
                    & (sy >= PLANES["right"])
                    & (sy <= PLANES["left"])
                )
                sampled = depth
                # Inverse mapping for each output pixel. Sample depth along the
                # source ray and iteratively find where the refracted ray intersects it.
                for _ in range(3):
                    wet = np.maximum(sampled - air, 0) * through_water
                    scale = (sampled - wet + wet * bend) / np.maximum(sampled, 1e-6)
                    mx = (WIDTH - 1) / 2 + dx * scale
                    my = (HEIGHT - 1) / 2 + dy * scale
                    sampled = cv2.remap(depth, mx, my, cv2.INTER_NEAREST, borderMode=cv2.BORDER_REPLICATE)
                head = cv2.remap(head, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
                wet = (np.clip(sampled - air, 0, 0.5) * through_water)[:, :, None]
                head = tint(head, wet)
                renderer.update_scene(d, camera="left")
                wrist = renderer.render()
                if d.cam_xpos[wristid, 2] < WATER:
                    wrist = cv2.remap(
                        wrist,
                        (WIDTH - 1) / 2 + dx * port_scale,
                        (HEIGHT - 1) / 2 + dy * port_scale,
                        cv2.INTER_LINEAR,
                    )
                    renderer.enable_depth_rendering()
                    dep = renderer.render()
                    renderer.disable_depth_rendering()
                    dep = cv2.remap(
                        dep,
                        (WIDTH - 1) / 2 + dx * port_scale,
                        (HEIGHT - 1) / 2 + dy * port_scale,
                        cv2.INTER_NEAREST,
                    )
                    wrist = tint(wrist, np.clip(dep, 0, 0.5)[:, :, None])
                wrist = cv2.rotate(wrist, cv2.ROTATE_180)
                pair = {"frame_t_s": t}
                for name, img in [("head", head), ("left", wrist)]:
                    ok, jpg = cv2.imencode(".jpg", img[:, :, ::-1], [cv2.IMWRITE_JPEG_QUALITY, 86])
                    if not ok:
                        raise RuntimeError(f"JPEG encoding failed for {name}")
                    pair[name] = base64.b64encode(jpg).decode("ascii")
                with self.condition:
                    self.latest = pair
                    self.rendered += 1
                    self.condition.notify_all()
                combined = np.concatenate([head, wrist], axis=1)
                cv2.rectangle(combined, (0, 0), (WIDTH * 2, 23), (23, 29, 36), -1)
                cv2.putText(
                    combined,
                    f"HEAD    {t:.1f}s    J2 {tau:+.2f} Nm    {str(label)[:38]}",
                    (8, 16),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.47,
                    (235, 241, 245),
                    1,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    combined,
                    "LEFT" + ("    TASK COMPLETE" if success else ""),
                    (WIDTH + 8, 16),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.47,
                    (235, 241, 245),
                    1,
                    cv2.LINE_AA,
                )
                index = int(math.floor(t * 5 + 0.5))
                if index > last_index:
                    for _ in range(last_index + 1, index):
                        encode(last_rgb if last_rgb is not None else combined)
                    encode(combined)
                    last_rgb, last_index = combined, index
                ledger.write(
                    json.dumps({"t_s": t, "index": index, "overwritten_snapshots": self.dropped}) + "\n"
                )
            for pkt in video_stream.encode():
                video.mux(pkt)


def tint(rgb, distance):
    """Assumed clear fresh water, Beer-Lambert absorption and slight blue veiling light."""
    trans = np.exp(-distance * np.array([1.3, 0.52, 0.22], dtype=np.float32))
    return np.clip(rgb * trans + (1 - trans) * [42, 95, 116], 0, 255).astype(np.uint8)
