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
"""Independent physics checks and offline JSON-lines integration, dreamtrial Python only."""

import base64
import io
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from dataclasses import replace
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import av
import mujoco as mj
import numpy as np
from physics import Tank
from PIL import Image
from scene import CLOSED, DT, MODEL, MOTORS, OPEN, ROOT, SLIDE
from thermal import BUDGET_NM2_S, Thermal

sys.path.insert(0, str(ROOT))
from makermodslab import tank_geometry as tg

ARM = ROOT / "scripts/viscous-dream1/real/runs/live-20261108T205156Z/arm/viscous-waterproof-tank.json"
START = dict(zip(MOTORS, (176.59, -122.9, 99.34, 8.56, -9, -4, CLOSED), strict=True))


def request(variant="current", task="cube", **kw):
    return dict(variant=variant, task=task, arm_file=str(ARM), start=START, **kw)


class Physics(unittest.TestCase):
    def test_jaws_map_tcp_and_mass(self):
        spec = json.loads(ARM.read_text())
        geometry = tg.load_model(tg.load_tank())
        q = {**START, "wrist_yaw": 37.0, "gripper": OPEN}
        t = Tank(request() | {"start": q})
        rot, origin = tg.forward_kinematics(geometry, q)["wrist_roll_link"]
        np.testing.assert_allclose(t.truth()["tip"], rot @ spec["tcp"] + origin, atol=1e-8)
        self.assertAlmostEqual(tg.gripper_opening_mm(geometry, q), 52.4, places=4)
        self.assertAlmostEqual(t.data.qpos[t.joints[-1]] + t.data.qpos[t.right], 2 * SLIDE)
        np.testing.assert_allclose(list(t.pos().values()), list(q.values()), atol=1e-9)
        for variant in ("current", "foamed"):
            s = json.loads((MODEL / "buoyancy" / f"{variant}.json").read_text())
            tank = Tank(request(variant))
            for n, link in s["links"].items():
                self.assertAlmostEqual(tank.model.body(n).mass[0], link["mass_kg"], places=6)
            mass = sum(tank.model.body_mass)
            self.assertAlmostEqual(
                mass,
                sum(link["mass_kg"] for link in s["links"].values())
                + sum(b["mass"] for b in s["foam_blocks"])
                + tank.metadata["object_dry_mass_kg"]
                + 0.5 * tank.metadata["rho"] * tank.vol.sum(),
                places=5,
            )

    def test_free_sinking_objects_and_ball_acceleration(self):
        for task in ("ball", "screwdriver", "cube"):
            t = Tank(request(task=task, object_xy=[-0.34, -0.11]))
            z = t.model.joint("object_free").qposadr[0] + 2
            t.data.qpos[z] += 0.08
            first = t.data.qpos[z]
            t.step()
            self.assertLess(t.data.qvel[t.oid + 2], 0)
            mass = t.metadata["object_dry_mass_kg"]
            displaced = t.metadata["object_volume_m3"] * 1000
            self.assertGreater(mass, displaced)
            if task == "ball":
                acc = -(mass - displaced) * 9.81 / (mass + 0.5 * displaced)
                self.assertAlmostEqual(t.data.qvel[t.oid + 2] / DT, acc, places=7)
            for _ in range(50):
                t.step()
            self.assertLess(t.data.qpos[z], first - 0.001)

    def test_native_fluid_has_no_static_buoyancy(self):
        xml = """<mujoco><option density="1000" viscosity=".001"/><worldbody>
        <body><freejoint/><geom type="sphere" size=".03" density="1000" fluidshape="ellipsoid"/></body>
        </worldbody></mujoco>"""
        m = mj.MjModel.from_xml_string(xml)
        d = mj.MjData(m)
        mj.mj_forward(m, d)
        self.assertAlmostEqual(d.qacc[2], -9.81, places=9)
        self.assertAlmostEqual(d.qfrc_passive[2], 0.0, places=9)

    def test_hydrostatics_and_partial_immersion(self):
        # Independent motor Jacobian and the same equal-volume input cells.
        sys.path.insert(0, str(ROOT / "scripts/viscous-buoyancy"))
        import hydro

        model = tg.load_model(tg.load_tank())
        # Use the motor convention of the harness on both FK chains.
        model = replace(model, gripper_motor_closed_deg=CLOSED, gripper_motor_open_deg=OPEN)
        for variant in ("current", "foamed"):
            side = json.loads((MODEL / "buoyancy" / f"{variant}.json").read_text())
            for q in [list(START.values()), [160.0, -100.0, 165.0, -45.0, 12.0, 30.0, OPEN]]:
                t = Tank(request(variant) | {"start": dict(zip(MOTORS, q, strict=True))})
                t.water_forces()
                tau = t.data.qfrc_bias.copy() - t.data.qfrc_gravcomp
                for i in t.bids:
                    vec = t.data.xfrc_applied[i]
                    mj.mj_applyFT(t.model, t.data, -vec[:3], -vec[3:], t.data.xipos[i], i, tau)
                dry, wet = hydro.mass_loads(model, np.array([q]), side["links"])
                expected = dry - wet @ np.array([1000 * v["volume_m3"] for v in side["links"].values()])
                for b in side["foam_blocks"]:
                    expected += hydro.block_load(model, np.array([q]), b) * b["volume"]
                np.testing.assert_allclose(tau[:6] * t.signs[:6], expected[0], atol=1e-7, rtol=0)

    def test_limp_latches_and_contact_at_release(self):
        t = Tank(request())
        t.engage()
        t.goal(START | {"gripper": OPEN})
        t.step()
        self.assertLess(t.data.actuator_force[6], -0.1)
        t.release(gripper_only=True)
        t.step()
        self.assertEqual(t.data.actuator_force[6], 0)
        t.thermal.dose[1] = BUDGET_NM2_S + 1
        t.step()
        t.engage()
        t.step()
        self.assertTrue(t.thermal.limp[1])
        self.assertEqual(t.data.actuator_force[1], 0)
        t.release()
        for _ in range(700):
            t.step()
        self.assertGreater(t.first_floor["normal_m_s"], 0.05)
        self.assertGreaterEqual(t.first_floor["speed_m_s"], t.first_floor["normal_m_s"])
        self.assertFalse(t.success)
        self.assertLess(t.glass_min, 0.001)

    def test_thermal_knee(self):
        s = Thermal(0.1, 2)
        for _ in range(1340):
            s.advance(np.array([4.49, 5.1]))
        self.assertEqual(s.dose[0], 0)
        self.assertTrue(s.limp[1])
        self.assertGreater(s.temp[1], 40)


class Protocol(unittest.TestCase):
    def test_child_clocks_images_and_files(self):
        with (
            tempfile.TemporaryDirectory(dir=Path(__file__).parent) as directory,
            subprocess.Popen(
                [sys.executable, str(Path(__file__).with_name("child.py"))],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                text=True,
                env=os.environ.copy(),
            ) as p,
        ):

            def op(operation, **fields):
                p.stdin.write(json.dumps(dict(op=operation, **fields)) + "\n")
                p.stdin.flush()
                reply = json.loads(p.stdout.readline())
                self.assertTrue(reply["ok"], str(reply)[:400])
                return reply

            op("init", **request("foamed", "ball", run_dir=directory))
            initial = op("engage")["pos"]
            op("goal", target=initial)
            first = op("truth")
            op("mark", label="release check")
            for name in ("head", "left"):
                with Image.open(io.BytesIO(base64.b64decode(op("frames")[name]))) as im:
                    self.assertEqual(im.size, (480, 360))
                    self.assertGreater(np.std(np.array(im)), 5)
            os.kill(p.pid, signal.SIGSTOP)
            try:
                time.sleep(0.25)
            finally:
                os.kill(p.pid, signal.SIGCONT)
            second = op("truth")
            self.assertGreaterEqual(second["t_s"] - first["t_s"], 0.24)
            self.assertGreater(second["torque_abs_integral_nm_s"]["shoulder_lift"], 0)
            op("gripper_off")
            op("release")
            time.sleep(0.45)
            self.assertEqual(max(abs(v) for v in op("read")["torque"].values()), 0.0)
            self.assertIsNotNone(op("truth")["first_floor_contact"])
            op("close")
            self.assertEqual(p.wait(timeout=8), 0)
            run = Path(directory)
            with av.open(str(run / "sim-video.mp4")) as clip:
                stream = clip.streams.video[0]
                self.assertEqual(stream.average_rate, 20)
                self.assertGreaterEqual(len(list(clip.decode(stream))), 3)
            rows = [json.loads(line) for line in (run / "sim-telemetry.jsonl").read_text().splitlines()]
            np.testing.assert_allclose(np.diff([r["t_s"] for r in rows[:-1]]), 0.05, atol=1e-8)
            self.assertEqual(json.loads((run / "sim-marks.jsonl").read_text())["label"], "release check")
            summary = json.loads((run / "sim-summary.json").read_text())
            self.assertGreater(summary["max_backlog_s"], 0.2)
            self.assertGreaterEqual(summary["t_s"] - summary["release_t_s"], 0.999)


if __name__ == "__main__":
    unittest.main()
