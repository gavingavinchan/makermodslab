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
"""Offline tests for the Dream-1 real-arm runner (scripts/viscous-dream1/real/).

Pure helpers and the control loop on fakes only: no hardware, no Dream client,
no network, injected clocks, no sleeps. The tool-point records below are
in-memory test fixtures, never written to disk.
Run: .venv/bin/python -m pytest tests/test_viscous_dream_real.py -q
"""

import base64
import io
import json
import math
import sys
import tempfile
import threading
import unittest
from pathlib import Path

import numpy as np

REAL = Path(__file__).resolve().parents[1] / "scripts" / "viscous-dream1" / "real"
sys.path.insert(0, str(REAL))

import dream_real  # noqa: E402
import dream_real_core as core  # noqa: E402

TANK = REAL / "tank_model"
LIMITS = {
    m: tuple(v)
    for m, v in json.loads(
        (core.ROOT / "configs/viscous/viscous_waterproof_01/evidence/lab-hardware-profile.json").read_text()
    )["follower"]["joint_limits"].items()
}
POSE = dict(dream_real.TRACE_POSE)


class FakeClock:
    def __init__(self, t=0.0):
        self.t = t

    def __call__(self):
        return self.t


def zeros(value=0.0):
    return dict.fromkeys(core.MOTORS, value)


class JointMapTest(unittest.TestCase):
    def setUp(self):
        self.jmap = core.JointMap.load()

    def test_tank_pan_shift_and_round_trip(self):
        ik = self.jmap.to_ik(POSE)
        self.assertAlmostEqual(math.degrees(ik[0]), POSE["shoulder_pan"] - 180.0, places=9)
        back = self.jmap.to_motor(ik)
        for m in core.ARM:
            self.assertAlmostEqual(back[m], POSE[m], places=9)

    def test_wrist_yaw_sign_is_flipped(self):
        ik = self.jmap.to_ik(POSE)
        self.assertAlmostEqual(math.degrees(ik[4]), -POSE["wrist_yaw"], places=9)

    def test_every_sign_is_verified(self):
        # J5/J6 were hand-tested 2026-11-08; any "pending"/"unverified"/"assumed" status re-arms the --live ack.
        self.assertEqual(self.jmap.unverified(), [])
        status = dict(self.jmap.status, wrist_roll="assumed, unverified")
        stale = core.JointMap(self.jmap.sign, self.jmap.offset_deg, status)
        self.assertEqual(stale.unverified(), ["wrist_roll"])

    def test_grip_map_and_clip(self):
        self.assertEqual(core.grip_to_motor(0), core.GRIP_CLOSED_DEG)
        self.assertEqual(core.grip_to_motor(1), core.GRIP_OPEN_DEG)
        self.assertEqual(core.grip_to_motor(1.03), core.GRIP_OPEN_DEG)
        self.assertEqual(core.grip_to_motor(-0.09), core.GRIP_CLOSED_DEG)
        self.assertAlmostEqual(core.motor_to_grip(core.grip_to_motor(0.4)), 0.4)

    def test_bad_rows_refused(self):
        with self.assertRaises(ValueError):
            core.ik_row_to_motor([0.0] * 6, self.jmap)
        with self.assertRaises(ValueError):
            core.ik_row_to_motor([0.0] * 6 + [float("nan")], self.jmap)


class LimitsSpeedTest(unittest.TestCase):
    def test_clamp_margin(self):
        inside = {m: (lo + hi) / 2 for m, (lo, hi) in LIMITS.items()}
        target = dict(inside, shoulder_lift=-210.0)
        out, clipped = core.clamp_to_limits(target, LIMITS, 1.0)
        self.assertEqual(clipped, ["shoulder_lift"])
        self.assertAlmostEqual(out["shoulder_lift"], LIMITS["shoulder_lift"][0] + 1.0)

    def test_j1_wrap_is_a_jump(self):
        a, b = dict(POSE, shoulder_pan=182.0), dict(POSE, shoulder_pan=-178.0)
        self.assertEqual(core.jumps(a, b, 20.0), ["shoulder_pan"])
        self.assertEqual(core.jumps(a, dict(a, elbow_flex=a["elbow_flex"] + 19.9), 20.0), [])

    def test_retime_caps_speed_and_keeps_a_straight_line(self):
        a = dict(POSE)
        b = dict(
            POSE, elbow_flex=POSE["elbow_flex"] - 7.0, wrist_flex=POSE["wrist_flex"] + 3.5, gripper=-40.0
        )
        steps = core.retime(a, b, arm_deg_s=15.0, grip_deg_s=60.0)
        prev = a
        for s in steps:
            self.assertLessEqual(max(abs(s[m] - prev[m]) for m in core.ARM), 15.0 * 0.05 + 1e-9)
            self.assertLessEqual(abs(s["gripper"] - prev["gripper"]), 60.0 * 0.05 + 1e-9)
            fractions = {
                round((s[m] - a[m]) / (b[m] - a[m]), 9) for m in ("elbow_flex", "wrist_flex", "gripper")
            }
            self.assertEqual(len(fractions), 1)  # every moving joint at the same fraction
            prev = s
        self.assertEqual(steps[-1], b)
        self.assertEqual(
            len(core.retime(a, dict(a, elbow_flex=a["elbow_flex"] + 0.5), arm_deg_s=15, grip_deg_s=60)), 1
        )


class TankModelTest(unittest.TestCase):
    def test_tank_urdf_is_the_physical_arm_turned_180(self):
        src = core.parse_urdf((core.ROOT / "configs/viscous/model/viscous_arm.urdf").read_bytes())
        tank = core.parse_urdf((TANK / "viscous_waterproof_tank.urdf").read_bytes())
        jmap = core.JointMap.load()
        rz = np.diag([-1.0, -1.0, 1.0])
        rng = np.random.default_rng(1)
        for _ in range(25):
            motor = {m: rng.uniform(*LIMITS[m]) for m in core.ARM}
            a = core.fk(
                src, "base_link", "tool0", {m: math.radians(v) for m, v in jmap.urdf_deg(motor).items()}
            )
            b = core.fk(tank, "dream_base", "tool0", dict(zip(core.ARM, jmap.to_ik(motor), strict=True)))
            np.testing.assert_allclose(rz @ a[:3, 3], b[:3, 3], atol=1e-12)
            np.testing.assert_allclose(rz @ a[:3, :3], b[:3, :3], atol=1e-12)

    def test_tank_limits_stay_inside_the_profile(self):
        tank = {
            j.name: j for j in core.parse_urdf((TANK / "viscous_waterproof_tank.urdf").read_bytes()).values()
        }
        ik = core.JointMap.load().ik_limits_deg(LIMITS)
        for m in core.ARM:
            lo, hi = math.degrees(tank[m].lower), math.degrees(tank[m].upper)
            self.assertGreaterEqual(lo, ik[m][0] - 1e-6)
            self.assertLessEqual(hi, ik[m][1] + 1e-6)
        self.assertAlmostEqual(math.degrees(tank["shoulder_pan"].lower), -90.0, places=6)

    def test_generated_model_is_pending_and_has_no_tool_record(self):
        provenance = json.loads((TANK / "viscous_waterproof_tank.provenance.json").read_text())
        self.assertEqual(provenance["approval"]["status"], "pending_gavin_approval")
        arm = json.loads((TANK / "viscous_waterproof_tank_arm.json").read_text())
        self.assertNotIn("tcp_calibration", arm)
        self.assertEqual(arm["base"], "dream_base")


class WorkspaceBoxTest(unittest.TestCase):
    def setUp(self):
        arm = json.loads((TANK / "viscous_waterproof_tank_arm.json").read_text())
        self.box = core.WorkspaceBox.load(
            dream_real.BOX_FILE, dream_real.SRC_URDF, core.JointMap.load(), arm["tcp"]
        )

    def test_trace_pose_is_inside(self):
        self.assertEqual(self.box.violations(POSE), [])
        tcp = self.box.positions_mm(POSE)["tcp"]
        self.assertLess(tcp[0], -400)  # reaching down the tank, toward base -x

    def test_swinging_toward_a_wall_trips(self):
        self.assertTrue(
            any(v.startswith("tcp.y") for v in self.box.violations(dict(POSE, shoulder_pan=150.0)))
        )


class TripMonitorTest(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.mon = core.TripMonitor(core.TripLimits(), self.clock)
        self.cmd = dict(POSE)

    def update(self, pos=None, torque=None, temp=None):
        return self.mon.update(pos or dict(POSE), self.cmd, torque or zeros(0.5), temp or zeros(30.0))

    def test_lag_needs_two_ticks_and_ignores_gripper(self):
        lagging = dict(POSE, elbow_flex=POSE["elbow_flex"] - 11)
        self.assertEqual(self.update(lagging), [])
        self.assertEqual([t.name for t in self.update(lagging)], ["lag"])
        self.assertEqual(self.update(dict(POSE, gripper=POSE["gripper"] - 50)), [])

    def test_torque_instant_and_sustained(self):
        self.assertEqual(self.update(torque=dict(zeros(0.5), shoulder_lift=6.0)), [])
        self.assertIn("torque", [t.name for t in self.update(torque=dict(zeros(0.5), shoulder_lift=-6.0))])
        mon = core.TripMonitor(core.TripLimits(), self.clock)
        over = dict(zeros(0.5), elbow_flex=4.0)
        self.assertEqual(mon.update(POSE, self.cmd, over, zeros(30)), [])
        self.clock.t = 1.9
        self.assertEqual(mon.update(POSE, self.cmd, over, zeros(30)), [])
        self.clock.t = 2.0
        self.assertEqual([t.name for t in mon.update(POSE, self.cmd, over, zeros(30))], ["torque_sustained"])

    def test_temperature_and_bad_reading(self):
        self.assertEqual(
            [t.name for t in self.update(temp=dict(zeros(30), wrist_roll=80.0))], ["temperature"]
        )
        self.assertEqual(
            [t.name for t in self.update(torque=dict(zeros(0.5), gripper=float("nan")))], ["bad_reading"]
        )

    def test_limits_refuse_unsafe_settings(self):
        for bad in ({"temp_trip_c": 85}, {"torque_trip_nm": 15}, {"lag_deg": 25}, {"torque_cap_nm": 6}):
            with self.assertRaises(ValueError):
                core.TripLimits(**bad).check()


def tool_digest(arm, urdf):
    tool = json.dumps({k: arm[k] for k in core.TOOL_KEYS}, sort_keys=True)
    return core.sha256_bytes(urdf + b"\0" + tool.encode())[:16]


class RefusalTest(unittest.TestCase):
    URDF = b"<robot name='x'/>\n"

    def arm(self, **record):
        arm = {
            "urdf": "x.urdf",
            "base": "dream_base",
            "flange": "f",
            "tcp": [0.1, 0, 0],
            "approach": [1, 0, 0],
            "jaw": [0, 1, 0],
            "rest": [0] * 6,
        }
        if record:
            arm["tcp_calibration"] = record  # in-memory fixture only
        return arm

    def test_tool_point(self):
        self.assertIn("no tcp_calibration", core.tool_point_problem(self.arm(), self.URDF))
        self.assertIn("not set", core.tool_point_problem(self.arm(status="skipped", at="t"), self.URDF))
        self.assertIn("no digest", core.tool_point_problem(self.arm(status="calibrated", at="t"), self.URDF))
        good = self.arm(status="calibrated", at="t", digest="")
        good["tcp_calibration"]["digest"] = tool_digest(good, self.URDF)
        self.assertIsNone(core.tool_point_problem(good, self.URDF))
        self.assertIsNone(core.tool_point_problem(dict(good, rest=[1] * 6), self.URDF))  # rest is not covered
        self.assertIn("changed", core.tool_point_problem(dict(good, tcp=[0.2, 0, 0]), self.URDF))
        self.assertIn("changed", core.tool_point_problem(good, self.URDF.rstrip()))

    def test_approval(self):
        self.assertIn("not a generated", core.approval_problem(self.URDF, None))
        pending = {"approval": {"status": "pending_gavin_approval"}}
        self.assertIn("pending", core.approval_problem(self.URDF, pending))
        ok = {
            "approval": {
                "status": "approved",
                "by": "Gavin",
                "urdf_sha256_normalized": core.normalized_urdf_sha256(self.URDF.rstrip()),
            }
        }
        self.assertIsNone(core.approval_problem(self.URDF, ok))  # dreamcli's copy adds a trailing newline
        self.assertIn("different URDF", core.approval_problem(b"<robot name='y'/>", ok))
        self.assertIn("base_link", core.approval_problem(self.URDF, ok, base="base_link"))

    def facts(self, **change):
        good = {
            "tcp_problem": None,
            "approval_problem": None,
            "unverified_signs": ["wrist_yaw"],
            "acked_signs": ["wrist_yaw"],
            "plan_id": "abc",
            "plan_id_given": "abc",
            "lab_holder": None,
            "port_holders": [],
            "missing_slots": [],
            "guard_problem": None,
            "dream_problem": None,
            "ledger_remaining": 5,
            "max_calls": 10,
            "arm_file_explicit": True,
            "credentials_present": True,
        }
        return core.LiveFacts(**{**good, **change})

    def test_live_refusals(self):
        self.assertEqual(core.live_refusals(self.facts()), [])
        for change in (
            {"tcp_problem": "x"},
            {"approval_problem": "x"},
            {"acked_signs": []},
            {"plan_id_given": None},
            {"lab_holder": "teleoperation"},
            {"port_holders": ["pid 1"]},
            {"missing_slots": ["overhead"]},
            {"guard_problem": "x"},
            {"dream_problem": "x"},
            {"ledger_remaining": 0},
            {"max_calls": 21},
            {"arm_file_explicit": False},
            {"credentials_present": False},
        ):
            self.assertEqual(len(core.live_refusals(self.facts(**change))), 1, change)


class LedgerTest(unittest.TestCase):
    def test_caps_and_free_statuses(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "ledger.json"
            ledger = core.Ledger(path, cap=3, clock=lambda: 0.0)
            self.assertEqual(ledger.remaining(), 3)
            self.assertFalse(path.exists())  # reading never creates it
            a = ledger.reserve("run1", 2)
            ledger.finish(a["id"], "not_sent")
            b = ledger.reserve("run1", 2)
            ledger.finish(b["id"], "pool_warming")
            ledger.finish(ledger.reserve("run1", 2)["id"], "billed")
            ledger.reserve("run1", 2)  # left reserved: an uncertain call counts as billed
            with self.assertRaises(core.BudgetEndError):
                ledger.reserve("run1", 2)
            ledger.finish(ledger.reserve("run2", 5)["id"], "billed")
            with self.assertRaises(core.BudgetEndError):
                ledger.reserve("run3", 5)
            self.assertEqual(core.Ledger(path, cap=99).remaining(), 0)  # the file's cap wins


class ImageTest(unittest.TestCase):
    def test_crop_resize_and_rgb_order(self):
        rgb = np.zeros((480, 640, 3), np.uint8)
        rgb[:60] = (0, 255, 0)  # cropped away
        rgb[60:420] = (255, 0, 0)  # red stays red
        out = core.to_model_frame(rgb)
        self.assertEqual(out.shape, (176, 320, 3))
        self.assertTrue((out[..., 0] > 200).all() and (out[..., 1] < 30).all())
        from PIL import Image

        decoded = np.asarray(Image.open(io.BytesIO(base64.b64decode(core.jpeg_b64(out)))).convert("RGB"))
        self.assertEqual(decoded.shape, (176, 320, 3))
        self.assertGreater(decoded[..., 0].mean(), 200)
        self.assertLess(decoded[..., 2].mean(), 40)

    def test_slots(self):
        slots, missing = core.resolve_slots(["wrist"], None, "wrist", "copy")
        self.assertEqual(slots, {"overhead": "black", "wrist_l": "wrist", "wrist_r": "copy:wrist_l"})
        self.assertEqual(len(missing), 1)
        _, missing = core.resolve_slots(["wrist"], "top", "wrist", "camera:right")
        self.assertEqual(len(missing), 2)
        frames = core.slot_frames({"wrist": np.full((480, 640, 3), 200, np.uint8)}, slots)
        self.assertEqual(set(frames), {"video.overhead", "video.wrist_l", "video.wrist_r"})
        self.assertEqual(frames["video.wrist_l"], frames["video.wrist_r"])

    def test_state_is_radians_with_parked_right_arm(self):
        rest = [0.0, -1.895, 2.094, -1.608, 0.0, 0.0]
        state = core.dream_state(POSE, core.JointMap.load(), rest, "put apple into the bowl")
        self.assertTrue(all(abs(v) < 4.5 for v in state["state.left_arm"]))
        self.assertEqual(state["state.right_arm"], rest)
        self.assertEqual(state["state.right_grip"], [1.0])


class EpisodeOnFakesTest(unittest.TestCase):
    """The loop end to end on a fake arm and the internal stub, virtual clock."""

    def episode(self, scenario="nominal", guard=None, max_calls=2, after=None, start=None):
        clock = dream_real.VirtualClock()
        jmap = core.JointMap.load()
        arm = dream_real.FakeArm(start or POSE, ["wrist"], clock, scenario)
        dream = dream_real.DreamInternalStub(jmap, scenario)
        worker = dream_real.Worker(dream, clock, threaded=False, latency_s=1.0, max_calls=max_calls)
        slots, _ = core.resolve_slots(["wrist"], None, "wrist", "black")
        settings = dream_real.Settings(
            "put apple into the bowl", 48, max_calls, 15.0, 60.0, 20.0, 1.0, core.TripLimits(), slots
        )
        tcp = json.loads((TANK / "viscous_waterproof_tank_arm.json").read_text())["tcp"]
        box = core.WorkspaceBox.load(dream_real.BOX_FILE, dream_real.SRC_URDF, jmap, tcp)
        stop, abort = threading.Event(), threading.Event()

        def sleep(dt):
            clock.sleep(dt)
            if after:
                after(clock, stop, abort)

        with tempfile.TemporaryDirectory() as tmp:
            ep = dream_real.Episode(
                arm,
                dream,
                worker,
                jmap,
                LIMITS,
                box,
                guard or NoGuard(),
                settings,
                clock,
                sleep,
                Path(tmp),
                stop,
                abort,
            )
            return ep.run(), arm

    def assert_stopped_safely(self, arm, returned=True):
        names = [e.split(":")[0] for e in arm.events]
        self.assertEqual(names[-1], "release")
        if returned:
            self.assertEqual(names[-2], "return to rest")
        self.assertFalse(arm.energized)

    def test_nominal_runs_to_the_call_cap_then_returns_and_releases(self):
        summary, arm = self.episode()
        self.assertEqual(summary["reason"], "call cap 2 reached")
        self.assertEqual(summary["calls"], 2)
        self.assert_stopped_safely(arm)

    def test_every_fault_stops_through_the_stop_path(self):
        for scenario, trip in (
            ("torque", "torque"),
            ("lag", "lag"),
            ("stale", "feedback"),
            ("temperature", "temperature"),
            ("workspace", "workspace"),
        ):
            summary, arm = self.episode(scenario, max_calls=5)
            self.assertIn(trip, [t["name"] for t in summary["trips"]], scenario)
            self.assert_stopped_safely(arm)

    def test_signal_stops(self):
        summary, arm = self.episode(after=lambda c, s, a: s.set() if c.t > 1004 else None)
        self.assertEqual(summary["reason"], "stop signal")
        self.assert_stopped_safely(arm)

    def test_unclear_return_path_holds_until_the_second_signal(self):
        def after(clock, stop, abort):
            if clock.t > 1004:
                stop.set()
            if clock.t > 1010:
                abort.set()

        summary, arm = self.episode(guard=NoGuard(clear=False), after=after)
        self.assertEqual(summary["stop"]["returned"], "held, released on the second signal")
        self.assert_stopped_safely(arm, returned=False)

    def test_refused_start_never_energizes(self):
        summary, arm = self.episode(start=dict(POSE, shoulder_pan=150.0))  # hand swung toward a wall
        self.assertTrue(summary["reason"].startswith("refused: start pose refused"))
        self.assertNotIn("engage", " ".join(arm.events))
        self.assertEqual(arm.events[-1].split(":")[0], "release")


class NoGuard:
    available, problem = False, "test"

    def __init__(self, clear=True):
        self.clear = clear

    def start(self, pose):
        return None

    def step(self, target):
        return "unguarded", target, ""

    def monitor(self, measured):
        return None

    def path_clear(self, a, b):
        return self.clear


if __name__ == "__main__":
    unittest.main()
