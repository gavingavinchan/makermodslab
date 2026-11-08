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
from types import SimpleNamespace

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

    def test_generated_model_is_approved_and_has_no_tool_record(self):
        # Gavin approved it provisionally (2026-11-08), on condition every live motion stays <= 15 deg/s.
        provenance = json.loads((TANK / "viscous_waterproof_tank.provenance.json").read_text())
        self.assertEqual(provenance["approval"]["by"], "Gavin")
        urdf = (TANK / "viscous_waterproof_tank.urdf").read_bytes()
        self.assertIsNone(core.approval_problem(urdf, provenance))
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

    def tick(self, mon, t, torque):
        self.clock.t = t
        return [x.name for x in mon.update(POSE, self.cmd, torque, zeros(30))]

    def test_the_tiers_are_gavins(self):
        lim = core.TripLimits()
        self.assertEqual((lim.torque_trip_nm, lim.torque_trip_ticks), (12.0, 2))
        self.assertEqual((lim.torque_cap_nm, lim.torque_cap_s), (5.0, 5.0))
        self.assertEqual((lim.temp_trip_c, lim.temp_warn_c), (80.0, 70.0))
        self.assertEqual((lim.overspeed_deg_s, lim.overspeed_ticks), (30.0, 2))

    def test_instant_tier_needs_two_ticks_over_12(self):
        mon = core.TripMonitor(core.TripLimits(), self.clock)
        self.assertEqual(self.tick(mon, 0.00, dict(zeros(0.5), shoulder_lift=13.0)), [])
        self.assertEqual(self.tick(mon, 0.05, dict(zeros(0.5), shoulder_lift=11.9)), [])  # resets the count
        self.assertEqual(self.tick(mon, 0.10, dict(zeros(0.5), shoulder_lift=-12.5)), [])
        self.assertIn("torque", self.tick(mon, 0.15, dict(zeros(0.5), shoulder_lift=12.5)))

    def test_a_visitor_may_push_10_nm_for_5_s(self):
        # Gavin: visitors push to feel how strong it is, ~10 Nm for at least 5 s. The sustained tier
        # (> 5 Nm for 5 s) ends it at 5 s; nothing trips before.
        mon = core.TripMonitor(core.TripLimits(), self.clock)
        push = dict(zeros(0.5), shoulder_pan=10.0)
        for i in range(100):  # 4.95 s of 20 Hz ticks
            self.assertEqual(self.tick(mon, i * 0.05, push), [], i)
        self.assertEqual(self.tick(mon, 5.0, push), ["torque_sustained"])

    def test_holding_the_start_pose_never_trips(self):
        # Holding the start pose takes ~3.9 Nm on the shoulder (rehearsal 2026-11-08); 4.9 Nm too.
        mon = core.TripMonitor(core.TripLimits(), self.clock)
        for i in range(1200):  # 60 s
            self.assertEqual(
                self.tick(mon, i * 0.05, dict(zeros(0.5), shoulder_lift=4.9, elbow_flex=1.9)), []
            )

    def test_temperature_and_bad_reading(self):
        self.assertEqual(
            [t.name for t in self.update(temp=dict(zeros(30), wrist_roll=80.0))], ["temperature"]
        )
        self.assertEqual(
            [t.name for t in self.update(torque=dict(zeros(0.5), gripper=float("nan")))], ["bad_reading"]
        )

    def test_limits_refuse_unsafe_settings(self):
        core.TripLimits().check()
        for bad in (
            {"temp_trip_c": 85},
            {"torque_trip_nm": 15},
            {"lag_deg": 25},
            {"torque_cap_nm": 13},
            {"torque_cap_s": 0},
            {"overspeed_deg_s": 15},
            {"overspeed_deg_s": 50},
            {"overspeed_ticks": 0},
        ):
            with self.assertRaises(ValueError, msg=bad):
                core.TripLimits(**bad).check()

    def test_overspeed_runs_inside_the_monitor_on_the_reading_clock(self):
        # The command follows the joint exactly (no lag), so only the measured speed can catch it.
        mon = core.TripMonitor(core.TripLimits(), self.clock)
        names = []
        for i in range(3):  # 2 deg per 50 ms on wrist_flex: 40 deg/s for two readings
            pos = dict(POSE, wrist_flex=POSE["wrist_flex"] + 2.0 * i)
            self.clock.t = i * 0.05
            names.append([t.name for t in mon.update(pos, pos, zeros(0.5), zeros(30))])
        self.assertEqual(names, [[], [], ["overspeed"]])
        self.assertGreater(mon.speed.peak["wrist_flex"], 30)


def rehearsal_logs():
    # The three soft-gain (30/2) rehearsals these bounds were measured on; later runs use stiffer gains.
    names = ("20261108T194701Z", "20261108T194724Z", "20261108T195211Z")
    return [REAL / "runs" / f"rehearse-live-{n}" / "ticks.jsonl" for n in names]


class SpeedMonitorTest(unittest.TestCase):
    def mon(self):
        return core.SpeedMonitor(30.0, 2)

    def test_two_readings_over_trip_and_one_does_not(self):
        mon = self.mon()
        self.assertEqual(mon.update(POSE, 0.0), [])
        self.assertEqual(mon.update(dict(POSE, elbow_flex=POSE["elbow_flex"] + 2.0), 0.05), [])  # 40 deg/s
        self.assertEqual(mon.update(dict(POSE, elbow_flex=POSE["elbow_flex"] + 2.5), 0.10), [])  # 10: reset
        self.assertEqual(mon.update(dict(POSE, elbow_flex=POSE["elbow_flex"] + 4.5), 0.15), [])
        trips = mon.update(dict(POSE, elbow_flex=POSE["elbow_flex"] + 6.5), 0.20)
        self.assertEqual([t.name for t in trips], ["overspeed"])
        self.assertIn("elbow_flex", trips[0].detail)

    def test_uses_the_timestamps_not_an_assumed_50_ms(self):
        mon = self.mon()
        mon.update(POSE, 0.0)
        for i in (1, 2):  # 2 deg per reading, but 100 ms apart: 20 deg/s
            self.assertEqual(mon.update(dict(POSE, shoulder_pan=POSE["shoulder_pan"] + 2.0 * i), 0.1 * i), [])
        self.assertAlmostEqual(mon.peak["shoulder_pan"], 20.0)

    def test_close_samples_are_merged_not_compared(self):
        mon = self.mon()
        mon.update(POSE, 0.0)
        # 0.2 deg 5 ms later would read as 40 deg/s: kept out, the 0.0 sample stays the reference.
        self.assertEqual(mon.update(dict(POSE, wrist_roll=POSE["wrist_roll"] + 0.2), 0.005), [])
        self.assertEqual(mon.peak["wrist_roll"], 0.0)
        mon.update(dict(POSE, wrist_roll=POSE["wrist_roll"] + 0.5), 0.05)
        self.assertAlmostEqual(mon.peak["wrist_roll"], 10.0)

    def test_the_gripper_is_excluded(self):
        mon = self.mon()
        for i in range(5):  # 60 deg/s jaws, and the limp gripper dropping on a stop
            self.assertEqual(mon.update(dict(POSE, gripper=POSE["gripper"] - 3.0 * i), 0.05 * i), [])

    @unittest.skipUnless(rehearsal_logs(), "no powered rehearsal logs in this checkout")
    def test_the_powered_rehearsals_never_trip(self):
        # 2026-11-08, the real arm at <= 15 deg/s commanded: the worst single reading was 20.3 deg/s
        # (elbow_flex, approach), the worst held over two readings 18.0 (wrist_flex).
        peak = 0.0
        for path in rehearsal_logs():
            mon = self.mon()
            for line in path.read_text().splitlines():
                tick = json.loads(line)
                self.assertEqual(mon.update(tick["pos"], tick["t"]), [], path.parent.name)
            peak = max(peak, *mon.peak.values())
        self.assertGreater(peak, 15.0)
        self.assertLess(peak, 21.0)


class HoldErrorTest(unittest.TestCase):
    def test_learns_the_standing_sag_and_a_short_push_barely_moves_it(self):
        hold = core.HoldError(tau_s=1.0, limit_deg=10.0)
        cmd = dict(POSE)
        sag = dict(POSE, shoulder_lift=POSE["shoulder_lift"] - 7.45)  # the rehearsal's start-pose sag
        for i in range(100):
            hold.update(sag, cmd, i * 0.05)
        self.assertAlmostEqual(hold.value()["shoulder_lift"], 7.45, places=6)
        pushed = dict(sag, shoulder_pan=POSE["shoulder_pan"] - 9.0)
        for i in range(100, 106):  # 0.3 s of a 9 deg push
            hold.update(pushed, cmd, i * 0.05)
        self.assertLess(hold.value()["shoulder_pan"], 2.0 * 0.3 + 1e-9)  # the estimate moves <= 2 deg/s
        self.assertAlmostEqual(hold.value()["shoulder_lift"], 7.45, places=6)

    def test_clipped_to_the_limit(self):
        hold = core.HoldError(limit_deg=10.0)
        hold.update(dict(POSE, elbow_flex=POSE["elbow_flex"] + 25), POSE, 0.0)
        self.assertEqual(hold.value()["elbow_flex"], -10.0)

    @unittest.skipUnless(rehearsal_logs(), "no powered rehearsal logs in this checkout")
    def test_tracks_the_real_standing_error_of_the_rehearsals(self):
        # A stop at the end of each powered rehearsal would re-anchor within ~1 deg of the error the
        # arm really held (shoulder_lift ~7.45 deg at the start pose).
        for path in rehearsal_logs():
            ticks = [json.loads(line) for line in path.read_text().splitlines()]
            hold = core.HoldError()
            for tick in ticks:
                hold.update(tick["pos"], tick["cmd"], tick["t"])
            true = {m: ticks[-1]["cmd"][m] - ticks[-1]["pos"][m] for m in core.ARM}
            for m in core.ARM:
                self.assertLess(abs(hold.value()[m] - true[m]), 1.1, (path.parent.name, m))
            self.assertGreater(hold.value()["shoulder_lift"], 7.0)


class FakeCanRobot:
    """Just enough of a RobStride follower for remote_can.get_observation and the return helper: no bus.
    Tracks each sent action at once; `fall` deg drop on shoulder_lift per observation; `miss` = the
    observation calls (by count) whose motor query gets no reply."""

    def __init__(self, fall=0.0, miss=()):
        self.pos = dict(POSE)
        self.fall, self.miss, self.reads, self.sent = fall, set(miss), 0, []
        self.config = SimpleNamespace(joint_limits=LIMITS)
        self.action_features = {f"{m}.pos": float for m in core.MOTORS}
        self.bus = SimpleNamespace(
            motors=list(core.MOTORS),
            _query_status_via_clear_fault=self._query,
            _decode_motor_state=lambda data: None,
        )

    def _query(self, motor, timeout):
        if motor == core.MOTORS[0]:
            self.reads += 1
        return False, None if self.reads in self.miss else SimpleNamespace(data=bytes(8))

    def get_observation(self):
        self.pos["shoulder_lift"] -= self.fall
        return {f"{m}.pos": v for m, v in self.pos.items()}

    def send_action(self, action):
        self.sent.append(dict(action))
        lost = {"shoulder_lift"} if self.fall else set()  # a joint that lost its hold does not follow
        self.pos.update({m: v for k, v in action.items() if (m := k.removesuffix(".pos")) not in lost})


class LabArmReturnTest(unittest.TestCase):
    """LabArm's return adapter around the Lab's return helper, on a fake robot (no CAN, no ports)."""

    def leg(self, robot, watch=None):
        arm = dream_real.LabArm(Path("."), {}, 15.0)
        arm.robot = robot
        pose = {m: POSE[m] for m in core.ARM} | {"elbow_flex": POSE["elbow_flex"] - 3.0}  # 0.2 s at 15
        return arm.return_to_rest(pose, threading.Event(), 15.0, watch)

    def test_a_falling_joint_ends_the_leg(self):
        import time

        robot = FakeCanRobot(fall=2.0)  # ~2 deg per ~35 ms read: far over 30 deg/s
        speed = core.SpeedMonitor(30.0, 2)
        trips = []

        def watch(pos):
            trips.extend(found := speed.update(pos, time.monotonic()))
            return found

        ok, message = self.leg(robot, watch)
        self.assertFalse(ok)
        self.assertEqual(trips[0].name, "overspeed")
        self.assertIn("shoulder_lift", message)
        # 30 Hz reads merge into >= 40 ms windows, so two over-speed readings take ~4 setpoints (~0.14 s).
        self.assertLessEqual(len(robot.sent), 5)

    def test_missed_replies_do_not_end_the_leg(self):
        # The planning read misses once (retried); a read before a setpoint misses (skipped, not raised).
        robot = FakeCanRobot(miss=(1, 4))
        ok, message = self.leg(robot, lambda pos: [])
        self.assertTrue(ok, message)
        self.assertAlmostEqual(robot.pos["elbow_flex"], POSE["elbow_flex"] - 3.0)


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


class RecordingStub(dream_real.DreamInternalStub):
    """The internal stub, remembering the commanded row of every request."""

    def __init__(self, *args):
        super().__init__(*args)
        self.commanded = []

    def request(self, commanded):
        self.commanded.append(commanded)
        return super().request(commanded)


class EpisodeOnFakesTest(unittest.TestCase):
    """The loop end to end on a fake arm and the internal stub, virtual clock."""

    def episode(
        self,
        scenario="nominal",
        guard=None,
        max_calls=2,
        after=None,
        start=None,
        rehearse=False,
        hold_s=5.0,
        arm_rest=None,
        sag=None,
        mutate=None,
        patch=None,
    ):
        clock = dream_real.VirtualClock()
        jmap = core.JointMap.load()
        arm = dream_real.FakeArm(start or POSE, ["wrist"], clock, scenario, sag=sag)
        if patch:
            patch(arm)
        self.sent = []
        send = arm.send
        arm.send = lambda target, previous: (self.sent.append(dict(target)), send(target, previous))
        self.dream = None if rehearse else RecordingStub(jmap, scenario)
        worker = (
            None
            if rehearse
            else dream_real.Worker(self.dream, clock, threaded=False, latency_s=1.0, max_calls=max_calls)
        )
        slots, _ = core.resolve_slots(["wrist"], None, "wrist", "black")
        settings = dream_real.Settings(
            "put apple into the bowl", 48, max_calls, 15.0, 60.0, 20.0, 1.0, core.TripLimits(), slots, hold_s
        )
        tcp = json.loads((TANK / "viscous_waterproof_tank_arm.json").read_text())["tcp"]
        box = core.WorkspaceBox.load(dream_real.BOX_FILE, dream_real.SRC_URDF, jmap, tcp)
        stop, abort = threading.Event(), threading.Event()
        release = arm.release
        self.release_after_abort = None  # whether all torque went off only after the second signal

        def released():
            self.release_after_abort = abort.is_set()
            return release()

        arm.release = released

        def sleep(dt):
            clock.sleep(dt)
            if clock.t > 4000:  # an unexpected hold (waiting for a second signal no test sends): fail fast
                raise AssertionError("virtual clock passed 3000 s: the stop is holding for a second signal")
            if after:
                after(clock, stop, abort)

        with tempfile.TemporaryDirectory() as tmp:
            self.ep = dream_real.Episode(
                arm,
                self.dream,
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
                mutate=mutate,
                arm_rest=arm_rest,
            )
            return self.ep.run(), arm

    def assert_stopped_safely(self, arm, returned=True):
        """After engage: the arm re-anchored at its measured pose, the gripper released, then the return
        legs (none starting with a goal far from the arm: no spring-back), then all torque off."""
        names = [e.split(":")[0] for e in arm.events]
        self.assertEqual(names[-1], "release")
        self.assertNotIn("SPRING-BACK", names)
        stop = names[names.index("engage") + 1 :]
        self.assertEqual(stop[:2], ["re-anchor", "release gripper"])
        if returned:
            self.assertGreaterEqual(stop.count("return leg"), 1)
            self.assertEqual(set(stop[2:-1]), {"return leg"})
            # The arm joints end at the measured rest; the limp gripper was never driven back.
            last_pose, _ = arm.returns[-1]
            self.assertEqual(last_pose, {m: POSE[m] for m in core.ARM})
        self.assertTrue(all(speed <= dream_real.HARD_ARM_SPEED_DEG_S for _, speed in arm.returns))
        self.assertFalse(arm.energized)

    def assert_under_speed_cap(self):
        cap = dream_real.HARD_ARM_SPEED_DEG_S * core.ROW_DT_S + 1e-6
        prev = POSE
        for target in self.sent:
            self.assertLessEqual(max(abs(target[m] - prev[m]) for m in core.ARM), cap)
            prev = target

    def test_nominal_runs_to_the_call_cap_then_returns_and_releases(self):
        summary, arm = self.episode()
        self.assertEqual(summary["reason"], "call cap 2 reached")
        self.assertEqual(summary["calls"], 2)
        self.assertEqual(summary["phases"][:5], ["connect", "start gate", "engage", "approach", "dream"])
        self.assert_stopped_safely(arm)
        self.assert_under_speed_cap()

    def test_approach_reaches_the_dream_start_pose_before_the_first_call(self):
        summary, _ = self.episode(max_calls=1)
        start = self.ep.dream_start
        first = self.dream.commanded[0]["q_l_chunk"]
        expected = core.motor_to_ik_row(start, core.JointMap.load())
        np.testing.assert_allclose(first, expected, atol=1e-9)
        waypoints = core.approach_waypoints(POSE, start)
        self.assertGreater(summary["approach_substeps"], len(waypoints))
        for m in core.ARM:  # the start pose is the arm file's rest through the joint map
            self.assertAlmostEqual(start[m], core.JointMap.load().to_motor(self.ep.right_rest)[m])
        self.assertEqual(start["gripper"], POSE["gripper"])

    def test_stop_past_the_approach_goes_back_along_the_route(self):
        _, arm = self.episode(max_calls=1)  # one chunk: the arm stops ~2 deg from the start pose
        start = self.ep.dream_start
        legs = [pose for pose, _ in arm.returns]
        route = [start, *core.return_waypoints(POSE, start)]
        # Within the lag trip of the start pose counts as at it (the leg back up would stop short).
        self.assertEqual(legs, [{m: wp[m] for m in core.ARM} for wp in route[1:]])

    def test_leg_selection_is_from_the_measured_pose(self):
        self.episode(max_calls=1)
        start = self.ep.dream_start
        route = [start, *core.return_waypoints(POSE, start)]
        sagged = dict(start, shoulder_lift=start["shoulder_lift"] - 7.45)  # the start-pose sag
        self.assertEqual(self.ep.legs_from(sagged), route[1:])
        away = dict(start, elbow_flex=start["elbow_flex"] + 12.0)
        self.assertEqual(self.ep.legs_from(away), route)
        self.ep.phase = "approach"  # the last leg (to the rest) always runs, however close
        self.ep.trail = [dict(POSE)]
        self.assertEqual(self.ep.legs_from(dict(POSE)), [POSE])

    def test_stop_mid_approach_retraces_the_legs_travelled(self):
        # Leg 1 (the elbow lift) takes about 3 s; stop during leg 2.
        summary, arm = self.episode(after=lambda c, s, a: s.set() if c.t > 1004.5 else None)
        self.assertEqual(summary["reason"], "stop signal")
        self.assertNotIn("dream", summary["phases"])
        lift = core.approach_waypoints(
            POSE, core.JointMap.load().to_motor(self.ep.right_rest) | {"gripper": 0}
        )[0]
        legs = [pose for pose, _ in arm.returns]
        self.assertEqual(len(legs), 2)
        for m in core.ARM:
            self.assertAlmostEqual(legs[0][m], lift[m], places=9)
        self.assertEqual(legs[1], {m: POSE[m] for m in core.ARM})
        self.assert_stopped_safely(arm)

    def test_every_fault_stops_through_the_stop_path(self):
        for scenario, trip in (
            ("torque", "torque"),
            ("lag", "lag"),
            ("stale", "feedback"),
            ("temperature", "temperature"),
            ("push", "lag"),
        ):
            summary, arm = self.episode(scenario, max_calls=5)
            self.assertIn(trip, [t["name"] for t in summary["trips"]], scenario)
            self.assert_stopped_safely(arm)
            self.assertEqual(summary["stop"]["gripper_release"], "ok")
            self.assertIsInstance(summary["stop"]["reanchor"], dict, scenario)

    def test_out_of_box_requests_are_overridden_not_executed(self):
        # Grace via Gavin 2026-11-08: Dream's out-of-limit requests are cut at the box and the run goes on.
        sent = []

        def record(arm):
            send = arm.send
            arm.send = lambda target, previous: (sent.append(dict(target)), send(target, previous))

        summary, arm = self.episode("workspace", max_calls=3, patch=record)
        self.assertNotIn("workspace", [t["name"] for t in summary["trips"]])
        self.assertTrue(summary.get("overrides"))
        self.assertIn("workspace", summary["overrides"][0]["why"])
        self.assertEqual(summary["calls"], 3)  # kept asking Dream instead of stopping
        self.assertTrue(sent)
        dream_part = sent[summary["approach_substeps"] :]
        self.assertEqual([q for q in dream_part if self.ep.box.violations(q)], [])
        self.assert_stopped_safely(arm)

    def test_calls_that_execute_nothing_stop_the_run(self):
        # 13 of 17 billed calls in live 20261108T212338Z executed nothing; three in a row now end the run.
        from unittest import mock

        with mock.patch.object(dream_real.Episode, "plan_chunk", lambda self, reply: []):
            summary, arm = self.episode(max_calls=20)
        self.assertEqual(summary["calls"], 3)
        self.assertTrue(summary["reason"].startswith("stuck"), summary["reason"])
        self.assert_stopped_safely(arm)

    def test_a_hold_after_a_trip_releases_on_the_first_press(self):
        # The stop came from a trip, not a signal; the hold must release on ONE press (live 215535Z).
        def patch(arm):
            back = arm.return_to_rest

            def stuck(pose, abort, speed, watch=None):
                back(pose, abort, speed, watch)
                arm.pos["shoulder_lift"] = pose["shoulder_lift"] - 25.0
                arm.limp.add("shoulder_lift")
                arm.FALL_DEG_PER_TICK = 0.0
                return False, "shoulder_lift still 25.0 deg away"

            arm.return_to_rest = stuck

        pressed = []

        def one_real_press(clock, stop, abort):
            # install_stop_signals' handler: the first signal is "stop", only a later one "abort".
            if clock.t > 1040 and not pressed:
                pressed.append(clock.t)
                (abort if stop.is_set() else stop).set()

        summary, arm = self.episode(max_calls=1, patch=patch, after=one_real_press)
        self.assertEqual(len(pressed), 1)
        self.assertIn("held", summary["stop"]["returned"])
        self.assertFalse(arm.energized)

    def test_a_pushed_arm_is_held_where_it_is_and_returns_from_there(self):
        guard = NoGuard()
        summary, arm = self.episode("push", max_calls=5, guard=guard)
        self.assertEqual([t["name"] for t in summary["trips"]], ["lag"])  # 18 deg/s: under the speed trip
        measured = summary["stop"]["reanchor"]["measured"]
        commanded = self.ep.commanded
        self.assertGreater(commanded["shoulder_pan"] - measured["shoulder_pan"], 10.0)  # pushed > 10 deg
        # The 18 deg/s push leaks ~1.3 deg into the hold-error estimate (1 s average, moving <= 2 deg/s):
        # the goal lands that close to where the arm was let go, instead of springing back > 10 deg.
        self.assertLess(abs(summary["stop"]["reanchor"]["offset_deg"]["shoulder_pan"]), 2.0)
        # The guard checks the re-anchored goal; the leg itself starts physically from the measured pose.
        first_from, _ = guard.paths[0]  # the guard judges the re-anchored goal (measured + hold error)
        self.assertAlmostEqual(
            first_from["shoulder_pan"], summary["stop"]["reanchor"]["goal"]["shoulder_pan"], 1
        )
        self.assertAlmostEqual(arm.leg_starts[0]["shoulder_pan"], measured["shoulder_pan"], places=2)
        self.assert_stopped_safely(arm)

    @staticmethod
    def second_signal_at(t):
        return lambda clock, stop, abort: abort.set() if clock.t > t else None

    def assert_held_until_the_second_signal(self, summary, arm, why):
        stop = summary["stop"]
        self.assertTrue(stop["returned"].startswith(f"held {why}"), stop["returned"])
        self.assertTrue(stop["returned"].endswith("released on the second signal"))
        self.assertIn("stop: HOLDING", summary["phases"])
        self.assertTrue(self.release_after_abort)  # never released mid-air before the second signal
        self.assert_stopped_safely(arm, returned=False)

    def test_a_joint_losing_its_hold_trips_overspeed_then_holds(self):
        summary, arm = self.episode("overspeed", max_calls=5, after=self.second_signal_at(1030))
        self.assertEqual([t["name"] for t in summary["trips"]], ["overspeed"])  # before the lag trip
        stop = summary["stop"]
        self.assertEqual([t["name"] for t in stop["return_trips"]], ["overspeed"])
        self.assertEqual(len(arm.returns), 1)
        self.assert_held_until_the_second_signal(summary, arm, "during leg 1: overspeed")
        self.assertEqual(stop["hold_reanchor"]["attempts"], 1)  # held where it is, not at the old goal
        self.assertEqual([e.split(":")[0] for e in arm.events].count("re-anchor"), 2)

    def test_a_sagging_arm_stopped_far_from_the_start_reaches_the_rest_before_release(self):
        # The real shoulder settles 7.45 deg short at the start pose: more than the return helper's 6 deg,
        # less than the 10 deg lag trip. A Dream stop > 10 deg from the start pose must not release there.
        summary, arm = self.episode(
            max_calls=1, sag=dream_real.START_POSE_SAG, mutate=dream_real.far_mutation()
        )
        start = self.ep.dream_start
        stop = summary["stop"]
        self.assertGreater(abs(stop["reanchor"]["measured"]["wrist_roll"] - start["wrist_roll"]), 10.0)
        self.assertAlmostEqual(stop["reanchor"]["offset_deg"]["shoulder_lift"], 7.45, delta=0.5)
        route = [start, *core.return_waypoints(POSE, start)]
        self.assertEqual([pose for pose, _ in arm.returns], [{m: wp[m] for m in core.ARM} for wp in route])
        self.assertEqual([leg["ok"] for leg in stop["legs"]], [False, False, True])  # short, short, there
        self.assertTrue(all("within the lag trip" in leg["arrived"] for leg in stop["legs"][:2]))
        self.assertEqual(stop["returned"], "at the rest")
        self.assertNotIn("stop: HOLDING", summary["phases"])
        self.assertFalse(self.release_after_abort)  # released by itself, at the rest
        self.assert_stopped_safely(arm)

    def test_a_leg_ending_beyond_the_lag_trip_holds(self):
        def patch(arm):
            back = arm.return_to_rest

            def stuck(pose, abort, speed, watch=None):
                back(pose, abort, speed, watch)
                arm.pos["shoulder_lift"] = (
                    pose["shoulder_lift"] - 25.0
                )  # caught on something, past its 20 deg lag trip
                arm.limp.add("shoulder_lift")
                arm.FALL_DEG_PER_TICK = 0.0  # stays there
                return False, "shoulder_lift still 25.0 deg away"

            arm.return_to_rest = stuck

        summary, arm = self.episode(max_calls=1, patch=patch, after=self.second_signal_at(1040))
        self.assert_held_until_the_second_signal(summary, arm, "leg 1 ended 25.0 deg off its target")
        self.assertEqual(len(arm.returns), 1)

    def test_no_fresh_reading_after_a_leg_holds_the_last_goal_and_retries(self):
        calls = []

        def patch(arm):
            reanchor = arm.reanchor

            def flaky(offsets):
                calls.append(1)
                if 2 <= len(calls) <= 4:  # the stop's first re-anchor works; then the bus goes quiet
                    raise ConnectionError("No fresh valid starting pose for shoulder_lift (fake)")
                return reanchor(offsets)

            arm.reanchor = flaky
            arm.return_to_rest = lambda pose, abort, speed, watch=None: (False, "unreadable")
            read = arm.read
            arm.read = lambda: read() if not calls else (_ for _ in ()).throw(ConnectionError("stale (fake)"))

        summary, arm = self.episode(max_calls=1, patch=patch, after=self.second_signal_at(1040))
        self.assert_held_until_the_second_signal(summary, arm, "after leg 1: no fresh reading")
        self.assertEqual(summary["stop"]["hold_reanchor"]["attempts"], 4)  # retried until a reading came

    def test_signal_stops(self):
        summary, arm = self.episode(after=lambda c, s, a: s.set() if c.t > 1009 else None)
        self.assertEqual(summary["reason"], "stop signal")
        self.assert_stopped_safely(arm)

    def test_unclear_return_path_holds_until_the_second_signal(self):
        def after(clock, stop, abort):
            if clock.t > 1009:
                stop.set()
            if clock.t > 1015:
                abort.set()

        summary, arm = self.episode(guard=NoGuard(clear=False), after=after)
        self.assertEqual(
            summary["stop"]["returned"],
            "held before leg 1: its path is not clear of the tank keep-out; released on the second signal",
        )
        self.assertTrue(arm.gripper_limp)  # released before the hold
        self.assert_stopped_safely(arm, returned=False)

    def test_second_signal_aborts_the_return_and_releases(self):
        def after(clock, stop, abort):
            if clock.t > 1009:
                stop.set()
                abort.set()

        summary, arm = self.episode(after=after)
        self.assertTrue(summary["stop"]["returned"].startswith("aborted by the second signal"))
        self.assertEqual(arm.returns, [])
        self.assert_stopped_safely(arm, returned=False)

    def test_refused_start_never_energizes(self):
        summary, arm = self.episode(start=dict(POSE, shoulder_lift=LIMITS["shoulder_lift"][0] - 6))
        self.assertTrue(summary["reason"].startswith("refused: start pose refused"))
        self.assertNotIn("engage", " ".join(arm.events))
        self.assertEqual([e.split(":")[0] for e in arm.events][-1], "release")

    def test_the_box_gates_the_dream_start_pose_not_the_rest(self):
        # A rest the box would refuse (hand swung toward a wall) is fine: only limits and the guard apply.
        summary, arm = self.episode(start=dict(POSE, shoulder_pan=150.0), max_calls=1)
        self.assertEqual(summary["reason"], "call cap 1 reached")
        # A Dream start pose outside the box is refused before anything is energized.
        rest = json.loads((TANK / "viscous_waterproof_tank_arm.json").read_text())["rest"]
        swung = [rest[0] - 0.5, *rest[1:]]
        summary, arm = self.episode(arm_rest=swung)
        self.assertIn("Dream start pose, workspace box", summary["reason"])
        self.assertNotIn("engage", " ".join(arm.events))

    def test_rehearse_approaches_holds_then_stops_without_dream(self):
        summary, arm = self.episode(rehearse=True, hold_s=5.0)
        self.assertEqual(summary["reason"], "held 5 s at the Dream start pose")
        self.assertEqual(summary["calls"], 0)
        self.assertEqual(summary["phases"][:5], ["connect", "start gate", "engage", "approach", "hold"])
        self.assert_stopped_safely(arm)
        self.assert_under_speed_cap()
        summary, arm = self.episode(
            rehearse=True, hold_s=60, after=lambda c, s, a: s.set() if c.t > 1010 else None
        )
        self.assertEqual(summary["reason"], "stop signal")
        self.assert_stopped_safely(arm)


class HardSpeedCapTest(unittest.TestCase):
    """Gavin: "as long as we go slow, must keep the 15deg/s limits"."""

    def test_the_cap_is_15(self):
        self.assertEqual(dream_real.HARD_ARM_SPEED_DEG_S, 15.0)

    def test_cli_refuses_a_faster_arm(self):
        for argv in (["--speed-deg-s", "15.5"], ["--rehearse", "--live", "--speed-deg-s", "20"]):
            with self.assertRaises(SystemExit):
                dream_real.main(argv)

    def test_episode_and_bus_backstop_refuse_a_faster_arm(self):
        slots, _ = core.resolve_slots(["wrist"], None, "wrist", "black")
        settings = dream_real.Settings("p", 48, 1, 16.0, 60.0, 20.0, 1.0, core.TripLimits(), slots)
        with self.assertRaises(dream_real.RefusedError):
            dream_real.Episode(*[None] * 7, settings, FakeClock(), None, Path("."), None, None)
        with self.assertRaises(dream_real.RefusedError):
            dream_real.LabArm(Path("."), {}, 30.0)  # constructs only: no hardware is touched
        dream_real.LabArm(Path("."), {}, 15.0)

    def test_a_fast_substep_trips_before_it_is_sent(self):
        slots, _ = core.resolve_slots(["wrist"], None, "wrist", "black")
        settings = dream_real.Settings("p", 48, 1, 15.0, 60.0, 20.0, 1.0, core.TripLimits(), slots)
        ep = dream_real.Episode(
            None, None, None, None, None, None, None, settings, FakeClock(), None, Path("."), None, None
        )
        ep.planned = dict(POSE)
        ep._check_speed(dict(POSE, elbow_flex=POSE["elbow_flex"] + 0.75))
        with self.assertRaises(dream_real.TripStopError):
            ep._check_speed(dict(POSE, elbow_flex=POSE["elbow_flex"] + 0.75 + 0.8))


class RehearseRefusalTest(unittest.TestCase):
    def test_only_hardware_refusals_apply(self):
        facts = RefusalTest().facts(
            tcp_problem="x",
            missing_slots=["overhead"],
            dream_problem="x",
            ledger_remaining=0,
            credentials_present=False,
            arm_file_explicit=False,
        )
        self.assertEqual(dream_real.rehearse_refusals(facts), [])
        for change in (
            {"approval_problem": "x"},
            {"acked_signs": []},
            {"plan_id_given": None},
            {"lab_holder": "teleoperation"},
            {"port_holders": ["pid 1"]},
            {"guard_problem": "x"},
        ):
            self.assertEqual(len(dream_real.rehearse_refusals(RefusalTest().facts(**change))), 1, change)


class StopButtonTest(unittest.TestCase):
    """The button's logic only (no window): what each press sends, and what it refuses to send."""

    def logic(self, pid_text="4242\n", command="python dream_real.py --live"):
        import stop_button

        self.tmp = tempfile.TemporaryDirectory()
        path = Path(self.tmp.name) / "dream_real.pid"
        if pid_text is not None:
            path.write_text(pid_text)
        self.kills, self.clock = [], FakeClock(100.0)
        return stop_button.StopLogic(
            path,
            kill=lambda pid, sig: self.kills.append((pid, sig)),
            clock=self.clock,
            command=lambda pid: command,
        )

    def tearDown(self):
        self.tmp.cleanup()

    def test_stop_then_release_all(self):
        import signal

        logic = self.logic()
        self.assertEqual(logic.label, "STOP")
        logic.press(from_key=True)
        self.assertEqual(self.kills, [(4242, signal.SIGTERM)])
        self.assertIn("RELEASE ALL TORQUE NOW", logic.label)
        logic.press(from_key=False)  # a double click: too soon
        self.clock.t += 0.5
        logic.press(from_key=True)  # a held key auto-repeating: keys never release all
        self.clock.t += 5
        logic.press(from_key=True)
        self.assertEqual(len(self.kills), 1)
        logic.press(from_key=False)
        self.assertEqual(self.kills, [(4242, signal.SIGTERM)] * 2)

    def test_never_signals_a_dead_or_foreign_pid(self):
        for pid_text, command in ((None, "x"), ("4242", None), ("4242", "/usr/bin/vim notes.txt")):
            logic = self.logic(pid_text, command)
            self.assertTrue(logic.press().startswith("NOT SENT"))
            self.assertEqual(self.kills, [])
            self.tearDown()
        self.tmp = tempfile.TemporaryDirectory()


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
        self.paths = [*getattr(self, "paths", []), (dict(a), dict(b))]
        return self.clear


if __name__ == "__main__":
    unittest.main()
