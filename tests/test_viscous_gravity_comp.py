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
"""Dry-air feedforward on fake arms only: no hardware, Dream or OS signals."""

import dataclasses
import hashlib
import json
import math
import sys
import threading
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import pytest

REAL = Path(__file__).resolve().parents[1] / "scripts/viscous-dream1/real"
sys.path.insert(0, str(REAL))
import dream_real as r  # noqa: E402
import dream_real_core as c  # noqa: E402
import gravity_comp as gc  # noqa: E402
import gravity_comp_report as report  # noqa: E402

import tests.test_viscous_dream_real as runner_tests  # noqa: E402


@pytest.fixture
def fit_path(tmp_path):
    fit = json.loads(gc.DEFAULT_FIT.read_text())
    fit["provisional"] = False
    path = tmp_path / "fit.json"
    path.write_text(json.dumps(fit))
    return path


def settings(path=None, scale=0.5):
    return r.Settings(
        "test",
        8,
        1,
        15,
        60,
        20,
        1,
        c.TripLimits(),
        dict.fromkeys(c.SLOTS, "black"),
        hold_s=2,
        gravity_comp=scale,
        gravity_fit=path,
    )


def episode(
    tmp_path, config, *, module=r, env="dry_air", slot="left", dream=False, callback=None, held=False
):
    tmp_path.mkdir(parents=True, exist_ok=True)
    clock = module.VirtualClock()
    arm = module.FakeArm(module.TRACE_POSE, [], clock)
    stop, abort = threading.Event(), threading.Event()
    arm_file = json.loads(module.DEFAULT_ARM_FILE.read_text())
    environment = module.ENVIRONMENTS[env]
    jmap = c.JointMap.load()
    stub = module.DreamInternalStub(jmap, arm_slot=slot) if dream else None
    worker = module.Worker(stub, clock, threaded=False, max_calls=1) if dream else None
    config.arm_slot = slot
    ep = module.Episode(
        arm,
        stub,
        worker,
        jmap,
        c.keepout_limits(environment.keepout_file),
        c.WorkspaceBox.load(environment.box_file, module.SRC_URDF, jmap, tcp_m=arm_file["tcp"]),
        module.Guard(environment),
        config,
        clock,
        clock.sleep,
        tmp_path,
        stop,
        abort,
        arm_rest=arm_file["rest"],
        environment=environment,
    )
    if held:
        original = ep.guard.path_clear
        ep.guard.path_clear = lambda *args: False if arm.stopping else original(*args)

    def sleep(dt):
        clock.sleep(dt)
        if callback:
            callback(ep, arm, clock, stop, abort)
        assert clock.t < 1100

    ep.sleep = sleep
    result = ep.run()
    rows = [json.loads(line) for line in (tmp_path / "ticks.jsonl").read_text().splitlines()]
    return ep, arm, result, rows


@pytest.mark.parametrize(
    "args",
    [
        ["--gravity-comp", "-0.1"],
        ["--gravity-comp", "1.1"],
        ["--gravity-comp", "nan"],
        ["--gravity-comp", "inf"],
        ["--gravity-comp", "0.25"],
        ["--gravity-comp", "0.25", "--environment", "dry_air", "--rehearse", "--gravity-sweep"],
    ],
)
def test_cli_refuses_bad_scale_tank_and_raw_sweep_before_external_state(args):
    with mock.patch.object(r, "load_record", side_effect=AssertionError("external state")):
        with pytest.raises(SystemExit) as error:
            r.main(args)
        assert error.value.code == 2


@pytest.mark.parametrize(
    "field,value,match",
    [
        ("robot", "other", "robot"),
        ("condition", "tank", "condition"),
        ("provisional", True, "provisional"),
    ],
)
def test_fit_refusals(fit_path, field, value, match):
    data = json.loads(fit_path.read_text())
    data[field] = value
    fit_path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match=match):
        gc.load(settings(fit_path), "viscous_waterproof_01", "dry_air")


def test_provisional_requires_explicit_override_and_ids_bind_scale_hash_and_sag(fit_path):
    fixture = runner_tests.EnvironmentTest()
    for scale in (0.25, 0.5, 1):
        plan = fixture.plan("dry_air", True, gravity_comp=scale, gravity_fit=fit_path)
        assert plan["gravity_comp"]["sag_mm_per_m2"] == c.SAG_MM_PER_M2 * (1 - scale)
    first = fixture.plan("dry_air", True, gravity_comp=0.5, gravity_fit=fit_path)
    fit_path.write_text(fit_path.read_text() + "\n")
    second = fixture.plan("dry_air", True, gravity_comp=0.5, gravity_fit=fit_path)
    assert first["plan_id"] != second["plan_id"]
    gc.load(dataclasses.replace(settings(), allow_provisional_fit=True), "viscous_waterproof_01", "dry_air")


def test_zero_scale_does_not_read_a_fit_or_change_any_plan():
    fixture = runner_tests.EnvironmentTest()
    with mock.patch.object(gc, "load", side_effect=AssertionError("fit read at zero")):
        for env in ("tank", "dry_air"):
            for rehearse in (False, True):
                for slot in ("left", "right"):
                    assert fixture.plan(env, rehearse, arm_slot=slot) == fixture.plan(
                        env,
                        rehearse,
                        arm_slot=slot,
                        gravity_comp=0,
                        gravity_fit=Path("missing"),
                        allow_provisional_fit=True,
                    )


def test_fit_changed_after_plan_is_refused_before_connect(fit_path, tmp_path):
    config = settings(fit_path)
    config.gravity_identity, _ = gc.load(config, "viscous_waterproof_01", "dry_air")
    fit_path.write_text(fit_path.read_text() + "\n")
    with (
        mock.patch.object(r.FakeArm, "connect", side_effect=AssertionError("connected")),
        pytest.raises(r.RefusedError, match="changed"),
    ):
        episode(tmp_path / "run", config)


def test_ramp_in_and_fake_sag_reduction_use_actual_innermost_installer():
    clock = r.VirtualClock()
    arm = r.FakeArm(r.TRACE_POSE, [], clock, sag={"shoulder_lift": 4})
    model = SimpleNamespace(joints=c.ARM, torque=lambda pose: dict.fromkeys(c.ARM, 1.0))
    arm.install_gravity(model, 0.5)
    arm.engage()
    assert arm.gravity_status["feedforward_nm"]["shoulder_lift"] == 0
    clock.sleep(1)
    arm.send(arm.target, arm.target)
    assert arm.gravity_status["feedforward_nm"]["shoulder_lift"] == pytest.approx(0.25)
    clock.sleep(1)
    arm.send(arm.target, arm.target)
    assert arm.gravity_status["feedforward_nm"]["shoulder_lift"] == pytest.approx(0.5)
    goal = arm.home["shoulder_lift"] + 30
    assert goal - arm._settle("shoulder_lift", goal) == pytest.approx(4 - math.degrees(0.5 / 90))
    assert all(frame[1] != "gripper" for frame in arm.ff_frames)


def test_hold_and_return_keep_ff_then_ramp_to_zero_before_release(fit_path, tmp_path):
    ep, arm, result, rows = episode(tmp_path, settings(fit_path))
    assert result["reason"].startswith("held 2 s") and result["stop"]["returned"] == "at the rest"
    assert result["stop"]["gravity_ramp"] == "zero at rest after 1 s"
    for phase in ("hold", "stop: return leg 1", "stop: return leg 2"):
        assert any(abs(row["ff"].get("shoulder_lift", 0)) > 0 for row in rows if row["phase"] == phase)
    ramp = [row for row in rows if row["phase"] == "stop: gravity ramp down"]
    assert len(ramp) == 20 and ramp[-1]["ff"]["shoulder_lift"] == 0
    assert all(row["cmd"] == ramp[0]["cmd"] for row in ramp)
    assert not arm.energized and arm.gravity_status["scale"] == 0
    assert not hasattr(arm.bus, "_makermodslab_gravity_original_write")
    assert all("gripper" not in row["ff"] for row in rows)
    assert report.report(tmp_path)["ticks"] > 1


def test_hold_mid_air_keeps_ff_until_release_all(fit_path, tmp_path):
    seen = []

    def callback(ep, arm, clock, stop, abort):
        if ep.status_phase == "stop: HOLDING":
            seen.append(dict(arm.gravity_status["feedforward_nm"]))
            assert arm.gravity_status["scale"] == 0.5
            if len(seen) == 3:
                abort.set()

    ep, arm, result, rows = episode(tmp_path, settings(fit_path), callback=callback, held=True)
    assert seen and all(abs(row["shoulder_lift"]) > 0 for row in seen)
    assert "gravity_ramp" not in result["stop"] and not arm.energized


def test_second_signal_during_ramp_releases_at_once(fit_path, tmp_path):
    def callback(ep, arm, clock, stop, abort):
        if ep.status_phase == "stop: gravity ramp down":
            abort.set()

    ep, arm, result, rows = episode(tmp_path, settings(fit_path), callback=callback)
    assert result["stop"]["gravity_ramp"] == "aborted by second signal"
    assert not any(row["phase"] == "stop: gravity ramp down" for row in rows)
    assert not arm.energized


def test_enabled_ff_also_runs_with_offline_dream_stub(fit_path, tmp_path):
    ep, arm, result, rows = episode(tmp_path, settings(fit_path), dream=True)
    assert result["calls"] == 1 and any(row["phase"] == "dream" and "ff" in row for row in rows)
    assert not arm.energized


def test_limp_uses_combined_effort_but_lag_and_hold_error_keep_actual_targets():
    arm = SimpleNamespace(gravity_status={"feedforward_nm": {"wrist_flex": -math.radians(4) * 30}})
    monitor = gc.FeedforwardLimp(c.TripLimits(), arm)
    pos = dict.fromkeys(c.ARM, 0.0)
    goal = dict(pos, wrist_flex=4.0)
    tau = dict.fromkeys(c.ARM, 1.0)
    tau["wrist_flex"] = 0
    assert not monitor.update(pos, goal, tau)  # ff cancels stiffness: healthy zero effort
    arm.gravity_status["feedforward_nm"]["wrist_flex"] = 0
    assert monitor.update(pos, goal, tau)[0].name == "limp"  # same physical lag, effort now expected
    hold = c.HoldError()
    hold.update(pos, goal, 0)
    assert hold.value()["wrist_flex"] == 4


def test_zero_ticks_events_and_summary_match_pre_feedforward_fixture(tmp_path):
    fixtures = json.loads((REAL / "gravity-zero-legacy.json").read_text())
    for case in fixtures["cases"]:
        env, dream, slot = case["environment"], case["dream"], case["slot"]
        config = settings(scale=0)
        ep, arm, result, rows = episode(
            tmp_path / f"{env}-{dream}-{slot}", config, env=env, slot=slot, dream=dream
        )

        def digest(data):
            return hashlib.sha256(json.dumps(data, sort_keys=True).encode()).hexdigest()

        assert digest(rows) == case["ticks_sha256"]
        assert digest(arm.events) == case["events_sha256"]
        assert digest(result) == case["summary_sha256"]
        assert all("ff" not in row for row in rows)


def test_floor_slide_only_scales_the_sag_term():
    arm = json.loads(r.DEFAULT_ARM_FILE.read_text())
    base, _ = r.make_floor_slide(arm, r.ENVIRONMENTS["dry_air"])
    half, _ = r.make_floor_slide(arm, r.ENVIRONMENTS["dry_air"], gravity_scale=0.5)
    full, _ = r.make_floor_slide(arm, r.ENVIRONMENTS["dry_air"], gravity_scale=1)
    assert half.sag_mm_per_m2 == base.sag_mm_per_m2 * 0.5
    assert full.sag_mm_per_m2 == 0
    assert half.floor_mm == full.floor_mm == base.floor_mm


def test_report_uses_joint_map_tank_tcp_fk_and_only_the_start_hold(tmp_path):
    pose = dict(r.TRACE_POSE)
    cmd = dict(pose, shoulder_lift=pose["shoulder_lift"] + 4)
    rows = [
        {
            "t": i * 0.05,
            "phase": "hold",
            "cmd": cmd,
            "pos": pose,
            "torque": dict.fromkeys(c.ARM, 2),
            "ff": dict.fromkeys(c.ARM, 1),
        }
        for i in range(50)
    ]
    (tmp_path / "ticks.jsonl").write_text("\n".join(map(json.dumps, rows)))
    data = report.report(tmp_path)
    assert data["joints"]["shoulder_lift"]["cmd_minus_pos_deg"] == 4
    assert data["joints"]["shoulder_lift"]["ff_nm"] == 1
    jmap = c.JointMap.load()
    joints = c.parse_urdf((REAL / "tank_model/viscous_waterproof_tank.urdf").read_bytes())
    arm = json.loads(r.DEFAULT_ARM_FILE.read_text())
    tcp = [*arm["tcp"], 1]

    def height(q):
        return (
            c.fk(joints, "dream_base", "wrist_roll_link", dict(zip(c.ARM, jmap.to_ik(q), strict=True))) @ tcp
        )[2] * 1000

    assert data["vertical_sag_mm"] == pytest.approx(height(cmd) - height(pose))


@pytest.mark.parametrize("fault", ["send", "feedback"])
def test_ramp_io_failure_holds_remaining_ff_until_release_all(fit_path, tmp_path, fault):
    seen = []

    def callback(ep, arm, clock, stop, abort):
        if ep.status_phase == "stop: HOLDING":
            seen.append(arm.gravity_status["scale"])
            assert arm.energized
            abort.set()

    method = "send" if fault == "send" else "read"
    original = getattr(r.FakeArm, method)

    def fail_once(arm, *args):
        # Only the ramp changes the live scale below its requested value.
        status = getattr(arm, "gravity_status", None)
        if status and 0 < status["scale"] < 0.5 and not seen:
            seen.append("fault")
            raise ConnectionError("injected ramp failure")
        return original(arm, *args)

    with mock.patch.object(r.FakeArm, method, fail_once):
        ep, arm, result, rows = episode(tmp_path, settings(fit_path), callback=callback)
    assert "interrupted" in result["stop"]["gravity_ramp"]
    assert any(isinstance(value, float) and value > 0 for value in seen)
    assert not arm.energized
