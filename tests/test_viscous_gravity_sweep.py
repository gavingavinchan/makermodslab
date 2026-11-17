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
"""Offline sweep and identification contracts; no hardware, signals or Dream."""

import dataclasses
import json
import sys
import threading
from pathlib import Path
from unittest import mock

import numpy as np
import pytest

REAL = Path(__file__).resolve().parents[1] / "scripts/viscous-dream1/real"
sys.path.insert(0, str(REAL))
import dream_real as runner  # noqa: E402
import dream_real_core as core  # noqa: E402
import gravity_fit as fit  # noqa: E402
import gravity_sweep as sweep  # noqa: E402

from makermodslab.arm_gravity import VISCOUS_URDF, GravityModel  # noqa: E402


@pytest.fixture(scope="module")
def context():
    env = runner.ENVIRONMENTS["dry_air"]
    arm_file = json.loads(
        (REAL.parents[2] / "tests/fixtures/viscous_gravity/viscous-waterproof-dry-air.json").read_text()
    )
    jmap = core.JointMap.load()
    hub = {**jmap.to_motor(arm_file["rest"]), "gripper": -47.35}
    limits = core.keepout_limits(env.keepout_file)
    box = core.WorkspaceBox.load(env.box_file, runner.SRC_URDF, jmap, tcp_m=arm_file["tcp"])
    guard = runner.Guard(env)
    settings = sweep.SweepSettings(poses=2)
    visits, report = sweep.plan(settings, hub, limits, box, guard, 15, pool=48)
    return env, arm_file, hub, limits, box, settings, visits, report


def test_seed_deterministic_and_resume_keeps_the_same_poses(context):
    env, _, hub, limits, box, settings, visits, report = context
    again, report2 = sweep.plan(settings, hub, limits, box, runner.Guard(env), 15, pool=48)
    assert again == visits
    assert {k: v for k, v in report2.items() if k != "planning_time_s"} == {
        k: v for k, v in report.items() if k != "planning_time_s"
    }
    resumed, _ = sweep.plan(
        dataclasses.replace(settings, start_at=1), hub, limits, box, runner.Guard(env), 15, pool=48
    )
    assert resumed == visits
    assert len(list(sweep.actions(resumed, dataclasses.replace(settings, start_at=1)))) < len(
        list(sweep.actions(visits, settings))
    )


def test_every_substep_is_guarded_sagged_torque_capped_and_can_return(context):
    env, _, hub, limits, box, settings, visits, _ = context
    mesh = runner.Guard(env).guard
    predictor = sweep.Predictor(settings, hub)
    assert len(visits) == settings.poses
    for visit in visits:
        previous = hub
        for route in visit["routes"]:
            assert route["direction"] in ("+", "-")
            for q in route["into"] + route["settle"]:
                assert max(abs(q[n] - previous[n]) for n in core.ARM) <= 15 * 0.05 + 1e-8
                assert q["shoulder_pan"] == hub["shoulder_pan"] and q["gripper"] == hub["gripper"]
                assert mesh.limit(previous, q).status == "ok"
                assert not box.violations(q)
                assert mesh.check_pose(predictor.sagged(q)).ok
                assert mesh.check_return_segment(q, hub).ok
                assert mesh.check_return_segment(predictor.sagged(q), hub).ok
                assert max(map(abs, predictor.torque(q).values())) <= 6
                previous = q
        for q in visit["back"]:
            assert mesh.limit(previous, q).status == "ok"
            assert mesh.check_pose(predictor.sagged(q)).ok
            assert mesh.check_return_segment(q, hub).ok
            previous = q
        assert previous == pytest.approx(hub)


def test_back_post_and_desk_candidates_are_dropped(context):
    env, _, hub, limits, box, settings, _, _ = context
    # All candidates folded across the +106 mm post or down through the desk.
    for bounds in (
        {"shoulder_lift": (-190.0, -185.0), "elbow_flex": (12.0, 20.0), "wrist_flex": (-100.0, -90.0)},
        {"shoulder_lift": (-30.0, -20.0), "elbow_flex": (80.0, 90.0), "wrist_flex": (20.0, 30.0)},
    ):
        candidate_box = {n: (hub[n] - 1, hub[n] + 1) for n in sweep.MOVING} | bounds
        with pytest.raises(ValueError, match="only 0 checked poses"):
            sweep.plan(
                dataclasses.replace(settings, poses=1, box=candidate_box),
                hub,
                limits,
                box,
                runner.Guard(env),
                15,
                pool=8,
            )


def episode(
    context, tmp_path, stop_after=None, hot=False, torque_poses=(), torque_phase="hold", stop_retrace=False
):
    env, arm_file, hub, limits, box, settings, visits, report = context
    clock = runner.VirtualClock()
    arm = runner.FakeArm(hub, [], clock, heat_per_tick=0)
    stop, abort = threading.Event(), threading.Event()
    config = runner.Settings("unused", 8, 1, 15.0, 60.0, 20.0, 1.0, core.TripLimits(), {}, sweep=settings)
    ep = runner.Episode(
        arm,
        None,
        None,
        core.JointMap.load(),
        limits,
        box,
        runner.Guard(env),
        config,
        clock,
        clock.sleep,
        tmp_path,
        stop,
        abort,
        arm_rest=arm_file["rest"],
        environment=env,
        mirror_dir=tmp_path / "raw",
        status_path=tmp_path / "status.json",
    )
    (tmp_path / "raw").mkdir(parents=True)
    arm.sent = []
    original_send, original_read = arm.send, arm.read

    def send(target, previous):
        arm.sent.append((ep.phase, dict(target)))
        original_send(target, previous)

    def read():
        reading = original_read()
        info = ep.sweep_observation or {}
        if info.get("pose") in torque_poses and torque_phase in ep.phase and not info.get("retrace"):
            reading.torque["elbow_flex"] = 7.1
        return reading

    arm.send, arm.read = send, read

    def sleep(dt):
        clock.sleep(dt)
        if stop_after is not None and ep.phase.startswith("sweep:") and arm.ticks >= stop_after:
            stop.set()  # the handler's event seam, never an OS signal
        if stop_retrace and "retrace" in ep.phase:
            stop.set()
        if hot and ep.phase.startswith("sweep: hub after"):
            old = arm.read

            def read_hot():
                r = old()
                r.temp["wrist_flex"] = 65.0
                return r

            arm.read = read_hot
        assert clock.t < 1800  # no infinite fake HOLD

    ep.sleep = sleep

    def planned(*args, **kwargs):
        assert not arm.energized
        return visits, dict(report)

    with mock.patch.object(sweep, "plan", side_effect=planned):
        result = ep.run()
    return ep, arm, result


def test_holds_write_last_second_medians_and_mirror(context, tmp_path):
    ep, arm, result = episode(context, tmp_path)
    assert result["reason"] == "gravity sweep complete"
    assert result["stop"]["returned"] == "at the rest"
    assert not arm.energized and arm.gripper_limp
    data = json.loads((tmp_path / "samples.json").read_text())
    assert data == json.loads((tmp_path / "raw/samples.json").read_text())
    holds = data["samples"]
    assert len(holds) == sum(len(v["routes"]) for v in context[6])
    ticks = [json.loads(s) for s in (tmp_path / "ticks.jsonl").read_text().splitlines()]
    for hold in holds:
        phase = f"sweep: pose {hold['pose'] + 1}/2 hold from {hold['approach']}"
        rows = [r for r in ticks if r["phase"] == phase]
        assert len(rows) == 40 and hold["ticks"] == 20
        for key in ("pos", "cmd", "torque", "temp"):
            assert hold[key]["shoulder_lift"] == pytest.approx(
                np.median([r[key]["shoulder_lift"] for r in rows[-20:]])
            )
        expected = -1 if hold["approach"] == "+" else 1
        visit = context[6][hold["pose"]]
        route = next(r for r in visit["routes"] if r["direction"] == hold["approach"])
        assert all(hold["direction"][n] == expected * np.sign(route["delta_deg"][n]) for n in sweep.APPROACH)
    assert json.loads((tmp_path / "status.json").read_text())["state"] == "released"


def test_mid_sweep_stop_uses_existing_stop_path_and_releases_at_rest(context, tmp_path):
    ep, arm, result = episode(context, tmp_path, stop_after=35)
    assert result["reason"] == "stop signal"
    assert ep.phase.startswith("sweep:")
    assert result["stop"]["reanchor"]
    assert result["stop"]["gripper_release"] == "ok"
    assert result["stop"]["returned"] == "at the rest"
    assert arm.pos == pytest.approx(ep.start_pose)
    assert not arm.energized
    assert arm.events[-1].startswith("release:")


def test_hot_hub_uses_stop_and_prints_resume_index(context, tmp_path):
    _, arm, result = episode(context, tmp_path, hot=True)
    assert result["reason"] == "sweep paused: hot"
    assert result["sweep_resume_start_at"] == 1
    assert result["stop"]["returned"] == "at the rest" and not arm.energized


@pytest.mark.parametrize(
    "extra",
    [
        [],
        ["--rehearse"],
        ["--rehearse", "--environment", "dry_air", "--speed-deg-s", "16"],
        *[
            ["--rehearse", "--environment", "dry_air", "--no-" + n + "-trip"]
            for n in ("lag", "limp", "overspeed")
        ],
    ],
)
def test_cli_refuses_before_loading_any_external_state(extra):
    with (
        mock.patch.object(sys.stdout, "reconfigure", create=True),
        mock.patch.object(runner, "load_record", side_effect=AssertionError),
    ):
        with pytest.raises(SystemExit) as exc:
            runner.main(["--gravity-sweep", *extra])
        assert exc.value.code == 2


def test_sweep_parameters_all_change_identity():
    base = sweep.SweepSettings()
    identity = core.canonical_id(base.identity())
    for field, value in {
        "seed": 1,
        "poses": 29,
        "hold_s": 3.0,
        "max_torque_nm": 5.0,
        "hot_c": 60.0,
        "start_at": 1,
        "box": dict.fromkeys(sweep.MOVING, (-30.0, 30.0)),
    }.items():
        assert core.canonical_id(dataclasses.replace(base, **{field: value}).identity()) != identity


def test_fit_hash_binds_identity_and_bad_fit_falls_back(context, tmp_path):
    path = tmp_path / "fit.json"
    path.write_bytes(sweep.DEFAULT_OUT.read_bytes())
    config = dataclasses.replace(context[5], fit=path)
    old = config.identity()
    path.write_text("{}")
    assert old["fit_sha256"] != config.identity()["fit_sha256"]
    predictor = sweep.Predictor(config, context[2])
    assert "fallback" in predictor.description and "CAD scaled" in predictor.description
    assert np.isfinite(list(predictor.torque(context[2]).values())).all()


def test_tick_selection_excludes_motion_trips_transitions_and_stop():
    rows = []
    for i in range(160):
        t = i * 0.05
        pos = dict.fromkeys(core.ARM, 0.0)
        cmd = dict(pos, shoulder_lift=min(1.0, t))
        rows.append(
            {
                "t": t,
                "phase": "hold" if i < 120 else "stop: return leg 1",
                "pos": pos,
                "cmd": cmd,
                "torque": dict(pos),
                "temp": dict.fromkeys(core.ARM, 30.0),
                "trips": ["lag"] if i == 80 else [],
            }
        )
    out = fit.select_ticks(rows)
    assert out and all(1.5 <= r["t"] < 6 for r in out)
    assert all(r["direction"]["shoulder_lift"] == 1 for r in out)
    assert all(not 4.0 <= r["t"] <= 4.5 for r in out)
    for row in rows:
        row["pos"]["wrist_yaw"] = row["t"] * 2
    assert not fit.select_ticks(rows)


def test_fitting_uses_no_cad_prior_and_reports_identifiable_combinations():
    model = GravityModel(VISCOUS_URDF, core.ARM, joint_map=fit.joint_map())
    rows = json.loads(sweep.REFERENCE.read_text())["rows"][:20]
    for row in rows:
        row["torque"] = dict.fromkeys(core.ARM, 0.0)
    theta, ident = fit.solve(model, rows)
    assert np.array_equal(theta, np.zeros_like(theta))
    assert ident["rank"] < 24
    assert len(ident["identified_combinations"]) == ident["rank"]


def test_samples_require_dry_air_real_metadata_and_group_by_pose(tmp_path):
    data = {"environment": {"name": "dry_air"}, "samples": [{"pose": 0}, {"pose": 0}, {"pose": 1}]}
    path = tmp_path / "samples.json"
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match="meta.json"):
        fit.load_inputs([path])
    meta = tmp_path / "meta.json"
    meta.write_text(json.dumps({"environment": {"name": "tank"}, "kind": "rehearse-live"}))
    with pytest.raises(ValueError, match="dry_air"):
        fit.load_inputs([path])
    meta.write_text(json.dumps({"environment": {"name": "dry_air"}, "kind": "rehearse-live"}))
    rows, _ = fit.load_inputs([path])
    assert rows[0]["group"] == rows[1]["group"] != rows[2]["group"]
    data["fake"] = True
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match="Fake"):
        fit.load_inputs([path])


def test_regressor_and_empirical_solution_ignore_urdf_inertials(tmp_path):
    import xml.etree.ElementTree as ET

    root = ET.fromstring(VISCOUS_URDF.read_bytes())
    for inertial in root.findall(".//inertial"):
        inertial.find("mass").set("value", "17")
        inertial.find("origin").set("xyz", "0.3 0.2 0.1")
    path = tmp_path / "different-inertials.urdf"
    path.write_bytes(ET.tostring(root))
    first = GravityModel(VISCOUS_URDF, core.ARM, joint_map=fit.joint_map())
    other = GravityModel(path, core.ARM, joint_map=fit.joint_map())
    rows = json.loads(sweep.REFERENCE.read_text())["rows"][:20]
    assert not np.allclose(first.prior, other.prior)
    assert np.allclose(fit.design(first, rows)[0], fit.design(other, rows)[0])
    assert np.allclose(fit.solve(first, rows)[0], fit.solve(other, rows)[0])


def test_lab_adapter_sends_no_gripper_targets_after_sweep_engage():
    from makermodslab import remote_can

    arm = object.__new__(runner.LabArm)
    arm.robot = object()
    arm.gripper_released = False
    arm.freeze_gripper = True
    pose = dict.fromkeys(core.MOTORS, 0.0)
    with mock.patch.object(remote_can, "send_action") as send:
        arm.send(pose, pose)
    assert set(send.call_args.args[1]) == {f"{n}.pos" for n in core.ARM}


def test_sweep_refusal_never_engages(context, tmp_path):
    env, arm_file, hub, limits, box, settings, _, _ = context
    clock = runner.VirtualClock()
    arm = runner.FakeArm(hub, [], clock)
    config = runner.Settings("unused", 8, 1, 15.0, 60.0, 20.0, 1.0, core.TripLimits(), {}, sweep=settings)
    ep = runner.Episode(
        arm,
        None,
        None,
        core.JointMap.load(),
        limits,
        box,
        runner.Guard(env),
        config,
        clock,
        clock.sleep,
        tmp_path,
        threading.Event(),
        threading.Event(),
        arm_rest=arm_file["rest"],
        environment=env,
    )
    with mock.patch.object(sweep, "plan", side_effect=ValueError("no proved return")):
        result = ep.run()
    assert result["reason"] == "refused: no proved return"
    assert not arm.energized and not any(e.startswith("engage:") for e in arm.events)


def test_provisional_fit_cannot_be_enabled_for_feedforward(monkeypatch):
    from types import SimpleNamespace

    from makermodslab import gravity_feedforward as ff

    monkeypatch.setenv(ff.ENV_FIT, str(sweep.DEFAULT_OUT))
    robot = SimpleNamespace(bus=SimpleNamespace(_mit_control=lambda: None, _gains={}), _last_positions={})
    with pytest.raises(ValueError, match="Provisional"):
        ff.install_from_env(robot, "viscous_waterproof_01")


def test_stop_shortcut_is_also_proved_before_engage(context, tmp_path):
    env, arm_file, hub, limits, box, settings, _, _ = context
    clock = runner.VirtualClock()
    config = runner.Settings("unused", 8, 1, 15.0, 60.0, 20.0, 1.0, core.TripLimits(), {}, sweep=settings)
    guard = runner.Guard(env)
    ep = runner.Episode(
        runner.FakeArm(hub, [], clock),
        None,
        None,
        core.JointMap.load(),
        limits,
        box,
        guard,
        config,
        clock,
        clock.sleep,
        tmp_path,
        threading.Event(),
        threading.Event(),
        arm_rest=arm_file["rest"],
        environment=env,
    )
    ep.start_pose, ep.dream_start, ep.phase = dict(runner.TRACE_POSE), hub, "sweep"
    with mock.patch.object(guard, "path_clear", return_value=False) as check:
        assert not ep._sweep_stop_clear(hub)
        check.assert_called_once()
    with pytest.raises(ValueError, match="hub fails"):
        sweep.plan(settings, hub, limits, box, guard, 15, pool=1, stop_path_clear=lambda p: False)


@pytest.mark.parametrize("phase", ["approach", "hold", "return hub"])
def test_excess_sweep_torque_retraces_sent_substeps_and_continues(context, tmp_path, phase):
    ep, arm, result = episode(context, tmp_path, torque_poses=(0,), torque_phase=phase)
    assert result["reason"] == "gravity sweep complete"
    assert result["stop"]["returned"] == "at the rest"
    data = json.loads((tmp_path / "samples.json").read_text())
    assert len(data["skipped_poses"]) == 1
    assert data["skipped_poses"][0]["joint"] == "elbow_flex"
    sent = [q for name, q in arm.sent if name.startswith("sweep: pose 1/2")]
    # Consecutive holds resend the same target: only movement substeps are retraced.
    unique = [context[2]]
    for q in sent:
        if q != unique[-1]:
            unique.append(q)
    retrace = [q for name, q in arm.sent if "retrace" in name]
    assert retrace == list(reversed(unique[:-1]))
    assert any(h["pose"] == 1 for h in data["samples"])
    assert not arm.energized


def test_two_torque_skips_end_at_hub_through_normal_stop(context, tmp_path):
    ep, arm, result = episode(context, tmp_path, torque_poses=(0, 1))
    assert result["reason"] == "sweep ended: two consecutive torque skips"
    assert len(ep.sweep_skips) == 2
    assert result["stop"]["returned"] == "at the rest"
    assert not arm.energized


def test_button_wins_during_torque_retrace(context, tmp_path):
    ep, arm, result = episode(context, tmp_path, torque_poses=(0,), stop_retrace=True)
    assert result["reason"] == "stop signal"
    assert result["stop"]["returned"] == "at the rest"
    assert not arm.energized


def test_narrow_defaults_and_hub_cap_exemption_keep_other_hub_checks(context):
    from types import SimpleNamespace

    env, _, hub, limits, box, settings, _, _ = context
    assert sweep.SweepSettings().poses == 14
    assert sweep.DEFAULT_BOX["shoulder_lift"] == (-86, -50)
    predictor = SimpleNamespace(
        torque=lambda q: dict.fromkeys(core.ARM, 6.03),
        sagged=lambda q: dict(q),
    )
    proof = sweep.Proof(settings, hub, limits, box, runner.Guard(env), 15, predictor, None)
    assert proof.checked(hub) is None
    with pytest.raises(ValueError, match="cap:"):
        proof.checked(hub | {"elbow_flex": hub["elbow_flex"] + 1})
    with pytest.raises(ValueError, match="stop_path"):
        sweep.Proof(settings, hub, limits, box, runner.Guard(env), 15, predictor, lambda q: False)


@pytest.fixture
def saved_plan(context, tmp_path):
    env, _, hub, limits, box, settings, visits, report = context
    path = tmp_path / "sweep-plan.json"
    sweep.save_plan(path, settings, hub, visits, report, 15)
    return dataclasses.replace(settings, plan_file=path)


def test_saved_plan_reproves_without_search_and_binds_file_hash(context, saved_plan):
    env, _, hub, limits, box, settings, visits, _ = context
    with mock.patch.object(sweep, "plan", side_effect=AssertionError("candidate search")):
        loaded, report = sweep.load_plan(saved_plan, hub, limits, box, runner.Guard(env), 15)
    assert loaded == visits
    assert report["both_direction_poses"] == 2 and report["reproof_time_s"] > 0
    before = saved_plan.identity()
    saved_plan.plan_file.write_text(saved_plan.plan_file.read_text() + "\n")
    assert before["plan_sha256"] != saved_plan.identity()["plan_sha256"]
    assert sweep.saved_identity(saved_plan) == sweep.saved_identity(settings)


@pytest.mark.parametrize(
    "bad", ["hub", "speed", "nan", "pan", "box", "direction", "empty", "endpoint", "settings"]
)
def test_saved_plan_tampering_refuses_before_engage(context, saved_plan, tmp_path, bad):
    data = json.loads(saved_plan.plan_file.read_text())
    route = data["visits"][0]["routes"][0]
    if bad == "hub":
        data["hub"]["shoulder_lift"] += 0.51
    elif bad == "speed":
        route["into"][0]["elbow_flex"] += 10
    elif bad == "nan":
        route["into"][0]["wrist_roll"] = float("nan")
    elif bad == "pan":
        route["into"][0]["shoulder_pan"] += 0.1
    elif bad == "box":
        data["visits"][0]["pose"]["wrist_yaw"] = 11
    elif bad == "direction":
        data["visits"][0]["routes"].pop()
    elif bad == "empty":
        route["into"] = []
    elif bad == "endpoint":
        route["settle"][-1]["wrist_roll"] += 0.1
    elif bad == "settings":
        data["settings"]["max_torque_nm"] = 5
    saved_plan.plan_file.write_text(json.dumps(data))
    items = list(context)
    items[5] = saved_plan
    _, arm, result = episode(tuple(items), tmp_path / "run")
    assert result["reason"].startswith("refused:")
    assert not any(e.startswith("engage:") for e in arm.events)


@pytest.mark.parametrize("failure", ["sag", "return", "stop"])
def test_loaded_substeps_repeat_sag_and_stop_proofs(context, saved_plan, failure):
    env, _, hub, limits, box, _, _, _ = context
    guard = runner.Guard(env)
    options = {}
    if failure == "sag":
        real_sag = sweep.Predictor.sagged

        def sag(predictor, q):
            return real_sag(predictor, q) | {"wrist_flex": 10000}

        patch = mock.patch.object(sweep.Predictor, "sagged", sag)
    elif failure == "return":
        patch = mock.patch.object(guard.guard, "check_return_segment", return_value=mock.Mock(ok=False))
    else:
        options["stop_path_clear"] = lambda q: False
        patch = mock.patch.object(sweep, "plan", side_effect=AssertionError("search"))
    with patch, pytest.raises(ValueError):
        sweep.load_plan(saved_plan, hub, limits, box, guard, 15, **options)






def test_loader_compares_the_snapshot_hash_bound_by_the_printed_plan(context, saved_plan):
    env, _, hub, limits, box, _, _, _ = context
    digest = saved_plan.identity()["plan_sha256"]
    saved_plan.plan_file.write_text(saved_plan.plan_file.read_text() + "\n")
    with pytest.raises(ValueError, match="changed while loading"):
        sweep.load_plan(saved_plan, hub, limits, box, runner.Guard(env), 15, expected_sha256=digest)


