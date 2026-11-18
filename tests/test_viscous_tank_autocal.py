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
"""Tank autocal (scripts/viscous-tank-autocal): plan safety, press detection on synthetic traces,
the fit on synthetic touches, and the abort paths on the numpy fake arm. No hardware, no MuJoCo."""

from __future__ import annotations

import json
import math
import sys
import threading
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parents[1] / "scripts/viscous-tank-autocal"
sys.path.insert(0, str(HERE))

import context  # noqa: E402
import fitting  # noqa: E402
import geometry  # noqa: E402
import planner as probe_planner  # noqa: E402
import press as press_checks  # noqa: E402

ARM = geometry.ARM


@pytest.fixture(scope="module")
def setup(tmp_path_factory):
    folder = tmp_path_factory.mktemp("autocal")
    model = context.offline_model(folder / "model")
    tank, keepout = geometry.load_inputs()
    rest = context.arm_file_rest(model)
    planner = probe_planner.Planner(
        model,
        keepout,
        model.limits,
        rest,
        bank=probe_planner.SeedBank(model.guard.model, model.limits, n=40000),
    )
    nominal = geometry.nominal_estimate(tank, keepout)
    return {
        "model": model,
        "tank": tank,
        "keepout": keepout,
        "rest": rest,
        "planner": planner,
        "nominal": nominal,
        "folder": folder,
    }


FLOOR = probe_planner.PROBES[0]


@pytest.fixture(scope="module")
def floor_plan(setup):
    plan = setup["planner"].plan(FLOOR, setup["nominal"], [], setup["planner"].hub(setup["nominal"]))
    assert plan.ok, plan.reasons
    return plan


# --- geometry and plan safety ------------------------------------------------------------------


def test_back_plane_comes_from_the_post_not_a_probe(setup):
    touch = geometry.back_touch(setup["nominal"])
    assert touch["label"] == "near" and touch["point_mm"][:2] == [118.0, 10.0]
    assert all(s.face in geometry.PROBED_FACES for s in probe_planner.PROBES + probe_planner.SPARES)


def test_normal_stiffness_matches_a_one_joint_lever():
    # One joint, a 0.4 m lever along x, kp 60: a push along y sees kp / L^2; along the lever, nothing.
    jac = np.zeros((3, 6))
    jac[1, 0] = 0.4
    kp = dict.fromkeys(ARM, 1e12)
    kp["shoulder_pan"] = 60.0
    assert geometry.normal_stiffness(jac, np.array([0, 1.0, 0]), kp) == pytest.approx(60 / 0.16, rel=1e-6)
    assert math.isinf(geometry.normal_stiffness(jac, np.array([1.0, 0, 0]), dict.fromkeys(ARM, 30.0)))


def test_planned_floor_probe_is_slow_soft_and_jaw_first(setup, floor_plan):
    plan = floor_plan
    assert plan.press_force_n <= probe_planner.F_MAX_N + 1e-9
    assert plan.d2_mm >= probe_planner.D2_MIN_MM and plan.d1_mm < plan.d2_mm
    assert plan.lead_mm >= probe_planner.LEAD_MIN_MM
    assert plan.standoff_mm >= probe_planner.STANDOFF_MIN_MM
    # The deepest command stops at the window plus the deep press, never further.
    assert plan.node_s_mm[-1] == pytest.approx(plan.standoff_mm + plan.window_mm + plan.d2_mm)
    # Every node is a 4 mm step: at 3 mm/s no joint target moves near 15 deg/s.
    for a, b, sa, sb in zip(
        plan.nodes[:-1], plan.nodes[1:], plan.node_s_mm[:-1], plan.node_s_mm[1:], strict=True
    ):
        dt = (sb - sa) / probe_planner.APPROACH_MM_S
        assert max(abs(a[m] - b[m]) for m in ARM) / dt <= 15
    # The lift and the standoff pass the NORMAL keep-out; the press line passes the probe one.
    model = setup["model"]
    assert model.guard.check_pose(plan.lift).ok and model.guard.check_pose(plan.nodes[0]).ok
    guard = probe_planner.probe_guard(
        setup["keepout"], model.guard.model, setup["nominal"], "floor", plan.window_mm + plan.d2_mm + 5
    )
    assert all(guard.check_segment(a, b).ok for a, b in zip(plan.nodes, plan.nodes[1:], strict=False))


def test_probe_guard_relaxes_only_the_probed_face(setup):
    model = setup["model"]
    normal = probe_planner.probe_guard(setup["keepout"], model.guard.model, setup["nominal"], "left", 0.0)
    probe = probe_planner.probe_guard(setup["keepout"], model.guard.model, setup["nominal"], "left", 60.0)
    for link, faces in probe.required.items():
        for face, need in faces.items():
            if face == "left" and need is not None:
                assert need <= normal.required[link][face]
                if link in geometry.FINGERS:
                    assert need == -60.0
                else:
                    pad = model.guard.model and probe_planner.tank_guard._value(
                        setup["keepout"]["padding_mm"]
                    )
                    assert (
                        need >= float(pad.get(link, pad["default"])) + probe_planner.LEAD_MIN_MM - 60.0 - 1e-9
                    )
            else:
                assert need == normal.required[link][face]


def test_a_stiff_pose_is_refused(setup, monkeypatch, floor_plan):
    # Make the arm 100x stiffer: the same probe can no longer press D2_MIN_MM under F_MAX_N.
    monkeypatch.setattr(probe_planner, "KP", {m: v * 100 for m, v in probe_planner.KP.items()})
    plan = setup["planner"].plan(FLOOR, setup["nominal"], [], None)
    assert not plan.ok
    assert any("too stiff" in r for r in plan.reasons)


def test_uncertainty_shrinks_with_touches(setup):
    est = setup["nominal"]
    first = probe_planner.uncertainty(est, [], "left", -330)
    assert first >= probe_planner.U_CENTRE_MM
    touches = [
        {"label": "left", "point_mm": est.surface_point("left", -230, 100).tolist()},
        {"label": "right", "point_mm": est.surface_point("right", -430, 100).tolist()},
    ]
    est2 = probe_planner.update_estimate(est, touches)
    assert probe_planner.uncertainty(est2, touches, "left", -330) < first / 2


# --- detection on synthetic traces ------------------------------------------------------------


def _baseline(rng, n=20, sigma=0.15):
    return press_checks.baseline_from(rng.normal(0.4, sigma, n), [dict.fromkeys(ARM, 1.0) for _ in range(n)])


def _run(detector, trace):
    for s, div, tau in trace:
        if event := detector.update(s, div, tau):
            return event
    return None


def test_free_space_with_noise_and_drift_never_fires():
    rng = np.random.default_rng(1)
    det = press_checks.PressDetector(_baseline(rng), probe_planner.TORQUE_BACKSTOP_NM, ARM)
    trace = [
        (s, 0.4 + 0.02 * s + rng.normal(0, 0.15), {m: 1.0 + rng.normal(0, 0.15) for m in ARM})
        for s in np.arange(0, 70, 0.15)
    ]
    assert _run(det, trace) is None


def test_rigid_contact_fires_and_estimates_the_onset():
    rng = np.random.default_rng(2)
    det = press_checks.PressDetector(_baseline(rng), probe_planner.TORQUE_BACKSTOP_NM, ARM)
    onset = 41.0
    trace = [
        (
            s,
            0.4 + max(0.0, s - onset) + rng.normal(0, 0.15),
            {m: 1.0 + 0.1 * max(0.0, s - onset) for m in ARM},
        )
        for s in np.arange(0, 70, 0.15)
    ]
    event = _run(det, trace)
    assert event is not None and event.kind == "contact"
    assert abs(event.onset_mm - onset) < 1.0
    assert event.s_mm - onset < press_checks.DETECT_MM + 2.0  # it stops the advance within a few mm


def test_a_single_spike_is_not_contact():
    rng = np.random.default_rng(3)
    det = press_checks.PressDetector(_baseline(rng), probe_planner.TORQUE_BACKSTOP_NM, ARM)
    trace = [
        (s, 0.4 + (8.0 if abs(s - 20) < 0.2 else 0.0), dict.fromkeys(ARM, 1.0))
        for s in np.arange(0, 40, 0.15)
    ]
    assert _run(det, trace) is None


def test_torque_without_the_jaw_stopping_is_a_false_contact():
    rng = np.random.default_rng(4)
    det = press_checks.PressDetector(_baseline(rng), probe_planner.TORQUE_BACKSTOP_NM, ARM)
    trace = [
        (s, 0.4 + rng.normal(0, 0.1), {**dict.fromkeys(ARM, 1.0), "elbow_flex": 1.0 + (2.0 if s > 15 else 0)})
        for s in np.arange(0, 40, 0.15)
    ]
    event = _run(det, trace)
    assert event is not None and event.kind == "torque" and "elbow_flex" in event.detail


def _hold(point, n, excess, rng, t0=0.0, force=3.0, jitter=0.05):
    pos = dict.fromkeys(ARM, 10.0)
    return [
        {
            "t": t0 + 0.05 * i,
            "pos": {m: v + rng.normal(0, 0.01) for m, v in pos.items()},
            "arm": ARM,
            "point": np.asarray(point) + rng.normal(0, jitter, 3),
            "div_excess": excess,
            "force_n": force,
        }
        for i in range(n)
    ]


def test_judge_accepts_rigid_glass_and_records_sliding():
    rng = np.random.default_rng(5)
    n = np.array([0, 0, -1.0])
    p1 = _hold([-330, 0, -28.0], 24, 8.0, rng, force=2.4)
    p2 = _hold([-334, 1, -28.1], 24, 14.0, rng, t0=2, force=4.2)
    v = press_checks.judge(p1, p2, 8.0, 14.0, n, 300.0, [])
    assert v["ok"], v["reasons"]
    assert v["slide_mm"] == pytest.approx(math.hypot(4, 1), abs=0.3)


def test_judge_rejects_something_soft():
    rng = np.random.default_rng(6)
    n = np.array([0, 1.0, 0])
    p1 = _hold([-330, 200.0, 100], 24, 5.0, rng)
    p2 = _hold([-330, 204.0, 100], 24, 7.0, rng, t0=2)  # the "plane" moved 4 mm with the push
    v = press_checks.judge(p1, p2, 8.0, 14.0, n, 300.0, [])
    assert not v["ok"]
    assert any("not rigid" in r for r in v["reasons"])
    assert any("gave way" in r or "bent the arm only" in r for r in v["reasons"])


def test_judge_rejects_a_disagreeing_torque_and_geometry_problems():
    rng = np.random.default_rng(7)
    n = np.array([0, 0, -1.0])
    p1 = _hold([-330, 0, -28.0], 24, 8.0, rng, force=0.1)
    p2 = _hold([-330, 0, -28.0], 24, 14.0, rng, t0=2, force=0.1)
    v = press_checks.judge(p1, p2, 8.0, 14.0, n, 300.0, ["the jaw is 2.0 mm from the measured left"])
    assert not v["ok"] and len(v["reasons"]) == 2


# --- the fit on synthetic touches --------------------------------------------------------------


def _truth(nominal, yaw=1.7, shift=-12.0, floor=6.0):
    truth = nominal.copy()
    truth.yaw_deg = yaw
    bearing = geometry.geo.tank_axes(yaw) @ np.array([geometry.BACK_GLASS_X_MM, geometry.POST_CENTRE_Y_MM, 0])
    truth.near = float(bearing[0])
    mid = (nominal.left + nominal.right) / 2 + shift
    truth.left, truth.right = mid + nominal.dims["inner_width"] / 2, mid - nominal.dims["inner_width"] / 2
    truth.floor = nominal.floor + floor
    return truth


def _touches(truth):
    return [
        {"label": s.label, "point_mm": truth.surface_point(s.face, s.u, s.v).tolist(), "positions": None}
        for s in probe_planner.PROBES
    ]


def test_fit_recovers_planes_and_yaw_from_exact_touches(setup):
    truth = _truth(setup["nominal"])
    result = fitting.robust_fit(_touches(truth), setup["tank"], setup["nominal"])
    fit = result["fit"]
    assert fit["valid"], fit["failures"]
    assert fit["yaw_deg"] == pytest.approx(truth.yaw_deg, abs=0.01)
    for face in ("left", "right", "floor", "near"):
        assert fit["offsets_mm"][face] == pytest.approx(getattr(truth, face), abs=0.05)
    assert not result["dropped"]
    report = fitting.placement_report(fit, setup["keepout"])
    assert report["j1_off_centre_mm"] == pytest.approx(-(truth.left + truth.right) / 2, abs=0.05)
    assert any("yawed" in f for f in report["flags"])  # 1.7 deg > the 1.5 deg flag


def test_fit_drops_a_touch_that_is_not_on_the_glass(setup):
    truth = _truth(setup["nominal"], yaw=0.4, shift=5.0, floor=-3.0)
    touches = _touches(truth)
    touches[0]["point_mm"][2] += 25.0  # the first floor touch landed on a 25 mm object
    result = fitting.robust_fit(touches, setup["tank"], setup["nominal"])
    assert result["fit"]["valid"], result["fit"]["failures"]
    assert result["dropped_labels"] == ["floor"]
    assert result["fit"]["offsets_mm"]["floor"] == pytest.approx(truth.floor, abs=0.05)


def test_width_fallback_keeps_measurements_and_held_out_disagreement(setup):
    truth = _truth(setup["nominal"], yaw=2.0)
    touches = [t for t in _touches(truth) if t["label"] != "left"]
    result = fitting.robust_fit(touches, setup["tank"], truth)
    assert result["fit"]["valid"]
    assert result["fit"]["left_derived"]["inner_width_mm"] == 434
    assert result["kept"] == touches
    assert result["fit"]["offsets_mm"]["left"] == pytest.approx(truth.left, abs=0.01)
    assert result["fit"]["yaw_deg"] == pytest.approx(truth.yaw_deg, abs=0.01)
    check = next(t for t in touches if t["label"] == "check:left")
    check["point_mm"][1] += 15
    bad = fitting.robust_fit(touches, setup["tank"], truth)
    assert not bad["fit"]["valid"]
    assert check in bad["kept"]


def test_candidate_files_are_pending_and_touch_nothing(setup, tmp_path):
    truth = _truth(setup["nominal"], yaw=0.3)
    before = {p: p.read_bytes() for p in (geometry.geo.DEFAULT_TANK, probe_planner.tank_guard.DEFAULT_CONFIG)}
    result = fitting.robust_fit(_touches(truth), setup["tank"], setup["nominal"])
    tank_file, keepout_file = fitting.candidate_files(
        result["fit"], result["kept"], setup["tank"], setup["keepout"], "test"
    )
    assert tank_file["approval"]["status"] == fitting.PENDING
    assert keepout_file["approval"]["status"] == fitting.PENDING
    assert keepout_file["planes_mm"]["left"]["status"] == "autocal_pending"
    assert keepout_file["planes_mm"]["near"]["value"] == pytest.approx(
        truth.near - geometry.POST_THICKNESS_MM, abs=0.1
    )
    # The proposal still loads as a keep-out (minus the approval note).
    probe_planner.tank_guard.TankGuard({k: v for k, v in keepout_file.items() if k != "approval"})
    json.dumps(tank_file)
    assert all(p.read_bytes() == b for p, b in before.items())


# --- sessions on the fake arm: a touch, and every abort path ends at rest, torque off -------------


def _session(
    setup, tmp_path, *, specs=(FLOOR,), fault=None, stop_at=None, second=False, truth=None, obstacles=()
):
    from context import core, real
    from fakes import PlaneFakeArm
    from runner import run_session

    model = setup["model"]
    rng = np.random.default_rng(11)
    truth = truth or _truth(setup["nominal"], yaw=0.5, shift=8.0, floor=-4.0)
    clock = real.VirtualClock()
    arm = PlaneFakeArm(
        setup["rest"],
        clock,
        model.guard.model,
        truth,
        {m: probe_planner.PROBE_GAINS[m][0] for m in core.MOTORS},
        obstacles=obstacles,
        rng=rng,
    )
    stop_event, abort_event = threading.Event(), threading.Event()
    holder = {"session": None, "holds": 0, "fired": False}

    def tick_hook(session, reading):
        holder["session"] = session
        if not holder["fired"] and stop_at and stop_at in session.step:
            holder["fired"] = True
            if fault:
                arm.fault, arm.fault_tick = fault, arm.ticks + 3
            else:
                stop_event.set()

    def read_hook():
        session = holder["session"]
        if session is None:
            return
        phase = session.status_phase or ""
        if second and stop_event.is_set() and phase.startswith("return leg"):
            abort_event.set()
        if phase.startswith(("stop: HOLDING", "stop: LIMP")):
            holder["holds"] += 1
            if holder["holds"] > 20:
                abort_event.set()

    arm.read_hook = read_hook
    out = tmp_path / "run"
    out.mkdir()
    planner = setup["planner"]
    result = run_session(
        arm,
        model,
        planner,
        out,
        (stop_event, abort_event),
        setup["tank"],
        setup["keepout"],
        out / "pid",
        clock,
        clock.sleep,
        specs=list(specs),
        spares=[],
        log=lambda *_: None,
        tick_hook=tick_hook,
    )
    return result, arm, truth


def test_fake_floor_touch_lands_on_the_true_floor(setup, tmp_path):
    result, arm, truth = _session(setup, tmp_path)
    touches = result["session"]["touches"]
    assert len(touches) == 1, result["session"]["rejected"]
    assert -truth.signed_depth("floor", np.asarray(touches[0]["point_mm"])) == pytest.approx(0.0, abs=0.5)
    assert result["stop"]["returned"].startswith("at the rest") and not arm.energized
    metrics = result["session"]["metrics"]
    assert metrics["max_cmd_joint_deg_s"] <= 15.0 + 1e-6
    assert metrics["max_cmd_tip_mm_s_in_approach"] <= probe_planner.APPROACH_MM_S + 1e-9
    assert max(f for _, f, _ in arm.contact_log) < probe_planner.F_MAX_N + 1.0
    assert max(v for _, _, v in arm.first_contacts) < 4.0  # mm/s at first contact


def test_red_button_mid_press_returns_and_releases(setup, tmp_path):
    result, arm, _ = _session(setup, tmp_path, stop_at="press 2")
    assert "red button" in result["reason"]
    assert result["stop"]["returned"].startswith("at the rest"), result["stop"]
    assert result["stop"]["release_problems"] is None and not arm.energized
    assert not result["session"]["touches"]


def test_second_signal_aborts_the_return_and_releases(setup, tmp_path):
    result, arm, _ = _session(setup, tmp_path, stop_at="press 1", second=True)
    assert "second signal" in result["stop"]["returned"]
    assert not arm.energized


@pytest.mark.parametrize("fault", ["stale", "limp"])
def test_a_motor_fault_mid_press_holds_then_releases_on_the_second_press(setup, tmp_path, fault):
    result, arm, _ = _session(setup, tmp_path, stop_at="press 2", fault=fault)
    stop = result["stop"]
    assert "held" in stop["returned"] and "second signal" in stop["returned"], stop
    assert not arm.energized


def test_no_glass_within_the_window_abandons_the_probe(setup, tmp_path):
    truth = _truth(setup["nominal"], yaw=0.0, shift=0.0, floor=-60.0)  # 60 mm deeper than nominal
    result, arm, _ = _session(setup, tmp_path, truth=truth)
    probe = result["session"]["probes"][0]
    assert probe["outcome"] == "abandoned" and "no contact within the window" in probe["reason"]
    assert max((f for _, f, _ in arm.contact_log), default=0.0) == 0.0
    assert result["stop"]["returned"].startswith("at the rest") and not arm.energized


def test_a_soft_obstacle_is_rejected_not_fitted(setup, tmp_path):
    from fakes import Obstacle

    truth = _truth(setup["nominal"], yaw=0.0, shift=0.0, floor=0.0)
    cable = Obstacle(
        "floor", tuple(truth.surface_point(FLOOR.face, FLOOR.u, FLOOR.v)), 40.0, 15.0, 300.0, "cable"
    )
    result, arm, _ = _session(setup, tmp_path, truth=truth, obstacles=[cable])
    probe = result["session"]["probes"][0]
    assert probe["outcome"] == "rejected", probe.get("verdict")
    assert not result["session"]["touches"]


def test_the_autocal_is_not_imported_by_the_runner_harness_or_guard():
    roots = [
        Path(__file__).resolve().parents[1] / p
        for p in ("makermodslab", "scripts/viscous-astra", "scripts/viscous-dream1/real")
    ]
    for root in roots:
        for path in root.rglob("*.py"):
            if path.name == "stop_button.py":
                continue  # it only matches the run's command line, so the red button can signal it
            assert "viscous-tank-autocal" not in path.read_text(errors="ignore"), path


def test_stop_button_recognises_the_autocal_run():
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts/viscous-dream1/real"))
    import stop_button

    assert stop_button.is_runner_command(
        "/usr/bin/python3 scripts/viscous-tank-autocal/autocal.py run --armed"
    )
    assert not stop_button.is_runner_command("python autocal.py sim")
    assert stop_button.is_runner_command("python3 scripts/viscous-astra/astra_op.py serve --backend lab")
