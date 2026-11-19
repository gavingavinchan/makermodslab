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
"""Tank autocal: find the aquarium's floor and long walls by pressing the closed jaw on them.

    autocal.py plan                     every probe pose and its guard/force verdict, no arm
    autocal.py sim  --backend mujoco    randomised tanks in MuJoCo (~/dreamtrial/.venv only)
    autocal.py run  [--armed]           the real arm on Gavin's Mac; unarmed by default

A standalone, opt-in convenience tool (Gavin, 2026-11-18: "not necessary once we know the exact tank
size from a measuring tape"). It is never part of the runner, the harness or the guard path. Its
output is a CANDIDATE tank geometry and keep-out, pending Gavin's approval, like the hand-guided
tank measurement's; it never writes the current tank files.
"""

from __future__ import annotations

import argparse
import contextlib
import dataclasses
import json
import math
import os
import sys
import tempfile
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import context  # noqa: E402,F401  (import paths)
import planner as probe_planner  # noqa: E402
from context import core, real  # noqa: E402
from geometry import ARM, back_touch, load_inputs, nominal_estimate  # noqa: E402

RUN_ROOT = Path("~/.makermods/makermodslab-viscous/run-data").expanduser()


# --- plan --------------------------------------------------------------------------------------


def cmd_plan(args) -> int:
    folder = Path(tempfile.mkdtemp(prefix="autocal-plan-"))
    model = context.offline_model(folder)
    tank, keepout = load_inputs()
    rest = context.arm_file_rest(model)
    planner = probe_planner.Planner(model, keepout, model.limits, rest)
    est = nominal_estimate(tank, keepout)
    result = probe_planner.plan_all(
        planner, est, probe_planner.PROBES + (probe_planner.SPARES if args.spares else ())
    )
    print(
        f"nominal planes (tank axes, mm): {json.dumps({k: round(v, 1) for k, v in est.offsets().items()})}, "
        f"yaw {est.yaw_deg:g} deg; back glass from the post (never probed): "
        f"{json.dumps(back_touch(est)['point_mm'])}"
    )
    print(f"hub = arm-file rest {_fmt(result['hub'])}  keep-out: {result['hub_guard']['ok']}")
    print(f"probe gains (kp, kd): {probe_planner.PROBE_GAINS}")
    ok = 0
    for i, plan in enumerate(result["plans"], 1):
        s = plan.spec
        head = f"{i:2d} {s.label:12s} {'spare ' if s.spare else ''}u={s.u:5.0f} v={s.v:5.0f}"
        if not plan.ok:
            print(f"{head}  REFUSED: {'; '.join(plan.reasons[:3])}")
            continue
        ok += 1
        print(
            f"{head}  OK tilt {plan.tilt_deg:4.1f} deg, standoff {plan.standoff_mm:.0f} mm, window {plan.window_mm:.0f} mm,"
            f" press {plan.d1_mm:.1f}/{plan.d2_mm:.1f} mm, stiffness {plan.stiffness_n_per_m:.0f} N/m -> "
            f"{plan.press_force_n:.1f} N (static torque trip {plan.backstop_force_n:.1f}–{plan.backstop_force_max_n:.1f} N), "
            f"lead {plan.lead_mm:.0f} mm "
            f"({plan.lead_link}), other face {plan.other_face_mm:.0f} mm, guard slack {plan.guard_slack_mm:.1f} mm"
        )
        print(f"      lift     {_fmt(plan.lift)}")
        for depth, node in zip(plan.node_s_mm, plan.nodes, strict=True):
            print(f"      s={depth:6.1f} mm {_fmt(node)}  OK")
        print(f"      deepest  {_fmt(plan.nodes[-1])}  (s = {plan.node_s_mm[-1]:.0f} mm)")
    print(
        f"{ok}/{len(result['plans'])} probes plannable assuming preceding feasible touches find the nominal panes"
    )
    if args.out:
        probe_planner.write_plan(Path(args.out), result)
        print(f"plan written to {args.out}")
    return 0 if ok else 1


def _fmt(q):
    return "{" + ", ".join(f"{m}: {q[m]:.1f}" for m in (*ARM, "gripper") if m in q) + "}"


# --- sim ---------------------------------------------------------------------------------------

SCENARIOS = {
    # name: (description, kwargs for simulate)
    "nominal": "random displacements about the CAD mount, nothing in the way",
    "centred": "random displacements about a physically centred mount, using the same CAD planner prior",
    "finite_panes": "finite glass boxes in the physical collision solver, with identical inner faces",
    "cable": "a soft cable bundle (300 N/m) under the first floor probe",
    "post": "a floor request at the mount post",
    "wrist_post": "a raised rigid pin under the wrist during a floor press with softened wrist gains",
    "rock": "a localized rigid 24 mm-tall object under the first floor probe",
    "rock_wide": "a rigid object extending under several floor probes",
    "floor_trap": "a wall target 2 mm above the estimated floor, with the true floor raised 22 mm",
    "buckle": "the wrist joints go soft (kp 3) during the presses: the wrist folds",
    "model_error": "0.5 deg encoder zero errors on lift, elbow and wrist_flex (a URDF/calibration error)",
    "no_flex": "random tanks with no structural flex or backlash (the method's own accuracy)",
    "flex_severe": "structural stiffness 600 / 300 Nm/rad and 0.1 deg backlash, unseen by the encoders",
    "red_button": "the stop signal arrives mid-press",
    "second_signal": "the stop signal mid-press, then a second one during the return",
    "stale": "a motor stops answering mid-press",
    "limp": "shoulder_lift cuts out mid-press",
    "missing_wall": "the right wall is 120 mm beyond its nominal location",
}


def simulate(cfg: dict) -> dict:
    """One session against one true tank. cfg: seed, backend, scenario, out, video."""
    import numpy as np
    from fakes import PlaneFakeArm, random_truth
    from runner import run_session

    seed, backend, scenario = cfg["seed"], cfg["backend"], cfg.get("scenario", "nominal")
    out = Path(cfg["out"]) / f"{scenario}-{backend}-{seed:03d}"
    out.mkdir(parents=True, exist_ok=False)
    rng = np.random.default_rng(seed)
    tank, keepout = load_inputs()
    model = context.offline_model(out / "model")
    rest = context.arm_file_rest(model)
    planner = probe_planner.Planner(model, keepout, model.limits, rest, bank=_bank(model))
    nominal = nominal_estimate(tank, keepout)
    simulated_mount = nominal.copy()
    if scenario == "centred":
        half_width = nominal.dims["inner_width"] / 2
        simulated_mount.left, simulated_mount.right = half_width, -half_width
    truth = random_truth(simulated_mount, rng)
    specs, spares = list(probe_planner.PROBES), list(probe_planner.SPARES)
    if scenario == "floor_trap":
        truth.floor = nominal.floor + 22.0
        specs = [s for s in specs if s.face == "floor"][:1] + [
            probe_planner.ProbeSpec("right", "right", -470, 2)
        ]
        spares = []
    if scenario == "wrist_post":
        specs, spares = [probe_planner.PROBES[0]], []
    if scenario == "post":
        specs, spares = [probe_planner.ProbeSpec("floor", "floor", 106, 10)], []
    if scenario == "missing_wall":
        truth.right = nominal.right - 120.0
        specs, spares = [next(s for s in specs if s.face == "right")], []
    kp_kd = probe_planner.PROBE_GAINS
    stop_event, abort_event = threading.Event(), threading.Event()
    log_lines = []

    def log(line):
        log_lines.append(line)
        if cfg.get("verbose"):
            print(f"[{scenario}-{seed}] {line}", flush=True)

    fault = {"stale": "stale", "limp": "limp"}.get(scenario)
    arm_kw = {}
    obstacles = []
    if backend == "mujoco":
        import sim

        physics = sim.Physics(finite_panes=scenario == "finite_panes")
        if scenario == "model_error":
            physics.encoder_bias_deg = {"shoulder_lift": 0.5, "elbow_flex": -0.5, "wrist_flex": 0.5}
        if scenario == "no_flex":
            physics.flex_nm_per_rad = {}
            physics.backlash_deg = 0.0
        if scenario == "flex_severe":
            physics.flex_nm_per_rad = {
                m: (600.0 if m in ("shoulder_pan", "shoulder_lift", "elbow_flex") else 300.0) for m in ARM
            }
            physics.backlash_deg = 0.1
        recorder = sim.Recorder(cfg["video"]) if cfg.get("video") else None
        arm = None
    pending = {"armed_fault": False}

    holder = {"session": None, "hold_reads": 0}

    def tick_hook(session, reading):
        holder["session"] = session
        step = session.step or ""
        if backend == "mujoco":
            session.arm.phase = step
            session.arm.probe_number = len(session.probes)
        if (
            scenario in ("red_button", "second_signal", "stale", "limp")
            and not pending["armed_fault"]
            and "press 2" in step
        ):
            pending["armed_fault"] = True
            if scenario in ("red_button", "second_signal"):
                stop_event.set()
            else:
                session.arm.fault, session.arm.fault_tick = fault, session.arm.ticks + 5
        if scenario in ("buckle", "wrist_post") and backend == "mujoco":
            soft = "press" in step
            session.arm.kp_override = (
                dict.fromkeys(("wrist_flex", "wrist_yaw", "wrist_roll"), 3.0) if soft else {}
            )

    def read_hook():
        """Gavin's part: on the second signal scenario, a second press during the return; on a HOLD,
        he supports the arm and presses RELEASE (after 3 s of holding)."""
        session = holder["session"]
        if session is None:
            return
        phase = session.status_phase or ""
        if scenario == "second_signal" and stop_event.is_set() and phase.startswith("return leg"):
            abort_event.set()
        if phase.startswith("stop: HOLDING") or phase.startswith("stop: LIMP"):
            holder["hold_reads"] += 1
            if holder["hold_reads"] > 30:
                abort_event.set()

    if scenario == "wrist_post":
        first = probe_planner.PROBES[0]
        centre = (first.u + 35.0, first.v - 75.0, truth.floor)
        obstacles.append(("post_pin", "floor", centre, 8.0, 80.0, None))
    if scenario in ("cable", "rock", "rock_wide"):
        first = probe_planner.PROBES[0]
        centre = truth.surface_point(first.face, first.u, first.v)
        if scenario == "cable":
            obstacles.append(("cable", first.face, centre, 38.0, 20.0, 300.0))
        else:
            obstacles.append(
                ("rock", first.face, centre, 38.0 if scenario == "rock_wide" else 12.0, 24.0, None)
            )
    if backend == "mujoco":
        import sim

        obs = [sim.SimObstacle(face, tuple(c), r, h, k, name) for name, face, c, r, h, k in obstacles]
        arm = sim.MujocoArm(rest, truth, model.guard.model, kp_kd, physics, obs, seed=seed, recorder=recorder)
        clock = arm.clock
    else:
        from fakes import Obstacle

        clock = real.VirtualClock()
        obs = [
            Obstacle(face, tuple(c), r, h, math.inf if k is None else k, name)
            for name, face, c, r, h, k in obstacles
        ]
        arm = PlaneFakeArm(
            rest,
            clock,
            model.guard.model,
            truth,
            {m: kp_kd[m][0] for m in core.MOTORS},
            obstacles=obs,
            rng=rng,
            **arm_kw,
        )
    arm.read_hook = read_hook
    t0 = time.time()
    result = run_session(
        arm,
        model,
        planner,
        out,
        (stop_event, abort_event),
        tank,
        keepout,
        out / "autocal.pid",
        clock,
        clock.sleep,
        specs=specs,
        spares=spares,
        log=log,
        tick_hook=tick_hook,
    )
    wall = time.time() - t0
    report = evaluate(result, truth, nominal, arm, backend)
    report.update(
        seed=seed,
        backend=backend,
        scenario=scenario,
        wall_s=wall,
        log=log_lines,
        truth={"yaw_deg": truth.yaw_deg, **truth.offsets()},
        folder=str(out),
    )
    if backend == "mujoco" and cfg.get("video") and recorder is not None and recorder.writer is not None:
        recorder.close()
        report["video"] = cfg["video"]
    (out / "report.json").write_text(json.dumps(report, indent=2, default=_jsonable) + "\n")
    return report


_BANKS = {}


def _bank(model):
    if "bank" not in _BANKS:
        _BANKS["bank"] = probe_planner.SeedBank(model.guard.model, model.limits)
    return _BANKS["bank"]


def plane_y(offset, yaw_deg, x):
    """Base-frame y of a long wall (tank-y = offset) at base x."""
    t = math.radians(yaw_deg)
    return (offset + math.sin(t) * x) / math.cos(t)


def evaluate(result, truth, nominal, arm, backend) -> dict:
    import numpy as np

    session = result["session"]
    fit = (result.get("fit") or {}).get("fit")
    out = {
        "reason": result["reason"],
        "touches": len(session["touches"]),
        "fit_failures": (fit or {}).get("failures", []),
        "rejected": [r["reasons"] for r in session["rejected"]],
        "abandoned": [p.get("reason") for p in session["probes"] if p.get("outcome") == "abandoned"],
        "unplannable": [p["spec"]["label"] for p in session["probes"] if p.get("outcome") == "unplannable"],
        "dropped": [
            (d["label"], d.get("dropped_residual_mm")) for d in (result.get("fit") or {}).get("dropped", [])
        ],
        "metrics": session["metrics"],
        "stop": {k: result["stop"].get(k) for k in ("reason", "returned", "release_problems", "legs")},
        "released": not getattr(arm, "energized", True),
    }
    # Touch errors: each accepted touch's distance from the TRUE plane (positive = short of it).
    errs = []
    for t in session["touches"]:
        face = t["label"].removeprefix("check:")
        errs.append((t["label"], -truth.signed_depth(face, np.asarray(t["point_mm"]))))
    out["touch_errors_mm"] = errs
    if fit is not None and "offsets_mm" in fit:
        o = fit["offsets_mm"]
        xs = np.linspace(-600, 0, 13)
        wall_err = {
            f: float(
                max(
                    abs(plane_y(o[f], fit["yaw_deg"], x) - plane_y(getattr(truth, f), truth.yaw_deg, x))
                    for x in xs
                )
            )
            for f in ("left", "right")
        }
        out["fit"] = {
            "valid": fit["valid"],
            "left_derived": fit.get("left_derived"),
            "failures": fit["failures"],
            "yaw_error_deg": fit["yaw_deg"] - truth.yaw_deg,
            "floor_error_mm": o["floor"] - truth.floor,
            "left_offset_error_mm": o["left"] - truth.left,
            "right_offset_error_mm": o["right"] - truth.right,
            "left_error_mm_max_over_reach": wall_err["left"],
            "right_error_mm_max_over_reach": wall_err["right"],
            "centre_error_mm": -(o["left"] + o["right"]) / 2 + (truth.left + truth.right) / 2,
            "rms_mm": fit.get("rms_mm"),
            "max_residual_mm": fit.get("max_mm"),
        }
    contacts = {}
    if backend == "mujoco":
        forces = (
            np.array([f for _, f, _, _ in arm.contact_peak if f > 0]) if arm.contact_peak else np.zeros(0)
        )
        contacts = {
            "peak_force_n": arm.peak_force_20ms_n,
            "per_probe": arm.probe_contacts,
            "peak_details": arm.peak_details,
            "approach_first_contact_speeds_mm_s": arm.approach_contacts,
            "peak_link_speed_deg_s": arm.peak_link_speed_deg_s,
            "peak_force_instant_n": arm.peak_force_n,
            "tick_force_p99_n": float(np.percentile(forces, 99)) if forces.size else 0.0,
            "max_first_contact_speed_mm_s": max((v for _, _, v in arm.first_contacts), default=0.0),
            "first_contacts": len(arm.first_contacts),
            "nonfinger_contact_ticks": sum(1 for _, f, _, nf in arm.contact_peak if nf and f > 0),
            "surfaces_touched": sorted({s for _, f, ss, _ in arm.contact_peak if f > 0 for s in ss}),
        }
    else:
        contacts = {
            "peak_force_n": max((f for _, f, _ in arm.contact_log), default=0.0),
            "max_first_contact_speed_mm_s": max((v for _, _, v in arm.first_contacts), default=0.0),
        }
    out["contacts"] = contacts
    return out


def _jsonable(v):
    import numpy as np

    if isinstance(v, np.ndarray):
        return v.tolist()
    if isinstance(v, (np.floating, np.integer)):
        return v.item()
    if isinstance(v, set):
        return sorted(v)
    if dataclasses.is_dataclass(v):
        return dataclasses.asdict(v)
    return str(v)


def cmd_sim(args) -> int:
    out = Path(args.out or RUN_ROOT / time.strftime("autocal-sim-%Y%m%dT%H%M%SZ", time.gmtime())).expanduser()
    if min(args.tanks, args.repeat, args.workers) < 1:
        raise SystemExit("tanks, repeat and workers must be positive")
    if out.exists() and any(out.iterdir()):
        raise SystemExit(f"choose an empty output directory: {out}")
    if args.video and Path(args.video).expanduser().exists():
        raise SystemExit("choose a new video filename")
    out.mkdir(parents=True, exist_ok=True)
    configs = []
    for scenario in args.scenario:
        seeds = range(args.seed, args.seed + (args.tanks if scenario == "nominal" else args.repeat))
        for seed in seeds:
            configs.append(
                {
                    "seed": seed,
                    "backend": args.backend,
                    "scenario": scenario,
                    "out": str(out),
                    "verbose": args.verbose,
                }
            )
    if args.video:
        configs[0]["video"] = str(Path(args.video).expanduser())
    if args.workers > 1:
        import multiprocessing as mp

        with mp.get_context("spawn").Pool(args.workers) as pool:
            reports = pool.map(_simulate_safe, configs)
    else:
        reports = [_simulate_safe(c) for c in configs]
    summary = summarise(reports)
    (out / "summary.json").write_text(
        json.dumps({"summary": summary, "runs": reports}, indent=2, default=_jsonable) + "\n"
    )
    print(json.dumps(summary, indent=2, default=_jsonable))
    print(f"results in {out}")
    return int(any("crash" in r for r in reports))


def _simulate_safe(cfg):
    import traceback

    try:
        return simulate(cfg)
    except Exception as error:  # noqa: BLE001 - one broken run must not hide the others
        return {
            "seed": cfg["seed"],
            "scenario": cfg.get("scenario"),
            "crash": f"{error!r}",
            "traceback": traceback.format_exc(),
        }


def error_stat(values):
    import numpy as np

    v = np.abs(values)
    return {"mean": float(v.mean()), "p95": float(np.percentile(v, 95)), "max": float(v.max())}


def summarise(reports) -> dict:
    import numpy as np

    by = {}
    for r in reports:
        by.setdefault(r.get("scenario"), []).append(r)
    out = {}
    for scenario, runs in by.items():
        fits = [r["fit"] for r in runs if r.get("fit")]
        valid = [f for f in fits if f["valid"]]
        entry = {
            "runs": len(runs),
            "crashed": [r.get("crash") for r in runs if r.get("crash")],
            "valid_fits": len(valid),
            "invalid_fit_failures": [r["fit_failures"] for r in runs if r.get("fit_failures")],
            "ended_released": sum(1 for r in runs if r.get("released")),
            "returned": sorted({str(r.get("stop", {}).get("returned")) for r in runs}),
            "reasons": sorted({str(r.get("reason")) for r in runs}),
            "rejected": sum(len(r.get("rejected", [])) for r in runs),
            "rejection_reasons": [x for r in runs for x in r.get("rejected", [])][:6],
            "abandoned": [x for r in runs for x in r.get("abandoned", [])][:6],
            "dropped": [x for r in runs for x in r.get("dropped", [])],
            "max_cmd_joint_deg_s": max(
                (r["metrics"]["max_cmd_joint_deg_s"] for r in runs if "metrics" in r), default=None
            ),
            "max_measured_joint_deg_s": max(
                (r["metrics"]["max_measured_joint_deg_s"] for r in runs if "metrics" in r), default=None
            ),
            "max_first_contact_speed_mm_s": max(
                (r["contacts"]["max_first_contact_speed_mm_s"] for r in runs if "contacts" in r), default=None
            ),
            "peak_force_n": max(
                (r["contacts"]["peak_force_n"] for r in runs if "contacts" in r), default=None
            ),
        }
        contacts = [r["contacts"] for r in runs if "contacts" in r]
        entry["derived_left_valid_fits"] = sum(bool(f.get("left_derived")) for f in valid)
        entry["peak_force_instant_n"] = max(
            (c["peak_force_instant_n"] for c in contacts if "peak_force_instant_n" in c), default=None
        )
        entry["peak_physics_joint_deg_s"] = max(
            (c["peak_link_speed_deg_s"] for c in contacts if "peak_link_speed_deg_s" in c), default=None
        )
        entry["max_first_approach_contact_mm_s"] = max(
            (v for c in contacts for v in c.get("approach_first_contact_speeds_mm_s", {}).values()),
            default=None,
        )
        if valid:
            for label, key in (
                ("yaw_error_deg", "yaw_error_deg"),
                ("floor_error_mm", "floor_error_mm"),
                ("left_offset_error_mm", "left_offset_error_mm"),
                ("right_offset_error_mm", "right_offset_error_mm"),
                ("left_error_mm", "left_error_mm_max_over_reach"),
                ("right_error_mm", "right_error_mm_max_over_reach"),
                ("centre_error_mm", "centre_error_mm"),
            ):
                entry[label] = error_stat([f[key] for f in valid])
            touch = np.abs([e for r in runs for _, e in r.get("touch_errors_mm", [])])
            if touch.size:
                entry["touch_error_mm"] = {"mean": float(touch.mean()), "max": float(touch.max())}
        out[scenario] = entry
    return out


# --- run (the real arm; Gavin's Mac only) ------------------------------------------------------


class ProbeLabArm(real.LabArm):
    """The runner's LabArm, connected with the probe gains instead of the runner's, no cameras."""

    def connect(self):
        saved = real.FOLLOWER_GAINS
        real.FOLLOWER_GAINS = dict(probe_planner.PROBE_GAINS)
        try:
            super().connect()
        finally:
            real.FOLLOWER_GAINS = saved


def cmd_run(args) -> int:
    import fcntl

    from runner import run_session
    from session import PassiveLabArm

    if sys.platform != "darwin":
        raise SystemExit("run drives the real arm and is for Gavin's Mac only (use sim here)")
    os.umask(0o077)
    home = real.lab_home(None)
    os.environ["MAKERMODSLAB_HOME"] = str(home)
    record = real.load_record(home, args.robot)
    limits, _ = real.follower_limits(home, record)
    station = Path("~/viscous-dream-real/station/arms/viscous-waterproof-tank.json").expanduser()
    arm_path = (args.arm_file or station).expanduser()
    pid = args.pid_file.expanduser().resolve()
    root = RUN_ROOT
    folder = root / time.strftime("autocal-%Y%m%dT%H%M%SZ", time.gmtime())
    folder.mkdir(parents=True)
    tank, keepout = load_inputs()
    with contextlib.ExitStack() as stack:
        lock = stack.enter_context(Path(str(pid) + ".lock").open("a"))
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if pid.exists() and real.run_data.pid_alive(int(pid.read_text().strip())):
            raise SystemExit(f"pid file already names a live process: {pid}")
        events = real.install_stop_signals(pid)
        stack.callback(real.remove_pid_file, pid)
        url = "http://127.0.0.1:8000"
        if holder := real.lab_holder(url, False) or real.port_holders(record["follower_port"]):
            raise SystemExit(f"Lab/port not idle: {holder}")
        real.release_lab_previews(url, False)
        model = context.CalibrationModel(arm_path, real.ENVIRONMENTS["tank"], folder, limits, True)
        if not args.armed:
            # Passive: bus open without the energizing handshake, read the start, plan, close.
            arm = PassiveLabArm.__new__(PassiveLabArm)
            real.LabArm.__init__(arm, home, record, 15, cameras=False)
            arm.grab = lambda: {}
            arm.close_cameras = lambda: None
            from lerobot.robots import make_robot_from_config
            from makermodslab.arms import registry
            from makermodslab.tank_measure import full_turn_offsets

            family = registry.get("viscous")
            arm.robot = make_robot_from_config(
                family.single_follower_config(record["follower_port"], record["follower_config"])
            )
            arm.robot.bus.connect(handshake=False)
            try:
                raw = family._read_fresh_positions(arm.robot)
                arm.robot._turn_offset = full_turn_offsets(raw, arm.robot.config)
                reading = real.LabArm.read(arm)
            finally:
                arm.robot.bus.disconnect(False)
            start = reading.pos
            gate = model.guard.start_gate(start)
            print(f"UNARMED: measured start {_fmt(start)}; start gate {'OK' if gate.ok else gate}")
            if not gate.ok:
                raise SystemExit("start gate refused the measured pose")
            planner = probe_planner.Planner(model, keepout, model.limits, context.arm_file_rest(model))
            est = nominal_estimate(tank, keepout)
            hub = planner.hub(est)
            model.plan(start, [hub], start)
            model.plan(hub, [start], start)
            for plan in probe_planner.plan_all(planner, est)["plans"]:
                spec = plan.spec
                print(
                    f"  {spec.label:12s} u={spec.u:5.0f} v={spec.v:4.0f} "
                    + (
                        f"OK tilt {plan.tilt_deg:.0f}, press {plan.press_force_n:.1f} N"
                        if plan.ok
                        else f"REFUSED {plan.reasons[:2]}"
                    )
                )
            print("No torque was enabled. Re-run with --armed after Gavin's go.")
            return 0
        rest_guess = context.arm_file_rest(model)
        planner = probe_planner.Planner(model, keepout, model.limits, rest_guess)
        arm = ProbeLabArm(home, record, 15, cameras=False)
        result = run_session(
            arm, model, planner, folder, events, tank, keepout, pid, time.monotonic, time.sleep
        )
        print(
            json.dumps(
                {k: result.get(k) for k in ("reason", "placement", "candidate_files")},
                indent=2,
                default=_jsonable,
            )
        )
        print(
            f"stop: {result['stop'].get('returned')}; release problems: {result['stop'].get('release_problems')}"
        )
        print(f"run folder: {folder}")
        return int(not result.get("candidate_files") or bool(result["stop"].get("release_problems")))


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="mode", required=True)
    a = sub.add_parser("plan", help="print every probe pose and its verdict; no arm")
    a.add_argument("--spares", action="store_true", help="also plan the spare probe locations")
    a.add_argument("--out", help="write the plan JSON here")
    s = sub.add_parser("sim", help="randomised tanks (MuJoCo needs ~/dreamtrial/.venv)")
    s.add_argument("--backend", choices=("fake", "mujoco"), default="mujoco")
    s.add_argument("--scenario", nargs="+", default=["nominal"], choices=sorted(SCENARIOS))
    s.add_argument("--tanks", type=int, default=20, help="random tanks for the nominal scenario")
    s.add_argument("--repeat", type=int, default=2, help="random tanks for each other scenario")
    s.add_argument("--seed", type=int, default=1)
    s.add_argument("--workers", type=int, default=1)
    s.add_argument("--video", help="write an MP4 of the first run here (mujoco)")
    s.add_argument("--out")
    s.add_argument("--verbose", action="store_true")
    r = sub.add_parser("run", help="the real arm (Gavin's Mac, his explicit go); unarmed by default")
    r.add_argument(
        "--armed", action="store_true", help="enable torque and probe; otherwise read and plan only"
    )
    r.add_argument("--robot", default="viscous_waterproof_01")
    r.add_argument("--arm-file", type=Path)
    r.add_argument("--pid-file", type=Path, default=Path("/tmp/viscous-autocal.pid"))
    args = p.parse_args(argv)
    return {"plan": cmd_plan, "sim": cmd_sim, "run": cmd_run}[args.mode](args)


if __name__ == "__main__":
    sys.exit(main())
