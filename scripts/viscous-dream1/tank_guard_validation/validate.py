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
"""Offline validation only. No simulator, client, robot, or credential imports.

Run from the repository with the commissioned shared Python and PYTHONPATH=.
Stages are independently reproducible; all seeds, sources and hashes are saved.
Independent verification uses geo.forward_kinematics and ALL collision vertices.
It never uses TankGuard.clearances/check_pose/check_segment for its verdicts.
"""

from __future__ import annotations

import argparse
import copy
import cProfile
import hashlib
import io
import json
import math
import os
import pstats
import sys
import time
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
from pathlib import Path

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")
import numpy as np

from makermodslab import tank_geometry as geo, tank_guard as tg

ROOT = geo.ROOT
OUT = ROOT / "scripts/viscous-dream1/runs/tank_guard_validation"
RUNS = Path.home() / "dreamtrial/viscous_sim/runs"
REST = dict(zip(tg.MOTORS, (189.0, -201.8, 169.6, 47.3, 31.3, -7.2, -0.5), strict=True))
WORK = dict(zip(tg.MOTORS, (182.0, -150.0, 120.0, 30.0, 0.0, 0.0, -10.0), strict=True))
SEED = 20261107
G = None


def init_worker():
    global G
    G = tg.TankGuard.from_file()


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(name, data):
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / name).write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")


def lerp(a, b, f):
    return {m: float(a[m] + (b[m] - a[m]) * f) for m in tg.MOTORS}


def finite_verdict(v):
    result = asdict(v)
    if not math.isfinite(result["slack_mm"]):
        result["slack_mm"] = str(result["slack_mm"])
    return result


def percentiles(values):
    a = np.asarray(values)
    return (
        {
            "mean": float(a.mean()),
            "median": float(np.median(a)),
            "p99": float(np.percentile(a, 99)),
            "max": float(a.max()),
        }
        if len(a)
        else None
    )


def plotting():
    # Optional offline wheel-cache install. Append so the commissioned numpy wins.
    if Path("/tmp/tankbox/plot-deps").is_dir():
        sys.path.append("/tmp/tankbox/plot-deps")
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    return plt


def points(model, q):
    """World vertices in mm, including fixed base and exempt yaw for raw plots."""
    transforms = geo.forward_kinematics(model, q)
    return {
        link: (v @ transforms[link][0].T + transforms[link][1]) * 1000 for link, v in model.vertices.items()
    }


def independent_setup(config, model):
    """Interpret original config independently, without guard-derived requirements."""

    def val(x):
        return x["value"] if isinstance(x, dict) and "value" in x else x

    planes = {k: float(val(v)) for k, v in config["planes_mm"].items()}
    yaw = math.radians(float(val(config["yaw_deg"])))
    c, s = math.cos(yaw), math.sin(yaw)
    axes = np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])
    requirements = {}
    for link in sorted(model.moving_links & model.vertices.keys()):
        rule = config.get("link_rules", {}).get(link, {})
        pad = val(config["padding_mm"]).get(link, val(config["padding_mm"])["default"])
        req = {}
        for face in tg.FACES:
            if face in rule.get("exempt_faces", []):
                continue
            if face == "floor":
                req[face] = -float(val(config["floor_press_mm"]))
            else:
                over = rule.get("faces", {}).get(face, {})
                req[face] = float(over.get("margin", val(config["margins_mm"])[face])) + float(
                    over.get("padding", pad)
                )
        requirements[link] = req
    return planes, axes, requirements


def independent_pose(model, q, setup, return_points=False):
    planes, axes, requirements = setup
    transformed = points(model, q)  # transform EVERY vertex, no hull or subset
    worst = (math.inf, None, None)
    for link, req in requirements.items():
        p = transformed[link] @ axes.T
        lo, hi = p.min(axis=0), p.max(axis=0)
        distances = {
            "near": planes["near"] - hi[0],
            "far": lo[0] - planes["far"],
            "left": planes["left"] - hi[1],
            "right": lo[1] - planes["right"],
            "floor": lo[2] - planes["floor"],
            "rim": planes["rim"] - hi[2],
        }
        for face, required in req.items():
            slack = float(distances[face] - required)
            if slack < worst[0]:
                worst = (slack, link, face)
    return (worst, transformed) if return_points else worst


def verify_segment(q0, q1, setup):
    n = max(1, math.ceil(max(abs(q1[m] - q0[m]) for m in tg.MOTORS) / 0.05))
    worst = (math.inf, None, None)
    violation = None
    for k in range(n + 1):
        q = lerp(q0, q1, k / n)
        v = independent_pose(G.model, q, setup)
        if v[0] < worst[0]:
            worst = v
        if v[0] < -1e-7 and violation is None:
            violation = {"fraction": k / n, "pose": q, "slack_mm": v[0], "link": v[1], "face": v[2]}
    return n + 1, worst, violation


def raw_variant(config):
    c = copy.deepcopy(config)
    c["margins_mm"] = {"value": dict.fromkeys(tg.WALL_FACES, 0.0)}
    c["padding_mm"] = {"value": {"default": 0.0, "gripper_left": 0.0, "gripper_right": 0.0}}
    return c


def intervals(mask, times):
    out = []
    start = None
    for i, yes in enumerate([*mask, False]):
        if yes and start is None:
            start = i
        elif not yes and start is not None:
            out.append(
                {
                    "first": start,
                    "last": i - 1,
                    "start_s": float(times[start]),
                    "end_s": float(times[i - 1]),
                    "readings": i - start,
                }
            )
            start = None
    return out


def trace_stage(path):
    g = tg.TankGuard.from_file()
    raw = tg.TankGuard(raw_variant(g.config), g.model)
    rows = [json.loads(s) for s in Path(path).read_text().splitlines() if '"event"' not in s]
    times = [r["t"] for r in rows]
    all_slacks, counts, verdicts = {}, {}, {}
    for label, guard in [("shipped", g), ("raw", raw)]:
        cs = [guard.clearances(r["positions_deg"]) for r in rows]
        vv = [guard.check_pose(r["positions_deg"]) for r in rows]
        all_slacks[label] = cs
        counts[label] = {"ok": sum(v.ok for v in vv), "outside": sum(not v.ok for v in vv)}
        verdicts[label] = {
            f"{link}/{face}": {
                "violating_readings": sum(v[(link, face)] < 0 for v in cs),
                "worst_slack_mm": min(v[(link, face)] for v in cs),
            }
            for link, face in cs[0]
        }
    ext = []
    for r in rows:
        p = np.concatenate(list(points(g.model, r["positions_deg"]).values()))
        ext.append([p[:, 1].min(), p[:, 1].max(), p[:, 2].min()])
    ext = np.array(ext)
    crossings = {}
    for face in tg.FACES:
        per = np.array([min(v for (link, f), v in cs.items() if f == face) for cs in all_slacks["raw"]])
        if face == "floor":
            per -= 10.0  # distinguish literal floor plane from allowed floor press
        crossings[face] = {
            "worst_raw_slack_mm": float(per.min()),
            "outside_readings": int((per < 0).sum()),
            "ranges": intervals(per < 0, times),
        }
    dump(
        "trace_summary.json",
        {
            "source": str(path),
            "sha256": digest(path),
            "readings": len(rows),
            "time_start_s": times[0],
            "time_end_s": times[-1],
            "counts": counts,
            "per_link_face": verdicts,
            "raw_plane_crossings": crossings,
            "raw_variant": "zero margins and padding; floor press 10 mm and yaw exemption retained; raw_plane_crossings removes the floor press allowance",
            "extrema_mm": {
                "y_min": float(ext[:, 0].min()),
                "y_max": float(ext[:, 1].max()),
                "z_min": float(ext[:, 2].min()),
            },
        },
    )
    dump("trace_series.json", {"t_s": times, "whole_mesh_y_min_y_max_z_min_mm": ext.round(5).tolist()})
    plt = plotting()
    fig, axs = plt.subplots(2, 1, figsize=(12, 7), sharex=True, layout="constrained")
    ax = axs[0]
    ax.plot(times, ext[:, 0], label="whole mesh y minimum", lw=1)
    ax.plot(times, ext[:, 1], label="whole mesh y maximum", lw=1)
    for face, color, direction in [("left", "tab:red", -1), ("right", "tab:purple", 1)]:
        p = g.planes[face]
        for offset, style, label in [
            (0, "-", f"{face} glass {p:g} mm"),
            (30, "--", f"{face} finger margin"),
            (70, ":", f"{face} cable envelope"),
        ]:
            ax.axhline(p + direction * offset, color=color, ls=style, alpha=0.7, label=label)
    ax.set_ylabel("y (mm)")
    ax.legend(ncol=2, fontsize=8)
    ax.grid(alpha=0.2)
    ax = axs[1]
    ax.plot(times, ext[:, 2], lw=1, label="whole mesh z minimum")
    for z, style, label in [
        (-6, "-", "floor bound -6 mm"),
        (-16, "--", "allowed target floor -16 mm"),
        (-15, ":", "Gavin floor estimate -15 mm"),
    ]:
        ax.axhline(z, ls=style, color="tab:red", label=label)
    ax.set(xlabel="session time (s)", ylabel="z (mm)")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.2)
    fig.suptitle("Torque-off hand trace: all collision vertices, including base and yaw")
    fig.savefig(OUT / "trace_extremes.png", dpi=150)
    plt.close(fig)
    print("trace", counts, flush=True)


def sim_inventory():
    selected, excluded = [], []
    for p in sorted(RUNS.rglob("chunks.jsonl")):
        rel = p.relative_to(RUNS)
        why = None
        if not p.stat().st_size:
            why = "empty chunk file; zero recorded executed rows"
        if any(part in ("legacy", "validation", "layout_v1") for part in rel.parts):
            why = "archived or validation copy; primary source replayed instead"
        log = p.parent / "log.json"
        data = json.loads(log.read_text()) if log.exists() else {}
        if not why:
            if isinstance(data, dict):
                arm = data.get("policy", {}).get("ik", {}).get("arm", "")
                rig = data.get("configuration", {}).get("rig", data.get("rig", ""))
                if rig == "yam" or arm in ("yam", "makers", "metal"):
                    why = f"other robot: {rig} {arm}"
                elif "viscous" not in str(arm) and rig != "viscous":
                    why = "no explicit Viscous identity in saved metadata"
            elif not (
                rel.parts[0].startswith("viscous_")
                or "viscous" in rel.parts[0]
                or rel.parts[0] == "replay_ik_smoke_fixed"
            ):
                why = "other robot in legacy run name"
        (excluded if why else selected).append({"path": str(rel), "reason": why} if why else str(rel))
    dump(
        "sim_inventory.json",
        {
            "source_root": str(RUNS),
            "selected": selected,
            "excluded": excluded,
            "scope": "all primary Viscous saved chunk runs, both virtual arms separately; includes dry/scripted/replay sources with labels; excludes archived duplicates and YAM/Maker/Metal",
        },
    )
    return selected


def sim_rows(path, side):
    chunks = [json.loads(s) for s in path.read_text().splitlines()]
    trajectory = path.parent / "trajectory.json"
    if trajectory.exists() and "execution" not in chunks[0]:
        data = json.loads(trajectory.read_text())
        rows = [r["commanded_" + side] for r in data]
        xyz = [r.get("tracking", {}).get(side, {}).get("requested_xyz") for r in data]
        if any(v is None for v in xyz):
            xyz = [
                chunks[int(r["call"]) - 1]["per_key"]["action." + side + "_eef_xyz"][int(r["row"])]
                for r in data
            ]
        source = "trajectory.json commanded_" + side + " (actual executor clipping retained)"
        hashes = {"chunks.jsonl": digest(path), "trajectory.json": digest(trajectory)}
    else:
        rows, xyz = [], []
        source = "chunks.jsonl execution." + side
        for c in chunks:
            if "execution" not in c:
                raise ValueError(f"No exact executed targets: {path}")
            n = c.get("executed_rows", len(c["execution"][side]))
            rr = c["execution"][side][:n]
            assert len(rr) == n
            rows.extend(rr)
            xyz.extend(c.get("per_key", {}).get("action." + side + "_eef_xyz", [None] * n)[:n])
        hashes = {"chunks.jsonl": digest(path)}
    assert len(rows) == sum(
        c["executed_rows"] if "executed_rows" in c else len(c["execution"][side]) for c in chunks
    )
    return np.asarray(rows), None if any(v is None for v in xyz) else np.asarray(xyz), source, hashes


def sim_map(rows):
    mapping = G.model.motor_to_urdf
    a = np.array(rows, dtype=float, copy=True)
    for i, m in enumerate(tg.ARM):
        a[:, i] = (np.degrees(a[:, i]) - mapping[m]["offset_deg"]) / mapping[m]["sign"]
    a[:, 0] += 180.0
    a[:, 6] = -2.92 + a[:, 6] * (-115.0 + 2.92)
    return a


def probe(q):
    return geo.probe_point(G.model, geo.forward_kinematics(G.model, q))


def sim_one(job):
    rel, side = job
    rows, xyz, source, hashes = sim_rows(RUNS / rel, side)
    a = sim_map(rows)
    requests = [dict(zip(tg.MOTORS, row.tolist(), strict=True)) for row in a]
    gate = G.start_gate(requests[0])
    runner = tg.GuardedTargets(G, requests[0] if gate.ok else REST)
    ramp = []
    if not gate.ok:
        # Explicit synthetic approach, <=1 degree per request from REST to first row.
        # It is NOT inserted into policy row statistics. Check guard start safety.
        n = math.ceil(max(abs(requests[0][m] - REST[m]) for m in tg.MOTORS))
        for k in range(1, n + 1):
            ramp.append(runner.step(lerp(REST, requests[0], k / n)).status)
    initialization_target = dict(runner.target)
    counts, reasons, binds = Counter(), Counter(), Counter()
    times, jd, pd, probes = [], [], [], []
    streak = longest = 0
    clipped = Counter()
    for q in requests:
        t = time.perf_counter_ns()
        d = runner.step(q)
        times.append((time.perf_counter_ns() - t) / 1e6)
        counts[d.status] += 1
        reasons[d.reason or "clear"] += 1
        clipped.update(d.clipped_joints)
        if d.status != "ok":
            streak += 1
            longest = max(longest, streak)
            if d.verdict and d.reason == "keepout":
                binds[f"{d.verdict.link}/{d.verdict.face}"] += 1
        else:
            streak = 0
        jd.append([q[m] - d.target[m] for m in tg.MOTORS])
        p = probe(q)
        probes.append(p)
        pd.append(float(np.linalg.norm(p - probe(d.target))))
    jd = np.abs(np.asarray(jd))
    probes = np.array(probes)
    if xyz is not None:
        xyz = xyz * np.array([-1000.0, -1000.0, 1000.0])  # virtual frame rotation, NO z translation
    planes = G.planes

    def axis_slack(x):
        return {
            "near": planes["near"] - float(x[:, 0].max()),
            "far": float(x[:, 0].min()) - planes["far"],
            "left": planes["left"] - float(x[:, 1].max()),
            "right": float(x[:, 1].min()) - planes["right"],
            "floor": float(x[:, 2].min()) - planes["floor"],
            "rim": planes["rim"] - float(x[:, 2].max()),
        }

    return {
        "run": rel,
        "side": side,
        "rows": len(rows),
        "input": source,
        "sha256": hashes,
        "first_start_gate": finite_verdict(gate),
        "initialization": "first row" if gate.ok else "REST plus synthetic <=1 degree ramp",
        "ramp_counts": dict(Counter(ramp)),
        "target_before_policy": initialization_target,
        "counts": dict(counts),
        "reasons": dict(reasons),
        "binding_link_faces": dict(binds),
        "clipped_joints": dict(clipped),
        "longest_non_ok_rows": longest,
        "longest_non_ok_s": longest * 0.05,
        "joint_deviation_max_abs_deg": percentiles(jd.max(axis=1)),
        "joint_deviation_l2_deg": percentiles(np.linalg.norm(jd, axis=1)),
        "joint_deviation_per_motor_deg": {
            m: {"max": float(jd[:, i].max()), "median": float(np.median(jd[:, i]))}
            for i, m in enumerate(tg.MOTORS)
        },
        "probe_deviation_mm": percentiles(pd),
        "step_ms": percentiles(times),
        "requested_probe_extent_mm": {"min": probes.min(axis=0).tolist(), "max": probes.max(axis=0).tolist()},
        "requested_probe_raw_plane_slack_mm": axis_slack(probes),
        "dream_raw_eef_extent_mm": None
        if xyz is None
        else {"min": xyz.min(axis=0).tolist(), "max": xyz.max(axis=0).tolist()},
        "dream_raw_eef_raw_plane_slack_mm": None if xyz is None else axis_slack(xyz),
    }


def sims_stage(workers):
    selected = sim_inventory()
    result = []
    with ProcessPoolExecutor(workers, initializer=init_worker) as pool:
        for r in pool.map(sim_one, [(p, s) for p in selected for s in ("left", "right")]):
            result.append(r)
            print("sim", r["run"], r["side"], r["counts"], flush=True)
    dump(
        "sim_summary.json",
        {
            "assumptions": [
                "motor=(URDF degrees-offset)/sign; shoulder_pan +=180 degrees",
                "real executor is assumed to present Dream with a virtual base rotated 180 degrees; forward=-x in base_link",
                "gripper fraction 0 -> -2.92 degrees, 1 -> -115 degrees; no z translation",
                "both virtual arms replayed independently as a single physical arm; each starts separately",
                "synthetic approach requests <=1 degree; policy jump refusals are never replaced by ramps",
            ],
            "runs": result,
        },
    )


def sample_worker(job):
    seed, n = job
    rng = np.random.default_rng(seed)
    setup = independent_setup(G.config, G.model)
    safe = []
    extrema = dict.fromkeys(tg.FACES, (math.inf, None, None))
    for i in range(n):
        if i % 2:
            q = {m: float(rng.uniform(*G.limits[m])) for m in tg.MOTORS}
        else:
            q = {
                m: float(np.clip(WORK[m] + rng.normal(0, [18, 30, 45, 45, 35, 65, 45][j]), *G.limits[m]))
                for j, m in enumerate(tg.MOTORS)
            }
        v, pp = independent_pose(G.model, q, setup, True)
        if v[0] >= 0:
            safe.append(q)
        for face in tg.FACES:
            # Endpoint chosen by literal face requirement, independent of other faces.
            best = (math.inf, None)
            for link, req in setup[2].items():
                if face not in req:
                    continue
                p = pp[link] @ setup[1].T
                axis = {"near": 0, "far": 0, "left": 1, "right": 1, "floor": 2, "rim": 2}[face]
                val = p[:, axis].max() if face in ("near", "left", "rim") else p[:, axis].min()
                slack = setup[0][face] - val if face in ("near", "left", "rim") else val - setup[0][face]
                slack -= req[face]
                if slack < best[0]:
                    best = (float(slack), link)
            if best[0] < extrema[face][0]:
                extrema[face] = (best[0], q, best[1])
    return safe, extrema


def exercise(job):
    category, seed, start, requests = job
    setup = independent_setup(G.config, G.model)
    runner = tg.GuardedTargets(G, start)
    counts = Counter()
    reasons = Counter()
    binds = Counter()
    samples = 0
    violations = []
    worst = (math.inf, None, None)
    times = []
    max_segment_step = 0.0
    binding_example = None
    for i, q in enumerate(requests):
        prev = dict(runner.target)
        t = time.perf_counter_ns()
        d = runner.step(q)
        times.append((time.perf_counter_ns() - t) / 1e6)
        counts[d.status] += 1
        reasons[d.reason or "clear"] += 1
        if d.verdict and d.reason == "keepout":
            binds[f"{d.verdict.link}/{d.verdict.face}"] += 1
            if binding_example is None:
                binding_example = {"index": i, "previous": prev, "requested": q, "decision": asdict(d)}
        if d.status in ("ok", "clamped"):
            ns, w, v = verify_segment(prev, d.target, setup)
            samples += ns
            max_segment_step = max(
                max_segment_step, max(abs(prev[m] - d.target[m]) for m in tg.MOTORS) / max(1, ns - 1)
            )
            if w[0] < worst[0]:
                worst = w
            if v:
                violations.append(
                    {
                        "category": category,
                        "seed": seed,
                        "index": i,
                        "previous": prev,
                        "requested": q,
                        "decision": asdict(d),
                        "violation": v,
                    }
                )
        elif d.target != prev:
            violations.append(
                {
                    "category": category,
                    "index": i,
                    "bug": "rejected target changed",
                    "previous": prev,
                    "requested": q,
                    "decision": asdict(d),
                }
            )
    return {
        "category": category,
        "seed": seed,
        "requests": len(requests),
        "counts": dict(counts),
        "reasons": dict(reasons),
        "binding_link_faces": dict(binds),
        "independent_samples": samples,
        "worst_independent_slack_mm": None if math.isinf(worst[0]) else worst[0],
        "worst_link": worst[1],
        "worst_face": worst[2],
        "max_sample_joint_step_deg": max_segment_step,
        "step_ms": percentiles(times),
        "violations": violations,
        "binding_example": binding_example,
    }


def bound_worker(seed):
    rng = np.random.default_rng(seed)
    worst_ratio = 0.0
    gap = math.inf
    bad = []
    for i in range(125):
        q0 = {m: float(rng.uniform(*G.limits[m])) for m in tg.MOTORS}
        # Include pure joints, grip-only, short and long simultaneous steps.
        delta = rng.uniform(-[0.01, 3.0, 20.0, 180.0][i % 4], [0.01, 3.0, 20.0, 180.0][i % 4], 7)
        if i % 5 == 0:
            delta[:] = 0.0
            delta[i % 7] = rng.uniform(-100, 100)
        q1 = {m: float(np.clip(q0[m] + delta[j], *G.limits[m])) for j, m in enumerate(tg.MOTORS)}
        a = points(G.model, q0)
        b = points(G.model, q1)
        true = max(
            float(np.linalg.norm(a[link] - b[link], axis=1).max())
            for link in G.model.moving_links
            if link in a
        )
        bound = G.motion_bound_mm(q0, q1)
        gap = min(gap, bound - true)
        ratio = true / bound if bound > 0 else 0.0
        worst_ratio = max(worst_ratio, ratio)
        if true > bound + 1e-7:
            bad.append({"previous": q0, "requested": q1, "bound_mm": bound, "true_mm": true})
    return {
        "steps": 125,
        "max_true_over_bound": worst_ratio,
        "min_bound_minus_true_mm": gap,
        "violations": bad,
    }


def adversarial_stage(workers):
    init_worker()
    with ProcessPoolExecutor(workers, initializer=init_worker) as pool:
        sampled = list(pool.map(sample_worker, [(SEED + i, 2000) for i in range(workers)]))
    safe = [q for ss, _ in sampled for q in ss]
    extrema = {f: min((ee[f] for _, ee in sampled), key=lambda x: x[0]) for f in tg.FACES}
    dump(
        "pose_pool.json",
        {
            "seed": SEED,
            "sampled": workers * 2000,
            "safe_count": len(safe),
            "safe_poses": safe[:1000],
            "face_endpoints": {f: {"slack_mm": v[0], "pose": v[1], "link": v[2]} for f, v in extrema.items()},
        },
    )
    rng = np.random.default_rng(SEED + 100)
    jobs = []
    for i in range(24):
        start = safe[int(rng.integers(len(safe)))]
        category = ["random_in_limits", "local_steps_3deg", "local_steps_19deg", "wraps", "ik_flips"][i % 5]
        requests = []
        for _ in range(100):
            if category == "random_in_limits":
                q = {m: float(rng.uniform(*G.limits[m])) for m in tg.MOTORS}
            else:
                q = dict(start)
                if category.startswith("local"):
                    span = 3 if "3deg" in category else 19
                    q = {
                        m: float(np.clip(start[m] + rng.uniform(-span, span), *G.limits[m]))
                        for m in tg.MOTORS
                    }
                elif category == "wraps":
                    q["shoulder_pan"] += float(rng.choice([-720, -360, 360, 720]))
                else:
                    q["elbow_flex"] += float(rng.choice([-1, 1]) * rng.uniform(90, 180))
                    q["wrist_flex"] -= 90
            requests.append(q)
        jobs.append((category, SEED + i, start, requests))
    for face, (_slack, target, _link) in extrema.items():
        # Far is geometrically out of reach. Still attempt the furthest sampled endpoint.
        n = math.ceil(max(abs(target[m] - WORK[m]) for m in tg.MOTORS) / 0.25)
        jobs.append(("ramp_" + face, SEED, WORK, [lerp(WORK, target, k / n) for k in range(1, n + 1)]))
    folded = {**WORK, "shoulder_lift": -100.0, "elbow_flex": 245.0, "wrist_flex": -100.0}
    n = 600
    jobs.append(("folded_elbow_near_wall", SEED, WORK, [lerp(WORK, folded, k / n) for k in range(1, n + 1)]))
    # Stateful tiny increments from granted targets test accumulation at a boundary.
    # Materialize with the real guard, then replay the identical requested sequence.
    for face, target in [
        ("left", {**WORK, "shoulder_pan": 187.5}),
        ("right", {**WORK, "shoulder_pan": 145.0}),
        ("floor", extrema["floor"][1]),
        ("near", folded),
    ]:
        r = tg.GuardedTargets(G, WORK)
        requests = []
        for _ in range(1600):
            q = {m: r.target[m] + float(np.clip(target[m] - r.target[m], -0.02, 0.02)) for m in tg.MOTORS}
            requests.append(q)
            r.step(q)
        jobs.append(("tiny_creep_" + face, SEED, WORK, requests))
    # Sweep to a side-wall boundary with closed fingers, then open fingers alone.
    for pan in (145.0, 187.5):
        r = tg.GuardedTargets(G, {**WORK, "gripper": -2.9220847551671985})
        requests = []
        for _ in range(400):
            q = {
                **r.target,
                "shoulder_pan": r.target["shoulder_pan"]
                + float(np.clip(pan - r.target["shoulder_pan"], -0.1, 0.1)),
            }
            requests.append(q)
            r.step(q)
        for grip in np.linspace(-2.9220847551671985, -115.0, 450):
            q = {**r.target, "gripper": float(grip)}
            requests.append(q)
            r.step(q)
        jobs.append(
            ("gripper_open_at_wall_" + str(pan), SEED, {**WORK, "gripper": -2.9220847551671985}, requests)
        )
    result = []
    with ProcessPoolExecutor(workers, initializer=init_worker) as pool:
        for r in pool.map(exercise, jobs):
            result.append(r)
            print(
                "adversarial",
                r["category"],
                r["counts"],
                "samples",
                r["independent_samples"],
                "bugs",
                len(r["violations"]),
                flush=True,
            )
        bounds = list(pool.map(bound_worker, [SEED + 1000 + i for i in range(24)]))
    dump(
        "adversarial_summary.json",
        {
            "seed": SEED,
            "verification_step_deg": 0.05,
            "verification_epsilon_mm": 1e-7,
            "verification": "independent FK -> every world collision vertex -> explicit rotated planes and independently parsed requirements",
            "campaigns": result,
            "requests": sum(r["requests"] for r in result),
            "independent_samples": sum(r["independent_samples"] for r in result),
            "violations": [v for r in result for v in r["violations"]],
            "motion_bound": {
                "steps": sum(r["steps"] for r in bounds),
                "max_true_over_bound": max(r["max_true_over_bound"] for r in bounds),
                "min_bound_minus_true_mm": min(r["min_bound_minus_true_mm"] for r in bounds),
                "violations": [v for r in bounds for v in r["violations"]],
            },
        },
    )


def workspace_worker(job):
    seed, n = job
    rng = np.random.default_rng(seed)
    variants = []
    for margin in (20.0, 30.0):
        for pad in (0.0, 20.0, 40.0):
            config = copy.deepcopy(G.config)
            config["margins_mm"] = {"value": dict.fromkeys(tg.WALL_FACES, margin)}
            config["padding_mm"] = {"value": {"default": pad, "gripper_left": 0.0, "gripper_right": 0.0}}
            variants.append((margin, pad, independent_setup(config, G.model)))
    # Stratify J1 independently; all other motors uniform over shipped limits.
    bins = {}
    accepted = [[] for _ in variants]
    count = np.zeros(len(variants), dtype=int)
    for i in range(n):
        q = {m: float(rng.uniform(*G.limits[m])) for m in tg.MOTORS}
        # Focus half the samples down the tank while retaining global coverage.
        if i % 2:
            q["shoulder_pan"] = float(rng.uniform(130.0, G.limits["shoulder_pan"][1]))
        pp = points(G.model, q)
        tf = geo.forward_kinematics(G.model, q)
        tip = geo.probe_point(G.model, tf)
        reach = float(np.linalg.norm(tip[:2]))
        height = float(tip[2])
        ext = {link: (p.min(axis=0), p.max(axis=0)) for link, p in pp.items()}
        for j, (_margin, _pad, setup) in enumerate(variants):
            ok = True
            for link, req in setup[2].items():
                lo, hi = ext[link]
                p = setup[0]
                d = {
                    "near": p["near"] - hi[0],
                    "far": lo[0] - p["far"],
                    "left": p["left"] - hi[1],
                    "right": lo[1] - p["right"],
                    "floor": lo[2] - p["floor"],
                    "rim": p["rim"] - hi[2],
                }
                if any(d[f] < v for f, v in req.items()):
                    ok = False
                    break
            if ok:
                count[j] += 1
                accepted[j].append([float(tip[0]), float(tip[1]), float(tip[2]), q["shoulder_pan"]])
                rb = math.floor(reach / 50) * 50
                zb = math.floor(height / 50) * 50
                key = (j, rb, zb)
                pan = q["shoulder_pan"]
                if key not in bins:
                    bins[key] = [pan, pan, 1]
                else:
                    bins[key] = [min(bins[key][0], pan), max(bins[key][1], pan), bins[key][2] + 1]
    return {"count": count.tolist(), "accepted": accepted, "bins": [[*k, *v] for k, v in bins.items()]}


def workspace_stage(workers):
    # 192k sampled poses at default 12 workers. Same candidates for all settings.
    with ProcessPoolExecutor(workers, initializer=init_worker) as pool:
        result = list(pool.map(workspace_worker, [(SEED + 2000 + i, 16000) for i in range(workers)]))
    total = workers * 16000
    settings = []
    pan_bins = {}
    bins = {}
    for r in result:
        for j, rb, zb, lo, hi, n in r["bins"]:
            key = (j, rb, zb)
            if key not in bins:
                bins[key] = [lo, hi, n]
            else:
                bins[key] = [min(lo, bins[key][0]), max(hi, bins[key][1]), n + bins[key][2]]
    plt = plotting()
    fig, axs = plt.subplots(2, 3, figsize=(15, 9), sharex=True, sharey=True, layout="constrained")
    variants = [(m, p) for m in (20.0, 30.0) for p in (0.0, 20.0, 40.0)]
    for j, (margin, pad) in enumerate(variants):
        a = np.array([p for r in result for p in r["accepted"][j]])
        for x, y, z, pan in a:
            key = (j, math.floor(math.hypot(x, y) / 50) * 50, math.floor(z / 50) * 50)
            pan_bins.setdefault(key, Counter())[math.floor(pan / 5) * 5] += 1
        occupied = len({(math.floor(x / 10), math.floor(y / 10)) for x, y in a[:, :2]})
        settings.append(
            {
                "margin_mm": margin,
                "cable_padding_mm": pad,
                "sampled": total,
                "allowed": len(a),
                "allowed_fraction": len(a) / total,
                "probe_min_mm": a[:, :3].min(axis=0).tolist(),
                "probe_max_mm": a[:, :3].max(axis=0).tolist(),
                "j1_min_deg": float(a[:, 3].min()),
                "j1_max_deg": float(a[:, 3].max()),
                "occupied_10mm_xy_cells": occupied,
                "occupied_xy_area_cm2": occupied,
            }
        )
        ax = axs.flat[j]
        sc = ax.scatter(
            a[:, 0], a[:, 1], c=a[:, 2], s=1.5, cmap="viridis", vmin=-16, vmax=397, alpha=0.6, rasterized=True
        )
        x0, x1, y0, y1 = -694, 40, -224, 210
        from matplotlib.patches import Rectangle

        ax.add_patch(
            Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, color="black", lw=1.2, label="glass planes")
        )
        for inset, color, style, label in [
            (margin, "tab:red", "--", "finger margin"),
            (margin + pad, "tab:orange", ":", "cable margin + padding"),
        ]:
            ax.add_patch(
                Rectangle(
                    (x0 + inset, y0 + inset),
                    x1 - x0 - 2 * inset,
                    y1 - y0 - 2 * inset,
                    fill=False,
                    color=color,
                    ls=style,
                    label=label,
                )
            )
        ax.plot(0, 0, "r+", ms=7)
        ax.set_title(
            f"margin {margin:g} mm / cables {pad:g} mm\n{len(a):,} poses; {occupied} occupied 10 mm cells"
        )
        ax.set_aspect("equal")
        ax.set(xlim=(-720, 70), ylim=(-245, 230))
        ax.grid(alpha=0.15)
        if j >= 3:
            ax.set_xlabel("base x (mm); forward is negative")
        if j % 3 == 0:
            ax.set_ylabel("base y (mm)")
    axs.flat[0].legend(fontsize=7, loc="lower left")
    fig.colorbar(sc, ax=axs.ravel().tolist(), label="fingertip probe height (mm)", shrink=0.75)
    fig.suptitle(
        "Sampled pose workspace: full mesh rules; endpoint feasibility only\nUpper-arm near exception and yaw exemption retained; colored dots are sampled fingertip positions"
    )
    fig.savefig(OUT / "workspace_topdown.png", dpi=150)
    plt.close(fig)
    dump(
        "workspace_summary.json",
        {
            "seed": SEED + 2000,
            "sampled": total,
            "sampling": "50% all seven motors uniform within limits; 50% J1 uniform [130,187.529] with remaining motors uniform; identical candidates for all six variants",
            "caution": "sampled endpoint workspace, not IK completeness, connected reachability, volume, or a path guarantee; J1 min/max may span gaps",
            "settings": settings,
            "j1_by_reach_height": [
                {
                    "setting": j,
                    "reach_bin_mm": [rb, rb + 50],
                    "height_bin_mm": [zb, zb + 50],
                    "j1_min_deg": v[0],
                    "j1_max_deg": v[1],
                    "poses": v[2],
                    "observed_j1_5deg_bins": [
                        {"lower_deg": pan, "upper_deg": min(pan + 5, 187.52908634631845), "poses": count}
                        for pan, count in sorted(pan_bins[j, rb, zb].items())
                    ],
                }
                for (j, rb, zb), v in sorted(bins.items())
            ],
        },
    )
    print("workspace", settings, flush=True)


def benchmark_stage():
    init_worker()
    wall = {**WORK, "shoulder_pan": 163.0}
    cases = {
        "hold": (WORK, WORK),
        "small_1deg": (WORK, {**WORK, "elbow_flex": 121.0}),
        "multi_joint_10deg": (
            WORK,
            {**WORK, "shoulder_lift": -160.0, "elbow_flex": 130.0, "wrist_flex": 20.0},
        ),
        "wall_clamp": (WORK, wall),
        "jump": (WORK, {**WORK, "shoulder_pan": -178.0}),
    }
    out = {}
    for name, (a, b) in cases.items():
        for _ in range(10):
            G.limit(a, b)
        ts = []
        for _ in range(150):
            t = time.perf_counter_ns()
            d = G.limit(a, b)
            ts.append((time.perf_counter_ns() - t) / 1e6)
        out[name] = {"step_ms": percentiles(ts), "status": d.status, "reason": d.reason}
    pr = cProfile.Profile()
    pr.enable()
    for _ in range(50):
        G.limit(WORK, wall)
    pr.disable()
    s = io.StringIO()
    pstats.Stats(pr, stream=s).strip_dirs().sort_stats("cumtime").print_stats(25)
    dump("profile_wall_clamp.json", {"pstats": s.getvalue()})
    dump(
        "benchmark.json",
        {
            "isolation": "one process, no other validation workers; 10 warmups then 150 repeats per case",
            "cases": out,
        },
    )
    print("benchmark", out, flush=True)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("stage", choices=["trace", "sims", "adversarial", "workspace", "benchmark"])
    p.add_argument("--workers", type=int, default=12)
    p.add_argument("--trace", default="/tmp/tankbox/trace.jsonl")
    args = p.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    start = time.time()
    if args.stage == "trace":
        trace_stage(args.trace)
    elif args.stage == "sims":
        sims_stage(args.workers)
    elif args.stage == "adversarial":
        adversarial_stage(args.workers)
    elif args.stage == "workspace":
        workspace_stage(args.workers)
    else:
        benchmark_stage()
    dump(
        "stage_" + args.stage + ".json",
        {
            "stage": args.stage,
            "wall_seconds": time.time() - start,
            "workers": args.workers,
            "script_sha256": digest(__file__),
            "guard_sha256": digest(ROOT / "makermodslab/tank_guard.py"),
            "config_sha256": digest(tg.DEFAULT_CONFIG),
        },
    )


if __name__ == "__main__":
    main()
