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
"""Build calibrated sidecars, static URDFs, foam meshes and workspace predictions."""

import argparse
import hashlib
import json

import numpy as np
import trimesh
from cad import measure, pocket_checks, pockets
from hydro import (
    BODIES,
    CORNERS,
    JOINTS,
    KEEP,
    NOTICE,
    OUT,
    RHO,
    ROOT,
    SURFACE,
    URDF,
    G,
    block_load,
    frames,
    geo,
    mass_loads,
    write_json,
)
from scipy.optimize import linprog, lsq_linear
from scipy.spatial import ConvexHull
from scipy.stats import qmc
from static_urdf import xml_output

from makermodslab.tank_guard import TankGuard

ORDER = [*JOINTS, "gripper"]
DENSITY = 60.0


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def predict(dry, water, mass):
    return dry - water @ mass


def summary(a):
    return {
        key: values.tolist()
        for key, values in (
            ("median", np.median(abs(a), axis=0)),
            ("p95", np.percentile(abs(a), 95, axis=0)),
            ("max", abs(a).max(axis=0)),
        )
    }


def calibrate(model, links, audit):
    records = json.loads((OUT / "wet_observations.json").read_text())["records"]
    poses = np.array([r["pos_deg"] for r in records])
    measured = np.array([r["torque_nm"] for r in records])
    groups = np.array([r["run"] for r in records])
    dry, water = mass_loads(model, poses, links)
    prior = np.array([link["volume_cad_m3"] * RHO for link in links.values()])
    envelope = np.array([audit[n]["guard_union_3mm_cm3"] * 0.001 for n in links])
    # Fix base/yaw (no vertical moment) and small fingers (poor excitation).
    fitted = np.arange(2, 7)
    design = water[:, :, fitted] * prior[fitted]
    target = predict(dry, water, prior) - measured
    # Allow flooding/potting and dry CAD error, but never more than the collision envelope.
    # For the two long links reserve confirmed open pockets used by the insert design.
    for _, n, _, size in pockets():
        envelope[BODIES.index(n)] -= RHO * np.prod(size)
    hi = np.minimum(2.0, envelope[fitted] / prior[fitted]) - 1

    def solve(keep, regularizer=0.35):
        # Penalise relative volume changes; prior scale is an assumed 35% per link.
        # Approximately one independent observation per run, typical torque error 0.35 Nm.
        w = np.array([1 / np.sqrt(np.sum(groups[keep] == g)) for g in groups[keep]])
        matrix = (design[keep] * w[:, None, None]).reshape(-1, len(fitted))
        rhs = (target[keep] * w[:, None]).ravel()
        a = np.vstack((matrix, np.eye(len(fitted)) * 0.35 / regularizer))
        b = np.r_[rhs, np.zeros(len(fitted))]
        change = lsq_linear(a, b, bounds=(-0.65, hi)).x
        out = prior.copy()
        out[fitted] *= 1 + change
        return out

    mass = solve(np.ones(len(poses), dtype=bool))
    cv = np.zeros_like(measured)
    leave = []
    for group in sorted(set(groups)):
        mask = groups == group
        m = solve(~mask)
        leave.append(m)
        cv[mask] = predict(dry[mask], water[mask], m)
    fitted_tau = predict(dry, water, mass)
    by_day = {}
    for date in ("20261108", "20261114"):
        test_mask = np.array([date in run for run in groups])
        held_out = predict(dry[test_mask], water[test_mask], solve(~test_mask))
        by_day[date] = np.sqrt(np.mean((held_out - measured[test_mask]) ** 2, axis=0)).tolist()

    def rms(delta):
        return np.sqrt(np.mean(delta**2, axis=0)).tolist()

    dry_ref = json.loads(
        (ROOT / "configs/viscous/viscous_waterproof_01/evidence/gravity-dry-air-selected.json").read_text()
    )["rows"]
    dry_poses = np.array([[r["pos"][n] for n in JOINTS] + [model.gripper_motor_closed_deg] for r in dry_ref])
    comparison, _ = mass_loads(model, dry_poses, links, submerged=True)
    dry_err = comparison - np.array([[r["torque"][n] for n in JOINTS] for r in dry_ref])
    uncertainty = [solve(np.ones(len(poses), bool), s) for s in (0.15, 0.70)]
    stats = {
        "pose_bins": len(records),
        "runs": len(set(groups)),
        "range_deg": [poses.min(0).tolist(), poses.max(0).tolist()],
        "measured_abs_nm": summary(measured),
        "fitted_abs_nm": summary(fitted_tau),
        "mean_measured_minus_model_nm": (measured - fitted_tau).mean(axis=0).tolist(),
        "nearest_dry_pose_max_joint_delta_deg": np.abs(poses[:, None, :6] - dry_poses[None, :, :6])
        .max(axis=-1)
        .min(axis=1)
        .tolist(),
        "dry_rms_nm": rms(dry - measured),
        "mesh_prior_rms_nm": rms(predict(dry, water, prior) - measured),
        "fitted_rms_nm": rms(fitted_tau - measured),
        "leave_run_out_rms_nm": rms(cv - measured),
        "leave_day_out_rms_nm": by_day,
        "max_abs_error_nm": np.max(abs(cv - measured), axis=0).tolist(),
        "dry_air_reference_bins": len(dry_ref),
        "dry_air_cad_rms_nm": rms(dry_err),
        "design_singular_values": np.linalg.svd(design.reshape(-1, len(fitted)), compute_uv=False).tolist(),
        "leave_run_out_volume_span_cm3": (np.ptp(leave, axis=0) * 1000).tolist(),
        "prior_sensitivity_displaced_kg": [v.tolist() for v in uncertainty],
        "cutout_rehearsal": [
            {"measured_nm": t.tolist(), "cad_dry_nm": d.tolist(), "wet_nm": p.tolist(), "pos_deg": q.tolist()}
            for t, d, p, q, g in zip(measured, dry, fitted_tau, poses, groups, strict=True)
            if g == "rehearse-live-20261114T082911Z"
        ],
    }
    for i, link in enumerate(links.values()):
        link["volume_m3"] = mass[i] / RHO
        if i in fitted:
            link["volume_source"] += "; relative displacement scale fitted to wet free holds, fixed CAD COB"
            link["trust"] = "effective gravity parameter, confounded with dry mass and first moment"
    return stats


def workspace(model, exponent=16):
    guard = TankGuard(KEEP, model=model)
    limits = np.array([guard.limits[n] for n in ORDER])
    samples = qmc.scale(qmc.Sobol(7, scramble=True, seed=20261118).random_base2(exponent), *limits.T)
    vertices = {n: v[ConvexHull(v).vertices] for n, v in model.vertices.items() if n in BODIES[2:]}
    kept, heights = [], []
    for batch in np.array_split(samples, len(samples) // 512):
        trans, _ = frames(model, batch)
        ok = np.ones(len(batch), bool)
        top = np.full(len(batch), -np.inf)
        for n, v in vertices.items():
            r, t = trans[n]
            req = guard.required[n]
            lo = (
                np.array(
                    [
                        guard.planes["far"] + req["far"],
                        guard.planes["right"] + req["right"],
                        guard.planes["floor"],
                    ]
                )
                / 1000
            )
            hi = (
                np.array(
                    [
                        guard.planes["near"] - req["near"],
                        guard.planes["left"] - req["left"],
                        guard.planes["rim"] - req["rim"],
                    ]
                )
                / 1000
            )
            points = v @ r.transpose(0, 2, 1) + t[:, None, :]
            ok &= ((points.min(1) >= lo) & (points.max(1) <= hi)).all(axis=1)
            top = np.maximum(top, points[:, :, 2].max(1))
        kept.extend(batch[ok])
        heights.extend(top[ok])
    return np.asarray(kept), np.asarray(heights)


def minimax(torque, deltas, bounds):
    # Variables multiply the full-size block deltas. Only the three loaded axes enter the objective.
    design = deltas[:, 1:4, :].reshape(-1, len(bounds))
    b = torque[:, 1:4].ravel()
    both = np.vstack([design, -design])
    result = linprog(
        [*[0] * len(bounds), 1],
        A_ub=np.column_stack([both, -np.ones(len(both))]),
        b_ub=np.r_[-b, b],
        bounds=[*bounds, (0, None)],
        method="highs",
    )
    if not result.success:
        raise ValueError(result.message)
    return result.x[:-1]


def write_meshes(blocks):
    for b in blocks:
        # ASCII STL permits the header to carry the licence in the solid name.
        name = NOTICE["_copyright"] + " " + NOTICE["_license"]
        stl = trimesh.creation.box(extents=b["size_m"]).export(file_type="stl_ascii")
        lines = stl.splitlines()
        lines[0] = "solid " + name
        lines[-1] = "endsolid " + name
        (ROOT / b["mesh"]).write_text("\n".join(lines) + "\n")


def run(remeasure=False):
    if remeasure:
        links, audit = measure()
        write_json("cad.json", {"links": links, "audit": audit})
    cad = json.loads((OUT / "cad.json").read_text())
    links = cad["links"]
    model = geo.load_model(geo.load_tank())
    fit = calibrate(model, links, cad["audit"])
    poses, top = workspace(model)
    dry, water = mass_loads(model, poses, links)
    mass = np.array([v["volume_m3"] * RHO for v in links.values()])
    baseline = predict(dry, water, mass)
    candidates = []
    for name, n, c, s in pockets():
        v = float(np.prod(s))
        block = {
            "name": name,
            "link": n,
            "size_m": s,
            "center_m": c,
            "rpy": [0, 0, 0],
            "density": DENSITY,
            "volume": v,
            "mass": v * DENSITY,
            "inside_existing_hull": True,
            "mesh": str((OUT / f"{name}.stl").relative_to(ROOT)),
        }
        block["cad_fit"] = pocket_checks(block)
        if not block["cad_fit"]["inside_guard_union"] or block["cad_fit"]["part_aabb_gap_mm"] < 1.9:
            raise ValueError(f"No geometric support for pocket {name}")
        candidates.append(block)
    deltas = np.stack([block_load(model, poses, b) * b["volume"] for b in candidates], axis=-1)
    # The adjustable quantity is the fraction of each pocket filled, uniformly
    # through its volume. At integral optima this is exactly a solid rectangular blank.
    fractions = minimax(baseline, deltas, [(0, 1)] * len(candidates))
    if np.max(abs(fractions - np.rint(fractions))) > 1e-8:
        raise ValueError("Fractional optimum requires new cut shapes and a second waterline evaluation")
    blocks = [b for b, f in zip(candidates, fractions, strict=True) if f > 0.5]
    results = {
        "total_proposals": 2**16,
        "accepted_poses": len(poses),
        "fully_submerged": int(np.sum(top <= SURFACE)),
        "joint_order": JOINTS,
        "fit": fit,
        "fractions": fractions.tolist(),
    }
    # External floats clear both wrist-roll and finger AABBs in the roll frame.
    # Their span must also be included in the guard; load the URDF with the added collision meshes.
    wrist = []
    for sign, name in [(-1, "wrist_minus_y"), (1, "wrist_plus_y")]:
        size = [0.090, 0.045, 0.050]
        vol = float(np.prod(size))
        wrist.append(
            {
                "name": name,
                "link": "wrist_roll_link",
                "center_m": [0.028, sign * 0.105, 0],
                "size_m": size,
                "rpy": [0, 0, 0],
                "density": DENSITY,
                "volume": vol,
                "mass": vol * DENSITY,
                "inside_existing_hull": False,
                "mesh": str((OUT / f"{name}.stl").relative_to(ROOT)),
            }
        )
    write_meshes(blocks + wrist)
    wrist_delta = sum(block_load(model, poses, b) * b["volume"] for b in wrist)
    stats_by_design = {}
    for variant, foam in [("current", []), ("foamed", blocks), ("wrist_alternative", blocks + wrist)]:
        side = {
            "schema": "viscous-buoyancy/1",
            "variant": variant,
            "water_density_kg_m3": RHO,
            "g_m_s2": G,
            "urdf": str(URDF.relative_to(ROOT)),
            "urdf_sha256": sha(URDF),
            "water_surface_z_m": SURFACE,
            "provenance": {
                "wet_data": "wet_observations.json",
                "wet_data_sha256": sha(OUT / "wet_observations.json"),
                "cad_geometry": "cad.json",
                "cad_geometry_sha256": sha(OUT / "cad.json"),
                "model": "viscous_waterproof_01",
                "build": "scripts/viscous-buoyancy/build.py",
                "volume_samples": "Equal-volume XYZ points in link metres; mean = COB. Multiply wet fraction by volume_m3.",
                "foam": "Add each block's mass and inertia, and displace its volume separately. Bare links identical across variants. inside_existing_hull means inside guard shell, in a flooded CAD gap.",
            },
            "links": links,
            "foam_blocks": foam,
            "gravity_only_urdf": str((OUT / f"{variant}_gravity_only.urdf").relative_to(ROOT)),
        }
        write_json(f"{variant}.json", side)
        xml_output(side)
        tau = baseline + deltas @ fractions if foam else baseline
        if variant == "wrist_alternative":
            tau = tau + wrist_delta
            wrist_model = geo.load_model(geo.load_tank() | {"arm_urdf": side["gravity_only_urdf"]})
            guard = TankGuard(KEEP, model=wrist_model)
            safe = np.array([guard.check_pose(dict(zip(ORDER, q, strict=True))).ok for q in poses])
            # Guard permits 20 mm floor press. Exclude all external foam below the
            # floor from free-water evaluation, and recompute its top for submergence.
            tr, _ = frames(model, poses)
            rot, trans = tr["wrist_roll_link"]

            newtop = top.copy()
            for b in wrist:
                v = CORNERS * b["size_m"] + b["center_m"]
                z = (v @ rot.transpose(0, 2, 1) + trans[:, None, :])[:, :, 2]
                safe &= z.min(axis=1) >= KEEP["planes_mm"]["floor"]["value"] / 1000
                newtop = np.maximum(newtop, z.max(axis=1))
            results["wrist_alternative"] = {
                "blocks": wrist,
                "shared_pose_all": summary(tau),
                "guard_clear_free_poses": int(safe.sum()),
                "all": summary(tau[safe]),
                "fully_submerged_poses": int((safe & (newtop <= SURFACE)).sum()),
                "fully_submerged": summary(tau[safe & (newtop <= SURFACE)]),
            }
            continue
        stats_by_design[variant] = {
            "all": summary(tau),
            "fully_submerged": summary(tau[top <= SURFACE]),
            "shoulder_over_5nm": int(np.sum(abs(tau[:, 1]) > 5)),
            "submerged_shoulder_over_5nm": int(np.sum(abs(tau[top <= SURFACE, 1]) > 5)),
            "smallest_peak_pitch_torque_nm": float(np.min(np.max(abs(tau[:, 1:4]), axis=1))),
            "smallest_submerged_peak_pitch_torque_nm": float(
                np.min(np.max(abs(tau[top <= SURFACE, 1:4]), axis=1))
            ),
            "worst_motor_deg": poses[np.argmax(abs(tau), axis=0)].tolist(),
        }
    cert = {}
    for rho in [30.0, 60.0, 100.0]:
        change = sum(block_load(model, poses, b | {"density": rho}) * b["volume"] for b in blocks)
        cert[str(int(rho))] = summary(baseline + change)
    results["foam_density_sensitivity"] = cert
    mesh_mass = np.array([link["volume_cad_m3"] * RHO for link in links.values()])
    mesh_foamed = predict(dry, water, mesh_mass) + deltas @ fractions
    results["mesh_prior_only_foamed"] = {
        "all": summary(mesh_foamed),
        "fully_submerged": summary(mesh_foamed[top <= SURFACE]),
    }
    # Equivalent lift at the available centres if the space bounds are removed;
    # fully immersed to avoid inventing an external float's shape or waterline.
    whole = top <= SURFACE
    lift = deltas[whole]
    theoretical = minimax(baseline[whole], lift, [(0, None)] * len(candidates))
    results["uncapped_full_submergence"] = {
        "equivalent_cm3_at_each_pocket": [
            b["volume"] * f * 1e6 for b, f in zip(candidates, theoretical, strict=True)
        ],
        "static": summary(baseline[whole] + lift @ theoretical),
    }
    # Virtual work of the two opposed slides per motor radian. This is only the
    # gravity load of an empty jaw, excluding any grip force or transmission loss.
    transforms, _ = frames(model, poses)
    travel = model.gripper_slide_open_m / np.radians(
        model.gripper_motor_open_deg - model.gripper_motor_closed_deg
    )
    jaw = np.zeros(len(poses))
    for name, sign in [("gripper_left", 1), ("gripper_right", -1)]:
        link = links[name]
        rot, origin = transforms[name]
        z = (np.asarray(link["volume_samples"]) @ rot.transpose(0, 2, 1) + origin[:, None, :])[:, :, 2]
        wet = (z < SURFACE).mean(axis=1)
        jaw += G * sign * travel * rot[:, 2, 1] * (link["mass_kg"] - RHO * link["volume_m3"] * wet)
    results["unloaded_gripper_motor_nm"] = summary(jaw)
    results["workspace"] = stats_by_design
    results["fit_sensitivity_foamed_max"] = [
        summary(predict(dry, water, m) + deltas @ fractions)["max"]
        for m in fit["prior_sensitivity_displaced_kg"]
    ]
    write_json("analysis.json", results)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--remeasure", action="store_true", help="re-integrate CAD-part meshes before fitting")
    run(p.parse_args().remeasure)
