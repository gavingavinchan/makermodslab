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
"""Build the human report from saved offline evidence; no recomputed guard results."""

import json
from collections import Counter

import validate as v


def read(name):
    return json.loads((v.OUT / (name + ".json")).read_text())


def fmt_extent(ext):
    if ext is None:
        return "not recorded"
    return "; ".join(f"{a:.1f}..{b:.1f}" for a, b in zip(ext["min"], ext["max"], strict=True))


def table(headers, rows):
    return "\n".join(
        [
            "| " + " | ".join(headers) + " |",
            "| " + " | ".join(["---"] * len(headers)) + " |",
            *["| " + " | ".join(str(x) for x in row) + " |" for row in rows],
        ]
    )


def j1_band(row):
    # Keep the forward-angle branch separate from the near-zero branch.
    bins = [b for b in row["observed_j1_5deg_bins"] if b["lower_deg"] >= 90]
    if not bins:
        return "none sampled"
    return f"{bins[0]['lower_deg']:.0f}..{bins[-1]['upper_deg']:.1f} ({sum(b['poses'] for b in bins)})"


def main():
    trace = read("trace_summary")
    sim = read("sim_summary")
    adv = read("adversarial_summary")
    direct = read("directed_summary")
    ind = read("sim_independent_verification")
    ws = read("workspace_summary")
    bench = read("benchmark")
    runs = sim["runs"]
    counts = Counter()
    binds = Counter()
    ramps = Counter()
    for run in runs:
        counts.update(run["counts"])
        binds.update(run["binding_link_faces"])
        ramps.update(run["ramp_counts"])
    total = adv["requests"] + direct["requests"]
    dense = adv["independent_samples"] + direct["independent_samples"] + ind["samples"]
    assert (
        not adv["violations"]
        and not direct["violations"]
        and not ind["violations"]
        and not adv["motion_bound"]["violations"]
    )
    appendix = [
        "# Sim run details",
        "",
        "Evidence: [sim_summary.json](sim_summary.json). Both virtual arms are replayed independently. O/C/R means ok/clamped/rejected. Policy rows exclude the synthetic approach. Joint error is the maximum absolute error across seven motors for each row. Error cells show median / maximum. Time cells show mean / p99 / maximum. A jump refusal has no binding face; see the JSON reasons and clipped-joint counts.",
        "",
        table(
            [
                "Run / side",
                "O / C / R",
                "Binding link / face",
                "Longest non-ok rows (s)",
                "Joint error (deg)",
                "Probe error (mm)",
                "Step time (ms)",
            ],
            [
                [
                    r["run"].removesuffix("/chunks.jsonl") + " / " + r["side"],
                    f"{r['counts'].get('ok', 0)} / {r['counts'].get('clamped', 0)} / {r['counts'].get('rejected', 0)}",
                    ", ".join(f"{k}: {n}" for k, n in r["binding_link_faces"].items()) or "jump filter",
                    f"{r['longest_non_ok_rows']} ({r['longest_non_ok_s']:.2f})",
                    f"{r['joint_deviation_max_abs_deg']['median']:.2f} / {r['joint_deviation_max_abs_deg']['max']:.2f}",
                    f"{r['probe_deviation_mm']['median']:.2f} / {r['probe_deviation_mm']['max']:.2f}",
                    f"{r['step_ms']['mean']:.4f} / {r['step_ms']['p99']:.4f} / {r['step_ms']['max']:.4f}",
                ]
                for r in runs
            ],
        ),
        "",
        "# Start gates and paths",
        "",
        "Extents list x; y; z, in mm, after the assumed virtual-base rotation. Probe means FK of the exact executed targets. Raw EEF means Dream’s saved Cartesian request; its TCP differs from the physical fingertip probe. Missing raw EEF values belong to local hold stubs. The JSON also gives each plane’s worst slack.",
        "",
        table(
            [
                "Run / side",
                "First gate: link / face (slack mm)",
                "Approach O / C / R",
                "Requested probe xyz extents (mm)",
                "Raw Dream EEF xyz extents (mm)",
            ],
            [
                [
                    r["run"].removesuffix("/chunks.jsonl") + " / " + r["side"],
                    f"{r['first_start_gate']['link']} / {r['first_start_gate']['face']} ({r['first_start_gate']['slack_mm']:.2f})",
                    " / ".join(str(r["ramp_counts"].get(s, 0)) for s in ["ok", "clamped", "rejected"]),
                    fmt_extent(r["requested_probe_extent_mm"]),
                    fmt_extent(r["dream_raw_eef_extent_mm"]),
                ]
                for r in runs
            ],
        ),
    ]
    (v.OUT / "sim_runs.md").write_text("\n".join(appendix) + "\n")
    examples = []
    for name in [
        "viscous_apple_makerwrist_live1",
        "viscous_box_makerwrist_live1",
        "sweep/B0/apple/ep_00",
        "sweep/B0/box/ep_00",
    ]:
        r = next(r for r in runs if r["run"] == name + "/chunks.jsonl" and r["side"] == "left")
        examples.append(
            [
                name,
                f"{r['rows']} rejected",
                fmt_extent(r["dream_raw_eef_extent_mm"]),
                f"{-r['dream_raw_eef_raw_plane_slack_mm']['left']:.2f}",
            ]
        )
    crossing = []
    for face in v.tg.FACES:
        c = trace["raw_plane_crossings"][face]
        ranges = ", ".join(f"{x['start_s']:.3f}–{x['end_s']:.3f}" for x in c["ranges"]) or "none"
        crossing.append([face, c["outside_readings"], f"{c['worst_raw_slack_mm']:.3f}", ranges])
    wstable = [
        [
            int(s["margin_mm"]),
            int(s["cable_padding_mm"]),
            f"{s['allowed']:,}",
            f"{100 * s['allowed_fraction']:.3f}%",
            s["occupied_10mm_xy_cells"],
        ]
        for s in ws["settings"]
    ]
    j1rows = []
    for setting, s in enumerate(ws["settings"]):
        for z in [0, 150]:
            row = [f"{s['margin_mm']:.0f} / {s['cable_padding_mm']:.0f}", f"{z}..{z + 50}"]
            for reach in [200, 400, 550]:
                r = next(
                    r
                    for r in ws["j1_by_reach_height"]
                    if r["setting"] == setting
                    and r["reach_bin_mm"][0] == reach
                    and r["height_bin_mm"][0] == z
                )
                row.append(j1_band(r))
            j1rows.append(row)
    btable = []
    for name, c in bench["cases"].items():
        t = c["step_ms"]
        btable.append([name, c["status"], f"{t['mean']:.3f}", f"{t['p99']:.3f}", f"{t['max']:.3f}"])
    minslack = min(
        [
            r["worst_independent_slack_mm"]
            for r in adv["campaigns"] + direct["campaigns"]
            if r["worst_independent_slack_mm"] is not None
        ]
        + [r["worst_slack_mm"] for r in ind["runs"] if r["worst_slack_mm"] is not None]
    )
    nonempty = [r for r in runs if r["dream_raw_eef_raw_plane_slack_mm"] is not None]
    fit = sum(all(x >= 0 for x in r["dream_raw_eef_raw_plane_slack_mm"].values()) for r in nonempty)
    acounts = Counter()
    for c in adv["campaigns"] + direct["campaigns"]:
        acounts.update(c["counts"])
    report = f"""# Aquarium keep-out guard: offline validation

No unsafe granted segment was found. The independent check covered **{dense:,} poses** at joint intervals of at most **0.05°**. The worst checked slack was **{minslack:.9g} mm**. This is numerical evidence for the shipped mesh model, not physical commissioning. The main operational problem is speed: a wall clamp can exceed both the **50 ms** policy-row budget and the **20 ms** bus-write budget. [Adversarial evidence](adversarial_summary.json), [directed evidence](directed_summary.json), [sim segment evidence](sim_independent_verification.json), [timing](benchmark.json).

The task serves goal B3: Dream-1 on the real waterproof arm. Gavin asked, “can we now build a bounding box to run dream1?” and allowed contact only with the aquarium floor. This work used saved files and pure geometry. It made no hardware connection or policy call. The guard, config and tests were left unchanged. No commit or push was made. [Scope and file checks](verification.json), [project intent](../../../../docs/viscous-haptics.md# aquarium-glass-protection-and-the-tank-measurement-2026-11-07).

## Numbers for the tank

The shipped planes are x **−694..40 mm**, y **−224..210 mm**, floor **−6 mm**, and rim **427 mm**. Fingers keep **30 mm** from walls and rim. Other moving links usually keep **70 mm**, including **40 mm** for cables. Targets may reach **z = −16 mm** because floor press is **10 mm**. `yaw_link` is exempt. The upper-arm near face has **0 mm** margin and padding. These exceptions remain in every variant. [Shipped config](../../../../configs/viscous/environments/aquarium_landen_75/keepout_viscous_waterproof_01.json).

The hand trace has **857 readings**, from **{trace["time_start_s"]:.3f} s** to **{trace["time_end_s"]:.3f} s**. The shipped guard accepts **540** and fails **317**. Zero margin and zero padding accept **725** and fail **132**. That variant retains floor press and the yaw exemption. Literal raw-plane crossings are listed below; floor press is removed only for that crossing calculation. Per-link/face counts and worst slacks are in [trace_summary.json](trace_summary.json).

{table(["Raw face", "Outside readings", "Worst raw slack (mm)", "Session time ranges (s)"], crossing)}

The whole mesh reaches **y = −255.978..218.503 mm** and **z minimum = −6.164 mm**. This reproduces the expected **8.503 mm** +y crossing and **31.978 mm** −y crossing. The trace records hand motion with a sliding base; these crossings do not show that the guard granted unsafe motion. [Trace series](trace_series.json), [trace summary](trace_summary.json).

![Whole-mesh y extremes and minimum z through the hand trace](trace_extremes.png)

## Recorded Dream-1 chunks

**Assumption:** the real executor presents a virtual base turned **180°**. Forward is therefore **−x** in `base_link`. Conversion is `motor_deg = (URDF_deg − offset_deg) / sign`, then **+180°** on shoulder_pan. Grip fraction **0** maps to **−2.92°** and **1** to **−115°**. z is unchanged. The motor map has a negative wrist-yaw sign. These are replay assumptions, not verified executor behavior. [Replay assumptions and sources](sim_summary.json), [motor map](../../../../configs/viscous/model/motor_to_urdf.json).

The inventory selects **{len(runs) // 2} primary Viscous runs**, with both arms checked separately: **{sum(counts.values()):,} policy rows**. Exact executor targets come from saved `execution` fields or older `trajectory.json` commanded rows. Archived copies, empty files and other robot models are excluded with reasons. Dry, scripted and replay sources are included and retain their path labels. [Inventory](sim_inventory.json), [all per-run numbers](sim_runs.md), [source hashes and detailed metrics](sim_summary.json).

All **{len(runs)} first poses** fail the start gate: **176** at forearm/near and **44** at forearm/rim. Each replay starts at the tests’ REST pose and requests a direct approach to row one in increments of at most **1°**. Only granted targets advance. This approach stalls at the rim. Policy totals are **{counts.get("ok", 0):,} ok**, **{counts.get("clamped", 0):,} clamped**, **{counts.get("rejected", 0):,} rejected**. The keep-out decisions bind forearm/rim; the remaining refusals come from the jump filter. The longest non-ok stretch is **{max(r["longest_non_ok_rows"] for r in runs)} rows = {max(r["longest_non_ok_s"] for r in runs):.1f} s**. These counts describe this required REST-and-ramp experiment. They do not prove that every later requested pose is unreachable by every possible approach. [Start gates, approach targets, reasons and counts](sim_summary.json).

The tabletop hand paths do **not** all fit. Only **{fit}/{len(nonempty)} arm streams** keep their full raw EEF request extent inside the raw six planes, even before applying a margin or checking the rest of the arm. Important left-arm examples follow. [All xyz extents and plane slacks](sim_summary.json), [readable per-run paths](sim_runs.md).

{table(["Run: left arm", "Policy rows", "Raw EEF x; y; z extents (mm)", "Past +y plane (mm)"], examples)}

Each run’s ok/clamped/rejected count, binding face, longest non-ok stretch, joint and probe error median/maximum, and timing mean/p99/maximum are in [sim_runs.md](sim_runs.md). The JSON additionally has each motor’s error, joint L2 error, start-gate slack and requested probe extents. Raw Dream EEF requests and FK fingertip probes use different tool points; they are reported separately. [Detailed metrics](sim_summary.json).

## Adversarial verification

The campaigns issue **{total:,} requests**: **{acounts["ok"]:,} ok**, **{acounts["clamped"]:,} clamped**, **{acounts["rejected"]:,} rejected**. They include uniform in-limit targets, local **±3° / ±19°** targets, J1 wraps of **±360° / ±720°**, IK-style flips, **0.25°** ramps, **0.02°** creep streams, floor dives, near-wall elbow requests, rim requests and opening fingers at a wall. [Random and creep campaigns](adversarial_summary.json), [directed campaigns](directed_summary.json).

For every ok or clamped segment, the checker calls only `tank_geometry.forward_kinematics` for transforms. It transforms every collision vertex into world coordinates. It independently reads margin, padding, plane and exemption rules from the config. It samples straight joint space at **≤0.05°**, including both endpoints, and compares those vertices with each plane. It never calls the guard’s clearance code for the verification verdict. The epsilon is **0.0000001 mm**. All sim policy grants and synthetic approach grants receive the same independent check: **{ind["verified_segments"]:,} segments**, **{ind["samples"]:,} sampled poses**. [Checker source](../../tank_guard_validation/validate.py), [sim checks](sim_independent_verification.json).

Directed tests bind near, left, right, floor and rim. Opening the fingers binds `gripper_left` against the right wall. Some long opening requests clamp only because the sweep proof reserves extra clearance; their slower ramps can remain clear. The folded-back near-wall attempt stops at wrist-pitch/rim before reaching the near plane. No far-face binding request was found. The closest sampled far clearance in **24,000** search poses was **{read("pose_pool")["face_endpoints"]["far"]["slack_mm"]:.2f} mm**, including the configured requirement. This search is not a proof of the arm’s global reach. [Directed details](directed_summary.json), [pose search](pose_pool.json).

On **3,000 random steps**, the motion bound covers true endpoint vertex displacement within the **0.0000001 mm** epsilon. The smallest bound-minus-displacement value is **{adv["motion_bound"]["min_bound_minus_true_mm"]:.3g} mm**, a floating-point rounding difference. No unsafe reproduction or correctness patch is needed for the tested inputs. [Motion-bound evidence](adversarial_summary.json).

## Room left by the box

The map uses **192,000 identical candidate poses** for all six settings. Half sample every motor uniformly within limits. Half concentrate J1 between **130°** and **187.529°**. Full collision meshes and all shipped exceptions are checked. This is a sampled endpoint map. It does not show that an allowed point is connected to REST, or give an exhaustive workspace volume. [Method and bins](workspace_summary.json).

{table(["Margin (mm)", "Cable padding (mm)", "Allowed sampled poses", "Fraction of candidates", "Occupied 10 mm xy cells"], wstable)}

At **30 mm** margin, changing cable padding from **40 mm** to **20 mm** adds **42.8%** more accepted sampled poses. Changing it to **0 mm** adds **84.8%**. Occupied xy cells rise from **1,388** to **1,646 / 1,794**. Each cell is **1 cm²**; this is sparse projected occupancy, not a measured floor area. Cable measurement could recover useful room. These variants are sensitivity tests, not permission to use zero padding. [Same-candidate comparison](workspace_summary.json).

![Allowed sampled fingertip positions for each margin and cable setting](workspace_topdown.png)

Sampled J1 angle envelopes for the forward branch (**J1 ≥90°**) follow. Reach is horizontal probe distance from J1; height is probe z. Cells show the outer edges of occupied **5°** bins, then the sample count in parentheses. A minimum-to-maximum envelope can contain holes. Full **5° J1 histograms** for every **50 mm reach × 50 mm height** bin are in [workspace_summary.json](workspace_summary.json), including the separate near-zero-angle branch.

{table(["Margin / padding (mm)", "Height (mm)", "Reach 200..250 mm", "Reach 400..450 mm", "Reach 550..600 mm"], j1rows)}

## Timing and proposed changes

The isolated benchmark runs **10 warmups + 150 repeats** per case. It times the guard step, without policy inference, FK reporting or the independent checker. A policy row has **50 ms** at **20 Hz**. A bus write has about **20 ms** at **50 Hz**. [Benchmark](benchmark.json).

{table(["Request", "Decision", "Mean (ms)", "p99 (ms)", "Maximum (ms)"], btable)}

A **10°** move on three joints can exceed the bus-write budget. The **19° J1 wall clamp** exceeds both budgets. The profile performs **26,950 pose checks** and **750 segment checks** for **50** clamp requests. Most time goes to mesh min/max reductions, then vertex projection and FK. The **14** bisection rounds repeat many checks. Fast jump rejections in the saved-run statistics hide this worst case. [Profile](profile_wall_clamp.json), [per-run timing](sim_summary.json).

Propose precomputing each link’s convex-hull vertex set and skipping fully exempt links before projection. A plane’s min/max is unchanged by this reduction. That would reduce vertices processed here from **9,059** to **2,794**. Cache or reuse subdivisions during bisection, and consider per-link motion bounds. Preserve the sweep proof and independently recheck every change. None of these changes was applied, and no speed-up factor is claimed. [Vertex counts](performance_proposal.json).

## Reproduce and limits

Use `PYTHONPATH=.` and `~/Documents/makermodslab-viscous/.venv/bin/python` from this worktree. Run `scripts/viscous-dream1/tank_guard_validation/validate.py` with stages `trace`, `sims`, `adversarial`, `workspace`, then run `directed.py`, `verify_sims.py`, and the `benchmark` stage without competing validation workers. Run `hull_counts.py` for the proposed reduction counts, then finish with `report.py`. Default workers are **12**. All data are read from saved files; none of these scripts imports Dream or a hardware driver. Plotting packages were installed only under `/tmp/tankbox/plot-deps` from the local offline wheel cache. Hull counting used SciPy under `/tmp/tankbox/analysis-deps`. The shared environment was unchanged. [Commands, versions, hashes and checks](verification.json).

The existing guard suite passes **15/15**. This run does not validate unmodelled cables, base motion, physical encoder signs, controller tracking or a downstream limiter that bends a straight joint segment. The documented sliding base must be fixed and the estimated near wall measured before a powered trial. The test’s virtual-base assumption also needs executor validation. [Checks](verification.json), [guard path contract](../../../../makermodslab/tank_guard.py), [measurement record](../../../../docs/viscous-haptics.md#aquarium-glass-protection-and-the-tank-measurement-2026-11-07).
"""
    report = report.replace("# aquarium-glass", "#aquarium-glass")
    (v.OUT / "report.md").write_text(report)
    print("wrote report.md and sim_runs.md")


if __name__ == "__main__":
    main()
