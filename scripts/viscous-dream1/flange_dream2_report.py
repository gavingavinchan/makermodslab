"""Generate the flange rerun report from audited episodes and reviewed video notes."""
import json
from pathlib import Path
import numpy as np
import sweep

OUT=sweep.HERE/'runs/flange_dream2'
def run():
    audit=json.loads((OUT/'validation/episode_audit.json').read_text())
    plan=json.loads((OUT/'stages.json').read_text())
    notes=json.loads((OUT/'video_notes.json').read_text())
    episodes=[(p,json.loads(p.read_text())) for p in sorted((OUT/'episodes').rglob('log.json'))]
    assert len(episodes)==12 and all(e['finished'] for p,e in episodes) and plan['complete']
    script=json.loads((OUT/'scripted/log.json').read_text())
    def link(p):return str(p.relative_to(sweep.HERE/'runs'))
    def vec(values,digits=2):return '['+', '.join(f'{v:.{digits}f}' for v in values)+']'
    def pair(e,key,rating=None):
        v=e[key] if rating is None else e[key][rating]
        return 'L '+vec(v['left'],3 if rating else 2)+'; R '+vec(v['right'],3 if rating else 2)
    billed=audit['task_requests']['billed'];uncertain=audit['task_requests']['uncertain'];refusals=audit['task_requests']['pool_warming']
    success=sum(e['success'] for p,e in episodes)
    lines=['# Flange Dream-1 trial 2 — 2026-10-24','',notes['outcome'],'',
    'The flange scene now uses **±25 Nm on all six joints of each Viscous arm**, with the existing **±160 N generalized jaw drive and bilateral-contact-gated MuJoCo grasp welds**. Other Viscous tasks retain **±5 Nm / ±10 N**. Fixture layout, cameras, masses, arm JSONs and ideal thread implementation are unchanged. No hardware was used.','',
    'Holding is unchanged from [trial 1](flange_dream_report.md): CAD crank radius 30 mm, rod 42 mm and slider offset 16 mm give a maximum transmission Jacobian of 30.57 mm/rad; 5 Nm / 0.03057 m = 163.56 N, conservatively rounded to 160 N (about 80 N/jaw). This is an ideal linkage estimate without efficiency/calibration. Contact-gated welds preserve payload load and release above a normalized opening command of 0.65; attach/release events appear in every episode log. Local simulation bypasses hardware/Lab/lerobot limits.','',
    f'**Scripted gate: failed**, [video](flange_dream2/scripted/run.mp4). Insertion {script["max_bolt_inserted_depth_m"]*1000:.1f} mm, engagement {script["nut_ever_engaged"]}, turns {script["max_nut_turns"]:.3f}. Raising torque alone did not fix the coaxial head-grasp approach. I added bounded integral joint-tracking correction to compensate payload sag, and multistart IK fallback while rejecting discontinuous branch jumps. The corrected script still pushes the bolt against the outside of the flange and then carries it beside the joint; the nut approaches beside it without engaging. Both parts remain attached.','',
    'The remaining failure is **joint-limit/IK-branch alignment, followed by bolt/fixture collision**. At the required pre-entry TCP x=195 mm relative to the hole, the best of the sampled 13-start solves over six wrist rolls misses position by 11.33 mm and orientation by 11.0°. A precise inserted endpoint at x=118 mm exists on the opposite elbow/wrist branch (joint 1 ≈−3.105 rad, joint 5 ≈−1.534 rad), but backward continuation toward the entrance hits joint limits and loses alignment. The original branch also reaches wrist limits. The corrected run additionally reaches 25 Nm during the failed motion; higher torque does not repair its geometry. This sampled search does not prove a regrasp strategy impossible. [Reach search](flange_dream2/validation/reach_probe.json), [branch continuation](flange_dream2/validation/branch_probe.json), [initial torque-lift retry](flange_dream2/validation/initial_script.json).','',
    'Per Gavin’s fallback authorization, Part 2 ran despite this failed gate: **all four Stage-A conditions**, including G3 (bolt fixture-welded through both rings, right-hand nut) and G2. Dream-1 could have exceeded the script. Zero-shot `dreamlabs/dream-1-beta`, vision mode, Viscous B0, 30 calls × 48 rows at 20 Hz per episode; >=2 engaged nut turns at two consecutive endpoints would stop early. Stage A uses paired seeds 20261024–25; Stage B uses fresh paired seeds 20261026–27. Loose bolt/nut x/y jitter is ±20 mm; pregrasp target jitter is ±2 mm.','',
    '| Arm | Condition | Prompt | Episodes | Max insertion mm | Through-both eps | Nut engaged eps | Max turns | Calls | Peak joint Nm | >5 Nm s | >14 Nm s |',
    '|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
    prompts=__import__('flange_dream_sweep').PROMPTS
    for rig,condition in [('viscous','T'),('viscous','G1'),('viscous','G2'),('viscous','G3'),('yam',plan['winner'])]:
        group=[e for p,e in episodes if e['rig']==rig and p.parent.parent.name==condition]
        peak=max(v for e in group for a in e['arm_torque_max_abs_Nm'].values() for v in a)
        over={rating:max(v for e in group for a in e['arm_time_above_s'][rating].values() for v in a) for rating in ('5','14')}
        lines.append(f'| {"Viscous B0" if rig=="viscous" else "YAM vision"} | {condition} | {prompts[condition]} | {len(group)} | {1000*max(e["max_bolt_inserted_depth_m"] for e in group):.2f} | {sum(e["bolt_through_both"] for e in group)} | {sum(e["nut_engaged"] for e in group)} | {max(e["max_nut_turns"] for e in group):.3f} | {sum(e["calls_used"] for e in group)} | {peak:.2f} | {over["5"]:.3f} | {over["14"]:.3f} |')
    lines+=['','Torque summary columns show the largest peak and largest **single-joint per-episode** over-rating duration in that group. G3’s 74 mm through-both result is **initialization**, not policy insertion.','',
        f'**Stage B selected {plan["winner"]}** by maximum insertion depth, then engaged episodes, then turns. The G3 initialization dominates the depth criterion; this does not establish it as a successful learned behavior. Stage-B Viscous and YAM each received two fresh episodes. YAM uses the same fixture, prompts, seed jitter, holding rule and reference torque thresholds, retaining its stock ±40 Nm arm servos, 2 ms timestep and cameras as in the propeller control. Its two independent jaws are limited to 80 N each. Its right-hand pregrasp is fit to YAM’s joint limits and centered at its stock TCP; Viscous retains the original flange pregrasp. This limits an exact arm-only causal comparison.','']
    for condition in ('T','G1','G2','G3'):lines += [f'**{condition}:** '+notes[condition],'']
    lines += [notes['comparison'],'',f'Best/most informative video: [{notes["best"]}]({notes["best"]}). Worst video: [{notes["worst"]}]({notes["worst"]}). Every policy episode retains `log.json`, `chunks.jsonl` and a three-pane **H.264/yuv420p, 1920×400** `run.mp4` (top, inspection crop, left wrist). Call-1 sheets contain actual encoded policy JPEGs; the inspection crop is recording-only.','',
    '**Per-episode torque exposure.** Each vector lists joints 1–6, separately for L/R. Peaks sample absolute actuator torque at every physics substep; durations integrate the substeps above 5 Nm continuous and 14 Nm peak RS00 reference ratings. These totals cover executed action rows, excluding observation warmup. The same reference thresholds are shown for YAM without claiming they are YAM ratings. Full precision: [CSV](flange_dream2/episode_torques.csv).','',
    '| Episode | Max per joint Nm | Per-joint seconds >5 Nm | Per-joint seconds >14 Nm |',
    '|---|---|---|---|',f'| Scripted (32.5 s) | {pair(script,"arm_torque_max_abs_Nm")} | {pair(script,"arm_time_above_s","5")} | {pair(script,"arm_time_above_s","14")} |']
    for p,e in episodes:
        label='/'.join(p.parent.relative_to(OUT/'episodes').parts)
        lines.append(f'| {label} | {pair(e,"arm_torque_max_abs_Nm")} | {pair(e,"arm_time_above_s","5")} | {pair(e,"arm_time_above_s","14")} |')
    lines+=['',f'**Accounting:** {billed} billed calls, {refusals} unbilled pool-warming refusals, {uncertain} uncertain outcomes; ledger **1626 → {audit["ledger"]["budget_used"]} used**, cap **2212 unchanged**. Task ceiling 400; full plan ceiling 360. {success}/12 confirmed two-turn successes. [Stage plan](flange_dream2/stages.json), [episode/video/accounting audit](flange_dream2/validation/episode_audit.json).','',
    'All existing no-call suites pass: sweep/FK/clamp/budget, fake-client stop/resume, 18 baseline modes, one-arm/isolation, propeller, flange physics/holding/pregrasp/native IK/clearance/stub replay. The propeller check was rerun with a disposable runner lock because live flange work held the shared lock; real-ledger and network guards stayed active. New flange checks verify ±25/±160 isolation from other tasks, G3 fixture hold, row/episode torque totals and two-endpoint stop/resume. One real G3 episode per rig was replayed without inference with exact row/torque equality. All episode videos fully decode; scene-builder/thread source and protected JSON hashes match the snapshot. [Validation](flange_dream2/validation/).','',
    'Changed: `flange_holding.py`, `flange_start.py`, `flange_scene.py` (measurements only), `flange_pregrasp_demo.py`, `sweep.py`, `SWEEP_README.md`, ledger usage. Added: `flange_yam.py`, `flange_dream_sweep.py`, `flange_dream2_check.py`, `flange_dream2_audit.py`, `flange_dream2_replay.py`, `flange_dream2_report.py`, this report and `runs/flange_dream2/` artifacts/source snapshots. The first report is preserved.','',
    '**Measured in simulation:** trajectories, contact/attachment events, insertion, ideal thread turns/advance, torque peaks/exposure, call accounting and video decoding. **Estimated/unvalidated physically:** jaw transmission efficiency, friction, masses/inertias/calibration and ideal threading. The raised torque limit deliberately exceeds the real arm’s ratings; these results do not establish real-arm feasibility. Video behavior descriptions are visual interpretations of sampled frames.']
    (sweep.HERE/'runs/flange_dream2_report.md').write_text('\n'.join(lines)+'\n')
    print('wrote runs/flange_dream2_report.md')
if __name__=='__main__':run()
