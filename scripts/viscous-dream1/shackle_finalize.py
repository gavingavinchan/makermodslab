"""Collect scene evidence and write the short review handoff; no simulation server."""
import json,hashlib,subprocess
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'runs/shackle_scene'
def run():
 from shackle_scene import CENTER,PIN,TOP,MOUTH,SEAT,PITCH,PROMPT
 demo=json.loads((OUT/'scripted_log.json').read_text());reach=json.loads((OUT/'reach.json').read_text());legacy=json.loads((OUT/'validation/legacy/results.json').read_text())
 assert demo['success'] and demo['final']['seated'];assert reach['passed'];assert all(x['exit_code']==0 for x in legacy)
 changed=[];verified=[]
 for line in (OUT/'validation/protected_before.sha256').read_text().splitlines():
  old,path=line.split('  ',1);new=hashlib.sha256((ROOT/path).read_bytes()).hexdigest()
  (verified if new==old else changed).append(path)
 assert changed==['astra_op/server.py'],changed
 (OUT/'validation/protected_after.json').write_text(json.dumps(dict(unchanged=verified,changed=changed,server_change='early __main__-only shackle dispatch; original AST verified identical'),indent=2))
 import imageio_ffmpeg
 reader=imageio_ffmpeg.read_frames(str(OUT/'scripted_demo.mp4'));metadata=next(reader);reader.close()
 assert metadata['codec']=='h264' and metadata['pix_fmt'].startswith('yuv420p')
 (OUT/'validation/video_metadata.json').write_text(json.dumps(metadata,indent=2))
 rows=demo['rows'];active=demo['arm_torque_max_abs_Nm']['left'];native_max=max(r['native_position_error_m'] for r in reach['targets']);native_ang=max(r['native_angle_error_deg'] for r in reach['targets'])
 events=demo['grasp_events'];attaches=sum(x['event']=='attach' for x in events);releases=sum(x['event']=='release' for x in events)
 phase=[]
 for r in rows:
  if r['engaged'] and not r['seated']:
   q=r['pin_quat_wxyz'];theta=2*np.arctan2(q[3],q[0]);res=(r['pin_xyz'][2]-SEAT)-PITCH*theta/(2*np.pi);phase.append(float(abs(res-round(res/PITCH)*PITCH)))
 above={v:demo['arm_time_above_s'][str(v)]['left'] for v in (5,14)}
 summary=dict(success=True,final_seated=True,final_engaged=demo['final']['engaged'],final_gap_m=demo['final']['gap_to_seated_m'],final_turns=demo['final']['turns'],max_phase_residual_m=max(phase,default=0),simulation_seconds=demo['final']['time'],grasp_attaches=attaches,grasp_releases=releases,native_max_error_m=native_max,native_max_angle_deg=native_ang,peak_active_torques_Nm=active,time_above_5_Nm_s=above[5],time_above_14_Nm_s=above[14],legacy_check_groups=len(legacy),inference_calls=0)
 (OUT/'validation/final_summary.json').write_text(json.dumps(summary,indent=2))
 report=f'''# Shackle station — local review handoff

**The scripted table-start demo seated the pin and released it successfully. No billed calls, live Dream-1/Astra episodes, hardware, ledger access, or additional Astra servers.** [Video](scripted_demo.mp4), [front](overview_front.png), [top](overview_top.png), [insertion close-up](overview_detail.png), [exact policy JPEG contact sheet](contact_sheet.jpg).

Layout is metres in the inherited station frame. Table top **0.760 m**; shackle bore midpoint **({CENTER[0]:.3f}, {CENTER[1]:.3f}, {CENTER[2]:.6f})**; loose upright pin tip **({PIN[0]:.3f}, {PIN[1]:.3f}, {PIN[2]:.6f})**, **{np.linalg.norm(PIN[:2]-CENTER[:2])*100:.2f} cm** away. Upper ear seating face **{TOP:.6f} m**, tapped mouth **{MOUTH:.6f} m**, seated tip **{SEAT:.6f} m** (9.33 mm above table). Body/holder retain their exact designed relative transform, both jointless static children of world (permanent glue). Holder envelope 21 × 40 × 15.75 mm. Stainless pin is **240.73 g** at **7900 kg/m³**, with mesh-derived COM/inertia; shank 19.69 mm, thread major 20 mm, under-collar 79.85 mm, total 107.90 mm. Dimensions and 2.5 mm pitch remain provisional as documented in shackle_params.json.

**Configuration and looks.** Left Viscous active, right visible and exactly parked, original pad TCP, Maker wrist optics/mounts and true right wrist feed. Left starts at rest/open. Same configuration for both runners. Original table, top camera/crop, lights and material defaults are unchanged. Added inspection cameras only; stainless uses the flange steel's specular/shininess/reflectance with plain steel colour, holder matte teal PLA. Threaded OBJ normals load with `smoothnormal="false"`. All final stills are 1280×720; movie is H.264/yuv420p, top + inspection side by side (1280×400, 10 fps). Contact sheet contains the actual 320×176 JPEG observations with labels outside the images. The final medial layout exposes the shackle away from the arm silhouette; preliminary lateral layouts are archived under development/.

**Reach/contact.** Native ClientIK, local model metadata only: {len(reach['targets'])} targets covering shank grasp, vertical pin above clear ear, head at mouth/seat, and the full 120° tightening/reset wrist window at mouth and seat. Maximum native error **{native_max*1000:.3f} mm / {native_ang:.3f}°**. No hand/fixture contacts at the certified poses. [Reach results](reach.json). Existing 212 convex shackle cells are unchanged and keep both physical holes open at 20.60 mm; unengaged insertion is ordinary contact with no pose correction. Holder convex envelope stays outside the bore corridor. Pin uses invisible shank/thread/collar cylinders and a conservative tab box; its cross-hole is visual only.

**Thread and seat.** Entry requires actual tip in the tapped mouth, radial error <3 mm and axis error <10°. A swept 1 ms crossing detects the phase-correct catch plane, avoiding a missed catch during gravity settling. It never corrects the pin in the clear ear. Once caught, an ideal screw manifold projects axial/angular momentum at **2.5 mm/rev**, clockwise from head advances down. Shared visual phase is `z_tip = z_seat + pitch * spin_about_+Z/(2π)` modulo pitch; measured pre-seat residual ≤**{max(phase,default=0)*1e6:.3f} μm**. Only the first catch can apply a substep axial correction (recorded in the thread event); no arbitrary task-stage snapping. Assumed passive drag is the flange Astra harness's 0.020 Nm breakaway / 0.015 Nm sliding / 0.002 Nm·s/rad damping. Pulling beyond entry +3 mm or unscrewing beyond −0.1 turn releases. Collar arrival activates a finite 3 ms world-to-pin seat weld at the matching zero-spin phase; reverse deflection >0.5° releases that stop. Success requires the seat constraint latched and collar gap <0.5 mm. This is a feasibility surrogate, not calibrated thread friction, cross-threading, tightening torque or adhesive failure physics.

**Demo result.** Actual bilateral contact picks the pin by the upper shank/collar region, lifts/carries it vertically, inserts through the top ear, then opens. The pin settles into the phase-correct tapped mouth and remains supported there while the hand changes to a higher head grip. **120° wrist strokes with open/reset/reclose cycles** run it to the collar seat, then release it. Final **{demo['final']['turns']:.4f} turns**, **{demo['final']['gap_to_seated_m']*1e6:.3f} μm gap**, still engaged/seated after release; **{demo['final']['time']:.1f} s** simulation, {attaches} contact-gated attachments / {releases} releases. Script uses measured object poses for waypoint feedback (privileged feasibility, no inference), and never sets a free pin pose or moves the fixture. Initial constant-offset scripts jammed at the narrow clearance; low head grips/open jaws struck the bow; feedback alignment, a 6 mm higher head grip, and recentering before regrasp fixed the controller. [Full hidden rows and events](scripted_log.json).

**Forces.** Same flange 160 N ideal CAD jaw drive and contact-gated 2 ms grasp weld, released on opening >0.65. Six arm drives ±25 Nm in this task only. Active-joint peaks J1–J6: **{', '.join(f'{x:.2f}' for x in active)} Nm**. Time above real **5 Nm continuous**, per joint: **{', '.join(f'{x:.3f}' for x in above[5])} s**; above **14 Nm peak**: **{', '.join(f'{x:.3f}' for x in above[14])} s**. This is a sim feasibility result; the torque overrun prevents claiming the same motion works on the 5 Nm real arm. Every row logs pin pose/axis, insertion, engagement, turns, seat gap, contact pairs, attachment events and peak joint torques; these stay outside policy/operator observations.

**Dream-1.** New `sweep.py --task shackle`; instruction **“{PROMPT}”**. Parked/Maker configuration is forced for comparability. Existing Ledger reservation/finish, pool-warming handling, durable chunks and per-episode cap are reused for a later explicitly reviewed live launch; default cap 30, shackle-only `--max-calls 1..30`, seated at two consecutive call endpoints. The only run here was `shackle_dry_check.py`: two 48-row local hold replies, **96 hidden rows / 0 calls**, with client construction, remote networking and accounting/runner-lock access forbidden. Partial inserted/engaged/turns metrics are logged. [Dry log](dry/ep_00/log.json).

**Astra.** Additive `--profile shackle` dispatch in astra_op/server.py loads separate shackle_astra.py; existing profiles and all astra_learn/ files are untouched. Reuses original robot wrapper, native IK, sensed jaw latch, motion guards, depth and command protocol. Left commands only; pitch/feed is 2.5 mm; right physical state is projected to its park pose each tick. New [operator prompt](../../ASTRA_OPERATOR_PROMPT_SHACKLE.md) and SHACKLE_README_OPERATOR.md describe the task/cameras/robot commands without object poses or hidden task flags. The parser and separate code path were checked; **Episode was not instantiated and the server smoke test is deferred to Claude after the A/B has stopped**, avoiding its duplicate-server guard. Review launch (no operator inference by itself; output/workspace must be fresh, both inside this repo):

```bash
MUJOCO_GL=egl TMPDIR=$PWD/.cache/tmp PYTHONDONTWRITEBYTECODE=1 \\
  ~/dreamtrial/.venv/bin/python astra_op/server.py --profile shackle --condition T \\
  --episode REVIEW_NAME --output runs/shackle_scene/astra/REVIEW_NAME
```

**Regression and remaining limits.** All **{len(legacy)} legacy check groups passed** (apple/box, single/parked/isolation modes, propeller IK, flange physics/holding/reach/clearance/pregrasp/runner/torque, durable fake-client runner and 18 baseline modes). Old task XML hashes are byte-identical; original server AST is identical after removing only the new early dispatch; all protected source hashes except that dispatch match. [Checks](validation/). No existing scene/profile physics changed. Small real hardware is still challenging at Dream's 320×176 resolution; the policy top image cannot resolve the threads. Inspection renders/video show them. Clean uniform steel, conservative pin collision proxies, ideal grasp/thread/seat constraints, and assumed dry drag omit real surface damage, cross-threading, glue flex and measured tightening loads. Astra's new profile still needs its first safe server smoke after the A/B.

Reproduce locally: `shackle_demo.py --render`, `shackle_reach.py`, `shackle_render.py`, `shackle_dry_check.py`, `shackle_checks.py`, `shackle_legacy_checks.py`, then `shackle_finalize.py`, using the requested venv, EGL, repo TMPDIR and PYTHONDONTWRITEBYTECODE=1. No live flags are needed. New modules are shackle_scene/env/demo/render/reach/sweep/astra/checks/dry_check/legacy_checks/finalize; shared additions are cross_scenes.py, sweep.py and the server dispatch only. The requested makerwrist report filename was absent; runs/codex_viscous_makerwrist_report.md was read instead, alongside the parked-arm findings.
'''
 (OUT/'shackle_scene_report.md').write_text(report)
 print(json.dumps(summary,indent=2))
if __name__=='__main__':run()
