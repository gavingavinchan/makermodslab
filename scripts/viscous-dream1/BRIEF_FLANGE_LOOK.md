# Task: make the pipe-flange scene look like real subsea pipework (scene only, NO billed calls)

Work in `~/dreamtrial/viscous_sim` on gavin-linux, same venv/env as before. Read first: `BRIEF_FLANGE_SCENE.md`,
`runs/flange/flange_scene_report.md`, `flange_scene.py`, `flange_render.py`, `flange_demo.py`. Keep everything
that already works (thread model, IK checks, runner task `flange`, all no-call checks).

## Why (Gavin, 2026-10-23)
He looked at `runs/flange/overview_*.png`: "make the pipe longer and the flange look nicer. Look up an underwater
pipe flange." The Viscous arm is for ROV work, so the flange should look like subsea pipework, not a blue disc.
The scene stays on the table for now (moving onto the BlueROV2 is a later task). The task is unchanged: push a
bolt through the aligned holes and screw on the nut. He wants to see the environment before any billed run.

## What a real subsea bolted flange looks like (use these; search for photos yourself if you can)
- Weld-neck flanges: a tapered hub (neck) welded to the pipe, then the flange ring; raised face or RTJ (a ring
  groove with an octagonal metal ring gasket). Swivel-ring flanges are common subsea (the ring rotates to align
  holes for a diver/ROV). Refs: https://oilstates.com/?p=2421 ,
  https://www.offshore-mag.com/subsea/article/16756366/bolting-unit-allows-flange-tie-in-of-deepwater-pipelines ,
  https://epcland.com/rtj-flange-engineering-guide/
- Fastened with **stud bolts** (threaded rod with a heavy hex nut on each side), typically 8-12 per flange, not 4.
  Offshore studs and nuts are very often fluoropolymer coated (Xylan/PTFE, a distinctive blue, sometimes red),
  or hot-dip galvanized.
- Pipe: thick external coating (fusion-bonded epoxy, often red-brown or green, or 3-layer polyethylene, black),
  the coating cut back near the flange to bare/painted steel; flanges painted grey/yellow epoxy or bare steel with
  some corrosion staining. Optional: a bracelet anode on the pipe. Light wear, edge chipping and staining help
  realism; do not bury the holes or fasteners in grime.

## Changes
1. **Longer pipe**: each section clearly a length of pipeline, not a stub: say 0.5-0.7 m each, running across the
   table and allowed to overhang the table ends, on pipe supports (saddles/clamps). It must not collide with the
   arms at rest or along the task paths (rerun the IK reach checks and the clearance checks).
2. **Nicer flanges**: weld-neck hubs, raised face or RTJ with the ring gasket visible in the gap, 8 holes on the
   bolt circle, chamfered edges, smooth normals on curved surfaces, crisp hex nuts with chamfered corners. Install
   studs with two nuts in 7 of the 8 holes; leave the most reachable hole on the arm side empty for the task. The
   loose stud (or bolt; say which, a stud needs a nut on both ends so a bolt is simpler for this task) and nut
   lie on the table as before, sized as before (nut >= 4.5 cm across flats).
3. **Materials**: as above (coated pipe, painted/bare steel flanges, coated studs and nuts). MuJoCo materials only
   go so far: use textures (procedural PNGs are fine, committed under `assets/flange/`), specular/shininess/
   reflectance, and keep the table, lights and cameras of the Dream station unless something reads badly.
4. **Readable to the policy**: check the call-1 policy images. The empty hole, the loose bolt and the nut must be
   visible in the top camera and at least one wrist camera. If the top camera sees the flange edge-on, say so and
   propose (do not apply) a fix such as rotating the pipe run.
5. If time allows after 1-4: make the scripted demo pick up the bolt. The 650 g steel bolt stalled the gripper;
   say why (gripper force vs friction vs arm torque) and fix it in the most honest way (e.g. a grasp point or jaw
   friction, or a lighter part, stating its real-world equivalent). This is secondary; the look comes first.

## Deliverables (under `runs/flange_v2/`; keep `runs/flange/` as the before)
`overview_front.png`, `overview_top.png`, `overview_closeup.png` (1280x720, same cameras as before),
`before_after.png` (old vs new close-up), `contact_sheet.jpg` (call-1 policy images), `scripted_demo.mp4` if
item 5 was done (H.264 yuv420p), and a short `flange_v2_report.md`: what changed, dimensions, materials, which
references you used, IK/clearance results, what still looks wrong.

## Hard rules
No billed calls (no `--live`, no ledger access). No hardware. Only write inside `~/dreamtrial/viscous_sim`. Do not
edit installed packages, the Lab repo, `viscous_arm.json` or `viscous_arm_fingertip.json`. Never print the API key.
Do not touch `~/sonar-recon`. Existing tasks and modes behave exactly as before: rerun the no-call checks.
