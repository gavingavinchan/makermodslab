# Task: make the Astra harness's G3 start realistic and Astra-ready (no Dream calls, no Astra runs)

Work in `~/dreamtrial/viscous_sim` on gavin-linux, same venv/env. Read `runs/astra_flange/harness_report.md`,
`astra_op/README.md`, `astra_op/README_OPERATOR.md` and `astra_op/server.py` first. Same hard rules as
`BRIEF_ASTRA_HARNESS.md`: write only in `astra_op/`, `runs/astra_flange/`, `~/dreamtrial/astra_ws`; no Dream-1 calls,
no ledger or runner lock, no hardware, no propeller files, do not edit the original `flange_*.py` / `sweep.py`.

## Why (Gavin, 2026-10-25)
Gavin wants one clean video he can post saying the flange bolting is fully autonomous: "I don't really care how
it's done, go very slow, whatever, as long as I can post this online and say it is fully autonomous." Astra
(`gpt-6-astra`) will be the operator, using only cameras, depth and proprioception through `./robot`. Your harness
validation found three things that would make that video wrong or needlessly hard:
1. In trial-2 G3 the bolt sticks out only 3 mm, so two turns (7 mm) collide with the flange. A real stud sticks out
   past its nut; the start is the problem, not the operator.
2. The thread model winds the wrong way for the right-hand thread we claim. Anyone checking the video would see a
   left-hand nut.
3. The stock G3 nut grasp hooks the nut bore, so the operator must first discover and clear a hooked grasp.

## Changes (all inside `astra_op/`, behind flags; the trial-2 defaults stay reproducible)
1. `--g3-seat-mm`: choose a realistic Astra default so the protruding bolt end is at least nut thickness plus
   about two thread pitches beyond where the nut seats on the flange face, i.e. the nut can be run down until it
   is snug. State the resulting protrusion in mm. Keep the bolt head side clear of the flange (check contacts).
2. `--thread-hand right|legacy` (Astra default `right`): a real right-hand thread. Looking at the nut from the bolt
   tip, turning it clockwise drives it toward the flange. Implement it in an `astra_op` wrapper or subclass of the
   thread model, not by editing `flange_*.py`. Log the hand in `episode.jsonl`.
3. `--g3-grasp flats|stock` (Astra default `flats`): in G3 the right gripper holds the nut across two opposite
   flats, near its outer face, bore along the approach, so opening releases it cleanly. Same arm pose family as
   trial 2, clear of the flange.
4. Add a `nut seated` measurement to the hidden log (nut face in contact with, or within 0.5 mm of, the flange
   face), next to turns and advance.
5. Astra defaults: `--command-cap 600`, `--wall-seconds 14400` (4 h; Astra thinks slowly). Put all Astra defaults
   in one flag `--profile astra` so a launch is short; `--profile trial2` gives the old behaviour.
6. Update `README_OPERATOR.md`: the right-hand convention in terms of the `turn` sign and `--feed-mm` sign for a
   hand approaching from the bolt-tip side (derive it from the code and check it in sim), and that the episode
   succeeds when the nut is threaded at least two full turns, ideally run down snug. Remove the "opposite winding
   sign" caveat for the right-hand profile. Do not add object coordinates or hints about where the parts are.

## Validate
1. Unit checks for the new flags (thread hand: a clockwise-from-tip turn advances toward the flange; flats grasp:
   open releases with zero load).
2. The scripted CLI-only operator (privileged targets allowed, marked as a validation cheat) under `--profile
   astra`, G3: nut engaged, >= 2 turns, ideally seated. Video `runs/astra_flange/validation/g3_astra_profile/run.mp4`.
3. `--profile trial2` still reproduces the old G3 start (same initial poses as before, bit for bit if possible).
4. A launch smoke test: start the server with `--profile astra`, create the workspace, run `./robot look` and
   `./robot status` from inside the workspace, shut down cleanly.

## Deliverables
Short update at the top of `runs/astra_flange/harness_report.md`: the new flags and defaults with numbers (seat,
protrusion, grasp), the scripted result, and the exact command that launches an Astra episode (server plus
workspace path). Leave the server not running when you finish.
