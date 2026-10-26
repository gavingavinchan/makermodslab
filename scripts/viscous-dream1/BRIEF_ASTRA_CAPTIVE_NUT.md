# Task: a nut deep on the thread must stay captive; find the mid-thread stall (no Astra runs)

Work on gavin-linux in `~/dreamtrial/viscous_sim`, same env and hard rules as `BRIEF_ASTRA_BACKUP_HOLD.md`.

## What happened (Astra episode `astra_rov_backup_g3_20261026_001`, profile `astra_rov_backup`)
Astra held the bolt head with the left hand and ran the nut 8.08 turns (advance 28.3 mm) with `screw`. Then:
1. From about t=505 s the nut stopped advancing about 5 mm short of the flange (gap 5.1 mm, turns 8.05 to 8.08
   during `turn right 20 --feed-mm 0.194444`); Astra reported "finger interference".
2. At t=516.5 s, during `grip right 0.15` (regrip), the thread model logged `release` at 8.08 turns and the nut moved
   about 25 mm sideways off the bolt (gap 9 mm to 46 mm in 0.5 s). A nut eight turns onto a bolt cannot leave it
   sideways. The pull-off gate (release after 3.5 mm pull behind the engagement point) still fires deep on the
   thread.
Astra then re-threaded and seated the nut (final gap 0.03 mm), but the video would show an impossible move.
See `runs/astra_flange/astra_rov_backup_g3_20261026_001*` (episode.jsonl, `_timeline.txt`, `_astra.log`).

## Fix
1. Captive thread: once engaged, the nut can only leave the bolt by unscrewing back past the tip (screw
   coordinate back to the entry), never by a radial or axial pull. Keep the existing tip behaviour (a nut that is
   only just caught can still be pulled off within the first pitch). Large forces must not break the constraint;
   if they would, the arm stalls (as with a real nut).
2. Find why the nut stopped advancing at a 5 mm gap at t=505 s (finger or jaw contact with the flange or studs,
   the nut hitting something, the seat stop firing early, the backup hold, or the screw feed). Fix it if it is a
   harness error; if it is real contact, say what touched what.
3. Apply to all profiles that use the right-hand thread (`astra`, `astra_rov`, `astra_rov_backup`).

## Validate
1. Replay episode `astra_rov_backup_g3_20261026_001` commands up to t=520 s on a fresh server: the nut stays on
   the bolt through the regrip at 516.5 s (report turns and gap).
2. Pulling a nut that is 3 turns on, sideways and axially with the arm: no release; the arm stalls.
3. Unscrewing it back off the tip releases it normally.
4. Scripted held G3 under `astra_rov_backup` seats the nut; `astra_rov` and tabletop `astra` scripted G3 still pass.
5. Short section at the top of `runs/astra_flange/harness_report.md`. Leave no server running.
