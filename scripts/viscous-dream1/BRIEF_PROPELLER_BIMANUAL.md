# Four-blade propeller: environment first (2026-10-24)

## Why

Goal B3a in `docs/viscous-haptics.md`, revisiting the unsatisfactory three-blade
zero-shot trial. Gavin wants a recognisable ship-propeller cleaning setup before
another policy attempt: retain a desk for Dream-1, use four blades pointing up,
down, left and right, and work on the horizontal left blade. The left arm grabs
its outer edge; the right arm scrubs that same blade. He permits Codex Astra to
help if Dream-1 does not work. Start with the environment, references and a
reviewable preview. No new inference budget is established by this brief.

## Video references

Found and visually inspected representative frames in the browser on 2026-10-24:

- [Diving Status — Propeller Polishing Rupert scale A at containership](https://www.youtube.com/watch?v=IlGRVwOnO_s).
  56.7 seconds; inspect approximately 0:15 and 0:32. The wider view at 0:32 shows
  a diver holding a circular polishing disc against a curved blade face, with
  hoses leading away from the tool. Best reference here for blade/tool scale
  and the contact orientation.
- [Karsa Underwater Company — Propeller cleaning with 3M polish](https://www.youtube.com/watch?v=nzaHLRLyV10).
  24.7 seconds; inspect approximately 0:10. Close view of the hand-held tool and
  circular abrasive head. Linked as propeller cleaning on the operator's
  [video page](https://www.karsamarineservice.com/video).
- [Karsa — Part 2](https://www.youtube.com/watch?v=BYl_egHTXto).
  23.5 seconds; inspect approximately 0:12. Additional view of the diver and
  tool beside the propeller. Secondary reference; more body occlusion.

These are real commercial underwater footage, not demonstrations of our exact
two-arm arrangement. The left-edge hold is Gavin's chosen robot task. References
support a curved bronze blade and a compact polishing pad in sustained surface
contact; a bench simulation can first use a passive abrasive pad and reciprocal
strokes. Do not claim it simulates motorised abrasion, fouling removal or
underwater hydrodynamics. No third-party footage is vendored into the repo.

## Scene and review gate

Keep the established table, two Viscous arms and three Dream camera streams.
Use a fixed horizontal shaft and a vertical four-blade propeller on a stand.
Define left in the arms' work view and make that convention visible in the
inspection render. Both hands must reach the same blade simultaneously without
the right wrist colliding with the left hand or hub. Show a neutral overview,
the ready pose, and all three actual policy images. A compact graspable cleaning
pad is closer to the references than the old kitchen sponge.

Check geometry, actual collision contacts, unwanted penetrations, tool retention
and a short scripted scrub if feasible. Label all initialized grasps or ideal
attachments. A scripted demonstration proves environment feasibility only;
it is not a Dream-1 result. Show Gavin the setup before any billed trial.

Simulation only. Do not open hardware or alter installed packages. Preserve the
old propeller experiment and concurrent flange work. Save assets, code,
instructions and small evidence under the repository with relative paths.
