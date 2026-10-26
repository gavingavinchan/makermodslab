# Nut threading: lessons for the ROV episode

The sixth approach succeeded; the earlier starts exposed grasp, alignment, and clearance problems.
Use bolt-relative geometry. Re-measure the ROV scene; do not reuse tabletop poses, pixels, or gravity assumptions.

## What finally worked

- Grip opposing **outer flats**, with enough pad overlap to avoid catching the bore or inner lip.
  Successful openings were roughly 46–53 mm, varying with hex orientation; these are observations, not closure targets.
- Align the nut's actual bore with both the bolt and the wrist rotation axis. A straight wrist alone is insufficient.
  Depth checks after the final handoff put bore-to-wrist error near 0.5°. Recheck after releasing the other hand.
- Measure the exposed bolt center directly; the earlier center estimate was about 2 mm off sideways.
- Change roll in clear space to avoid neighboring fasteners. The final retry used a 150° relative roll change;
  later shifting the entry roll by about 35° put the screw/reset cycle in a usable arc. Find that arc anew.
- The successful start was `turn right 20 --feed-mm 0.194444`, followed by `screw right 1080`.
  That first screw call achieved only ~89° and two regrasp cycles before `lost_grasp`; it was not two turns.
- A 1 mm transverse shift switched the loaded finger; moving halfway back restored bilateral contact
  (~1.6/4.3 N). Later a ~0.5 mm correction likewise restored contact. Center gently instead of squeezing harder.
- After grip and roll corrections, `screw right 1080` completed ~1080°, 18 regrasp cycles, and ~10.5 mm advance.
  Further runs added ~866° and ~2000°, with grip corrections between them. Total advancement exceeded eleven turns.
  Only then did a final ~0.2° stall, together with flange seating in the images, support the snug conclusion.

## Failure sequence and prevention

1. Initial approach: large moves hit torque/joint limits; final pushes accumulated error, the 30° start barely
   rotated, and one jaw lost contact. Opening an unengaged grip and subsequent clearing motions led to drop #1.
   Stop at the first tracking/contact anomaly; retain or transfer support before opening.
2. First handoff: a 30° turn appeared successful, but screw advanced effectively zero; loads reached ~400–630 N
   and the opening collapsed to ~34 mm. The grip was wedged, with residual angular offset. A completed hand turn
   did not prove a thread start. Back off gently, support the nut, and reset the outer-flat grasp in clear space.
3. Second handoff: free rotation passed, but nut tilt, reach constraints, and slipping onto the inner edge caused
   another jam. The nut stayed at the tip with fingers open, then fell when the hand cleared: drop #2.
   A nut resting squarely on the bolt is not necessarily threaded; never use unsupported release as the test.
4. Deeper grasp: the wrist remained tilted, loading one finger; screw stalled immediately. Repeated opening,
   regrasping, and obstructed resets left the nut perched at the entrance. A clearance-tilt maneuver preceded
   drop #3, eventually under the flange. Do not tilt or sweep around an unsecured nut; fix alignment before contact.
5. Final handoff, first approach: bore alignment and low loads were much better, yet turns stopped at nearly
   the same wrist angle in either direction. Rotation worked away from the fixture: evidence of a blocked sweep,
   not a need for more torque. Withdraw while retaining the nut, change roll in free space, then realign.
6. Final retry: thread engagement progressed, but off-center regrasp and a narrow usable arc still caused stops.
   Correct contact and the entry roll; do not blindly repeat the same screw call or interpret every stall as snug.

## Observation and stop interpretation

- `look` before decisions and after every motion; inspect overview and wrist views. Use the other wrist view
  when occluded. `point` returns the visible surface, possibly a finger, bevel, or background through the bore.
- Sample opposing nut-face edges for center and at least three non-collinear points on the same flat face for
  its normal. Measure bolt tip/shaft and flange separately. Use fresh calibration after movement; `point` advances time.
- Track nut-to-tip and nut-to-flange displacement, bore concentricity, rotation, and bolt emergence through the nut.
  Expected axial advance is 3.5 mm/revolution: two turns ≈7 mm, three ≈10.5 mm. TCP feed alone is not proof.
- Read reached pose, translation/orientation residuals, `ik_hit_limit`, `stalled`, and saturated joints, even if `ok`.
  Small IK error does not rule out physical obstruction. Use `status` to check settling, opening, and jaw loads.
- `holding` is a bilateral-load signal, not object identity or engagement. A wedge can report holding; one pad
  can unload after a valid grasp and report empty. Combine images, actual opening, load history, and motion.
- `screw` reports hand rotation while gripped, not certified nut rotation. Read `degrees_turned`, `regrasp_cycles`,
  `stop_phase`, and `stop_reason`. Near-zero early `stall` means investigate alignment/grasp/clearance, not success.
- `lost_grasp`: inspect whether the nut is retained and recenter before reclose. `loaded_on_open`: fingers remain
  loaded/hooked; do not reset through that contact. With the nut securely supported, open and withdraw axially gently.
- `completed` means the requested stroke finished; verify physical advance. A late stall is seating evidence only
  with prior confirmed advance and a closed flange gap. Wait for long calls; use `last` after a disconnect.
- `timed_out` is incomplete work: inspect the reached state. Screw's 0.15 Nm roll budget is not a seating detector.

## First-try procedure

1. Inspect the new scene and confirm the held nut, bolt axis, exposed tip, flange face, and clear approach corridor.
2. Verify an outer-flat grasp and measure bore-to-wrist alignment in clear space. If a regrasp is needed, have the
   left hand securely hold the nut before releasing the right; verify the receiving grip before releasing the donor.
3. Choose reachable staging and entry poses with wrist margin and room for the fingers, open jaws, and resets.
   The TCP is between the original pads; fingertips extend about 34 mm along approach. Account for the whole hand.
4. Approach from the bolt-tip side toward the flange. Aim well inside the ~3 mm lateral / <10° angular limits,
   preferably submillimetre centering and near-parallel axes. Use 1–2 mm final nudges and verify actual movement.
5. Touch the leading face to the tip without pushing through resistance. Start with +20° / +0.1944 mm feed;
   +30° / +0.2917 mm is another short increment. Positive roll/feed tighten in this approach convention.
6. Inspect actual nut rotation and matching advance before authorizing screw's automatic releases. If uncertain,
   retain support and diagnose; back off with matched negative roll/feed if engaged, not a forceful straight pull.
7. Once started and securely gripped, use `screw right 1080`; it handles the 60° working window and wrist resets.
   Verify ≥720° of genuine nut rotation with matching advance, then continue in inspected batches toward seating.
8. Stop at a visually supported snug stall. Release only with confirmed thread support, verify unloaded jaws,
   withdraw axially, inspect the assembly once more, and call `done`. Underwater, assume no recovery table exists.
