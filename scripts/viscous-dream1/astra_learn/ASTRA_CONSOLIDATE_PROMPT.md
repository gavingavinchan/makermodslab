You are revising your own practical notes for operating a two-arm nut-threading robot.
Work only in this directory. Do not read, search, or write any other directory; do not
run a robot or launch another agent. Read the three supplied files:
- LESSONS.md: the current notes, inherited from earlier episodes.
- previous_attempt.log: your own full operator transcript for one later episode.
- outcome.txt: simulator measurements computed afterwards by the experiment scorer.

Rewrite LESSONS.md in place. Revise rather than append: keep lessons still supported,
correct advice this episode contradicted, merge duplicates, and remove obsolete claims.
Keep useful earlier recovery/alignment guidance while prioritizing reliable first attempts.
Mark each section or lesson as General, Tabletop, ROV one-hand, or ROV two-hand; distinguish
profile/scene-dependent behavior from general advice. Do not transfer privileged simulator
measurements into claims that the live operator can observe them. These notes are advisory:
the next workspace's README is authoritative about current commands and capabilities.

Treat outcome.txt as ground truth over your own impressions in the transcript. For example,
if you claimed "seated" but the simulator measured a 2 mm flange gap, correct your seating
criterion and explain how a collision or loaded wrist can mimic a snug torque stall.
Distinguish >=2 relative nut-minus-bolt turns (task success) from seating and from hand
rotation or world nut rotation. Report uncertainty honestly; do not certify seating from
stall alone. Outcome evidence establishes what happened, not necessarily why; mark inferred
causes as hypotheses unless supported by the transcript. With a freely turning bolt,
account for backup-head retention and common rotation. Preserve scene clearance cautions.

Use at most 100 lines, preferably 70–90, of concise Markdown. No absolute coordinates,
pixels, full poses, quaternion/vector tuples, joint angles, or reproducible scene-specific
pose recipes. Relative distances, offsets, jaw openings, loads, rates, relative turn/feed
increments and physical thread pitch are fine. Do not embed raw logs or full outcome JSON.
Write only LESSONS.md; your final response should briefly describe the substantive revisions.
