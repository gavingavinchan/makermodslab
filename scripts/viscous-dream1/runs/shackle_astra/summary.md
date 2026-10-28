# Shackle Astra learning chain

Status: stopped: first seated success; updated 2026-10-28T05:21:17.765466-05:00.

Success means final simulator seating. s001 derives from v006 + the baseline below.
No starts at/after 2026-10-28 10:30 CDT; stop at 11:30 CDT. At most three new episodes.
Ever states and claims are post-hoc; the claim comparison includes a wording heuristic.

| Episode | Lessons file | Success | Seated | Ever inserted | Ever engaged | Turns | Drops | Commands | Sim s | Wall s | Ended by | Claim vs truth |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| shackle_L006_20261028_001 | astra_learn/lessons/v006.md | false | false | false | false | 0.000 | 2 | 263 | 397.250 | 1881.441 | done | Claim: Incomplete: pin entered the upper ear but was lost during regrasp recovery; final views show empty ears. Left arm parked, right arm unchanged.; No disagreement detected in the final claim (wording heuristic). Simulator: pin not seated, inserted=False, engaged=False, turns=0.000. Upper-ear entry claim unsupported: ever_inserted=false (heuristic missed it). |
| shackle_S001_20261028_001 | shackle_learn/lessons/s001.md | true | true | true | true | 8.038 | 2 | 494 | 946.850 | 3290.614 | done | Claim: Used the left arm to insert the pin through the upper ear, screw it clockwise into the lower ear, and seat its collar on the upper ear; released the pin and visually verified seating with the gripper clear.; No disagreement detected in the final claim (wording heuristic). Simulator: pin seated, inserted=True, engaged=True, turns=8.038. |

Progress: `tail -n 30 runs/shackle_astra/chain.log`
