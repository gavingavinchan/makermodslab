# Astra lessons A/B

Fixed profile astra_rov_backup2, seed 20261024, G3; 240 min operator cap.
L uses fixed v001 SHA256 `336d0acffa31303cd38efb966403d7682f9cb852db132ca13bc9e6295f8b66be`; N has no LESSONS.md.

| Episode | Arm | Success | Seated | Commands | Sim s | Wall s | Drops | Ended by | Claim mismatch |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| astra_ab_L1 | L | True | False | 157 | 532.50 | 1115.64 | 0 | done | — |
| astra_ab_N1 | N | False | False | 846 | 1191.70 | 14400.19 | 3 | wall_cap | — |
| astra_ab_L2 | L | True | True | 180 | 513.39 | 2373.80 | 0 | done | claimed more than ten turns; sim 9.42 relative turns. |
| astra_ab_N2 | N | True | True | 429 | 1460.93 | 9098.55 | 1 | done | — |
| astra_ab_L3 | L | True | True | 127 | 489.14 | 1612.90 | 0 | done | — |
| astra_ab_N3 | N | True | True | 94 | 525.35 | 1280.39 | 0 | done | — |

| Arm | Successes | Seated | Median commands | Median sim s | Median wall s | Total drops | Claim mismatches |
| --- | --- | --- | --- | --- | --- | --- | --- |
| L | 3/3 | 2 | 157 | 513.39 | 1612.90 | 0 | 1 |
| N | 2/3 | 2 | 429 | 1191.70 | 9098.55 | 4 | 0 |

Medians include every recorded episode with that metric, including failures/caps; missing crash metrics are excluded.
