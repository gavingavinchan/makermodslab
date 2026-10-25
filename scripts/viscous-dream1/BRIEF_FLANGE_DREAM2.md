# Task: rerun the flange Dream-1 trial with the arm torque limits lifted

Same work dir, venv and env as `BRIEF_FLANGE_DREAM.md`. Read it and your report `runs/flange_dream_report.md`.
Everything in that brief still applies (Part 1 scripted pregrasp gate, then Part 2 Stage A/B, logging, videos,
report, hard rules), with ONE change:

## Change (Gavin, 2026-10-24): "don't worry about the torque limits"
Your Part 1 stopped because the insertion/nut poses need 7.8-9.4 Nm against the 5 Nm arm actuator limits. Gavin
says not to let that block the trial. In the **flange task only**, raise the six arm joints' actuator force limits
to +-25 Nm (keep the 160 N jaw drive and the contact-gated grasp welds you built). Other tasks keep +-5 Nm and
+-10 N exactly. Log per row the peak absolute torque of each arm joint, and report per episode the max per joint
and the time spent above 5 Nm and above 14 Nm (the RS00's continuous and peak ratings), so the gap to the real
arm stays visible. Do not change the fixture layout, cameras or masses.

If the scripted pregrasp demo still fails with the torque lifted, say exactly why (alignment, joint limits, IK
branch, collision) and fix the script, not the physics. If after a real effort it still cannot insert, run Part 2
anyway with G3 (bolt pre-inserted, nut held) and G2, since Dream-1 may do better than the script, and say so.

## Budget
Ledger cap is 2212 (raised already, do not raise again), 1626 used. Max 400 calls for this task, as before.

## Report
Write `runs/flange_dream2_report.md` (keep the first report), same shape, plus the torque columns.
