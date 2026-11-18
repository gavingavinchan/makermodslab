<!--
# Copyright 2026 MakerMods. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
-->

# Waterproof Viscous · tonight at the arm

Allow about 20 minutes at the arm after printing and software setup. Claude can run the fit and inspect the results afterward. Use the physical waterproof follower for every reading.

1. **Prepare on the Mac.** Open the calibration branch in a separate worktree. Use the existing Lab Python 3.12 environment and the running local Lab at port 8000. Install the calibration requirement ahead of the session if SciPy is missing. Release the runner, teleop and CAN monitor. Support the arm with torque off throughout. The capture client only starts, polls and stops Lab's tank measurement.

   ```sh
   cd ~/wt/urdf-cal
   C=scripts/viscous-urdf-cal
   P="$HOME/Documents/makermodslab-viscous/.venv/bin/python"
   "$P" -c 'import scipy' || uv pip install --python "$P" -r "$C/requirements.txt"
   D=$(mktemp -d "$HOME/viscous-touch.XXXXXX")
   echo "$D"
   ```

2. **Place and measure (3 minutes).** Print the matching A4/A3 PDF at **Actual Size / 100%**. Laminate flat or use waterproof stock. Check the 100 mm ruler and five intervals along each axis (150 mm) after printing or lamination. Reprint if off by more than 0.5 mm. Fix the base and sheet to a level desk or tank floor. Measure the rough base-link X/Y of target **1** from the J1 axis. Measure Z to the bare floor and top of the sheet from the bottom of the arm base, including laminate thickness. Down the tank is normally base **−X**.

   To estimate J1, measure the signed heading of the first target row against a straightedge referenced to the physical mount’s **base_link X** datum. Use `atan2(ΔY, ΔX)` in degrees, CCW viewed from above. Aim within 0.3° (1 mm over 200 mm). If the datum is uncertain, leave the heading blank. The fit then estimates yaw and holds J1. It always fits sheet X/Y.

3. **Touch the grid (10–12 minutes).** Close the jaws lightly. Mark the midpoint between their two **distal ends**. Place that mark on each crosshair. If tilting would bring a side edge into contact first, use a narrow rigid post centred on each mark. For a constant-height post, add its height to sheet Z and use it on the block too. Touch the bare floor directly with the fingers pointing down.

   Choose 15–25 reachable IDs across the sheet. The example uses 15 on A4. Cover each twice with at least 8° elbow or wrist-pitch change. Vary wrist yaw and roll too. Start away from target 1. Move to the displayed ID, hold 3 s, listen for the save beep. A long pause in transit can record a false touch.

   ```sh
   # Run these lines in bash on the Mac. Use the Lab record for this mount.
   ROBOT=viscous_waterproof_01
   read -rp 'Origin base X and Y, mm: ' X Y
   read -rp 'Bare floor Z, sheet contact Z, block height, mm: ' FLOOR SHEET BLOCK
   read -rp 'Surveyed sheet +X heading in degrees (empty if unknown): ' YAW
   HEADING=()
   if [[ -n "$YAW" ]]; then HEADING=(--survey-yaw-deg "$YAW"); fi
   IDS=1,48,6,43,22,4,25,12,39,16,33,8,30,45,19
   "$P" "$C/capture.py" "$D/touch.jsonl" --robot "$ROBOT" --sheet A4 \
     --origin-mm "$X" "$Y" --floor-z-mm "$FLOOR" --sheet-z-mm "$SHEET" \
     "${HEADING[@]}" --watch "$IDS" --block-mm "$BLOCK" --level-targets 1,22,43
   ```

4. **Block, floor and roll (3 minutes).** Centre the measured block over each of the three requested marks. Touch a top crosshair directly above the sheet mark. Next touch three different bare floor spots. For the last three contacts, level the jaw-opening rail with a small level or equal ruler heights at its two ends. Keep the roll axis oblique to vertical. Without reliable level observations, omit `--level-targets`; J6 stays at its prior zero.

   **Manual entry** uses the same command without `--watch`, `--block-mm` and `--level-targets`. Enter `N`, `N@height_mm`, `floor` or `level:N` while holding still. Type `done` to release. If the base or sheet moves, start a fresh file. Preserve the raw data and document any excluded mislabelled rows.

5. **J7 (2–3 minutes).** At a consistent pad depth, measure five **full inside gaps** from gently closed through quarter, half, three-quarter and fully open. Enter each caliper reading in mm while holding still. Use the observed residual gap at closure.

   ```sh
   "$P" "$C/gripper.py" capture "$D/jaws.jsonl" --robot "$ROBOT"
   "$P" "$C/gripper.py" fit "$D/jaws.jsonl" "$D/jaws.pending.json"
   ```

6. **Fit and review.** Require **≤3 mm fitted and held-out RMS**. Inspect the worst points, signed floor Z, block and level residuals, sign trials and singular values in `residuals.json`. Positive Z means reported too high. Re-touch suspect IDs with a different posture. For a persistent pattern, check print scale, contact point, heights, heading and mount slip. Verify a suspect sign by hand. Only after those checks, try `fit.py "$D/touch.jsonl" "$D/fit-links" --links` for suspected link-length error. Failed quality checks produce diagnostics.

   ```sh
   "$P" "$C/fit.py" "$D/touch.jsonl" "$D/fit"
   # After a passing fit, generate the separate review bundle.
   "$P" "$C/make_tank_arm.py" "$D/fit" "$D/fit/tank_model"
   ```

7. **Approval and handoff.** Gavin checks fresh base-referenced contacts at working reach, especially downward error and the lowest jaw edge. Review held zeros, tool axes and historical rest in tonight’s mount. Supply the matching motor map, TCP, URDF and tank geometry to IK and the guard; the existing Dream runner loads its motor map separately. After approval redo `dreamcli tcp` in a fresh station/export with the generated URDF bytes and tool definition. [Station TCP walkthrough](../viscous-dream1/real/GAVIN_TCP_AND_INPUT_CHECK.md)
