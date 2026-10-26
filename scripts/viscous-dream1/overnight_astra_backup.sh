#!/bin/bash
# Overnight pipeline (2026-10-26): wait for Sol's backup-hold build, then run Astra two-handed on the ROV.
export PATH=$HOME/.nvm/versions/node/v22.17.1/bin:$PATH
cd ~/dreamtrial/viscous_sim
LOG=runs/astra_flange/overnight_20261026.log
say() { echo "$(date '+%F %T') $*" >> $LOG; }
say "start; waiting for Sol (pid 3825519)"
while kill -0 3825519 2>/dev/null; do sleep 60; done
say "Sol finished"
if [ ! -s astra_backup_codex_report.md ]; then say "ABORT: no Sol report"; exit 1; fi
if ! grep -q "astra_rov_backup" astra_op/server.py; then say "ABORT: profile astra_rov_backup not in server.py"; exit 1; fi
if [ ! -f runs/astra_flange/validation/rov_backup_g3/run.mp4 ]; then say "ABORT: no held validation video"; exit 1; fi
if pgrep -f "astra_op/server.py" >/dev/null; then say "ABORT: a sim server is already running"; exit 1; fi
EP=astra_rov_backup_g3_20261026_001; OUT=$PWD/runs/astra_flange/$EP; WS=~/dreamtrial/astra_ws/$EP
say "launch server $EP"
MUJOCO_GL=egl PYTHONDONTWRITEBYTECODE=1 TMPDIR="$PWD/astra_op/.cache/tmp" nohup setsid /home/gavin/dreamtrial/.venv/bin/python astra_op/server.py --scene rov --profile astra_rov_backup --condition G3 --seed 20261024 --episode $EP --output runs/astra_flange/$EP > ${OUT}_server.log 2>&1 < /dev/null &
SRV=$!
for i in $(seq 1 180); do grep -q '"ready"' ${OUT}_server.log && break; sleep 2; done
if ! grep -q '"ready"' ${OUT}_server.log; then say "ABORT: server not ready"; tail -20 ${OUT}_server.log >> $LOG; kill $SRV; exit 1; fi
cp ~/dreamtrial/astra_ws/lessons_from_006/LESSONS.md $WS/
say "launch Astra"
cd $WS
codex exec -m gpt-6-astra -c model_reasoning_effort="high" --dangerously-bypass-approvals-and-sandbox --skip-git-repo-check -C $WS -o ${OUT}_astra_final.md "$(cat ~/dreamtrial/viscous_sim/ASTRA_OPERATOR_PROMPT_BACKUP.md)" > ${OUT}_astra.log 2>&1 < /dev/null
say "Astra exited ($?)"
cd ~/dreamtrial/viscous_sim
if kill -0 $SRV 2>/dev/null; then
  if ! grep -q '"ended": true' ${OUT}_astra.log; then (cd $WS && ./robot done "Operator process ended without calling done" >> $LOG 2>&1); fi
fi
while kill -0 $SRV 2>/dev/null; do sleep 15; done
say "server exited; video exported"
python3 /tmp/timeline.py $OUT/episode.jsonl > ${OUT}_timeline.txt 2>&1
python3 - "$OUT" >> $LOG 2>&1 <<'PY'
import json, sys
rows = [json.loads(l) for l in open(sys.argv[1] + "/episode.jsonl")]
ph = [r for r in rows if r.get("kind") == "physics"]; ce = [r for r in rows if r.get("kind") == "command_end"]
last = ph[-1]
print("RESULT cmds", len(ce), "sim_s", round(rows[-1].get("time", 0), 1),
      {k: last.get(k) for k in ("nut_turns", "bolt_turns", "nut_seated", "nut_face_gap_m", "left_holds_bolt")},
      "min nut z", round(min(r["nut_xyz"][2] for r in ph), 3))
PY
F=$(grep -l "$EP" ~/.codex/sessions/2026/10/2*/*.jsonl 2>/dev/null | head -1)
[ -n "$F" ] && python3 /tmp/audit_astra.py "$F" > $OUT/audit.txt 2>&1 && echo "session $F" >> $OUT/audit.txt
say "done; final claim: $(cat ${OUT}_astra_final.md 2>/dev/null | head -3)"
