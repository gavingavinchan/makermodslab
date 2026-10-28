Host-only shackle orchestration; never copy this directory or hidden logs/outcomes
into an operator workspace. See [LAUNCH.md](LAUNCH.md) for the reviewed launch.

`run_episode.py` owns `shackle_learn/.runner.lock`, checks only actual shackle
Python servers, checks Codex `stage1_outputs` read-only (must be zero), reserves
fresh output/workspace names, and starts `shackle_astra.py` directly. Workspaces
contain only README.md, robot, obs/, plus LESSONS.md if explicitly supplied. All
Codex model/effort/approval flags match the lessons A/B runner.

A four-hour default wall cap terminates the operator process group and requests
done; server exit includes deferred video export. Hidden scoring uses final
`seated` for success and reports final inserted/engaged, turns, signed collar-to-ear
seat gap, unique completed commands, simulation/operator wall seconds, pin drops,
end reason and final done claim. The seat gap is geometrical and alone does not
prove insertion (it can be negative while the loose pin stands elsewhere).
A drop is a >=10 mm descent during one continuous interval without grasp or
thread support. Re-support rearms the detector. Claim comparison is a wording
heuristic with a plain simulator-truth sentence, not a semantic guarantee.

Each output includes outcome.json, timeline.txt, audit.txt, workspace_audit.json,
run.mp4 and saved video states. Audit records initial/final file hashes and uses
a local read-only copy of the A/B's heuristic Codex tool/path audit. The operator
workspace boundary is instructional, matching the A/B; it is not an OS sandbox.
The robot server blocks network connections, Dream clients and accounting files.
It loads shared motion methods without editing the shared source, redirecting its
one import-time cache binding to shackle_learn/.cache.

Preparation evidence is in `runs/shackle_look/v2/`. `smoke.py` is a one-shot
integration test using fixed fresh names; both names have already been consumed.
`--verify-only` rechecks its stored evidence without starting any episode.
