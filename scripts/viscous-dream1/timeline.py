import json, sys
prev = {}
for l in open(sys.argv[1]):
    r = json.loads(l); k = r.get("kind")
    if k == "physics":
        for e in r.get("thread_events") or []:
            print(f"{e['time']:7.1f}   THREAD {e['event']} turns={e.get('turns',0):.2f}")
    if k == "command_result":
        res = r.get("result") or {}
        flags = [f for f in ("stalled", "timed_out") if res.get(f)]
        sr = res.get("stop_reason")
        if flags or sr:
            print(f"{r['time']:7.1f} #{r.get('command_number')} {r['command'][:40]:40s} {flags} {sr or ''} deg={res.get('degrees_turned','')}")
    if k == "command_end":
        ga = (r.get("grasp_attached") or {}).get("right")
        if prev.get("ga") != ga:
            print(f"{r['time']:7.1f} #{r.get('command_number')} {r['command'][:40]:40s} right holds -> {ga}  turns={r.get('nut_turns') or 0:.2f} eng={r.get('nut_engaged')}")
        prev["ga"] = ga
