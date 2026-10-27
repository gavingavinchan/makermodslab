# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0. Run from the repo root on the machine with unit 1.
"""Read-only: record the follower and Star gripper travel while a human moves them. Energizes nothing."""
import importlib.util, sys, time, serial, json
sys.path.insert(0, ".")
spec = importlib.util.spec_from_file_location("g", "scripts/gripper-haptics-poc.py"); g = importlib.util.module_from_spec(spec); spec.loader.exec_module(g)
from makermodslab.fashionstar_raw import RawBus, MONITOR
bus, (lo, hi), _ = g.follower_bus(g.CAN_PORT)
bus.connect(handshake=False)
raw0 = float(bus.read("Present_Position", "gripper")); turn = g.turn_offset(raw0, lo + 2, hi - 2)
print(f"turn offset {turn:+.0f}; follower start {raw0 + turn:+.1f}", flush=True)
star = RawBus(serial.Serial(g.STAR_PORT, 1_000_000, timeout=0.05, write_timeout=0.05, exclusive=True))
fmin = fmax = smin = smax = None; rows = []; t0 = time.monotonic(); phase = None
try:
    while (t := time.monotonic() - t0) < 50:
        p = "FOLLOWER: close the follower jaws fully by hand, then open fully, 2-3 times" if t < 25 else "STAR: close the Star gripper fully, then open fully, 2-3 times"
        if p != phase: phase = p; print(f"EVENT {t:5.1f}s {p}", flush=True)
        bus._batch_refresh(["gripper"])
        f = float(bus._last_known_states["gripper"]["position"]) + turn
        s = star.request(MONITOR, 6).angle_deg
        rows.append((round(t, 3), f, s))
        changed = False
        if fmin is None or f < fmin - 0.5: fmin = f; changed = True
        if fmax is None or f > fmax + 0.5: fmax = f; changed = True
        if smin is None or s < smin - 0.5: smin = s; changed = True
        if smax is None or s > smax + 0.5: smax = s; changed = True
        if changed and int(t * 2) % 2 == 0: print(f"RANGE {t:5.1f}s follower {fmin:+.1f}..{fmax:+.1f}  star {smin:+.1f}..{smax:+.1f}", flush=True)
        time.sleep(0.05)
finally:
    star.close(); bus.disconnect(disable_torque=False)
    fol = [r[1] for r in rows if r[0] < 25]; st = [r[2] for r in rows if r[0] >= 25]
    out = {"follower_range_while_moving_follower": [min(fol), max(fol)], "star_range_while_moving_star": [min(st), max(st)],
           "overall": {"follower": [fmin, fmax], "star": [smin, smax]}, "turn_offset": turn, "follower_soft_limits": [lo, hi]}
    json.dump({"summary": out, "rows": rows}, open(f"configs/viscous/viscous_01/evidence/gripper-range-{time.strftime('%Y%m%dT%H%M%S')}.json", "w"))
    print("FINAL", json.dumps(out), flush=True)
