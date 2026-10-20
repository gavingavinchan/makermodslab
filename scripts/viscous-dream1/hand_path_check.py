"""Offline regression: instrumentation is transparent, FK errors align, grips clip, replay fixture."""
import yam_baseline as baseline
import json
from pathlib import Path

import numpy as np
from cross_scenes import ViscousAppleEnv
from dream.client_ik import ClientIK, POSE_KEYS
from dream.models import Model
from dream.smoothing import SeamSmoother
from hand_path_tools import IKTrace, clip_viscous_grips, tracking


def main():
    out = Path("runs/hand_path_check")
    out.mkdir(exist_ok=True)
    model = Model(baseline.BETA_MODEL, "public", baseline.BETA_MODEL, "yam", True, None)
    ik = ClientIK({"arm": str(Path("viscous_arm.json").resolve()), "head": "eef"},
                  20, baseline.YAM_EMBODIMENT, model)
    assert ik.traj_ready.wait(120)
    env = ViscousAppleEnv()
    state = env.state()
    obs = {f"state.{s}_arm": state[s][:6] for s in ("left", "right")}
    obs.update({f"state.{s}_grip": state[s][6:] for s in ("left", "right")})
    original_observe, original_solve = ik.observe, ik.solve
    anchor = original_observe(obs)
    action = {"per_key": {f"action.{s}_{part}": np.repeat(
        np.asarray(anchor[f"state.{s}_{part}"])[None], 48, axis=0).tolist()
        for s in ("left", "right") for part in ("eef_xyz", "eef_rot6")}}
    action["per_key"].update({f"action.{s}_grip": [[1.]] * 48 for s in ("left", "right")})
    sm1, sm2 = SeamSmoother(20, pose_keys=POSE_KEYS), SeamSmoother(20, pose_keys=POSE_KEYS)
    before = original_solve(action, obs, sm1, .5, None)
    trace = IKTrace(ik)
    observed = ik.observe(obs)
    for key in trace.anchor:
        np.testing.assert_array_equal(observed[key], anchor[key])
    after = ik.solve(action, obs, sm2, .5, None)
    for key in ("q_l_chunk", "q_r_chunk"):
        np.testing.assert_array_equal(before[key], after[key])
    reply = sm2.smooth(obs, after, 0, .5, None)
    record = trace.chunk(1, reply, env, 48)
    (out / "fixture.jsonl").write_text(json.dumps(record) + "\n")
    left, right = baseline.validate_chunk(env, reply)
    errors = tracking(ik, action["per_key"], left, right)
    assert len(errors) == 48 and np.isfinite(errors[0]["left"]["raw_rotation_error_deg"])
    invalid = dict(reply)
    invalid["q_l_chunk"] = np.asarray(reply["q_l_chunk"]).copy()
    invalid["q_l_chunk"][0, 6] = -.089
    notices = []
    clipped = clip_viscous_grips(invalid, notices)
    assert clipped["q_l_chunk"][0, 6] == 0 and invalid["q_l_chunk"][0, 6] == -.089
    assert notices and clipped["per_key"] is invalid["per_key"]
    baseline.validate_chunk(env, clipped)
    print("PASS: anchor/solve unchanged; 48 aligned FK rows; negative grip clipping preserves raw reply; no network")
    env.close()


if __name__ == "__main__":
    main()
