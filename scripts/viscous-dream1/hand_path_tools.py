"""Read-only instrumentation of Dream's client IK and shared offline diagnostics."""
import json

import numpy as np
from dream_ik import decode

SIDES = ("left", "right")


def plain(value):
    return json.loads(json.dumps(value, default=lambda v: np.asarray(v).tolist()))


def apple_in_base(env, side):
    try:
        env.model.body(f"{side}_base")
        name = f"{side}_base"
    except KeyError:
        name = f"{side}_base_link"
    base = env.data.body(name)
    try:
        fruit = env.data.body("apple")
    except KeyError:
        fruit = env.data.body("cube")
    return base.xmat.reshape(3, 3).T @ (fruit.xpos - base.xpos)


class IKTrace:
    """Wrap instance methods; return the exact objects the original methods returned."""

    def __init__(self, ik):
        self.ik = ik
        self.anchor = None
        self.context = None
        observe, solve = ik.observe, ik.solve

        def observed(observation):
            result = observe(observation)
            self.anchor = plain({k: v for k, v in result.items() if "_eef_" in k})
            return result

        def solved(action, observation, smoother, rtt_s, commanded):
            histories = {}
            count = len(action["per_key"]["action.left_eef_xyz"])
            for side in SIDES:
                key = f"q_{side[0]}_chunk"
                measured = np.asarray(observation[f"state.{side}_arm"])
                histories[side] = smoother.takeover_history(
                    key, (count, ik.wires[side].joints + 1), measured, rtt_s,
                    None if commanded is None else commanded.get(key))
            self.context = plain({"rtt_s": rtt_s, "solve": ik._solve_spec().solve,
                                  "observation_joints": {s: observation[f"state.{s}_arm"] for s in SIDES},
                                  "takeover_history": histories})
            result = solve(action, observation, smoother, rtt_s, commanded)
            self.context["before_seam_joint_chunk"] = plain({k: result[k] for k in ("q_l_chunk", "q_r_chunk")})
            return result

        ik.observe, ik.solve = observed, solved

    def chunk(self, call, reply, env, rows):
        return {"call": call, "anchor": self.anchor, "per_key": plain(reply["per_key"]),
                "solved_joint_chunk": plain({k: reply[k] for k in ("q_l_chunk", "q_r_chunk")}),
                "executed_rows": rows, "client_ik": self.context,
                "apple_base_at_request": {s: apple_in_base(env, s).tolist() for s in SIDES}}


def tracking(ik, per_key, left, right):
    """FK of commands after client seam and executor clipping, against raw and filtered requests."""
    result = []
    for side, rows in zip(SIDES, (left, right)):
        requested = decode.head_poses(np.asarray(per_key[f"action.{side}_eef_xyz"]),
                                      np.asarray(per_key[f"action.{side}_eef_rot6"]))
        filtered = decode.smooth_poses(requested, ik._smooth.smooth_window, ik._smooth.smooth_polyorder)
        fk = ik._solver.fk(ik.wires[side].spec(rows[:, :6]), side)
        errors = {}
        for label, targets in (("raw", requested), ("filtered", filtered)):
            targets = targets[:len(rows)]
            relative = np.swapaxes(targets[:, :3, :3], -1, -2) @ fk[:, :3, :3]
            errors[label] = (np.linalg.norm(fk[:, :3, 3] - targets[:, :3, 3], axis=1),
                             np.degrees(np.arccos(np.clip((np.trace(relative, axis1=-2, axis2=-1) - 1) / 2, -1, 1))))
        for i in range(len(rows)):
            if len(result) <= i:
                result.append({})
            result[i][side] = {"fk_xyz": fk[i, :3, 3].tolist(),
                               "requested_xyz": requested[i, :3, 3].tolist(),
                               **{f"{label}_{metric}": float(values[j][i]) for label, values in errors.items()
                                  for j, metric in enumerate(("position_error_m", "rotation_error_deg"))}}
    return result


def clip_viscous_grips(reply, adjustments):
    """Keep raw per_key intact; clip only the executor's copy of joint/grip rows."""
    result = dict(reply)
    for side in SIDES:
        key = f"q_{side[0]}_chunk"
        rows = np.array(reply[key], dtype=float, copy=True)
        grip = rows[:, 6].copy()
        rows[:, 6] = np.where(np.isfinite(grip), np.clip(grip, 0, 1), grip)
        if np.any(grip != rows[:, 6]):
            adjustments.append(f"{side} gripper: clipped {np.count_nonzero(grip != rows[:, 6])} values "
                               f"from [{grip.min():.6f}, {grip.max():.6f}] to [0, 1]")
        result[key] = rows
    return result
