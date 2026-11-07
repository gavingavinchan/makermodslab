# Copyright 2026 MakerMods. All rights reserved.
"""The Viscous two-arm body as Dream's fine-tune service registers it (`DreamClient.add_embodiment`).

Why this shape (read from the installed client, `shared/datasets/embodiments.py`, 2026-11-06):

- A fine-tune that keeps the base model's end-effector head (`use_eef`) needs BOTH arms' grasp-point position, its
  rotation and the gripper in state AND action; a single-arm EEF body is not supported, so the parked right arm is
  recorded too.
- Registering a URDF instead, with joint actions, trains joints only ("the gripper poses come from the joints, which
  is not built yet"), losing the base's hand-pose prior. So the poses are recorded explicitly and no URDF is named.
- Joints ride beside the poses in both vectors (as YAM's EEF data has them): state reads them, and arm targets train
  relative to the same arm's joints, which is why the state must carry them at the same width.
- Widths: the trained action is 2 x (6 arm + 1 grip + 3 xyz + 6 rot6) = 32, under the base's 34-wide action head;
  the trained state 2 x (6 + 1 gripper + 1 grip + 3 + 6) = 34, under 64.
- Pose convention (the base model's): xyz is the grasp point (between the closed fingertips) in metres, rot6 the
  tool frame's first two rotation-matrix columns `[R00, R10, R20, R01, R11, R21]`, both in that arm's own base frame.
  The parts parser cannot check this; compute them with dream_ik's FK on the same arm file the client uses, so the
  training poses and the inference poses are the same numbers.

Per-arm vector layout (left first, then right), identical in state and action:
  arm 6 | grip 1 | eef_xyz 3 | eef_rot6 6   = 16 per arm, 32 in all.

The parts are named exactly as `EEF_PARTS` (`<side>_grip`, `<side>_eef_xyz`, `<side>_eef_rot6`): the client's
`finetune(use_eef=True)` checks those literal names before queueing, beside the service's own plan.
"""

ARM_JOINTS = 6
PER_ARM = ARM_JOINTS + 1 + 3 + 6
SIDES = ("left", "right")

# Gripper units as recorded: the sim and the dream client both speak the 0-closed-to-1-open opening, so the data is
# written in those units. Revisit for real Lab recordings, whose gripper is in motor degrees.
GRIP_CLOSED, GRIP_OPEN = 0.0, 1.0


def _spans() -> dict[str, list[int]]:
    spans, start = {}, 0
    for side in SIDES:
        for part, width in (("arm", ARM_JOINTS), ("grip", 1), ("eef_xyz", 3), ("eef_rot6", 6)):
            spans[f"{side}_{part}"] = [start, start + width]
            start += width
    return spans


def _answers() -> dict[str, dict]:
    answers = {}
    for side in SIDES:
        answers[f"{side}_arm"] = {"kind": "arm", "side": side}
        answers[f"{side}_grip"] = {"kind": "gripper", "side": side, "closed": GRIP_CLOSED, "open": GRIP_OPEN}
        answers[f"{side}_eef_xyz"] = {"kind": "position", "side": side, "point": "grasp", "frame": "arm_base"}
        answers[f"{side}_eef_rot6"] = {"kind": "rotation", "side": side, "format": "rot6_cols"}
    return answers


def names() -> list[str]:
    """One name per vector value, in order, for `meta/info.json`'s `names`."""
    out = []
    for side in SIDES:
        out += [f"{side}_joint_{i + 1}" for i in range(ARM_JOINTS)]
        out += [f"{side}_grip"]
        out += [f"{side}_eef_{a}" for a in "xyz"]
        out += [f"{side}_eef_rot6_{i}" for i in range(6)]
    return out


def config(cameras: dict[str, str]) -> dict:
    """The embodiment config. `cameras`: role (`overhead`, `wrist_l`, `wrist_r`; at most 3) to the dataset's video key."""
    spans = _spans()
    return {
        "state": spans,
        "action": dict(spans),
        "video": dict(cameras),
        "head_camera": "overhead",
        "parts": {"state": _answers(), "action": _answers()},
        "actions": "absolute",
    }


DEFAULT_CAMERAS = {
    "overhead": "observation.images.overhead",
    "wrist_l": "observation.images.wrist_l",
}
WIDTH = 2 * PER_ARM
