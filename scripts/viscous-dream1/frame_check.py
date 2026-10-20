"""Check the Viscous arm file's hand frame against the frame Dream-1 was trained in, with dream's own IK.

Dream's convention (dream/facade.py, dream/arm_file.py): the base is x forward, y to the arm's left, z up; the tool
frame is z = approach (the way the fingers point), x = jaw (the line the fingers close along, pointing to the arm's
left when the hand points straight forward). On `other_embodiment` the model reads none of the state, and its answer
is "composed on the hand pose the request carries", so a hand frame that disagrees with YAM's turns every motion the
model means into a different one.

Prints, for YAM (dream's reference), the Makers arm (the officially supported cousin) and the Viscous arm file:
the tool pose at rest and at the start poses the trial used, and each arm's axes at a hand-forward pose.
No network, no hardware. Run with the trial venv from the directory holding viscous_arm.json.
"""

import sys
from pathlib import Path

import numpy as np

from dream import arm_file
from dream.client_ik import PROPRIO_AVG
from dream_ik.solver import ChunkIK
from dream_ik.specs import EMBODIMENTS, SERVE

ARM_FILE = Path(sys.argv[1] if len(sys.argv) > 1 else "viscous_arm.json").resolve()


def solver(name):
    return ChunkIK(name, SERVE.tool, 20.0)


def describe(label, ik, q):
    pose = np.asarray(ik.fk(np.asarray(q, float), "left"))
    p, r = pose[:3, 3], pose[:3, :3]
    print(
        f"  {label:<26} xyz {np.round(p, 3)}  approach(z) {np.round(r[:, 2], 2)}  "
        f"jaw(x) {np.round(r[:, 0], 2)}  y {np.round(r[:, 1], 2)}"
    )
    return pose


def forward_pose(ik, base_q, sweep):
    """The joints near base_q (only the `sweep` joints varied) whose approach is most nearly +x base."""
    best, best_q = -2.0, None
    lo, hi = ik.side_lo[0], ik.side_hi[0]
    grids = [np.linspace(lo[j], hi[j], 41) for j in sweep]
    for values in np.stack(np.meshgrid(*grids), -1).reshape(-1, len(sweep)):
        q = np.array(base_q, float)
        q[list(sweep)] = values
        z = np.asarray(ik.fk(q, "left"))[:3, 2]
        if z[0] > best:
            best, best_q = z[0], q
    return best_q


def main():
    viscous = arm_file.register(str(ARM_FILE))
    arms = {"yam": "yam", "makers": "makers", "viscous": viscous}
    for label, name in arms.items():
        ik = solver(name)
        joints = EMBODIMENTS[name].side_arm("left").joints
        print(f"\n{label}: joints {list(joints)}")
        describe("rest (solver lean)", ik, ik.side_rest[0])
        if label == "yam":
            describe("training mid-state (left)", ik, PROPRIO_AVG["state.left_arm"])
            print(f"  {'training mid hand xyz':<26} {PROPRIO_AVG['state.left_eef_xyz']}")
        if label == "viscous":
            describe("--start neutral (zeros)", ik, np.zeros(len(joints)))
        # Hand straight forward with the wrist's yaw/roll at zero: vary only the pitch joints.
        n = len(joints)
        pitch = (1, 2, 3) if n >= 4 else (1, 2)
        q = forward_pose(ik, np.zeros(n), pitch)
        pose = describe("hand forward, roll 0", ik, q)
        jaw_y = pose[1, 0]
        verdict = "LEFT (matches convention)" if jaw_y > 0.7 else "RIGHT (mirrored)" if jaw_y < -0.7 else "NOT along y"
        print(f"  -> with the hand forward the jaw axis points {verdict}: jaw.y = {jaw_y:+.2f}")


if __name__ == "__main__":
    main()
