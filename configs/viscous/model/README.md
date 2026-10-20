# Viscous Arm CAD model (vendored)

`viscous_arm.urdf` and the collision meshes it references (`meshes/collision*.stl`)
are copied unchanged from
[gavingavinchan/viscous-arm-simulation](https://github.com/gavingavinchan/viscous-arm-simulation)
at commit `86d2ccef7f37d77547e6964fd4e6d34835204794` (`model/`). SHA-256 of the URDF:
`c1e6c3876a67ce2a2e229bdd8a80960e68f085e08833d699536b1feddefb6ff7`.

Why it lives here: Lab's gravity model (`makermodslab/arm_gravity.py`), the gravity
identification scripts (`scripts/viscous-gravity-*.py`) and the teleop gravity
feedforward need it at run time, and a fresh clone must work without a second
checkout. Every recorded gravity run and fit stores this file's hash; the fit
refuses a model that changed under it.

What it is: CAD-derived kinematics (joint axes from motor barrel surfaces) and
mass properties from CAD volumes and assumed densities (RS00 = 0.310 kg each,
6061 aluminium, steel, 30%-solid PLA+). Total ~3.51 kg, 2.91 kg moving. It omits
cables, screws and any potting, and its joint zeros/signs were provisional; the
2026-10-20 runs on unit 1 (`configs/viscous/viscous_01/`) found its gravity
torques within ~5% of measurement, with matching signs on every loaded joint.

The visual meshes (`part_*.stl`, ~21 MB) are display assets and are not vendored;
get them from the source repository. Update by copying the new `model/` files,
then re-run the gravity fit: old fits are tied to the old hash.
