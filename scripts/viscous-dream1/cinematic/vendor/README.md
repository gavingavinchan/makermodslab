`bimanual_common.py` is copied from the user's original
`viscous-rov-model/scripts/bimanual_common.py` (SHA256
`86017c5eda1e7227be8666f083439916827e172b241b643c92f7230d269abc58`).
It implements URDF forward kinematics and SciPy least-squares inverse kinematics.

The vendored copy changes only its project root/URDF path to the packaged
`assets/kinematics.urdf` and removes an unused output-directory creation side
effect. The native kinematic equations, axes, limits and solver are retained.
The source carried no license header; this note records provenance and does not
assign it a newly invented license. No bench joint angles are imported.
