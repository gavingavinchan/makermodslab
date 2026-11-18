# Copyright 2026 MakerMods. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Build the sim-only foam guard model using the tank model's own frame conversion."""

import json
import math
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

REAL = Path(__file__).resolve().parents[1] / "viscous-dream1/real"
sys.path.insert(0, str(REAL))
import make_tank_arm as tank  # noqa: E402

OUTPUT = REAL / "tank_model_foamed"
CALIBRATED = REAL / "runs/live-20261108T205156Z/arm/viscous-waterproof-tank.json"
SIM_PROFILES = {"current": CALIBRATED, "foamed": OUTPUT / "arm.json"}
FOAM = tank.SRC_URDF.parent / "buoyancy/foamed.json"


def vector(values):
    return " ".join(format(float(v), ".12g") for v in values)


def main():
    OUTPUT.mkdir(exist_ok=True)
    mapping = tank.core.JointMap.load()
    limits = json.loads(tank.PROFILE.read_text())["follower"]["joint_limits"]
    ik_limits = mapping.ik_limits_deg(limits)
    lo, hi = ik_limits["shoulder_pan"]
    ik_limits["shoulder_pan"] = max(lo, -90), min(hi, 90)
    source = tank.SRC_URDF.read_bytes()
    robot = ET.fromstring(tank.build_urdf(source, ik_limits))
    header = Path(__file__).read_text().split('"""')[0]
    robot.insert(
        0, ET.Comment("\n" + header + "Simulated foam envelope. Physical installation unverified.\n")
    )
    for mesh in robot.iter("mesh"):
        mesh.set("filename", os.path.relpath(tank.SRC_URDF.parent / mesh.attrib["filename"], OUTPUT))
    sidecar = json.loads(FOAM.read_text())
    for block in sidecar["foam_blocks"]:
        link = robot.find(f"link[@name='{block['link']}']")
        if link is None:
            raise ValueError(f"Unknown foam parent {block['link']}")
        filename = Path(block["mesh"])
        candidates = (FOAM.parent / filename, tank.core.ROOT / filename)
        mesh = next((path for path in candidates if path.is_file()), None)
        if mesh is None:
            raise FileNotFoundError(candidates)
        for tag in ("visual", "collision"):
            part = ET.SubElement(link, tag, name=block["name"])
            ET.SubElement(part, "origin", xyz=vector(block["center_m"]), rpy=vector(block["rpy"]))
            geom = ET.SubElement(part, "geometry")
            ET.SubElement(geom, "mesh", filename=os.path.relpath(mesh, OUTPUT))
    ET.indent(robot, "  ")
    variant = ET.tostring(robot, encoding="utf-8", xml_declaration=True) + b"\n"
    err = tank.frame_check(source, variant, mapping, limits)
    if not math.isfinite(err) or err > 1e-9:
        raise ValueError(f"Tank frame mismatch {err:g}")
    path = OUTPUT / "viscous_waterproof_tank_foamed.urdf"
    path.write_bytes(variant)
    spec = json.loads(CALIBRATED.read_text())
    spec.pop("tcp_calibration")
    spec.update(
        urdf=path.name,
        _copyright=sidecar["_copyright"],
        _license=sidecar["_license"],
        purpose="Simulation only. TCP copied from live-20261108T205156Z, frame and limits from make_tank_arm.",
        sidecar_sha256=tank.core.sha256_bytes(FOAM.read_bytes()),
        urdf_sha256=tank.core.sha256_bytes(variant),
    )
    tank.core.atomic_json(OUTPUT / "arm.json", spec)


if __name__ == "__main__":
    main()
