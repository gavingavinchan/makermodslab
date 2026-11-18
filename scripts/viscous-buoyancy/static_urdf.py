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
"""Generate a static gravity model with signed mass and first moment, including neutral bodies."""

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from hydro import RHO, ROOT, URDF, geo


def gravity_parameters(side):
    """Shift a signed mass at a hinge origin to its parent; every moving mass stays positive.

    Static-only signed first moments survive exact neutral buoyancy. Jaws are
    locked closed by arm_gravity; transfer at prismatic origins is valid at that reference.
    """
    params = {}
    for n, link in side["links"].items():
        m = link["mass_kg"]
        w = link["volume_m3"] * RHO
        params[n] = [m - w, m * np.array(link["com_m"]) - w * np.array(link["cob_m"])]
    for b in side["foam_blocks"]:
        delta = b["mass"] - b["volume"] * RHO
        params[b["link"]][0] += delta
        params[b["link"]][1] += delta * np.array(b["center_m"])
    xml = ET.parse(URDF)
    for j in reversed(xml.findall("joint")):
        c = j.find("child").get("link")
        if c not in params or params[c][0] >= 0.001:
            continue
        # Adding a point mass at the child origin and subtracting the same
        # from the coincident point in parent coordinates has zero net gravity.
        p = j.find("parent").get("link")
        borrow = 0.001 - params[c][0]
        params[c][0] += borrow
        params[p][0] -= borrow
        params[p][1] -= borrow * geo.numbers(j.find("origin").get("xyz"))
    params["base_link"][0] = max(0.001, params["base_link"][0])
    return params


def xml_output(side):
    xml = ET.parse(URDF)
    root = xml.getroot()
    root.set("name", f"viscous_{side['variant']}_fully_submerged_static_only_NOT_FOR_DYNAMICS")
    license_text = (ROOT / "scripts/viscous-astra/model.py").read_text().split('"""')[0]
    root.insert(
        0,
        ET.Comment(
            "\n" + license_text.replace("# ", "").replace("#\n", "\n") + "\n"
            "Fully immersed, closed jaws. Net gravity ONLY. NOT FOR DYNAMICS or control."
        ),
    )
    params = gravity_parameters(side)
    for link in root.findall("link"):
        for visual in link.findall("visual"):
            link.remove(visual)
        for mesh in link.iter("mesh"):
            mesh.set("filename", "../" + mesh.get("filename"))
        name = link.get("name")
        if name in params:
            mass, moment = params[name]
            i = link.find("inertial")
            i.find("mass").set("value", str(mass))
            i.find("origin").set("xyz", " ".join(map(str, moment / mass)))
        for b in (b for b in side["foam_blocks"] if b["link"] == name):
            # Shape meshes are centred on zero. Sidecar/URDF origins perform placement once.
            for tag in ("visual", "collision"):
                el = ET.SubElement(link, tag, name=b["name"])
                ET.SubElement(el, "origin", xyz=" ".join(map(str, b["center_m"])), rpy="0 0 0")
                ET.SubElement(ET.SubElement(el, "geometry"), "mesh", filename=Path(b["mesh"]).name)
    ET.indent(root)
    xml.write(ROOT / side["gravity_only_urdf"], encoding="utf-8", xml_declaration=True)
