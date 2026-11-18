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
"""Regenerate a review-only model in a NEW directory using Dream's offline tank builder."""

import argparse
import importlib.util
import shutil
import xml.etree.ElementTree as ET
from datetime import UTC, datetime
from pathlib import Path

import numpy as np
from fit import save
from kinematics import ARM_FILE, JOINTS, LINK_JOINTS, MAP, MODEL, REAL, ROOT, URDF, Arm, core, digest, read

PROFILE = ROOT / "configs/viscous/viscous_waterproof_01/evidence/lab-hardware-profile.json"
REST = REAL / "runs/start_pose/20261108T183949Z.json"
BUILDER = REAL / "make_tank_arm.py"
VISUAL = ROOT / "scripts/viscous-dream1/assets/propeller_cardinal/arm"


def generate(bundle, out):
    spec = importlib.util.spec_from_file_location("dream_tank_builder", BUILDER)
    builder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(builder)
    map_path = bundle / "motor_to_urdf.pending.json"
    tcp_path = bundle / "tcp.pending.json"
    report_path = bundle / "residuals.json"
    tcp, report = read(tcp_path), read(report_path)
    prov = tcp["provenance"]
    if report["issues"] or report["provenance"] != prov or read(map_path)["provenance"] != prov:
        raise ValueError("Calibration files must come from one successful pending fit")
    for key, path in [("urdf", URDF), ("motor_map", MAP), ("arm", ARM_FILE)]:
        if digest(path) != prov["sha256"][key]:
            raise ValueError(f"Changed calibration source {path}")
    base = ET.parse(URDF).getroot()
    chain = Arm()
    for i, delta in zip(LINK_JOINTS, tcp["link_length_delta_mm"], strict=True):
        xyz = chain.xyz[i]
        joint = base.find(f"joint[@name='{JOINTS[i]}']/origin")
        joint.set("xyz", " ".join(f"{v:.12g}" for v in ((xyz + delta * xyz / np.linalg.norm(xyz)) / 1000)))
    base.find("joint[@name='tool_fixed']/origin").set("xyz", " ".join(map(str, tcp["tcp"])))
    source = ET.tostring(base, encoding="utf-8", xml_declaration=True)
    mapping = core.JointMap.load(map_path)
    limits = read(PROFILE)["follower"]["joint_limits"]
    ik = mapping.ik_limits_deg(limits)
    lo, hi = ik[JOINTS[0]]
    ik[JOINTS[0]] = max(lo, -90), min(hi, 90)
    rest = mapping.to_ik(read(REST)["motor_deg"])
    if any(not lo <= np.rad2deg(q) <= hi for (lo, hi), q in zip(ik.values(), rest, strict=True)):
        raise ValueError("Historical rest falls outside corrected limits")
    variant = builder.build_urdf(source, ik)
    error = builder.frame_check(source, variant, mapping, limits)
    if error > 1e-9:
        raise ValueError(f"Tank frame FK failed: {error}")
    meshes = {}
    for mesh in base.findall(".//mesh"):
        name = mesh.get("filename")
        path = MODEL / name
        if not path.is_file():
            path = VISUAL / Path(name).name
        if not path.is_file():
            raise FileNotFoundError(path)
        meshes[name] = path
    out.mkdir(parents=True, exist_ok=False)
    try:
        (out / "meshes").mkdir()
        for name, path in meshes.items():
            shutil.copyfile(path, out / name)
        (out / "viscous_arm.pending.urdf").write_bytes(source)
        stem = builder.NAME
        urdf = out / f"{stem}.urdf"
        urdf.write_bytes(variant)
        arm = read(ARM_FILE) | {"tcp": tcp["tcp"], "rest": rest, "urdf": urdf.name}
        save(out / f"{stem}_arm.json", arm)
        shutil.copyfile(map_path, out / map_path.name)
        save(
            out / f"{stem}.provenance.json",
            {
                "approval": {"status": "pending_gavin_approval", "by": None, "at": None},
                "generated_utc": datetime.now(UTC).isoformat(),
                "calibration": prov,
                "frame_check_max_error": error,
                "pan": "IK deg = sign * motor_deg + corrected offset_deg - 180",
                "urdf": {"sha256": digest(urdf), "sha256_normalized": core.normalized_urdf_sha256(variant)},
                "input_sha256": {
                    p.name: digest(p) for p in [BUILDER, PROFILE, REST, map_path, tcp_path, report_path]
                },
                "mesh_sha256": {name: digest(path) for name, path in meshes.items()},
            },
        )
    except BaseException:
        shutil.rmtree(out)
        raise


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bundle", type=Path)
    parser.add_argument("new_directory", type=Path)
    args = parser.parse_args()
    generate(args.bundle, args.new_directory)
