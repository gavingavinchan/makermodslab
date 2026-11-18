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
"""Offline torque-off checks with articulated dry inertias, explicit displacement and glass contacts."""

import argparse
import itertools
import json
import xml.etree.ElementTree as ET
from concurrent.futures import ProcessPoolExecutor

import mujoco as mj
import numpy as np
from build import ORDER, minimax, summary, workspace
from hydro import BODIES, JOINTS, KEEP, OUT, RHO, SURFACE, G, block_load, geo, mass_loads, write_json

NAMES = BODIES[2:]
FLOOR = KEEP["planes_mm"]["floor"]["value"] / 1000
SCENE = geo.ENV_DIR / "aquarium_scene.xml"
SCENARIOS = {
    # (quadratic Cd, added-water mass / displaced water mass, joint friction multiplier)
    "nominal": (1.0, 0.6, 1.0),
    "low_resistance": (0.5, 0.25, 0.0),
    "high_resistance": (1.6, 1.2, 2.0),
}


def fmt(values):
    return " ".join(map(str, values))


class Drop:
    def __init__(self, variant, scenario, dt=0.002):
        side = json.loads((OUT / f"{variant}.json").read_text())
        geom = geo.load_model(geo.load_tank())
        self.geom = geom
        self.dt = dt
        self.cd, added, friction = SCENARIOS[scenario]
        tree = ET.parse(SCENE)
        root = tree.getroot()
        root.remove(root.find("keyframe"))
        world = root.find("worldbody")
        for old in world.findall("geom"):
            world.remove(old)
        for name, pos, rotation in [
            ("floor", [0, 0, FLOOR], [0, 0, 0]),
            ("left", [0, 0.227, 0], [np.pi / 2, 0, 0]),
            ("right", [0, -0.207, 0], [-np.pi / 2, 0, 0]),
            ("post", [0.106, 0, 0], [0, -np.pi / 2, 0]),
            ("far", [-0.616, 0, 0], [0, np.pi / 2, 0]),
        ]:
            ET.SubElement(
                world,
                "geom",
                name=name,
                type="plane",
                pos=fmt(pos),
                euler=fmt(rotation),
                size="1 1 .01",
                contype="2",
                conaffinity="1",
            )
        for mesh in root.iter("mesh"):
            mesh.set("file", str((SCENE.parent / mesh.get("file")).resolve()))
        root.find("option").attrib.update(timestep=str(dt), integrator="implicitfast")
        self.axes = geom.motor_to_urdf
        for name, loss in zip(JOINTS, (0.04, 0.11, 0.04, 0.07, 0.04, 0.04), strict=True):
            joint = root.find(f".//joint[@name='{name}']")
            sign = self.axes[name]["sign"]
            limits = np.sort(
                np.array(KEEP["joint_limits_deg"]["value"][name]) * sign + self.axes[name]["offset_deg"]
            )
            joint.set("range", fmt(np.radians(limits + [-5, 5])))
            joint.set("frictionloss", str(loss * friction))
        samples = []
        displaced = []
        dimensions = []
        for n in NAMES:
            body = root.find(f".//body[@name='{n}']")
            for el in body.findall("geom"):
                # Permit self contacts except between immediately adjacent bodies
                # (MuJoCo's default) and between opposite jaw slides.
                el.set("conaffinity", "3")
            link = side["links"][n]
            samples.append(np.array(link["volume_samples"]))
            displaced.append(link["volume_m3"])
            dims = np.ptp(geom.vertices[n], axis=0)
            dimensions.append(dims)
            self.add_water(body, link["cob_m"], dims, added * RHO * displaced[-1])
        contacts = ET.SubElement(root, "contact")
        ET.SubElement(contacts, "exclude", body1="gripper_left", body2="gripper_right")
        # Give each foam insert its real inertia and integrate buoyancy at 32
        # symmetric interior points. The original guard shell already encloses it.
        self.parents = list(NAMES)
        self.drag_flags = [True] * len(NAMES)
        for b in side["foam_blocks"]:
            n = b["link"]
            body = root.find(f".//body[@name='{n}']")
            size = np.array(b["size_m"])
            center = np.array(b["center_m"])
            foam = ET.SubElement(body, "body", name=b["name"], pos=fmt(center))
            diag = b["mass"] * (sum(size**2) - size**2) / 12
            ET.SubElement(foam, "inertial", pos="0 0 0", mass=str(b["mass"]), diaginertia=fmt(diag))
            self.add_water(foam, [0, 0, 0], size, added * RHO * b["volume"])
            grid = np.array(
                list(
                    itertools.product(
                        (-0.375, -0.125, 0.125, 0.375), (-0.375, -0.125, 0.125, 0.375), (-0.25, 0.25)
                    )
                )
            )
            samples.append(center + grid * size)
            displaced.append(b["volume"])
            dimensions.append(size)
            self.parents.append(n)
            self.drag_flags.append(False)
        self.m = mj.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
        self.d = mj.MjData(self.m)
        self.bids = np.array([self.m.body(n).id for n in self.parents])
        self.samples = np.asarray(samples)
        self.weight = RHO * np.array(displaced) * G
        dims = np.array(dimensions)
        self.areas = (np.pi / 4 * np.prod(dims, axis=1)[:, None] / dims) * np.array(self.drag_flags)[:, None]
        self.faces = {self.m.geom(n).id: n for n in ("floor", "far", "post", "left", "right")}
        self.root = self.m.body_rootid[self.bids]
        self.vel = np.zeros(6)

    @staticmethod
    def add_water(body, center, dims, mass):
        water = ET.SubElement(body, "body", name="water_" + body.get("name"), pos=fmt(center), gravcomp="1")
        diag = mass * (sum(dims**2) - dims**2) / 20
        ET.SubElement(water, "inertial", pos="0 0 0", mass=str(mass), diaginertia=fmt(diag))

    def forces(self):
        d = self.d
        # cvel is world-oriented and referenced at the tree COM; xfrc_applied
        # is world-oriented and referenced at the receiving body's inertial COM.
        r = d.xmat[self.bids].reshape(-1, 3, 3)
        points = self.samples @ r.transpose(0, 2, 1) + d.xpos[self.bids, None, :]
        wet = points[:, :, 2] < SURFACE
        arm = points - d.subtree_com[self.root, None, :]
        omega = d.cvel[self.bids, None, :3]
        vel = d.cvel[self.bids, None, 3:] + np.cross(omega, arm)
        vl = vel @ r
        drag = (-0.5 * RHO * self.cd * self.areas[:, None, :] * vl * abs(vl)) @ r.transpose(0, 2, 1)
        drag[:, :, 2] += self.weight[:, None]
        force = drag * (wet[:, :, None] / points.shape[1])
        moment = np.cross(points - d.xipos[self.bids, None, :], force)
        d.xfrc_applied[:] = 0
        # Multiple inserts can act on the same receiving body.
        np.add.at(d.xfrc_applied, self.bids, np.column_stack((force.sum(axis=1), moment.sum(axis=1))))

    def run(self, pose, *, duration=1.7, settle=False):
        m, d = self.m, self.d
        mj.mj_resetData(m, d)
        q = geo.joint_positions(self.geom, dict(zip(ORDER, pose, strict=True)))
        for n, v in q.items():
            d.qpos[m.joint(n).qposadr[0]] = v
        first = None
        side = None
        start_tip = None
        start_time = None
        stopped = False
        for step in range(round(duration / self.dt)):
            mj.mj_step1(m, d)
            if step == 0:
                if d.ncon:
                    return {"excluded": "initial contact"}
                start_tip = d.body("tool0").xpos.copy()
            floor_speeds = []
            for c in d.contact:
                surface = next((g for g in (c.geom1, c.geom2) if g in self.faces), None)
                if surface is None or c.dist > 0:
                    continue
                surface_name = self.faces[surface]
                moving = c.geom2 if surface == c.geom1 else c.geom1
                n = m.geom_bodyid[moving]
                if n == 0:
                    continue
                if surface_name == "floor":
                    if first is None:
                        mj.mj_objectVelocity(m, d, mj.mjtObj.mjOBJ_BODY, n, self.vel, 0)
                        v = self.vel[3:] + np.cross(self.vel[:3], c.pos - d.xipos[n])
                        floor_speeds.append((max(0.0, -v[2]), m.body(n).name, float(np.linalg.norm(v))))
                elif side is None:
                    side = {"face": surface_name, "time": float(d.time)}
            if floor_speeds:
                speed, link, speed3 = max(floor_speeds)
                first = {
                    "floor_speed_m_s": float(speed),
                    "floor_time_s": float(d.time),
                    "link": link,
                    "speed_3d_m_s": speed3,
                }
                if not settle:
                    break
            if settle and first and np.max(np.abs(d.qvel)) < 0.01:
                if start_time is None:
                    start_time = d.time
                elif d.time - start_time > 0.5:
                    stopped = True
                    break
            else:
                start_time = None
            self.forces()
            mj.mj_step2(m, d)
        result = (first or {"floor_speed_m_s": None}) | {"side": side}
        if settle:
            result.update(
                duration_s=float(d.time),
                tool0_drift_m=(d.body("tool0").xpos - start_tip).tolist(),
                max_joint_speed_rad_or_m_s=float(np.max(np.abs(d.qvel))),
                settled=stopped,
                floor_contacts_at_end=sum(
                    any(self.faces.get(g) == "floor" for g in [c.geom1, c.geom2]) for c in d.contact
                ),
            )
        return result


def trials(args):
    variant, scenario, dt, poses = args
    tank = Drop(variant, scenario, dt)
    return [(i, tank.run(q)) for i, q in poses]


def clear_static(model, poses, tops, side):
    dry, water = mass_loads(model, poses, side["links"])
    base = dry - water @ np.array([RHO * link["volume_m3"] for link in side["links"].values()])
    blocks = side["foam_blocks"]
    fractions = []
    tau = base
    if blocks:
        design = np.stack([b["volume"] * block_load(model, poses, b) for b in blocks], axis=-1)
        fractions = minimax(base, design, [(0, 1)] * len(blocks)).tolist()
        tau = base + design.sum(axis=-1)
    return {
        "poses": len(poses),
        "submerged": int(sum(tops <= SURFACE)),
        "all": summary(tau),
        "fully_submerged": summary(tau[tops <= SURFACE]),
        "minimax_pocket_fractions": fractions,
        "over_5nm_shoulder": int(sum(abs(tau[:, 1]) > 5)),
        "submerged_over_5nm_shoulder": int(sum(abs(tau[tops <= SURFACE, 1]) > 5)),
        "minimax_pitch_load_over_poses_nm": float(np.min(np.max(abs(tau[:, 1:4]), axis=1))),
    }


def run(workers):
    model = geo.load_model(geo.load_tank())
    poses, top = workspace(model)
    # Search every sampled free-water pose, including poses that break the surface.
    index = np.arange(len(poses))
    jobs = [
        (v, s, 0.002, [(int(i), poses[i]) for i in chunk])
        for v in ("current", "foamed")
        for s in SCENARIOS
        for chunk in np.array_split(index, max(1, workers))
    ]
    groups = {}
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for (v, s, _, _), rows in zip(jobs, pool.map(trials, jobs), strict=True):
            groups.setdefault((v, s), []).extend(rows)
    out = {
        "release_poses": len(index),
        "fully_wet_proposals": int(np.sum(top[index] <= SURFACE)),
        "settings": SCENARIOS,
        "results": {},
    }
    for (variant, scenario), rows in groups.items():
        floor = [(i, r) for i, r in rows if r.get("floor_speed_m_s") is not None]
        i, worst = max(floor, key=lambda ir: ir[1]["floor_speed_m_s"])
        full = [(i, r) for i, r in floor if top[i] <= SURFACE]
        wi, wr = max(full, key=lambda ir: ir[1]["floor_speed_m_s"])
        res = {
            "initial_contact_exclusions": sum("excluded" in r for _, r in rows),
            "floor_impacts": len(floor),
            "first_side_before_floor": sum(r["side"] is not None for _, r in floor),
            "fully_wet_floor_impacts": len(full),
            "fully_wet_worst": wr | {"start_motor_deg": poses[wi].tolist()},
            "floor_normal_percentiles_m_s": np.percentile(
                [r["floor_speed_m_s"] for _, r in floor], [5, 50, 95]
            ).tolist(),
            "worst": worst
            | {"start_motor_deg": poses[i].tolist(), "fully_submerged_at_release": bool(top[i] <= SURFACE)},
        }
        out["results"].setdefault(variant, {})[scenario] = res
    for variant in ("current", "foamed"):
        valid = [i for i, r in groups[variant, "nominal"] if "excluded" not in r]
        side = json.loads((OUT / f"{variant}.json").read_text())
        out["results"][variant]["contact_free_statics"] = clear_static(model, poses[valid], top[valid], side)
        worst = out["results"][variant]["nominal"]["worst"]
        tank = Drop(variant, "nominal", 0.001)
        out["results"][variant]["worst_half_step"] = tank.run(
            worst["start_motor_deg"], duration=6, settle=True
        )
        bench = json.loads((OUT / "analysis.json").read_text())["fit"]["cutout_rehearsal"][0]["pos_deg"]
        out["results"][variant]["rehearsal_pose_release"] = tank.run(bench, duration=6, settle=True)
    write_json("drops.json", out)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workers", type=int, default=4)
    run(parser.parse_args().workers)
