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
"""The autocal session: the astra operator's lifecycle with a press routine instead of a socket.

What is reused, unchanged: session.Operator (begin: connect, start gate, engage; execute: the
15 deg/s route planner with the tank guard and the slow floor zone; legs_from/end) and, under it,
dream_real.Episode (the trip monitor: lag, torque tiers, overspeed, limp, temperature, the
measured-pose keep-out check; and stop(): re-anchor, gripper limp, return to the measured start leg
by leg at <= 15 deg/s on fresh feedback, all torque off, HOLDING instead of a mid-air release).
The only stop signal is the process's SIGTERM/SIGINT from stop_button.py (install_stop_signals);
nothing new signals the process.

What is new: press(), the compliance probe; a probe keep-out swapped in while the jaw is near the
glass (so the monitor and the stop path accept fingers ON the probed face, and nothing else); and
the per-tick metrics the sim and the report read.
"""

from __future__ import annotations

import dataclasses
import json
import math
import time

import numpy as np
import planner as probe_planner
import press as press_checks
from context import core, real
from geometry import ARM, back_touch, contact_point, point_jacobian
from session import Operator

from makermodslab import tank_geometry as geo

DT = core.ROW_DT_S
HOLD_S = 1.2  # each press is held this long; the settled part of it gives the plane points
SETTLE_S = 1.5  # at the lift pose and at the standoff before measuring the sag
MAX_OFFSET_DEG = 10.0  # sag compensation larger than this is not sag: the probe is abandoned


class ProbeError(RuntimeError):
    """This probe is abandoned (backed off, no touch recorded); the session continues."""


class ProbeReturnGuard(real.Guard):
    """dream_real.Guard over a TankGuard that can be swapped (the transit or a probe keep-out)."""

    def __init__(self, guard):
        self.problem = None
        self.guard = guard
        self.targets = None


@dataclasses.dataclass
class Metrics:
    max_cmd_joint_deg_s: float = 0.0
    max_cmd_tip_mm_s_in_approach: float = 0.0
    max_measured_joint_deg_s: float = 0.0


class AutocalSession(Operator):
    def __init__(
        self,
        arm,
        model,
        planner,
        folder,
        env,
        events,
        pid_file,
        tank,
        keepout,
        clock,
        sleep,
        armed=True,
        specs=probe_planner.PROBES,
        spares=probe_planner.SPARES,
        log=print,
    ):
        self.model, self.folder, self.armed = model, folder, armed
        self.planner, self.tank, self.keepout = planner, tank, keepout
        self.number, self.budget = 0, 10**9
        self.trace = (folder / "commands.jsonl").open("a", buffering=1)
        self.floor_sample = None
        self.grip_history = []
        self.closing = self.quit = self.finished = self.connected = False
        self.goal_tip = None
        self.last_poll = 0.0
        self.reading = None
        self.say = log
        settings = real.Settings("", 0, 0, probe_planner.SLEW_DEG_S, 15, 35.0, 0, core.TripLimits(), {})
        self.transit_guard = ProbeReturnGuard(model.guard)
        real.Episode.__init__(
            self,
            arm,
            None,
            None,
            model.map,
            model.limits,
            None,
            self.transit_guard,
            settings,
            clock,
            sleep,
            folder,
            *events,
            status_path=str(pid_file) + core.STATUS_SUFFIX,
            environment=env,
            arm_rest=model.spec["rest"],
        )
        self.say = log  # Episode.__init__ reset it
        self.step = "start"  # this tool's own phase (Episode's self.phase belongs to its loop)
        self.specs, self.spares = list(specs), list(spares)
        self.nominal = probe_planner.update_estimate(planner_nominal(planner, tank, keepout), [])
        self.estimate = self.nominal.copy()
        self.touches: list[dict] = []
        self.rejected: list[dict] = []
        self.dropped: list[dict] = []
        self.probes: list[dict] = []
        self.metrics = Metrics()
        self.hub = None
        self.base_trail = None
        self.tick_hook = None  # sim: called with each reading (contact force bookkeeping)
        self._last_cmd = None
        self._last_cmd_t = None

    # --- the harness hooks, without cameras ----------------------------------------------------

    def _phase(self, name, detail=""):
        self.step = f"{name}: {detail}" if detail else name
        if hasattr(self.arm, "phase"):
            self.arm.phase = self.step
        super()._phase(name, detail)

    def poll(self):
        if self.stop_event.is_set():
            raise RuntimeError("red button")
        r = self.arm.read()
        self.reading = r
        reference = self.commanded
        trips = self.monitor.update(r.pos, reference, r.torque, r.temp)
        if problem := self.guard.monitor(r.pos):
            trips.append(core.Trip("tank_guard_measured", problem))
        self.last_state = self._state(r, reference, self.phase)
        self._tick(self.step, r, reference, trips)
        self._status(r)
        self.last_poll = time.monotonic()
        speeds = self.monitor.speed.peak
        self.metrics.max_measured_joint_deg_s = max(speeds.values())
        if self.tick_hook is not None:
            self.tick_hook(self, r)
        if trips:
            self.log(trips=[vars(t) for t in trips])
            raise real.TripStopError(trips)
        self.hold_error.update(r.pos, reference, self.clock())

    def send(self, target):
        """One guarded tick: speed-checked against the 15 deg/s cap, then sent, then a fresh reading."""
        if self.stop_event.is_set():
            raise RuntimeError("red button")
        for m in ARM:
            lo, hi = self.limits[m]
            if not lo <= target[m] <= hi:
                raise ProbeError(f"compensated {m} target {target[m]:.2f} outside {lo:.2f}..{hi:.2f}")
        step = max(abs(target[m] - self.commanded[m]) for m in ARM)
        speed = step / DT
        if speed > real.HARD_ARM_SPEED_DEG_S + 1e-6:
            raise RuntimeError(f"internal: a {speed:.1f} deg/s tick exceeds the 15 deg/s cap")
        self.metrics.max_cmd_joint_deg_s = max(self.metrics.max_cmd_joint_deg_s, speed)
        self.arm.send(target, self.commanded)
        self.commanded = dict(target)
        self.sleep(DT)
        self.poll()

    def hold(self, seconds):
        samples = []
        for _ in range(max(1, round(seconds / DT))):
            self.send(self.commanded)
            self.reading.t = self.clock()  # when it was read (the sim/fake clock, or monotonic)
            samples.append(self.reading)
        return samples

    def slew(self, goal, speed_deg_s=probe_planner.SLEW_DEG_S, guard=None, intended=None, returning=False):
        """Straight joint-space slew at <= speed_deg_s, proven before the first tick.

        `intended` = (a, b): the poses the arm is expected to actually take (the command minus the
        measured standing error). Those must clear the keep-out outright; the compensated command
        may eat into it by the guard's tracking allowance only (what the harness already allows a
        measured pose), at the goal. `returning`: a way back, judged as the stop path judges its
        legs (tank_guard.check_return_segment: never closer than the start)."""
        goal = {**goal, "gripper": self.commanded["gripper"]}
        if guard is not None:
            if returning:
                v = guard.check_return_segment(self.commanded, goal)
            elif intended is not None:
                a, b = ({**x, "gripper": goal["gripper"]} for x in intended)
                v = guard.check_segment(a, b)
                if v.ok:
                    v = guard.check_pose(goal, need_mm=-guard.tracking_allowance_mm)
            else:
                v = guard.check_segment(self.commanded, goal)
            if not v.ok:
                raise ProbeError(f"slew refused by the keep-out: {v.link} vs {v.face} ({v.slack_mm:.1f} mm)")
        for m in ARM:
            lo, hi = self.limits[m]
            if not lo <= goal[m] <= hi:
                raise ProbeError(f"compensated slew would exceed the {m} limit")
        start = dict(self.commanded)
        ticks = max(1, math.ceil(max(abs(goal[m] - start[m]) for m in ARM) / (speed_deg_s * DT)))
        for i in range(1, ticks + 1):
            self.send({m: start[m] + (goal[m] - start[m]) * i / ticks for m in start})

    # --- the routine -------------------------------------------------------------------------

    def run_all(self):
        """Hub, every probe (replanned from the touches so far), replacements, the fit."""
        if abs(self.reading.pos["gripper"] - core.GRIP_CLOSED_DEG) > 1.0:
            raise ProbeError("calibration requires a measured closed jaw")
        self._phase("autocal", "to the hub")
        self.hub = self.planner.hub(self.estimate)
        if max(abs(self.commanded[m] - self.hub[m]) for m in ARM) > 0.5:
            self.execute([self.hub])
        self.planner.poll = self.heartbeat
        self.base_trail = [dict(p) for p in self.trail]
        queue = list(self.specs)
        spares = list(self.spares)
        while queue:
            spec = queue.pop(0)
            outcome = self.probe(spec)
            if outcome != "touch":
                replacement = next(
                    (
                        s
                        for s in spares
                        if probe_planner.face_of(s.label) == probe_planner.face_of(spec.label)
                    ),
                    None,
                )
                if replacement is not None:
                    spares.remove(replacement)
                    queue.insert(0, dataclasses.replace(replacement, label=spec.label))
        from fitting import robust_fit

        result = robust_fit(self.touches, self.tank, self.estimate)
        # A touch the fit could not place on the glass is dropped and re-probed at a spare
        # location (once per dropped touch); then the final fit.
        if result["dropped"]:
            self.touches = result["kept"]
            self.estimate = probe_planner.update_estimate(self.nominal, self.touches)
            probe_planner.sync_guard(self.model.guard, self.estimate, self.planner.nominal, conservative=True)
            self.dropped = result["dropped"]
            for label in result["dropped_labels"]:
                replacement = next(
                    (s for s in spares if probe_planner.face_of(s.label) == probe_planner.face_of(label)),
                    None,
                )
                if replacement is None:
                    continue
                spares.remove(replacement)
                self.probe(dataclasses.replace(replacement, label=label))
            result = robust_fit(self.touches, self.tank, self.estimate)
            result["dropped"] = self.dropped + result["dropped"]
        self.touches = result["kept"]
        fit = result["fit"]
        counts = {
            label: sum(t["label"] == label for t in self.touches)
            for label in ("floor", "right", "left", "check:floor", "check:right", "check:left")
        }
        fit["measured_counts"] = counts
        required = {"floor": 4, "right": 3, "check:floor": 1, "check:right": 1}
        if not result["left_derived"]:
            required.update(left=3, **{"check:left": 1})
        for label, count in required.items():
            if counts[label] < count:
                fit["failures"].append(f"{label}: obtained {counts[label]} of {count} required real touches")
        fit["valid"] = not fit["failures"]
        return result

    def probe(self, spec) -> str:
        record = {"spec": dataclasses.asdict(spec), "t0": self.clock()}
        self.probes.append(record)
        self.say(f"probe {len(self.probes)}: {spec.label} at ({spec.u:.0f}, {spec.v:.0f})")
        plan = self.planner.plan(spec, self.estimate, self.touches, self.hub)
        record["plan"] = plan.summary()
        if not plan.ok:
            record["outcome"] = "unplannable"
            self.say(f"  unplannable: {plan.reasons[:2]}")
            return "unplannable"
        try:
            result = self.press(spec, plan, record)
        except ProbeError as error:
            record["outcome"] = "abandoned"
            record["reason"] = str(error)
            self.say(f"  abandoned: {error}")
            self.back_to_hub(plan)
            return "abandoned"
        self.back_to_hub(plan)
        record["verdict"] = result
        if result["ok"]:
            touch = {
                "label": spec.label,
                "point_mm": result["point_mm"],
                "positions": result["positions"],
                "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                "method": "autocal compliance press",
                "probe": len(self.probes),
            }
            self.touches.append(touch)
            self.estimate = probe_planner.update_estimate(self.nominal, self.touches)
            probe_planner.sync_guard(self.model.guard, self.estimate, self.planner.nominal, conservative=True)
            record["outcome"] = "touch"
            self.say(
                f"  touch {spec.label}: {np.round(result['point_mm'], 1).tolist()} "
                f"(shift {result['shift_mm']:+.2f} mm, slide {result['slide_mm']:.1f} mm)"
            )
            return "touch"
        record["outcome"] = "rejected"
        self.rejected.append({"label": spec.label, "reasons": result["reasons"], "verdict": result})
        self.say(f"  rejected: {result['reasons']}")
        return "rejected"

    def back_to_hub(self, plan):
        """Off the glass along the way in (standoff, lift), the sag compensation removed at the lift,
        then the harness route to the hub; the stop path's trail is reset to the hub's."""
        if self.retreat_path:
            for pose in self.retreat_path:
                self.slew(pose, guard=self.guard.guard, returning=True)
        self._phase("autocal", "back to the hub")
        self.guard = self.transit_guard
        self.execute([self.hub])
        self.trail = [dict(p) for p in self.base_trail]
        self.retreat_path = None

    retreat_path = None

    def press(self, spec, plan, record) -> dict:
        face, n = spec.face, np.asarray(plan.normal)
        pm = self.planner.pm
        nodes, svals = plan.nodes, np.asarray(plan.node_s_mm)
        probe_guard = probe_planner.probe_guard(
            self.keepout, pm, self.estimate, face, plan.window_mm + plan.d2_mm + 5.0
        )

        def line(s):
            """Joint targets for commanded depth s (mm past the standoff), linear between nodes."""
            s = min(max(s, 0.0), float(svals[-1]))
            k = min(int(np.searchsorted(svals, s, side="right")) - 1, len(nodes) - 2)
            f = (s - svals[k]) / (svals[k + 1] - svals[k])
            return {m: nodes[k][m] + f * (nodes[k + 1][m] - nodes[k][m]) for m in ARM}

        intended = {}

        def intended_point(s):
            key = round(s, 3)
            if key not in intended:
                intended[key] = contact_point(pm, {**line(s), "gripper": self.commanded["gripper"]}, n)
            return intended[key]

        # 1. The harness route to the lift pose (normal keep-out, 15 deg/s, slow near the floor).
        self._phase("autocal", f"transit to {spec.label} lift")
        self.execute([{**plan.lift, "gripper": self.commanded["gripper"]}])
        self._phase(f"probe {spec.label}", "lift: measure the sag")
        self.guard = ProbeReturnGuard(probe_guard)
        lift = {**plan.lift, "gripper": self.commanded["gripper"]}
        self.retreat_path = [lift]
        offset = dict.fromkeys(ARM, 0.0)

        def bounded_offset(proposed, poses):
            # Pan standing error at a limit can be <0.1 degree. Use only the support
            # available over the entire stroke, then measure the residual at standoff.
            return {
                m: float(
                    np.clip(
                        proposed[m],
                        self.limits[m][0] - min(q[m] for q in poses),
                        self.limits[m][1] - max(q[m] for q in poses),
                    )
                )
                for m in ARM
            }

        # 2. Sag compensation: the MIT standing error, measured and added back, twice.
        for _ in range(2):
            readings = self.hold(SETTLE_S)[-round(1.0 / DT) :]
            for m in ARM:
                offset[m] += float(np.mean([lift[m] - r.pos[m] for r in readings]))
            if max(abs(v) for v in offset.values()) > MAX_OFFSET_DEG:
                raise ProbeError(f"standing error over {MAX_OFFSET_DEG:g} deg at the lift pose: {offset}")
            self.planner.sag[face] = dict(offset)
            offset = bounded_offset(offset, [lift, nodes[0]])
            self.slew({m: lift[m] + offset[m] for m in ARM}, guard=probe_guard, intended=(lift, lift))
        record["offset_deg"] = dict(offset)
        # 3. Down to the standoff, re-measure the standing error there.
        if self.trail[-1] != lift:
            self.trail.append(dict(lift))
        self.retreat_path = [lift]
        self._phase(f"probe {spec.label}", "descend to the standoff")
        self.slew({m: nodes[0][m] + offset[m] for m in ARM}, guard=probe_guard, intended=(lift, nodes[0]))
        readings = self.hold(SETTLE_S)[-round(1.0 / DT) :]
        for m in ARM:
            offset[m] += float(np.mean([nodes[0][m] - r.pos[m] for r in readings]))
        if max(abs(v) for v in offset.values()) > MAX_OFFSET_DEG:
            raise ProbeError(f"standing error over {MAX_OFFSET_DEG:g} deg at the standoff")
        offset = bounded_offset(offset, nodes)
        record["standoff_offset_deg"] = dict(offset)
        self.slew({m: nodes[0][m] + offset[m] for m in ARM}, guard=probe_guard, intended=(nodes[0], nodes[0]))
        stand_cmd = dict(self.commanded)
        self.trail.append(dict(stand_cmd))
        # Back out: standoff, then straight to the UNcompensated lift, so the standing-error offset
        # is removed while the jaw rises 60 mm (removing it at the standoff would let the hand sag
        # onto the glass; keeping it to the lift can push the upper arm's heel toward the post).
        self.retreat_path = [stand_cmd, lift]

        def command(s):
            return {**{m: line(s)[m] + offset[m] for m in ARM}, "gripper": self.commanded["gripper"]}

        def sample(s, reading):
            meas = reading.pos
            point = contact_point(pm, meas, n)
            return {
                "t": getattr(reading, "t", self.clock()),
                "pos": dict(meas),
                "arm": ARM,
                "point": point,
                "div": float(n @ (intended_point(s) - point)),
                "torque": dict(reading.torque),
            }

        # Prove every compensated segment through the deepest command before advancing.
        # Apply only the existing 15 mm tracking allowance to the compensated commands.
        relax = dict.fromkeys(probe_guard.clearances(stand_cmd), probe_guard.tracking_allowance_mm)
        before = stand_cmd
        allowed_s = 0.0
        for depth in svals[1:]:
            self.heartbeat()
            target = command(depth)
            if any(not self.limits[m][0] <= target[m] <= self.limits[m][1] for m in ARM):
                break
            if not probe_guard._scan(before, target, to_violation=False, relax=relax)[0].ok:
                break
            before, allowed_s = target, float(depth)
        limit = min(plan.standoff_mm + plan.window_mm, allowed_s - plan.d2_mm)
        if limit < plan.standoff_mm + probe_planner.WINDOW_MIN_MM:
            raise ProbeError("compensated line too short inside joint limits and keep-out")
        record["compensated_search_limit_mm"] = limit
        # 4. The free-space baseline: hold at the standoff.
        self._phase(f"probe {spec.label}", "baseline at the standoff")
        hold = [sample(0.0, r) for r in self.hold(1.0)]
        base = press_checks.baseline_from([h["div"] for h in hold], [h["torque"] for h in hold])
        detector = press_checks.PressDetector(base, probe_planner.TORQUE_BACKSTOP_NM, ARM)
        record["baseline"] = dataclasses.asdict(base)
        # 5. The approach, at APPROACH_MM_S, until the divergence says the plane is reached.
        self._phase(f"probe {spec.label}", "approach")
        v = probe_planner.APPROACH_MM_S * DT
        s, event, trace = 0.0, None, []
        allowance = -probe_guard.tracking_allowance_mm
        while s < limit:
            s = min(s + v, limit)
            target = command(s)
            if not probe_guard.check_pose(target, need_mm=allowance).ok:
                self.retreat(command, s - v)
                raise ProbeError("the compensated approach would leave the probe keep-out")
            self.send(target)
            smp = sample(s, self.reading)
            trace.append((s, smp["div"] - detector.mean))
            event = detector.update(s, smp["div"], smp["torque"])
            if event:
                break
        self.metrics.max_cmd_tip_mm_s_in_approach = max(self.metrics.max_cmd_tip_mm_s_in_approach, v / DT)
        record["approach"] = {
            "s_end_mm": s,
            "event": dataclasses.asdict(event) if event else None,
            "peak_excess_mm": detector.peak_excess,
        }
        if event is None:
            self.retreat(command, s)
            raise ProbeError(
                f"no contact within the window ({plan.window_mm:.0f} mm past the expected plane)"
            )
        if event.kind == "torque":
            self.retreat(command, s)
            raise ProbeError(f"torque without the jaw stopping (a false contact): {event.detail}")
        # 6. Two presses past the onset, each held; settled samples are plane points.
        jac_link = "gripper_left"

        backstop_ticks = 0

        def check_load():
            nonlocal backstop_ticks
            over = max(abs(self.reading.torque[m] - base.torque_nm[m]) for m in ARM)
            backstop_ticks = backstop_ticks + 1 if over > probe_planner.TORQUE_BACKSTOP_NM else 0
            if backstop_ticks >= press_checks.TORQUE_TICKS:
                self.retreat(command, s)
                raise ProbeError(f"torque backstop during press ({over:.2f} Nm above baseline)")

        def press_to(depth):
            target = min(event.onset_mm + depth, float(svals[-1]))
            nonlocal s
            while s < target - 1e-9:
                s = min(s + v, target)
                self.send(command(s))
                check_load()
            samples = []
            for _ in range(round(HOLD_S / DT)):
                self.send(self.commanded)
                check_load()
                r = self.reading
                r.t = self.clock()
                smp = sample(s, r)
                smp["div_excess"] = smp["div"] - detector.mean
                jac = point_jacobian(pm, r.pos, smp["point"], jac_link)
                dtau = np.array([r.torque[m] - base.torque_nm[m] for m in ARM])
                smp["force_n"] = press_checks.force_from_torques(dtau, jac, n)
                samples.append(smp)
            return samples

        self._phase(f"probe {spec.label}", "press 1")
        p1 = press_to(plan.d1_mm)
        self._phase(f"probe {spec.label}", "press 2")
        p2 = press_to(plan.d2_mm)
        deep_pose = p2[-1]["pos"]
        problems = self.contact_problems(face, n, deep_pose)
        verdict = press_checks.judge(p1, p2, plan.d1_mm, plan.d2_mm, n, plan.stiffness_n_per_m, problems)
        verdict["positions"] = {m: float(np.median([smp["pos"][m] for smp in p1 + p2])) for m in deep_pose}
        verdict["onset_mm"] = event.onset_mm
        record["presses"] = {
            "press1": [{"point": smp["point"].tolist(), "div_excess": smp["div_excess"]} for smp in p1],
            "press2": [{"point": smp["point"].tolist(), "div_excess": smp["div_excess"]} for smp in p2],
        }
        self.retreat(command, s)
        return verdict

    def retreat(self, command, s):
        """Back along the line to the standoff at RETREAT_MM_S (back_to_hub then takes the lift)."""
        self._phase("probe", "retreat")
        v = probe_planner.RETREAT_MM_S * DT
        while s > 0:
            s = max(0.0, s - v)
            self.send(command(s))

    def contact_problems(self, face, n, pose) -> list[str]:
        """Is the thing pressing really the jaw, against THIS face? From the measured pose."""
        from geometry import lead_margin

        problems = []
        lead, link = lead_margin(self.planner.pm, pose, n, self.planner.padding)
        if lead < probe_planner.LEAD_MIN_MM / 2:
            problems.append(f"{link} (+ padding) is within {lead:.0f} mm of the jaw toward the glass")
        for other in ("floor", "left", "right"):
            if other == face or other not in self.estimate.known:
                continue
            nn = self.estimate.normal(other)
            gap = -self.estimate.signed_depth(other, contact_point(self.planner.pm, pose, nn))
            if gap < 6.0:
                problems.append(f"the jaw is {gap:.1f} mm from the measured {other}: it may be touching that")
        return problems

    def report(self) -> dict:
        return {
            "touches": self.touches,
            "rejected": self.rejected,
            "probes": self.probes,
            "metrics": dataclasses.asdict(self.metrics),
            "peaks": {
                "torque_nm": self.monitor.peak_torque,
                "lag_deg": self.monitor.peak_lag,
                "measured_speed_deg_s": dict(self.monitor.speed.peak),
            },
            "back_touch": back_touch(self.estimate),
        }


def planner_nominal(planner, tank, keepout):
    from geometry import nominal_estimate

    return nominal_estimate(tank, keepout)


def dump(path, obj):
    path.write_text(json.dumps(obj, indent=2, default=_default) + "\n")


def _default(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, set):
        return sorted(value)
    if dataclasses.is_dataclass(value):
        return dataclasses.asdict(value)
    return str(value)


__all__ = ["AutocalSession", "ProbeError", "geo"]


def run_session(
    arm,
    model,
    planner,
    folder,
    events,
    tank,
    keepout,
    pid_file,
    clock,
    sleep,
    *,
    specs=probe_planner.PROBES,
    spares=probe_planner.SPARES,
    log=print,
    tick_hook=None,
    setup=None,
):
    """begin -> every probe -> fit -> stop. Every exit, including an exception or the red button,
    goes through the harness's stop (Operator.end -> Episode.stop): return to the measured start at
    <= 15 deg/s, then all torque off (or HOLDING when a return is not provably clear)."""
    from fitting import candidate_files, placement_report

    session = AutocalSession(
        arm,
        model,
        planner,
        folder,
        env=real.ENVIRONMENTS["tank"],
        events=events,
        pid_file=pid_file,
        tank=tank,
        keepout=keepout,
        clock=clock,
        sleep=sleep,
        specs=specs,
        spares=spares,
        log=log,
    )
    session.tick_hook = tick_hook
    if setup is not None:
        setup(session)
    out = {"reason": None, "fit": None}
    try:
        session.begin()
        result = session.run_all()
        out["fit"] = result
        out["reason"] = "done"
    except BaseException as error:  # noqa: BLE001 - every exit stops the arm the same way
        out["reason"] = f"{error.__class__.__name__}: {error}"
        out["exception"] = error.__class__.__name__
    finally:
        out["stop"] = session.end(out["reason"] or "exit")
    out["session"] = session.report()
    fit = (out["fit"] or {}).get("fit")
    if fit is not None:
        out["placement"] = placement_report(fit, keepout)
        if fit["valid"]:
            tank_file, keepout_file = candidate_files(
                fit, out["fit"]["fit_touches"], tank, keepout, folder.name
            )
            dump(folder / "candidate-tank.PENDING-APPROVAL.json", tank_file)
            dump(folder / "candidate-keepout.PENDING-APPROVAL.json", keepout_file)
            out["candidate_files"] = [
                str(folder / "candidate-tank.PENDING-APPROVAL.json"),
                str(folder / "candidate-keepout.PENDING-APPROVAL.json"),
            ]
    dump(
        folder / "autocal-result.json",
        {k: v for k, v in out.items() if k != "session"}
        | {
            "touches": out["session"]["touches"],
            "rejected": out["session"]["rejected"],
            "metrics": out["session"]["metrics"],
            "peaks": out["session"]["peaks"],
        },
    )
    dump(folder / "probes.json", out["session"]["probes"])
    out["session_object"] = session
    return out
