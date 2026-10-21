#!/usr/bin/env python
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
"""A2: characterize ONE Star actuator under explicit per-command mW caps.

Use --dry-run first (ping/monitor ONLY). Powered runs require a human holding
and supporting the leader, with the physical power switch in reach. No host
cleanup can release a servo after bus loss, SIGKILL or loss of host power.
The mW field is not a calibrated hand-force or torque bound. --stop-test opens no port.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import time
from dataclasses import asdict
from datetime import UTC, datetime
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from makermodslab.fashionstar_raw import (  # noqa: E402
    DAMPING,
    IDS,
    PING,
    Guard,
    RawBus,
    SafetyError,
    Travel,
    angle_packet,
    power_limit,
    stop_packet,
)

PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
EVIDENCE = Path("configs/viscous/viscous_01/evidence")


def interrupt(signum, _frame):
    raise KeyboardInterrupt(f"received {signal.Signals(signum).name}")


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--joint", default="wrist_flex", help="exactly one joint name")
    p.add_argument("--port", default=PORT)
    p.add_argument("--powers", default="100,200,400,800", help="positive integer mW, comma separated")
    p.add_argument(
        "--allow-power", type=int, metavar="MAX_MW", help="explicit ceiling override above 1000 mW"
    )
    p.add_argument("--move-deg", type=float, default=10.0, help="0 < excursion <= 20 degrees")
    p.add_argument("--move-s", type=float, default=1.0, help="positive duration, <=65.535 s")
    p.add_argument("--phase-s", type=float, default=8.0, help="hold/damping duration, 0 < N <= 60 s")
    p.add_argument(
        "--dry-run", action="store_true", help="only ping/read requests; no state writes, even on exit"
    )
    p.add_argument("--stop-test", action="store_true", help="wait for signal; no port is opened")
    p.add_argument("--yes", action="store_true", help="skip powered-run 'go' confirmation")
    p.add_argument(
        "--only",
        choices=("hold", "move", "damping", "latency"),
        action="append",
        help="run only these phase kinds (repeatable); lets a chat-driven session go one step at a time",
    )
    return p


def validate(args):
    maximum = args.allow_power if args.allow_power is not None else 1000
    power_limit(1, maximum)
    powers = [int(value.strip()) for value in args.powers.split(",")]
    if not powers or len(set(powers)) != len(powers):
        raise ValueError("Powers must be a nonempty list of distinct positive integer mW values")
    for power in powers:
        power_limit(power, maximum)
    for name, high in (("move_deg", 20), ("move_s", 65.535), ("phase_s", 60)):
        value = getattr(args, name)
        if not math.isfinite(value) or not 0 < value <= high:
            raise ValueError(f"{name} must be finite and in (0, {high}]")
    if round(args.move_s * 1000) < 1:
        raise ValueError("move_s must encode at least 1 ms")
    return powers, maximum


def configuration(args):
    # Config data only: no leader object, connect/configure, motorbridge or reset.
    from lerobot.teleoperators.rebot_102_leader.config_rebot_102_leader_viscous import (
        RebotArm102LeaderViscousConfig,
    )

    config = RebotArm102LeaderViscousConfig(port=args.port)
    if set(config.joint_ids.values()) != set(IDS):
        raise ValueError("Expected all seven servo IDs 0-6")
    if args.joint not in config.joint_ids:
        raise ValueError(f"Unknown joint {args.joint}; choose one of {list(config.joint_ids)}")
    direction = config.joint_directions[args.joint]
    offset = config.joint_offsets.get(args.joint, 0)
    lo, hi = sorted((bound - offset) / direction for bound in config.joint_ranges[args.joint])
    return config, Travel(lo, hi)


def quantize(value):
    # Nearest tenth, matching angle_packet; validate the actual encoded value.
    return int(round(value * 10)) / 10


def phase_plan(args, motor_id, current, travel, powers):
    current = travel.check(quantize(current))
    plus = travel.check(quantize(current + args.move_deg))
    minus = travel.check(quantize(current - args.move_deg))
    step = travel.check(quantize(current + 2.5))
    move_ms = round(args.move_s * 1000)
    hold_ms = min(move_ms, max(1, round(args.phase_s * 1000)))
    phases = []
    for power in powers:
        phases.append(
            {
                "kind": "hold",
                "power_mw": power,
                "seconds": args.phase_s,
                "targets": [current],
                "duration_ms": hold_ms,
            }
        )
    for power in powers:
        phases.append(
            {
                "kind": "move",
                "power_mw": power,
                "seconds": 4 * args.move_s,
                "targets": [plus, current, minus, current],
                "duration_ms": move_ms,
            }
        )
    for power in powers:
        phases.append({"kind": "damping", "power_mw": power, "seconds": args.phase_s, "targets": []})
    phases.append(
        {
            "kind": "latency",
            "power_mw": sorted(powers)[len(powers) // 2],
            "seconds": 2 * args.move_s,
            "targets": [step, current],
            "duration_ms": move_ms,
        }
    )
    if getattr(args, "only", None):
        phases = [phase for phase in phases if phase["kind"] in args.only]
    for phase in phases:
        phase["commands"] = (
            [stop_packet(motor_id, DAMPING, phase["power_mw"], max_power_mw=args.maximum).hex(" ")]
            if phase["kind"] == "damping"
            else [
                angle_packet(
                    motor_id, target, phase["duration_ms"], phase["power_mw"], max_power_mw=args.maximum
                ).hex(" ")
                for target in phase["targets"]
            ]
        )
    return phases


class Evidence:
    def __init__(self, args, *, clock=time.monotonic):
        self.clock, self.started = clock, clock()
        stamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%S.%fZ")
        self.path = EVIDENCE / f"star-actuation-{stamp}.json"
        self.data = {
            "goal": "A2: characterize hand-held Star actuator power caps before leader drive",
            "started_utc": datetime.now(UTC).isoformat(),
            "arguments": vars(args).copy(),
            "path": self.path.as_posix(),
            "events": [],
            "samples": [],
            "outcome": "running",
        }
        self.last_save = -math.inf

    def emit(self, kind, **fields):
        self.data["events"].append({"t": self.clock() - self.started, "kind": kind, **fields})
        if self.clock() - self.last_save >= 0.5:
            self.save()

    def save(self):
        path = REPO / self.path
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(".json.tmp")
        with temporary.open("w") as stream:
            json.dump(self.data, stream, allow_nan=False, separators=(",", ":"))
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        self.last_save = self.clock()


def sample_window(bus, evidence, phase, seconds, reference, *, clock=time.monotonic, sleep=time.sleep):
    started, deadline = clock(), clock() + seconds
    next_tick = started
    rows = []
    while clock() < deadline:
        # All seven temperatures checked each tick; selected angle/current sampled at nominal 50 Hz.
        selected = bus.monitor(bus.guard.motor_id)
        measured = clock()
        row = {
            "t": measured - evidence.started,
            "phase_t": measured - started,
            "phase": phase["kind"],
            "command_power_mw": phase["power_mw"],
            "reference_deg": reference,
            "deflection_deg": selected.angle_deg - reference,
            **asdict(selected),
        }
        evidence.data["samples"].append(row)
        rows.append(row)
        for motor_id in IDS:
            if motor_id != bus.guard.motor_id:
                bus.monitor(motor_id)
        next_tick += 0.02
        if clock() > next_tick:
            evidence.emit("sample_overrun", seconds=clock() - next_tick)
            next_tick = clock()  # no catch-up bursts or invented 50 Hz timestamps
        sleep(max(0, min(next_tick, deadline) - clock()))
    return rows


def describe(args, config, travel, current, phases):
    print(f"A2: ONLY {args.joint}, servo {config.joint_ids[args.joint]}, {config.baudrate} baud 8N1")
    print(
        f"Star derived travel [{travel.low:.6f}, {travel.high:.6f}] deg; "
        f"command travel [{travel.low + travel.margin:.6f}, {travel.high - travel.margin:.6f}] (5 deg margin)"
    )
    print(
        f"Snapshot current {current:.1f} deg; abort envelope "
        f"[{current - args.move_deg - 10:.1f}, {current + args.move_deg + 10:.1f}]; any servo >=60 C trips"
    )
    print("0x08 <BhHH>: id, signed 0.1deg, time ms, power mW; no resets or acceleration variants")
    print("Powers mW:", args.powers, "ceiling:", args.maximum)
    print("Powered startup, between phases, and exit: release ALL IDs 0-6, twice each:")
    for motor_id in IDS:
        print(f"  x2 {stop_packet(motor_id, 0x10).hex(' ')}")
    print("After every release: monitor all IDs; <=45 mA looks near 30 mA rest, cannot prove release.")
    print("All targets below are inside Star command travel and single-turn frame. No clipping/wrapping.")
    print("Live phases re-read CURRENT angle while released; recompute/refuse targets before energizing.")
    for phase in phases:
        print(
            f"{phase['kind'].upper()} {phase['power_mw']} mW: {phase['seconds']:.3f} s; "
            f"targets={phase['targets']} deg; duration/command={phase.get('duration_ms', 'n/a')} ms"
        )
        for command in phase["commands"]:
            print("  WOULD SEND", command)
    print("HOLD: push gently. MOVE: both +excursion and -excursion then back; human may block.")
    print(
        "DAMPING: move by hand; feel is recorded by human. LATENCY: +2.5 deg, then back; threshold 0.3 deg."
    )
    print(
        "Telemetry nominal 50 Hz, actual timestamps/overruns logged; Enter between phases if stdin is a tty."
    )
    print("Ping 0x01 and monitor 0x16 are the ONLY requests sent in dry-run, including cleanup.", flush=True)


def run_phases(bus, evidence, args, travel, powers, plan):
    for index, planned in enumerate(plan):
        # Phase boundaries are released, so tty input never blocks an energized servo.
        if sys.stdin.isatty():
            input(f"Enter to begin {planned['kind']} {planned['power_mw']} mW (Ctrl-C stops): ")
        all_monitors = [bus.monitor(motor_id) for motor_id in IDS]
        current = all_monitors[bus.guard.motor_id].angle_deg
        phase = phase_plan(args, bus.guard.motor_id, current, travel, powers)[index]
        evidence.emit("phase", phase=phase, current_deg=current)
        evidence.save()
        kind, power = phase["kind"], phase["power_mw"]
        cue = {
            "hold": "push the joint gently now",
            "move": "joint moves +/-, you may block it",
            "damping": "move the joint by hand now",
            "latency": "small +2.5 deg step now",
        }[kind]
        print(f"{kind.upper()} {power} mW: {cue} ({phase['seconds']:g} s)", flush=True)
        if kind == "damping":
            bus.stop(bus.guard.motor_id, DAMPING, power)
            sample_window(bus, evidence, phase, args.phase_s, current)
        else:
            for leg, target in enumerate(phase["targets"]):
                commanded = time.monotonic()
                bus.angle(bus.guard.motor_id, target, phase["duration_ms"], power, travel)
                rows = sample_window(
                    bus, evidence, phase, args.phase_s if kind == "hold" else args.move_s, current
                )
                if kind == "move":
                    final = rows[-1]["angle_deg"] if rows else None
                    evidence.emit(
                        "move_result",
                        power_mw=power,
                        target_deg=target,
                        final_deg=final,
                        completion="completed"
                        if final is not None and abs(final - target) <= 1
                        else "blocked_or_incomplete",
                    )
                if kind == "latency" and leg == 0:
                    first = next((r for r in rows if abs(r["angle_deg"] - current) >= 0.3), None)
                    evidence.emit(
                        "latency_result",
                        threshold_deg=0.3,
                        first_motion_s=(first["t"] + evidence.started - commanded) if first else None,
                        note="host write-to-read upper bound; includes serial/poll latency; hand may block",
                    )
        report = bus.release_all()
        if report["write_errors"] or any(r["looks_released"] is not True for r in report["readback"]):
            raise SafetyError(
                "Release readback uncertain; stop before the next phase and check physical power"
            )
        evidence.save()


def summary(data):
    result = {}
    for power in data["arguments"]["powers_list"]:
        rows = [r for r in data["samples"] if r["command_power_mw"] == power]
        holds = [r for r in rows if r["phase"] == "hold"]
        result[power] = {
            "peak_current_ma": max((r["current_ma"] for r in rows), default=None),
            "hold_deflection_vs_time": [[r["phase_t"], r["deflection_deg"]] for r in holds],
            "hold_peak_abs_deflection_deg": max((abs(r["deflection_deg"]) for r in holds), default=None),
            "moves": [e for e in data["events"] if e["kind"] == "move_result" and e["power_mw"] == power],
            "damping_feel": "human observation required; not inferred from unsigned current/power",
        }
    return result


def main(argv=None):
    args = parser().parse_args(argv)
    # Explicitly override inherited nohup SIGINT ignore before ANY waiting/hardware.
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    if args.stop_test:
        print(f"Stop test: pid {os.getpid()} waiting for SIGINT or SIGTERM; NO PORT", flush=True)
        try:
            time.sleep(120)
        except KeyboardInterrupt as exc:
            print(f"Stop test passed: {exc}; no hardware opened", flush=True)
            return 0
        print("Stop test FAILED: no signal within 120 s", flush=True)
        return 1
    powers, args.maximum = validate(args)
    args.powers_list = powers
    config, travel = configuration(args)
    motor_id = config.joint_ids[args.joint]
    if not args.dry_run:
        print(
            f"ONLY {args.joint} (id {motor_id}) will be energized; ALL other servos released. "
            "Hold/support leader; physical power switch is the hard stop.",
            flush=True,
        )
        if not args.yes and input("Energize this ONE servo? Type 'go': ").strip() != "go":
            print("Cancelled before opening port")
            return 0
    evidence = Evidence(args)
    evidence.data["config"] = {
        "joint_ids": config.joint_ids,
        "baudrate": config.baudrate,
        "star_travel": asdict(travel),
    }
    exit_code = 0
    try:
        import serial

        # exclusive=True uses the POSIX advisory exclusive lock. Never create a native bus.
        with RawBus(
            serial.Serial(
                port=None, baudrate=config.baudrate, timeout=0.03, write_timeout=0.1, exclusive=True
            ),
            read_only=args.dry_run,
            max_power_mw=args.maximum,
            emit=evidence.emit,
        ) as bus:
            # Own cleanup before open, including a signal/exception during opening.
            bus.port.port = args.port
            bus.port.open()
            if not args.dry_run:
                release = bus.release_all()
                if release["write_errors"] or any(
                    r["looks_released"] is not True for r in release["readback"]
                ):
                    raise SafetyError("Initial release uncertain; refuse energizing")
            monitors = []
            for servo in IDS:
                bus.request(PING, servo)
                monitors.append(bus.monitor(servo))
                print("READ ONLY preflight:", json.dumps(asdict(monitors[-1])), flush=True)
            current = monitors[motor_id].angle_deg
            bus.guard = Guard(motor_id, current, args.move_deg)
            plan = phase_plan(args, motor_id, current, travel, powers)
            evidence.data["preflight"] = [asdict(m) for m in monitors]
            evidence.data["plan"] = plan
            describe(args, config, travel, current, plan)
            evidence.save()
            if args.dry_run:
                evidence.data["outcome"] = "dry-run: ping/monitor only; no servo-state writes"
            else:
                run_phases(bus, evidence, args, travel, powers, plan)
                evidence.data["outcome"] = "completed"
        if not args.dry_run:
            evidence.data["final_release"] = bus.released
            if bus.released["write_errors"] or any(
                r["looks_released"] is not True for r in bus.released["readback"]
            ):
                raise SafetyError("Final release uncertain; check physical power")
    except (Exception, KeyboardInterrupt) as exc:
        evidence.data["outcome"] = f"aborted: {type(exc).__name__}: {exc}"
        print(evidence.data["outcome"], flush=True)
        exit_code = 130 if isinstance(exc, KeyboardInterrupt) else 1
    finally:
        evidence.data["summary"] = summary(evidence.data)
        evidence.save()
        print(
            "Summary:",
            json.dumps(
                {
                    p: {k: v for k, v in s.items() if k != "hold_deflection_vs_time"}
                    for p, s in evidence.data["summary"].items()
                }
            ),
            flush=True,
        )
        print(f"Evidence: {evidence.path.as_posix()}", flush=True)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
