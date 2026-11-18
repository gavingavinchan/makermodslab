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
"""Viscous tank operator. The serve/op/ops interface is adapted from metal-arm-harness cli."""

import argparse
import contextlib
import fcntl
import json
import math
import os
import queue
import secrets
import shlex
import socket
import sys
import threading
import time
from pathlib import Path

from transport import address, send

ARCHIVE = Path("~/dream-run-archive/mac-run-data/live-20261117T082827Z").expanduser()
ROBOT = "viscous_waterproof_01_dry_air"


def receive(listener, token, requests, done):
    """Untrusted slow clients never block motor telemetry. One command in flight."""
    while not done.is_set():
        try:
            sock, _ = listener.accept()
        except TimeoutError:
            continue
        except OSError:
            return
        with sock, sock.makefile("rwb") as io:
            try:
                sock.settimeout(2)
                packet = json.loads(io.readline(4097))
                if (
                    not isinstance(packet, dict)
                    or not isinstance(packet.get("token"), str)
                    or not secrets.compare_digest(packet["token"], token)
                ):
                    raise ValueError("authentication refused")
                words = packet.get("command")
                if (
                    not isinstance(words, list)
                    or len(words) > 12
                    or any(not isinstance(w, str) or len(w) > 128 for w in words)
                ):
                    raise ValueError("invalid command")
                response = queue.Queue(1)
                requests.put((words, response))
                while True:
                    try:
                        reply = response.get(timeout=0.1)
                        break
                    except queue.Empty:
                        if done.is_set():
                            return
            except (ValueError, OSError) as e:
                reply = {"ok": False, "guard": str(e)}
            with contextlib.suppress(OSError):
                io.write(json.dumps(reply).encode() + b"\n")
                io.flush()


def serve(args):
    from model import Model, core, real
    from session import Operator, PassiveLabArm, ReplayArm

    lab = args.backend == "lab"
    if lab and (args.environment != "tank" or args.fake_cube or args.frames):
        raise ValueError("lab requires tank and record cameras")
    if lab and sys.platform != "darwin":
        raise ValueError("lab backend is for Gavin's Mac")
    if args.budget < 1:
        raise ValueError("budget must be positive")
    if args.fake_cube and not all(math.isfinite(v) for v in args.fake_cube):
        raise ValueError("fake cube coordinates must be finite metres")
    os.umask(0o077)
    env = real.ENVIRONMENTS[args.environment]
    home = real.lab_home(None)
    root = (args.run_root or real.run_data.RUN_DATA).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    pid = args.pid_file.expanduser().resolve()
    with contextlib.ExitStack() as stack:
        lock = stack.enter_context(Path(str(pid) + ".lock").open("a"))
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if pid.exists() and real.run_data.pid_alive(int(pid.read_text().strip())):
            raise ValueError(f"pid file already names a live process: {pid}")
        events = real.install_stop_signals(pid)
        stack.callback(real.remove_pid_file, pid)
        bundle = real.run_data.RunBundle.start(
            time.strftime("astra-%Y%m%dT%H%M%SZ", time.gmtime()),
            root,
            meta={"backend": args.backend, "armed": args.armed, "environment": env.name},
        )
        stack.callback(bundle.finish)
        folder = bundle.dir.resolve()
        if lab:
            os.environ["MAKERMODSLAB_HOME"] = str(home)
            from makermodslab.dream_camera_lock import terminal_run
            from makermodslab.utils.config import dream_terminal_run_path

            # A second operator cannot claim this physical unit with a different pid file.
            owner = stack.enter_context((home / "astra-waterproof.lock").open("a"))
            fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
            record = real.load_record(home, ROBOT)
            cams = {c["name"]: c for c in record["cameras"]}
            if (
                record["name"] != ROBOT
                or record["follower_config"] != "viscous_waterproof_01"
                or set(cams) != {"head", "left"}
                or cams["left"].get("rotation") != 180
            ):
                raise ValueError("expected waterproof unit profile, head and rotated left camera")
            limits, _ = real.follower_limits(home, record)
            station = Path("~/viscous-dream-real/station/arms/viscous-waterproof-tank.json").expanduser()
            arm_path = (args.arm_file or station).expanduser()
            stack.enter_context(terminal_run("astra", folder.name, path=dream_terminal_run_path(home)))
            url = "http://127.0.0.1:8000"
            if holder := real.lab_holder(url, False) or real.port_holders(record["follower_port"]):
                raise ValueError(f"Lab/port not idle: {holder}")
            real.release_lab_previews(url, False)
        else:
            from make_tank_arm import PROFILE

            limits = json.loads(PROFILE.read_text())["follower"]["joint_limits"]
            arm_path = (args.arm_file or real.DEFAULT_ARM_FILE).expanduser()
        model = Model(arm_path, env, folder, limits, lab)
        if args.floor_calibration:
            data = json.loads(args.floor_calibration.read_text())
            a, b = data["samples_mm"]
            if (
                data["backend"] != args.backend
                or data["environment"] != env.name
                or data["urdf_sha256"] != model.sha
                or not math.isfinite(a + b)
                or abs(a - b) > 3
                or data["floor_mm"] != (a + b) / 2
            ):
                raise ValueError("floor record mismatch")
            model.guard.planes["floor"] = data["floor_mm"]
        arm = (
            (real.LabArm if args.armed else PassiveLabArm)(home, record, 15)
            if lab
            else ReplayArm(
                model, (args.frames or ARCHIVE / "frames").expanduser(), args.fake_cube, real.VirtualClock()
            )
        )
        session = Operator(arm, model, folder, env, args.armed, events, pid, args.budget)
        stack.callback(session.trace.close)
        listener = stack.enter_context(socket.socket(socket.AF_INET if args.tcp else socket.AF_UNIX))
        token = secrets.token_hex(32)
        endpoint = {"token": token, "backend": args.backend, "budget": args.budget}
        if args.tcp:
            listener.bind(address(args.tcp))
            endpoint["tcp"] = "127.0.0.1:" + str(listener.getsockname()[1])
        else:
            path = args.socket or folder / "op.sock"
            listener.bind(str(path.resolve()))
            stack.callback(path.unlink, missing_ok=True)
            endpoint["socket"] = str(path.resolve())
        listener.listen(4)
        listener.settimeout(0.2)
        requests = queue.Queue(1)
        done = threading.Event()
        thread = threading.Thread(target=receive, args=(listener, token, requests, done), daemon=True)
        reason = "server exit"
        response = None
        try:
            session.begin()
            core.atomic_json(folder / "connection.json", endpoint)
            (folder / "agent").mkdir(mode=0o700)
            thread.start()
            session_pointer = Path(str(pid) + ".session")
            session_pointer.write_text(str(folder) + "\n")
            stack.callback(session_pointer.unlink, missing_ok=True)
            print(f"READY {folder}", flush=True)
            while not session.finished and not events[0].is_set():
                try:
                    words, response = requests.get(timeout=core.ROW_DT_S)
                except queue.Empty:
                    if not thread.is_alive():
                        raise RuntimeError("operator socket listener exited") from None
                    session.poll()
                    continue
                reply = session.reply(words)
                response.put(reply)
                response = None
            reason = "red button" if events[0].is_set() else reason
        except BaseException as exc:
            reason = str(exc)
            if not events[0].is_set():
                raise
        finally:
            try:
                stop = session.end(reason)
                if response is not None:
                    fault = {"ok": False, "guard": reason, "stop": stop}
                    # The stop takes precedence over disk, JPEG encoding, and socket I/O.
                    if session.reading is not None:
                        try:
                            fault.update(session.observation())
                            fault["observation_phase"] = "last available sample before stop"
                        except Exception as problem:
                            fault["observation_error"] = str(problem)
                    session.log(reply=fault)
                    response.put(fault)
            finally:
                done.set()
                if thread.is_alive():
                    thread.join(2.5)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest="mode", required=True)
    s = sub.add_parser("serve")
    s.add_argument("--backend", choices=("fake", "lab"), required=True)
    s.add_argument("--environment", choices=("tank", "dry_air"), default="tank")
    s.add_argument("--armed", action="store_true")
    s.add_argument("--arm-file", type=Path)
    s.add_argument("--pid-file", type=Path, default=Path("/tmp/viscous-astra.pid"))
    s.add_argument("--run-root", type=Path)
    s.add_argument("--frames", type=Path)
    s.add_argument("--fake-cube", nargs=2, type=float, metavar=("X", "Y"))
    s.add_argument("--floor-calibration", type=Path)
    s.add_argument("--budget", type=int, default=40)
    net = s.add_mutually_exclusive_group()
    net.add_argument("--socket", type=Path)
    net.add_argument("--tcp")
    for name in ("op", "ops"):
        c = sub.add_parser(name)
        c.add_argument("--session", type=Path, default=os.getenv("ASTRA_SESSION"))
        c.add_argument("--json", action="store_true")
        c.add_argument("words", nargs=argparse.REMAINDER)
    args = p.parse_args(argv)
    if args.mode == "serve":
        serve(args)
        return 0
    if "--json" in args.words:
        args.words = [word for word in args.words if word != "--json"]
        args.json = True
    if not args.words:
        p.error("supply at least one operator command")
    if not args.session:
        p.error("set ASTRA_SESSION or pass --session DIR before the command")
    cmds = [args.words] if args.mode == "op" else map(shlex.split, args.words)
    for words in cmds:
        reply = send(args.session, words)
        print(json.dumps(reply) if args.json else "\n".join(f"{k} = {v}" for k, v in reply.items()))
        if not reply["ok"]:
            return 1
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError) as error:
        sys.exit(str(error))
