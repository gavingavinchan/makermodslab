#!/usr/bin/env python3
"""Restore the commissioned waterproof arm's files without opening hardware."""

import argparse
import hashlib
import json
import os
from pathlib import Path

DEFAULT_BUNDLE = Path(__file__).resolve().parents[1] / "configs/viscous/viscous_waterproof_01"


def restore(bundle, lab_home, calibration_home, follower_port, leader_port, *, dry_run=False):
    bundle, lab_home, calibration_home = map(Path, (bundle, lab_home, calibration_home))
    manifest = json.loads((bundle / "manifest.json").read_text())
    for relative, expected in manifest["files"].items():
        source = bundle / relative
        if not source.resolve().is_relative_to(bundle.resolve()):
            raise ValueError(f"Bundle path escapes its directory: {relative}")
        if hashlib.sha256(source.read_bytes()).hexdigest() != expected:
            raise ValueError(f"Bundle checksum failed: {relative}")

    files = {}
    for relative in (
        "robots/viscous_follower/viscous_waterproof_01.json",
        "teleoperators/rebot_102_leader/star_viscous_waterproof_01.json",
    ):
        files[calibration_home / relative] = json.loads((bundle / "calibration" / relative).read_text())
    for name in ("viscous_waterproof_01", "star_viscous_waterproof_01"):
        files[lab_home / "viscous_profiles" / f"{name}.json"] = json.loads(
            (bundle / "viscous_profiles" / f"{name}.json").read_text()
        )
    record = json.loads((bundle / "robot-record-template.json").read_text())
    record.update(follower_port=follower_port, leader_port=leader_port)
    files[lab_home / "robots/viscous_waterproof_01.json"] = record

    # Check the complete plan first, so an existing different calibration is
    # never silently replaced and a refused restore writes nothing.
    for destination, payload in files.items():
        if destination.exists() and json.loads(destination.read_text()) != payload:
            raise FileExistsError(f"Existing file differs; preserve it before restoring: {destination}")
    for destination, payload in files.items():
        if not dry_run and not destination.exists():
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_text(json.dumps(payload, indent=2) + "\n")
    return list(files)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--follower-port", required=True, help="CAN adapter port on this machine")
    parser.add_argument("--leader-port", required=True, help="Purple Star leader port on this machine")
    parser.add_argument(
        "--lab-home",
        type=Path,
        default=Path(os.environ.get("MAKERMODSLAB_HOME", "~/.makermods/makermodslab-viscous")).expanduser(),
        help="Must match the state directory used by the Lab server",
    )
    parser.add_argument(
        "--calibration-home",
        type=Path,
        default=Path("~/.cache/huggingface/lerobot/calibration").expanduser(),
    )
    parser.add_argument("--bundle", type=Path, default=DEFAULT_BUNDLE)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    try:
        paths = restore(
            args.bundle,
            args.lab_home,
            args.calibration_home,
            args.follower_port,
            args.leader_port,
            dry_run=args.dry_run,
        )
    except (OSError, ValueError) as error:
        parser.exit(1, f"Restore refused: {error}\n")
    print("Restore plan:" if args.dry_run else "Restored or already matching:")
    for path in paths:
        print(f"  {path}")
    print(
        "Start Lab and select viscous_waterproof_01. This command did not open hardware or change motor zeros."
    )


if __name__ == "__main__":
    main()
