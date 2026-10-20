#!/usr/bin/env python3
"""Restore a commissioned Viscous arm's files without opening hardware.

Every name comes from the bundle itself: the robot record from
robot-record-template.json, calibrations from calibration/, and hardware profiles
from viscous_profiles/ when the bundle has them (unit 1 has none; it runs on the
driver's built-in presets). The default bundle is the waterproof arm's.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path

BUNDLES = Path(__file__).resolve().parents[1] / "configs/viscous"
DEFAULT_BUNDLE = BUNDLES / "viscous_waterproof_01"


def _bundle_files(bundle, folder, verified):
    """The JSON files under one bundle folder, each of which must be checksummed."""
    root = bundle / folder
    if not root.is_dir():
        return []
    relatives = sorted(path.relative_to(bundle).as_posix() for path in root.rglob("*.json"))
    for relative in relatives:
        if relative not in verified:
            raise ValueError(f"Bundle file is not in its manifest: {relative}")
    return relatives


def restore(bundle, lab_home, calibration_home, follower_port, leader_port, *, dry_run=False):
    bundle, lab_home, calibration_home = map(Path, (bundle, lab_home, calibration_home))
    manifest = json.loads((bundle / "manifest.json").read_text())
    for relative, expected in manifest["files"].items():
        source = bundle / relative
        if not source.resolve().is_relative_to(bundle.resolve()):
            raise ValueError(f"Bundle path escapes its directory: {relative}")
        if hashlib.sha256(source.read_bytes()).hexdigest() != expected:
            raise ValueError(f"Bundle checksum failed: {relative}")
    verified = set(manifest["files"])
    if "robot-record-template.json" not in verified:
        raise ValueError("Bundle manifest does not cover robot-record-template.json")

    record = json.loads((bundle / "robot-record-template.json").read_text())
    name = record.get("name")
    if not isinstance(name, str) or not name or Path(name).name != name:
        raise ValueError(f"Invalid robot name in bundle template: {name!r}")
    if manifest.get("arm") != name:
        raise ValueError(f"Bundle manifest arm {manifest.get('arm')!r} does not match its robot {name!r}")

    calibrations = _bundle_files(bundle, "calibration", verified)
    stems = {Path(relative).stem for relative in calibrations}
    for key in ("follower_config", "leader_config"):
        if record.get(key) not in stems:
            raise ValueError(f"Bundle has no calibration for the template's {key}: {record.get(key)!r}")

    files = {}
    for relative in calibrations:
        files[calibration_home / Path(relative).relative_to("calibration")] = json.loads(
            (bundle / relative).read_text()
        )
    for relative in _bundle_files(bundle, "viscous_profiles", verified):
        files[lab_home / relative] = json.loads((bundle / relative).read_text())
    record.update(follower_port=follower_port, leader_port=leader_port)
    files[lab_home / "robots" / f"{name}.json"] = record

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
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--follower-port", required=True, help="CAN adapter port on this machine")
    parser.add_argument("--leader-port", required=True, help="Star leader port on this machine")
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
    parser.add_argument(
        "--bundle",
        type=Path,
        default=DEFAULT_BUNDLE,
        help=f"Arm bundle directory, e.g. {BUNDLES / 'viscous_01'} (default: the waterproof arm)",
    )
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
    print(f"Start Lab and select {paths[-1].stem}. This command did not open hardware or change motor zeros.")


if __name__ == "__main__":
    main()
