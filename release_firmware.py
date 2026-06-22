#!/usr/bin/env python3
"""
Build FrictionTester Frame firmware and publish a GitHub release asset.

Examples:
  ./release_firmware.py v1.2.3
  ./release_firmware.py v1.2.3 --dry-run
  ./release_firmware.py v1.2.3 --skip-release
"""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
from pathlib import Path


PROJECT_TITLE = "Frame"
ASSET_PREFIX = "frame"
ENVIRONMENT = "esp32s3"
REPO = Path(__file__).resolve().parent
VERSION_HEADER = Path("include/version.h")
FIRMWARE_BIN = Path(".pio") / "build" / ENVIRONMENT / "firmware.bin"


def run(command: list[str], cwd: Path, dry_run: bool = False) -> None:
    printable = " ".join(command)
    print(f"\n[{cwd}] {printable}")
    if dry_run:
        return
    subprocess.run(command, cwd=cwd, check=True)


def parse_tag(tag: str) -> tuple[int, int, int]:
    match = re.fullmatch(r"v?(\d+)\.(\d+)\.(\d+)", tag)
    if not match:
        raise ValueError("Tag must look like v1.2.3 or 1.2.3")
    return tuple(int(part) for part in match.groups())


def normalize_tag(tag: str) -> str:
    return tag if tag.startswith("v") else f"v{tag}"


def require_tool(name: str) -> None:
    if shutil.which(name) is None:
        raise RuntimeError(f"Required command not found on PATH: {name}")


def update_version_header(version: tuple[int, int, int], dry_run: bool) -> None:
    header_path = REPO / VERSION_HEADER
    if not header_path.exists():
        raise FileNotFoundError(f"Missing version header: {header_path}")

    major, minor, patch = version
    original = header_path.read_text()
    updated = original
    replacements = {
        r"^#define VERSION_MAJOR \d+$": f"#define VERSION_MAJOR {major}",
        r"^#define VERSION_MINOR \d+$": f"#define VERSION_MINOR {minor}",
        r"^#define VERSION_PATCH \d+$": f"#define VERSION_PATCH {patch}",
    }

    for pattern, replacement in replacements.items():
        updated, count = re.subn(pattern, replacement, updated, count=1, flags=re.MULTILINE)
        if count != 1:
            raise RuntimeError(f"Could not update {pattern!r} in {header_path}")

    if updated == original:
        print(f"{header_path} already has version {major}.{minor}.{patch}")
        return

    print(f"Updating {header_path} to {major}.{minor}.{patch}")
    if not dry_run:
        header_path.write_text(updated)


def build_project(dry_run: bool) -> Path:
    run(["pio", "run", "-e", ENVIRONMENT], cwd=REPO, dry_run=dry_run)

    source = REPO / FIRMWARE_BIN
    if dry_run:
        return source
    if not source.exists():
        raise FileNotFoundError(f"Build finished, but firmware was not found: {source}")
    return source


def copy_asset(tag: str, firmware_bin: Path, dry_run: bool) -> Path:
    asset = REPO / f"{ASSET_PREFIX}-{tag}.bin"
    print(f"Copying {firmware_bin} -> {asset}")
    if not dry_run:
        shutil.copy2(firmware_bin, asset)
    return asset


def create_release(tag: str, asset: Path, dry_run: bool) -> None:
    run(
        [
            "gh",
            "release",
            "create",
            tag,
            asset.name,
            "--title",
            f"{PROJECT_TITLE} {tag}",
            "--notes",
            f"{PROJECT_TITLE} firmware {tag}",
        ],
        cwd=REPO,
        dry_run=dry_run,
    )


def git_status_short() -> str:
    result = subprocess.run(
        ["git", "status", "--short"],
        cwd=REPO,
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )
    return result.stdout.strip()


def main() -> int:
    parser = argparse.ArgumentParser(
        description=f"Update {PROJECT_TITLE} firmware version, build PlatformIO firmware, and create a GitHub release.",
    )
    parser.add_argument("tag", help="Release tag, for example v1.2.3")
    parser.add_argument(
        "--skip-release",
        action="store_true",
        help="Build and copy the asset without calling gh release create.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print actions without changing files, building, or creating a release.",
    )
    parser.add_argument(
        "--require-clean",
        action="store_true",
        help="Fail if this repo has uncommitted changes before the version bump.",
    )
    args = parser.parse_args()

    tag = normalize_tag(args.tag)
    version = parse_tag(tag)

    if not args.dry_run:
        require_tool("pio")
        if not args.skip_release:
            require_tool("gh")

    status = git_status_short()
    if status and args.require_clean:
        raise RuntimeError(f"{REPO} has uncommitted changes. Commit/stash them or rerun without --require-clean.")
    if status:
        print(f"\nWarning: {REPO} already has uncommitted changes:\n{status}")

    update_version_header(version, dry_run=args.dry_run)
    firmware_bin = build_project(dry_run=args.dry_run)
    asset = copy_asset(tag, firmware_bin, dry_run=args.dry_run)
    if args.skip_release:
        print(f"Skipping GitHub release. Asset path: {asset}")
    else:
        create_release(tag, asset, dry_run=args.dry_run)

    print(f"\nDone: {PROJECT_TITLE} {tag}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (FileNotFoundError, RuntimeError, ValueError, subprocess.CalledProcessError) as error:
        print(f"\nError: {error}", file=sys.stderr)
        raise SystemExit(1)
