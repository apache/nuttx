#!/usr/bin/env python3
"""
init.py  --  Out-of-tree NuttX configure

Resolves NuttX/apps paths from the CMake User Package Registry, then
directly invokes cmake configure + build.  No intermediate files.

Usage:
    python init.py --nuttx_instance nuttx_v1 \\
                   --nuttx_apps_instance nuttx_apps_v1 \\
                   --board_config ics-stm32h723zg:boot

    python init.py ... -B build/mybuild         # custom build dir
    python init.py ... --external_nuttx_apps_dir apps # default: ./apps
    python init.py ... --external_nuttx_dir nuttx  # default: ./nuttx
    python init.py ... --config-only           # configure only
    python init.py ... --clean                 # clean before configure

SPDX-License-Identifier: Apache-2.0
"""

from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path


# ---------------------------------------------------------------------------
#  Registry helpers
# ---------------------------------------------------------------------------

def _read_registry_package_dir(instance: str) -> Path | None:
    """Find the CMake package directory for *instance* from user registry."""

    if os.name == "nt":
        import winreg

        key_path = rf"Software\Kitware\CMake\Packages\{instance}"
        try:
            with winreg.OpenKey(winreg.HKEY_CURRENT_USER, key_path) as key:
                idx = 0
                while True:
                    try:
                        _, value, _ = winreg.EnumValue(key, idx)
                        p = Path(value)
                        if p.is_dir():
                            return p
                        idx += 1
                    except OSError:
                        break
        except FileNotFoundError:
            pass
    else:
        pkg_root = Path.home() / ".cmake" / "packages" / instance
        if pkg_root.is_dir():
            for f in sorted(pkg_root.iterdir()):
                if f.is_file():
                    candidate = Path(f.read_text(encoding="utf-8").strip())
                    if candidate.is_dir():
                        return candidate
    return None


def _parse_cmake_variable(config_file: Path, var: str) -> str | None:
    """Extract set(<var> "value") from <name>Config.cmake."""
    text = config_file.read_text(encoding="utf-8")
    m = re.search(rf'set\(\s*{re.escape(var)}\s+"([^"]+)"\s*\)', text)
    return m.group(1) if m else None


def resolve_instance(name: str, cmake_var: str) -> str:
    """Resolve *name* via CMake registry and return the exported *cmake_var* path."""
    pkg_dir = _read_registry_package_dir(name)
    if pkg_dir is None:
        sys.exit(
            f"ERROR: instance '{name}' not found in CMake package registry.\n"
            f"  Register it:  python nuttx/tools/register-nuttx.py {name} --kind ..."
        )

    config = pkg_dir / f"{name}Config.cmake"
    if not config.exists():
        sys.exit(f"ERROR: {config} does not exist")

    value = _parse_cmake_variable(config, cmake_var)
    if not value:
        sys.exit(f"ERROR: variable '{cmake_var}' not found in {config}")

    if not Path(value).is_dir():
        sys.exit(f"ERROR: resolved path does not exist: {value}")

    return value


def _cmake_path(p: str | Path) -> str:
    """Forward-slash path for cmake."""
    return str(p).replace("\\", "/")


def _resolve_user_path(value: str, base_dir: Path) -> Path:
    """Resolve user-provided path; relative paths are based on *base_dir*."""
    p = Path(value).expanduser()
    if not p.is_absolute():
        p = base_dir / p
    return p.resolve()


# ---------------------------------------------------------------------------
#  cmake invocation
# ---------------------------------------------------------------------------

def run_cmd(args: list[str]) -> None:
    """Run a command, stream output in real-time, exit on failure."""
    print(f"[init] {' '.join(args)}")
    ret = subprocess.call(args)
    if ret != 0:
        sys.exit(ret)


# ---------------------------------------------------------------------------
#  CLI
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Out-of-tree NuttX configure & build"
    )
    parser.add_argument(
        "--nuttx_instance", required=True,
        help="Registered NuttX source instance name (e.g. nuttx_v1)",
    )
    parser.add_argument(
        "--nuttx_apps_instance", required=True,
        help="Registered NuttX apps instance name (e.g. nuttx_apps_v1)",
    )
    parser.add_argument(
        "--board_config", required=True,
        help="Board:config pair (e.g. ics-stm32h723zg:boot)",
    )
    parser.add_argument(
        "-B", "--build", default="build",
        help="Build directory (default: build)",
    )
    parser.add_argument(
        "--generator", default="Ninja",
        help="CMake generator (default: Ninja)",
    )
    parser.add_argument(
        "--external_nuttx_apps_dir", default="",
        help="External NuttX apps root (default: ./apps)",
    )
    parser.add_argument(
        "--external_nuttx_dir", default="",
        help="External NuttX extension root (default: ./nuttx)",
    )
    parser.add_argument(
        "--clean", action="store_true",
        help="Remove build directory before configure",
    )
    parser.add_argument(
        "--config-only", action="store_true",
        help="Only run cmake configure, skip build",
    )
    args = parser.parse_args()

    script_dir = Path(__file__).resolve().parent
    build_dir = (script_dir / args.build).resolve()

    # ---- resolve from registry ----
    nuttx_base = resolve_instance(args.nuttx_instance, "NUTTX_BASE")
    nuttx_apps = resolve_instance(args.nuttx_apps_instance, "NUTTX_APPS_BASE")

    if args.external_nuttx_apps_dir:
        external_nuttx_apps_dir_path = _resolve_user_path(
            args.external_nuttx_apps_dir, script_dir
        )
    else:
        external_nuttx_apps_dir_path = (script_dir / "apps").resolve()

    if args.external_nuttx_dir:
        external_nuttx_dir_path = _resolve_user_path(args.external_nuttx_dir, script_dir)
    else:
        external_nuttx_dir_path = (script_dir / "nuttx").resolve()

    if (external_nuttx_dir_path / "boards").exists():
        boards_dir_path = (external_nuttx_dir_path / "boards").resolve()
    else:
        boards_dir_path = (Path(nuttx_base) / "boards").resolve()

    if (external_nuttx_dir_path / "drivers").exists():
        drivers_platform_dir_path = (external_nuttx_dir_path / "drivers").resolve()
    else:
        drivers_platform_dir_path = None

    external_nuttx_apps_dir = _cmake_path(external_nuttx_apps_dir_path)
    external_nuttx_dir = _cmake_path(external_nuttx_dir_path)
    external_nuttx_dir_for_cmake = (
        external_nuttx_dir
        if (external_nuttx_dir_path / "Kconfig").exists()
        else ""
    )
    boards_dir = _cmake_path(boards_dir_path)
    drivers_platform_dir = (
        _cmake_path(drivers_platform_dir_path)
        if drivers_platform_dir_path is not None
        else ""
    )

    print(f"[init] NuttX source : {nuttx_base}")
    print(f"[init] NuttX apps   : {nuttx_apps}")
    print(f"[init] External NuttX apps: {external_nuttx_apps_dir}")
    print(f"[init] External NuttX: {external_nuttx_dir}")
    if not external_nuttx_dir_for_cmake:
        print("[init] EXTERNAL_NUTTX_DIR skipped (Kconfig not found)")
    print(f"[init] Boards dir   : {boards_dir}")
    if drivers_platform_dir:
        print(f"[init] Drivers dir  : {drivers_platform_dir}")
    print(f"[init] Board config : {args.board_config}")
    print(f"[init] Build dir    : {build_dir}")
    print()

    # ---- clean ----
    if args.clean and build_dir.exists():
        print(f"[init] Cleaning {build_dir} ...")
        shutil.rmtree(build_dir)

    # ---- cmake configure ----
    configure_args = [
        "cmake",
        "-G", args.generator,
        "-S", _cmake_path(nuttx_base),
        "-B", _cmake_path(build_dir),
        f"-DBOARD_CONFIG={args.board_config}",
        f"-DNUTTX_APPS_DIR={_cmake_path(nuttx_apps)}",
        f"-DEXTERNAL_NUTTX_APPS_DIR={_cmake_path(external_nuttx_apps_dir)}",
        f"-DNUTTX_EXTRA_BOARD_DIRS={boards_dir}",
    ]
    if external_nuttx_dir_for_cmake:
        configure_args.append(
            f"-DEXTERNAL_NUTTX_DIR={_cmake_path(external_nuttx_dir_for_cmake)}"
        )
    if drivers_platform_dir:
        configure_args.append(
            f"-DNUTTX_DRIVERS_PLATFORM_DIR={drivers_platform_dir}"
        )
    run_cmd(configure_args)

if __name__ == "__main__":
    main()
