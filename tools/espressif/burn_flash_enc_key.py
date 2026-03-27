#!/usr/bin/env python3
# tools/espressif/burn_flash_enc_key.py
#
# SPDX-License-Identifier: Apache-2.0
#
# Burn the flash encryption key into hardware eFuses (same role as
# tools/espressif/Config.mk BURN_EFUSES + espefuse.py burn_key).
#
# Invocation
# ----------
# Normally you do not run this script by hand. The CMake target ``burn_enc_key``
# (arch/risc-v/src/common/espressif/CMakeLists.txt) runs:
#
#   cmake -E env NUTTX_ESPEFUSE=<path> NUTTX_KEY=<path> -- python3 "$0"
#
# The serial port is not passed by CMake: set ESPTOOL_PORT in the environment
# when you run the build, e.g.:
#
#   ESPTOOL_PORT=/dev/ttyUSB0 NOCHECK=1 cmake --build <builddir> -t burn_enc_key
#
# Environment variables
# ---------------------
# Required (set by CMake via -E env):
#   NUTTX_ESPEFUSE   Absolute path to espefuse.py (from find_program at configure time).
#   NUTTX_KEY        Absolute path to the XTS_AES_128 flash encryption key (.bin).
#
# Required (your shell / build environment):
#   ESPTOOL_PORT     Serial device for espefuse.py -p (e.g. /dev/ttyUSB0).
#
# Required (safety gate; same idea as ``make NOCHECK`` in Config.mk):
#   NOCHECK          Must be set (any value) so non-interactive burn is explicit.
#                    If unset, the script exits with an error before calling espefuse.
#
# Behaviour
# ---------
# 1. Runs ``espefuse.py --port $ESPTOOL_PORT summary``; fails if that command fails.
# 2. If the summary shows ``?? ??`` in the BLOCK1 region, the key is treated as
#    already programmed and the script exits 0 without burning again.
# 3. Otherwise runs ``burn_key BLOCK_KEY0 <key> XTS_AES_128_KEY``, always with
#    ``--do-not-confirm`` when NOCHECK is set (as required above).
#
# Exit status: 0 on skip (already burned) or successful burn; non-zero on error.

from __future__ import annotations

import os
import subprocess
import sys


def _efuse_summary_lines(summary_out: str) -> str:
    """Text from espefuse summary corresponding to ``grep -A1 BLOCK1``."""

    chunks: list[str] = []
    lines = summary_out.splitlines()
    for i, line in enumerate(lines):
        if "BLOCK1" in line:
            chunks.append(line)
            if i + 1 < len(lines):
                chunks.append(lines[i + 1])
    return "\n".join(chunks)


def main() -> int:
    for var in ("NUTTX_ESPEFUSE", "ESPTOOL_PORT", "NUTTX_KEY"):
        if var not in os.environ or not os.environ[var]:
            print(
                f"Error: {var} is not set or empty.",
                file=sys.stderr,
            )
            return 1

    if "NOCHECK" not in os.environ:
        print(
            "Error: NOCHECK is not set; refusing to run without explicit confirmation bypass.",
            file=sys.stderr,
        )
        print(
            "Set NOCHECK=1 (or NOCHECK with any value) to proceed non-interactively.",
            file=sys.stderr,
        )
        return 1

    espefuse = os.environ["NUTTX_ESPEFUSE"]
    port = os.environ["ESPTOOL_PORT"]
    key_path = os.environ["NUTTX_KEY"]

    proc = subprocess.run(
        [espefuse, "--port", port, "summary"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    summary_out = proc.stdout or ""
    if proc.returncode != 0:
        print(
            f"espefuse.py summary failed for {port}: {summary_out}",
            file=sys.stderr,
        )
        return proc.returncode

    efuse_summary = _efuse_summary_lines(summary_out)
    if "?? ??" in efuse_summary:
        print("Encryption key already burned. Skipping...")
        return 0

    print("Burning flash encryption key...")
    burn = subprocess.run(
        [
            espefuse,
            "--do-not-confirm",
            "--port",
            port,
            "burn_key",
            "BLOCK_KEY0",
            key_path,
            "XTS_AES_128_KEY",
        ],
    )
    return burn.returncode


if __name__ == "__main__":
    sys.exit(main())
