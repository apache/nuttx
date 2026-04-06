#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0

"""Convenience wrapper to register apps repositories.

Equivalent to:
  python tools/register-nuttx.py <name> --kind apps --path <apps-path>
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path


def main() -> int:
    script = Path(__file__).with_name("register-nuttx.py")
    cmd = [sys.executable, str(script)]
    cmd.extend(sys.argv[1:])

    if "--kind" not in cmd:
  cmd.extend(["--kind", "apps"])

    return subprocess.call(cmd)


if __name__ == "__main__":
    raise SystemExit(main())
