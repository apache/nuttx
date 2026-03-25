#!/usr/bin/env python3
############################################################################
# boards/arm/ht32f491x3/esk32/tools/flash.py
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

# Wrapper that dispatches to flash.sh in WSL and flash.ps1 on Windows.

import os
import platform
import shutil
import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent


def is_windows_host():
    system = platform.system().lower()
    return os.name == "nt" or system.startswith("msys") or system.startswith("cygwin")


def is_wsl():
    if "WSL_INTEROP" in os.environ or "WSL_DISTRO_NAME" in os.environ:
        return True

    release = platform.release().lower()
    version = platform.version().lower()
    return "microsoft" in release or "microsoft" in version


def build_command(argv):
    if is_windows_host():
        powershell = shutil.which("powershell.exe") or shutil.which("pwsh.exe")
        if powershell is None:
            raise RuntimeError("Unable to find powershell.exe or pwsh.exe in PATH.")

        backend = SCRIPT_DIR / "flash.ps1"
        return [powershell, "-ExecutionPolicy", "Bypass", "-File", str(backend), *argv]

    if sys.platform.startswith("linux"):
        if not is_wsl():
            raise RuntimeError(
                "Unsupported host: this wrapper supports Windows native and WSL."
            )

        bash = shutil.which("bash")
        if bash is None:
            raise RuntimeError("Unable to find bash in PATH.")

        backend = SCRIPT_DIR / "flash.sh"
        return [bash, str(backend), *argv]

    raise RuntimeError(
        "Unsupported host: this wrapper supports Windows native and WSL."
    )


def main(argv):
    try:
        cmd = build_command(argv)
        completed = subprocess.run(cmd, check=False)
        return completed.returncode
    except RuntimeError as err:
        print(err, file=sys.stderr)
        return 1
    except OSError as err:
        print(f"Failed to start backend script: {err}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
