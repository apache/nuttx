#!/usr/bin/env python3
############################################################################
# arch/sim/src/patch_macho_initsection.py
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
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
# implied.  See the License for the specific language governing
# permissions and limitations under the License.
#
############################################################################

"""Patch Mach-O init section type flags to prevent dyld from
auto-running C++ constructors.

Changes MOD_INIT_FUNC_POINTERS (0x9) section types to REGULAR (0x0)
so that dyld ignores them.  NuttX will invoke the constructors
explicitly from lib_cxx_initialize().

Requires: pip install lief
"""

import argparse
import subprocess
import sys

try:
    import lief
except ImportError:
    print(
        "Error: lief is required. Install with: pip install lief",
        file=sys.stderr,
    )
    sys.exit(1)


def main():
    parser = argparse.ArgumentParser(description="Patch Mach-O init section type flags")
    parser.add_argument("binary", help="Path to the Mach-O binary")
    args = parser.parse_args()

    binary = lief.MachO.parse(args.binary)
    if binary is None:
        print(f"Error: failed to parse {args.binary}", file=sys.stderr)
        sys.exit(1)

    fat = binary.at(0)
    T = lief.MachO.Section.TYPE
    patched = 0
    for section in fat.sections:
        if section.type == T.MOD_INIT_FUNC_POINTERS:
            section.type = T.REGULAR
            patched += 1

    if patched:
        fat.write(args.binary)

        if sys.platform == "darwin":
            subprocess.run(
                ["codesign", "--force", "--sign", "-", args.binary],
                check=True,
            )
            subprocess.run(
                ["codesign", "--verify", "--verbose", args.binary],
                check=True,
            )

    return 0


if __name__ == "__main__":
    sys.exit(main())
