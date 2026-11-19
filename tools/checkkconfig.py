#!/usr/bin/env python3
# tools/checkkconfig.py
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

#
# [checkkconfig.py] is a tool that simulates the effects of modifying a CONFIG item,
# Can be used to check whether my config changes are what I expected.
#
# usage: checkkconfig.py [-h] -f FILE (-s CONFIG VALUE | -d DIFF)
#
# optional arguments:
#  -h, --help            show this help message and exit
#  -f FILE, --file FILE  Path to the input defconfig file
#  -s CONFIG_XXX VALUE, --single CONFIG VALUE
#                        Analyze single change: CONFIG_NAME y/m/n
#  -d DIFF, --diff DIFF  Analyze changes from diff file
#
# example: ./tools/checkkconfig.py -f defconfig -s ELF n
#
# outputs:
# Change report for ELF=n
# Config Option                            Old                  New
# ----------------------------------------------------------------------
# BINFMT_LOADABLE                          y                    n
# ELF                                      y                    n
# ELF_STACKSIZE                            8192                 <unset>
# LIBC_ARCH_ELF                            y                    n
# LIBC_MODLIB                              y                    n
# MODLIB_ALIGN_LOG2                        2                    <unset>
# MODLIB_BUFFERINCR                        32                   <unset>
# MODLIB_BUFFERSIZE                        32                   <unset>
# MODLIB_MAXDEPEND                         2                    <unset>
# MODLIB_RELOCATION_BUFFERCOUNT            256                  <unset>
# MODLIB_SYMBOL_CACHECOUNT                 256                  <unset>
#
# As we can see, we can clearly know that
# if I turn off ELF in defconfig at this time,
# it will bring about the following configuration linkage changes
#
# It can also parse diff files, which can be used to check the changes of multiple configs.
# diff file example:
#  diff --git a/boards/demo/configs/nsh/defconfig b/boards/demo/configs/nsh/defconfig
#  index cf7d07c..5de20d4 100644
#  --- a/boards/demo/configs/nsh/defconfig
#  +++ b/boards/demo/configs/nsh/defconfig
#  @@ -51,7 +51,6 @@ CONFIG_ARMV7A_STRING_FUNCTION=y
#   CONFIG_ARM_PSCI=y
#   CONFIG_ARM_SEMIHOSTING_HOSTFS=y
#   CONFIG_ARM_THUMB=y
#  -CONFIG_AUDIO=y
#   CONFIG_BCH=y
#   CONFIG_BINFMT_ELF_EXECUTABLE=y
#   CONFIG_BLUETOOTH=y
#  @@ -104,7 +103,6 @@ CONFIG_DRIVERS_VIRTIO_SERIAL=y
#   CONFIG_DRIVERS_VIRTIO_SOUND=y
#   CONFIG_DRIVERS_WIFI_SIM=y
#   CONFIG_DRIVERS_WIRELESS=y
#  -CONFIG_ELF=y
#   CONFIG_ETC_ROMFS=y
#   CONFIG_EVENT_FD=y
#   CONFIG_EXAMPLES_FB=y
#
# example: ./tools/checkkconfig.py -f defconfig -d changes.diff
#
# outputs:
# Change report for diff: changes.diff
# Config Option                            Old                  New
# ----------------------------------------------------------------------
# AUDIO                                    y                    n
# AUDIO_BUFFER_NUMBYTES                    8192                 <unset>
# AUDIO_EXCLUDE_EQUALIZER                  y                    n
# AUDIO_EXCLUDE_REWIND                     y                    n
# AUDIO_FORMAT_AMR                         y                    n
# AUDIO_FORMAT_MP3                         y                    n
# AUDIO_FORMAT_OPUS                        y                    n
# AUDIO_FORMAT_PCM                         y                    n
# AUDIO_FORMAT_SBC                         y                    n
# AUDIO_NUM_BUFFERS                        2                    <unset>
# BINFMT_LOADABLE                          y                    n
# ELF                                      y                    n
# ELF_STACKSIZE                            8192                 <unset>
# LIBC_ARCH_ELF                            y                    n
# LIBC_MODLIB                              y                    n
# MODLIB_ALIGN_LOG2                        2                    <unset>
# MODLIB_BUFFERINCR                        32                   <unset>
# MODLIB_BUFFERSIZE                        32                   <unset>
# MODLIB_MAXDEPEND                         2                    <unset>
# MODLIB_RELOCATION_BUFFERCOUNT            256                  <unset>
# MODLIB_SYMBOL_CACHECOUNT                 256                  <unset>
# NXPLAYER_COMMAND_LINE                    y                    n
# NXPLAYER_DEFAULT_MEDIADIR                /music               <unset>
# NXPLAYER_FMT_FROM_EXT                    y                    n
# NXPLAYER_INCLUDE_DEVICE_SEARCH           y                    n
# NXPLAYER_INCLUDE_HELP                    y                    n
# NXPLAYER_INCLUDE_MEDIADIR                y                    n
# NXPLAYER_INCLUDE_PREFERRED_DEVICE        y                    n
# NXPLAYER_MAINTHREAD_STACKSIZE            8192                 <unset>
# NXPLAYER_PLAYTHREAD_STACKSIZE            8192                 <unset>
# NXRECORDER_COMMAND_LINE                  y                    n
# NXRECORDER_FMT_FROM_EXT                  y                    n
# NXRECORDER_INCLUDE_HELP                  y                    n
# NXRECORDER_MAINTHREAD_STACKSIZE          8192                 <unset>
# NXRECORDER_RECORDTHREAD_STACKSIZE        8192                 <unset>
# SYSTEM_NXPLAYER                          y                    n
# SYSTEM_NXRECORDER                        y                    n
#
#
# RECAUTION:
# Because NuttX apps Kconfig of menu is generated by build system,
# and arch/board bridge Kconfig is symlink to real arch board dir.
# So it is best to check the defconfig that has been configured.
# If the environment does not generate Kconfig menu, etc.
# the tool will execute `configure.sh` and distclean at the end.
#

import argparse
import os
import re
import subprocess
import sys
from pathlib import Path

try:
    from kconfiglib import Kconfig
except ImportError:
    print(
        "ERROR: checkkconfig tool depends on kconfiglib for parse Kconfig tree.\nplease install it.\npip install kconfiglib",
        file=sys.stderr,
    )
    sys.exit(1)

TOPDIR = Path(__file__).resolve().parent.parent
DPATH = Path("defconfig")
CMAKE_BINARY_DIR = None
NEED_RESET = False
os.chdir(TOPDIR)


# Prepare environment for Kconfig
def prepare_env():
    global NEED_RESET
    if CMAKE_BINARY_DIR:
        # we are in cmake env
        os.environ["APPSDIR"] = str(TOPDIR / Path("../apps"))
        os.environ["APPSBINDIR"] = str(CMAKE_BINARY_DIR / Path("apps"))
        os.environ["EXTERNALDIR"] = "dummy"
        os.environ["BINDIR"] = str(CMAKE_BINARY_DIR)
        os.environ["KCONFIG_CONFIG"] = str(DPATH)
    else:
        # check if we are in the configured Kconfig environment
        full_config_file = TOPDIR / Path(".config")
        if not full_config_file.exists():
            print("apps preconfig do not generate yet \nrun configure.sh first")
            result = subprocess.run(
                [f"{TOPDIR}/tools/configure.sh", "-e", f"{str(DPATH.parent)}"],
                capture_output=True,
                text=True,
            )
            if result.returncode != 0:
                print(
                    f"ERROR: {TOPDIR}/tools/configure.sh run fail\n configure path: {str(DPATH.parent)}",
                    file=sys.stderr,
                )
                print(f"STDERROR:{result.stderr}", file=sys.stderr)
                sys.exit(1)
            NEED_RESET = True
        os.environ["APPSDIR"] = "../apps"
        os.environ["APPSBINDIR"] = "../apps"
        os.environ["EXTERNALDIR"] = "dummy"
        os.environ["BINDIR"] = str(TOPDIR)
        os.environ["KCONFIG_CONFIG"] = str(DPATH)


# Reset environment to previous
def reset_env():
    os.environ.pop("APPSDIR", None)
    os.environ.pop("APPSBINDIR", None)
    os.environ.pop("EXTERNALDIR", None)
    os.environ.pop("BINDIR", None)
    os.environ.pop("KCONFIG_CONFIG", None)
    if NEED_RESET:
        result = subprocess.run(
            ["make", "distclean"], cwd=TOPDIR, capture_output=True, text=True
        )
        print(result.stdout)
        if result.returncode != 0:
            print(
                "ERROR: distclean error please clean up your workspace manually",
                file=sys.stderr,
            )
            print(f"STDERROR:{result.stderr}", file=sys.stderr)


# Parse a diff file and return a dict of changes
def parse_diff(diff_path):
    changes = {}
    diff_pattern = re.compile(r"^([+-])CONFIG_(\w+)=([ymn])$")
    with open(diff_path, "r") as f:
        for line in f:
            line = line.strip()

            match = diff_pattern.match(line)
            if not match:
                continue

            op, name, value = match.groups()
            full_name = f"{name}"

            if op == "-":
                changes[full_name] = "n"
            elif op == "+":
                changes[full_name] = value.lower()
    return changes


# Apply a set of changes to a Kconfig tree and return a list of changed symbols
def apply_changes(kconf, changes):

    # step 1: keep track of original values
    orig_state = {sym.name: sym.str_value for sym in kconf.defined_syms}

    # step 2: apply the config settings
    value_map = {"n": 0, "m": 1, "y": 2}
    for target, value in changes.items():
        sym = kconf.syms.get(target)
        if not sym:
            print(f"Warning: {target} not found, skipped")
            continue
        if value not in value_map:
            sym.set_value(value)
            print(f"Setting the value {value} for {target} directly")
            continue
        sym.set_value(value_map[value])

    # step 3: check for changes
    changed = []
    for sym in kconf.defined_syms:
        orig = orig_state.get(sym.name, "")
        curr = sym.str_value
        if orig != curr:
            changed.append((sym.name, orig, curr))

    return changed


def track_multiple_changes(config_changes):
    kconf = Kconfig()
    kconf.load_config()

    # Convert list of [config, value, config, value, ...] to dictionary
    changes_dict = {}
    for i in range(0, len(config_changes), 2):
        if i + 1 < len(config_changes):
            config_name = config_changes[i]
            config_value = config_changes[i + 1].lower()
            changes_dict[config_name] = config_value
    return apply_changes(kconf, changes_dict)


def track_diff_changes(diff_path):
    kconf = Kconfig()
    kconf.load_config()
    changes = parse_diff(diff_path)
    return apply_changes(kconf, changes)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Simulate the effects of modifying CONFIG items in NuttX"
    )
    parser.add_argument(
        "-o", "--out_dir", required=False, help="Binary dir for CMake configured"
    )
    parser.add_argument(
        "-f", "--file", required=True, help="Path to the input defconfig file"
    )
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument(
        "-s",
        "--set",
        nargs="+",
        metavar=("CONFIG", "VALUE"),
        help="Analyze multiple changes: CONFIG_NAME1 y/m/n CONFIG_NAME2 y/m/n ...",
    )
    group.add_argument("-d", "--diff", help="Analyze changes from diff file")

    args = parser.parse_args()

    if args.out_dir:
        CMAKE_BINARY_DIR = Path(args.out_dir)

    DPATH = Path(args.file)

    if not DPATH.is_absolute():
        DPATH = TOPDIR / DPATH

    if not DPATH.exists:
        print("ERROR: defconfig file DO NOT exist", file=sys.stderr)
        sys.exit(1)

    prepare_env()

    if args.set:
        # Check if we have an even number of arguments
        if len(args.set) % 2 != 0:
            print(
                "ERROR: -s/--set requires an even number of arguments (config-value pairs)",
                file=sys.stderr,
            )
            sys.exit(1)
        changes = track_multiple_changes(args.set)
        title = "Change report for config changes"

    elif args.diff:
        changes = track_diff_changes(args.diff)
        title = f"Change report for diff: {args.diff}"

    reset_env()

    print(f"\n{title}")
    print(f"{'Config Option':<40} {'Old':<20} {'New':<20}")
    print("-" * 70)
    for name, old, new in sorted(changes, key=lambda x: x[0]):
        print(f"{name:<40} {old or '<unset>':<20} {new or '<unset>':<20}")
