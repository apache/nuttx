#!/usr/bin/env python3
############################################################################
# tools/nxmake2cmake.py
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

__version__ = "0.0.1"

import argparse
import os
import sys
from string import Template

if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")


def board_cmakelist_get(path):
    template = """# ##############################################################################
# $board_path/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more contributor
# license agreements.  See the NOTICE file distributed with this work for
# additional information regarding copyright ownership.  The ASF licenses this
# file to you under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License.  You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations under
# the License.
#
# ##############################################################################

add_subdirectory(src)
"""
    temp_obj = Template(template)
    return temp_obj.substitute(board_path=path)


def add_board_cmakelist(path):
    cmpath = os.path.join(path, "CMakeLists.txt")

    print("Add", cmpath)

    with open(cmpath, "w") as f:
        content = board_cmakelist_get(path)
        f.write(content)


def file_update_path(s, oldpath, newpath):
    return s.replace(oldpath, newpath)


def convert_make_conditional(content):
    content = content.replace(",y)", "")
    content = content.replace("else ifeq ($", "elseif")
    content = content.replace("ifeq ($", "if")
    content = content.replace("ifneq ($(", "if(NOT ")
    content = content.replace("else\n", "else()\n")
    content = content.replace("endif\n", "endif()\n")
    content = content.replace("CHIP_CSRCS = ", "set(SRCS ")
    content = content.replace("CHIP_CSRCS += ", "  list(APPEND SRCS ")
    content = content.replace("CSRCS = ", "set(SRCS ")
    content = content.replace("CSRCS += ", "  list(APPEND SRCS ")
    content = content.replace(".c\n", ".c)\n")

    return content


# common for board Makefile and Make.defs
def convert_board_common(path, content):
    content = convert_make_conditional(content)

    # remove make specific lines
    content = "\n".join(line for line in content.split("\n") if "include" not in line)
    content = "\n".join(line for line in content.split("\n") if "ASRCS" not in line)
    content = "\n".join(line for line in content.split("\n") if "DEPPATH" not in line)
    content = "\n".join(line for line in content.split("\n") if "VPATH" not in line)
    content = "\n".join(line for line in content.split("\n") if "CFLAGS" not in line)

    content += "target_sources(board PRIVATE ${SRCS})\n\n"

    # linker script
    if os.path.exists(os.path.join(path, "scripts/ld.script")):
        content += 'set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/ld.script")'
    elif os.path.exists(os.path.join(path, "scripts/flash.ld")):
        content += 'set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/flash.ld")'
    else:
        print("ERROR: not found linker script")

    content += "\n"
    content = content.replace("\n\n\n", "\n\n")
    content = content.replace("SRCS  ", "SRCS ")

    return content


# convert board src/Make.defs to CMakeLists.txt
def convert_board_makedefs(path):
    mdpath = os.path.join(path, "src/Make.defs")
    cpath = os.path.join(path, "src/CMakeLists.txt")

    content = ""
    with open(mdpath, "r") as fm:
        content = fm.read()
        content = content.replace("src/Make.defs", "src/CMakeLists.txt")
        content = convert_board_common(path, content)

    if content:
        print("add", cpath)
        with open(cpath, "w") as fc:
            fc.write(content)


# convert board src/Makefile to CMakeLists.txt
def convert_board_makefile(path):
    mpath = os.path.join(path, "src/Makefile")
    cpath = os.path.join(path, "src/CMakeLists.txt")

    content = ""
    with open(mpath, "r") as fm:
        content = fm.read()
        content = content.replace("src/Make.defs", "src/CMakeLists.txt")
        content = convert_board_common(path, content)

    if content:
        print("add", cpath)
        with open(cpath, "w") as fc:
            fc.write(content)


# convert board Make build to CMake
def convert_board(path):
    add_board_cmakelist(path)

    mpath = os.path.join(path, "src/Makefile")
    mdpath = os.path.join(path, "src/Make.defs")

    if os.path.exists(mpath):
        convert_board_makefile(path)

    elif os.path.exists(mdpath):
        convert_board_makedefs(path)
    else:
        print("board Makefile or Make.defs not found in ", path)
        exit(1)


# convert arch Make.defs to CMakeLists.txt
def convert_arch_makedefs(path):
    mdpath = os.path.join(path, "Make.defs")
    cpath = os.path.join(path, "CMakeLists.txt")

    content = ""
    with open(mdpath, "r") as fm:
        content = fm.read()
        content = content.replace("Make.defs", "CMakeLists.txt")
        content = convert_make_conditional(content)

        content += "\n"

        content += "target_sources(arch PRIVATE ${SRCS})\n"

        content = "\n".join(
            line for line in content.split("\n") if "include" not in line
        )

        content = content.replace("\n\n\n", "\n\n")
        content = content.replace("SRCS  ", "SRCS ")

    if content:
        print("add", cpath)
        with open(cpath, "w") as fc:
            fc.write(content)


# convert arch Make build to CMake
def convert_arch(path):
    convert_arch_makedefs(path)


def main():
    parser = argparse.ArgumentParser(
        prog="nxmake2cmake",
        formatter_class=argparse.RawTextHelpFormatter,
        description=(
            """
            A tool that convert NuttX Makefile to CMakeLists file"
            Usage:
              nxmake2cmake.py <path-dir>
            """
        ),
        epilog=(
            """
            You can use this tool to on any system locale.
            """
        ),
    )

    # Version
    parser.add_argument("-v", "--version", action="version", version=__version__)

    parser.add_argument(
        "path",
        type=str,
        help=(
            """
            The path pointing to the board or arch directory.
            If the directory contains Makefiles or Make.defs files, they will be converted into CMakeLists files.

            eg.:

               arch  ->  arch/arm/src/stm32f0l0g0

               board ->  boards/arm/stm32/nucleo-f302r8
            """
        ),
    )

    args = parser.parse_args()

    if not os.path.exists(args.path):
        print("invalid path", args.path)
        exit(1)

    path_split = args.path.split("/")

    if "boards" in args.path:
        if len(path_split) != 4 or not len(path_split[3]):
            print("invalid board path")
            exit(1)

        convert_board(args.path)

    elif "arch" in args.path:
        if len(path_split) != 4 or not len(path_split[3]):
            print("invalid arch path")
            exit(1)

        print("convert arch")
        convert_arch(args.path)
    else:
        print("not supported path", args.path)
        exit(1)


if __name__ == "__main__":
    main()
