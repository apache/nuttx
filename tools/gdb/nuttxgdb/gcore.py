############################################################################
# tools/gdb/nuttxgdb/gcore.py
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

import argparse
import os
import shutil

import gdb

from . import utils


def create_file_with_size(filename, size):
    with open(filename, "wb") as f:
        f.write(b"\0" * size)


def parse_args(args):
    parser = argparse.ArgumentParser()
    parser.add_argument("-o", "--output", help="Gcore output file")
    parser.add_argument(
        "-t",
        "--objcopy",
        help="Select the appropriate architecture for the objcopy tool",
        type=str,
    )

    parser.add_argument(
        "--trust-readonly",
        help="Trust readonly section from elf, bypass read from device.",
        action="store_true",
        default=True,
    )

    parser.add_argument(
        "--no-trust-readonly",
        help="Do not trust readonly section from elf, read from device.",
        action="store_false",
        dest="trust_readonly",
    )

    parser.add_argument(
        "-r",
        "--memrange",
        type=str,
    )
    return parser.parse_args(args)


class NXGcore(gdb.Command):
    def __init__(self):
        self.tempfile = utils.import_check(
            "tempfile",
            errmsg="No tempfile module found, please try gdb-multiarch instead.\n",
        )

        if not self.tempfile:
            return

        super().__init__("nxgcore", gdb.COMMAND_USER)

    def invoke(self, args, from_tty):
        try:
            args = parse_args(gdb.string_to_argv(args))
        except SystemExit:
            return

        objfile = gdb.current_progspace().objfiles()[0]
        elffile = objfile.filename
        tmpfile = self.tempfile.NamedTemporaryFile(suffix=".elf")

        # Create temporary ELF file with sections for each memory region

        shutil.copy(elffile, tmpfile.name)

        # If no parameters are passed in

        if args.output:
            corefile = args.output
        else:
            corefile = elffile + ".core"

        # Obtain memory range from macros or parameters

        if args.memrange:
            memregion = args.memrange
        else:
            memregion = str(utils.get_symbol_value("CONFIG_BOARD_MEMORY_RANGE"))
            if len(memregion) < 3:
                print(
                    "Please set CONFIG_BOARD_MEMORY_RANGE.\n"
                    "Memory range of board. format: <start>,<end>,<flags>,....\n"
                    "start: start address of memory range\n"
                    "end: end address of memory range\n"
                    "flags: Readable 0x1, writable 0x2, executable 0x4\n"
                    "example:0x1000,0x2000,0x1,0x2000,0x3000,0x3,0x3000,0x4000,0x7"
                )
                return

        # Resolve memory range and shell run commands

        values = memregion.replace('"', "").split(",")

        for i in range(0, len(values), 3):
            start = int(values[i], 16)
            end = int(values[i + 1], 16)

            # Create a random section name

            section = tmpfile.name + f"{i // 3}"

            # Add objcopy insertion segment command and modify segment start address command

            create_file_with_size(section, end - start)

            os.system(
                f"{args.objcopy} --add-section {section}={section} "
                f"--set-section-flags {section}=noload,alloc {tmpfile.name}"
            )
            os.system(
                f"{args.objcopy} --change-section-address "
                f"{section}={hex(start)} {tmpfile.name}"
            )

            os.remove(section)

        gdb.execute(f"file {tmpfile.name}")

        if args.trust_readonly:
            gdb.execute("set trust-readonly-sections on")

        gdb.execute(f"gcore {corefile}")

        if args.trust_readonly:
            # Restore trust-readonly-sections to default off state
            gdb.execute("set trust-readonly-sections off")

        gdb.execute(f'file "{elffile}"')
        tmpfile.close()

        print(f"Please run gdbserver.py to parse {corefile}")
