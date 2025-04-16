#!/usr/bin/env python3
# tools/symbolshare.py
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

import argparse
import os
import subprocess
import sys
import tempfile

import lief


def find_symbol_in_section(elf_path, section_names):
    symbols = []
    sections = []
    binary = lief.parse(elf_path)
    for section_name in section_names:
        section = binary.get_section(section_name)
        if section is None:
            print(f"Section '{section_name}' not found in {elf_path}")
            continue

        sections.append(
            (
                section.virtual_address,
                section.virtual_address + section.size,
                section_name,
            )
        )

    for symbol in binary.symbols:
        for start, end, section_name in sections:
            if (
                symbol.value < end
                and symbol.value >= start
                and symbol.type == lief.ELF.Symbol.TYPE.OBJECT
            ):
                symbols.append((symbol.name, symbol.value, symbol.size, section_name))

    return symbols


def objfile_iter(path):
    for p in path:
        temp_dir = tempfile.mkdtemp()
        subprocess.run(["ar", "x", os.path.abspath(p)], cwd=temp_dir)
        for filename in os.listdir(temp_dir):
            yield os.path.join(temp_dir, filename)


def find_symbols_in_library(libpath):
    undefined_names = []
    defined_names = []

    for obj in objfile_iter(libpath):
        binary = lief.parse(obj)
        for symbol in binary.symbols:
            if symbol.type == lief.ELF.Symbol.TYPE.NOTYPE:
                undefined_names.append(symbol.name)
            else:
                defined_names.append(symbol.name)

    defined_names = set(defined_names)
    undefined_names = list(set(undefined_names) - defined_names)
    return (defined_names, undefined_names)


def args_parser():
    parser = argparse.ArgumentParser(description="Show symbols in ELF sections.")
    parser.add_argument("-e", "--elf", required=True, help="Path to the ELF file")
    parser.add_argument("-l", "--ld", default="tmp.ld", help="Output link script")
    parser.add_argument(
        "-s", "--section", required=True, nargs="+", help="Section names to inspect"
    )
    parser.add_argument(
        "-a",
        "--libs",
        required=True,
        nargs="+",
        help="Library paths to inspect for symbols",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = args_parser()
    section_symbols = find_symbol_in_section(args.elf, args.section)
    defined_names, undefined_names = find_symbols_in_library(args.libs)

    section_names = {symbol[0] for symbol in section_symbols}
    common_names = section_names.intersection(defined_names)
    if len(common_names) != 0:
        print("Common symbols between section and library:")
        for symbol in common_names:
            print(symbol)
        sys.exit(-1)

    with open(args.ld, "w") as fd:
        for symbol in section_symbols:
            fd.write(f"{symbol[0]} = 0x{symbol[1]:x};\n")
