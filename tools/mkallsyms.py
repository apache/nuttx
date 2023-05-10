#!/usr/bin/env python3
############################################################################
# tools/mkallsyms.py
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

import errno
import os
import re
import sys

try:
    import cxxfilt
    from elftools import __version__
    from elftools.elf.elffile import ELFFile
    from elftools.elf.sections import SymbolTableSection

except ModuleNotFoundError:
    print("Please execute the following command to install dependencies:")
    print("pip install pyelftools cxxfilt")


class SymbolTables(object):
    def __init__(self, elffile, output):
        try:
            file = open(elffile, "rb")
            self.elffile = ELFFile(file)
        except FileNotFoundError:
            self.elffile = None
        self.output = output
        self.symbol_list = []

    def symbol_filter(self, symbol):
        if symbol["st_info"]["type"] != "STT_FUNC":
            return None
        if symbol["st_info"]["bind"] == "STB_WEAK":
            return None
        if symbol["st_shndx"] == "SHN_UNDEF":
            return None
        return symbol

    def print_symbol_tables(self, isnoconst=False):
        noconst = "const"
        if not isnoconst:
            noconst = ""

        self.emitline("#include <nuttx/compiler.h>")
        self.emitline("#include <nuttx/symtab.h>\n")
        self.emitline("extern int g_nallsyms;\n")
        self.emitline(
            "extern struct symtab_s g_allsyms[%d + 2];\n" % len(self.symbol_list)
        )
        self.emitline("%s int g_nallsyms = %d + 2;" % (noconst, len(self.symbol_list)))
        self.emitline(
            "%s struct symtab_s g_allsyms[%d + 2] =\n{"
            % (noconst, len(self.symbol_list))
        )
        self.emitline('  { "Unknown", (FAR %s void *)0x00000000 },' % (noconst))
        for symbol in self.symbol_list:
            self.emitline(
                '  { "%s", (FAR %s void *)%s },' % (symbol[1], noconst, hex(symbol[0]))
            )
        self.emitline('  { "Unknown", (FAR %s void *)0xffffffff }\n};' % (noconst))

    def get_symtable(self):
        symbol_tables = [
            (idx, s)
            for idx, s in enumerate(self.elffile.iter_sections())
            if isinstance(s, SymbolTableSection)
        ]

        if not symbol_tables and self.elffile.num_sections() == 0:
            self.emitline("")
            return

        for section_index, section in symbol_tables:
            if not isinstance(section, SymbolTableSection):
                continue

            if section["sh_entsize"] == 0 or section.name != ".symtab":
                continue

            return section

    def parse_symbol(self):
        if self.elffile is None:
            return
        symtable = self.get_symtable()
        for nsym, symbol in enumerate(symtable.iter_symbols()):
            if self.symbol_filter(symbol) is not None:
                try :
                    symbol_name = cxxfilt.demangle(symbol.name)
                    func_name = re.sub(r"\(.*$", "", symbol_name)
                except:
                    symbol_name = symbol.name
                self.symbol_list.append((symbol["st_value"] & ~0x01, func_name))
        self.symbol_list = sorted(self.symbol_list, key=lambda item: item[0])

    def emitline(self, s=""):
        self.output.write(str(s) + "\n")


def usage():
    print("Usage: mkallsyms.py [noconst] <ELFBIN> [output file]")
    os._exit(errno.ENOENT)


def parse_args(argv):
    index = 1
    argc = len(argv)
    outfile = None
    elffile = None

    if argc > index and argv[index] == "--version":
        print("mkallsyms.py: based on pyelftools %s" % __version__)
        os.exit(0)

    if argc > index and argv[index] == "noconst":
        noconst = True
        index += 1
    else:
        noconst = False

    if argc > index:
        elffile = argv[index]
        index += 1

    if argc > index:
        outfile = open(argv[index], "w")
    else:
        outfile = sys.stdout

    return noconst, elffile, outfile


if __name__ == "__main__":
    noconst, elffile, outfile = parse_args(sys.argv)
    readelf = SymbolTables(elffile, outfile)
    readelf.parse_symbol()
    readelf.print_symbol_tables(noconst)
