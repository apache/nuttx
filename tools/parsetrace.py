#!/usr/bin/env python3
############################################################################
# tools/parsetrace.py
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
import bisect
import os
import re
from typing import Union

try:
    import cxxfilt
    import parse
    from elftools.elf.elffile import ELFFile
    from elftools.elf.sections import SymbolTableSection
    from pydantic import BaseModel

except ModuleNotFoundError:
    print("Please execute the following command to install dependencies:")
    print("pip install pyelftools cxxfilt pydantic parse")


class SymbolTables(object):
    def __init__(self, file):
        elffile = open(file, "rb")
        self.elffile = ELFFile(elffile)
        self.symbol_dict = dict()
        self.addr_list = list()

    def __symbol_filter(self, symbol):
        if symbol["st_info"]["type"] != "STT_FUNC":
            return None
        if symbol["st_info"]["bind"] == "STB_WEAK":
            return None
        if symbol["st_shndx"] == "SHN_UNDEF":
            return None
        return symbol

    def __get_symtable(self):
        symbol_tables = [
            s
            for _, s in enumerate(self.elffile.iter_sections())
            if isinstance(s, SymbolTableSection)
            and s.name == ".symtab"
            and s["sh_entsize"]
        ]

        if not symbol_tables:
            return None

        return symbol_tables[0]

    def parse_symbol(self):
        if self.elffile is None:
            return

        symtable = self.__get_symtable()
        for nsym, symbol in enumerate(symtable.iter_symbols()):
            if self.__symbol_filter(symbol) is not None:
                symbol_name = cxxfilt.demangle(symbol.name)
                func_name = re.sub(r"\(.*$", "", symbol_name)
                if func_name[0] == "_" or func_name.find(".") != -1:
                    continue

                self.symbol_dict[symbol["st_value"] & ~0x01] = func_name
        self.addr_list = sorted(self.symbol_dict)

    def addr2symbol(self, addr: int):
        index = bisect.bisect(self.addr_list, addr)
        if index != -1:
            return self.symbol_dict[self.addr_list[index - 1]]
        return "<%#x>: unknow function" % addr


class OtherModel(BaseModel):
    payload: str

    def dump(self):
        return self.payload

    def parse(self, string):
        pattern = re.compile(
            r"[sched_switch|sched_wakeup_new|"
            r"sched_waking|irq_handler_entry|irq_handler_exit]:.*"
        )
        ret = pattern.match(string)
        if ret is not None:
            return OtherModel(string)


class AtraceModel(BaseModel):
    sign: str
    pid: int
    func: str

    def parse(self, string):
        pattern = parse.compile("tracing_mark_write: {sign:>.1}|{pid:d}|{func}")

        ret = pattern.parse(string)
        if ret is not None:
            return AtraceModel(**ret.named)

    def dump(self):
        return "tracing_mark_write: %c|%d|%s" % (self.sign, self.pid, self.func)


class TraceModel(BaseModel):
    name: str
    tid: int
    cpu: int
    time: float
    payload: Union[AtraceModel, OtherModel]

    def dump_one_trace(self):
        header = "%16s-%-5d [%03d] %12.6f: %s" % (
            self.name,
            self.tid,
            self.cpu,
            self.time,
            self.payload.dump(),
        )
        return header


class Trace(object):
    def __payloadParse(self, string):
        trace = AtraceModel.parse(AtraceModel, string)
        if trace is not None:
            return trace
        trace = OtherModel.parse(OtherModel, string)
        if trace is not None:
            return trace

    def __init__(self, file):
        with open(file, "r") as tracefile:
            self.lines = tracefile.readlines()
        self.alltrace = list()
        self.parse()

    def parse(self):
        header_pattern = parse.compile(
            "{name}-{tid:d}{:s}[{cpu:d}]{:s}{time:f}: {payload:payload}",
            dict(payload=self.__payloadParse),
        )

        for line in self.lines:
            ret = header_pattern.parse(line.strip())
            if ret and ret.named["payload"]:
                self.alltrace.append(TraceModel(**ret.named))

    def dump_trace(self):
        formatted = ["# tracer: nop", "#"]
        for trace in self.alltrace:
            formatted.append(trace.dump_one_trace())
        return formatted


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-t", "--trace", help="original trace file", required=True)
    parser.add_argument("-e", "--elf", help="elf file")
    parser.add_argument(
        "-o",
        "--out",
        help="filtered trace file, default output trace.systrace",
        default="trace.systrace",
    )
    args = parser.parse_args()

    trace = Trace(args.trace)
    if args.elf:
        symbol = SymbolTables(args.elf)
        symbol.parse_symbol()

        for onetrace in trace.alltrace:

            if isinstance(onetrace.payload, AtraceModel) and re.fullmatch(
                r"^0x[0-9a-fA-F]+$", onetrace.payload.func
            ):
                onetrace.payload.func = symbol.addr2symbol(
                    int(onetrace.payload.func, 16)
                )

    lines = trace.dump_trace()
    with open(args.out, "w") as out:
        out.writelines("\n".join(lines))
        print(os.path.abspath(args.out))
