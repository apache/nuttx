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
import subprocess
from typing import Union

from pycstruct import pycstruct

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
            r"sched_waking|irq_handler_entry|irq_handler_exit]"
        )
        ret = pattern.match(string)
        if ret is not None:
            return OtherModel(payload=string)


class ATraceModel(BaseModel):
    sign: str
    pid: int
    func: str

    def parse(self, string):
        pattern = parse.compile("tracing_mark_write: {sign:>.1}|{pid:d}|{func}")

        ret = pattern.parse(string)
        if ret is not None:
            return ATraceModel(**ret.named)

    def dump(self):
        return "tracing_mark_write: %c|%d|%s" % (
            self.sign,
            self.pid,
            self.func,
        )


class TraceModel(BaseModel):
    name: str
    tid: int
    cpu: int
    time: float
    payload: Union[ATraceModel, OtherModel]

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
        trace = ATraceModel.parse(ATraceModel, string)
        if trace is not None:
            return trace
        trace = OtherModel.parse(OtherModel, string)
        if trace is not None:
            return trace

    def __init__(self, file):
        with open(file, "r") as tracefile:
            self.lines = tracefile.readlines()
        self.all_trace = list()
        self.parse()

    def parse(self):
        header_pattern = parse.compile(
            "{name}-{tid:d}{:s}[{cpu:d}]{:s}{time:f}: {payload:payload}",
            dict(payload=self.__payloadParse),
        )

        for line in self.lines:
            ret = header_pattern.parse(line.strip())
            if ret and ret.named["payload"]:
                self.all_trace.append(TraceModel(**ret.named))

    def dump_trace(self):
        formatted = ["# tracer: nop", "#"]
        for trace in self.all_trace:
            formatted.append(trace.dump_one_trace())
        return formatted


class ParseBinaryLogTool:
    def __init__(
        self,
        binary_log_path,
        elf_nuttx_path,
        out_path=None,
        size_long=4,
        config_endian_big=False,
        config_smp=0,
    ):
        self.binary_log_path = binary_log_path
        self.elf_nuttx_path = elf_nuttx_path
        self.out_path = out_path
        self.symbol_tables = SymbolTables(self.elf_nuttx_path)
        self.symbol_tables.parse_symbol()
        with open(self.binary_log_path, "rb") as f:
            self.in_bytes = f.read()
        self.parsed = list()
        self.task_name_dict = dict()
        self.size_long = size_long
        self.size_note_common = 3 + size_long * 3
        self.config_endian_big = config_endian_big
        self.config_smp = config_smp

    def parse_by_endian(self, lis):
        res = [hex(e)[2:] for e in lis]  # strip prefix "0x"
        res = [e if len(e) == 2 else "0" + e if len(e) == 1 else "00" for e in res]
        if not self.config_endian_big:
            res.reverse()
        res = "0x" + "".join(res)
        return int(res, 16), res

    def parse_one(self, st: int):
        if st >= len(self.in_bytes):
            print("error, index break bound")
        one = pycstruct.StructDef()
        one.add("uint8", "nc_length")
        one.add("uint8", "nc_type")
        one.add("uint8", "nc_priority")
        if self.config_smp > 0:
            one.add("uint8", "nc_cpu")
        one.add("uint8", "nc_pid", self.size_long)
        one.add("uint8", "nc_systime_sec", self.size_long)
        one.add("uint8", "nc_systime_nsec", self.size_long)
        res = one.deserialize(self.in_bytes, st)

        # case type
        if res["nc_type"] == 0:
            one.add("uint8", "nsa_name", res["nc_length"] - self.size_note_common)
        elif res["nc_type"] == 22:
            one.add("uint8", "nst_ip", self.size_long)  # pointer of func
            one.add("uint8", "nst_data")  # B|E
        elif res["nc_type"] == 20:  # case: NOTE_IRQ_ENTER
            one.add("uint8", "nih_irq")
        elif res["nc_type"] == 21:  # case: NOTE_IRQ_LEAVE
            one.add("uint8", "nih_irq")
        else:
            print(f'skipped note, nc_type={res["nc_type"]}')

        res = one.deserialize(self.in_bytes, st)
        # parse pid, systime ...
        res["nc_pid"] = self.parse_by_endian(res["nc_pid"])[0]
        res["nc_systime_sec"] = self.parse_by_endian(res["nc_systime_sec"])[0]
        res["nc_systime_nsec"] = self.parse_by_endian(res["nc_systime_nsec"])[0]
        if "nst_ip" in res:
            res["nst_ip"] = self.parse_by_endian(res["nst_ip"])[1]

        # parse cpu, name ...
        if "nc_cpu" not in res:
            res["nc_cpu"] = 0
        if "nsa_name" in res:
            nsa_name = "".join(chr(i) for i in res["nsa_name"][:-1])
            self.task_name_dict[res["nc_pid"]] = nsa_name
        if "nst_data" in res:
            res["nst_data"] = chr(res["nst_data"])
        return res

    def track_one(self, one):  # print by case
        nc_type = one["nc_type"]
        nc_pid = one["nc_pid"]
        nc_cpu = one["nc_cpu"]
        nsa_name = self.task_name_dict.get(nc_pid, "noname")
        float_time = float(
            str(one["nc_systime_sec"]) + "." + str(one["nc_systime_nsec"])
        )

        # case nc_type
        a_model, other_model = None, None
        if nc_type == 0:  # case: NOTE_START
            payload = (
                f"sched_wakeup_new: comm={nsa_name} pid={nc_pid} target_cpu={nc_cpu}"
            )
            other_model = OtherModel(payload="").parse(payload)
        if nc_type == 3:  # case: NOTE_RESUME
            payload = f"sched_waking: comm={nsa_name} pid={nc_pid} target_cpu={nc_cpu}"
            other_model = OtherModel(payload="").parse(payload)
        if nc_type == 22:  # case: NOTE_DUMP_STRING
            func_name = self.symbol_tables.symbol_dict.get(
                int(one["nst_ip"], 16), "no_func_name"
            )
            payload = f'tracing_mark_write: {one["nst_data"]}|{nc_pid}|{func_name}'
            a_model = ATraceModel(sign="", pid=-1, func="").parse(payload)
        if nc_type == 20:  # case: NOTE_IRQ_ENTER
            payload = f'irq_handler_entry: irq={one["nih_irq"]} name={one["nih_irq"]}'
            other_model = OtherModel(payload="").parse(payload)
        if nc_type == 21:  # case: NOTE_IRQ_LEAVE
            payload = f'irq_handler_exit: irq={one["nih_irq"]} name={one["nih_irq"]}'
            other_model = OtherModel(payload="").parse(payload)

        for mod in [a_model, other_model]:
            if mod is not None:
                self.parsed.append(
                    TraceModel(
                        name=nsa_name,
                        tid=nc_pid,
                        cpu=nc_cpu,
                        time=float_time,
                        payload=mod,
                    )
                )

    def parse_binary_log(self):
        st = 0
        while st < len(self.in_bytes):
            one = self.parse_one(st)
            self.track_one(one)
            st += one["nc_length"]
        if self.out_path is not None:
            with open(self.out_path, "wt") as f:
                for mod in self.parsed:
                    f.write(mod.dump_one_trace() + "\n")
        else:
            for mod in self.parsed:
                print(f"debug, dump one={mod.dump_one_trace()}")


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

    file_type = subprocess.check_output(f"file -b {args.trace}", shell=True)
    file_type = str(file_type, "utf-8").lower()
    if "ascii" in file_type:
        print("trace log type is text")
        trace = Trace(args.trace)
        symbol = SymbolTables(args.elf)
        symbol.parse_symbol()
        for onetrace in trace.all_trace:
            if isinstance(onetrace.payload, ATraceModel) and re.fullmatch(
                r"^0x[0-9a-fA-F]+$", onetrace.payload.func
            ):
                onetrace.payload.func = symbol.addr2symbol(
                    int(onetrace.payload.func, 16)
                )
        lines = trace.dump_trace()
        with open(args.out, "w") as out:
            out.writelines("\n".join(lines))
            print(os.path.abspath(args.out))
    else:
        print("trace log type is binary")
        if args.elf:
            print(
                "parse_binary_log, default config, size_long=4, config_endian_big=False, config_smp=0"
            )
            parse_binary_log_tool = ParseBinaryLogTool(args.trace, args.elf, args.out)
            parse_binary_log_tool.symbol_tables.parse_symbol()
            parse_binary_log_tool.parse_binary_log()
        else:
            print("error, please add elf file path")
