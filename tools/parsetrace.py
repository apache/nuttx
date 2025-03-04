#!/usr/bin/env python3
############################################################################
# tools/parsetrace.py
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
import bisect
import logging
import os
import re
import struct
import subprocess
from typing import Union

try:
    import colorlog
    import cxxfilt
    import parse
    import serial
    from elftools.elf.elffile import ELFFile
    from elftools.elf.sections import SymbolTableSection
    from pycstruct import pycstruct
    from pydantic import BaseModel

except ModuleNotFoundError:
    print("Please execute the following command to install dependencies:")
    print("pip install pyelftools cxxfilt pydantic parse pycstruct colorlog serial")
    exit(1)

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

stream_handler = logging.StreamHandler()
formatter = colorlog.ColoredFormatter(
    "%(log_color)s %(message)s",
    log_colors={
        "DEBUG": "light_black",
        "INFO": "white",
        "WARNING": "yellow",
        "ERROR": "red",
    },
)

stream_handler.setFormatter(formatter)
logger.addHandler(stream_handler)


class SymbolTables(object):
    def __init__(self, file):
        elffile = open(file, "rb")
        self.elffile = ELFFile(elffile)
        self.elfinfo = dict()
        self.typeinfo = dict()
        self.symbol_dict = dict()
        self.addr_list = list()
        self.__parse_header()
        self.parse_symbol()

    def __parse_header(self):
        elf_header = self.elffile.header
        bitness = elf_header["e_ident"]["EI_CLASS"]
        self.elfinfo["bitwides"] = 32 if bitness == "ELFCLASS32" else 64
        endianness = elf_header["e_ident"]["EI_DATA"]
        self.elfinfo["byteorder"] = "little" if endianness == "ELFDATA2LSB" else "big"
        self.typeinfo["size_t"] = "uint%d" % self.elfinfo["bitwides"]
        self.typeinfo["long"] = "int%d" % self.elfinfo["bitwides"]
        self.typeinfo["pid_t"] = "int32"

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
                try:
                    symbol_name = cxxfilt.demangle(symbol.name)
                except Exception:
                    symbol_name = symbol.name
                self.symbol_dict[symbol["st_value"] & ~0x01] = symbol_name
        self.addr_list = sorted(self.symbol_dict)

    def get_typesize(self, type_name):
        if not self.elffile.has_dwarf_info():
            raise ValueError("not found dwarf info!")

        dwarfinfo = self.elffile.get_dwarf_info()
        for CU in dwarfinfo.iter_CUs():
            for DIE in CU.iter_DIEs():
                if DIE.tag == "DW_TAG_typedef":
                    name = DIE.attributes["DW_AT_name"].value.decode("utf-8")
                    if name == type_name:
                        if "DW_AT_type" in DIE.attributes:
                            type_attr = DIE.attributes["DW_AT_type"]
                            base_type_die = dwarfinfo.get_DIE_from_refaddr(
                                type_attr.value + CU.cu_offset
                            )
                            if base_type_die.tag == "DW_TAG_base_type":
                                size = base_type_die.attributes["DW_AT_byte_size"].value
                            elif base_type_die.tag == "DW_TAG_typedef":
                                type_name = base_type_die.attributes[
                                    "DW_AT_name"
                                ].value.decode("utf-8")
                                continue
                        else:
                            size = DIE.attributes["DW_AT_byte_size"].value
                        return size
        raise ValueError("not found type")

    def readstring(self, addr):
        data = b""
        while True:
            data += self.read(addr, 256)
            data = data.split(b"\x00")[0]
            if len(data) < 256:
                break
        return data.decode("utf-8")

    def read(self, addr, size):
        for segment in self.elffile.iter_segments():
            seg_addr = segment["p_paddr"]
            seg_size = min(segment["p_memsz"], segment["p_filesz"])
            if addr >= seg_addr and addr + size <= seg_addr + seg_size:
                data = segment.data()
                start = addr - seg_addr
                return data[start : start + size]

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
        with open(file, "rb+") as tracefile:
            self.lines = tracefile.read()
        self.lines = self.lines.split(b"\n")
        self.all_trace = list()
        self.parse()

    def parse(self):
        header_pattern = parse.compile(
            "{name}-{tid:d}{:s}[{cpu:d}]{:s}{time:f}: {payload:payload}",
            dict(payload=self.__payloadParse),
        )

        for line in self.lines:
            try:
                line = line.decode("utf-8")
                ret = header_pattern.parse(line.strip())
                if ret and ret.named["payload"]:
                    self.all_trace.append(TraceModel(**ret.named))
            except Exception:
                continue

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


class TraceDecoder(SymbolTables):
    def __init__(self, elffile):
        super().__init__(elffile)
        self.data = b""
        self.typeinfo["time_t"] = "int%d" % (self.get_typesize("time_t") * 8)

    def note_common_define(self):
        note_common = pycstruct.StructDef(alignment=4)
        note_common.add("uint8", "nc_length")
        note_common.add("uint8", "nc_type")
        note_common.add("uint8", "nc_priority")
        note_common.add("uint8", "nc_cpu")
        note_common.add(self.typeinfo["pid_t"], "nc_pid")
        note_common.add(self.typeinfo["time_t"], "nc_systime_sec")
        note_common.add(self.typeinfo["long"], "nc_systime_nsec")
        return note_common

    def note_printf_define(self, length):
        struct_def = pycstruct.StructDef(alignment=4)
        struct_def.add(self.note_common_define(), "npt_cmn")
        struct_def.add(self.typeinfo["size_t"], "npt_ip")
        struct_def.add(self.typeinfo["size_t"], "npt_fmt")
        struct_def.add("uint32", "npt_type")
        if length > 0:
            struct_def.add("uint8", "npt_data", length=length)
        return struct_def

    def extract_int(self, fmt, data):
        pattern = re.match(
            r"%([-+ #0]*)?(\d+|\*)?(\.)?(\d+|\*)?([hljzt]|ll|hh)?([diuxXop])", fmt
        ).groups()
        format = "%" if pattern[0] is None else "%" + pattern[0]

        if pattern[4] == "l" or pattern[4] == "z" or pattern[4] == "t":
            length = 4 if self.typeinfo["size_t"] == "int32" else 8
        elif pattern[4] == "ll":
            length = 8
        elif pattern[4] == "h":
            length = 2
        elif pattern[4] == "hh":
            length = 1
        else:
            length = 4

        width = 0
        if pattern[1] == "*":
            width = int.from_bytes(
                data[:4], byteorder=self.elfinfo["byteorder"], signed=True
            )
            data = data[4:]
            format += str(width)
        elif pattern[1] is not None:
            width = int(pattern[1])
            format += pattern[1]

        format += pattern[5]
        if pattern[5] == "d" or pattern[5] == "i":
            value = int.from_bytes(
                data[:length], byteorder=self.elfinfo["byteorder"], signed=True
            )
        elif (
            pattern[5] == "u"
            or pattern[5] == "x"
            or pattern[5] == "X"
            or pattern[5] == "o"
            or pattern[5] == "O"
        ):
            value = int.from_bytes(
                data[:length], byteorder=self.elfinfo["byteorder"], signed=False
            )

        value = format % value
        return "%s", length, value

    def extract_float(self, fmt, data):
        pattern = re.match(
            r"%([-+ #0]*)?(\d+|\*)?(\.)?(\d+|\*)?(L)?([fFeEgGaA])", fmt
        ).groups()
        if pattern[4] == "L":
            length = 16
        else:
            length = 8
        value = struct.unpack("<d", data[:length])[0]
        return "%s", length, fmt % value

    def extract_string(self, fmt, data):
        length = 0
        if fmt == "%.*s":
            length = int.from_bytes(
                data[:4], byteorder=self.elfinfo["byteorder"], signed=True
            )
            data = data[4:]
        else:
            length = 0

        try:
            string = data.split(b"\x00")[0].decode("utf-8")
            if len(data.split(b"\x00")[0]) != len(string) and len(string):
                size = 4 if self.typeinfo["size_t"] == "uint32" else 8
                address = int.from_bytes(
                    data[:size], byteorder=self.elfinfo["byteorder"], signed=False
                )
                string = f"<<0x{address}>>"
                length += size
            else:
                length += len(string) + 1
        except Exception:
            size = 4 if self.typeinfo["size_t"] == "uint32" else 8
            address = int.from_bytes(
                data[:size], byteorder=self.elfinfo["byteorder"], signed=False
            )
            string = f"<<0x{address}>>"
            length += size
        return "%s", length, string

    def extract_point(self, fmt, data):
        length = 4 if self.typeinfo["size_t"] == "int32" else 8
        value = int.from_bytes(
            data[:length], byteorder=self.elfinfo["byteorder"], signed=False
        )
        return "%s", length, f"{value:x}"

    conversions = {
        r"%p": extract_point,
        r"%c": lambda _0, _1, data: ("%c", 1, chr(data[0])),
        r"%([-+ #0]*)?(\d+|\*)?(\.)?(\d+|\*)?(L)?([fFeEgGaA])": extract_float,
        r"%([-+ #0]*)?(\d+|\*)?(\.)?(\d+|\*)?([hljzt]|ll|hh)?([diuxXop])": extract_int,
        r"%([ ]*)?([\d+|\*])?(\.)?([\d+|\*])?s": extract_string,
    }

    patterns = {re.compile(pattern): func for pattern, func in conversions.items()}

    def printf(self, format, data):
        try:
            fmt = []
            values = []

            parts = [
                part
                for part in re.split(
                    r"(%[-+#0\s]*[\d|\*]*(?:\.[\d|\*])?[lhjztL]?[diufFeEgGxXoscpn%])",
                    format,
                )
            ]
            for part in parts:
                if "%" not in part:
                    fmt.append(part)
                    continue

                for pattern, handler in self.patterns.items():
                    if pattern.match(part):
                        part, length, value = handler(self, part, data)
                        values.append(value)
                        fmt.append(part)
                        data = data[length:]
                        break
                else:
                    fmt.append(part)

            return "".join(fmt) % tuple(values)
        except Exception as e:
            logger.error(f"format failed: {e}")

    def print_format(self, note):
        payload = dict()
        payload["time"] = (
            note["npt_cmn"]["nc_systime_sec"]
            + note["npt_cmn"]["nc_systime_nsec"] / 1000000000
        )
        payload["pid"] = note["npt_cmn"]["nc_pid"]
        payload["cpu"] = (
            0 if "nc_cpu" not in note["npt_cmn"] else note["npt_cmn"]["nc_cpu"]
        )
        payload["format"] = self.readstring(note["npt_fmt"])
        prefix = "[{time:.9f}] [{pid}] [CPU{cpu}]: ".format(**payload)
        string = self.printf(payload["format"], note["npt_data"]).rstrip("\n")
        logger.info(prefix + string)

    def parse_note(self, rawdata=None):
        while len(self.data) > 0:
            data = self.data if rawdata is None else rawdata
            try:
                common_struct = self.note_common_define()
                if len(data) < common_struct.size():
                    return
                common_note = common_struct.deserialize(data)
                nc_length = common_note["nc_length"]
                if nc_length < common_struct.size():
                    raise ValueError("Invalid note length")

                if common_note["nc_type"] == 24:
                    note_struct = self.note_printf_define(0)
                    length = nc_length - note_struct.size()
                    note = note_struct.deserialize(data)
                    note["npt_data"] = data[
                        note_struct.size() : note_struct.size() + length
                    ]
                    self.print_format(note)
                else:
                    raise ValueError("Invalid note type")
            except Exception as e:
                logger.debug(f"skip one byte, data: {hex(self.data[0])} {e}")
                self.data = data[1:]
                continue

            self.data = data[nc_length:]

    def tty_received(self):
        while True:
            data = ser.read(16384)
            if len(data) == 0:
                continue
            self.data += data
            logger.debug(f"serial buffer data: {self.data.hex()}")
            decode.parse_note()


def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("-t", "--trace", help="original trace file")
    parser.add_argument("-e", "--elf", help="elf file")
    parser.add_argument("-d", "--device", help="Physical serial device name")
    parser.add_argument(
        "-b", "--baudrate", help="Physical serial device baud rate", default=115200
    )
    parser.add_argument("-v", "--verbose", help="verbose output", action="store_true")
    parser.add_argument(
        "-o",
        "--output",
        help="filtered trace file, default output trace.systrace",
        default="trace.systrace",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_arguments()
    out_path = args.output if args.output else "trace.systrace"
    logger.setLevel(logging.DEBUG if args.verbose else logging.INFO)

    if args.trace is None and args.device is None:
        print("error, please add trace file path or device name")
        print(
            "usage: parsetrace.py [-h] [-t TRACE] [-e ELF] [-d DEVICE] [-b BAUDRATE] [-v] [-o OUTPUT]"
        )
        exit(1)

    if args.trace:
        file_type = subprocess.check_output(f"file -b {args.trace}", shell=True)
        file_type = str(file_type, "utf-8").lower()
        if "ascii" in file_type:
            print("trace log type is text")
            trace = Trace(args.trace)
            if args.elf:
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
                with open(out_path, "w") as out:
                    out.write("\n".join(lines))
                    out.write("\n")
                    print(os.path.abspath(out_path))
        else:
            print("trace log type is binary")
            if args.elf:
                print(
                    "parse_binary_log, default config, size_long=4, config_endian_big=False, config_smp=0"
                )
                parse_binary_log_tool = ParseBinaryLogTool(
                    args.trace, args.elf, out_path
                )
                parse_binary_log_tool.symbol_tables.parse_symbol()
                parse_binary_log_tool.parse_binary_log()
            else:
                print("error, please add elf file path")

    if args.device:
        if args.elf is None:
            print("error, please add elf file path")
            exit(1)

        decode = TraceDecoder(args.elf)
        with serial.Serial(args.device, baudrate=args.baudrate) as ser:
            ser.timeout = 0
            decode.tty_received()
