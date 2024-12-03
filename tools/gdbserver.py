#!/usr/bin/env python3
# tools/gdbserver.py
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
import binascii
import logging
import multiprocessing
import os
import re
import shutil
import socket
import struct
import subprocess
import sys
import traceback

import elftools
from elftools.elf.elffile import ELFFile

# ELF section flags
SHF_WRITE = 0x1
SHF_ALLOC = 0x2
SHF_EXEC = 0x4
SHF_WRITE_ALLOC = SHF_WRITE | SHF_ALLOC
SHF_ALLOC_EXEC = SHF_ALLOC | SHF_EXEC

GDB_SIGNAL_DEFAULT = 7

UINT16_MAX = 65535


DEFAULT_GDB_INIT_CMD = "-ex 'bt full' -ex 'info reg' -ex 'display /40i $pc-40'"

logger = logging.getLogger()

# The global register table is dictionary like {arch:{reg:ndx}}
#
# where arch is the CPU architecture name;
#       reg  is the name of the register as used in log file
#       ndx  is the index of the register in GDB group registers list
#
# Registers with multiple convenient names can have multiple entries here, one
# for each name and with the same index.
reg_table = {
    "arm": {
        "R0": 0,
        "R1": 1,
        "R2": 2,
        "R3": 3,
        "R4": 4,
        "R5": 5,
        "R6": 6,
        "FP": 7,
        "R8": 8,
        "SB": 9,
        "SL": 10,
        "R11": 11,
        "IP": 12,
        "SP": 13,
        "LR": 14,
        "PC": 15,
        "xPSR": 16,
    },
    "arm-a": {
        "R0": 0,
        "R1": 1,
        "R2": 2,
        "R3": 3,
        "R4": 4,
        "R5": 5,
        "R6": 6,
        "R7": 7,
        "R8": 8,
        "SB": 9,
        "SL": 10,
        "FP": 11,
        "IP": 12,
        "SP": 13,
        "LR": 14,
        "PC": 15,
        "CPSR": 41,
    },
    "arm-t": {
        "R0": 0,
        "R1": 1,
        "R2": 2,
        "R3": 3,
        "R4": 4,
        "R5": 5,
        "R6": 6,
        "FP": 7,
        "R8": 8,
        "SB": 9,
        "SL": 10,
        "R11": 11,
        "IP": 12,
        "SP": 13,
        "LR": 14,
        "PC": 15,
        "CPSR": 41,
    },
    "arm64": {
        "X0": 0,
        "X1": 1,
        "X2": 2,
        "X3": 3,
        "X4": 4,
        "X5": 5,
        "X6": 6,
        "X7": 7,
        "X8": 8,
        "X9": 9,
        "X10": 10,
        "X11": 11,
        "X12": 12,
        "X13": 13,
        "X14": 14,
        "X15": 15,
        "X16": 16,
        "X17": 17,
        "X18": 18,
        "X19": 19,
        "X20": 20,
        "X21": 21,
        "X22": 22,
        "X23": 23,
        "X24": 24,
        "X25": 25,
        "X26": 26,
        "X27": 27,
        "X28": 28,
        "X29": 29,
        "X30": 30,
        "SP_ELX": 31,
        "ELR": 32,
    },
    # rv64 works with gdb-multiarch on Ubuntu
    "riscv": {
        "ZERO": 0,
        "RA": 1,
        "SP": 2,
        "GP": 3,
        "TP": 4,
        "T0": 5,
        "T1": 6,
        "T2": 7,
        "FP": 8,
        "S1": 9,
        "A0": 10,
        "A1": 11,
        "A2": 12,
        "A3": 13,
        "A4": 14,
        "A5": 15,
        "A6": 16,
        "A7": 17,
        "S2": 18,
        "S3": 19,
        "S4": 20,
        "S5": 21,
        "S6": 22,
        "S7": 23,
        "S8": 24,
        "S9": 25,
        "S10": 26,
        "S11": 27,
        "T3": 28,
        "T4": 29,
        "T5": 30,
        "T6": 31,
        "PC": 32,
        "S0": 8,
        "EPC": 32,
    },
    # use xtensa-esp32s3-elf-gdb register table
    "esp32s3": {
        "PC": 0,
        "PS": 73,
        "A0": 1,
        "A1": 2,
        "A2": 3,
        "A3": 4,
        "A4": 5,
        "A5": 6,
        "A6": 7,
        "A7": 8,
        "A8": 9,
        "A9": 10,
        "A10": 11,
        "A11": 12,
        "A12": 13,
        "A13": 14,
        "A14": 15,
        "A15": 16,
        "WINDOWBASE": 69,
        "WINDOWSTART": 70,
        "CAUSE": 190,
        "VADDR": 196,
        "LBEG": 65,
        "LEND": 66,
        "LCNT": 67,
        "SAR": 68,
        "SCOM": 76,
    },
    # use xt-gdb register table
    "xtensa": {
        "PC": 32,
        "PS": 742,
        "A0": 256,
        "A1": 257,
        "A2": 258,
        "A3": 259,
        "A4": 260,
        "A5": 261,
        "A6": 262,
        "A7": 263,
        "A8": 264,
        "A9": 265,
        "A10": 266,
        "A11": 267,
        "A12": 268,
        "A13": 269,
        "A14": 270,
        "A15": 271,
        "WINDOWBASE": 584,
        "WINDOWSTART": 585,
        "CAUSE": 744,
        "VADDR": 750,
        "LBEG": 512,
        "LEND": 513,
        "LCNT": 514,
        "SAR": 515,
        "SCOM": 524,
    },
}

# make sure the a0-a15 can be remapped to the correct register
reg_fix_value = {
    "esp32s3": {
        "WINDOWBASE": (0, 69),
        "WINDOWSTART": (1, 70),
        "PS": (0x40000, 73),
    },
    "xtensa": {
        "WINDOWBASE": (0, 584),
        "WINDOWSTART": (1, 585),
        "PS": (0x40000, 742),
    },
    "riscv": {
        "ZERO": 0,
        "WINDOWBASE": (0, 584),
        "WINDOWSTART": (1, 585),
        "PS": (0x40000, 742),
    },
}


def str_get_after(s, sub):
    index = s.find(sub)
    if index == -1:
        return None
    return s[index + len(sub) :]


def pack_memory(start, end, data):
    return {"start": start, "end": end, "data": data}


class DumpELFFile:
    """
    Class to parse ELF file for memory content in various sections.
    There are read-only sections (e.g. text and rodata) where
    the memory content does not need to be dumped via coredump
    and can be retrieved from the ELF file.
    """

    def __init__(self, elffile: str):
        self.elffile = elffile
        self.__memories = []
        self.__arch = None
        self.__xlen = None
        self.__text = 0

    def parse(self, load_symbol: bool):
        self.__memories = []
        elf = ELFFile.load_from_path(self.elffile)
        self.__arch = elf.get_machine_arch().lower().replace("-", "")
        self.__xlen = elf.elfclass

        for section in elf.iter_sections():
            # REALLY NEED to match exact type as all other sections
            # (debug, text, etc.) are descendants where
            # isinstance() would match.
            if (
                type(section) is not elftools.elf.sections.Section
            ):  # pylint: disable=unidiomatic-typecheck
                continue

            size = section["sh_size"]
            flags = section["sh_flags"]
            start = section["sh_addr"]
            end = start + size - 1

            store = False
            desc = "?"

            if section["sh_type"] == "SHT_PROGBITS":
                if (flags & SHF_ALLOC_EXEC) == SHF_ALLOC_EXEC:
                    # Text section
                    store = True
                    desc = "text"
                elif (flags & SHF_WRITE_ALLOC) == SHF_WRITE_ALLOC:
                    # Data or Rodata section, rodata store in ram in some case
                    store = True
                    desc = "data or rodata"
                elif (flags & SHF_ALLOC) == SHF_ALLOC:
                    # Read only data section
                    store = True
                    desc = "read-only data"

            if store:
                memory = pack_memory(start, end, section.data())
                logger.debug(
                    f"ELF Section: {hex(memory['start'])} to {hex(memory['end'])} of size {len(memory['data'])} ({desc})"
                )

                self.__memories.append(memory)

        # record first text segment address
        for segment in elf.iter_segments():
            if segment.header.p_flags & 1 and not self.__text:
                self.__text = segment.header.p_vaddr

        self.load_symbol = load_symbol
        if load_symbol:
            symtab = elf.get_section_by_name(".symtab")
            self.symbol = {}
            for symbol in symtab.iter_symbols():
                if symbol["st_info"]["type"] != "STT_OBJECT":
                    continue

                    if symbol.name in (
                        "g_tcbinfo",
                        "g_pidhash",
                        "g_npidhash",
                        "g_last_regs",
                        "g_running_tasks",
                    ):
                        self.symbol[symbol.name] = symbol
                        logger.debug(
                            f"name:{symbol.name} size:{symbol['st_size']} value:{hex(symbol['st_value'])}"
                        )

        elf.close()
        return True

    def merge(self, other):
        if other.arch() == self.arch() and other.xlen() == self.xlen():
            self.__memories += other.get_memories()
        else:
            raise TypeError("inconsistent ELF types")

    def get_memories(self):
        return self.__memories

    def arch(self):
        return self.__arch

    def xlen(self):
        return self.__xlen

    def text(self):
        return self.__text


class DumpLogFile:
    def __init__(self, logfile):
        self.logfile = logfile
        self.registers = []
        self.__memories = list()
        self.reg_table = dict()
        self.reg_len = 32

    def _init_register(self):
        # registers list should be able to hold the max index
        self.registers = [b"x"] * (max(self.reg_table.values()) + 1)

    def _parse_register(self, line):
        line = str_get_after(line, "up_dump_register:")
        if line is None:
            return False

        line = line.strip()
        # find register value
        find_res = re.findall(r"(?P<REG>\w+):\s*(?P<REGV>[0-9a-fxA-FX]+)", line)

        for reg_name, reg_val in find_res:
            reg_name = reg_name.upper()
            if reg_name in self.reg_table:
                reg_index = self.reg_table[reg_name]
                self.registers[reg_index] = int(reg_val, 16)
                self.reg_len = max(self.reg_len, len(reg_val) * 4)

        return True

    def _parse_fix_register(self, arch):
        if arch in reg_fix_value:
            for reg_name, reg_vals in reg_fix_value[arch].items():
                reg_index = self.reg_table[reg_name]
                self.registers[reg_index] = reg_vals[0]

    def _parse_stack(self, line, start, data):
        line = str_get_after(line, "stack_dump:")
        if line is None:
            return None

        line = line.strip()

        # find stack-dump
        match_res = re.match(r"(?P<ADDR_START>0x\w+): (?P<VALS>( ?\w+)+)", line)
        if match_res is None:
            return None

        addr_start = int(match_res.groupdict()["ADDR_START"], 16)
        if start + len(data) != addr_start:
            # stack is not contiguous
            if len(data) == 0:
                start = addr_start
            else:
                self.__memories.append(pack_memory(start, start + len(data), data))
                data = b""
                start = addr_start

        reg_fmt = "<I" if self.reg_len <= 32 else "<Q"
        for val in match_res.groupdict()["VALS"].split():
            data = data + struct.pack(reg_fmt, int(val, 16))

        return start, data

    def parse(self, arch):
        self.reg_table = reg_table[arch]
        self._init_register()

        data = bytes()
        start = 0

        if isinstance(self.logfile, list):
            lines = self.logfile
        else:
            with open(self.logfile, "r") as f:
                lines = f.readlines()

        for line_num, line in enumerate(lines):
            if line == "":
                break

            try:
                if self._parse_register(line):
                    continue

                res = self._parse_stack(line, start, data)
                if res:
                    start, data = res
                    continue

            except Exception as e:
                logger.error("parse log file error: %s line_number %d" % (e, line_num))
                sys.exit(1)

        self._parse_fix_register(arch)
        if data:
            self.__memories.append(pack_memory(start, start + len(data), data))

    def get_memories(self):
        return self.__memories


class RawMemoryFile:
    def __init__(self, rawfile):
        self.__memories = list()

        if rawfile is None:
            return

        for raw in rawfile:
            file, start = raw.split(":")
            start = int(start, 0)

            size = os.path.getsize(file)
            with open(file, "rb") as f:
                data = f.read(size)
                self.__memories.append(pack_memory(start, start + len(data), data))

    def get_memories(self):
        return self.__memories


class CoreDumpFile:
    def __init__(self, coredump):
        self.__memories = list()

        if coredump is None:
            return

        with open(coredump, "rb") as f:
            elffile = ELFFile(f)
            for segment in elffile.iter_segments():
                if segment["p_type"] != "PT_LOAD":
                    continue
                logger.debug(f"Segment Flags: {segment['p_flags']}")
                logger.debug(
                    f"Segment Offset: {segment['p_offset']}",
                )
                logger.debug(f"Segment Virtual Address: {hex(segment['p_vaddr'])}")
                logger.debug(f"Segment Physical Address: {hex(segment['p_paddr'])}")
                logger.debug(f"Segment File Size:{segment['p_filesz']}")
                logger.debug(f"Segment Memory Size:{segment['p_memsz']}")
                logger.debug(f"Segment Alignment:{segment['p_align']}")
                logger.debug("=" * 40)
                f.seek(segment["p_offset"], 0)
                data = f.read(segment["p_filesz"])
                self.__memories.append(
                    pack_memory(
                        segment["p_vaddr"], segment["p_vaddr"] + len(data), data
                    )
                )

    def get_memories(self):
        return self.__memories


class GDBStub:
    def __init__(
        self,
        logfile: DumpLogFile,
        elffile: DumpELFFile,
        rawfile: RawMemoryFile,
        coredump: CoreDumpFile,
        arch: str,
    ):
        self.registers = logfile.registers
        self.elffile = elffile
        self.socket = None
        self.gdb_signal = GDB_SIGNAL_DEFAULT
        self.arch = arch
        self.reg_fmt = "<I" if elffile.xlen() <= 32 else "<Q"
        self.int_size = elffile.xlen() // 8
        self.reg_digits = elffile.xlen() // 4

        # new list oreder is coredump, rawfile, logfile, elffile

        self.mem_regions = (
            coredump.get_memories()
            + rawfile.get_memories()
            + logfile.get_memories()
            + self.elffile.get_memories()
        )
        self.reg_digits = elffile.xlen() // 4
        self.reg_fmt = "<I" if elffile.xlen() <= 32 else "<Q"

        self.threadinfo = []
        self.current_thread = 0
        self.regfix = False
        if elffile.load_symbol:
            try:
                self.parse_thread()
                logger.debug(f"Have {len(self.threadinfo)} threads to debug.")
                if len(self.threadinfo) == 0:
                    logger.critical(
                        "Check if your coredump or raw file matches the ELF file"
                    )
                    sys.exit(1)

                if arch in reg_fix_value.keys():
                    self.regfix = True
                    logger.info(f"Current arch is {arch}, need reg index fix.")

            except TypeError as e:
                if not self.registers:
                    logger.critical(
                        "Logfile, coredump, or rawfile do not contain register,"
                        f"Please check if the files are correct. {e}"
                    )

                    stack_trace = traceback.format_exc()
                    logger.debug(stack_trace)
                    sys.exit(1)

    def get_gdb_packet(self):
        socket = self.socket
        if socket is None:
            return None

        data = b""
        checksum = 0
        # Wait for '$'
        while True:
            ch = socket.recv(1)
            if ch == b"$":
                break

        # Get a full packet
        while True:
            ch = socket.recv(1)
            if ch == b"#":
                # End of packet
                break

            checksum += ord(ch)
            data += ch

        # Get checksum (2-bytes)
        ch = socket.recv(2)
        in_chksum = ord(binascii.unhexlify(ch))

        logger.debug(f"Received GDB packet: {data}")

        if (checksum % 256) == in_chksum:
            # ACK
            logger.debug("ACK")
            socket.send(b"+")

            return data
        else:
            # NACK
            logger.debug(f"NACK (checksum {in_chksum} != {checksum}")
            socket.send(b"-")

            return None

    def put_gdb_packet(self, data):
        socket = self.socket
        if socket is None:
            return

        checksum = 0
        for d in data:
            checksum += d

        pkt = b"$" + data + b"#"

        checksum = checksum % 256
        pkt += format(checksum, "02X").encode()

        logger.debug(f"Sending GDB packet: {pkt}")

        socket.send(pkt)

    def handle_signal_query_packet(self):
        # the '?' packet
        pkt = b"S"
        pkt += format(self.gdb_signal, "02X").encode()

        self.put_gdb_packet(pkt)

    def handle_register_group_read_packet(self):

        def put_register_packet(regs):
            reg_fmt = self.reg_fmt
            pkt = b""

            for reg in regs:
                if reg != b"x":
                    bval = struct.pack(reg_fmt, reg)
                    pkt += binascii.hexlify(bval)
                else:
                    # Register not in coredump -> unknown value
                    # Send in "xxxxxxxx"
                    pkt += b"x" * self.reg_digits

            self.put_gdb_packet(pkt)

        if not self.threadinfo:
            put_register_packet(self.registers)
        else:
            for thread in self.threadinfo:
                if thread["tcb"]["pid"] == self.current_thread:
                    if thread["tcb"]["tcbptr"] in self.running_tasks.keys():
                        put_register_packet(self.running_tasks[thread["tcb"]["tcbptr"]])
                    else:
                        put_register_packet(thread["gdb_regs"])
                    break

    def handle_register_single_read_packet(self, pkt):
        logger.debug(f"pkt: {pkt}")

        def put_one_register_packet(regs):

            reg = int(pkt[1:].decode("utf8"), 16)
            regval = None

            if self.regfix:
                for reg_name, reg_vals in reg_fix_value[self.arch].items():
                    if reg == reg_vals[1]:
                        logger.debug(f"{reg_name} fix to {reg_vals[0]}")
                        regval = reg_vals[0]

                if regval is None:
                    # tcbinfo index to gdb index
                    reg_gdb_index = list(reg_table[self.arch].values())
                    if reg in reg_gdb_index:
                        reg = reg_gdb_index.index(reg)
                        regval = regs[reg]

            elif reg < len(regs) and regs[reg] != b"x":
                regval = regs[reg]

            if regval is not None:
                bval = struct.pack(self.reg_fmt, regval)
                self.put_gdb_packet(binascii.hexlify(bval))
            else:
                self.put_gdb_packet(b"x" * self.reg_digits)

        if not self.threadinfo:
            put_one_register_packet(self.registers)
        else:
            for thread in self.threadinfo:
                if thread["tcb"]["pid"] == self.current_thread:
                    if thread["tcb"]["tcbptr"] in self.running_tasks.keys():
                        put_one_register_packet(
                            self.running_tasks[thread["tcb"]["tcbptr"]]
                        )
                    else:
                        put_one_register_packet(thread["gdb_regs"])
                    break

    def handle_register_group_write_packet(self):
        # the 'G' packet for writing to a group of registers
        #
        # We don't support writing so return error
        self.put_gdb_packet(b"E01")

    def handle_register_single_write_packet(self, pkt):
        # the 'P' packet for writing to registers

        index, value = pkt[1:].split(b"=")
        reg_val = 0
        for i in range(0, len(value), 2):
            data = value[i : i + 2]
            reg_val = reg_val + (int(data.decode("utf8"), 16) << (i * 4))

        reg = int(index.decode("utf8"), 16)
        if reg < len(self.registers):
            self.registers[reg] = reg_val

        self.put_gdb_packet(b"OK")

    def get_mem_region(self, addr, mem_regions=None):
        mem_regions = mem_regions or self.mem_regions
        for mem in mem_regions:
            if mem["start"] <= addr < mem["end"]:
                return mem

        return None

    def handle_memory_read_packet(self, pkt):
        # the 'm' packet for reading memory: m<addr>,<len>

        # extract address and length from packet
        # and convert them into usable integer values
        addr, length = pkt[1:].split(b",")
        s_addr = int(addr, 16)
        length = int(length, 16)

        remaining = length
        addr = s_addr
        barray = b""
        r = self.get_mem_region(addr)
        while remaining > 0:
            if r is None:
                barray = None
                break

            offset = addr - r["start"]
            barray += r["data"][offset : offset + 1]

            addr += 1
            remaining -= 1

        if barray is not None:
            pkt = binascii.hexlify(barray)
            self.put_gdb_packet(pkt)
        else:
            self.put_gdb_packet(b"E01")

    def handle_memory_write_packet(self, pkt):
        # the 'M' packet for writing to memory
        #
        # We don't support writing so return error
        self.put_gdb_packet(b"E02")

    def handle_is_thread_active(self, pkt):
        self.current_thread = int(pkt[1:]) - 1
        self.put_gdb_packet(b"OK")

    def handle_thread_context(self, pkt):
        if b"g" == pkt[1:2]:
            self.current_thread = int(pkt[2:]) - 1
        elif b"c" == pkt[1:2]:
            self.current_thread = int(pkt[3:]) - 1

        if self.current_thread == -1:
            self.current_thread = 0
        self.put_gdb_packet(b"OK")

    def parse_thread(self):
        def unpack_data(addr, fmt, from_elf=False):
            if from_elf:
                r = self.get_mem_region(addr, self.elffile.get_memories())
            else:
                r = self.get_mem_region(addr)
            offset = addr - r["start"]
            data = r["data"]
            return struct.unpack_from(fmt, data, offset)

        TCBINFO_FMT = "<8HQ"

        # uint16_t pid_off;                      /* Offset of tcb.pid               */
        # uint16_t state_off;                    /* Offset of tcb.task_state        */
        # uint16_t pri_off;                      /* Offset of tcb.sched_priority    */
        # uint16_t name_off;                     /* Offset of tcb.name              */
        # uint16_t stack_off;                    /* Offset of tcb.stack_alloc_ptr   */
        # uint16_t stack_size_off;               /* Offset of tcb.adj_stack_size    */
        # uint16_t regs_off;                     /* Offset of tcb.regs              */
        # uint16_t regs_num;                     /* Num of general regs             */
        # union
        #   {
        #     uint8_t             u[8];
        #     FAR const uint16_t *p;
        #   }

        unpacked_data = unpack_data(
            self.elffile.symbol["g_tcbinfo"]["st_value"],
            TCBINFO_FMT,
            True,
        )
        tcbinfo = {
            "pid_off": int(unpacked_data[0]),
            "state_off": int(unpacked_data[1]),
            "pri_off": int(unpacked_data[2]),
            "name_off": int(unpacked_data[3]),
            "stack_off": int(unpacked_data[4]),
            "stack_size_off": int(unpacked_data[5]),
            "regs_off": int(unpacked_data[6]),
            "regs_num": int(unpacked_data[7]),
            "reg_off": int(unpacked_data[8]),
        }

        unpacked_data = unpack_data(
            self.elffile.symbol["g_npidhash"]["st_value"],
            "<I",
        )
        npidhash = int(unpacked_data[0])
        logger.debug(f"g_npidhash is {hex(npidhash)}")

        unpacked_data = unpack_data(
            self.elffile.symbol["g_pidhash"]["st_value"],
            self.reg_fmt,
        )
        pidhash = int(unpacked_data[0])
        logger.debug(f"g_pidhash is {hex(pidhash)}")

        tcbptr_list = []
        for i in range(0, npidhash):
            unpacked_data = unpack_data(pidhash + i * self.int_size, self.reg_fmt)
            tcbptr_list.append(int(unpacked_data[0]))

        def parse_tcb(tcbptr):
            tcb = {}
            tcb["pid"] = int(unpack_data(tcbptr + tcbinfo["pid_off"], "<I")[0])
            tcb["state"] = int(unpack_data(tcbptr + tcbinfo["state_off"], "<B")[0])
            tcb["pri"] = int(unpack_data(tcbptr + tcbinfo["pri_off"], "<B")[0])
            tcb["stack"] = int(
                unpack_data(tcbptr + tcbinfo["stack_off"], self.reg_fmt)[0]
            )
            tcb["stack_size"] = int(
                unpack_data(tcbptr + tcbinfo["stack_size_off"], self.reg_fmt)[0]
            )
            tcb["regs"] = int(
                unpack_data(tcbptr + tcbinfo["regs_off"], self.reg_fmt)[0]
            )
            tcb["tcbptr"] = tcbptr
            i = 0
            tcb["name"] = ""
            while True:
                c = int(unpack_data(tcbptr + tcbinfo["name_off"] + i, "<B")[0])
                if c == 0:
                    break
                i += 1
                tcb["name"] += chr(c)

            return tcb

        def parse_regs_to_gdb(regs):
            gdb_regs = []
            for i in range(0, tcbinfo["regs_num"]):
                reg_off = int(unpack_data(tcbinfo["reg_off"] + i * 2, "<H", True)[0])
                if reg_off == UINT16_MAX:
                    gdb_regs.append(b"x")
                else:
                    gdb_regs.append(int(unpack_data(regs + reg_off, self.reg_fmt)[0]))
            return gdb_regs

        self.cpunum = self.elffile.symbol["g_running_tasks"]["st_size"] // 4
        logger.debug(f"Have {self.cpunum} cpu")
        unpacked_data = unpack_data(
            self.elffile.symbol["g_running_tasks"]["st_value"],
            f"<{self.cpunum}I",
        )

        self.running_tasks = {}
        last_regs_size = self.elffile.symbol["g_last_regs"]["st_size"] // self.cpunum
        logger.debug(f"last_regs_size is {last_regs_size}")
        for i in range(0, self.cpunum):
            self.running_tasks[int(unpacked_data[i])] = parse_regs_to_gdb(
                self.elffile.symbol["g_last_regs"]["st_value"] + i * last_regs_size
            )

        for tcbptr in tcbptr_list:
            if tcbptr == 0:
                continue
            thread_dict = {}
            tcb = parse_tcb(tcbptr)
            thread_dict["tcb"] = tcb
            thread_dict["gdb_regs"] = parse_regs_to_gdb(tcb["regs"])
            self.threadinfo.append(thread_dict)

    def handle_general_query_packet(self, pkt):
        if b"Rcmd" == pkt[1:5]:
            self.put_gdb_packet(b"OK")
        elif b"qfThreadInfo" == pkt[: len(b"qfThreadInfo")]:
            reply_str = "m"
            for thread in self.threadinfo:
                pid = thread["tcb"]["pid"]
                reply_str += "," + str(pid + 1)  # pid + 1 for gdb index

            reply = reply_str.encode("utf-8")
            self.put_gdb_packet(reply)

        elif b"qsThreadInfo" == pkt[: len(b"qsThreadInfo")]:
            self.put_gdb_packet(b"l")

        elif b"qThreadExtraInfo" == pkt[: len(b"qThreadExtraInfo")]:
            cmd, pid = pkt[1:].split(b",")
            pid = int(pid) - 1

            for thread in self.threadinfo:
                if thread["tcb"]["pid"] == pid:

                    pkt_str = "Name: %s, State: %d, Pri: %d, Stack: %x, Size: %d" % (
                        thread["tcb"]["name"],
                        thread["tcb"]["state"],
                        thread["tcb"]["pri"],
                        thread["tcb"]["stack"],
                        thread["tcb"]["stack_size"],
                    )
                    pkt = pkt_str.encode()
                    pkt_str = pkt.hex()
                    pkt = pkt_str.encode()
                    self.put_gdb_packet(pkt)
                    break
        else:
            self.put_gdb_packet(b"")

    def handle_vkill_packet(self, pkt):
        self.put_gdb_packet(b"OK")
        logger.debug("quit with gdb")
        sys.exit(0)

    def run(self, socket: socket.socket):
        self.socket = socket

        while True:
            pkt = self.get_gdb_packet()
            if pkt is None:
                continue

            pkt_type = pkt[0:1]
            logger.debug(f"Got packet type: {pkt_type}")

            if pkt_type == b"?":
                self.handle_signal_query_packet()
            elif pkt_type in (b"C", b"S"):
                # Continue/stepping execution, which is not supported.
                # So signal exception again
                self.handle_signal_query_packet()
            elif pkt_type == b"g":
                self.handle_register_group_read_packet()
            elif pkt_type == b"G":
                self.handle_register_group_write_packet()
            elif pkt_type == b"p":
                self.handle_register_single_read_packet(pkt)
            elif pkt_type == b"P":
                self.handle_register_single_write_packet(pkt)
            elif pkt_type == b"m":
                self.handle_memory_read_packet(pkt)
            elif pkt_type == b"M":
                self.handle_memory_write_packet(pkt)
            elif pkt_type == b"q":
                self.handle_general_query_packet(pkt)
            elif pkt.startswith(b"vKill") or pkt_type == b"k":
                # GDB quits
                self.handle_vkill_packet(pkt)
            elif pkt_type == b"H":
                self.handle_thread_context(pkt)
            elif pkt_type == b"T":
                self.handle_is_thread_active(pkt)
            else:
                self.put_gdb_packet(b"")


def arg_parser():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "-e", "--elffile", required=True, action="append", help="elffile"
    )
    parser.add_argument("-l", "--logfile", help="logfile")
    parser.add_argument(
        "-a",
        "--arch",
        help="Only use if can't be learnt from ELFFILE.",
        required=False,
        choices=[arch for arch in reg_table.keys()],
    )
    parser.add_argument("-p", "--port", help="gdbport", type=int, default=1234)
    parser.add_argument(
        "-g",
        "--gdb",
        help="provided a custom GDB path, automatically start GDB session and exit gdbserver when exit GDB. ",
        type=str,
    )
    parser.add_argument(
        "-i",
        "--init-cmd",
        nargs="?",
        default=argparse.SUPPRESS,
        help="provided a custom GDB init command, automatically start GDB sessions and input what you provide. "
        f"if you don't provide any command, it will use default command [{DEFAULT_GDB_INIT_CMD}]. ",
    )
    parser.add_argument(
        "-r",
        "--rawfile",
        nargs="*",
        help="rawfile is a binary file, args format like ram.bin:0x10000 ...",
    )

    parser.add_argument(
        "-c",
        "--coredump",
        nargs="?",
        help="coredump file, will prase memory in this file",
    )

    parser.add_argument(
        "-s",
        "--symbol",
        action="store_true",
        help="Analyze the symbol table in the ELF file, use in thread awareness"
        "if use logfile input, this option will is false by default"
        "if use rawfile or coredump input, this option will is true by default",
    )

    parser.add_argument(
        "--debug",
        action="store_true",
        default=False,
        help="if enabled, it will show more logs.",
    )
    return parser.parse_args()


def config_log(debug):
    if debug:
        logger.setLevel(logging.DEBUG)
    else:
        logger.setLevel(logging.INFO)

    logging.basicConfig(
        format="[%(levelname)s][%(asctime)s][%(lineno)d] %(message)s",
        datefmt="%H:%M:%S",
    )


def auto_parse_log_file(logfile):
    with open(logfile, errors="ignore") as f:
        dumps = []
        tmp_dmp = []
        start = False
        for line in f.readlines():
            line = line.strip()
            if len(line) == 0:
                continue

            if "up_dump_register" in line or "stack" in line:
                start = True
            else:
                if start:
                    start = False
                    dumps.append(tmp_dmp)
                    tmp_dmp = []
            if start:
                tmp_dmp.append(line)

        if start:
            dumps.append(tmp_dmp)

    terminal_width, _ = shutil.get_terminal_size()
    terminal_width = max(terminal_width - 4, 0)

    def get_one_line(lines):
        return "    ".join(lines[:2])[:terminal_width]

    if len(dumps) == 0:
        logger.error(f"Cannot find any dump in {logfile}, exiting...")
        sys.exit(1)

    if len(dumps) == 1:
        return dumps[0]

    for i in range(len(dumps)):
        print(f"{i}: {get_one_line(dumps[i])}")

    index_input = input("Dump number[0]: ").strip()
    if index_input == "":
        index_input = 0
    return dumps[int(index_input)]


def main(args):
    args.elffile = tuple(set(args.elffile))
    for name in args.elffile:
        if not os.path.isfile(name):
            logger.error(f"Cannot find file {name}, exiting...")
            sys.exit(1)

    if args.logfile:
        if not os.path.isfile(args.logfile):
            logger.error(f"Cannot find file {args.logfile}, exiting...")
            sys.exit(1)

    if not args.rawfile and not args.logfile and not args.coredump:
        logger.error("Must have a input file log or rawfile or coredump, exiting...")
        sys.exit(1)

    config_log(args.debug)
    elf = DumpELFFile(args.elffile[0])
    if args.symbol is False:
        if args.rawfile or args.coredump:
            args.symbol = True

    elf.parse(args.symbol)
    elf_texts = [elf.text()]
    for name in args.elffile[1:]:
        other = DumpELFFile(name)
        other.parse()
        elf_texts.append(other.text())
        elf.merge(other)

    if args.logfile is not None:
        selected_log = auto_parse_log_file(args.logfile)
        log = DumpLogFile(selected_log)
    else:
        log = DumpLogFile(None)

    if args.logfile is not None:
        if args.arch:
            log.parse(args.arch)
        elif elf.arch() in reg_table.keys():
            log.parse(elf.arch())
        else:
            logger.error("Architecture unknown, exiting...")
            sys.exit(2)

    raw = RawMemoryFile(args.rawfile)
    coredump = CoreDumpFile(args.coredump)
    gdb_stub = GDBStub(log, elf, raw, coredump, args.arch)

    gdbserver = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Reuse address so we don't have to wait for socket to be
    # close before we can bind to the port again
    gdbserver.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    try:
        gdbserver.bind(("", args.port))
    except OSError:
        gdbserver.bind(("", 0))
        logger.info(
            f"Port {args.port} is already in use, using port {gdbserver.getsockname()[1]} instead."
        )
        args.port = gdbserver.getsockname()[1]

    gdbserver.listen(1)

    gdb_exec = "gdb" if not args.gdb else args.gdb

    gdb_init_cmd = ""
    if hasattr(args, "init_cmd"):
        if args.init_cmd is not None:
            gdb_init_cmd = args.init_cmd.strip()
        else:
            gdb_init_cmd = DEFAULT_GDB_INIT_CMD

    gdb_cmd = [
        f"{gdb_exec} {args.elffile[0]} -ex 'target remote localhost:{args.port}' "
        f"{gdb_init_cmd}"
    ]
    for i in range(len(elf_texts[1:])):
        name = args.elffile[1 + i]
        text = hex(elf_texts[1 + i])
        gdb_cmd.append(f"-ex 'add-symbol-file {name} {text}'")
    gdb_cmd = "".join(gdb_cmd)

    logger.info(f"Waiting GDB connection on port {args.port} ...")

    if not args.gdb:
        logger.info("Press Ctrl+C to stop ...")
        logger.info(f"Hint: {gdb_cmd}")
    else:
        logger.info(f"Run GDB command: {gdb_cmd}")

        def gdb_run(cmd):
            try:
                subprocess.run(cmd, shell=True)
            except KeyboardInterrupt:
                pass

        multiprocessing.Process(target=gdb_run, args=(gdb_cmd,)).start()

    while True:
        try:
            conn, remote = gdbserver.accept()

            if conn:
                logger.info(f"Accepted GDB connection from {remote}")
                gdb_stub.run(conn)
        except KeyboardInterrupt:
            break

    gdbserver.close()


if __name__ == "__main__":
    main(arg_parser())
