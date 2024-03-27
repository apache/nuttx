#!/usr/bin/env python3
# tools/minidumpserver.py
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

import elftools
from elftools.elf.elffile import ELFFile

# ELF section flags
SHF_WRITE = 0x1
SHF_ALLOC = 0x2
SHF_EXEC = 0x4
SHF_WRITE_ALLOC = SHF_WRITE | SHF_ALLOC
SHF_ALLOC_EXEC = SHF_ALLOC | SHF_EXEC

GDB_SIGNAL_DEFAULT = 7

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
        "WINDOWBASE": 0,
        "WINDOWSTART": 1,
        "PS": 0x40000,
    },
    "xtensa": {
        "WINDOWBASE": 0,
        "WINDOWSTART": 1,
        "PS": 0x40000,
    },
    "riscv": {
        "ZERO": 0,
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

    def parse(self):
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
                    # Data section
                    #
                    # Running app changes the content so no need
                    # to store
                    pass
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

    def _init_register(self):
        # registers list should be able to hold the max index
        self.registers = [b"x"] * (max(self.reg_table.values()) + 1)

    def _parse_register(self, line):
        line = str_get_after(line, "up_dump_register:")
        if line is None:
            return False

        line = line.strip()
        # find register value
        find_res = re.findall(r"(?P<REG>\w+): (?P<REGV>[0-9a-fA-F]+)", line)

        for reg_name, reg_val in find_res:
            if reg_name in self.reg_table:
                reg_index = self.reg_table[reg_name]
                self.registers[reg_index] = int(reg_val, 16)
            else:
                raise Exception("Unknown register name: ", reg_name)

        return True

    def _parse_fix_register(self, arch):
        if arch in reg_fix_value:
            for reg_name, reg_vals in reg_fix_value[arch].items():
                reg_index = self.reg_table[reg_name]
                self.registers[reg_index] = reg_vals

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

        for val in match_res.groupdict()["VALS"].split():
            data = data + struct.pack("<I", int(val, 16))

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


class GDBStub:
    def __init__(self, logfile: DumpLogFile, elffile: DumpELFFile):
        self.logfile = logfile
        self.elffile = elffile
        self.socket = None
        self.gdb_signal = GDB_SIGNAL_DEFAULT
        self.mem_regions = self.elffile.get_memories() + self.logfile.get_memories()
        self.reg_digits = elffile.xlen() // 4

        self.mem_regions.sort(key=lambda x: x["start"])

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
        reg_fmt = "<Q"
        pkt = b""

        for reg in self.logfile.registers:
            if reg != b"x":
                bval = struct.pack(reg_fmt, reg)
                pkt += binascii.hexlify(bval)
            else:
                # Register not in coredump -> unknown value
                # Send in "xxxxxxxx"
                pkt += b"x" * self.reg_digits

        self.put_gdb_packet(pkt)

    def handle_register_single_read_packet(self, pkt):
        reg_fmt = "<Q"
        logger.debug(f"pkt: {pkt}")

        reg = int("0x" + pkt[1:].decode("utf8"), 16)
        if reg < len(self.logfile.registers) and self.logfile.registers[reg] != b"x":
            bval = struct.pack(reg_fmt, self.logfile.registers[reg])
            self.put_gdb_packet(binascii.hexlify(bval))
        else:
            self.put_gdb_packet(b"x" * self.reg_digits)

    def handle_register_group_write_packet(self):
        # the 'G' packet for writing to a group of registers
        #
        # We don't support writing so return error
        self.put_gdb_packet(b"E01")

    def handle_register_single_write_packet(self, pkt):
        # the 'P' packet for writing to registers
        #
        # We don't support writing so return error
        self.put_gdb_packet(b"E01")

    def handle_memory_read_packet(self, pkt):
        # the 'm' packet for reading memory: m<addr>,<len>

        def get_mem_region(addr):
            left = 0
            right = len(self.mem_regions) - 1
            while left <= right:
                mid = (left + right) // 2
                if (
                    self.mem_regions[mid]["start"]
                    <= addr
                    <= self.mem_regions[mid]["end"]
                ):
                    return self.mem_regions[mid]
                elif addr < self.mem_regions[mid]["start"]:
                    right = mid - 1
                else:
                    left = mid + 1

            return None

        # extract address and length from packet
        # and convert them into usable integer values
        addr, length = pkt[1:].split(b",")
        s_addr = int(addr, 16)
        length = int(length, 16)

        remaining = length
        addr = s_addr
        barray = b""
        r = get_mem_region(addr)
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

    def handle_general_query_packet(self, pkt):
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
            else:
                self.put_gdb_packet(b"")


def arg_parser():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "-e", "--elffile", required=True, action="append", help="elffile"
    )
    parser.add_argument("-l", "--logfile", required=True, help="logfile")
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
        help="provided a custom GDB path, automatically start GDB session and exit minidumpserver when exit GDB. ",
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

    logging.basicConfig(format="[%(levelname)s][%(name)s] %(message)s")


def auto_parse_log_file(logfile):
    with open(logfile, errors="ignore") as f:
        dumps = []
        tmp_dmp = []
        start = False
        for line in f.readlines():
            line = line.strip()
            if (
                "up_dump_register" in line
                or "dump_stack" in line
                or "stack_dump" in line
            ):
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

    if not os.path.isfile(args.logfile):
        logger.error(f"Cannot find file {args.logfile}, exiting...")
        sys.exit(1)

    config_log(args.debug)

    selected_log = auto_parse_log_file(args.logfile)

    # parse ELF fisrt to get arch
    elf = DumpELFFile(args.elffile[0])
    elf.parse()
    elf_texts = [elf.text()]
    for name in args.elffile[1:]:
        other = DumpELFFile(name)
        other.parse()
        elf_texts.append(other.text())
        elf.merge(other)

    log = DumpLogFile(selected_log)
    if args.arch:
        log.parse(args.arch)
    elif elf.arch() in reg_table.keys():
        log.parse(elf.arch())
    else:
        logger.error("Architecture unknown, exiting...")
        sys.exit(2)

    gdb_stub = GDBStub(log, elf)

    gdbserver = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Reuse address so we don't have to wait for socket to be
    # close before we can bind to the port again
    gdbserver.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    gdbserver.bind(("", args.port))
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
