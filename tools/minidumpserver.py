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
import os
import re
import socket
import struct
import sys

import elftools
from elftools.elf.elffile import ELFFile

# ELF section flags
SHF_WRITE = 0x1
SHF_ALLOC = 0x2
SHF_EXEC = 0x4
SHF_WRITE_ALLOC = SHF_WRITE | SHF_ALLOC
SHF_ALLOC_EXEC = SHF_ALLOC | SHF_EXEC


logger = logging.getLogger()


class dump_elf_file:
    """
    Class to parse ELF file for memory content in various sections.
    There are read-only sections (e.g. text and rodata) where
    the memory content does not need to be dumped via coredump
    and can be retrieved from the ELF file.
    """

    def __init__(self, elffile):
        self.elffile = elffile
        self.fd = None
        self.elf = None
        self.memories = list()

    def open(self):
        self.fd = open(self.elffile, "rb")
        self.elf = ELFFile(self.fd)

    def close(self):
        self.fd.close()

    def parse(self):
        if self.fd is None:
            self.open()

        for section in self.elf.iter_sections():
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
                memory = {"start": start, "end": end, "data": section.data()}
                logger.info(
                    "ELF Section: 0x%x to 0x%x of size %d (%s)"
                    % (memory["start"], memory["end"], len(memory["data"]), desc)
                )

                self.memories.append(memory)

        return True


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
    },
    "xtensa": {
        "PC": 0,
        "SAR": 68,
        "PS": 73,
        "SCOM": 29,
        "A0": 21,
        "A1": 22,
        "A2": 23,
        "A3": 24,
        "A4": 25,
        "A5": 26,
        "A6": 27,
        "A7": 28,
        "A8": 29,
        "A9": 30,
        "A10": 31,
        "A11": 32,
        "A12": 33,
        "A13": 34,
        "A14": 35,
        "A15": 36,
    }
}


class dump_log_file:
    def __init__(self, logfile):
        self.logfile = logfile
        self.fd = None
        self.arch = ""
        self.registers = []
        self.memories = list()

    def open(self):
        self.fd = open(self.logfile, "r")

    def close(self):
        self.fd.closeself()

    def parse(self):
        data = bytes()
        start = 0
        if self.fd is None:
            self.open()
        while 1:
            line = self.fd.readline()
            if line == "":
                break

            tmp = re.search(r"([^ ]*)_registerdump:?", line)
            if tmp is not None:
                # find arch
                self.arch = tmp.group(1)
                if self.arch not in reg_table:
                    logger.error("%s not supported" % (self.arch))
                # init register list
                if len(self.registers) == 0:
                    for x in range(max(reg_table[self.arch].values()) + 1):
                        self.registers.append(b"x")

                # find register value
                line = line[tmp.span()[1] :]
                line = line.replace("\n", " ")
                while 1:
                    tmp = re.search("([^ ]+):", line)
                    if tmp is None:
                        break
                    register = tmp.group(1)
                    line = line[tmp.span()[1] :]
                    tmp = re.search("([0-9a-fA-F]+) ", line)
                    if tmp is None:
                        break
                    if register in reg_table[self.arch].keys():
                        self.registers[reg_table[self.arch][register]] = int(
                            "0x" + tmp.group().replace(" ", ""), 16
                        )
                    line = line[tmp.span()[1] :]
                continue

            tmp = re.search("stackdump:", line)
            if tmp is not None:
                # find stackdump
                line = line[tmp.span()[1] :]
                tmp = re.search("([0-9a-fA-F]+):", line)
                if tmp is not None:
                    line_start = int("0x" + tmp.group()[:-1], 16)

                    if start + len(data) != line_start:
                        # stack is not contiguous
                        if len(data) == 0:
                            start = line_start
                        else:
                            memory = {
                                "start": start,
                                "end": start + len(data),
                                "data": data,
                            }
                            self.memories.append(memory)
                            data = b""
                            start = line_start

                    line = line[tmp.span()[1] :]
                    line = line.replace("\n", " ")

                    while 1:
                        # record stack value
                        tmp = re.search(" ([0-9a-fA-F]+)", line)
                        if tmp is None:
                            break
                        data = data + struct.pack(
                            "<I", int("0x" + tmp.group().replace(" ", ""), 16)
                        )
                        line = line[tmp.span()[1] :]

        if len(data):
            memory = {"start": start, "end": start + len(data), "data": data}
            self.memories.append(memory)


GDB_SIGNAL_DEFAULT = 7


class gdb_stub:
    def __init__(self, logfile, elffile):
        self.logfile = logfile
        self.elffile = elffile
        self.socket = None
        self.gdb_signal = GDB_SIGNAL_DEFAULT
        self.mem_regions = self.elffile.memories + self.logfile.memories

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
        reg_fmt = "<I"
        pkt = b""

        for reg in self.logfile.registers:
            if reg != b"x":
                bval = struct.pack(reg_fmt, reg)
                pkt += binascii.hexlify(bval)
            else:
                # Register not in coredump -> unknown value
                # Send in "xxxxxxxx"
                pkt += b"x" * 8

        self.put_gdb_packet(pkt)

    def handle_register_single_read_packet(self, pkt):
        # Mark registers as "<unavailable>".
        # 'p' packets are usually used for registers
        # other than the general ones (e.g. eax, ebx)
        # so we can safely reply "xxxxxxxx" here.
        self.put_gdb_packet(b"x" * 8)

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
            for r in self.mem_regions:
                if r["start"] <= addr <= r["end"]:
                    return r

            return None

        # extract address and length from packet
        # and convert them into usable integer values
        addr, length = pkt[1:].split(b",")
        s_addr = int(b"0x" + addr, 16)
        length = int(b"0x" + length, 16)

        # FIXME: Need more efficient way of extracting memory content
        remaining = length
        addr = s_addr
        barray = b""
        r = get_mem_region(addr)
        while remaining > 0:
            if r is None:
                barray = None
                break

            if addr > r["end"]:
                r = get_mem_region(addr)
                continue

            offset = addr - r["start"]
            barray += r["data"][offset:offset + 1]

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

    def run(self, socket):
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
            elif pkt_type == b"k":
                # GDB quits
                break
            else:
                self.put_gdb_packet(b"")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("-e", "--elffile", required=True, help="elffile")

    parser.add_argument("-l", "--logfile", required=True, help="logfile")

    parser.add_argument("-p", "--port", help="gdbport", type=int, default=1234)

    parser.add_argument("--debug", action="store_true", default=False)

    args = parser.parse_args()

    if not os.path.isfile(args.elffile):
        logger.error(f"Cannot find file {args.elffile}, exiting...")
        sys.exit(1)

    if not os.path.isfile(args.logfile):
        logger.error(f"Cannot find file {args.logfile}, exiting...")
        sys.exit(1)

    if args.debug:
        logger.setLevel(logging.DEBUG)
    else:
        logger.setLevel(logging.INFO)

    log = dump_log_file(args.logfile)
    log.parse()
    elf = dump_elf_file(args.elffile)
    elf.parse()

    gdbstub = gdb_stub(log, elf)
    logging.basicConfig(format="[%(levelname)s][%(name)s] %(message)s")

    gdbserver = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Reuse address so we don't have to wait for socket to be
    # close before we can bind to the port again
    gdbserver.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    gdbserver.bind(("", args.port))
    gdbserver.listen(1)

    logger.info(f"Waiting GDB connection on port {args.port} ...")

    conn, remote = gdbserver.accept()

    if conn:
        logger.info(f"Accepted GDB connection from {remote}")

        gdbstub.run(conn)

        conn.close()

    gdbserver.close()
