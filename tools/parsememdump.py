#!/usr/bin/env python3
# tools/parsememdump.py
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
import re
from concurrent.futures import ThreadPoolExecutor

program_description = """
This program will help you analyze memdump log files,
analyze the number of occurrences of backtrace,
and output stack information
memdump log files need this format:
pid   size  overhead seq  addr   mem
"""


class dump_line:
    def __init__(self, line_str):
        self.mem = []
        self.err = False
        self.cnt = 1
        self.pid = None
        self.size = None
        self.overhead = None
        self.seq = None
        self.addr = None
        self.parse_line(line_str)

    def parse_line(self, line_str):
        match = re.search(
            r"\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+((?:\s+0x[0-9a-fA-F]+)+)", line_str
        )
        if match:
            self.pid = int(match.group(1))
            self.size = int(match.group(2))
            self.overhead = int(match.group(3))
            self.seq = int(match.group(4))
            addresses = match.group(5).split()
            self.addr = addresses[0]
            self.mem = addresses[1:]
        else:
            self.err = True


class log_output:
    def __init__(self, args):
        if args.output:
            self.file = open(args.output, "w")

    def output(self, str):
        if hasattr(self, "file"):
            self.file.write(str)
        else:
            print(str, end="")

    def __del__(self):
        if hasattr(self, "file"):
            self.file.close()


def compare_dump_line(dump_line_list, str):
    t = dump_line(str)
    if t.err:
        return

    if dump_line_list.__len__() == 0:
        dump_line_list.append(t)
        return

    find = 0
    for line in dump_line_list:
        if line.mem == t.mem and line.size == t.size and t.mem != []:
            find = 1
            line.cnt += 1
            break

    if find == 0:
        dump_line_list.append(t)


def multi_thread_executer(cmd):
    result = ""
    p = os.popen(cmd, "r")
    while True:
        line = p.readline()
        if line == "":
            break
        result += f"    {line}"
    return result


class addr2line_db:
    def __init__(self, mem=[], ncpu=1, prefix="", file="nuttx.elf", batch_max=1):
        self.mem = mem
        self.ncpu = ncpu
        self.db = {}
        self.prefix = prefix
        self.file = file
        self.batch_max = batch_max
        self.parse_all()

    def split_array(self, arr, num_splits):
        k, m = divmod(len(arr), num_splits)
        return [
            arr[i * k + min(i, m) : (i + 1) * k + min(i + 1, m)]
            for i in range(num_splits)
        ]

    def parse_all(self):
        cmds = []
        batch_cnt = len(self.mem) // self.ncpu
        if batch_cnt > self.batch_max:
            batch_cnt = self.batch_max
        segments = self.split_array(self.mem, batch_cnt)

        for seg in segments:
            addrs = " ".join(seg)
            cmds.append(f"{self.prefix}addr2line -Cfe {self.file} {addrs}")

        with ThreadPoolExecutor(max_workers=self.ncpu) as executor:
            for keys, v in zip(segments, executor.map(multi_thread_executer, cmds)):
                lines = v.split("\n")
                values = [
                    lines[i] + "\n" + lines[i + 1] + "\n"
                    for i in range(0, len(lines) - 1, 2)
                ]
                for i in range(len(keys)):
                    self.db[keys[i]] = values[i]

    def parse(self, mem):
        if mem in self.db.keys():
            return self.db[mem]
        else:
            return ""


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=program_description, formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument("-f", "--file", help="dump file", nargs=1, required=True)
    parser.add_argument(
        "-p", "--prefix", help="addr2line program prefix", nargs=1, default=""
    )
    parser.add_argument(
        "-j",
        "--ncpu",
        help="multi thread count, default all",
        type=int,
        default=0,
        required=False,
    )
    parser.add_argument(
        "-e",
        "--elffile",
        default="",
        help="elf file,use it can output stack info",
        nargs=1,
    )

    parser.add_argument("-o", "--output", help="output file, default output shell")

    args = parser.parse_args()
    dump_file = open("%s" % args.file[0], "r")
    lines = []
    while 1:
        line = dump_file.readline()
        if line == "":
            break
        compare_dump_line(lines, line)
    dump_file.close()
    lines.sort(key=lambda x: x.cnt, reverse=True)

    log = log_output(args)
    total_dir = {}
    for t in lines:
        if t.pid in total_dir:
            total_dir[t.pid] += t.size * t.cnt
        else:
            total_dir.setdefault(t.pid, t.size * t.cnt)

    log.output("total memory used for ervey pid\n")
    log.output("pid       total size\n")
    total_size = 0
    for pid, size in sorted(total_dir.items(), key=lambda x: x[1]):
        log.output("%-3d       %-6d\n" % (pid, size))
        total_size += size
    log.output("all used memory %-6d\n" % (total_size))

    log.output("cnt   size   pid  overhead   addr         mem\n")

    mems = []
    for line in lines:
        if line.mem == []:
            continue
        for mem in line.mem:
            if mem not in mems:
                mems.append(mem)

    ncpu = args.ncpu
    if ncpu == 0:
        ncpu = os.cpu_count()

    db = addr2line_db(mem=mems, ncpu=ncpu, prefix=args.prefix[0], file=args.elffile[0])
    for t in lines:
        addr2line_str = ""
        log.output(
            "%-4d  %-6d %-6d %-3d   %s   " % (t.cnt, t.size, t.overhead, t.pid, t.addr)
        )
        if t.mem == []:
            log.output("\n")
            continue
        for mem in t.mem:
            log.output("%s " % mem)
            addr2line_str += db.parse(mem)
        log.output("\n")
        if addr2line_str != "":
            log.output(addr2line_str)
        log.output("\n")
    log.__del__()
