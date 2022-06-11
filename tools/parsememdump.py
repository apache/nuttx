#!/usr/bin/python3
# tools/parsememdump.py
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

program_description = """
This program will help you analyze memdump log files,
analyze the number of occurrences of backtrace,
and output stack information
memdump log files need this format:
pid   size  addr   mem
"""


class dump_line:
    def __init__(self, line_str):
        self.mem = []
        self.err = 0
        self.cnt = 1
        tmp = re.search("( \d+ )", line_str)
        if tmp is None:
            self.err = 1
            return
        self.pid = int(tmp.group(0)[1:])
        tmp = re.search("( \d+ )", line_str[tmp.span()[1] :])
        if tmp is None:
            self.err = 1
            return
        self.size = int(tmp.group(0)[1:])

        tmp = re.findall("0x([0-9a-fA-F]+)", line_str[tmp.span()[1] :])
        self.addr = tmp[0]
        for str in tmp[1:]:
            self.mem.append(str)


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
    for tmp in dump_line_list:
        if tmp.mem is t.mem:
            find = 1
            tmp.cnt += 1
            break

    if find == 0:
        dump_line_list.append(t)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=program_description, formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument("-f", "--file", help="dump file", nargs=1, required=True)

    parser.add_argument(
        "-e",
        "--elffile",
        default="",
        help="elf file,use it can output stack info",
        nargs=1,
    )

    parser.add_argument("-o", "--output", help="output file,defult output shell")

    args = parser.parse_args()
    dump_file = open("%s" % args.file[0], "r")
    list = []
    while 1:
        str = dump_file.readline()
        if str == "":
            break
        compare_dump_line(list, str)
    dump_file.close()
    list.sort(key=lambda x: x.cnt, reverse=True)

    log = log_output(args)
    log.output("cnt   size   pid   addr         mem\n")
    for t in list:
        memstr = ""
        log.output("%-4d  %-6d %-3d   %s   " % (t.cnt, t.size, t.pid, t.addr))
        for mem in t.mem:
            log.output("%s " % mem)
            memstr += mem + " "
        log.output("\n")
        if args.elffile != "":
            addr2line_file = os.popen(
                "addr2line -Cfe %s %s" % (args.elffile[0], memstr), "r"
            )
            while 1:
                add2line_str = addr2line_file.readline()
                if add2line_str == "":
                    break
                log.output("    " + add2line_str)
            log.output("\n" + add2line_str)
    log.__del__()
