#!/usr/bin/env python3
# tools/callstack.py
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

import sys

syms = []


def get_symbol(x):
    try:
        addr = int(x, 16)
    except ValueError:
        return
    for num in range(len(syms) - 1):
        if syms[num][0] <= addr and addr < syms[num + 1][0]:
            return "[%08x] " % (addr) + syms[num][1] + " + 0x%x" % (addr - syms[num][0])


def main():
    argv = sys.argv
    argc = len(argv)

    if argc < 3:
        print("Usage: python %s <System.map> <stackdump.log>" % argv[0])
        quit()

    for line in open(argv[1], "r"):
        try:
            address, type, symbol = line[:-1].split(" ")
            if type == "T" or type == "t" or type == "W" or type == "w":
                syms.append((int(address, 16), symbol))
        except AttributeError:
            pass

    callstack = []
    for line in open(argv[2], "r"):
        print(line[:-1])
        if "stack_dump:" in line:
            for item in line.split(" "):
                callstack.append(get_symbol(item))

    print("----------------- callstack -----------------")
    for cs in callstack:
        if cs is not None:
            print(cs)


if __name__ == "__main__":
    main()
