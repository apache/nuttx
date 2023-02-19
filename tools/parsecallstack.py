#!/usr/bin/env python3
# tools/parsecallstack.py
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


def parse_args():

    parser = argparse.ArgumentParser(
        """
        parsecallstack.py -c CPUTYPE -f FILENAME\n\
        This file can get the call stack when you get the log with the
        register values from R0 to R15, together with the stack dump.\n
        Then you will get a file with name callstack.cmm, run this file
        in Trace32, load the symbol according to the indication, the call
        stack will pop up.\n
        Trace32 software is available at: https://www.lauterbach.com
        """
    )

    parser.add_argument(
        "-f",
        "--filename",
        action="store",
        help="log file with registers and stack information",
    )
    parser.add_argument(
        "-c",
        "--cputype",
        action="store",
        help='''It supports ARM family CPU such as:
            "CortexM0" "CortexM1"  "CortexM3"  "CortexM4"
            "CortexM7" "CortexM23" "CortexM33" "CortexM35P"
            "CortexR5" "CortexR7"  "CortexA5"  "CortexA7"''',
    )

    return parser.parse_args()


def get_regs(filename):

    reglist = []
    with open(filename, mode="r") as fl:
        for line in fl:
            lst = line.strip("\n").split(" ")
            if "R0:" in lst:
                reglist = lst[-8:]
            if "R8:" in lst:
                reglist += lst[-8:]

    return reglist


def get_stackvalue(filename):

    stackvalue = []
    first = 1
    with open(filename, mode="r") as fl:
        for line in fl:
            lst = line.strip("\n").split(" ")
            if "up_stackdump:" in lst:
                if first == 1:
                    first += 1
                    # strip ":" of sp
                    sp = lst[-9].strip(":")
                    # The first item is the sp to restore the stack.
                    stackvalue.append(sp)
                stackvalue += lst[-8:]

    return stackvalue


def generate_cmm(cpu, regs, stackvalue):

    filename = os.path.join(os.getcwd(), "callstack.cmm")
    with open(filename, mode="w") as fl:
        # Select the CPU and symbol.
        fl.write("SYStem.CPU %d\n" % cpu)
        fl.write("SYS.M UP\n")
        fl.write("Data.LOAD *\n")
        fl.write("\n")

        # Set R0-R15.
        for num in range(len(regs)):
            fl.write("Register.Set R%d 0x%s\n" % num, regs[num])
        fl.write("\n")

        # Recover the value in stack.
        sp = int(stackvalue[0], 16)
        for num in range(len(stackvalue) - 1):
            address = hex(sp + num * 4)
            value = stackvalue[num + 1]
            fl.write("Data.Set ZSD:%d %%LE %%Long 0x%d\n" % address, value)
        fl.write("\n")

        # Show the call stack.
        fl.write("data.view %%sYmbol.long %x\n" % sp)
        fl.write("frame.view /Locals /Caller\n")


if __name__ == "__main__":
    args = parse_args()
    regs = get_regs(args.filename)
    stackvalue = get_stackvalue(args.filename)
    generate_cmm(args.cpu, regs, stackvalue)
