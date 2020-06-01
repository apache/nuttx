#!/usr/bin/python
# -*- coding:utf-8 -*-
#
# nuttx/tools/parsecallstack.py
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

import os
import argparse

def parse_args():

    parser = argparse.ArgumentParser("\n\
        parsecallstack.py -c cputype -f filename\n\
        This file can get the call stack when you get the log with the\n\
        register values from R0 to R15, together with the stack dump.\n\n\
        Then you can get a file with name callstack.cmm, run this file in\n\
        Trace32 simulator, load the symbol accoring to the indication,\n\
        the call stack will pop up.\n\n\
        Trace32 software is avaliable at: https://www.lauterbach.com\n")

    parser.add_argument("-f", "--filename", action = "store",
        help = "log file with registers and stack information")
    parser.add_argument("-c", "--cputype", action = "store",
        help = "It supports ARM family CPU such as:\n\
            \"CortexM0\"\n\
            \"CortexM1\"\n\
            \"CortexM3\"\n\
            \"CortexM4\"\n\
            \"CortexM7\"\n\
            \"CortexM23\"\n\
            \"CortexM33\"\n\
            \"CortexM35P\"\n\
            \"CortexR5\"\n\
            \"CortexR7\"\n\
            \"CortexA5\"\n\
            \"CortexA7\"\n\
            ")
    args = parser.parse_args()

    return args

def get_regs(filename):

    reglist = []
    with open(filename, mode='r') as fl:
        for line in fl:
            lst = line.strip('\n').split(' ')
            if "R0:" in lst:
                reglist = lst[-8:]
            if "R8:" in lst:
                reglist += lst[-8:]

    return reglist

def get_stackvalue(filename):

    stackvalue = []
    first = 1
    with open(filename, mode='r') as fl:
        for line in fl:
            lst = line.strip('\n').split(' ')
            if "up_stackdump:" in lst:
                if first == 1:
                    first += 1
                    # strip ":" of sp
                    sp = lst[-9].strip(':')
                    # The first item is the sp to restore the stack.
                    stackvalue.append(sp)
                stackvalue += lst[-8:]

    return stackvalue

def generate_cmm(cpu, regs, stackvalue):

    dir = os.getcwd()
    filename = dir + "\\callstack.cmm"

    with open(filename, mode='w') as fl:
        # Select the CPU and symbol.
        fl.write("SYStem.CPU " + cpu + "\n")
        fl.write("SYS.M UP\n")
        fl.write("Data.LOAD *\n")
        fl.write("\n")

        # Set R0-R15.
        for num in range(len(regs)):
            fl.write("Register.Set R" + str(num) + " 0x" + regs[num] +'\n')
        fl.write('\n')

        # Recover the value in stack.
        sp = int("0x" + stackvalue[0], 16)
        for num in range(len(stackvalue) - 1):
            address = hex(sp + num * 4)
            value = stackvalue[num + 1]
            fl.write("Data.Set ZSD:" + str(address) + " %LE %Long 0x"
                + str(value) +'\n')
        fl.write('\n')

        # Show the call stack.
        fl.write("data.view %sYmbol.long " + str(hex(sp)) + '\n')
        fl.write("frame.view /Locals /Caller" +'\n')

if __name__ == "__main__":
    try:
        args = parse_args()
        filename = args.filename
        cpu = args.cputype
        if (os.path.isfile(filename)):
            regs = get_regs(filename)
            stackvalue = get_stackvalue(filename)
            generate_cmm(cpu, regs, stackvalue)
        else:
            print("The file is not exist!")

    except TypeError:
        print("Please provide the log file!")
