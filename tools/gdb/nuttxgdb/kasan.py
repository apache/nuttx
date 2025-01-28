############################################################################
# tools/gdb/nuttxgdb/kasan.py
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

import itertools

import gdb

from . import utils


class KASanGeneric:
    def __init__(self, begin, end, shadow, bitwidth, scale, shift=0):
        self.begin = begin
        self.end = end
        self.shadow = shadow
        self.bitwidth = bitwidth
        self.scale = scale
        self.shift = shift

    def check_addr(self, addr):
        distance = (addr - self.begin) / self.scale
        index = distance / self.bitwidth
        bit = distance % self.bitwidth
        return True if self.shadow[index] >> bit & 0x01 else False

    def is_member(self, addr):
        if self.begin <= self.untag_addr(addr) <= self.end:
            return True
        return False

    def get_tag(self, addr):
        return addr >> self.shift

    def untag_addr(self, addr):
        return addr


class KASanSwtags(KASanGeneric):
    def check_addr(self, addr):
        tag = self.get_tag(addr)
        untag_addr = self.untag_addr(addr)
        distance = untag_addr - self.begin
        index = distance / self.scale
        return False if self.shadow[index] == tag else True

    def untag_addr(self, addr):
        mask = ~(0xFF << self.shift)
        return addr & mask


class KASan(gdb.Command):

    def __init__(self):
        super().__init__("kasandebug", gdb.COMMAND_USER)

        """ Common bit width and kasan alignment length in multiple modes """
        bitwidth = utils.get_symbol_value("sizeof(long)") * 8
        scale = utils.get_symbol_value("KASAN_SHADOW_SCALE")

        """ Get the array of KASan regions """
        g_region_count = utils.get_symbol_value("g_region_count")
        g_region = utils.get_symbol_value("g_region")

        self.regions: list[KASanGeneric] = []
        if utils.get_symbol_value("CONFIG_MM_KASAN_GENERIC") is not None:
            print("KASan Mode: Generic")
            for index in range(g_region_count):
                region = g_region[index]
                self.regions.append(
                    KASanGeneric(
                        region["begin"],
                        region["end"],
                        region["shadow"],
                        bitwidth,
                        scale,
                    )
                )

        if utils.get_symbol_value("CONFIG_MM_KASAN_SW_TAGS") is not None:
            print("KASan Mode: Softtags")
            shift = utils.get_symbol_value("KASAN_TAG_SHIFT")
            for index in range(g_region_count):
                region = g_region[index]
                self.regions.append(
                    KASanSwtags(
                        region["begin"],
                        region["end"],
                        region["shadow"],
                        bitwidth,
                        scale,
                        shift,
                    )
                )

        if utils.get_symbol_value("CONFIG_MM_KASAN_GLOBAL") is not None:
            print("KASan Support Checking Global Variables")
            scale = utils.get_symbol_value("CONFIG_MM_KASAN_GLOBAL_ALIGN")
            for index in itertools.count(0):
                addr = utils.get_symbol_value(f"g_global_region[{index}]")
                if addr == 0:
                    break
                region = utils.get_symbol_value(f"*g_global_region[{index}]")
                self.regions.append(
                    KASanGeneric(
                        region["begin"],
                        region["end"],
                        region["shadow"],
                        bitwidth,
                        scale,
                    )
                )

    def invoke(self, args, from_tty):
        addrs = [
            int(arg, 16) if arg.startswith("0x") else int(arg)
            for arg in args.split()
            if arg.startswith("0x") or arg.isdigit()
        ]
        for addr in addrs:
            for region in self.regions:
                if region.is_member(addr):
                    if region.check_addr(addr):
                        print("Addr 0x%x Error" % (addr))
                    else:
                        print("Addr 0x%x OK" % (addr))
                    break
