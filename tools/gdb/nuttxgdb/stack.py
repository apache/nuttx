############################################################################
# tools/gdb/nuttxgdb/stack.py
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

import traceback

import gdb

from . import utils

STACK_COLORATION_PATTERN = utils.get_symbol_value(
    "STACK_COLOR", locspec="up_create_stack"
)


class Stack(object):
    def __init__(self, name, entry, base, alloc, size, cursp, align):
        # We don't care about the stack growth here, base always point to the lower address!
        self._thread_name = name
        self._thread_entry = entry
        self._stack_base = base
        self._stack_alloc = alloc
        self._stack_top = base + size
        self._cur_sp = cursp
        self._stack_size = size
        self._align = align
        self._pattern = STACK_COLORATION_PATTERN

        self._sanity_check()

    def _sanity_check(self):
        # do some basic sanity checking to make sure we have a sane stack object
        if (
            self._stack_base < self._stack_alloc
            or not self._stack_size
            or self._cur_sp <= self._stack_base
            or self._cur_sp > self._stack_base + self._stack_size
        ):

            gdb.write(
                f"base: {self._stack_base}, \
                size: {self._stack_size}, sp: {self._cur_sp}\n"
            )

            raise gdb.GdbError("Inconsistant stack size...Maybe memory corruption?")

        # TODO: check if stack ptr is located at a sane address range!

    def cur_usage(self):
        usage = self._stack_top - self._cur_sp

        if self.is_stackof():
            gdb.write("An overflow detected, dumping the stack:\n")

            ptr_4bytes = gdb.Value(self._stack_base).cast(
                utils.lookup_type("unsigned int").pointer()
            )

            for i in range(0, self._stack_size // 4):
                if i % 8 == 0:
                    gdb.write(f"{hex(self._stack_base + 4 * i)}: ")

                gdb.write(f"{hex(ptr_4bytes[i]):10} ")

                if i % 8 == 7:
                    gdb.write("\n")

            gdb.write("\n")
            raise gdb.GdbError(
                "pls check your stack size! @ {0} sp:{1:x} base:{2:x}".format(
                    self._thread_name, self._cur_sp, self._stack_base
                )
            )

        return usage

    def check_max_usage(self):
        ptr_4bytes = gdb.Value(self._stack_base).cast(
            utils.lookup_type("unsigned int").pointer()
        )

        spare = 0

        for i in range(0, self._stack_size // 4):
            if int(ptr_4bytes[i]) != self._pattern:
                spare = i * 4
                break
        return self._stack_size - spare

    def max_usage(self):
        if not utils.get_symbol_value("CONFIG_STACK_COLORATION"):
            return 0

        return self.check_max_usage()

    def avalaible(self):
        cur_usage = self.cur_usage()
        return self._stack_size - cur_usage

    def maxdepth_backtrace(self):
        raise gdb.GdbError("Not implemented yet", traceback.print_stack())

    def cur_sp(self):
        return self._cur_sp

    def is_stackof(self):
        # we should notify the user if the stack overflow is about to happen as well!
        return self._cur_sp <= self._stack_base

    def has_stackof(self):
        max_usage = self.max_usage()

        return max_usage >= self._stack_size


# Always refetch the stack infos, never cached as we may have threads created/destroyed
# dynamically!
def fetch_stacks():
    stacks = dict()

    for tcb in utils.get_tcbs():
        # We have no way to detect if we are in an interrupt context for now.
        # Originally we use `and not utils.in_interrupt_context()`
        if tcb["task_state"] == gdb.parse_and_eval("TSTATE_TASK_RUNNING"):
            sp = utils.get_sp()
        else:
            sp = utils.get_sp(tcb=tcb)

        try:
            stacks[int(tcb["pid"])] = Stack(
                utils.get_task_name(tcb),
                hex(tcb["entry"]["pthread"]),  # should use main?
                int(tcb["stack_base_ptr"]),
                int(tcb["stack_alloc_ptr"]),
                int(tcb["adj_stack_size"]),
                sp,
                4,
            )

        except gdb.GdbError as e:
            pid = tcb["pid"]
            gdb.write(
                f"Failed to construction stack object for tcb {pid} due to: {e}\n"
            )

    return stacks


class StackUsage(gdb.Command):
    """Display the stack usage of each thread, similar to cat /proc/<pid>/stack"""

    def __init__(self):
        super().__init__("stack-usage", gdb.COMMAND_USER)
        self._stacks = []
        # format template
        self._fmt = (
            "{0: <4} | {1: <10} | {2: <10} | {3: <20} | {4: <10} | {5: <10} | {6: <10}"
        )

    def format_print(self, pid, stack):
        def gen_info_str(x):
            usage = x / stack._stack_size
            res = ",".join([str(x), "{0:.2%}".format(usage)])
            if usage > 0.8:
                res += "!"
            return res

        gdb.write(
            self._fmt.format(
                pid,
                stack._thread_name[:10],
                stack._thread_entry,
                hex(stack._stack_base),
                stack._stack_size,
                gen_info_str(stack.cur_usage()),
                gen_info_str(stack.max_usage()),
            )
        )
        gdb.write("\n")

    def invoke(self, args, from_tty):
        stacks = fetch_stacks()

        args = [int(arg) for arg in args.split()]

        pids = stacks.keys() if len(args) == 0 else args

        gdb.write(
            self._fmt.format(
                "Pid", "Name", "Entry", "Base", "Size", "CurUsage", "MaxUsage"
            )
        )
        gdb.write("\n")

        for pid in pids:
            stack = stacks.get(pid)

            if not stack:
                continue

            self.format_print(pid, stack)
