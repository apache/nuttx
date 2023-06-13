############################################################################
# tools/gdb/memdump.py
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

import gdb
import utils
from lists import list_for_each_entry, sq_for_every, sq_is_empty, sq_queue

mempool_backtrace = utils.CachedType("struct mempool_backtrace_s")

MM_ALLOC_BIT = 0x1
MM_PREVFREE_BIT = 0x2
MM_MASK_BIT = MM_ALLOC_BIT | MM_PREVFREE_BIT


PID_MM_FREE = -4
PID_MM_ALLOC = -3
PID_MM_LEAK = -2
PID_MM_MEMPOOL = -1


def mm_foreach(heap):
    """Iterate over a heap, yielding each node"""
    node = gdb.Value(heap["mm_heapstart"][0]).cast(
        gdb.lookup_type("struct mm_allocnode_s").pointer()
    )
    while 1:
        yield node
        next = gdb.Value(node).cast(gdb.lookup_type("char").pointer())
        next = gdb.Value(next + (node["size"] & ~MM_MASK_BIT)).cast(
            gdb.lookup_type("struct mm_allocnode_s").pointer()
        )
        if node >= heap["mm_heapend"].dereference() or next == node:
            break
        node = next


def mempool_multiple_foreach(mpool):
    """Iterate over all pools in a mempool, yielding each pool"""
    i = 0
    while i < mpool["npools"]:
        pool = mpool["pools"] + i
        yield pool
        i += 1


class Nxmemdump(gdb.Command):
    """Dump the heap and mempool memory"""

    def __init__(self):
        super(Nxmemdump, self).__init__("memdump", gdb.COMMAND_USER)

    def mempool_dump(self, mpool, pid, seqmin, seqmax):
        """Dump the mempool memory"""
        for pool in mempool_multiple_foreach(mpool):
            if pid == PID_MM_FREE:
                entry = sq_queue.get_type().pointer()

                for entry in sq_for_every(pool["queue"], entry):
                    gdb.write("%12u%#*x\n" % (pool["blocksize"], self.align, entry))
                    self.aordblks += 1
                    self.uordblks += pool["blocksize"]

                for entry in sq_for_every(pool["iqueue"], entry):
                    gdb.write("%12u%#*x\n" % (pool["blocksize"], self.align, entry))
                    self.aordblks += 1
                    self.uordblks += pool["blocksize"]
            else:
                for node in list_for_each_entry(
                    pool["alist"], mempool_backtrace.get_type().pointer(), "node"
                ):
                    if (pid == node["pid"] or pid == PID_MM_ALLOC) and (
                        node["seqno"] >= seqmin and node["seqno"] < seqmax
                    ):
                        charnode = gdb.Value(node).cast(
                            gdb.lookup_type("char").pointer()
                        )
                        gdb.write(
                            "%6d%12u%12u%#*x"
                            % (
                                node["pid"],
                                pool["blocksize"] & ~MM_MASK_BIT,
                                node["seqno"],
                                self.align,
                                (int)(charnode - pool["blocksize"]),
                            )
                        )
                        if node.type.has_key("backtrace"):
                            max = node["backtrace"].type.range()[1]
                            for x in range(0, max):
                                gdb.write(" ")
                                gdb.write(
                                    node["backtrace"][x].format_string(
                                        raw=False, symbols=True, address=False
                                    )
                                )

                        gdb.write("\n")
                        self.aordblks += 1
                        self.uordblks += pool["blocksize"]

    def memdump(self, pid, seqmin, seqmax):
        """Dump the heap memory"""
        if pid >= PID_MM_ALLOC:
            gdb.write("Dump all used memory node info:\n")
            gdb.write(
                "%6s%12s%12s%*s %s\n"
                % ("PID", "Size", "Sequence", self.align, "Address", "Callstack")
            )
        else:
            gdb.write("Dump all free memory node info:\n")
            gdb.write("%12s%*s\n" % ("Size", self.align, "Address"))

        heap = gdb.parse_and_eval("g_mmheap")
        if heap.type.has_key("mm_mpool"):
            self.mempool_dump(heap["mm_mpool"], pid, seqmin, seqmax)

        for node in mm_foreach(heap):
            if node["size"] & MM_ALLOC_BIT != 0:
                if (pid == node["pid"] or (pid == PID_MM_ALLOC and node["pid"] != PID_MM_MEMPOOL)) and (
                    node["seqno"] >= seqmin and node["seqno"] < seqmax
                ):
                    charnode = gdb.Value(node).cast(gdb.lookup_type("char").pointer())
                    gdb.write(
                        "%6d%12u%12u%#*x"
                        % (
                            node["pid"],
                            node["size"] & ~MM_MASK_BIT,
                            node["seqno"],
                            self.align,
                            (int)(
                                charnode
                                + gdb.lookup_type("struct mm_allocnode_s").sizeof
                            ),
                        )
                    )

                    if node.type.has_key("backtrace"):
                        max = node["backtrace"].type.range()[1]
                        for x in range(0, max):
                            gdb.write(" ")
                            gdb.write(
                                node["backtrace"][x].format_string(
                                    raw=False, symbols=True, address=False
                                )
                            )

                        gdb.write("\n")

                    self.aordblks += 1
                    self.uordblks += node["size"] & ~MM_MASK_BIT
            else:
                if pid == PID_MM_FREE:
                    charnode = gdb.Value(node).cast(gdb.lookup_type("char").pointer())
                    gdb.write(
                        "%12u%#*x\n"
                        % (
                            node["size"] & ~MM_MASK_BIT,
                            self.align,
                            (int)(
                                charnode
                                + gdb.lookup_type("struct mm_allocnode_s").sizeof
                            ),
                        )
                    )
                    self.aordblks += 1
                    self.uordblks += node["size"] & ~MM_MASK_BIT

        gdb.write("%12s%12s\n" % ("Total Blks", "Total Size"))
        gdb.write("%12d%12d\n" % (self.aordblks, self.uordblks))

    def complete(self, text, word):
        return gdb.COMPLETE_SYMBOL

    def invoke(self, args, from_tty):
        if gdb.lookup_type("size_t").sizeof == 4:
            self.align = 11
        else:
            self.align = 19

        arg = args.split(" ")

        if arg[0] == "":
            pid = PID_MM_ALLOC
        elif arg[0] == "used":
            pid = PID_MM_ALLOC
        elif arg[0] == "free":
            pid = PID_MM_LEAK
        else:
            pid = int(arg[0])

        if len(arg) == 2:
            seqmin = int(arg[1])
        else:
            seqmin = 0

        if len(arg) == 3:
            seqmax = int(arg[2])
        else:
            seqmax = 0xFFFFFFFF

        self.aordblks = 0
        self.uordblks = 0
        self.memdump(pid, seqmin, seqmax)


Nxmemdump()
