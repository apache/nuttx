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

import argparse
import bisect
import time

import gdb
from lists import sq_for_every, sq_queue
from utils import get_long_type, get_symbol_value, read_ulong

MM_ALLOC_BIT = 0x1
MM_PREVFREE_BIT = 0x2
MM_MASK_BIT = MM_ALLOC_BIT | MM_PREVFREE_BIT
MEMPOOL_MAGIC_ALLOC = 0x55555555

PID_MM_FREE = -4
PID_MM_ALLOC = -3
PID_MM_LEAK = -2
PID_MM_MEMPOOL = -1


def align_up(size, align) -> int:
    """Align the size to the specified alignment"""
    return (size + (align - 1)) & ~(align - 1)


def mm_nodesize(size) -> int:
    """Return the real size of a memory node"""
    return size & ~MM_MASK_BIT


def mm_foreach(heap):
    """Iterate over a heap, yielding each node"""
    node = gdb.Value(heap["mm_heapstart"][0]).cast(
        gdb.lookup_type("struct mm_allocnode_s").pointer()
    )

    for region in range(0, get_symbol_value("CONFIG_MM_REGIONS")):
        while 1:
            yield node
            next = gdb.Value(node).cast(gdb.lookup_type("char").pointer())
            next = gdb.Value(next + mm_nodesize(node["size"])).cast(
                gdb.lookup_type("struct mm_allocnode_s").pointer()
            )

            if next >= heap["mm_heapend"][region] or next == node:
                break
            node = next


def mm_dumpnode(node, count, align, simple, detail, alive):
    if node["size"] & MM_ALLOC_BIT != 0:
        charnode = gdb.Value(node).cast(gdb.lookup_type("char").pointer())
        if not alive:
            # if pid is not alive put a red asterisk.
            gdb.write("\x1b[33;1m*\x1b[m")

        if not detail:
            gdb.write("%*d" % (6 if alive else 5, count))

        gdb.write(
            "%6d%12u%12u%#*x"
            % (
                node["pid"],
                mm_nodesize(node["size"]),
                node["seqno"],
                align,
                (int)(charnode + gdb.lookup_type("struct mm_allocnode_s").sizeof),
            )
        )

        if node.type.has_key("backtrace"):
            max = node["backtrace"].type.range()[1]
            firstrow = True
            for x in range(0, max):
                backtrace = int(node["backtrace"][x])
                if backtrace == 0:
                    break

                if simple:
                    gdb.write(" %0#*x" % (align, backtrace))
                else:
                    if firstrow:
                        firstrow = False
                    else:
                        if not detail:
                            gdb.write(" " * 6)
                        gdb.write(" " * (6 + 12 + 12 + align))
                    gdb.write(
                        "  [%0#*x] %-20s %s:%d\n"
                        % (
                            align,
                            backtrace,
                            node["backtrace"][x].format_string(
                                raw=False, symbols=True, address=False
                            ),
                            gdb.find_pc_line(backtrace).symtab,
                            gdb.find_pc_line(backtrace).line,
                        )
                    )

    else:
        charnode = gdb.Value(node).cast(gdb.lookup_type("char").pointer())
        gdb.write(
            "%12u%#*x"
            % (
                mm_nodesize(node["size"]),
                align,
                (int)(charnode + gdb.lookup_type("struct mm_allocnode_s").sizeof),
            )
        )

    gdb.write("\n")


def mempool_multiple_foreach(mpool):
    """Iterate over all pools in a mempool, yielding each pool"""
    i = 0
    while i < mpool["npools"]:
        pool = mpool["pools"] + i
        yield pool
        i += 1


def mempool_realblocksize(pool):
    """Return the real block size of a mempool"""

    if get_symbol_value("CONFIG_MM_DFAULT_ALIGNMENT") is None:
        mempool_align = 2 * gdb.lookup_type("size_t").sizeof
    else:
        mempool_align = get_symbol_value("CONFIG_MM_DFAULT_ALIGNMENT")

    if mempool_align == 0:
        mempool_align = 2 * gdb.lookup_type("size_t").sizeof

    if get_symbol_value("CONFIG_MM_BACKTRACE") >= 0:
        return align_up(
            pool["blocksize"] + gdb.lookup_type("struct mempool_backtrace_s").sizeof,
            mempool_align,
        )
    else:
        return pool["blocksize"]


def get_backtrace(node):

    backtrace_list = []
    max = node["backtrace"].type.range()[1]
    for x in range(0, max):
        if node["backtrace"][x] != 0:
            backtrace_list.append(int(node["backtrace"][x]))
        else:
            break

    return tuple(backtrace_list)


def record_backtrace(node, size, backtrace_dict):
    if node.type.has_key("backtrace"):
        backtrace = get_backtrace(node)
        if (backtrace, int(node["pid"])) not in backtrace_dict.keys():
            info = {}
            info["node"] = node
            info["count"] = 1
            info["size"] = size
            info["pid"] = node["pid"]
            backtrace_dict[(backtrace, int(node["pid"]))] = info
        else:
            backtrace_dict[(backtrace, int(node["pid"]))]["count"] += 1

    return backtrace_dict


def get_count(element):
    return element["count"]


def mempool_foreach(pool):
    """Iterate over all block in a mempool"""

    blocksize = mempool_realblocksize(pool)
    if pool["ibase"] != 0:
        nblk = pool["interruptsize"] / blocksize
        while nblk > 0:
            bufaddr = gdb.Value(pool["ibase"] + nblk * blocksize + pool["blocksize"])
            buf = bufaddr.cast(gdb.lookup_type("struct mempool_backtrace_s").pointer())
            yield buf
            nblk -= 1

    entry = sq_queue.get_type().pointer()
    for entry in sq_for_every(pool["equeue"], entry):
        nblk = (pool["expandsize"] - gdb.lookup_type("sq_entry_t").sizeof) / blocksize
        base = (
            gdb.Value(entry).cast(gdb.lookup_type("char").pointer()) - nblk * blocksize
        )
        while nblk > 0:
            nblk -= 1
            bufaddr = gdb.Value(base + nblk * blocksize + pool["blocksize"])
            buf = bufaddr.cast(gdb.lookup_type("struct mempool_backtrace_s").pointer())
            yield buf


def mempool_dumpbuf(buf, blksize, count, align, simple, detail, alive):
    charnode = gdb.Value(buf).cast(gdb.lookup_type("char").pointer())

    if not alive:
        # if pid is not alive put a red asterisk.
        gdb.write("\x1b[33;1m*\x1b[m")

    if not detail:
        gdb.write("%*d" % (6 if alive else 5, count))

    gdb.write(
        "%6d%12u%12u%#*x"
        % (
            buf["pid"],
            blksize,
            buf["seqno"],
            align,
            (int)(charnode - blksize),
        )
    )

    if buf.type.has_key("backtrace"):
        max = buf["backtrace"].type.range()[1]
        firstrow = True
        for x in range(0, max):
            backtrace = int(buf["backtrace"][x])
            if backtrace == 0:
                break

            if simple:
                gdb.write(" %0#*x" % (align, backtrace))
            else:
                if firstrow:
                    firstrow = False
                else:
                    if not detail:
                        gdb.write(" " * 6)
                    gdb.write(" " * (6 + 12 + 12 + align))
                gdb.write(
                    "  [%0#*x] %-20s %s:%d\n"
                    % (
                        align,
                        backtrace,
                        buf["backtrace"][x].format_string(
                            raw=False, symbols=True, address=False
                        ),
                        gdb.find_pc_line(backtrace).symtab,
                        gdb.find_pc_line(backtrace).line,
                    )
                )

    gdb.write("\n")


class Nxmemdump(gdb.Command):
    """Dump the heap and mempool memory"""

    def __init__(self):
        super(Nxmemdump, self).__init__("memdump", gdb.COMMAND_USER)

    def check_alive(self, pid):
        return self.pidhash[pid & self.npidhash - 1] != 0

    def mempool_dump(self, mpool, pid, seqmin, seqmax, address, simple, detail):
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
                for buf in mempool_foreach(pool):
                    if (
                        (pid == buf["pid"] or pid == PID_MM_ALLOC)
                        and (buf["seqno"] >= seqmin and buf["seqno"] < seqmax)
                        and buf["magic"] == MEMPOOL_MAGIC_ALLOC
                    ):
                        charnode = gdb.Value(buf).cast(
                            gdb.lookup_type("char").pointer()
                        )
                        if detail:
                            mempool_dumpbuf(
                                buf,
                                pool["blocksize"],
                                1,
                                self.align,
                                simple,
                                detail,
                                self.check_alive(buf["pid"]),
                            )
                        else:
                            self.backtrace_dict = record_backtrace(
                                buf, pool["blocksize"], self.backtrace_dict
                            )
                        if address and (
                            address < int(charnode)
                            and address >= (int)(charnode - pool["blocksize"])
                        ):

                            mempool_dumpbuf(
                                buf,
                                pool["blocksize"],
                                1,
                                self.align,
                                simple,
                                detail,
                                self.check_alive(buf["pid"]),
                            )
                            gdb.write(
                                "\nThe address 0x%x found belongs to"
                                "the mempool node with base address 0x%x\n"
                                % (address, charnode)
                            )
                            print_node = "p *(struct mempool_backtrace_s *)0x%x" % (
                                charnode
                            )
                            gdb.write(print_node + "\n")
                            gdb.execute(print_node)
                            return True
                        self.aordblks += 1
                        self.uordblks += pool["blocksize"]
        return False

    def memdump(self, pid, seqmin, seqmax, address, simple, detail):
        """Dump the heap memory"""
        if pid >= PID_MM_ALLOC:
            gdb.write(
                "Dump all used memory node info, use '\x1b[33;1m*\x1b[m' mark pid is not exist:\n"
            )
            if not detail:
                gdb.write("%6s" % ("CNT"))

            gdb.write(
                "%6s%12s%12s%*s %s\n"
                % ("PID", "Size", "Sequence", self.align, "Address", "Callstack")
            )
        else:
            gdb.write("Dump all free memory node info:\n")
            gdb.write("%12s%*s\n" % ("Size", self.align, "Address"))

        heap = gdb.parse_and_eval("g_mmheap")
        if heap.type.has_key("mm_mpool"):
            if self.mempool_dump(
                heap["mm_mpool"], pid, seqmin, seqmax, address, simple, detail
            ):
                return

        for node in mm_foreach(heap):
            if node["size"] & MM_ALLOC_BIT != 0:
                if (
                    pid == node["pid"]
                    or (pid == PID_MM_ALLOC and node["pid"] != PID_MM_MEMPOOL)
                ) and (node["seqno"] >= seqmin and node["seqno"] < seqmax):
                    if detail:
                        mm_dumpnode(
                            node,
                            1,
                            self.align,
                            simple,
                            detail,
                            self.check_alive(node["pid"]),
                        )
                    else:
                        self.backtrace_dict = record_backtrace(
                            node, mm_nodesize(node["size"]), self.backtrace_dict
                        )

                    charnode = gdb.Value(node).cast(gdb.lookup_type("char").pointer())
                    if address and (
                        address < int(charnode + node["size"])
                        and address
                        >= (int)(
                            charnode + gdb.lookup_type("struct mm_allocnode_s").sizeof
                        )
                    ):
                        mm_dumpnode(
                            node,
                            1,
                            self.align,
                            simple,
                            detail,
                            self.check_alive(node["pid"]),
                        )
                        gdb.write(
                            "\nThe address 0x%x found belongs to"
                            "the memory node with base address 0x%x\n"
                            % (address, charnode)
                        )
                        print_node = "p *(struct mm_allocnode_s *)0x%x" % (charnode)
                        gdb.write(print_node + "\n")
                        gdb.execute(print_node)
                        return
                    self.aordblks += 1
                    self.uordblks += mm_nodesize(node["size"])
            else:
                if pid == PID_MM_FREE:
                    mm_dumpnode(
                        node,
                        1,
                        self.align,
                        simple,
                        detail,
                        self.check_alive(node["pid"]),
                    )
                    self.aordblks += 1
                    self.uordblks += mm_nodesize(node["size"])

        if not detail:
            output = []
            for node in self.backtrace_dict.values():
                output.append(node)

            output.sort(key=get_count, reverse=True)
            for node in output:
                if (
                    node["node"].type
                    == gdb.lookup_type("struct mm_allocnode_s").pointer()
                ):
                    mm_dumpnode(
                        node["node"],
                        node["count"],
                        self.align,
                        simple,
                        detail,
                        self.check_alive(node["pid"]),
                    )
                else:
                    mempool_dumpbuf(
                        node["node"],
                        node["size"],
                        node["count"],
                        self.align,
                        simple,
                        detail,
                        self.check_alive(node["pid"]),
                    )

        gdb.write("%12s%12s\n" % ("Total Blks", "Total Size"))
        gdb.write("%12d%12d\n" % (self.aordblks, self.uordblks))

    def complete(self, text, word):
        return gdb.COMPLETE_SYMBOL

    def parse_arguments(self, argv):
        parser = argparse.ArgumentParser(description="memdump command")
        parser.add_argument("-p", "--pid", type=str, help="Thread PID")
        parser.add_argument("-a", "--addr", type=str, help="Query memory address")
        parser.add_argument("-i", "--min", type=str, help="Minimum value")
        parser.add_argument("-x", "--max", type=str, help="Maximum value")
        parser.add_argument("--used", action="store_true", help="Used flag")
        parser.add_argument("--free", action="store_true", help="Free flag")
        parser.add_argument(
            "-d",
            "--detail",
            action="store_true",
            help="Output details of each node",
            default=False,
        )
        parser.add_argument(
            "-s",
            "--simple",
            action="store_true",
            help="Simplified Output",
            default=False,
        )

        if argv[0] == "":
            argv = None
        try:
            args = parser.parse_args(argv)
        except SystemExit:
            return None

        return {
            "pid": int(args.pid, 0) if args.pid else None,
            "seqmin": int(args.min, 0) if args.min else 0,
            "seqmax": int(args.max, 0) if args.max else 0xFFFFFFFF,
            "used": args.used,
            "free": args.free,
            "addr": int(args.addr, 0) if args.addr else None,
            "simple": args.simple,
            "detail": args.detail,
        }

    def invoke(self, args, from_tty):
        if gdb.lookup_type("size_t").sizeof == 4:
            self.align = 11
        else:
            self.align = 19

        arg = self.parse_arguments(args.split(" "))

        if arg is None:
            return

        pid = PID_MM_ALLOC
        if arg["used"]:
            pid = PID_MM_ALLOC
        elif arg["free"]:
            pid = PID_MM_FREE
        elif arg["pid"]:
            pid = arg["pid"]
        if get_symbol_value("CONFIG_MM_BACKTRACE") <= 0:
            arg["detail"] = True

        self.aordblks = 0
        self.uordblks = 0
        self.backtrace_dict = {}
        self.npidhash = gdb.parse_and_eval("g_npidhash")
        self.pidhash = gdb.parse_and_eval("g_pidhash")
        self.memdump(
            pid, arg["seqmin"], arg["seqmax"], arg["addr"], arg["simple"], arg["detail"]
        )


Nxmemdump()


class Memleak(gdb.Command):
    """Memleak check"""

    def __init__(self):
        super(Memleak, self).__init__("memleak", gdb.COMMAND_USER)

    def check_alive(self, pid):
        return self.pidhash[pid & self.npidhash - 1] != 0

    def next_ptr(self):

        inf = gdb.inferiors()[0]
        heap = gdb.parse_and_eval("g_mmheap")
        start = []
        end = []
        longsize = get_long_type().sizeof
        region = get_symbol_value("CONFIG_MM_REGIONS")

        for i in range(0, region):
            start.append(int(gdb.Value(heap["mm_heapstart"][i])))
            end.append(int(gdb.Value(heap["mm_heapend"][i])))

        # Serach in global variables
        sdata = gdb.parse_and_eval("(uintptr_t)&_sdata")
        ebss = gdb.parse_and_eval("(uintptr_t)&_ebss")
        gdb.write(f"Searching in global variables {hex(sdata)} ~ {hex(ebss)}\n")
        mem = inf.read_memory(int(sdata), int(ebss) - int(sdata))
        i = 0
        while i < int(ebss) - int(sdata):
            ptr = read_ulong(mem[i : i + longsize].tobytes(), 0)
            for j in range(0, region):
                if ptr >= start[j] and ptr < end[j]:
                    yield ptr
                    break

            i = i + longsize

        gdb.write("Searching in grey memory\n")
        for node in self.grey_list:
            addr = node["addr"]
            mem = inf.read_memory(addr, node["size"])
            i = 0
            while i < node["size"]:
                ptr = read_ulong(mem[i : i + longsize].tobytes(), 0)
                for j in range(0, region):
                    if ptr >= start[j] and ptr < end[j]:
                        yield ptr
                        break
                i = i + longsize

    def collect_white_dict(self):
        white_dict = {}
        allocnode_size = gdb.lookup_type("struct mm_allocnode_s").sizeof

        # collect all user malloc ptr

        heap = gdb.parse_and_eval("g_mmheap")
        for node in mm_foreach(heap):
            if node["size"] & MM_ALLOC_BIT != 0 and node["pid"] != PID_MM_MEMPOOL:
                addr = (
                    gdb.Value(node).cast(gdb.lookup_type("char").pointer())
                    + allocnode_size
                )

                node_dict = {}
                node_dict["node"] = node
                node_dict["size"] = mm_nodesize(node["size"]) - allocnode_size
                node_dict["addr"] = addr
                white_dict[int(addr)] = node_dict

        if heap.type.has_key("mm_mpool"):
            for pool in mempool_multiple_foreach(heap["mm_mpool"]):
                for buf in mempool_foreach(pool):
                    if buf["magic"] == MEMPOOL_MAGIC_ALLOC:
                        addr = (
                            gdb.Value(buf).cast(gdb.lookup_type("char").pointer())
                            - pool["blocksize"]
                        )

                        buf_dict = {}
                        buf_dict["node"] = buf
                        buf_dict["size"] = pool["blocksize"]
                        buf_dict["addr"] = addr
                        white_dict[int(addr)] = buf_dict

        return white_dict

    def parse_arguments(self, argv):
        parser = argparse.ArgumentParser(description="memleak command")
        parser.add_argument(
            "-s",
            "--simple",
            action="store_true",
            help="Simplified Output",
            default=False,
        )
        parser.add_argument(
            "-d",
            "--detail",
            action="store_true",
            help="Output details of each node",
            default=False,
        )

        if argv[0] == "":
            argv = None
        try:
            args = parser.parse_args(argv)
        except SystemExit:
            return None

        return {"simple": args.simple, "detail": args.detail}

    def invoke(self, args, from_tty):
        if gdb.lookup_type("size_t").sizeof == 4:
            align = 11
        else:
            align = 19

        arg = self.parse_arguments(args.split(" "))

        if arg is None:
            return

        if get_symbol_value("CONFIG_MM_BACKTRACE") <= 0:
            gdb.write("Better use CONFIG_MM_BACKTRACE=16 or 8 get more information\n")

        white_dict = self.collect_white_dict()

        self.grey_list = []
        gdb.write("Searching for leaked memory, please wait a moment\n")
        start = time.time()

        sorted_keys = sorted(white_dict.keys())

        for ptr in self.next_ptr():
            # Find a closest addres in white_dict
            pos = bisect.bisect_right(sorted_keys, ptr)
            if pos == 0:
                continue
            grey_key = sorted_keys[pos - 1]
            if (
                grey_key in white_dict.keys()
                and ptr < grey_key + white_dict[grey_key]["size"]
            ):
                self.grey_list.append(white_dict[grey_key])
                del white_dict[grey_key]

        # All white node is leak

        end = time.time()
        gdb.write(f"Search all memory use {(end - start):.2f} seconds\n")

        gdb.write("\n")
        if len(white_dict) == 0:
            gdb.write("All node have references, no memory leak!\n")
            return

        gdb.write("Leak catch!, use '\x1b[33;1m*\x1b[m' mark pid is not exist:\n")

        if get_symbol_value("CONFIG_MM_BACKTRACE") > 0 and arg["detail"] is False:
            gdb.write("%6s" % ("CNT"))

        gdb.write(
            "%6s%12s%12s%*s %s\n"
            % ("PID", "Size", "Sequence", align, "Address", "Callstack")
        )

        self.npidhash = gdb.parse_and_eval("g_npidhash")
        self.pidhash = gdb.parse_and_eval("g_pidhash")

        if get_symbol_value("CONFIG_MM_BACKTRACE") > 0 and arg["detail"] is False:

            # Filter same backtrace

            backtrace_dict = {}
            for addr in white_dict.keys():
                backtrace_dict = record_backtrace(
                    white_dict[addr]["node"], white_dict[addr]["size"], backtrace_dict
                )

            leaksize = 0
            leaklist = []
            for node in backtrace_dict.values():
                leaklist.append(node)

            # sort by count
            leaklist.sort(key=get_count, reverse=True)

            i = 0
            for node in leaklist:
                if (
                    node["node"].type
                    == gdb.lookup_type("struct mm_allocnode_s").pointer()
                ):
                    mm_dumpnode(
                        node["node"],
                        node["count"],
                        align,
                        arg["simple"],
                        arg["detail"],
                        self.check_alive(node["pid"]),
                    )
                else:
                    mempool_dumpbuf(
                        node["node"],
                        node["size"],
                        node["count"],
                        align,
                        arg["simple"],
                        arg["detail"],
                        self.check_alive(node["pid"]),
                    )

                leaksize += node["count"] * node["size"]
                i += 1

            gdb.write(
                f"Alloc {len(white_dict)} count,\
have {i} some backtrace leak, total leak memory is {int(leaksize)} bytes\n"
            )
        else:
            leaksize = 0
            for node in white_dict.values():
                if (
                    node["node"].type
                    == gdb.lookup_type("struct mm_allocnode_s").pointer()
                ):
                    mm_dumpnode(
                        node["node"],
                        1,
                        align,
                        arg["simple"],
                        True,
                        self.check_alive(node["pid"]),
                    )
                else:
                    mempool_dumpbuf(
                        node["node"],
                        node["size"],
                        1,
                        align,
                        arg["simple"],
                        True,
                        self.check_alive(node["pid"]),
                    )
                leaksize += node["size"]

            gdb.write(
                f"Alloc {len(white_dict)} count, total leak memory is {int(leaksize)} bytes\n"
            )


Memleak()
