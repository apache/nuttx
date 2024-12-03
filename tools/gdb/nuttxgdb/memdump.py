############################################################################
# tools/gdb/nuttxgdb/memdump.py
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

import argparse
import re
from collections import defaultdict
from typing import Dict, Generator, List, Protocol, Tuple

import gdb

from . import mm, utils


class MMNodeDump(Protocol):
    """Node information protocol for dump"""

    address: int  # Note that address should be in type of int
    nodesize: int
    seqno: int
    pid: int
    backtrace: Tuple[int]
    is_free: bool
    from_pool: bool
    overhead: int

    def contains(self, addr: int) -> bool: ...

    def read_memory(self) -> memoryview: ...


def filter_node(
    pid=None,
    nodesize=None,
    used=None,
    free=None,
    seqmin=None,
    seqmax=None,
    orphan=None,
    no_pid=None,
    no_heap=None,
    no_pool=None,
) -> bool:
    return lambda node: (
        (pid is None or node.pid == pid)
        and (no_pid is None or node.pid != no_pid)
        and (nodesize is None or node.nodesize == nodesize)
        and (not used or not node.is_free)
        and (not free or node.is_free)
        and (seqmin is None or node.seqno >= seqmin)
        and (seqmax is None or node.seqno <= seqmax)
        and (not orphan or node.is_orphan)
    )


def dump_nodes(
    heaps: List[mm.MMHeap],
    filters=None,
) -> Generator[MMNodeDump, None, None]:
    no_heap = filters and filters.get("no_heap")
    no_pool = filters and filters.get("no_pool")

    if not no_heap:
        yield from (
            node
            for heap in heaps
            for node in filter(filter_node(**filters), heap.nodes)
        )

    if not no_pool:
        yield from (
            blk
            for pool in mm.get_pools(heaps)
            for blk in filter(filter_node(**filters), pool.blks)
        )


def group_nodes(
    nodes: List[MMNodeDump], grouped: Dict[MMNodeDump, List[MMNodeDump]] = None
) -> Dict[MMNodeDump, List[MMNodeDump]]:
    grouped = grouped or defaultdict(list)
    for node in nodes:
        grouped[node].append(node)
    return grouped


def print_node(node: MMNodeDump, alive, count=1, formatter=None, no_backtrace=False):
    formatter = (
        formatter or "{:>1} {:>4} {:>12} {:>12} {:>12} {:>9} {:>14} {:>18} {:}\n"
    )
    gdb.write(
        formatter.format(
            "\x1b[33;1m*\x1b[m" if not alive else "",
            "P" if node.from_pool else "H",
            count,
            node.pid,
            node.nodesize,
            node.overhead,
            node.seqno,
            hex(node.address),
            "",
        )
    )

    if mm.CONFIG_MM_BACKTRACE and not no_backtrace:
        leading = formatter.format("", "", "", "", "", "", "", "", "")[:-1]
        btformat = leading + "{1:<48}{2}\n"
        if node.backtrace and node.backtrace[0]:
            gdb.write(f"{utils.Backtrace(node.backtrace, formatter=btformat)}\n")


def print_header(formatter=None):
    formatter = (
        formatter or "{:>1} {:>4} {:>12} {:>12} {:>12} {:>9} {:>14} {:>18} {:}\n"
    )
    head = (
        "",
        "Pool",
        "CNT",
        "PID",
        "Size",
        "Overhead",
        "Seqno",
        "Address",
        "Backtrace",
    )
    gdb.write(formatter.format(*head))


def get_heaps(args_heap=None) -> List[mm.MMHeap]:
    if args_heap:
        return [mm.MMHeap(gdb.parse_and_eval(args_heap))]
    return mm.get_heaps()


def parse_memdump_log(logfile, filters=None) -> Generator[MMNodeDump, None, None]:
    nodes = []

    class DumpNode(MMNodeDump):
        def __init__(self, address, nodesize, seqno, pid, backtrace, is_free, overhead):
            # C code dump the start address of the node, convert it the actual start address
            self.address = address - overhead
            self.nodesize = nodesize
            self.seqno = seqno
            self.pid = pid
            self.backtrace = backtrace
            self.overhead = overhead
            self.is_free = False
            self.from_pool = False

        def __repr__(self) -> str:
            return f"node@{self.address:#x}: size:{self.nodesize} seq:{self.seqno} pid:{self.pid} "

        def contains(self, addr: int) -> bool:
            return self.address <= addr < self.address + self.nodesize

        @property
        def prevnode(self):
            return next(
                (node for node in nodes if node.contains(self.address - 1)), None
            )

        @property
        def nextnode(self):
            address = self.address + self.nodesize  # address of the next node
            return next(
                (node for node in nodes if node.address == address),
                None,
            )

    with open(logfile) as f:
        for line in f:
            match = re.search(
                r"\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+((?:\s+0x[0-9a-fA-F]+)+)", line
            )
            if not match:
                continue

            try:
                pid = int(match.group(1))
                size = int(match.group(2))
                overhead = int(match.group(3))
                seq = int(match.group(4))
                addresses = match.group(5).split()
                addr = int(addresses[0], base=16)
                mem = tuple(int(addr, base=16) for addr in addresses[1:])
                nodes.append(DumpNode(addr, size, seq, pid, mem, False, overhead))
            except Exception as e:
                print(f"Error parsing line: {line}, {e}")

    return filter(filter_node(**filters), nodes) if filters else nodes


class MMDump(gdb.Command):
    """Dump memory manager heap"""

    def __init__(self):
        super().__init__("mm dump", gdb.COMMAND_USER)
        # define memdump as mm dump
        utils.alias("memdump", "mm dump")

    def parse_args(self, arg):
        parser = argparse.ArgumentParser(description=self.__doc__)
        parser.add_argument(
            "-a",
            "--address",
            type=str,
            default=None,
            help="Find the node that contains the address and exit",
        )

        parser.add_argument(
            "-l",
            "--log",
            type=str,
            default=None,
            help="Use the memdump log file generated by memdump command on device instead of live dump",
        )

        parser.add_argument(
            "--heap",
            type=str,
            default=None,
            help="Which heap to inspect",
        )

        parser.add_argument(
            "-p", "--pid", type=int, default=None, help="Thread PID, -1 for mempool"
        )
        parser.add_argument(
            "-i", "--min", type=int, default=None, help="Minimum sequence number"
        )
        parser.add_argument(
            "-x", "--max", type=int, default=None, help="Maximum sequence number"
        )
        parser.add_argument("--free", action="store_true", help="Free flag")
        parser.add_argument("--biggest", action="store_true", help="biggest allocated")
        parser.add_argument(
            "--orphan", action="store_true", help="Filter nodes that are orphan"
        )
        parser.add_argument(
            "--top", type=int, default=None, help="biggest top n, default to all"
        )
        parser.add_argument(
            "--size", type=int, default=None, help="Node block size filter."
        )
        parser.add_argument(
            "--no-pool",
            "--nop",
            action="store_true",
            help="Exclude dump from memory pool",
        )
        parser.add_argument(
            "--no-heap", "--noh", action="store_true", help="Exclude heap dump"
        )
        parser.add_argument(
            "--no-group", "--nog", action="store_true", help="Do not group the nodes"
        )
        parser.add_argument(
            "--no-backtrace",
            "--nob",
            action="store_true",
            help="Do not print backtrace",
        )
        parser.add_argument(
            "--no-reverse",
            "--nor",
            action="store_true",
            help="Do not reverse the sort result",
        )
        parser.add_argument(
            "--no-pid", type=int, default=None, help="Exclude nodes from this PID"
        )

        # add option to sort the node by size or count
        parser.add_argument(
            "--sort",
            type=str,
            choices=["size", "nodesize", "count", "seq", "address"],
            default="count",
            help="sort the node by size(nodesize * count), nodesize,  count or sequence number",
        )

        try:
            return parser.parse_args(gdb.string_to_argv(arg))
        except SystemExit:
            return

    def find_address(self, addr, heap=None, log=None):
        """Find the node that contains the address from memdump log or live dump."""
        addr = int(gdb.parse_and_eval(addr))
        if log:
            nodes = parse_memdump_log(log)
            node = next((node for node in nodes if node.contains(addr)), None)
        else:
            heaps = [mm.MMHeap(gdb.parse_and_eval(heap))] if heap else mm.get_heaps()

            # Find pool firstly
            node = next(
                (blk for pool in mm.get_pools(heaps) if (blk := pool.find(addr))), None
            )

            # Try heap if not found in pool
            node = node or next(
                (node for heap in heaps if (node := heap.find(addr))), None
            )

        return addr, node

    def collect_nodes(self, heap, log=None, filters=None):
        if log:
            nodes = parse_memdump_log(log, filters=filters)
        else:
            heaps = [mm.MMHeap(gdb.parse_and_eval(heap))] if heap else mm.get_heaps()
            nodes = dump_nodes(heaps, filters)

        return nodes

    def invoke(self, arg: str, from_tty: bool) -> None:
        if not (args := self.parse_args(arg)):
            return

        print_header()

        pids = [int(tcb["pid"]) for tcb in utils.get_tcbs()]

        def printnode(node, count):
            print_node(node, node.pid in pids, count, no_backtrace=args.no_backtrace)

        # Find the node by address, find directly and then quit
        if args.address:
            addr, node = self.find_address(args.address, args.heap, args.log)
            if not node:
                print(f"Address {addr:#x} not found in any heap")
            else:
                source = "Pool" if node.from_pool else "Heap"
                printnode(node, 1)
                print(f"{addr: #x} found belongs to {source} - {node}")

                if node.prevnode:
                    print(f"prevnode: {node.prevnode}")
                if node.nextnode:
                    print(f"nextnode: {node.nextnode}")
            return

        filters = {
            "pid": args.pid,
            "nodesize": args.size,
            "used": not args.free,
            "free": args.free,
            "seqmin": args.min,
            "seqmax": args.max,
            "orphan": args.orphan,
            "no_heap": args.no_heap,
            "no_pool": args.no_pool,
            "no_pid": args.no_pid,
        }

        nodes = self.collect_nodes(args.heap, log=args.log, filters=filters)

        sort_method = {
            "count": lambda node: 1,
            "size": lambda node: node.nodesize,
            "nodesize": lambda node: node.nodesize,
            "seq": lambda node: node.seqno,
            "address": lambda node: node.address,
        }

        def sort_nodes(nodes, sort=None):
            sort = sort or args.sort
            nodes = sorted(nodes, key=sort_method[sort], reverse=not args.no_reverse)
            if args.top is not None:
                nodes = nodes[: args.top] if args.top > 0 else nodes[args.top :]
            return nodes

        if args.biggest:
            # Dump the biggest node is same as sort by nodesize and do not group them
            args.sort = "nodesize"
            args.no_group = True

        if args.no_group:
            # Print nodes without grouping
            nodes = list(nodes)

            for node in sort_nodes(nodes):
                printnode(node, 1)

            gdb.write(f"Total blks: {len(nodes)}\n")
        else:
            # Group the nodes and then print

            grouped: Dict[MMNodeDump, MMNodeDump] = defaultdict(list)
            grouped = group_nodes(nodes)

            # Replace the count and size to count grouped nodes
            sort_method["count"] = lambda node: len(grouped[node])
            sort_method["size"] = lambda node: node.nodesize * len(grouped[node])
            total_blk = total_size = 0
            for node in sort_nodes(grouped.keys()):
                count = len(grouped[node])
                total_blk += count
                if node.pid != mm.PID_MM_MEMPOOL:
                    total_size += count * node.nodesize
                printnode(node, count)

            print(f"Total {total_blk} blks, {total_size} bytes")


class MMfrag(gdb.Command):
    """Show memory fragmentation rate"""

    def __init__(self):
        super().__init__("mm frag", gdb.COMMAND_USER)
        utils.alias("memfrag", "mm frag")

    def invoke(self, args, from_tty):
        parser = argparse.ArgumentParser(description=self.__doc__)
        parser.add_argument(
            "--heap",
            type=str,
            default=None,
            help="Which heap to inspect",
        )

        try:
            args = parser.parse_args(gdb.string_to_argv(args))
        except SystemExit:
            return None

        for heap in get_heaps(args.heap):
            nodes = list(
                sorted(heap.nodes_free(), key=lambda node: node.nodesize, reverse=True)
            )
            if not nodes:
                gdb.write(f"{heap}: no free nodes\n")
                continue

            freesize = sum(node.nodesize for node in nodes)
            remaining = freesize
            fragrate = 0

            for node in nodes:
                fragrate += (1 - (node.nodesize / remaining)) * (
                    node.nodesize / freesize
                )
                remaining -= node.nodesize

            fragrate = fragrate * 1000
            gdb.write(
                f"{heap.name}@{heap.address:#x}, fragmentation rate:{fragrate:.2f},"
                f" heapsize: {heap.heapsize}, free size: {freesize},"
                f" free count: {len(nodes)}, largest: {nodes[0].nodesize}\n"
            )


class MMMap(gdb.Command):
    """Generate memory map image to visualize memory layout"""

    def __init__(self):
        self.np = utils.import_check("numpy", errmsg="Please pip install numpy\n")
        self.plt = utils.import_check(
            "matplotlib", "pyplot", errmsg="Please pip install matplotlib\n"
        )
        self.math = utils.import_check("math")
        if not self.np or not self.plt or not self.math:
            return

        super().__init__("mm map", gdb.COMMAND_USER)
        utils.alias("memmap", "mm map")

    def save_memory_map(self, nodes: List[MMNodeDump], output_file):
        mallinfo = sorted(nodes, key=lambda node: node.address)
        start = mallinfo[0].address
        size = mallinfo[-1].address - start
        order = self.math.ceil(size**0.5)
        img = self.np.zeros([order, order])

        for node in mallinfo:
            addr = node.address
            size = node.nodesize
            start_index = addr - start
            end_index = start_index + size
            img.flat[start_index:end_index] = 1 + self.math.log2(node.seqno + 1)

        self.plt.imsave(output_file, img, cmap=self.plt.get_cmap("Greens"))

    def parse_arguments(self, argv):
        parser = argparse.ArgumentParser(description=self.__doc__)
        parser.add_argument(
            "-o", "--output", type=str, default=None, help="img output file"
        )
        parser.add_argument(
            "--heap", type=str, help="Which heap's pool to show", default=None
        )

        try:
            args = parser.parse_args(argv)
        except SystemExit:
            return None

        return args

    def invoke(self, args, from_tty):
        if not (args := self.parse_arguments(gdb.string_to_argv(args))):
            return

        for heap in get_heaps(args.heap):
            name = heap.name or f"heap@{heap.address:#x}"
            output = args.output or f"{name}.png"
            self.save_memory_map(heap.nodes_used(), output)
            gdb.write(f"Memory map saved to {output}\n")
