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


def dump_nodes(
    heaps: List[mm.MMHeap],
    pid=None,
    nodesize=None,
    used=None,
    free=None,
    seqmin=None,
    seqmax=None,
    orphan=None,
    no_heap=False,
    no_pool=False,
    no_pid=None,
) -> Generator[MMNodeDump, None, None]:
    def filter_node(node: MMNodeDump) -> bool:
        return (
            (pid is None or node.pid == pid)
            and (no_pid is None or node.pid != no_pid)
            and (nodesize is None or node.nodesize == nodesize)
            and (not used or not node.is_free)
            and (not free or node.is_free)
            and (seqmin is None or node.seqno >= seqmin)
            and (seqmax is None or node.seqno <= seqmax)
            and (not orphan or node.is_orphan)
        )

    if not no_heap:
        yield from (node for heap in heaps for node in filter(filter_node, heap.nodes))

    if not no_pool:
        yield from (
            blk
            for pool in mm.get_pools(heaps)
            for blk in filter(filter_node, pool.blks)
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


class MMDump(gdb.Command):
    """Dump memory manager heap"""

    def __init__(self):
        super().__init__("mm dump", gdb.COMMAND_USER)
        # define memdump as mm dump
        utils.alias("memdump", "mm dump")

    def find(self, heaps: List[mm.MMHeap], addr):
        """Find the node that contains the address"""
        # Search pools firstly.
        for pool in mm.get_pools(heaps):
            if blk := pool.find(addr):
                return blk

        # Search heaps
        for heap in heaps:
            if node := heap.find(addr):
                return node

    def parse_args(self, arg):
        parser = argparse.ArgumentParser(description=self.__doc__)
        parser.add_argument(
            "-a",
            "--address",
            type=str,
            default=None,
            help="The address to inspect",
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

    def invoke(self, arg: str, from_tty: bool) -> None:
        if not (args := self.parse_args(arg)):
            return

        heaps = (
            [mm.MMHeap(gdb.parse_and_eval(args.heap))] if args.heap else mm.get_heaps()
        )
        pids = [int(tcb["pid"]) for tcb in utils.get_tcbs()]

        print_header()

        def printnode(node, count):
            print_node(node, node.pid in pids, count, no_backtrace=args.no_backtrace)

        if args.address:
            addr = int(gdb.parse_and_eval(args.address))
            # Address specified, find and return directly.
            node = None
            for pool in mm.get_pools(heaps):
                if node := pool.find(addr):
                    break

            if node or (node := self.find(heaps, addr)):
                printnode(node, 1)
                source = "Pool" if node.from_pool else "Heap"
                print(f"{addr: #x} found belongs to {source} {node}")
                if node.prevnode:
                    print(f"prevnode: {node.prevnode}")
                if node.nextnode:
                    print(f"nextnode: {node.nextnode}")
            else:
                print(f"Address {addr:#x} not found in any heap")
            return

        filters = {
            "pid": args.pid,
            "nodesize": args.size,
            "used": not args.free,
            "free": args.free,
            "seqmin": args.min,
            "seqmax": args.max,
            "orphan": args.orphan,
        }

        heap_nodes = dump_nodes(heaps, **filters, no_heap=args.no_heap, no_pool=True)
        pool_nodes = dump_nodes(heaps, **filters, no_heap=True, no_pool=args.no_pool)

        if args.biggest:
            # Find the biggest nodes, only applicable to heaps
            nodes = sorted(
                heap_nodes,
                key=lambda node: node.nodesize,
                reverse=True,
            )
            for node in nodes[: args.top]:
                print(f"node@{node.address}: {node}")
            return

        sort_method = {
            "count": lambda node: 1,
            "size": lambda node: node.nodesize,
            "nodesize": lambda node: node.nodesize,
            "seq": lambda node: node.seqno,
            "address": lambda node: node.address,
        }

        def sort_nodes(nodes):
            nodes = sorted(nodes, key=sort_method[args.sort], reverse=True)
            if args.top is not None:
                nodes = nodes[: args.top] if args.top > 0 else nodes[args.top :]
            return nodes

        if args.no_group:
            # Print nodes without grouping
            nodes = list(heap_nodes)
            nodes.extend(pool_nodes)

            for node in sort_nodes(nodes):
                printnode(node, 1)

            gdb.write(f"Total blks: {len(nodes)}\n")
            return

        # Finally group the nodes and then print

        grouped: Dict[MMNodeDump, MMNodeDump] = defaultdict(list)
        grouped = group_nodes(heap_nodes)
        grouped = group_nodes(pool_nodes, grouped)

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
