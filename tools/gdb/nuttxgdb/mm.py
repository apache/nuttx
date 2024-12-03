############################################################################
# tools/gdb/nuttxgdb/mm.py
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

from __future__ import annotations

import argparse
from typing import Generator, List, Tuple

import gdb

from . import lists, utils
from .protocols import mm as p
from .utils import Value

CONFIG_MM_BACKTRACE = utils.get_symbol_value("CONFIG_MM_BACKTRACE")
CONFIG_MM_BACKTRACE = -1 if CONFIG_MM_BACKTRACE is None else int(CONFIG_MM_BACKTRACE)


PID_MM_INVALID = -100
PID_MM_MEMPOOL = -1


class MemPoolBlock:
    """
    Memory pool block instance.
    """

    MAGIC_ALLOC = 0x5555_5555

    mempool_backtrace_s = utils.lookup_type("struct mempool_backtrace_s")

    def __init__(self, addr: int, blocksize: int, overhead: int) -> None:
        """
        Initialize the memory pool block instance.
        block: must be start address of the block,
        blocksize: block size without backtrace overhead,
        overhead: backtrace overhead size.
        """
        self.overhead = overhead
        self.from_pool = True
        self.is_orphan = False
        self.address = addr
        self.blocksize = int(blocksize)
        self.nodesize = int(blocksize) + self.overhead
        # Lazy evaluation
        self._backtrace = self._pid = self._seqno = self._magic = self._blk = None

    def __repr__(self) -> str:
        return f"block@{hex(self.address)},size:{self.blocksize},seqno:{self.seqno},pid:{self.pid}"

    def __str__(self) -> str:
        return self.__repr__()

    def __hash__(self) -> int:
        return hash((self.pid, self.nodesize, self.backtrace))

    def __eq__(self, value: MemPoolBlock) -> bool:
        return (
            self.pid == value.pid
            and self.nodesize == value.nodesize
            and self.backtrace == value.backtrace
        )

    def contains(self, address: int) -> bool:
        """Check if the address is in block's range, excluding overhead"""
        return self.address <= address < self.address + self.blocksize

    @property
    def blk(self) -> p.MemPoolBlock:
        if not self._blk:
            addr = int(self.address) + self.blocksize
            self._blk = (
                gdb.Value(addr).cast(self.mempool_backtrace_s.pointer()).dereference()
            )
        return self._blk

    @property
    def is_free(self) -> bool:
        if CONFIG_MM_BACKTRACE < 0:
            return False

        if not self._magic:
            self._magic = int(self.blk["magic"])

        return self._magic != self.MAGIC_ALLOC

    @property
    def seqno(self) -> int:
        if not self._seqno:
            self._seqno = int(self.blk["seqno"]) if CONFIG_MM_BACKTRACE >= 0 else -100
        return self._seqno

    @property
    def pid(self) -> int:
        if not self._pid:
            self._pid = (
                int(self.blk["pid"]) if CONFIG_MM_BACKTRACE >= 0 else PID_MM_INVALID
            )
        return self._pid

    @property
    def backtrace(self) -> Tuple[int]:
        if CONFIG_MM_BACKTRACE <= 0:
            return ()

        if not self._backtrace:
            self._backtrace = tuple(
                int(self.blk["backtrace"][i]) for i in range(CONFIG_MM_BACKTRACE)
            )
        return self._backtrace

    def read_memory(self) -> memoryview:
        return gdb.selected_inferior().read_memory(self.address, self.blocksize)


class MemPool(Value, p.MemPool):
    """
    Memory pool instance.
    """

    def __init__(self, mpool: Value, name=None) -> None:
        if mpool.type.code == gdb.TYPE_CODE_PTR:
            mpool = mpool.dereference()
        super().__init__(mpool)
        self._blksize = None
        self._nfree = None
        self._nifree = None
        self._overhead = None

    def __repr__(self) -> str:
        return f"{self.name}@{hex(self.address)},size:{self.size}/{self['blocksize']},nused:{self.nused},nfree:{self.nfree}"

    def __str__(self) -> str:
        return self.__repr__()

    @property
    def name(self) -> str:
        try:
            return self.procfs.name.string()
        except Exception:
            return "<noname>"

    @property
    def memranges(self) -> Generator[Tuple[int, int], None, None]:
        """Memory ranges of the pool"""
        sq_entry_t = utils.lookup_type("sq_entry_t")
        blksize = self.size

        if self.ibase:
            blks = int(self.interruptsize) // blksize
            base = int(self.ibase)
            yield (base, base + blks * blksize)

        if not self.equeue.head:
            return None

        # First queue has size of initialsize
        ninit = int(self.initialsize)
        ninit = ninit and (ninit - sq_entry_t.sizeof) // blksize
        nexpand = (int(self.expandsize) - sq_entry_t.sizeof) // blksize

        for entry in lists.NxSQueue(self.equeue):
            blks = ninit or nexpand
            ninit = 0
            yield (int(entry) - blks * blksize, int(entry))

    @property
    def size(self) -> int:
        """Real block size including backtrace overhead"""
        if not self._blksize:
            blksize = self["blocksize"]
            backtrace = utils.get_symbol_value("CONFIG_MM_BACKTRACE")
            if CONFIG_MM_BACKTRACE is not None and backtrace >= 0:
                mempool_backtrace_s = utils.lookup_type("struct mempool_backtrace_s")
                size_t = utils.lookup_type("size_t")
                align = (
                    utils.get_symbol_value("CONFIG_MM_DEFAULT_ALIGNMENT")
                    or 2 * size_t.sizeof
                )
                blksize = blksize + mempool_backtrace_s.sizeof
                blksize = (blksize + align - 1) & ~(align - 1)
            self._blksize = int(blksize)
        return self._blksize

    @property
    def overhead(self) -> int:
        if not self._overhead:
            self._overhead = self.size - int(self["blocksize"])
        return self._overhead

    @property
    def nwaiter(self) -> int:
        return -int(self.waitsem.semcount) if self.wait and self.expandsize == 0 else 0

    @property
    def nused(self) -> int:
        return int(self.nalloc)

    @property
    def free(self) -> int:
        return (self.nfree + self.nifree) * self.size

    @property
    def nfree(self) -> int:
        if not self._nfree:
            self._nfree = lists.sq_count(self.queue)
        return self._nfree + self.nifree

    @property
    def nifree(self) -> int:
        """Interrupt pool free blocks count"""
        if not self._nifree:
            self._nifree = lists.sq_count(self.iqueue)
        return self._nifree

    @property
    def total(self) -> int:
        nqueue = lists.sq_count(self.equeue)
        sq_entry_t = utils.lookup_type("sq_entry_t")
        blocks = self.nused + self.nfree
        return int(nqueue * sq_entry_t.sizeof + blocks * self.size)

    @property
    def blks(self) -> Generator[MemPoolBlock, None, None]:
        """Iterate over all blocks in the pool"""
        sq_entry_t = utils.lookup_type("sq_entry_t")
        blksize = self.size  # Real block size including backtrace overhead
        blocksize = self["blocksize"]

        def iterate(entry, nblocks):
            base = int(entry) - nblocks * blksize
            while nblocks > 0:
                yield MemPoolBlock(base, blocksize, self.overhead)
                base += blksize
                nblocks -= 1

        if self.ibase:
            blks = int(self.interruptsize) // blksize
            yield from iterate(self.ibase + blks * blksize, blks)

        if not self.equeue.head:
            return None

        # First queue has size of initialsize
        ninit = int(self.initialsize)
        ninit = ninit and (ninit - sq_entry_t.sizeof) // blksize
        nexpand = (int(self.expandsize) - sq_entry_t.sizeof) // blksize

        for entry in lists.NxSQueue(self.equeue):
            yield from iterate(entry, ninit or nexpand)
            ninit = 0

    def contains(self, address: int) -> Tuple[bool, Value]:
        ranges = self.memranges
        if not ranges:
            return False, None

        for start, end in ranges:
            if start <= address < end:
                return True, None

    def find(self, address: int) -> Value:
        """Find the block that contains the given address"""
        sq_entry_t = utils.lookup_type("sq_entry_t")
        blksize = self.size
        blocksize = self["blocksize"]

        def get_blk(base):
            blkstart = base + (address - base) // blksize * blksize
            return MemPoolBlock(blkstart, blocksize, self.overhead)

        if self.ibase:
            # Check if it belongs to interrupt pool
            blks = int(self.interruptsize) // blksize
            base = int(self.ibase)
            if base <= address < base + blks * blksize:
                return get_blk(base)

        if not self.equeue.head:
            return None

        # First queue has size of initialsize
        ninit = int(self.initialsize)
        ninit = ninit and (ninit - sq_entry_t.sizeof) // blksize
        nexpand = (int(self.expandsize) - sq_entry_t.sizeof) // blksize

        for entry in lists.NxSQueue(self.equeue):
            blks = ninit or nexpand
            ninit = 0
            base = int(entry) - blks * blksize
            if base <= address < int(entry):
                return get_blk(base)

    def blks_free(self) -> Generator[MemPoolBlock, None, None]:
        """Iterate over all free blocks in the pool"""
        blocksize = self["blocksize"]
        for entry in lists.NxSQueue(self.queue):
            yield MemPoolBlock(int(entry), blocksize, self.overhead)

    def blks_used(self) -> Generator[MemPoolBlock, None, None]:
        """Iterate over all used blocks in the pool"""
        return filter(lambda blk: not blk.is_free, self.blks)


class MemPoolMultiple(Value, p.MemPoolMultiple):
    """
    Multiple level memory pool instance.
    """

    def __init__(self, mpool: Value, name=None) -> None:
        if mpool.type.code == gdb.TYPE_CODE_PTR:
            mpool = mpool.dereference()
        super().__init__(mpool)

    def __repr__(self) -> str:
        return f"Multiple Level Memory Pool: {self.address}"

    def __str__(self) -> str:
        return self.__repr__()

    @property
    def pools(self) -> Generator[MemPool, None, None]:
        for pool in utils.ArrayIterator(self["pools"], self.npools):
            yield MemPool(pool)

    @property
    def free(self) -> int:
        return sum(pool.free for pool in self.pools)


class MMNode(gdb.Value, p.MMFreeNode):
    """
    One memory node in the memory manager heap, either free or allocated.
    The instance is always dereferenced to the actual node.
    """

    MM_ALLOC_BIT = 0x1
    MM_PREVFREE_BIT = 0x2
    MM_MASK_BIT = MM_ALLOC_BIT | MM_PREVFREE_BIT
    MM_SIZEOF_ALLOCNODE = utils.sizeof("struct mm_allocnode_s")
    MM_ALLOCNODE_OVERHEAD = MM_SIZEOF_ALLOCNODE - utils.sizeof("mmsize_t")
    MM_MIN_CHUNK = utils.get_symbol_value("MM_MIN_CHUNK", locspec="mm_initialize")

    def __init__(self, node: gdb.Value):
        if node.type.code == gdb.TYPE_CODE_PTR:
            node = node.dereference()
        self._backtrace = None
        self._address = None
        self._nodesize = None
        super().__init__(node)

    def __repr__(self):
        return (
            f"{hex(self.address)}({'F' if self.is_free else 'A'}{'F' if self.is_prev_free else 'A'})"
            f" size:{self.nodesize}/{self.prevsize if self.is_prev_free else '-'}"
            f" seq:{self.seqno} pid:{self.pid} "
        )

    def __str__(self) -> str:
        return self.__repr__()

    def __hash__(self) -> int:
        return hash((self.pid, self.nodesize, self.backtrace))

    def __eq__(self, value: MMNode) -> bool:
        return (
            self.pid == value.pid
            and self.nodesize == value.nodesize
            and self.backtrace == value.backtrace
        )

    def contains(self, address):
        """Check if the address is in node's range, excluding oeprhead"""
        return (
            self.address + self.overhead
            <= address
            < self.address + self.nodesize - MMNode.MM_ALLOCNODE_OVERHEAD
        )

    def read_memory(self):
        addr = int(self.address) + MMNode.MM_ALLOCNODE_OVERHEAD
        size = self.nodesize - MMNode.MM_ALLOCNODE_OVERHEAD
        return gdb.selected_inferior().read_memory(addr, size)

    @property
    def address(self) -> int:
        """Change 'void *' to int"""
        if not self._address:
            self._address = int(super().address)
        return self._address

    @property
    def prevsize(self) -> int:
        """Size of preceding chunk size"""
        return int(self["preceding"]) & ~MMNode.MM_MASK_BIT

    @property
    def nodesize(self) -> int:
        """Size of this chunk, including overhead"""
        if not self._nodesize:
            self._nodesize = int(self["size"]) & ~MMNode.MM_MASK_BIT
        return self._nodesize

    @property
    def usersize(self) -> int:
        """Size of this chunk, excluding overhead"""
        return self.nodesize - MMNode.MM_ALLOCNODE_OVERHEAD

    @property
    def flink(self):
        # Only free node has flink and blink
        return MMNode(self["flink"]) if self.is_free and self["flink"] else None

    @property
    def blink(self):
        # Only free node has flink and blink
        return MMNode(self["blink"]) if self.is_free and self["blink"] else None

    @property
    def pid(self) -> int:
        # Only available when CONFIG_MM_BACKTRACE >= 0
        if CONFIG_MM_BACKTRACE >= 0:
            return int(self["pid"])
        return PID_MM_INVALID

    @property
    def seqno(self) -> int:
        return int(self["seqno"]) if CONFIG_MM_BACKTRACE >= 0 else -1

    @property
    def backtrace(self) -> List[Tuple[int, str, str]]:
        if CONFIG_MM_BACKTRACE <= 0:
            return ()

        if not self._backtrace:
            self._backtrace = tuple(
                int(self["backtrace"][i]) for i in range(CONFIG_MM_BACKTRACE)
            )
        return self._backtrace

    @property
    def prevnode(self) -> MMNode:
        if not self.is_prev_free:
            return None

        addr = int(self.address) - self.prevsize
        type = utils.lookup_type("struct mm_freenode_s").pointer()
        return MMNode(gdb.Value(addr).cast(type))

    @property
    def nextnode(self) -> MMNode:
        if not self.nodesize:
            return None

        addr = int(self.address) + self.nodesize
        type = utils.lookup_type("struct mm_freenode_s").pointer()
        # Use gdb.Value for better performance
        return MMNode(gdb.Value(addr).cast(type))

    @property
    def is_free(self) -> bool:
        return not self["size"] & MMNode.MM_ALLOC_BIT

    @property
    def is_prev_free(self) -> bool:
        return self["size"] & MMNode.MM_PREVFREE_BIT

    @property
    def is_orphan(self) -> bool:
        # Report orphaned node and node likely to be orphaned(free-used-used-free)
        return self.is_prev_free or self.nextnode.is_free

    @property
    def from_pool(self) -> bool:
        return False

    @property
    def overhead(self) -> int:
        return MMNode.MM_ALLOCNODE_OVERHEAD


class MMHeap(Value, p.MMHeap):
    """
    One memory manager heap. It may contains multiple regions.
    """

    def __init__(self, heap: Value, name=None) -> None:
        if heap.type.code == gdb.TYPE_CODE_PTR:
            heap = heap.dereference()
        super().__init__(heap)

        self.name = name or "<noname>"
        self._regions = None

    def __repr__(self) -> str:
        regions = [
            f"{hex(start.address)}~{hex(end.address)}" for start, end in self.regions
        ]
        return f"{self.name}@{self.address}, {int(self.heapsize) / 1024 :.1f}kB {self.nregions}regions: {','.join(regions)}"

    def __str__(self) -> str:
        return self.__repr__()

    @property
    def curused(self) -> int:
        return int(self.mm_curused)

    @property
    def heapsize(self) -> int:
        return int(self.mm_heapsize)

    @property
    def free(self) -> int:
        return self.heapsize - self.curused

    @property
    def nregions(self) -> int:
        return int(utils.get_field(self, "mm_nregions", default=1))

    @property
    def mm_mpool(self) -> Value:
        return utils.get_field(self, "mm_mpool", default=None)

    @property
    def regions(self):
        if not self._regions:
            regions = self.nregions
            self._regions = []
            for start, end in zip(
                utils.ArrayIterator(self.mm_heapstart, regions),
                utils.ArrayIterator(self.mm_heapend, regions),
            ):
                self._regions.append((MMNode(start), MMNode(end)))
        return self._regions

    @property
    def nodes(self) -> Generator[MMNode, None, None]:
        for start, end in self.regions:
            node = start
            while node and node.address <= end.address:
                yield node
                node = node.nextnode

    def nodes_free(self) -> Generator[MMNode, None, None]:
        return filter(lambda node: node.is_free, self.nodes)

    def nodes_used(self) -> Generator[MMNode, None, None]:
        return filter(lambda node: not node.is_free, self.nodes)

    def contains(self, address: int) -> bool:
        ranges = [[int(start.address), int(end.address)] for start, end in self.regions]
        ranges[0][0] = int(self.address)  # The heap itself is also in the range
        return any(start <= address <= end for start, end in ranges)

    def find(self, address: int) -> MMNode:
        for node in self.nodes:
            if node.address <= address < node.address + node.nodesize:
                return node


def get_heaps() -> List[MMHeap]:
    # parse g_procfs_meminfo to get all heaps
    heaps = []
    meminfo: p.ProcfsMeminfoEntry = utils.gdb_eval_or_none("g_procfs_meminfo")
    if not meminfo and (heap := gdb.parse_and_eval("g_mmheap")):
        heaps.append(MMHeap(heap))

    while meminfo:
        heaps.append(MMHeap(meminfo.heap, name=meminfo.name.string()))
        meminfo = meminfo.next

    return heaps


def get_pools(heaps: List[Value] = []) -> Generator[MemPool, None, None]:
    for heap in heaps or get_heaps():
        if not (mm_pool := heap.mm_mpool):
            continue

        mpool = MemPoolMultiple(mm_pool)
        for pool in mpool.pools:
            yield pool


class MMHeapInfo(gdb.Command):
    """Show basic heap information"""

    def __init__(self):
        super().__init__("mm heap", gdb.COMMAND_USER)

    def invoke(self, arg: str, from_tty: bool) -> None:
        for heap in get_heaps():
            regions = [(start.address, end.address) for start, end in heap.regions]
            gdb.write(f"{heap} - has {len(list(heap.nodes))} nodes, regions:")
            gdb.write(" ".join(f"{hex(start)}~{hex(end)}" for start, end in regions))
            gdb.write("\n")


class MMPoolInfo(gdb.Command):
    """Show basic heap information"""

    def __init__(self):
        super().__init__("mm pool", gdb.COMMAND_USER)
        utils.alias("mempool", "mm pool")

    def invoke(self, arg: str, from_tty: bool) -> None:
        parser = argparse.ArgumentParser(description="Dump memory pool information.")
        parser.add_argument(
            "--heap", type=str, help="Which heap's pool to show", default=None
        )

        try:
            args = parser.parse_args(gdb.string_to_argv(arg))
        except SystemExit:
            return

        heaps = [gdb.parse_and_eval(args.heap)] if args.heap else get_heaps()
        if not (pools := list(get_pools(heaps))):
            gdb.write("No pools found.\n")
            return

        count = len(pools)
        gdb.write(f"Total {count} pools\n")

        name_max = max(len(pool.name) for pool in pools) + 11  # 11: "@0x12345678"
        formatter = (
            "{:>%d} {:>11} {:>9} {:>9} {:>9} {:>9} {:>9} {:>9} {:>9}\n" % name_max
        )
        head = (
            "",
            "total",
            "blocksize",
            "bsize",
            "overhead",
            "nused",
            "nfree",
            "nifree",
            "nwaiter",
        )

        gdb.write(formatter.format(*head))
        for pool in pools:
            gdb.write(
                formatter.format(
                    f"{pool.name}@{pool.address:#x}",
                    pool.total,
                    pool.blocksize,
                    pool.size,
                    pool.overhead,
                    pool.nused,
                    pool.nfree,
                    pool.nifree,
                    pool.nwaiter,
                )
            )
