############################################################################
# tools/gdb/nuttxgdb/protocols/mm.py
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

from typing import List

from .value import Value


class ProcfsMeminfoEntry(Value):
    """struct procfs_meminfo_entry_s"""

    name: Value
    heap: Value
    next: ProcfsMeminfoEntry


class MMAllocNode(Value):
    """struct mm_allocnode_s"""

    preceding: Value
    size: Value
    pid: Value
    seqno: Value
    backtrace: Value


class MMFreeNode(Value):
    """struct mm_freenode_s"""

    preceding: Value
    size: Value
    pid: Value
    seqno: Value
    backtrace: Value
    flink: MMFreeNode
    blink: MMFreeNode


class MMHeap(Value):
    """struct mm_heap_s"""

    mm_lock: Value
    mm_heapsize: Value
    mm_maxused: Value
    mm_curused: Value
    mm_heapstart: List[MMAllocNode]
    mm_heapend: List[MMAllocNode]
    mm_nregions: Value
    mm_nodelist: Value


class MemPool(Value):
    """struct mempool_s"""

    initialsize: Value
    interruptsize: Value
    expandsize: Value
    wait: Value
    priv: Value
    alloc: Value
    free: Value
    check: Value
    ibase: Value
    queue: Value
    iqueue: Value
    equeue: Value
    nalloc: Value
    lock: Value
    waitsem: Value
    procfs: Value


class MemPoolMultiple(Value):
    """struct mempool_multiple_s"""

    pools: List[MemPool]
    npools: Value
    expandsize: Value
    minpoolsize: Value
    arg: Value
    alloc: Value
    alloc_size: Value
    free: Value
    alloced: Value
    delta: Value
    lock: Value
    chunk_queue: Value
    chunk_size: Value
    dict_used: Value
    dict_col_num_log2: Value
    dict_row_num: Value
    dict: Value


class MemPoolBlock(Value):
    """struct mempool_backtrace_s"""

    magic: Value
    pid: Value
    seqno: Value
    backtrace: Value
