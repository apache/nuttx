############################################################################
# tools/pynuttx/nxgdb/memcheck.py
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
from collections import defaultdict
from typing import Dict, List, Tuple

import gdb

from . import memdump, mm, utils


class MMCheck(gdb.Command):
    """Check memory manager and pool integrity"""

    def __init__(self):
        super().__init__("mm check", gdb.COMMAND_USER)
        utils.alias("memcheck", "mm check")

    def check_heap(self, heap: mm.MMHeap) -> Dict[int, List[str]]:  # noqa: C901
        """Check heap integrity and return list of issues in string"""
        issues = defaultdict(list)  # key: address, value: list of issues

        def report(e, heap, node: mm.MMNode) -> None:
            gdb.write(f"Error happened during heap check: {e}\n")
            try:
                gdb.write(f" heap: {heap}\n")
                gdb.write(f"current node: {node}")
                if node.prevnode:
                    gdb.write(f" prev node: {node.prevnode}")
                if node.nextnode:
                    gdb.write(f" next node: {node.nextnode}")

                gdb.write("\n")
            except gdb.error as e:
                gdb.write(f"Error happened during report: {e}\n")

        def is_node_corrupted(node: mm.MMNode) -> Tuple[bool, str]:
            # Must be in this heap
            if not heap.contains(node.address):
                return True, f"node@{hex(node.address)} not in heap"

            # Check next node
            if node.nodesize > node.MM_SIZEOF_ALLOCNODE:
                nextnode = node.nextnode
                if not heap.contains(nextnode.address):
                    return True, f"nexnode@{hex(nextnode.address)} not in heap"
                if node.is_free:
                    if not nextnode.is_prev_free:
                        # This node is free, then next node must have prev free set
                        return (
                            True,
                            f"nextnode@{hex(nextnode.address)} not marked as prev free",
                        )

                    if nextnode.prevsize != node.nodesize:
                        return (
                            True,
                            f"nextnode @{hex(nextnode.address)} prevsize not match",
                        )

            if node.is_free:
                if node.nodesize < node.MM_MIN_CHUNK:
                    return True, f"nodesize {int(node.nodesize)} too small"

                if node.flink and node.flink.blink != node:
                    return (
                        True,
                        f"flink not intact: {hex(node.flink.blink)}, node: {hex(node.address)}",
                    )

                if node.blink.flink != node:
                    return (
                        True,
                        f"blink not intact: {hex(node.blink.flink)}, node: {hex(node.address)}",
                    )

                # Node should be in correctly sorted order
                if (blinksize := mm.MMNode(node.blink).nodesize) > node.nodesize:
                    return (
                        True,
                        f"blink node not in sorted order: {blinksize} > {node.nodesize}",
                    )

                fnode = mm.MMNode(node.flink) if node.flink else None
                if fnode and fnode.nodesize and fnode.nodesize < node.nodesize:
                    return (
                        True,
                        f"flink node not in sorted order: {fnode.nodesize} < {node.nodesize}",
                    )
            else:
                # Node is allocated.
                if node.nodesize < node.MM_SIZEOF_ALLOCNODE:
                    return True, f"nodesize {node.nodesize} too small"

            return False, ""

        try:
            # Check nodes in physical memory order
            for node in heap.nodes:
                corrupted, reason = is_node_corrupted(node)
                if corrupted:
                    issues[node.address].append(reason)

            # Check free list
            for node in utils.ArrayIterator(heap.mm_nodelist):
                # node is in type of gdb.Value, struct mm_freenode_s
                while node:
                    address = int(node.address)
                    if node["flink"] and not heap.contains(node["flink"]):
                        issues[address].append(
                            f"flink {hex(node['flink'])} not in heap"
                        )
                        break

                    if address in issues or node["size"] == 0:
                        # This node is already checked or size is 0, which is a node in node table
                        node = node["flink"]
                        continue

                    # Check if this node is corrupted
                    corrupted, reason = is_node_corrupted(mm.MMNode(node))
                    if corrupted:
                        issues[address].append(reason)
                        break

                    # Continue to it's flink
                    node = node["flink"]

        except Exception as e:
            report(e, heap, node)
            traceback.print_exc()

        return issues

    def dump_issues(self, heap, issues: Dict[int, List[str]]) -> None:
        for address, reasons in issues.items():
            gdb.write(
                f"{len(reasons)} issues @{hex(address)}: " f"{','.join(reasons)}\n"
            )

    def invoke(self, arg: str, from_tty: bool) -> None:
        try:
            heaps = memdump.get_heaps()
            for heap in heaps:
                issues = self.check_heap(heap)
                if not issues:
                    continue

                print(f"Found {len(issues)} issues in heap {heap}")
                self.dump_issues(heap, issues)
        except Exception as e:
            print(f"Error happened during check: {e}")
            traceback.print_exc()

        print("Check done.")
