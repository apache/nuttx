############################################################################
# tools/gdb/nuttxgdb/lists.py
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

import gdb

from . import utils

list_node_type = utils.lookup_type("struct list_node")
sq_queue_type = utils.lookup_type("sq_queue_t")
sq_entry_type = utils.lookup_type("sq_entry_t")
dq_queue_type = utils.lookup_type("dq_queue_t")


class NxList:
    def __init__(self, list, container_type=None, member=None, reverse=False):
        """Initialize the list iterator. Optionally specify the container type and member name."""

        if not list:
            raise ValueError("The head cannot be None.\n")

        if list.type.code != gdb.TYPE_CODE_PTR:
            list = list.address  # Make sure list is a pointer.

        if container_type and not member:
            raise ValueError("Must specify the member name in container.\n")

        self.list = list
        self.reverse = reverse
        self.container_type = container_type
        self.member = member
        self.current = self._get_first()

    def _get_first(self):
        """Get the initial node based on the direction of traversal."""

        prev = self.list["prev"]
        next = self.list["next"]

        first = prev if self.reverse else next
        return first if first and first != self.list else None

    def _get_next(self, node):
        #   for(node = (list)->next; node != (list); node = node->next)
        return node["next"] if node["next"] != self.list else None

    def _get_prev(self, node):
        #   for(node = (list)->next; node != (list); node = node->prev)
        return node["prev"] if node["prev"] != self.list else None

    def __iter__(self):
        return self

    def __next__(self):
        if self.current is None:
            raise StopIteration

        node = self.current
        self.current = self._get_prev(node) if self.reverse else self._get_next(node)
        return (
            utils.container_of(node, self.container_type, self.member)
            if self.container_type
            else node
        )


class NxSQueue(NxList):
    def __init__(self, list, container_type=None, member=None, reverse=False):
        """Initialize the singly linked list iterator. Optionally specify the container type and member name."""
        if reverse:
            raise ValueError(
                "Reverse iteration is not supported for singly linked lists.\n"
            )
        super().__init__(list, container_type, member, reverse)

    def _get_first(self):
        #   for ((p) = (q)->head; (p) != NULL; (p) = (p)->flink)
        return self.list["head"] or None

    def _get_next(self, node):
        # if not node["flink"], then return None, to indicate end of list
        return node["flink"] or None


class NxDQueue(NxList):
    def __init__(self, list, container_type=None, member=None, reverse=False):
        """Initialize the doubly linked list iterator. Optionally specify the container type and member name."""
        super().__init__(list, container_type, member, reverse)

    def _get_first(self):
        head = self.list["head"]
        tail = self.list["tail"]

        first = head if not self.reverse else tail
        return first or None

    def _get_next(self, node):
        #   for ((p) = (q)->head; (p) != NULL; (p) = (p)->flink)
        return node["flink"] or None

    def _get_prev(self, node):
        #   for ((p) = (q)->tail; (p) != NULL; (p) = (p)->blink)
        return node["blink"] or None


def list_check(head):
    """Check the consistency of a list"""
    nb = 0

    if head.type == list_node_type.pointer():
        head = head.dereference()
    elif head.type != list_node_type:
        raise gdb.GdbError("argument must be of type (struct list_node [*])")
    c = head
    try:
        gdb.write("Starting with: {}\n".format(c))
    except gdb.MemoryError:
        gdb.write("head is not accessible\n")
        return
    while True:
        p = c["prev"].dereference()
        n = c["next"].dereference()
        try:
            if p["next"] != c.address:
                gdb.write(
                    "prev.next != current: "
                    "current@{current_addr}={current} "
                    "prev@{p_addr}={p}\n".format(
                        current_addr=c.address,
                        current=c,
                        p_addr=p.address,
                        p=p,
                    )
                )
                return
        except gdb.MemoryError:
            gdb.write(
                "prev is not accessible: "
                "current@{current_addr}={current}\n".format(
                    current_addr=c.address, current=c
                )
            )
            return
        try:
            if n["prev"] != c.address:
                gdb.write(
                    "next.prev != current: "
                    "current@{current_addr}={current} "
                    "next@{n_addr}={n}\n".format(
                        current_addr=c.address,
                        current=c,
                        n_addr=n.address,
                        n=n,
                    )
                )
                return
        except gdb.MemoryError:
            gdb.write(
                "next is not accessible: "
                "current@{current_addr}={current}\n".format(
                    current_addr=c.address, current=c
                )
            )
            return
        c = n
        nb += 1
        if c == head:
            gdb.write("list is consistent: {} node(s)\n".format(nb))
            return


def sq_is_empty(sq):
    """Check if a singly linked list is empty"""
    if sq.type == sq_queue_type.pointer():
        sq = sq.dereference()
    elif sq.type != sq_queue_type:
        return False

    if sq["head"] == 0:
        return True
    else:
        return False


def sq_check(sq, verbose=True) -> int:
    """Check the consistency of a singly linked list"""
    nb = 0
    if sq.type == sq_queue_type.pointer():
        sq = sq.dereference()
    elif sq.type != sq_queue_type:
        gdb.write("Must be struct sq_queue not {}".format(sq.type))
        return nb

    if sq["head"] == 0:
        if verbose:
            gdb.write("sq_queue head is empty {}\n".format(sq.address))
        return nb

    nodes = set()
    entry = sq["head"].dereference()
    try:
        while entry.address:
            nb += 1
            if int(entry.address) in nodes:
                gdb.write("sq_queue is circular: {}\n".format(entry.address))
                return nb
            nodes.add(int(entry.address))
            entry = entry["flink"].dereference()
    except gdb.MemoryError:
        gdb.write("entry address is unaccessible {}\n".format(entry.address))
        return nb

    if int(sq["tail"]) not in nodes:
        gdb.write("sq_queue tail is not in the list {}\n".format(sq["tail"]))
        return nb
    if sq["tail"]["flink"] != 0:
        gdb.write("sq_queue tail->flink is not null {}\n".format(sq["tail"]))
        return nb

    if verbose:
        gdb.write("sq_queue is consistent: {} node(s)\n".format(nb))
    return nb


def sq_count(sq) -> int:
    """Count sq elements, abort if check failed"""
    return sq_check(sq, verbose=False)


def dq_for_every(dq, entry=None):
    """Iterate over a doubly linked list"""
    if dq.type == dq_queue_type.pointer():
        dq = dq.dereference()
    elif dq.type != dq_queue_type:
        gdb.write("Must be struct dq_queue not {}".format(dq.type))
        return

    if dq["head"] == 0:
        return

    if not entry:
        entry = dq["head"].dereference()

    while entry.address:
        yield entry.address
        entry = entry["flink"].dereference()


def dq_check(dq):
    """Check the consistency of a doubly linked list"""
    nb = 0
    if dq.type == dq_queue_type.pointer():
        dq = dq.dereference()
    elif dq.type != dq_queue_type:
        gdb.write("Must be struct dq_queue not {}".format(dq.type))
        return

    if dq["head"] == 0:
        gdb.write("dq_queue head is empty {}\n".format(dq.address))
        return

    nodes = set()
    entry = dq["head"].dereference()
    try:
        while entry.address:
            nb += 1
            if int(entry.address) in nodes:
                gdb.write("dq_queue is circular: {}\n".format(entry.address))
                return
            nodes.add(int(entry.address))
            entry = entry["flink"].dereference()
    except gdb.MemoryError:
        gdb.write("entry address is unaccessible {}\n".format(entry.address))
        return

    if int(dq["tail"]) not in nodes:
        gdb.write("dq_queue tail is not in the list {}\n".format(dq["tail"]))
        return
    if dq["tail"]["flink"] != 0:
        gdb.write("dq_queue tail->flink is not null {}\n".format(dq["tail"]))
        return

    gdb.write("dq_queue is consistent: {} node(s)\n".format(nb))


class ListCheck(gdb.Command):
    """Verify a list consistency"""

    def __init__(self):
        super().__init__("list_check", gdb.COMMAND_DATA, gdb.COMPLETE_EXPRESSION)

    def invoke(self, arg, from_tty):
        argv = gdb.string_to_argv(arg)
        if len(argv) != 1:
            raise gdb.GdbError("nx-list-check takes one argument")

        obj = gdb.parse_and_eval(argv[0])
        if obj.type == list_node_type.pointer():
            list_check(obj)
        elif obj.type == sq_queue_type.pointer():
            sq_check(obj)
        elif obj.type == dq_queue_type.pointer():
            dq_check(obj)
        else:
            raise gdb.GdbError("Invalid argument type: {}".format(obj.type))


class ForeachListEntry(gdb.Command):
    """Dump list members for a given list"""

    def __init__(self):
        super().__init__("foreach list", gdb.COMMAND_DATA, gdb.COMPLETE_EXPRESSION)

    def invoke(self, arg, from_tty):
        argv = gdb.string_to_argv(arg)

        parser = argparse.ArgumentParser(description="Iterate the items in list")
        parser.add_argument("head", type=str, help="List head")
        parser.add_argument(
            "-n",
            "--next",
            type=str,
            help="The name of the next pointer in the list node",
            default="next",
        )
        parser.add_argument(
            "-c", "--container", type=str, default=None, help="Optional container type"
        )
        parser.add_argument(
            "-m", "--member", type=str, default=None, help="Member name in container"
        )
        parser.add_argument(
            "-e",
            "--element",
            type=str,
            help="Only dump this element in array member struct.",
            default=None,
        )
        try:
            args = parser.parse_args(argv)
        except SystemExit:
            gdb.write("Invalid arguments\n")
            return

        pointer = gdb.parse_and_eval(args.head)
        node = pointer
        i = 0
        while node:
            entry = (
                utils.container_of(node, args.container, args.member)
                if args.container
                else node
            )
            entry = entry.dereference()
            entry = entry[args.element] if args.element else entry
            gdb.write(
                f"{i} *({entry.type} *){hex(entry.address)} {entry.format_string(styling=True)}\n"
            )
            i += 1
            node = node[args.next]
            if node == pointer:
                break


class ForeachArray(gdb.Command):
    """Dump array members."""

    def __init__(self):
        super().__init__("foreach array", gdb.COMMAND_DATA, gdb.COMPLETE_EXPRESSION)

    def invoke(self, arg, from_tty):
        argv = gdb.string_to_argv(arg)

        parser = argparse.ArgumentParser(description="Iterate the items in array")
        parser.add_argument("head", type=str, help="List head")
        parser.add_argument(
            "-l",
            "--length",
            type=int,
            help="The array length",
            default=None,
        )
        parser.add_argument(
            "-e",
            "--element",
            type=str,
            help="Only dump this element in array member struct.",
            default=None,
        )
        try:
            args = parser.parse_args(argv)
        except SystemExit:
            gdb.write("Invalid arguments\n")
            return

        pointer = gdb.parse_and_eval(args.head)
        node = pointer
        len = args.length if args.length else utils.nitems(pointer)
        for i in range(len):
            entry = node[i][args.element] if args.element else node[i]
            gdb.write(f"{i}: {entry.format_string(styling=True)}\n")
