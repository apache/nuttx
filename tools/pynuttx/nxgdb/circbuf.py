############################################################################
# tools/pynuttx/nxgdb/circbuf.py
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
from typing import Generator

import gdb

from . import utils
from .protocols import circbuf as p


class CircBuf(utils.Value, p.CircBuf):
    def __init__(
        self, obj: gdb.Value | utils.Value, datatype: gdb.Type | None = None
    ) -> None:
        circbuf_s = utils.lookup_type("struct circbuf_s")
        if obj.type.code == gdb.TYPE_CODE_INT:
            obj = obj.cast(circbuf_s.pointer())

        if obj.type.code == gdb.TYPE_CODE_PTR:
            obj.cast(circbuf_s.pointer())
            obj = obj.dereference()

        super().__init__(obj)

        # datatype must not be a pointer, because we are going to construct value from memory
        if not datatype:
            datatype = utils.lookup_type("char")

        if isinstance(datatype, str):
            datatype = utils.lookup_type(datatype)

        if datatype.code == gdb.TYPE_CODE_PTR:
            datatype = datatype.target()

        self.datatype = datatype

    def __str__(self) -> str:
        return (
            f"(struct circbuf_s *){hex(self.address)} base: {self.base} "
            f"size: {self.size} head: {self.head} tail: {self.tail}"
        )

    @property
    def size(self) -> int:
        return int(self["size"])

    @property
    def used(self) -> int:
        return int(self["head"]) - int(self["tail"])

    @property
    def space(self) -> int:
        return self.size - self.used

    @property
    def is_inited(self) -> bool:
        return bool(self["base"])

    @property
    def is_empty(self) -> bool:
        return self.used == 0

    @property
    def is_full(self) -> bool:
        return not self.space

    def _peekat(self, pos, len) -> memoryview:
        if len > self.size:
            return None

        pos = pos % self.size
        total = len
        if pos + len > self.size:
            len = self.size - pos

        memory = gdb.selected_inferior().read_memory(self.base + pos, len)
        if len < total:
            memory += gdb.selected_inferior().read_memory(self.base, total - len)
        return memory

    @property
    def history(self) -> Generator[utils.Value, None, None]:
        """Iterate over the history data in the circbuf_s, from oldest to newest"""
        if not self.base or not self.size:
            # Uninitialized buffer
            return []

        head = int(self.head)
        size = int(self.size)
        sizeof = self.datatype.sizeof

        if head < size:
            # The buffer is never wrapped, read from the beginning
            offset = 0
            end = head
        else:
            # The buffer is wrapped, read from the head
            offset = head % size
            end = offset + size

        while offset < end:
            memory = self._peekat(offset, sizeof)
            value = gdb.Value(memory, self.datatype)
            yield value
            offset += sizeof

    @property
    def unread(self) -> Generator[utils.Value, None, None]:
        """Return all unread data in circle buffer"""
        if not self.base or not self.size:
            return []

        # Read from tail towards head for all data.
        tail = int(self.tail)
        head = int(self.head)
        sizeof = self.datatype.sizeof
        offset = tail
        while offset < head:
            memory = self._peekat(offset, sizeof)
            yield gdb.Value(memory, self.datatype)
            offset += sizeof


class CircBufInfo(gdb.Command):
    """Print circbuf_s information"""

    def __init__(self):
        super().__init__("circbuf", gdb.COMMAND_USER)

    def invoke(self, arg: str, from_tty: bool) -> None:
        parser = argparse.ArgumentParser(description="Dump circle buffer information")
        parser.add_argument(
            "--type",
            type=str,
            help="The data type the circbuf_s contains",
            default=None,
        )
        parser.add_argument(
            "--history",
            action="store_true",
            help="Dump the history data in the circbuf_s",
        )
        parser.add_argument(
            "--unread",
            action="store_true",
            help="Dump the unread data in the circbuf_s",
        )
        parser.add_argument(
            "address",
            type=str,
            help="The address of the circubuf_s",
        )

        try:
            args = parser.parse_args(gdb.string_to_argv(arg))
        except SystemExit:
            gdb.write("Invalid arguments\n")
            return

        entry = utils.parse_and_eval(args.address)
        circbuf = CircBuf(entry, datatype=args.type)

        print(circbuf)  # Dump buffer basic information

        if args.history:
            dumpdata = circbuf.history
        elif args.unread:
            dumpdata = circbuf.unread
        else:
            dumpdata = []

        print(f"Dumping data with type {args.type}")
        for i, data in enumerate(dumpdata):
            print(f"{i}: {data.format_string(styling=True)}")
