############################################################################
# tools/pynuttx/nxgdb/fs/inode.py
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

import enum
from typing import Generator, Tuple, Union

import gdb
import nxgdb.protocols.fs as p
import nxgdb.utils as utils
from nxgdb.fs.utils import (
    CONFIG_PSEUDOFS_ATTRIBUTES,
    CONFIG_PSEUDOFS_FILE,
    FSNODEFLAG_TYPE_MASK,
)

g_special_inodes = {}  # Map of the special inodes including epoll, inotify, etc.


class InodeType(enum.Enum):
    # define   FSNODEFLAG_TYPE_PSEUDODIR  0x00000000 /*   Pseudo dir (default)   */
    # define   FSNODEFLAG_TYPE_DRIVER     0x00000001 /*   Character driver       */
    # define   FSNODEFLAG_TYPE_BLOCK      0x00000002 /*   Block driver           */
    # define   FSNODEFLAG_TYPE_MOUNTPT    0x00000003 /*   Mount point            */
    # define   FSNODEFLAG_TYPE_NAMEDSEM   0x00000004 /*   Named semaphore        */
    # define   FSNODEFLAG_TYPE_MQUEUE     0x00000005 /*   Message Queue          */
    # define   FSNODEFLAG_TYPE_SHM        0x00000006 /*   Shared memory region   */
    # define   FSNODEFLAG_TYPE_MTD        0x00000007 /*   Named MTD driver       */
    # define   FSNODEFLAG_TYPE_SOFTLINK   0x00000008 /*   Soft link              */
    # define   FSNODEFLAG_TYPE_SOCKET     0x00000009 /*   Socket                 */
    # define   FSNODEFLAG_TYPE_PIPE       0x0000000a /*   Pipe                   */
    # define   FSNODEFLAG_TYPE_NAMEDEVENT 0x0000000b /*   Named event group      */
    PSEUDODIR = 0
    DRIVER = 1
    BLOCK = 2
    MOUNTPT = 3
    NAMEDSEM = 4
    MQUEUE = 5
    SHM = 6
    MTD = 7
    SOFTLINK = 8
    SOCKET = 9
    PIPE = 10
    NAMEDEVENT = 11
    UNKNOWN = 12


class Inode(utils.Value, p.Inode):
    def __init__(self, obj: Union[gdb.Value, utils.Value]):
        super().__init__(obj)

    def __str__(self) -> str:
        name = get_inode_name(self)
        type = inode_gettype(self)
        return f"{name}{'/' if type == InodeType.PSEUDODIR else ''}{int(self): #x} {type.name}"

    def __repr__(self) -> str:
        return self.__str__()

    def details(self) -> str:
        details = (
            f"ino: {self.i_ino}, crefs: {self.i_crefs}"
            f", flags: {self.i_flags}, private: {self.i_private}"
        )

        if CONFIG_PSEUDOFS_FILE:
            details += f", i_size: {self.i_size}"
        if CONFIG_PSEUDOFS_ATTRIBUTES:
            details += f", i_mode: {self.i_mode}, i_owner: {self.i_owner}, i_group: {self.i_group}"
            details += f", i_atime: {self.i_atime}, i_mtime: {self.i_mtime}, i_ctime: {self.i_ctime}"
        return details

    @property
    def nodetype(self) -> InodeType:
        return inode_gettype(self)


def inode_gettype(inode: p.Inode) -> InodeType:
    if not inode:
        return InodeType.UNKNOWN

    type = int(inode.i_flags & FSNODEFLAG_TYPE_MASK)

    # check if it's a valid type in InodeType
    if type in [e.value for e in InodeType]:
        return InodeType(type)

    return InodeType.UNKNOWN


def get_inode_name(inode: p.Inode):
    try:
        if not inode:
            return ""

        def special_inode_name(inode):
            global g_special_inodes
            if not g_special_inodes:
                g_special_inodes = {}
                for special in (
                    "perf",
                    "timerfd",
                    "signalfd",
                    "dir",
                    "inotify",
                    "epoll",
                    "eventfd",
                    "sock",
                ):
                    value = utils.gdb_eval_or_none(f"g_{special}_inode")
                    if value:
                        g_special_inodes[special] = value.address

            for name, value in g_special_inodes.items():
                if value == inode:
                    return name

            return None

        if name := special_inode_name(inode):
            return name

        ptr = inode.i_name.cast(gdb.lookup_type("char").pointer())
        return ptr.string()
    except gdb.MemoryError:
        return "<MemoryError>"


def inode_getpath(inode: p.Inode):
    """get path from inode"""
    try:
        if not inode:
            return ""

        name = get_inode_name(inode)

        if inode.i_parent:
            return inode_getpath(inode.i_parent) + "/" + name

        return name
    except gdb.MemoryError:
        return "<MemoryError>"


def get_root_inode(addr_or_expr):
    try:
        return utils.Value(int(addr_or_expr, 0)).cast(
            gdb.lookup_type("struct inode").pointer()
        )
    except ValueError:
        return utils.gdb_eval_or_none(addr_or_expr)


def print_inode_info(
    nodes, node: Inode, level=1, prefix="", maxlevel=4096, type=None, verbose=False
):
    if level > maxlevel:
        return

    while node:
        if node.i_peer:
            initial_indent = prefix + "├── "
            newprefix = prefix + "│   "
        else:
            initial_indent = prefix + "└── "
            newprefix = prefix + "    "

        if (
            not type
            or node.nodetype == type
            or any(n.i_parent == node for n, _ in nodes)
        ):
            gdb.write(f"{initial_indent}{node} {node.details() if verbose else ''}\n")

        if node.i_child:
            print_inode_info(
                nodes,
                Inode(node.i_child),
                level + 1,
                newprefix,
                maxlevel,
                type,
                verbose,
            )
        node = Inode(node.i_peer)


def foreach_inode(root=None, path="") -> Generator[Tuple[p.Inode, str], None, None]:
    node: p.Inode = root or utils.parse_and_eval("g_root_inode").i_child
    while node:
        newpath = path + "/" + get_inode_name(node)
        yield node, newpath
        if node.i_child:
            yield from foreach_inode(node.i_child, newpath)
        node = node.i_peer


def get_fstype(node: p.Inode):
    try:
        if not node:
            return ""

        statfs = node.u.i_mops.statfs
        funcname = gdb.block_for_pc(int(statfs)).function.print_name
        return funcname.split("_")[0]

    except gdb.MemoryError:
        return "<MemoryError>"


def fstype_filter(fstype):
    """Filter inodes by filesystem type"""

    def filter(item):
        inode, path = item
        if inode_gettype(inode) != InodeType.MOUNTPT:
            return False
        return get_fstype(inode) == fstype

    return filter
