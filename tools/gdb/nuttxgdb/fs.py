############################################################################
# tools/gdb/nuttxgdb/fs.py
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
import enum

import gdb

from . import utils

FSNODEFLAG_TYPE_MASK = utils.get_symbol_value("FSNODEFLAG_TYPE_MASK")

CONFIG_PSEUDOFS_FILE = utils.get_symbol_value("CONFIG_PSEUDOFS_FILE")
CONFIG_PSEUDOFS_ATTRIBUTES = utils.get_symbol_value("CONFIG_PSEUDOFS_ATTRIBUTES")

CONFIG_FS_BACKTRACE = utils.get_symbol_value("CONFIG_FS_BACKTRACE")
CONFIG_NFILE_DESCRIPTORS_PER_BLOCK = int(
    utils.get_symbol_value("CONFIG_NFILE_DESCRIPTORS_PER_BLOCK")
)
CONFIG_FS_SHMFS = utils.get_symbol_value("CONFIG_FS_SHMFS")


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


def get_inode_name(inode):
    if not inode:
        return ""
    ptr = inode["i_name"].cast(gdb.lookup_type("char").pointer())
    return ptr.string()


def inode_getpath(inode):
    """get path fron inode"""
    if not inode:
        return ""

    name = get_inode_name(inode)

    if inode["i_parent"]:
        return inode_getpath(inode["i_parent"]) + "/" + name

    return name


def inode_gettype(inode) -> InodeType:
    if not inode:
        return InodeType.UNKNOWN

    type = int(inode["i_flags"] & FSNODEFLAG_TYPE_MASK)

    # check if it's a valid type in InodeType
    if type in [e.value for e in InodeType]:
        return InodeType(type)

    return InodeType.UNKNOWN


def get_file(tcb, fd):
    group = tcb["group"]
    filelist = group["tg_filelist"]
    fl_files = filelist["fl_files"]
    fl_rows = filelist["fl_rows"]

    row = fd // CONFIG_NFILE_DESCRIPTORS_PER_BLOCK
    col = fd % CONFIG_NFILE_DESCRIPTORS_PER_BLOCK

    if row >= fl_rows:
        return None

    return fl_files[row][col]


def foreach_inode(handler, root=None, path=""):
    node = root if root else gdb.parse_and_eval("g_root_inode")["i_child"]
    while node:
        newpath = path + "/" + get_inode_name(node)
        handler(node, newpath)
        if node["i_child"]:
            foreach_inode(handler, node["i_child"], newpath)
        node = node["i_peer"]


def foreach_file(tcb):
    """Iterate over all file descriptors in a tcb"""
    group = tcb["group"]
    filelist = group["tg_filelist"]
    fl_files = filelist["fl_files"]
    fl_rows = filelist["fl_rows"]

    for row in range(fl_rows):
        for col in range(CONFIG_NFILE_DESCRIPTORS_PER_BLOCK):
            file = fl_files[row][col]

            if not file or not file["f_inode"]:
                continue

            fd = row * CONFIG_NFILE_DESCRIPTORS_PER_BLOCK + col

            yield fd, file


class Fdinfo(gdb.Command):
    """Dump fd info information of process"""

    def __init__(self):
        super().__init__("fdinfo", gdb.COMMAND_DATA, gdb.COMPLETE_EXPRESSION)

    def print_file_info(self, fd, file, formatter):
        backtrace_formatter = "{0:<5} {1:<36} {2}"

        oflags = int(file["f_oflags"])
        pos = int(file["f_pos"])
        path = inode_getpath(file["f_inode"])

        output = []
        if CONFIG_FS_BACKTRACE:
            backtrace = utils.backtrace(
                file["f_backtrace"][i] for i in range(CONFIG_FS_BACKTRACE)
            )

            backtrace = [
                backtrace_formatter.format(
                    hex(addr),
                    func,
                    source,
                )
                for addr, func, source in backtrace
            ]

            output.append(formatter.format(fd, oflags, pos, path, backtrace[0]))
            output.extend(formatter.format("", "", "", "", bt) for bt in backtrace[1:])
            output.append("")  # separate each backtrace
        else:
            output = [formatter.format(fd, oflags, pos, path, "")]

        gdb.write("\n".join(output))
        gdb.write("\n")

    def print_fdinfo_by_tcb(self, tcb):
        """print fdlist from tcb"""
        gdb.write(f"PID: {tcb['pid']}\n")
        group = tcb["group"]

        if not group:
            return

        if group in self.processed_groups:
            return

        self.processed_groups.add(group)

        headers = ["FD", "OFLAGS", "POS", "PATH", "BACKTRACE"]
        formatter = "{:<4}{:<8}{:<6}{:<22}{:<50}"
        gdb.write(formatter.format(*headers) + "\n")

        for fd, file in foreach_file(tcb):
            self.print_file_info(fd, file, formatter)

        gdb.write("\n")

    def invoke(self, arg, from_tty):
        parser = argparse.ArgumentParser(
            description="Get fdinfo for a process or all processes."
        )
        parser.add_argument("-p", "--pid", type=int, help="Optional process ID")

        try:
            args = parser.parse_args(gdb.string_to_argv(arg))
        except SystemExit:
            gdb.write("Invalid arguments.\n")
            return

        self.processed_groups = set()
        tcbs = [utils.get_tcb(args.pid)] if args.pid else utils.get_tcbs()
        for tcb in tcbs:
            self.print_fdinfo_by_tcb(tcb)


class Mount(gdb.Command):
    def __init__(self):
        if not utils.get_symbol_value("CONFIG_DISABLE_MOUNTPOINT"):
            super().__init__("mount", gdb.COMMAND_USER)

    def mountpt_filter(self, node, path):
        if inode_gettype(node) == InodeType.MOUNTPT:
            statfs = node["u"]["i_mops"]["statfs"]
            funcname = gdb.block_for_pc(int(statfs)).function.print_name
            fstype = funcname.split("_")[0]
            gdb.write("  %s type %s\n" % (path, fstype))

    def invoke(self, args, from_tty):
        foreach_inode(self.mountpt_filter)


class ForeachInode(gdb.Command):
    """Dump each inode info"""

    def __init__(self):
        super().__init__("foreach inode", gdb.COMMAND_USER)
        self.level = 4096

    def get_root_inode(self, addr_or_expr):
        try:
            return gdb.Value(int(addr_or_expr, 0)).cast(
                gdb.lookup_type("struct inode").pointer()
            )
        except ValueError:
            return utils.gdb_eval_or_none(addr_or_expr)

    def parse_arguments(self, argv):
        parser = argparse.ArgumentParser(description="foreach inode command")
        parser.add_argument(
            "-L",
            "--level",
            type=int,
            default=None,
            help="Only render the tree to a specific depth",
        )
        parser.add_argument(
            "addr_or_expr",
            type=str,
            nargs="?",
            default=None,
            help="set the start inode to be tranversed",
        )
        try:
            args = parser.parse_args(argv)
        except SystemExit:
            return None
        return {
            "level": args.level if args.level else 4096,
            "root_inode": (
                self.get_root_inode(args.addr_or_expr)
                if args.addr_or_expr
                else utils.gdb_eval_or_none("g_root_inode")
            ),
        }

    def print_inode_info(self, node, level, prefix):
        if level > self.level:
            return
        while node:
            if node["i_peer"]:
                initial_indent = prefix + "├── "
                subsequent_indent = prefix + "│   "
                newprefix = prefix + "│   "
            else:
                initial_indent = prefix + "└── "
                subsequent_indent = prefix + "    "
                newprefix = prefix + "    "
            gdb.write(
                "%s [%s], %s, %s\n"
                % (initial_indent, get_inode_name(node), node["i_ino"], node)
            )
            gdb.write(
                "%s i_crefs: %s, i_flags: %s, i_private: %s\n"
                % (
                    subsequent_indent,
                    node["i_crefs"],
                    node["i_flags"],
                    node["i_private"],
                )
            )
            if CONFIG_PSEUDOFS_FILE:
                gdb.write("%s i_size: %s\n" % (subsequent_indent, node["i_size"]))
            if CONFIG_PSEUDOFS_ATTRIBUTES:
                gdb.write(
                    "%s i_mode: %s, i_owner: %s, i_group: %s\n"
                    % (
                        subsequent_indent,
                        node["i_mode"],
                        node["i_owner"],
                        node["i_group"],
                    )
                )
                gdb.write(
                    "%s i_atime: %s, i_mtime: %s, i_ctime: %s\n"
                    % (
                        subsequent_indent,
                        node["i_atime"],
                        node["i_mtime"],
                        node["i_ctime"],
                    )
                )
            if node["i_child"]:
                self.print_inode_info(node["i_child"], level + 1, newprefix)
            node = node["i_peer"]

    def invoke(self, args, from_tty):
        arg = self.parse_arguments(args.split(" "))
        if not arg:
            return
        self.level = arg["level"]
        self.print_inode_info(arg["root_inode"], 1, "")


class InfoShmfs(gdb.Command):
    """Show share memory usage"""

    def __init__(self):
        if CONFIG_FS_SHMFS:
            super().__init__("info shm", gdb.COMMAND_USER)

    def shm_filter(self, node, path):
        if inode_gettype(node) != InodeType.SHM:
            return

        obj = node["i_private"].cast(gdb.lookup_type("struct shmfs_object_s").pointer())
        length = obj["length"]
        paddr = obj["paddr"]
        print(f"  {path} memsize: {length}, paddr: {paddr}")

    def invoke(self, args, from_tty):
        foreach_inode(self.shm_filter)
