############################################################################
# tools/gdb/fs.py
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
import utils

FSNODEFLAG_TYPE_MASK = utils.get_symbol_value("FSNODEFLAG_TYPE_MASK")
FSNODEFLAG_TYPE_MOUNTPT = utils.get_symbol_value("FSNODEFLAG_TYPE_MOUNTPT")

CONFIG_PSEUDOFS_FILE = utils.get_symbol_value("CONFIG_PSEUDOFS_FILE")
CONFIG_PSEUDOFS_ATTRIBUTES = utils.get_symbol_value("CONFIG_PSEUDOFS_ATTRIBUTES")


def get_inode_name(inode):
    if not inode:
        return ""
    ptr = inode["i_name"].cast(gdb.lookup_type("char").pointer())
    return ptr.string()


def foreach_inode(handler, root=None, path=""):
    node = root if root else gdb.parse_and_eval("g_root_inode")["i_child"]
    while node:
        newpath = path + "/" + get_inode_name(node)
        handler(node, newpath)
        if node["i_child"]:
            foreach_inode(handler, node["i_child"], newpath)
        node = node["i_peer"]


class Mount(gdb.Command):
    def __init__(self):
        super(Mount, self).__init__("mount", gdb.COMMAND_USER)

    def mountpt_filter(self, node, path):
        if node["i_flags"] & FSNODEFLAG_TYPE_MASK == FSNODEFLAG_TYPE_MOUNTPT:
            statfs = node["u"]["i_mops"]["statfs"]
            funcname = gdb.block_for_pc(int(statfs)).function.print_name
            fstype = funcname.split("_")[0]
            gdb.write("  %s type %s\n" % (path, fstype))

    def invoke(self, args, from_tty):
        foreach_inode(self.mountpt_filter)


class ForeachInode(gdb.Command):
    """Dump each inode info"""

    def __init__(self):
        super(ForeachInode, self).__init__("foreach_inode", gdb.COMMAND_USER)
        self.level = 4096

    def get_root_inode(self, addr_or_expr):
        try:
            return gdb.Value(int(addr_or_expr, 0)).cast(
                gdb.lookup_type("struct inode").pointer()
            )
        except ValueError:
            return utils.gdb_eval_or_none(addr_or_expr)

    def parse_arguments(self, argv):
        parser = argparse.ArgumentParser(description="foreach_inode command")
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


if not utils.get_symbol_value("CONFIG_DISABLE_MOUNTPOINT"):
    Mount()

ForeachInode()
