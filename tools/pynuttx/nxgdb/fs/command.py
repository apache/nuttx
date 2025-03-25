############################################################################
# tools/pynuttx/nxgdb/fs/command.py
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
import nxgdb.autocompeletion as autocompeletion
import nxgdb.fs.fatfs as fatfs
import nxgdb.fs.littlefs as littlefs
import nxgdb.fs.romfs as romfs
import nxgdb.fs.yaffs as yaffs
import nxgdb.utils as utils
from nxgdb.fs.fs import print_fdinfo_by_tcb
from nxgdb.fs.inode import (
    Inode,
    InodeType,
    foreach_inode,
    fstype_filter,
    get_fstype,
    get_root_inode,
    inode_gettype,
    print_inode_info,
)
from nxgdb.fs.utils import CONFIG_FS_SHMFS


@autocompeletion.complete
class ForeachInode(gdb.Command):
    """Dump each inode info"""

    def get_argparser(self):
        parser = argparse.ArgumentParser(description="foreach inode command")
        parser.add_argument(
            "-L",
            "--level",
            type=int,
            default=4096,
            help="Only render the tree to a specific depth",
        )
        parser.add_argument(
            "--nodetype",
            type=str,
            choices=[e.name.lower() for e in InodeType],
            default=None,
            help="Only show the specific type of inode",
        )
        parser.add_argument(
            "-v",
            "--verbose",
            action="store_true",
            help="Show more information",
        )
        parser.add_argument(
            "addr_or_expr",
            type=str,
            metavar="symbol",
            nargs="?",
            default=None,
            help="set the start inode to be tranversed",
        )
        return parser

    def __init__(self):
        super().__init__("foreach inode", gdb.COMMAND_USER)
        utils.alias("inode-foreach", "foreach inode")
        self.parser = self.get_argparser()

    def parse_arguments(self, argv):
        try:
            args = self.parser.parse_args(argv)
        except SystemExit:
            return None

        return args

    def diagnose(self, *args, **kwargs):
        output = gdb.execute("foreach inode", to_string=True)

        return {
            "title": "File Node Information",
            "summary": "inode formation dump",
            "command": "foreach inode",
            "result": "info",
            "category": utils.DiagnoseCategory.fs,
            "message": output,
        }

    @utils.dont_repeat_decorator
    def invoke(self, args, from_tty):
        args = self.parse_arguments(args.split(" "))
        if not args:
            return

        root = (
            get_root_inode(args.addr_or_expr)
            if args.addr_or_expr
            else utils.gdb_eval_or_none("g_root_inode")
        )

        nodetype = InodeType[args.nodetype.upper()] if args.nodetype else None
        if nodetype:
            self.nodes = list(
                filter(lambda x: inode_gettype(x[0]) == nodetype, foreach_inode(root))
            )

        print_inode_info(
            self.nodes if hasattr(self, "nodes") else None,
            Inode(root),
            maxlevel=args.level,
            type=nodetype,
            verbose=args.verbose,
        )


@autocompeletion.complete
class Fdinfo(gdb.Command):
    """Dump fd info information of process"""

    def get_argparser(self):
        parser = argparse.ArgumentParser(
            description="Get fdinfo for a process or all processes."
        )
        parser.add_argument("-p", "--pid", type=int, help="Optional process ID")
        return parser

    def __init__(self):
        super().__init__("fdinfo", gdb.COMMAND_DATA)
        self.total_fd_count = 0
        self.parser = self.get_argparser()

    def diagnose(self, *args, **kwargs):
        output = gdb.execute("fdinfo", to_string=True)

        return {
            "title": "File Descriptors Information",
            "summary": f"Total files opened:{self.total_fd_count}",
            "result": "info",
            "category": utils.DiagnoseCategory.fs,
            "command": "fdinfo",
            "message": output,
        }

    @utils.dont_repeat_decorator
    def invoke(self, arg, from_tty):
        try:
            args = self.parser.parse_args(gdb.string_to_argv(arg))
        except SystemExit:
            gdb.write("Invalid arguments.\n")
            return

        self.processed_groups = set()
        self.total_fd_count = 0
        tcbs = [utils.get_tcb(args.pid)] if args.pid else utils.get_tcbs()
        for tcb in tcbs:
            print_fdinfo_by_tcb(tcb, self.processed_groups, self.total_fd_count)


class Mount(gdb.Command):
    def __init__(self):
        if not utils.get_symbol_value("CONFIG_DISABLE_MOUNTPOINT"):
            super().__init__("mount", gdb.COMMAND_USER)
            self.mount_count = 0

    def diagnose(self, *args, **kwargs):
        output = gdb.execute("mount", to_string=True)

        return {
            "title": "File System Mount Information",
            "summary": f"Total {self.mount_count} mount points",
            "command": "mount",
            "result": "info",
            "category": utils.DiagnoseCategory.fs,
            "message": output or "No mount",
        }

    @utils.dont_repeat_decorator
    def invoke(self, args, from_tty):
        self.mount_count = 0
        nodes = filter(
            lambda x: inode_gettype(x[0]) == InodeType.MOUNTPT, foreach_inode()
        )
        for node, path in nodes:
            fstype = get_fstype(node)
            gdb.write("  %s type %s\n" % (path, fstype))
            self.mount_count += 1


class InfoShmfs(gdb.Command):
    """Show share memory usage"""

    def __init__(self):
        if CONFIG_FS_SHMFS:
            super().__init__("info shm", gdb.COMMAND_USER)
            self.total_size = 0
            self.block_count = 0

    def diagnose(self, *args, **kwargs):
        output = gdb.execute("info shm", to_string=True)

        return {
            "title": "Share Memory Usage",
            "summary": f"Total used:{self.total_size}kB, {self.block_count}blocks",
            "result": "info",
            "category": utils.DiagnoseCategory.fs,
            "command": "info shm",
            "message": output or "No InfoShmfs",
        }

    @utils.dont_repeat_decorator
    def invoke(self, args, from_tty):
        self.total_size = 0
        self.block_count = 0
        nodes = filter(lambda x: inode_gettype(x[0]) == InodeType.SHM, foreach_inode())
        for node, path in nodes:
            obj = node.i_private.cast(
                gdb.lookup_type("struct shmfs_object_s").pointer()
            )
            length = obj.length
            paddr = obj.paddr
            print(f"  {path} memsize: {length}, paddr: {paddr}")

            self.total_size += length / 1024
            self.block_count += 1


@autocompeletion.complete
class InfoRomfs(gdb.Command):
    """Show romfs cache information"""

    def get_argparser(self):
        parser = argparse.ArgumentParser(description=self.__doc__)
        parser.add_argument(
            "-P",
            "--path",
            type=str,
            metavar="file",
            default=None,
            help="set the romfs path to be dumped",
        )
        return parser

    def __init__(self):
        if utils.get_symbol_value("CONFIG_FS_ROMFS_CACHE_NODE"):
            super().__init__("info romfs", gdb.COMMAND_USER)
        self.parser = self.get_argparser()

    def parse_arguments(self, argv):
        try:
            args = self.parser.parse_args(argv)
        except SystemExit:
            return None

        return args

    def diagnose(self, *args, **kwargs):
        output = gdb.execute("info romfs", to_string=True)

        return {
            "title": "Romfs Cache Information",
            "summary": "Romfs nodeinfo dump",
            "command": "info romfs",
            "result": "info",
            "category": utils.DiagnoseCategory.fs,
            "message": output or "No romfs information",
        }

    @utils.dont_repeat_decorator
    def invoke(self, args, from_tty):
        args = self.parse_arguments(gdb.string_to_argv(args))
        nodes = filter(fstype_filter("romfs"), foreach_inode())
        for node, path in nodes:
            if args and args.path and path != args.path:
                continue
            romfs.dump_romfs_cache(node, path)


class InfoLittlefs(gdb.Command):
    """Show littlefs cache information"""

    def __init__(self):
        if utils.get_symbol_value("CONFIG_FS_LITTLEFS"):
            super().__init__("info littlefs", gdb.COMMAND_USER)

    def parse_arguments(self, argv):
        parser = argparse.ArgumentParser(description=self.__doc__)
        parser.add_argument(
            "-P",
            "--path",
            type=str,
            default=None,
            help="set the littlefs path to be dumped",
        )

        try:
            args = parser.parse_args(argv)
        except SystemExit:
            return None

        return args

    def diagnose(self, *args, **kwargs):
        output = gdb.execute("info littlefs", to_string=True)

        return {
            "title": "Littlefs cache information",
            "summary": "Littlefs nodeinfo dump",
            "command": "info littlefs",
            "result": "info",
            "message": output or "No littlefs information",
        }

    @utils.dont_repeat_decorator
    def invoke(self, args, from_tty):
        args = self.parse_arguments(gdb.string_to_argv(args))
        nodes = filter(fstype_filter("littlefs"), foreach_inode())
        for node, path in nodes:
            if args and args.path and path != args.path:
                continue
            littlefs.dump_little_cache(node, path)


@autocompeletion.complete
class InfoYaffs(gdb.Command):
    """Show yaffs cache information"""

    def get_argparser(self):
        parser = argparse.ArgumentParser(description=gdb.__doc__)
        parser.add_argument(
            "-P",
            "--path",
            type=str,
            metavar="file",
            default=None,
            help="set the yaffs path to be dumped",
        )
        return parser

    def __init__(self):
        if utils.get_symbol_value("CONFIG_FS_YAFFS"):
            super().__init__("info yaffs", gdb.COMMAND_USER)
        self.parser = self.get_argparser()

    def parse_arguments(self, argv):
        try:
            args = self.parser.parse_args(argv)
        except SystemExit:
            return None

        return args

    def diagnose(self, *args, **kwargs):
        output = gdb.execute("info yaffs", to_string=True)
        return {
            "title": "Yaffs Cache Information",
            "summary": "Yaffs information dump",
            "command": "info Yaffs",
            "result": "info",
            "category": utils.DiagnoseCategory.fs,
            "message": output or "No yaffs information",
        }

    @utils.dont_repeat_decorator
    def invoke(self, args, from_tty):
        args = self.parse_arguments(gdb.string_to_argv(args))
        nodes = filter(fstype_filter("yaffs"), foreach_inode())
        for node, path in nodes:
            if args and args.path and path != args.path:
                continue
            yaffs.dump_yaffs_cache(node, path)


class InfoFatfs(gdb.Command):
    """Show fatfs information"""

    def __init__(self):
        if utils.get_symbol_value("CONFIG_FS_FATFS"):
            super().__init__("info fatfs", gdb.COMMAND_USER)

    def parse_arguments(self, argv):
        parser = argparse.ArgumentParser(description=gdb.__doc__)
        parser.add_argument(
            "-P",
            "--path",
            type=str,
            default=None,
            help="set the fatfs path to be dumped",
        )
        parser.add_argument(
            "-F",
            "--filep",
            type=str,
            default=None,
            help="Show file information",
        )
        try:
            args = parser.parse_args(argv)
        except SystemExit:
            return None
        return args

    def diagnose(self, *args, **kwargs):
        output = gdb.execute("info fatfs", to_string=True)
        return {
            "title": "Fatfs information",
            "summary": "Fatfs information dump",
            "command": "info fatfs",
            "result": "info",
            "message": output or "No fatfs information",
        }

    def invoke(self, args, from_tty):
        args = self.parse_arguments(gdb.string_to_argv(args))
        nodes = filter(fstype_filter("fatfs"), foreach_inode())
        if args and args.filep:
            fatfs.dump_fatfs_file(utils.Value(utils.parse_arg(args.filep)))
            return
        for node, path in nodes:
            if args and args.path and path != args.path:
                continue
            fatfs.dump_fatfs_cache(node, path)
