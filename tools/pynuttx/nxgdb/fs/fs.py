############################################################################
# tools/pynuttx/nxgdb/fs.py
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

import gdb
import nxgdb.backtrace as bt
import nxgdb.protocols.fs as p
from nxgdb.fs.inode import inode_getpath
from nxgdb.fs.utils import CONFIG_FS_BACKTRACE, CONFIG_NFILE_DESCRIPTORS_PER_BLOCK
from nxgdb.protocols.thread import Tcb


def get_file(tcb: Tcb, fd):
    group = tcb.group
    fdlist = group.tg_fdlist
    fl_fds = fdlist.fl_fds
    fl_rows = fdlist.fl_rows

    row = fd // CONFIG_NFILE_DESCRIPTORS_PER_BLOCK
    col = fd % CONFIG_NFILE_DESCRIPTORS_PER_BLOCK

    if row >= fl_rows:
        return None

    return fl_fds[row][col]


def print_file_info(fd, fdp: p.Fd, file: p.File, formatter: str):
    backtrace_formatter = "{0:<5} {1:<36} {2}"

    oflags = int(file.f_oflags)
    pos = int(file.f_pos)
    path = inode_getpath(file.f_inode)

    output = []
    if CONFIG_FS_BACKTRACE:
        backtrace = tuple(bt.BacktraceEntry(fdp.f_backtrace).get())
        backtrace = bt.Backtrace(backtrace, backtrace_formatter)

        output.append(
            formatter.format(fd, hex(file), oflags, pos, path, backtrace.formatted[0])
        )
        output.extend(
            formatter.format("", "", "", "", "", bt) for bt in backtrace.formatted[1:]
        )
        output.append("")  # separate each backtrace
    else:
        output = [formatter.format(fd, hex(file), hex(oflags), pos, path, "")]

    gdb.write("\n".join(output).rstrip())  # strip trailing spaces
    gdb.write("\n")


def print_fdinfo_by_tcb(tcb, processed_groups, total_fd_count):
    """print fdlist from tcb"""
    gdb.write(f"PID: {tcb['pid']}\n")
    group = tcb.group

    if not group:
        return

    if group in processed_groups:
        return

    processed_groups.add(group)

    headers = ["FD", "FILEP", "OFLAGS", "POS", "PATH", "BACKTRACE"]
    formatter = "{:<4}{:<12}{:<12}{:<10}{:<48}{:<50}"
    gdb.write(formatter.format(*headers) + "\n")

    fd_count = 0
    for fd, fdp, file in foreach_file(tcb):
        print_file_info(fd, fdp, file, formatter)
        fd_count += 1
    total_fd_count += fd_count

    gdb.write("\n")


def foreach_file(tcb: Tcb):
    """Iterate over all file descriptors in a tcb"""
    group = tcb.group
    fdlist = group.tg_fdlist
    fl_fds = fdlist.fl_fds
    fl_rows = fdlist.fl_rows

    for row in range(fl_rows):
        for col in range(CONFIG_NFILE_DESCRIPTORS_PER_BLOCK):
            fdp = fl_fds[row][col]

            if not fdp or not fdp.f_file:
                continue

            fd = row * CONFIG_NFILE_DESCRIPTORS_PER_BLOCK + col
            file = fdp.f_file

            yield fd, fdp, file
