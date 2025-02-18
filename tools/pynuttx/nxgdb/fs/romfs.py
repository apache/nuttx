############################################################################
# tools/pynuttx/nxgdb/fs/romfs.py
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

import nxgdb.utils as utils
from nxgdb.fs.inode import Inode


def dump_romfs_mpt(mpt):
    sector_size = mpt.rm_hwsectorsize
    sector_num = mpt.rm_hwnsectors
    volume_size = mpt.rm_volsize
    xip_base = mpt.rm_xipbase
    buffer_base = mpt.rm_buffer
    print(
        f" HW sector size: {sector_size} HW sector number: {sector_num}\n"
        f" Volume size: {volume_size} XIP_addr: {hex(xip_base)}"
        f" Buffer_addr: {hex(buffer_base)}"
    )


def dump_romfs_files(node, level=1, prefix="", maxlevel=4096):
    if level > maxlevel:
        return

    if node.rn_count > 0:
        initial_indent = prefix + "├── "
        newprefix = prefix + "│   "
        dirfix = "/"
    else:
        initial_indent = prefix + "└── "
        newprefix = prefix + "    "
        dirfix = ""

    name = node.rn_name.string(length=node.rn_namesize)
    print(
        f"{initial_indent}{name}{dirfix} offset:{node.rn_offset} next:{node.rn_next}"
        f" size:{node.rn_size} child_count:{node.rn_count}"
    )

    for child in utils.ArrayIterator(node.rn_child, node.rn_count):
        dump_romfs_files(child, level + 1, newprefix, maxlevel)


def dump_romfs_cache(node: Inode, path):
    mpt = node.i_private.cast(utils.lookup_type("struct romfs_mountpt_s").pointer())
    root = mpt.rm_root.cast(utils.lookup_type("struct romfs_nodeinfo_s").pointer())
    print(f"Romfs {path} mount point information: {hex(mpt)}")
    dump_romfs_mpt(mpt)
    dump_romfs_files(root)
