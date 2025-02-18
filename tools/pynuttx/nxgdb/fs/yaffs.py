############################################################################
# tools/pynuttx/nxgdb/fs/yaffs.py
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
from nxgdb.lists import NxList

YAFFS_FILETYPE_MAP = {
    0: "yaffs_UNKNOWN_var",
    1: "yaffs_file_var",
    2: "yaffs_symlink_var",
    3: "yaffs_dir_var",
    4: "yaffs_hardlink_var",
    5: "yaffs_SPECIAL_var",
}


def dump_yaffs_files(node, level=1, prefix="", maxlevel=4096):
    if level > maxlevel:
        return

    file_type = YAFFS_FILETYPE_MAP[int(node.variant_type)]
    if file_type == "yaffs_dir_var":
        initial_indent = prefix + "├── "
        newprefix = prefix + "│   "
        dirfix = "/"
        moreinfo = ""
    elif file_type == "yaffs_file_var":
        initial_indent = prefix + "└── "
        newprefix = prefix + "    "
        dirfix = ""
        file = node.variant.cast(utils.lookup_type("struct yaffs_file_var"))
        file_size = file.file_size
        stored = file.stored_size
        moreinfo = f" file_size:{file_size},stored:{stored}"
    else:
        initial_indent = prefix + "└── "
        newprefix = prefix + "    "
        dirfix = ""
        moreinfo = ""

    obj_id = node.obj_id
    short_name = node.short_name.string(length=16).rstrip("\0")
    if short_name == "":
        short_name = "(null)"
    hdr_chunk = node.hdr_chunk
    n_data_chunks = node.n_data_chunks
    serial = str(node.serial).split(" ")[0]
    baseinfo = {
        f"name:{short_name}{dirfix},obj_id:{obj_id},hdr_chunk:{hdr_chunk},"
        f"n_data_chunks:{n_data_chunks},serial:{serial}{moreinfo}"
    }

    print(f"{initial_indent}{file_type.rsplit('_', 2)[1]} {baseinfo}")
    if file_type == "yaffs_dir_var":
        head = node.variant.cast(utils.lookup_type("struct yaffs_dir_var"))
        for siblings in NxList(head.children, "struct yaffs_obj", "siblings"):
            dump_yaffs_files(siblings, level + 1, newprefix, maxlevel)


def dump_yaffs_mpt(mpt):
    geo = mpt.geo.cast(utils.lookup_type("struct mtd_geometry_s"))
    dev = mpt.dev.cast(utils.lookup_type("struct yaffs_dev").pointer())
    blksize = geo.blocksize
    erasesize = geo.erasesize
    erasecount = geo.neraseblocks
    print(f" Block size:{blksize} Erase size:{erasesize} Erase count:{erasecount}")
    print(f"Dump yaffs_dev: {hex(dev)}")
    print(dev.dereference().format_string(pretty_structs=True, styling=True))


def dump_yaffs_cache(node: Inode, path):
    mpt = node.i_private.cast(utils.lookup_type("struct yaffs_mountpt_s").pointer())
    dev = mpt.dev.cast(utils.lookup_type("struct yaffs_dev").pointer())
    root = dev.root_dir.cast(utils.lookup_type("struct yaffs_obj").pointer())
    print(f"Yaffs {path} mount point information: {hex(mpt)}")
    dump_yaffs_mpt(mpt)
    print(f"Yaffs {path} file tree information: {hex(root)}")
    dump_yaffs_files(root)
