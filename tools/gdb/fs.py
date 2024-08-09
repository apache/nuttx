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

import gdb
import utils

FSNODEFLAG_TYPE_MASK = utils.get_symbol_value("FSNODEFLAG_TYPE_MASK")
FSNODEFLAG_TYPE_MOUNTPT = utils.get_symbol_value("FSNODEFLAG_TYPE_MOUNTPT")


def get_inode_name(inode):
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


if not utils.get_symbol_value("CONFIG_DISABLE_MOUNTPOINT"):
    Mount()
