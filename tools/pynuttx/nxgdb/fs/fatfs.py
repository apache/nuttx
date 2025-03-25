############################################################################
# tools/pynuttx/nxgdb/fs/fatfs.py
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


def dump_fatfs_cache(node: Inode, path):
    mpt = node.i_private.cast(utils.lookup_type("struct fatfs_mountpt_s").pointer())
    g_drv = utils.parse_and_eval("g_drv").address
    driver = g_drv[mpt.pdrv].cast(utils.lookup_type("struct fatfs_driver_s").pointer())
    opt = driver.drv.u.i_bops.cast(
        utils.lookup_type("struct block_operations").pointer()
    )
    print(f"Fatfs {path} mount point information: {hex(mpt)}")
    print(mpt.dereference().format_string(pretty_structs=True, styling=True))
    print(f"Fatfs {path} driver information: {hex(driver)}")
    print(f"ratio {hex(driver.ratio)}\nblock operations:")
    print(opt.dereference().format_string(pretty_structs=True, styling=True))


def dump_fatfs_file(pointer: utils.Value):
    filep = pointer.cast(utils.lookup_type("struct file").pointer())
    print(f"Fatfs filep information: {hex(filep)}")
    print(filep.dereference().format_string(pretty_structs=True, styling=True))
    fp = filep.f_priv.cast(utils.lookup_type("struct fatfs_file_s").pointer())
    print(f"Fatfs file information: {hex(fp)}")
    print(fp.dereference().format_string(pretty_structs=True, styling=True))
