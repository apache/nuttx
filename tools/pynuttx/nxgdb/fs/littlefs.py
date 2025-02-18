############################################################################
# tools/pynuttx/nxgdb/fs/littlefs.py
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


def dump_little_cache(node: Inode, path):
    mpt = node.i_private.cast(utils.lookup_type("struct littlefs_mountpt_s").pointer())
    drv = mpt.drv.dereference()
    geo = mpt.geo
    cfg = mpt.cfg
    lfs = mpt.lfs
    print("littlefs {path} mount point information: {hex(mpt)}")
    print("drv:")
    print(drv.format_string(pretty_structs=True, styling=True))
    print("cfg:")
    print(cfg.format_string(pretty_structs=True, styling=True))
    print("geo:")
    print(geo.format_string(pretty_structs=True, styling=True))
    print("lfs:")
    print(lfs.format_string(pretty_structs=True, styling=True))
