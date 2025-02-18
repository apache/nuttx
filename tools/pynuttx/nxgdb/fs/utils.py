############################################################################
# tools/pynuttx/nxgdb/fs/utils.py
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

from .. import utils

FSNODEFLAG_TYPE_MASK = 0x0000000F

CONFIG_PSEUDOFS_FILE = utils.get_symbol_value("CONFIG_PSEUDOFS_FILE")
CONFIG_PSEUDOFS_ATTRIBUTES = utils.get_symbol_value("CONFIG_PSEUDOFS_ATTRIBUTES")

CONFIG_FS_BACKTRACE = utils.has_field("struct fd", "f_backtrace")
CONFIG_FS_SHMFS = utils.get_symbol_value("CONFIG_FS_SHMFS")

CONFIG_NFILE_DESCRIPTORS_PER_BLOCK = utils.get_field_nitems(
    "struct fdlist", "fl_prefds"
)

if CONFIG_NFILE_DESCRIPTORS_PER_BLOCK is None:
    # For some branches, this field does not exist
    CONFIG_NFILE_DESCRIPTORS_PER_BLOCK = (
        int(utils.gdb_eval_or_none("CONFIG_NFILE_DESCRIPTORS_PER_BLOCK")) or 8
    )
    CONFIG_NFILE_DESCRIPTORS_PER_BLOCK = int(CONFIG_NFILE_DESCRIPTORS_PER_BLOCK)
