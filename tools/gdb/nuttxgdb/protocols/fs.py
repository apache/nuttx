############################################################################
# tools/gdb/nuttxgdb/protocols/fs.py
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

from .value import Value


class File(Value):
    """struct file"""

    f_oflags: Value
    f_refs: Value
    f_pos: Value
    f_inode: Value
    f_priv: Value
    f_tag_fdsan: Value
    f_tag_fdcheck: Value
    f_backtrace: Value
    locked: Value


class Inode(Value):
    """struct inode"""

    i_parent: Value
    i_peer: Value
    i_child: Value
    i_crefs: Value
    i_flags: Value
    u: Value
    i_ino: Value
    i_size: Value
    i_mode: Value
    i_owner: Value
    i_group: Value
    i_atime: Value
    i_mtime: Value
    i_ctime: Value
    i_private: Value
    i_name: Value


class FileList(Value):
    """struct filelist_s"""

    fl_rows: Value
    fl_files: Value
