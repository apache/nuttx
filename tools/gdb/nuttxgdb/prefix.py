############################################################################
# tools/gdb/nuttx_gdb/prefix.py
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


class ForeachPrefix(gdb.Command):
    """foreach commands prefix."""

    def __init__(self):
        super(ForeachPrefix, self).__init__("foreach", gdb.COMMAND_USER, prefix=True)


class MMPrefixCommand(gdb.Command):
    """Memory manager related commands prefix."""

    def __init__(self):
        super().__init__("mm", gdb.COMMAND_USER, prefix=True)
