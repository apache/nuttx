############################################################################
# tools/gdb/dmesg.py
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

CONFIG_RAMLOG_SYSLOG = utils.get_symbol_value("CONFIG_RAMLOG_SYSLOG")


class Dmesg(gdb.Command):
    def __init__(self):
        super().__init__("dmesg", gdb.COMMAND_USER)

    def invoke(self, args, from_tty):
        sysdev = utils.gdb_eval_or_none("g_sysdev")
        if not sysdev:
            gdb.write("RAM log not available.\n")
            return

        rl_head = sysdev["rl_header"]
        rl_bufsize = sysdev["rl_bufsize"]
        gdb.write("Ramlog have %d bytes to show\n" % rl_bufsize)

        inf = gdb.selected_inferior()
        buf = inf.read_memory(rl_head["rl_buffer"], rl_bufsize)
        clean_data = bytes(buf).replace(b"\x00", "␀".encode("utf-8"))
        gdb.write(clean_data.decode("utf-8"))
        gdb.write("\n")


if CONFIG_RAMLOG_SYSLOG:
    Dmesg()
