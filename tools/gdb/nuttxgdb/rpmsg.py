############################################################################
# tools/gdb/rpmsg.py
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

from . import utils
from .lists import NxList


class RPMsgDump(gdb.Command):
    """Dump rpmsg service"""

    CALLBACK_HEADER = ["rpmsg_cb_s at", "ns_match", "ns_bind"]
    ENDPOINT_HEADER = [
        "endpoint_addr",
        "name",
        "addr",
        "dest_addr",
        "cb",
        "ns_bound_cb",
        "ns_unbind_cb",
    ]

    CALLBACK_FORAMTTER = "{:<20} {:<40} {:<40}"
    ENDPOINT_FORMATTER = "{:<20} {:<20} {:<12} {:<12} {:<40} {:<40} {:<40}"

    def __init__(self):
        if utils.get_symbol_value("CONFIG_RPMSG"):
            super(RPMsgDump, self).__init__("rpmsgdump", gdb.COMMAND_USER)

    def print_headers(self, headers, formatter):
        gdb.write(formatter.format(*headers) + "\n")
        gdb.write(formatter.format(*["-" * len(header) for header in headers]) + "\n")

    def dump_rdev_epts(self, endpoints_head):
        gdb.write(f"dump_rdev_epts:{endpoints_head}\n")
        self.print_headers(self.ENDPOINT_HEADER, self.ENDPOINT_FORMATTER)

        output = []
        for endpoint in NxList(endpoints_head, "struct rpmsg_endpoint", "node"):
            output.append(
                self.ENDPOINT_FORMATTER.format(
                    f"{endpoint}",
                    f"{endpoint['name'].string()}",
                    f"{endpoint['addr']}",
                    f"{endpoint['dest_addr']}",
                    f"{endpoint['cb']}",
                    f"{endpoint['ns_bound_cb']}",
                    f"{endpoint['ns_unbind_cb']}",
                )
            )

        gdb.write("\n".join(output) + "\n")

    def dump_rdev_bitmap(self, rdev):
        bitmap_values = [hex(bit) for bit in utils.ArrayIterator(rdev["bitmap"])]

        gdb.write(
            f"bitmap:{' '.join(bitmap_values):<20} bitmaplast: {rdev['bitmap']}\n"
        )

    def dump_rdev(self, rdev):
        gdb.write(f"device:{rdev}\n")

        self.dump_rdev_bitmap(rdev)
        self.dump_rdev_epts(rdev["endpoints"])

    def dump_rpmsg_cb(self):
        gdb.write("g_rpmsg_cb:\n")
        self.print_headers(self.CALLBACK_HEADER, self.CALLBACK_FORAMTTER)

        output = []
        for cb in NxList(gdb.parse_and_eval("g_rpmsg_cb"), "struct rpmsg_cb_s", "node"):
            output.append(
                self.CALLBACK_FORAMTTER.format(
                    str(cb), str(cb["ns_match"]), str(cb["ns_bind"])
                )
            )
        gdb.write("\n".join(output) + "\n")

    def dump_rpmsg(self):
        gdb.write("g_rpmsg:\n")
        for rpmsg in NxList(gdb.parse_and_eval("g_rpmsg"), "struct rpmsg_s", "node"):
            self.dump_rdev(rpmsg["rdev"])

    def invoke(self, args, from_tty):
        self.dump_rpmsg_cb()
        self.dump_rpmsg()
