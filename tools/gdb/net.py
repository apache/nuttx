############################################################################
# tools/gdb/net.py
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


class NetStats(gdb.Command):
    """Network statistics"""

    def __init__(self):
        super(NetStats, self).__init__("netstats", gdb.COMMAND_USER)

    def iob_stats(self):
        try:
            size = utils.get_symbol_value("CONFIG_IOB_BUFSIZE")
            ntotal = utils.get_symbol_value("CONFIG_IOB_NBUFFERS")

            nfree = gdb.parse_and_eval("g_iob_sem")["semcount"]
            nwait, nfree = (0, nfree) if nfree >= 0 else (-nfree, 0)

            nthrottle = (
                gdb.parse_and_eval("g_throttle_sem")["semcount"]
                if utils.get_symbol_value("CONFIG_IOB_THROTTLE") > 0
                else 0
            )

            gdb.write(
                "IOB: %10s%10s%10s%10s%10s\n"
                % ("size", "ntotal", "nfree", "nwait", "nthrottle")
            )
            gdb.write(
                "     %10d%10d%10d%10d%10d\n" % (size, ntotal, nfree, nwait, nthrottle)
            )
        except gdb.error as e:
            gdb.write("Failed to get IOB stats: %s\n" % e)

    def pkt_stats(self):
        try:
            netstats = gdb.parse_and_eval("g_netstats")
            gdb.write(
                "Packets:%7s%7s%7s%7s%7s%7s\n"
                % ("IPv4", "IPv6", "TCP", "UDP", "ICMP", "ICMPv6")
            )

            def stats_line(title, member):
                gdb.write("%-8s" % title)
                for proto in ("ipv4", "ipv6", "tcp", "udp", "icmp", "icmpv6"):
                    gdb.write(
                        "%7s"
                        % utils.get_field(utils.get_field(netstats, proto), member, "-")
                    )
                gdb.write("\n")

            stats_line("Received", "recv")
            stats_line("Dropped", "drop")
            stats_line("  VHL", "vhlerr")
            stats_line("  Frag", "fragerr")
            stats_line("  Chksum", "chkerr")
            stats_line("  Type", "typeerr")
            stats_line("  Proto", "protoerr")
            # TODO: Maybe print TCP's ackerr, rst, syndrop, synrst here
            stats_line("Sent", "sent")
            stats_line("  Rexmit", "rexmit")

        except gdb.error as e:
            gdb.write("Failed to get Net Stats: %s\n" % e)

    def invoke(self, args, from_tty):
        if utils.get_symbol_value("CONFIG_MM_IOB"):
            self.iob_stats()
            gdb.write("\n")
        if utils.get_symbol_value("CONFIG_NET_STATISTICS"):
            self.pkt_stats()
            gdb.write("\n")


if utils.get_symbol_value("CONFIG_NET"):
    NetStats()
