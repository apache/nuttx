# ##############################################################################
# net/utils/CMakeLists.txt
#
# Licensed to the Apache Software Foundation (ASF) under one or more contributor
# license agreements.  See the NOTICE file distributed with this work for
# additional information regarding copyright ownership.  The ASF licenses this
# file to you under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License.  You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations under
# the License.
#
# ##############################################################################

# Common utilities

set(SRCS
    net_dsec2tick.c
    net_dsec2timeval.c
    net_timeval2dsec.c
    net_chksum.c
    net_ipchksum.c
    net_incr32.c
    net_lock.c
    net_snoop.c
    net_cmsg.c
    net_iob_concat.c
    net_getrandom.c
    net_mask2pref.c)

# IPv6 utilities

if(CONFIG_NET_IPv6)
  list(APPEND SRCS net_ipv6_maskcmp.c net_ipv6_pref2mask.c net_ipv6_payload.c)
endif()

# TCP utilities

if(CONFIG_NET_TCP)
  list(APPEND SRCS net_tcpchksum.c)
endif()

# UDP utilities

if(CONFIG_NET_UDP)
  list(APPEND SRCS net_udpchksum.c)
endif()

# ICMP utilities

if(CONFIG_NET_ICMP AND NOT CONFIG_NET_ICMP_NO_STACK)
  list(APPEND SRCS net_icmpchksum.c)
elseif(CONFIG_NET_ICMPv6 AND NOT CONFIG_NET_ICMPv6_NO_STACK)
  list(APPEND SRCS net_icmpchksum.c)
endif()

target_sources(net PRIVATE ${SRCS})
