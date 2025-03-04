# ##############################################################################
# net/icmpv6/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
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

if(CONFIG_NET_ICMPv6 AND NOT CONFIG_NET_ICMPv6_NO_STACK)
  # ICMPv6 source files

  set(SRCS icmpv6_input.c icmpv6_solicit.c icmpv6_advertise.c
           icmpv6_linkipaddr.c icmpv6_reply.c icmpv6_initialize.c)

  if(CONFIG_NET_ICMPv6_SOCKET)
    list(
      APPEND
      SRCS
      icmpv6_sockif.c
      icmpv6_conn.c
      icmpv6_sendmsg.c
      icmpv6_recvmsg.c
      icmpv6_netpoll.c
      icmpv6_ioctl.c)
  endif()

  if(CONFIG_NET_ICMPv6_NEIGHBOR)
    list(APPEND SRCS icmpv6_neighbor.c icmpv6_notify.c)
  endif()

  if(CONFIG_NET_ICMPv6_SOCKET OR CONFIG_NET_ICMPv6_NEIGHBOR)
    list(APPEND SRCS icmpv6_poll.c)
  endif()

  if(CONFIG_NET_ICMPv6_AUTOCONF)
    list(APPEND SRCS icmpv6_autoconfig.c icmpv6_rsolicit.c icmpv6_rnotify.c)
  endif()

  if(CONFIG_NET_ICMPv6_ROUTER)
    list(APPEND SRCS icmpv6_radvertise.c)
  endif()

  if(NOT "${CONFIG_NET_ICMPv6_PMTU_ENTRIES}" STREQUAL "0")
    list(APPEND SRCS icmpv6_pmtu.c)
  endif()

  target_sources(net PRIVATE ${SRCS})
endif()
