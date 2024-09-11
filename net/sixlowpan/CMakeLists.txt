# ##############################################################################
# net/sixlowpan/CMakeLists.txt
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

# IEEE 802.15.4 support

if(CONFIG_NET_6LOWPAN)
  # Include IEEE 802.15.4 file in the build

  set(SRCS
      sixlowpan_initialize.c
      sixlowpan_globals.c
      sixlowpan_utils.c
      sixlowpan_input.c
      sixlowpan_framer.c
      sixlowpan_framelist.c
      sixlowpan_reassbuf.c)

  if(CONFIG_NET_TCP)
    list(APPEND SRCS sixlowpan_tcpsend.c)
  endif()

  if(CONFIG_NET_UDP)
    list(APPEND SRCS sixlowpan_udpsend.c)
  endif()

  if(CONFIG_NET_ICMPv6)
    list(APPEND SRCS sixlowpan_icmpv6send.c)
  endif()

  if(CONFIG_NET_UDP)
    list(APPEND SRCS sixlowpan_send.c)
  elseif(CONFIG_NET_ICMPv6)
    list(APPEND SRCS sixlowpan_send.c)
  endif()

  if(CONFIG_NET_6LOWPAN_COMPRESSION_HC1)
    list(APPEND SRCS sixlowpan_hc1.c)
  endif()

  if(CONFIG_NET_6LOWPAN_COMPRESSION_HC06)
    list(APPEND SRCS sixlowpan_hc06.c)
  endif()

  target_sources(net PRIVATE ${SRCS})
endif()
