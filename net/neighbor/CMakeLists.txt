# ##############################################################################
# net/neighbor/CMakeLists.txt
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

# Logic specific to IPv6 Neighbor Discovery Protocol

if(CONFIG_NET_IPv6)
  set(SRCS neighbor_globals.c neighbor_add.c neighbor_lookup.c
           neighbor_update.c neighbor_findentry.c neighbor_out.c)

  # Link layer specific support
  if(CONFIG_NET_ETHERNET)
    list(APPEND SRCS neighbor_ethernet_out.c)
  endif()

  # if (CONFIG_NET_6LOWPAN) list(APPEND SRCS neighbor_6lowpan_out.c) endif()

  if(CONFIG_DEBUG_NET_INFO)
    list(APPEND SRCS neighbor_dumpentry.c)
  endif()

  if(CONFIG_NETLINK_ROUTE)
    list(APPEND SRCS neighbor_snapshot.c)
  endif()

  target_sources(net PRIVATE ${SRCS})
endif()
