# ##############################################################################
# net/route/CMakeLists.txt
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

if(CONFIG_NET_ROUTE)

  # General routing table support

  set(SRCS net_initroute.c net_router.c netdev_router.c)

  # Support in-memory, RAM-based routing tables

  if(CONFIG_ROUTE_IPv4_RAMROUTE)
    list(
      APPEND
      SRCS
      net_alloc_ramroute.c
      net_add_ramroute.c
      net_del_ramroute.c
      net_queue_ramroute.c
      net_foreach_ramroute.c)
  elseif(CONFIG_ROUTE_IPv6_RAMROUTE)
    list(
      APPEND
      SRCS
      net_alloc_ramroute.c
      net_add_ramroute.c
      net_del_ramroute.c
      net_queue_ramroute.c
      net_foreach_ramroute.c)
  endif()

  # Support for in-memory, read-only (ROM) routing tables

  if(CONFIG_ROUTE_IPv4_ROMROUTE)
    list(APPEND SRCS net_foreach_romroute.c)
  elseif(CONFIG_ROUTE_IPv6_ROMROUTE)
    list(APPEND SRCS net_foreach_romroute.c)
  endif()

  # Support for routing tables in files

  if(CONFIG_ROUTE_IPv4_FILEROUTE)
    list(APPEND SRCS net_fileroute.c net_add_fileroute.c net_del_fileroute.c
         net_foreach_fileroute.c)
  elseif(CONFIG_ROUTE_IPv6_FILEROUTE)
    list(APPEND SRCS net_fileroute.c net_add_fileroute.c net_del_fileroute.c
         net_foreach_fileroute.c)
  endif()

  # In-memory cache for file-based routing tables

  if(CONFIG_ROUTE_IPv4_CACHEROUTE)
    list(APPEND SRCS net_cacheroute.c)
  elseif(CONFIG_ROUTE_IPv6_CACHEROUTE)
    list(APPEND SRCS net_cacheroute.c)
  endif()

  if(CONFIG_DEBUG_NET_INFO)
    list(APPEND SRCS net_dumproute.c)
  endif()

  target_sources(net PRIVATE ${SRCS})
endif()
