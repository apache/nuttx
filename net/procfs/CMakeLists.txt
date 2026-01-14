# ##############################################################################
# net/procfs/CMakeLists.txt
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

# Network procfs support

if(NOT CONFIG_DISABLE_MOUNTPOINT
   AND CONFIG_FS_PROCFS
   AND NOT CONFIG_FS_PROCFS_EXCLUDE_NET)

  set(SRCS net_procfs.c netdev_statistics.c)

  # General network statistics

  if(CONFIG_NET_STATISTICS)
    list(APPEND SRCS net_statistics.c)
    if(CONFIG_NET_MLD)
      list(APPEND SRCS net_mld.c)
    endif()
    if(CONFIG_NET_TCP)
      list(APPEND SRCS net_tcp.c)
    endif()
    if(CONFIG_NET_UDP)
      list(APPEND SRCS net_udp.c)
    endif()
  endif()

  # Routing table

  if(CONFIG_NET_ROUTE)
    list(APPEND SRCS net_procfs_route.c)
  endif()

  target_sources(net PRIVATE ${SRCS})
endif()
