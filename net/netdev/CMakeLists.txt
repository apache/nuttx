# ##############################################################################
# net/netdev/CMakeLists.txt
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

set(SRCS
    netdev_register.c
    netdev_ioctl.c
    netdev_txnotify.c
    netdev_findbyname.c
    netdev_findbyaddr.c
    netdev_findbyindex.c
    netdev_count.c
    netdev_ifconf.c
    netdev_foreach.c
    netdev_unregister.c
    netdev_carrier.c
    netdev_default.c
    netdev_verify.c
    netdev_lladdrsize.c)

if(CONFIG_MM_IOB)
  list(APPEND SRCS netdev_input.c netdev_iob.c)
endif()

if(CONFIG_NETDOWN_NOTIFIER)
  list(APPEND SRCS netdown_notifier.c)
endif()

if(CONFIG_NET_IPv6)
  list(APPEND SRCS netdev_ipv6.c)
endif()

if(CONFIG_NETDEV_STATISTICS)
  list(APPEND SRCS netdev_stats.c)
endif()

if(CONFIG_NETDEV_RSS)
  list(APPEND SRCS netdev_notify_recvcpu.c)
endif()

target_sources(net PRIVATE ${SRCS})
