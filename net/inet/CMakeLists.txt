# ##############################################################################
# net/inet/CMakeLists.txt
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

set(SRCS inet_txdrain.c)

if(CONFIG_NET_IPv4)
  list(APPEND SRCS inet_sockif.c inet_globals.c)
elseif(CONFIG_NET_IPv6)
  list(APPEND SRCS inet_sockif.c inet_globals.c)
endif()

if(CONFIG_NET_IPv4)
  list(
    APPEND
    SRCS
    ipv4_setsockopt.c
    ipv4_getsockopt.c
    ipv4_getsockname.c
    ipv4_getpeername.c
    ipv4_build_header.c)
endif()

if(CONFIG_NET_IPv6)
  list(
    APPEND
    SRCS
    ipv6_setsockopt.c
    ipv6_getsockname.c
    ipv6_getpeername.c
    ipv6_build_header.c
    ipv6_getsockopt.c)
endif()

target_sources(net PRIVATE ${SRCS})
