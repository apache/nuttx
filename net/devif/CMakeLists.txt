# ##############################################################################
# net/devif/CMakeLists.txt
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

set(SRCS devif_initialize.c devif_callback.c)

# Device driver IP packet receipt interfaces

if(CONFIG_MM_IOB)

  list(APPEND SRCS devif_send.c devif_loopback.c)

  if(CONFIG_NET_IPv4)
    list(APPEND SRCS ipv4_input.c)
  endif()

  if(CONFIG_NET_IPv6)
    list(APPEND SRCS ipv6_input.c)
  endif()

  # IP forwarding

  if(CONFIG_NET_IPFORWARD)
    list(APPEND SRCS devif_forward.c)
  endif()

  list(APPEND SRCS devif_poll.c devif_iobsend.c devif_filesend.c)

endif()

target_sources(net PRIVATE ${SRCS})
