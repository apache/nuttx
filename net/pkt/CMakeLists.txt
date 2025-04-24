# ##############################################################################
# net/pkt/CMakeLists.txt
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

# Packet socket support

if(CONFIG_NET_PKT)

  set(SRCS # Socket layer
      pkt_sockif.c
      pkt_recvmsg.c
      # Transport layer
      pkt_conn.c
      pkt_input.c
      pkt_callback.c
      pkt_poll.c
      pkt_netpoll.c
      pkt_finddev.c)

  if(CONFIG_NET_PKT_WRITE_BUFFERS)
    list(APPEND SRCS pkt_sendmsg_buffered.c) # Socket layer
  else()
    list(APPEND SRCS pkt_sendmsg_unbuffered.c) # Socket layer

  endif()

  if(CONFIG_NET_PKTPROTO_OPTIONS)
    list(APPEND SRCS pkt_setsockopt.c pkt_getsockopt.c) # Socket layer
  endif()

  target_sources(net PRIVATE ${SRCS})
endif()
