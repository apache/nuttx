# ##############################################################################
# net/udp/CMakeLists.txt
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

# UDP source files

if(CONFIG_NET_UDP AND NOT CONFIG_NET_UDP_NO_STACK)
  # Socket layer

  set(SRCS udp_recvfrom.c)

  if(CONFIG_NET_UDPPROTO_OPTIONS)
    list(APPEND SRCS udp_setsockopt.c)
  endif()

  if(CONFIG_NET_UDP_WRITE_BUFFERS)
    list(APPEND SRCS udp_sendto_buffered.c)
  else()
    list(APPEND SRCS udp_sendto_unbuffered.c)
  endif()

  if(CONFIG_NET_UDP_NOTIFIER)
    list(APPEND SRCS udp_notifier.c)

    if(CONFIG_NET_UDP_WRITE_BUFFERS)
      list(APPEND SRCS udp_txdrain.c)
    endif()
  endif()

  # Transport layer

  list(
    APPEND
    SRCS
    udp_conn.c
    udp_devpoll.c
    udp_send.c
    udp_input.c
    udp_finddev.c
    udp_close.c
    udp_callback.c
    udp_ipselect.c
    udp_netpoll.c
    udp_ioctl.c)

  # UDP write buffering

  if(CONFIG_NET_UDP_WRITE_BUFFERS)
    list(APPEND SRCS udp_wrbuffer.c)

    if(CONFIG_DEBUG_FEATURES)
      list(APPEND SRCS udp_wrbuffer_dump.c)
    endif()
  endif()

  target_sources(net PRIVATE ${SRCS})
endif()
