# ##############################################################################
# net/tcp/CMakeLists.txt
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

# TCP/IP source files

if(CONFIG_NET_TCP AND NOT CONFIG_NET_TCP_NO_STACK)

  # Socket layer

  set(SRCS tcp_connect.c tcp_accept.c tcp_recvfrom.c)

  if(CONFIG_NET_TCP_WRITE_BUFFERS)
    list(APPEND SRCS tcp_send_buffered.c)
  else()
    list(APPEND SRCS tcp_send_unbuffered.c)
  endif()

  if(CONFIG_NET_SENDFILE)
    list(APPEND SRCS tcp_sendfile.c)
  endif()

  if(CONFIG_NET_TCP_NOTIFIER)
    list(APPEND SRCS tcp_notifier.c)

    if(CONFIG_NET_TCP_WRITE_BUFFERS)
      list(APPEND SRCS tcp_txdrain.c)
    endif()
  endif()

  if(CONFIG_NET_TCPPROTO_OPTIONS)
    list(APPEND SRCS tcp_setsockopt.c tcp_getsockopt.c)
  endif()

  # Transport layer

  list(
    APPEND
    SRCS
    tcp_conn.c
    tcp_seqno.c
    tcp_devpoll.c
    tcp_finddev.c
    tcp_timer.c
    tcp_send.c
    tcp_input.c
    tcp_appsend.c
    tcp_listen.c
    tcp_close.c
    tcp_monitor.c
    tcp_callback.c
    tcp_backlog.c
    tcp_ipselect.c
    tcp_recvwindow.c
    tcp_netpoll.c
    tcp_ioctl.c
    tcp_shutdown.c)

  # TCP write buffering

  if(CONFIG_NET_TCP_WRITE_BUFFERS)
    list(APPEND SRCS tcp_wrbuffer.c)
  endif()

  # TCP congestion control

  if(CONFIG_NET_TCP_CC_NEWRENO)
    list(APPEND SRCS tcp_cc.c)
  endif()

  # TCP debug

  if(CONFIG_DEBUG_FEATURES)
    list(APPEND SRCS tcp_dump.c)
  endif()

  target_sources(net PRIVATE ${SRCS})
endif()
