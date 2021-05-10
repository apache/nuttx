############################################################################
# net/tcp/CMakeLists.txt
#
#   Copyright (C) 2014, 2017-2018 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

# TCP/IP source files

if (CONFIG_NET_TCP AND NOT CONFIG_NET_TCP_NO_STACK)

  # Socket layer

  set(SRCS tcp_connect.c tcp_accept.c tcp_recvfrom.c)

  if (CONFIG_NET_TCP_WRITE_BUFFERS)
    list(APPEND SRCS tcp_send_buffered.c)
  else()
    list(APPEND SRCS tcp_send_unbuffered.c)
  endif()

  if (CONFIG_NET_SENDFILE)
    list(APPEND SRCS tcp_sendfile.c)
  endif()

  if (CONFIG_NET_TCP_NOTIFIER)
    list(APPEND SRCS tcp_notifier.c)

    if (CONFIG_NET_TCP_WRITE_BUFFERS)
    list(APPEND SRCS tcp_txdrain.c)
    endif()
  endif()

  if (CONFIG_NET_TCPPROTO_OPTIONS)
    list(APPEND SRCS tcp_setsockopt.c tcp_getsockopt.c)
  endif()

  # Transport layer

  list(APPEND SRCS
    tcp_conn.c tcp_seqno.c tcp_devpoll.c tcp_finddev.c tcp_timer.c
    tcp_send.c tcp_input.c tcp_appsend.c tcp_listen.c tcp_close.c
    tcp_monitor.c tcp_callback.c tcp_backlog.c tcp_ipselect.c
    tcp_recvwindow.c tcp_netpoll.c)

  # TCP write buffering

  if (CONFIG_NET_TCP_WRITE_BUFFERS)
    list(APPEND SRCS tcp_wrbuffer.c)

    if (CONFIG_DEBUG_FEATURES)
      list(APPEND SRCS tcp_wrbuffer_dump.c)
    endif()
  endif()

  target_sources(net PRIVATE ${SRCS})
endif()
