############################################################################
# net/udp/CMakeLists.txt
#
#   Copyright (C) 2014-2015, 2018 Gregory Nutt. All rights reserved.
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

# UDP source files

if (CONFIG_NET_UDP AND NOT CONFIG_NET_UDP_NO_STACK)
  # Socket layer

  set(SRCS udp_recvfrom.c)

  if (CONFIG_NET_UDPPROTO_OPTIONS)
    list(APPEND SRCS udp_setsockopt.c)
  endif()

  if (CONFIG_NET_UDP_WRITE_BUFFERS)
    list(APPEND SRCS udp_sendto_buffered.c)
  else()
    list(APPEND SRCS udp_sendto_unbuffered.c)
  endif()

  if (CONFIG_NET_UDP_NOTIFIER)
    list(APPEND SRCS udp_notifier.c)

    if (CONFIG_NET_UDP_WRITE_BUFFERS)
      list(APPEND SRCS udp_txdrain.c)
    endif()
  endif()

  # Transport layer

  list(APPEND SRCS
    udp_conn.c udp_devpoll.c udp_send.c udp_input.c udp_finddev.c
    udp_close.c udp_callback.c udp_ipselect.c udp_netpoll.c)

  # UDP write buffering

  if (CONFIG_NET_UDP_WRITE_BUFFERS)
    list(APPEND SRCS udp_wrbuffer.c)

    if (CONFIG_DEBUG_FEATURES)
      list(APPEND SRCS udp_wrbuffer_dump.c)
    endif()
  endif()

  target_sources(net PRIVATE ${SRCS})
endif()
