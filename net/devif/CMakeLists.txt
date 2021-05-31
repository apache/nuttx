############################################################################
# net/inet/CMakeLists.txt
#
#   Copyright (C) 2007, 2009-2010, 2014-2015, 2017 Gregory Nutt. All rights reserved.
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

set(SRCS
  devif_initialize.c devif_send.c devif_poll.c devif_callback.c
  devif_loopback.c)

# Device driver IP packet receipt interfaces

if (CONFIG_NET_IPv4)
  list(APPEND SRCS ipv4_input.c)
endif()

if (CONFIG_NET_IPv6)
  list(APPEND SRCS ipv6_input.c)
endif()

# IP forwarding

if (CONFIG_NET_IPFORWARD)
  list(APPEND SRCS devif_forward.c)
endif()

# I/O buffer chain support required?

if (CONFIG_MM_IOB)
  list(APPEND SRCS devif_iobsend.c)
endif()

# Raw packet socket support

if (CONFIG_NET_PKT)
  list(APPEND SRCS devif_pktsend.c)
endif()

if (CONFIG_NET_CAN)
  list(APPEND SRCS devif_cansend.c)
endif()

target_sources(net PRIVATE ${SRCS})

