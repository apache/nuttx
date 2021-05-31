############################################################################
# net/socket/CMakeLists.txt
#
#   Copyright (C) 2014-2015, 2017, 2018 Gregory Nutt. All rights reserved.
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

# Include socket source files

set(SRCS
  bind.c connect.c getsockname.c getpeername.c
  recv.c recvfrom.c send.c sendto.c
  socket.c net_close.c
  recvmsg.c sendmsg.c
  net_dup2.c net_sockif.c net_poll.c net_vfcntl.c
  net_fstat.c)

# TCP/IP support or Local Unix domain support
if (CONFIG_NET_TCP OR CONFIG_NET_LOCAL_STREAM)
  list(APPEND SRCS listen.c accept.c)
endif()

# Socket options

if (CONFIG_NET_SOCKOPTS)
  list(APPEND SRCS setsockopt.c getsockopt.c net_timeo.c)
endif()

# Support for sendfile()

if (CONFIG_NET_SENDFILE)
  list(APPEND SRCS net_sendfile.c)
endif()

target_sources(net PRIVATE ${SRCS})
