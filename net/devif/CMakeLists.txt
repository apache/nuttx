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

set(CSRCS)

# Network device interface source files

list(APPEND CSRCS devif_initialize.c devif_send.c devif_poll.c devif_callback.c)
list(APPEND CSRCS devif_loopback.c)

# Device driver IP packet receipt interfaces

if("${CONFIG_NET_IPv4}" STREQUAL "y")
	list(APPEND CSRCS ipv4_input.c)
endif()

if("${CONFIG_NET_IPv6}" STREQUAL "y")
	list(APPEND CSRCS ipv6_input.c)
endif()

# IP forwarding

if("${CONFIG_NET_IPFORWARD}" STREQUAL "y")
	list(APPEND CSRCS devif_forward.c)
endif()

# I/O buffer chain support required?

if("${CONFIG_MM_IOB}" STREQUAL "y")
	list(APPEND CSRCS devif_iobsend.c)
endif()

# Raw packet socket support

if("${CONFIG_NET_PKT}" STREQUAL "y")
	list(APPEND CSRCS devif_pktsend.c)
endif()

# Include network device interface build support

set_parent_srcs(${CSRCS})
