############################################################################
# net/utils/CMakeLists.txt
#
#   Copyright (C) 2014, 2017 Gregory Nutt. All rights reserved.
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

# Common utilities

set(SRCS
  net_dsec2tick.c net_dsec2timeval.c net_timeval2dsec.c
  net_chksum.c net_ipchksum.c net_incr32.c net_lock.c)

# IPv6 utilities

if (CONFIG_NET_IPv6)
  list(APPEND SRCS net_ipv6_maskcmp.c net_ipv6_mask2pref.c net_ipv6_pref2mask.c)
endif()

# TCP utilities

if (CONFIG_NET_TCP)
  list(APPEND SRCS net_tcpchksum.c)
endif()

# UDP utilities

if (CONFIG_NET_UDP)
  list(APPEND SRCS net_udpchksum.c)
endif()

# ICMP utilities

if (CONFIG_NET_ICMP)
  list(APPEND SRCS net_icmpchksum.c)
elseif (CONFIG_NET_ICMPv6)
  list(APPEND SRCS net_icmpchksum.c)
endif()

target_sources(net PRIVATE ${SRCS})
