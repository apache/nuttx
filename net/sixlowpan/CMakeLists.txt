############################################################################
# net/sixlowpan/CMakeLists.txt
#
#   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

# IEEE 802.15.4 support

if("${CONFIG_NET_6LOWPAN}" STREQUAL "y")

	set(CSRCS)

	# Include IEEE 802.15.4 file in the build
	
	list(APPEND CSRCS sixlowpan_initialize.c sixlowpan_globals.c sixlowpan_utils.c)
	list(APPEND CSRCS sixlowpan_input.c sixlowpan_framer.c sixlowpan_framelist.c)
	list(APPEND CSRCS sixlowpan_reassbuf.c)
	
	if("${CONFIG_NET_TCP}" STREQUAL "y")
		list(APPEND CSRCS sixlowpan_tcpsend.c)
	endif()
	
	if("${CONFIG_NET_UDP}" STREQUAL "y")
		list(APPEND CSRCS sixlowpan_udpsend.c)
	endif()
	
	if("${CONFIG_NET_ICMPv6}" STREQUAL "y")
		list(APPEND CSRCS sixlowpan_icmpv6send.c)
	endif()
	
	if("${CONFIG_NET_UDP}" STREQUAL "y")
		list(APPEND CSRCS sixlowpan_send.c)
	elseif("${CONFIG_NET_ICMPv6}" STREQUAL "y")
		list(APPEND CSRCS sixlowpan_send.c)
	endif()
	
	if("${CONFIG_NET_6LOWPAN_COMPRESSION_HC1}" STREQUAL "y")
		list(APPEND CSRCS sixlowpan_hc1.c)
	endif()
	
	if("${CONFIG_NET_6LOWPAN_COMPRESSION_HC06}" STREQUAL "y")
		list(APPEND CSRCS sixlowpan_hc06.c)
	endif()
	
	# Include the sixlowpan directory in the build

	set_parent_srcs(${CSRCS})

endif() # CONFIG_NET_6LOWPAN
