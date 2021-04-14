############################################################################
# net/local/CMakeLists.txt
#
#   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

# Unix domain socket source files

if("${CONFIG_NET_LOCAL}" STREQUAL "y")

	set(CSRCS)

	list(APPEND CSRCS local_conn.c local_release.c local_bind.c local_fifo.c)
	list(APPEND CSRCS local_recvfrom.c local_sendpacket.c local_recvutils.c)
	list(APPEND CSRCS local_sockif.c)
	
	if("${CONFIG_NET_LOCAL_STREAM}" STREQUAL "y")
		list(APPEND CSRCS local_connect.c local_listen.c local_accept.c local_send.c)
	endif()
	
	if("${CONFIG_NET_LOCAL_DGRAM}" STREQUAL "y")
		list(APPEND CSRCS local_sendto.c)
	endif()

	if(NOT "${CONFIG_DISABLE_POLL}" STREQUAL "y")
		list(APPEND CSRCS local_netpoll.c)
	endif()

	# Include Unix domain socket build support

	set_parent_srcs(${CSRCS})

endif() # CONFIG_NET_LOCAL
