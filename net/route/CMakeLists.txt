############################################################################
# net/route/CMakeLists.txt
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

if("${CONFIG_NET_ROUTE}" STREQUAL "y")

	set(CSRCS)

	# General routing table support
	
	list(APPEND CSRCS net_initroute.c net_router.c netdev_router.c)
	
	# Support in-memory, RAM-based routing tables
	
	if("${CONFIG_ROUTE_IPv4_RAMROUTE}" STREQUAL "y")
		list(APPEND CSRCS net_alloc_ramroute.c  net_add_ramroute.c net_del_ramroute.c)
		list(APPEND CSRCS net_queue_ramroute.c net_foreach_ramroute.c)
	elseif("${CONFIG_ROUTE_IPv6_RAMROUTE}" STREQUAL "y")
		list(APPEND CSRCS net_alloc_ramroute.c  net_add_ramroute.c net_del_ramroute.c)
		list(APPEND CSRCS net_queue_ramroute.c net_foreach_ramroute.c)
	endif()
	
	# Support for in-memory, read-only (ROM) routing tables
	
	if("${CONFIG_ROUTE_IPv4_ROMROUTE}" STREQUAL "y")
		list(APPEND CSRCS net_foreach_romroute.c)
	elseif("${CONFIG_ROUTE_IPv6_ROMROUTE}" STREQUAL "y")
		list(APPEND CSRCS net_foreach_romroute.c)
	endif()
	
	# Support for routing tables in files
	
	if("${CONFIG_ROUTE_IPv4_FILEROUTE}" STREQUAL "y")
		list(APPEND CSRCS net_fileroute.c net_add_fileroute.c net_del_fileroute.c)
		list(APPEND CSRCS net_foreach_fileroute.c)
	elseif("${CONFIG_ROUTE_IPv6_FILEROUTE}" STREQUAL "y")
		list(APPEND CSRCS net_fileroute.c net_add_fileroute.c net_del_fileroute.c)
		list(APPEND CSRCS net_foreach_fileroute.c)
	endif()
	
	# In-memory cache for file-based routing tables
	
	if("${CONFIG_ROUTE_IPv4_CACHEROUTE}" STREQUAL "y")
		list(APPEND CSRCS net_cacheroute.c)
	elseif("${CONFIG_ROUTE_IPv6_CACHEROUTE}" STREQUAL "y")
		list(APPEND CSRCS net_cacheroute.c)
	endif()
	
	if("${CONFIG_DEBUG_NET_INFO}" STREQUAL "y")
		list(APPEND CSRCS net_dumproute.c)
	endif()
	
	# Include routing table build support

	set_parent_srcs(${CSRCS})

endif()
