############################################################################
# net/procfs/CMakeLists.txt
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

# Network procfs support

if(NOT "${CONFIG_DISABLE_MOUNTPOINT}" STREQUAL "y")
	if("${CONFIG_FS_PROCFS}" STREQUAL "y")
		if(NOT "${CONFIG_FS_PROCFS_EXCLUDE_NET}" STREQUAL "y")
		
			set(CSRCS)

			list(APPEND CSRCS net_procfs.c netdev_statistics.c)
			
			# General network statistics
	
			if("${CONFIG_NET_STATISTICS}" STREQUAL "y")
			  list(APPEND CSRCS net_statistics.c)
				if("${CONFIG_NET_MLD}" STREQUAL "y")
					list(APPEND CSRCS net_mld.c)
				endif()
			endif()
			
			# Routing table
			
			if("${CONFIG_NET_ROUTE}" STREQUAL "y")
				list(APPEND CSRCS net_procfs_route.c)
			endif()
			
			# Include packet socket build support
			
			set_parent_srcs(${CSRCS})

		endif() # CONFIG_FS_PROCFS_EXCLUDE_NET
	endif() # CONFIG_FS_PROCFS
endif() # CONFIG_DISABLE_MOUNTPOINT
