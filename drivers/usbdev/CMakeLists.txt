############################################################################
# drivers/usbdev/CMakeLists.txt
#
#   Copyright (C) 2008, 2010-2013 Gregory Nutt. All rights reserved.
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

if("${CONFIG_USBDEV}" STREQUAL "y")

	set(CSRCS)

	# Include USB device drivers
	
	if("${CONFIG_PL2303}" STREQUAL "y")
		list(APPEND CSRCS pl2303.c)
	endif()
	
	if("${CONFIG_CDCACM}" STREQUAL "y")
		list(APPEND CSRCS cdcacm.c cdcacm_desc.c)
	endif()
	
	if("${CONFIG_USBMSC}" STREQUAL "y")
		list(APPEND CSRCS usbmsc.c usbmsc_desc.c usbmsc_scsi.c)
	endif()
	
	if("${CONFIG_USBDEV_COMPOSITE}" STREQUAL "y")
		list(APPEND CSRCS composite.c composite_desc.c)
	endif()
	
	if("${CONFIG_USBDEV_TRACE_STRINGS}" STREQUAL "y")
		list(APPEND CSRCS usbdev_strings.c)
	endif()
	
	if("${CONFIG_RNDIS}" STREQUAL "y")
		list(APPEND CSRCS rndis.c)
	endif()
	
	if("${CONFIG_DFU}" STREQUAL "y")
		list(APPEND CSRCS dfu.c)
	endif()
	
	if("${CONFIG_NET_CDCECM}" STREQUAL "y")
		list(APPEND CSRCS cdcecm.c)
	endif()
	
	list(APPEND CSRCS usbdev_trace.c usbdev_trprintf.c)
	
	# Include USB device build support
	
	set_parent_srcs(${CSRCS})
endif()
