############################################################################
# drivers/usbhost/CMakeLists.txt
#
#   Copyright (C) 2010-2015, 2018 Gregory Nutt. All rights reserved.
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

list(APPEND CSRCS hid_parser.c)

if("${CONFIG_USBHOST}" STREQUAL "y")

	# Include built-in USB host driver logic
	
	list(APPEND CSRCS usbhost_registry.c usbhost_registerclass.c usbhost_findclass.c)
	list(APPEND CSRCS usbhost_enumerate.c usbhost_devaddr.c)
	
	if("${CONFIG_USBHOST_HUB}" STREQUAL "y")
		list(APPEND CSRCS usbhost_hub.c)
	endif()
	
	if("${CONFIG_USBHOST_COMPOSITE}" STREQUAL "y")
		list(APPEND CSRCS usbhost_composite.c)
	endif()
	
	if("${CONFIG_USBHOST_MSC}" STREQUAL "y")
		list(APPEND CSRCS usbhost_storage.c)
	endif()
	
	if("${CONFIG_USBHOST_CDCACM}" STREQUAL "y")
		list(APPEND CSRCS usbhost_cdcacm.c)
	endif()
	
	if("${CONFIG_USBHOST_HIDKBD}" STREQUAL "y")
		list(APPEND CSRCS usbhost_hidkbd.c)
	endif()
	
	if("${CONFIG_USBHOST_HIDMOUSE}" STREQUAL "y")
		list(APPEND CSRCS usbhost_hidmouse.c)
	endif()
	
	if("${CONFIG_USBHOST_XBOXCONTROLLER}" STREQUAL "y")
		list(APPEND CSRCS usbhost_xboxcontroller.c)
	endif()
	
	if("${CONFIG_USBHOST_MAX3421E}" STREQUAL "y")
		list(APPEND CSRCS usbhost_max3421e.c)
	endif()
	
	# HCD debug/trace logic
	
	if("${CONFIG_USBHOST_TRACE}" STREQUAL "y")
		list(APPEND CSRCS usbhost_trace.c)
	else()
		if("${CONFIG_DEBUG_USB}" STREQUAL "y")
			list(APPEND CSRCS usbhost_trace.c)
		endif()
	endif()
	
	# Include add-on USB host driver logic if present and selected (availabel in
	# the NuttX drivers GIT repository)
	
	if("${CONFIG_USBHOST_RTL8187}" STREQUAL "y")
		#RTL8187_CSRC := ${shell if [ -f usbhost$(DELIM)rtl8187x.c ]; then echo "rtl8187x.c"; fi}
		if(EXISTS "${CMAKE_CURRENT_SOUCE_DIR}/rtl8187x.c")
			list(APPEND CSRCS rtl8187x.c)
		endif()
	endif()
endif()

# Include USB host driver build logic

set_parent_srcs(${CSRCS})
