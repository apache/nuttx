############################################################################
# drivers/wireless/CMakeLists.txt
#
#   Copyright (C) 2011-2012, 2016 Gregory Nutt. All rights reserved.
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

if("${CONFIG_DRIVERS_WIRELESS}" STREQUAL "y")

	set(SRCS)

	# Include IEEE 802.15.4 support
	
	if("${CONFIG_DRIVERS_IEEE802154}" STREQUAL "y")
		add_subdirectory(ieee802154)
	endif()
	
	# Include IEEE 802.11 support
	
	if("${CONFIG_DRIVERS_IEEE80211}" STREQUAL "y")
		add_subdirectory(ieee80211)
	endif()
	
	# Include Bluetooth support
	
	if("${CONFIG_DRIVERS_BLUETOOTH}" STREQUAL "y")
		add_subdirectory(bluetooth)
	endif()
	
	# Include wireless drivers
	
	if("${CONFIG_WL_CC1101}" STREQUAL "y")
		list(APPEND SRCS cc1101.c ISM1_868MHzGFSK100kbps.c ISM2_905MHzGFSK250kbps.c)
		list(APPEND SRCS ISM2_433MHzMSK500kbps.c)
	endif()
	
	if("${CONFIG_WL_SPIRIT}" STREQUAL "y")
		add_subdirectory(spirit)
	endif()
	
	if("${CONFIG_WL_NRF24L01}" STREQUAL "y")
		list(APPEND SRCS nrf24l01.c)
	endif()
	
	# Include wireless devices build support

	nuttx_add_library(drivers_wireless ${SRCS})

endif()
