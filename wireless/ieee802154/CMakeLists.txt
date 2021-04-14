############################################################################
# wireless/ieee802145/Make.defs
#
#   Copyright (C) 2016-2017 Gregory Nutt. All rights reserved.
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

if("${CONFIG_WIRELESS_IEEE802154}" STREQUAL "y")

	# Include IEEE 802.15.4 support
	set(CSRCS)
	
	list(APPEND CSRCS ieee802154_primitive.c)
	
	if("${CONFIG_IEEE802154_MAC}" STREQUAL "y")
	
		list(APPEND CSRCS mac802154.c  mac802154_assoc.c mac802154_disassoc.c)
		list(APPEND CSRCS mac802154_bind.c mac802154_data.c mac802154_get_mhrlen.c)
		list(APPEND CSRCS mac802154_getset.c mac802154_gts.c mac802154_ioctl.c)
		list(APPEND CSRCS mac802154_orphan.c mac802154_poll.c mac802154_purge.c)
		list(APPEND CSRCS mac802154_reset.c mac802154_rxenable.c mac802154_scan.c)
		list(APPEND CSRCS mac802154_start.c mac802154_sync.c)
		
		# Include wireless devices build support
		
		if("${CONFIG_IEEE802154_MACDEV}" STREQUAL "y")
			list(APPEND CSRCS mac802154_device.c)
		endif()
		
		if("${CONFIG_IEEE802154_NETDEV}" STREQUAL "y")
			list(APPEND CSRCS mac802154_netdev.c)
		endif()
	
	endif()
	
	if("${CONFIG_IEEE802154_LOOPBACK}" STREQUAL "y")
		list(APPEND CSRCS mac802154_loopback.c)
	endif()
	
	set_parent_srcs(${CSRCS})

endif() # CONFIG_WIRELESS_IEEE802154
