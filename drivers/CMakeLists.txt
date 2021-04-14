############################################################################
# drivers/CMakeLists.txt
#
#   Copyright (C) 2007-2014, 2016 Gregory Nutt. All rights reserved.
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

set(SRCS)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(1wire)
add_subdirectory(analog)
add_subdirectory(audio)
add_subdirectory(bch)
add_subdirectory(can)
add_subdirectory(contactless)
add_subdirectory(crypto)
add_subdirectory(eeprom)
add_subdirectory(i2c)
add_subdirectory(i2s)
add_subdirectory(input)
add_subdirectory(ioexpander)
add_subdirectory(lcd)
add_subdirectory(leds)
add_subdirectory(loop)
add_subdirectory(mmcsd)
add_subdirectory(modem)
add_subdirectory(mtd)
add_subdirectory(net)
add_subdirectory(pipes)
add_subdirectory(power)
add_subdirectory(sensors)
add_subdirectory(serial)
add_subdirectory(spi)
add_subdirectory(syslog)
add_subdirectory(timers)
add_subdirectory(usbdev)
add_subdirectory(usbhost)
add_subdirectory(usbmisc)
add_subdirectory(usbmonitor)
add_subdirectory(video)
add_subdirectory(wireless)

if(NOT "${CONFIG_NFILE_DESCRIPTORS}" STREQUAL "0")

	if(NOT "${CONFIG_DISABLE_MOUNTPOINT}" STREQUAL "y")
		list(APPEND SRCS ramdisk.c)

		if(NOT "${CONFIG_DRVR_WRITEBUFFER}" STREQUAL "y")
			list(APPEND SRCS rwbuffer.c)
		else()
			if("${CONFIG_DRVR_READAHEAD}" STREQUAL "y")
				list(APPEND SRCS rwbuffer.c)
			endif()
		endif()
	endif()
	
	nuttx_add_library(drivers
		dev_null.c
		dev_zero.c
		${SRCS}
	)
	target_include_directories(drivers INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

endif() # CONFIG_NFILE_DESCRIPTORS != 0
