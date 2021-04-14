############################################################################
# drivers/input/CMakeLists.txt
#
#   Copyright (C) 2011-2012, 2015 Gregory Nutt. All rights reserved.
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

# Don't build anything if there is no support for input devices

if("${CONFIG_INPUT}" STREQUAL "y")

	set(CSRCS)

	# Include the selected touchscreen drivers
	
	if("${CONFIG_INPUT_TSC2007}" STREQUAL "y")
		list(APPEND CSRCS tsc2007.c)
	endif()
	
	if("${CONFIG_INPUT_FT5X06}" STREQUAL "y")
		list(APPEND CSRCS ft5x06.c)
	endif()
	
	if("${CONFIG_INPUT_ADS7843E}" STREQUAL "y")
		list(APPEND CSRCS ads7843e.c)
	endif()
	
	if("${CONFIG_INPUT_MAX11802}" STREQUAL "y")
		list(APPEND CSRCS max11802.c)
	endif()
	
	if("${CONFIG_INPUT_MXT}" STREQUAL "y")
		list(APPEND CSRCS mxt.c)
	endif()
	
	if("${CONFIG_INPUT_STMPE811}" STREQUAL "y")
		list(APPEND CSRCS stmpe811_base.c)
		if(NOT "${CONFIG_INPUT_STMPE811_TSC_DISABLE}" STREQUAL "y")
			list(APPEND CSRCS stmpe811_tsc.c)
		endif()
		if(NOT "${CONFIG_INPUT_STMPE811_GPIO_DISABLE}" STREQUAL "y")
			list(APPEND CSRCS stmpe811_gpio.c)
		endif()
		if(NOT "${CONFIG_INPUT_STMPE811_ADC_DISABLE}" STREQUAL "y")
			list(APPEND CSRCS stmpe811_adc.c)
		endif()
		if(NOT "${CONFIG_INPUT_STMPE811_TEMP_DISABLE}" STREQUAL "y")
			list(APPEND CSRCS stmpe811_temp.c)
		endif()
	endif()
	
	if("${CONFIG_INPUT_CYPRESS_MBR3108}" STREQUAL "y")
		list(APPEND CSRCS cypress_mbr3108.c)
	endif()
	
	if("${CONFIG_BUTTONS}" STREQUAL "y")
		list(APPEND CSRCS button_upper.c)
		if("${CONFIG_BUTTONS_LOWER}" STREQUAL "y")
			list(APPEND CSRCS button_lower.c)
		endif()
	
	endif()
	
	if("${CONFIG_DJOYSTICK}" STREQUAL "y")
		list(APPEND CSRCS djoystick.c)
	endif()
	
	if("${CONFIG_AJOYSTICK}" STREQUAL "y")
		list(APPEND CSRCS ajoystick.c)
	endif()
	
	if("${CONFIG_INPUT_NUNCHUCK}" STREQUAL "y")
		list(APPEND CSRCS nunchuck.c)
	endif()
	
	# Include input device driver build support

	set_parent_srcs(${CSRCS})
endif()

