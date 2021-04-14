############################################################################
# drivers/analog/CMakeLists.txt
#
#   Copyright (C) 2011-2012, 2017 Gregory Nutt. All rights reserved.
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

# Don't build anything if there is no support for analog devices
set(CSRCS)

# Check for DAC devices

if("${CONFIG_DAC}" STREQUAL "y")

	# Include the common DAC character driver
	
	list(APPEND CSRCS dac.c)
	
	# Include DAC device drivers
	
	if("${CONFIG_DAC_AD5410}" STREQUAL "y")
		list(APPEND CSRCS ad5410.c)
	endif()
	
	if("${CONFIG_DAC7571}" STREQUAL "y")
		list(APPEND CSRCS dac7571.c)
	endif()
endif()
	
# Check for COMP devices

if("${CONFIG_COMP}" STREQUAL "y")
	
	# Include the common COMP character driver
	
	list(APPEND CSRCS comp.c)
	
endif()
	
# Check for OPAMP devices
	
if("${CONFIG_OPAMP}" STREQUAL "y")
	
	# Include the common OPAMP character driver
	
	list(APPEND CSRCS opamp.c)
	
endif()
	
# Check for ADC devices
	
if("${CONFIG_ADC}" STREQUAL "y")
	
	# Include the common ADC character driver
	
	list(APPEND CSRCS adc.c)
	
	# Amplifiers/multiplexers
	
	if("${CONFIG_ADC_PGA11X}" STREQUAL "y")
		list(APPEND CSRCS pga11x.c)
	endif()
	
	# Include ADC device drivers
	
	if("${CONFIG_ADC_ADS1242}" STREQUAL "y")
		list(APPEND CSRCS ads1242.c)
	endif()
	
	if("${CONFIG_ADC_ADS125X}" STREQUAL "y")
		list(APPEND CSRCS ads1255.c)
	endif()
	
	if("${CONFIG_ADC_LTC1867L}" STREQUAL "y")
		list(APPEND CSRCS ltc1867l.c)
	endif()
endif()
	
# Add analog driver build support (the nested if-then-else implements an OR)

if("${CONFIG_DAC}" STREQUAL "y")
	set_parent_srcs(${CSRCS})
else()
	if("${CONFIG_ADC}" STREQUAL "y")
		set_parent_srcs(${CSRCS})
	else()
		if("${CONFIG_COMP}" STREQUAL "y")
			set_parent_srcs(${CSRCS})
		else()
			if("${CONFIG_OPAMP}" STREQUAL "y")
				set_parent_srcs(${CSRCS})
			endif()
		endif()
	endif()
endif()
