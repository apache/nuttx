############################################################################
# drivers/ioexpander/CMakeLists.txt
#
#   Copyright (C) 2015-2016 Gregory Nutt. All rights reserved.
#   Author: Sebastien Lorquet <sebastien@lorquet.fr>
#           Gregory Nutt <gnutt@nuttx.org>
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

# Check if I/O expander support is enabled

set(CSRCS)

if("${CONFIG_IOEXPANDER}" STREQUAL "y")

	# Include the selected I/O expander drivers

	if("${CONFIG_IOEXPANDER_PCA9555}" STREQUAL "y")
		list(APPEND CSRCS pca9555.c)
	endif()

	if("${CONFIG_IOEXPANDER_TCA64XX}" STREQUAL "y")
		list(APPEND CSRCS tca64xx.c)
	endif()

	if("${CONFIG_IOEXPANDER_PCF8574}" STREQUAL "y")
		list(APPEND CSRCS pcf8574.c)
	endif()

endif() # CONFIG_IOEXPANDER

# GPIO test device driver (independent of IOEXPANDERS)

if("${CONFIG_DEV_GPIO}" STREQUAL "y")
	list(APPEND CSRCS gpio.c)
	if("${CONFIG_GPIO_LOWER_HALF}" STREQUAL "y")
		list(APPEND CSRCS gpio_lower_half.c)
	endif()
endif()

# The folling implements an awkward OR

if("${CONFIG_IOEXPANDER}" STREQUAL "y")

	# Include ioexpander I/O device driver build support

	set_parent_srcs(${CSRCS})

elseif("${CONFIG_DEV_GPIO}" STREQUAL "y")

	# Include ioexpander I/O device driver build support

	set_parent_srcs(${CSRCS})

endif()
