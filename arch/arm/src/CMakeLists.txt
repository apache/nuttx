############################################################################
# arch/arm/src/CMakeLists.txt
#
#   Copyright (C) 2007-2009, 2011-2012, 2014-2016 Gregory Nutt. All rights
#     reserved.
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

set(ARM_SUBDIR)

if("${CONFIG_ARCH_CORTEXA5}" STREQUAL "y")        # Cortex-A5 is ARMv7-A
	set(ARM_SUBDIR armv7-a)
elseif("${CONFIG_ARCH_CORTEXA8}" STREQUAL "y")   # Cortex-A8 is ARMv7-A
	set(ARM_SUBDIR armv7-a)
elseif("${CONFIG_ARCH_CORTEXA9}" STREQUAL "y")   # Cortex-A9 is ARMv7-A
	set(ARM_SUBDIR armv7-a)
elseif("${CONFIG_ARCH_CORTEXR4}" STREQUAL "y")   # Cortex-R4 is ARMv7-R
	set(ARM_SUBDIR armv7-r)
elseif("${CONFIG_ARCH_CORTEXR4F}" STREQUAL "y")  # Cortex-R4F is ARMv7-R
	set(ARM_SUBDIR armv7-r)
elseif("${CONFIG_ARCH_CORTEXR5}" STREQUAL "y")   # Cortex-R5 is ARMv7-R
	set(ARM_SUBDIR armv7-r)
elseif("${CONFIG_ARCH_CORTEXR5F}" STREQUAL "y")  # Cortex-R5F is ARMv7-R
	set(ARM_SUBDIR armv7-r)
elseif("${CONFIG_ARCH_CORTEXR7}" STREQUAL "y")   # Cortex-R7 is ARMv7-R
	set(ARM_SUBDIR armv7-r)
elseif("${CONFIG_ARCH_CORTEXR7F}" STREQUAL "y")  # Cortex-R7F is ARMv7-R
	set(ARM_SUBDIR armv7-r)
elseif("${CONFIG_ARCH_CORTEXM3}" STREQUAL "y")   # Cortex-M3 is ARMv7-M
	set(ARM_SUBDIR armv7-m)
elseif("${CONFIG_ARCH_CORTEXM4}" STREQUAL "y")   # Cortex-M4 is ARMv7E-M
	set(ARM_SUBDIR armv7-m)
elseif("${CONFIG_ARCH_CORTEXM7}" STREQUAL "y")   # Cortex-M4 is ARMv7E-M
	set(ARM_SUBDIR armv7-m)
elseif("${CONFIG_ARCH_CORTEXM0}" STREQUAL "y")   # Cortex-M0 is ARMv6-M
	set(ARM_SUBDIR armv6-m)
else()                                    # ARM9, ARM7TDMI
	set(ARM_SUBDIR arm)
endif()

include_directories(${CMAKE_BINARY_DIR}/include)

# TODO: use linking dependencies
include_directories(${ARM_SUBDIR})
include_directories(${CONFIG_ARCH_CHIP})
include_directories(common)
include_directories(${CMAKE_SOURCE_DIR}/sched)

set(SRCS)
add_subdirectory(${ARM_SUBDIR})
add_subdirectory(${CONFIG_ARCH_CHIP})
add_subdirectory(common)


# config board
#add_subdirectory(${NUTTX_DIR}/configs/${NUTTX_BOARD}/src board)


# create arch library
nuttx_add_library(arch ${SRCS})
