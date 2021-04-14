############################################################################
# arch/arm/src/kl/CMakeLists.txt
#
#   Copyright (C) 2013-2014 Gregory Nutt. All rights reserved.
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

set(CHIP_CSRCS)
list(APPEND CHIP_CSRCS kl_clockconfig.c kl_gpio.c kl_irq.c kl_lowputc.c)
list(APPEND CHIP_CSRCS kl_serial.c kl_start.c kl_cfmconfig.c)

# Configuration-dependent Kinetis L files

if(NOT "${CONFIG_ARCH_IDLE_CUSTOM}" STREQUAL "y")
	list(APPEND CHIP_CSRCS kl_idle.c)
endif()

if(NOT "${CONFIG_SCHED_TICKLESS}" STREQUAL "y")
	list(APPEND CHIP_CSRCS kl_timerisr.c)
endif()

if("${CONFIG_BUILD_PROTECTED}" STREQUAL "y")
	list(APPEND CHIP_CSRCS kl_userspace.c)
endif()

if("${CONFIG_KL_GPIOIRQ}" STREQUAL "y")
	list(APPEND CHIP_CSRCS kl_gpioirq.c)
endif()

if("${CONFIG_ARCH_IRQPRIO}" STREQUAL "y")
	list(APPEND CHIP_CSRCS kl_irqprio.c)
endif()

if("${CONFIG_KL_SPI0}" STREQUAL "y")
	list(APPEND CHIP_CSRCS kl_spi.c)
else()
	if("${CONFIG_KL_SPI1}" STREQUAL "y")
		list(APPEND CHIP_CSRCS kl_spi.c)
	endif()
endif()

if("${CONFIG_PWM}" STREQUAL "y")
	list(APPEND CHIP_CSRCS kl_pwm.c)
endif()

if("${CONFIG_DEBUG_FEATURES}" STREQUAL "y")
	list(APPEND CHIP_CSRCS kl_dumpgpio.c)
endif()

set_parent_srcs(${CHIP_CSRCS})
