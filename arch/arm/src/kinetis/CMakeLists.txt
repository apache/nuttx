############################################################################
# arch/arm/src/kinetis/CMakeLists.txt
#
#   Copyright (C) 2011, 2013-2018 Gregory Nutt. All rights reserved.
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
list(APPEND CHIP_CSRCS kinetis_allocateheap.c kinetis_clockconfig.c kinetis_clrpend.c)
list(APPEND CHIP_CSRCS kinetis_irq.c kinetis_lowputc.c kinetis_pin.c kinetis_pingpio.c)
list(APPEND CHIP_CSRCS kinetis_serialinit.c kinetis_start.c kinetis_uid.c kinetis_wdog.c)
list(APPEND CHIP_CSRCS kinetis_cfmconfig.c kinetis_mpuinit.c)

# Configuration-dependent Kinetis K files

if(NOT "${CONFIG_ARCH_IDLE_CUSTOM}" STREQUAL "y")
	list(APPEND CHIP_CSRCS kinetis_idle.c)
endif()

if(NOT "${CONFIG_SCHED_TICKLESS}" STREQUAL "y")
	list(APPEND CHIP_CSRCS kinetis_timerisr.c)
endif()

if("${CONFIG_BUILD_PROTECTED}" STREQUAL "y")
	list(APPEND CHIP_CSRCS kinetis_userspace.c)
endif()

if("${CONFIG_KINETIS_GPIOIRQ}" STREQUAL "y")
	list(APPEND CHIP_CSRCS kinetis_pinirq.c)
endif()

if("${CONFIG_DEBUG_GPIO_INFO}" STREQUAL "y")
	list(APPEND CHIP_CSRCS kinetis_pindump.c)
endif()

if("${CONFIG_KINETIS_UART}" STREQUAL "y")
	list(APPEND CHIP_CSRCS kinetis_serial.c)
endif()

if("${CONFIG_KINETIS_LPUART}" STREQUAL "y")
	list(APPEND CHIP_CSRCS kinetis_lpserial.c)
endif()

if("${CONFIG_KINETIS_SDHC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS kinetis_sdhc.c)
endif()

if("${CONFIG_SPI}" STREQUAL "y")
	list(APPEND CHIP_CSRCS kinetis_spi.c)
endif()

if("${CONFIG_USBDEV}" STREQUAL "y")
	list(APPEND CHIP_CSRCS kinetis_usbdev.c)
endif()

if("${CONFIG_USBHOST}" STREQUAL "y")
	list(APPEND CHIP_CSRCS kinetis_usbhost.c)
endif()

if("${CONFIG_KINETIS_DMA}" STREQUAL "y")
	list(APPEND CHIP_CSRCS kinetis_dma.c kinetis_pindma.c)
endif()

if("${CONFIG_PWM}" STREQUAL "y")
	list(APPEND CHIP_CSRCS kinetis_pwm.c)
endif()

if("${CONFIG_I2C}" STREQUAL "y")
	list(APPEND CHIP_CSRCS kinetis_i2c.c)
endif()

if("${CONFIG_RTC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS kinetis_rtc.c)
	if("${CONFIG_RTC_DRIVER}" STREQUAL "y")
		list(APPEND CHIP_CSRCS kinetis_rtc_lowerhalf.c)
	endif()
endif()

if("${CONFIG_NET}" STREQUAL "y")
	if("${CONFIG_KINETIS_ENET}" STREQUAL "y")
		list(APPEND CHIP_CSRCS kinetis_enet.c)
	endif()
endif()

set_parent_srcs(${CHIP_CSRCS})
