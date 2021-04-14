##make -f Makefile.cmake stm32f769i-disco/nsh##########################################################################
# arch/arm/src/stm32f7/CMakeLists.txt
#
#   Copyright (C) 2015, 2018 Gregory Nutt. All rights reserved.
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

# Required STM32F7 files

set(CHIP_CSRCS)
list(APPEND CHIP_CSRCS stm32_allocateheap.c stm32_exti_gpio.c stm32_gpio.c)
list(APPEND CHIP_CSRCS stm32_irq.c stm32_lowputc.c stm32_rcc.c stm32_serial.c)
list(APPEND CHIP_CSRCS stm32_start.c stm32_capture.c stm32_flash.c)

if(NOT "${CONFIG_SCHED_TICKLESS}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_timerisr.c)
endif()

if("${CONFIG_BUILD_PROTECTED}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_userspace.c stm32_mpuinit.c)
endif()

if("${CONFIG_ARMV7M_DTCM}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_dtcm.c)
	if("${CONFIG_STM32F7_DTCM_PROCFS}" STREQUAL "y")
		list(APPEND CHIP_CSRCS stm32_procfs_dtcm.c)
	endif()
endif()

if("${CONFIG_STM32F7_DMA}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_dma.c)
endif()

if("${CONFIG_PM}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_pmstandby.c stm32_pmstop.c stm32_pmsleep.c)
	if(NOT "${CONFIG_ARCH_CUSTOM_PMINIT}" STREQUAL "y")
		list(APPEND CHIP_CSRCS stm32_pminitialize.c)
	endif()
endif()

if("${CONFIG_STM32F7_PWR}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_pwr.c stm32_exti_pwr.c)
endif()

if("${CONFIG_STM32F7_RTC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_rtc.c)
	if("${CONFIG_RTC_ALARM}" STREQUAL "y")
		list(APPEND CHIP_CSRCS stm32_exti_alarm.c)
	endif()
	if("${CONFIG_RTC_PERIODIC}" STREQUAL "y")
		list(APPEND CHIP_CSRCS stm32_exti_wakeup.c)
	endif()
	if("${CONFIG_RTC_DRIVER}" STREQUAL "y")
		list(APPEND CHIP_CSRCS stm32_rtc_lowerhalf.c)
	endif()
endif()


if(("${CONFIG_STM32F7_IWDG}" STREQUAL "y") OR ("${CONFIG_STM32F7_RTC_LSICLOCK}" STREQUAL "y"))
	list(APPEND CHIP_CSRCS stm32_lsi.c)
endif()

if("${CONFIG_STM32F7_RTC_LSECLOCK}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_lse.c)
endif()

if("${CONFIG_STM32F7_I2C}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_i2c.c)
endif()

if("${CONFIG_STM32F7_SPI}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_spi.c)
endif()

if("${CONFIG_STM32F7_SDMMC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_sdmmc.c)
endif()

if("${CONFIG_USBDEV}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_otgdev.c)
endif()

if("${CONFIG_USBHOST}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_otghost.c)
endif()

if("${CONFIG_STM32F7_TIM}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_tim.c)
endif()

if("${CONFIG_STM32F7_ADC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_adc.c)
endif()

if("${CONFIG_STM32F7_RTC}" STREQUAL "y")
	if("${CONFIG_RTC_ALARM}" STREQUAL "y")
		list(APPEND CHIP_CSRCS stm32_exti_alarm.c)
	endif()
endif()

if("${CONFIG_STM32F7_ETHMAC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_ethernet.c)
endif()

if("${CONFIG_DEBUG_FEATURES}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_dumpgpio.c)
endif()

if("${CONFIG_STM32F7_BBSRAM}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_bbsram.c)
endif()

if("${CONFIG_STM32F7_RNG}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_rng.c)
endif()

if("${CONFIG_STM32F7_LTDC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_ltdc.c)
endif()

if("${CONFIG_STM32F7_DMA2D}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_dma2d.c)
endif()

if("${CONFIG_SENSORS_QENCODER}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_qencoder.c)
endif()

set_parent_srcs(${CHIP_CSRCS})
