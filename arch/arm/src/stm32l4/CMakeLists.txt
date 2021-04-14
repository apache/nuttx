############################################################################
# arch/arm/src/stm32l4/CMakeLists.txt
#
#   Copyright (C) 2017-2018 Gregory Nutt. All rights reserved.
#   Copyright (C) 2015-2016 Sebastien Lorquet. All rights reserved.
#   Author: Sebastien Lorquet <sebastien@lorquet.fr>
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
list(APPEND CHIP_CSRCS stm32l4_allocateheap.c stm32l4_exti_gpio.c stm32l4_gpio.c)
list(APPEND CHIP_CSRCS stm32l4_irq.c stm32l4_lowputc.c stm32l4_rcc.c)
list(APPEND CHIP_CSRCS stm32l4_serial.c stm32l4_start.c stm32l4_waste.c stm32l4_uid.c)
list(APPEND CHIP_CSRCS stm32l4_spi.c stm32l4_i2c.c stm32l4_lse.c stm32l4_lsi.c)
list(APPEND CHIP_CSRCS stm32l4_pwr.c stm32l4_tim.c stm32l4_flash.c)

if(NOT "${CONFIG_ARCH_IDLE_CUSTOM}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32l4_idle.c)
endif()

if("${CONFIG_TIMER}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32l4_tim_lowerhalf.c)
endif()

if(NOT "${CONFIG_SCHED_TICKLESS}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32l4_timerisr.c)
else()
	list(APPEND CHIP_CSRCS stm32l4_tickless.c)
endif()

if("${CONFIG_STM32L4_ONESHOT}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32l4_oneshot.c stm32l4_oneshot_lowerhalf.c)
endif()

if("${CONFIG_STM32L4_FREERUN}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32l4_freerun.c)
endif()

if("${CONFIG_BUILD_PROTECTED}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32l4_userspace.c stm32l4_mpuinit.c)
endif()

if("${CONFIG_STM32L4_ADC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32l4_adc.c)
endif()

if("${CONFIG_STM32L4_DAC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32l4_dac.c)
endif()

if("${CONFIG_STM32L4_DFSDM}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32l4_dfsdm.c)
endif()

if("${CONFIG_STM32L4_DMA}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32l4_dma.c)
endif()

if("${CONFIG_USBDEV}" STREQUAL "y")
	if("${CONFIG_STM32L4_OTGFS}" STREQUAL "y")
		list(APPEND CHIP_CSRCS stm32l4_otgfsdev.c)
	endif()
endif()

if("${CONFIG_USBHOST}" STREQUAL "y")
	if("${CONFIG_STM32L4_OTGFS}" STREQUAL "y")
		list(APPEND CHIP_CSRCS stm32l4_otgfshost.c)
	endif()
endif()

if("${CONFIG_USBHOST}" STREQUAL "y")
	if("${CONFIG_USBHOST_TRACE}" STREQUAL "y")
		list(APPEND CHIP_CSRCS stm32l4_usbhost_trace.c)
	else()
		if("${CONFIG_DEBUG_USB}" STREQUAL "y")
			list(APPEND CHIP_CSRCS stm32l4_usbhost_trace.c)
		endif()
	endif()
endif()

if("${CONFIG_PM}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32l4_pmlpr.c stm32l4_pmsleep.c stm32l4_pmstandby.c)
	list(APPEND CHIP_CSRCS stm32l4_pmstop.c)

	if(NOT "${CONFIG_ARCH_CUSTOM_PMINIT}" STREQUAL "y")
		list(APPEND CHIP_CSRCS stm32l4_pminitialize.c)
	endif()
endif()

if("${CONFIG_STM32L4_PWR}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32l4_exti_pwr.c)
endif()

if("${CONFIG_STM32L4_RTC}" STREQUAL "y")
	if("${CONFIG_RTC_ALARM}" STREQUAL "y")
		list(APPEND CHIP_CSRCS stm32l4_exti_alarm.c)
	endif()
	if("${CONFIG_RTC_PERIODIC}" STREQUAL "y")
		list(APPEND CHIP_CSRCS stm32l4_exti_wakeup.c)
	endif()
	if("${CONFIG_RTC_DRIVER}" STREQUAL "y")
		list(APPEND CHIP_CSRCS stm32l4_rtc_lowerhalf.c)
	list(APPEND CHIP_CSRCS stm32l4_rtc.c)
	endif()
endif()

if("${CONFIG_DEBUG_FEATURES}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32l4_dumpgpio.c)
endif()

if("${CONFIG_STM32L4_COMP}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32l4_comp.c stm32l4_exti_comp.c)
endif()

if("${CONFIG_STM32L4_RNG}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32l4_rng.c)
endif()

if("${CONFIG_STM32L4_SAI}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32l4_sai.c)
endif()

if("${CONFIG_STM32L4_LPTIM}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32l4_lptim.c)
endif()

if("${CONFIG_PWM}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32l4_pwm.c)
endif()

if("${CONFIG_SENSORS_QENCODER}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32l4_qencoder.c)
endif()

if("${CONFIG_STM32L4_QSPI}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32l4_qspi.c)
endif()

if("${CONFIG_STM32L4_CAN}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32l4_can.c)
endif()

if("${CONFIG_STM32L4_FIREWALL}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32l4_firewall.c)
endif()

if("${CONFIG_STM32L4_IWDG}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32l4_iwdg.c)
endif()

if("${CONFIG_STM32L4_SDMMC1}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32l4_sdmmc.c)
endif()

if("${CONFIG_STM32L4_1WIREDRIVER}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32l4_1wire.c)
endif()

set_parent_srcs(${CHIP_CSRCS})
