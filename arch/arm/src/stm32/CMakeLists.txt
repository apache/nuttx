############################################################################
# arch/arm/src/stm32/CMakeLists.txt
#
#   Copyright (C) 2009, 2011-2016, 2018 Gregory Nutt. All rights reserved.
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
list(APPEND CHIP_CSRCS stm32_allocateheap.c stm32_start.c stm32_rcc.c stm32_lse.c)
list(APPEND CHIP_CSRCS stm32_lsi.c stm32_gpio.c stm32_exti_gpio.c stm32_flash.c)
list(APPEND CHIP_CSRCS stm32_irq.c stm32_dma.c stm32_lowputc.c)
list(APPEND CHIP_CSRCS stm32_serial.c stm32_spi.c stm32_i2s.c stm32_sdio.c stm32_tim.c)
list(APPEND CHIP_CSRCS stm32_waste.c stm32_ccm.c stm32_uid.c stm32_capture.c)

if("${CONFIG_TIMER}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_tim_lowerhalf.c)
endif()

if(NOT "${CONFIG_SCHED_TICKLESS}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_timerisr.c)
else()
	list(APPEND CHIP_CSRCS stm32_tickless.c)
endif()

if("${CONFIG_STM32_ONESHOT}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_oneshot.c stm32_oneshot_lowerhalf.c)
endif()

if("${CONFIG_STM32_FREERUN}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_freerun.c)
endif()

if("${CONFIG_BUILD_PROTECTED}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_userspace.c stm32_mpuinit.c)
endif()

if("${CONFIG_STM32_CCM_PROCFS}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_procfs_ccm.c)
endif()

if("${CONFIG_STM32_I2C_ALT}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_i2c_alt.c)
elseif("${CONFIG_STM32_STM32F30XX}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32f30xxx_i2c.c)
elseif("${CONFIG_STM32_STM32F37XX}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32f30xxx_i2c.c)
elseif("${CONFIG_STM32_STM32F4XXX}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32f40xxx_i2c.c)
else()
	list(APPEND CHIP_CSRCS stm32_i2c.c)
endif()

if("${CONFIG_USBDEV}" STREQUAL "y")
	if("${CONFIG_STM32_USB}" STREQUAL "y")
		list(APPEND CHIP_CSRCS stm32_usbdev.c)
	endif()
	if("${CONFIG_STM32_OTGFS}" STREQUAL "y")
		list(APPEND CHIP_CSRCS stm32_otgfsdev.c)
	endif()
	if("${CONFIG_STM32_OTGHS}" STREQUAL "y")
		list(APPEND CHIP_CSRCS stm32_otghsdev.c)
	endif()
endif()

if("${CONFIG_STM32_USBHOST}" STREQUAL "y")
	if("${CONFIG_STM32_OTGFS}" STREQUAL "y")
		list(APPEND CHIP_CSRCS stm32_otgfshost.c)
	endif()
	if("${CONFIG_STM32_OTGHS}" STREQUAL "y")
		list(APPEND CHIP_CSRCS stm32_otghshost.c)
	endif()
	if("${CONFIG_USBHOST_TRACE}" STREQUAL "y")
		list(APPEND CHIP_CSRCS stm32_usbhost.c)
	else()
		if("${CONFIG_DEBUG_USB}" STREQUAL "y")
			list(APPEND CHIP_CSRCS stm32_usbhost.c)
		endif()
	endif()
endif()

if(NOT "${CONFIG_ARCH_IDLE_CUSTOM}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_idle.c)
endif()

list(APPEND CHIP_CSRCS stm32_pmstop.c stm32_pmstandby.c stm32_pmsleep.c)

if(NOT "${CONFIG_ARCH_CUSTOM_PMINIT}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_pminitialize.c)
endif()

if("${CONFIG_STM32_ETHMAC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_eth.c)
endif()

if("${CONFIG_STM32_PWR}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_pwr.c stm32_exti_pwr.c)
endif()

if("${CONFIG_STM32_RTC}" STREQUAL "y")
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

if("${CONFIG_STM32_ADC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_adc.c)
endif()

if("${CONFIG_STM32_SDADC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_sdadc.c)
endif()

if("${CONFIG_STM32_DAC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_dac.c)
endif()

if("${CONFIG_STM32_COMP}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_comp.c)
endif()

if("${CONFIG_STM32_OPAMP}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_opamp.c)
endif()

if("${CONFIG_STM32_HRTIM}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_hrtim.c)
endif()

if("${CONFIG_STM32_1WIREDRIVER}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_1wire.c)
endif()

if("${CONFIG_STM32_HCIUART}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_hciuart.c)
endif()

if("${CONFIG_STM32_RNG}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_rng.c)
endif()

if("${CONFIG_STM32_LTDC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_ltdc.c)
endif()

if("${CONFIG_STM32_DMA2D}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_dma2d.c)
endif()

if("${CONFIG_STM32_PWM}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_pwm.c)
endif()

if("${CONFIG_SENSORS_QENCODER}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_qencoder.c)
endif()

if("${CONFIG_STM32_CAN}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_can.c)
endif()

if("${CONFIG_STM32_IWDG}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_iwdg.c)
endif()

if("${CONFIG_STM32_WWDG}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_wwdg.c)
endif()

if("${CONFIG_DEBUG_FEATURES}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_dumpgpio.c)
endif()

if("${CONFIG_STM32_AES}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_aes.c)
endif()

if("${CONFIG_STM32_BBSRAM}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_bbsram.c)
endif()

target_sources(arch PRIVATE ${CHIP_CSRCS})
