############################################################################
# arch/arm/src/efm32/CMakeLists.txt
#
#   Copyright (C) 2014-2016, 2018 Gregory Nutt. All rights reserved.
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

HEAD_ASRC =

CMN_UASRCS =
CMN_UCSRCS =

set(CMN_ASRCS)
list(APPEND CMN_ASRCS up_saveusercontext.S up_fullcontextrestore.S up_switchcontext.S)
list(APPEND CMN_ASRCS up_testset.S up_fetchadd.S vfork.S)

set(CMN_CSRCS)
list(APPEND CMN_ASRCS up_allocateheap.c up_assert.c up_blocktask.c up_copyfullstate.c)
list(APPEND CMN_CSRCS up_createstack.c up_doirq.c up_exit.c up_hardfault.c)
list(APPEND CMN_CSRCS up_initialize.c up_initialstate.c up_interruptcontext.c)
list(APPEND CMN_CSRCS up_itm.c up_mdelay.c up_memfault.c up_modifyreg8.c)
list(APPEND CMN_CSRCS up_modifyreg16.c up_modifyreg32.c up_releasepending.c)
list(APPEND CMN_CSRCS up_releasestack.c up_reprioritizertr.c up_schedulesigaction.c)
list(APPEND CMN_CSRCS up_sigdeliver.c up_stackframe.c up_svcall.c up_systemreset.c)
list(APPEND CMN_CSRCS up_trigger_irq.c up_udelay.c up_unblocktask.c up_usestack.c up_vfork.c)

if("${CONFIG_ARMV7M_LAZYFPU}" STREQUAL "y")
	list(APPEND CMN_ASRCS up_lazyexception.S)
else()
	list(APPEND CMN_ASRCS up_exception.S)
endif()
list(APPEND CMN_CSRCS up_vectors.c)

if("${CONFIG_ARCH_RAMVECTORS}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_ramvec_initialize.c up_ramvec_attach.c)
endif()

if("${CONFIG_BUILD_PROTECTED}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_mpu.c up_task_start.c up_pthread_start.c)
if(NOT "${CONFIG_DISABLE_SIGNALS}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_signal_dispatch.c)
list(APPEND CMN_UASRCS up_signal_handler.S)
endif()
endif()

if("${CONFIG_STACK_COLORATION}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_checkstack.c)
endif()

if("${CONFIG_ARCH_FPU}" STREQUAL "y")
	list(APPEND CMN_ASRCS up_fpu.S)
list(APPEND CMN_CSRCS up_copyarmstate.c)
endif()

if("${CONFIG_ARMV7M_ITMSYSLOG}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_itm_syslog.c)
endif()


set(CHIP_CSRCS)
list(APPEND CHIP_CSRCS efm32_start.c efm32_clockconfig.c efm32_irq.c efm32_timerisr.c)
list(APPEND CHIP_CSRCS efm32_gpio.c efm32_lowputc.c efm32_timer.c efm32_i2c.c)

if("${CONFIG_EFM32_FLASHPROG}" STREQUAL "y")
	list(APPEND CHIP_CSRCS efm32_flash.c)
endif()

if("${CONFIG_EFM32_BITBAND}" STREQUAL "y")
	list(APPEND CHIP_CSRCS efm32_bitband.c)
endif()

if(NOT "${CONFIG_ARCH_IDLE_CUSTOM}" STREQUAL "y")
	list(APPEND CHIP_CSRCS efm32_idle.c)
endif()

if("${CONFIG_EFM32_USART_ISUART}" STREQUAL "y")
	list(APPEND CHIP_CSRCS efm32_serial.c)
else()
if("${CONFIG_EFM32_UART}" STREQUAL "y")
	list(APPEND CHIP_CSRCS efm32_serial.c)
endif()
endif()

if("${CONFIG_EFM32_RMU}" STREQUAL "y")
	list(APPEND CHIP_CSRCS efm32_rmu.c)
endif()

if("${CONFIG_EFM32_USART_ISSPI}" STREQUAL "y")
	list(APPEND CHIP_CSRCS efm32_spi.c)
endif()

if("${CONFIG_EFM32_LEUART}" STREQUAL "y")
	list(APPEND CHIP_CSRCS efm32_leserial.c)
endif()

if("${CONFIG_EFM32_GPIO_IRQ}" STREQUAL "y")
	list(APPEND CHIP_CSRCS efm32_gpioirq.c)
endif()

if("${CONFIG_EFM32_DMA}" STREQUAL "y")
	list(APPEND CHIP_CSRCS efm32_dma.c)
endif()

if("${CONFIG_EFM32_RTC_BURTC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS efm32_rtc_burtc.c)
endif()

if("${CONFIG_EFM32_OTGFS}" STREQUAL "y")
if("${CONFIG_USBDEV}" STREQUAL "y")
	list(APPEND CHIP_CSRCS efm32_usbdev.c)
endif()
if("${CONFIG_USBHOST}" STREQUAL "y")
	list(APPEND CHIP_CSRCS efm32_usbhost.c)
endif()
endif()

if("${CONFIG_PWM}" STREQUAL "y")
	list(APPEND CHIP_CSRCS efm32_pwm.c)
endif()
