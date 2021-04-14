############################################################################
# arch/arm/src/lpc43xx/CMakeLists.txt
#
#   Copyright (C) 2012-2015, 2017 Gregory Nutt. All rights reserved.
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

HEAD_ASRC  =

set(CMN_ASRCS)
list(APPEND CMN_ASRCS up_saveusercontext.S up_fullcontextrestore.S up_switchcontext.S)
list(APPEND CMN_ASRCS up_testset.S up_fetchadd.S vfork.S)

set(CMN_CSRCS)
list(APPEND CMN_ASRCS up_assert.c up_blocktask.c up_copyfullstate.c up_createstack.c)
list(APPEND CMN_CSRCS up_doirq.c up_exit.c up_hardfault.c up_initialize.c)
list(APPEND CMN_CSRCS up_initialstate.c up_interruptcontext.c up_mdelay.c)
list(APPEND CMN_CSRCS up_memfault.c up_modifyreg8.c up_modifyreg16.c up_modifyreg32.c)
list(APPEND CMN_CSRCS up_releasepending.c up_releasestack.c up_reprioritizertr.c)
list(APPEND CMN_CSRCS up_schedulesigaction.c up_sigdeliver.c up_stackframe.c)
list(APPEND CMN_CSRCS up_svcall.c up_trigger_irq.c up_unblocktask.c up_udelay.c)
list(APPEND CMN_CSRCS up_usestack.c up_vfork.c)

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

set(CHIP_CSRCS)
list(APPEND CHIP_CSRCS lpc43_allocateheap.c lpc43_cgu.c lpc43_clrpend.c lpc43_gpio.c)
list(APPEND CHIP_CSRCS lpc43_irq.c lpc43_pinconfig.c lpc43_rgu.c lpc43_serial.c)
list(APPEND CHIP_CSRCS lpc43_start.c lpc43_uart.c)

if(NOT "${CONFIG_SCHED_TICKLESS}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc43_timerisr.c)
else()
	list(APPEND CHIP_CSRCS lpc43_tickless_rit.c)
endif()

if("${CONFIG_BUILD_PROTECTED}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc43_userspace.c lpc43_mpuinit.c)
endif()

if(NOT "${CONFIG_ARCH_IDLE_CUSTOM}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc43_idle.c)
endif()

if("${CONFIG_DEBUG_FEATURES}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc43_debug.c)
endif()

if("${CONFIG_LPC43_GPDMA}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc43_gpdma.c)
endif()

if("${CONFIG_LPC43_GPIO_IRQ}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc43_gpioint.c)
endif()

if("${CONFIG_LPC43_WWDT}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc43_wwdt.c)
endif()

if("${CONFIG_LPC43_SDMMC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc43_sdmmc.c)
endif()

if("${CONFIG_LPC43_ETHERNET}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc43_ethernet.c)
endif()

if("${CONFIG_LPC43_EMC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc43_emc.c)
endif()

if("${CONFIG_LPC43_SPI}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc43_spi.c)
elseif("${CONFIG_LPC43_SSP0}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc43_spi.c)
elseif("${CONFIG_LPC43_SSP1}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc43_spi.c)
endif()

if("${CONFIG_LPC43_SPIFI}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc43_spifi.c)
endif()

if("${CONFIG_LPC43_SSP0}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc43_ssp.c)
elseif("${CONFIG_LPC43_SSP1}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc43_ssp.c)
endif()

if("${CONFIG_LPC43_TIMER}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc43_timer.c)
endif()

if("${CONFIG_LPC43_RIT}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc43_rit.c)
endif()

if("${CONFIG_LPC43_RTC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc43_rtc.c)
endif()

if("${CONFIG_LPC43_I2C0}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc43_i2c.c)
elseif("${CONFIG_LPC43_I2C1}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc43_i2c.c)
endif()

if("${CONFIG_LPC43_CAN0}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc43_can.c)
elseif("${CONFIG_LPC43_CAN1}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc43_can.c)
endif()

if("${CONFIG_LPC43_ADC0}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc43_adc.c)
elseif("${CONFIG_LPC43_ADC1}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc43_adc.c)
endif()

if("${CONFIG_LPC43_DAC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc43_dac.c)
endif()

if("${CONFIG_LPC43_USBOTG}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc43_ehci.c)
endif()

if("${CONFIG_CRYPTO_AES}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc43_aes.c)
endif()

if("${CONFIG_LPC43_USB0}" STREQUAL "y")
if("${CONFIG_USBDEV}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc43_usb0dev.c)
endif()
endif()

-include chip/spifi/src/CMakeLists.txt
