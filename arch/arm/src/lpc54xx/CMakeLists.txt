############################################################################
# arch/arm/src/lpc54xx/CMakeLists.txt
#
#   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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
list(APPEND CHIP_CSRCS lpc54_start.c lpc54_clockconfig.c lpc54_irq.c lpc54_clrpend.c)
list(APPEND CHIP_CSRCS lpc54_allocateheap.c lpc54_lowputc.c lpc54_gpio.c lpc54_reset.c)

if(NOT "${CONFIG_SCHED_TICKLESS}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc54_timerisr.c)
else()
	list(APPEND CHIP_CSRCS lpc54_tickless.c)
endif()

if("${CONFIG_BUILD_PROTECTED}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc54_userspace.c lpc54_mpuinit.c)
endif()

if("${CONFIG_LPC54_DMA}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc54_dma.c)
endif()

if(NOT "${CONFIG_ARCH_IDLE_CUSTOM}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc54_idle.c)
endif()

if("${CONFIG_LPC54_GPIOIRQ}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc54_gpioirq.c)
endif()

if("${CONFIG_RTC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc54_rtc.c)
if("${CONFIG_RTC_DRIVER}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc54_rtc_lowerhalf.c)
endif()
endif()

if("${CONFIG_LPC54_WWDT}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc54_wwdt.c)
endif()

if("${CONFIG_LPC54_RNG}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc54_rng.c)
endif()

if("${CONFIG_LPC54_HAVE_USART}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc54_serial.c)
endif()

if("${CONFIG_LPC54_HAVE_I2C_MASTER}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc54_i2c_master.c)
endif()

if("${CONFIG_LPC54_HAVE_SPI_MASTER}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc54_spi_master.c)
endif()

if("${CONFIG_LPC54_EMC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc54_emc.c)
endif()

if("${CONFIG_LPC54_ETHERNET}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc54_ethernet.c)
endif()
if("${CONFIG_LPC54_LCD}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc54_lcd.c)
endif()

if("${CONFIG_LPC54_SDMMC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc54_sdmmc.c)
endif()

