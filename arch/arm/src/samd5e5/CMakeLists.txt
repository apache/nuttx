############################################################################
# arch/arm/src/samd5e5/CMakeLists.txt
#
#   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

# The start-up, "head", file

HEAD_ASRC =

# Common ARM and Cortex-M4 files

CMN_UASRCS =
CMN_UCSRCS =

set(CMN_ASRCS)
list(APPEND CMN_ASRCS up_saveusercontext.S up_fullcontextrestore.S up_switchcontext.S)
list(APPEND CMN_ASRCS up_testset.S up_fetchadd.S vfork.S)

set(CMN_CSRCS)
list(APPEND CMN_ASRCS up_allocateheap.c up_assert.c up_blocktask.c up_copyfullstate.c)
list(APPEND CMN_CSRCS up_createstack.c up_exit.c up_initialize.c up_initialstate.c)
list(APPEND CMN_CSRCS up_interruptcontext.c up_mdelay.c up_memfault.c up_modifyreg8.c)
list(APPEND CMN_CSRCS up_modifyreg16.c up_modifyreg32.c up_releasepending.c)
list(APPEND CMN_CSRCS up_releasestack.c up_reprioritizertr.c up_schedulesigaction.c)
list(APPEND CMN_CSRCS up_sigdeliver.c up_stackframe.c up_svcall.c up_trigger_irq.c)
list(APPEND CMN_CSRCS up_unblocktask.c up_udelay.c up_usestack.c up_doirq.c)
list(APPEND CMN_CSRCS up_hardfault.c up_vfork.c)

# Configuration-dependent common files

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

if("${CONFIG_ARCH_FPU}" STREQUAL "y")
	list(APPEND CMN_ASRCS up_fpu.S)
list(APPEND CMN_CSRCS up_copyarmstate.c)
endif()

if("${CONFIG_STACK_COLORATION}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_checkstack.c)
endif()

# Required SAMD5x/E5x files

set(CHIP_CSRCS)
list(APPEND CHIP_CSRCS sam_clockconfig.c sam_gclk.c sam_irq.c sam_lowputc.c)
list(APPEND CHIP_CSRCS sam_port.c sam_serial.c sam_start.c sam_usart.c)

# Configuration-dependent SAMD5x/E5x files

if(NOT "${CONFIG_SCHED_TICKLESS}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_timerisr.c)
endif()

if("${CONFIG_SAMD5E5_SERCOM}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_sercom.c)
endif()

if("${CONFIG_SAMD5E5_TC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_tc.c)

if("${CONFIG_SAMD5E5_ONESHOT}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_oneshot.c sam_oneshot_lowerhalf.c)
endif() # CONFIG_SAMD5E5_ONESHOT

if("${CONFIG_SAMD5E5_FREERUN}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_freerun.c)
endif() # CONFIG_SAMD5E5_FREERUN

if("${CONFIG_SCHED_TICKLESS}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_tickless.c)
endif() # CONFIG_SCHED_TICKLESS
endif() # CONFIG_SAMD5E5_TC

if("${CONFIG_BUILD_PROTECTED}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_userspace.c sam_mpuinit.c)
endif()

if("${CONFIG_SAMD5E5_EIC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_eic.c)
endif()

if("${CONFIG_SAMD5E5_CMCC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_cmcc.c)
endif()

if("${CONFIG_SAMD5E5_DMAC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_dmac.c)
endif()

if(NOT "${CONFIG_ARCH_IDLE_CUSTOM}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_idle.c)
endif()

if("${CONFIG_SAMD5E5_HAVE_SPI}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_spi.c)
endif()

if("${CONFIG_SAMD5E5_HAVE_I2C_MASTER}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_i2c_master.c)
endif()

if("${CONFIG_SAMD5E5_WDT}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_wdt.c)
endif()

if("${CONFIG_SAMD5E5_GMAC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_gmac.c)
endif()

if("${CONFIG_SAMD5E5_USB}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_usb.c)
endif()

if("${CONFIG_SAMD5E5_SDHC0}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_sdhc.c)
elseif("${CONFIG_SAMD5E5_SDHC1}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_sdhc.c)
endif()

if("${CONFIG_SAMD5E5_AES}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_aes.c)
endif()

if("${CONFIG_SAMD5E5_RTC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_rtc.c)
endif()
