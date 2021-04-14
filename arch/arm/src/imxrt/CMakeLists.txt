############################################################################
# arch/arm/src/imxrt/CMakeLists.txt
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

HEAD_ASRC  =

# Common ARM and Cortex-M7 files

set(CMN_ASRCS)
list(APPEND CMN_ASRCS up_saveusercontext.S up_fullcontextrestore.S up_switchcontext.S)
list(APPEND CMN_ASRCS up_testset.S up_fetchadd.S vfork.S)

set(CMN_CSRCS)
list(APPEND CMN_ASRCS up_assert.c up_blocktask.c up_copyfullstate.c)
list(APPEND CMN_CSRCS up_createstack.c up_mdelay.c up_udelay.c up_exit.c)
list(APPEND CMN_CSRCS up_initialize.c up_initialstate.c up_interruptcontext.c)
list(APPEND CMN_CSRCS up_memfault.c up_modifyreg8.c up_modifyreg16.c up_modifyreg32.c)
list(APPEND CMN_CSRCS up_releasepending.c up_releasestack.c up_reprioritizertr.c)
list(APPEND CMN_CSRCS up_schedulesigaction.c up_sigdeliver.c up_stackframe.c)
list(APPEND CMN_CSRCS up_unblocktask.c up_usestack.c up_doirq.c up_hardfault.c)
list(APPEND CMN_CSRCS up_svcall.c up_vfork.c up_trigger_irq.c)

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

if("${CONFIG_STACK_COLORATION}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_checkstack.c)
endif()

if("${CONFIG_ARMV7M_DCACHE}" STREQUAL "y")
	list(APPEND CMN_CSRCS arch_enable_dcache.c arch_disable_dcache.c)
list(APPEND CMN_CSRCS arch_invalidate_dcache.c arch_invalidate_dcache_all.c)
if(NOT "${CONFIG_ARMV7M_DCACHE_WRITETHROUGH}" STREQUAL "y")
	list(APPEND CMN_CSRCS arch_clean_dcache.c arch_clean_dcache_all.c)
list(APPEND CMN_CSRCS arch_flush_dcache.c arch_flush_dcache_all.c)
endif()
endif()

if("${CONFIG_ARCH_FPU}" STREQUAL "y")
	list(APPEND CMN_ASRCS up_fpu.S)
list(APPEND CMN_CSRCS up_copyarmstate.c)
endif()

# Required i.MX RT files

set(CHIP_CSRCS)
list(APPEND CHIP_CSRCS imxrt_allocateheap.c imxrt_start.c imxrt_clockconfig.c)
list(APPEND CHIP_CSRCS imxrt_periphclks.c imxrt_irq.c imxrt_clrpend.c imxrt_gpio.c)
list(APPEND CHIP_CSRCS imxrt_wdog.c imxrt_iomuxc.c imxrt_serial.c imxrt_lowputc.c)

# Configuration-dependent i.MX RT files

if(NOT "${CONFIG_ARCH_IDLE_CUSTOM}" STREQUAL "y")
	list(APPEND CHIP_CSRCS imxrt_idle.c)
endif()

if(NOT "${CONFIG_SCHED_TICKLESS}" STREQUAL "y")
	list(APPEND CHIP_CSRCS imxrt_timerisr.c)
endif()

if("${CONFIG_IMXRT_GPIO_IRQ}" STREQUAL "y")
	list(APPEND CHIP_CSRCS imxrt_gpioirq.c)
endif()

if("${CONFIG_ARM_MPU}" STREQUAL "y")
	list(APPEND CHIP_CSRCS imxrt_mpuinit.c)
if("${CONFIG_BUILD_PROTECTED}" STREQUAL "y")
	list(APPEND CHIP_CSRCS imxrt_userspace.c)
endif()
endif()

if("${CONFIG_IMXRT_EDMA}" STREQUAL "y")
	list(APPEND CHIP_CSRCS imxrt_edma.c)
endif()

ifdef CONFIG_IMXRT_USDHC
list(APPEND CHIP_CSRCS imxrt_usdhc.c)
endif()

if("${CONFIG_IMXRT_SNVS_LPSRTC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS imxrt_lpsrtc.c)
list(APPEND CHIP_CSRCS imxrt_hprtc.c)
elseif("${CONFIG_IMXRT_SNVS_HPRTC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS imxrt_hprtc.c)
endif()

if("${CONFIG_RTC_DRIVER}" STREQUAL "y")
	list(APPEND CHIP_CSRCS imxrt_rtc_lowerhalf.c)
endif()

if("${CONFIG_IMXRT_ENET}" STREQUAL "y")
	list(APPEND CHIP_CSRCS imxrt_enet.c)
endif()

if("${CONFIG_IMXRT_LPI2C}" STREQUAL "y")
	list(APPEND CHIP_CSRCS imxrt_lpi2c.c)
endif()

if("${CONFIG_IMXRT_LPSPI}" STREQUAL "y")
	list(APPEND CHIP_CSRCS imxrt_lpspi.c)
endif()
