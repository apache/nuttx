############################################################################
# arch/arm/src/sam34/CMakeLists.txt
#
#   Copyright (C) 2009-2011, 2013-2015, 2018 Gregory Nutt. All rights
#     reserved.
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

# Common ARM and Cortex-M3 files

CMN_UASRCS =
CMN_UCSRCS =

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

if(NOT "${CONFIG_SMP}" STREQUAL "y")
if(NOT "${CONFIG_ARCH_IDLE_CUSTOM}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_idle.c)
endif()
endif()

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

# Required SAM3/4 files

set(CHIP_CSRCS)
list(APPEND CHIP_CSRCS sam_allocateheap.c sam_irq.c sam_lowputc.c sam_serial.c)
list(APPEND CHIP_CSRCS sam_start.c)

# Configuration-dependent SAM3/4 files

if(NOT "${CONFIG_SCHED_TICKLESS}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_timerisr.c)
endif()

if("${CONFIG_CRYPTO_AES}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_aes.c)
endif()

if("${CONFIG_ARCH_CHIP_SAM4CM}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam4cm_supc.c)
endif()

if("${CONFIG_ARCH_CHIP_SAM4L}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam4l_clockconfig.c sam4l_periphclks.c sam4l_gpio.c)
else()
	list(APPEND CHIP_CSRCS sam_clockconfig.c sam_gpio.c sam_gpioirq.c)
endif()

if("${CONFIG_BUILD_PROTECTED}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_userspace.c sam_mpuinit.c)
endif()

if("${CONFIG_SAM34_CMCC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_cmcc.c)
endif()

if("${CONFIG_SAM34_DMAC0}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_dmac.c)
endif()

if("${CONFIG_ARCH_CHIP_SAM4L}" STREQUAL "y")
if("${CONFIG_SAM34_PDCA}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam4l_pdca.c)
endif()
endif()

if("${CONFIG_SAM34_EMAC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_emac.c)
endif()

if("${CONFIG_SAM34_UDP}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_udp.c)
endif()

if("${CONFIG_SAM34_HSMCI}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_hsmci.c)
endif()

if("${CONFIG_SAM34_SPI0}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_spi.c)
else()
if("${CONFIG_SAM34_SPI1}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_spi.c)
endif()
endif()

if("${CONFIG_SAM34_TWIM}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_twi.c)
endif()

if("${CONFIG_SAM34_AES}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_aes.c)
endif()

if("${CONFIG_SAM34_RTC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_rtc.c)
endif()

if("${CONFIG_SAM34_RTT}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_rtt.c)
endif()

if("${CONFIG_SAM34_WDT}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_wdt.c)
endif()

if("${CONFIG_TIMER}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_tc.c)
endif()

if("${CONFIG_ARCH_CHIP_SAM4CM}" STREQUAL "y")
if("${CONFIG_SAM34_TC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam4cm_tc.c)

if("${CONFIG_SAM34_ONESHOT}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam4cm_oneshot.c sam4cm_oneshot_lowerhalf.c)
endif() # CONFIG_SAM34_ONESHOT

if("${CONFIG_SAM34_FREERUN}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam4cm_freerun.c)
endif() # CONFIG_SAM34_FREERUN

if("${CONFIG_SCHED_TICKLESS}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam4cm_tickless.c)
endif() # CONFIG_SCHED_TICKLESS
endif() # CONFIG_SAM34_TC

if("${CONFIG_SMP}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam4cm_cpuindex.c sam4cm_cpuidlestack.c)
list(APPEND CHIP_CSRCS sam4cm_cpupause.c sam4cm_cpustart.c)
if(NOT "${CONFIG_ARCH_IDLE_CUSTOM}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam4cm_idle.c)
endif()
endif() # CONFIG_SMP

endif() # CONFIG_ARCH_CHIP_SAM4CM
