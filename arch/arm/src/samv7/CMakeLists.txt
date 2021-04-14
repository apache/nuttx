############################################################################
# arch/arm/src/samv7/CMakeLists.txt
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

# The start-up, "head", file.  Only common vectors are support so there
# isn't one.

HEAD_ASRC =

# Common ARM and Cortex-M7 files

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

# Configuration-dependent common files

if(NOT "${CONFIG_ARCH_IDLE_CUSTOM}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_idle.c)
endif()

if("${CONFIG_ARMV7M_STACKCHECK}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_stackcheck.c)
endif()

if("${CONFIG_ARMV7M_LAZYFPU}" STREQUAL "y")
	list(APPEND CMN_ASRCS up_lazyexception.S)
else()
	list(APPEND CMN_ASRCS up_exception.S)
endif()
list(APPEND CMN_CSRCS up_vectors.c)

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

if("${CONFIG_ARCH_RAMVECTORS}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_ramvec_initialize.c up_ramvec_attach.c)
endif()

if("${CONFIG_ARM_MPU}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_mpu.c)
if("${CONFIG_BUILD_PROTECTED}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_task_start.c up_pthread_start.c)
if(NOT "${CONFIG_DISABLE_SIGNALS}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_signal_dispatch.c)
list(APPEND CMN_UASRCS up_signal_handler.S)
endif()
endif()
endif()

if("${CONFIG_ELF}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_coherent_dcache.c)
elseif("${CONFIG_MODULE}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_coherent_dcache.c)
endif()

if("${CONFIG_STACK_COLORATION}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_checkstack.c)
endif()

# Required SAMV7 files

set(CHIP_CSRCS)
list(APPEND CHIP_CSRCS sam_start.c sam_clockconfig.c sam_irq.c sam_allocateheap.c)
list(APPEND CHIP_CSRCS sam_lowputc.c sam_serial.c sam_gpio.c sam_pck.c)

# Configuration-dependent SAMV7 files

if(NOT "${CONFIG_SCHED_TICKLESS}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_timerisr.c)
endif()

if("${CONFIG_ARM_MPU}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_mpuinit.c)
if("${CONFIG_BUILD_PROTECTED}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_userspace.c)
endif()
endif()

if("${CONFIG_SAMV7_GPIO_IRQ}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_gpioirq.c)
endif()

if("${CONFIG_SAMV7_XDMAC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_xdmac.c)
endif()

if("${CONFIG_SAMV7_WDT}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_wdt.c)
endif()

if("${CONFIG_SAMV7_RSWDT}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_rswdt.c)
endif()

if("${CONFIG_SAMV7_SYSTEMRESET}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_systemreset.c)
endif()

if("${CONFIG_SAMV7_SPI_MASTER}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_spi.c)
endif()

if("${CONFIG_SAMV7_SPI_SLAVE}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_spi_slave.c)
endif()

if("${CONFIG_SAMV7_QSPI}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_qspi.c)
endif()

if("${CONFIG_SAMV7_TWIHS0}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_twihs.c)
elseif("${CONFIG_SAMV7_TWIHS1}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_twihs.c)
elseif("${CONFIG_SAMV7_TWIHS2}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_twihs.c)
endif()

if("${CONFIG_SAMV7_SSC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_ssc.c)
endif()

if("${CONFIG_SAMV7_HAVE_TC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_tc.c)
if("${CONFIG_SAMV7_ONESHOT}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_oneshot.c sam_oneshot_lowerhalf.c)
endif()
if("${CONFIG_SAMV7_FREERUN}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_freerun.c)
endif()
if("${CONFIG_SCHED_TICKLESS}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_tickless.c)
endif()
endif()

if("${CONFIG_SAMV7_HSMCI}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_hsmci.c sam_hsmci_clkdiv.c)
endif()

if("${CONFIG_SAMV7_EMAC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_emac.c sam_ethernet.c)
endif()

if("${CONFIG_SAMV7_MCAN}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_mcan.c)
endif()

if("${CONFIG_SAMV7_USBDEVHS}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_usbdevhs.c)
endif()

if("${CONFIG_SAMV7_TRNG}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_trng.c)
endif()

if("${CONFIG_SAMV7_PROGMEM}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_progmem.c)
endif()

if("${CONFIG_SAMV7_DAC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_dac.c)
endif()
