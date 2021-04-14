############################################################################
# arch/arm/tms570/CMakeLists.txt
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
# 3. Neither the name Gregory Nutt nor the names of its contributors may be
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

# The vector table is the "head" object, i.e., the one that must forced into
# the link in order to draw in all of the other components

HEAD_ASRC  = arm_vectortab.S)

# Common assembly language files

list(APPEND CMN_ASRCS arm_vectortab.S arm_vectors.S arm_head.S arm_fullcontextrestore.S)
list(APPEND CMN_ASRCS arm_saveusercontext.S arm_vectoraddrexcptn.S arm_vfork.S)
list(APPEND CMN_ASRCS arm_testset.S arm_fetchadd.S)
list(APPEND CMN_ASRCS cp15_coherent_dcache.S cp15_invalidate_dcache.S)
list(APPEND CMN_ASRCS cp15_clean_dcache.S cp15_flush_dcache.S)
list(APPEND CMN_ASRCS cp15_invalidate_dcache_all.S)

# Configuration dependent assembly language files

if("${CONFIG_ARCH_FPU}" STREQUAL "y")
	list(APPEND CMN_ASRCS arm_fpuconfig.S)
endif()

# Common C source files

set(CMN_CSRCS)
list(APPEND CMN_ASRCS up_allocateheap.c up_initialize.c up_interruptcontext.c)
list(APPEND CMN_CSRCS up_exit.c up_createstack.c up_releasestack.c up_usestack.c)
list(APPEND CMN_CSRCS up_vfork.c up_puts.c up_mdelay.c up_stackframe.c up_udelay.c)
list(APPEND CMN_CSRCS up_modifyreg8.c up_modifyreg16.c up_modifyreg32.c)

list(APPEND CMN_CSRCS arm_assert.c arm_blocktask.c arm_copyfullstate.c arm_dataabort.c)
list(APPEND CMN_CSRCS arm_doirq.c arm_initialstate.c arm_prefetchabort.c)
list(APPEND CMN_CSRCS arm_releasepending.c arm_reprioritizertr.c)
list(APPEND CMN_CSRCS arm_schedulesigaction.c arm_sigdeliver.c arm_syscall.c)
list(APPEND CMN_CSRCS arm_unblocktask.c arm_undefinedinsn.c)

# Configuration dependent C files

if(NOT "${CONFIG_ARCH_IDLE_CUSTOM}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_idle.c)
endif()

if("${CONFIG_BUILD_PROTECTED}" STREQUAL "y")
	list(APPEND CMN_CSRCS arm_mpu.c up_task_start.c up_pthread_start.c)
if(NOT "${CONFIG_DISABLE_SIGNALS}" STREQUAL "y")
	list(APPEND CMN_CSRCS arm_signal_dispatch.c)
list(APPEND CMN_UASRCS arm_signal_handler.S)
endif()
endif()

if("${CONFIG_ARMV7R_L2CC_PL310}" STREQUAL "y")
	list(APPEND CMN_CSRCS arm_l2cc_pl310.c)
endif()

if("${CONFIG_ELF}" STREQUAL "y")
	list(APPEND CMN_CSRCS arm_coherent_dcache.c)
elseif("${CONFIG_MODULE}" STREQUAL "y")
	list(APPEND CMN_CSRCS arm_coherent_dcache.c)
endif()

if("${CONFIG_ARCH_FPU}" STREQUAL "y")
	list(APPEND CMN_ASRCS arm_savefpu.S arm_restorefpu.S)
list(APPEND CMN_CSRCS arm_copyarmstate.c)
endif()

if("${CONFIG_STACK_COLORATION}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_checkstack.c)
endif()

# SAMA5-specific assembly language files


# SAMA5-specific C source files

set(CHIP_CSRCS)
list(APPEND CHIP_CSRCS tms570_boot.c tms570_clockconfig.c tms570_esm.c tms570_gio.c)
list(APPEND CHIP_CSRCS tms570_irq.c tms570_lowputc.c tms570_serial.c tms570_spi.c)

# Configuration dependent C and assembly language files

if(NOT "${CONFIG_SCHED_TICKLESS}" STREQUAL "y")
	list(APPEND CHIP_CSRCS tms570_timerisr.c)
endif()

if("${CONFIG_TMS570_SELFTEST}" STREQUAL "y")
	list(APPEND CHIP_CSRCS tms570_selftest.c)
endif()

if("${CONFIG_TMS570_GIO_IRQ}" STREQUAL "y")
	list(APPEND CHIP_CSRCS tms570_gioirq.c)
endif()
