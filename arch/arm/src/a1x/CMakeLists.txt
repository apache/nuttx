############################################################################
# arch/arm/a1x/CMakeLists.txt
#
#   Copyright (C) 2013-2014, 2016, 2018 Gregory Nutt. All rights reserved.
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

if("${CONFIG_BUILD_KERNEL}" STREQUAL "y")
STARTUP_OBJS = crt0$(OBJEXT)
endif()

# Force the start-up logic to be at the beginning of the .text to simplify
# debug.

if("${CONFIG_PAGING}" STREQUAL "y")
set(CMN_ASRCS)
list(APPEND CMN_ASRCS arm_pghead.S)
else()
set(CMN_ASRCS)
list(APPEND CMN_ASRCS arm_head.S)
endif()

# Common assembly language files

list(APPEND CMN_ASRCS arm_vectors.S arm_fpuconfig.S arm_fullcontextrestore.S)
list(APPEND CMN_ASRCS arm_saveusercontext.S arm_vectoraddrexcptn.S arm_vfork.S)
list(APPEND CMN_ASRCS arm_testset.S arm_fetchadd.S)
list(APPEND CMN_ASRCS cp15_coherent_dcache.S cp15_invalidate_dcache.S)
list(APPEND CMN_ASRCS cp15_clean_dcache.S cp15_flush_dcache.S cp15_invalidate_dcache_all.S)

# Common C source files

set(CMN_CSRCS)
list(APPEND CMN_ASRCS up_initialize.c up_interruptcontext.c up_exit.c up_createstack.c)
list(APPEND CMN_CSRCS up_releasestack.c up_usestack.c up_vfork.c up_puts.c up_mdelay.c)
list(APPEND CMN_CSRCS up_stackframe.c up_udelay.c up_modifyreg8.c up_modifyreg16.c)
list(APPEND CMN_CSRCS up_modifyreg32.c)

list(APPEND CMN_CSRCS arm_assert.c arm_blocktask.c arm_copyfullstate.c arm_dataabort.c)
list(APPEND CMN_CSRCS arm_doirq.c arm_initialstate.c arm_mmu.c arm_prefetchabort.c)
list(APPEND CMN_CSRCS arm_releasepending.c arm_reprioritizertr.c)
list(APPEND CMN_CSRCS arm_schedulesigaction.c arm_sigdeliver.c arm_syscall.c)
list(APPEND CMN_CSRCS arm_unblocktask.c arm_undefinedinsn.c)

# Use common heap allocation for now (may need to be customized later)

list(APPEND CMN_CSRCS up_allocateheap.c)

# Configuration dependent C and assembly language files

if(NOT "${CONFIG_ARCH_IDLE_CUSTOM}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_idle.c)
endif()

if("${CONFIG_PAGING}" STREQUAL "y")
	list(APPEND CMN_CSRCS arm_allocpage.c arm_checkmapping.c arm_pginitialize.c)
list(APPEND CMN_CSRCS arm_va2pte.c)
endif()

if("${CONFIG_BUILD_KERNEL}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_task_start.c up_pthread_start.c arm_signal_dispatch.c)
endif()

if("${CONFIG_ARCH_ADDRENV}" STREQUAL "y")
	list(APPEND CMN_CSRCS arm_addrenv.c arm_addrenv_utils.c arm_pgalloc.c)
if("${CONFIG_ARCH_STACK_DYNAMIC}" STREQUAL "y")
	list(APPEND CMN_CSRCS arm_addrenv_ustack.c)
endif()
if("${CONFIG_ARCH_KERNEL_STACK}" STREQUAL "y")
	list(APPEND CMN_CSRCS arm_addrenv_kstack.c)
endif()
if("${CONFIG_MM_SHM}" STREQUAL "y")
	list(APPEND CMN_CSRCS arm_addrenv_shm.c)
endif()
endif()

if("${CONFIG_MM_PGALLOC}" STREQUAL "y")
	list(APPEND CMN_CSRCS arm_physpgaddr.c)
if("${CONFIG_ARCH_PGPOOL_MAPPING}" STREQUAL "y")
	list(APPEND CMN_CSRCS arm_virtpgaddr.c)
endif()
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

# A1x-specific assembly language files


# A1x-specific C source files

set(CHIP_CSRCS)
list(APPEND CHIP_CSRCS a1x_boot.c a1x_irq.c a1x_pio.c a1x_lowputc.c a1x_serial.c)

if(NOT "${CONFIG_SCHED_TICKLESS}" STREQUAL "y")
	list(APPEND CHIP_CSRCS a1x_timerisr.c)
endif()
