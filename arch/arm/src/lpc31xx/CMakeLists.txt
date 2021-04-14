############################################################################
# arch/arm/lpc31xx/CMakeLists.txt
#
#   Copyright (C) 2009-2011, 2014, 2018 Gregory Nutt. All rights reserved.
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

HEAD_ASRC = up_head.S)

set(CMN_ASRCS)
list(APPEND CMN_ASRCS up_cache.S up_fullcontextrestore.S up_saveusercontext.S)
list(APPEND CMN_ASRCS up_vectors.S up_vectoraddrexcptn.S up_vectortab.S vfork.S)

set(CMN_CSRCS)
list(APPEND CMN_ASRCS up_assert.c up_blocktask.c up_copyfullstate.c up_createstack.c)
list(APPEND CMN_CSRCS up_dataabort.c up_mdelay.c up_udelay.c up_exit.c)
list(APPEND CMN_CSRCS up_initialize.c up_initialstate.c up_interruptcontext.c)
list(APPEND CMN_CSRCS up_modifyreg8.c up_modifyreg16.c up_modifyreg32.c)
list(APPEND CMN_CSRCS up_prefetchabort.c up_releasepending.c up_releasestack.c)
list(APPEND CMN_CSRCS up_reprioritizertr.c up_schedulesigaction.c)
list(APPEND CMN_CSRCS up_sigdeliver.c up_stackframe.c up_syscall.c up_unblocktask.c)
list(APPEND CMN_CSRCS up_undefinedinsn.c up_usestack.c up_vfork.c)

if(NOT "${CONFIG_ARCH_IDLE_CUSTOM}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_idle.c)
endif()

if("${CONFIG_PAGING}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_pginitialize.c up_checkmapping.c up_allocpage.c up_va2pte.c)
endif()

if("${CONFIG_STACK_COLORATION}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_checkstack.c)
endif()

CGU_ASRCS  =

CGU_CSRCS  = lpc31_bcrndx.c lpc31_clkdomain.c lpc31_clkexten.c)
CGU_CSRCS += lpc31_clkfreq.c lpc31_clkinit.c lpc31_defclk.c)
CGU_CSRCS += lpc31_esrndx.c lpc31_fdcndx.c lpc31_fdivinit.c)
CGU_CSRCS += lpc31_freqin.c lpc31_pllconfig.c lpc31_resetclks.c)
CGU_CSRCS += lpc31_setfreqin.c lpc31_setfdiv.c lpc31_softreset.c)

CHIP_ASRCS  = $(CGU_ASRCS)

set(CHIP_CSRCS)
list(APPEND CHIP_CSRCS lpc31_allocateheap.c lpc31_boot.c lpc31_decodeirq.c)
list(APPEND CHIP_CSRCS lpc31_irq.c lpc31_lowputc.c lpc31_serial.c lpc31_i2c.c)
list(APPEND CHIP_CSRCS lpc31_spi.c $(CGU_CSRCS)

if(NOT "${CONFIG_SCHED_TICKLESS}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc31_timerisr.c)
endif()

if("${CONFIG_LPC31_USBOTG}" STREQUAL "y")
if("${CONFIG_USBHOST}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc31_ehci.c)
endif()
if("${CONFIG_USBDEV}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc31_usbdev.c)
endif()
endif()
