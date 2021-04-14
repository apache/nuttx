############################################################################
# arch/z80/src/z180/CMakeLists.txt
#
#   Copyright (C) 2012, 2014 Gregory Nutt. All rights reserved.
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

if("${CONFIG_ARCH_HAVEHEAD),)
if("${CONFIG_LINKER_ROM_AT_0000}" STREQUAL "y")
HEAD_ASRC = z180_rom.asm
else()
HEAD_ASRC = z180_head.asm
endif()
endif()

CMN_ASRCS  =
set(CMN_CSRCS)
list(APPEND CMN_ASRCS up_allocateheap.c up_assert.c up_blocktask.c up_createstack.c)
list(APPEND CMN_CSRCS up_doirq.c up_exit.c up_idle.c up_initialize.c)
list(APPEND CMN_CSRCS up_interruptcontext.c up_mdelay.c up_releasepending.c)
list(APPEND CMN_CSRCS up_releasestack.c up_stackframe.c up_reprioritizertr.c)
list(APPEND CMN_CSRCS up_unblocktask.c up_udelay.c up_usestack.c)

CHIP_ASRCS  = z180_restoreusercontext.asm z180_saveusercontext.asm
CHIP_ASRCS += z180_vectcommon.asm

if("${CONFIG_LINKER_ROM_AT_0000}" STREQUAL "y")
CHIP_ASRCS += z180_romvectors.asm
else()
CHIP_ASRCS += z180_vectors.asm
endif()

set(CHIP_CSRCS)
list(APPEND CHIP_CSRCS z180_copystate.c z180_initialstate.c z180_io.c z180_irq.c)
list(APPEND CHIP_CSRCS z180_lowscc.c z180_lowserial.c z180_modifiyreg8.c z180_mmu.c)
list(APPEND CHIP_CSRCS z180_registerdump.c z180_schedulesigaction.c z180_sigdeliver.c)

if(NOT "${CONFIG_SCHED_TICKLESS}" STREQUAL "y")
	list(APPEND CHIP_CSRCS z180_timerisr.c)
endif()
