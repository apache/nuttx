############################################################################
# arch/z80/src/ez80/CMakeLists.txt
#
#   Copyright (C) 2008-2009, 2014 Gregory Nutt. All rights reserved.
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

HEAD_ASRC = ez80_vectors.asm
HEAD_SSRC =

CMN_SSRCS  =
set(CMN_CSRCS)
list(APPEND CMN_ASRCS up_initialize.c up_allocateheap.c up_createstack.c)
list(APPEND CMN_CSRCS up_releasestack.c up_interruptcontext.c up_blocktask.c)
list(APPEND CMN_CSRCS up_unblocktask.c up_exit.c up_releasepending.c)
list(APPEND CMN_CSRCS up_reprioritizertr.c up_idle.c up_assert.c up_doirq.c)
list(APPEND CMN_CSRCS up_mdelay.c up_stackframe.c up_udelay.c up_usestack.c)
list(APPEND CMN_CSRCS up_puts.c)

CHIP_ASRCS  = ez80_startup.asm ez80_io.asm ez80_irqsave.asm
CHIP_ASRCS += ez80_saveusercontext.asm ez80_restorecontext.asm
if("${CONFIG_ARCH_CHIP_EZ80F91}" STREQUAL "y")
CHIP_ASRCS += ez80f91_init.asm
endif()

CHIP_SSRCS  =
set(CHIP_CSRCS)
list(APPEND CHIP_CSRCS ez80_clock.c ez80_initialstate.c ez80_irq.c ez80_copystate.c)
list(APPEND CHIP_CSRCS ez80_schedulesigaction.c ez80_sigdeliver.c ez80_lowuart.c)
list(APPEND CHIP_CSRCS ez80_serial.c ez80_spi.c ez80_i2c.c ez80_registerdump.c)

if(NOT "${CONFIG_SCHED_TICKLESS}" STREQUAL "y")
	list(APPEND CHIP_CSRCS ez80_timerisr.c)
endif()

if("${CONFIG_ARCH_CHIP_EZ80F91}" STREQUAL "y")
if("${CONFIG_EZ80_EMAC}" STREQUAL "y")
CHIP_CSRCS	+= ez80_emac.c)
endif()
endif()
