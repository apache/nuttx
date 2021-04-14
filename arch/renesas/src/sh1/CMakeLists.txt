##############################################################################
# arch/renesas/src/sh1/CMakeLists.txt
#
#   Copyright (C) 2008, 2009, 2014 Gregory Nutt. All rights reserved.
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
##############################################################################

HEAD_ASRC  = sh1_head.S)

CMN_ASRCS  =
set(CMN_CSRCS)
list(APPEND CMN_ASRCS up_allocateheap.c up_assert.c up_blocktask.c)
list(APPEND CMN_CSRCS up_createstack.c up_doirq.c up_exit.c up_idle.c up_initialize.c)
list(APPEND CMN_CSRCS up_initialstate.c up_interruptcontext.c up_lowputs.c)
list(APPEND CMN_CSRCS up_mdelay.c up_puts.c up_releasepending.c up_releasestack.c)
list(APPEND CMN_CSRCS up_reprioritizertr.c up_stackframe.c up_udelay.c)
list(APPEND CMN_CSRCS up_unblocktask.c up_usestack.c)

CHIP_ASRCS  = sh1_vector.S sh1_saveusercontext.S)
set(CHIP_CSRCS)
list(APPEND CHIP_CSRCS sh1_lowputc.c sh1_irq.c sh1_serial.c sh1_initialstate.c)
list(APPEND CHIP_CSRCS sh1_copystate.c sh1_dumpstate.c)

if(NOT "${CONFIG_SCHED_TICKLESS}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sh1_timerisr.c)
endif()

if(NOT "${CONFIG_DISABLE_SIGNALS}" STREQUAL "y")
	list(APPEND CMN_CSRCS sh1_schedulesigaction.c sh1_sigdeliver.c)
endif()

if("${CONFIG_USBDEV}" STREQUAL "y")
CHIP_CSRCS +=
endif()

