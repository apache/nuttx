############################################################################
# arch/arm/src/moxart/CMakeLists.txt
#
#   Copyright (C) 2007, 2013-2015, 2018 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
#
#   Copyright (C) 2011 Stefan Richter. All rights reserved.
#   Author: Stefan Richter <ichgeh@l--putt.de>
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

HEAD_ASRC = moxart_head.S)

set(CMN_ASRCS)
list(APPEND CMN_ASRCS up_saveusercontext.S up_fullcontextrestore.S up_vectors.S)
list(APPEND CMN_ASRCS up_nommuhead.S vfork.S)
set(CMN_CSRCS)
list(APPEND CMN_ASRCS up_allocateheap.c up_assert.c up_blocktask.c up_copyfullstate.c)
list(APPEND CMN_CSRCS up_createstack.c up_dataabort.c up_mdelay.c up_udelay.c)
list(APPEND CMN_CSRCS up_exit.c up_initialstate.c up_initialize.c)
list(APPEND CMN_CSRCS up_interruptcontext.c up_prefetchabort.c up_releasepending.c)
list(APPEND CMN_CSRCS up_releasestack.c up_reprioritizertr.c up_schedulesigaction.c)
list(APPEND CMN_CSRCS up_sigdeliver.c up_stackframe.c up_syscall.c up_unblocktask.c)
list(APPEND CMN_CSRCS up_undefinedinsn.c up_usestack.c up_vfork.c up_etherstub.c)

CHIP_ASRCS  = moxart_lowputc.S)

set(CHIP_CSRCS)
list(APPEND CHIP_CSRCS moxart_16550.c moxart_irq.c moxart_timer.c moxart_systemreset.c)

# Configuration-dependent Kinetis K files

if(NOT "${CONFIG_ARCH_IDLE_CUSTOM}" STREQUAL "y")
	list(APPEND CHIP_CSRCS moxart_idle.c)
endif()

