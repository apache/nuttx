##############################################################################
# lpc23xx/CMakeLists.txt
#
#   Copyright (C) 2010 Rommel Marcelo. All rights reserved.
#   Author: Rommel Marcelo
#
# This file is part of the NuttX RTOS and based on the lpc2148 port:
#
#   Copyright (C) 2010, 2013-2014 Gregory Nutt. All rights reserved.
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

HEAD_ASRC = lpc23xx_head.S)

set(CMN_ASRCS)
list(APPEND CMN_ASRCS up_saveusercontext.S up_fullcontextrestore.S up_vectors.S)
list(APPEND CMN_ASRCS vfork.S)
set(CMN_CSRCS)
list(APPEND CMN_ASRCS up_allocateheap.c up_assert.c up_blocktask.c up_copyfullstate.c)
list(APPEND CMN_CSRCS up_createstack.c up_dataabort.c up_mdelay.c up_udelay.c)
list(APPEND CMN_CSRCS up_exit.c up_initialize.c up_initialstate.c)
list(APPEND CMN_CSRCS up_interruptcontext.c up_prefetchabort.c up_releasepending.c)
list(APPEND CMN_CSRCS up_releasestack.c up_reprioritizertr.c up_stackframe.c)
list(APPEND CMN_CSRCS up_syscall.c up_unblocktask.c up_undefinedinsn.c)
list(APPEND CMN_CSRCS up_usestack.c up_lowputs.c up_vfork.c)

if(NOT "${CONFIG_ARCH_IDLE_CUSTOM}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_idle.c)
endif()

if(NOT "${CONFIG_DISABLE_SIGNALS}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_schedulesigaction.c up_sigdeliver.c)
endif()

if("${CONFIG_STACK_COLORATION}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_checkstack.c)
endif()

CHIP_ASRCS  = lpc23xx_lowputc.S)
list(APPEND CHIP_CSRCS lpc23xx_pllsetup.c  lpc23xx_decodeirq.c lpc23xx_irq.c)
list(APPEND CHIP_CSRCS lpc23xx_serial.c lpc23xx_io.c)

if(NOT "${CONFIG_SCHED_TICKLESS}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc23xx_timerisr.c)
endif()

if("${CONFIG_LPC2378_SPI}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc23xx_spi.c)
endif()

if("${CONFIG_I2C}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc23xx_i2c.c)
endif()

if("${CONFIG_USBDEV}" STREQUAL "y")
#list(APPEND CHIP_CSRCS lpc23xx_usbdev.c)
endif()
