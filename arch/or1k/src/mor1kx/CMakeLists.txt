############################################################################
# arch/or1k/src/mor1kx/CMakeLists.txt
#
#   Copyright (C) 2014-2015, 2017 Gregory Nutt. All rights reserved.
#   Author: Matt Thompson <matt@extent3d.com>
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

HEAD_ASRC =

set(CMN_ASRCS)
list(APPEND CMN_ASRCS up_vectortab.S \
             up_saveusercontext.S \
             up_fullcontextrestore.S)

set(CMN_CSRCS)
list(APPEND CMN_ASRCS up_initialize.c \
             up_interruptcontext.c \
             up_allocateheap.c \
             up_createstack.c \
             up_releasestack.c \
             up_stackframe.c \
             up_initialstate.c \
             up_blocktask.c \
             up_unblocktask.c \
             up_releasepending.c \
             up_schedulesigaction.c \
             up_copyfullstate.c \
             up_reprioritizertr.c \
             up_assert.c \
             up_exit.c \
             up_udelay.c \
             up_mdelay.c \
             up_idle.c \
             up_irq.c \
             up_uart.c \
             up_timer.c \
             up_doirq.c \
             up_cpuinfo.c \
             mor1kx_start.c)

if("${CONFIG_BUILD_PROTECTED}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_task_start.c up_pthread_start.c)
if(NOT "${CONFIG_DISABLE_SIGNALS}" STREQUAL "y")
#list(APPEND CMN_CSRCS up_signal_dispatch.c)
#list(APPEND CMN_UASRCS up_signal_handler.S)
endif()
endif()

if("${CONFIG_STACK_COLORATION}" STREQUAL "y")
#list(APPEND CMN_CSRCS up_checkstack.c)
endif()

if("${CONFIG_DEBUG_FEATURES}" STREQUAL "y")
#list(APPEND CMN_CSRCS up_dumpnvic.c)
endif()

CHIP_CSRCS  =

if(NOT "${CONFIG_SCHED_TICKLESS}" STREQUAL "y")
#list(APPEND CHIP_CSRCS sam_timerisr.c)
endif()

if("${CONFIG_BUILD_PROTECTED}" STREQUAL "y")
#list(APPEND CHIP_CSRCS sam_userspace.c)
endif()

if("${CONFIG_ARCH_IRQPRIO}" STREQUAL "y")
#list(APPEND CHIP_CSRCS sam_irqprio.c)
endif()

