############################################################################
# arch/avr/src/at32uc3/CMakeLists.txt
#
#   Copyright (C) 2010, 2014 Gregory Nutt. All rights reserved.
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

# The start-up, "head", file

HEAD_ASRC = up_nommuhead.S)

# Common AVR/AVR32 files

set(CMN_ASRCS)
list(APPEND CMN_ASRCS up_exceptions.S up_fullcontextrestore.S up_switchcontext.S)
set(CMN_CSRCS)
list(APPEND CMN_ASRCS up_assert.c up_allocateheap.c up_blocktask.c up_copystate.c)
list(APPEND CMN_CSRCS up_createstack.c up_mdelay.c up_udelay.c up_exit.c up_idle.c)
list(APPEND CMN_CSRCS up_initialize.c up_initialstate.c up_interruptcontext.c)
list(APPEND CMN_CSRCS up_modifyreg8.c up_modifyreg16.c up_modifyreg32.c)
list(APPEND CMN_CSRCS up_releasepending.c up_releasestack.c up_reprioritizertr.c)
list(APPEND CMN_CSRCS up_schedulesigaction.c up_sigdeliver.c up_stackframe.c)
list(APPEND CMN_CSRCS up_unblocktask.c up_usestack.c up_doirq.c)

# Configuration-dependent common files

if("${CONFIG_ARCH_STACKDUMP}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_dumpstate.c)
endif()

# Required AT32UC3 files

set(CHIP_CSRCS)
list(APPEND CHIP_CSRCS at32uc3_clkinit.c at32uc3_gpio.c at32uc3_irq.c)
list(APPEND CHIP_CSRCS at32uc3_lowconsole.c at32uc3_lowinit.c at32uc3_serial.c)

# Configuration-dependent AT32UC3 files

if(NOT "${CONFIG_SCHED_TICKLESS}" STREQUAL "y")
	list(APPEND CHIP_CSRCS at32uc3_timerisr.c)
endif()

if("${CONFIG_AVR32_GPIOIRQ}" STREQUAL "y")
	list(APPEND CHIP_CSRCS at32uc3_gpioirq.c)
endif()
