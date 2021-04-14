############################################################################
# arch/avr/src/at90usb/CMakeLists.txt
#
#   Copyright (C) 2011, 2014 Gregory Nutt. All rights reserved.
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

HEAD_ASRC = at90usb_head.S)

# Common AVR files

set(CMN_ASRCS)
list(APPEND CMN_ASRCS up_switchcontext.S)
set(CMN_CSRCS)
list(APPEND CMN_ASRCS up_allocateheap.c up_assert.c up_blocktask.c up_copystate.c)
list(APPEND CMN_CSRCS up_createstack.c up_doirq.c up_exit.c  up_idle.c up_initialize.c)
list(APPEND CMN_CSRCS up_initialstate.c up_interruptcontext.c up_irq.c up_lowputs.c)
list(APPEND CMN_CSRCS up_mdelay.c up_modifyreg8.c up_modifyreg16.c up_modifyreg32.c)
list(APPEND CMN_CSRCS up_puts.c up_releasepending.c up_releasestack.c)
list(APPEND CMN_CSRCS up_reprioritizertr.c up_schedulesigaction.c up_sigdeliver.c)
list(APPEND CMN_CSRCS up_stackframe.c up_udelay.c up_unblocktask.c up_usestack.c)

# Configuration-dependent common files

if("${CONFIG_ARCH_STACKDUMP}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_dumpstate.c)
endif()

if("${CONFIG_AVR_SPI}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_spi.c)
endif()

if("${CONFIG_STACK_COLORATION}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_checkstack.c)
endif()

if("${CONFIG_ARCH_ROMGETC}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_romgetc.c)
endif()

# Required AT90USB files

CHIP_ASRCS = at90usb_exceptions.S)
CHIP_CSRCS = at90usb_lowconsole.c at90usb_lowinit.c at90usb_serial.c)

# Configuration-dependent aT90USB files

if(NOT "${CONFIG_SCHED_TICKLESS}" STREQUAL "y")
	list(APPEND CHIP_CSRCS at90usb_timerisr.c)
endif()

if("${CONFIG_AVR_USBDEV}" STREQUAL "y")
	list(APPEND CHIP_CSRCS at90usb_usbdev.c)
endif()
