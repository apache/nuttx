############################################################################
# arch/mips/src/pic32mz/CMakeLists.txt
#
#   Copyright (C) 2015, 2018 Gregory Nutt. All rights reserved.
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

HEAD_ASRC = pic32mz-head.S)

# Common MIPS files

set(CMN_ASRCS)
list(APPEND CMN_ASRCS up_syscall0.S vfork.S)
set(CMN_CSRCS)
list(APPEND CMN_ASRCS up_allocateheap.c up_assert.c up_blocktask.c up_copystate.c)
list(APPEND CMN_CSRCS up_createstack.c up_doirq.c up_exit.c up_initialize.c)
list(APPEND CMN_CSRCS up_initialstate.c up_interruptcontext.c up_irq.c up_lowputs.c)
list(APPEND CMN_CSRCS up_mdelay.c up_modifyreg8.c up_modifyreg16.c up_modifyreg32.c)
list(APPEND CMN_CSRCS up_puts.c up_releasepending.c up_releasestack.c)
list(APPEND CMN_CSRCS up_reprioritizertr.c up_schedulesigaction.c up_sigdeliver.c)
list(APPEND CMN_CSRCS up_stackframe.c up_swint0.c up_udelay.c up_unblocktask.c)
list(APPEND CMN_CSRCS up_usestack.c up_vfork.c)

# Configuration dependent common files

if(NOT "${CONFIG_ARCH_IDLE_CUSTOM}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_idle.c)
endif()

# Use of common/up_etherstub.c is deprecated.  The preferred mechanism is to
# use CONFIG_NETDEV_LATEINIT=y to suppress the call to up_netinitialize() in
# up_initialize().  Then this stub would not be needed.

if(NOT "${CONFIG_PIC32MZ_ETHERNET}" STREQUAL "y")
if("${CONFIG_NET}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_etherstub.c)
endif()
endif()

if("${CONFIG_ARCH_STACKDUMP}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_dumpstate.c)
endif()

# Required PIC32MZ files

set(CHIP_CSRCS)
list(APPEND CHIP_CSRCS pic32mz-lowinit.c pic32mz-exception.c pic32mz-decodeirq.c)
list(APPEND CHIP_CSRCS pic32mz-irq.c pic32mz-timerisr.c pic32mz-gpio.c)
list(APPEND CHIP_CSRCS pic32mz-lowconsole.c pic32mz-serial.c)

# Configuration-dependent PIC32MZ files

if("${CONFIG_PIC32MZ_GPIOIRQ}" STREQUAL "y")
	list(APPEND CHIP_CSRCS pic32mz-gpioirq.c)
endif()

if("${CONFIG_PIC32MZ_SPI}" STREQUAL "y")
	list(APPEND CHIP_CSRCS pic32mz-spi.c)
endif()

if("${CONFIG_PIC32MZ_ETHERNET}" STREQUAL "y")
	list(APPEND CHIP_CSRCS pic32mz-ethernet.c)
endif()
