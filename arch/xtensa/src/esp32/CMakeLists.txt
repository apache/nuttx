############################################################################
# arch/xtensa/src/lx6/CMakeLists.txt
#
#   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

# The start-up, "head", file.  May be either a .S or a .c file.

HEAD_ASRC  = xtensa_vectors.S xtensa_window_vector.S xtensa_windowspill.S)
HEAD_ASRC += xtensa_int_handlers.S  xtensa_user_handler.S)
HEAD_CSRC  = esp32_start.c)

# Common XTENSA files (arch/xtensa/src/common)

set(CMN_ASRCS)
list(APPEND CMN_ASRCS xtensa_context.S xtensa_coproc.S xtensa_cpuint.S xtensa_panic.S)
list(APPEND CMN_ASRCS xtensa_sigtramp.S)

set(CMN_CSRCS)
list(APPEND CMN_ASRCS xtensa_assert.c xtensa_blocktask.c xtensa_copystate.c)
list(APPEND CMN_CSRCS xtensa_cpenable.c xtensa_createstack.c xtensa_exit.c xtensa_idle.c)
list(APPEND CMN_CSRCS xtensa_initialize.c xtensa_initialstate.c xtensa_interruptcontext.c)
list(APPEND CMN_CSRCS xtensa_irqdispatch.c xtensa_lowputs.c xtensa_mdelay.c)
list(APPEND CMN_CSRCS xtensa_modifyreg8.c xtensa_modifyreg16.c xtensa_modifyreg32.c)
list(APPEND CMN_CSRCS xtensa_puts.c xtensa_releasepending.c xtensa_releasestack.c)
list(APPEND CMN_CSRCS xtensa_reprioritizertr.c xtensa_schedsigaction.c)
list(APPEND CMN_CSRCS xtensa_sigdeliver.c xtensa_stackframe.c xtensa_udelay.c)
list(APPEND CMN_CSRCS xtensa_unblocktask.c xtensa_usestack.c)

# Configuration-dependent common XTENSA files

if("${CONFIG_DEBUG_ALERT}" STREQUAL "y")
  list(APPEND CMN_CSRCS xtensa_dumpstate.c)
endif()

if("${CONFIG_SPINLOCK}" STREQUAL "y")
  list(APPEND CMN_CSRCS xtensa_testset.c)
endif()

if("${CONFIG_SMP}" STREQUAL "y")
  list(APPEND CMN_CSRCS xtensa_cpupause.c)
endif()

if("${CONFIG_STACK_COLORATION}" STREQUAL "y")
  list(APPEND CMN_CSRCS xtensa_checkstack.c)
endif()


# Use of common/xtensa_etherstub.c is deprecated.  The preferred mechanism
# is to use CONFIG_NETDEV_LATEINIT=y to suppress the call to
# up_netinitialize() in xtensa_initialize.c.  Then this stub would not be
# needed.

if(NOT "${CONFIG_LX6_ETHERNET}" STREQUAL "y")
if("${CONFIG_NET}" STREQUAL "y")
  list(APPEND CMN_CSRCS xtensa_etherstub.c)
endif()
endif()

# Required ESP32 files (arch/xtensa/src/lx6)

set(CHIP_CSRCS)
list(APPEND CHIP_CSRCS esp32_allocateheap.c esp32_clockconfig.c esp32_cpuint.c)
list(APPEND CHIP_CSRCS esp32_gpio.c esp32_intdecode.c esp32_irq.c esp32_region.c)
list(APPEND CHIP_CSRCS esp32_timerisr.c)

# Configuration-dependent ESP32 files

if("${CONFIG_SMP}" STREQUAL "y")
CHIP_ASRCS  = esp32_cpuindex.S)
CMN_CSRCS  += esp32_cpuidlestack.c esp32_cpustart.c esp32_intercpu_interrupt.c)
endif()

if("${CONFIG_ESP32_UART}" STREQUAL "y")
	list(APPEND CMN_CSRCS esp32_serial.c)
endif()
