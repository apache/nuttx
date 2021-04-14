############################################################################
# arch/arm/src/nrf52/CMakeLists.txt
#
#   Copyright (C) 2018 Gregory Nutt. All rights reserved.
#   Author:  Janne Rosberg <janne@offcode.fi>
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

HEAD_ASRC  =

set(CMN_ASRCS)
list(APPEND CMN_ASRCS up_saveusercontext.S up_fullcontextrestore.S up_switchcontext.S)
list(APPEND CMN_ASRCS up_testset.S vfork.S)

set(CMN_CSRCS)
list(APPEND CMN_ASRCS up_assert.c up_blocktask.c up_copyfullstate.c)
list(APPEND CMN_CSRCS up_createstack.c up_doirq.c up_exit.c up_hardfault.c)
list(APPEND CMN_CSRCS up_initialize.c up_initialstate.c up_interruptcontext.c)
list(APPEND CMN_CSRCS up_memfault.c up_mdelay.c up_modifyreg8.c up_modifyreg16.c)
list(APPEND CMN_CSRCS up_modifyreg32.c up_releasepending.c up_releasestack.c)
list(APPEND CMN_CSRCS up_reprioritizertr.c up_schedulesigaction.c up_sigdeliver.c)
list(APPEND CMN_CSRCS up_stackframe.c up_svcall.c up_trigger_irq.c up_udelay.c)
list(APPEND CMN_CSRCS up_unblocktask.c up_usestack.c up_vfork.c)

if("${CONFIG_ARMV7M_LAZYFPU}" STREQUAL "y")
	list(APPEND CMN_ASRCS up_lazyexception.S)
else()
	list(APPEND CMN_ASRCS up_exception.S)
endif()
list(APPEND CMN_CSRCS up_vectors.c)

if("${CONFIG_ARCH_RAMVECTORS}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_ramvec_initialize.c up_ramvec_attach.c)
endif()

if("${CONFIG_BUILD_PROTECTED}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_mpu.c up_task_start.c up_pthread_start.c)
if(NOT "${CONFIG_DISABLE_SIGNALS}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_signal_dispatch.c)
list(APPEND CMN_UASRCS up_signal_handler.S)
endif()
endif()

if("${CONFIG_STACK_COLORATION}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_checkstack.c)
endif()

if("${CONFIG_ARCH_FPU}" STREQUAL "y")
	list(APPEND CMN_ASRCS up_fpu.S)
list(APPEND CMN_CSRCS up_copyarmstate.c)
endif()

set(CHIP_CSRCS)
list(APPEND CHIP_CSRCS nrf52_start.c nrf52_clockconfig.c nrf52_irq.c nrf52_clrpend.c)
list(APPEND CHIP_CSRCS nrf52_allocateheap.c nrf52_lowputc.c nrf52_gpio.c nrf52_nvmc.c)

if(NOT "${CONFIG_SCHED_TICKLESS}" STREQUAL "y")
	list(APPEND CHIP_CSRCS nrf52_timerisr.c)
else()
	list(APPEND CHIP_CSRCS nrf52_tickless.c)
endif()

if("${CONFIG_BUILD_PROTECTED}" STREQUAL "y")
	list(APPEND CHIP_CSRCS nrf52_userspace.c nrf52_mpuinit.c)
endif()

if(NOT "${CONFIG_ARCH_IDLE_CUSTOM}" STREQUAL "y")
	list(APPEND CHIP_CSRCS nrf52_idle.c)
endif()

if("${CONFIG_NRF52_HAVE_UART}" STREQUAL "y")
	list(APPEND CHIP_CSRCS nrf52_serial.c)
endif()

if("${CONFIG_NRF52_PROGMEM}" STREQUAL "y")
CHIP_CSRCS  += nrf52_flash.c)
endif()

if("${CONFIG_NRF52_WDT}" STREQUAL "y")
	list(APPEND CHIP_CSRCS nrf52_wdt.c)
endif()
