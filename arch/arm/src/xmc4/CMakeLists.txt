############################################################################
# arch/arm/src/kinetis/CMakeLists.txt
#
#   Copyright (C) 2017-2018 Gregory Nutt. All rights reserved.
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

HEAD_ASRC =

CMN_UASRCS =
CMN_UCSRCS =

set(CMN_ASRCS)
list(APPEND CMN_ASRCS up_saveusercontext.S up_fullcontextrestore.S up_switchcontext.S)
list(APPEND CMN_ASRCS up_testset.S up_fetchadd.S vfork.S)

set(CMN_CSRCS)
list(APPEND CMN_ASRCS up_assert.c up_blocktask.c up_copyfullstate.c up_createstack.c)
list(APPEND CMN_CSRCS up_doirq.c up_exit.c up_initialize.c up_initialstate.c)
list(APPEND CMN_CSRCS up_hardfault.c up_interruptcontext.c up_memfault.c up_mdelay.c)
list(APPEND CMN_CSRCS up_modifyreg8.c up_modifyreg16.c up_modifyreg32.c)
list(APPEND CMN_CSRCS up_releasestack.c up_reprioritizertr.c up_schedulesigaction.c)
list(APPEND CMN_CSRCS up_releasepending.c up_sigdeliver.c up_stackframe.c up_svcall.c)
list(APPEND CMN_CSRCS up_systemreset.c up_udelay.c up_unblocktask.c up_usestack.c)
list(APPEND CMN_CSRCS up_vfork.c)

if("${CONFIG_ARMV7M_STACKCHECK}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_stackcheck.c)
endif()

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

# Use of common/up_etherstub.c is deprecated.  The preferred mechanism is to
# use CONFIG_NETDEV_LATEINIT=y to suppress the call to up_netinitialize() in
# up_initialize().  Then this stub would not be needed.

if("${CONFIG_NET}" STREQUAL "y")
if(NOT "${CONFIG_XMC4_ENET}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_etherstub.c)
endif()
endif()

if("${CONFIG_ARCH_FPU}" STREQUAL "y")
	list(APPEND CMN_ASRCS up_fpu.S)
list(APPEND CMN_CSRCS up_copyarmstate.c)
endif()

if("${CONFIG_ARMV7M_ITMSYSLOG}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_itm_syslog.c)
endif()

# Required XMC4xxx files


set(CHIP_CSRCS)
list(APPEND CHIP_CSRCS xmc4_allocateheap.c xmc4_clockconfig.c xmc4_clockutils.c)
list(APPEND CHIP_CSRCS xmc4_clrpend.c xmc4_irq.c xmc4_lowputc.c xmc4_gpio.c)
list(APPEND CHIP_CSRCS xmc4_serial.c xmc4_start.c xmc4_usic.c)

# Configuration-dependent Kinetis files

if(NOT "${CONFIG_ARCH_IDLE_CUSTOM}" STREQUAL "y")
	list(APPEND CHIP_CSRCS xmc4_idle.c)
endif()

if(NOT "${CONFIG_SCHED_TICKLESS}" STREQUAL "y")
	list(APPEND CHIP_CSRCS xmc4_timerisr.c)
endif()

if("${CONFIG_BUILD_PROTECTED}" STREQUAL "y")
	list(APPEND CHIP_CSRCS xmc4_userspace.c xmc4_mpuinit.c)
endif()

if("${CONFIG_DEBUG_GPIO_INFO}" STREQUAL "y")
	list(APPEND CHIP_CSRCS xmc4_pindump.c)
endif()

if("${CONFIG_XMC4_DMA}" STREQUAL "y")
	list(APPEND CHIP_CSRCS xmc4_dma.c)
endif()

if("${CONFIG_PWM}" STREQUAL "y")
	list(APPEND CHIP_CSRCS xmc4_pwm.c)
endif()

if("${CONFIG_I2C}" STREQUAL "y")
	list(APPEND CHIP_CSRCS xmc4_i2c.c)
endif()

if("${CONFIG_XMC4_USCI_SPI}" STREQUAL "y")
	list(APPEND CHIP_CSRCS xmc4_spi.c)
endif()
