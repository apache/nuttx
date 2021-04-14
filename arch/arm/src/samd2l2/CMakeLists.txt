############################################################################
# arch/arm/src/samd2l2/CMakeLists.txt
#
#   Copyright (C) 2014-2015, 2017-2018 Gregory Nutt. All rights reserved.
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

set(CMN_ASRCS)
list(APPEND CMN_ASRCS up_exception.S up_saveusercontext.S up_fullcontextrestore.S)
list(APPEND CMN_ASRCS up_switchcontext.S vfork.S)

set(CMN_CSRCS)
list(APPEND CMN_ASRCS up_allocateheap.c up_assert.c up_blocktask.c up_copyfullstate.c)
list(APPEND CMN_CSRCS up_createstack.c up_mdelay.c up_udelay.c up_exit.c)
list(APPEND CMN_CSRCS up_initialize.c up_initialstate.c up_interruptcontext.c)
list(APPEND CMN_CSRCS up_modifyreg8.c up_modifyreg16.c up_modifyreg32.c)
list(APPEND CMN_CSRCS up_releasepending.c up_releasestack.c up_reprioritizertr.c)
list(APPEND CMN_CSRCS up_schedulesigaction.c up_sigdeliver.c up_stackframe.c)
list(APPEND CMN_CSRCS up_systemreset.c up_unblocktask.c up_usestack.c up_doirq.c)
list(APPEND CMN_CSRCS up_hardfault.c up_svcall.c up_vectors.c up_vfork.c)

if("${CONFIG_BUILD_PROTECTED}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_task_start.c up_pthread_start.c)
if(NOT "${CONFIG_DISABLE_SIGNALS}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_signal_dispatch.c)
list(APPEND CMN_UASRCS up_signal_handler.S)
endif()
endif()

if("${CONFIG_STACK_COLORATION}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_checkstack.c)
endif()

if("${CONFIG_DEBUG_FEATURES}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_dumpnvic.c)
endif()

set(CHIP_CSRCS)
list(APPEND CHIP_CSRCS sam_irq.c sam_lowputc.c sam_port.c sam_sercom.c sam_serial.c)
list(APPEND CHIP_CSRCS sam_start.c sam_usart.c)

# Configuration-dependent SAM D/L files

if("${CONFIG_ARCH_FAMILY_SAMD20}" STREQUAL "y")
	list(APPEND CHIP_CSRCS samd_clockconfig.c samd_gclk.c)
elseif("${CONFIG_ARCH_FAMILY_SAMD21}" STREQUAL "y")
	list(APPEND CHIP_CSRCS samd_clockconfig.c samd_gclk.c)
elseif("${CONFIG_ARCH_FAMILY_SAML21}" STREQUAL "y")
	list(APPEND CHIP_CSRCS saml_clockconfig.c saml_gclk.c)
endif()

if(NOT "${CONFIG_ARCH_IDLE_CUSTOM}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_idle.c)
endif()

if(NOT "${CONFIG_SCHED_TICKLESS}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_timerisr.c)
endif()

if("${CONFIG_BUILD_PROTECTED}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_userspace.c)
endif()

if("${CONFIG_ARCH_IRQPRIO}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_irqprio.c)
endif()

if("${CONFIG_SAMD2L2_DMAC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_dmac.c)
endif()

if("${CONFIG_SAMD2L2_HAVE_SPI}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_spi.c)
endif()

if("${CONFIG_SAMD2L2_HAVE_I2C}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_i2c_master.c)
endif()

if("${CONFIG_SAMD2L2_USB}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_usb.c)
endif()

if("${CONFIG_SAMD2L2_EIC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_eic.c)
endif()

if("${CONFIG_SAMD2L2_AC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS sam_ac.c)
endif()
