############################################################################
# arch/arm/src/tiva/CMakeLists.txt
#
#   Copyright (C) 2009-2011, 2013-2014, 2018 Gregory Nutt. All rights
#     reserved.
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

HEAD_ASRC  =

set(CMN_ASRCS)
list(APPEND CMN_ASRCS up_saveusercontext.S up_fullcontextrestore.S up_switchcontext.S)
list(APPEND CMN_ASRCS up_testset.S up_fetchadd.S  vfork.S)

set(CMN_CSRCS)
list(APPEND CMN_ASRCS up_assert.c up_blocktask.c up_copyfullstate.c up_createstack.c)
list(APPEND CMN_CSRCS up_doirq.c up_exit.c up_hardfault.c up_initialize.c)
list(APPEND CMN_CSRCS up_initialstate.c up_interruptcontext.c up_mdelay.c)
list(APPEND CMN_CSRCS up_memfault.c up_modifyreg8.c up_modifyreg16.c up_modifyreg32.c)
list(APPEND CMN_CSRCS up_releasepending.c up_releasestack.c up_reprioritizertr.c)
list(APPEND CMN_CSRCS up_schedulesigaction.c up_sigdeliver.c up_stackframe.c)
list(APPEND CMN_CSRCS up_svcall.c up_trigger_irq.c up_unblocktask.c up_udelay.c)
list(APPEND CMN_CSRCS up_usestack.c up_vfork.c)

if(NOT "${CONFIG_ARCH_IDLE_CUSTOM}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_idle.c)
endif()

if("${CONFIG_ARMV7M_LAZYFPU}" STREQUAL "y")
	list(APPEND CMN_ASRCS up_lazyexception.S)
else()
	list(APPEND CMN_ASRCS up_exception.S)
endif()
list(APPEND CMN_CSRCS up_vectors.c)

if("${CONFIG_ARCH_FPU}" STREQUAL "y")
	list(APPEND CMN_ASRCS up_fpu.S)
list(APPEND CMN_CSRCS up_copyarmstate.c)
endif()

if("${CONFIG_ARCH_RAMVECTORS}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_ramvec_initialize.c up_ramvec_attach.c)
endif()

if("${CONFIG_STACK_COLORATION}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_checkstack.c)
endif()

if("${CONFIG_BUILD_PROTECTED}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_mpu.c up_task_start.c up_pthread_start.c)
if(NOT "${CONFIG_DISABLE_SIGNALS}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_signal_dispatch.c)
list(APPEND CMN_UASRCS up_signal_handler.S)
endif()
else()
	list(APPEND CMN_CSRCS up_allocateheap.c)
endif()

set(CHIP_CSRCS)
list(APPEND CHIP_CSRCS tiva_allocateheap.c tiva_irq.c tiva_lowputc.c tiva_serial.c 
list(APPEND CHIP_CSRCS tiva_ssi.c)

if("${CONFIG_ARCH_CHIP_LM3S}" STREQUAL "y")
	list(APPEND CHIP_CSRCS tiva_start.c lm3s_gpio.c tiva_gpioirq.c)
elseif("${CONFIG_ARCH_CHIP_LM4F}" STREQUAL "y")
	list(APPEND CHIP_CSRCS tiva_start.c lm4f_gpio.c tiva_gpioirq.c)
elseif("${CONFIG_ARCH_CHIP_TM4C}" STREQUAL "y")
	list(APPEND CHIP_CSRCS tiva_start.c tm4c_gpio.c tiva_gpioirq.c)
elseif("${CONFIG_ARCH_CHIP_CC13X0}" STREQUAL "y")
	list(APPEND CHIP_CSRCS cc13xx_start.c cc13xx_gpio.c cc13xx_gpioirq.c)
elseif("${CONFIG_ARCH_CHIP_CC13X2}" STREQUAL "y")
	list(APPEND CHIP_CSRCS cc13xx_start.c cc13xx_gpio.c cc13xx_gpioirq.c)
endif()

if("${CONFIG_DEBUG_GPIO_INFO}" STREQUAL "y")
	list(APPEND CHIP_CSRCS tiva_dumpgpio.c)
endif()

if("${CONFIG_ARCH_CHIP_TM4C129}" STREQUAL "y")
	list(APPEND CHIP_CSRCS tm4c129_sysctrl.c)
else()
	list(APPEND CHIP_CSRCS tiva_sysctrl.c)
endif()

if(NOT "${CONFIG_SCHED_TICKLESS}" STREQUAL "y")
	list(APPEND CHIP_CSRCS tiva_timerisr.c)
endif()

if("${CONFIG_BUILD_PROTECTED}" STREQUAL "y")
	list(APPEND CHIP_CSRCS tiva_userspace.c tiva_mpuinit.c)
endif()

if("${CONFIG_TIVA_I2C}" STREQUAL "y")
	list(APPEND CHIP_CSRCS tiva_i2c.c)
endif()

if("${CONFIG_TIVA_PWM}" STREQUAL "y")
	list(APPEND CHIP_CSRCS tiva_pwm.c)
endif()

if("${CONFIG_TIVA_QEI}" STREQUAL "y")
	list(APPEND CHIP_CSRCS tiva_qencoder.c)
endif()

if("${CONFIG_TIVA_TIMER}" STREQUAL "y")
	list(APPEND CHIP_CSRCS tiva_timerlib.c)
if("${CONFIG_TIVA_TIMER32_PERIODIC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS tiva_timerlow32.c)
endif()
endif()

if("${CONFIG_TIVA_ADC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS tiva_adclow.c)
list(APPEND CHIP_CSRCS tiva_adclib.c)
endif()

if("${CONFIG_TIVA_ETHERNET}" STREQUAL "y")
if("${CONFIG_ARCH_CHIP_LM3S}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lm3s_ethernet.c)
endif()
if("${CONFIG_ARCH_CHIP_TM4C}" STREQUAL "y")
	list(APPEND CHIP_CSRCS tm4c_ethernet.c)
endif()
endif()

if("${CONFIG_TIVA_FLASH}" STREQUAL "y")
	list(APPEND CHIP_CSRCS tiva_flash.c)
endif()

if("${CONFIG_TIVA_EEPROM}" STREQUAL "y")
	list(APPEND CHIP_CSRCS tiva_eeprom.c)
endif()

# Paths to source files

VPATH += chip/common

if("${CONFIG_ARCH_CHIP_LM}" STREQUAL "y")
VPATH += chip/lm
elseif("${CONFIG_ARCH_CHIP_TM4C}" STREQUAL "y")
VPATH += chip/tm4c
elseif("${CONFIG_ARCH_CHIP_CC13X0}" STREQUAL "y")
VPATH += chip/cc13x0
elseif("${CONFIG_ARCH_CHIP_CC13X2}" STREQUAL "y")
VPATH += chip/cc13x2
endif()
