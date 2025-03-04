# ##############################################################################
# arch/arm/src/tiva/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more contributor
# license agreements.  See the NOTICE file distributed with this work for
# additional information regarding copyright ownership.  The ASF licenses this
# file to you under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License.  You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations under
# the License.
#
# ##############################################################################

if(CONFIG_ARCH_CHIP_LM)
  set(ARCH_CHIP lm)
elseif(CONFIG_ARCH_CHIP_TM4C)
  set(ARCH_CHIP tm4c)
elseif(CONFIG_ARCH_CHIP_CC13X0)
  set(ARCH_CHIP cc13xx)
elseif(CONFIG_ARCH_CHIP_CC13X2)
  set(ARCH_CHIP cc13xx)
endif()

set(SRCS)

if(NOT CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS tiva_idle.c)
endif()

list(APPEND SRCS tiva_allocateheap.c tiva_irq.c tiva_lowputc.c tiva_serial.c)
list(APPEND SRCS tiva_ssi.c)

if(CONFIG_ARCH_CHIP_LM3S)
  list(APPEND SRCS lmxx_tm4c_start.c lm3s_gpio.c lmxx_tm4c_gpioirq.c)
  list(APPEND SRCS lm4xx_tm3c_sysctrl.c)
elseif(CONFIG_ARCH_CHIP_LM4F)
  list(APPEND SRCS lmxx_tm4c_start.c lm4f_gpio.c lmxx_tm4c_gpioirq.c)
  list(APPEND SRCS lm4xx_tm3c_sysctrl.c)
elseif(CONFIG_ARCH_CHIP_TM4C)
  list(APPEND SRCS lmxx_tm4c_start.c tm4c_gpio.c lmxx_tm4c_gpioirq.c)
  if(CONFIG_ARCH_CHIP_TM4C129)
    list(APPEND SRCS tm4c129_sysctrl.c)
  else()
    list(APPEND SRCS lm4xx_tm3c_sysctrl.c)
  endif()

elseif(CONFIG_ARCH_CHIP_CC13X0)
  list(APPEND SRCS cc13xx_start.c cc13xx_prcm.c cc13xx_chipinfo.c cc13xx_gpio.c)
  list(APPEND SRCS cc13xx_gpioirq.c cc13xx_enableclks.c cc13xx_enablepwr.c)
  list(APPEND SRCS cc13x0_trim.c cc13x0_rom.c)
elseif(CONFIG_ARCH_CHIP_CC13X2)
  list(APPEND SRCS cc13xx_start.c cc13xx_prcm.c cc13xx_chipinfo.c cc13xx_gpio.c)
  list(APPEND SRCS cc13xx_gpioirq.c cc13xx_enableclks.c cc13xx_enablepwr.c)
  list(APPEND SRCS cc13x2_aux_sysif.c)
  if(CONFIG_ARCH_CHIP_CC13XX_V1)
    list(APPEND SRCS cc13x2_v1_trim.c cc13x2_cc26x2_v1_rom.c)
  else()
    list(APPEND SRCS cc13x2_v2_trim.c)
  endif()
endif()

if(CONFIG_DEBUG_GPIO_INFO)
  list(APPEND SRCS tiva_dumpgpio.c)
endif()

if(NOT CONFIG_SCHED_TICKLESS)
  list(APPEND SRCS tiva_timerisr.c)
endif()

if(CONFIG_BUILD_PROTECTED)
  list(APPEND SRCS tiva_userspace.c tiva_mpuinit.c)
endif()

if(CONFIG_TIVA_I2C)
  list(APPEND SRCS tiva_i2c.c)
endif()

if(CONFIG_TIVA_PWM)
  list(APPEND SRCS tiva_pwm.c)
endif()

if(CONFIG_TIVA_QEI)
  list(APPEND SRCS tiva_qencoder.c)
endif()

if(CONFIG_TIVA_TIMER)
  list(APPEND SRCS tiva_timerlib.c)
  if(CONFIG_TIVA_TIMER32_PERIODIC)
    list(APPEND SRCS tiva_timerlow32.c)
  endif()
endif()

if(CONFIG_TIVA_ADC)
  list(APPEND SRCS tiva_adclow.c)
  list(APPEND SRCS tiva_adclib.c)
endif()

if(CONFIG_TIVA_CAN)
  list(APPEND SRCS tiva_can.c)
endif()

if(CONFIG_TIVA_ETHERNET)
  if(CONFIG_ARCH_CHIP_LM3S)
    list(APPEND SRCS lm3s_ethernet.c)
  endif()
  if(CONFIG_ARCH_CHIP_TM4C)
    list(APPEND SRCS tm4c_ethernet.c)
  endif()
endif()

if(CONFIG_TIVA_FLASH)
  list(APPEND SRCS tiva_flash.c)
endif()

if(CONFIG_TIVA_EEPROM)
  list(APPEND SRCS tiva_eeprom.c)
endif()

if(CONFIG_TIVA_HCIUART)
  list(APPEND SRCS tiva_hciuart.c)
endif()

set(COMMON_SRCS)

foreach(src ${SRCS})
  if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/common/${src})
    list(APPEND COMMON_SRCS common/${src})
  else()
    list(APPEND COMMON_SRCS ${ARCH_CHIP}/${src})
  endif()
endforeach()

target_sources(arch PRIVATE ${COMMON_SRCS})
