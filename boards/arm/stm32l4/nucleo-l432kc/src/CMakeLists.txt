# ##############################################################################
# boards/arm/stm32l4/nucleo-l432kc/src/CMakeLists.txt
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

set(SRCS stm32_boot.c stm32_bringup.c stm32_spi.c)

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS stm32_autoleds.c)
else()
  list(APPEND SRCS stm32_userleds.c)
endif()

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS stm32_buttons.c)
endif()

if(CONFIG_DEV_GPIO)
  list(APPEND SRCS stm32_gpio.c)
endif()

if(CONFIG_STM32L4_ADC)
  list(APPEND SRCS stm32_adc.c)
endif()

if(CONFIG_STM32L4_DAC)
  list(APPEND SRCS stm32_dac.c)
endif()

if(CONFIG_DAC7571)
  list(APPEND SRCS stm32_dac7571.c)
endif()

if(CONFIG_MTD_AT45DB)
  list(APPEND SRCS stm32_at45db.c)
endif()

if(CONFIG_SENSORS_INA226)
  list(APPEND SRCS stm32_ina226.c)
endif()

if(CONFIG_SENSORS_INA219)
  list(APPEND SRCS stm32_ina219.c)
endif()

if(CONFIG_SENSORS_QENCODER)
  list(APPEND SRCS stm32_qencoder.c)
endif()

if(CONFIG_PWM)
  list(APPEND SRCS stm32_pwm.c)
endif()

if(CONFIG_SENSORS_ZEROCROSS)
  list(APPEND SRCS stm32_zerocross.c)
endif()

if(CONFIG_TIMER)
  list(APPEND SRCS stm32_timer.c)
endif()

if(CONFIG_BOARDCTL)
  list(APPEND SRCS stm32_appinit.c)
  if(CONFIG_BOARDCTL_IOCTL)
    list(APPEND SRCS stm32_ioctl.c)
  endif()
endif()

if(CONFIG_BOARDCTL_UNIQUEID)
  list(APPEND SRCS stm32_uid.c)
endif()

if(CONFIG_NUCLEOL432KC_SPWM)
  list(APPEND SRCS stm32_spwm.c)
endif()

if(CONFIG_NUCLEOL432KC_DAC_WGEN)
  list(APPEND SRCS stm32_dac_wgen.c)
endif()

if(NOT CONFIG_STM32_ETHMAC)
  if(CONFIG_NETDEVICES)
    list(APPEND SRCS stm32_netinit.c)
  endif()
endif()

target_sources(board PRIVATE ${SRCS})

set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/l432kc.ld")
