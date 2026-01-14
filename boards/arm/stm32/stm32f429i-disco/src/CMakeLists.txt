# ##############################################################################
# boards/arm/stm32/stm32f429i-disco/src/CMakeLists.txt
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

if(CONFIG_BOARDCTL)
  list(APPEND SRCS stm32_appinit.c)
endif()

if(CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS stm32_idle.c)
endif()

if(CONFIG_STM32_FMC)
  list(APPEND SRCS stm32_extmem.c)
endif()

if(CONFIG_STM32_OTGHS)
  list(APPEND SRCS stm32_usb.c)
endif()

if(CONFIG_INPUT_STMPE811)
  list(APPEND SRCS stm32_stmpe811.c)
endif()

if(CONFIG_STM32F429I_DISCO_ILI9341)
  list(APPEND SRCS stm32_ili93414ws.c)
endif()

if(CONFIG_STM32F429I_DISCO_ILI9341_LCDIFACE
   AND CONFIG_STM32F429I_DISCO_ILI9341_FBIFACE
   AND CONFIG_STM32_LTDC)
  list(APPEND SRCS stm32_lcd.c)
endif()

if(CONFIG_PWM)
  list(APPEND SRCS stm32_pwm.c)
endif()

if(CONFIG_ADC)
  list(APPEND SRCS stm32_adc.c)
endif()

if(CONFIG_STM32_CAN_CHARDRIVER)
  list(APPEND SRCS stm32_can.c)
endif()

if(CONFIG_STM32F429I_DISCO_HIGHPRI)
  list(APPEND SRCS stm32_highpri.c)
endif()

target_sources(board PRIVATE ${SRCS})

set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/ld.script")
