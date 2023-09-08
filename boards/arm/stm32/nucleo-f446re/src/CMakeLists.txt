# ##############################################################################
# boards/arm/stm32/nucleo-f446re/src/CMakeLists.txt
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

if(CONFIG_LCD_ILI9225)
  list(APPEND SRCS stm32_ili9225.c)
endif()

if(NOT CONFIG_STM32_FOC)
  if(CONFIG_ADC)
    list(APPEND SRCS stm32_adc.c)
    if(CONFIG_INPUT_AJOYSTICK)
      list(APPEND SRCS stm32_ajoystick.c)
    endif()
  endif()
endif()

if(CONFIG_STM32_CAN)
  if(CONFIG_STM32_CAN_CHARDRIVER)
    list(APPEND SRCS stm32_can.c)
  endif()
  if(CONFIG_STM32_CAN_SOCKET)
    list(APPEND SRCS stm32_cansock.c)
  endif()
endif()

if(CONFIG_STM32_PWM)
  list(APPEND SRCS stm32_pwm.c)
endif()

if(CONFIG_DEV_GPIO)
  list(APPEND SRCS stm32_gpio.c)
endif()

if(CONFIG_DAC)
  list(APPEND SRCS stm32_dac.c)
endif()

if(CONFIG_BOARDCTL)
  list(APPEND SRCS stm32_appinit.c)
endif()

if(CONFIG_BOARD_STM32_IHM08M1)
  list(APPEND SRCS stm32_foc_ihm08m1.c)
endif()

if(CONFIG_STM32_ROMFS)
  list(APPEND SRCS stm32_romfs_initialize.c)
endif()

target_sources(board PRIVATE ${SRCS})

set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/f446re.ld")
