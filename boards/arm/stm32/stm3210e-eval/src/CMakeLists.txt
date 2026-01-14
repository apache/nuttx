# ##############################################################################
# boards/arm/stm32/stm3210e-eval/src/CMakeLists.txt
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

set(SRCS stm32_boot.c stm32_bringup.c stm32_leds.c stm32_spi.c stm32_usbdev.c)

if(CONFIG_STM32_FSMC)
  list(APPEND SRCS stm32_lcd.c stm32_extcontext.c stm32_extmem.c
       stm32_selectnor.c)
  list(APPEND SRCS stm32_deselectnor.c stm32_selectsram.c stm32_deselectsram.c)
  list(APPEND SRCS stm32_selectlcd.c stm32_deselectlcd.c)
endif()

if(CONFIG_BOARDCTL)
  list(APPEND SRCS stm32_appinit.c)
endif()

if(CONFIG_ADC)
  list(APPEND SRCS stm32_adc.c)
endif()

if(CONFIG_USBMSC)
  list(APPEND SRCS stm32_usbmsc.c)
endif()

if(CONFIG_USBDEV_COMPOSITE)
  list(APPEND SRCS stm32_composite.c)
endif()

if(CONFIG_STM32_CAN_CHARDRIVER)
  list(APPEND SRCS stm32_can.c)
endif()

if(CONFIG_ARCH_CUSTOM_PMINIT)
  list(APPEND SRCS stm32_pm.c)
endif()

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS stm32_buttons.c)

  if(CONFIG_PM_BUTTONS)
    list(APPEND SRCS stm32_pmbuttons.c)
  endif()
endif()

if(CONFIG_INPUT_DJOYSTICK)
  list(APPEND SRCS stm32_djoystick.c)
endif()

if(CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS stm32_idle.c)
endif()

target_sources(board PRIVATE ${SRCS})

set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/ld.script")
