# ##############################################################################
# boards/arm/stm32/nucleo-f401re/src/CMakeLists.txt
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

set(SRCS stm32_boot.c stm32_spi.c stm32_bringup.c)

if(CONFIG_VIDEO_FB)
  if(CONFIG_LCD_SSD1306)
    list(APPEND SRCS stm32_lcd_ssd1306.c)
  endif()
endif()

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS stm32_autoleds.c)
else()
  list(APPEND SRCS stm32_userleds.c)
endif()

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS stm32_buttons.c)
endif()

if(CONFIG_ADC)
  list(APPEND SRCS stm32_adc.c)
  if(CONFIG_INPUT_AJOYSTICK)
    list(APPEND SRCS stm32_ajoystick.c)
  endif()
endif()

if(CONFIG_CAN_MCP2515)
  list(APPEND SRCS stm32_mcp2515.c)
endif()

if(CONFIG_BOARDCTL)
  list(APPEND SRCS stm32_appinit.c)
endif()

target_sources(board PRIVATE ${SRCS})

set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/flash.ld")
