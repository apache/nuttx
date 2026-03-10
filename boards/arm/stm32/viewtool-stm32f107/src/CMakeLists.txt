# ##############################################################################
# boards/arm/stm32/viewtool-stm32f107/src/CMakeLists.txt
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

set(SRCS stm32_boot.c stm32_bringup.c stm32_leds.c stm32_spi.c)

if(CONFIG_STM32_CAN_CHARDRIVER)
  list(APPEND SRCS stm32_can.c)
endif()

if(CONFIG_MMCSD)
  list(APPEND SRCS stm32_mmcsd.c)
endif()

if(CONFIG_STM32_OTGFS)
  list(APPEND SRCS stm32_usbdev.c)
else()
  if(CONFIG_STM32_USB)
    list(APPEND SRCS stm32_usbdev.c)
  endif()
endif()

if(CONFIG_INPUT_ADS7843E)
  list(APPEND SRCS stm32_ads7843e.c)
endif()

if(CONFIG_LCD_SSD1289)
  list(APPEND SRCS stm32_ssd1289.c)
endif()

if(CONFIG_BOARDCTL)
  list(APPEND SRCS stm32_appinit.c)
endif()

if(CONFIG_USBMSC)
  list(APPEND SRCS stm32_usbmsc.c)
endif()

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS stm32_buttons.c)
endif()

if(CONFIG_VIEWTOOL_HIGHPRI)
  list(APPEND SRCS stm32_highpri.c)
endif()

if(CONFIG_VIEWTOOL_FT80X_SPI1)
  list(APPEND SRCS stm32_ft80x.c)
elseif(CONFIG_VIEWTOOL_FT80X_SPI2)
  list(APPEND SRCS stm32_ft80x.c)
endif()

if(CONFIG_VIEWTOOL_MAX3421E_SPI1)
  list(APPEND SRCS stm32_max3421e.c)
elseif(CONFIG_VIEWTOOL_MAX3421E_SPI2)
  list(APPEND SRCS stm32_max3421e.c)
endif()

target_sources(board PRIVATE ${SRCS})

if(CONFIG_STM32_DFU)
  set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/dfu.ld")
else()
  set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/flash.ld")
endif()
