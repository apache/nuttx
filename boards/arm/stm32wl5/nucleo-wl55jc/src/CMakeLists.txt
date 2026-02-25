# ##############################################################################
# boards/arm/stm32wl5/nucleo-wl55jc/src/CMakeLists.txt
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

set(SRCS stm32_boot.c stm32_leds.c)

if(CONFIG_BOARDCTL)
  list(APPEND SRCS stm32_appinit.c)
endif()

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS stm32_buttons.c)
endif()

if(CONFIG_ARCH_BOARD_FLASH_MOUNT)
  list(APPEND SRCS stm32_flash.c)
endif()

if(CONFIG_SPI_DRIVER)
  list(APPEND SRCS stm32_spi.c)
endif()

if(CONFIG_ARCH_BOARD_IPCC)
  list(APPEND SRCS stm32_ipcc.c)
endif()

if(CONFIG_VIDEO_FB)
  if(CONFIG_LCD_SSD1680)
    list(APPEND SRCS stm32_ssd1680.c)
  endif()
endif()

target_sources(board PRIVATE ${SRCS})

set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/wl55jc.ld")
