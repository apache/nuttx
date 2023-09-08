# ##############################################################################
# boards/arm/stm32/hymini-stm32v/src/CMakeLists.txt
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

set(SRCS stm32_boot.c stm32_leds.c stm32_buttons.c stm32_spi.c stm32_usbdev.c)

if(CONFIG_LCD_SSD1289)
  list(APPEND SRCS stm32_ssd1289.c)
else()
  if(CONFIG_LCD_R61505U)
    list(APPEND SRCS stm32_r61505u.c)
  endif()
endif()

if(CONFIG_BOARDCTL)
  list(APPEND SRCS stm32_appinit.c)
endif()

if(CONFIG_INPUT)
  list(APPEND SRCS stm32_ts.c)
endif()

if(CONFIG_USBMSC)
  list(APPEND SRCS stm32_usbmsc.c)
endif()

target_sources(board PRIVATE ${SRCS})

set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/ld.script")
