# ##############################################################################
# boards/arm/sam34/arduino-due/src/CMakeLists.txt
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

set(SRCS sam_boot.c sam_bringup.c)

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS sam_autoleds.c)
else()
  list(APPEND SRCS sam_userleds.c)
endif()

if(CONFIG_ARDUINO_ITHEAD_TFT)

  if(CONFIG_SPI_BITBANG)
    if(CONFIG_MMCSD_SPI)
      list(APPEND SRCS sam_mmcsd.c)
    endif()

    if(CONFIG_INPUT_ADS7843E)
      list(APPEND SRCS sam_touchscreen.c)
    endif()
  endif()

  if(CONFIG_LCD)
    list(APPEND SRCS sam_lcd.c)
  endif()
endif()

if(CONFIG_BOARDCTL)
  list(APPEND SRCS sam_appinit.c)
endif()

if(CONFIG_SAM34_SPI0)
  list(APPEND SRCS sam_spidev.c)
endif()

target_sources(board PRIVATE ${SRCS})

set_property(GLOBAL PROPERTY LD_SCRIPT
                             "${NUTTX_BOARD_DIR}/scripts/arduino-due.ld")
