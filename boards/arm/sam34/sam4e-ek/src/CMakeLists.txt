# ##############################################################################
# boards/arm/sam34/sam4e-ek/src/CMakeLists.txt
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

set(SRCS sam_boot.c sam_leds.c sam_buttons.c sam_udp.c)

if(CONFIG_BOARDCTL)
  list(APPEND SRCS sam_appinit.c)
endif()

if(CONFIG_SAM34_HSMCI)
  list(APPEND SRCS sam_hsmci.c)
endif()

if(CONFIG_LCD)
  if(CONFIG_SAM4EEK_LCD_ILI9325)
    list(APPEND SRCS sam_ili9325.c)
  else()
    if(CONFIG_SAM4EEK_LCD_ILI9341)
      list(APPEND SRCS sam_ili9341.c)
    endif()
  endif()
endif()

if(CONFIG_SAM34_EMAC)
  list(APPEND SRCS sam_ethernet.c)
endif()

if(CONFIG_INPUT_ADS7843E)
  list(APPEND SRCS sam_ads7843e.c)
endif()

if(CONFIG_SAM34_SPI0)
  list(APPEND SRCS sam_spi.c)
  if(CONFIG_MTD_AT25)
    list(APPEND SRCS sam_at25.c)
  endif()
endif()

if(CONFIG_USBMSC)
  list(APPEND SRCS sam_usbmsc.c)
endif()

target_sources(board PRIVATE ${SRCS})

set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/flash.ld")
