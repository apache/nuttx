# ##############################################################################
# boards/risc-v/esp32p4/esp32p4-tab5/src/CMakeLists.txt
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

set(SRCS esp32p4_boot.c esp32p4_bringup.c)

if(CONFIG_ESP32P4_TAB5_IOEXPANDER)
  list(APPEND SRCS esp32p4_ioexpander.c)
endif()

if(CONFIG_ESP32P4_TAB5_HMI_POWER)
  list(APPEND SRCS esp32p4_hmi_power.c)
endif()

if(CONFIG_ESP32P4_TAB5_LCD)
  list(APPEND SRCS esp32p4_display.c esp32p4_mipi_dpi.c)
  if(CONFIG_ESP32P4_TAB5_LCD_ST7121)
    list(APPEND SRCS esp32p4_lcd_st7121.c)
  elseif(CONFIG_ESP32P4_TAB5_LCD_ST7123)
    list(APPEND SRCS esp32p4_lcd_st7123.c)
  elseif(CONFIG_ESP32P4_TAB5_LCD_ILI9881C)
    list(APPEND SRCS esp32p4_lcd_ili9881c.c)
  endif()
endif()

if(CONFIG_ESP32P4_TAB5_TOUCHSCREEN)
  if(CONFIG_ESP32P4_TAB5_TOUCH_ST7123)
    list(APPEND SRCS esp32p4_touch_st7123.c)
  elseif(CONFIG_ESP32P4_TAB5_TOUCH_GT911)
    list(APPEND SRCS esp32p4_touch_gt911.c)
  endif()
endif()

if(CONFIG_BOARDCTL)
  if(CONFIG_BOARDCTL_RESET)
    list(APPEND SRCS esp32p4_reset.c)
  endif()
endif()

target_sources(board PRIVATE ${SRCS})

# ##############################################################################
# Linker scripts (P4: sections or sections.rev3 when REV_MIN_300)
# ##############################################################################

string(REPLACE "\"" "" CHIP_SERIES "${CONFIG_ESPRESSIF_CHIP_SERIES}")

set(BOARD_COMMON_DIR "${NUTTX_BOARD_DIR}/../common")

set(LDSCRIPTS "${BOARD_COMMON_DIR}/scripts/${CHIP_SERIES}_aliases.ld"
              "${BOARD_COMMON_DIR}/scripts/${CHIP_SERIES}_flat_memory.ld")

if(CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT OR CONFIG_ESPRESSIF_SIMPLE_BOOT)
  if(NOT "${CONFIG_ESP32P4_SELECTS_REV_LESS_V3}" STREQUAL "y")
    list(APPEND LDSCRIPTS
         "${BOARD_COMMON_DIR}/scripts/${CHIP_SERIES}_sections.rev3.ld")
  else()
    list(APPEND LDSCRIPTS
         "${BOARD_COMMON_DIR}/scripts/${CHIP_SERIES}_sections.ld")
  endif()
endif()

set_property(GLOBAL APPEND PROPERTY LD_SCRIPT ${LDSCRIPTS})
