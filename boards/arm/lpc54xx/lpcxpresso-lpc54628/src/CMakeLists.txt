# ##############################################################################
# boards/arm/lpc54xx/lpcxpresso-lpc54628/src/CMakeLists.txt
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

set(SRCS lpc54_boot.c lpc54_bringup.c lpc54_userleds.c lpc54_lcd.c)

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS lpc54_autoleds.c)
endif()

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS lpc54_buttons.c)
endif()

if(CONFIG_BOARDCTL)
  list(APPEND SRCS lpc54_appinit.c)
endif()

if(CONFIG_LPC54_HAVE_I2C_MASTER)
  list(APPEND SRCS lpc54_i2c.c)
endif()

if(CONFIG_SYSTEM_I2CTOOL)
  list(APPEND SRCS lpc54_i2ctool.c)
endif()

if(CONFIG_INPUT_FT5X06)
  list(APPEND SRCS lpc54_ft5x06.c)
endif()

if(CONFIG_LPC54_EMC)
  if(CONFIG_LPC54_EMC_DYNAMIC)
    list(APPEND SRCS lpc54_sdram.c)
  endif()
endif()

target_sources(board PRIVATE ${SRCS})

set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/flash.ld")
