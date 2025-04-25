# ##############################################################################
# boards/arm/tiva/tm4c123g-launchpad/src/CMakeLists.txt
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

set(SRCS tm4c_boot.c tm4c_bringup.c tm4c_ssi.c)

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS tm4c_autoleds.c)
endif()

if(CONFIG_BOARDCTL)
  list(APPEND SRCS tm4c_appinit.c)
endif()

if(CONFIG_TIVA_TIMER)
  list(APPEND SRCS tm4c_timer.c)
endif()

if(CONFIG_TIVA_ADC)
  list(APPEND SRCS tm4c_adc.c)
endif()

if(CONFIG_TIVA_CAN)
  list(APPEND SRCS tm4c_can.c)
endif()

if(CONFIG_CAN_MCP2515)
  list(APPEND SRCS tm4c_mcp2515.c)
endif()

if(CONFIG_MTD_AT24XX)
  if(CONFIG_TIVA_I2C0)
    list(APPEND SRCS tm4c_at24.c)
  endif()
endif()

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS tm4c_buttons.c)
endif()

target_sources(board PRIVATE ${SRCS})

set_property(GLOBAL PROPERTY LD_SCRIPT
                             "${NUTTX_BOARD_DIR}/scripts/tm4c123g-launchpad.ld")
