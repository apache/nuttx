# ##############################################################################
# boards/arm/rzv2h/rzv2h-evk/src/CMakeLists.txt
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

# Core board initialization files
set(SRCS rzv2h_boot.c rzv2h_bringup.c)

# Application examples support
if(CONFIG_RZV2H_EXAMPLE_SUPPORT)
  list(APPEND SRCS rzv2h_appexamples.c)
endif()

# Board-specific initialization
if(CONFIG_BOARDCTL)
  list(APPEND SRCS rzv2h_appinit.c)
endif()

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS rzv2h_buttons.c)
endif()

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS rzv2h_autoleds.c)
else()
  list(APPEND SRCS rzv2h_userleds.c)
endif()

if(CONFIG_DEV_GPIO)
  list(APPEND SRCS rzv2h_gpio.c)
endif()

# Peripheral driver initialization

if(CONFIG_RZV2H_SPI_B)
  list(APPEND SRCS rzv2h_spi.c)
endif()

target_sources(board PRIVATE ${SRCS})

# Set linker script based on core configuration
if(CONFIG_RZV2H_CORTEXR8_0)
  set_property(GLOBAL PROPERTY LD_SCRIPT
                               "${NUTTX_BOARD_DIR}/scripts/rzv2h-evk_cr8_0.ld")
endif()
