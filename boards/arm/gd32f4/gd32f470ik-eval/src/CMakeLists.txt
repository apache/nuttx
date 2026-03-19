# ##############################################################################
# boards/arm/gd32f4/gd32f470ik-eval/src/CMakeLists.txt
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

set(SRCS gd32f4xx_boot.c)

if(CONFIG_BOARDCTL)
  list(APPEND SRCS gd32f4xx_appinit.c gd32f4xx_bringup.c)
elseif(CONFIG_BOARD_LATE_INITIALIZE)
  list(APPEND SRCS gd32f4xx_bringup.c)
endif()

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS gd32f4xx_autoleds.c)
else()
  list(APPEND SRCS gd32f4xx_userleds.c)
endif()

if(CONFIG_DEV_GPIO)
  list(APPEND SRCS gd32f4xx_gpio.c)
endif()

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS gd32f4xx_buttons.c)
endif()

if(CONFIG_SPI)
  list(APPEND SRCS gd32f4xx_spi.c)
endif()

if(CONFIG_MMCSD)
  list(APPEND SRCS gd32f4xx_sdio.c)
endif()

if(CONFIG_MTD_GD25)
  list(APPEND SRCS gd32f4xx_gd25.c)
endif()

if(CONFIG_MTD_AT24XX)
  list(APPEND SRCS gd32f4xx_at24.c)
endif()

target_sources(board PRIVATE ${SRCS})

set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/ld.script")
