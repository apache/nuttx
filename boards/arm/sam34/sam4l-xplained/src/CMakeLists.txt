# ##############################################################################
# boards/arm/sam34/sam4l-xplained/src/CMakeLists.txt
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

set(SRCS sam_boot.c)

if(CONFIG_SAM34_SPI0)
  list(APPEND SRCS sam_spi.c)
endif()

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS sam_autoleds.c)
else()
  list(APPEND SRCS sam_userleds.c)
endif()

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS sam_buttons.c)
endif()

if(CONFIG_BOARDCTL)
  list(APPEND SRCS sam_appinit.c)
endif()

if(CONFIG_SAM34_LCDCA)
  if(CONFIG_SAM4L_XPLAINED_SLCD1MODULE)
    list(APPEND SRCS sam_slcd.c)
  endif()
endif()

if(CONFIG_SAM34_SPI0)
  if(CONFIG_SAM4L_XPLAINED_IOMODULE)
    list(APPEND SRCS sam_mmcsd.c)
  endif()
endif()

if(CONFIG_SAM34_SPI0)
  if(CONFIG_SAM4L_XPLAINED_OLED1MODULE)
    if(CONFIG_LCD_UG2832HSWEG04)
      list(APPEND SRCS sam_ug2832hsweg04.c)
    endif()
  endif()
endif()

target_sources(board PRIVATE ${SRCS})

set_property(GLOBAL PROPERTY LD_SCRIPT
                             "${NUTTX_BOARD_DIR}/scripts/sam4l-xplained.ld")
