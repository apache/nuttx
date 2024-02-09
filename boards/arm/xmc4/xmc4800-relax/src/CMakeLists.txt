# ##############################################################################
# boards/arm/xmc4/xmc4800/src/CMakeLists.txt
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

set(SRCS xmc4_boot.c xmc4_bringup.c)

if(CONFIG_INPUT_BUTTONS)
  list(APPEND SRCS xmc4_buttons.c)
endif()

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS xmc4_autoleds.c)
else()
  list(APPEND SRCS xmc4_userleds.c)
endif()

if(CONFIG_XMC4_USCI_SPI)
  list(APPEND SRCS xmc4_spi.c)
endif()

if(CONFIG_BOARDCTL)
  list(APPEND SRCS xmc4_appinit.c)
endif()

target_sources(board PRIVATE ${SRCS})

set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/flash.ld")
