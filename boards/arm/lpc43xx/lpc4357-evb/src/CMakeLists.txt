# ##############################################################################
# boards/arm/lpc43xx/lpc4357-evb/src/CMakeLists.txt
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

set(SRCS lpc43_boot.c)

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS lpc43_autoleds.c)
else()
  list(APPEND SRCS lpc43_userleds.c)
endif()

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS lpc43_buttons.c)
endif()

if(CONFIG_USBMSC)
  list(APPEND SRCS lpc43_usbmsc.c)
endif()

target_sources(board PRIVATE ${SRCS})

if(CONFIG_LPC43_BOOT_SRAM)
  set(LDFILE ramconfig.ld)
endif()
if(CONFIG_LPC43_BOOT_SPIFI)
  set(LDFILE spificonfig.ld)
endif()
if(CONFIG_LPC43_BOOT_FLASHA)
  set(LDFILE flashaconfig.ld)
endif()
if(CONFIG_LPC43_BOOT_FLASHB)
  set(LDFILE flashaconfig.ld)
endif()
if(CONFIG_LPC43_BOOT_CS0FLASH)
  set(LDFILE cs0flash.ld)
endif()

set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/${LDFILE}")
