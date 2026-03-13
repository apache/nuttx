# ##############################################################################
# boards/arm/lpc17xx_40xx/pnev5180b/src/CMakeLists.txt
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

set(SRCS lpc17_40_boot.c lpc17_40_bringup.c)

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS lpc17_40_leds.c)
endif()

if(CONFIG_BOARDCTL)
  list(APPEND SRCS lpc17_40_appinit.c)
endif()

if(CONFIG_USBDEV_COMPOSITE)
  list(APPEND SRCS lpc17_40_composite.c)
endif()

if(CONFIG_LPC17_40_SPI)
  list(APPEND SRCS lpc17_40_spi.c)
endif()

if(CONFIG_ELF)
  list(APPEND SRCS lpc17_40_symtab.c)
endif()

if(CONFIG_NXFLAT)
  list(APPEND SRCS lpc17_40_symtab.c)
endif()

if(CONFIG_LPC17_40_ROMFS)
  list(APPEND SRCS lpc17_40_romfs.c)
endif()

target_sources(board PRIVATE ${SRCS})

set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/flash.ld")
