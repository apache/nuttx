# ##############################################################################
# boards/arm/s32k3xx/mr-canhubk3/src/CMakeLists.txt
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

set(SRCS s32k3xx_boot.c s32k3xx_bringup.c s32k3xx_clockconfig.c
         s32k3xx_periphclocks.c)

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS s32k3xx_buttons.c)
endif()

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS s32k3xx_autoleds.c)
else()
  list(APPEND SRCS s32k3xx_userleds.c)
endif()

if(CONFIG_BOARDCTL)
  list(APPEND SRCS s32k3xx_appinit.c)
endif()

if(CONFIG_FAT_DMAMEMORY)
  list(APPEND SRCS s32k3xx_dma_alloc.c)
endif()

if(CONFIG_S32K3XX_FLEXCAN)
  list(APPEND SRCS s32k3xx_tja1153.c)
endif()

if(CONFIG_S32K3XX_LPI2C)
  list(APPEND SRCS s32k3xx_i2c.c)
endif()

if(CONFIG_S32K3XX_LPSPI)
  list(APPEND SRCS s32k3xx_spi.c)
endif()

if(CONFIG_S32K3XX_SELFTEST)
  list(APPEND SRCS s32k3xx_selftest.c)
endif()

target_sources(board PRIVATE ${SRCS})

if(CONFIG_BOOT_RUNFROMFLASH)
  set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/flash.ld")
elseif(CONFIG_BOOT_RUNFROMISRAM)
  set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/sram.ld")
endif()
