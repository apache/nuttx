# ##############################################################################
# boards/arm/imxrt/imxrt1170-evk/src/CMakeLists.txt
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

set(SRCS imxrt_boot.c imxrt_flexspi_nor_boot.c imxrt_flexspi_nor_flash.c
         imxrt_clockconfig.c)

if(CONFIG_IMXRT_SDRAMC)
  list(APPEND SRCS imxrt_sdram.c)
endif()

if(CONFIG_BOARDCTL)
  list(APPEND SRCS imxrt_appinit.c imxrt_bringup.c)
elseif(CONFIG_BOARD_LATE_INITIALIZE)
  list(APPEND SRCS imxrt_bringup.c)
endif()

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS imxrt_autoleds.c)
else()
  list(APPEND SRCS imxrt_userleds.c)
endif()

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS imxrt_buttons.c)
endif()

if(CONFIG_IMXRT_ENET)
  list(APPEND SRCS imxrt_ethernet.c)
endif()

if(CONFIG_IMXRT_LPSPI)
  list(APPEND SRCS imxrt_spi.c)
endif()

if(CONFIG_MMCSD_SPI)
  list(APPEND SRCS imxrt_mmcsd_spi.c)
endif()

if(CONFIG_DEV_GPIO)
  list(APPEND SRCS imxrt_gpio.c)
endif()

if(CONFIG_IMXRT_ADC)
  list(APPEND SRCS imxrt_adc.c)
endif()

if(CONFIG_INPUT_FT5X06)
  list(APPEND SRCS imxrt_ft5x06.c)
endif()

if(CONFIG_IMXRT_LCD)
  list(APPEND SRCS imxrt_lcd.c)
endif()

if(CONFIG_IMXRT_USBOTG)
  list(APPEND SRCS imxrt_usbhost.c)
endif()

if(CONFIG_IMXRT_FLEXCAN)
  list(APPEND SRCS imxrt_flexcan.c)
endif()

if(CONFIG_IMXRT1170_EVK_SDRAM)
  list(APPEND SRCS imxrt_sdram_ini_dcd.c)
endif()

if(CONFIG_IMXRT1170_FLEXSPI_NOR)
  list(APPEND SRCS imxrt_flexspi_nor.c)
endif()

if(CONFIG_IMXRT1170_FLEXSPI_FRAM)
  list(APPEND SRCS imxrt_flexspi_fram.c)
endif()

target_sources(board PRIVATE ${SRCS})

if(CONFIG_BOOT_RUNFROMFLASH)
  set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/flash.ld")
elseif(CONFIG_BOOT_RUNFROMISRAM)
  set_property(GLOBAL PROPERTY LD_SCRIPT
                               "${NUTTX_BOARD_DIR}/scripts/flash-ocram.ld")
endif()
