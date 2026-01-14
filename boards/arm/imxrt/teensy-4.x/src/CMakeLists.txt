# ##############################################################################
# boards/arm/imxrt/teensy-4.x/src/CMakeLists.txt
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

set(SRCS imxrt_boot.c imxrt_flexspi_nor_boot.c imxrt_flexspi_nor_flash.c)

if(CONFIG_BOARDCTL)
  list(APPEND SRCS imxrt_appinit.c imxrt_bringup.c)
elseif(CONFIG_BOARD_LATE_INITIALIZE)
  list(APPEND SRCS imxrt_bringup.c)
endif()

if(CONFIG_BOARDCTL_RESET)
  list(APPEND SRCS imxrt_reset.c)
endif()

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS imxrt_autoleds.c)
else()
  list(APPEND SRCS imxrt_userleds.c)
endif()

if(CONFIG_IMXRT_LPSPI)
  list(APPEND SRCS imxrt_spi.c)
endif()

if(CONFIG_IMXRT_LPI2C)
  list(APPEND SRCS imxrt_i2c.c)
endif()

if(CONFIG_IMXRT_FLEXCAN)
  list(APPEND SRCS imxrt_flexcan.c)
endif()

if(CONFIG_IMXRT_FLEXPWM)
  list(APPEND SRCS imxrt_flexpwm.c)
endif()

if(CONFIG_IMXRT_ENET)
  list(APPEND SRCS imxrt_ethernet.c)
endif()

if(CONFIG_DEV_GPIO)
  list(APPEND SRCS imxrt_gpio.c)
endif()

if(CONFIG_IMXRT_ADC)
  list(APPEND SRCS imxrt_adc.c)
endif()

if(CONFIG_LCD_ST7789)
  list(APPEND SRCS imxrt_st7789.c)
endif()

if(CONFIG_IMXRT_ENC)
  list(APPEND SRCS imxrt_enc.c)
endif()

target_sources(board PRIVATE ${SRCS})

if(CONFIG_BOOT_RUNFROMFLASH)
  set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/flash.ld")
elseif(CONFIG_BOOT_RUNFROMISRAM)
  set_property(GLOBAL PROPERTY LD_SCRIPT
                               "${NUTTX_BOARD_DIR}/scripts/flash-ocram.ld")
endif()
