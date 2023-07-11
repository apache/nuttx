# ##############################################################################
# boards/arm/stm32h7/stm32h747i-disco/src/CMakeLists.txt
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

set(SRCS stm32_boot.c stm32_bringup.c)

if(CONFIG_ADC)
  list(APPEND SRCS stm32_adc.c)
endif()

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS stm32_autoleds.c)
else()
  list(APPEND SRCS stm32_userleds.c)
endif()

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS stm32_buttons.c)
endif()

if(CONFIG_STM32H7_SPI)
  list(APPEND SRCS stm32_spi.c)
endif()

if(CONFIG_STM32H7_OTGHS)
  list(APPEND SRCS stm32_usb.c)
endif()

if(CONFIG_BOARDCTL_UNIQUEID)
  list(APPEND SRCS stm32_uid.c)
endif()

if(CONFIG_BOARDCTL)
  list(APPEND SRCS stm32_appinitialize.c)
endif()

if(CONFIG_STM32H7_SDMMC)
  list(APPEND SRCS stm32_sdmmc.c)
endif()

if(CONFIG_FAT_DMAMEMORY)
  list(APPEND SRCS stm32_dma_alloc.c)
endif()

target_sources(board PRIVATE ${SRCS})

set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/flash.ld")

if(NOT CONFIG_BUILD_FLAT)
  add_subdirectory(${NUTTX_BOARD_DIR}/kernel)
  set_property(
    GLOBAL PROPERTY LD_SCRIPT_USER ${NUTTX_BOARD_DIR}/scripts/memory.ld
    ${NUTTX_BOARD_DIR}/scripts/user-space.ld)
endif()
