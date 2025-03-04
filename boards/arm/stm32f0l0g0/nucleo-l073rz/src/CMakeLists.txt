# ##############################################################################
# boards/arm/stm32f0l0g0/nucleo-l073rz/src/CMakeLists.txt
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

set(SRCS stm32_boot.c stm32_bringup.c)

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS stm32_autoleds.c)
else()
  list(APPEND SRCS stm32_userleds.c)
endif()

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS stm32_buttons.c)
endif()

if(CONFIG_STM32F0L0G0_SPI)
  list(APPEND SRCS stm32_spi.c)
endif()

if(CONFIG_BOARDCTL)
  list(APPEND SRCS stm32_appinit.c)
endif()

if(CONFIG_WL_NRF24L01)
  list(APPEND SRCS stm32_nrf24l01.c)
endif()

if(CONFIG_LPWAN_SX127X)
  list(APPEND SRCS stm32_sx127x.c)
endif()

if(CONFIG_CL_MFRC522)
  list(APPEND SRCS stm32_mfrc522.c)
endif()

target_sources(board PRIVATE ${SRCS})

set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/ld.script")
