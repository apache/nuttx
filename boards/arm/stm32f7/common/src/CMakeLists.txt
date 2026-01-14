# ##############################################################################
# boards/arm/stm32f7/common/src/CMakeLists.txt
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

set(SRCS)

if(CONFIG_SENSORS_BH1750FVI)
  list(APPEND SRCS stm32_bh1750.c)
endif()

if(CONFIG_SENSORS_BMI270_I2C)
  list(APPEND SRCS stm32_bmi270.c)
endif()

if(CONFIG_AUDIO_CS4344)
  list(APPEND SRCS stm32_cs4344.c)
endif()

if(CONFIG_STM32F7_CAN)
  if(CONFIG_STM32F7_CAN_CHARDRIVER)
    list(APPEND SRCS stm32_can_setup.c)
  endif()
  if(CONFIG_STM32F7_CAN_SOCKET)
    list(APPEND SRCS stm32_cansock_setup.c)
  endif()
endif()

if(CONFIG_STM32F7_ROMFS)
  list(APPEND SRCS stm32_romfs_initialize.c)
endif()

if(CONFIG_BOARDCTL_RESET)
  list(APPEND SRCS stm32_reset.c)
endif()

if(CONFIG_STM32F7_SPI_TEST)
  list(APPEND SRCS stm32_spitest.c)
endif()

target_sources(board PRIVATE ${SRCS})
