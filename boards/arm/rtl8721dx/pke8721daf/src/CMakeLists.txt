# ##############################################################################
# boards/arm/rtl8721dx/pke8721daf/src/CMakeLists.txt
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

set(SRCS rtl8721dx_boot.c rtl8721dx_bringup.c)

if(CONFIG_AMEBA_GPIO)
  list(APPEND SRCS rtl8721dx_gpio.c)
endif()

if(CONFIG_AMEBA_UART)
  list(APPEND SRCS rtl8721dx_uart.c)
endif()

if(CONFIG_AMEBA_I2C)
  list(APPEND SRCS rtl8721dx_i2c.c)
endif()

if(CONFIG_AMEBA_SPI)
  list(APPEND SRCS rtl8721dx_spi.c)
endif()

if(CONFIG_AMEBA_PWM)
  list(APPEND SRCS rtl8721dx_pwm.c)
endif()

if(CONFIG_AMEBA_ADC)
  list(APPEND SRCS rtl8721dx_adc.c)
endif()

target_sources(board PRIVATE ${SRCS})

if(CONFIG_AMEBA_GPIO
   OR CONFIG_AMEBA_UART
   OR CONFIG_AMEBA_I2C
   OR CONFIG_AMEBA_SPI
   OR CONFIG_AMEBA_PWM
   OR CONFIG_AMEBA_ADC)
  # The board pin/UART tables pull in the shared driver's public headers from
  # arch/arm/src/common/ameba/, not on the default board include path.
  target_include_directories(board
                             PRIVATE ${NUTTX_DIR}/arch/arm/src/common/ameba)
endif()

# LD_SCRIPT is not set here: the Ameba image2 linker script is generated
# (prebuilt/ld.script.gen) and published by the shared ameba_board.cmake.
