# ##############################################################################
# arch/arm/src/rp2040/CMakeLists.txt
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

set(SRCS
    rp2040_start.c
    rp2040_clock.c
    rp2040_gpio.c
    rp2040_irq.c
    rp2040_idle.c
    rp2040_pio.c
    rp2040_pll.c
    rp2040_xosc.c
    rp2040_flash_initialize.S
    rp2040_timerisr.c
    rp2040_testset.c)

if(CONFIG_RP2040_DMAC)
  list(APPEND SRCS rp2040_dmac.c)
endif()

if(CONFIG_RP2040_UART0 OR CONFIG_RP2040_UART1)
  list(APPEND SRCS rp2040_serial.c)
endif()

if(CONFIG_RP2040_SPI)
  list(APPEND SRCS rp2040_spi.c)
endif()

if(CONFIG_RP2040_I2C)
  list(APPEND SRCS rp2040_i2c.c)
endif()

if(CONFIG_RP2040_I2C_SLAVE)
  list(APPEND SRCS rp2040_i2c_slave.c)
endif()

if(CONFIG_RP2040_PWM)
  list(APPEND SRCS rp2040_pwm.c)
endif()

if(CONFIG_RP2040_I2S)
  list(APPEND SRCS rp2040_i2s.c rp2040_i2s_pio.c)
endif()

if(CONFIG_RP2040_ADC)
  list(APPEND SRCS rp2040_adc.c)
endif()

if(CONFIG_RP2040_BOARD_HAS_WS2812)
  list(APPEND SRCS rp2040_ws2812.c)
endif()

if(CONFIG_RP2040_FLASH_FILE_SYSTEM)
  list(APPEND SRCS rp2040_flash_mtd.c)
endif()

if(CONFIG_SMP)
  list(APPEND SRCS rp2040_smpcall.c rp2040_cpuidlestack.c rp2040_cpuindex.c
       rp2040_cpustart.c)
endif()

target_sources(arch PRIVATE ${SRCS})
