# ##############################################################################
# arch/arm/src/rp23xx/CMakeLists.txt
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

list(
  APPEND
  SRCS
  rp23xx_idle.c
  rp23xx_irq.c
  rp23xx_uart.c
  rp23xx_serial.c
  rp23xx_start.c
  rp23xx_timerisr.c
  rp23xx_gpio.c
  rp23xx_pio.c
  rp23xx_clock.c
  rp23xx_xosc.c
  rp23xx_pll.c)

if(CONFIG_SMP)
  list(APPEND SRCS rp23xx_cpustart.c rp23xx_smpcall.c rp23xx_cpuidlestack.c
       rp23xx_testset.c)
endif()

if(CONFIG_ARCH_HAVE_MULTICPU)
  list(APPEND SRCS rp23xx_cpuindex.c)
endif()

if(CONFIG_RP23XX_DMAC)
  list(APPEND SRCS rp23xx_dmac.c)
endif()

if(CONFIG_RP23XX_SPI)
  list(APPEND SRCS rp23xx_spi.c)
endif()

if(CONFIG_RP23XX_PWM)
  list(APPEND SRCS rp23xx_pwm.c)
endif()

if(CONFIG_RP23XX_I2C)
  list(APPEND SRCS rp23xx_i2c.c)
endif()

if(CONFIG_RP23XX_I2C_SLAVE)
  list(APPEND SRCS rp23xx_i2c_slave.c)
endif()

if(CONFIG_RP23XX_I2S)
  list(APPEND SRCS rp23xx_i2s.c)
  list(APPEND SRCS rp23xx_i2s_pio.c)
endif()

if(CONFIG_USBDEV)
  list(APPEND SRCS rp23xx_usbdev.c)
endif()

if(CONFIG_WS2812)
  list(APPEND SRCS rp23xx_ws2812.c)
endif()

if(CONFIG_ADC)
  list(APPEND SRCS rp23xx_adc.c)
endif()

if(CONFIG_WATCHDOG)
  list(APPEND SRCS rp23xx_wdt.c)
endif()

target_sources(arch PRIVATE ${SRCS})
