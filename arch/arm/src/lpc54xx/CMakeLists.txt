# ##############################################################################
# arch/arm/src/lpc54xx/CMakeLists.txt
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
    lpc54_start.c
    lpc54_clockconfig.c
    lpc54_irq.c
    lpc54_clrpend.c
    lpc54_allocateheap.c
    lpc54_lowputc.c
    lpc54_gpio.c
    lpc54_reset.c)

if(NOT CONFIG_SCHED_TICKLESS)
  list(APPEND SRCS lpc54_timerisr.c)
else()
  list(APPEND SRCS lpc54_tickless.c)
endif()

if(CONFIG_BUILD_PROTECTED)
  list(APPEND SRCS lpc54_userspace.c lpc54_mpuinit.c)
endif()

if(CONFIG_LPC54_DMA)
  list(APPEND SRCS lpc54_dma.c)
endif()

if(NOT CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS lpc54_idle.c)
endif()

if(CONFIG_LPC54_GPIOIRQ)
  list(APPEND SRCS lpc54_gpioirq.c)
endif()

if(CONFIG_RTC)
  list(APPEND SRCS lpc54_rtc.c)
  if(CONFIG_RTC_DRIVER)
    list(APPEND SRCS lpc54_rtc_lowerhalf.c)
  endif()
endif()

if(CONFIG_LPC54_WWDT)
  list(APPEND SRCS lpc54_wwdt.c)
endif()

if(CONFIG_LPC54_RNG)
  list(APPEND SRCS lpc54_rng.c)
endif()

if(CONFIG_LPC54_HAVE_USART)
  list(APPEND SRCS lpc54_serial.c)
endif()

if(CONFIG_LPC54_HAVE_I2C_MASTER)
  list(APPEND SRCS lpc54_i2c_master.c)
endif()

if(CONFIG_LPC54_HAVE_SPI_MASTER)
  list(APPEND SRCS lpc54_spi_master.c)
endif()

if(CONFIG_LPC54_EMC)
  list(APPEND SRCS lpc54_emc.c)
endif()

if(CONFIG_LPC54_ETHERNET)
  list(APPEND SRCS lpc54_ethernet.c)
endif()

if(CONFIG_LPC54_LCD)
  list(APPEND SRCS lpc54_lcd.c)
endif()

if(CONFIG_LPC54_SDMMC)
  list(APPEND SRCS lpc54_sdmmc.c)
endif()

target_sources(arch PRIVATE ${SRCS})
