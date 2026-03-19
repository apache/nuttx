# ##############################################################################
# arch/arm/src/gd32f4/CMakeLists.txt
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
  gd32f4xx_allocateheap.c
  gd32f4xx_start.c
  gd32f4xx_rcu.c
  gd32f4xx_serial.c
  gd32f4xx_lowputc.c
  gd32f4xx_gpio.c
  gd32f4xx_syscfg.c
  gd32f4xx_irq.c
  gd32f4xx_exti.c
  gd32f4xx_dma.c)

if(NOT CONFIG_SCHED_TICKLESS)
  list(APPEND SRCS gd32f4xx_timerisr.c)
else()
  list(APPEND SRCS gd32f4xx_tickless.c)
endif()

if(NOT CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS gd32f4xx_idle.c)
endif()

list(APPEND SRCS gd32f4xx_fmc.c gd32f4xx_progmem.c)

if(CONFIG_GD32F4_ENETMAC)
  list(APPEND SRCS gd32f4xx_enet.c)
endif()

if(CONFIG_GD32F4_PMU)
  list(APPEND SRCS gd32f4xx_pmu.c)
endif()

if(CONFIG_GD32F4_SPI)
  list(APPEND SRCS gd32f4xx_spi.c)
endif()

if(CONFIG_GD32F4_I2C)
  list(APPEND SRCS gd32f4xx_i2c.c)
endif()

if(CONFIG_GD32F4_SDIO)
  list(APPEND SRCS gd32f4xx_sdio.c)
endif()

target_sources(arch PRIVATE ${SRCS})
