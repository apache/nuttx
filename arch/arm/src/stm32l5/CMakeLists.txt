# ##############################################################################
# arch/arm/src/stm32l5/CMakeLists.txt
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

# Required STM32L5 files

list(
  APPEND
  SRCS
  stm32l5_allocateheap.c
  stm32l5_exti_gpio.c
  stm32l5_gpio.c
  stm32l5_irq.c
  stm32l5_lowputc.c
  stm32l5_rcc.c
  stm32l5_serial.c
  stm32l5_start.c
  stm32l5_waste.c
  stm32l5_uid.c
  stm32l5_spi.c
  stm32l5_lse.c
  stm32l5_lsi.c
  stm32l5_pwr.c
  stm32l5_tim.c
  stm32l5_flash.c
  stm32l5_timerisr.c)

if(NOT CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS stm32l5_idle.c)
endif()

if(CONFIG_TIMER)
  list(APPEND SRCS stm32l5_tim_lowerhalf.c)
endif()

if(CONFIG_BUILD_PROTECTED)
  list(APPEND SRCS stm32l5_userspace.c stm32l5_mpuinit.c)
endif()

if(CONFIG_DEBUG_FEATURES)
  list(APPEND SRCS stm32l5_dumpgpio.c)
endif()

# Required chip type specific files

if(CONFIG_STM32L5_STM32L562XX)
  list(APPEND SRCS stm32l562xx_rcc.c)
endif()

target_sources(arch PRIVATE ${SRCS})
