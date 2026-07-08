# ##############################################################################
# arch/arm/src/stm32n6/CMakeLists.txt
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

# Required STM32N6 files

list(
  APPEND
  SRCS
  stm32_gpio.c
  stm32_irq.c
  stm32_lowputc.c
  stm32_rcc.c
  stm32_start.c
  stm32_pwr.c
  stm32_timerisr.c)

if(NOT CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS stm32_idle.c)
endif()

if(CONFIG_STM32_USART)
  list(APPEND SRCS stm32_serial.c)
endif()

# Chip-specific RCC

if(CONFIG_STM32_STM32N6XXXX)
  list(APPEND SRCS stm32n6xx_rcc.c)
endif()

target_sources(arch PRIVATE ${SRCS})
