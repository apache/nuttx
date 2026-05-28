# ##############################################################################
# arch/arm/src/stm32/CMakeLists.txt
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

list(
  APPEND
  SRCS
  stm32_allocateheap.c
  stm32_start.c
  stm32_rcc.c
  stm32_lse.c
  stm32_lsi.c
  stm32_irq.c
  stm32_lowputc.c)

if(CONFIG_STM32_TICKLESS_TIMER)
  list(APPEND SRCS stm32_tickless.c)
else()
  list(APPEND SRCS stm32_timerisr.c)
endif()

if(CONFIG_BUILD_PROTECTED)
  list(APPEND SRCS stm32_userspace.c stm32_mpuinit.c)
endif()

if(NOT CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS stm32_idle.c)
endif()

list(APPEND SRCS stm32_pmstop.c stm32_pmstandby.c stm32_pmsleep.c)

if(NOT CONFIG_ARCH_CUSTOM_PMINIT)
  list(APPEND SRCS stm32_pminitialize.c)
endif()

target_sources(arch PRIVATE ${SRCS})
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../common/stm32 stm32_common)
