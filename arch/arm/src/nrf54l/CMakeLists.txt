# ##############################################################################
# arch/arm/src/nrf54l/CMakeLists.txt
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
    nrf54l_start.c
    nrf54l_clockconfig.c
    nrf54l_oscconfig.c
    nrf54l_irq.c
    nrf54l_allocateheap.c
    nrf54l_gpio.c
    nrf54l_lowputc.c
    nrf54l_uid.c)

if(CONFIG_NRF54L_SYSTIMER_SYSTICK)
  list(APPEND SRCS nrf54l_systick.c)
endif()
if(CONFIG_NRF54L_GRTC)
  list(APPEND SRCS nrf54l_grtc.c)
endif()
if(CONFIG_NRF54L_SYSTIMER_GRTC)
  list(APPEND SRCS nrf54l_tickless_grtc.c)
endif()
if(CONFIG_NRF54L_UART)
  list(APPEND SRCS nrf54l_serial.c)
endif()
if(CONFIG_NRF54L_TIMER)
  list(APPEND SRCS nrf54l_tim.c)
  if(CONFIG_TIMER)
    list(APPEND SRCS nrf54l_tim_lowerhalf.c)
  endif()
endif()
if(NOT CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS nrf54l_idle.c)
endif()
if(CONFIG_PM)
  list(APPEND SRCS nrf54l_pminitialize.c)
endif()

target_sources(arch PRIVATE ${SRCS})
