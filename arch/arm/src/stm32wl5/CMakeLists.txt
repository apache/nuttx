# ##############################################################################
# arch/arm/src/stm32wl5/CMakeLists.txt
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

# Required STM32WL5 files

set(SRCS
    stm32wl5_allocateheap.c
    stm32wl5_exti_gpio.c
    stm32wl5_gpio.c
    stm32wl5_irq.c
    stm32wl5_lowputc.c
    stm32wl5_rcc.c
    stm32wl5_serial.c
    stm32wl5_start.c
    stm32wl5_waste.c
    stm32wl5_uid.c
    stm32wl5_lse.c
    stm32wl5_lsi.c
    stm32wl5_idle.c
    stm32wl5_pwr.c
    stm32wl5_tim.c
    stm32wl5_flash.c
    stm32wl5_timerisr.c
    stm32wl5_spi.c)

if(CONFIG_STM32WL5_IPCC)
  list(APPEND SRCS stm32wl5_ipcc.c)
endif()

target_sources(arch PRIVATE ${SRCS})
