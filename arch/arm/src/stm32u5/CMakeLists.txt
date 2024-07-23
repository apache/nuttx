# ##############################################################################
# arch/arm/src/stm32u5/CMakeLists.txt
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
    stm32_allocateheap.c
    stm32_exti_gpio.c
    stm32_gpio.c
    stm32_irq.c
    stm32_lowputc.c
    stm32_rcc.c
    stm32_i2c.c
    stm32_serial.c
    stm32_start.c
    stm32_waste.c
    stm32_uid.c
    stm32_spi.c
    stm32_lse.c
    stm32_lsi.c
    stm32u5xx_rcc.c
    stm32_pwr.c
    stm32_tim.c
    stm32_flash.c
    stm32_timerisr.c)

if(NOT CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS stm32_idle.c)
endif()

if(CONFIG_TIMER)
  list(APPEND SRCS stm32_tim_lowerhalf.c)
endif()

if(CONFIG_BUILD_PROTECTED)
  list(APPEND SRCS stm32_userspace.c stm32_mpuinit.c)
endif()

if(CONFIG_DEBUG_FEATURES)
  list(APPEND SRCS stm32_dumpgpio.c)
endif()

if(CONFIG_USBDEV)
  list(APPEND SRCS stm32_otgdev.c)
endif()

target_sources(arch PRIVATE ${SRCS})
