# ##############################################################################
# arch/arm/src/stm32wb/CMakeLists.txt
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

# Required STM32WB files

set(SRCS
    stm32wb_allocateheap.c
    stm32wb_exti_gpio.c
    stm32wb_gpio.c
    stm32wb_irq.c
    stm32wb_lowputc.c
    stm32wb_rcc.c
    stm32wb_spi.c
    stm32wb_serial.c
    stm32wb_i2c.c
    stm32wb_start.c
    stm32wb_waste.c
    stm32wb_rcc_lse.c
    stm32wb_rcc_lsi.c
    stm32wb_pwr.c
    stm32wb_tim.c
    stm32wb_flash.c
    stm32wb_uid.c)

if(NOT CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS stm32wb_idle.c)
endif()

if(CONFIG_TIMER)
  list(APPEND SRCS stm32wb_tim_lowerhalf.c)
endif()

if(NOT CONFIG_SCHED_TICKLESS)
  list(APPEND SRCS stm32wb_timerisr.c)
else()
  list(APPEND SRCS stm32wb_tickless.c)
endif()

if(CONFIG_STM32WB_ONESHOT)
  list(APPEND SRCS stm32wb_oneshot.c stm32wb_oneshot_lowerhalf.c)
endif()

if(CONFIG_STM32WB_FREERUN)
  list(APPEND SRCS stm32wb_freerun.c)
endif()

if(CONFIG_BUILD_PROTECTED)
  list(APPEND SRCS stm32wb_userspace.c stm32wb_mpuinit.c)
endif()

if(CONFIG_STM32WB_HAVE_HSI48)
  list(APPEND SRCS stm32wb_rcc_hsi48.c)
endif()

if(CONFIG_STM32WB_DMA)
  list(APPEND SRCS stm32wb_dma.c)
endif()

if(CONFIG_PM)
  list(APPEND SRCS stm32wb_pmlpr.c stm32wb_pmsleep.c stm32wb_pmstandby.c)
  list(APPEND SRCS stm32wb_pmstop.c)

  if(NOT CONFIG_ARCH_CUSTOM_PMINIT)
    list(APPEND SRCS stm32wb_pminitialize.c)
  endif()
endif()

if(CONFIG_STM32WB_PWR)
  list(APPEND SRCS stm32wb_exti_pwr.c)
endif()

if(CONFIG_STM32WB_RTC)
  if(CONFIG_RTC_ALARM)
    list(APPEND SRCS stm32wb_exti_alarm.c)
  endif()
  if(CONFIG_RTC_PERIODIC)
    list(APPEND SRCS stm32wb_exti_wakeup.c)
  endif()
  if(CONFIG_RTC_DRIVER)
    list(APPEND SRCS stm32wb_rtc.c stm32wb_rtc_lowerhalf.c)
  endif()
endif()

if(CONFIG_STM32WB_IPCC)
  list(APPEND SRCS stm32wb_ipcc.c)
endif()

if(CONFIG_STM32WB_MBOX)
  list(APPEND SRCS stm32wb_mbox.c)
endif()

if(CONFIG_STM32WB_BLE)
  list(APPEND SRCS stm32wb_blehci.c)
endif()

if(CONFIG_DEBUG_FEATURES)
  list(APPEND SRCS stm32wb_dumpgpio.c)
endif()

target_sources(arch PRIVATE ${SRCS})
