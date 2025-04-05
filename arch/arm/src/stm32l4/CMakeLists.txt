# ##############################################################################
# arch/arm/src/stm32l4/CMakeLists.txt
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
  stm32l4_allocateheap.c
  stm32l4_exti_gpio.c
  stm32l4_gpio.c
  stm32l4_irq.c
  stm32l4_lowputc.c
  stm32l4_rcc.c
  stm32l4_serial.c
  stm32l4_start.c
  stm32l4_waste.c
  stm32l4_uid.c
  stm32l4_spi.c
  stm32l4_i2c.c
  stm32l4_lse.c
  stm32l4_lsi.c
  stm32l4_pwr.c
  stm32l4_tim.c
  stm32l4_flash.c
  stm32l4_dfumode.c)

if(NOT CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS stm32l4_idle.c)
endif()

if(CONFIG_TIMER)
  list(APPEND SRCS stm32l4_tim_lowerhalf.c)
endif()

if(NOT CONFIG_SCHED_TICKLESS)
  list(APPEND SRCS stm32l4_timerisr.c)
else()
  list(APPEND SRCS stm32l4_tickless.c)
endif()

if(CONFIG_STM32L4_ONESHOT)
  list(APPEND SRCS stm32l4_oneshot.c stm32l4_oneshot_lowerhalf.c)
endif()

if(CONFIG_STM32L4_FREERUN)
  list(APPEND SRCS stm32l4_freerun.c)
endif()

if(CONFIG_BUILD_PROTECTED)
  list(APPEND SRCS stm32l4_userspace.c stm32l4_mpuinit.c)
endif()

if(CONFIG_STM32L4_HAVE_HSI48)
  list(APPEND SRCS stm32l4_hsi48.c)
endif()

if(CONFIG_STM32L4_ADC)
  list(APPEND SRCS stm32l4_adc.c)
endif()

if(CONFIG_STM32L4_DAC)
  list(APPEND SRCS stm32l4_dac.c)
endif()

if(CONFIG_STM32L4_DFSDM)
  list(APPEND SRCS stm32l4_dfsdm.c)
endif()

if(CONFIG_STM32L4_DMA)
  list(APPEND SRCS stm32l4_dma.c)
endif()

if(CONFIG_USBDEV)
  if(CONFIG_STM32L4_USBFS)
    list(APPEND SRCS stm32l4_usbdev.c)
  endif()
  if(CONFIG_STM32L4_OTGFS)
    list(APPEND SRCS stm32l4_otgfsdev.c)
  endif()
endif()

if(CONFIG_USBHOST)
  if(CONFIG_STM32L4_OTGFS)
    list(APPEND SRCS stm32l4_otgfshost.c)
  endif()
endif()

if(CONFIG_USBHOST)
  if(CONFIG_USBHOST_TRACE)
    list(APPEND SRCS stm32l4_usbhost_trace.c)
  else()
    if(CONFIG_DEBUG_USB)
      list(APPEND SRCS stm32l4_usbhost_trace.c)
    endif()
  endif()
endif()

if(CONFIG_PM)
  list(APPEND SRCS stm32l4_pmlpr.c stm32l4_pmsleep.c stm32l4_pmstandby.c
       stm32l4_pmstop.c)

  if(NOT CONFIG_ARCH_CUSTOM_PMINIT)
    list(APPEND SRCS stm32l4_pminitialize.c)
  endif()
endif()

if(CONFIG_STM32L4_PWR)
  list(APPEND SRCS stm32l4_exti_pwr.c)
endif()

if(CONFIG_STM32L4_RTC)
  if(CONFIG_RTC_ALARM)
    list(APPEND SRCS stm32l4_exti_alarm.c)
  endif()
  if(CONFIG_RTC_PERIODIC)
    list(APPEND SRCS stm32l4_exti_wakeup.c)
  endif()
  if(CONFIG_RTC_DRIVER)
    list(APPEND SRCS stm32l4_rtc_lowerhalf.c stm32l4_rtc.c)
  endif()
endif()

if(CONFIG_DEBUG_FEATURES)
  list(APPEND SRCS stm32l4_dumpgpio.c)
endif()

if(CONFIG_STM32L4_COMP)
  list(APPEND SRCS stm32l4_comp.c stm32l4_exti_comp.c)
endif()

if(CONFIG_STM32L4_RNG)
  list(APPEND SRCS stm32l4_rng.c)
endif()

if(CONFIG_STM32L4_SAI)
  list(APPEND SRCS stm32l4_sai.c)
endif()

if(CONFIG_STM32L4_LPTIM)
  list(APPEND SRCS stm32l4_lptim.c)
endif()

if(CONFIG_STM32L4_PWM)
  list(APPEND SRCS stm32l4_pwm.c)
endif()

if(CONFIG_SENSORS_QENCODER)
  list(APPEND SRCS stm32l4_qencoder.c)
endif()

if(CONFIG_STM32L4_QSPI)
  list(APPEND SRCS stm32l4_qspi.c)
endif()

if(CONFIG_STM32L4_CAN)
  list(APPEND SRCS stm32l4_can.c)
endif()

if(CONFIG_STM32L4_FIREWALL)
  list(APPEND SRCS stm32l4_firewall.c)
endif()

if(CONFIG_STM32L4_IWDG)
  list(APPEND SRCS stm32l4_iwdg.c)
endif()

if(CONFIG_STM32L4_IWDG)
  list(APPEND SRCS stm32l4_iwdg.c)
endif()

if(CONFIG_STM32L4_SDMMC1)
  list(APPEND SRCS stm32l4_sdmmc.c)
endif()

if(CONFIG_STM32L4_1WIREDRIVER)
  list(APPEND SRCS stm32l4_1wire.c)
endif()

target_sources(arch PRIVATE ${SRCS})
