# ##############################################################################
# arch/arm/src/stm32h7/CMakeLists.txt
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
  stm32_exti_gpio.c
  stm32_gpio.c
  stm32_irq.c
  stm32_start.c
  stm32_rcc.c
  stm32_lowputc.c
  stm32_serial.c
  stm32_uid.c)

if(CONFIG_STM32H7_PROGMEM)
  list(APPEND SRCS stm32_flash.c)
endif()

if(CONFIG_ARCH_STM32H7_DUALCORE)
  list(APPEND SRCS stm32_dualcore.c)
endif()

if(CONFIG_STM32H7_HSEM)
  list(APPEND SRCS stm32_hsem.c)
endif()

if(CONFIG_RPTUN)
  list(APPEND SRCS stm32_rptun.c)
endif()

if(CONFIG_SCHED_TICKLESS)
  list(APPEND SRCS stm32_tickless.c)
else()
  list(APPEND SRCS stm32_timerisr.c)
endif()

if(CONFIG_STM32H7_ONESHOT)
  list(APPEND SRCS stm32_oneshot.c stm32_oneshot_lowerhalf.c)
endif()

if(CONFIG_ARM_MPU)
  list(APPEND SRCS stm32_mpuinit.c)
endif()

if(CONFIG_BUILD_PROTECTED)
  list(APPEND SRCS stm32_userspace.c)
endif()

if(CONFIG_ARMV7M_DTCM)
  list(APPEND SRCS stm32_dtcm.c)
endif()

if(CONFIG_STM32H7_ADC)
  list(APPEND SRCS stm32_adc.c)
endif()

if(CONFIG_STM32H7_FDCAN)
  list(APPEND SRCS stm32_fdcan_sock.c)
endif()

if(CONFIG_STM32H7_BBSRAM)
  list(APPEND SRCS stm32_bbsram.c)
endif()

if(CONFIG_STM32H7_DMA)
  list(APPEND SRCS stm32_dma.c)
endif()

if(CONFIG_STM32H7_FMC)
  list(APPEND SRCS stm32_fmc.c)
endif()

if(CONFIG_STM32H7_IWDG OR CONFIG_STM32H7_RTC_LSICLOCK)
  list(APPEND SRCS stm32_lsi.c)
endif()

if(CONFIG_STM32H7_RTC_LSECLOCK)
  list(APPEND SRCS stm32_lse.c)
endif()

if(CONFIG_STM32H7_I2C)
  list(APPEND SRCS stm32_i2c.c)
endif()

if(CONFIG_STM32H7_PWR)
  list(APPEND SRCS stm32_pwr.c)
endif()

if(CONFIG_STM32H7_QUADSPI)
  list(APPEND SRCS stm32_qspi.c)
endif()

if(CONFIG_STM32H7_RTC)
  list(APPEND SRCS stm32_rtc.c)
  if(CONFIG_RTC_ALARM)
    list(APPEND SRCS stm32_exti_alarm.c)
  endif()
  if(CONFIG_RTC_PERIODIC)
    list(APPEND SRCS stm32_exti_wakeup.c)
  endif()
  if(CONFIG_RTC_DRIVER)
    list(APPEND SRCS stm32_rtc_lowerhalf.c)
  endif()
endif()

if(CONFIG_STM32H7_SPI)
  list(APPEND SRCS stm32_spi.c)
endif()

if(CONFIG_SPI_SLAVE)
  list(APPEND SRCS stm32_spi_slave.c)
endif()

if(CONFIG_STM32H7_SDMMC)
  list(APPEND SRCS stm32_sdmmc.c)
endif()

if(CONFIG_TIMER)
  list(APPEND SRCS stm32_tim_lowerhalf.c)
endif()

if(CONFIG_USBDEV)
  list(APPEND SRCS stm32_otgdev.c)
endif()

if(CONFIG_USBHOST)
  list(APPEND SRCS stm32_otghost.c)
  if(CONFIG_USBHOST_TRACE)
    list(APPEND SRCS stm32_usbhost.c)
  else()
    if(CONFIG_DEBUG_USB)
      list(APPEND SRCS stm32_usbhost.c)
    endif()
  endif()
endif()

if(CONFIG_STM32H7_TIM)
  list(APPEND SRCS stm32_tim.c)
endif()

if(CONFIG_STM32H7_LPTIM)
  list(APPEND SRCS stm32_lptim.c)
endif()

if(CONFIG_STM32H7_PWM)
  list(APPEND SRCS stm32_pwm.c)
endif()

if(CONFIG_STM32H7_ETHMAC)
  list(APPEND SRCS stm32_ethernet.c)
endif()

if(CONFIG_SENSORS_QENCODER)
  list(APPEND SRCS stm32_qencoder.c)
endif()

if(CONFIG_PM)
  list(APPEND SRCS stm32_pmsleep.c stm32_pmstandby.c stm32_pmstop.c)
  if(NOT CONFIG_ARCH_CUSTOM_PMINIT)
    list(APPEND SRCS stm32_pminitialize.c)
  endif()
endif()

if(CONFIG_STM32H7_IWDG)
  list(APPEND SRCS stm32_iwdg.c)
endif()

if(CONFIG_STM32H7_WWDG)
  list(APPEND SRCS stm32_wwdg.c)
endif()

target_sources(arch PRIVATE ${SRCS})
