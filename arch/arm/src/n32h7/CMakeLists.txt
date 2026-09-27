# ##############################################################################
# arch/arm/src/n32h7/CMakeLists.txt
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
  n32_allocateheap.c
  n32_exti_gpio.c
  n32_gpio.c
  n32_irq.c
  n32_rcc.c
  n32_lowputc.c
  n32_serial.c
  n32_cordic.c
  n32_uid.c
  n32_systemreset.c
  n32_start.c)

if(CONFIG_N32H7_PROGMEM)
  list(APPEND SRCS n32_flash.c)
endif()

if(CONFIG_ARCH_N32H7_DUALCORE)
  list(APPEND SRCS n32_dualcore.c)
endif()

if(CONFIG_N32H7_HSEM)
  list(APPEND SRCS n32_hsem.c)
endif()

if(CONFIG_RPTUN)
  list(APPEND SRCS n32_rptun.c)
endif()

if(CONFIG_SCHED_TICKLESS)
  list(APPEND SRCS n32_tickless.c)
else()
  list(APPEND SRCS n32_timerisr.c)
endif()

if(CONFIG_N32H7_ONESHOT)
  list(APPEND SRCS n32_oneshot.c)
endif()

if(CONFIG_ONESHOT)
  list(APPEND SRCS n32_oneshot_lowerhalf.c)
endif()

if(CONFIG_ARM_MPU)
  list(APPEND SRCS n32_mpuinit.c)
endif()

if(CONFIG_BUILD_PROTECTED)
  list(APPEND SRCS n32_userspace.c)
endif()

if(CONFIG_N32H7_ADC)
  list(APPEND SRCS n32_adc.c)
endif()

if(CONFIG_N32H7_FDCAN)
  list(APPEND SRCS n32_fdcan_sock.c)
endif()

if(CONFIG_N32H7_BBSRAM)
  list(APPEND SRCS n32_bbsram.c)
endif()

if(CONFIG_N32H7_DMA)
  list(APPEND SRCS n32_dma.c)
endif()

if(CONFIG_N32H7_FMC)
  list(APPEND SRCS n32_fmc.c)
endif()

if(CONFIG_N32H7_IWDG OR CONFIG_N32H7_RTC_LSICLOCK)
  list(APPEND SRCS n32_lsi.c)
endif()

if(CONFIG_N32H7_RTC_LSECLOCK)
  list(APPEND SRCS n32_lse.c)
endif()

if(CONFIG_N32H7_I2C)
  list(APPEND SRCS n32_i2c.c)
endif()

if(CONFIG_N32H7_PWR)
  list(APPEND SRCS n32_pwr.c)
endif()

if(CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS n32_idle.c)
endif()

if(CONFIG_N32H7_QUADSPI)
  list(APPEND SRCS n32_qspi.c)
endif()

if(CONFIG_N32H7_RTC)
  list(APPEND SRCS n32_rtc.c)
  if(CONFIG_RTC_ALARM)
    list(APPEND SRCS n32_exti_alarm.c)
  endif()
  if(CONFIG_RTC_PERIODIC)
    list(APPEND SRCS n32_exti_wakeup.c)
  endif()
  if(CONFIG_RTC_DRIVER)
    list(APPEND SRCS n32_rtc_lowerhalf.c)
  endif()
endif()

if(CONFIG_N32H7_SPI)
  list(APPEND SRCS n32_spi.c)
endif()

if(CONFIG_SPI_SLAVE)
  list(APPEND SRCS n32_spi_slave.c)
endif()

if(CONFIG_N32H7_SDMMC)
  list(APPEND SRCS n32_sdmmc.c)
endif()

if(CONFIG_TIMER)
  list(APPEND SRCS n32_tim_lowerhalf.c)
endif()

if(CONFIG_N32H7_CAPTURE)
  list(APPEND SRCS n32_capture.c)
endif()

if(CONFIG_CAPTURE)
  list(APPEND SRCS n32_capture_lowerhalf.c)
endif()

if(CONFIG_USBDEV)
  list(APPEND SRCS n32_usbhsdev.c)
endif()

if(CONFIG_USBHOST)
  list(APPEND SRCS n32_usbhshost.c)
endif()

if(CONFIG_N32H7_TIM)
  list(APPEND SRCS n32_tim.c)
endif()

if(CONFIG_N32H7_LPTIM)
  list(APPEND SRCS n32_lptim.c)
endif()

if(CONFIG_N32H7_PWM)
  list(APPEND SRCS n32_pwm.c)
endif()

if(CONFIG_N32H7_ETHMAC)
  list(APPEND SRCS n32_ethernet.c)
endif()

if(CONFIG_SENSORS_QENCODER)
  list(APPEND SRCS n32_qencoder.c)
endif()

if(CONFIG_PM)
  list(APPEND SRCS n32_pmsleep.c n32_pmstandby.c n32_pmstop.c)
  if(NOT CONFIG_ARCH_CUSTOM_PMINIT)
    list(APPEND SRCS n32_pminitialize.c)
  endif()
endif()

if(CONFIG_N32H7_IWDG)
  list(APPEND SRCS n32_iwdg.c)
endif()

if(CONFIG_N32H7_WWDG)
  list(APPEND SRCS n32_wwdg.c)
endif()

target_sources(arch PRIVATE ${SRCS})
