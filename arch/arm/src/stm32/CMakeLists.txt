############################################################################
# arch/arm/src/stm32/Make.defs
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

set(SRCS)

list(APPEND SRCS
  stm32_allocateheap.c stm32_start.c stm32_rcc.c stm32_lse.c
  stm32_lsi.c stm32_gpio.c stm32_exti_gpio.c stm32_flash.c
  stm32_irq.c stm32_dma.c stm32_lowputc.c
  stm32_serial.c stm32_spi.c stm32_i2s.c stm32_sdio.c stm32_tim.c
  stm32_waste.c stm32_ccm.c stm32_uid.c stm32_capture.c
  stm32_dfumode.c)

if (CONFIG_TIMER)
  list(APPEND SRCS stm32_tim_lowerhalf.c)
endif()

if (CONFIG_STM32_TICKLESS_TIMER)
  list(APPEND SRCS stm32_tickless.c)
else()
  list(APPEND SRCS stm32_timerisr.c)
endif()

if (CONFIG_STM32_ONESHOT)
  list(APPEND SRCS stm32_oneshot.c stm32_oneshot_lowerhalf.c)
endif()

if (CONFIG_STM32_FREERUN)
  list(APPEND SRCS stm32_freerun.c)
endif()

if (CONFIG_BUILD_PROTECTED)
  list(APPEND SRCS stm32_userspace.c stm32_mpuinit.c)
endif()

if (CONFIG_STM32_CCM_PROCFS)
  list(APPEND SRCS stm32_procfs_ccm.c)
endif()

if (CONFIG_STM32_HAVE_IP_I2C_V1)
  if (CONFIG_STM32_I2C_ALT)
    list(APPEND SRCS stm32_i2c_alt.c)
  elseif (CONFIG_STM32_STM32F4XXX)
    list(APPEND SRCS stm32f40xxx_i2c.c)
  else()
    list(APPEND SRCS stm32_i2c.c)
  endif()
elseif (CONFIG_STM32_HAVE_IP_I2C_V2)
  list(APPEND SRCS stm32_i2c_v2.c)
endif()

if (CONFIG_USBDEV)
  if (CONFIG_STM32_USB)
    list(APPEND SRCS stm32_usbdev.c)
  endif()
  if (CONFIG_STM32_OTGFS)
    list(APPEND SRCS stm32_otgfsdev.c)
  endif()
  if (CONFIG_STM32_OTGHS)
    list(APPEND SRCS stm32_otghsdev.c)
  endif()
endif()

if (CONFIG_STM32_USBHOST)
  if (CONFIG_STM32_OTGFS)
    list(APPEND SRCS stm32_otgfshost.c)
  endif()
  if (CONFIG_STM32_OTGHS)
    list(APPEND SRCS stm32_otghshost.c)
  endif()
  if (CONFIG_USBHOST_TRACE)
    list(APPEND SRCS stm32_usbhost.c)
  else()
    if (CONFIG_DEBUG_USB)
      list(APPEND SRCS stm32_usbhost.c)
    endif()
  endif()
endif()

if (NOT CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS stm32_idle.c)
endif()

list(APPEND SRCS stm32_pmstop.c stm32_pmstandby.c stm32_pmsleep.c)

if (NOT CONFIG_ARCH_CUSTOM_PMINIT)
  list(APPEND SRCS stm32_pminitialize.c)
endif()

if (CONFIG_STM32_ETHMAC)
  list(APPEND SRCS stm32_eth.c)
endif()

if (CONFIG_STM32_PWR)
  list(APPEND SRCS stm32_pwr.c stm32_exti_pwr.c)
endif()

if (CONFIG_STM32_RTC)
  list(APPEND SRCS stm32_rtc.c)
  if (CONFIG_RTC_ALARM)
    list(APPEND SRCS stm32_exti_alarm.c)
  endif()
  if (CONFIG_RTC_PERIODIC)
    list(APPEND SRCS stm32_exti_wakeup.c)
  endif()
  if (CONFIG_RTC_DRIVER)
    list(APPEND SRCS stm32_rtc_lowerhalf.c)
  endif()
endif()

if (CONFIG_STM32_ADC)
  list(APPEND SRCS stm32_adc.c)
endif()

if (CONFIG_STM32_SDADC)
  list(APPEND SRCS stm32_sdadc.c)
endif()

if (CONFIG_STM32_DAC)
  list(APPEND SRCS stm32_dac.c)
endif()

if (CONFIG_STM32_COMP)
  list(APPEND SRCS stm32_comp.c)
endif()

if (CONFIG_STM32_OPAMP)
  list(APPEND SRCS stm32_opamp.c)
endif()

if (CONFIG_STM32_HRTIM)
  list(APPEND SRCS stm32_hrtim.c)
endif()

if (CONFIG_STM32_1WIREDRIVER)
  list(APPEND SRCS stm32_1wire.c)
endif()

if (CONFIG_STM32_HCIUART)
  list(APPEND SRCS stm32_hciuart.c)
endif()

if (CONFIG_STM32_RNG)
  list(APPEND SRCS stm32_rng.c)
endif()

if (CONFIG_STM32_LTDC)
  list(APPEND SRCS stm32_ltdc.c)
endif()

if (CONFIG_STM32_DMA2D)
  list(APPEND SRCS stm32_dma2d.c)
endif()

if (CONFIG_STM32_PWM)
  list(APPEND SRCS stm32_pwm.c)
endif()

if (CONFIG_SENSORS_QENCODER)
  list(APPEND SRCS stm32_qencoder.c)
endif()

if (CONFIG_STM32_CAN)
  list(APPEND SRCS stm32_can.c)
endif()

if (CONFIG_STM32_IWDG)
  list(APPEND SRCS stm32_iwdg.c)
endif()

if (CONFIG_STM32_WWDG)
  list(APPEND SRCS stm32_wwdg.c)
endif()

if (CONFIG_DEBUG_FEATURES)
  list(APPEND SRCS stm32_dumpgpio.c)
endif()

if (CONFIG_STM32_AES)
  list(APPEND SRCS stm32_aes.c)
endif()

if (CONFIG_STM32_BBSRAM)
  list(APPEND SRCS stm32_bbsram.c)
endif()

if (CONFIG_STM32_FMC)
  list(APPEND SRCS stm32_fmc.c)
endif()

if (CONFIG_STM32_FSMC)
  list(APPEND SRCS stm32_fsmc.c)
endif()

if (CONFIG_STM32_FOC)
  list(APPEND SRCS stm32_foc.c)
endif()

target_sources(arch PRIVATE ${SRCS})
