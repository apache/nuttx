# ##############################################################################
# arch/arm/src/stm32f0l0g0/CMakeLists.txt
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
  stm32_start.c
  stm32_gpio.c
  stm32_exti_gpio.c
  stm32_irq.c
  stm32_lse.c
  stm32_lowputc.c
  stm32_serial.c
  stm32_rcc.c)

if(CONFIG_STM32F0L0G0_DMA)
  list(APPEND SRCS stm32_dma_v1.c)
endif()

if(CONFIG_STM32F0L0G0_PWR)
  list(APPEND SRCS stm32_pwr.c)
endif()

if(NOT CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS stm32_idle.c)
endif()

if(NOT CONFIG_SCHED_TICKLESS)
  list(APPEND SRCS stm32_timerisr.c)
endif()

if(CONFIG_BUILD_PROTECTED)
  list(APPEND SRCS stm32_userspace.c)
endif()

if(CONFIG_STM32F0L0G0_GPIOIRQ)
  list(APPEND SRCS stm32_gpioint.c)
endif()

if(CONFIG_ARCH_IRQPRIO)
  list(APPEND SRCS stm32_irqprio.c)
endif()

if(CONFIG_STM32F0L0G0_HAVE_HSI48)
  list(APPEND SRCS stm32_hsi48.c)
endif()

if(CONFIG_STM32F0L0G0_USB)
  list(APPEND SRCS stm32_usbdev.c)
endif()

if(CONFIG_STM32F0L0G0_I2C)
  list(APPEND SRCS stm32_i2c.c)
endif()

if(CONFIG_STM32F0L0G0_SPI)
  list(APPEND SRCS stm32_spi.c)
endif()

if(CONFIG_STM32F0L0G0_PWM)
  list(APPEND SRCS stm32_pwm.c)
endif()

if(CONFIG_STM32F0L0G0_ADC)
  list(APPEND SRCS stm32_adc.c)
endif()

if(CONFIG_STM32F0L0G0_AES)
  list(APPEND SRCS stm32_aes.c)
endif()

if(CONFIG_STM32F0L0G0_RNG)
  list(APPEND SRCS stm32_rng.c)
endif()

if(CONFIG_STM32F0L0G0_TIM)
  list(APPEND SRCS stm32_tim.c stm32_tim_lowerhalf.c)
endif()

target_sources(arch PRIVATE ${SRCS})
