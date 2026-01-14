# ##############################################################################
# arch/arm/src/stm32h5/CMakeLists.txt
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

# Common ARM and Cortex-M33 files

if(CONFIG_STM32H5_PROGMEM)
  list(APPEND SRCS stm32_flash.c)
endif()

# Required STM32H5 files

list(
  APPEND
  SRCS
  stm32_gpio.c
  stm32_irq.c
  stm32_lowputc.c
  stm32_rcc.c
  stm32_start.c
  stm32_pwr.c
  stm32_timerisr.c
  stm32_lse.c
  stm32_lsi.c
  stm32_uid.c)

if(CONFIG_STM32H5_USART)
  list(APPEND SRCS stm32_serial.c)
endif()

if(NOT CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS stm32_idle.c)
endif()

if(CONFIG_TIMER)
  list(APPEND SRCS stm32_tim_lowerhalf.c)
endif()

if(CONFIG_STM32H5_I2C)
  list(APPEND SRCS stm32_i2c.c)
endif()

if(CONFIG_ADC)
  list(APPEND SRCS stm32_adc.c)
endif()

if(CONFIG_STM32H5_FDCAN_CHARDRIVER)
  list(APPEND SRCS stm32_fdcan.c)
endif()

if(CONFIG_STM32H5_ICACHE)
  list(APPEND SRCS stm32_icache.c)
endif()

if(CONFIG_STM32H5_SPI)
  list(APPEND SRCS stm32_spi.c)
endif()

if(CONFIG_STM32H5_QSPI1)
  list(APPEND SRCS stm32_qspi.c)
endif()

if(CONFIG_STM32H5_TIM)
  list(APPEND SRCS stm32_tim.c)
endif()

if(CONFIG_STM32H5_HAVE_HSI48)
  list(APPEND SRCS stm32_hsi48.c)
endif()

if(CONFIG_STM32H5_USBFS)
  list(APPEND SRCS stm32_usbfs.c)
endif()

if(CONFIG_STM32H5_ETHMAC)
  list(APPEND SRCS stm32_ethernet.c)
endif()

if(CONFIG_STM32H5_DMA)
  list(APPEND SRCS stm32_dma.c)
endif()

if(CONFIG_STM32H5_DTS)
  list(APPEND SRCS stm32_dts.c)
endif()

if(CONFIG_STM32H5_PWM)
  list(APPEND SRCS stm32_pwm.c)
endif()

# Required chip type specific files

if(CONFIG_STM32H5_STM32H5XXXX)
  list(APPEND SRCS stm32h5xx_rcc.c)
endif()

target_sources(arch PRIVATE ${SRCS})
