# ##############################################################################
# arch/arm/src/lpc43xx/CMakeLists.txt
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

list(
  APPEND
  SRCS
  lpc43_allocateheap.c
  lpc43_cgu.c
  lpc43_clrpend.c
  lpc43_gpio.c
  lpc43_irq.c
  lpc43_pinconfig.c
  lpc43_rgu.c
  lpc43_serial.c
  lpc43_start.c
  lpc43_uart.c)

if(NOT CONFIG_SCHED_TICKLESS)
  list(APPEND SRCS lpc43_timerisr.c)
else()
  list(APPEND SRCS lpc43_tickless_rit.c)
endif()

if(CONFIG_BUILD_PROTECTED)
  list(APPEND SRCS lpc43_userspace.c lpc43_mpuinit.c)
endif()

if(NOT CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS lpc43_idle.c)
endif()

if(CONFIG_DEBUG_FEATURES)
  list(APPEND SRCS lpc43_debug.c)
endif()

if(CONFIG_LPC43_GPDMA)
  list(APPEND SRCS lpc43_gpdma.c)
endif()

if(CONFIG_LPC43_GPIO_IRQ)
  list(APPEND SRCS lpc43_gpioint.c)
endif()

if(CONFIG_LPC43_WWDT)
  list(APPEND SRCS lpc43_wwdt.c)
endif()

if(CONFIG_LPC43_SDMMC)
  list(APPEND SRCS lpc43_sdmmc.c)
endif()

if(CONFIG_LPC43_ETHERNET)
  list(APPEND SRCS lpc43_ethernet.c)
endif()

if(CONFIG_LPC43_EMC)
  list(APPEND SRCS lpc43_emc.c)
endif()

if(CONFIG_LPC43_SPI)
  list(APPEND SRCS lpc43_spi.c)
elseif(CONFIG_LPC43_SSP0)
  list(APPEND SRCS lpc43_spi.c)
elseif(CONFIG_LPC43_SSP1)
  list(APPEND SRCS lpc43_spi.c)
endif()

if(CONFIG_LPC43_SPIFI)
  list(APPEND SRCS lpc43_spifi.c)
endif()

if(CONFIG_LPC43_SSP0)
  list(APPEND SRCS lpc43_ssp.c)
elseif(CONFIG_LPC43_SSP1)
  list(APPEND SRCS lpc43_ssp.c)
endif()

if(CONFIG_LPC43_TIMER)
  list(APPEND SRCS lpc43_timer.c)
endif()

if(CONFIG_LPC43_RIT)
  list(APPEND SRCS lpc43_rit.c)
endif()

if(CONFIG_LPC43_RTC)
  list(APPEND SRCS lpc43_rtc.c)
endif()

if(CONFIG_LPC43_I2C0)
  list(APPEND SRCS lpc43_i2c.c)
elseif(CONFIG_LPC43_I2C1)
  list(APPEND SRCS lpc43_i2c.c)
endif()

if(CONFIG_LPC43_CAN0)
  list(APPEND SRCS lpc43_can.c)
elseif(CONFIG_LPC43_CAN1)
  list(APPEND SRCS lpc43_can.c)
endif()

if(CONFIG_LPC43_ADC0)
  list(APPEND SRCS lpc43_adc.c)
elseif(CONFIG_LPC43_ADC1)
  list(APPEND SRCS lpc43_adc.c)
endif()

if(CONFIG_LPC43_DAC)
  list(APPEND SRCS lpc43_dac.c)
endif()

if(CONFIG_LPC43_USBOTG)
  list(APPEND SRCS lpc43_ehci.c)
endif()

if(CONFIG_CRYPTO_AES)
  list(APPEND SRCS lpc43_aes.c)
endif()

if(CONFIG_CRYPTO_CRYPTODEV_HARDWARE)
  list(APPEND SRCS lpc43_crypto.c)
endif()

if(CONFIG_LPC43_USB0)
  if(CONFIG_USBDEV)
    list(APPEND SRCS lpc43_usb0dev.c)
  endif()
endif()

if(CONFIG_SPIFI_LIBRARY)
  add_subdirectory(spifi)
endif()

target_sources(arch PRIVATE ${SRCS})
