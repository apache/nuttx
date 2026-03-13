# ##############################################################################
# arch/arm/src/lpc17xx_40xx/CMakeLists.txt
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

# Required LPC17xx files

list(
  APPEND
  SRCS
  lpc17_40_allocateheap.c
  lpc17_40_clockconfig.c
  lpc17_40_clrpend.c
  lpc17_40_gpio.c
  lpc17_40_i2c.c
  lpc17_40_irq.c
  lpc17_40_lowputc.c
  lpc17_40_serial.c
  lpc17_40_spi.c
  lpc17_40_ssp.c
  lpc17_40_start.c)

if(NOT CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS lpc17_40_idle.c)
endif()

if(NOT CONFIG_SCHED_TICKLESS)
  list(APPEND SRCS lpc17_40_timerisr.c)
endif()

if(CONFIG_BUILD_PROTECTED)
  list(APPEND SRCS lpc17_40_userspace.c lpc17_40_mpuinit.c)
endif()

if(CONFIG_LPC17_40_EMC)
  list(APPEND SRCS lpc17_40_emc.c)
endif()

if(CONFIG_LPC17_40_GPIOIRQ)
  list(APPEND SRCS lpc17_40_gpioint.c)
endif()

if(CONFIG_DEBUG_GPIO_INFO)
  list(APPEND SRCS lpc17_40_gpiodbg.c)
endif()

if(CONFIG_LPC17_40_LCD)
  list(APPEND SRCS lpc17_40_lcd.c)
endif()

if(CONFIG_USBDEV)
  list(APPEND SRCS lpc17_40_usbdev.c)
endif()

if(CONFIG_USBHOST)
  list(APPEND SRCS lpc17_40_usbhost.c)
endif()

if(CONFIG_LPC17_40_GPDMA)
  list(APPEND SRCS lpc17_40_gpdma.c)
endif()

if(CONFIG_LPC17_40_SDCARD)
  list(APPEND SRCS lpc17_40_sdcard.c)
endif()

if(CONFIG_NET)
  if(CONFIG_LPC17_40_ETHERNET)
    list(APPEND SRCS lpc17_40_ethernet.c)
  endif()
endif()

if(CONFIG_LPC17_40_CAN)
  list(APPEND SRCS lpc17_40_can.c)
endif()

if(CONFIG_LPC17_40_ADC)
  list(APPEND SRCS lpc17_40_adc.c)
endif()

if(CONFIG_LPC17_40_DAC)
  list(APPEND SRCS lpc17_40_dac.c)
endif()

if(CONFIG_LPC17_40_RTC)
  list(APPEND SRCS lpc176x_rtc.c)
endif()

if(CONFIG_LPC17_40_WDT)
  list(APPEND SRCS lpc17_40_wdt.c)
endif()

if(CONFIG_LPC17_40_PWM1)
  list(APPEND SRCS lpc17_40_pwm.c)
endif()

if(CONFIG_LPC17_40_MCPWM)
  list(APPEND SRCS lpc17_40_mcpwm.c)
endif()

if(CONFIG_LPC17_40_TMR0)
  list(APPEND SRCS lpc17_40_timer.c)
endif()

if(CONFIG_LPC17_40_PROGMEM)
  list(APPEND SRCS lpc17_40_progmem.c)
endif()

target_sources(arch PRIVATE ${SRCS})
