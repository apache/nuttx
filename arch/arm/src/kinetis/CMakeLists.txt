# ##############################################################################
# arch/arm/src/kinetis/CMakeLists.txt
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

# Required Kinetis files

list(
  APPEND
  SRCS
  kinetis_allocateheap.c
  kinetis_clockconfig.c
  kinetis_clrpend.c
  kinetis_irq.c
  kinetis_lowputc.c
  kinetis_pin.c
  kinetis_pingpio.c
  kinetis_serialinit.c
  kinetis_start.c
  kinetis_uid.c
  kinetis_wdog.c
  kinetis_cfmconfig.c
  kinetis_mpuinit.c)

# Configuration-dependent Kinetis K files

if(NOT CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS kinetis_idle.c)
endif()

if(NOT CONFIG_SCHED_TICKLESS)
  list(APPEND SRCS kinetis_timerisr.c)
endif()

if(CONFIG_BUILD_PROTECTED)
  list(APPEND SRCS kinetis_userspace.c)
endif()

if(CONFIG_KINETIS_GPIOIRQ)
  list(APPEND SRCS kinetis_pinirq.c)
endif()

if(CONFIG_DEBUG_GPIO_INFO)
  list(APPEND SRCS kinetis_pindump.c)
endif()

if(CONFIG_KINETIS_UART)
  list(APPEND SRCS kinetis_serial.c)
endif()

if(CONFIG_KINETIS_LPUART)
  list(APPEND SRCS kinetis_lpserial.c)
endif()

if(CONFIG_KINETIS_SDHC)
  list(APPEND SRCS kinetis_sdhc.c)
endif()

if(CONFIG_SPI)
  list(APPEND SRCS kinetis_spi.c)
endif()

if(CONFIG_USBDEV)
  list(APPEND SRCS kinetis_usbdev.c)
endif()

if(CONFIG_USBHOST)
  if(NOT CONFIG_KINETIS_USBHS)
    list(APPEND SRCS kinetis_usbhost.c)
  endif()
endif()

if(CONFIG_USBHOST)
  if(CONFIG_KINETIS_USBHS)
    list(APPEND SRCS kinetis_usbhshost.c)
  endif()
endif()

if(CONFIG_KINETIS_EDMA)
  list(APPEND SRCS kinetis_edma.c kinetis_pindma.c)
endif()

if(CONFIG_PWM)
  list(APPEND SRCS kinetis_pwm.c)
endif()

if(CONFIG_I2C)
  list(APPEND SRCS kinetis_i2c.c)
endif()

if(CONFIG_RTC)
  list(APPEND SRCS kinetis_rtc.c)
  if(CONFIG_RTC_DRIVER)
    list(APPEND SRCS kinetis_rtc_lowerhalf.c)
  endif()
endif()

if(CONFIG_NET)
  if(CONFIG_KINETIS_ENET)
    list(APPEND SRCS kinetis_enet.c)
  endif()

  if(CONFIG_NET_CAN)
    if(CONFIG_KINETIS_FLEXCAN)
      list(APPEND SRCS kinetis_flexcan.c)
    endif()
  endif()

endif()

target_sources(arch PRIVATE ${SRCS})
