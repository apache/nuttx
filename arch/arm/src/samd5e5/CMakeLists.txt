# ##############################################################################
# arch/arm/src/samd5e5/CMakeLists.txt
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

# Common ARM and Cortex-M4 files

# Required SAMD5x/E5x files

list(
  APPEND
  SRCS
  sam_clockconfig.c
  sam_gclk.c
  sam_irq.c
  sam_lowputc.c
  sam_port.c
  sam_serial.c
  sam_start.c
  sam_usart.c)

# Configuration-dependent SAMD5x/E5x files

if(NOT CONFIG_SCHED_TICKLESS)
  list(APPEND SRCS sam_timerisr.c)
endif()

if(CONFIG_SAMD5E5_SERCOM)
  list(APPEND SRCS sam_sercom.c)
endif()

if(CONFIG_SAMD5E5_TC)
  list(APPEND SRCS sam_tc.c)

  if(CONFIG_SAMD5E5_ONESHOT)
    list(APPEND SRCS sam_oneshot.c sam_oneshot_lowerhalf.c)
  endif()

  if(CONFIG_SAMD5E5_FREERUN)
    list(APPEND SRCS sam_freerun.c)
  endif()

  if(CONFIG_SCHED_TICKLESS)
    list(APPEND SRCS sam_tickless.c)
  endif()
endif()

if(CONFIG_BUILD_PROTECTED)
  list(APPEND SRCS sam_userspace.c sam_mpuinit.c)
endif()

if(CONFIG_SAMD5E5_EIC)
  list(APPEND SRCS sam_eic.c)
endif()

if(CONFIG_SAMD5E5_CMCC)
  list(APPEND SRCS sam_cmcc.c)
endif()

if(CONFIG_SAMD5E5_DMAC)
  list(APPEND SRCS sam_dmac.c)
endif()

if(NOT CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS sam_idle.c)
endif()

if(CONFIG_SAMD5E5_HAVE_SPI)
  list(APPEND SRCS sam_spi.c)
endif()

if(CONFIG_SAMD5E5_HAVE_I2C_MASTER)
  list(APPEND SRCS sam_i2c_master.c)
endif()

if(CONFIG_SAMD5E5_WDT)
  list(APPEND SRCS sam_wdt.c)
endif()

if(CONFIG_SAMD5E5_GMAC)
  list(APPEND SRCS sam_gmac.c sam_ethernet.c)
endif()

if(CONFIG_SAMD5E5_USB)
  list(APPEND SRCS sam_usb.c)
  if(CONFIG_USBHOST)
    if(CONFIG_USBHOST_TRACE)
      list(APPEND SRCS sam_usbhost.c)
    else()
      if(CONFIG_DEBUG_USB)
        list(APPEND SRCS sam_usbhost.c)
      endif()
    endif()
  endif()
endif()

if(CONFIG_SAMD5E5_SDHC0)
  list(APPEND SRCS sam_sdhc.c)
elseif(CONFIG_SAMD5E5_SDHC1)
  list(APPEND SRCS sam_sdhc.c)
endif()

if(CONFIG_SAMD5E5_AES)
  list(APPEND SRCS sam_aes.c)
endif()

if(CONFIG_SAMD5E5_RTC)
  list(APPEND SRCS sam_rtc.c)
endif()

if(CONFIG_SAMD5E5_PROGMEM)
  list(APPEND SRCS sam_progmem.c)
endif()

target_sources(arch PRIVATE ${SRCS})
